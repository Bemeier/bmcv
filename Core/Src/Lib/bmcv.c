#include "bmcv.h"
#include "bmcv_probe.h"
#include "buttons_encoders.h"
#include "config.h"
#include "dac_adc.h"
#include "dac_adc_hal.h"
#include "engine.h"
#include "engine_state.h"
#include "helpers.h"
#include "hw_setup.h"
#include "input_fold.h"
#include "instance.h"
#include "led_fb.h"
#include "mcp.h"
#include "midi.h"
#include "presets.h"
#include "stm32g474xx.h"
#include "usblink.h"
#include "version.h"
#include "ws2811.h"
#include <stdint.h>

// Hardware Config
static uint16_t mpc_interrupt_pin;
static ADC_TypeDef* slider_adc;
static volatile uint16_t slider_adc_value;

// Task scheduler.
//
// The three polls are requests raised in interrupt context - TIM4 for the LED
// flush and the periodic MCP re-arm, EXTI for an expander interrupt, the SPI
// DMA completion for the DAC - and consumed here in the main loop. See IsrFlag
// in helpers.h for why that cannot be a plain uint8_t.
//
// `task` is not one of them: it is read and written only inside
// bmcv_poll_tasks(), which is entirely TIM4's, so it never crosses a context.
static uint8_t task = 0;
// How many times the DAC service covers all eight outputs within one engine
// tick, and the chunk interval that follows from it - one frame is
// DAC_CHANNELS chunks, so this is what the service is rate-limited to.
//
// The ratio used to be emergent rather than chosen: the service re-armed on
// every loop pass a DMA completion allowed, which happened to land at 15.8
// chunks per tick because that is where the HAL turnaround and the tick period
// crossed. Stating it makes it a number that can be reasoned about - and turned
// down, which is the cheap lever for CPU, since a chunk costs single-digit
// microseconds where a tick costs over a hundred.
//
// It also spreads the chunks evenly instead of letting them bunch into the gap
// between ticks, which keeps the ADC's sampling cadence even as a side effect.
//
// 1, and not the 4 this asked for until the timer made the number real. Every
// figure below is bmcv_profile off the module, with eight channels patched.
//
// At 4 the cadence is 15.0us. Through HAL one dac_service() measured 8.18us of
// that - 55% of the CPU in this interrupt before the completion handler it
// feeds was counted - and the module came up with dac_fps 54335, engine_fps 0
// and `just profile` reporting *0 ticks since boot*. The engine had never run
// once; `just where` found the core in SPI_DMATransmitReceiveCplt.
//
// Taking HAL out of the arming and completion path (see dac_adc.c) brought the
// service to 5.1us and the whole transaction from ~17.4us to ~9us. That is what
// made the rate a choice rather than a cliff, and it is where the substeps
// question was re-asked with real numbers:
//
//   substeps  frames/s  engine_fps  load
//          1      4032        2921  1.39
//          2      8063        2260  1.65
//
// So 1. Doubling the frame rate costs the engine 22%, and output latency is one
// engine tick - the interpolation puts the levels computed on tick N onto the
// pins across tick N+1 - so a slower engine hands back in staleness most of
// what the extra frames buy. Measured end to end against a sine, both read the
// same R^2.
//
// What 1 buys is still the point of the change: DAC_CHANNELS transactions per
// tick is one full frame per tick, so every output carries every value the
// engine computes, where it was getting one frame every four ticks. What it
// gives up is oversampling - at 1 the interpolation has nothing to interpolate
// between, and it only starts earning its keep again at 2.
//
// 2 was that answer at a 500us tick: a 62us chunk, 4032 frames/s per pin, the
// interpolation finally oversampling twice per tick instead of landing exactly
// on it.
//
// 4 now, and for the output rather than for the engine. The step a pin holds is
// (dV/dt * frame period), so halving the period halves the step - and moves the
// ripple an octave up, where the 100R/2.2uF at the jack attenuates it 6dB
// harder. Both together are ~12dB off the staircase for one doubling, which is
// the cheapest noise there is to buy.
//
// Nothing in the converter objects. AD5754 Rev F: SCLK cycle 33ns (30MHz, we
// run 18), minimum SYNC high in daisy-chain mode 200ns against the 28us this
// leaves, settling 10us for a full-scale step against a 124us frame. Six bytes
// is 2.67us of wire in a 31us chunk, so the bus is 9% used. The cost is all
// CPU, and it is two interrupts per chunk, not one - see bmcv_profile.dac and
// bmcv_profile.dac_cplt, which is why the second one is now measured.
//
// Measured on the module, and the per-chunk cost is rate-independent: dac 4.4us
// plus dac_cplt 2.1us, so 6.5us of every chunk either way. That is 10.5% of the
// CPU at 62us and 21% at 31us, and it took `load` from 0.39 to 0.48 with
// engine_fps holding 2000.03. The 8% this file used to claim was dac alone,
// before the completion interrupt was measured at all.
#define DAC_SUBSTEPS 4

// The cadence the service runs at: DAC_SUBSTEPS frames per tick, a frame being
// DAC_CHANNELS transactions of two outputs each. 62.5us at a 2kHz tick.
//
// It is now a cadence rather than a target. Serviced from the main loop it was
// a target and never met: measured on the module, transactions turned around
// every 55us with nothing patched and slowed to the engine's own rate with
// eight stepped channels, because a completion sat waiting for the loop to
// notice it and the loop was inside a tick that costs 307us of a 250us period.
// Six bytes at 18MHz is 2.67us of bus time, so the wire was idle 95% of the
// time and all of the rest was latency. A timer owns the service now - see
// bmcv_dac_tick() - so the output rate is a number this file chooses instead of
// a side effect of how busy the engine is.
#define DAC_CHUNK_US (ENGINE_TICK_US / (DAC_SUBSTEPS * DAC_CHANNELS))

uint32_t bmcv_dac_service_hz(void) { return 1000000u / DAC_CHUNK_US; }

// Set by the DMA completion, taken by the timer: "the transfer that was in
// flight has landed, so the wire is free". Starts clear because it starts true
// - main() arms the first transfer before starting the timer, so there is
// always exactly one thing that begins the chain, and this flag never has to
// claim a completion that has not happened.
static IsrFlag dac_poll = 0;
static uint32_t last_dac_poll;

// Defined with the interpolation it drives; called from the timer interrupt.
static void dac_service(uint32_t now_us);
static IsrFlag mcp_poll = 0;
static IsrFlag led_poll = 0;

// The module. One struct holding config, signal path, interaction state, the
// input layer and the wiring between them - see instance.h. The firmware has
// exactly one; a simulator has one per instance. Declared in bmcv.h, and
// external only so a debugger can name it.
BmcvInstance bmcv;

// ...and a note in flash saying where that is, for a debugger that has no ELF
// to ask - see bmcv_probe.h. The linker script pins the section to
// BMCV_PROBE_INFO_ADDR and asserts that it landed there.
//
// `used` because nothing in the firmware reads it and -ffunction-sections plus
// --gc-sections would otherwise be entirely right to throw it away.
#define STRINGIFY_(x) #x
#define STRINGIFY(x) STRINGIFY_(x)

__attribute__((section(".probe_info"), used)) const BmcvProbeInfo bmcv_probe_info = {
    .magic         = BMCV_PROBE_MAGIC,
    .info_version  = BMCV_PROBE_INFO_VERSION,
    .instance_size = (uint16_t) BMCV_SNAPSHOT_BYTES,
    .instance_addr = (uint32_t) &bmcv,
    .version       = STRINGIFY(FW_VERSION_MAJOR) "." STRINGIFY(FW_VERSION_MINOR) "." STRINGIFY(FW_VERSION_PATCH),
};

// What one tick costs, measured with the Cortex-M4's DWT cycle counter: a
// free-running 32-bit counter in the debug block that increments once per CPU
// clock. At 144MHz that is 6.9ns a count, against the 1us of the TIM2 timestamp
// the rest of this file runs on - a tick is tens of microseconds, so TIM2 was
// never going to tell us more than its order of magnitude. main() turns the
// counter on (init_cycle_counter); the subtraction is unsigned, so the 29.8s
// wrap needs no handling for a span this short.
//
// Left in the build: the probe is four register loads and a few float ops
// against a 500us budget, under 0.1%. Set to 0 to compile it out.
//
// What it measures is wall time with interrupts enabled - the DMA completion,
// TIM4, EXTI and USB all still preempt a tick. `avg_us` is therefore what the
// work costs, and `max_us` is that plus the worst interference it has run into,
// which is the one that decides whether a shorter tick period would fit.
#define BMCV_PROFILE 1

BmcvProfile bmcv_profile;

#if BMCV_PROFILE
#define PROFILE_NOW() (DWT->CYCCNT)

// Both set once in bmcv_init, so the per-tick work is a multiply and a compare
// rather than a divide.
static float us_per_cycle;
static uint32_t overrun_cycles; // one tick period, in cycles

static void span_record(BmcvSpan* s, uint32_t cycles)
{
  const float us = (float) cycles * us_per_cycle;

  s->last_cycles = cycles;
  s->last_us     = us;
  s->avg_us      = (s->avg_us == 0.0f) ? us : s->avg_us * 0.95f + 0.05f * us;

  // 0 means unset, which is also what writing 0 from a debugger means: either
  // way the next tick re-arms it.
  if (cycles < s->min_cycles || s->min_cycles == 0)
  {
    s->min_cycles = cycles;
    s->min_us     = us;
  }
  if (cycles > s->max_cycles)
  {
    s->max_cycles  = cycles;
    s->max_us      = us;
    s->max_at_tick = bmcv_profile.ticks;
  }
}
#else
#define PROFILE_NOW() 0u
#endif

// Preset persistence for the core. On the module this is FRAM; the simulator
// and a VCV Rack instance plug their own storage in here instead.
static int8_t fram_store(void* user, const EngineConfig* cfg, int8_t slot)
{
  (void) user;
  return preset_store(cfg, slot);
}

static int8_t fram_load(void* user, EngineConfig* cfg, int8_t slot)
{
  (void) user;
  return preset_load(cfg, slot);
}

static int8_t fram_clear(void* user)
{
  (void) user;
  return preset_clear();
}

static const PresetIo fram_preset_io = {.store = fram_store, .load = fram_load, .clear = fram_clear, .user = NULL};

void bmcv_init(uint16_t _mpc_interrupt_pin, ADC_TypeDef* _slider_adc)
{
  mpc_interrupt_pin = _mpc_interrupt_pin;
  slider_adc        = _slider_adc;

#if BMCV_PROFILE
  // From the clock the RCC actually came up on rather than a literal 144, so
  // the microsecond columns stay right if SystemClock_Config ever changes.
  us_per_cycle   = 1000000.0f / (float) SystemCoreClock;
  overrun_cycles = ENGINE_TICK_US * (SystemCoreClock / 1000000u);
#endif

  bmcv_instance_init(&bmcv, &fram_preset_io, 0);
}

void bmcv_handle_adc_conversion_complete(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == slider_adc)
  {
    slider_adc_value = HAL_ADC_GetValue(hadc);
  }
}

void bmcv_handle_gpio_exti(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == mpc_interrupt_pin || GPIO_Pin == 0)
  {
    isr_flag_set(&mcp_poll);
  }
}

// The expander's SPI transfer has landed. The DAC's no longer comes through
// here - see bmcv_handle_dac_complete - so this is the MCP's alone, and it no
// longer needs a timestamp: the DAC was the only thing here that recorded one.
void bmcv_handle_txrx_complete(SPI_HandleTypeDef* hspi) { mcp_handle_txrx_complete(hspi); }

// A DAC transaction has landed. This only says so; the timer arms the next one.
//
// Called from the RX DMA channel's own vector, not from HAL's SPI completion
// callback, because that callback is where a third of the module's CPU was
// going - see dac_adc.c. What arrives here is already decoded.
//
// Re-arming from the completion was tried once, and on the module it produced a
// stable transaction rate and a dead output: transfers running, nothing
// latching. What it disturbs was never proven, and the likeliest candidate is
// the DAC's own SYNC line - raised as the transfer completes and lowered again
// by the next arm, which back to back is a microsecond or two rather than the
// tens the loop was giving it.
//
// Which is the second reason the timer owns the service, after the fixed
// cadence. Armed on a clock, the gap between raising SYNC and lowering it is
// the chunk period less the transfer, and it is that on every chunk rather than
// whenever the loop got round to it. The same goes for the ADC's conversion
// window, which is the interval between one transaction's CNVST pulse and the
// next transaction reading the result: also a whole chunk period, and now the
// same one every time.
void bmcv_handle_dac_complete(void)
{
  [[maybe_unused]] const uint32_t t_start = PROFILE_NOW();

  // Recorded only when the transfer was ours. The DMA vector is shared, so
  // dacadc_dma_isr() returns 0 for somebody else's flags after a handful of
  // instructions - counting those would average the cheap early return into the
  // very number the substep count has to be decided against.
  if (dacadc_dma_isr())
  {
    isr_flag_set(&dac_poll);

#if BMCV_PROFILE
    span_record(&bmcv_profile.dac_cplt, PROFILE_NOW() - t_start);
#endif
  }
}

void bmcv_poll_tasks()
{
  task = task + 1;
  isr_flag_set(&led_poll);

  if (task == 1)
  {
    isr_flag_set(&mcp_poll);
  }
  else if (task >= 3)
  {
    task = 0;
  }

  // The MIDI slot that used to sit at task == 2 is gone, and the cycle keeps
  // its length of three so the MCP poll rate is unchanged. Draining the MIDI
  // queue is now gated on the endpoint being free instead - see bmcv_tick.
  //
  // This timer runs at about 303Hz, so a third of it was ~101Hz: below the rate
  // midi_out publishes at, which overflowed the queue whenever more than a
  // couple of channels were moving, and put 10ms of jitter on a clock message
  // whose whole interval is 20.8ms at 120BPM.
}

static uint32_t last_led_flush;
static uint32_t last_engine_us; // start of the tick being interpolated across
static uint32_t next_engine_us; // when the next one is due
static uint8_t engine_started;  // so the first tick is not counted as a resync

// Ship whatever midi_out has queued: the eight channel outputs and four CV
// inputs as control changes, plus the clock. What to say is decided in
// midi_out.c, which is core code with no USB in it; this is only the transport.
//
// It replaces a loop that sent the four inputs on CC 0x10..0x13 and was the
// only use the USB MIDI stack had - undocumented, and its CCs now belong to the
// channels. See docs/midi.md for the mapping.
//
// One transfer's worth per call, and only when the endpoint is free. Anything
// that does not fit stays queued for the next pass; a transfer holds 16 events,
// which is more than one publish slot produces.
static void midi_publish(void)
{
  MidiMsg msgs[MIDI_MSGS_PER_TRANSFER];
  midi_send_msgs(msgs, midi_out_drain(&bmcv.midi_out, msgs, MIDI_MSGS_PER_TRANSFER));
}

// The engine produces one set of levels per ENGINE_TICK_US. The DAC service
// ships a frame about four times as often, and every one of those extra frames
// used to re-send the value unchanged - a staircase whose steps are one tick
// wide, 263us of it on a scope, with the zero-order-hold images sitting at the
// tick rate where a cheap output filter barely touches them and a VCA makes
// them audible.
//
// So slide between the levels rather than repeating them. The bus was already
// running fast enough to do this - the frames were being sent either way, they
// just carried the same number four times - so this costs no extra traffic and
// nothing on the analog side had to change.
//
// On elapsed time rather than a sub-step counter: the tick period is not exact.
// It jitters by a loop pass and overruns about once every two seconds, and a
// fraction of the time that has actually passed stays right through both, where
// a counter would drift against them.
//
// One tick of output latency is the price - the levels computed on tick N are
// on the pins across tick N+1. Extrapolating would avoid the latency and
// overshoot on every direction change, which is a worse artifact than the one
// being removed here.
//
// Gates and stepped shapes are interpolated too, deliberately. A step becomes a
// 263us ramp, which is well under what reads as a slew and is gentler through a
// VCA than the broadband click a hard edge makes. What it does cost is a fixed
// ~50us before a gate crosses a downstream trigger threshold, and a transient
// only one tick long reaching about three quarters of its peak.
//
// Firmware-side, on the way out. engine_state still holds the exact per-tick
// levels, so internal trigger routing, the LED render and the other hosts see
// what they always saw.

// How late a tick may be before the scheduler stops trying to keep the grid and
// rebases. Small enough that ordinary loop jitter stays on the grid, large
// enough to catch a real overrun, which is the only thing that shortens an
// interval.
//
// What it costs is drift, and the drift is worth checking rather than assuming:
// on a patch with six stepped channels, 6.1% of ticks overrun and engine_fps
// reads 1979 against a nominal 2000. That 1% is the overruns alone - 0.061 x
// ~80us of lateness is the 5us a tick it works out to - so nothing is rebasing
// that should not be. If engine_fps ever sags further than the overrun rate
// explains, this is too small for the loop it is running in.
#define ENGINE_TICK_SLACK_US (ENGINE_TICK_US / 16)

// What the output ramp is spread over, tracked rather than assumed - see
// dac_ramp_span_us and dac_write_interpolated(). This is where it starts and
// what it returns to when the engine is keeping up.
#define DAC_RAMP_SPAN_US (ENGINE_TICK_US - ENGINE_TICK_SLACK_US)

// The interval the ramp is spread over, following the interval the engine
// actually achieves.
//
// A fixed span is right only while the engine keeps its period. Under a load
// that holds ticks at, say, 800us - a crossfader sweep empties the stepped
// caches every tick, which is the documented worst case - a 469us ramp finishes
// and then holds flat for the remaining 331us of every tick. That is not a step
// and so not a click, but it is a 40% duty hold at the tick rate, arriving
// exactly when the scene blend is making the deltas large.
//
// Shrinks at once and grows slowly, which makes it track the *shortest* recent
// interval rather than the mean. That direction is the whole point: a span
// longer than the next interval leaves the ramp unfinished and steps the
// output, while a span shorter than it only costs a flat tail. Growth is
// deliberately slow for the same reason - tick lengths under load vary, and the
// mean of them is longer than the short ones.
static uint32_t dac_ramp_span_us = DAC_RAMP_SPAN_US;

static int16_t dac_level_prev[N_CHANNELS];
static int16_t dac_level_curr[N_CHANNELS];

static void dac_write_interpolated(uint32_t now_us)
{
  // The span the ramp is spread over: the interval the engine is achieving, not
  // the one it is nominally asked for. See dac_ramp_span_us for how it is
  // tracked, and the swap in bmcv_main for what happens when it guesses long.
  const uint32_t span = dac_ramp_span_us;

  uint32_t elapsed = now_us - last_engine_us;
  if (elapsed > span)
  {
    elapsed = span; // a late tick holds at the target, never past it
  }

  const float frac = (float) elapsed / (float) span;

  // Only the pair about to go out. All eight used to be interpolated on every
  // chunk and six of them overwritten before they were ever clocked anywhere -
  // a transaction carries one word to each DAC of the chain, so two outputs.
  // The values are identical either way, because a slot is now written at the
  // same instant it is sent rather than on each of the four chunks before it.
  const uint8_t group = dacadc_next_group();

  for (uint8_t c = 0; c < N_CHANNELS; c++)
  {
    const uint8_t idx = bmcv.ux_setup->channels[c].dac_channel;
    if ((uint8_t) (idx >> 1) != group)
    {
      continue; // DAC_BUF pairs the two chips per channel letter: idx 2g, 2g+1
    }

    const float prev = (float) dac_level_prev[c];
    const float v    = prev + ((float) dac_level_curr[c] - prev) * frac;
    dacadc_write(idx, (int16_t) v);
  }
}

// One DAC transaction: the levels for this instant, then the transfer that
// carries them.
//
// Ordered so the bookkeeping is done before anything can complete: a DMA
// completion can only arrive after dacadc_dma_next(), so by the time anything
// can run this again, last_dac_poll and the rate already describe the transfer
// that is in flight.
static void dac_service(uint32_t now_us)
{
  // Consumed before the work, not after it: a completion landing while the
  // frame below is being armed is a request for the *next* chunk, and clearing
  // afterwards would throw it away.
  isr_flag_take(&dac_poll);

  // Immediately before the frame is armed, so the levels it carries are the
  // ones interpolated for this instant rather than for the last tick.
  dac_write_interpolated(now_us);

  bmcv.engine_state.dac_fps = rate_smooth_hz(bmcv.engine_state.dac_fps, now_us - last_dac_poll);
  last_dac_poll             = now_us;

  dacadc_dma_next();
}

// The DAC's own clock. A timer interrupt, on the fixed cadence DAC_CHUNK_US
// asks for and nothing else gets a say in.
//
// The output rate used to be a property of how busy the engine was, because the
// service ran from the main loop and the loop spends the whole of a heavy tick
// inside engine_tick(). Eight stepped channels took the frame rate from 4.5kHz
// to under 1kHz - the interpolation stopped oversampling anything and started
// smoothing over an output that could not keep up - and it did so exactly when
// the most was being asked of the module.
//
// Interrupt priority is what makes this safe rather than clever. This runs at
// the same NVIC priority as the SPI DMA completion, so the two cannot preempt
// each other: dac_service() can never be re-entered against
// dacadc_dma_complete(), and no lock is needed between them. Nothing in the
// main loop touches the service any more, so there is no third caller to race.
void bmcv_dac_tick(uint32_t now_us)
{
  // Never arm a transfer over one still running.
  //
  // A skipped chunk is a chunk, not a stall: the next timer tick tries again
  // and dac_write_interpolated() reads the clock rather than counting substeps,
  // so the output lands where the elapsed time says it should either way. A
  // transfer that errors still reports a completion - see dacadc_dma_isr - so
  // the chain recovers by dropping one frame rather than by a separate path.
  if (!isr_flag_peek(&dac_poll))
  {
    return;
  }

  [[maybe_unused]] const uint32_t t_start = PROFILE_NOW();

  dac_service(now_us);

#if BMCV_PROFILE
  span_record(&bmcv_profile.dac, PROFILE_NOW() - t_start);
#endif
}

void bmcv_main(uint32_t now_us)
{
  /* ---- hardware in ------------------------------------------------ */
  // The DAC is not here any more. It runs off its own timer interrupt - see
  // bmcv_dac_tick() - because serving it from this loop made the output rate a
  // property of how long the last engine tick took.

  // Taken before mcp_read() rather than cleared after it. An expander interrupt
  // arriving during the read is a *new* set of edges, and the old clear-after
  // wiped it - a button or detent the module never saw. Put back if the read
  // could not start, so the request survives a busy SPI.
  if (isr_flag_take(&mcp_poll))
  {
    if (mcp_read())
      mcu_read_buttons();
    else
      isr_flag_set(&mcp_poll);
  }

  /* ---- pure engine, on a fixed period ----------------------------- */
  //
  // main() calls this in a bare `while (1)`, so the engine used to run once per
  // iteration and the interval between DAC updates was however long the last
  // pass happened to take - a mute ramp, an LED flush and a USB frame all land
  // in some passes and not others. The oscillators are dt-driven and stay
  // correct through that, but the samples leaving the module are not evenly
  // spaced, and an LFO's edges carry that jitter.
  //
  // The deadline advances by a fixed period rather than restarting from now_us.
  // Restarting made each tick begin late from wherever the last one landed and
  // never corrected, so a 250us period ran at 263 and engine_fps read 3800
  // against a nominal 4000. That 5% was harmless while the DAC only repeated
  // values; it stopped being harmless once the output interpolates across a
  // tick, because the interpolation divides by the nominal period and so
  // reached its target 13us early and sat flat there for the rest of every
  // tick. Nominal and actual have to agree for that fraction to mean anything.
  //
  // Still not a catch-up. A tick that lands late rebases the schedule rather
  // than being repaid as a burst of ticks with made-up timestamps: the engine
  // is dt-driven, so one long dt is the honest account of a loop that could not
  // keep up, and `resyncs` counts the ones that lost a whole period.
  //
  // Rebased on *any* late tick, not only one a whole period behind. A fixed
  // grid repays a long tick with a short one - the deadline it missed has
  // already passed, so the next tick fires back to back - and the intervals
  // come out 500, 700, 300, 500. The 300 is what was heard: the interpolation
  // cannot finish a ramp inside an interval shorter than its span, so the
  // output was still 57% short of the target when the pair advanced, and it
  // stepped the rest of the way. Measured at one overrun every three seconds,
  // which is a crackle rather than a tone.
  //
  // What rebasing costs is grid drift, which is what the fixed grid was for.
  // At the measured overrun rate that is ~60us per second - engine_fps 2000.03
  // becomes 1999.9, 0.006%. The 5% the fixed grid was introduced to fix came
  // from rebasing on *every* tick, not on one in six thousand.
  const int32_t late_us = (int32_t) (now_us - next_engine_us);

  if (late_us >= 0)
  {
    // Late by more than the slack: rebase, so the interval this tick opens is a
    // whole period rather than what is left of one.
    //
    // Measured against `late_us` and not against the deadline already advanced
    // past it, which is the version that did not work: a 700us tick leaves the
    // next tick 200us late, that is less than a period, and the test never
    // fired on the case it was written for.
    if (late_us > (int32_t) ENGINE_TICK_SLACK_US)
    {
#if BMCV_PROFILE
      if (engine_started && late_us >= (int32_t) ENGINE_TICK_US)
      {
        bmcv_profile.resyncs++; // a whole period lost, not merely a late tick
      }
#endif
      next_engine_us = now_us + ENGINE_TICK_US;
    }
    else
    {
      next_engine_us += ENGINE_TICK_US;
    }

    const uint8_t engine_started_before = engine_started;
    engine_started                      = 1;

    /* ---- hardware out, for the tick that just ended ----------------- */
    // Advanced here and not after engine_tick(), because the pair and the
    // instant the interpolation measures from have to be the same instant.
    //
    // They were not. `last_engine_us` was taken here and the pair was advanced
    // after the engine had run - 204us later, measured. So for the first 204us
    // of every tick, frac restarted at 0 against the pair the *previous* tick
    // had already walked to frac 1: the output jumped back a whole inter-tick
    // delta, re-climbed 40% of it, then jumped forward again when the pair
    // finally moved. A 2kHz sawtooth of one tick's delta, riding on the signal
    // - 1.57V p-p on a 100Hz sine at full swing, which is what was on the scope
    // and what was heard.
    //
    // It could not be filtered and it could not be outrun: 2kHz is only 2.8x
    // above the 723Hz pole at the jack, and the amplitude is set by the tick
    // rate, so doubling DAC_SUBSTEPS sampled the same broken trajectory more
    // finely and changed nothing audible.
    //
    // channels_gated_level[] still holds what the last engine_tick() computed,
    // which is exactly the far end of the span about to be interpolated. Mute
    // is an output-stage gain and engine_tick has already applied it.
    //
    // How far the ramp actually got across the interval just ended, measured
    // against the span that was in force for it.
    const uint32_t span_was = dac_ramp_span_us;
    const uint32_t ran_for  = now_us - last_engine_us;
    const float frac_end    = (!engine_started_before || ran_for >= span_was) ? 1.0f : (float) ran_for / (float) span_was;

    // The new ramp starts from where the output actually is, which is not always
    // where the old one was aiming.
    //
    // dac_ramp_span_us is set from the interval that just *ended*, so it is one
    // interval behind: when a sustained load lets go - the end of a crossfade -
    // the span in force is still the long one while the interval is back to
    // nominal, frac tops out short of 1, and taking the far end as the new near
    // end would step the output by the difference. Carrying the value the ramp
    // actually reached makes the join continuous whatever the span guessed. In
    // the steady state frac_end is exactly 1 and this is the far end, so
    // nothing changes where nothing is wrong.
    //
    // Computed before the mask; only the stores need to be inside it.
    int16_t next_prev[N_CHANNELS];
    for (uint8_t c = 0; c < N_CHANNELS; c++)
    {
      const float p = (float) dac_level_prev[c];
      next_prev[c]  = (int16_t) (p + ((float) dac_level_curr[c] - p) * frac_end);
    }

    // Under a mask, because "the same instant" has to be true for the DAC
    // interrupt as well as for the reader of this code. The service runs every
    // DAC_CHUNK_US and preempts the main loop freely, so without this it can
    // land between the new origin and a channel the loop has not reached yet -
    // and read frac 0 against the *previous* pair, which is one whole delta
    // low. One chunk in seventy lands in that window, and the wrong level then
    // sits on the pin until its next frame, up to DAC_CHANNELS chunks later.
    //
    // The mask spans the origin, the span and two stores per channel - eighteen
    // in all, well under a microsecond, against the 4.4us the service it defers
    // costs to run.

    // The interval just achieved, less the same slack the scheduler allows.
    uint32_t span_target = DAC_RAMP_SPAN_US;
    if (engine_started_before)
    {
      const uint32_t seen = now_us - last_engine_us;

      span_target = seen - (seen / 16u);
      if (span_target < ENGINE_TICK_US / 4u)
      {
        span_target = ENGINE_TICK_US / 4u;
      }
      else if (span_target > ENGINE_TICK_US * 4u)
      {
        span_target = ENGINE_TICK_US * 4u;
      }
    }

    // Down at once, up a sixty-fourth at a time - and never by nothing. The
    // step is rounded up because a bare shift truncates to zero once the gap is
    // under 64us, which would leave the span stalled just short of the interval
    // for as long as the load lasted, holding the output flat for the last of
    // every tick.
    uint32_t span_next = span_target;
    if (span_target > dac_ramp_span_us)
    {
      span_next = dac_ramp_span_us + ((span_target - dac_ramp_span_us) >> 6) + 1u;
      if (span_next > span_target)
      {
        span_next = span_target; // never overshoot: a span past the interval steps the output
      }
    }

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    dac_ramp_span_us = span_next;

    last_engine_us = now_us;
    for (uint8_t c = 0; c < N_CHANNELS; c++)
    {
      dac_level_prev[c] = next_prev[c];
      dac_level_curr[c] = bmcv.engine_state.channels_gated_level[c];
    }

    __set_PRIMASK(primask);

    [[maybe_unused]] const uint32_t t_tick_start = PROFILE_NOW();

    // Reset and forget-everything, asked for from outside. Before the input is
    // folded because it may rebuild the whole instance, which is not something
    // to do halfway through a tick.
    usblink_take_remote_command(&bmcv.command);
    bmcv_instance_take_command(&bmcv, &fram_preset_io, now_us);

    // Anything a host has sent over MIDI, moved into the instance here rather
    // than in the USB interrupt that received it: input_fold is about to read
    // this mailbox, and an interrupt writing it halfway through that read is
    // the one thing its design does not tolerate. Nothing can interleave
    // between these two lines.
    usblink_take_remote_input(&bmcv.input.remote);

    // input_fold points bmcv.ux.hw_state at the frame it just filled.
    // engine_fps is measured inside engine_tick, so every host agrees on it.
    uint8_t dirty = bmcv_state_update(now_us);

    [[maybe_unused]] const uint32_t t_engine_start = PROFILE_NOW();

    engine_tick(&bmcv.ux, now_us, dirty);

    [[maybe_unused]] const uint32_t t_engine_end = PROFILE_NOW();

    // Outside the engine span, so that span stays comparable with the builds
    // before this existed. It only fills a queue - what leaves the endpoint is
    // decided in the housekeeping block below.
    //
    // Here rather than in bmcv_instance_tick because the firmware does not use
    // that call, for the reason instance.h gives: it interleaves the profiling
    // above between the two halves.
    midi_out_publish(&bmcv.midi_out, &bmcv.engine_state, &bmcv.input.curr, now_us);

#if BMCV_PROFILE
    const uint32_t tick_cycles = PROFILE_NOW() - t_tick_start;

    span_record(&bmcv_profile.engine, t_engine_end - t_engine_start);
    span_record(&bmcv_profile.tick, tick_cycles);

    if (tick_cycles > overrun_cycles)
    {
      bmcv_profile.overruns++;
    }

    bmcv_profile.load = bmcv_profile.tick.avg_us * (1.0f / (float) ENGINE_TICK_US);
    bmcv_profile.ticks++;
#endif
  }

  /* ---- housekeeping ----------------------------------------------- */
  // Outside the tick: both are gated on their own transport being idle, and
  // neither should be able to hold up the engine or be held up by it.
  // Whenever the endpoint is free, rather than on a timer slot: what to send
  // and how often is midi_out's decision, and the transport should only be
  // asking how much of it fits. An empty queue costs a state read and a
  // compare, which is what the LED flush below does with its DMA for the same
  // reason.
  if (midi_idle())
  {
    midi_publish();
  }

  // Snapshots go out on their own endpoint, so this competes with nothing.
  //
  // Over MIDI they shared one with the control changes above, and since only a
  // System Real-Time message may interleave a SysEx, every snapshot had to hold
  // the endpoint for eleven milliseconds while the engine's output waited. That
  // whole problem is gone with the transport that caused it.
  //
  // Still outside the tick, so the copy it takes is between two of them and is
  // internally consistent.
  usblink_poll(&bmcv);

  if (isr_flag_peek(&led_poll) && ws2811_dma_completed())
  {
    isr_flag_take(&led_poll);
    bmcv_flush_leds();
    ws2811_update();
    bmcv.engine_state.led_fps = rate_smooth_hz(bmcv.engine_state.led_fps, now_us - last_led_flush);
    last_led_flush            = now_us;
  }
}

// Push the rendered framebuffer to the LED driver. The only place LED colour
// data crosses from the engine into hardware, and so the only place the 8.8
// framebuffer becomes eight bits.
//
// The remainder each LED is owed lives here rather than in the framebuffer: it
// is a property of this output path and of its rate, and nothing upstream of
// the flush should be able to see that the panel cannot draw what it was asked
// for. ui_render rebuilds at 125Hz and this runs at ~300, so most frames
// re-quantise an unchanged framebuffer - which is exactly what spreads the
// error.
void bmcv_flush_leds(void)
{
  static LedDither dither[LED_COUNT];
  uint8_t rgb[LED_COUNT * 3];

  led_fb_quantize(bmcv.engine_state.leds, dither, rgb, LED_COUNT);

  for (int16_t i = 0; i < LED_COUNT; i++)
  {
    ws2811_setled_rgb(i, rgb[i * 3], rgb[i * 3 + 1], rgb[i * 3 + 2]);
  }
}

// Read the peripherals into an InputSample. This is all that is left of the
// old bmcv_state_update: the bookkeeping it used to do around these reads -
// press levels, encoder deltas, clock dispatch, slider CV, autosave - now
// lives in input_fold.c, where a host without an STM32 can reach it.
uint8_t bmcv_state_update(uint32_t now_us)
{
  // Zeroed rather than left to the field-by-field fill below, so adding a
  // field to InputSample cannot leave one reading stack garbage.
  InputSample sample = {0};

  sample.slider_raw = slider_adc_value;

  for (uint8_t ch = 0; ch < N_INPUTS; ch++)
  {
    sample.cv_raw[ch]  = get_adc(ch);
    sample.cv_trig[ch] = adc_read_trig_state(ch);
  }

  sample.midi_clock_trig = midi_read_clock_trig();
  sample.midi_reset_trig = midi_read_reset_trig();

  for (uint8_t b = 0; b < N_BUTTONS; b++)
  {
    sample.button_down[b] = get_btn_state(b);
  }

  for (uint8_t e = 0; e < N_ENCODERS; e++)
  {
    sample.encoder_pos[e] = get_enc_state(e);
  }

  return input_fold(&bmcv.input, &bmcv.ux, &sample, now_us);
}
