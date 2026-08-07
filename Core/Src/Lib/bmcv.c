#include "bmcv.h"
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
#include "mcp.h"
#include "midi.h"
#include "presets.h"
#include "stm32g474xx.h"
#include "ws2811.h"
#include <stdint.h>

// Hardware Config
static uint16_t mpc_interrupt_pin;
static ADC_TypeDef* slider_adc;
static volatile uint16_t slider_adc_value;

// Task scheduler
static uint8_t task      = 0;
static uint8_t dac_poll  = 1;
static uint8_t mcp_poll  = 0;
static uint8_t led_poll  = 0;
static uint8_t midi_poll = 0;

// The module. One struct holding config, signal path, interaction state, the
// input layer and the wiring between them - see instance.h. The firmware has
// exactly one; a simulator has one per instance. Declared in bmcv.h, and
// external only so a debugger can name it.
BmcvInstance bmcv;

// What one tick costs, measured with the Cortex-M4's DWT cycle counter: a
// free-running 32-bit counter in the debug block that increments once per CPU
// clock. At 144MHz that is 6.9ns a count, against the 1us of the TIM2 timestamp
// the rest of this file runs on - a tick is tens of microseconds, so TIM2 was
// never going to tell us more than its order of magnitude. main() turns the
// counter on (init_cycle_counter); the subtraction is unsigned, so the 29.8s
// wrap needs no handling for a span this short.
//
// Left in the build: the probe is four register loads and a few float ops
// against a 250us budget, under 0.2%. Set to 0 to compile it out.
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

static const PresetIo fram_preset_io = {.store = fram_store, .load = fram_load, .user = NULL};

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
    mcp_poll = 1;
  }
}

void bmcv_handle_txrx_complete(SPI_HandleTypeDef* hspi)
{
  mcp_handle_txrx_complete(hspi);

  if (dacadc_dma_complete(hspi))
  {
    dac_poll = 1;
  }
}

void bmcv_poll_tasks()
{
  task     = task + 1;
  led_poll = 1;

  if (task == 1)
  {
    mcp_poll = 1;
  }
  else if (task == 2)
  {
    midi_poll = 1;
  }
  else
  {
    task = 0;
  }
}

static uint32_t last_dac_poll;
static uint32_t last_engine_us; // start of the tick being interpolated across
static uint32_t next_engine_us; // when the next one is due
static uint8_t engine_started;  // so the first tick is not counted as a resync

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
#define DAC_SUBSTEPS 4
#define DAC_CHUNK_US (ENGINE_TICK_US / (DAC_SUBSTEPS * DAC_CHANNELS))

// The four CV inputs, sent out of the USB port as MIDI control changes: channel
// 1, CC 0x10..0x13, each input's ADC reading scaled to 0..127.
//
// It reads like a debugging aid left running, and it may be one - it is the
// only thing the USB MIDI stack is used for, nothing asks for it and nothing
// documents it. It is kept because it costs a USB frame every third poll and
// somebody may be patching the module into a DAW with it; it is behind a
// switch, and named, so that turning it off is one line rather than an
// archaeology exercise.
#define BMCV_MIDI_CC_INPUTS 1

static void midi_publish_inputs(void)
{
#if BMCV_MIDI_CC_INPUTS
  for (uint8_t ch = 0; ch < DAC_CHANNELS; ch++)
  {
    MIDI_addToUSBReport(0, 0xB0, 0x10 + ch, sclamp(get_adc(ch) / 32, 0, 127));
  }
  update_midi();
#endif
}

// Exponential average, so a single slow loop does not make the readout jump.
static float fps_smooth(float prev, uint32_t dt_us)
{
  if (dt_us == 0)
    return prev;
  return prev * 0.95f + 0.05f * (1000000.0f / (float) dt_us);
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
static int16_t dac_level_prev[N_CHANNELS];
static int16_t dac_level_curr[N_CHANNELS];

static void dac_write_interpolated(uint32_t now_us)
{
  uint32_t elapsed = now_us - last_engine_us;
  if (elapsed > ENGINE_TICK_US)
  {
    elapsed = ENGINE_TICK_US; // a late tick holds at the target, never past it
  }

  const float frac = (float) elapsed * (1.0f / (float) ENGINE_TICK_US);

  for (uint8_t c = 0; c < N_CHANNELS; c++)
  {
    const float prev = (float) dac_level_prev[c];
    const float v    = prev + ((float) dac_level_curr[c] - prev) * frac;
    dacadc_write(bmcv.ux_setup->channels[c].dac_channel, (int16_t) v);
  }
}

void bmcv_main(uint32_t now_us)
{
  /* ---- hardware in ------------------------------------------------ */
  // Every pass: both are event-driven, and a DMA completion should be picked up
  // when it lands rather than at the next engine tick.
  if ((dac_poll == 1 || dacadc_error()) && (uint32_t) (now_us - last_dac_poll) >= DAC_CHUNK_US)
  {
    dac_poll = 0;
    // Immediately before the frame is armed, so the levels it carries are the
    // ones interpolated for this instant rather than for the last tick.
    dac_write_interpolated(now_us);
    dacadc_dma_next();
    bmcv.engine_state.dac_fps = fps_smooth(bmcv.engine_state.dac_fps, now_us - last_dac_poll);
    last_dac_poll             = now_us;
  }

  if (mcp_poll == 1 && mcp_read())
  {
    mcp_poll = 0;
    mcu_read_buttons();
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
  // Still not a catch-up. A tick that lands a whole period late is dropped and
  // the schedule resynchronised, rather than repaid as a burst of ticks with
  // made-up timestamps: the engine is dt-driven, so one long dt is the honest
  // account of a loop that could not keep up, and `resyncs` counts how often
  // that has happened.
  if ((int32_t) (now_us - next_engine_us) >= 0)
  {
    next_engine_us += ENGINE_TICK_US;

    if ((int32_t) (now_us - next_engine_us) >= (int32_t) ENGINE_TICK_US)
    {
      next_engine_us = now_us + ENGINE_TICK_US;
#if BMCV_PROFILE
      if (engine_started)
      {
        bmcv_profile.resyncs++;
      }
#endif
    }

    engine_started = 1;
    last_engine_us = now_us;

    [[maybe_unused]] const uint32_t t_tick_start = PROFILE_NOW();

    // input_fold points bmcv.ux.hw_state at the frame it just filled.
    // engine_fps is measured inside engine_tick, so every host agrees on it.
    uint8_t dirty = bmcv_state_update(now_us);

    [[maybe_unused]] const uint32_t t_engine_start = PROFILE_NOW();

    engine_tick(&bmcv.ux, now_us, dirty);

    [[maybe_unused]] const uint32_t t_engine_end = PROFILE_NOW();

    /* ---- hardware out --------------------------------------------- */
    // Advance the pair the DAC service slides between, rather than writing the
    // buffer here: what reaches the pins is now interpolated between two ticks,
    // and this is the tick that just became the far end of it.
    //
    // channels_gated_level[] rather than channels_output_level[]: mute is an
    // output-stage gain, and engine_tick has already applied it.
    for (uint8_t c = 0; c < N_CHANNELS; c++)
    {
      dac_level_prev[c] = dac_level_curr[c];
      dac_level_curr[c] = bmcv.engine_state.channels_gated_level[c];
    }

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
  if (midi_poll && midi_idle())
  {
    midi_poll = 0;
    midi_publish_inputs();
  }

  if (led_poll && ws2811_dma_completed())
  {
    led_poll = 0;
    bmcv_flush_leds();
    ws2811_update();
  }
}

// Push the rendered framebuffer to the LED driver. The only place LED colour
// data crosses from the engine into hardware.
void bmcv_flush_leds(void)
{
  for (int16_t i = 0; i < LED_COUNT; i++)
  {
    const LedRgb* led = &bmcv.engine_state.leds[i];
    ws2811_setled_rgb(i, led->r, led->g, led->b);
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
