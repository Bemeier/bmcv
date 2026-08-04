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
static uint32_t last_engine_us;

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

void bmcv_main(uint32_t now_us)
{
  /* ---- hardware in ------------------------------------------------ */
  // Every pass: both are event-driven, and a DMA completion should be picked up
  // when it lands rather than at the next engine tick.
  if (dac_poll == 1 || dacadc_error())
  {
    dac_poll = 0;
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
  // A floor rather than a catch-up: the period is ENGINE_TICK_US plus whatever
  // is left of the iteration that crossed it, which is short, and if the loop
  // ever cannot keep up then a longer dt is the honest thing to hand the engine
  // - a burst of made-up ticks afterwards would not be.
  //
  // It is also what makes engine_fps mean something. 4kHz was an estimate the
  // three hosts already assume (sim_rt.h); the module now actually holds it,
  // provided the loop can - if it cannot, this changes nothing and engine_fps
  // says so.
  if ((uint32_t) (now_us - last_engine_us) >= ENGINE_TICK_US)
  {
    last_engine_us = now_us;

    // input_fold points bmcv.ux.hw_state at the frame it just filled.
    // engine_fps is measured inside engine_tick, so every host agrees on it.
    uint8_t dirty = bmcv_state_update(now_us);

    engine_tick(&bmcv.ux, now_us, dirty);

    /* ---- hardware out --------------------------------------------- */
    // channels_gated_level[] rather than channels_output_level[]: mute is an
    // output-stage gain, and engine_tick has already applied it.
    for (uint8_t c = 0; c < N_CHANNELS; c++)
    {
      dacadc_write(bmcv.ux_setup->channels[c].dac_channel, bmcv.engine_state.channels_gated_level[c]);
    }
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
