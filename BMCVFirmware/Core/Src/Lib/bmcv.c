#include "bmcv.h"
#include "assign.h"
#include "channel.h"
#include "clock_sync.h"
#include "color_presets.h"
#include "config_validate.h"
#include "dac_adc.h"
#include "dac_adc_hal.h"
#include "engine.h"
#include "error.h"
#include "helpers.h"
#include "hw_setup.h"
#include "led_fb.h"
#include "mcp.h"
#include "midi.h"
#include "presets.h"
#include "scene.h"
#include "state.h"
#include "stm32g474xx.h"
#include "ux_setup.h"
#include "ux_state.h"
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

// Presets  Read/Write
static uint32_t last_write          = 0; // timestamp of last write
static int last_crc                 = 0;
static uint32_t write_indicator_for = 0;

// System Config
static const HwSetup* hw_setup;
static const UxSetup* ux_setup;

// Hardware State Buffer
static uint8_t state_idx;
static HwState state[STATE_RINGBUF_SIZE];
static HwState* prev_hw = &state[0];
static HwState* curr_hw = &state[1];

// Engine & UX State
static UxState ux_state;
static EngineConfig engine_config;
static EngineState engine_state;
// uint32_t ux_update_time; // last ux update

void bmcv_init(uint16_t _mpc_interrupt_pin, ADC_TypeDef* _slider_adc)
{
  hw_setup = HwSetup_Get();
  ux_setup = UxSetup_InitFromHw(hw_setup);

  mpc_interrupt_pin = _mpc_interrupt_pin;
  slider_adc        = _slider_adc;

  Clock_Init();

  ux_state.hw_setup                     = hw_setup;
  ux_state.ux_setup                     = ux_setup;
  ux_state.engine_config                = &engine_config;
  ux_state.engine_state                 = &engine_state;
  ux_state.engine_state->selected_param = CH_PARAM_SHP;
  ux_state.engine_state->shift_state    = SHIFT_STATE_NONE;
  assign_reset(&ux_state); // assign_src_id must start at -1, not 0
  // ux_update_time = 0;

  for (uint8_t c = 0; c < N_ENCODERS; c++)
  {
    init_channel(&ux_setup->channels[c], &ux_state);
  }

  if (!preset_load(&engine_config, FRAM_CONFIG_SLOTS - 1))
  {
    engine_config.input_mode[0] = INPUT_CLOCK;
    engine_config.input_mode[1] = INPUT_RESET;
    engine_config.input_mode[2] = INPUT_DEFAULT;
    engine_config.input_mode[3] = INPUT_DEFAULT;
    engine_config.scene_a       = 0;
    engine_config.scene_b       = 6;
    engine_config.quantize_mask = 0b111111111111;
    for (uint8_t c = 0; c < N_ENCODERS; c++)
    {
      engine_config.channel_state[c].src_input     = -1;
      engine_config.channel_state[c].quantize_mode = QUANTIZE_DISABLED;
    }

    for (uint8_t c = 0; c < N_ENCODERS; c++)
    {
      reset_channel(&ux_setup->channels[c], &ux_state, -1);
    }
    error_set(6);
  }

  // Holds for the defaults above as well as a loaded preset, so the rest of
  // the firmware can index on these fields unconditionally.
  config_validate(&engine_config);

  last_crc = crc32(&engine_config, sizeof(EngineConfig));
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

uint32_t last_dac_poll;

void bmcv_main(uint32_t now_us)
{
  /* ---- hardware in ------------------------------------------------ */
  if (dac_poll == 1 || dacadc_error())
  {
    dac_poll = 0;
    dacadc_dma_next();
    uint32_t dac_dt                = now_us - last_dac_poll;
    float dac_fps                  = 1000000.0f / dac_dt;
    ux_state.engine_state->dac_fps = ux_state.engine_state->dac_fps * 0.95f + 0.05f * dac_fps;
    last_dac_poll                  = now_us;
  }

  if (mcp_poll == 1 && mcp_read())
  {
    mcp_poll = 0;
    mcu_read_buttons();
  }

  int8_t dirty      = bmcv_state_update(now_us);
  ux_state.hw_state = curr_hw;

  float engine_fps                  = 1000000.0f / ux_state.hw_state->dt;
  ux_state.engine_state->engine_fps = ux_state.engine_state->engine_fps * 0.95f + 0.05f * engine_fps;

  /* ---- pure engine ------------------------------------------------ */
  engine_tick(&ux_state, now_us, dirty);

  /* ---- hardware out ----------------------------------------------- */
  for (uint8_t c = 0; c < N_CHANNELS; c++)
  {
    write_channel_dac(&ux_setup->channels[c], &ux_state);
  }

  if (write_indicator_for > 0)
  {
    led_set_hsv(&ux_state, ux_setup->ctrl_buttons[1].led, HUE_RED, SAT_MAX, VAL_MED);
  }

  if (midi_poll && midi_idle())
  {
    midi_poll = 0;
    for (uint8_t ch = 0; ch < DAC_CHANNELS; ch++)
    {
      MIDI_addToUSBReport(0, 0xB0, 0x10 + ch, sclamp(get_adc(ch) / 32, 0, 127));
    }
    update_midi();
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
    const LedRgb* led = &engine_state.leds[i];
    ws2811_setled_rgb(i, led->r, led->g, led->b);
  }
}

uint8_t bmcv_state_update(uint32_t now_us)
{
  uint8_t dirty      = 0;
  int32_t slider_cv  = 0;
  uint32_t deltaTime = now_us - curr_hw->time;
  prev_hw            = &state[state_idx];
  state_idx          = (state_idx + 1) % STATE_RINGBUF_SIZE;
  curr_hw            = &state[state_idx];
  curr_hw->dt        = deltaTime;
  curr_hw->time      = now_us;

  // Trig State
  for (uint8_t g = 0; g < N_INPUTS; g++)
  {
    curr_hw->trigger_src[g] = adc_read_trig_state(hw_setup->input_adc_idx[g]);
  }

  for (uint8_t c = 0; c < N_CHANNELS; c++)
  {
    curr_hw->trigger_src[N_INPUTS + c] = read_channel_trig_state(&ux_setup->channels[c], &ux_state);
  }

  Clock_Poll(now_us);

  for (uint8_t g = 0; g < N_INPUTS; g++)
  {
    if (engine_config.input_mode[g] == INPUT_RESET && curr_hw->trigger_src[g])
    {
      Clock_Reset(now_us);
      for (uint8_t c = 0; c < N_CHANNELS; c++)
      {
        reset_channel_phase(&ux_setup->channels[c], &ux_state);
      }
    }
    if (engine_config.input_mode[g] == INPUT_SLIDER)
    {
      slider_cv += get_adc(hw_setup->input_adc_idx[g]) * 2;
    }
  }

  for (uint8_t g = 0; g < N_INPUTS; g++)
  {
    // TODO: Clock input configuration
    if (engine_config.input_mode[g] == INPUT_CLOCK && curr_hw->trigger_src[g])
    {
      Clock_Trigger(now_us);
    }
  }

  curr_hw->slider_state = iclamp(slider_adc_value - slider_cv, SLIDER_MIN_VALUE, SLIDER_MAX_VALUE);

  if (now_us - last_write > MS(2000))
  {
    last_write  = now_us;
    int crc_now = crc32(&engine_config, sizeof(EngineConfig));
    if (last_crc != crc_now)
    {
      preset_store(&engine_config, FRAM_CONFIG_SLOTS - 1);
      last_crc            = crc_now;
      write_indicator_for = MS(100);
    }
  }

  if (deltaTime < write_indicator_for)
  {
    write_indicator_for -= deltaTime;
  }
  else
  {
    write_indicator_for = 0;
  }

  for (uint8_t i = 0; i < N_INPUTS; i++)
  {
    curr_hw->input_state[i] = get_adc(hw_setup->input_adc_idx[i]) * 4;
  }

  for (uint8_t b = 0; b < N_BUTTONS; b++)
  {
    curr_hw->button_state[b]      = get_btn_state(b);
    curr_hw->button_released_t[b] = 0;

    if (curr_hw->button_state[b] && prev_hw->button_state[b])
    {
      curr_hw->button_pressed_t[b] += deltaTime;
    }
    else
    {
      curr_hw->button_pressed_t[b] = 0;
    }

    if (!curr_hw->button_state[b] && prev_hw->button_state[b])
    {
      curr_hw->button_released_t[b] = prev_hw->button_pressed_t[b];
    }

    dirty += curr_hw->button_state[b] != prev_hw->button_state[b];
  }

  for (uint8_t e = 0; e < N_ENCODERS; e++)
  {
    curr_hw->encoder_state[e] = get_enc_state(e);
    curr_hw->encoder_delta[e] = (int16_t) (curr_hw->encoder_state[e] - prev_hw->encoder_state[e]);
    dirty += curr_hw->encoder_delta[e] != 0;
  }

  if (error_any())
  {
    // draw error code
    led_clear_all(&ux_state);

    for (int s = 0; s < 7; s++)
    {
      uint8_t val = error_get(s) * 64;
      led_set_hsv(&ux_state, ux_setup->scenes[s].led, 0, SAT_OFF, val);
    }

    if (dirty)
    { // any interaction cleans error
      error_clear();
    }

    return 0;
  }

  return dirty;
}
