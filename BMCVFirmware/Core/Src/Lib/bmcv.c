#include "bmcv.h"
#include "assign.h"
#include "channel.h"
#include "clock_sync.h"
#include "color_presets.h"
#include "dac_adc.h"
#include "helpers.h"
#include "hw_setup.h"
#include "mcp.h"
#include "midi.h"
#include "presets.h"
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
static uint32_t last_write            = 0; // timestamp of last write
static int last_crc                   = 0;
static uint32_t write_indicator_until = 0;

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

void bmcv_init(uint16_t _mpc_interrupt_pin, ADC_TypeDef* _slider_adc)
{
    hw_setup = HwSetup_Get();
    ux_setup = UxSetup_InitFromHw(hw_setup);

    mpc_interrupt_pin = _mpc_interrupt_pin;
    slider_adc        = _slider_adc;

    Clock_Init();

    engine_config.input_mode[0] = INPUT_CLOCK;
    engine_config.input_mode[1] = INPUT_RESET;
    engine_config.input_mode[2] = INPUT_DEFAULT;
    engine_config.input_mode[3] = INPUT_DEFAULT;
    engine_config.scene_l       = 0;
    engine_config.scene_r       = 6;
    engine_config.quantize_mask = 0b111111111111;

    ux_state.hw_setup                     = hw_setup;
    ux_state.ux_setup                     = ux_setup;
    ux_state.engine_config                = &engine_config;
    ux_state.engine_state                 = &engine_state;
    ux_state.engine_state->selected_param = CH_PARAM_OFS;

    for (uint8_t c = 0; c < N_ENCODERS; c++)
    {
        init_channel(&ux_setup->channels[c], &ux_state, -1);
    }

    preset_load(&engine_config, 8);
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

void bmcv_main(uint32_t now_us)
{

    uint32_t now_ms = now_us / 1000;
    if (dac_poll == 1 || dacadc_error())
    {
        dac_poll = 0;
        dacadc_dma_next();
    }

    for (uint8_t g = 0; g < N_INPUTS; g++)
    {
        if (engine_config.input_mode[g] == INPUT_RESET && adc_read_trig_state(hw_setup->input_adc_idx[g]))
        {
            Clock_Reset(now_us);
            for (uint8_t c = 0; c < N_CHANNELS; c++)
            {
                reset_channel_phase(&ux_setup->channels[c], &ux_state);
            }
        }
    }

    for (uint8_t g = 0; g < N_INPUTS; g++)
    {
        // TODO: Clock input configuration
        if (engine_config.input_mode[g] == INPUT_CLOCK && adc_read_trig_state(hw_setup->input_adc_idx[g]))
        {
            Clock_Trigger(now_us);
        }
    }

    if (mcp_poll == 1 && mcp_read())
    {
        mcp_poll = 0;
        mcu_read_buttons();
    }

    if (now_ms > curr_hw->time)
    {
        Clock_Poll(now_us);
        ux_state.engine_state->blink_fast = (now_ms % FAST_BLINK_PERIOD) < (FAST_BLINK_PERIOD / 2);
        ux_state.engine_state->blink_slow = (now_ms % SLOW_BLINK_PERIOD) < (SLOW_BLINK_PERIOD / 2);

        if (bmcv_state_update(now_ms))
        {
            ux_state.hw_state = curr_hw;
            update_ux_state(&ux_state);
        }
    }

    if (write_indicator_until >= now_ms)
    {
        ws2811_setled_hsv(ux_setup->ctrl_buttons[1].led, HUE_RED, SAT_MAX, VAL_MED);
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
        if (ux_state.engine_state->shift_state == SHIFT_STATE_MON && assign_state() != ASSIGN_INPUT)
        {
            for (uint8_t i = 0; i < N_INPUTS; i++)
            {
                int16_t adc_val = get_adc(hw_setup->input_adc_idx[i]);
                ws2811_setled_adcr(ux_setup->scenes[i].led, adc_val);

                for (uint8_t c = 0; c < N_CHANNELS; c++)
                {
                    if (ux_state.engine_config->channel_state[c].src_input == i)
                    {
                        ws2811_setled_adcr(ux_setup->channels[c].led, adc_val);
                    }
                    else
                    {
                        ws2811_setled_adcr(ux_setup->channels[c].led, 0);
                    }
                }
            }
        }
        ws2811_update();
    }
}

uint8_t bmcv_state_update(uint32_t now_ms)
{
    uint32_t deltaTime = now_ms - curr_hw->time;
    if (deltaTime < 5)
    {
        return 0;
    }
    prev_hw               = &state[state_idx];
    state_idx             = (state_idx + 1) % STATE_RINGBUF_SIZE;
    curr_hw               = &state[state_idx];
    curr_hw->dt           = deltaTime;
    curr_hw->time         = now_ms;
    curr_hw->slider_state = slider_adc_value;

    if (now_ms - last_write > 2000)
    {
        last_write  = now_ms;
        int crc_now = crc32(&engine_config, sizeof(EngineConfig));
        if (last_crc != crc_now)
        {
            preset_store(&engine_config, 8);
            last_crc              = crc_now;
            write_indicator_until = now_ms + 100;
        }
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
    }

    for (uint8_t e = 0; e < N_ENCODERS; e++)
    {
        curr_hw->encoder_state[e] = get_enc_state(e);
        curr_hw->encoder_delta[e] = (int16_t) (curr_hw->encoder_state[e] - prev_hw->encoder_state[e]);
    }
    return 1;
}
