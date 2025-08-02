#include "bmcv.h"
#include "channel.h"
#include "dac_adc.h"
#include "dualmcp.h"
#include "helpers.h"
#include "midi.h"
#include "scene.h"
#include "state.h"
#include "stm32g474xx.h"
#include "ws2811.h"
#include <stdint.h>

#define STATE_RINGBUF_SIZE 2

static uint8_t state_idx;
static State state[STATE_RINGBUF_SIZE];

static State* prev_state = &state[0];
static State* curr_state = &state[1];

static uint16_t slider;

static uint8_t task = 0;

static uint8_t next_dac  = 0;
static uint8_t mcp_poll  = 0;
static uint8_t led_poll  = 0;
static uint8_t midi_poll = 0;

static uint16_t mpc_interrupt_pin;
static ADC_TypeDef* slider_adc;

static Scene scene[N_SCENES];
static Channel channel[N_ENCODERS];

// NOLINTBEGIN(readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers)
static uint8_t scene_leds[N_SCENES]    = {20, 19, 18, 17, 16, 15, 14};
static uint8_t scene_buttons[N_SCENES] = {11, 9, 13, 14, 18, 19, 20};
static uint8_t scene_colors[N_SCENES]  = {0, 35, 70, 105, 140, 175, 210};

static uint8_t channel_leds[N_ENCODERS]     = {5, 4, 7, 6, 3, 2, 0, 1};
static uint8_t channel_buttons[N_ENCODERS]  = {1, 2, 4, 0, 3, 5, 7, 6};
static uint8_t channel_encoder[N_ENCODERS]  = {3, 2, 4, 5, 1, 0, 7, 6};
static uint8_t channel_channels[N_ENCODERS] = {7, 3, 5, 1, 6, 2, 4, 0};

// static ENVELOPE env[4];
//  NOLINTEND(readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers)

void bmcv_init(uint16_t _mpc_interrupt_pin, ADC_TypeDef* _slider_adc)
{
    mpc_interrupt_pin = _mpc_interrupt_pin;
    slider_adc        = _slider_adc;

    for (uint8_t s = 0; s < N_SCENES; s++)
    {
        scene[s].id     = s;
        scene[s].led    = scene_leds[s];
        scene[s].button = scene_buttons[s];
        scene[s].color  = scene_colors[s];
    }

    for (uint8_t c = 0; c < N_ENCODERS; c++)
    {
        channel[c].button      = channel_buttons[c];
        channel[c].led         = channel_leds[c];
        channel[c].encoder     = channel_encoder[c];
        channel[c].dac_channel = channel_channels[c];
        init_channel(&channel[c]);
    }
}

void bmcv_handle_adc_conversion_complete(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == slider_adc)
    {
        slider = HAL_ADC_GetValue(hadc);
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
        next_dac = 1;
    }
}

void bmcv_handle_timer_period_elapsed(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM2)
    { // ~ 100 Hz

        task = task + 1;
        if (task == 1)
        {
            mcp_poll = 1;
            if (dacadc_update())
            {
                next_dac = 1;
            }
        }
        else if (task == 2)
        {
            led_poll = 1;
        }
        else if (task == 3)
        {
            midi_poll = 1;
        }
        else
        {
            task = 0;
        }
    }
}

void bmcv_main(uint32_t _t)
{
    if (mcp_poll == 1 && mcp_read())
    {
        mcp_poll = 0;
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

        ws2811_update();
    }

    if (next_dac)
    {
        next_dac = 0;
        dacadc_dma_next();
    }
}

void bmcv_state_update(uint32_t time)
{
    uint32_t deltaTime = time - curr_state->time;
    if (deltaTime < 4)
    { // 250 Hz
        return;
    }
    prev_state                  = &state[state_idx];
    state_idx                   = (state_idx + 1) % STATE_RINGBUF_SIZE;
    curr_state                  = &state[state_idx];
    curr_state->dt              = deltaTime;
    curr_state->time            = time;
    curr_state->slider_position = slider;
    curr_state->active_scene_id = prev_state->active_scene_id;

    for (uint8_t b = 0; b < N_BUTTONS; b++)
    {
        curr_state->button_state[b]      = mcp_button_state(b);
        curr_state->button_released_t[b] = 0;

        if (curr_state->button_state[b] && !prev_state->button_state[b])
        {
            curr_state->button_pressed_t[b] = deltaTime;
        }

        if (curr_state->button_state[b] && prev_state->button_state[b])
        {
            curr_state->button_pressed_t[b] += deltaTime;
        }

        if (!curr_state->button_state[b] && prev_state->button_state[b])
        {
            curr_state->button_released_t[b] = curr_state->button_pressed_t[b];
        }
    }

    for (uint8_t e = 0; e < N_ENCODERS; e++)
    {
        curr_state->encoder_state[e] = mcp_encoder_state(e);
        curr_state->encoder_delta[e] = (int16_t) (curr_state->encoder_state[e] - prev_state->encoder_state[e]);
    }

    for (uint8_t s = 0; s < N_SCENES; s++)
    {
        if (curr_state->button_released_t[scene[s].button] > 0)
        {
            curr_state->active_scene_id = s;
        }
    }

    for (uint8_t s = 0; s < N_SCENES; s++)
    {
        update_scene(&scene[s], curr_state);
    }

    for (uint8_t c = 0; c < N_ENCODERS; c++)
    {
        update_channel(&channel[c], curr_state);
    }
}
