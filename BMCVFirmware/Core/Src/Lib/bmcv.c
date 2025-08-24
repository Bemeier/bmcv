#include "bmcv.h"
#include "clock_sync.h"
#include "dac_adc.h"
#include "helpers.h"
#include "mcp.h"
#include "midi.h"
#include "state.h"
#include "stm32g474xx.h"
#include "ws2811.h"
#include <stdint.h>

#define STATE_RINGBUF_SIZE 2

static uint8_t state_idx;

static uint16_t slider;

static uint8_t task = 0;

static uint8_t next_dac  = 0;
static uint8_t mcp_poll  = 0;
static uint8_t led_poll  = 0;
static uint8_t midi_poll = 0;

static uint16_t mpc_interrupt_pin;
static ADC_TypeDef* slider_adc;

static uint16_t last_active_ctrl;

// NOLINTBEGIN(readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers)

// Control
static int8_t scene_button_idx[N_SCENES]      = {11, 9, 13, 14, 18, 19, 20};
static int8_t channel_button_idx[N_ENCODERS]  = {1, 2, 4, 0, 3, 5, 7, 6};
static int8_t ctrl_button_idx[N_CTRL_BUTTONS] = {10, 8, 12, 15, 16, 17, 23, 22, 21};
static int8_t channel_encoder_idx[N_ENCODERS] = {3, 2, 4, 5, 1, 0, 7, 6};
static int8_t channel_dac_idx[N_ENCODERS]     = {7, 3, 5, 1, 6, 2, 4, 0};

static uint16_t ctrl_button_flags[N_CTRL_BUTTONS] = {CTRL_FRQ | CTRL_STL,
                                                     CTRL_SHP | CTRL_LAT,
                                                     CTRL_PHS | CTRL_SYS,
                                                     CTRL_INP | CTRL_MON,
                                                     CTRL_AMP | CTRL_SEQ,
                                                     CTRL_OFS | CTRL_STR,
                                                     CTRL_QNT,
                                                     CTRL_CPY,
                                                     CTRL_CLR};

static uint16_t persistent_flags = CTRL_FRQ | CTRL_SHP | CTRL_PHS | CTRL_AMP | CTRL_OFS;

static uint8_t scene_button_alt_ctrl_flags[N_SCENES] = {ALT_CTRL_IN1, ALT_CTRL_IN2, ALT_CTRL_IN3, ALT_CTRL_IN4,
                                                        ALT_CTRL_CLK, CTRL_DEFAULT, ALT_CTRL_PLY};

static uint8_t quantizer_button_idx[12] = {11, 10, 9, 8, 13, 14, 15, 18, 16, 19, 17, 20};

static uint8_t input_map[N_INPUTS] = {2, 3, 0, 1};

// View
static int8_t scene_button_led_idx[N_SCENES]      = {20, 19, 18, 17, 16, 15, 14};
static int8_t ctrl_button_led_idx[N_CTRL_BUTTONS] = {8, 9, 10, 11, 12, 13, -1, -1, -1};
static int8_t channel_led_idx[N_ENCODERS]         = {5, 4, 7, 6, 3, 2, 0, 1};

static uint8_t scene_button_color[N_SCENES]      = {HUE_RED, HUE_YELLOW, HUE_GREEN, HUE_CYAN, HUE_BLUE, HUE_MAGENTA, HUE_RED};
static uint8_t ctrl_button_color[N_CTRL_BUTTONS] = {HUE_RED, HUE_YELLOW, HUE_GREEN, HUE_CYAN, HUE_BLUE, HUE_MAGENTA, 0, 0, 0};

static uint8_t quantizer_button_led_idx[12] = {20, 8, 19, 9, 18, 17, 11, 16, 12, 15, 13, 14};

// menu_state |= MENU_STATE_FRQ;  // set bit
// menu_state &= ~MENU_STATE_FRQ; // unset
// menu_state ^= MENU_STATE_FRQ; // toggle
// menu_state & MENU_STATE_FRQ // is set
// menu_state = MENU_STATE_SHP | MENU_STATE_PHS; // state is only those two

// static ENVELOPE env[4];
//  NOLINTEND(readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers)

void bmcv_init(uint16_t _mpc_interrupt_pin, ADC_TypeDef* _slider_adc)
{
    mpc_interrupt_pin = _mpc_interrupt_pin;
    slider_adc        = _slider_adc;

    Clock_Init();

    for (uint8_t s = 0; s < N_SCENES; s++)
    {
        scene[s].id             = s;
        scene[s].led            = scene_button_led_idx[s];
        scene[s].button         = scene_button_idx[s];
        scene[s].alt_ctrl_flags = scene_button_alt_ctrl_flags[s];
        scene[s].color          = scene_button_color[s];
    }

    for (uint8_t b = 0; b < N_CTRL_BUTTONS; b++)
    {
        ctrl_buttons[b].button     = ctrl_button_idx[b];
        ctrl_buttons[b].led        = ctrl_button_led_idx[b];
        ctrl_buttons[b].color      = ctrl_button_color[b];
        ctrl_buttons[b].ctrl_flags = ctrl_button_flags[b];
        init_ctrl_button(&ctrl_buttons[b]);
    }

    for (uint8_t c = 0; c < N_ENCODERS; c++)
    {
        channel[c].button      = channel_button_idx[c];
        channel[c].led         = channel_led_idx[c];
        channel[c].encoder     = channel_encoder_idx[c];
        channel[c].dac_channel = channel_dac_idx[c];
        init_channel(&channel[c]);
    }

    prev_state->ctrl_flags           = CTRL_OFS;
    prev_state->scene_latch_position = 4032;
    prev_state->scene_l              = 0;
    prev_state->scene_r              = 6;
    prev_state->scene_latch          = -1;
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

void bmcv_poll_tasks()
{
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

void bmcv_main(uint32_t now_us, uint32_t now_ms)
{
    if (next_dac)
    {
        next_dac = 0;
        dacadc_dma_next();
    }

    for (uint8_t g = 0; g < N_INPUTS; g++)
    {
        // TODO: Clock input configuration
        if (g == 0 && adc_read_trig_state(input_map[g]))
        {
            Clock_Trigger(now_us);
        }
    }

    if (mcp_poll == 1 && mcp_read())
    {
        mcp_poll = 0;
        mcu_read_buttons();
    }

    if (now_ms > curr_state->time + 1)
    {
        Clock_Poll(now_us);
        bmcv_state_update(now_ms);
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
}

void bmcv_state_update(uint32_t now)
{
    uint32_t deltaTime = now - curr_state->time;
    if (deltaTime < 4)
    { // 250 Hz
        return;
    }
    prev_state                            = &state[state_idx];
    state_idx                             = (state_idx + 1) % STATE_RINGBUF_SIZE;
    curr_state                            = &state[state_idx];
    curr_state->dt                        = deltaTime;
    curr_state->time                      = now;
    curr_state->slider_position           = slider;
    curr_state->active_scene_id           = prev_state->active_scene_id;
    curr_state->ctrl_last_channel_touched = prev_state->ctrl_last_channel_touched;
    curr_state->scene_latch_position      = prev_state->scene_latch_position;
    curr_state->scene_l                   = prev_state->scene_l;
    curr_state->scene_r                   = prev_state->scene_r;
    curr_state->scene_latch               = prev_state->scene_latch;
    curr_state->ctrl_flags                = prev_state->ctrl_flags;
    curr_state->blink_fast                = (now % FAST_BLINK_PERIOD) < (FAST_BLINK_PERIOD / 2);
    curr_state->blink_slow                = (now % SLOW_BLINK_PERIOD) < (SLOW_BLINK_PERIOD / 2);
    /*
    curr_state->beat_phase                = g_clk.phase;
    curr_state->beat_time                 = g_clk.beat_dt_us;
    curr_state->beat_freq                 = g_clk.bpm / 60.0f;
    */

    for (uint8_t g = 0; g < N_INPUTS; g++)
    {
    }

    if (curr_state->ctrl_flags > CTRL_DEFAULT)
    {
        last_active_ctrl = curr_state->ctrl_flags;
    }
    else
    {
        last_active_ctrl = CTRL_OFS;
    }

    for (uint8_t i = 0; i < N_INPUTS; i++)
    {
        curr_state->input_state[i] = get_adc(input_map[i]);
    }

    for (uint8_t b = 0; b < N_BUTTONS; b++)
    {
        curr_state->button_state[b]      = get_btn_state(b);
        curr_state->button_released_t[b] = 0;

        if (curr_state->button_state[b] && prev_state->button_state[b])
        {
            curr_state->button_pressed_t[b] += deltaTime;
        }
        else
        {
            curr_state->button_pressed_t[b] = 0;
        }

        if (!curr_state->button_state[b] && prev_state->button_state[b])
        {
            curr_state->button_released_t[b] = prev_state->button_pressed_t[b];
        }
    }

    for (uint8_t e = 0; e < N_ENCODERS; e++)
    {
        curr_state->encoder_state[e] = get_enc_state(e);
        curr_state->encoder_delta[e] = (int16_t) (curr_state->encoder_state[e] - prev_state->encoder_state[e]);
    }

    uint16_t oldest_ctrl_button = 0;

    for (uint8_t b = 0; b < N_CTRL_BUTTONS; b++)
    {
        if (curr_state->button_pressed_t[ctrl_buttons[b].button] > oldest_ctrl_button)
        {
            curr_state->ctrl_flags = ctrl_buttons[b].ctrl_flags;
        }
    }

    if (curr_state->ctrl_flags & CTRL_LAT)
    {
        if (curr_state->slider_position <= SLIDER_MIN_VALUE)
        {
            curr_state->ctrl_flags |= CTRL_STR;
        }

        if (curr_state->slider_position >= SLIDER_MAX_VALUE)
        {
            curr_state->ctrl_flags |= CTRL_STL;
        }
    }

    if (curr_state->ctrl_flags != prev_state->ctrl_flags)
    {
        curr_state->ctrl_active_t = 0;
    }

    curr_state->ctrl_active_t += deltaTime;

    curr_state->quantize_mask = quantize_mask;

    if (curr_state->ctrl_flags & CTRL_QNT)
    {
        for (uint16_t st = 0; st < 12; st++)
        {
            if (curr_state->button_released_t[quantizer_button_idx[st]] > 0)
            {
                quantize_mask ^= (1u << st);
            }

            // uint8_t sat = 255 * curr_state->button_pressed_t[quantizer_button_idx[st]] > 0;
            uint8_t sat = 0;
            uint8_t val = (quantize_mask & (1u << st)) ? 25 : 0;
            ws2811_setled_hsv(quantizer_button_led_idx[st], 0, sat, val);
        }
    }
    else if (curr_state->ctrl_flags & CTRL_MON)
    {
        for (uint8_t i = 0; i < N_INPUTS; i++)
        {
            ws2811_setled_adcr(scene[i].led, curr_state->input_state[i]);
        }
    }
    else
    {

        uint16_t oldest_scene_button_hold     = 0;
        uint16_t oldest_scene_button_released = 0;
        curr_state->ctrl_scene_hold           = -1;
        curr_state->ctrl_scene_released       = -1;
        for (uint8_t s = 0; s < N_SCENES; s++)
        {
            if (curr_state->button_pressed_t[scene[s].button] > oldest_scene_button_hold)
            {
                curr_state->ctrl_scene_hold = s;
            }
            if (curr_state->button_released_t[scene[s].button] > oldest_scene_button_released)
            {
                curr_state->ctrl_scene_released = s;
            }
        }

        for (uint8_t c = 0; c < N_CHANNELS; c++)
        {
            if ((curr_state->button_pressed_t[channel[c].button] > 0 || curr_state->button_released_t[channel[c].button] > 0 ||
                 curr_state->encoder_delta[channel[c].encoder] != 0) &&
                curr_state->ctrl_last_channel_touched != c)
            {
                curr_state->ctrl_last_channel_touched = c;
                if (curr_state->ctrl_flags & CTRL_INP)
                {
                    curr_state->button_pressed_t[channel[c].button]  = 0;
                    curr_state->button_released_t[channel[c].button] = 0;
                    curr_state->encoder_state[channel[c].encoder]    = prev_state->encoder_state[channel[c].encoder];
                    curr_state->encoder_delta[channel[c].encoder]    = 0;
                }
            }
        }

        for (uint8_t b = 0; b < N_CTRL_BUTTONS; b++)
        {
            update_ctrl_button(&ctrl_buttons[b], curr_state);
        }

        for (uint8_t s = 0; s < N_SCENES; s++)
        {
            update_scene_button(&scene[s], curr_state);
            scene[s].contribution = 0;
        }
    }

    uint8_t scene_a         = curr_state->scene_l;
    uint8_t scene_b         = curr_state->scene_r;
    uint16_t scene_a_anchor = SLIDER_MAX_VALUE;
    uint16_t scene_b_anchor = SLIDER_MIN_VALUE;

    if (curr_state->scene_latch > 0 && curr_state->slider_position <= curr_state->scene_latch_position)
    {
        scene_a        = curr_state->scene_latch;
        scene_a_anchor = curr_state->scene_latch_position - LATCH_DEADZONE / 2;
    }

    if (curr_state->scene_latch > 0 && curr_state->slider_position > curr_state->scene_latch_position)
    {
        scene_b        = curr_state->scene_latch;
        scene_b_anchor = curr_state->scene_latch_position + LATCH_DEADZONE / 2;
    }

    if (scene_a == scene_b)
    {
        scene[scene_a].contribution = 255;
    }
    else
    {
        scene[scene_a].contribution = interpolate_clamped(scene_b_anchor, scene_a_anchor, curr_state->slider_position);
        scene[scene_b].contribution = 255 - scene[scene_a].contribution;
    }
    curr_state->active_scene_id = scene[scene_a].contribution > scene[scene_b].contribution ? scene_a : scene_b;

    for (uint8_t s = 0; s < N_SCENES; s++)
    {
        // Only LED state for now?
        update_scene(&scene[s], curr_state);
    }

    for (uint8_t c = 0; c < N_CHANNELS; c++)
    {
        // Channel state/settings
        update_channel(&channel[c], curr_state);
    }

    for (uint8_t c = 0; c < N_CHANNELS; c++)
    {
        // Channel state/settings
        compute_channel(&channel[c], curr_state, &scene[0]);
    }

    for (uint8_t c = 0; c < N_CHANNELS; c++)
    {
        // Write channel state to DACs & LEDs
        write_channel(&channel[c], curr_state);
    }

    curr_state->ctrl_flags &= persistent_flags;
    if (curr_state->ctrl_flags == CTRL_DEFAULT)
    {
        curr_state->ctrl_flags = last_active_ctrl;
    }
}
