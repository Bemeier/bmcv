#include "bmcv.h"
#include "assign.h"
#include "channel.h"
#include "clock_sync.h"
#include "dac_adc.h"
#include "fram.h"
#include "helpers.h"
#include "mcp.h"
#include "midi.h"
#include "state.h"
#include "stm32g474xx.h"
#include "uxstate.h"
#include "ws2811.h"
#include <stdint.h>

#define STATE_RINGBUF_SIZE 2

#define N_SEMITONES 12

static uint8_t state_idx;

static uint8_t task = 0;

static uint8_t dac_poll  = 1;
static uint8_t mcp_poll  = 0;
static uint8_t led_poll  = 0;
static uint8_t midi_poll = 0;

static int8_t monitor = 0;

static uint16_t mpc_interrupt_pin;
static ADC_TypeDef* slider_adc;

// NOLINTBEGIN(readability-magic-numbers, cppcoreguidelines-avoid-magic-numbers)

// ADC/DAC
static uint8_t input_adc_idx[N_INPUTS]    = {2, 3, 0, 1};
static int8_t channel_dac_idx[N_ENCODERS] = {7, 3, 5, 1, 6, 2, 4, 0};

// Buttons & Encoders
static int8_t channel_encoder_idx[N_ENCODERS]    = {3, 2, 4, 5, 1, 0, 7, 6};
static int8_t channel_button_idx[N_ENCODERS]     = {1, 2, 4, 0, 3, 5, 7, 6};
static uint8_t quantizer_button_idx[N_SEMITONES] = {11, 10, 9, 8, 13, 14, 15, 18, 16, 19, 17, 20};
static int8_t ctrl_button_idx[N_CTRL_BUTTONS]    = {10, 8, 12, 15, 16, 17, 23, 22, 21};
static int8_t scene_button_idx[N_SCENES]         = {11, 9, 13, 14, 18, 19, 20};

// LEDs
static int8_t channel_led_idx[N_ENCODERS]            = {5, 4, 7, 6, 3, 2, 0, 1};
static uint8_t quantizer_button_led_idx[N_SEMITONES] = {20, 8, 19, 9, 18, 17, 11, 16, 12, 15, 13, 14};
static int8_t ctrl_button_led_idx[N_CTRL_BUTTONS]    = {8, 9, 10, 11, 12, 13, -1, -1, -1};
static int8_t scene_button_led_idx[N_SCENES]         = {20, 19, 18, 17, 16, 15, 14};

static uint8_t ctrl_button_color[N_CTRL_BUTTONS] = {HUE_RED, HUE_YELLOW, HUE_GREEN, HUE_CYAN, HUE_BLUE, HUE_MAGENTA, 0, 0, 0};

static uint32_t last_write = 0; // timestamp of last write
static int last_crc        = 0;

static uint32_t write_indicator_until = 0;

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
        scene[s].id     = s;
        scene[s].led    = scene_button_led_idx[s];
        scene[s].button = scene_button_idx[s];
    }

    for (uint8_t b = 0; b < N_CTRL_BUTTONS; b++)
    {
        ctrl_buttons[b].id     = b;
        ctrl_buttons[b].button = ctrl_button_idx[b];
        ctrl_buttons[b].led    = ctrl_button_led_idx[b];
        ctrl_buttons[b].color  = ctrl_button_color[b];
        init_ctrl_button(&ctrl_buttons[b]);
    }

    for (uint8_t c = 0; c < N_ENCODERS; c++)
    {
        channel[c].id          = c;
        channel[c].button      = channel_button_idx[c];
        channel[c].led         = channel_led_idx[c];
        channel[c].encoder     = channel_encoder_idx[c];
        channel[c].dac_channel = channel_dac_idx[c];
        init_channel(&channel[c], &system_state.channel_state[c], -1);
    }

    // move out of state to settings?
    system_state.input_mode[0] = INPUT_CLOCK;
    system_state.input_mode[1] = INPUT_RESET;
    system_state.input_mode[2] = INPUT_SLIDER;
    system_state.input_mode[3] = INPUT_DEFAULT;
    system_state.scene_l       = 0;
    system_state.scene_r       = 6;

    base_state.selected_param = CH_PARAM_OFS;
    base_state.quantize_mask  = 0b111111111111;

    bmcv_load_setup(8);
    last_crc = crc32(&system_state, sizeof(ConfigState));
}

void bmcv_handle_adc_conversion_complete(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == slider_adc)
    {
        base_state.slider = HAL_ADC_GetValue(hadc);
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
    /*
    else if (task == 3)
    {
    }
    */
    else
    {
        task = 0;
    }
}

void bmcv_assign_input_to_channel(int8_t i, int8_t c)
{
    if (system_state.channel_state[c].src_input == i)
    {
        system_state.channel_state[c].src_input = -1;
    }
    else
    {
        system_state.channel_state[c].src_input = i;
    }
}

void bmcv_clear_channel(int8_t c, int8_t all_scenes)
{
    init_channel(&channel[c], &system_state.channel_state[c], all_scenes ? -1 : base_state.active_scene);
}

void bmcv_clear_scene(int8_t s)
{
    for (int8_t c = 0; c < N_CHANNELS; c++)
    {
        init_channel(&channel[c], &system_state.channel_state[c], s);
    }
}

void copy_scene_channel(int8_t c_src, int8_t s_src, int8_t c_dst, int8_t s_dst)
{
    if (c_dst >= N_ENCODERS || c_src >= N_ENCODERS || s_src >= N_SCENES || s_dst >= N_SCENES)
        return;
    memcpy(system_state.channel_state[c_dst].params[s_dst], system_state.channel_state[c_src].params[s_src],
           sizeof system_state.channel_state[c_dst].params[s_dst]);
}

void bmcv_assign_channel_to_channel(int8_t c_src, int8_t c_dst)
{
    copy_scene_channel(c_src, base_state.active_scene, c_dst, base_state.active_scene);
}

void bmcv_assign_channel_to_scene(int8_t c_src, int8_t s_dst) { copy_scene_channel(c_src, base_state.active_scene, c_src, s_dst); }

void bmcv_assign_scene_to_scene(int8_t s_src, int8_t s_dst)
{
    for (uint8_t c = 0; c < N_CHANNELS; c++)
    {
        copy_scene_channel(c, s_src, c, s_dst);
    }
}

int8_t bmcv_store_setup(int8_t dst)
{
    if (dst >= FRAM_CONFIG_SLOTS)
        return false;

    ConfigStateRecord rec = {.hdr =
                                 {
                                     .magic   = FRAM_MAGIC,
                                     .version = CONFIG_STATE_VERSION,
                                     .length  = sizeof(ConfigState),
                                     .crc     = crc32(&system_state, sizeof(ConfigState)),
                                 },
                             .data = system_state};
    uint16_t addr         = FRAM_CONFIG_BASE_ADDR + dst * FRAM_CONFIG_SLOT_SIZE;
    fram_Write(addr, (uint8_t*) &rec, sizeof(rec));
    return true;
}

int8_t bmcv_load_setup(int8_t src)
{
    if (src >= FRAM_CONFIG_SLOTS)
        return false;

    ConfigStateRecord rec;
    uint16_t addr = FRAM_CONFIG_BASE_ADDR + src * FRAM_CONFIG_SLOT_SIZE;

    fram_Read(addr, (uint8_t*) &rec, sizeof(rec));

    /* Validate header */
    if (rec.hdr.magic != FRAM_MAGIC)
        return false;

    if (rec.hdr.version != CONFIG_STATE_VERSION)
        return false; // Or trigger migration

    if (rec.hdr.length != sizeof(ConfigState))
        return false;

    if (rec.hdr.crc != crc32(&rec.data, sizeof(ConfigState)))
        return false;

    system_state = rec.data;
    return true;
}

void bmcv_main(uint32_t now_us, uint32_t _now_ms)
{

    uint32_t now_ms = now_us / 1000;
    if (dac_poll == 1 || dacadc_error())
    {
        dac_poll = 0;
        dacadc_dma_next();
    }

    for (uint8_t g = 0; g < N_INPUTS; g++)
    {
        // TODO: Clock input configuration
        if (system_state.input_mode[g] == INPUT_CLOCK && adc_read_trig_state(input_adc_idx[g]))
        {
            Clock_Trigger(now_us);
        }
        else if (system_state.input_mode[g] == INPUT_RESET && adc_read_trig_state(input_adc_idx[g]))
        {
            Clock_Reset();
        }
    }

    if (mcp_poll == 1 && mcp_read())
    {
        mcp_poll = 0;
        mcu_read_buttons();
    }

    if (now_ms > curr_state->time)
    {
        Clock_Poll(now_us);
        base_state.blink_fast = (now_ms % FAST_BLINK_PERIOD) < (FAST_BLINK_PERIOD / 2);
        base_state.blink_slow = (now_ms % SLOW_BLINK_PERIOD) < (SLOW_BLINK_PERIOD / 2);

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
        if (monitor > 0)
        {
            for (uint8_t i = 0; i < N_INPUTS; i++)
            {
                if (assign_state() == ASSIGN_INPUT && assign_src() == i)
                {
                    ws2811_setled_hsv(scene[i].led, 0, 0, base_state.blink_fast * 12);
                }
                else
                {
                    ws2811_setled_adcr(scene[i].led, get_adc(input_adc_idx[i]));
                }
            }
        }
        ws2811_update();
    }
}

void bmcv_state_update(uint32_t now)
{
    uint32_t deltaTime = now - curr_state->time;
    if (deltaTime < 8)
    { // 250 Hz
        return;
    }
    prev_state       = &state[state_idx];
    state_idx        = (state_idx + 1) % STATE_RINGBUF_SIZE;
    curr_state       = &state[state_idx];
    curr_state->dt   = deltaTime;
    curr_state->time = now;

    if (now - last_write > 2000)
    {
        last_write  = now;
        int crc_now = crc32(&system_state, sizeof(ConfigState));
        if (last_crc != crc_now)
        {
            bmcv_store_setup(8);
            last_crc              = crc_now;
            write_indicator_until = now + 100;
        }
    }

    for (uint8_t i = 0; i < N_INPUTS; i++)
    {
        curr_state->input_state[i] = get_adc(input_adc_idx[i]);
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
}

void bmcv_update_ux_state(BaseState* state)
{
    state->shift_state = SHIFT_STATE_NONE;
    // TODO: ensure buttons are initialized and ordered by shift state
    for (uint8_t s = 0; s < SHIFT_STATE_COUNT - 1; s++)
    {
        // TODO: move to ctrl_buttons[s].shiftActive() ?
        // TODO: magic number
        if (state->system->button_pressed_t[ctrl_buttons[s].button] > 200)
        {
            state->shift_state = (ShiftStates) s;
            break;
        }
    }

    if (state->shift_state == SHIFT_STATE_NONE)
    {
        for (uint8_t p = 0; p < CH_PARAM_COUNT; p++)
        {
            // TODO: move to ctrl_buttons[s].switchedParam()
            // TOOD: magic numbers
            if (state->system->button_released_t[ctrl_buttons[p].button] > 0 &&
                state->system->button_released_t[ctrl_buttons[p].button] < 400)
            {
                state->selected_param = (ChannelParameters) p;
            }
        }
    }
}

void bmcv_update_active_scene(BaseState* state)
{
    // TODO: could allow mixing momentary scenes
    // TODO: of if max 1, could check oldest/newest momentary?
    int8_t momentary_scene = -1;
    state->active_scene    = -1;
    if (state->shift_state == SHIFT_STATE_NONE)
    {
        for (uint8_t s = 0; s < N_SCENES; s++)
        {
            if (momentary_scene < 0 && state->system->button_pressed_t[scene_button_idx[s]] > 0)
            {
                scene[s].contribution = 255;
                momentary_scene       = s;
                state->active_scene   = s;
            }
            else
            {
                scene[s].contribution = 0;
            }
        }
    }

    if (momentary_scene >= 0)
    {
        return;
    }

    uint8_t scene_a         = system_state.scene_l;
    uint8_t scene_b         = system_state.scene_r;
    uint16_t scene_a_anchor = SLIDER_MAX_VALUE;
    uint16_t scene_b_anchor = SLIDER_MIN_VALUE;

    if (scene_a == scene_b)
    {
        scene[scene_a].contribution = 255;
    }
    else
    {
        scene[scene_a].contribution = interpolate_clamped(scene_b_anchor, scene_a_anchor, state->slider);
        scene[scene_b].contribution = 255 - scene[scene_a].contribution;
    }
    state->active_scene = scene[scene_a].contribution > scene[scene_b].contribution ? scene_a : scene_b;
}

void bmcv_update_quantizer(BaseState* state)
{
    for (uint16_t st = 0; st < N_SEMITONES; st++)
    {
        if (curr_state->button_released_t[quantizer_button_idx[st]] > 0)
        {
            state->quantize_mask ^= (1u << st);
        }

        // TODO: MOVE THIS TO VIEW CONTROLLER LOGIC
        // uint8_t sat = 255 * curr_state->button_pressed_t[quantizer_button_idx[st]] > 0;
        uint8_t sat = 0;
        uint8_t val = (state->quantize_mask & (1u << st)) ? 25 : 0;
        ws2811_setled_hsv(quantizer_button_led_idx[st], 0, sat, val);
    }
}

// TODO: clear state / clear param, etc

// TODO: copy state / assign param

// TODO: quantizer mode

// TODO: Edit params (default no shift)

/*
    // >>>>>

    if (curr_state->ctrl_flags > CTRL_DEFAULT)
    {
        last_active_ctrl = curr_state->ctrl_flags;
    }
    else
    {
        last_active_ctrl = CTRL_OFS;
    }

    uint16_t oldest_ctrl_button = 0;

    for (uint8_t b = 0; b < N_CTRL_BUTTONS; b++)
    {
        if (curr_state->button_pressed_t[ctrl_buttons[b].button] > oldest_ctrl_button)
        {
            curr_state->ctrl_flags = ctrl_buttons[b].ctrl_flags;
            S
    }

    if (curr_state->ctrl_flags != prev_state->ctrl_flags)
    {
        curr_state->ctrl_active_t = 0;
    }

    curr_state->ctrl_active_t += deltaTime;

    curr_state->quantize_mask = quantize_mask;

    int8_t momentary_scene = -1;

    monitor = 0;

    int8_t override_scene_buttons = 0;

    if (!(curr_state->ctrl_flags & (CTRL_MON | CTRL_CPY | CTRL_CLR)))
    {
        assign_reset();
    }

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
    else if (curr_state->ctrl_flags & CTRL_CLR)
    {
        for (uint8_t c = 0; c < N_CHANNELS; c++)
        {
            if (curr_state->button_released_t[channel[c].button] > 1000)
            {
                bmcv_clear_channel(c, 1);
            }
            else if (curr_state->button_released_t[channel[c].button] > 50)
            {
                bmcv_clear_channel(c, 0);
            }
        }

        for (uint8_t s = 0; s < N_SCENES; s++)
        {
            if (curr_state->button_released_t[scene[s].button] > 50)
            {
                bmcv_clear_scene(s);
            }
        }
    }
    else if (curr_state->ctrl_flags & CTRL_CPY)
    {
        for (uint8_t c = 0; c < N_CHANNELS; c++)
        {
            if (curr_state->button_released_t[channel[c].button] > 50)
            {
                assign_event(ASSIGN_CHANNEL, c);
            }
        }

        for (uint8_t s = 0; s < N_SCENES; s++)
        {
            if (curr_state->button_released_t[scene[s].button] > 50)
            {
                assign_event(ASSIGN_SCENE, s);
            }
        }
    }
    else if (curr_state->ctrl_flags & CTRL_MON)
    {
        monitor = 1;

        for (uint8_t i = 0; i < N_INPUTS; i++)
        {
            if (curr_state->button_released_t[scene[i].button] > 50)
            {
                assign_reset();
                assign_event(ASSIGN_INPUT, i);
            }
        }

        for (uint8_t b = 0; b < N_CTRL_BUTTONS; b++)
        {
            update_ctrl_button(&ctrl_buttons[b], curr_state);
        }
    }
    else if (curr_state->ctrl_flags & CTRL_SAV)
    {

        override_scene_buttons = 1;
        for (uint8_t s = 0; s < N_SCENES; s++)
        {

            uint8_t mode_col  = HUE_GREEN;
            uint8_t state_sat = 0;
            if (curr_state->button_pressed_t[scene[s].button] > 50)
            {
                state_sat = 255;
            }

            if (curr_state->button_pressed_t[scene[s].button] > 1000)
            {
                mode_col = HUE_RED;
            }

            ws2811_setled_hsv(scene[s].led, mode_col, state_sat, 10);

            if (curr_state->button_released_t[scene[s].button] > 1000)
            {
                bmcv_store_setup(s);
            }
            else if (curr_state->button_released_t[scene[s].button] > 50)
            {
                bmcv_load_setup(s);
            }
        }

        for (uint8_t s = 0; s < N_SCENES; s++)
        {
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

        for (uint8_t b = 0; b < N_CTRL_BUTTONS; b++)
        {
            update_ctrl_button(&ctrl_buttons[b], curr_state);
        }

        for (uint8_t s = 0; s < N_SCENES; s++)
        {
            if (update_scene_button(&scene[s], curr_state, &system_state) > 0)
            {
                momentary_scene = s;
            }
            scene[s].contribution = 0;
        }
    }

    if (momentary_scene == -1)
    {
        uint8_t scene_a         = system_state.scene_l;
        uint8_t scene_b         = system_state.scene_r;
        uint16_t scene_a_anchor = SLIDER_MAX_VALUE;
        uint16_t scene_b_anchor = SLIDER_MIN_VALUE;

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
    }
    else
    {
        curr_state->active_scene_id                     = momentary_scene;
        scene[curr_state->active_scene_id].contribution = 255;
    }

    if (override_scene_buttons == 0)
    {
        for (uint8_t s = 0; s < N_SCENES; s++)
        {
            // Only LED state for now?
            update_scene(&scene[s], curr_state, &system_state);
        }
    }

    for (uint8_t c = 0; c < N_CHANNELS; c++)
    {
        // Channel state/settings
        update_channel(&channel[c], curr_state, &system_state.channel_state[c]);
    }

    for (uint8_t c = 0; c < N_CHANNELS; c++)
    {
        // Channel state/settings
        compute_channel(&channel[c], curr_state, &scene[0], &system_state.channel_state[c]);
    }

    for (uint8_t c = 0; c < N_CHANNELS; c++)
    {
        // Write channel state to DACs & LEDs
        write_channel(&channel[c], curr_state, &system_state.channel_state[c]);
    }

    if (write_indicator_until >= now)
    {
        ws2811_setled_hsv(ctrl_button_led_idx[1], HUE_RED, 255, 25);
    }

    curr_state->ctrl_flags &= persistent_flags;
    if (curr_state->ctrl_flags == CTRL_DEFAULT)
    {
        curr_state->ctrl_flags = last_active_ctrl;
    }
}
*/