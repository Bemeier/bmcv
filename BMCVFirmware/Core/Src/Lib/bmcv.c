#include "bmcv.h"
#include "state.h"
#include "scene.h"
#include "channel.h"
#include "dualmcp.h"
#include "dac_adc.h"

#define STATE_RINGBUF_SIZE 2

uint8_t state_idx;
static State state[STATE_RINGBUF_SIZE];

State * prev_state = &state[0];
State * curr_state = &state[1];

static Scene scene[N_SCENES];
static Channel channel[N_ENCODERS];

static uint8_t scene_leds[N_SCENES] =    { 20, 19, 18, 17, 16, 15, 14 };
static uint8_t scene_buttons[N_SCENES] = { 11,  9, 13, 14, 18, 19, 20 };
static uint8_t scene_colors[N_SCENES] =  {  0,  35, 70, 105, 140, 175, 210 };

static uint8_t channel_leds[N_ENCODERS] =     { 5, 4, 7, 6, 3, 2, 0, 1 };
static uint8_t channel_buttons[N_ENCODERS] =  { 1, 2, 4, 0, 3, 5, 7, 6 };
static uint8_t channel_encoder[N_ENCODERS] =  { 3, 2, 4, 5, 1, 0, 7, 6 };
static uint8_t channel_channels[N_ENCODERS] = { 7, 3, 5, 1, 6, 2, 4, 0 };

void init_bmcv() {
    for (uint8_t s = 0; s < N_SCENES; s++) {
        scene[s].id = s;
        scene[s].led = scene_leds[s];
        scene[s].button = scene_buttons[s];
        scene[s].color = scene_colors[s];
    }

    for (uint8_t c = 0; c < N_ENCODERS; c++) {
        channel[c].button = channel_buttons[c];
        channel[c].led = channel_leds[c];
        channel[c].encoder = channel_encoder[c];
        channel[c].dac_channel = channel_channels[c];
        init_channel(&channel[c]);
    }

}

void update_bmcv(uint32_t t, uint16_t slider) {
    uint32_t dt = t - curr_state->time;
    if (dt < 4) { // 250 Hz
        return;
    }
    prev_state = &state[state_idx];
    state_idx = (state_idx + 1) % STATE_RINGBUF_SIZE;
    curr_state = &state[state_idx];
    curr_state->dt = dt;
    curr_state->time = t;
    curr_state->slider_position = slider;
    curr_state->active_scene_id = prev_state->active_scene_id;

    for (uint8_t b = 0; b < N_BUTTONS; b++) {
        curr_state->button_state[b] = mcp_instance()->button_state[b];
        curr_state->button_released_t[b] = 0;

        if (curr_state->button_state[b] && !prev_state->button_state[b]) {
            curr_state->button_pressed_t[b] = dt;
        }

        if (curr_state->button_state[b] && prev_state->button_state[b]) {
            curr_state->button_pressed_t[b] += dt;
        }

        if (!curr_state->button_state[b] && prev_state->button_state[b]) {
            curr_state->button_released_t[b] = curr_state->button_pressed_t[b];
        }
    }

    for (uint8_t e = 0; e < N_ENCODERS; e++) {
        curr_state->encoder_state[e] = mcp_instance()->enc_position_state[e];
        curr_state->encoder_delta[e] = curr_state->encoder_state[e] - prev_state->encoder_state[e];
    }


    for (uint8_t s = 0; s < N_SCENES; s++) {
        if (curr_state->button_released_t[scene[s].button] > 0) {
            curr_state->active_scene_id = s;
        }
    }

    for (uint8_t s = 0; s < N_SCENES; s++) {
        update_scene(&scene[s], curr_state);
    }

    for (uint8_t c = 0; c < N_ENCODERS; c++) {
        update_channel(&channel[c], curr_state);
    }

}