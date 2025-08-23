#include "channel.h"
#include "dac_adc.h"
#include "helpers.h"
#include "math.h"
#include "scene.h"
#include "state.h"
#include "wavetables.h"
#include "ws2811.h"
#include <stdint.h>

int8_t param_index(uint16_t flags)
{
    if (flags & CTRL_AMP)
    {
        return PARAM_AMP;
    }

    if (flags & CTRL_FRQ)
    {
        return PARAM_FRQ;
    }

    if (flags & CTRL_SHP)
    {
        return PARAM_SHP;
    }

    if (flags & CTRL_PHS)
    {
        return PARAM_PHS;
    }

    if (flags & CTRL_OFS)
    {
        return PARAM_OFS;
    }

    return PARAM_OFS;
}

void init_channel(Channel* ch)
{
    ch->shared_phase = 0;
    for (uint8_t s = 0; s < N_SCENES; s++)
    {
        for (uint8_t p = 0; p < N_PARAMS; p++)
        {
            ch->params[p][s] = 0;
        }
        ch->params[PARAM_AMP][s] = 16000;
        ch->params[PARAM_FRQ][s] = -510;
    }
}

int16_t freq_neighbour(int16_t freq, int16_t delta)
{
    delta = (int16_t) iclamp(delta, -1, 1);
    if (delta == 0)
    {
        return -1;
    }

    // sorted small, if delta is negative, we look from largest to smallest until we find a smaller one
    // if delta is positive, we look from smallest to largest until we finde one that is larger
    for (uint8_t i = 0; i < N_FREQ_MULTIPLIERS; i++)
    {
        uint8_t idx = delta > 0 ? i : N_FREQ_MULTIPLIERS - (i + 1);
        if ((delta > 0 && quantized_multipliers[idx] > freq) || (delta < 0 && quantized_multipliers[idx] < freq))
        {
            return idx;
        }
    }
    return (int16_t) delta > 0 ? 0 : N_FREQ_MULTIPLIERS - 1;
}

void update_channel(Channel* ch, State* state)
{
    int8_t param  = param_index(state->ctrl_flags);
    int16_t delta = state->encoder_delta[ch->encoder];
    int8_t shift  = state->button_pressed_t[ch->button] > 0;

    if (state->ctrl_flags & CTRL_FRQ)
    {
        int16_t idx = freq_neighbour(ch->params[param][state->active_scene_id], delta);
        if (idx >= 0)
        {
            ch->params[param][state->active_scene_id] = quantized_multipliers[idx];
            ch->blink_hue                             = quantized_multipliers_colors[idx];
            ch->blink_until                           = state->time + 1500;
        }
    }
    else if (state->ctrl_flags & CTRL_QNT)
    {
        ch->quantize_mode = delta_modulo_step(ch->quantize_mode, state->encoder_delta[ch->encoder], QUANTIZE_MODE_COUNT);
    }
    else
    {
        ch->params[param][state->active_scene_id] += shift ? delta * 512 : delta * 64;

        /*
        if (state->button_released_t[ch->button] > 1000)
        {
            init_channel(ch);
        }
        else if (state->button_released_t[ch->button] > 50)
        {
            ch->params[param][state->active_scene_id] = 0;
        }
        */
    }
}

void compute_channel(Channel* ch, State* state, Scene* scenes)
{
    int16_t avg[N_PARAMS] = {0};
    for (uint8_t s = 0; s < N_SCENES; s++)
    {
        if (scenes[s].contribution == 0)
        {
            continue;
        }

        for (uint8_t p = 0; p < N_PARAMS; p++)
        {
            avg[p] += (int16_t) (((int32_t) ch->params[p][s] * scenes[s].contribution) / 255);
        }
    }

    float freq = (float) avg[PARAM_FRQ] / (float) N_FREQ_SCALE;
    if (freq < 0)
    {
        freq = -1 / freq;
    }

    float offset = (float) avg[PARAM_OFS];
    float amp    = (float) avg[PARAM_AMP] / 2.0f;
    float shape  = (float) avg[PARAM_SHP] / INT16_MAX;
    float phs    = (float) avg[PARAM_PHS] / INT16_MAX;
    ch->shared_phase += (state->dt / 1000.0f) * (state->base_freq * freq);
    float phase = fmodf(ch->shared_phase + phs, 1.0f);
    float raw   = wavetable_lookup(phase, shape) / (float) INT16_MAX;
    float value = offset + amp * raw;

    if (value > INT16_MAX)
        value = INT16_MAX;
    else if (value < INT16_MIN)
        value = INT16_MIN;

    switch (ch->quantize_mode)
    {
    case QUANTIZE_CONTINUOUS:
        state->channel_level[ch->dac_channel] = quantize_value((int16_t) value, state->quantize_mask);
        break;
    default:
        state->channel_level[ch->dac_channel] = (int16_t) value;
    }
}

void write_channel(Channel* ch, State* state)
{
    // ws2811_setled_hsv(ch->led, 255, 255, 255);
    if (state->ctrl_flags == CTRL_QNT)
    {
        ws2811_setled_hsv(ch->led, quantize_mode_color[ch->quantize_mode], 255, ch->quantize_mode == QUANTIZE_DISABLED ? 0 : 25);
    }
    else if (ch->blink_until > state->time)
    {
        ws2811_setled_hsv(ch->led, ch->blink_hue, 255, state->blink_fast ? 25 : 0);
    }
    else
    {
        ws2811_setled_dac(ch->led, state->channel_level[ch->dac_channel]);
    }

    dacadc_write(ch->dac_channel, state->channel_level[ch->dac_channel]);
}
