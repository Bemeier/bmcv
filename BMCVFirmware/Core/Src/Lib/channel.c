#include "channel.h"
#include "assign.h"
#include "clock_sync.h"
#include "dac_adc.h"
#include "helpers.h"
#include "math.h"
#include "scene.h"
#include "state.h"
#include "wavetables.h"
#include "ws2811.h"
#include <stdint.h>

float fmod_pos(float a, float n)
{
    float r = fmod(a, n);
    if (r < 0)
        r += n;
    return r;
}

float phase_error(float a, float b, float X) { return fmod_pos((a - b) + X / 2.0, X) - X / 2.0; }

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

    if (flags & CTRL_INP)
    {
        return PARAM_INP_AMP;
    }

    return PARAM_OFS;
}

void init_channel(Channel* ch, ChannelState* chst, int8_t scene)
{
    ch->shared_phase = 0;

    if (scene < 0)
    {
        chst->src_input = -1;
        for (uint8_t s = 0; s < N_SCENES; s++)
        {
            for (uint8_t p = 0; p < N_PARAMS; p++)
            {
                chst->params[s][p] = 0;
            }
            chst->params[s][PARAM_FRQ]     = 85;
            chst->params[s][PARAM_INP_AMP] = INT16_MAX;
        }
    }
    else
    {
        for (uint8_t p = 0; p < N_PARAMS; p++)
        {
            chst->params[scene][p] = 0;
        }
        chst->params[scene][PARAM_FRQ]     = 85;
        chst->params[scene][PARAM_INP_AMP] = INT16_MAX;
    }
}

int16_t find_denominator(float value, int16_t max_mult, float tol)
{
    // Remove integer part first
    float frac = value - floorf(value);
    if (frac == 0.0f)
    {
        return 1; // already integer
    }

    for (int16_t mult = 1; mult <= max_mult; mult++)
    {
        float test    = frac * mult;
        float nearest = roundf(test);
        if (fabsf(test - nearest) < tol)
        {
            return mult;
        }
    }
    return -1; // not found within max_mult
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

void update_channel(Channel* ch, SystemState* state, ChannelState* chst)
{
    int8_t param  = param_index(state->ctrl_flags);
    int16_t delta = state->encoder_delta[ch->encoder];
    int8_t shift  = state->button_pressed_t[ch->button] > 0;

    if (state->ctrl_flags & CTRL_FRQ)
    {
        int16_t idx = freq_neighbour(chst->params[state->active_scene_id][param], delta);
        if (idx >= 0)
        {
            chst->params[state->active_scene_id][param] = quantized_multipliers[idx];
            ch->blink_hue                               = quantized_multipliers_colors[idx];
            ch->blink_until                             = state->time + 1500;
        }
    }
    else if (state->ctrl_flags & CTRL_QNT)
    {
        chst->quantize_mode = delta_modulo_step(chst->quantize_mode, state->encoder_delta[ch->encoder], QUANTIZE_MODE_COUNT);
    }
    else if (state->ctrl_flags & CTRL_MON)
    {
        if (state->button_released_t[ch->button] > 50)
        {
            assign_event(ASSIGN_CHANNEL, ch->id);
        }
    }
    else
    {
        chst->params[state->active_scene_id][param] += shift ? delta * 512 : delta * 64;

        /*
        if (state->button_released_t[ch->button] > 1000)
        {
            init_channel(ch);
        }
        else if (state->button_released_t[ch->button] > 50)
        {
            ch->params[state->active_scene_id][param] = 0;
        }
        */
    }
}

void compute_channel(Channel* ch, SystemState* state, Scene* scenes, ChannelState* chst)
{
    float dt_s            = state->dt / 1e3f;
    int16_t avg[N_PARAMS] = {0};
    for (uint8_t s = 0; s < N_SCENES; s++)
    {
        if (scenes[s].contribution == 0)
        {
            continue;
        }

        for (uint8_t p = 0; p < N_PARAMS; p++)
        {
            avg[p] += (int16_t) (((int32_t) chst->params[s][p] * scenes[s].contribution) / 255);
        }
    }

    float freq_param = avg[PARAM_FRQ] / (float) N_FREQ_SCALE;
    if (freq_param >= 0)
    {
        ch->freq_multiplier = freq_param + 1.0f;
    }
    else
    {
        ch->freq_multiplier = -1.0f / (freq_param - 1.0f);
    }

    ch->gcd = find_denominator(ch->freq_multiplier, 8, 0.01f);

    float offset       = (float) avg[PARAM_OFS];
    float amp          = (float) avg[PARAM_AMP] / 2.0f;
    float shape        = (float) avg[PARAM_SHP] / INT16_MAX;
    float phs          = (float) avg[PARAM_PHS] / INT16_MAX;
    float eff_freq     = g_clk.beat_freq_smooth * ch->freq_multiplier;
    float phase_delta  = dt_s * (eff_freq + ch->phase_correction);
    float phase_length = ch->gcd > 0 ? ch->gcd * ch->freq_multiplier : 1.0f;
    ch->shared_phase   = fmodf(ch->shared_phase + phase_delta, phase_length);

    if (ch->gcd > 0 && g_clk.have_beat)
    {
        float beat_mode  = floorf(fmodf(g_clk.beat_counter, ch->gcd)) + g_clk.beat_phase;
        ch->target_phase = fmodf(beat_mode * ch->freq_multiplier, phase_length);
        ch->diff         = ch->gcd > 0 ? phase_error(ch->target_phase, ch->shared_phase, phase_length) : 0;
        // TODO: If we momentary activate scene, set ch->phase to target_phase?
    }
    else
    {
        ch->diff = 0;
    }

    ch->phase_correction = (ch->phase_correction * (1.0f - k_sync) + ch->diff * k_sync);

    float phase = fmodf(ch->shared_phase + phs, 1.0f);
    float raw   = wavetable_lookup(phase, shape) / (float) INT16_MAX;
    float value = offset + amp * raw;

    float input_amp = (float) avg[PARAM_INP_AMP] / INT16_MAX;

    if (chst->src_input >= 0)
    {
        value += state->input_state[chst->src_input] * input_amp;
    }

    if (value > INT16_MAX)
        value = INT16_MAX;
    else if (value < INT16_MIN)
        value = INT16_MIN;

    switch (chst->quantize_mode)
    {
    case QUANTIZE_CONTINUOUS:
        ch->output_level = quantize_value((int16_t) value, state->quantize_mask);
        break;
    default:
        ch->output_level = (int16_t) value;
    }
}

void write_channel(Channel* ch, SystemState* state, ChannelState* chst)
{
    // ws2811_setled_hsv(ch->led, 255, 255, 255);
    if (state->ctrl_flags == CTRL_QNT)
    {
        ws2811_setled_hsv(ch->led, quantize_mode_color[chst->quantize_mode], 255, chst->quantize_mode == QUANTIZE_DISABLED ? 0 : 25);
    }
    else if (state->ctrl_flags & CTRL_MON)
    {
        if (chst->src_input >= 0)
        {
            ws2811_setled_hsv(ch->led, 0, 0, 12);
        }
        else
        {
            ws2811_setled_hsv(ch->led, 0, 0, 0);
        }
    }
    else if (state->ctrl_flags & CTRL_INP)
    {
        ws2811_setled_dac(ch->led, chst->params[state->active_scene_id][PARAM_INP_AMP]);
    }
    else if (ch->blink_until > state->time)
    {
        ws2811_setled_hsv(ch->led, ch->blink_hue, 255, state->blink_fast ? 25 : 0);
    }
    else
    {
        ws2811_setled_dac(ch->led, ch->output_level);
    }

    dacadc_write(ch->dac_channel, ch->output_level);
}
