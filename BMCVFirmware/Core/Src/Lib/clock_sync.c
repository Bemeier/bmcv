#include "clock_sync.h"
#include <math.h>
#include <stdint.h>

ClockState g_clk = {};

void Clock_Init(void)
{
    g_clk.PULSES_PER_BEAT = 4;
    g_clk.have_beat       = false;
    Clock_Reset();
}

float smooth_freq(float new_sample)
{
    static float freq_est = 0.0f;
    const float alpha     = 0.05f; // smoothing factor (0..1), lower = smoother
    if (freq_est == 0.0f)
        freq_est = new_sample;
    freq_est = alpha * new_sample + (1.0f - alpha) * freq_est;
    return freq_est;
}

void Clock_Trigger(uint32_t now_us)
{
    if (!g_clk.have_beat)
    {
        Clock_Reset();
    }

    if (g_clk.pulse_counter == 0 && g_clk.beat_counter == 0)
    {
        g_clk.last_reset_us = now_us;
    }

    g_clk.have_beat = true;
    g_clk.pulse_counter++;
    g_clk.last_pulse_delta_us = now_us - g_clk.last_pulse_us;

    if (g_clk.last_pulse_us > 0) // && g_clk.last_ipi_us > 5000000)
    {
        g_clk.beat_freq        = 1000000.0f / (float) (g_clk.last_pulse_delta_us * g_clk.PULSES_PER_BEAT);
        g_clk.beat_freq_smooth = smooth_freq(g_clk.beat_freq);
        g_clk.bpm              = roundf(g_clk.beat_freq_smooth * 600.0f) / 10.0f;
    }

    if (g_clk.pulse_counter >= g_clk.PULSES_PER_BEAT)
    {
        g_clk.pulse_counter = 0;
        g_clk.beat_counter += 1;
        g_clk.last_beat_start_us = now_us;
        g_clk.beat_phase         = 0.0f;
    }
    g_clk.last_pulse_us = now_us;
}

void Clock_Poll(uint32_t now_us)
{
    if (g_clk.last_pulse_delta_us > 0 && now_us - g_clk.last_pulse_us > 4 * g_clk.last_pulse_delta_us)
    {
        g_clk.have_beat = false;
    }

    if (g_clk.have_beat)
    {
        uint32_t dt_pulse = now_us - g_clk.last_pulse_us;
        if (dt_pulse == 0 || g_clk.last_pulse_delta_us == 0)
            return;
        float pulse_fraction = (float) dt_pulse / (float) g_clk.last_pulse_delta_us;
        float next_phase     = (g_clk.pulse_counter + pulse_fraction) / (float) g_clk.PULSES_PER_BEAT;
        g_clk.beat_phase     = fmodf(next_phase, 1.0f);
    }
    else
    {
        g_clk.beat_phase = fmodf((now_us / 1000000.0f) * g_clk.beat_freq, 1.0f);
    }
}

void Clock_Reset(void)
{
    g_clk.pulse_counter       = 0;
    g_clk.beat_counter        = 0;
    g_clk.last_reset_us       = 0;
    g_clk.last_pulse_delta_us = 0;
    g_clk.beat_phase          = 0.0f;
    g_clk.beat_freq           = 1.0f;
    g_clk.beat_freq_smooth    = 1.0f;
}
