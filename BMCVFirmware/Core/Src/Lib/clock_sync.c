#include "clock_sync.h"
#include <math.h>
#include <stdint.h>

ClockState g_clk = {};

void Clock_Init(void)
{
    g_clk.PPB = 4;
    Clock_Reset();
}

void Clock_Trigger(uint32_t now_us)
{
    g_clk.pp_counter++;
    g_clk.last_ipi_us = now_us - g_clk.last_edge_us;
    if (g_clk.last_edge_us > 0 && g_clk.last_ipi_us < 5000000)
    {
        g_clk.beat_freq = 1000000.0f / (float) (g_clk.last_ipi_us * g_clk.PPB);
        g_clk.bpm       = g_clk.beat_freq * 60.0f;
    }

    if (g_clk.pp_counter >= g_clk.PPB)
    {
        g_clk.pp_counter = 0;
        g_clk.beat_counter += 1;
        g_clk.last_beat_anchor = now_us;
        g_clk.phase            = 0;
    }
}

void Clock_Poll(uint32_t now_us)
{
    uint32_t dt_pulse = now_us - g_clk.last_edge_us;
    if (dt_pulse == 0)
        return;
    float pulse_fraction = (float) dt_pulse / (float) g_clk.last_ipi_us;
    g_clk.phase          = fmodf((g_clk.pp_counter + pulse_fraction) / (float) g_clk.PPB, 1.0f);
}

void Clock_Reset(void)
{
    g_clk.pp_counter   = 0;
    g_clk.beat_counter = 0;
    g_clk.phase        = 0.0f;
    g_clk.last_phase   = 0.0f;
}
