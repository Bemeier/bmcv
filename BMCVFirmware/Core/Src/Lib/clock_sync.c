#include "clock_sync.h"

// static inline uint32_t tim2_now(void) { return __HAL_TIM_GET_COUNTER(&htim2); }

void Clock_Init(void)
{
    g_clk.guard_us      = 2000;
    g_clk.min_period_us = 1000;
    g_clk.max_period_us = 20000000;
    g_clk.alpha         = 0.1f;
    g_clk.phase         = 0.0f;
    // TIM2 @ 1MHz:
    // PSC = (timclk / 1MHz) - 1; ARR = 0xFFFFFFFF
    // CH1: Input Capture on rising edge
    // CH2..CHn: Output Compare (for multipliers)
    // HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
}

void Clock_AttachDivider(uint8_t ch, uint8_t N)
{
    if (ch >= N_CLOCK_DIVIDERS || N == 0)
        return;
    g_clk.divs[ch].N             = N;
    g_clk.divs[ch].cnt           = 0;
    g_clk.divs[ch].pulse_pending = false;
}

void Clock_AttachMultiplier(uint8_t ch, uint8_t M)
{
    if (ch >= N_CLOCK_MULTIPLIERS || M < 2)
        return;
    g_clk.mults[ch].M     = M;
    g_clk.mults[ch].armed = false;
    g_clk.mults[ch].k     = 1;
}

// --- Input Capture IRQ handler (TIM2 CH1) ---
void Clock_Trigger(uint32_t now)
{
    uint32_t dt        = now - g_clk.last_capture; // wraps naturally on uint32_t
    g_clk.last_capture = now;

    if (dt < g_clk.guard_us)
        return; // reject bounces/doubles

    g_clk.period_raw = dt;

    // range check
    if (dt < g_clk.min_period_us || dt > g_clk.max_period_us)
    {
        // ignore outliers but still update last_edge_time to keep running
        g_clk.last_edge_time = now;
        return;
    }

    // EMA smoothing
    if (!g_clk.have_period)
    {
        g_clk.avg_period  = dt;
        g_clk.have_period = true;
    }
    else
    {
        float ap = (float) g_clk.avg_period;
        ap += g_clk.alpha * ((float) dt - ap);
        if (ap < (float) g_clk.min_period_us)
            ap = (float) g_clk.min_period_us;
        if (ap > (float) g_clk.max_period_us)
            ap = (float) g_clk.max_period_us;
        g_clk.avg_period = (uint32_t) ap;
    }

    g_clk.running        = true;
    g_clk.last_edge_time = now;
    g_clk.phase          = 0.0f;

    // Dividers: count and flag pulse
    for (int i = 0; i < N_CLOCK_DIVIDERS; ++i)
    {
        if (g_clk.divs[i].N == 0)
            continue;
        g_clk.divs[i].cnt++;
        if (g_clk.divs[i].cnt >= g_clk.divs[i].N)
        {
            g_clk.divs[i].cnt           = 0;
            g_clk.divs[i].pulse_pending = true; // drive a short gate in main loop or timer
        }
    }

    // Multipliers: (re)arm a schedule between now and next edge
    for (int i = 0; i < N_CLOCK_MULTIPLIERS; ++i)
    {
        if (g_clk.mults[i].M < 2)
            continue;
        g_clk.mults[i].k         = 1;
        g_clk.mults[i].armed     = true;
        uint32_t step            = g_clk.avg_period / g_clk.mults[i].M;
        g_clk.mults[i].next_tick = now + step;
        // Program TIM2 CH(2+i) compare register to next_tick and enable its IRQ
        // __HAL_TIM_SET_COMPARE(&htim2, channel_for(i), g_clk.mults[i].next_tick);
        // HAL_TIM_OC_Start_IT(&htim2, channel_for(i));
    }

    if (!g_clk.have_period)
    {
        g_clk.bpm = 2.0f;
    }
    else
    {
        g_clk.bpm = 60000000.0f / (float) g_clk.avg_period;
    }
}

// --- Output Compare IRQ handler(s) for multiplier pulses ---
void Clock_OnOutputCompare(uint8_t ch)
{
    if (!g_clk.mults[ch].armed)
        return;

    // Emit pulse on the corresponding output (hardware gate pin or timer PWM poke)
    // emit_gate_pulse(mult_output_pin[ch]);

    // Schedule next virtual tick if any
    uint8_t M = g_clk.mults[ch].M;
    g_clk.mults[ch].k++;
    if (g_clk.mults[ch].k >= M)
    {
        g_clk.mults[ch].armed = false; // done; next set will arm on the next real edge
        return;
    }

    uint32_t step = g_clk.avg_period / M;

    // Optional: small slew to new avg_period if it changed a lot
    g_clk.mults[ch].next_tick += step;
    // __HAL_TIM_SET_COMPARE(&htim2, channel_for(ch), g_clk.mults[ch].next_tick);
}

// --- Reset EXTI ---
void Clock_Reset(void)
{
    // Zero transport & phase; keep avg_period so tempo stays known
    g_clk.phase = 0.0f;
    for (int i = 0; i < 8; ++i)
    {
        if (g_clk.divs[i].N)
            g_clk.divs[i].cnt = 0;
        if (g_clk.mults[i].M >= 2)
            g_clk.mults[i].armed = false;
    }
}

// --- Poll timeout in 1 ms tick or main loop ---
void Clock_PollTimeout(uint32_t now)
{
    uint32_t to = g_clk.have_period ? (g_clk.avg_period * 5) / 2 : 500000; // 2.5x or 500ms
    uint32_t dt = now - g_clk.last_edge_time;
    if (dt > to)
    {
        g_clk.running = false;
    }
}
