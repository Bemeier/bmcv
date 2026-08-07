#include "clock_sync.h"
#include <math.h>
#include <stdint.h>

void Clock_Init(ClockState* clk)
{
  clk->PULSES_PER_BEAT  = 4;
  clk->have_beat        = false;
  clk->have_pulse       = false;
  clk->beat_freq        = 1.0f;
  clk->beat_freq_smooth = 1.0f;
  clk->freq_est         = 0.0f;

  // Explicitly, not by way of a zeroed caller: it is the divisor in both
  // Clock_Trigger and Clock_Poll, and both read it before anything writes it.
  clk->last_pulse_delta_us = 0;
  clk->last_pulse_us       = 0;

  Clock_Reset(clk, 0);
}

static float smooth_freq(ClockState* clk, float new_sample)
{
  const float alpha = 0.3f;
  if (clk->freq_est == 0.0f)
    clk->freq_est = new_sample;
  clk->freq_est = alpha * new_sample + (1.0f - alpha) * clk->freq_est;
  return clk->freq_est;
}

void Clock_Trigger(ClockState* clk, uint32_t now_us)
{
  // Before anything observes the pulse, so a bounce cannot advance the beat
  // counter either.
  if (clk->have_pulse && (now_us - clk->last_pulse_us) < CLOCK_MIN_PULSE_US)
  {
    return;
  }

  if (!clk->have_beat)
  {
    Clock_Reset(clk, now_us);
  }

  clk->have_beat = true;
  if (now_us - clk->last_reset_us > CLOCK_RESET_GUARD_US)
  {
    clk->pulse_counter++;

    // Only against a pulse that actually happened. Clock_Reset drops the pulse
    // history, so the first pulse after a reset has nothing to measure from.
    if (clk->have_pulse)
    {
      clk->last_pulse_delta_us = now_us - clk->last_pulse_us;
    }
  }

  if (clk->last_pulse_delta_us > 0)
  {
    float beat_freq = 1000000.0f / ((float) clk->last_pulse_delta_us * (float) clk->PULSES_PER_BEAT);

    // An implausible interval is a glitch, not a tempo. Rejected rather than
    // clamped: clamping still drags the estimate to the bound and takes several
    // good pulses to come back, which is audible as every LFO changing speed.
    if (beat_freq * 60.0f >= CLOCK_BPM_MIN && beat_freq * 60.0f <= CLOCK_BPM_MAX)
    {
      clk->beat_freq        = beat_freq;
      clk->beat_freq_smooth = smooth_freq(clk, beat_freq);
      clk->bpm              = roundf(clk->beat_freq_smooth * 600.0f) / 10.0f;
    }
  }

  if (clk->pulse_counter >= clk->PULSES_PER_BEAT)
  {
    clk->pulse_counter = 0;
    clk->beat_counter += 1;
    clk->last_beat_start_us = now_us;
    clk->beat_phase         = 0.0f;
  }

  clk->have_pulse    = true;
  clk->last_pulse_us = now_us;
}

void Clock_Poll(ClockState* clk, uint32_t now_us)
{
  if (clk->last_pulse_delta_us > 0 && now_us - clk->last_pulse_us > 4u * clk->last_pulse_delta_us)
  {
    clk->have_beat = false;
  }

  if (clk->have_beat)
  {
    uint32_t dt_pulse = now_us - clk->last_pulse_us;
    if (clk->last_pulse_delta_us == 0)
    {
      clk->beat_phase = 0.0f;
      return;
    }
    float pulse_fraction = (float) dt_pulse / (float) clk->last_pulse_delta_us;
    float next_phase     = (clk->pulse_counter + pulse_fraction) / (float) clk->PULSES_PER_BEAT;
    if (next_phase >= 1.0f)
      next_phase -= 1.0f;
    clk->beat_phase = next_phase;
  }
  else
  {
    float phase = ((now_us - clk->last_reset_us) * 0.000001f) * clk->beat_freq;
    phase -= (float) ((uint32_t) phase);
    clk->beat_phase = phase;
  }
}

void Clock_Reset(ClockState* clk, uint32_t now_us)
{
  clk->pulse_counter      = 0;
  clk->beat_counter       = 0;
  clk->last_reset_us      = now_us;
  clk->last_beat_start_us = now_us;
  clk->beat_phase         = 0.0f;

  // The interval that spans a reset is not an interval: the pulse before it
  // belongs to whatever was playing previously. Dropping it costs one pulse of
  // tempo measurement and is what keeps a zero delta out of the divide in
  // Clock_Trigger. last_pulse_delta_us itself stays - it is also Clock_Poll's
  // timeout reference, and a reset must not make a stopped clock look live.
  clk->have_pulse = false;
}
