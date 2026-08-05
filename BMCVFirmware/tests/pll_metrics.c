#include "pll_metrics.h"
#include "clock_sync.h"
#include "hw_setup.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

void pll_trace_reset(PllTrace* tr)
{
  tr->n       = 0;
  tr->skipped = 0;
}

void pll_clock_init(PllClock* c, float bpm)
{
  memset(c, 0, sizeof(*c));
  c->bpm             = bpm;
  c->jitter          = 0.0f;
  c->seed            = 12345u;
  c->running         = 1;
  c->pulses_per_beat = 4; // ClockState.PULSES_PER_BEAT
  c->next_us         = 0;
}

// A fixed sequence rather than rand(), so a jitter run that fails fails the
// same way next time.
static float jitter_next(PllClock* c)
{
  c->seed = c->seed * 1664525u + 1013904223u;
  return ((float) ((c->seed >> 8) & 0xFFFFu) / 32768.0f) - 1.0f; // -1..1
}

static uint32_t pulse_period_us(const PllClock* c) { return (uint32_t) (60000000.0f / (c->bpm * (float) c->pulses_per_beat)); }

static void trace_push(PllTrace* tr, const Fixture* f, uint8_t ch, float t_s)
{
  if (tr == NULL || tr->n >= PLL_TRACE_MAX)
    return;

  if (tr->decimate > 1)
  {
    if (tr->skipped + 1 < tr->decimate)
    {
      tr->skipped++;
      return;
    }
    tr->skipped = 0;
  }

  const ChannelEffective* eff = &f->engine_state.channels_effective[ch];
  const ClockState* clk       = &f->engine_state.clock;

  uint32_t i = tr->n++;

  tr->t_s[i] = t_s;

  // The loop works in cycles of the channel's own waveform. A beat is what the
  // module is lining up with, and is comparable across ratios.
  float ratio       = eff->freq_ratio;
  tr->err_beats[i]  = (ratio != 0.0f) ? eff->phase_error / ratio : 0.0f;
  tr->corr_hz[i]    = f->engine_state.channels_phase_correction[ch];
  tr->freq_hz[i]    = eff->freq_hz;
  tr->locked[i]     = (eff->gcd > 0 && clk->have_beat) ? 1u : 0u;
  tr->phase[i]      = eff->phase;
  tr->beat[i]       = clk->beat_counter;
  tr->beat_phase[i] = clk->beat_phase;
  tr->gcd[i]        = eff->gcd;
}

void pll_step(Fixture* f, PllClock* c, uint8_t ch, uint32_t dt_us, PllTrace* tr)
{
  uint32_t start = f->hw_state.time;

  // A pulse lands on the tick whose interval contains it. That is what the
  // hardware does too - input_fold latches an edge and engine_tick acts on it
  // with that tick's timestamp - so the quantisation here is the real one and
  // not an artifact of the harness.
  uint8_t pulse = 0;
  if (c->running)
  {
    uint32_t period = pulse_period_us(c);
    while ((int32_t) (c->next_us - (start + dt_us)) <= 0)
    {
      pulse = 1;
      c->next_us += period;
      if (c->jitter > 0.0f)
      {
        c->next_us = (uint32_t) ((float) c->next_us + jitter_next(c) * c->jitter * (float) period);
      }
    }
  }

  f->hw_state.clock_pulse = pulse;
  fixture_tick(f, dt_us);
  f->hw_state.clock_pulse = 0; // input_fold owns this on hardware; here we do

  trace_push(tr, f, ch, (float) f->hw_state.time * 1e-6f);
}

void pll_run(Fixture* f, PllClock* c, uint8_t ch, float seconds, PllTrace* tr)
{
  uint32_t ticks = (uint32_t) (seconds * 1e6f / (float) ENGINE_TICK_US);
  for (uint32_t i = 0; i < ticks; i++)
  {
    pll_step(f, c, ch, ENGINE_TICK_US, tr);
  }
}

// Run until the clock is at a given point within the beat.
//
// Where in the beat a disturbance lands changes how big it is - a ratio change
// exactly on a beat boundary produces no error at all, because that is the one
// phase every ratio agrees on. A test that happens to start on a whole number
// of seconds at 120bpm lands there every time and measures nothing, which is
// what the first version of the scene-transition case did.
void pll_run_to_beat_phase(Fixture* f, PllClock* c, uint8_t ch, float target)
{
  for (uint32_t guard = 0; guard < 200000; guard++)
  {
    float before = f->engine_state.clock.beat_phase;
    pll_step(f, c, ch, ENGINE_TICK_US, NULL);
    float after = f->engine_state.clock.beat_phase;

    // Crossed the target, wrap included.
    if ((before < target && after >= target) || (after < before && target > before))
      return;
  }
}

PllMetrics pll_measure(const PllTrace* tr, float tol_beats)
{
  PllMetrics m;
  memset(&m, 0, sizeof(m));
  m.settle_s = -1.0f;

  if (tr->n == 0)
    return m;

  m.duration_s = tr->t_s[tr->n - 1] - tr->t_s[0];

  // Settling, found from the end: the last moment it was outside tolerance is
  // the moment it settled. Looking forward instead would stop at the first
  // sample that happens to be small on the way through zero.
  uint32_t last_bad = 0;
  uint8_t ever_bad  = 0;
  for (uint32_t i = 0; i < tr->n; i++)
  {
    if (!tr->locked[i])
      continue;
    if (fabsf(tr->err_beats[i]) > tol_beats)
    {
      last_bad = i;
      ever_bad = 1;
    }
  }
  if (!ever_bad)
    m.settle_s = 0.0f;
  else if (last_bad + 1 < tr->n)
    m.settle_s = tr->t_s[last_bad] - tr->t_s[0];
  // else: never settled, stays -1

  // Peak first, because the crossing deadband is relative to it.
  float initial   = 0.0f;
  uint8_t have_in = 0;
  for (uint32_t i = 0; i < tr->n; i++)
  {
    if (!tr->locked[i])
      continue;
    if (!have_in)
    {
      initial = tr->err_beats[i];
      have_in = 1;
    }
    float a = fabsf(tr->err_beats[i]);
    if (a > m.peak_err_beats)
      m.peak_err_beats = a;
  }

  // Crossings, with a deadband.
  //
  // A settled loop's error dithers around zero at the noise floor, and counting
  // raw sign changes there gave 150 "oscillations" for a run that was visibly
  // dead flat. Only an excursion that gets meaningfully off zero counts, so
  // this is the number of times the loop genuinely went past and came back -
  // 0 for a monotone approach, 1 for a single overshoot, more for ringing.
  float deadband = fmaxf(tol_beats, 0.05f * m.peak_err_beats);
  int8_t sign    = 0;

  for (uint32_t i = 0; i < tr->n; i++)
  {
    if (!tr->locked[i] || fabsf(tr->err_beats[i]) <= deadband)
      continue;

    int8_t s = (tr->err_beats[i] > 0.0f) ? 1 : -1;
    if (sign == 0)
    {
      sign = s;
    }
    else if (s != sign)
    {
      m.crossings++;
      sign = s;
    }

    if (m.crossings > 0 && fabsf(tr->err_beats[i]) > m.overshoot_beats)
      m.overshoot_beats = fabsf(tr->err_beats[i]);
  }

  // Only meaningful against a disturbance worth measuring. A run that starts
  // already locked has an initial error at the noise floor, and dividing by it
  // produced ratios in the thousands.
  m.overshoot_ratio = (fabsf(initial) > 10.0f * tol_beats) ? m.overshoot_beats / fabsf(initial) : 0.0f;

  // Frequency pull, and how fast it changes. Both relative to the channel's own
  // rate, since a 0.1Hz correction is nothing to a 10Hz LFO and everything to a
  // 0.1Hz one.
  for (uint32_t i = 0; i < tr->n; i++)
  {
    float nominal = fabsf(tr->freq_hz[i]);
    if (nominal < 1e-6f)
      continue;

    float dev = fabsf(tr->corr_hz[i]) / nominal;
    if (dev > m.max_freq_dev)
      m.max_freq_dev = dev;

    if (i > 0)
    {
      float dt = tr->t_s[i] - tr->t_s[i - 1];
      if (dt > 1e-9f)
      {
        float slew = fabsf(tr->corr_hz[i] - tr->corr_hz[i - 1]) / dt / nominal;
        if (slew > m.max_freq_slew)
          m.max_freq_slew = slew;
      }
    }
  }

  // What is left at the end.
  uint32_t tail_from = tr->n - tr->n / 3;
  double sum_sq      = 0.0;
  uint32_t count     = 0;
  for (uint32_t i = tail_from; i < tr->n; i++)
  {
    if (!tr->locked[i])
      continue;
    float a = fabsf(tr->err_beats[i]);
    sum_sq += (double) a * (double) a;
    count++;
    if (a > m.max_err_tail_beats)
      m.max_err_tail_beats = a;
  }
  m.rms_err_tail_beats = (count > 0) ? (float) sqrt(sum_sq / (double) count) : 0.0f;

  for (uint32_t i = 1; i < tr->n; i++)
  {
    if (tr->gcd[i] != tr->gcd[i - 1])
      m.gcd_changes++;
  }

  // Phase continuity. The advance the oscillator's own rate accounts for,
  // against the advance that actually happened - both wrapped into +/- half a
  // cycle, since a wrap at a whole cycle is not a step.
  //
  // Only meaningful on an undecimated trace: sampling every Nth tick makes an
  // ordinary advance look like a jump.
  if (tr->decimate <= 1)
  {
    for (uint32_t i = 1; i < tr->n; i++)
    {
      float dt = tr->t_s[i] - tr->t_s[i - 1];
      if (dt <= 1e-9f)
        continue;

      float expected = (tr->freq_hz[i] + tr->corr_hz[i]) * dt;
      float actual   = tr->phase[i] - tr->phase[i - 1];

      while (actual > 0.5f)
        actual -= 1.0f;
      while (actual < -0.5f)
        actual += 1.0f;

      float jump = fabsf(actual - expected);
      if (jump > m.max_phase_jump)
        m.max_phase_jump = jump;
    }
  }

  return m;
}

void pll_report_header(void)
{
  fprintf(stdout, "\n  %-34s %8s %8s %6s %8s %9s %9s %9s\n", "scenario", "settle", "peak", "cross", "fdev", "fslew", "rms_tail", "jump");
  fprintf(stdout, "  %-34s %8s %8s %6s %8s %9s %9s %9s\n", "", "s", "beats", "n", "x rate", "x rate/s", "beats", "cycles");
  fprintf(stdout, "  %-34s %8s %8s %6s %8s %9s %9s %9s\n", "----------------------------------", "--------", "--------", "------",
          "--------", "---------", "---------", "---------");
}

void pll_report(const char* name, const PllMetrics* m)
{
  char settle[16];
  if (m->settle_s < 0.0f)
    snprintf(settle, sizeof(settle), "never");
  else
    snprintf(settle, sizeof(settle), "%.3f", (double) m->settle_s);

  fprintf(stdout, "  %-34s %8s %8.4f %6u %8.3f %9.1f %9.5f %9.5f\n", name, settle, (double) m->peak_err_beats, m->crossings,
          (double) m->max_freq_dev, (double) m->max_freq_slew, (double) m->rms_err_tail_beats, (double) m->max_phase_jump);
}
