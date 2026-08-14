#include "sim_rt.h"
#include "input_fold.h"
#include <string.h>

void sim_tickdiv_config(SimTickDiv* d, float sample_rate_hz, float control_rate_hz)
{
  memset(d, 0, sizeof(*d));

  if (sample_rate_hz <= 0.0f || control_rate_hz <= 0.0f)
  {
    d->divider = 1;
    d->dt_q32  = (uint64_t) 1 << 32;
    return;
  }

  double ratio = (double) sample_rate_hz / (double) control_rate_hz;
  int64_t div  = (int64_t) (ratio + 0.5);
  if (div < 1)
    div = 1;
  d->divider = (uint32_t) div;

  // Derive the step from the rounded divider, not from the requested control
  // rate: the engine really does tick every `divider` frames, and dt has to
  // agree with that or phase accumulates an error.
  double dt_us = (1000000.0 * (double) d->divider) / (double) sample_rate_hz;
  d->dt_q32    = (uint64_t) (dt_us * 4294967296.0 + 0.5);
  if (d->dt_q32 == 0)
    d->dt_q32 = 1;
}

void sim_tickdiv_reconfig(SimTickDiv* d, float sample_rate_hz, float control_rate_hz)
{
  uint64_t us_q32 = d->us_q32;
  uint32_t now_us = d->now_us;

  sim_tickdiv_config(d, sample_rate_hz, control_rate_hz);

  d->us_q32 = us_q32;
  d->now_us = now_us;
  // counter deliberately starts over: the new divider counts whole frames of a
  // different length, so a partial count from the old one means nothing.
}

uint8_t sim_tickdiv_step(SimTickDiv* d)
{
  d->counter++;
  if (d->counter < d->divider)
    return 0;

  d->counter = 0;
  d->us_q32 += d->dt_q32;

  uint32_t now = (uint32_t) (d->us_q32 >> 32);
  d->dt_us     = now - d->now_us; // wrap-safe, like every other dt here
  d->now_us    = now;
  return 1;
}

void sim_trig_reset(SimTrigLatch* t) { memset(t, 0, sizeof(*t)); }

void sim_trig_sample(SimTrigLatch* t, uint8_t channel, int16_t cv)
{
  if (channel >= N_INPUTS)
    return;

  // Same hysteresis the ADC driver applies, so the sim sees the edges the
  // hardware would.
  if (input_trig_step(cv, &t->state[channel]))
  {
    t->pending[channel] = 1;
  }
}

uint8_t sim_trig_take(SimTrigLatch* t, uint8_t channel)
{
  if (channel >= N_INPUTS)
    return 0;

  uint8_t p           = t->pending[channel];
  t->pending[channel] = 0;
  return p;
}

void sim_trig_fire(SimTrigLatch* t, uint8_t channel)
{
  if (channel < N_INPUTS)
    t->pending[channel] = 1;
}

/* ---- filling an InputSample --------------------------------------------- */

void sim_input_cv(InputSample* s, SimTrigLatch* t, const HwSetup* hw, uint8_t jack, float volts)
{
  if (jack >= N_INPUTS)
    return;

  uint8_t ch  = hw->input_adc_idx[jack];
  int16_t raw = sim_volts_to_adc(volts);

  s->cv_raw[ch] = raw;
  sim_trig_sample(t, ch, raw);
}

void sim_input_take_trigs(InputSample* s, SimTrigLatch* t)
{
  for (uint8_t ch = 0; ch < N_INPUTS; ch++)
  {
    s->cv_trig[ch] = sim_trig_take(t, ch);
  }
}

uint16_t sim_slider_raw(float pos01)
{
  if (pos01 < 0.0f)
    pos01 = 0.0f;
  if (pos01 > 1.0f)
    pos01 = 1.0f;
  return (uint16_t) (SLIDER_MIN_VALUE + pos01 * (float) (SLIDER_MAX_VALUE - SLIDER_MIN_VALUE) + 0.5f);
}

void sim_input_slider(InputSample* s, float pos01) { s->slider_raw = sim_slider_raw(pos01); }

void sim_input_adopt(InputSample* s, const HwState* hw_state)
{
  if (!s || !hw_state)
    return;

  s->slider_raw = hw_state->slider_state;

  for (uint8_t b = 0; b < N_BUTTONS; b++)
  {
    s->button_down[b] = hw_state->button_state[b];
  }

  for (uint8_t e = 0; e < N_ENCODERS; e++)
  {
    s->encoder_pos[e] = hw_state->encoder_state[e];
  }

  // Not cv_raw: the CV a host is feeding in is the host's patch, and nothing
  // downstream reads a stale one as a change the way an encoder position is
  // read as a turn. The next frame overwrites it regardless.
}
