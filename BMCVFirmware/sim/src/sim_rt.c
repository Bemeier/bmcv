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
