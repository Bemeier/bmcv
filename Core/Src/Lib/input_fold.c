#include "input_fold.h"
#include "channel.h"
#include "config.h"
#include "helpers.h"
#include "hw_setup.h"
#include "hw_state.h"
#include "ux_state.h"
#include <string.h>

uint8_t input_trig_step(int16_t cv, uint8_t* state)
{
  if (!*state && cv >= TRIG_THRESH)
  {
    *state = 1;
    return 1;
  }
  if (cv < TRIG_THRESH_LOW)
  {
    *state = 0;
  }
  return 0;
}

void input_frames_init(InputFrames* in, UxState* ux, uint32_t now_us)
{
  memset(in, 0, sizeof(*in));
  in->curr.time = now_us;
  ux->hw_state  = &in->curr;
}

uint8_t input_fold(InputFrames* in, UxState* ux, const InputSample* sample, uint32_t now_us)
{
  const HwSetup* hw = ux->hw_setup;

  // This tick's frame is built in place over the last one, which is kept whole
  // for the level comparisons below. Every field is written before anything
  // downstream reads it - engine_tick runs after input_fold returns - and the
  // one thing called from inside here, channel_take_trig, reads engine state
  // rather than this.
  in->prev = in->curr;

  HwState* prev = &in->prev;
  HwState* curr = &in->curr;

  curr->dt   = now_us - prev->time;
  curr->time = now_us;

  for (uint8_t g = 0; g < N_INPUTS; g++)
  {
    curr->trigger_src[TRIG_SRC_INPUT(g)] = sample->cv_trig[hw->input_adc_idx[g]];
  }

  // Channels can trigger each other. This consumes the pending flag off
  // EngineState, so it must happen exactly once per tick.
  for (uint8_t c = 0; c < N_CHANNELS; c++)
  {
    curr->trigger_src[TRIG_SRC_CHANNEL(c)] = channel_take_trig(c, ux->engine_state);
  }

  // Clock and reset are *latched*, not acted on - engine_tick owns the clock.
  // Doing it here meant this layer reached into ClockState and into every
  // channel's phase, which is not what a transducer does. The engine sees the
  // same now_us on the same tick, so nothing about the timing changes.
  curr->clock_pulse = 0;
  curr->clock_reset = 0;

  uint8_t has_clock_input = 0;
  uint8_t has_reset_input = 0;

  int32_t slider_cv = 0;
  for (uint8_t g = 0; g < N_INPUTS; g++)
  {
    const InputMode mode = (InputMode) ux->engine_config->input_mode[g];

    if (mode == INPUT_CLOCK)
    {
      has_clock_input = 1;
      if (curr->trigger_src[TRIG_SRC_INPUT(g)])
        curr->clock_pulse = 1;
    }

    if (mode == INPUT_RESET)
    {
      has_reset_input = 1;
      if (curr->trigger_src[TRIG_SRC_INPUT(g)])
        curr->clock_reset = 1;
    }

    if (mode == INPUT_SLIDER)
      slider_cv += sample->cv_raw[hw->input_adc_idx[g]] * 2;
  }

  // MIDI only drives the clock/reset when nothing is patched for the job - a
  // jack always wins over a cable that is not even physical. The two gate
  // independently: a module might have a reset jack in use but no clock jack,
  // or the other way round.
  curr->clock_source_is_midi = !has_clock_input;
  if (!has_clock_input && sample->midi_clock_trig)
    curr->clock_pulse = 1;
  if (!has_reset_input && sample->midi_reset_trig)
    curr->clock_reset = 1;

  curr->slider_state = (uint16_t) iclamp((int32_t) sample->slider_raw - slider_cv, SLIDER_MIN_VALUE, SLIDER_MAX_VALUE);

  for (uint8_t i = 0; i < N_INPUTS; i++)
  {
    // ADC units are a quarter of DAC units at the same voltage.
    curr->input_state[i] = (int16_t) (sample->cv_raw[hw->input_adc_idx[i]] * 4);
  }

  uint8_t dirty = 0;

  for (uint8_t b = 0; b < N_BUTTONS; b++)
  {
    curr->button_state[b] = sample->button_down[b] != 0;

    // Level changes force a UX pass; durations and gestures are ui_input's
    // job, derived from this level every engine tick.
    dirty |= curr->button_state[b] != prev->button_state[b];
  }

  for (uint8_t e = 0; e < N_ENCODERS; e++)
  {
    curr->encoder_state[e] = sample->encoder_pos[e];
    curr->encoder_delta[e] = (int16_t) (curr->encoder_state[e] - prev->encoder_state[e]);
    dirty |= curr->encoder_delta[e] != 0;
  }

  return dirty;
}
