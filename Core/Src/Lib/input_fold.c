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

  // Zero is a legal slider position, so the "nobody is holding it" sentinel has
  // to be written rather than left to the memset. seq does not need the same
  // treatment: zero already means never written, which is what a fresh instance
  // wants to say.
  in->remote.slider_raw  = REMOTE_SLIDER_NONE;
  in->remote_slider_prev = REMOTE_SLIDER_NONE;
}

// Which remote panel to obey this tick - the mailbox, or nothing - and the two
// handovers that decide it.
//
// Liveness is RemoteInput.seq's second job; the header says why it is not
// optional. The slider is the one control that needs a rule beyond "merge
// both", because it is an absolute position: two panels claiming it are
// claiming different positions, where two panels holding a button are holding
// the same button.
//
// That rule is last-mover-wins. A new value in the mailbox takes the fader and
// remembers where the physical one was sitting at the time; the physical one
// takes it back by moving REMOTE_SLIDER_RELEASE_RAW away from there. The
// threshold is the whole reason this works on hardware: without it the ADC's
// own noise would read as a hand on the fader and hand control back within a
// tick or two of every remote move.
static const RemoteInput* remote_refresh(InputFrames* in, const InputSample* sample, uint32_t now_us)
{
  static const RemoteInput idle = {.slider_raw = REMOTE_SLIDER_NONE};

  const uint8_t was_live = in->remote_live;

  if (in->remote.seq != in->remote_seq)
  {
    in->remote_seq     = in->remote.seq;
    in->remote_seen_us = now_us;
    in->remote_live    = 1;
  }
  else if (in->remote_live && (uint32_t) (now_us - in->remote_seen_us) > REMOTE_TIMEOUT_US)
  {
    in->remote_live = 0;
  }

  // A mailbox arriving or leaving adds or removes its whole encoder
  // contribution at once. That is a step, not a turn, so the frame it will be
  // compared against moves with it and the delta comes out zero.
  //
  // This is what buys the writer its free choice of origin, and it is the
  // reason nothing on the far end has to know the module's absolute encoder
  // positions or agree a baseline with it - the same failure sim_input_adopt()
  // exists for, solved here instead of in every host. Without it, connecting a
  // page whose encoder happened to sit at 1000 would apply a thousand detents
  // of edit to a patch nobody touched.
  //
  // in->prev is this tick's copy of the last frame, so this affects the delta
  // being computed now and nothing after it.
  if (in->remote_live != was_live)
  {
    for (uint8_t e = 0; e < N_ENCODERS; e++)
    {
      const int16_t step        = in->remote.encoder_pos[e];
      in->prev.encoder_state[e] = (int16_t) (in->remote_live ? in->prev.encoder_state[e] + step : in->prev.encoder_state[e] - step);
    }
  }

  if (!in->remote_live)
  {
    // Forgotten rather than frozen, so that a writer coming back has to take
    // the fader again explicitly rather than inheriting a claim it made before
    // it went away.
    in->remote_slider_held = 0;
    in->remote_slider_prev = REMOTE_SLIDER_NONE;
    return &idle;
  }

  const int16_t want = in->remote.slider_raw;

  if (want != in->remote_slider_prev)
  {
    in->remote_slider_prev = want;
    in->remote_slider_held = want >= 0;
    in->remote_slider_ref  = sample->slider_raw;
  }
  else if (in->remote_slider_held)
  {
    int32_t moved = (int32_t) sample->slider_raw - (int32_t) in->remote_slider_ref;
    if (moved < 0)
      moved = -moved;
    if (moved > REMOTE_SLIDER_RELEASE_RAW)
      in->remote_slider_held = 0;
  }

  return &in->remote;
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

  // Merged into the three places below and nowhere else, so everything
  // downstream reads one HwState and cannot tell where a press came from.
  const RemoteInput* rem = remote_refresh(in, sample, now_us);

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

  // CV sums into whichever fader is being obeyed. A patched slider input is a
  // property of the module, not of the hand on the panel, so it applies to a
  // remote position exactly as it does to a physical one.
  const uint16_t slider_raw = in->remote_slider_held ? (uint16_t) rem->slider_raw : sample->slider_raw;

  curr->slider_state = (uint16_t) iclamp((int32_t) slider_raw - slider_cv, SLIDER_MIN_VALUE, SLIDER_MAX_VALUE);

  for (uint8_t i = 0; i < N_INPUTS; i++)
  {
    // ADC units are a quarter of DAC units at the same voltage.
    curr->input_state[i] = (int16_t) (sample->cv_raw[hw->input_adc_idx[i]] * 4);
  }

  uint8_t dirty = 0;

  for (uint8_t b = 0; b < N_BUTTONS; b++)
  {
    curr->button_state[b] = (sample->button_down[b] | rem->button_down[b]) != 0;

    // Level changes force a UX pass; durations and gestures are ui_input's
    // job, derived from this level every engine tick.
    dirty |= curr->button_state[b] != prev->button_state[b];
  }

  for (uint8_t e = 0; e < N_ENCODERS; e++)
  {
    curr->encoder_state[e] = (int16_t) (sample->encoder_pos[e] + rem->encoder_pos[e]);
    curr->encoder_delta[e] = (int16_t) (curr->encoder_state[e] - prev->encoder_state[e]);
    dirty |= curr->encoder_delta[e] != 0;
  }

  return dirty;
}
