#include "engine.h"
#include "channel.h"
#include "clock_sync.h"
#include "error.h"
#include "helpers.h"
#include "hw_setup.h"
#include "hw_state.h"
#include "scene.h"
#include "ui_input.h"
#include "ux_state.h"

#define UX_UPDATE_INTERVAL MS(8)

// Act on the clock events the input layer latched into HwState this tick.
//
// This used to run inside input_fold, which meant the layer whose job
// is "raw hardware -> HwState" also drove the clock and reset every channel's
// phase. It latches now and the engine acts, so the input side is a pure
// transducer and the timestamp both see is the same one.
static void apply_clock_events(UxState* state, uint32_t now_us)
{
  ClockState* clk = &state->engine_state->clock;

  // Set before Poll as well as Trigger/Reset: Poll's phase interpolation
  // divides by PULSES_PER_BEAT too, and a stale value there would show as a
  // one-tick glitch in the LED phase right after the source changes.
  //
  // Through Clock_SetPulsesPerBeat rather than by assignment, because the pulse
  // count and the pulse it was counted from are in the old source's units and
  // have to go with it. Assigning here left beat_phase up to 4.75 - whole beats
  // of phase error handed to every channel's sync loop - for as long as it took
  // the new source to send a pulse.
  Clock_SetPulsesPerBeat(clk, state->hw_state->clock_source_is_midi ? CLOCK_PULSES_PER_BEAT_MIDI : CLOCK_PULSES_PER_BEAT_CV);

  Clock_Poll(clk, now_us);

  // Reset first: a reset arriving on the same tick as a pulse must land
  // before it, or the beat counter advances past the reset.
  if (state->hw_state->clock_reset)
  {
    Clock_Reset(clk, now_us);
    for (uint8_t c = 0; c < N_CHANNELS; c++)
    {
      channel_reset_phase(c, state->engine_state);
    }
  }

  if (state->hw_state->clock_pulse)
  {
    Clock_Trigger(clk, now_us);
  }
}

// Exponential average, so one slow loop does not make the readout jump.
static float fps_smooth(float prev, uint32_t dt_us)
{
  if (dt_us == 0)
    return prev;
  return prev * 0.95f + 0.05f * (1000000.0f / (float) dt_us);
}

void engine_tick(UxState* state, uint32_t now_us, uint8_t input_dirty)
{
  // Every tick, not just the ticks the UX layer runs on: a hold crossing its
  // threshold is not a level change, so nothing else would notice it.
  ui_input_update(&state->ui->in, state->hw_state);

  // Measured here rather than by each host, so every host reports the same
  // number computed the same way. dac_fps stays with the firmware, which is
  // the only host with a DAC service loop of its own.
  state->engine_state->engine_fps = fps_smooth(state->engine_state->engine_fps, state->hw_state->dt);

  // Any interaction dismisses a displayed error. That is UI policy, so it
  // belongs here rather than in the input layer that happens to detect the
  // interaction.
  if (input_dirty && error_any(state->engine_state))
  {
    error_clear(state->engine_state);
  }

  apply_clock_events(state, now_us);

  state->ui->blink_slow = (now_us % SLOW_BLINK_PERIOD) < (SLOW_BLINK_PERIOD / 2);

  scene_compute_contribution(state->engine_state, state->engine_config, state->hw_state->slider_state, state->ui->momentary_scene);

  for (uint8_t c = 0; c < N_CHANNELS; c++)
  {
    channel_compute(c, state->engine_state, state->engine_config, state->hw_state);
  }

  for (uint8_t c = 0; c < N_CHANNELS; c++)
  {
    channel_detect_trigger(c, state->engine_state);
  }

  // UX runs slower than the signal path: on input change, or on a fixed tick.
  // in.dt is the accumulated input time, so a handler and a renderer in the
  // same pass always age their timers by the same amount.
  if (input_dirty || state->ui->in.dt > UX_UPDATE_INTERVAL)
  {
    ux_update(state, now_us);
    ui_input_drain(&state->ui->in);
  }

  // Last, and once per tick. Last because the UX pass above is what toggles
  // UiState.muted[], and the level leaving the module this tick should reflect
  // the button press handled this tick rather than lag it by one. Once per
  // tick because that is what makes the ramp rate a property of the engine
  // instead of of how often a particular host asks for the output.
  for (uint8_t c = 0; c < N_CHANNELS; c++)
  {
    channel_apply_mute(c, state->engine_state, state->ui->muted[c], state->hw_state->dt);
  }
}
