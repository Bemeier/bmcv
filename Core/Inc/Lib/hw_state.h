#ifndef INC_LIB_HW_STATE_H_
#define INC_LIB_HW_STATE_H_

#include "hw_setup.h"
#include <stdint.h>

// One frame of hardware, as the engine sees it: what the knobs, buttons, jacks
// and encoders were doing at a given microsecond. Produced by input_fold.c from
// a raw InputSample and consumed by everything downstream.

typedef struct
{
  uint32_t time; // timestamp of state
  uint32_t dt;   // time since last state

  uint16_t slider_state;

  // Indexed by TRIG_SRC_INPUT(i) / TRIG_SRC_CHANNEL(c) - see hw_setup.h.
  uint8_t trigger_src[N_TRIG_SRC];

  // Clock events the input layer saw this tick, acted on by engine_tick. The
  // input layer only latches them: it is a transducer from raw hardware to
  // HwState and nothing more, so it does not reach into the clock or the
  // channels itself.
  uint8_t clock_pulse;
  uint8_t clock_reset;

  // True when no input is configured as the clock source, so clock_pulse (if
  // set this tick) came from MIDI rather than a jack - a fact about this
  // tick's config, not a latch, so it reads the same whether or not
  // clock_pulse itself is set. engine_tick uses it to pick
  // ClockState.PULSES_PER_BEAT: a CV clock is one pulse per step, MIDI Clock
  // is fixed at 24 per quarter note.
  uint8_t clock_source_is_midi;

  // Raw level only. Press durations and gestures are derived once, in
  // ui_input.c - HwState deliberately no longer carries them, so there is no
  // second source of truth for "how long has this been held".
  uint8_t button_state[N_BUTTONS];

  int16_t encoder_state[N_ENCODERS];
  int16_t encoder_delta[N_ENCODERS]; // change of encoder since last state

  int16_t input_state[N_INPUTS];
} HwState;

#endif /* INC_LIB_HW_STATE_H_ */
