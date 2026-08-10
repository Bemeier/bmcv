#ifndef INC_LIB_CLOCK_SYNC_H_
#define INC_LIB_CLOCK_SYNC_H_

#include <stdbool.h>
#include <stdint.h>

// A clock input is a physical edge, and physical edges bounce; an audio-rate
// signal patched into the jack does the same thing deliberately. Two bounds
// stand between the jack and the tempo estimate:
//
//   - a pulse closer than CLOCK_MIN_PULSE_US to the previous one is not a
//     pulse. At 4 pulses per beat this still admits 15000 BPM, so nothing a
//     sequencer sends is ever rejected - only bounce is.
//   - an interval that works out to a tempo outside [CLOCK_BPM_MIN,
//     CLOCK_BPM_MAX] is not a measurement, and does not move beat_freq.
//
// Neither is theoretical. 1e6 / 0 used to reach beat_freq whenever two pulses
// landed inside the post-reset guard below, and freq_est is a leaky integrator:
// one inf makes every later estimate inf, every channel's phase NaN, and the
// wavetable index whatever a NaN casts to. Nothing short of a reboot recovered.
#define CLOCK_MIN_PULSE_US 1000u
#define CLOCK_BPM_MIN 1.0f
#define CLOCK_BPM_MAX 1000.0f

// A reset and a clock pulse are usually patched from the same source and so
// arrive together. The first pulse after a reset therefore starts the count
// rather than measuring an interval against whatever came before it.
#define CLOCK_RESET_GUARD_US 2000u

// PULSES_PER_BEAT differs by source and the caller sets it accordingly - see
// engine.c's apply_clock_events. A CV clock is one pulse per step; MIDI Clock
// is fixed at 24 per quarter note by the MIDI spec, not a module setting.
#define CLOCK_PULSES_PER_BEAT_CV 4u
#define CLOCK_PULSES_PER_BEAT_MIDI 24u

typedef struct
{
  uint8_t PULSES_PER_BEAT;

  uint32_t last_pulse_us;
  uint32_t last_beat_start_us;
  uint32_t last_reset_us;
  uint32_t last_pulse_delta_us;
  uint32_t pulse_counter;
  uint64_t beat_counter;

  bool have_beat;
  bool have_pulse;

  float beat_freq;
  float beat_freq_smooth;
  float bpm;

  float beat_phase;

  float freq_est; // running estimate behind beat_freq_smooth; must reset with the rest of the state
} ClockState;

// The clock lives in EngineState, one per module instance. It used to be a
// single global, which was fine for firmware (there is exactly one module) but
// makes two simulated modules in one process share a tempo.
void Clock_Init(ClockState* clk);
void Clock_Reset(ClockState* clk, uint32_t now_us);
void Clock_Trigger(ClockState* clk, uint32_t now_us);
void Clock_Poll(ClockState* clk, uint32_t now_us);

// Change the source's resolution, dropping the pulse history that was counted
// in the old one. Call this rather than assigning PULSES_PER_BEAT: the counter
// is expressed in the old source's units, and Clock_Poll divides by the new
// one, so a bare assignment leaves beat_phase reading whole beats out of range
// until the next pulse happens to renormalize it. A no-op when the value has
// not actually changed, so it is safe to call every tick.
void Clock_SetPulsesPerBeat(ClockState* clk, uint8_t ppb);

#endif /* INC_LIB_CLOCK_SYNC_H_ */
