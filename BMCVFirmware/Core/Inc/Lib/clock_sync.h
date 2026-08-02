#ifndef INC_LIB_CLOCK_SYNC_H_
#define INC_LIB_CLOCK_SYNC_H_

#include <stdbool.h>
#include <stdint.h>

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

#endif /* INC_LIB_CLOCK_SYNC_H_ */
