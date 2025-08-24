#ifndef INC_LIB_CLOCK_SYNC_H_
#define INC_LIB_CLOCK_SYNC_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint32_t last_edge_us;     // µs timestamp of last pulse
    uint32_t last_beat_anchor; // µs timestamp of "pulse 0" of the beat
    uint32_t beat_period_us;   // smoothed duration of one beat
    uint32_t beat_dt_us;
    uint32_t last_ipi_us;
    uint32_t beat_period_raw; // last measured beat duration
    uint32_t pp_counter;      // counts pulses within current beat (0..PPB-1)
    uint8_t PPB;              // pulses per beat (config: 1/2/4/8/16/24)
    uint32_t beat_counter;
    uint32_t last_poll_us;

    float beat_freq;

    float synced_beats;   // Master beat phase accumulator
    float pll_integral;   // Integral error for PI control
    float last_beat_time; // Last beat timestamp (seconds)

    float k_p;           // Proportional gain
    float k_i;           // Integral gain
    float integral_max;  // Anti-windup clamp
    float beat_freq_min; // Min beat freq (Hz, ~6 BPM)
    float beat_freq_max; // Max beat freq (Hz, ~1200 BPM)

    float ema_alpha; // smoothing factor, e.g. 0.1
    bool have_beat;  // true after first full beat measured
    float clock_s;
    float phase;
    float last_phase;
    float bpm;
} ClockState;

void Clock_Init(void);
void Clock_Reset(void);
void Clock_Trigger(uint32_t now_us);
void Clock_Poll(uint32_t now_us);

extern ClockState g_clk;

#endif /* INC_LIB_CLOCK_SYNC_H_ */
