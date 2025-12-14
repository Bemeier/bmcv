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
    uint32_t beat_counter;

    bool have_beat;

    float beat_freq;
    float beat_freq_smooth;
    float bpm;

    float beat_phase;
} ClockState;

void Clock_Init(void);
void Clock_Reset(void);
void Clock_Trigger(uint32_t now_us);
void Clock_Poll(uint32_t now_us);

extern ClockState g_clk;

#endif /* INC_LIB_CLOCK_SYNC_H_ */
