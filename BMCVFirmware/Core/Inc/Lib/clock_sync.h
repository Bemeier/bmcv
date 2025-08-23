#ifndef INC_LIB_CLOCK_SYNC_H_
#define INC_LIB_CLOCK_SYNC_H_

#include <stdbool.h>
#include <stdint.h>

#define N_CLOCK_DIVIDERS 8
#define N_CLOCK_MULTIPLIERS 8

typedef struct
{
    volatile uint32_t last_capture;
    volatile uint32_t avg_period;
    volatile uint32_t period_raw;
    volatile bool have_period;
    volatile bool running;

    uint32_t guard_us;
    uint32_t min_period_us;
    uint32_t max_period_us;
    float alpha;

    uint32_t timeout_us;
    volatile uint32_t last_edge_time;

    struct
    {
        uint8_t N;
        uint8_t cnt;
        bool pulse_pending;
    } divs[N_CLOCK_DIVIDERS];

    struct
    {
        uint8_t M;
        uint32_t next_tick;
        uint8_t k;
        bool armed;
    } mults[N_CLOCK_MULTIPLIERS];

    float phase;
    float bpm;
} ClockState;

void Clock_Init(void);
void Clock_AttachDivider(uint8_t ch, uint8_t N);
void Clock_AttachMultiplier(uint8_t ch, uint8_t M);
void Clock_Reset(void);
void Clock_Trigger(uint32_t now);
void Clock_PollTimeout(uint32_t now);   // call in main loop or 1ms tick
void Clock_OnOutputCompare(uint8_t ch); // called from TIM2 OC IRQ for mult pulses

static ClockState g_clk;

#endif /* INC_LIB_CLOCK_SYNC_H_ */
