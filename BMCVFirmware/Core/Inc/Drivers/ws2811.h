#ifndef INC_DRIVERS_WS2811_H_
#define INC_DRIVERS_WS2811_H_

#include "stm32g4xx_hal.h" // IWYU pragma: keep

// WS2812
#define LED_COUNT 21
#define WS2811_BITS 24
#define RST_PERIODS 64
#define WS2811_BUF_LEN ((WS2811_BITS * LED_COUNT) + RST_PERIODS)

// 800KHz = 1.25ms pulses
#define T1H 45; // 44/90 * 1.25ms = 0.625ms
#define T0H 18; // 18/90 * 1.25ms = 0.25ms

typedef union
{
    struct
    {
        uint8_t b;
        uint8_t g;
        uint8_t r;
    } color;

    uint32_t data;
} WS2811_LED_DATA;

void ws2811_init(TIM_HandleTypeDef *htim, uint32_t channel);

void ws2811_setled_rgb(uint16_t idx, uint8_t r, uint8_t g, uint8_t b);
void ws2811_setled_hsv(uint16_t idx, uint8_t h, uint8_t s, uint8_t v);
void ws2811_setled_adcr(uint16_t idx, int16_t val);
void ws2811_setled_dac(uint16_t idx, int32_t val);

void ws2811_commit();
void ws2811_update();

void ws2811_dma_complete_callback(TIM_HandleTypeDef *htim);
uint8_t ws2811_dma_completed(void);

#endif /* INC_DRIVERS_WS2811_H_ */