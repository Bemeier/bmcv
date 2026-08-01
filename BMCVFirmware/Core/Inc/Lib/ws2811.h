#ifndef INC_DRIVERS_WS2811_H_
#define INC_DRIVERS_WS2811_H_

#include "hw_setup.h" // IWYU pragma: keep - LED_COUNT
#include <stdint.h>

// WS2812
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

// Raw pixel write. Colour decisions (HSV, CV-level mapping) belong to the
// presentation layer in led_fb.c - this driver only pushes bytes.
void ws2811_setled_rgb(uint16_t idx, uint8_t r, uint8_t g, uint8_t b);

void ws2811_commit();
void ws2811_update();

uint8_t ws2811_dma_completed(void);

#endif /* INC_DRIVERS_WS2811_H_ */
