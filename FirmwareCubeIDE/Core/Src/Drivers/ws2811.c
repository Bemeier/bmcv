#include "ws2811.h"
#include "stdlib.h"

WS2811_LED_DATA ws2811_rgb_data[LED_COUNT];
uint16_t ws2811_pwm_data[WS2811_BUF_LEN];

extern TIM_HandleTypeDef htim3;
volatile uint8_t WS_DATA_COMPLETE_FLAG;

static TIM_HandleTypeDef *ws2811_timer = NULL;
static uint32_t ws2811_channel = 0;

void ws2811_init(TIM_HandleTypeDef *htim, uint32_t channel) {
    ws2811_timer = htim;
    ws2811_channel = channel;

	for (uint16_t bufidx = 0; bufidx < WS2811_BUF_LEN; bufidx++) {
        ws2811_pwm_data[bufidx] = 0;
	}

    for (uint8_t ledidx = 0; ledidx < LED_COUNT; ledidx++) {
        ws2811_setled_rgb(ledidx, 0, 0, 0);
    }

    ws2811_commit();

	WS_DATA_COMPLETE_FLAG = 1;
}

void ws2811_setled_hsv(uint16_t idx, uint8_t h, uint8_t s, uint8_t v) {
    WS2811_LED_DATA * led = &ws2811_rgb_data[idx];
    uint8_t region, remainder, p, q, t;

    if (s == 0) {
        led->color.r = v;
        led->color.g = v;
        led->color.b = v;
        return;
    }

    region = h / 43;
    remainder = (h - region * 43) * 6;

    p = (v * (255 - s)) >> 8;
    q = (v * (255 - ((s * remainder) >> 8))) >> 8;
    t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;

    switch (region) {
        case 0:
            led->color.r = v; led->color.g = t; led->color.b = p; break;
        case 1:
            led->color.r = q; led->color.g = v; led->color.b = p; break;
        case 2:
            led->color.r = p; led->color.g = v; led->color.b = t; break;
        case 3:
            led->color.r = p; led->color.g = q; led->color.b = v; break;
        case 4:
            led->color.r = t; led->color.g = p; led->color.b = v; break;
        default:
            led->color.r = v; led->color.g = p; led->color.b = q; break;
    }
}


void ws2811_setled_adcr(uint16_t idx, int16_t val) {
    WS2811_LED_DATA * led = &ws2811_rgb_data[idx];
	// int16_t only safe because we know ADC values are only 14 bits, so they won't overflow here.
	int16_t abs_val = abs(val);
	int16_t blue_range = abs_val - 4096;
	uint8_t base_val = 255;
	if (blue_range < 0) {
		blue_range = 0;
		base_val = (abs_val / 16) & 0xFF;
	}
	led->color.b = ((blue_range / 16) & 0xFF);
	if (val > 0) {
		led->color.g = base_val;
		led->color.r =  0;
	} else {
		led->color.r = base_val;
		led->color.g = 0;
	}
}

void ws2811_setled_dac(uint16_t idx, int32_t val) {
    WS2811_LED_DATA * led = &ws2811_rgb_data[idx];
	// int16_t only safe because we know ADC values are only 14 bits, so they won't overflow here.
	int32_t abs_val = abs(val);
	int32_t blue_range = abs_val - 16384;
	uint8_t base_val = 255;
	if (blue_range < 0) {
		blue_range = 0;
		base_val = (abs_val / 64) & 0xFF;
	}
	led->color.b = ((blue_range / 64) & 0xFF);
	if (val > 0) {
		led->color.g = base_val;
		led->color.r =  0;
	} else {
		led->color.r = base_val;
		led->color.g = 0;
	}
}

void ws2811_setled_rgb(uint16_t idx, uint8_t r, uint8_t g, uint8_t b) {
    WS2811_LED_DATA * led = &ws2811_rgb_data[idx];
	led->color.r = r;
	led->color.g = g;
	led->color.b = b;
}

void ws2811_commit() {
	uint16_t bufidx = 0;
	for (uint8_t led = 0; led < LED_COUNT; led++) {
		for (uint8_t bits = WS2811_BITS; bits > 0; bits--) {
			if ((ws2811_rgb_data[led].data >> (bits - 1)) & 0x01) {
                        ws2811_pwm_data[bufidx] = T1H;
            }	else {
                        ws2811_pwm_data[bufidx] = T0H;
            }
            bufidx++;
	    }
	}
}

void ws2811_update() {
    if (ws2811_timer == NULL || !ws2811_dma_completed()) return;
    ws2811_commit();
    WS_DATA_COMPLETE_FLAG = 0;
    HAL_StatusTypeDef result = HAL_TIM_PWM_Start_DMA(ws2811_timer, ws2811_channel, (uint32_t*) ws2811_pwm_data, WS2811_BUF_LEN);
    if (result != HAL_OK) {
        // TODO
    }
}


void ws2811_dma_complete_callback(TIM_HandleTypeDef *htim) {
    if (htim == ws2811_timer) {
        HAL_TIM_PWM_Stop_DMA(ws2811_timer, ws2811_channel);
        WS_DATA_COMPLETE_FLAG = 1;
    }
}

uint8_t ws2811_dma_completed(void) {
    return WS_DATA_COMPLETE_FLAG;
}
