#include "ws2811_hal.h"

WS2811_LED_DATA ws2811_rgb_data[LED_COUNT];
uint16_t ws2811_pwm_data[WS2811_BUF_LEN];

extern TIM_HandleTypeDef htim3;
volatile uint8_t WS_DATA_COMPLETE_FLAG;

static TIM_HandleTypeDef* ws2811_timer = NULL;
static uint32_t ws2811_channel         = 0;

void ws2811_init(TIM_HandleTypeDef* htim, uint32_t channel)
{
  ws2811_timer   = htim;
  ws2811_channel = channel;

  for (uint16_t bufidx = 0; bufidx < WS2811_BUF_LEN; bufidx++)
  {
    ws2811_pwm_data[bufidx] = 0;
  }

  for (uint8_t ledidx = 0; ledidx < LED_COUNT; ledidx++)
  {
    ws2811_setled_rgb(ledidx, 0, 0, 0);
  }

  ws2811_commit();

  WS_DATA_COMPLETE_FLAG = 1;
}

void ws2811_setled_rgb(uint16_t idx, uint8_t r, uint8_t g, uint8_t b)
{
  WS2811_LED_DATA* led = &ws2811_rgb_data[idx];
  led->color.r         = r;
  led->color.g         = g;
  led->color.b         = b;
}

void ws2811_commit()
{
  uint16_t bufidx = 0;
  for (uint8_t led = 0; led < LED_COUNT; led++)
  {
    for (uint8_t bits = WS2811_BITS; bits > 0; bits--)
    {
      if ((ws2811_rgb_data[led].data >> (bits - 1)) & 0x01)
      {
        ws2811_pwm_data[bufidx] = T1H;
      }
      else
      {
        ws2811_pwm_data[bufidx] = T0H;
      }
      bufidx++;
    }
  }
}

// Give up on the frame in flight and let the next tick try again. Both callers
// are failure paths where the DMA will never report completion for itself.
static void ws2811_abandon_frame(void)
{
  HAL_TIM_PWM_Stop_DMA(ws2811_timer, ws2811_channel);
  WS_DATA_COMPLETE_FLAG = 1;
}

void ws2811_update()
{
  if (ws2811_timer == NULL || !ws2811_dma_completed())
    return;
  ws2811_commit();
  WS_DATA_COMPLETE_FLAG    = 0;
  HAL_StatusTypeDef result = HAL_TIM_PWM_Start_DMA(ws2811_timer, ws2811_channel, (uint32_t*) ws2811_pwm_data, WS2811_BUF_LEN);
  if (result != HAL_OK)
  {
    // Nothing was started, so the completion callback that clears this flag
    // will never run - and the guard at the top of this function refuses to
    // draw while the flag reads "in flight". One failed start would therefore
    // park the panel on its last frame permanently, and take the amber DFU
    // indication with it, since fw_update.c waits on this same flag before it
    // hands over to the bootloader.
    //
    // Stopping the channel first so the HAL's own state is not left
    // half-started, which is what would make the retry fail the same way.
    ws2811_abandon_frame();
  }
}

// A transfer error is delivered to the HAL's error callback rather than to the
// completion one, so without this it leaves the flag clear and freezes the
// panel exactly as a failed start would.
void ws2811_dma_error_callback(TIM_HandleTypeDef* htim)
{
  if (htim == ws2811_timer)
  {
    ws2811_abandon_frame();
  }
}

void ws2811_dma_complete_callback(TIM_HandleTypeDef* htim)
{
  if (htim == ws2811_timer)
  {
    HAL_TIM_PWM_Stop_DMA(ws2811_timer, ws2811_channel);
    WS_DATA_COMPLETE_FLAG = 1;
  }
}

uint8_t ws2811_dma_completed(void) { return WS_DATA_COMPLETE_FLAG; }
