#ifndef INC_LIB_BMCV_H_
#define INC_LIB_BMCV_H_

#include "channel.h"
#include "ctrl_button.h"
#include "scene.h"
#include "stm32g474xx.h"
#include "stm32g4xx_hal.h" // IWYU pragma: keep
#include <stdint.h>

#define STATE_RINGBUF_SIZE 2
#include "state.h"

static SystemState state[STATE_RINGBUF_SIZE];
static SystemState* prev_state = &state[0];
static SystemState* curr_state = &state[1];

static ConfigState system_state;

static Scene scene[N_SCENES];
static Channel channel[N_ENCODERS];
static CtrlButton ctrl_buttons[N_CTRL_BUTTONS];

static uint16_t quantize_mask = 0b111111111111;

void bmcv_init(uint16_t mpc_interrupt_pin, ADC_TypeDef* slider_adc);

void bmcv_main(uint32_t now_us, uint32_t now_ms);

void bmcv_state_update(uint32_t now);

void bmcv_handle_adc_conversion_complete(ADC_HandleTypeDef* hadc);

void bmcv_poll_tasks();

void bmcv_handle_gpio_exti(uint16_t GPIO_Pin);

void bmcv_handle_txrx_complete(SPI_HandleTypeDef* hspi);

void bmcv_assign_input_to_channel(int8_t i, int8_t c);

void bmcv_clear_channel(int8_t c, int8_t all_scenes);

void bmcv_clear_scene(int8_t s);

void bmcv_assign_channel_to_channel(int8_t c_src, int8_t c_dst);

void bmcv_assign_channel_to_scene(int8_t c_src, int8_t s_dst);

void bmcv_assign_scene_to_scene(int8_t s_src, int8_t s_dst);

void bmcv_store_setup(int8_t dst);

void bmcv_load_setup(int8_t src);

#endif /* INC_LIB_BMCV_H_ */
