#ifndef INC_LIB_BMCV_H_
#define INC_LIB_BMCV_H_

#include "instance.h"
#include "stm32g474xx.h"
#include "stm32g4xx_hal.h" // IWYU pragma: keep
#include <stdint.h>

// The module. Everything the firmware runs on lives inside this one struct -
// see instance.h - and it is named here rather than kept file-static purely so
// a live debugger has something to attach to: every measurement worth watching
// on hardware (engine_state.dac_fps, engine_state.clock.bpm, the output levels,
// the folded input frame) is a member path from here, at a fixed address.
//
// Nothing in the firmware should reach for it. The calls below take what they
// need, and other hosts allocate their own instance - which is the whole
// reason the clock and the error flags stopped being globals.
extern BmcvInstance bmcv;

void bmcv_init(uint16_t mpc_interrupt_pin, ADC_TypeDef* slider_adc);

void bmcv_main(uint32_t now_us);

uint8_t bmcv_state_update(uint32_t now);

void bmcv_flush_leds(void);

void bmcv_handle_adc_conversion_complete(ADC_HandleTypeDef* hadc);

void bmcv_poll_tasks();

void bmcv_handle_gpio_exti(uint16_t GPIO_Pin);

void bmcv_handle_txrx_complete(SPI_HandleTypeDef* hspi);

#endif /* INC_LIB_BMCV_H_ */
