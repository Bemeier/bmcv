#ifndef INC_DRIVERS_MIDI_H_
#define INC_DRIVERS_MIDI_H_

#include "stm32g4xx_hal.h" // IWYU pragma: keep
#include "usbd_midi.h"     // IWYU pragma: keep

void MIDI_addToUSBReport(uint8_t cable, uint8_t message, uint8_t param1, uint8_t param2);
void update_midi();
uint8_t midi_idle();

#endif /* INC_DRIVERS_MIDI_H_ */