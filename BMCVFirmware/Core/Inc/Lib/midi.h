#ifndef INC_DRIVERS_MIDI_H_
#define INC_DRIVERS_MIDI_H_

#include "stm32g4xx_hal.h" // IWYU pragma: keep
#include "usbd_midi.h"     // IWYU pragma: keep

void MIDI_addToUSBReport(uint8_t cable, uint8_t message, uint8_t param1, uint8_t param2);
void update_midi();
uint8_t midi_idle();

// Non-zero once the host has asked the module to reboot into the ROM DFU
// bootloader. Latched in the USB interrupt and never cleared - the main loop's
// only correct response is fw_update_enter_dfu(), which does not return.
uint8_t midi_dfu_requested();

// Answer any pending host control request that needs the IN endpoint. Cheap
// enough to call every pass; it does nothing until there is something to say.
void midi_poll_control();

#endif /* INC_DRIVERS_MIDI_H_ */
