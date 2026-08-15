#ifndef INC_LIB_FW_UPDATE_H_
#define INC_LIB_FW_UPDATE_H_

// Handing the module to the STM32's ROM DFU bootloader, in two halves.
//
// It is done across a reset rather than by jumping straight there, because the
// bootloader will not bring USB up if it inherits a running system. Measured,
// not guessed: after a direct jump the core had SCB->VTOR = 0x1FFF0000, so the
// bootloader was demonstrably executing, but the USB and SYSCFG registers could
// not be read over SWD at all - their clocks were still gated - and the module
// never enumerated. Coming in through CPY works because BOOT0 hands over a chip
// that has just reset, and this is how the software path gets the same thing.
//
// CPY is what the button is lettered on the panel; the schematic calls the same
// net FN2/Boot, which is the name to search the KiCad files for.

// Ask for the ROM bootloader, then reset into it. Never returns.
//
// Call from the main loop, not from the USB interrupt that asked for it: it
// tears down the USB stack on its way out.
void fw_update_enter_dfu(void);

// Act on that request, if there is one. Call as the first thing in main(),
// before HAL_Init and before a single peripheral is touched - the whole point
// is to reach the bootloader with the chip still in the state reset left it in.
//
// Does nothing and returns immediately on an ordinary boot.
void fw_update_check_boot(void);

#endif /* INC_LIB_FW_UPDATE_H_ */
