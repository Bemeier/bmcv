// Empty on purpose.
//
// usblink.c includes this with an IWYU pragma because the real usbd_midi.h
// drags the HAL in behind it, not because it uses anything from it. On a host
// there is no HAL, and the fact that this file can be empty and usblink.c still
// compiles is itself worth knowing: nothing in the link touches the chip.

#ifndef BMCV_TEST_FAKE_STM32G4XX_HAL_H_
#define BMCV_TEST_FAKE_STM32G4XX_HAL_H_
#endif
