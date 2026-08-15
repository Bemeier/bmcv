// Just enough of the USB device stack for usblink.c to compile on a host.
//
// usblink.c is the only driver-layer file worth testing natively: everything in
// it is bookkeeping - opcodes, credits, a busy flag - and every one of its
// failure modes is a module that answers nothing, which is indistinguishable on
// the bench from a cable problem, a driver problem or a browser problem. It took
// an evening of pulling cables to find the last one.
//
// So the three symbols it needs from the stack are declared here instead, and
// tests/test_usblink.c supplies them. Nothing in this file is a copy of stack
// behaviour: pdev is opaque, the return codes are the two values usblink.c
// compares against, and the handlers are the ones it overrides.
//
// The real ones are in Middlewares/ST/.../Class/MIDI/Inc/usbd_midi.h. If they
// gain an argument, this stops compiling, which is the intended outcome.

#ifndef BMCV_TEST_FAKE_USBD_MIDI_H_
#define BMCV_TEST_FAKE_USBD_MIDI_H_

#include <stdint.h>

#define USBD_OK 0U
#define USBD_FAIL 3U

// Opaque here. usblink.c only ever passes &hUsbDeviceFS straight through.
typedef struct
{
  uint8_t dev_state;
} USBD_HandleTypeDef;

uint8_t USBD_BMCV_VendorSend(USBD_HandleTypeDef* pdev, uint8_t* data, uint16_t len);

// Overridden by usblink.c, which is the code under test.
void USBD_BMCV_VendorDataOut(uint8_t* data, uint16_t len);
void USBD_BMCV_VendorDataIn(void);
void USBD_BMCV_VendorReset(void);
const char* USBD_BMCV_Version(uint16_t* length);

#endif /* BMCV_TEST_FAKE_USBD_MIDI_H_ */
