#ifndef USBD_WEBUSB_H_
#define USBD_WEBUSB_H_

#include <stdint.h>

// What lets a browser talk to this module directly, with no driver to install.
//
// The module carries a vendor-specific interface alongside its MIDI one. On its
// own that would be a device Windows has no driver for; these descriptors are
// how it is told to bind WinUSB to it, automatically and with nothing for
// anyone to install. Chrome can then claim the interface through navigator.usb.
//
// The reason it exists is not speed, though it is faster. It is that Chrome
// enumerates MIDI once and hands out port objects backed by that enumeration,
// so a module that leaves the USB bus - a power cycle, or a reflash - cannot be
// reached again until the browser is restarted. WebUSB has no such problem: the
// device is picked when it is used, and hot-plug is reported properly. See
// docs/live-module.md.
//
// Deliberately free of every HAL and stack header. These are byte arrays whose
// internal lengths have to agree with each other in four places, which is
// exactly the kind of thing a compiler cannot check and a test can - see
// tests/test_usb_descriptors.c. usbd_desc.c wraps them in the signatures the ST
// stack wants.

// The interface WinUSB is asked to bind to. Interface 0 is MIDI and stays with
// the class driver it already has.
#define BMCV_WEBUSB_INTERFACE 1

// Vendor request codes, published in the BOS descriptor so the host learns them
// rather than assuming. Any values would do; these two are simply not zero.
#define BMCV_WEBUSB_VENDOR_CODE 0x41
#define BMCV_MSOS20_VENDOR_CODE 0x42

// wIndex on the request Windows makes for the descriptor set below.
#define BMCV_MSOS20_DESCRIPTOR_INDEX 0x07

// Ask the module what firmware it is running: a vendor request answered with a
// NUL-terminated version string.
//
// A control request rather than a message on the bulk endpoints, because it is
// a question about the device rather than part of the stream - and because a
// short reply sharing an endpoint with 2384-byte snapshots would have to be
// told apart from one by length, which is the kind of framing this transport
// exists to avoid.
#define BMCV_REQ_VERSION 0x43

// Sizes, stated so the descriptors that carry them and the test that checks
// them are reading the same numbers.
#define BMCV_BOS_TOTAL_LEN 57
#define BMCV_MSOS20_TOTAL_LEN 178

// The BOS descriptor: what capabilities this device has beyond USB 2.0. Two of
// them, one saying "this is a WebUSB device" and one saying "ask Windows to
// read the descriptor set below".
const uint8_t* bmcv_bos_descriptor(uint16_t* length);

// The MS OS 2.0 descriptor set: the part that actually names WinUSB as the
// driver for the vendor interface.
const uint8_t* bmcv_msos20_descriptor(uint16_t* length);

#endif /* USBD_WEBUSB_H_ */
