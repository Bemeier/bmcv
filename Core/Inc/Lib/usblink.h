#ifndef INC_LIB_USBLINK_H_
#define INC_LIB_USBLINK_H_

#include "input_fold.h" // IWYU pragma: keep
#include "instance.h"   // IWYU pragma: keep
#include <stdint.h>

// Everything a browser says to this module, and everything it hears back.
//
// A dedicated pair of bulk endpoints on the vendor interface - see
// USB_Device/App/usbd_webusb.h for how a browser comes to be allowed near them.
// Nothing here is encoded or framed: bulk transfers keep their boundaries, so
// one transfer is one message and a snapshot is simply the instance's bytes.
//
// This replaced the same traffic carried over MIDI SysEx, and the difference is
// most of a layer. That path had to encode every byte into seven bits, frame
// 2384 of them into a message spanning fifty-seven transfers, reassemble it on
// the far side, and take its turn against the engine's own control changes on a
// shared endpoint. None of that exists here.
//
// What made it worth moving was none of those, though: a browser enumerates
// MIDI once, so a module that leaves the USB bus cannot be reached again until
// the browser restarts. WebUSB has no such problem. See docs/live-module.md.

// One transfer, first byte first.
typedef enum
{
  // Send one snapshot. One request buys one, so the host paces the module -
  // see USBLINK_MAX_CREDITS.
  USBLINK_OP_SNAPSHOT_REQ = 0x01,

  // A RemoteInput follows. Levels and free-running positions, merged by
  // input_fold; see RemoteInput.
  USBLINK_OP_REMOTE_INPUT = 0x02,

  // A RemoteCommand follows: reset, or reset and forget storage.
  USBLINK_OP_REMOTE_COMMAND = 0x03,

  // Hand the module to the ROM DFU bootloader. Acted on from the main loop,
  // never from the USB interrupt this arrives in - entering DFU tears down the
  // USB stack, which is not something to do from inside its own handler.
  //
  // Here rather than over MIDI because the page that uses it is already a
  // WebUSB page: it flashes over DFU. One transport for the whole of flashing
  // beats a reboot that goes one way and the image that follows it another.
  USBLINK_OP_ENTER_DFU = 0x04,
} UsbLinkOp;

// How many snapshots the module will run ahead of the host's requests. The same
// bargain the MIDI path struck and for the same reason: a module streaming flat
// out sends faster than a browser can decode and draw, and what that produces
// is not a higher frame rate but a main thread that never catches up. More than
// one so the endpoint does not idle for a round trip; not many more, or the
// pacing is lost and stale frames queue behind the live one.
#define USBLINK_MAX_CREDITS 2

// Collect a mailbox if one arrived since the last call. Both are staged in the
// interrupt and moved into the instance here, from the main loop between ticks:
// input_fold reads the input mailbox every tick, and an interrupt rewriting it
// halfway through that read is the one thing its design does not tolerate.
uint8_t usblink_take_remote_input(RemoteInput* dst);
uint8_t usblink_take_remote_command(RemoteCommand* dst);

// Non-zero once a host has asked for the bootloader. Latched and never cleared -
// the only correct response is fw_update_enter_dfu(), which does not return.
uint8_t usblink_dfu_requested(void);

// Send a snapshot if one has been asked for and the endpoint is free. Copies
// the instance, so call it between ticks and what goes out is internally
// consistent - which a debug probe's read of the same memory is not.
void usblink_poll(const BmcvInstance* m);

#endif /* INC_LIB_USBLINK_H_ */
