#include "usblink.h"

#include "stm32g4xx_hal.h" // IWYU pragma: keep
#include "usbd_midi.h"
#include "version.h"
#include <string.h>

#define STRINGIFY_(x) #x
#define STRINGIFY(x) STRINGIFY_(x)

extern USBD_HandleTypeDef hUsbDeviceFS;

// Set from the USB interrupt, read from the main loop, so volatile and nothing
// more. Neither side does a read-modify-write the other can land inside:
// `asked` is only ever incremented by the interrupt and `sent` only by the main
// loop, which is why they are two counters rather than one credit both adjust.
static volatile uint32_t asked = 0;
static uint32_t sent           = 0;

static volatile bool tx_busy = false;

static volatile bool remote_pending = false;
static RemoteInput remote_staged;

static volatile bool command_pending = false;
static RemoteCommand command_staged;

static volatile bool dfu_requested = false;

// One instance, frozen while it goes out. The stack splits it into packets
// itself and reports completion once, at the end, so this has to sit still for
// the whole transfer - which is why it is a copy and not the live instance.
static uint8_t snapshot[sizeof(BmcvInstance)];

// What this build is, for a host that has just found the device and has no
// other way to ask. The same string the flash descriptor carries for a debug
// probe - see bmcv_probe.h - reached the other way round.
const char* USBD_BMCV_Version(uint16_t* length)
{
  static const char version[] = STRINGIFY(FW_VERSION_MAJOR) "." STRINGIFY(FW_VERSION_MINOR) "." STRINGIFY(FW_VERSION_PATCH);

  *length = (uint16_t) sizeof(version); // including the terminator
  return version;
}

/* ---- what arrives ------------------------------------------------------- */

// Overrides the __weak stub in the USB class, and runs in the USB interrupt.
// It does the least it can: copy, latch, return.
void USBD_BMCV_VendorDataOut(uint8_t* data, uint16_t len)
{
  if (len < 1)
    return;

  switch (data[0])
  {
  case USBLINK_OP_SNAPSHOT_REQ:
    asked++;
    break;

  case USBLINK_OP_REMOTE_INPUT:
    // Length checked rather than assumed: a short message is a host that got
    // it wrong, and copying what follows would read past what arrived.
    if (len >= 1 + sizeof(RemoteInput))
    {
      memcpy(&remote_staged, data + 1, sizeof(RemoteInput));
      remote_pending = true;
    }
    break;

  case USBLINK_OP_REMOTE_COMMAND:
    if (len >= 1 + sizeof(RemoteCommand))
    {
      memcpy(&command_staged, data + 1, sizeof(RemoteCommand));
      command_pending = true;
    }
    break;

  case USBLINK_OP_ENTER_DFU:
    dfu_requested = true;
    break;

  default:
    break; // a request this build does not know is ignored, not answered
  }
}

// The snapshot has left. Fires once for the whole transfer, however many
// packets the stack split it into.
void USBD_BMCV_VendorDataIn(void) { tx_busy = false; }

// A host is starting, and nothing this side is holding belongs to it.
//
// Overrides the __weak stub in the USB class, which calls it on enumeration and
// whenever a host clears a halt on the vendor endpoints - which a browser does
// on every connect.
//
// Without this the link survived exactly one session. A browser that closes the
// device mid-snapshot leaves a transfer part-collected; clearing the halt then
// resets the endpoint out from under it, so its completion interrupt never
// arrives and `tx_busy` stays set for ever. usblink_poll returns at its first
// line from then on, and the module answers no host again until it is power
// cycled - not even a replug, since a USB reset does not reset the MCU.
//
// The credit counters go with it for a smaller reason: a session that asked for
// snapshots it never collected has no claim on the next one's first frame.
void USBD_BMCV_VendorReset(void)
{
  tx_busy = false;
  asked   = 0;
  sent    = 0;

  // Whatever the last host was holding down, it is not holding it now. The
  // input layer would time this out by itself a quarter of a second later; not
  // handing the next session a button already pressed is worth the two lines.
  remote_pending  = false;
  command_pending = false;

  // dfu_requested is deliberately not cleared. It is latched because the only
  // answer to it is a reboot that does not return, and a host that asked for
  // one and then went away still asked.
}

/* ---- what the main loop does with it ------------------------------------ */

uint8_t usblink_take_remote_input(RemoteInput* dst)
{
  if (!remote_pending)
    return 0;

  // Cleared first: a newer mailbox landing during the copy is newer, and
  // leaving the flag set means it is taken next pass rather than lost. The copy
  // itself can be torn by that, which is exactly what the mailbox's levels are
  // built to survive - see RemoteInput.
  remote_pending = false;
  *dst           = remote_staged;
  return 1;
}

uint8_t usblink_take_remote_command(RemoteCommand* dst)
{
  if (!command_pending)
    return 0;

  command_pending = false;
  *dst            = command_staged;
  return 1;
}

uint8_t usblink_dfu_requested(void) { return dfu_requested; }

void usblink_poll(const BmcvInstance* m)
{
  if (tx_busy)
    return;

  // A host that asked while the module was busy, or that asked far more often
  // than it collected, does not get to bank the difference: credit beyond the
  // window is dropped, so what goes out is the module as it is now rather than
  // a queue of frames from a moment ago.
  const uint32_t a = asked;
  if (a - sent > USBLINK_MAX_CREDITS)
    sent = a - USBLINK_MAX_CREDITS;

  if (a == sent)
    return; // nobody is waiting for one

  // Taken here, between engine ticks, so the copy is internally consistent.
  memcpy(snapshot, m, sizeof snapshot);

  tx_busy = true;
  sent++;

  if (USBD_BMCV_VendorSend(&hUsbDeviceFS, snapshot, (uint16_t) sizeof snapshot) != USBD_OK)
  {
    // Not configured, or the endpoint would not take it. The credit is spent
    // either way - the host will ask again, and pretending otherwise would
    // leave a snapshot owed for ever.
    tx_busy = false;
  }
}
