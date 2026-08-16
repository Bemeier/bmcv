// The link a browser talks to the module over: opcodes in, snapshots out.
//
// Small enough to read in a sitting and worth testing anyway, because it has no
// good failure mode. Everything it can get wrong - a credit not spent, a busy
// flag never cleared, a length not checked - shows up on the bench as a module
// that answers nothing, which is exactly what a bad cable, an unbound driver
// and a browser holding a stale handle also look like. Telling those apart cost
// several evenings; this is where that stops being necessary.
//
// The USB stack is three symbols, declared in tests/fakes/usb/usbd_midi.h and
// supplied below. Nothing here simulates an endpoint: what a transfer *does* is
// the stack's business, and what this file checks is what the module decides to
// hand it.

#include "instance.h"
#include "testkit.h"
#include "usbd_midi.h"
#include "usblink.h"
#include <string.h>

/* ---- the stack, as far as usblink.c can see it --------------------------- */

USBD_HandleTypeDef hUsbDeviceFS;

static struct
{
  int calls;          // how many transfers were handed over
  uint16_t last_len;  // and how long the last one was
  uint8_t first_byte; // its first byte, which is enough to identify a snapshot
  uint8_t refuse;     // make the endpoint refuse, the way a torn-down stack does
} tx;

uint8_t USBD_BMCV_VendorSend(USBD_HandleTypeDef* pdev, uint8_t* data, uint16_t len)
{
  CHECK(pdev == &hUsbDeviceFS);

  if (tx.refuse)
    return USBD_FAIL;

  tx.calls++;
  tx.last_len   = len;
  tx.first_byte = len ? data[0] : 0;
  return USBD_OK;
}

// One whole snapshot leaving the endpoint: the stack reports completion once,
// at the end, however many packets it split it into.
static void transfer_completes(void) { USBD_BMCV_VendorDataIn(); }

/* ---- what a host sends --------------------------------------------------- */

static BmcvInstance module;

static void reset_all(void)
{
  memset(&tx, 0, sizeof tx);
  bmcv_instance_init(&module, NULL, 0);

  // The state the module comes up in, and the state it is put back into
  // whenever a host arrives. Without it every case here would inherit the
  // credits and the busy flag of the one before, which is the same coupling
  // between sessions this function exists to break on hardware.
  USBD_BMCV_VendorReset();
}

static void ask_for_snapshot(void)
{
  uint8_t msg[] = {USBLINK_OP_SNAPSHOT_REQ};
  USBD_BMCV_VendorDataOut(msg, sizeof msg);
}

// A mailbox with `seq` in it, laid out the way the host builds it: the opcode,
// then the struct.
static void send_mailbox(uint32_t seq, uint8_t button0)
{
  uint8_t msg[1 + sizeof(RemoteInput)];
  RemoteInput in = {0};

  in.slider_raw     = REMOTE_SLIDER_NONE;
  in.seq            = seq;
  in.button_down[0] = button0;

  msg[0] = USBLINK_OP_REMOTE_INPUT;
  memcpy(msg + 1, &in, sizeof in);
  USBD_BMCV_VendorDataOut(msg, sizeof msg);
}

static void send_command(uint32_t seq, uint8_t op)
{
  uint8_t msg[1 + sizeof(RemoteCommand)];
  RemoteCommand cmd = {.op = op, .seq = seq};

  msg[0] = USBLINK_OP_REMOTE_COMMAND;
  memcpy(msg + 1, &cmd, sizeof cmd);
  USBD_BMCV_VendorDataOut(msg, sizeof msg);
}

/* ---- one request buys one snapshot --------------------------------------- */

TEST_CASE(nothing_goes_out_unasked)
{
  reset_all();

  for (int i = 0; i < 10; i++)
    usblink_poll(&module);

  CHECK(tx.calls == 0);
}

TEST_CASE(a_request_buys_exactly_one_snapshot)
{
  reset_all();
  ask_for_snapshot();

  usblink_poll(&module);
  CHECK(tx.calls == 1);
  CHECK(tx.last_len == BMCV_SNAPSHOT_BYTES);

  // And the credit is spent. Polling again without another request sends
  // nothing, which is the whole of the pacing: the module cannot get ahead of
  // what the page has managed to draw.
  transfer_completes();
  usblink_poll(&module);
  CHECK(tx.calls == 1);
}

TEST_CASE(the_endpoint_carries_one_transfer_at_a_time)
{
  reset_all();
  ask_for_snapshot();
  ask_for_snapshot();

  usblink_poll(&module);
  CHECK(tx.calls == 1);

  // The second request is not refused, it is simply not acted on until the
  // first transfer is done with the buffer - it is one static instance, frozen
  // while it goes out.
  usblink_poll(&module);
  CHECK(tx.calls == 1);

  transfer_completes();
  usblink_poll(&module);
  CHECK(tx.calls == 2);
}

TEST_CASE(a_host_that_asks_faster_than_it_reads_cannot_bank_the_difference)
{
  reset_all();

  // A page that asked a hundred times while the module was busy does not get a
  // hundred frames from a moment ago when it comes back; it gets the module as
  // it is now. Anything else is a queue of stale snapshots wearing the current
  // one's clothes.
  for (int i = 0; i < 100; i++)
    ask_for_snapshot();

  int sent = 0;
  for (int i = 0; i < 100; i++)
  {
    usblink_poll(&module);
    transfer_completes();
    sent = tx.calls;
    if (i > USBLINK_MAX_CREDITS + 2)
      break;
  }

  CHECK(sent == USBLINK_MAX_CREDITS);
}

TEST_CASE(a_refused_transfer_spends_its_credit_rather_than_owing_a_frame)
{
  reset_all();
  tx.refuse = 1;
  ask_for_snapshot();

  // The endpoint would not take it - not configured, or torn down. The host
  // will ask again; what must not happen is the busy flag being left set,
  // because nothing would ever clear it.
  usblink_poll(&module);
  CHECK(tx.calls == 0);

  tx.refuse = 0;
  ask_for_snapshot();
  usblink_poll(&module);
  CHECK(tx.calls == 1);
}

/* ---- a session ending, and the next one starting ------------------------- */

// The one this file exists for.
//
// A browser closing the device leaves whatever snapshot was in flight
// half-collected: the host stops reading, so the transfer never completes and
// its DataIn never arrives. Then the next session clears the endpoint's halt,
// which resets it out from under that transfer - and the flag saying "a send is
// outstanding" would stay set for the rest of the module's power-on life.
//
// Every session after the first found a module that answered nothing, and no
// amount of retrying, reloading or replugging fixed it: a USB reset does not
// reset the MCU, so nothing in here was ever cleared. Only a power cycle was.
TEST_CASE(a_session_that_ended_mid_transfer_does_not_wedge_the_next_one)
{
  reset_all();
  ask_for_snapshot();
  usblink_poll(&module);
  CHECK(tx.calls == 1);

  // The host goes away here, without ever collecting it. No DataIn.

  // A new one arrives: the stack calls this on enumeration and whenever a host
  // clears a halt on the vendor endpoints, which a browser does on connect.
  USBD_BMCV_VendorReset();

  ask_for_snapshot();
  usblink_poll(&module);
  CHECK(tx.calls == 2);
}

TEST_CASE(a_new_session_does_not_inherit_the_last_ones_credit)
{
  reset_all();
  for (int i = 0; i < 5; i++)
    ask_for_snapshot();

  USBD_BMCV_VendorReset();

  // Whatever the last host asked for, it is not owed to this one - its first
  // frame should be a snapshot it asked for itself.
  for (int i = 0; i < 5; i++)
    usblink_poll(&module);
  CHECK(tx.calls == 0);

  ask_for_snapshot();
  usblink_poll(&module);
  CHECK(tx.calls == 1);
}

// The same rule, but with the reset landing where it actually does: partway
// through a session that has credit outstanding and a transfer in flight.
//
// Honest about what it can and cannot catch. The bug this accompanies needed
// the reset to land *inside* usblink_poll, between its read of `asked` and its
// write of `sent`, which left `sent` ahead of `asked` - their difference then
// underflowed and the clamp handed the next session frames it never asked for.
// No single-threaded test can produce that interleaving, and this one does not
// fail against the old code.
//
// What it does is pin the contract the fix restored - one writer per counter,
// so the crossed state is unreachable rather than merely unlikely - and cover
// the reachable half: a reset with credit banked and a transfer outstanding
// owes the next session nothing. The invariant itself is held by reading the
// code, which is why usblink.c now states it as one.
TEST_CASE(a_reset_partway_through_a_session_owes_the_next_one_nothing)
{
  reset_all();

  for (int i = 0; i < 100; i++)
    ask_for_snapshot(); // a page asking far faster than it reads

  usblink_poll(&module);
  CHECK(tx.calls == 1); // one is out, and the endpoint is busy

  USBD_BMCV_VendorReset(); // the page goes away mid-transfer
  transfer_completes();    // ...and the stack reports the last one anyway

  for (int i = 0; i < 20; i++)
    usblink_poll(&module);
  CHECK(tx.calls == 1); // nothing further, however much credit was banked

  ask_for_snapshot();
  usblink_poll(&module);
  CHECK(tx.calls == 2); // and the new session's own request still works
}

TEST_CASE(a_new_session_does_not_inherit_a_held_button)
{
  RemoteInput taken;

  reset_all();
  send_mailbox(1, 1); // a button down when the page went away

  USBD_BMCV_VendorReset();
  CHECK(usblink_take_remote_input(&taken) == 0);
}

/* ---- the mailboxes ------------------------------------------------------- */

TEST_CASE(a_mailbox_is_handed_over_once)
{
  RemoteInput taken;

  reset_all();
  CHECK(usblink_take_remote_input(&taken) == 0);

  send_mailbox(42, 1);
  CHECK(usblink_take_remote_input(&taken) == 1);
  CHECK(taken.seq == 42);
  CHECK(taken.button_down[0] == 1);

  // Read and cleared. The input layer reads this every tick and a mailbox
  // handed over twice is a press the module keeps seeing arrive.
  CHECK(usblink_take_remote_input(&taken) == 0);
}

TEST_CASE(only_the_newest_mailbox_survives)
{
  RemoteInput taken;

  reset_all();
  send_mailbox(1, 1);
  send_mailbox(2, 0);

  // Levels, not events: two that arrive between ticks are not two gestures,
  // they are one state and one that is out of date.
  CHECK(usblink_take_remote_input(&taken) == 1);
  CHECK(taken.seq == 2);
  CHECK(taken.button_down[0] == 0);
}

TEST_CASE(a_command_is_handed_over_once)
{
  RemoteCommand taken;

  reset_all();
  CHECK(usblink_take_remote_command(&taken) == 0);

  send_command(3, REMOTE_OP_RESET_WIPE);
  CHECK(usblink_take_remote_command(&taken) == 1);
  CHECK(taken.op == REMOTE_OP_RESET_WIPE);
  CHECK(taken.seq == 3);
  CHECK(usblink_take_remote_command(&taken) == 0);
}

/* ---- what a host gets wrong ---------------------------------------------- */

TEST_CASE(a_message_shorter_than_its_payload_is_dropped)
{
  RemoteInput taken;
  uint8_t truncated[1 + sizeof(RemoteInput) - 1] = {USBLINK_OP_REMOTE_INPUT};

  reset_all();
  USBD_BMCV_VendorDataOut(truncated, sizeof truncated);

  // Copying what follows would read past what arrived. A host that got this
  // wrong is a host with a bug, and the answer is to ignore it rather than to
  // act on whatever was next in the buffer.
  CHECK(usblink_take_remote_input(&taken) == 0);
}

TEST_CASE(an_empty_transfer_is_ignored)
{
  uint8_t nothing = 0;

  reset_all();
  USBD_BMCV_VendorDataOut(&nothing, 0);

  usblink_poll(&module);
  CHECK(tx.calls == 0);
}

TEST_CASE(an_opcode_this_build_does_not_know_is_ignored_not_answered)
{
  uint8_t unknown[] = {0x7f, 1, 2, 3};

  reset_all();
  USBD_BMCV_VendorDataOut(unknown, sizeof unknown);

  usblink_poll(&module);
  CHECK(tx.calls == 0);
  CHECK(usblink_dfu_requested() == 0);
}

/* ---- the bootloader ------------------------------------------------------ */

TEST_CASE(a_dfu_request_latches_and_survives_a_new_session)
{
  uint8_t msg[] = {USBLINK_OP_ENTER_DFU};

  reset_all();
  CHECK(usblink_dfu_requested() == 0);

  USBD_BMCV_VendorDataOut(msg, sizeof msg);
  CHECK(usblink_dfu_requested() != 0);

  // Not cleared by a host arriving, unlike everything else here. The only
  // answer to it is fw_update_enter_dfu(), which does not return - and a host
  // that asked for the bootloader and then went away still asked.
  USBD_BMCV_VendorReset();
  CHECK(usblink_dfu_requested() != 0);
}

/* ---- what the module says it is ------------------------------------------ */

TEST_CASE(the_version_is_the_build_it_was_compiled_from)
{
  uint16_t len       = 0;
  const char* answer = USBD_BMCV_Version(&len);

  // Against tests/fakes/usb/version.h, which is why those three numbers are
  // not the project's: what is checked is that the string is assembled from
  // them, not what happens to be in VERSION today.
  CHECK(strcmp(answer, "7.8.9") == 0);

  // Including the terminator, because a host reads a NUL-terminated string out
  // of however many bytes the control transfer gave it.
  CHECK(len == strlen("7.8.9") + 1);
}

int main(void)
{
  RUN_TEST(nothing_goes_out_unasked);
  RUN_TEST(a_request_buys_exactly_one_snapshot);
  RUN_TEST(the_endpoint_carries_one_transfer_at_a_time);
  RUN_TEST(a_host_that_asks_faster_than_it_reads_cannot_bank_the_difference);
  RUN_TEST(a_refused_transfer_spends_its_credit_rather_than_owing_a_frame);
  RUN_TEST(a_session_that_ended_mid_transfer_does_not_wedge_the_next_one);
  RUN_TEST(a_new_session_does_not_inherit_the_last_ones_credit);
  RUN_TEST(a_reset_partway_through_a_session_owes_the_next_one_nothing);
  RUN_TEST(a_new_session_does_not_inherit_a_held_button);
  RUN_TEST(a_mailbox_is_handed_over_once);
  RUN_TEST(only_the_newest_mailbox_survives);
  RUN_TEST(a_command_is_handed_over_once);
  RUN_TEST(a_message_shorter_than_its_payload_is_dropped);
  RUN_TEST(an_empty_transfer_is_ignored);
  RUN_TEST(an_opcode_this_build_does_not_know_is_ignored_not_answered);
  RUN_TEST(a_dfu_request_latches_and_survives_a_new_session);
  RUN_TEST(the_version_is_the_build_it_was_compiled_from);
  return TESTKIT_SUMMARY();
}
