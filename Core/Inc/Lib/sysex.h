#ifndef INC_LIB_SYSEX_H_
#define INC_LIB_SYSEX_H_

#include <stdbool.h>
#include <stdint.h>

// The module's one control channel that is not a knob: SysEx arriving on the
// USB MIDI endpoint the firmware already enumerates.
//
// It exists because there is no reset button on the panel - NRST only reaches
// the programming header - so without a way to ask the running firmware to
// reboot itself, the only route into the ROM DFU bootloader is to hold FN2 and
// power-cycle the case. The browser update page sends ENTER_UPDATE instead.
//
// Hardware-free on purpose. This is a byte parser, and the tests feed it
// packets no USB stack was involved in producing.

// Messages are F0 7D 42 4D <cmd> F7. 0x7D is the manufacturer ID reserved for
// non-commercial use, which is what this is; it is shared with every other
// hobby project, so 'B','M' follows it to keep us from answering somebody
// else's traffic.
#define SYSEX_ID_NONCOMMERCIAL 0x7D
#define SYSEX_ID_B 0x42
#define SYSEX_ID_M 0x4D

// Long enough for the longest message the module accepts - the remote input
// mailbox, at 3 ID bytes + 1 command + 55 encoded - and nothing more. A longer
// SysEx is somebody else's, and is skipped without buffering the rest of it.
#define SYSEX_MAX_LEN 64

typedef enum
{
  SYSEX_CMD_NONE = 0,

  // Hand the module to the ROM DFU bootloader. Acted on from the main loop,
  // never from the USB interrupt this parser runs in.
  SYSEX_CMD_ENTER_UPDATE = 0x01,

  // Reply with the running firmware version. Display only - see the note on
  // sysex_identity_reply().
  SYSEX_CMD_IDENTITY_REQ = 0x02,

  // SPIKE, not a feature. Send a fixed burst of SYSEX_BENCH_MESSAGES packets as
  // fast as the endpoint will take them, so a browser can time them and say what
  // this transport is actually worth. See docs/plans/midi-transport.md.
  //
  // The answer decides whether the module can publish its own state over its
  // USB port instead of over a debug probe, and it is the one number the whole
  // question hangs on. Everything else about that design is arithmetic; this is
  // measurement.
  SYSEX_CMD_BENCH_REQ = 0x10,

  // What the burst is made of. Never sent unless asked for.
  SYSEX_CMD_BENCH_DATA = 0x11,

  // Host -> module. The remote input mailbox, 48 raw bytes seven-bit encoded to
  // 55. Merged by input_fold exactly as one written over a debug probe is; see
  // RemoteInput in input_fold.h. This is the whole of the write direction.
  SYSEX_CMD_REMOTE_INPUT = 0x20,

  // Host -> module. Keep sending snapshots. Deliberately a repeated request
  // rather than an on/off pair, for the reason RemoteInput.seq is a heartbeat:
  // a browser tab that goes away must not leave the module streaming at nobody.
  // The module stops SYSEX_STREAM_TIMEOUT_US after the last one.
  SYSEX_CMD_STREAM_REQ = 0x21,

  // Module -> host. One whole BmcvInstance, seven-bit encoded. Sent only while
  // stream requests keep arriving.
  SYSEX_CMD_SNAPSHOT = 0x22,

  // Host -> module. A RemoteCommand: reset, or reset and forget storage. Eight
  // raw bytes, seven-bit encoded to ten. Sent when asked for rather than
  // continuously - see RemoteCommand in instance.h for why it is not part of
  // the input mailbox.
  SYSEX_CMD_REMOTE_COMMAND = 0x23,
} SysexCmd;

typedef struct
{
  uint8_t buf[SYSEX_MAX_LEN];
  uint8_t len;
  bool in_message; // between F0 and F7
  bool overflow;   // this message outgrew buf; drop it, keep skipping to F7
} SysexParser;

void sysex_reset(SysexParser* p);

// Called once for each complete message recognised, as it completes, with its
// payload - everything after the three ID bytes and the command - still intact.
typedef void (*SysexHandler)(SysexCmd cmd, const uint8_t* payload, uint8_t len, void* user);

// Feed one USB MIDI OUT transfer: `len` bytes of 4-byte USB-MIDI event packets.
// A message split across two transfers completes on the second.
//
// Every message in the transfer is reported, not just the first. That used to
// be "returns the first command recognised", which was survivable while the
// only commands were a reboot and a version request - both rare, both sent
// alone. It stopped being survivable the moment a host sent a 61-byte remote
// input mailbox (84 packet bytes, so always spanning two transfers) alongside
// small requests: a host stack is free to pack whatever it has into one
// transfer, and the second command in it simply vanished. What that looked like
// was a module going quiet a few times a second for no visible reason.
void sysex_feed(SysexParser* p, const uint8_t* packets, uint8_t len, SysexHandler on_cmd, void* user);

// Bytes sysex_identity_reply() writes: F0 7D 42 4D 02 <major> <minor> <patch>
// F7 is nine bytes, which packs into three USB-MIDI event packets.
#define SYSEX_IDENTITY_REPLY_LEN 12

// Build the reply to SYSEX_CMD_IDENTITY_REQ as USB-MIDI event packets, ready
// to hand to the endpoint. Returns bytes written; `out` needs
// SYSEX_IDENTITY_REPLY_LEN of room.
//
// The version this carries is for the update page to show next to the image it
// is about to write. Nothing enforces it: any build is flashable over any
// other, and the ROM bootloader would not honour a policy anyway.
uint8_t sysex_identity_reply(uint8_t* out, uint8_t cable, uint8_t major, uint8_t minor, uint8_t patch);

// How many snapshots the module will run ahead of the host's requests.
//
// One request buys one snapshot, so the host paces the module rather than the
// other way round - a module that streams flat out sends faster than a browser
// can decode and draw, and what that produces is not a higher frame rate but a
// main thread that never catches up and a rate that swings with whatever else
// the page is doing. The debug probe never had this problem because it is
// polled: a read is asked for, finished, and only then asked for again.
//
// More than one, so the module can begin the next snapshot while the host is
// still working on the last and the USB does not idle for a round trip. Not
// many more, or the pacing is lost again and stale frames queue up behind the
// live one.
//
// This also replaces the timeout that used to stop a stream: a host that goes
// away stops asking, credit runs out, and the module falls silent on its own.
#define SYSEX_STREAM_MAX_CREDITS 2

// The remote input mailbox, in raw bytes. Held to sizeof(RemoteInput) by a
// static assert in midi.c, which sees both - this header stays clear of
// firmware types so its tests keep running without them.
#define SYSEX_REMOTE_INPUT_BYTES 48

// The remote command mailbox, in raw bytes. Held to sizeof(RemoteCommand) by a
// static assert in midi.c, for the same reason as above.
#define SYSEX_REMOTE_COMMAND_BYTES 8

/* ---- seven-bit encoding -------------------------------------------------- */
//
// Nothing between F0 and F7 may have its high bit set, so arbitrary bytes have
// to be carried seven bits at a time. Seven data bytes become eight: a byte of
// their high bits, then the seven with those bits cleared.
//
// Here rather than in the firmware or the frontend because both ends need it
// and there must be one of it. The browser reaches it through the wasm - see
// bmcv_sim_sysex7_* - so a snapshot is encoded and decoded by the same compiled
// source, the same bargain bmcv_sim_import() already makes for the layout.

// Round trip: sysex7_decoded_len(sysex7_encoded_len(n)) == n.
uint16_t sysex7_encoded_len(uint16_t raw_len);
uint16_t sysex7_decoded_len(uint16_t enc_len);

// Return bytes written. `out` needs the matching *_len() of room, and may not
// overlap `in`.
uint16_t sysex7_encode(uint8_t* out, const uint8_t* in, uint16_t len);
uint16_t sysex7_decode(uint8_t* out, const uint8_t* in, uint16_t len);

/* ---- streaming a payload out --------------------------------------------- */
//
// A snapshot is 2368 bytes, which is 2707 encoded and 57 transfers of the
// 64-byte endpoint. Too big to hand over in one go, so it goes out a transfer
// at a time from the main loop, whenever the endpoint is free.
//
// The encoding happens as it goes rather than into a second buffer: only the
// source has to stay put, which is why this holds a pointer rather than a copy.

typedef struct
{
  const uint8_t* raw;
  uint16_t raw_len;
  uint16_t raw_pos;

  uint8_t group[8]; // one encoded group, being handed out
  uint8_t group_len;
  uint8_t group_pos;

  uint8_t stage; // header, body, trailer, done
  uint8_t hdr_pos;
  uint8_t cmd;
} SysexStream;

// `raw` must outlive the stream and must not change while it runs - a snapshot
// taken at a tick boundary, not the live instance.
void sysex_stream_begin(SysexStream* s, uint8_t cmd, const uint8_t* raw, uint16_t raw_len);

// Fill one transfer: up to SYSEX_STREAM_TRANSFER_BYTES of USB-MIDI event
// packets. Returns bytes written, or 0 when the message is finished.
#define SYSEX_STREAM_TRANSFER_BYTES 64
uint8_t sysex_stream_next(SysexStream* s, uint8_t* out, uint8_t cable);

uint8_t sysex_stream_done(const SysexStream* s);

/* ---- the throughput spike ------------------------------------------------ */

// How many messages one request sends. Bounded rather than a start/stop pair on
// purpose: a browser tab that goes away mid-run cannot leave the module
// streaming at a host that is not listening, which would need a power cycle to
// stop. Ask again for a longer run.
#define SYSEX_BENCH_MESSAGES 2000

// One bench message is 48 MIDI bytes, which is exactly 16 USB-MIDI event
// packets, which is exactly one 64-byte transfer on the endpoint. That
// alignment is the point: the browser counts whole messages, and each one it
// counts is one transfer the hardware actually did, so bytes/second on the wire
// needs no assumption about how the stack packed anything.
//
//   F0 7D 42 4D 11 <seq lo7> <seq hi7> <40 payload> F7
#define SYSEX_BENCH_MSG_BYTES 48
#define SYSEX_BENCH_PACKET_BYTES 64

// Build message `seq` of a burst. `out` needs SYSEX_BENCH_PACKET_BYTES.
// Returns bytes written, always SYSEX_BENCH_PACKET_BYTES.
//
// The payload varies with seq so a run cannot be measured as fast because
// something downstream coalesced or dropped identical messages.
uint8_t sysex_bench_message(uint8_t* out, uint8_t cable, uint16_t seq);

#endif /* INC_LIB_SYSEX_H_ */
