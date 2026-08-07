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

// Long enough for the messages above and nothing more. A longer SysEx is
// somebody else's, and is skipped without buffering the rest of it.
#define SYSEX_MAX_LEN 16

typedef enum
{
  SYSEX_CMD_NONE = 0,

  // Hand the module to the ROM DFU bootloader. Acted on from the main loop,
  // never from the USB interrupt this parser runs in.
  SYSEX_CMD_ENTER_UPDATE = 0x01,

  // Reply with the running firmware version. Display only - see the note on
  // sysex_identity_reply().
  SYSEX_CMD_IDENTITY_REQ = 0x02,
} SysexCmd;

typedef struct
{
  uint8_t buf[SYSEX_MAX_LEN];
  uint8_t len;
  bool in_message; // between F0 and F7
  bool overflow;   // this message outgrew buf; drop it, keep skipping to F7
} SysexParser;

void sysex_reset(SysexParser* p);

// Feed one USB MIDI OUT transfer: `len` bytes of 4-byte USB-MIDI event packets.
// Returns the first command recognised in this transfer, or SYSEX_CMD_NONE.
// The whole buffer is parsed either way, so a message split across two
// transfers still completes on the second.
SysexCmd sysex_feed(SysexParser* p, const uint8_t* packets, uint8_t len);

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

#endif /* INC_LIB_SYSEX_H_ */
