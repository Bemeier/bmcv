#include "sysex.h"

void sysex_reset(SysexParser* p)
{
  p->len        = 0;
  p->in_message = false;
  p->overflow   = false;
}

static void sysex_push(SysexParser* p, uint8_t b)
{
  if (p->len < SYSEX_MAX_LEN)
  {
    p->buf[p->len++] = b;
  }
  else
  {
    p->overflow = true;
  }
}

// buf holds the message body, F0 and F7 stripped.
static SysexCmd sysex_classify(const SysexParser* p)
{
  if (p->overflow || p->len != 4)
    return SYSEX_CMD_NONE;

  if (p->buf[0] != SYSEX_ID_NONCOMMERCIAL || p->buf[1] != SYSEX_ID_B || p->buf[2] != SYSEX_ID_M)
    return SYSEX_CMD_NONE;

  switch (p->buf[3])
  {
  case SYSEX_CMD_ENTER_UPDATE:
    return SYSEX_CMD_ENTER_UPDATE;
  case SYSEX_CMD_IDENTITY_REQ:
    return SYSEX_CMD_IDENTITY_REQ;
  default:
    return SYSEX_CMD_NONE;
  }
}

static SysexCmd sysex_byte(SysexParser* p, uint8_t b)
{
  if (b == 0xF0)
  {
    // A second F0 without an F7 means the first message was truncated. Start
    // over on the new one rather than gluing the two together.
    p->in_message = true;
    p->len        = 0;
    p->overflow   = false;
    return SYSEX_CMD_NONE;
  }

  if (!p->in_message)
    return SYSEX_CMD_NONE;

  if (b == 0xF7)
  {
    SysexCmd cmd = sysex_classify(p);
    sysex_reset(p);
    return cmd;
  }

  if (b & 0x80)
  {
    // Any other status byte inside a SysEx aborts it. Real-time bytes are the
    // one thing MIDI allows to interleave, but USB-MIDI carries those in their
    // own packets (CIN 0xF), so they never reach here.
    sysex_reset(p);
    return SYSEX_CMD_NONE;
  }

  sysex_push(p, b);
  return SYSEX_CMD_NONE;
}

SysexCmd sysex_feed(SysexParser* p, const uint8_t* packets, uint8_t len)
{
  SysexCmd found = SYSEX_CMD_NONE;

  for (uint8_t i = 0; (uint16_t) i + 4u <= (uint16_t) len; i += 4)
  {
    uint8_t n;
    switch (packets[i] & 0x0F)
    {
    case 0x4:
      n = 3;
      break; // SysEx starts or continues
    case 0x5:
      n = 1;
      break; // ends with one byte
    case 0x6:
      n = 2;
      break; // ends with two
    case 0x7:
      n = 3;
      break; // ends with three
    default:
      // Not SysEx. This is also what skips the zero padding of a transfer that
      // was not filled: a zero header byte is code index 0, which is reserved.
      continue;
    }

    for (uint8_t k = 0; k < n; k++)
    {
      SysexCmd cmd = sysex_byte(p, packets[i + 1 + k]);
      if (cmd != SYSEX_CMD_NONE && found == SYSEX_CMD_NONE)
        found = cmd;
    }
  }

  return found;
}

uint8_t sysex_identity_reply(uint8_t* out, uint8_t cable, uint8_t major, uint8_t minor, uint8_t patch)
{
  const uint8_t hdr = (uint8_t) (cable << 4);
  uint8_t i         = 0;

  out[i++] = hdr | 0x4;
  out[i++] = 0xF0;
  out[i++] = SYSEX_ID_NONCOMMERCIAL;
  out[i++] = SYSEX_ID_B;

  out[i++] = hdr | 0x4;
  out[i++] = SYSEX_ID_M;
  out[i++] = SYSEX_CMD_IDENTITY_REQ;
  out[i++] = (uint8_t) (major & 0x7F);

  out[i++] = hdr | 0x7;
  out[i++] = (uint8_t) (minor & 0x7F);
  out[i++] = (uint8_t) (patch & 0x7F);
  out[i++] = 0xF7;

  return i;
}
