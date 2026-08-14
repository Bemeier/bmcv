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
//
// The length is checked per command rather than once for all of them: a command
// byte that arrived with the wrong amount after it is somebody else's message
// that happens to collide, not ours with a typo, and acting on it would be
// acting on a payload we cannot account for.
static SysexCmd sysex_classify(const SysexParser* p)
{
  if (p->overflow || p->len < 4)
    return SYSEX_CMD_NONE;

  if (p->buf[0] != SYSEX_ID_NONCOMMERCIAL || p->buf[1] != SYSEX_ID_B || p->buf[2] != SYSEX_ID_M)
    return SYSEX_CMD_NONE;

  const uint8_t payload = (uint8_t) (p->len - 4);

  switch (p->buf[3])
  {
  case SYSEX_CMD_ENTER_UPDATE:
    return payload == 0 ? SYSEX_CMD_ENTER_UPDATE : SYSEX_CMD_NONE;
  case SYSEX_CMD_IDENTITY_REQ:
    return payload == 0 ? SYSEX_CMD_IDENTITY_REQ : SYSEX_CMD_NONE;
  case SYSEX_CMD_BENCH_REQ:
    return payload == 0 ? SYSEX_CMD_BENCH_REQ : SYSEX_CMD_NONE;
  case SYSEX_CMD_STREAM_REQ:
    return payload == 0 ? SYSEX_CMD_STREAM_REQ : SYSEX_CMD_NONE;
  case SYSEX_CMD_REMOTE_INPUT:
    return payload == sysex7_encoded_len(SYSEX_REMOTE_INPUT_BYTES) ? SYSEX_CMD_REMOTE_INPUT : SYSEX_CMD_NONE;
  default:
    return SYSEX_CMD_NONE;
  }
}

static SysexCmd sysex_byte(SysexParser* p, uint8_t b, SysexHandler on_cmd, void* user)
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
    if (cmd != SYSEX_CMD_NONE && on_cmd)
    {
      // Handed over here, while the buffer still holds this message, rather
      // than left for the caller to fetch afterwards. A transfer can carry
      // several messages and the buffer only ever holds the last of them.
      on_cmd(cmd, p->buf + 4, (uint8_t) (p->len - 4), user);
    }
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

void sysex_feed(SysexParser* p, const uint8_t* packets, uint8_t len, SysexHandler on_cmd, void* user)
{
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
      sysex_byte(p, packets[i + 1 + k], on_cmd, user);
    }
  }
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

/* ---- seven-bit encoding -------------------------------------------------- */

// Seven raw bytes to a group, carried as eight: their high bits collected into
// a leading byte, then the seven with those bits cleared. A trailing partial
// group is the same shape with fewer of each, which is what makes the two
// length functions exact rather than approximate.

uint16_t sysex7_encoded_len(uint16_t raw_len) { return (uint16_t) (raw_len + (raw_len + 6) / 7); }

uint16_t sysex7_decoded_len(uint16_t enc_len) { return (uint16_t) (enc_len - (enc_len + 7) / 8); }

uint16_t sysex7_encode(uint8_t* out, const uint8_t* in, uint16_t len)
{
  uint16_t o = 0;

  for (uint16_t i = 0; i < len; i += 7)
  {
    const uint16_t n = (uint16_t) (len - i) < 7 ? (uint16_t) (len - i) : 7;

    uint8_t high = 0;
    for (uint16_t k = 0; k < n; k++)
    {
      if (in[i + k] & 0x80)
        high |= (uint8_t) (1u << k);
    }

    out[o++] = high;
    for (uint16_t k = 0; k < n; k++)
    {
      out[o++] = (uint8_t) (in[i + k] & 0x7F);
    }
  }

  return o;
}

uint16_t sysex7_decode(uint8_t* out, const uint8_t* in, uint16_t len)
{
  uint16_t o = 0;

  for (uint16_t i = 0; i < len; i += 8)
  {
    const uint16_t n = (uint16_t) (len - i) < 8 ? (uint16_t) (len - i) : 8;
    if (n < 2)
      break; // a lone high-bit byte with nothing to apply it to

    const uint8_t high = in[i];
    for (uint16_t k = 1; k < n; k++)
    {
      out[o++] = (uint8_t) (in[i + k] | ((high & (1u << (k - 1))) ? 0x80 : 0));
    }
  }

  return o;
}

/* ---- streaming a payload out --------------------------------------------- */

enum
{
  STREAM_HEADER,
  STREAM_BODY,
  STREAM_TRAILER,
  STREAM_DONE,
};

void sysex_stream_begin(SysexStream* s, uint8_t cmd, const uint8_t* raw, uint16_t raw_len)
{
  s->raw       = raw;
  s->raw_len   = raw_len;
  s->raw_pos   = 0;
  s->group_len = 0;
  s->group_pos = 0;
  s->stage     = STREAM_HEADER;
  s->hdr_pos   = 0;
  s->cmd       = cmd;
}

uint8_t sysex_stream_done(const SysexStream* s) { return s->stage == STREAM_DONE; }

// The next byte of the message body, F0 and F7 included. Returns 0 once there
// are none left, which the caller distinguishes by the stage rather than by the
// value - zero is a perfectly good payload byte.
static uint8_t stream_byte(SysexStream* s)
{
  static const uint8_t header[] = {0xF0, SYSEX_ID_NONCOMMERCIAL, SYSEX_ID_B, SYSEX_ID_M};

  switch (s->stage)
  {
  case STREAM_HEADER:
    if (s->hdr_pos < sizeof header)
      return header[s->hdr_pos++];

    s->stage = STREAM_BODY;
    return s->cmd;

  case STREAM_BODY:
    if (s->group_pos >= s->group_len)
    {
      if (s->raw_pos >= s->raw_len)
      {
        s->stage = STREAM_TRAILER;
        return 0xF7;
      }

      // Encode the next group on the way past, so nothing has to hold the whole
      // encoded message.
      const uint16_t n = (uint16_t) (s->raw_len - s->raw_pos) < 7 ? (uint16_t) (s->raw_len - s->raw_pos) : 7;
      s->group_len     = (uint8_t) sysex7_encode(s->group, s->raw + s->raw_pos, (uint16_t) n);
      s->group_pos     = 0;
      s->raw_pos       = (uint16_t) (s->raw_pos + n);
    }
    return s->group[s->group_pos++];

  default:
    return 0;
  }
}

uint8_t sysex_stream_next(SysexStream* s, uint8_t* out, uint8_t cable)
{
  if (s->stage == STREAM_DONE)
    return 0;

  const uint8_t hdr = (uint8_t) (cable << 4);
  uint8_t o         = 0;

  while (o + 4 <= SYSEX_STREAM_TRANSFER_BYTES && s->stage != STREAM_DONE)
  {
    uint8_t trio[3] = {0, 0, 0};
    uint8_t n       = 0;

    while (n < 3 && s->stage != STREAM_DONE)
    {
      trio[n++] = stream_byte(s);
      if (s->stage == STREAM_TRAILER)
      {
        s->stage = STREAM_DONE; // that byte was the F7, and nothing follows it
        break;
      }
    }

    if (n == 0)
      break;

    // Code index 0x4 while more is coming, and 0x5/0x6/0x7 on the packet that
    // carries the F7 - the count tells the host how many of the three are real.
    const uint8_t cin = (s->stage == STREAM_DONE) ? (n == 1 ? 0x5 : n == 2 ? 0x6 : 0x7) : 0x4;

    out[o++] = (uint8_t) (hdr | cin);
    out[o++] = trio[0];
    out[o++] = trio[1];
    out[o++] = trio[2];
  }

  return o;
}

uint8_t sysex_bench_message(uint8_t* out, uint8_t cable, uint16_t seq)
{
  uint8_t body[SYSEX_BENCH_MSG_BYTES];
  uint8_t n = 0;

  body[n++] = 0xF0;
  body[n++] = SYSEX_ID_NONCOMMERCIAL;
  body[n++] = SYSEX_ID_B;
  body[n++] = SYSEX_ID_M;
  body[n++] = SYSEX_CMD_BENCH_DATA;
  body[n++] = (uint8_t) (seq & 0x7F);
  body[n++] = (uint8_t) ((seq >> 7) & 0x7F);

  // Filler up to the last byte. Seven bits because everything between F0 and F7
  // has to be data, and varying with seq so that a run cannot come out fast
  // because something between here and the browser noticed it was sending the
  // same message over and over.
  while (n < SYSEX_BENCH_MSG_BYTES - 1)
  {
    body[n] = (uint8_t) ((seq + n) & 0x7F);
    n++;
  }
  body[n++] = 0xF7;

  // Three data bytes to a 4-byte event packet, which is why 48 divides into 16
  // packets with nothing left over. Every packet is "SysEx starts or continues"
  // except the last, which ends on its third byte.
  const uint8_t hdr = (uint8_t) (cable << 4);
  uint8_t i         = 0;

  for (uint8_t at = 0; at < SYSEX_BENCH_MSG_BYTES; at += 3)
  {
    out[i++] = (uint8_t) (hdr | (at + 3 >= SYSEX_BENCH_MSG_BYTES ? 0x7 : 0x4));
    out[i++] = body[at];
    out[i++] = body[at + 1];
    out[i++] = body[at + 2];
  }

  return i;
}
