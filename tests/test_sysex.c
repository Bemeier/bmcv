// The SysEx control channel. These matter more than their size suggests: the
// only command here reboots the module into a bootloader it cannot get back
// from on its own, so a parser that fires on somebody else's traffic is a
// module that drops out of a patch mid-set.

#include "sysex.h"
#include "testkit.h"

#include <string.h>

// Pack a raw SysEx message into USB-MIDI event packets the way a host does:
// three bytes per packet with code index 0x4, and a final packet whose code
// index says how many bytes of the last group are real.
static uint8_t pack(uint8_t* out, const uint8_t* msg, uint8_t n, uint8_t cable)
{
  const uint8_t hdr = (uint8_t) (cable << 4);
  uint8_t o = 0, i = 0;

  while ((uint8_t) (n - i) > 3)
  {
    out[o++] = hdr | 0x4;
    out[o++] = msg[i++];
    out[o++] = msg[i++];
    out[o++] = msg[i++];
  }

  const uint8_t rem = (uint8_t) (n - i);
  out[o++]          = hdr | (rem == 1 ? 0x5 : rem == 2 ? 0x6 : 0x7);
  out[o++]          = msg[i++];
  out[o++]          = rem >= 2 ? msg[i++] : 0;
  out[o++]          = rem >= 3 ? msg[i++] : 0;

  return o;
}

static SysexCmd feed_message(SysexParser* p, const uint8_t* msg, uint8_t n)
{
  // Four packet bytes per three of message, and the longest one here is the
  // remote input mailbox at 61 bytes - so 64 was enough only while every
  // message was six bytes long.
  uint8_t packets[128];
  memset(packets, 0, sizeof packets);
  uint8_t len = pack(packets, msg, n, 0);
  return sysex_feed(p, packets, len);
}

static const uint8_t enter_update[] = {0xF0, 0x7D, 0x42, 0x4D, 0x01, 0xF7};
static const uint8_t identity_req[] = {0xF0, 0x7D, 0x42, 0x4D, 0x02, 0xF7};

TEST_CASE(recognises_the_two_commands)
{
  SysexParser p;
  sysex_reset(&p);
  CHECK(feed_message(&p, enter_update, sizeof enter_update) == SYSEX_CMD_ENTER_UPDATE);

  sysex_reset(&p);
  CHECK(feed_message(&p, identity_req, sizeof identity_req) == SYSEX_CMD_IDENTITY_REQ);
}

TEST_CASE(ignores_other_peoples_sysex)
{
  SysexParser p;

  // Right shape, wrong manufacturer.
  const uint8_t other_vendor[] = {0xF0, 0x43, 0x42, 0x4D, 0x01, 0xF7};
  sysex_reset(&p);
  CHECK(feed_message(&p, other_vendor, sizeof other_vendor) == SYSEX_CMD_NONE);

  // The non-commercial ID is shared with every other hobby project, so the
  // 'B','M' after it is what actually keeps us out of their traffic.
  const uint8_t same_id_other_project[] = {0xF0, 0x7D, 0x11, 0x22, 0x01, 0xF7};
  sysex_reset(&p);
  CHECK(feed_message(&p, same_id_other_project, sizeof same_id_other_project) == SYSEX_CMD_NONE);

  // Ours, but a command from a future firmware this build knows nothing about.
  const uint8_t unknown_cmd[] = {0xF0, 0x7D, 0x42, 0x4D, 0x7E, 0xF7};
  sysex_reset(&p);
  CHECK(feed_message(&p, unknown_cmd, sizeof unknown_cmd) == SYSEX_CMD_NONE);

  // Ours, right command byte, but trailing junk makes it a different message.
  const uint8_t too_long[] = {0xF0, 0x7D, 0x42, 0x4D, 0x01, 0x00, 0xF7};
  sysex_reset(&p);
  CHECK(feed_message(&p, too_long, sizeof too_long) == SYSEX_CMD_NONE);
}

TEST_CASE(ignores_ordinary_midi_traffic)
{
  SysexParser p;
  sysex_reset(&p);

  // Note on, note off, control change, pitch bend - the packets a DAW pointed
  // at this port would actually send.
  const uint8_t traffic[] = {
      0x09, 0x90, 0x40, 0x7F, // note on
      0x08, 0x80, 0x40, 0x00, // note off
      0x0B, 0xB0, 0x07, 0x64, // control change
      0x0E, 0xE0, 0x00, 0x40, // pitch bend
  };
  CHECK(sysex_feed(&p, traffic, sizeof traffic) == SYSEX_CMD_NONE);

  // And the channel is still usable afterwards.
  CHECK(feed_message(&p, enter_update, sizeof enter_update) == SYSEX_CMD_ENTER_UPDATE);
}

TEST_CASE(zero_padding_is_not_a_message)
{
  SysexParser p;
  sysex_reset(&p);

  // The class hands over the whole 64-byte buffer every time, padded with
  // zeroes. Code index 0 is reserved, so those packets have to be skipped.
  uint8_t packets[64];
  memset(packets, 0, sizeof packets);
  CHECK(sysex_feed(&p, packets, sizeof packets) == SYSEX_CMD_NONE);

  uint8_t len = pack(packets, enter_update, sizeof enter_update, 0);
  (void) len;
  CHECK(sysex_feed(&p, packets, sizeof packets) == SYSEX_CMD_ENTER_UPDATE);
}

TEST_CASE(a_message_split_across_transfers_still_completes)
{
  SysexParser p;
  sysex_reset(&p);

  uint8_t packets[8];
  memset(packets, 0, sizeof packets);
  pack(packets, enter_update, sizeof enter_update, 0);

  // First packet alone carries F0 7D 42 - nothing to act on yet.
  CHECK(sysex_feed(&p, packets, 4) == SYSEX_CMD_NONE);
  // Second completes it.
  CHECK(sysex_feed(&p, packets + 4, 4) == SYSEX_CMD_ENTER_UPDATE);
}

TEST_CASE(a_truncated_message_does_not_wedge_the_parser)
{
  SysexParser p;
  sysex_reset(&p);

  // A start with no end, then a fresh message. The two must not be glued into
  // one, and the second must still be recognised.
  uint8_t opener[4] = {0x04, 0xF0, 0x7D, 0x42};
  CHECK(sysex_feed(&p, opener, sizeof opener) == SYSEX_CMD_NONE);
  CHECK(feed_message(&p, enter_update, sizeof enter_update) == SYSEX_CMD_ENTER_UPDATE);
}

TEST_CASE(an_oversized_sysex_is_dropped_without_blocking_the_next_one)
{
  SysexParser p;
  sysex_reset(&p);

  // Longer than the parser's buffer, so it cannot be held - but it must be
  // skipped to its F7 rather than leaving the parser mid-message.
  uint8_t big[SYSEX_MAX_LEN + 8];
  big[0] = 0xF0;
  for (size_t i = 1; i < sizeof big - 1; i++)
  {
    big[i] = 0x01;
  }
  big[sizeof big - 1] = 0xF7;

  CHECK(feed_message(&p, big, (uint8_t) sizeof big) == SYSEX_CMD_NONE);
  CHECK(feed_message(&p, enter_update, sizeof enter_update) == SYSEX_CMD_ENTER_UPDATE);
}

TEST_CASE(a_status_byte_inside_a_message_aborts_it)
{
  SysexParser p;
  sysex_reset(&p);

  const uint8_t interrupted[] = {0xF0, 0x7D, 0x42, 0x4D, 0x80, 0x01, 0xF7};
  CHECK(feed_message(&p, interrupted, sizeof interrupted) == SYSEX_CMD_NONE);
  CHECK(feed_message(&p, enter_update, sizeof enter_update) == SYSEX_CMD_ENTER_UPDATE);
}

TEST_CASE(identity_reply_is_a_well_formed_sysex)
{
  uint8_t out[SYSEX_IDENTITY_REPLY_LEN];
  memset(out, 0xAA, sizeof out);

  CHECK(sysex_identity_reply(out, 0, 1, 2, 3) == SYSEX_IDENTITY_REPLY_LEN);

  // Three packets: two continuations and a three-byte end.
  CHECK(out[0] == 0x04);
  CHECK(out[4] == 0x04);
  CHECK(out[8] == 0x07);

  const uint8_t body[] = {out[1], out[2], out[3], out[5], out[6], out[7], out[9], out[10], out[11]};
  const uint8_t want[] = {0xF0, 0x7D, 0x42, 0x4D, 0x02, 1, 2, 3, 0xF7};
  CHECK(memcmp(body, want, sizeof want) == 0);
}

TEST_CASE(identity_reply_honours_the_cable_number)
{
  uint8_t out[SYSEX_IDENTITY_REPLY_LEN];
  sysex_identity_reply(out, 3, 0, 1, 0);
  CHECK(out[0] == 0x34);
  CHECK(out[4] == 0x34);
  CHECK(out[8] == 0x37);
}

/* ---- seven-bit encoding -------------------------------------------------- */

TEST_CASE(seven_bit_lengths_are_exact)
{
  // Exact rather than approximate, because the parser uses the encoded length
  // to decide whether a message is ours - a length that was merely close would
  // reject a good mailbox or accept a malformed one.
  CHECK(sysex7_encoded_len(0) == 0);
  CHECK(sysex7_encoded_len(1) == 2);
  CHECK(sysex7_encoded_len(7) == 8);
  CHECK(sysex7_encoded_len(8) == 10);
  CHECK(sysex7_encoded_len(48) == 55);     // the remote input mailbox
  CHECK(sysex7_encoded_len(2368) == 2707); // a whole instance

  for (uint16_t n = 0; n < 600; n++)
  {
    CHECK(sysex7_decoded_len(sysex7_encoded_len(n)) == n);
  }
}

TEST_CASE(seven_bit_round_trips_every_byte_value)
{
  uint8_t raw[256], enc[512], back[256];
  for (uint16_t i = 0; i < 256; i++)
  {
    raw[i] = (uint8_t) i;
  }

  const uint16_t n = sysex7_encode(enc, raw, 256);
  CHECK(n == sysex7_encoded_len(256));

  // The whole point: nothing that goes out may have its high bit set, or it is
  // not a SysEx and a host discards the message rather than decoding it.
  for (uint16_t i = 0; i < n; i++)
  {
    CHECK((enc[i] & 0x80) == 0);
  }

  CHECK(sysex7_decode(back, enc, n) == 256);
  CHECK(memcmp(raw, back, 256) == 0);
}

TEST_CASE(seven_bit_round_trips_awkward_lengths)
{
  // A partial trailing group is where an off-by-one lives.
  uint8_t raw[32], enc[64], back[32];
  for (uint16_t i = 0; i < sizeof raw; i++)
  {
    raw[i] = (uint8_t) (0x80 | i); // high bit set throughout, the harder case
  }

  for (uint16_t len = 0; len <= sizeof raw; len++)
  {
    const uint16_t n = sysex7_encode(enc, raw, len);
    CHECK(n == sysex7_encoded_len(len));
    CHECK(sysex7_decode(back, enc, n) == len);
    CHECK(memcmp(raw, back, len) == 0);
  }
}

/* ---- streaming ----------------------------------------------------------- */

// Reassemble a streamed message the way a host's MIDI stack does, and check it
// comes back byte for byte. This is the read direction end to end, short of the
// USB itself.
TEST_CASE(a_streamed_payload_round_trips)
{
  uint8_t raw[300];
  for (uint16_t i = 0; i < sizeof raw; i++)
  {
    raw[i] = (uint8_t) (i * 7 + (i >> 3));
  }

  SysexStream s;
  sysex_stream_begin(&s, SYSEX_CMD_SNAPSHOT, raw, sizeof raw);

  uint8_t body[1024];
  uint16_t body_len  = 0;
  uint16_t transfers = 0;

  for (;;)
  {
    uint8_t packets[SYSEX_STREAM_TRANSFER_BYTES];
    const uint8_t n = sysex_stream_next(&s, packets, 0);
    if (n == 0)
      break;

    CHECK(n % 4 == 0);
    CHECK(n <= SYSEX_STREAM_TRANSFER_BYTES);
    transfers++;

    for (uint8_t i = 0; i + 4 <= n; i += 4)
    {
      uint8_t take;
      switch (packets[i] & 0x0F)
      {
      case 0x4:
        take = 3;
        break;
      case 0x5:
        take = 1;
        break;
      case 0x6:
        take = 2;
        break;
      case 0x7:
        take = 3;
        break;
      default:
        take = 0;
        break;
      }
      for (uint8_t k = 0; k < take; k++)
      {
        body[body_len++] = packets[i + 1 + k];
      }
    }
  }

  CHECK(sysex_stream_done(&s));
  CHECK(transfers > 1); // it really did span several

  // F0 7D 42 4D <cmd> <encoded> F7
  CHECK(body[0] == 0xF0);
  CHECK(body[1] == SYSEX_ID_NONCOMMERCIAL);
  CHECK(body[2] == SYSEX_ID_B);
  CHECK(body[3] == SYSEX_ID_M);
  CHECK(body[4] == SYSEX_CMD_SNAPSHOT);
  CHECK(body[body_len - 1] == 0xF7);
  CHECK(body_len == 6 + sysex7_encoded_len(sizeof raw));

  for (uint16_t i = 1; i < body_len - 1; i++)
  {
    CHECK((body[i] & 0x80) == 0);
  }

  uint8_t back[sizeof raw];
  const uint16_t got = sysex7_decode(back, body + 5, (uint16_t) (body_len - 6));
  CHECK(got == sizeof raw);
  CHECK(memcmp(raw, back, sizeof raw) == 0);
}

// The size that matters, and the number the throughput measurement was read
// against: a whole instance has to be 57 transfers, or the snapshots-per-second
// figure on the bench page means something else.
TEST_CASE(an_instance_sized_payload_is_57_transfers)
{
  static uint8_t raw[2368];
  SysexStream s;
  sysex_stream_begin(&s, SYSEX_CMD_SNAPSHOT, raw, sizeof raw);

  uint16_t transfers = 0;
  uint8_t packets[SYSEX_STREAM_TRANSFER_BYTES];
  while (sysex_stream_next(&s, packets, 0) != 0)
  {
    transfers++;
  }

  CHECK(transfers == 57);
}

TEST_CASE(a_stream_honours_the_cable_number)
{
  uint8_t raw[16] = {0};
  SysexStream s;
  sysex_stream_begin(&s, SYSEX_CMD_SNAPSHOT, raw, sizeof raw);

  uint8_t packets[SYSEX_STREAM_TRANSFER_BYTES];
  sysex_stream_next(&s, packets, 5);
  CHECK((packets[0] >> 4) == 5);
}

/* ---- the remote input mailbox, inbound ----------------------------------- */

TEST_CASE(a_remote_input_message_is_recognised_and_carries_its_payload)
{
  uint8_t mailbox[SYSEX_REMOTE_INPUT_BYTES];
  for (uint16_t i = 0; i < sizeof mailbox; i++)
  {
    mailbox[i] = (uint8_t) (0x80 | (i * 3));
  }

  uint8_t msg[128];
  uint8_t n = 0;
  msg[n++]  = 0xF0;
  msg[n++]  = SYSEX_ID_NONCOMMERCIAL;
  msg[n++]  = SYSEX_ID_B;
  msg[n++]  = SYSEX_ID_M;
  msg[n++]  = SYSEX_CMD_REMOTE_INPUT;
  n += (uint8_t) sysex7_encode(msg + n, mailbox, sizeof mailbox);
  msg[n++] = 0xF7;

  SysexParser p;
  sysex_reset(&p);
  CHECK(feed_message(&p, msg, n) == SYSEX_CMD_REMOTE_INPUT);

  uint8_t len;
  const uint8_t* payload = sysex_payload(&p, &len);
  CHECK(len == sysex7_encoded_len(SYSEX_REMOTE_INPUT_BYTES));

  uint8_t back[SYSEX_REMOTE_INPUT_BYTES];
  CHECK(sysex7_decode(back, payload, len) == sizeof mailbox);
  CHECK(memcmp(mailbox, back, sizeof mailbox) == 0);
}

// A payload of the wrong size is somebody else's message that happens to
// collide, not ours with a typo - and acting on it would mean copying an
// unaccountable number of bytes into the input layer.
TEST_CASE(a_remote_input_message_of_the_wrong_size_is_refused)
{
  SysexParser p;

  uint8_t msg[128];
  for (uint8_t extra = 1; extra <= 3; extra++)
  {
    uint8_t n = 0;
    msg[n++]  = 0xF0;
    msg[n++]  = SYSEX_ID_NONCOMMERCIAL;
    msg[n++]  = SYSEX_ID_B;
    msg[n++]  = SYSEX_ID_M;
    msg[n++]  = SYSEX_CMD_REMOTE_INPUT;
    for (uint8_t i = 0; i < sysex7_encoded_len(SYSEX_REMOTE_INPUT_BYTES) + extra; i++)
    {
      msg[n++] = 0x01;
    }
    msg[n++] = 0xF7;

    sysex_reset(&p);
    CHECK(feed_message(&p, msg, n) == SYSEX_CMD_NONE);
  }

  // And short.
  uint8_t n = 0;
  msg[n++]  = 0xF0;
  msg[n++]  = SYSEX_ID_NONCOMMERCIAL;
  msg[n++]  = SYSEX_ID_B;
  msg[n++]  = SYSEX_ID_M;
  msg[n++]  = SYSEX_CMD_REMOTE_INPUT;
  msg[n++]  = 0xF7;
  sysex_reset(&p);
  CHECK(feed_message(&p, msg, n) == SYSEX_CMD_NONE);
}

// The commands that carry nothing must still reject a payload, or a longer
// message from another project that opens the same way could reboot the module.
TEST_CASE(a_bare_command_with_a_payload_is_refused)
{
  SysexParser p;
  sysex_reset(&p);

  const uint8_t trailing[] = {0xF0, 0x7D, 0x42, 0x4D, SYSEX_CMD_ENTER_UPDATE, 0x00, 0xF7};
  CHECK(feed_message(&p, trailing, sizeof trailing) == SYSEX_CMD_NONE);
}

/* ---- the throughput spike ------------------------------------------------ */

// The measurement is only worth what the packing is. If a bench message were
// not exactly one transfer, the browser would still count messages and the
// bytes-per-second it reported would be wrong by whatever the mismatch was -
// which is the one way this experiment could produce a confident wrong answer.
TEST_CASE(a_bench_message_is_exactly_one_transfer)
{
  uint8_t out[SYSEX_BENCH_PACKET_BYTES];
  CHECK(sysex_bench_message(out, 0, 0) == SYSEX_BENCH_PACKET_BYTES);
  CHECK(SYSEX_BENCH_MSG_BYTES % 3 == 0);
  CHECK(SYSEX_BENCH_PACKET_BYTES == (SYSEX_BENCH_MSG_BYTES / 3) * 4);
}

TEST_CASE(a_bench_message_is_a_well_formed_sysex)
{
  uint8_t out[SYSEX_BENCH_PACKET_BYTES];
  sysex_bench_message(out, 0, 0x1234);

  // Every packet carries three data bytes; only the last one ends the message.
  for (uint8_t i = 0; i + 4 <= SYSEX_BENCH_PACKET_BYTES; i += 4)
  {
    const uint8_t last = (i + 4 == SYSEX_BENCH_PACKET_BYTES);
    CHECK((out[i] & 0x0F) == (last ? 0x7 : 0x4));
  }

  CHECK(out[1] == 0xF0);
  CHECK(out[2] == SYSEX_ID_NONCOMMERCIAL);
  CHECK(out[3] == SYSEX_ID_B);
  CHECK(out[5] == SYSEX_ID_M);
  CHECK(out[6] == SYSEX_CMD_BENCH_DATA);
  CHECK(out[SYSEX_BENCH_PACKET_BYTES - 1] == 0xF7);

  // The sequence number, so a browser can tell a slow run from a lossy one.
  CHECK(out[7] == (0x1234 & 0x7F));
  CHECK(out[9] == ((0x1234 >> 7) & 0x7F));

  // Nothing between F0 and F7 may have its high bit set, or the message is not
  // a SysEx and a host will discard the burst rather than count it.
  for (uint8_t i = 0; i + 4 <= SYSEX_BENCH_PACKET_BYTES; i += 4)
  {
    for (uint8_t k = 1; k < 4; k++)
    {
      const uint8_t at = (uint8_t) (i + k);
      if (at != 1 && at != SYSEX_BENCH_PACKET_BYTES - 1)
        CHECK((out[at] & 0x80) == 0);
    }
  }
}

// It has to round-trip through the module's own parser, since that is what a
// second module - or a loopback - would see.
TEST_CASE(a_bench_request_is_recognised)
{
  SysexParser p;
  sysex_reset(&p);

  const uint8_t req[] = {
      0x04, 0xF0, SYSEX_ID_NONCOMMERCIAL, SYSEX_ID_B, 0x07, SYSEX_ID_M, SYSEX_CMD_BENCH_REQ, 0xF7,
  };
  CHECK(sysex_feed(&p, req, sizeof(req)) == SYSEX_CMD_BENCH_REQ);
}

// The burst is bounded so a browser tab that goes away cannot leave the module
// streaming at nobody, which would need a power cycle to stop.
TEST_CASE(a_burst_is_bounded)
{
  CHECK(SYSEX_BENCH_MESSAGES > 0);
  CHECK(SYSEX_BENCH_MESSAGES <= UINT16_MAX);
}

int main(void)
{
  RUN_TEST(recognises_the_two_commands);
  RUN_TEST(ignores_other_peoples_sysex);
  RUN_TEST(ignores_ordinary_midi_traffic);
  RUN_TEST(zero_padding_is_not_a_message);
  RUN_TEST(a_message_split_across_transfers_still_completes);
  RUN_TEST(a_truncated_message_does_not_wedge_the_parser);
  RUN_TEST(an_oversized_sysex_is_dropped_without_blocking_the_next_one);
  RUN_TEST(a_status_byte_inside_a_message_aborts_it);
  RUN_TEST(identity_reply_is_a_well_formed_sysex);
  RUN_TEST(identity_reply_honours_the_cable_number);
  RUN_TEST(seven_bit_lengths_are_exact);
  RUN_TEST(seven_bit_round_trips_every_byte_value);
  RUN_TEST(seven_bit_round_trips_awkward_lengths);
  RUN_TEST(a_streamed_payload_round_trips);
  RUN_TEST(an_instance_sized_payload_is_57_transfers);
  RUN_TEST(a_stream_honours_the_cable_number);
  RUN_TEST(a_remote_input_message_is_recognised_and_carries_its_payload);
  RUN_TEST(a_remote_input_message_of_the_wrong_size_is_refused);
  RUN_TEST(a_bare_command_with_a_payload_is_refused);
  RUN_TEST(a_bench_message_is_exactly_one_transfer);
  RUN_TEST(a_bench_message_is_a_well_formed_sysex);
  RUN_TEST(a_bench_request_is_recognised);
  RUN_TEST(a_burst_is_bounded);
  return TESTKIT_SUMMARY();
}
