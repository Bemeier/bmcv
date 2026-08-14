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
  uint8_t packets[64];
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
  RUN_TEST(a_bench_message_is_exactly_one_transfer);
  RUN_TEST(a_bench_message_is_a_well_formed_sysex);
  RUN_TEST(a_bench_request_is_recognised);
  RUN_TEST(a_burst_is_bounded);
  return TESTKIT_SUMMARY();
}
