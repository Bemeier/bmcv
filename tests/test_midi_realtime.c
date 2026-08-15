// System Real-Time decode: what a USB-MIDI transfer means for the clock, with
// no USB in it. midi.c feeds it every transfer that arrives on the MIDI OUT
// endpoint, so it has to pick its four status bytes out of whatever else a DAW
// is sending and leave the rest alone.

#include "midi_realtime.h"
#include "testkit.h"

#include <string.h>

// One System Real-Time byte as a USB-MIDI Single Byte packet: CIN 0xF, the
// byte, then two bytes of padding a real transfer would also carry.
static void pack_rt(uint8_t* out, uint8_t cable, uint8_t byte)
{
  out[0] = (uint8_t) ((cable << 4) | 0x0F);
  out[1] = byte;
  out[2] = 0;
  out[3] = 0;
}

TEST_CASE(clock_byte_is_reported)
{
  uint8_t packet[4];
  pack_rt(packet, 0, MIDI_RT_CLOCK);

  MidiRealtimeEvents ev = midi_realtime_feed(packet, sizeof packet);
  CHECK(ev.clocks == 1);
  CHECK(ev.start == 0);
}

TEST_CASE(start_byte_is_reported)
{
  uint8_t packet[4];
  pack_rt(packet, 0, MIDI_RT_START);

  MidiRealtimeEvents ev = midi_realtime_feed(packet, sizeof packet);
  CHECK(ev.clocks == 0);
  CHECK(ev.start == 1);
}

// Continue must not read as Start - it resumes rather than restarts, and
// reporting it the same way would reset phase on every DAW punch-in. Stop
// needs no report at all: a source that stops pulsing already reads as gone
// once Clock_Poll's own timeout passes.
TEST_CASE(continue_and_stop_report_nothing)
{
  uint8_t packets[8];
  pack_rt(packets, 0, MIDI_RT_CONTINUE);
  pack_rt(packets + 4, 0, MIDI_RT_STOP);

  MidiRealtimeEvents ev = midi_realtime_feed(packets, sizeof packets);
  CHECK(ev.clocks == 0);
  CHECK(ev.start == 0);
}

// Active Sensing (0xFE) and System Reset (0xFF) are Real-Time too, but are
// neither the clock nor the transport - MIDI's "Reset" resets the receiving
// device, not the beat, and is not this module's concept of a reset at all.
TEST_CASE(other_realtime_bytes_report_nothing)
{
  uint8_t packets[8];
  pack_rt(packets, 0, 0xFE);
  pack_rt(packets + 4, 0, 0xFF);

  MidiRealtimeEvents ev = midi_realtime_feed(packets, sizeof packets);
  CHECK(ev.clocks == 0);
  CHECK(ev.start == 0);
}

// The case the boolean this replaced could not express, and the one that
// actually happens: a host that has fallen behind delivers its clocks in a
// batch, and every one of them is a beat. Reporting the batch as a single
// clock dropped the rest, which the sync loop saw as a gap in the beat grid.
//
// A transfer holds sixteen packets, so the count has to survive a full one.
TEST_CASE(several_clocks_in_one_transfer_are_all_counted)
{
  uint8_t two[8];
  pack_rt(two, 0, MIDI_RT_CLOCK);
  pack_rt(two + 4, 0, MIDI_RT_CLOCK);
  CHECK(midi_realtime_feed(two, sizeof two).clocks == 2);

  uint8_t full[16 * 4];
  for (int i = 0; i < 16; i++)
    pack_rt(full + i * 4, 0, MIDI_RT_CLOCK);
  CHECK(midi_realtime_feed(full, sizeof full).clocks == 16);
}

// Clocks count, a reset does not: advancing the beat twice is two beats, but
// resetting twice lands in the same place as resetting once.
TEST_CASE(a_repeated_start_is_still_one_reset)
{
  uint8_t packets[12];
  pack_rt(packets, 0, MIDI_RT_START);
  pack_rt(packets + 4, 0, MIDI_RT_START);
  pack_rt(packets + 8, 0, MIDI_RT_CLOCK);

  MidiRealtimeEvents ev = midi_realtime_feed(packets, sizeof packets);
  CHECK(ev.start == 1);
  CHECK(ev.clocks == 1);
}

TEST_CASE(both_in_one_transfer_are_both_reported)
{
  uint8_t packets[8];
  pack_rt(packets, 0, MIDI_RT_START);
  pack_rt(packets + 4, 0, MIDI_RT_CLOCK);

  MidiRealtimeEvents ev = midi_realtime_feed(packets, sizeof packets);
  CHECK(ev.clocks == 1);
  CHECK(ev.start == 1);
}

// The packets a DAW's note/CC traffic actually produces, plus a SysEx
// packet - none of it is Code Index Number 0xF, so none of it should read as
// Real-Time.
TEST_CASE(ignores_ordinary_midi_and_sysex_traffic)
{
  const uint8_t traffic[] = {
      0x09, 0x90, 0x40, 0x7F, // note on
      0x08, 0x80, 0x40, 0x00, // note off
      0x0B, 0xB0, 0x07, 0x64, // control change
      0x04, 0xF0, 0x7D, 0x42, // sysex start
      0x06, 0x4D, 0x01, 0xF7, // sysex end
  };
  MidiRealtimeEvents ev = midi_realtime_feed(traffic, sizeof traffic);
  CHECK(ev.clocks == 0);
  CHECK(ev.start == 0);
}

TEST_CASE(zero_padding_is_not_a_message)
{
  // The class hands over the whole buffer every time, padded with zeroes.
  // Code index 0 is reserved and must not read as a Single Byte packet.
  uint8_t packets[16];
  memset(packets, 0, sizeof packets);
  MidiRealtimeEvents ev = midi_realtime_feed(packets, sizeof packets);
  CHECK(ev.clocks == 0);
  CHECK(ev.start == 0);
}

TEST_CASE(cable_number_does_not_affect_recognition)
{
  uint8_t packet[4];
  pack_rt(packet, 7, MIDI_RT_CLOCK);

  MidiRealtimeEvents ev = midi_realtime_feed(packet, sizeof packet);
  CHECK(ev.clocks == 1);
}

int main(void)
{
  RUN_TEST(clock_byte_is_reported);
  RUN_TEST(start_byte_is_reported);
  RUN_TEST(continue_and_stop_report_nothing);
  RUN_TEST(other_realtime_bytes_report_nothing);
  RUN_TEST(several_clocks_in_one_transfer_are_all_counted);
  RUN_TEST(a_repeated_start_is_still_one_reset);
  RUN_TEST(both_in_one_transfer_are_both_reported);
  RUN_TEST(ignores_ordinary_midi_and_sysex_traffic);
  RUN_TEST(zero_padding_is_not_a_message);
  RUN_TEST(cable_number_does_not_affect_recognition);
  return TESTKIT_SUMMARY();
}
