// System Real-Time decode: what a USB-MIDI transfer means for the clock, with
// no USB in it. midi.c feeds it the same buffer sysex_feed reads; the two
// look for disjoint Code Index Numbers and neither should see the other's
// traffic.

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
  CHECK(ev.clock == 1);
  CHECK(ev.start == 0);
}

TEST_CASE(start_byte_is_reported)
{
  uint8_t packet[4];
  pack_rt(packet, 0, MIDI_RT_START);

  MidiRealtimeEvents ev = midi_realtime_feed(packet, sizeof packet);
  CHECK(ev.clock == 0);
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
  CHECK(ev.clock == 0);
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
  CHECK(ev.clock == 0);
  CHECK(ev.start == 0);
}

TEST_CASE(both_in_one_transfer_are_both_reported)
{
  uint8_t packets[8];
  pack_rt(packets, 0, MIDI_RT_START);
  pack_rt(packets + 4, 0, MIDI_RT_CLOCK);

  MidiRealtimeEvents ev = midi_realtime_feed(packets, sizeof packets);
  CHECK(ev.clock == 1);
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
  CHECK(ev.clock == 0);
  CHECK(ev.start == 0);
}

TEST_CASE(zero_padding_is_not_a_message)
{
  // The class hands over the whole buffer every time, padded with zeroes.
  // Code index 0 is reserved and must not read as a Single Byte packet.
  uint8_t packets[16];
  memset(packets, 0, sizeof packets);
  MidiRealtimeEvents ev = midi_realtime_feed(packets, sizeof packets);
  CHECK(ev.clock == 0);
  CHECK(ev.start == 0);
}

TEST_CASE(cable_number_does_not_affect_recognition)
{
  uint8_t packet[4];
  pack_rt(packet, 7, MIDI_RT_CLOCK);

  MidiRealtimeEvents ev = midi_realtime_feed(packet, sizeof packet);
  CHECK(ev.clock == 1);
}

int main(void)
{
  RUN_TEST(clock_byte_is_reported);
  RUN_TEST(start_byte_is_reported);
  RUN_TEST(continue_and_stop_report_nothing);
  RUN_TEST(other_realtime_bytes_report_nothing);
  RUN_TEST(both_in_one_transfer_are_both_reported);
  RUN_TEST(ignores_ordinary_midi_and_sysex_traffic);
  RUN_TEST(zero_padding_is_not_a_message);
  RUN_TEST(cable_number_does_not_affect_recognition);
  return TESTKIT_SUMMARY();
}
