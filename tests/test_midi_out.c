// What the module publishes on the MIDI bus, with no USB in it. The mirror of
// test_midi_realtime.c: that one decodes the bus, this one drives it.
//
// EngineState and HwState are built by hand rather than run through a whole
// instance, because what is under test is the mapping and the rate limiting,
// and both are easier to state as "these levels produce these messages".

#include "midi_out.h"
#include "testkit.h"

#include <string.h>

typedef struct
{
  MidiOut out;
  EngineState es;
  HwState hw;
  uint32_t now_us;
} Fix;

static void fix_init(Fix* f)
{
  memset(f, 0, sizeof(*f));
  midi_out_init(&f->out);

  // The default posture for the CC tests: MIDI is the clock source, so the
  // clock half stays silent and every message drained is a control change.
  f->hw.clock_source_is_midi = 1;
}

static void advance(Fix* f, uint32_t dt_us)
{
  f->now_us += dt_us;
  midi_out_publish(&f->out, &f->es, &f->hw, f->now_us);
}

// Drain everything, since a test wants the whole slot rather than a transport's
// worth. MIDI_OUT_QUEUE_LEN is the most that can be waiting.
static uint8_t drain(Fix* f, MidiMsg* dst) { return midi_out_drain(&f->out, dst, MIDI_OUT_QUEUE_LEN); }

static uint8_t count_status(const MidiMsg* m, uint8_t n, uint8_t status)
{
  uint8_t k = 0;
  for (uint8_t i = 0; i < n; i++)
    if (m[i].status == status)
      k++;
  return k;
}

// Find the value of the last control change for one CC, or -1 if there is none.
static int cc_value(const MidiMsg* m, uint8_t n, uint8_t cc)
{
  int v = -1;
  for (uint8_t i = 0; i < n; i++)
    if (m[i].status == MIDI_OUT_STATUS_CC && m[i].d1 == cc)
      v = m[i].d2;
  return v;
}

/* ---- the mapping -------------------------------------------------------- */

TEST_CASE(the_converter_range_maps_onto_seven_bits)
{
  CHECK(midi_out_scale7(-DAC_10V) == 0);
  CHECK(midi_out_scale7(DAC_10V - 1) == 127);

  // 0V sits at the middle of the range, not at the bottom: the outputs are
  // bipolar and a CC is not, so half the range is below zero volts.
  CHECK(midi_out_scale7(0) == 63);

  // Monotonic, and it does not wrap at either end.
  CHECK(midi_out_scale7(-DAC_10V / 2) < midi_out_scale7(0));
  CHECK(midi_out_scale7(DAC_10V / 2) > midi_out_scale7(0));
}

TEST_CASE(out_of_range_levels_clamp_rather_than_wrap)
{
  CHECK(midi_out_scale7(INT16_MIN) == 0);
  CHECK(midi_out_scale7(INT16_MAX) == 127);
}

TEST_CASE(channels_and_inputs_share_one_contiguous_cc_block)
{
  Fix f;
  fix_init(&f);

  for (uint8_t c = 0; c < N_CHANNELS; c++)
    f.es.channels_gated_level[c] = (int16_t) (-DAC_10V + c);
  for (uint8_t i = 0; i < N_INPUTS; i++)
    f.hw.input_state[i] = (int16_t) (DAC_10V - 1);

  advance(&f, MIDI_OUT_PUBLISH_US);

  MidiMsg m[MIDI_OUT_QUEUE_LEN];
  const uint8_t n = drain(&f, m);
  CHECK(n == MIDI_OUT_CC_COUNT);

  for (uint8_t c = 0; c < N_CHANNELS; c++)
    CHECK(cc_value(m, n, (uint8_t) (MIDI_OUT_CC_BASE + c)) == 0);
  for (uint8_t i = 0; i < N_INPUTS; i++)
    CHECK(cc_value(m, n, (uint8_t) (MIDI_OUT_CC_INPUT_BASE + i)) == 127);

  // Channel 1 on the MIDI side, and a three-byte message.
  for (uint8_t i = 0; i < n; i++)
  {
    CHECK(m[i].status == MIDI_OUT_STATUS_CC);
    CHECK(m[i].len == 3);
  }
}

// Mute is an output-stage gain, so a muted channel reads as silent here for the
// same reason its jack does - the gated level is what leaves the module.
TEST_CASE(the_gated_level_is_what_gets_published)
{
  Fix f;
  fix_init(&f);

  f.es.channels_output_level[0] = DAC_10V - 1;
  f.es.channels_gated_level[0]  = 0;

  advance(&f, MIDI_OUT_PUBLISH_US);

  MidiMsg m[MIDI_OUT_QUEUE_LEN];
  const uint8_t n = drain(&f, m);
  CHECK(cc_value(m, n, MIDI_OUT_CC_BASE) == 63);
}

/* ---- rate limiting ------------------------------------------------------ */

TEST_CASE(an_unchanged_value_is_sent_once)
{
  Fix f;
  fix_init(&f);

  MidiMsg m[MIDI_OUT_QUEUE_LEN];

  advance(&f, MIDI_OUT_PUBLISH_US);
  CHECK(drain(&f, m) == MIDI_OUT_CC_COUNT); // first slot states everything

  advance(&f, MIDI_OUT_PUBLISH_US);
  CHECK(drain(&f, m) == 0); // nothing moved, so nothing is said

  f.es.channels_gated_level[3] = DAC_10V / 2;
  advance(&f, MIDI_OUT_PUBLISH_US);

  const uint8_t n = drain(&f, m);
  CHECK(n == 1);
  CHECK(m[0].d1 == MIDI_OUT_CC_BASE + 3);
}

TEST_CASE(publishing_is_on_a_divider_not_every_tick)
{
  Fix f;
  fix_init(&f);

  MidiMsg m[MIDI_OUT_QUEUE_LEN];
  advance(&f, MIDI_OUT_PUBLISH_US);
  drain(&f, m);

  // A quarter of a slot at a time, with the level moving every tick. Only the
  // ticks that land on a slot boundary may say anything.
  uint8_t slots = 0;
  for (uint8_t i = 0; i < 8; i++)
  {
    f.es.channels_gated_level[0] = (int16_t) (i * (DAC_10V / 16));
    advance(&f, MIDI_OUT_PUBLISH_US / 4);
    if (drain(&f, m) > 0)
      slots++;
  }

  CHECK(slots == 2);
}

// A value that changes below the 7-bit grid is not a change. This is what keeps
// a slow LFO from emitting on every slot.
TEST_CASE(sub_quantum_movement_says_nothing)
{
  Fix f;
  fix_init(&f);

  MidiMsg m[MIDI_OUT_QUEUE_LEN];
  advance(&f, MIDI_OUT_PUBLISH_US);
  drain(&f, m);

  f.es.channels_gated_level[0] = 1;
  advance(&f, MIDI_OUT_PUBLISH_US);
  CHECK(drain(&f, m) == 0);
}

/* ---- the clock ---------------------------------------------------------- */

// Following a jack: the module forwards its clock to the host.
static void fix_clock_from_jack(Fix* f)
{
  f->hw.clock_source_is_midi = 0;
  f->es.clock.have_beat      = true;
}

TEST_CASE(a_beat_produces_twenty_four_clocks)
{
  Fix f;
  fix_init(&f);
  fix_clock_from_jack(&f);

  MidiMsg m[MIDI_OUT_QUEUE_LEN];
  uint8_t clocks = 0;
  uint8_t starts = 0;

  // One beat, walked in 240 steps so no step crosses more than one slot.
  for (uint16_t i = 0; i < 240; i++)
  {
    f.es.clock.beat_phase = (float) i / 240.0f;
    advance(&f, 100);

    const uint8_t n = drain(&f, m);
    clocks += count_status(m, n, MIDI_RT_CLOCK);
    starts += count_status(m, n, MIDI_RT_START);
  }

  // The first pass sends Start plus the downbeat's clock, then one per slot.
  CHECK(starts == 1);
  CHECK(clocks == MIDI_OUT_CLOCK_PPQN);
}

// The clock is not on the publish divider: at 120BPM a slot is 20.8ms, and
// waiting 2ms to notice one would be 10% of jitter.
TEST_CASE(clock_is_evaluated_every_tick)
{
  Fix f;
  fix_init(&f);
  fix_clock_from_jack(&f);

  MidiMsg m[MIDI_OUT_QUEUE_LEN];
  advance(&f, 100); // Start + downbeat
  drain(&f, m);

  // Well inside one publish slot, but a whole 24-PPQN slot of phase.
  f.es.clock.beat_phase = 1.5f / (float) MIDI_OUT_CLOCK_PPQN;
  advance(&f, 100);

  const uint8_t n = drain(&f, m);
  CHECK(count_status(m, n, MIDI_RT_CLOCK) == 1);
}

// A patched cable wins and MIDI is the fallback - the same rule input_fold
// applies to the input direction. When the host is the master, saying its own
// clock back to it is at best redundant.
TEST_CASE(nothing_is_sent_while_midi_is_the_clock_source)
{
  Fix f;
  fix_init(&f);
  f.es.clock.have_beat = true; // but clock_source_is_midi stays set

  MidiMsg m[MIDI_OUT_QUEUE_LEN];
  for (uint16_t i = 0; i < 240; i++)
  {
    f.es.clock.beat_phase = (float) i / 240.0f;
    advance(&f, 100);

    const uint8_t n = drain(&f, m);
    CHECK(count_status(m, n, MIDI_RT_CLOCK) == 0);
    CHECK(count_status(m, n, MIDI_RT_START) == 0);
  }
}

TEST_CASE(losing_the_beat_stops_the_far_end)
{
  Fix f;
  fix_init(&f);
  fix_clock_from_jack(&f);

  MidiMsg m[MIDI_OUT_QUEUE_LEN];
  advance(&f, 100);
  drain(&f, m);

  f.es.clock.have_beat = false;
  advance(&f, 100);

  uint8_t n = drain(&f, m);
  CHECK(count_status(m, n, MIDI_RT_STOP) == 1);

  // And only once - a stopped clock keeps not running.
  advance(&f, 100);
  n = drain(&f, m);
  CHECK(n == 0);
}

// Clock_Reset zeroes beat_phase before this runs, so the Start refers to the
// downbeat the slot is taken from.
TEST_CASE(a_reset_restarts_rather_than_nudges)
{
  Fix f;
  fix_init(&f);
  fix_clock_from_jack(&f);

  MidiMsg m[MIDI_OUT_QUEUE_LEN];
  f.es.clock.beat_phase = 0.5f;
  advance(&f, 100);
  drain(&f, m);

  f.hw.clock_reset      = 1;
  f.es.clock.beat_phase = 0.0f;
  advance(&f, 100);

  const uint8_t n = drain(&f, m);
  CHECK(count_status(m, n, MIDI_RT_START) == 1);
  CHECK(count_status(m, n, MIDI_RT_CLOCK) == 1);
}

// A phase jump is not elapsed time, and replaying it as clock would hand the
// far end a tempo spike.
TEST_CASE(a_phase_jump_does_not_replay_the_gap)
{
  Fix f;
  fix_init(&f);
  fix_clock_from_jack(&f);

  MidiMsg m[MIDI_OUT_QUEUE_LEN];
  advance(&f, 100);
  drain(&f, m);

  f.es.clock.beat_phase = 0.9f; // most of a beat, in one tick
  advance(&f, 100);

  const uint8_t n = drain(&f, m);
  CHECK(count_status(m, n, MIDI_RT_CLOCK) == 1);
}

/* ---- the queue ---------------------------------------------------------- */

TEST_CASE(draining_takes_at_most_what_was_asked_for)
{
  Fix f;
  fix_init(&f);

  for (uint8_t c = 0; c < N_CHANNELS; c++)
    f.es.channels_gated_level[c] = DAC_10V / 2;
  advance(&f, MIDI_OUT_PUBLISH_US);

  MidiMsg m[MIDI_OUT_QUEUE_LEN];
  CHECK(midi_out_drain(&f.out, m, 3) == 3);
  CHECK(midi_out_drain(&f.out, m, MIDI_OUT_QUEUE_LEN) == MIDI_OUT_CC_COUNT - 3);
  CHECK(midi_out_drain(&f.out, m, MIDI_OUT_QUEUE_LEN) == 0);
}

// A host that stops draining must not leave the far end holding a value that is
// no longer true: a dropped control change is re-stated on the next slot.
TEST_CASE(a_dropped_control_change_is_said_again)
{
  Fix f;
  fix_init(&f);

  // Fill the queue without draining it, moving every value each slot.
  for (uint8_t i = 0; i < 8; i++)
  {
    for (uint8_t c = 0; c < N_CHANNELS; c++)
      f.es.channels_gated_level[c] = (int16_t) (i * (DAC_10V / 8));
    advance(&f, MIDI_OUT_PUBLISH_US);
  }

  MidiMsg m[MIDI_OUT_QUEUE_LEN];
  CHECK(drain(&f, m) == MIDI_OUT_QUEUE_LEN - 1); // full, minus the empty slot

  // The values that never made it out are still pending, so the next slot
  // states them even though nothing moved since.
  advance(&f, MIDI_OUT_PUBLISH_US);
  const uint8_t n = drain(&f, m);
  CHECK(n > 0);

  const uint8_t expect = midi_out_scale7((int16_t) (7 * (DAC_10V / 8)));
  for (uint8_t c = 0; c < N_CHANNELS; c++)
  {
    const int v = cc_value(m, n, (uint8_t) (MIDI_OUT_CC_BASE + c));
    CHECK(v == -1 || v == (int) expect);
  }
}

int main(void)
{
  RUN_TEST(the_converter_range_maps_onto_seven_bits);
  RUN_TEST(out_of_range_levels_clamp_rather_than_wrap);
  RUN_TEST(channels_and_inputs_share_one_contiguous_cc_block);
  RUN_TEST(the_gated_level_is_what_gets_published);

  RUN_TEST(an_unchanged_value_is_sent_once);
  RUN_TEST(publishing_is_on_a_divider_not_every_tick);
  RUN_TEST(sub_quantum_movement_says_nothing);

  RUN_TEST(a_beat_produces_twenty_four_clocks);
  RUN_TEST(clock_is_evaluated_every_tick);
  RUN_TEST(nothing_is_sent_while_midi_is_the_clock_source);
  RUN_TEST(losing_the_beat_stops_the_far_end);
  RUN_TEST(a_reset_restarts_rather_than_nudges);
  RUN_TEST(a_phase_jump_does_not_replay_the_gap);

  RUN_TEST(draining_takes_at_most_what_was_asked_for);
  RUN_TEST(a_dropped_control_change_is_said_again);
  return TESTKIT_SUMMARY();
}
