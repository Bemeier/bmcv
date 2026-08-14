// Adopting a whole module as bytes.
//
// This is the path a debug probe takes: read the firmware's `bmcv` global out
// of the module's RAM, hand the blob to bmcv_sim_import(), and read the module
// back through the same accessors the simulator publishes. What it has to get
// right is that a BmcvInstance is not self-contained - nine of its members are
// pointers into itself and into the host's setup tables, and a blob carries the
// addresses of whichever module produced it.
//
// So the interesting failure is not "the numbers came out wrong". It is an
// instance that reads correctly and then walks into another module's addresses
// the first time anything follows one, which is a tick later and looks like
// something else entirely. Hence the two halves here: what a snapshot publishes,
// and what it does when run on.

#include "bmcv_sim.h"
#include "instance.h"
#include "testkit.h"
#include <stdlib.h>
#include <string.h>

#define TICK_US 250

// Something with every layer of the module doing work: a clock running, the
// slider away from its stop, a channel edited, a mode entered. A zeroed module
// would import correctly whatever the wiring did.
static void bring_to_life(BmcvSim* s)
{
  bmcv_sim_set_slider01(s, 0.4f);
  bmcv_sim_run(s, TICK_US, 200);

  // A shift mode, so ui_state is not at its defaults either.
  bmcv_sim_set_button(s, 0, 1);
  bmcv_sim_run(s, TICK_US, 40);
  bmcv_sim_add_encoder(s, 2, 3);
  bmcv_sim_run(s, TICK_US, 40);
  bmcv_sim_set_button(s, 0, 0);
  bmcv_sim_run(s, TICK_US, 40);

  // Four clock pulses at 120bpm, so the clock has a tempo and the channels have
  // something to lock to.
  for (int p = 0; p < 4; p++)
  {
    bmcv_sim_fire_gate(s, 0);
    bmcv_sim_run(s, TICK_US, 500);
  }
}

static void check_same_readings(const BmcvSim* a, const BmcvSim* b)
{
  const float* oa = bmcv_sim_outputs_v(a);
  const float* ob = bmcv_sim_outputs_v(b);
  const float* ea = bmcv_sim_effective(a);
  const float* eb = bmcv_sim_effective(b);
  const uint16_t* la = bmcv_sim_leds_rgb(a);
  const uint16_t* lb = bmcv_sim_leds_rgb(b);

  for (int c = 0; c < BMCV_SIM_CHANNELS; c++)
  {
    CHECK(oa[c] == ob[c]);
    CHECK(bmcv_sim_channel_muted(a, c) == bmcv_sim_channel_muted(b, c));
    CHECK(bmcv_sim_channel_shape_mode(a, c) == bmcv_sim_channel_shape_mode(b, c));

    for (int f = 0; f < BMCV_EFF_COUNT; f++)
    {
      CHECK(ea[c * BMCV_EFF_COUNT + f] == eb[c * BMCV_EFF_COUNT + f]);
    }
    for (int p = 0; p < 6; p++)
    {
      CHECK(bmcv_sim_channel_param(a, c, p) == bmcv_sim_channel_param(b, c, p));
    }
  }

  for (int i = 0; i < BMCV_SIM_LEDS * 3; i++)
  {
    CHECK(la[i] == lb[i]);
  }

  CHECK(bmcv_sim_shift_state(a) == bmcv_sim_shift_state(b));
  CHECK(bmcv_sim_selected_param(a) == bmcv_sim_selected_param(b));
  CHECK(bmcv_sim_active_scene(a) == bmcv_sim_active_scene(b));
  CHECK(bmcv_sim_bpm(a) == bmcv_sim_bpm(b));
  CHECK(bmcv_sim_active_bpm(a) == bmcv_sim_active_bpm(b));
  CHECK(bmcv_sim_have_beat(a) == bmcv_sim_have_beat(b));
  CHECK(bmcv_sim_error_flags(a) == bmcv_sim_error_flags(b));
}

// The published readings come out of the snapshot, without a tick. A host
// looking at hardware cannot advance the module to see them - the module is
// already running, somewhere else.
TEST_CASE(an_imported_snapshot_publishes_what_the_original_did)
{
  BmcvSim* a = bmcv_sim_create();
  BmcvSim* b = bmcv_sim_create();

  bring_to_life(a);

  void* blob = malloc((size_t) bmcv_sim_instance_size());
  bmcv_sim_export(a, blob);
  CHECK(bmcv_sim_import(b, blob, bmcv_sim_instance_size()) == 1);

  check_same_readings(a, b);

  // The snapshot carries its own clock, so the importing instance reports where
  // the module that produced it had got to rather than its own uptime - which
  // was zero, since b has never been run.
  CHECK(bmcv_sim_now_us(b) == bmcv_sim_now_us(a));

  free(blob);
  bmcv_sim_destroy(a);
  bmcv_sim_destroy(b);
}

// The half that catches a pointer left pointing at the original: every one of
// them is followed during a tick, so an instance that imported cleanly and was
// then run would either diverge or read another module's memory.
//
// Both sides are given the same inputs, so any difference is the import's.
TEST_CASE(an_imported_snapshot_keeps_running_in_step)
{
  BmcvSim* a = bmcv_sim_create();
  BmcvSim* b = bmcv_sim_create();

  // b is left at its power-on inputs deliberately - slider at the bottom, every
  // encoder at zero - because that is the case import has to survive: it is
  // what a browser tab looks like the moment before it adopts a module that has
  // been played. If the panel positions were not re-baselined, b's first tick
  // would read the whole difference as a gesture.
  bring_to_life(a);

  void* blob = malloc((size_t) bmcv_sim_instance_size());
  bmcv_sim_export(a, blob);
  CHECK(bmcv_sim_import(b, blob, bmcv_sim_instance_size()) == 1);

  for (int step = 0; step < 8; step++)
  {
    bmcv_sim_run(a, TICK_US, 137);
    bmcv_sim_run(b, TICK_US, 137);
    check_same_readings(a, b);
  }

  free(blob);
  bmcv_sim_destroy(a);
  bmcv_sim_destroy(b);
}

// A frontend computes the length from a wire message, and a probe read can come
// back short. Neither may reach memcpy.
TEST_CASE(a_blob_of_the_wrong_length_is_refused)
{
  BmcvSim* s = bmcv_sim_create();
  const int32_t n = bmcv_sim_instance_size();

  void* blob = calloc(1, (size_t) n);
  bmcv_sim_export(s, blob);

  CHECK(bmcv_sim_import(s, blob, n - 1) == 0);
  CHECK(bmcv_sim_import(s, blob, n + 1) == 0);
  CHECK(bmcv_sim_import(s, blob, 0) == 0);
  CHECK(bmcv_sim_import(s, blob, -n) == 0);
  CHECK(bmcv_sim_import(s, NULL, n) == 0);
  CHECK(bmcv_sim_import(s, blob, n) == 1);

  free(blob);
  bmcv_sim_destroy(s);
}

// The size is the one number a bridge has to agree with the module about before
// it reads a byte, so it is worth stating that it is the struct and nothing
// else - no header, no padding of the simulator's own.
TEST_CASE(the_instance_size_is_the_struct_size)
{
  CHECK(bmcv_sim_instance_size() == (int32_t) sizeof(BmcvInstance));
}

int main(void)
{
  RUN_TEST(an_imported_snapshot_publishes_what_the_original_did);
  RUN_TEST(an_imported_snapshot_keeps_running_in_step);
  RUN_TEST(a_blob_of_the_wrong_length_is_refused);
  RUN_TEST(the_instance_size_is_the_struct_size);
  return TESTKIT_SUMMARY();
}
