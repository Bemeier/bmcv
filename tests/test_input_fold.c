// The step between raw hardware and HwState.
//
// All of this used to live inside bmcv.c and was therefore unreachable from a
// host: clock trigger/reset dispatch, slider CV summing, CV scaling, encoder
// deltas and the config autosave had no coverage at all. They do now.

#include "channel.h"
#include "clock_sync.h"
#include "engine.h"
#include "hw_setup.h"
#include "input_fold.h"
#include "presets.h"
#include "testkit.h"
#include "ui_feedback.h"
#include "ui_select.h"
#include "ux_setup.h"
#include "ux_state.h"
#include <string.h>

typedef struct
{
  const HwSetup* hw;
  const UxSetup* ux_setup;
  EngineConfig cfg;
  EngineState es;
  UiState ui;
  UxState ux;
  InputFrames in;
  InputSample sample;
  PresetIo io;

  int store_calls;
  int8_t last_store_slot;
} Rig;

static int8_t counting_store(void* user, const EngineConfig* cfg, int8_t slot)
{
  (void) cfg;
  Rig* r = (Rig*) user;
  r->store_calls++;
  r->last_store_slot = slot;
  return 1;
}

static void rig_init(Rig* r)
{
  memset(r, 0, sizeof(*r));
  r->hw       = HwSetup_Get();
  r->ux_setup = UxSetup_InitFromHw(r->hw);

  r->io.store = counting_store;
  r->io.user  = r;

  r->ux.hw_setup      = r->hw;
  r->ux.ux_setup      = r->ux_setup;
  r->ux.engine_config = &r->cfg;
  r->ux.engine_state  = &r->es;
  r->ux.ui            = &r->ui;
  r->ux.presets       = &r->io;

  r->ui.momentary_scene = -1;
  ui_sel_reset(&r->ui);

  for (uint8_t i = 0; i < N_INPUTS; i++)
  {
    r->cfg.input_mode[i] = INPUT_DEFAULT;
  }

  r->sample.slider_raw = SLIDER_MIN_VALUE;

  Clock_Init(&r->es.clock);
  input_frames_init(&r->in, &r->ux, 0);
  ux_autosave_init(&r->ux, 0);
}

// Fold one sample into HwState and stop there - the input layer on its own.
// `now_us` is absolute, matching how the firmware drives it off a free-running
// microsecond counter.
static uint8_t rig_fold(Rig* r, uint32_t now_us)
{
  uint8_t dirty = input_fold(&r->in, &r->ux, &r->sample, now_us);
  // Gate pulses are one-shot: the caller latches an edge, the input layer
  // consumes it. Mirror that so a held-high sample does not retrigger - the
  // real firmware gets this for free, since midi_read_clock_trig() and
  // adc_read_trig_state() are both read-and-clear.
  memset(r->sample.cv_trig, 0, sizeof(r->sample.cv_trig));
  r->sample.midi_clock_trig = 0;
  r->sample.midi_reset_trig = 0;
  return dirty;
}

// A whole tick, the way bmcv_main and bmcv_instance_tick run it. Anything the
// input layer only *latches* - the clock, the reset, the autosave timer - is
// acted on by the engine, so a test of that behaviour has to run both halves.
static void rig_tick(Rig* r, uint32_t now_us)
{
  uint8_t dirty = rig_fold(r, now_us);
  engine_tick(&r->ux, now_us, dirty);
}

// -- trigger edge detection ------------------------------------------------

TEST_CASE(trig_step_has_hysteresis)
{
  uint8_t st = 0;

  CHECK(input_trig_step(0, &st) == 0);
  CHECK(input_trig_step(TRIG_THRESH - 1, &st) == 0);
  CHECK(input_trig_step(TRIG_THRESH, &st) == 1); // rising edge
  CHECK(input_trig_step(TRIG_THRESH, &st) == 0); // still high, no repeat
  CHECK(input_trig_step(20000, &st) == 0);

  // Between the two thresholds the latch stays set, so a noisy gate does not
  // produce a burst of triggers.
  CHECK(input_trig_step(TRIG_THRESH_LOW + 1, &st) == 0);
  CHECK(input_trig_step(TRIG_THRESH, &st) == 0);

  CHECK(input_trig_step(TRIG_THRESH_LOW - 1, &st) == 0); // releases
  CHECK(input_trig_step(TRIG_THRESH, &st) == 1);         // and can fire again
}

// -- encoders --------------------------------------------------------------

TEST_CASE(encoder_delta_is_the_change_since_the_last_tick)
{
  Rig r;
  rig_init(&r);

  r.sample.encoder_pos[0] = 10;
  CHECK(rig_fold(&r, MS(1)) != 0); // movement makes the tick dirty
  CHECK(r.ux.hw_state->encoder_delta[0] == 10);
  CHECK(r.ux.hw_state->encoder_state[0] == 10);

  r.sample.encoder_pos[0] = 13;
  rig_fold(&r, MS(2));
  CHECK(r.ux.hw_state->encoder_delta[0] == 3);

  // No movement: delta returns to zero and the tick is clean.
  CHECK(rig_fold(&r, MS(3)) == 0);
  CHECK(r.ux.hw_state->encoder_delta[0] == 0);

  r.sample.encoder_pos[0] = 8;
  rig_fold(&r, MS(4));
  CHECK(r.ux.hw_state->encoder_delta[0] == -5);
}

// The encoder position is free-running, so a host may let it wrap. The delta
// must stay small across the wrap rather than jumping by 65535.
TEST_CASE(encoder_delta_survives_int16_wraparound)
{
  Rig r;
  rig_init(&r);

  r.sample.encoder_pos[0] = INT16_MAX;
  rig_fold(&r, MS(1));

  r.sample.encoder_pos[0] = (int16_t) (INT16_MIN); // one step past the top
  rig_fold(&r, MS(2));
  CHECK(r.ux.hw_state->encoder_delta[0] == 1);

  r.sample.encoder_pos[0] = INT16_MAX; // and back down across the wrap
  rig_fold(&r, MS(3));
  CHECK(r.ux.hw_state->encoder_delta[0] == -1);
}

// -- buttons ---------------------------------------------------------------

TEST_CASE(button_level_changes_mark_the_tick_dirty)
{
  Rig r;
  rig_init(&r);

  CHECK(rig_fold(&r, MS(1)) == 0);

  r.sample.button_down[5] = 1;
  CHECK(rig_fold(&r, MS(2)) != 0);
  CHECK(r.ux.hw_state->button_state[5] == 1);

  // Held, not changed: no longer dirty. Durations are ui_input's job.
  CHECK(rig_fold(&r, MS(3)) == 0);
  CHECK(r.ux.hw_state->button_state[5] == 1);

  r.sample.button_down[5] = 0;
  CHECK(rig_fold(&r, MS(4)) != 0);
  CHECK(r.ux.hw_state->button_state[5] == 0);
}

// -- CV inputs -------------------------------------------------------------

TEST_CASE(cv_is_scaled_from_adc_units_into_dac_units)
{
  Rig r;
  rig_init(&r);

  // cv_raw is indexed by converter channel; input_adc_idx maps input -> channel.
  r.sample.cv_raw[r.hw->input_adc_idx[2]] = 1000;
  rig_fold(&r, MS(1));

  // ADC full scale is a quarter of DAC full scale at the same voltage.
  CHECK(r.ux.hw_state->input_state[2] == 4000);
  CHECK(ADC_10V * 4 == DAC_10V);
}

TEST_CASE(gate_edges_reach_trigger_src_through_the_input_mapping)
{
  Rig r;
  rig_init(&r);

  r.sample.cv_trig[r.hw->input_adc_idx[1]] = 1;
  rig_fold(&r, MS(1));

  CHECK(r.ux.hw_state->trigger_src[1] == 1);
  CHECK(r.ux.hw_state->trigger_src[0] == 0);

  rig_fold(&r, MS(2));
  CHECK(r.ux.hw_state->trigger_src[1] == 0); // one-shot
}

// -- slider ----------------------------------------------------------------

TEST_CASE(slider_passes_through_and_clamps)
{
  Rig r;
  rig_init(&r);

  r.sample.slider_raw = 4000;
  rig_fold(&r, MS(1));
  CHECK(r.ux.hw_state->slider_state == 4000);

  r.sample.slider_raw = SLIDER_MAX_VALUE + 500;
  rig_fold(&r, MS(2));
  CHECK(r.ux.hw_state->slider_state == SLIDER_MAX_VALUE);
}

// An input set to INPUT_SLIDER offsets the crossfader by its CV. This is the
// path that let a patch cable sweep scenes, and it had no coverage.
TEST_CASE(input_slider_mode_offsets_the_slider_by_cv)
{
  Rig r;
  rig_init(&r);

  r.cfg.input_mode[0] = INPUT_SLIDER;
  r.sample.slider_raw = 4000;

  r.sample.cv_raw[r.hw->input_adc_idx[0]] = 500;
  rig_fold(&r, MS(1));
  CHECK(r.ux.hw_state->slider_state == 4000 - 2 * 500);

  // Negative CV pushes the other way.
  r.sample.cv_raw[r.hw->input_adc_idx[0]] = -600;
  rig_fold(&r, MS(2));
  CHECK(r.ux.hw_state->slider_state == 4000 + 2 * 600);

  // Two inputs in slider mode sum.
  r.cfg.input_mode[1]                     = INPUT_SLIDER;
  r.sample.cv_raw[r.hw->input_adc_idx[0]] = 300;
  r.sample.cv_raw[r.hw->input_adc_idx[1]] = 200;
  rig_fold(&r, MS(3));
  CHECK(r.ux.hw_state->slider_state == 4000 - 2 * 500);

  // And an input left in DEFAULT contributes nothing.
  r.cfg.input_mode[1] = INPUT_DEFAULT;
  rig_fold(&r, MS(4));
  CHECK(r.ux.hw_state->slider_state == 4000 - 2 * 300);
}

// -- clock -----------------------------------------------------------------

// The input layer's whole contribution to the clock: it decides *that* a pulse
// or a reset happened, from the input modes, and latches it. Acting on it is
// engine_tick's job, so this half is assertable without a clock at all.
TEST_CASE(clock_and_reset_are_latched_into_hw_state)
{
  Rig r;
  rig_init(&r);
  r.cfg.input_mode[0] = INPUT_CLOCK;
  r.cfg.input_mode[1] = INPUT_RESET;
  r.cfg.input_mode[2] = INPUT_DEFAULT;

  r.sample.cv_trig[r.hw->input_adc_idx[0]] = 1;
  rig_fold(&r, MS(1));
  CHECK(r.ux.hw_state->clock_pulse == 1);
  CHECK(r.ux.hw_state->clock_reset == 0);

  r.sample.cv_trig[r.hw->input_adc_idx[1]] = 1;
  rig_fold(&r, MS(2));
  CHECK(r.ux.hw_state->clock_pulse == 0); // one-shot, like the edge itself
  CHECK(r.ux.hw_state->clock_reset == 1);

  // An input left in DEFAULT latches neither, however hard it is gated.
  r.sample.cv_trig[r.hw->input_adc_idx[2]] = 1;
  rig_fold(&r, MS(3));
  CHECK(r.ux.hw_state->clock_pulse == 0);
  CHECK(r.ux.hw_state->clock_reset == 0);
}

TEST_CASE(input_clock_mode_drives_the_clock)
{
  Rig r;
  rig_init(&r);
  r.cfg.input_mode[0] = INPUT_CLOCK;

  CHECK(!r.es.clock.have_beat);

  // 4 pulses per beat; 125ms per pulse -> 2Hz beat.
  uint32_t t = 0;
  for (int i = 0; i < 12; i++)
  {
    t += MS(125);
    r.sample.cv_trig[r.hw->input_adc_idx[0]] = 1;
    rig_tick(&r, t);
  }

  CHECK(r.es.clock.have_beat);
  CHECK_NEAR(r.es.clock.beat_freq_smooth, 2.0, 0.05);

  // An input left in DEFAULT must not drive the clock.
  Rig r2;
  rig_init(&r2);
  r2.cfg.input_mode[0] = INPUT_DEFAULT;
  t                    = 0;
  for (int i = 0; i < 12; i++)
  {
    t += MS(125);
    r2.sample.cv_trig[r2.hw->input_adc_idx[0]] = 1;
    rig_tick(&r2, t);
  }
  CHECK(!r2.es.clock.have_beat);

  // ...and the first instance's clock is untouched by the second's pulses.
  CHECK(r.es.clock.have_beat);
}

TEST_CASE(input_reset_mode_resets_clock_and_every_channel_phase)
{
  Rig r;
  rig_init(&r);
  r.cfg.input_mode[1] = INPUT_RESET;

  for (uint8_t c = 0; c < N_CHANNELS; c++)
  {
    r.es.channels_shared_phase[c]     = 0.5f;
    r.es.channels_phase_correction[c] = 0.25f;
  }
  r.es.clock.beat_counter = 7;

  r.sample.cv_trig[r.hw->input_adc_idx[1]] = 1;
  rig_tick(&r, MS(10));

  CHECK(r.es.clock.beat_counter == 0);
  CHECK(r.es.clock.last_reset_us == MS(10));
  for (uint8_t c = 0; c < N_CHANNELS; c++)
  {
    // Not exactly zero: the reset lands at the top of the tick and
    // channel_compute then advances the phase by one tick's worth. What
    // matters is that it came back from 0.5 to the start of the cycle.
    CHECK(r.es.channels_shared_phase[c] < 0.05f);
    CHECK(r.es.channels_phase_correction[c] == 0.0f);
  }
}

// A reset and a clock pulse landing on the same tick must reset first, or the
// beat counter advances past the reset the user just asked for.
TEST_CASE(reset_is_dispatched_before_clock_on_the_same_tick)
{
  Rig r;
  rig_init(&r);
  r.cfg.input_mode[0] = INPUT_CLOCK;
  r.cfg.input_mode[1] = INPUT_RESET;

  uint32_t t = 0;
  for (int i = 0; i < 8; i++)
  {
    t += MS(125);
    r.sample.cv_trig[r.hw->input_adc_idx[0]] = 1;
    rig_tick(&r, t);
  }
  CHECK(r.es.clock.beat_counter > 0);

  t += MS(125);
  r.sample.cv_trig[r.hw->input_adc_idx[0]] = 1;
  r.sample.cv_trig[r.hw->input_adc_idx[1]] = 1;
  rig_tick(&r, t);

  CHECK(r.es.clock.beat_counter == 0);
}

// -- MIDI clock/reset fallback -----------------------------------------------
//
// MIDI only drives the clock or reset when nothing is patched for the job - a
// jack always wins, whether or not anything is actually plugged into it. The
// two gate independently.

TEST_CASE(midi_clock_drives_the_clock_when_no_input_is_configured)
{
  Rig r;
  rig_init(&r); // every input starts at INPUT_DEFAULT

  CHECK(!r.es.clock.have_beat);

  // 24 pulses per beat at MIDI's fixed PPQN; ~20.8ms per pulse -> 2Hz beat.
  uint32_t t = 0;
  for (int i = 0; i < 48; i++)
  {
    t += 20833;
    r.sample.midi_clock_trig = 1;
    rig_tick(&r, t);
  }

  CHECK(r.es.clock.have_beat);
  CHECK(r.es.clock.PULSES_PER_BEAT == CLOCK_PULSES_PER_BEAT_MIDI);
  CHECK_NEAR(r.es.clock.beat_freq_smooth, 2.0, 0.05);
}

TEST_CASE(a_configured_clock_input_blocks_midi_even_when_the_input_is_idle)
{
  Rig r;
  rig_init(&r);
  r.cfg.input_mode[0] = INPUT_CLOCK; // configured, but never patched

  r.sample.midi_clock_trig = 1;
  rig_tick(&r, MS(1));

  CHECK(r.ux.hw_state->clock_pulse == 0);
  CHECK(!r.es.clock.have_beat);
  CHECK(r.es.clock.PULSES_PER_BEAT == CLOCK_PULSES_PER_BEAT_CV);
}

TEST_CASE(midi_reset_resets_only_when_no_input_is_configured_as_reset)
{
  Rig r;
  rig_init(&r);
  r.es.clock.beat_counter = 7;

  r.sample.midi_reset_trig = 1;
  rig_tick(&r, MS(1));
  CHECK(r.es.clock.beat_counter == 0);

  // Now a physical reset input is configured: MIDI must no longer reach it,
  // patched or not.
  Rig r2;
  rig_init(&r2);
  r2.cfg.input_mode[1]     = INPUT_RESET;
  r2.es.clock.beat_counter = 7;

  r2.sample.midi_reset_trig = 1;
  rig_tick(&r2, MS(1));
  CHECK(r2.es.clock.beat_counter == 7);
}

// Clock and reset gate independently: a reset jack in use must not block a
// MIDI clock that has nowhere physical to come from, or the other way round.
TEST_CASE(clock_and_reset_fallback_gate_independently)
{
  Rig r;
  rig_init(&r);
  r.cfg.input_mode[1] = INPUT_RESET; // reset is spoken for, clock is not

  r.sample.midi_clock_trig = 1;
  rig_fold(&r, MS(1));
  CHECK(r.ux.hw_state->clock_pulse == 1);

  r.sample.midi_reset_trig = 1;
  rig_fold(&r, MS(2));
  CHECK(r.ux.hw_state->clock_reset == 0);
}

// -- autosave --------------------------------------------------------------

TEST_CASE(autosave_writes_only_when_the_config_actually_changed)
{
  Rig r;
  rig_init(&r);

  // Nothing changed: crossing the interval must not write.
  rig_tick(&r, MS(2500));
  CHECK(r.store_calls == 0);

  r.cfg.scene_b = 3;

  // Still inside the interval since the last check.
  rig_tick(&r, MS(3000));
  CHECK(r.store_calls == 0);

  rig_tick(&r, MS(5000));
  CHECK(r.store_calls == 1);
  CHECK(r.last_store_slot == CONFIG_AUTOSAVE_SLOT);

  // Unchanged since that write: no second write.
  rig_tick(&r, MS(8000));
  CHECK(r.store_calls == 1);

  r.cfg.scene_b = 4;
  rig_tick(&r, MS(11000));
  CHECK(r.store_calls == 2);
}

// The autosave is housekeeping on a timer, not something the user did, so it
// must not paint a confirmation - that used to flash the whole scene row every
// time a knob settled.
TEST_CASE(autosave_is_silent)
{
  Rig r;
  rig_init(&r);

  r.cfg.scene_b = 3;
  rig_tick(&r, MS(2500));
  rig_tick(&r, MS(5000));
  CHECK(r.store_calls == 1);

  for (uint8_t s = 0; s < N_SCENES; s++)
  {
    CHECK(!ui_feedback_active(&r.ui, TGT_SCENE, (int8_t) s, NULL));
  }
  CHECK(!ui_feedback_active(&r.ui, TGT_SCENE, -1, NULL));
}

// The load path treats a NULL PresetIo as "nothing stored", which is what
// bmcv_init reads as first boot, and store must not crash.
TEST_CASE(null_preset_io_is_safe)
{
  Rig r;
  rig_init(&r);
  r.ux.presets = NULL;

  CHECK(ux_preset_load(&r.ux, 0) == 0);
  CHECK(ux_preset_store(&r.ux, 0) == 0);

  r.cfg.scene_b = 5;
  rig_tick(&r, MS(2500));
  rig_tick(&r, MS(5000));
  CHECK(r.store_calls == 0);
}

// -- frame bookkeeping -----------------------------------------------------

TEST_CASE(frames_carry_time_and_dt)
{
  Rig r;
  rig_init(&r);

  rig_fold(&r, MS(5));
  CHECK(r.ux.hw_state->time == MS(5));
  CHECK(r.ux.hw_state->dt == MS(5));

  rig_fold(&r, MS(12));
  CHECK(r.ux.hw_state->time == MS(12));
  CHECK(r.ux.hw_state->dt == MS(7));
}

// Two instances must not interfere through the frame ring buffer. (The clock
// is still a global - see docs/plans/virtual-bmcv.md phase 1b.)
TEST_CASE(two_instances_keep_separate_input_state)
{
  Rig a, b;
  rig_init(&a);
  rig_init(&b);

  a.sample.encoder_pos[0] = 10;
  a.sample.button_down[3] = 1;
  rig_fold(&a, MS(1));
  rig_fold(&b, MS(1));

  CHECK(a.ux.hw_state->encoder_delta[0] == 10);
  CHECK(b.ux.hw_state->encoder_delta[0] == 0);
  CHECK(a.ux.hw_state->button_state[3] == 1);
  CHECK(b.ux.hw_state->button_state[3] == 0);
}

int main(void)
{
  RUN_TEST(trig_step_has_hysteresis);
  RUN_TEST(encoder_delta_is_the_change_since_the_last_tick);
  RUN_TEST(encoder_delta_survives_int16_wraparound);
  RUN_TEST(button_level_changes_mark_the_tick_dirty);
  RUN_TEST(cv_is_scaled_from_adc_units_into_dac_units);
  RUN_TEST(gate_edges_reach_trigger_src_through_the_input_mapping);
  RUN_TEST(slider_passes_through_and_clamps);
  RUN_TEST(input_slider_mode_offsets_the_slider_by_cv);
  RUN_TEST(clock_and_reset_are_latched_into_hw_state);
  RUN_TEST(input_clock_mode_drives_the_clock);
  RUN_TEST(input_reset_mode_resets_clock_and_every_channel_phase);
  RUN_TEST(reset_is_dispatched_before_clock_on_the_same_tick);
  RUN_TEST(midi_clock_drives_the_clock_when_no_input_is_configured);
  RUN_TEST(a_configured_clock_input_blocks_midi_even_when_the_input_is_idle);
  RUN_TEST(midi_reset_resets_only_when_no_input_is_configured_as_reset);
  RUN_TEST(clock_and_reset_fallback_gate_independently);
  RUN_TEST(autosave_writes_only_when_the_config_actually_changed);
  RUN_TEST(autosave_is_silent);
  RUN_TEST(null_preset_io_is_safe);
  RUN_TEST(frames_carry_time_and_dt);
  RUN_TEST(two_instances_keep_separate_input_state);
  return TESTKIT_SUMMARY();
}
