// One module per BmcvInstance, with nothing shared behind its back.
//
// The firmware only ever has one module, so a global clock and a file-static
// error latch were harmless there. A simulator or a VCV Rack patch can hold
// several, and two of them sharing a tempo is the kind of bug that is very
// hard to see and impossible to work around. These tests are what keep that
// state out of file scope.

#include "clock_sync.h"
#include "error.h"
#include "hw_setup.h"
#include "instance.h"
#include "presets.h"
#include "testkit.h"
#include "ui_input.h"
#include <string.h>

// A preset backend per instance, so "stored config" is not shared either.
typedef struct
{
  EngineConfig slots[FRAM_CONFIG_SLOTS];
  uint8_t occupied[FRAM_CONFIG_SLOTS];
  int stores;
  int loads;
} FakeFram;

static int8_t fake_store(void* user, const EngineConfig* cfg, int8_t slot)
{
  FakeFram* f = (FakeFram*) user;
  if (slot < 0 || slot >= FRAM_CONFIG_SLOTS)
    return 0;
  f->slots[slot]    = *cfg;
  f->occupied[slot] = 1;
  f->stores++;
  return 1;
}

static int8_t fake_load(void* user, EngineConfig* cfg, int8_t slot)
{
  FakeFram* f = (FakeFram*) user;
  if (slot < 0 || slot >= FRAM_CONFIG_SLOTS || !f->occupied[slot])
    return 0;
  *cfg = f->slots[slot];
  f->loads++;
  return 1;
}

static void tick(BmcvInstance* m, const InputSample* s, uint32_t now_us) { bmcv_instance_tick(m, s, now_us); }

TEST_CASE(a_fresh_instance_comes_up_with_first_boot_defaults)
{
  BmcvInstance m;
  bmcv_instance_init(&m, NULL, 0);

  // No PresetIo means nothing was stored, which is the first-boot path.
  CHECK(m.engine_config.input_mode[0] == INPUT_CLOCK);
  CHECK(m.engine_config.input_mode[1] == INPUT_RESET);
  CHECK(m.engine_config.scene_a == 0);
  CHECK(m.engine_config.scene_b == 6);
  CHECK(m.engine_config.quantize_mask == 0b111111111111);

  // A module that has never been used is not in an error state: no stored
  // config is the expected condition, and config_validate has already made the
  // defaults safe to index.
  CHECK(!error_any(&m.engine_state));

  // And the UxState is fully wired, so a tick is safe immediately.
  CHECK(m.ux.hw_setup == m.hw_setup);
  CHECK(m.ux.ux_setup == m.ux_setup);
  CHECK(m.ux.engine_config == &m.engine_config);
  CHECK(m.ux.engine_state == &m.engine_state);
  CHECK(m.ux.ui == &m.ui_state);
  CHECK(m.ux.hw_state != NULL);
  CHECK(m.ui_state.momentary_scene == -1);

  InputSample s;
  memset(&s, 0, sizeof(s));
  s.slider_raw = SLIDER_MIN_VALUE;
  tick(&m, &s, MS(1));
}

TEST_CASE(a_stored_config_is_loaded_instead_of_the_defaults)
{
  FakeFram fram;
  memset(&fram, 0, sizeof(fram));

  EngineConfig saved;
  memset(&saved, 0, sizeof(saved));
  saved.scene_a       = 2;
  saved.scene_b       = 3;
  saved.quantize_mask = 0b101010101010;
  for (uint8_t c = 0; c < N_ENCODERS; c++)
  {
    saved.channel_state[c].src_input = -1;
    saved.channel_state[c].src_trig  = -1;
  }
  fram.slots[FRAM_CONFIG_SLOTS - 1]    = saved;
  fram.occupied[FRAM_CONFIG_SLOTS - 1] = 1;

  PresetIo io = {.store = fake_store, .load = fake_load, .user = &fram};

  BmcvInstance m;
  bmcv_instance_init(&m, &io, 0);

  CHECK(fram.loads == 1);
  CHECK(m.engine_config.scene_a == 2);
  CHECK(m.engine_config.scene_b == 3);
  CHECK(!error_any(&m.engine_state)); // nothing went wrong, so no error shown
}

// The point of the whole exercise.
TEST_CASE(two_instances_run_independent_clocks)
{
  BmcvInstance a, b;
  bmcv_instance_init(&a, NULL, 0);
  bmcv_instance_init(&b, NULL, 0);

  InputSample sa, sb;
  memset(&sa, 0, sizeof(sa));
  memset(&sb, 0, sizeof(sb));
  sa.slider_raw = SLIDER_MIN_VALUE;
  sb.slider_raw = SLIDER_MIN_VALUE;

  // Both boot with input 0 as INPUT_CLOCK. Feed a at 2Hz and b at 1Hz.
  uint8_t clock_ch = a.hw_setup->input_adc_idx[0];

  uint32_t t = 0;
  for (int i = 0; i < 40; i++)
  {
    t += MS(125);

    sa.cv_trig[clock_ch] = 1;
    tick(&a, &sa, t);
    sa.cv_trig[clock_ch] = 0;

    if (i % 2 == 1) // half as often
    {
      sb.cv_trig[clock_ch] = 1;
      tick(&b, &sb, t);
      sb.cv_trig[clock_ch] = 0;
    }
    else
    {
      tick(&b, &sb, t);
    }
  }

  CHECK_NEAR(a.engine_state.clock.beat_freq_smooth, 2.0, 0.05);
  CHECK_NEAR(b.engine_state.clock.beat_freq_smooth, 1.0, 0.05);
  CHECK(a.engine_state.clock.beat_counter != b.engine_state.clock.beat_counter);
}

TEST_CASE(two_instances_do_not_share_error_flags)
{
  BmcvInstance a, b;
  bmcv_instance_init(&a, NULL, 0);
  bmcv_instance_init(&b, NULL, 0);

  CHECK(!error_any(&a.engine_state));
  CHECK(!error_any(&b.engine_state));

  error_set(&a.engine_state, 3);
  CHECK(error_get(&a.engine_state, 3));
  CHECK(!error_get(&b.engine_state, 3));
  CHECK(!error_any(&b.engine_state));

  error_clear(&a.engine_state);
  error_set(&b.engine_state, 5);
  CHECK(!error_any(&a.engine_state));
  CHECK(error_get(&b.engine_state, 5));
}

TEST_CASE(two_instances_do_not_share_input_or_ui_state)
{
  BmcvInstance a, b;
  bmcv_instance_init(&a, NULL, 0);
  bmcv_instance_init(&b, NULL, 0);

  InputSample sa, sb;
  memset(&sa, 0, sizeof(sa));
  memset(&sb, 0, sizeof(sb));
  sa.slider_raw = 4000;
  sb.slider_raw = 6000;

  int8_t btn          = a.ux_setup->ctrl_buttons[SHIFT_STATE_QNT].button;
  sa.button_down[btn] = 1;
  sa.encoder_pos[0]   = 17;

  // Long enough for the QNT hold to cross UI_T_HOLD and latch the mode.
  for (uint32_t i = 1; i <= 200; i++)
  {
    tick(&a, &sa, MS(i));
    tick(&b, &sb, MS(i));
  }

  CHECK(a.ux.hw_state->slider_state == 4000);
  CHECK(b.ux.hw_state->slider_state == 6000);
  CHECK(a.ux.hw_state->button_state[btn] == 1);
  CHECK(b.ux.hw_state->button_state[btn] == 0);
  CHECK(a.ux.hw_state->encoder_state[0] == 17);
  CHECK(b.ux.hw_state->encoder_state[0] == 0);

  // Holding QNT put a into that mode; b must be untouched.
  CHECK(a.ui_state.shift_state == SHIFT_STATE_QNT);
  CHECK(b.ui_state.shift_state == SHIFT_STATE_NONE);
}

TEST_CASE(two_instances_do_not_share_preset_storage)
{
  FakeFram fa, fb;
  memset(&fa, 0, sizeof(fa));
  memset(&fb, 0, sizeof(fb));
  PresetIo ioa = {.store = fake_store, .load = fake_load, .user = &fa};
  PresetIo iob = {.store = fake_store, .load = fake_load, .user = &fb};

  BmcvInstance a, b;
  bmcv_instance_init(&a, &ioa, 0);
  bmcv_instance_init(&b, &iob, 0);

  a.engine_config.scene_b = 1;
  CHECK(ux_preset_store(&a.ux, 0) == 1);

  CHECK(fa.stores == 1);
  CHECK(fb.stores == 0);
  CHECK(fa.occupied[0]);
  CHECK(!fb.occupied[0]);
}

// Each instance renders into its own framebuffer.
TEST_CASE(two_instances_do_not_share_the_led_framebuffer)
{
  BmcvInstance a, b;
  bmcv_instance_init(&a, NULL, 0);
  bmcv_instance_init(&b, NULL, 0);

  InputSample sa, sb;
  memset(&sa, 0, sizeof(sa));
  memset(&sb, 0, sizeof(sb));
  sa.slider_raw = SLIDER_MIN_VALUE;
  sb.slider_raw = SLIDER_MIN_VALUE;

  sa.button_down[a.ux_setup->ctrl_buttons[SHIFT_STATE_QNT].button] = 1;

  for (uint32_t i = 1; i <= 200; i++)
  {
    tick(&a, &sa, MS(i));
    tick(&b, &sb, MS(i));
  }

  int8_t qnt_led = a.ux_setup->ctrl_buttons[SHIFT_STATE_QNT].led;
  CHECK(qnt_led >= 0);

  // a is in QNT and b is not, so their framebuffers must differ somewhere.
  uint8_t differs = 0;
  for (uint8_t i = 0; i < LED_COUNT; i++)
  {
    const LedRgb* la = &a.engine_state.leds[i];
    const LedRgb* lb = &b.engine_state.leds[i];
    if (la->r != lb->r || la->g != lb->g || la->b != lb->b)
      differs = 1;
  }
  CHECK(differs);
}

int main(void)
{
  RUN_TEST(a_fresh_instance_comes_up_with_first_boot_defaults);
  RUN_TEST(a_stored_config_is_loaded_instead_of_the_defaults);
  RUN_TEST(two_instances_run_independent_clocks);
  RUN_TEST(two_instances_do_not_share_error_flags);
  RUN_TEST(two_instances_do_not_share_input_or_ui_state);
  RUN_TEST(two_instances_do_not_share_preset_storage);
  RUN_TEST(two_instances_do_not_share_the_led_framebuffer);
  return TESTKIT_SUMMARY();
}
