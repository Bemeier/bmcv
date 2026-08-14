// ui_scene.c had no test of its own. It is the whole scene-button dispatch:
// one switch over the mode table's scene_btn_action, so every arm of it is a
// different user-visible behaviour on the same seven buttons.
#include "config.h"
#include "error.h"
#include "fixture.h"
#include "testkit.h"
#include "ui_mode.h"
#include "ux_state.h"
#include <string.h>

static int8_t scene_btn(Fixture* f, uint8_t id) { return f->ux_setup->scenes[id].button; }

// SCN_MOMENTARY, the no-shift default: hold to audition a scene, release to
// drop it. momentary_scene is -1 rather than 0 when nothing is held, because
// scene_compute_contribution reads 0 as "scene 0 is being held".
TEST_CASE(holding_a_scene_button_makes_it_momentary_and_releasing_clears_it)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state = SHIFT_STATE_NONE;

  CHECK(f.ui_state.momentary_scene == -1);

  fixture_hold(&f, scene_btn(&f, 3), UI_T_DEBOUNCE + MS(20));
  CHECK(f.ui_state.momentary_scene == 3);

  fixture_release(&f, scene_btn(&f, 3));
  CHECK(f.ui_state.momentary_scene == -1);
}

// A second scene pressed while the first is still held must not steal the
// slot, or releasing the first would leave the second stuck on.
TEST_CASE(a_second_momentary_press_does_not_displace_the_first)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state = SHIFT_STATE_NONE;

  fixture_hold(&f, scene_btn(&f, 1), UI_T_DEBOUNCE + MS(20));
  CHECK(f.ui_state.momentary_scene == 1);

  fixture_hold(&f, scene_btn(&f, 2), UI_T_DEBOUNCE + MS(20));
  CHECK(f.ui_state.momentary_scene == 1);

  fixture_release(&f, scene_btn(&f, 2));
  CHECK(f.ui_state.momentary_scene == 1);
}

// SCN_SET_XFADE. STA and STB run the same code path and differ only by the
// mode table's xfade_end, which is exactly the thing worth pinning down.
TEST_CASE(sta_wires_a_scene_to_the_a_end_and_stb_to_the_b_end)
{
  Fixture f;
  fixture_init(&f);

  f.ui_state.shift_state = SHIFT_STATE_STA;
  fixture_press(&f, scene_btn(&f, 5), MS(30));
  CHECK(f.engine_config.scene_a == 5);
  CHECK(f.engine_config.scene_b == 0);

  f.ui_state.shift_state = SHIFT_STATE_STB;
  fixture_press(&f, scene_btn(&f, 6), MS(30));
  CHECK(f.engine_config.scene_a == 5);
  CHECK(f.engine_config.scene_b == 6);
}

// SCN_INPUT_MODE cycles one input's role and wraps, so a user can get back to
// where they started by pressing the same button INPUT_MODE_COUNT times.
TEST_CASE(sys_cycles_an_input_mode_and_wraps_round)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state = SHIFT_STATE_SYS;

  int8_t start = f.engine_config.input_mode[1];

  fixture_press(&f, scene_btn(&f, 1), MS(30));
  CHECK(f.engine_config.input_mode[1] == (start + 1) % INPUT_MODE_COUNT);

  for (int i = 1; i < INPUT_MODE_COUNT; i++)
    fixture_press(&f, scene_btn(&f, 1), MS(30));

  CHECK(f.engine_config.input_mode[1] == start);
}

// There are seven scene buttons and four inputs. In the modes where the scene
// row addresses inputs, the last three have nothing to address - and indexing
// input_mode[] with them would run off the end of the array.
TEST_CASE(scene_buttons_past_the_input_count_do_nothing_in_an_input_mode)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state = SHIFT_STATE_SYS;

  // The whole config, not just input_mode[]: without the guard the write lands
  // one past the end of that array, which is inside the next member rather than
  // outside the struct - so comparing only input_mode[] passes while memory
  // that matters has been changed. Sanitizers catch the index; this catches the
  // effect, in the ordinary build too.
  EngineConfig before = f.engine_config;

  for (uint8_t s = N_INPUTS; s < N_SCENES; s++)
    fixture_press(&f, scene_btn(&f, s), MS(30));

  CHECK(memcmp(&before, &f.engine_config, sizeof(EngineConfig)) == 0);
}

// MUT gives the scene row no action at all. Pressing one must then be inert
// rather than falling through to whatever the previous mode did.
TEST_CASE(a_mode_with_no_scene_action_leaves_the_scene_row_inert)
{
  Fixture f;
  fixture_init(&f);

  f.ui_state.shift_state = SHIFT_STATE_MUT;
  fixture_press(&f, scene_btn(&f, 2), MS(30));

  CHECK(f.engine_config.scene_a == 0);
  CHECK(f.engine_config.scene_b == 0);
  CHECK(f.ui_state.momentary_scene == -1);
}

int main(void)
{
  RUN_TEST(holding_a_scene_button_makes_it_momentary_and_releasing_clears_it);
  RUN_TEST(a_second_momentary_press_does_not_displace_the_first);
  RUN_TEST(sta_wires_a_scene_to_the_a_end_and_stb_to_the_b_end);
  RUN_TEST(sys_cycles_an_input_mode_and_wraps_round);
  RUN_TEST(scene_buttons_past_the_input_count_do_nothing_in_an_input_mode);
  RUN_TEST(a_mode_with_no_scene_action_leaves_the_scene_row_inert);
  return TESTKIT_SUMMARY();
}
