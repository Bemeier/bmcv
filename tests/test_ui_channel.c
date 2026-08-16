// ui_channel.c had no test of its own. It is the channel row's whole
// dispatch: a button switch and an encoder switch, both keyed off the mode
// table, so each arm is a different setting reachable from the same knob.
#include "channel.h"
#include "config.h"
#include "fixture.h"
#include "testkit.h"
#include "ui_mode.h"
#include "ux_state.h"

static int8_t ch_btn(Fixture* f, uint8_t id) { return f->ux_setup->channels[id].button; }
static int8_t ch_enc(Fixture* f, uint8_t id) { return f->ux_setup->channels[id].encoder; }

// CHB_MUTE_TOGGLE. On release rather than press, which is what makes it match
// every other momentary action in the UI.
TEST_CASE(the_channel_button_toggles_mute_in_mut)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state = SHIFT_STATE_MUT;

  CHECK(!f.ui_state.muted[2]);

  fixture_press(&f, ch_btn(&f, 2), MS(30));
  CHECK(f.ui_state.muted[2]);

  fixture_press(&f, ch_btn(&f, 2), MS(30));
  CHECK(!f.ui_state.muted[2]);
}

// ENC_MUTE is absolute where the button is a toggle: left always mutes and
// right always unmutes, so a row can be muted by feel without reading each
// channel's current state. Turning the same way twice must not undo itself.
TEST_CASE(the_mut_encoder_is_absolute_not_a_toggle)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state = SHIFT_STATE_MUT;

  fixture_encoder(&f, ch_enc(&f, 3), -1);
  CHECK(f.ui_state.muted[3]);

  fixture_encoder(&f, ch_enc(&f, 3), -1);
  CHECK(f.ui_state.muted[3]);

  fixture_encoder(&f, ch_enc(&f, 3), +1);
  CHECK(!f.ui_state.muted[3]);

  fixture_encoder(&f, ch_enc(&f, 3), +1);
  CHECK(!f.ui_state.muted[3]);
}

// step_setting clamps rather than wraps. These are short lists of unrelated
// states, and rolling off "off" into the most extreme setting is the largest
// change on the page and never one anyone means to make.
TEST_CASE(a_discrete_setting_clamps_at_both_ends_instead_of_wrapping)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state = SHIFT_STATE_SYS; // ENC_SHAPE

  for (int i = 0; i < SHAPE_MODE_COUNT + 4; i++)
    fixture_encoder(&f, ch_enc(&f, 0), +1);
  CHECK(f.engine_config.channel_state[0].shape_mode == SHAPE_MODE_COUNT - 1);

  for (int i = 0; i < SHAPE_MODE_COUNT + 4; i++)
    fixture_encoder(&f, ch_enc(&f, 0), -1);
  CHECK(f.engine_config.channel_state[0].shape_mode == 0);
}

// One detent per tick however far the encoder was spun in it: the delta is
// clamped to +/-1 before it is applied, so a fast spin steps the list at the
// same rate as a slow one rather than skipping entries.
TEST_CASE(a_large_encoder_delta_still_steps_one_setting)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state = SHIFT_STATE_SYS;

  int8_t start = f.engine_config.channel_state[1].shape_mode;
  fixture_encoder(&f, ch_enc(&f, 1), +9);
  CHECK(f.engine_config.channel_state[1].shape_mode == start + 1);
}

// ENC_STEPPED_LENGTH is dark on channels whose shape has no pattern length, and the
// encoder does nothing there too rather than silently moving a hidden setting.
TEST_CASE(pattern_length_only_moves_on_a_stepped_shape)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state = SHIFT_STATE_STA; // ENC_STEPPED_LENGTH

  f.engine_config.channel_state[4].shape_mode    = SHAPE_LFO;
  f.engine_config.channel_state[4].st_length_idx = 0;
  fixture_encoder(&f, ch_enc(&f, 4), +1);
  CHECK(f.engine_config.channel_state[4].st_length_idx == 0);

  f.engine_config.channel_state[4].shape_mode = SHAPE_STEPPED;
  fixture_encoder(&f, ch_enc(&f, 4), +1);
  CHECK(f.engine_config.channel_state[4].st_length_idx == 1);
}

// Turning the pattern length records an edit, which is what tells the engine to
// apply the new length immediately rather than waiting for the cycle to wrap.
TEST_CASE(turning_pattern_length_records_a_channel_edit)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state                      = SHIFT_STATE_STA;
  f.engine_config.channel_state[5].shape_mode = SHAPE_STEPPED;

  fixture_tick(&f, MS(50));
  f.engine_state.channels_last_delta[5] = 0;

  fixture_encoder(&f, ch_enc(&f, 5), +1);
  CHECK(f.engine_state.channels_last_delta[5] != 0);
}

// In MIX the channel button is picking a routing source, so the same hand
// holding it must not also drive the encoder into the amp mode.
TEST_CASE(holding_the_button_in_mon_blocks_the_amp_mode_encoder)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state = SHIFT_STATE_MIX;

  int8_t start = f.engine_config.channel_state[6].input_amp_mode;

  fixture_hold(&f, ch_btn(&f, 6), MS(30));
  fixture_encoder(&f, ch_enc(&f, 6), +1);
  CHECK(f.engine_config.channel_state[6].input_amp_mode == start);

  fixture_release(&f, ch_btn(&f, 6));
  fixture_encoder(&f, ch_enc(&f, 6), +1);
  CHECK(f.engine_config.channel_state[6].input_amp_mode != start);
}

// One encoder must move one channel. These are eight identical strips and a
// crossed wire between them would be invisible in any single-channel test.
TEST_CASE(an_encoder_moves_only_its_own_channel)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state = SHIFT_STATE_SAV; // ENC_CLAMP

  fixture_encoder(&f, ch_enc(&f, 7), +1);

  CHECK(f.engine_config.channel_state[7].clamp_mode == 1);
  for (uint8_t c = 0; c < N_CHANNELS - 1; c++)
    CHECK(f.engine_config.channel_state[c].clamp_mode == 0);
}

// How many detents of fine adjust it takes to walk from one grid entry to the
// next, starting from `from`.
static int detents_to_cross(Fixture* f, int16_t from, int16_t to)
{
  f->ui_state.shift_state         = SHIFT_STATE_NONE;
  f->engine_config.selected_param = CH_PARAM_FRQ;
  fixture_set_param(f, 0, 0, CH_PARAM_FRQ, from);

  int16_t dir = to > from ? +1 : -1;
  int n       = 0;
  fixture_hold(f, ch_btn(f, 0), MS(10)); // the fine-adjust modifier

  while (n < 500)
  {
    int16_t before = f->engine_config.channel_state[0].params[0][CH_PARAM_FRQ];
    if ((dir > 0 && before >= to) || (dir < 0 && before <= to))
      break;
    fixture_encoder(f, ch_enc(f, 0), dir);
    n++;
    if (f->engine_config.channel_state[0].params[0][CH_PARAM_FRQ] == before)
      break; // clamped, not moving
  }

  fixture_release(f, ch_btn(f, 0));
  return n;
}

// The frequency grid is 1/f-linear, so a flat fine-adjust step meant something
// different everywhere on it: half a gap near 1x, where two detents crossed to
// the next ratio, and 0.2% of one between 1/64 and 1/128, where crossing took
// 250. A gap-relative step costs the same handful of detents per ratio wherever
// you are, which is also what makes the off-grid wash behave the same at both
// ends.
TEST_CASE(fine_adjust_costs_the_same_detents_per_ratio_across_the_grid)
{
  Fixture f;
  fixture_init(&f);

  int near_one = detents_to_cross(&f, 0, 64);          // 1x    -> 5/4
  int near_top = detents_to_cross(&f, 7905, 16065);    // 32x   -> 64x
  int near_bot = detents_to_cross(&f, -32385, -16065); // 1/128 -> 1/64

  CHECK(near_one > 0 && near_one <= FREQ_FINE_STEPS_PER_GAP + 1);
  CHECK(near_top > 0 && near_top <= FREQ_FINE_STEPS_PER_GAP + 1);
  CHECK(near_bot > 0 && near_bot <= FREQ_FINE_STEPS_PER_GAP + 1);
}

// Fine adjust used to clamp nothing, so twelve detents from the top of the grid
// overflowed int16_t and wrapped to -32768: the fastest setting on the panel
// snapping straight past the slowest.
TEST_CASE(fine_adjust_clamps_at_the_ends_of_the_frequency_grid)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state         = SHIFT_STATE_NONE;
  f.engine_config.selected_param = CH_PARAM_FRQ;

  fixture_set_param(&f, 0, 0, CH_PARAM_FRQ, 16065); // 64x, the top entry
  fixture_hold(&f, ch_btn(&f, 0), MS(10));
  for (int i = 0; i < 40; i++)
    fixture_encoder(&f, ch_enc(&f, 0), +1);
  fixture_release(&f, ch_btn(&f, 0));
  CHECK(f.engine_config.channel_state[0].params[0][CH_PARAM_FRQ] == 16065);

  fixture_set_param(&f, 0, 0, CH_PARAM_FRQ, -32385); // 1/128, the bottom entry
  fixture_hold(&f, ch_btn(&f, 0), MS(10));
  for (int i = 0; i < 40; i++)
    fixture_encoder(&f, ch_enc(&f, 0), -1);
  fixture_release(&f, ch_btn(&f, 0));
  CHECK(f.engine_config.channel_state[0].params[0][CH_PARAM_FRQ] == -32385);
}

// The other five parameters are linear and keep the flat step - the gap-
// relative one is meaningless off the frequency grid.
TEST_CASE(fine_adjust_on_a_linear_parameter_keeps_its_flat_step)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state         = SHIFT_STATE_NONE;
  f.engine_config.selected_param = CH_PARAM_AMP;
  fixture_set_param(&f, 0, 0, CH_PARAM_AMP, 0);

  fixture_hold(&f, ch_btn(&f, 0), MS(10));
  fixture_encoder(&f, ch_enc(&f, 0), +1);
  fixture_release(&f, ch_btn(&f, 0));

  CHECK(f.engine_config.channel_state[0].params[0][CH_PARAM_AMP] == 32);
}

// The reset guard asks whether the press spanned an encoder edit, by comparing
// how long the button has been down against how long ago the last edit was.
// That only works if both are measured from the same instant, and held_us used
// to stop accumulating one tick early - so an edit made on the very tick the
// button went down looked older than the press that contained it, and the
// release wiped the value that press had just adjusted.
//
// Nobody presses and turns within one tick by hand, but the web panel's
// shift-wheel does exactly that: it asserts the push and feeds the detent in
// the same gesture, so fine adjust there always ended in a reset.
TEST_CASE(an_edit_on_the_press_tick_still_blocks_the_reset)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state         = SHIFT_STATE_NONE;
  f.engine_config.selected_param = CH_PARAM_AMP;
  fixture_set_param(&f, 0, 0, CH_PARAM_AMP, 4096);

  // Push and detent land together, as the panel sends them.
  f.hw_state.button_state[ch_btn(&f, 0)]  = 1;
  f.hw_state.encoder_delta[ch_enc(&f, 0)] = +1;
  fixture_tick(&f, MS(1));
  CHECK(f.engine_config.channel_state[0].params[0][CH_PARAM_AMP] == 4096 + 32);

  for (int i = 0; i < 140; i++)
    fixture_tick(&f, MS(1));
  fixture_release(&f, ch_btn(&f, 0));

  CHECK(f.engine_config.channel_state[0].params[0][CH_PARAM_AMP] == 4096 + 32);
}

int main(void)
{
  RUN_TEST(the_channel_button_toggles_mute_in_mut);
  RUN_TEST(the_mut_encoder_is_absolute_not_a_toggle);
  RUN_TEST(a_discrete_setting_clamps_at_both_ends_instead_of_wrapping);
  RUN_TEST(a_large_encoder_delta_still_steps_one_setting);
  RUN_TEST(pattern_length_only_moves_on_a_stepped_shape);
  RUN_TEST(turning_pattern_length_records_a_channel_edit);
  RUN_TEST(holding_the_button_in_mon_blocks_the_amp_mode_encoder);
  RUN_TEST(an_encoder_moves_only_its_own_channel);
  RUN_TEST(fine_adjust_costs_the_same_detents_per_ratio_across_the_grid);
  RUN_TEST(fine_adjust_clamps_at_the_ends_of_the_frequency_grid);
  RUN_TEST(fine_adjust_on_a_linear_parameter_keeps_its_flat_step);
  RUN_TEST(an_edit_on_the_press_tick_still_blocks_the_reset);
  return TESTKIT_SUMMARY();
}
