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

// ENC_SR_LENGTH is dark on channels whose shape has no pattern length, and the
// encoder does nothing there too rather than silently moving a hidden setting.
TEST_CASE(pattern_length_only_moves_on_a_stepped_shape)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state = SHIFT_STATE_STA; // ENC_SR_LENGTH

  f.engine_config.channel_state[4].shape_mode    = SHAPE_LFO;
  f.engine_config.channel_state[4].sr_length_idx = 0;
  fixture_encoder(&f, ch_enc(&f, 4), +1);
  CHECK(f.engine_config.channel_state[4].sr_length_idx == 0);

  f.engine_config.channel_state[4].shape_mode = SHAPE_STEPPED;
  fixture_encoder(&f, ch_enc(&f, 4), +1);
  CHECK(f.engine_config.channel_state[4].sr_length_idx == 1);
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

// In MON the channel button is picking a routing source, so the same hand
// holding it must not also drive the encoder into the amp mode.
TEST_CASE(holding_the_button_in_mon_blocks_the_amp_mode_encoder)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state = SHIFT_STATE_MON;

  ChannelInputAmpMode start = f.engine_config.channel_state[6].input_amp_mode;

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
  return TESTKIT_SUMMARY();
}
