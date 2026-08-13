#include "fixture.h"
#include "testkit.h"
#include "ui_feedback.h"
#include "ui_select.h"

TEST_CASE(an_emitted_flash_is_active_until_its_duration_elapses)
{
  Fixture f;
  fixture_init(&f);

  CHECK(!ui_feedback_active(&f.ui_state, TGT_CHANNEL, 3, NULL));

  ui_feedback_emit(&f.ui_state, FB_WRITE, TGT_CHANNEL, 3);
  FeedbackKind k;
  CHECK(ui_feedback_active(&f.ui_state, TGT_CHANNEL, 3, &k));
  CHECK(k == FB_WRITE);

  ui_feedback_tick(&f.ui_state, UI_FB_DURATION - MS(1));
  CHECK(ui_feedback_active(&f.ui_state, TGT_CHANNEL, 3, NULL));

  ui_feedback_tick(&f.ui_state, MS(2));
  CHECK(!ui_feedback_active(&f.ui_state, TGT_CHANNEL, 3, NULL));
}

TEST_CASE(a_flash_is_scoped_to_its_own_element_and_kind)
{
  Fixture f;
  fixture_init(&f);

  ui_feedback_emit(&f.ui_state, FB_CLEAR, TGT_SCENE, 2);

  CHECK(ui_feedback_active(&f.ui_state, TGT_SCENE, 2, NULL));
  CHECK(!ui_feedback_active(&f.ui_state, TGT_SCENE, 3, NULL));
  CHECK(!ui_feedback_active(&f.ui_state, TGT_CHANNEL, 2, NULL));
}

// Used for whole-device events - a preset load, or an error - where no single
// element is responsible.
TEST_CASE(a_negative_id_flashes_every_element_of_that_kind)
{
  Fixture f;
  fixture_init(&f);

  ui_feedback_emit(&f.ui_state, FB_LOAD, TGT_SCENE, -1);

  for (int8_t s = 0; s < N_SCENES; s++)
  {
    CHECK(ui_feedback_active(&f.ui_state, TGT_SCENE, s, NULL));
  }
  CHECK(!ui_feedback_active(&f.ui_state, TGT_CHANNEL, 0, NULL));
}

TEST_CASE(re_emitting_the_same_flash_restarts_it_without_consuming_a_slot)
{
  Fixture f;
  fixture_init(&f);

  ui_feedback_emit(&f.ui_state, FB_WRITE, TGT_CHANNEL, 1);
  ui_feedback_tick(&f.ui_state, UI_FB_DURATION - MS(10));

  ui_feedback_emit(&f.ui_state, FB_WRITE, TGT_CHANNEL, 1);
  ui_feedback_tick(&f.ui_state, MS(20));

  // Would already have expired had the second emit not restarted it.
  CHECK(ui_feedback_active(&f.ui_state, TGT_CHANNEL, 1, NULL));
}

// More simultaneous actions than slots must not lose the newest feedback -
// that is the one the user is waiting to see.
TEST_CASE(overflowing_the_queue_keeps_the_most_recent_flash)
{
  Fixture f;
  fixture_init(&f);

  for (int8_t i = 0; i < UI_FB_SLOTS; i++)
  {
    ui_feedback_emit(&f.ui_state, FB_WRITE, TGT_CHANNEL, i);
    ui_feedback_tick(&f.ui_state, MS(10)); // stagger, so they age differently
  }

  ui_feedback_emit(&f.ui_state, FB_CLEAR, TGT_CHANNEL, 7);
  CHECK(ui_feedback_active(&f.ui_state, TGT_CHANNEL, 7, NULL));
}

TEST_CASE(each_action_class_has_its_own_colour)
{
  UiColor write = ui_feedback_color(FB_WRITE);
  UiColor clear = ui_feedback_color(FB_CLEAR);
  UiColor load  = ui_feedback_color(FB_LOAD);

  // Red is now errors only, and purple is clearing rather than "selectable".
  CHECK(write.h != clear.h);
  CHECK(load.h != write.h && load.h != clear.h);
  CHECK(clear.h != ui_feedback_color(FB_ERROR).h);
  // Brightness discipline: a confirmation is a flash *over* a page, so it has
  // to clear the brightness that page is drawn at - and stop one step above it,
  // since the step above is the whole vocabulary for "brighter than what is
  // underneath". Asserted against VAL_BASE rather than against a number, so
  // moving the base level cannot silently leave confirmations invisible.
  CHECK(write.v > VAL_BASE && clear.v > VAL_BASE && load.v > VAL_BASE);
  CHECK(write.v <= VAL_HIG && clear.v <= VAL_HIG && load.v <= VAL_HIG);
}

// Clearing one scene and clearing every scene are the same act, so they wear
// the same colour and are told apart by how long the flash lasts.
TEST_CASE(clearing_everywhere_flashes_for_longer_than_clearing_here)
{
  Fixture f;
  fixture_init(&f);

  ui_feedback_emit(&f.ui_state, FB_CLEAR, TGT_CHANNEL, 0);
  ui_feedback_emit(&f.ui_state, FB_CLEAR_ALL, TGT_CHANNEL, 1);

  CHECK(ui_feedback_color(FB_CLEAR).h == ui_feedback_color(FB_CLEAR_ALL).h);

  ui_feedback_tick(&f.ui_state, UI_FB_DURATION + MS(1));
  CHECK(!ui_feedback_active(&f.ui_state, TGT_CHANNEL, 0, NULL));
  CHECK(ui_feedback_active(&f.ui_state, TGT_CHANNEL, 1, NULL));
}

// Committing an action must produce feedback without the call site having to
// remember to ask for it.
TEST_CASE(committing_a_copy_flashes_the_destination)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state = SHIFT_STATE_CPY;

  ui_sel_press(&f.ux, TGT_CHANNEL, 0, 0);
  CHECK(!ui_feedback_active(&f.ui_state, TGT_CHANNEL, 5, NULL));

  ui_sel_press(&f.ux, TGT_CHANNEL, 5, 0);
  FeedbackKind k;
  CHECK(ui_feedback_active(&f.ui_state, TGT_CHANNEL, 5, &k));
  CHECK(k == FB_WRITE);
}

TEST_CASE(clearing_flashes_in_the_destructive_colour)
{
  Fixture f;
  fixture_init(&f);
  f.ui_state.shift_state = SHIFT_STATE_CLR;

  ui_sel_press(&f.ux, TGT_SCENE, 4, 0);

  FeedbackKind k;
  CHECK(ui_feedback_active(&f.ui_state, TGT_SCENE, 4, &k));
  CHECK(k == FB_CLEAR);
}

int main(void)
{
  RUN_TEST(an_emitted_flash_is_active_until_its_duration_elapses);
  RUN_TEST(a_flash_is_scoped_to_its_own_element_and_kind);
  RUN_TEST(a_negative_id_flashes_every_element_of_that_kind);
  RUN_TEST(re_emitting_the_same_flash_restarts_it_without_consuming_a_slot);
  RUN_TEST(overflowing_the_queue_keeps_the_most_recent_flash);
  RUN_TEST(each_action_class_has_its_own_colour);
  RUN_TEST(clearing_everywhere_flashes_for_longer_than_clearing_here);
  RUN_TEST(committing_a_copy_flashes_the_destination);
  RUN_TEST(clearing_flashes_in_the_destructive_colour);
  return TESTKIT_SUMMARY();
}
