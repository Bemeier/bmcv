#include "fixture.h"
#include "scene.h"
#include "testkit.h"

TEST_CASE(same_scene_a_and_b_gives_it_full_weight)
{
  Fixture f;
  fixture_init(&f);
  f.engine_config.scene_a = 2;
  f.engine_config.scene_b = 2;

  compute_scenes_contribution(&f.ux);

  CHECK(f.engine_state.scenes_contribution[2] == 255);
  CHECK(f.engine_state.active_scene == 2);
  for (uint8_t s = 0; s < N_SCENES; s++)
  {
    if (s != 2)
      CHECK(f.engine_state.scenes_contribution[s] == 0);
  }
}

TEST_CASE(slider_at_max_fully_favors_scene_a)
{
  Fixture f;
  fixture_init(&f);
  f.engine_config.scene_a = 1;
  f.engine_config.scene_b = 3;
  f.hw_state.slider_state = SLIDER_MAX_VALUE;

  compute_scenes_contribution(&f.ux);

  CHECK(f.engine_state.scenes_contribution[1] == 255);
  CHECK(f.engine_state.scenes_contribution[3] == 0);
  CHECK(f.engine_state.active_scene == 1);
}

TEST_CASE(slider_at_min_fully_favors_scene_b)
{
  Fixture f;
  fixture_init(&f);
  f.engine_config.scene_a = 1;
  f.engine_config.scene_b = 3;
  f.hw_state.slider_state = SLIDER_MIN_VALUE;

  compute_scenes_contribution(&f.ux);

  CHECK(f.engine_state.scenes_contribution[1] == 0);
  CHECK(f.engine_state.scenes_contribution[3] == 255);
  CHECK(f.engine_state.active_scene == 3);
}

TEST_CASE(slider_midpoint_splits_roughly_evenly)
{
  Fixture f;
  fixture_init(&f);
  f.engine_config.scene_a = 1;
  f.engine_config.scene_b = 3;
  f.hw_state.slider_state = (SLIDER_MIN_VALUE + SLIDER_MAX_VALUE) / 2;

  compute_scenes_contribution(&f.ux);

  CHECK(f.engine_state.scenes_contribution[1] + f.engine_state.scenes_contribution[3] == 255);
  CHECK(f.engine_state.scenes_contribution[1] > 100 && f.engine_state.scenes_contribution[1] < 155);
}

TEST_CASE(momentary_scene_overrides_the_crossfade)
{
  Fixture f;
  fixture_init(&f);
  f.engine_config.scene_a         = 1;
  f.engine_config.scene_b         = 3;
  f.engine_state.momentary_scene  = 4;

  compute_scenes_contribution(&f.ux);

  CHECK(f.engine_state.scenes_contribution[4] == 255);
  CHECK(f.engine_state.active_scene == 4);
}

int main(void)
{
RUN_TEST(same_scene_a_and_b_gives_it_full_weight);
RUN_TEST(slider_at_max_fully_favors_scene_a);
RUN_TEST(slider_at_min_fully_favors_scene_b);
RUN_TEST(slider_midpoint_splits_roughly_evenly);
RUN_TEST(momentary_scene_overrides_the_crossfade);
  return TESTKIT_SUMMARY();
}
