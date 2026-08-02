#include "clock_sync.h"
#include "testkit.h"

TEST_CASE(beat_freq_smooth_converges_to_the_pulse_rate)
{
  ClockState clk;
  Clock_Init(&clk);

  const uint32_t dt_pulse = 125000; // 4 pulses/250ms beat -> 2.0 Hz beat rate
  uint32_t t              = 0;
  for (int i = 0; i < 30; i++)
  {
    t += dt_pulse;
    Clock_Trigger(&clk, t);
  }

  CHECK_NEAR(clk.beat_freq_smooth, 2.0, 0.02);
  CHECK(clk.have_beat);
}

TEST_CASE(have_beat_drops_after_clock_loss)
{
  ClockState clk;
  Clock_Init(&clk);

  Clock_Trigger(&clk, 0);
  Clock_Trigger(&clk, 125000);
  CHECK(clk.have_beat);

  Clock_Poll(&clk, 125000 + 4 * 125000 + 1);

  CHECK(!clk.have_beat);
}

// Two clocks in one process must not see each other. This is what makes N
// module instances possible in a single VCV Rack patch.
TEST_CASE(clocks_are_independent)
{
  ClockState a, b;
  Clock_Init(&a);
  Clock_Init(&b);

  uint32_t t = 0;
  for (int i = 0; i < 30; i++)
  {
    t += 125000; // 2.0 Hz
    Clock_Trigger(&a, t);
  }

  t = 0;
  for (int i = 0; i < 30; i++)
  {
    t += 250000; // 1.0 Hz
    Clock_Trigger(&b, t);
  }

  CHECK_NEAR(a.beat_freq_smooth, 2.0, 0.02);
  CHECK_NEAR(b.beat_freq_smooth, 1.0, 0.02);
  CHECK(a.have_beat);
  CHECK(b.have_beat);

  // Losing one clock does not drop the other.
  Clock_Poll(&b, t + 4 * 250000 + 1);
  CHECK(!b.have_beat);
  CHECK(a.have_beat);
}

int main(void)
{
  RUN_TEST(beat_freq_smooth_converges_to_the_pulse_rate);
  RUN_TEST(have_beat_drops_after_clock_loss);
  RUN_TEST(clocks_are_independent);
  return TESTKIT_SUMMARY();
}
