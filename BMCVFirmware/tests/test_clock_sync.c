#include "clock_sync.h"
#include "testkit.h"

static void reset_clock(void) { Clock_Init(); }

TEST_CASE(beat_freq_smooth_converges_to_the_pulse_rate)
{
  reset_clock();

  const uint32_t dt_pulse = 125000; // 4 pulses/250ms beat -> 2.0 Hz beat rate
  uint32_t t              = 0;
  for (int i = 0; i < 30; i++)
  {
    t += dt_pulse;
    Clock_Trigger(t);
  }

  CHECK_NEAR(g_clk.beat_freq_smooth, 2.0, 0.02);
  CHECK(g_clk.have_beat);
}

TEST_CASE(have_beat_drops_after_clock_loss)
{
  reset_clock();

  Clock_Trigger(0);
  Clock_Trigger(125000);
  CHECK(g_clk.have_beat);

  Clock_Poll(125000 + 4 * 125000 + 1);

  CHECK(!g_clk.have_beat);
}

int main(void)
{
RUN_TEST(beat_freq_smooth_converges_to_the_pulse_rate);
RUN_TEST(have_beat_drops_after_clock_loss);
  return TESTKIT_SUMMARY();
}
