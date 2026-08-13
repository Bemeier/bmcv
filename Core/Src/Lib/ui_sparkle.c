#include "ui_sparkle.h"
#include "hw_setup.h"
#include "panel_layout.h"
#include <math.h>

static_assert(LED_COUNT == 21, "the field samples panel_led_pos, which is per-LED");

// One lattice point's value, 0..1.
//
// A hash rather than a table. The stepped random generator's tables were the
// obvious thing to reuse and do not fit: that generator is one-dimensional and
// locked to a channel's own phase, where a field needs a value for an (x, y, t)
// triple and no period at all. Integer mixing gives that in six lines and costs
// less than the lookup would.
static float lattice(int32_t x, int32_t y, int32_t t)
{
  uint32_t h = (uint32_t) x * 374761393u + (uint32_t) y * 668265263u + (uint32_t) t * 2246822519u;
  h ^= h >> 13;
  h *= 1274126177u;
  h ^= h >> 16;
  return (float) h * (1.0f / 4294967296.0f);
}

// Ken Perlin's second smoothstep. Zero first *and* second derivative at both
// ends, which matters here: with the plain cubic the field's rate of change
// jumps as a sample crosses a cell boundary, and on a slow pulse that reads as
// a tick.
static float smootherstep(float t) { return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f); }

static float lerp(float a, float b, float t) { return a + (b - a) * t; }

// Value noise in three dimensions: the eight lattice corners around the sample,
// interpolated. Gradient noise would avoid the faint axis alignment value noise
// has, and at a resolution of eight cells sampled by twenty-one lights there is
// nothing there to see.
static float noise3(float x, float y, float t)
{
  float fx = floorf(x), fy = floorf(y), ft = floorf(t);
  int32_t ix = (int32_t) fx, iy = (int32_t) fy, it = (int32_t) ft;

  float sx = smootherstep(x - fx);
  float sy = smootherstep(y - fy);
  float st = smootherstep(t - ft);

  float x00 = lerp(lattice(ix, iy, it), lattice(ix + 1, iy, it), sx);
  float x10 = lerp(lattice(ix, iy + 1, it), lattice(ix + 1, iy + 1, it), sx);
  float x01 = lerp(lattice(ix, iy, it + 1), lattice(ix + 1, iy, it + 1), sx);
  float x11 = lerp(lattice(ix, iy + 1, it + 1), lattice(ix + 1, iy + 1, it + 1), sx);

  return lerp(lerp(x00, x10, sy), lerp(x01, x11, sy), st);
}

// The once-a-period flash, 0..1: every candidate full on together, whatever the
// field is doing. Zero for the share of the period the sparkle has to itself.
//
// A bump rather than a gate, so it arrives and leaves without a step. It has to
// reach exactly 1, because reaching the top of the scale is its whole job - it
// is the moment that says "all of these", which a sparse field on its own never
// promises.
static float flash(uint32_t now_us)
{
  // Guarded rather than left to the phase test below: at a duty of 1.0 the
  // width the ramps divide by is zero, and the compiler folds this whole
  // function away rather than carrying a division that can never be reached.
  if (MARK_SPARKLE_DUTY >= 1.0f)
    return 0.0f;

  float phase = (float) (now_us % MARK_SPARKLE_PERIOD) / (float) MARK_SPARKLE_PERIOD;
  if (phase < MARK_SPARKLE_DUTY)
    return 0.0f;

  float u = (phase - MARK_SPARKLE_DUTY) / (1.0f - MARK_SPARKLE_DUTY);
  if (u < MARK_SPARKLE_FLASH_EDGE)
    return smootherstep(u / MARK_SPARKLE_FLASH_EDGE);
  if (u > 1.0f - MARK_SPARKLE_FLASH_EDGE)
    return smootherstep((1.0f - u) / MARK_SPARKLE_FLASH_EDGE);
  return 1.0f;
}

float ui_sparkle_level(uint32_t now_us, int16_t led, SparkleKind kind)
{
  if (led < 0 || led >= LED_COUNT)
    return 0.0f;

  // A held source is a gesture in progress: it does not get to drop a
  // destination, so its field sits on a floor where the mark's does not.
  float floor = kind == SPARKLE_TARGET ? TARGET_SPARKLE_FLOOR : MARK_SPARKLE_FLOOR;

  PanelPoint p = panel_led_pos[led];
  float gx     = (p.x - MARK_FIELD_X0) / (MARK_FIELD_X1 - MARK_FIELD_X0) * (float) MARK_FIELD_COLS;
  float gy     = (p.y - MARK_FIELD_Y0) / (MARK_FIELD_Y1 - MARK_FIELD_Y0) * (float) MARK_FIELD_ROWS;

  // Straight from the microsecond counter, so the field keeps running rather
  // than restarting - the panel looks like one surface, not like a loop being
  // replayed. It steps discontinuously once when the counter wraps, every 71
  // minutes, which in a sparkle is not something there is a way to notice.
  float secs = (float) now_us * 1e-6f;

  // Sampled progressively further back along x as time runs, which is the same
  // thing as the field sliding forward: whatever was at the left arrives at the
  // right MARK_FIELD_COLS / MARK_SPARKLE_DRIFT seconds later. Subtracted from
  // the sample point rather than added to it, or the panel travels right to
  // left.
  gx -= secs * MARK_SPARKLE_DRIFT;

  // And the third axis advances regardless, so what crosses the panel is not
  // the same pattern being towed past over and over.
  float t = secs * MARK_SPARKLE_HZ;

  // Value noise clusters around its middle and never reaches either end, so it
  // is stretched to the full range before being peaked - without this the floor
  // and the contrast are both operating on the middle third of a scale.
  float n = noise3(gx, gy, t);
  n       = smootherstep(fminf(fmaxf((n - MARK_SPARKLE_GATE) / (MARK_SPARKLE_GATE_TOP - MARK_SPARKLE_GATE), 0.0f), 1.0f));
  n       = powf(n, MARK_SPARKLE_CONTRAST);

  // Whichever is brighter. The flash does not scale the field or interrupt it -
  // it passes over the top of it, so a light already near the peak simply stays
  // there and nothing steps as the flash arrives or leaves.
  float level = floor + (1.0f - floor) * n;
  float lift  = flash(now_us);
  if (lift > level)
    level = lift;

  // Clamped, not assumed. smootherstep's polynomial form overshoots 1 by about
  // a millionth near the top of its range - float rounding in
  // t*t*t*(t*(t*6-15)+10), not a mistake in the coefficients - and the callers
  // scale this by a brightness ceiling, so a level a hair over 1 is a duty a
  // hair over the ceiling. Cheaper to hold the contract here than to make every
  // caller defend against it.
  return level > 1.0f ? 1.0f : level;
}
