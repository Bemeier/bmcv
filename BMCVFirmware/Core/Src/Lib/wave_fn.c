#include "wave_fn.h"
#include "helpers.h"
#include "wavetables.h"

float wavetable_lookup(float phase, float shape)
{
  // Wrap phase [0,1) -> index into N samples
  float fidx = phase * N;
  int i0     = (int) fidx;
  int i1     = (i0 + 1) & (N - 1); // wrap around cheaply if N is power-of-two
  float t    = fidx - i0;

  float sidx = ((shape + 1.0) * 0.5) * M;
  int k0     = (int) floorf(sidx); // always rounds down
  float u    = sidx - (float) k0;  // fractional part ∈ [0,1)
  k0         = (k0 % M + M) % M;   // safe modulo for negatives
  int k1     = (k0 + 1) % M;

  // Fetch 4 samples (two phase neighbors from two slices)
  float a0 = shape_table[k0][i0];
  float a1 = shape_table[k0][i1];
  float b0 = shape_table[k1][i0];
  float b1 = shape_table[k1][i1];

  // Phase interpolation within each slice
  float ya = a0 + t * (a1 - a0);
  float yb = b0 + t * (b1 - b0);

  return ya + u * (yb - ya);
}

float wave_fn(float phase, float shape, float mod)
{
  // TODO: optimize code
  // Clamp inputs
  phase = fmodf(phase, 1.0f); // or phase -= floor(phase);
  if (phase < 0)
    phase += 1.0f;
  shape = fclamp(shape, -1.0f, 1.0f);
  mod   = fclamp(mod, -1.0f, 1.0f);

  // --- 1. Phase warping for mod (asymmetry / "attack/decay" timing) ---
  // mod > 0  -> slower rise / faster fall  (like saw-ish or PWM)
  // mod < 0  -> faster rise / slower fall
  // mod == 0 -> symmetric
  float warp = 0.5f + mod * 0.45f; // Keep some movement even at extremes (~0.05–0.95)

  float warped_phase;
  if (phase < warp)
  {
    warped_phase = phase / (2.0f * warp); // stretch/compress first half
  }
  else
  {
    warped_phase = 0.5f + (phase - warp) / (2.0f * (1.0f - warp));
  }

  // Optional: soft knee on warp transitions for extra smoothness
  warped_phase = smoothstep_edge(0.0f, 1.0f, warped_phase); // if too angular

  float p = warped_phase; // Use this for all base shapes below

  // --- 2. Base shapes (all procedural) ---
  float sine = sin(2.0f * M_PI * p);         // -1 to 1
  float tri  = 4.0f * fabs(p - 0.5f) - 1.0f; // Triangle (linear)
  float saw  = 2.0f * p - 1.0f;              // Rising saw
  // Square with soft edges (so mod has slope to work with)
  float sq = (p < 0.5f) ? 1.0f : -1.0f;
  sq       = tanh(8.0f * sq); // or use a small sigmoid for rounded corners

  // --- 3. Shape morphing ( -1 to 1 ) ---
  // Divide into regions for natural interpolation:
  // -1.0 .. -0.33 : Saw -> Triangle
  // -0.33 ..  0.33: Triangle <-> Sine  (most "musical" center)
  //  0.33 ..  1.0 : Sine -> Square

  float out;
  if (shape <= -0.333f)
  {
    // Saw -> Triangle
    float t = (shape + 1.0f) / 0.667f; // 0..1
    out     = lerp(saw, tri, t);
  }
  else if (shape <= 0.333f)
  {
    // Triangle <-> Sine
    float t = (shape + 0.333f) / 0.666f; // 0..1
    out     = lerp(tri, sine, t);
  }
  else
  {
    // Sine -> Square
    float t = (shape - 0.333f) / 0.667f; // 0..1
    out     = lerp(sine, sq, t);
  }

  // Optional final shaping for more musicality / to tame extremes
  out = tanh(out * 1.2f); // gentle saturation, keeps peaks near ±1

  return out;
}
