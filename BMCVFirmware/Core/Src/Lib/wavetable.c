#include "wavetable.h"
#include "helpers.h" // IWYU pragma: keep - math.h
#include "wavetables.h"

float wavetable_lookup(float phase, float shape)
{
  // Wrap phase [0,1) -> index into WT_LEN samples. Both indices are masked, not
  // just the neighbour: i0 used to be trusted to the caller's contract, so a
  // phase that was not in [0,1) - a NaN casts to INT_MIN - indexed the table
  // from an arbitrary address. One AND buys a wrong sample instead of a wild
  // read on a path with eight callers a tick.
  float fidx = phase * WT_LEN;

  // Floor, not truncate. A cast rounds toward zero, so a phase just below zero
  // gave a *negative* fraction and the interpolation below extrapolated past
  // the sample instead of blending toward it - which can leave the range
  // entirely. Done with a compare rather than floorf() because this runs eight
  // times a tick and floorf is a call on this target.
  int base = (int) fidx;
  float t  = fidx - (float) base;
  if (t < 0.0f)
  {
    t += 1.0f;
    base -= 1;
  }

  int i0 = base & (WT_LEN - 1);
  int i1 = (i0 + 1) & (WT_LEN - 1); // wrap cheaply: WT_LEN is a power of two

  // The shape axis is a loop: shape +1 lands back on slice 0, which is where
  // the generator puts the same square it starts from. SHP wraps at the
  // parameter too, so turning the encoder forever cycles the shapes.
  float sidx = ((shape + 1.0f) * 0.5f) * WT_SLICES;
  int k0     = (int) floorf(sidx);                       // always rounds down
  float u    = sidx - (float) k0;                        // fractional part in [0,1)
  k0         = (k0 % WT_SLICES + WT_SLICES) % WT_SLICES; // safe modulo for negatives
  int k1     = (k0 + 1) % WT_SLICES;

  // Fetch 4 samples (two phase neighbours from two slices)
  float a0 = shape_table[k0][i0];
  float a1 = shape_table[k0][i1];
  float b0 = shape_table[k1][i0];
  float b1 = shape_table[k1][i1];

  // Phase interpolation within each slice
  float ya = a0 + t * (a1 - a0);
  float yb = b0 + t * (b1 - b0);

  // ...then across the shape axis. Every slice is anchored to the full swing
  // with no DC, and a blend of two of them is anchored the same way - see the
  // generator. That is why this interpolation cannot quietly lose amplitude,
  // which the hand-drawn table it replaced did, all the way to silence.
  return ya + u * (yb - ya);
}
