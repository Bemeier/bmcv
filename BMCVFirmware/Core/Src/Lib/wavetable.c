#include "wavetable.h"
#include "helpers.h"
#include "wavetables.h"

float wavetable_lookup(float phase, float shape)
{
  // Wrap phase [0,1) -> index into N samples. Both indices are masked, not
  // just the neighbour: i0 used to be trusted to the caller's contract, so a
  // phase that was not in [0,1) - a NaN casts to INT_MIN - indexed the table
  // from an arbitrary address. One AND buys a wrong sample instead of a wild
  // read on a path with eight callers a tick.
  float fidx = phase * N;
  int i0     = ((int) fidx) & (N - 1);
  int i1     = (i0 + 1) & (N - 1); // wrap around cheaply if N is power-of-two
  float t    = fidx - (float) (int) fidx;

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
