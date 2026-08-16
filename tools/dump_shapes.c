// Renders every shape mode across SHP and MOD, for tools/gen_shape_figures.py
// to draw. Output is a grid of cycles, one line each:
//
//   <mode> <shp> <mod> : v0 v1 ... vN-1
//
// It calls the module's own shape functions rather than describing them, for
// the same reason gen_wavetable.py emits its plots from the table it generates:
// a picture of a shape the firmware does not have is worse than no picture.
#include "helpers.h"
#include "pwm.h"
#include "stepped.h"
#include "stepped_table.h"
#include "wavetable.h"
#include <stdint.h>
#include <stdio.h>

#define SAMPLES 256

// Eight steps, which is where the stepped mode's pattern is legible in a
// picture this size - long enough to show a phrase, short enough that the steps
// are wider than the trace.
#define STEPPED_LENGTH_IDX 4

static float value(const char* mode, float phase, float shape, float mod)
{
  if (mode[0] == 'l')
  {
    // Through phase_mod(), because that is how channel.c plays it: MOD warps
    // where the cycle spends its time, which is visible inside one cycle and
    // would have been missing from the picture.
    return wavetable_lookup(phase_mod(phase, mod), shape) / (float) INT16_MAX;
  }
  if (mode[0] == 'p')
  {
    return pwm_shape(phase, shape, mod);
  }

  StDrive d   = st_drive(shape, mod);
  StNorm norm = st_norm_exact(shape, mod, STEPPED_LENGTH_IDX);
  return stepped_shape_with(phase, shape, mod, STEPPED_LENGTH_IDX, &d, &norm);
}

static void emit(const char* mode, float shape, float mod)
{
  printf("%s %+.3f %+.3f :", mode, shape, mod);
  for (int i = 0; i < SAMPLES; i++)
  {
    printf(" %.4f", value(mode, (float) i / (float) SAMPLES, shape, mod));
  }
  printf("\n");
}

int main(void)
{
  const float knobs[] = {-1.0f, -0.5f, 0.0f, 0.5f, 1.0f};
  const char* modes[] = {"lfo", "stepped", "pwm"};

  for (int m = 0; m < 3; m++)
  {
    for (int s = 0; s < 5; s++)
    {
      for (int d = 0; d < 5; d++)
      {
        emit(modes[m], knobs[s], knobs[d]);
      }
    }
  }
  return 0;
}
