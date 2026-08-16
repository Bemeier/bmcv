// Dumps stepped_shape() over a grid so the Python replica can be verified.
#include "stepped.h"
#include "stepped_table.h"
#include <stdio.h>

int main(void)
{
  const float shapes[] = {-1.0f, -0.6f, -0.13f, 0.0f, 0.31f, 0.7f, 1.0f};
  const float mods[]   = {-1.0f, -0.4f, 0.0f, 0.25f, 0.8f, 1.0f};
  for (int si = 0; si < 7; si++)
    for (int mi = 0; mi < 6; mi++)
      for (int li = 0; li < ST_LENGTH_COUNT; li++)
        for (int p = 0; p < 32; p++)
          printf("%.6f\n", stepped_shape((float) p / 32.0f, shapes[si], mods[mi], li, 0.0f));
  return 0;
}
