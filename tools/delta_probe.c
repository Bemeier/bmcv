// How much of a BmcvInstance actually changes between snapshots?
//
// Decides whether a MIDI transport has to send the whole struct every frame or
// can send only what moved. Run the sim, snapshot at a given rate, and report
// the byte delta and how many fixed-size chunks it touches.

#include "bmcv_sim.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int changed_chunks(const unsigned char* a, const unsigned char* b, int n, int chunk)
{
  int hit = 0;
  for (int off = 0; off < n; off += chunk)
  {
    int len = (off + chunk <= n) ? chunk : (n - off);
    if (memcmp(a + off, b + off, (size_t) len) != 0)
      hit++;
  }
  return hit;
}

typedef struct
{
  double bytes, c16, c32, c64;
  int worst_bytes;
  int n;
} Stat;

static void run(const char* label, int hz, int busy, int seconds)
{
  BmcvSim* s          = bmcv_sim_create();
  const int size      = bmcv_sim_instance_size();
  unsigned char* prev = malloc((size_t) size);
  unsigned char* curr = malloc((size_t) size);

  const int tick_us            = 250; // 4kHz engine
  const int ticks_per_snapshot = (1000000 / hz) / tick_us;

  if (busy)
  {
    // A module doing something: amplitude dialled in on a few channels so the
    // engine is actually generating, and a clock running.
    for (int c = 0; c < 8; c++)
    {
      bmcv_sim_add_encoder(s, c, 30);
      bmcv_sim_run(s, tick_us, 40);
    }
  }
  bmcv_sim_run(s, tick_us, 4000);
  bmcv_sim_export(s, prev);

  Stat st        = {0};
  st.worst_bytes = 0;

  for (int i = 0; i < hz * seconds; i++)
  {
    if (busy)
    {
      // A gate on input 0 every so often, so the clock and the phases move.
      if (i % 4 == 0)
        bmcv_sim_fire_gate(s, 0);
    }
    bmcv_sim_run(s, tick_us, ticks_per_snapshot);
    bmcv_sim_export(s, curr);

    int diff = 0;
    for (int b = 0; b < size; b++)
      if (prev[b] != curr[b])
        diff++;

    st.bytes += diff;
    st.c16 += changed_chunks(prev, curr, size, 16);
    st.c32 += changed_chunks(prev, curr, size, 32);
    st.c64 += changed_chunks(prev, curr, size, 64);
    if (diff > st.worst_bytes)
      st.worst_bytes = diff;
    st.n++;

    memcpy(prev, curr, (size_t) size);
  }

  const double n = st.n;
  printf("%-22s %4d Hz  changed %6.0f/%d bytes (%4.1f%%)  worst %5d  "
         "chunks: 16B %5.1f/%d  32B %5.1f/%d  64B %5.1f/%d\n",
         label, hz, st.bytes / n, size, 100.0 * (st.bytes / n) / size, st.worst_bytes, st.c16 / n, (size + 15) / 16, st.c32 / n,
         (size + 31) / 32, st.c64 / n, (size + 63) / 64);

  // What that costs on the wire, once 7-bit encoded and packed into USB-MIDI
  // event packets (3 payload bytes per 4-byte packet, 64-byte endpoint).
  double payload_full = size * 8.0 / 7.0;
  double payload_c32  = (st.c32 / n) * (32 + 2) * 8.0 / 7.0; // + a 2-byte chunk index
  printf("%-22s          wire: full %6.0f B = %4.1f transfers   "
         "32B-chunks %6.0f B = %4.1f transfers\n",
         "", payload_full * 4 / 3, payload_full * 4 / 3 / 64, payload_c32 * 4 / 3, payload_c32 * 4 / 3 / 64);

  free(prev);
  free(curr);
  bmcv_sim_destroy(s);
}

int main(void)
{
  printf("instance = %d bytes\n\n", bmcv_sim_instance_size());
  run("idle", 30, 0, 4);
  run("idle", 15, 0, 4);
  run("running", 30, 1, 4);
  run("running", 15, 1, 4);
  run("running", 5, 1, 4);
  return 0;
}
