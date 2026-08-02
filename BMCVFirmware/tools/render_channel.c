// Renders a channel's output by running the real engine (engine_tick, the
// same function bmcv_main calls on hardware) against a scripted timeline,
// and dumps the result to CSV or WAV.
//
// Because it drives the actual signal path, what you see here includes scene
// blending, the PLL, quantization and amp/offset - not just a bare shape
// lookup. Change channel.c / wave_fn.c / stepped_random.c, rebuild, re-run.
//
// Usage:
//   render_channel [--shape-mode=lfo|stepped_random] [--shape=F] [--mod=F]
//                  [--shape-sweep=A:B] [--mod-sweep=A:B]
//                  [--freq=MULT] [--bpm=F] [--amp=N] [--offset=N]
//                  [--quantize] [--duration=SEC] [--rate=HZ]
//                  --out=PATH(.csv|.wav)
//
// shape/mod are floats in [-1,1] (the CH_PARAM_SHP/CH_PARAM_MOD domain).
// --freq is the frequency multiplier relative to the clock (1.0 = one cycle
// per beat). --bpm drives a synthetic clock so the PLL has something to lock
// to. *-sweep interpolates that parameter across the render.
#include "clock_sync.h"
#include "fixture.h"
#include "helpers.h"
#include "wav_writer.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RENDER_CH 0

typedef struct
{
  int shape_mode;
  float shape, shape_to;
  int shape_sweep;
  float mod, mod_to;
  int mod_sweep;
  float freq_mult;
  float bpm;
  int amp, offset;
  int quantize;
  float duration_s;
  uint32_t rate;
  const char* out;
} Args;

static const struct
{
  const char* name;
  int mode;
} shape_mode_names[] = {
    {"lfo", SHAPE_LFO},
    {"smooth", SHAPE_STEPPED_SMOOTH},
    {"semi", SHAPE_STEPPED_SEMI},
    {"stepped", SHAPE_STEPPED_HARD},
    {"stepped_random", SHAPE_STEPPED_SMOOTH}, // legacy spelling
};

static int shape_mode_from_name(const char* name)
{
  for (size_t i = 0; i < sizeof(shape_mode_names) / sizeof(shape_mode_names[0]); i++)
  {
    if (strcmp(name, shape_mode_names[i].name) == 0)
      return shape_mode_names[i].mode;
  }
  fprintf(stderr, "unknown --shape-mode '%s', falling back to lfo\n", name);
  return SHAPE_LFO;
}

static int opt(const char* arg, const char* name, const char** val)
{
  size_t n = strlen(name);
  if (strncmp(arg, name, n) == 0 && arg[n] == '=')
  {
    *val = arg + n + 1;
    return 1;
  }
  return 0;
}

static void parse_args(int argc, char** argv, Args* a)
{
  *a = (Args) {.freq_mult = 1.0f, .bpm = 120.0f, .amp = 20000, .offset = 0, .duration_s = 2.0f, .rate = 1000, .out = NULL};

  for (int i = 1; i < argc; i++)
  {
    const char* v;
    if (opt(argv[i], "--shape-mode", &v))
      a->shape_mode = shape_mode_from_name(v);
    else if (opt(argv[i], "--shape-sweep", &v))
      a->shape_sweep = sscanf(v, "%f:%f", &a->shape, &a->shape_to) == 2;
    else if (opt(argv[i], "--shape", &v))
      a->shape = (float) atof(v);
    else if (opt(argv[i], "--mod-sweep", &v))
      a->mod_sweep = sscanf(v, "%f:%f", &a->mod, &a->mod_to) == 2;
    else if (opt(argv[i], "--mod", &v))
      a->mod = (float) atof(v);
    else if (opt(argv[i], "--freq", &v))
      a->freq_mult = (float) atof(v);
    else if (opt(argv[i], "--bpm", &v))
      a->bpm = (float) atof(v);
    else if (opt(argv[i], "--amp", &v))
      a->amp = atoi(v);
    else if (opt(argv[i], "--offset", &v))
      a->offset = atoi(v);
    else if (opt(argv[i], "--duration", &v))
      a->duration_s = (float) atof(v);
    else if (opt(argv[i], "--rate", &v))
      a->rate = (uint32_t) atoi(v);
    else if (opt(argv[i], "--out", &v))
      a->out = v;
    else if (strcmp(argv[i], "--quantize") == 0)
      a->quantize = 1;
  }
}

static int ends_with(const char* s, const char* suffix)
{
  size_t ls = strlen(s), lf = strlen(suffix);
  return ls >= lf && strcmp(s + ls - lf, suffix) == 0;
}

// CH_PARAM_FRQ is stored as a quantized multiplier index: >=0 means
// (value/255 + 1)x, <0 means 1/(1 - value/255)x. Invert that here so the CLI
// can take a plain multiplier.
static int16_t freq_param_from_multiplier(float mult)
{
  float p = (mult >= 1.0f) ? (mult - 1.0f) : (1.0f - 1.0f / mult);
  return (int16_t) lrintf(p * 255.0f);
}

int main(int argc, char** argv)
{
  Args a;
  parse_args(argc, argv, &a);

  if (!a.out)
  {
    fprintf(stderr, "usage: render_channel [--shape-mode=lfo|smooth|semi|stepped] [--shape=F] [--mod=F]\n"
                    "                      [--shape-sweep=A:B] [--mod-sweep=A:B] [--freq=MULT] [--bpm=F]\n"
                    "                      [--amp=N] [--offset=N] [--quantize] [--duration=SEC] [--rate=HZ]\n"
                    "                      --out=PATH(.csv|.wav)\n");
    return 1;
  }

  Fixture f;
  fixture_init(&f);

  ChannelConfig* cfg = &f.engine_config.channel_state[RENDER_CH];
  cfg->shape_mode    = (int8_t) a.shape_mode;
  cfg->quantize_mode = a.quantize ? QUANTIZE_CONTINUOUS : QUANTIZE_DISABLED;

  fixture_set_param(&f, RENDER_CH, 0, CH_PARAM_FRQ, freq_param_from_multiplier(a.freq_mult));
  fixture_set_param(&f, RENDER_CH, 0, CH_PARAM_AMP, (int16_t) a.amp);
  fixture_set_param(&f, RENDER_CH, 0, CH_PARAM_OFS, (int16_t) a.offset);

  uint32_t n     = (uint32_t) (a.duration_s * (float) a.rate);
  float* samples = malloc((size_t) n * sizeof(float));
  if (!samples)
  {
    fprintf(stderr, "out of memory for %u samples\n", n);
    return 1;
  }

  FILE* csv = NULL;
  if (!ends_with(a.out, ".wav"))
  {
    csv = fopen(a.out, "w");
    if (!csv)
    {
      fprintf(stderr, "could not open %s for writing\n", a.out);
      free(samples);
      return 1;
    }
    fprintf(csv, "t,phase,shape,mod,level,norm\n");
  }

  const uint32_t dt_us    = 1000000u / a.rate;
  const uint32_t pulse_us = (uint32_t) (60000000.0f / (a.bpm * (float) f.engine_state.clock.PULSES_PER_BEAT));
  uint32_t next_pulse_us  = 0;

  for (uint32_t i = 0; i < n; i++)
  {
    float t     = n > 1 ? (float) i / (float) (n - 1) : 0.0f;
    float shape = a.shape_sweep ? lerp(a.shape, a.shape_to, t) : a.shape;
    float mod   = a.mod_sweep ? lerp(a.mod, a.mod_to, t) : a.mod;

    fixture_set_param(&f, RENDER_CH, 0, CH_PARAM_SHP, (int16_t) lrintf(shape * INT16_MAX));
    fixture_set_param(&f, RENDER_CH, 0, CH_PARAM_MOD, (int16_t) lrintf(mod * INT16_MAX));

    // Synthetic clock, so the PLL behaves as it would with a patched clock in.
    while (f.hw_state.time >= next_pulse_us)
    {
      Clock_Trigger(&f.engine_state.clock, next_pulse_us);
      next_pulse_us += pulse_us;
    }
    Clock_Poll(&f.engine_state.clock, f.hw_state.time);

    fixture_tick(&f, dt_us);

    int16_t level = f.engine_state.channels_output_level[RENDER_CH];
    samples[i]    = (float) level / (float) INT16_MAX;

    if (csv)
    {
      fprintf(csv, "%.6f,%.6f,%.6f,%.6f,%d,%.6f\n", i / (double) a.rate, (double) f.engine_state.channels_effective[RENDER_CH].phase, (double) shape,
              (double) mod, level, (double) samples[i]);
    }
  }

  if (csv)
  {
    fclose(csv);
    fprintf(stderr, "wrote %u rows to %s\n", n, a.out);
  }
  else if (!wav_write_mono_f32(a.out, samples, n, a.rate))
  {
    fprintf(stderr, "could not open %s for writing\n", a.out);
    free(samples);
    return 1;
  }
  else
  {
    fprintf(stderr, "wrote %u samples (%.2fs @ %uHz) to %s\n", n, a.duration_s, a.rate, a.out);
  }

  free(samples);
  return 0;
}
