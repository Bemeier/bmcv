// Drive the module from a scripted input timeline and print what it did.
//
// The point is regression coverage for whole interactions - "hold STA, tap
// scene 3, turn encoder 0" - which unit tests cannot express and which had no
// coverage at all. Run it, eyeball the CSV once, commit it as a golden file.
//
//   bmcv_sim_cli --script=flows/save_recall.txt --emit=outputs
//   bmcv_sim_cli --duration=2s --bpm=120 --emit=all
//
// Script format, one command per line, times non-decreasing:
//
//   # comment
//   0ms     slider 0.0
//   100ms   ctrl QNT 1        # hold the QNT shift button
//   700ms   ctrl QNT 0
//   1s      enc 3 +4          # 4 detents clockwise on encoder 3
//   1s500ms scene 2 1         # scene button 2 down
//   2s      cv 0 5.0          # 5V on input jack 0
//   2s      gate 1            # one trigger on input jack 1
//   3s      chbtn 0 1         # channel 0's encoder push
//   3s      btn 12 1          # or a raw button index
//   4s      end

#include "bmcv_sim.h"
#include "hw_setup.h"
#include "ui_mode.h"
#include "ux_setup.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_EVENTS 4096

typedef enum
{
  EV_BUTTON,
  EV_ENCODER,
  EV_SLIDER,
  EV_CV,
  EV_GATE,
  EV_END,
} EvKind;

typedef struct
{
  uint32_t at_us;
  EvKind kind;
  int32_t a;
  float f;
} Event;

static Event events[MAX_EVENTS];
static int n_events;

// Ctrl button ids are ShiftStates, and the names come from the firmware's own
// mode table via bmcv_sim_mode_name() rather than being retyped here.
static const char* ctrl_name(int id) { return bmcv_sim_mode_name(id); }

/* ---- time parsing ------------------------------------------------------- */

// Accepts 1s, 250ms, 1s500ms, 1500000us, or a bare number of milliseconds.
static int parse_time(const char* s, uint32_t* out)
{
  uint32_t total = 0;
  int any        = 0;

  while (*s)
  {
    char* end;
    double v = strtod(s, &end);
    if (end == s)
      return 0;

    if (strncmp(end, "ms", 2) == 0)
    {
      total += (uint32_t) (v * 1000.0);
      s = end + 2;
    }
    else if (strncmp(end, "us", 2) == 0)
    {
      total += (uint32_t) v;
      s = end + 2;
    }
    else if (*end == 's')
    {
      total += (uint32_t) (v * 1000000.0);
      s = end + 1;
    }
    else if (*end == '\0')
    {
      total += (uint32_t) (v * 1000.0); // bare number means milliseconds
      s = end;
    }
    else
    {
      return 0;
    }
    any = 1;
  }

  if (!any)
    return 0;
  *out = total;
  return 1;
}

/* ---- script parsing ----------------------------------------------------- */

static int lookup_ctrl(const char* name)
{
  for (int i = 0; i < N_CTRL_BUTTONS; i++)
  {
    if (strcmp(name, ctrl_name(i)) == 0)
      return i;
  }
  return -1;
}

static int add_event(uint32_t at, EvKind kind, int32_t a, float f)
{
  if (n_events >= MAX_EVENTS)
  {
    fprintf(stderr, "too many events (max %d)\n", MAX_EVENTS);
    return 0;
  }
  events[n_events].at_us = at;
  events[n_events].kind  = kind;
  events[n_events].a     = a;
  events[n_events].f     = f;
  n_events++;
  return 1;
}

static int parse_script(const char* path, const UxSetup* ux)
{
  FILE* fh = strcmp(path, "-") == 0 ? stdin : fopen(path, "r");
  if (!fh)
  {
    fprintf(stderr, "cannot open %s\n", path);
    return 0;
  }

  char line[512];
  int lineno = 0;
  int ok     = 1;

  while (fgets(line, sizeof(line), fh))
  {
    lineno++;

    char* hash = strchr(line, '#');
    if (hash)
      *hash = '\0';

    char time_s[64], cmd[64], arg1[64], arg2[64];
    int n = sscanf(line, "%63s %63s %63s %63s", time_s, cmd, arg1, arg2);
    if (n < 2)
      continue; // blank or comment-only

    uint32_t at;
    if (!parse_time(time_s, &at))
    {
      fprintf(stderr, "%s:%d: bad time '%s'\n", path, lineno, time_s);
      ok = 0;
      continue;
    }

    if (strcmp(cmd, "end") == 0)
    {
      ok &= add_event(at, EV_END, 0, 0.0f);
    }
    else if (strcmp(cmd, "slider") == 0 && n >= 3)
    {
      ok &= add_event(at, EV_SLIDER, 0, (float) atof(arg1));
    }
    else if (strcmp(cmd, "cv") == 0 && n >= 4)
    {
      ok &= add_event(at, EV_CV, atoi(arg1), (float) atof(arg2));
    }
    else if (strcmp(cmd, "gate") == 0 && n >= 3)
    {
      ok &= add_event(at, EV_GATE, atoi(arg1), 0.0f);
    }
    else if (strcmp(cmd, "enc") == 0 && n >= 4)
    {
      ok &= add_event(at, EV_ENCODER, atoi(arg1), (float) atoi(arg2));
    }
    else if (strcmp(cmd, "btn") == 0 && n >= 4)
    {
      ok &= add_event(at, EV_BUTTON, atoi(arg1), (float) atoi(arg2));
    }
    else if (strcmp(cmd, "ctrl") == 0 && n >= 4)
    {
      int id = lookup_ctrl(arg1);
      if (id < 0)
      {
        fprintf(stderr, "%s:%d: unknown ctrl button '%s'\n", path, lineno, arg1);
        ok = 0;
        continue;
      }
      ok &= add_event(at, EV_BUTTON, ux->ctrl_buttons[id].button, (float) atoi(arg2));
    }
    else if (strcmp(cmd, "scene") == 0 && n >= 4)
    {
      int id = atoi(arg1);
      if (id < 0 || id >= N_SCENES)
      {
        fprintf(stderr, "%s:%d: scene %d out of range\n", path, lineno, id);
        ok = 0;
        continue;
      }
      ok &= add_event(at, EV_BUTTON, ux->scenes[id].button, (float) atoi(arg2));
    }
    else if (strcmp(cmd, "chbtn") == 0 && n >= 4)
    {
      int id = atoi(arg1);
      if (id < 0 || id >= N_CHANNELS)
      {
        fprintf(stderr, "%s:%d: channel %d out of range\n", path, lineno, id);
        ok = 0;
        continue;
      }
      ok &= add_event(at, EV_BUTTON, ux->channels[id].button, (float) atoi(arg2));
    }
    else
    {
      fprintf(stderr, "%s:%d: unknown command '%s'\n", path, lineno, cmd);
      ok = 0;
    }
  }

  if (fh != stdin)
    fclose(fh);

  for (int i = 1; i < n_events; i++)
  {
    if (events[i].at_us < events[i - 1].at_us)
    {
      fprintf(stderr, "%s: events are not in time order (event %d)\n", path, i + 1);
      ok = 0;
      break;
    }
  }

  return ok;
}

/* ---- main --------------------------------------------------------------- */

static void usage(void)
{
  printf("bmcv_sim_cli - drive the BMCV engine from a scripted input timeline\n\n"
         "  --script=PATH     input timeline ('-' for stdin); see the header of this file\n"
         "  --duration=TIME   how long to run (default 2s, or the script's last event)\n"
         "  --tick=TIME       engine tick period (default 250us = 4kHz)\n"
         "  --rate=TIME       CSV row interval (default 5ms)\n"
         "  --emit=WHAT       outputs | leds | ui | all   (default outputs)\n"
         "  --bpm=N           synthesise a clock on input 0 at N bpm\n"
         "  --help\n\n"
         "Times accept 1s, 250ms, 500us, 1s500ms, or a bare number of milliseconds.\n");
}

static int match(const char* arg, const char* key, const char** val)
{
  size_t n = strlen(key);
  if (strncmp(arg, key, n) == 0)
  {
    *val = arg + n;
    return 1;
  }
  return 0;
}

int main(int argc, char** argv)
{
  const char* script   = NULL;
  const char* emit     = "outputs";
  uint32_t duration_us = 0;
  uint32_t tick_us     = 250;
  uint32_t rate_us     = 5000;
  float bpm            = 0.0f;

  for (int i = 1; i < argc; i++)
  {
    const char* v;
    if (strcmp(argv[i], "--help") == 0)
    {
      usage();
      return 0;
    }
    else if (match(argv[i], "--script=", &v))
      script = v;
    else if (match(argv[i], "--emit=", &v))
      emit = v;
    else if (match(argv[i], "--bpm=", &v))
      bpm = (float) atof(v);
    else if (match(argv[i], "--duration=", &v))
    {
      if (!parse_time(v, &duration_us))
      {
        fprintf(stderr, "bad --duration\n");
        return 1;
      }
    }
    else if (match(argv[i], "--tick=", &v))
    {
      if (!parse_time(v, &tick_us) || tick_us == 0)
      {
        fprintf(stderr, "bad --tick\n");
        return 1;
      }
    }
    else if (match(argv[i], "--rate=", &v))
    {
      if (!parse_time(v, &rate_us) || rate_us == 0)
      {
        fprintf(stderr, "bad --rate\n");
        return 1;
      }
    }
    else
    {
      fprintf(stderr, "unknown argument '%s' (try --help)\n", argv[i]);
      return 1;
    }
  }

  const UxSetup* ux = UxSetup_InitFromHw(HwSetup_Get());

  if (script && !parse_script(script, ux))
    return 1;

  if (duration_us == 0)
  {
    duration_us = n_events ? events[n_events - 1].at_us : 0;
    if (duration_us == 0)
      duration_us = 2000000;
  }

  int want_outputs = strcmp(emit, "outputs") == 0 || strcmp(emit, "all") == 0;
  int want_leds    = strcmp(emit, "leds") == 0 || strcmp(emit, "all") == 0;
  int want_ui      = strcmp(emit, "ui") == 0 || strcmp(emit, "all") == 0;
  if (!want_outputs && !want_leds && !want_ui)
  {
    fprintf(stderr, "bad --emit (outputs | leds | ui | all)\n");
    return 1;
  }

  BmcvSim* s = bmcv_sim_create();
  if (!s)
  {
    fprintf(stderr, "out of memory\n");
    return 1;
  }

  // Header
  printf("time_us");
  if (want_outputs)
    for (int c = 0; c < BMCV_SIM_CHANNELS; c++)
      printf(",out%d", c);
  if (want_ui)
  {
    printf(",shift,param,scene,bpm,err");
    for (int c = 0; c < BMCV_SIM_CHANNELS; c++)
      printf(",mute%d", c);
  }
  if (want_leds)
    for (int i = 0; i < BMCV_SIM_LEDS; i++)
      printf(",led%d_r,led%d_g,led%d_b", i, i, i);
  printf("\n");

  uint32_t clock_period_us = 0;
  if (bpm > 0.0f)
  {
    // 4 pulses per beat, matching ClockState.PULSES_PER_BEAT.
    clock_period_us = (uint32_t) (60000000.0f / (bpm * 4.0f));
  }

  int next_event      = 0;
  uint32_t next_row   = 0;
  uint32_t next_pulse = clock_period_us;

  for (uint32_t t = 0; t <= duration_us; t += tick_us)
  {
    while (next_event < n_events && events[next_event].at_us <= t)
    {
      const Event* e = &events[next_event++];
      switch (e->kind)
      {
      case EV_BUTTON:
        bmcv_sim_set_button(s, e->a, (int32_t) e->f);
        break;
      case EV_ENCODER:
        bmcv_sim_add_encoder(s, e->a, (int32_t) e->f);
        break;
      case EV_SLIDER:
        bmcv_sim_set_slider01(s, e->f);
        break;
      case EV_CV:
        bmcv_sim_set_cv(s, e->a, e->f);
        break;
      case EV_GATE:
        bmcv_sim_fire_gate(s, e->a);
        break;
      case EV_END:
        duration_us = t;
        break;
      }
    }

    if (clock_period_us)
    {
      while (t >= next_pulse)
      {
        bmcv_sim_fire_gate(s, 0);
        next_pulse += clock_period_us;
      }
    }

    bmcv_sim_run(s, tick_us, 1);

    if (t >= next_row)
    {
      next_row += rate_us;

      printf("%u", t);

      if (want_outputs)
      {
        const float* out = bmcv_sim_outputs_v(s);
        for (int c = 0; c < BMCV_SIM_CHANNELS; c++)
          printf(",%.4f", out[c]);
      }

      if (want_ui)
      {
        printf(",%d,%d,%d,%.1f,%d", bmcv_sim_shift_state(s), bmcv_sim_selected_param(s), bmcv_sim_active_scene(s), (double) bmcv_sim_bpm(s),
               bmcv_sim_error_flags(s));
        for (int c = 0; c < BMCV_SIM_CHANNELS; c++)
          printf(",%d", bmcv_sim_channel_muted(s, c));
      }

      if (want_leds)
      {
        const uint8_t* leds = bmcv_sim_leds_rgb(s);
        for (int i = 0; i < BMCV_SIM_LEDS * 3; i++)
          printf(",%u", leds[i]);
      }

      printf("\n");
    }
  }

  bmcv_sim_destroy(s);
  return 0;
}
