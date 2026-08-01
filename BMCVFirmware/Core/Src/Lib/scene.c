#include "scene.h"
#include "assign.h"
#include "color_presets.h"
#include "dac_adc.h"
#include "error.h"
#include "helpers.h"
#include "hw_setup.h"
#include "led_fb.h"
#include "presets.h"
#include "state.h"
#include "ui_input.h"
#include "ux_state.h"
#include <stdint.h>

static uint8_t input_mode_color[INPUT_MODE_COUNT] = {HUE_GREEN, HUE_RED, HUE_CYAN, HUE_MAGENTA};

static uint8_t min_value = 12;

void write_scene_button_led(const SceneSetup* scene, UxState* state)
{
  // Scene contribution
  uint8_t val =
      state->engine_state->scenes_contribution[scene->id] / 8 + (state->engine_state->active_scene == scene->id ? min_value : VAL_OFF);

  switch (state->ui->shift_state)
  {
  case SHIFT_STATE_STA:
  case SHIFT_STATE_STB:
    if (state->engine_config->scene_a == scene->id || state->engine_config->scene_b == scene->id)
    {
      val = imax(val, VAL_LOW) * state->ui->blink_fast;
    }
    led_set_hsv(state, scene->led, 0, SAT_OFF, val);
    break;
  case SHIFT_STATE_NONE:
    led_set_hsv(state, scene->led, 0, SAT_OFF, val);
    break;
  case SHIFT_STATE_MON:
    if (scene->id < N_INPUTS)
    {
      int16_t adc_val = get_adc(state->hw_setup->input_adc_idx[scene->id]);
      led_set_adcr(state, scene->led, adc_val);
    }
    /*
        if (assign_state(state) == ASSIGN_CHANNEL)
        {
            led_set_hsv(state, scene->led, HUE_GREEN, SAT_HIG, assign_src(state) == scene->id ? VAL_LOW : VAL_OFF);
        }
        */
    // Monitoring inputs handleded on hardware level for better update rate
    // scene->id >= N_INPUTS can be handled here still (no function currently)
    break;
  case SHIFT_STATE_SYS:
    if (scene->id < N_INPUTS)
      led_set_hsv(state, scene->led, input_mode_color[state->engine_config->input_mode[scene->id]], SAT_HIG, VAL_LOW);
    else
      led_set_hsv(state, scene->led, 0, SAT_OFF, VAL_OFF);
    break;
  case SHIFT_STATE_SAV:
    // Same predicates the action path uses, so the LED cannot disagree with
    // what the release will actually do.
    int8_t load = btn_down(&state->ui->in, scene->button);
    int8_t save = btn_holding(&state->ui->in, scene->button, UI_T_VLONG);
    led_set_hsv(state, scene->led, save ? HUE_RED : HUE_GREEN, SAT_MAX, load ? VAL_HIG : VAL_MED);
    break;
  case SHIFT_STATE_CLR:
    int8_t held = btn_down(&state->ui->in, scene->button);
    led_set_hsv(state, scene->led, HUE_RED, SAT_HIG, (state->ui->blink_fast || held) * VAL_LOW);
    break;
  case SHIFT_STATE_CPY:
    if (assign_state(state) == ASSIGN_NONE)
    {
      led_set_hsv(state, scene->led, 0, SAT_OFF, state->ui->blink_fast * VAL_LOW);
    }
    else if (assign_state(state) == ASSIGN_SCENE || assign_state(state) == ASSIGN_CHANNEL)
    {
      if (assign_src(state) == scene->id && assign_state(state) == ASSIGN_SCENE)
      {
        led_set_hsv(state, scene->led, HUE_GREEN, SAT_HIG, VAL_LOW);
      }
      else
      {
        led_set_hsv(state, scene->led, 0, SAT_OFF, state->ui->blink_fast * VAL_LOW);
      }
    }
    else
    {
      led_set_hsv(state, scene->led, 0, SAT_OFF, VAL_OFF);
    }
    break;
  case SHIFT_STATE_QNT:
    if (assign_state(state) == ASSIGN_TRIG_SRC)
    {
      if (scene->id < N_INPUTS)
      {
        led_set_hsv(state, scene->led, 0, SAT_OFF, state->ui->blink_fast * VAL_LOW);
      }
      else
      {
        led_set_hsv(state, scene->led, 0, SAT_OFF, VAL_OFF);
      }
    }
    break;
  default:
    led_set_hsv(state, scene->led, 0, SAT_OFF, VAL_OFF);
    break;
  }
}

// Pure: the crossfade depends only on the slider, the configured A/B pair and
// whichever scene the user is holding. Taking momentary_scene as an argument
// rather than reading UiState keeps the one UI->DSP dependency explicit and
// lets this be exercised without an interaction layer at all.
void compute_scenes_contribution(EngineState* es, const EngineConfig* cfg, uint16_t slider, int8_t momentary_scene)
{
  for (uint8_t s = 0; s < N_SCENES; s++)
  {
    es->scenes_contribution[s] = 0;
  }

  if (momentary_scene >= 0)
  {
    es->scenes_contribution[momentary_scene] = 255;
    es->active_scene                         = momentary_scene;
  }
  else
  {
    uint8_t scene_a         = cfg->scene_a;
    uint8_t scene_b         = cfg->scene_b;
    uint16_t scene_a_anchor = SLIDER_MAX_VALUE;
    uint16_t scene_b_anchor = SLIDER_MIN_VALUE;

    if (scene_a == scene_b)
    {
      es->scenes_contribution[scene_a] = 255;
    }
    else
    {
      es->scenes_contribution[scene_a] = interpolate_clamped(scene_b_anchor, scene_a_anchor, slider);
      es->scenes_contribution[scene_b] = 255 - es->scenes_contribution[scene_a];
    }
    es->active_scene = es->scenes_contribution[scene_a] > es->scenes_contribution[scene_b] ? scene_a : scene_b;
  }
}

void update_scene_button(const SceneSetup* scene, UxState* state)
{
  int8_t pressed   = btn_ev(&state->ui->in, scene->button, BTN_EV_UP);
  int8_t momentary = btn_holding(&state->ui->in, scene->button, UI_T_DEBOUNCE);
  // Momentarty activation
  switch (state->ui->shift_state)
  {
  case SHIFT_STATE_NONE: // Normal mode, momentary scene activation
    if (momentary && state->ui->momentary_scene < 0)
    {
      state->ui->momentary_scene = scene->id;
    }

    if (!momentary && state->ui->momentary_scene == scene->id)
    {
      state->ui->momentary_scene = -1;
    }
    break;
  case SHIFT_STATE_STA:
    if (pressed)
      state->engine_config->scene_a = scene->id;
    break;
  case SHIFT_STATE_STB:
    if (pressed)
      state->engine_config->scene_b = scene->id;
    break;
  case SHIFT_STATE_SYS:
    if (pressed && scene->id < N_INPUTS)
      state->engine_config->input_mode[scene->id] = (state->engine_config->input_mode[scene->id] + 1) % INPUT_MODE_COUNT;
    break;
  case SHIFT_STATE_MON:
    if (pressed && scene->id < N_INPUTS)
    {
      assign_event(ASSIGN_INPUT, scene->id, state);
      assign_reset(state);
    }
    break;
  case SHIFT_STATE_SAV:
    // Store fires the moment the hold crosses UI_T_VLONG, so the red LED and
    // the write happen together; the matching release must then not also load.
    if (btn_ev(&state->ui->in, scene->button, BTN_EV_VLONG))
    {
      preset_store(state->engine_config, scene->id);
    }
    else if (pressed && btn_held(&state->ui->in, scene->button) < UI_T_VLONG)
    {
      if (!preset_load(state->engine_config, scene->id))
      {
        error_set(5);
      }
    }
    break;
  case SHIFT_STATE_CPY:
    if (pressed)
      assign_event(ASSIGN_SCENE, scene->id, state);
    break;
  case SHIFT_STATE_CLR:
    if (pressed)
      clear_scene(scene->id, state);
    break;
  case SHIFT_STATE_QNT:
    if (scene->id < N_INPUTS && pressed)
    {
      assign_event(ASSIGN_INPUT, scene->id, state);
    }
  default:
    break;
  }
}
