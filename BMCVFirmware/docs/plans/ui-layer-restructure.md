# UI Layer Restructure


## Context

The firmware's interaction layer grew feature-by-feature and has no shared vocabulary
for the things it does repeatedly. Concretely, from reading the current code:

1. **`EngineState` is one bag** (`Core/Inc/Lib/state.h:153-205`) mixing DSP state
   (`channels_shared_phase`, `cfrm`, `csphs`, `channels_output_level`), interaction state
   (`assign_type`, `assign_src_id`, `shift_state`, `selected_param`, `momentary_scene`) and
   view state (`channels_mark_for/hue`, `blink_slow/fast`, `leds[]`). No boundary exists, so
   any module reaches into anything.

2. **Every call site re-derives button semantics** from raw `button_pressed_t` /
   `button_released_t` with its own threshold: `MS(10)`, `MS(200)`, `MS(500)`, `MS(1000)`,
   `CTRL_SHIFT_ACTIVATION` (100000), plus a raw `> 10` at `scene.c:64` — 10 µs, almost
   certainly meant as 10 ms. Render code re-derives independently of input code, so the two
   can disagree: SAV renders on `pressed_t > MS(1000)` (`scene.c:60`) but acts on
   `released_t > MS(1000)` (`scene.c:144`).

3. **No confirmation vocabulary.** Three unrelated one-off timers: `channels_mark_for[]`
   (decremented *inside* the render function, `channel.c:464`), `write_indicator_for`
   (painted outside the render pass, `bmcv.c:192`), and `error_*` (painted with an early
   return in `bmcv_state_update`, `bmcv.c:332`). Copy, clear, save, load and scene-assign
   produce no feedback at all.

4. **Affordance is re-implemented per mode.** "What can I press now" is coded separately in
   `channel.c:502-522` (CPY), `scene.c:67-87` (CPY), `scene.c:88-100` (QNT) and
   `channel.c:480-496` (QNT) — near-identical but not identical. Nothing ties "what blinks"
   to "what responds".

5. **Render leaves stale pixels.** `write_scene_button_led`'s QNT case (`scene.c:88-100`)
   writes nothing unless `assign_state == ASSIGN_TRIG_SRC`, so the framebuffer keeps the
   previous frame. `write_channel_led`'s MON case falls through into `default`
   (`channel.c:537`), letting the mark overlay clobber the mode colour.

6. **The assign FSM is a hardcoded pending×target matrix** (`assign.c:79-116`) with its
   exceptions living outside it: MON's "re-press source to unassign" is in
   `channel.c:266-270`, QNT's "first press also forces `quantize_mode`" is in
   `channel.c:244-248`. `assign_reset` is called from four unrelated places.

7. **Two different `dt`s.** `hw_state->dt` (engine tick) vs `state->dt` (UX interval).
   Timers age against whichever one the author reached for. Already flagged in `README.md`.

The outcome we want: a clean split between UI/view state and the underlying engine, with
one shared implementation for each recurring concept — button gestures, "pick a source then
a target", "here is what you can pick", and "that worked".

## Decisions taken

- **Channel LED rule:** base layer is always the output level; any touch reveals the value
  being edited for a timeout, then decays back. Applies in every shift mode, not just NONE.
  Entering a shift mode counts as a touch for all elements, so you still get the
  "show me every channel's shape mode" glance that SYS gives today — it just decays.
- **Confirmation:** one unified flash on the target LED(s), same timing everywhere, colour
  keyed to action class.
- **Shift modes:** hold ≥100 ms latches; a tap on the mode's own button exits. A tap on
  another ctrl button exits too — but is *consumed* by the exit and no longer changes
  `selected_param`. **QNT keeps its exception**: in QNT the ctrl and scene buttons are a
  piano-style keyboard for semitone selection, so only QNT's own button exits. This becomes
  a declared property of the mode, not a special case in `update_shift_mode`.
  (Note: `quantizer_button_idx` covers ctrl buttons 0,1,3,4,5 and all 7 scene buttons —
  ctrl button 2, which is QNT itself, is deliberately absent. That is the "one non-semitone
  button" TODO in `quantizer.c:10`.)
- **MUT is implemented in this pass.** In MUT, a channel button toggles that channel's mute
  **on release**, not on press. Mute forces that channel's DAC output to 0 V. A muted channel
  renders dim purple in place of its output level — in *every* mode, not just MUT, so muted
  channels are recognisable at a glance. That makes mute a base-layer property, which the
  layered renderer already has a natural slot for.

`EngineConfig` and therefore the FRAM layout are **not** touched by any stage.

## Target architecture

```
HwState  (raw samples, ring buffer)                bmcv.c            unchanged
   |
   v  ui_input_update()   -- every engine tick     ui_input.c        NEW
UiInput  { ButtonEvent ev[]; held_us[]; enc_delta[]; dt; }
   |
   v  ui_dispatch()       -- mode-table driven     ui_mode.c         NEW
UiState  { shift_state, selected_param, momentary_scene,
           Selection sel;                          ui_select.c       NEW
           Feedback  fb;                           ui_feedback.c     NEW
           muted[N_CHANNELS];
           edit_hold[]; blink_slow/fast; UiInput in; }
   |
   v  ui_render()         -- layered               ui_render.c       NEW
EngineState.leds[]
```

`EngineState` keeps only DSP output: phases, `channels_output_level`, trigger edges,
`scenes_contribution`, `active_scene`, debug mirrors, fps. `UxState` stays as the
composition root and gains a `UiState* ui` member.

## Stages

Each stage ends with tests green and the ARM firmware building. New `.c` files go in
`cmake/core_sources.cmake` (single source of truth for both builds).

### Stage 1 — button event layer

New `Core/Inc/Lib/ui_input.h`, `Core/Src/Lib/ui_input.c`.

```c
typedef enum { BTN_EV_NONE, BTN_EV_DOWN, BTN_EV_TAP, BTN_EV_HOLD, BTN_EV_LONG, BTN_EV_UP } ButtonEvent;

#define UI_T_HOLD  MS(100)   // replaces CTRL_SHIFT_ACTIVATION
#define UI_T_LONG  MS(500)
#define UI_T_VLONG MS(1000)
```

`UiInput` holds `ev[N_BUTTONS]`, `down[]`, `held_us[]`, per-button `crossed_hold/long`
latches, `enc_delta[N_ENCODERS]` and the single `dt`.

**Critical:** `update_ux_state` only runs when `input_dirty || dt > 8ms` (`engine.c:29`), and
`input_dirty` is set on button *level* change (`bmcv.c:322`) — a HOLD/LONG threshold crossing
is not a level change. So `ui_input_update()` must run **every engine tick**, before the
rate-limit check, OR-ing events into a pending set that dispatch drains. Encoder deltas
accumulate the same way. This is the README's "Consistent events (add delta until cleared?)"
item.

Then rewrite each existing predicate in terms of events:

| site | today | becomes |
|---|---|---|
| `ctrl_button.c:13` | `pressed_t > CTRL_SHIFT_ACTIVATION` | `BTN_EV_HOLD` |
| `ctrl_button.c:17` | `released_after > 0 && <= 100ms` | `BTN_EV_TAP` |
| `ctrl_button.c:27` | `released_after > 0 && < MS(200)` | `BTN_EV_TAP` |
| `scene.c:143` | `released_t > MS(10)` | `BTN_EV_TAP` |
| `scene.c:144` | `released_t > MS(1000)` | `BTN_EV_LONG` at crossing (fixes render/action disagreement) |
| `scene.c:64` | `pressed_t > 10` (µs) | `down[]` |
| `channel.c:234` | `released_t > MS(500)` | `BTN_EV_LONG` |
| `channel.c:296` | `long_pressed && released_t < t_no_rotation` | keep, as an explicit "button was not used as an encoder modifier" guard |

New test `tests/test_ui_input.c`: edge derivation, each threshold crossing, and accumulation
across skipped UX ticks.

### Stage 2 — split `UiState` out of `EngineState`

New `Core/Inc/Lib/ui_state.h`. Move out of `EngineState`: `assign_type`, `assign_src_id`,
`shift_state`, `selected_param`, `momentary_scene`, `blink_slow/fast`,
`channels_mark_for/hue`. Add `muted[N_CHANNELS]` here — see the mute note in stage 5 for why
it does not go in `EngineConfig`.

Two cross-boundary reads need explicit handling:

- `compute_scenes_contribution` (`scene.c:107`) reads `momentary_scene` (UI) and writes
  `active_scene` (DSP). Change its signature to take the slider value and
  `momentary_scene` as parameters, making it pure and removing its `UxState` dependency.
  `tests/test_scene.c` follows.
- `compute_channel` reads `channels_last_delta` for the MOD edit window
  (`channel.c:391`). Keep that field in `EngineState` but let only
  `ui_channel_note_edit()` write it.

`Fixture` (`tests/fixtures/fixture.h`) gains a `UiState` member.

### Stage 3 — generalised selection FSM

New `Core/Inc/Lib/ui_select.h`, `Core/Src/Lib/ui_select.c`. Replaces the
`assign_type`/`assign_src_id` pair and the matrix in `assign.c:79-116`.

```c
typedef enum { TGT_NONE, TGT_CHANNEL, TGT_SCENE, TGT_INPUT, TGT_TRIG_SRC, TGT_PRESET } TargetKind;
typedef struct { uint8_t action; uint8_t src_kind; int8_t src_id; } Selection;

void ui_sel_reset(UiState*);
int  ui_sel_is_src(const UiState*, TargetKind, int8_t id);
int  ui_sel_is_candidate(const UxState*, TargetKind, int8_t id);  // "pressing this does something"
void ui_sel_press(UxState*, TargetKind, int8_t id);               // sole entry point
```

`ui_sel_press` reads the active mode descriptor (stage 5) for legal source/target kinds,
commits through the **existing** mutation functions in `assign.c` (`assign_channel_to_channel`,
`assign_scene_to_scene`, `clear_channel`, `clear_scene`, `copy_scene_channel`) and
`presets.c` (`preset_store`, `preset_load`) — those stay as-is and keep their `test_assign.c`
coverage — then emits feedback and resets per the mode's `auto_reset` flag.

The two current exceptions become descriptor flags rather than code:
- MON re-press of source → `allow_deselect = 1` (sets `src_input = -1`, resets).
- QNT first press → the descriptor's `on_pick_src` hook sets `quantize_mode = QUANTIZE_TRIG_SRC`.

`ui_sel_is_candidate` is called by **both** the dispatcher and the renderer, so "what blinks"
and "what responds" cannot drift apart. This is what makes copy/clear/assign show their
valid targets consistently.

New test `tests/test_ui_select.c`: candidate sets per mode, commit, deselect, auto-reset.

### Stage 4 — feedback queue + layered renderer

New `Core/Inc/Lib/ui_feedback.h`, `Core/Src/Lib/ui_feedback.c`:

```c
typedef enum { FB_WRITE, FB_CLEAR, FB_LOAD, FB_ERROR } FeedbackKind;
#define UI_FB_SLOTS 4
#define UI_FB_DURATION MS(400)

void ui_feedback_emit(UiState*, FeedbackKind, TargetKind, int8_t id);  // id < 0 = all of kind
void ui_feedback_tick(UiState*, uint32_t dt);
int  ui_feedback_active(const UiState*, TargetKind, int8_t id, FeedbackKind* out);
```

Colour per kind comes from the semantic palette below, not from raw hue/sat/val at the call
site. This subsumes all three ad hoc timers: `channels_mark_for` → the edit layer
below, `write_indicator_for` (`bmcv.c:192`) → `ui_feedback_emit(FB_WRITE, TGT_PRESET, -1)`,
`error_*` blit (`bmcv.c:332`) → `ui_feedback_emit(FB_ERROR, ...)` with the early return removed.

New `Core/Src/Lib/ui_render.c`. Every element renders as strict layers, highest wins:

```c
static void render_channel(const ChannelSetup* ch, UxState* s) {
  if (s->ui->muted[ch->id])                                                 // 0 base
    led_set_hsv(s, ch->led, HUE_PURPLE, SAT_HIG, VAL_LOW);                  //   muted: dim purple
  else
    led_set_dac(s, ch->led, s->engine_state->channels_output_level[ch->id]);
  ui_render_context(s, ch->led, TGT_CHANNEL, ch->id);                       // 1 candidate / source
  ui_render_edit(s, ch->led, TGT_CHANNEL, ch->id);                          // 2 value on touch
  ui_render_feedback(s, ch->led, TGT_CHANNEL, ch->id);                      // 3 confirm flash
}
```

Layer 0 always writes, so stale pixels become structurally impossible (fixes item 5).
Because mute lives in layer 0, muted channels read as purple in every mode without any mode
knowing about mute.

**Semantic palette.** Add to `color_presets.h` a set of named `{h, s, v}` triples that the
renderer uses instead of raw hue/sat/val at each call site — this is the README's
"UX: Consistent colors" item:

| name | h, s, v | blink | used for |
|---|---|---|---|
| `UI_COL_CANDIDATE` | `HUE_PURPLE, SAT_MED, VAL_LOW` | fast | "you can pick this" — replaces today's desaturated white at `channel.c:505`, `scene.c:70` |
| `UI_COL_SOURCE` | `HUE_PURPLE, SAT_MAX, VAL_LOW` | steady | the already-picked source |
| `UI_COL_MUTED` | `HUE_PURPLE, SAT_HIG, VAL_LOW` | steady | muted channel base layer |
| `UI_COL_CONFIRM_WRITE` | `HUE_GREEN, SAT_MAX, VAL_MED` | — | copy, save, assign committed |
| `UI_COL_CONFIRM_CLEAR` | `HUE_RED, SAT_MAX, VAL_MED` | — | clear committed |
| `UI_COL_CONFIRM_LOAD` | `HUE_PURPLE, SAT_MED, VAL_MED` | — | preset load |
| `UI_COL_ERROR` | `HUE_RED, SAT_MAX, VAL_MED` | slow | error |

`HUE_PURPLE` (~180) is new, sitting between `HUE_BLUE` 160 and `HUE_MAGENTA` 200; magenta is
already taken by `quantize_mode_color`. Purple is deliberately off the red/green/blue axis
that `led_set_dac` uses for voltage, so nothing wearing it can be mistaken for a level — which
is why the whole selection/transfer family plus mute all sit on it.

Within purple, **motion is the discriminator, not hue**: candidate blinks, source and mute are
steady, and they separate further by saturation (MED / MAX / HIG). A muted channel that is
also a valid copy target therefore blinks lighter purple over steady purple. That is the one
genuinely subtle combination in the scheme — worth a look on hardware, and if it does not read,
the fix is to let the candidate layer blink to black instead of to purple.

**Brightness discipline.** LEDs read as far too bright above `VAL_MED` on this hardware. Base
and context layers stay at `VAL_LOW`; confirmation flashes go no higher than `VAL_MED`;
`VAL_HIG` and `VAL_MAX` are not used by the new renderer at all. (Today `scene.c:61` uses
`VAL_HIG` for SAV.)

Layer 1 is driven entirely by `ui_sel_is_candidate` / `ui_sel_is_src`. STA/STB's
"which scenes are currently A and B" indication moves here, replacing the bespoke
`imax(val, VAL_LOW) * blink_fast` at `scene.c:30`; the act of assigning A or B emits the
standard confirm flash like every other action.
Layer 2 implements the agreed rule: `edit_hold[]` is armed to `UI_EDIT_DISPLAY` (1000 ms) by
any encoder delta, param press, **or shift-mode entry**, and shows whichever value that mode's
encoder currently drives — `shape_mode` in SYS, `quantize_mode` in QNT, `input_amp_mode` in
MON, `params[active_scene][selected_param]` in NONE. The descriptor names the value, so one
renderer serves every mode.

All timers age in exactly one place: `ui_feedback_tick` and the `edit_hold` decay, both at
the top of `ui_dispatch`, against `ui->in.dt`. No timer is ever decremented in a render
function again.

New test `tests/test_ui_feedback.c`; extend `tests/test_led_render.c` with layer-precedence
assertions and a no-stale-pixel case for QNT.

### Stage 5 — declarative mode table

New `Core/Inc/Lib/ui_mode.h`:

```c
typedef struct {
  const char* name;             // "CPY"
  uint8_t exits_on_other_ctrl;  // 0 for QNT (keyboard overlay), 1 otherwise
  uint8_t overlays_keyboard;    // QNT: scene + ctrl buttons are semitones
  uint8_t action;               // ACT_COPY / ACT_CLEAR / ACT_ASSIGN_INPUT / ...
  uint8_t src_kinds, dst_kinds; // TargetKind bitmasks
  uint8_t allow_deselect, auto_reset;
  uint8_t channel_enc_target;   // ENC_PARAM / ENC_SHAPE / ENC_QUANT / ENC_AMPMODE / ENC_NONE
  uint8_t channel_btn_target;   // CHB_SELECT / CHB_MUTE_TOGGLE / CHB_NONE
  uint8_t scene_btn_target;     // SCN_MOMENTARY / SCN_SET_A / SCN_SET_B / SCN_INPUT_MODE /
                                // SCN_PRESET / SCN_SELECT / SCN_NONE
} UiModeDesc;

extern const UiModeDesc ui_modes[SHIFT_STATE_COUNT];
```

`update_channel`'s and `update_scene_button`'s `switch(shift_state)` collapse into
`switch(desc->channel_enc_target)` / `switch(desc->channel_btn_target)` /
`switch(desc->scene_btn_target)`. The `write_*_led` switches disappear into the stage-4
renderer, which reads the same descriptor.

`CHB_SELECT` forwards to `ui_sel_press(s, TGT_CHANNEL, id, is_long)` — the long flag is what
CLR uses to distinguish "clear this scene" from "clear all scenes"
(`channel.c:284-292`).

**MUT entry.** `channel_btn_target = CHB_MUTE_TOGGLE`, fired on `BTN_EV_TAP` (release, as
asked) rather than `BTN_EV_DOWN`; `channel_enc_target = ENC_NONE`, `scene_btn_target =
SCN_NONE`, `exits_on_other_ctrl = 1`. The toggle flips `ui->muted[id]`.

**Clickless.** A hard jump to 0 clicks, so the gate is a ramp. `ui->muted[]` is the *target*;
`EngineState.channels_mute_gain[N_CHANNELS]` (float, 0..1) is the ramped value, slewed toward
the target over `MUTE_RAMP_MS` (5 ms) and applied at the DAC write, `channel.c:578`:

```c
float* g = &state->engine_state->channels_mute_gain[ch->id];
float target = state->ui->muted[ch->id] ? 0.0f : 1.0f;
*g += fclampf(target - *g, -step, step);      // step = dt_s / (MUTE_RAMP_MS * 1e-3f)
dacadc_write(ch->dac_channel, (int16_t) (*g * state->engine_state->channels_output_level[ch->id]));
```

The ramp must run at DAC rate, not UI rate, so it lives here rather than in `ui_dispatch`, and
uses `state->hw_state->dt` (`write_channel_dac` is called from `bmcv_main:187` right after
`engine_tick`). The LED goes purple immediately on toggle while the audio ramps — the ramp is
short enough that the discrepancy is invisible.

Gating at the DAC write, not by zeroing `channels_output_level`, keeps the channel's real
value available to cross-modulation (`chcfg->src_input` routing) and to
`detect_channel_trigger` — so a muted channel still works as a trigger source for others.
Mute is deliberately the output stage only.

No confirm flash on a mute toggle: the purple base layer *is* the feedback and it persists,
so a 400 ms flash would only obscure it.

`muted[]` lives in `UiState`, not `EngineConfig` — adding a field to `ChannelConfig` would
change the FRAM record layout and break existing presets, and booting into a muted channel
is not behaviour you want. Mute therefore clears on power cycle.

`update_shift_mode` becomes:

```c
if (ev == BTN_EV_HOLD)       ui->shift_state = btn->id;
else if (ev == BTN_EV_TAP) {
  if (btn->id == ui->shift_state)                          ui_shift_exit(ui);
  else if (ui_modes[ui->shift_state].exits_on_other_ctrl)   ui_shift_exit(ui);  // consumed
  else if (ui->shift_state == SHIFT_STATE_NONE && btn->id < CH_PARAM_COUNT)
                                                            ui->selected_param = btn->id;
}
```

QNT's exception is now one `0` in the table. New test `tests/test_ui_mode.c`: exit semantics
including the QNT case, and that an exit-tap leaves `selected_param` unchanged.

## Files

**New:** `Core/{Inc,Src}/Lib/ui_input.*`, `ui_state.h`, `ui_select.*`, `ui_feedback.*`,
`ui_render.*`, `ui_mode.*` — all added to `cmake/core_sources.cmake`.

**Heavily changed:** `Core/Inc/Lib/state.h` (split), `Core/Src/Lib/ux_state.c` (becomes
`ui_dispatch` + `ui_render` orchestration), `channel.c` and `scene.c` (lose their `write_*_led`
switches and most of their `update_*` switches; `write_channel_dac` gains the mute gate),
`ctrl_button.c`, `quantizer.c`, `color_presets.h` (`HUE_PURPLE`).

**Light touch:** `bmcv.c` (feedback instead of direct LED writes and the error early-return),
`engine.c` (call `ui_input_update` every tick), `tests/fixtures/fixture.{h,c}`.

**Untouched:** `assign.c` mutations, `presets.c`, `config_validate.c`, `EngineConfig`/FRAM
layout, the DSP in `compute_channel`, `clock_sync.c`, `stepped_random.c`, `wave_fn.c`.

## Verification

Per stage:

```
just test        # cmake --build build-native && ctest --output-on-failure
just build       # ARM firmware still compiles
```

Existing tests that must stay green throughout: `test_assign`, `test_scene`, `test_channel`,
`test_quantizer`, `test_led_render`, `test_stepped_random`, `test_clock_sync`,
`test_config_validate`. `test_scene` and `test_led_render` need updates in stages 2 and 4
respectively; the rest should not need to change — if they do, the split leaked.

New: `test_ui_input`, `test_ui_select`, `test_ui_feedback`, `test_ui_mode`. Mute is covered in
`test_ui_mode` (toggle fires on release not press) and `test_led_render` (muted channel renders
purple in NONE, SYS and CPY alike); a `test_channel` case asserts a muted channel still drives
`detect_channel_trigger`.

On-hardware check after stage 4 and again after stage 5, since these are the visible ones:
enter each shift mode and confirm (a) every element lights immediately on entry and decays to
output level, (b) valid targets blink and invalid ones do not, (c) copy / clear / save / load /
scene-assign / A-B-assign each flash the target with the same timing, (d) tapping out of a
mode does not change the selected param, (e) QNT still takes the keyboard and only exits via
its own button, (f) MUT toggles on release, the channel goes to 0 V, and the purple stays
visible after leaving MUT.

## Settled

- Timings: `UI_EDIT_DISPLAY` 1000 ms, `UI_FB_DURATION` 400 ms, `MUTE_RAMP_MS` 5 ms — start
  here, tune on hardware.
- Colours: as in the palette table. Candidate, source, mute and load-confirm all sit on the
  new `HUE_PURPLE`, separated by blink and saturation rather than hue; brightness capped at
  `VAL_MED`.
- Mute ramp: 5 ms. At the measured ~3000 Hz DAC rate that is ~15 steps, and the converter is
  band-limited to ~1.5 kHz anyway, so it is comfortably enough. 10 ms is the fallback.
- `SAV` channel encoders and `MUT` scene buttons stay disabled (`ENC_NONE` / `SCN_NONE`) until
  there is a use for them.
- Mute ramps rather than jumping.

## Unresolved questions

None outstanding. One thing to check on hardware and report back on: whether a muted channel
that is simultaneously a valid copy/clear target reads clearly — blinking `SAT_MED` purple over
steady `SAT_HIG` purple. If not, the candidate layer blinks to black instead.
