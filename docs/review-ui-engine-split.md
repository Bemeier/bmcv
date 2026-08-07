# Review: UI/engine decoupling + simulator

Reviewed at `ui-layer-restructure` (23de3c4 + working tree), then applied.
Everything below marked **done** is in the tree; see CHANGELOG.md for the
user-facing summary.

Verification after the pass, all of it under `just check`: 17/17 native tests,
all four `sim/flows/` golden files byte-identical, `web/smoke.mjs` and
`web/frontend-check.mjs` all-pass, ARM firmware links (FLASH 159404 B, RAM
9000 B - unchanged), native + sim builds clean under
`-Wall -Wextra -Wmissing-prototypes`.

---

## What the review found, and what happened to it

| # | Finding | Status |
|---|---|---|
| 1 | `UxState*` threaded through the DSP as a god pointer | **done** - narrow signatures |
| 2 | `render_scene` still switched on `shift_state` | **done** - `scene_btn_base`, `xfade_end` |
| 3 | `ui_select.c` had QNT/MON-specific branches | **done** - `assign_*` + `UiAction` dispatch |
| 4 | `input_state.c` drove the clock and the autosave | **done** - latches only |
| 5 | `channel.c` was four modules in one file | **done** - `ui_channel.c`, `ui_scene.c` |
| 6 | Mute was a mutating getter with a prose contract | **done** - `channels_gated_level[]` |
| 7 | Index spaces with three meanings, no assertion | **done** - static asserts, `TRIG_SRC_*` |
| 8 | Mode-name table duplicated in four languages | **done** - three now derived |
| 9 | Save/autosave/preset-slot correctness | **done** |
| 10 | `CHANNEL_TRIG_THRESH` at 0.31V | **done** - matches the input side at 1.25V |
| 11 | Dead code and stale comments | **done** |
| 12 | `led_set_adcr`/`led_set_dac` duplication, edit-hold repetition | **done** |
| 13 | Naming and `state.h` as a grab bag | **done** |
| 14 | `web/main.js` is a 926-line monolith | **done** - nine modules |

Found while applying, not in the original review:

- **Four test files never failed the build** (`main` called `TESTKIT_SUMMARY()`
  without `return`). This is the most consequential thing in the pass: those
  four are the newest suites, so everything they covered was unverified.
  Fixed, and `[[nodiscard]]` now prevents a repeat.
- **Scene buttons 4-6 blinked as candidates in MON/QNT** for inputs that do not
  exist - the renderer and the handler disagreed about the `>= N_INPUTS` guard.
- **`envelope.c` was entirely unused** - nothing outside the file referenced
  `trigger_envelope`, `update_envelope` or `ENVELOPE`, yet it compiled into
  every build. Confirmed as an abandoned experiment and removed.
- **Enum members inside the packed FRAM record.** `ChannelConfig` is
  `__attribute__((packed))` but `input_amp_mode` and `quantize_mode` are enums,
  so they are int-sized, not `int8_t` like `shape_mode` beside them. Left
  alone: narrowing them would change the record length and invalidate every
  saved preset. Worth doing behind a `CONFIG_STATE_VERSION` bump if the format
  is ever revised for another reason.

---

## Still open

Nothing from the review. The one item left hanging - whether the unused
`envelope.c` was work in progress or an abandoned experiment - was the second,
and it is gone.

---

## Behaviour changes to be aware of on hardware

Two things in this pass are deliberately not behaviour-preserving:

1. **Channel-to-channel triggering now needs >1.25V**, where before 0.31V
   sufficed. A channel driving another channel's sample & hold must swing
   properly; a very low-amplitude LFO used as a clock will stop working. This
   was the point of the change - the old number was an unrescaled copy - but it
   is the one thing worth checking on the module.
2. **A failed preset store now reports** (`ERR_PRESET_STORE`, scene button 4
   blinking) instead of flashing the same green as a successful one.

The mute ramp's timing is unchanged: `channel_apply_mute` runs at the end of
`engine_tick`, after the UX pass sets `muted[]`, which is where the old
`write_channel_dac` call sat relative to it. Moving it earlier shifted the ramp
by one tick and showed up as a one-line diff in `sim/flows/mute_channel.csv` -
a good sign the golden files are doing their job.
