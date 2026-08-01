# Changelog

## UI layer restructure

Separated the interaction layer from the engine and gave the recurring UI
concepts one implementation each.

### Added

- **MUT shift mode.** Channel button toggles that channel's mute on release.
  Gates the DAC write only, so a muted channel still feeds cross-modulation and
  still works as a trigger source. Ramps over 5ms so it does not click. Renders
  dim purple in every mode. Not persisted - clears on power cycle.
- **Confirmation feedback** (`ui_feedback.c`) for copy, clear, save, load,
  assign and scene A/B - same flash, same timing, colour by action class. Most
  of these previously gave no feedback at all.
- **Semantic palette** in `color_presets.h` plus `HUE_PURPLE`. The renderer
  names meanings, not colours. Brightness capped at `VAL_MED`.
- **Candidate highlighting** driven by the same predicate the dispatcher uses,
  so what blinks and what responds cannot disagree.
- Tests: `test_ui_input`, `test_ui_gestures`, `test_ui_select`,
  `test_ui_feedback`, `test_ui_mode`. The button path had no coverage before.

### Changed

- **Button gestures derived once** (`ui_input.c`) as edge events, with the
  press durations defined in one place. Events accumulate across ticks where
  the UX layer does not run, so a hold crossing its threshold is never missed.
- **`EngineState` split**: signal path only; interaction and view state moved
  to `UiState`. `compute_scenes_contribution` is now pure.
- **Selection model** (`ui_select.c`) replaces the assign type/target matrix
  and the mode-specific exceptions that lived in its callers.
- **Layered renderer** (`ui_render.c`) replaces the four `write_*_led`
  switches: base, context, transient value, confirmation. The base layer always
  writes, so stale pixels are impossible.
- **Channel LEDs** show the output level as their base in every mode; the value
  being edited surfaces on touch and decays. Entering a mode reveals every
  channel's state for that mode briefly.
- **Mode behaviour is a table** (`ui_mode.c`) read by both dispatch and render.
- **Shift-mode exit**: a mode's own button always exits; other ctrl buttons
  exit unless the mode opts out (only QNT does, because its buttons are a
  semitone keyboard). The exit tap no longer also changes the selected param.
- All UI timers age in one place against one dt.

### Fixed

- `scene.c` compared a hold against 10 microseconds where 10 milliseconds was
  meant.
- SAV drew its LED from the press timer but acted on the release timer, so the
  two disagreed at the boundary. Store now fires on the hold crossing.
- The QNT scene-button arm wrote no LED unless a trigger source was pending,
  leaving the framebuffer holding the previous frame.
- The MON channel arm fell through into the default, letting the value overlay
  clobber the mode colour.
- An error display returned early from the input update, suppressing the whole
  UX pass while it showed.
- `momentary_scene` was zero-initialised on hardware, reading as "scene 0 held"
  until the first UX pass corrected it.

### Removed

- `HwState.button_pressed_t` / `button_released_t`, `AssignType`, and the three
  ad hoc indicator timers they fed.
