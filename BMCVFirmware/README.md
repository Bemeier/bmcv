# BMCV Firmware

# TODOs

- [x] Refactor update code & state access
  - [x] Consistent time (one UI dt, accumulated in ui_input and drained per UX pass)
  - [x] Consistent events (ui_input.h derives button gestures once; deltas accumulate until drained)
- [ ] PLL error term & sensitivity tuning
- [ ] Verify output levels & ranges (also precision)
- [x] Add stepped random modes (smooth, semismooth & stepped variants?)
  - SHP morphs the pattern, MOD sets pattern length (3..64 steps, curated set)
  - Loops seamlessly, so it stays locked under the PLL
- [ ] Full Input & Channel Cross modulation support
- [x] UX: Fix long press control button in quantizer mode
- [x] UX: Consistent colors (semantic palette in color_presets.h)
- [x] UX: Consistent blinking (candidate pulses; source and mute steady)
- [x] UX: Consistency with channel knob display & adjustments in shift mode(s)
- [x] UX: Consistent "mark" feature (transient value display, all modes)
- [ ] UX: Tune UI_EDIT_DISPLAY / UI_FB_DURATION / MUTE_RAMP_MS on hardware
- [ ] UX: Check a muted channel that is also a copy target still reads clearly

# Shift Modes

## STA/STB - Assign Scenes

- Scene Buttons:
  - Assign Scene 1-7 to A/B
- Channel Encoders:
  - Nothing (Maybe: transition/smoothing sensitivity)

## SYS - System Config & Channel Modes

- Scene Buttons:
  - Input mode (1-4)
  - TODO: Clock div
  - TODO: PLL sensitivity? Could also be per channel?
- Channel:
  - Set Waveshape mode (Table, Stepped random, ...)

## SAV - Save & Load

- Scene Buttons:
  - Save to/Load from slot 1-7
- Channel:
  - ??? (Could put mute here?)

## MON - Monitor & Mixing

Scene Buttons: - Display inputs
Channel: - Press: select Input (TODO: Add other channels besides inputs) - Rotate: Cross Modulation mode (Add, Mult, ... TODO)

## QNT - Quantizer

Scene Buttons: - Quantizer State
Channel: - Rot: Quantizer Mode - Press: Assign Sample Trigger

## CLR - Clear Channels & Scenes

Scene Buttons: - Select scene to clear
Channel: - Select channel to clear

## CPY - Copy Channels & Scenes

Scene Buttons: - Select scene to copy from/to
Channel: - Select channel to copy from/to

## MUT - Mute

Scene Buttons: - Nothing yet
Channel: - Press (toggles on release) mutes that channel's output. Ramped over
5ms so it does not click. The channel keeps running: it still feeds
cross-modulation and still works as a trigger source for other channels. A
muted channel shows dim purple in every mode, not just this one, and mute
clears on power cycle.
