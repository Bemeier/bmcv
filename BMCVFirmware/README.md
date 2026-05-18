# BMCV Firmware

# TODOs

- [ ] Refactor update code & state access
  - [ ] Consistent time (commulative delta time inaccurate for timeouts)
  - [ ] Consistent events (add delta until cleared? Button handling until cleared?)
- [ ] PLL error term & sensitivity tuning
- [ ] Verify output levels & ranges (also precision)
- [ ] Add stepped random modes (smooth, semismooth & stepped variants?)
- [ ] Full Input & Channel Cross modulation support
- [ ] UX: Fix long press control button in quantizer mode
- [ ] UX: Consistent colors
- [ ] UX: Consistent blinking
- [ ] UX: Consistency with channel knob display & adjustments in shift mode(s)
- [ ] UX: Consistent "mark" feature (in shift & channel modes)

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

## MUT - Mute & ???

Scene Buttons: - ???
Channel: - Mute channel output completely
