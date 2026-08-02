# BMCV Firmware

# TODOs

- [x] Refactor update code & state access
  - [x] Consistent time (one UI dt, accumulated in ui_input and drained per UX pass)
  - [x] Consistent events (ui_input.h derives button gestures once; deltas accumulate until drained)
- [ ] PLL error term & sensitivity tuning
- [x] Verify output levels & ranges (AMP at full scale is the full +/-10V)
- [ ] Verify output precision
- [x] Add stepped random modes (smooth, semismooth & stepped variants?)
  - SHP morphs the pattern, MOD sets pattern length (3..64 steps, curated set)
  - Loops seamlessly, so it stays locked under the PLL
- [ ] Full Input & Channel Cross modulation support
- [x] UX: Fix long press control button in quantizer mode
- [x] UX: Consistent colors (semantic palette in color_presets.h)
- [x] UX: Consistent blinking (white mark for assignment; source and mute steady)
- [x] UX: Consistency with channel knob display & adjustments in shift mode(s)
- [x] UX: Consistent "mark" feature (transient value display, all modes)
- [x] UX: A shift mode's channel LEDs show that mode's setting, never the output
- [ ] UX: Tune UI_EDIT_DISPLAY / UI_FB_DURATION / MARK_BLINK_ON on hardware

# LED language

One fact per LED, and the same colour for the same idea on every page.

- **Setting states** are the base layer. Index 0 of every setting - disabled,
  default, neutral - is **purple**; **cyan** is continuous / level-following or
  multiplicative, **green** additive or half-way, **yellow** triggered / clocked
  / stepped, **red** reset. See `HUE_STATE_*` in `color_presets.h`. The output
  clamp is the one place brightness also carries meaning, because it is two
  facts on one LED.
- **White is assignment, and nothing else uses it.** A short white flash every
  1.6s over an element's own colour means "you can pick this". Once something is
  held, the places it can go are steady white with a short dropout, the held
  source is steady saturated purple, and everything else goes **dark** - if it
  cannot be pressed it does not light.
- **Output level** only appears when no shift mode is active. In a shift mode
  the encoder ring shows that mode's setting, or nothing if it has none.
- **Confirmations** are a brief flash on top: green wrote, purple cleared, cyan
  loaded. Red is errors only.

# Shift Modes

## STA/STB - Assign Scenes

- Scene Buttons:
  - Assign Scene 1-7 to A/B
- Channel Encoders (STA):
  - Stepped-random pattern length: how many steps a cycle is divided into, from
    a curated set (3..64). Per channel, not per scene - there is nothing between
    8 steps and 12, so a crossfaded value would be meaningless. Dark and inert
    on channels that are not in a stepped mode.
- Channel Encoders (STB):
  - Nothing (Maybe: transition/smoothing sensitivity)

## SYS - System Config & Channel Modes

- Scene Buttons:
  - Input mode (1-4)
  - TODO: Clock div
  - TODO: PLL sensitivity? Could also be per channel?
- Channel:
  - Set Waveshape mode: wavetable, three stepped-random flavours, PWM square.
    In PWM, SHP is the pulse width.

## SAV - Save & Load

- Scene Buttons:
  - Save to/Load from slot 1-7
- Channel:
  - Output clamp, per channel: +/-10V (purple), +/-5V (dim purple), 0..10V
    (green), 0..5V (dim green). A clamp and not a scaling, so the parameters
    keep meaning what they say. It describes what the module is patched into
    rather than what the patch is, which is why it is not per scene and why a
    channel clear leaves it alone.

## MON - Monitor & Mixing

Scene Buttons: - Display inputs (only the four that are inputs; the rest are
dark and inert)
Channel: - Press: select Input (TODO: Add other channels besides inputs) -
Rotate: Cross Modulation mode (Off, Add, Mult)

## QNT - Quantizer

Scene Buttons: - Quantizer State
Channel: - Rot: Quantizer Mode - Press: Assign Sample Trigger

## CLR - Clear Channels & Scenes

Everything on the page is one act, so it wears one colour: purple, marked white.

Scene Buttons: - Select scene to clear
Channel: - Tap clears that channel in the active scene. Hold clears the whole
channel: every scene, plus its MON routing and cross-modulation mode, its QNT
mode and trigger source, its shape mode and pattern length. Not the output
clamp.

## CPY - Copy Channels & Scenes

The same, in blue.

Scene Buttons: - Select scene to copy from/to
Channel: - Select channel to copy from/to

## MUT - Mute

Scene Buttons: - Nothing yet
Channel: - Press (toggles on release) mutes that channel's output; rotating is
absolute, right unmutes and left mutes, so a row can be muted by feel. Ramped
over 5ms so it does not click. The channel keeps running: it still feeds
cross-modulation and still works as a trigger source for other channels. A
muted channel shows dim purple here and with no mode active - on the other
pages that LED belongs to whatever the page edits. Mute clears on power cycle.

## No mode

Scene Buttons: - Hold to make that scene active momentarily
Channel: - The ring is a level meter, showing what the channel is putting out,
and borrows itself briefly to show the selected parameter while it is being
turned. Press to clear that parameter in the active scene, hold to clear it in
every scene - a purple flash confirms, longer for the wider one. Holding while
turning is the fine-adjust modifier and clears nothing.
