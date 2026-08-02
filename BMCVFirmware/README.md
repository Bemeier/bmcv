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
- [x] MOD as a second shaping parameter in every shape mode
- [x] UX: Fix long press control button in quantizer mode
- [x] UX: Consistent colors (semantic palette in color_presets.h)
- [x] UX: Consistent blinking (white mark for assignment; source and mute steady)
- [x] UX: Consistency with channel knob display & adjustments in shift mode(s)
- [x] UX: Consistent "mark" feature (transient value display, all modes)
- [x] UX: A shift mode's channel LEDs show that mode's setting, never the output
- [ ] UX: Tune UI_EDIT_DISPLAY / UI_FB_DURATION / MARK_BLINK_ON on hardware

# LED language

One fact per LED, and the same colour for the same idea on every page.

- **Ctrl-page settings clamp, they do not wrap.** Each is a short list of
  unrelated states with its default at index 0, so spinning an encoder fully
  left resets it and you can do that without looking. Rolling off one end into
  the other would be the largest change on the page.
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
- **A held press shows what letting go would do**, in the colour of that act and
  blinking. Where holding longer does something wider - clearing every scene
  rather than this one - the same colour gets brighter once that threshold is
  crossed. Nothing commits until the release.

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
  - Set Waveshape mode: wavetable, stepped random, PWM square.

# Shape modes

SHP picks the shape; **MOD is the second axis**, and what it means per mode:

| mode | SHP | MOD |
|---|---|---|
| wavetable | table slice | **skew** - leans the waveform early or late |
| stepped random | pattern morph | **density** - how often a step ties to the previous value instead of taking a new one |
| PWM | pulse width | **envelope** - ramp time on one edge |

Three modes, not five: the stepped variants were one algorithm at three hold
values, which made a long list out of one idea. If hold comes back it should
come back as a per-channel setting, the way pattern length did.

Where MOD leans something, the sign is the same everywhere: **negative leans
early, positive leans late.** Stepped random is the exception - density has no
early or late - and there negative is busy, positive is sparse.

- **Wavetable skew** is a rational phase warp that fixes both cycle endpoints
  and is monotone, so the loop still closes; it turns a sine into a skewed sine
  and a triangle into a ramp.
- **Stepped density** at MOD 0 is the 30% tie probability the pattern was
  designed around: fully left every step takes a new value, fully right the
  cycle is a handful of long notes. Faded in on short patterns, where one tie
  flattens too much. The normalisation table carries a probability axis so the
  correction holds as MOD moves.
- **PWM envelope**: MOD 0 is a hard gate. Negative snaps up and decays through
  the off-time; positive swells across the on-time and drops at the end. Each
  ramp is confined to its own segment, so the width still means what it says at
  either extreme, and the ramps are curved - concave attack, convex decay - so
  the negative side reads as an envelope rather than a triangle. A narrow pulse
  with MOD hard left is a percussive envelope locked to the beat.

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

Ctrl Buttons: - Each parameter button glows very dim in its own colour, so the
row says which colour means which parameter; the selected one is several times
brighter. All dark while a shift mode is running.
Scene Buttons: - Hold to make that scene active momentarily
Channel: - The ring is a level meter, showing what the channel is putting out.
Touching any encoder, or tapping a parameter button (including the one already
selected), shows that parameter on **all eight** for a couple of seconds - the
point of seeing it is to compare the channels against each other - then decays
back to the meter. Leaving a shift mode does not: that lands straight back on
the meter.

Press to clear the parameter in the active scene, hold to clear it in every
scene - purple while held, brighter for the wider one, and a purple flash on
release. Holding while turning is the fine-adjust modifier and clears nothing.
