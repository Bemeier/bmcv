#!/usr/bin/env bash
# Does the DAC actually put out what the engine asked for, and does the ADC read
# it back? Patch a channel's output into an input jack and this answers both.
#
#   CH1 out ---> IN3        just dac-loopback
#   just dac-loopback --ch 1 --jack 3 -n 80
#
# `--ch` and `--jack` are the numbers printed on the panel, which is labelled
# from zero - CH0..CH7 and IN0..IN3 - and those are the array indices too.
# `channels_gated_level[]` is in panel order, and `input_state[]` is put in
# panel order by hw_setup's input_adc_idx, so neither needs mapping here.
#
# Why this exists: re-arming the DAC transfer from the SPI completion interrupt
# gave a stable transaction rate and a dead output - transfers running, nothing
# latching - and nothing in the firmware's own numbers said so. dac_fps looked
# better than ever. A loopback is the only reading that can tell "the bus is
# busy" apart from "the outputs are moving", and it is exactly the regression to
# watch for while the DAC's clock is being changed.
#
# It correlates rather than traces. Samples arrive about eight a second over
# SWD, which cannot follow an LFO - but both fields come out of a single
# instance read, so each (commanded, measured) pair is internally consistent
# however badly the sampling aliases. Scatter enough pairs across the waveform
# and they lie on a line whether or not they were taken in order, so the channel
# can be left running whatever it was running.
#
# What the verdict means:
#   dead      the outputs are not moving, whatever the bus is doing
#   smeared   full amplitude arrives but at the wrong instant. What a frame rate
#             below the engine rate looks like: the pin holds a value up to a
#             frame old while the commanded level has moved on, so the pairs
#             scatter around the line instead of lying on it
#   loose     the amplitude is wrong too - a cable in the wrong jack, or an ADC
#             reading a conversion that was not finished
#   ok        amplitude and timing both
#
# The span ratio is what separates smeared from loose, and it is why both are
# reported. A regression slope alone cannot: decorrelation drags the slope
# toward zero, so a perfectly scaled but badly timed output reports a gain of
# 0.86 with spans that match to half a percent - which is exactly what the
# firmware this was written against does.
#
# Needs the ST-Link attached and an ELF matching what is flashed.
set -uo pipefail

CH=1
JACK=3
N=60
ELF="build-rel/BMCVFirmware.elf"
SETUP=1

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ch) CH="$2"; shift 2 ;;
    --jack) JACK="$2"; shift 2 ;;
    -n) N="$2"; shift 2 ;;
    -e|--elf) ELF="$2"; shift 2 ;;
    --no-setup) SETUP=0; shift ;;
    *) echo "usage: dac-loopback.sh [--ch N] [--jack N] [-n COUNT] [-e ELF] [--no-setup]" >&2; exit 2 ;;
  esac
done

cd "$(dirname "$0")/.."

# Arm the channel before measuring, rather than hoping somebody left an LFO on
# it. A reset reloads the saved preset, so a sine dialled in by hand disappears
# the moment the module is reflashed - and this check then reports
# `inconclusive` for a reason that has nothing to do with the DAC.
#
# Written to *every* scene, not the active one. The crossfader blends all seven
# and its position is whatever it was left at, so a value set in one scene
# arrives attenuated by however far the fader sits from it. The same value
# everywhere makes the reading independent of the fader.
#
# A plain full-scale LFO: SHAPE_LFO, no phase distortion, no offset, AMP at
# full. The rate is left as the clock's own - a slow shape is what makes the
# correlation sharp, and anything fast enough to alias against an eight-samples-
# a-second reader would be measuring the reader.
if [[ "$SETUP" == "1" ]]; then
  pokes=("bmcv.engine_config.channel_state[$CH].shape_mode=0" "bmcv.ui_state.muted[$CH]=0")
  for s in 0 1 2 3 4 5 6; do
    pokes+=("bmcv.engine_config.channel_state[$CH].params[$s][0]=0")     # FRQ: x1 the beat
    pokes+=("bmcv.engine_config.channel_state[$CH].params[$s][1]=0")     # SHP
    pokes+=("bmcv.engine_config.channel_state[$CH].params[$s][2]=0")     # MOD
    pokes+=("bmcv.engine_config.channel_state[$CH].params[$s][3]=0")     # PHS
    pokes+=("bmcv.engine_config.channel_state[$CH].params[$s][4]=32767") # AMP, full swing
    pokes+=("bmcv.engine_config.channel_state[$CH].params[$s][5]=0")     # OFS
  done

  echo "arming CH$CH as a full-scale LFO..." >&2
  if ! ./scripts/hil-poke.sh -e "$ELF" "${pokes[@]}" >/dev/null; then
    echo "dac-loopback: could not set the channel up; use --no-setup to measure it as it is" >&2
    exit 1
  fi
fi

echo "CH$CH out -> IN$JACK, $N samples..." >&2

./scripts/hil.sh -n "$N" -e "$ELF" \
  "bmcv.engine_state.channels_gated_level[$CH]" \
  "bmcv.input.curr.input_state[$JACK]" \
  bmcv.engine_state.dac_fps \
  bmcv.engine_state.engine_fps \
| python3 -c '
import sys

ch_label   = "CH" + sys.argv[1]
jack_label = "IN" + sys.argv[2]

rows = [l.split("\t") for l in sys.stdin.read().splitlines()[1:] if l.strip()]
if len(rows) < 8:
    sys.exit("dac-loopback: too few samples came back to say anything")

cmd = [float(r[1]) for r in rows]
adc = [float(r[2]) for r in rows]
dac_fps    = sum(float(r[3]) for r in rows) / len(rows)
engine_fps = sum(float(r[4]) for r in rows) / len(rows)

n = len(cmd)
mx, my = sum(cmd) / n, sum(adc) / n
sxx = sum((x - mx) ** 2 for x in cmd)
syy = sum((y - my) ** 2 for y in adc)
sxy = sum((x - mx) * (y - my) for x, y in zip(cmd, adc))

gain = sxy / sxx if sxx else 0.0
r2   = (sxy * sxy) / (sxx * syy) if sxx and syy else 0.0

cmd_span = max(cmd) - min(cmd)
adc_span = max(adc) - min(adc)
span_ratio = adc_span / cmd_span if cmd_span else 0.0

# Full scale is +/-32767 on both sides, so a span is quoted against 65534 to be
# read as a fraction of the range rather than as counts.
print()
print(f"commanded  {ch_label}   span {cmd_span:7.0f}  ({100 * cmd_span / 65534:.1f}% of range)")
print(f"measured   {jack_label}   span {adc_span:7.0f}  ({100 * adc_span / 65534:.1f}% of range)")
print()
print(f"span ratio {span_ratio:.3f}   (measured span per commanded span - amplitude)")
print(f"gain       {gain:+.3f}   (regression slope - amplitude AND timing)")
print(f"R^2        {r2:.4f}   (how much of the output the commanded level explains)")
print()
print(f"dac_fps    {dac_fps:7.0f} transactions/s -> {dac_fps / 4:.0f} frames/s")
print(f"engine_fps {engine_fps:7.0f}")
print()

# A channel sitting still says nothing either way, so that is a refusal to judge
# rather than a pass.
if cmd_span < 0.05 * 65534:
    sys.exit(f"inconclusive: {ch_label} barely moved - give it an LFO with some depth")

if span_ratio < 0.05:
    print("verdict    DEAD - the engine is moving and the output is not")
    sys.exit(1)

if r2 < 0.9:
    # The amplitude arriving intact while the correlation does not is timing,
    # not wiring, and the two want opposite things looked at next.
    if 0.85 < span_ratio < 1.15:
        print(f"verdict    SMEARED - full amplitude ({span_ratio:.3f}) at the wrong instant (R^2 {r2:.3f})")
        print(f"           dac_fps {dac_fps:.0f} against engine_fps {engine_fps:.0f}: each output is")
        print(f"           refreshed {dac_fps / 4:.0f} times a second while the level moves {engine_fps:.0f} times")
    else:
        print(f"verdict    LOOSE - amplitude {span_ratio:.3f} and R^2 {r2:.3f} are both wrong")
        print("           check the patch cable is in that jack, then suspect the ADC window")
    sys.exit(1)

print(f"verdict    ok - amplitude {span_ratio:.3f}, R^2 {r2:.4f}")
' "$CH" "$JACK"
