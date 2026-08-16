#!/usr/bin/env bash
# Does the DAC actually put out what the engine asked for, and does the ADC read
# it back? Patch a channel's output into an input jack and this answers both.
#
#   CH1 out ---> IN3        just dac-loopback
#   just dac-loopback --ch 0 --jack 2 -n 80
#
# `--ch` is the engine channel (0-based, so CH1 on the panel is 0) and `--jack`
# is the input jack (0-based, so IN3 is 2).
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
#   loose     they move but do not track - a patch cable in the wrong jack,
#             or an ADC reading a conversion that was not finished
#   ok        gain and R^2 as expected
#
# Needs the ST-Link attached and an ELF matching what is flashed.
set -uo pipefail

CH=0
JACK=2
N=60
ELF="build-rel/BMCVFirmware.elf"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ch) CH="$2"; shift 2 ;;
    --jack) JACK="$2"; shift 2 ;;
    -n) N="$2"; shift 2 ;;
    -e|--elf) ELF="$2"; shift 2 ;;
    *) echo "usage: dac-loopback.sh [--ch N] [--jack N] [-n COUNT] [-e ELF]" >&2; exit 2 ;;
  esac
done

cd "$(dirname "$0")/.."

echo "CH$((CH + 1)) out -> IN$((JACK + 1)), $N samples..." >&2

./scripts/hil.sh -n "$N" -e "$ELF" \
  "bmcv.engine_state.channels_gated_level[$CH]" \
  "bmcv.input.curr.input_state[$JACK]" \
  bmcv.engine_state.dac_fps \
  bmcv.engine_state.engine_fps \
| python3 -c '
import sys

ch_label   = "CH%d" % (int(sys.argv[1]) + 1)
jack_label = "IN%d" % (int(sys.argv[2]) + 1)

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

# Full scale is +/-32767 on both sides, so a span is quoted against 65534 to be
# read as a fraction of the range rather than as counts.
print()
print(f"commanded  {ch_label}   span {cmd_span:7.0f}  ({100 * cmd_span / 65534:.1f}% of range)")
print(f"measured   {jack_label}   span {adc_span:7.0f}  ({100 * adc_span / 65534:.1f}% of range)")
print()
print(f"gain       {gain:+.3f}   (measured per commanded)")
print(f"R^2        {r2:.4f}")
print()
print(f"dac_fps    {dac_fps:7.0f} transactions/s -> {dac_fps / 4:.0f} frames/s")
print(f"engine_fps {engine_fps:7.0f}")
print()

# A channel sitting still says nothing either way, so that is a refusal to judge
# rather than a pass.
if cmd_span < 0.05 * 65534:
    sys.exit(f"inconclusive: {ch_label} barely moved - give it an LFO with some depth")

if adc_span < 0.05 * cmd_span:
    print("verdict    DEAD - the engine is moving and the output is not")
    sys.exit(1)
if r2 < 0.9:
    print(f"verdict    LOOSE - it moves but does not track (R^2 {r2:.3f})")
    print("           check the patch cable is in that jack, then suspect the ADC window")
    sys.exit(1)

print(f"verdict    ok - tracking at gain {gain:+.3f}, R^2 {r2:.4f}")
' "$CH" "$JACK"
