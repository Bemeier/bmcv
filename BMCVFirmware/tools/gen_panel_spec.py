#!/usr/bin/env python3
"""Generate the BMCV panel spec from the hardware repo's KiCad output.

The panel layout is *derived*, never typed in. Three sources are merged:

  BMCV.kicad_pcb  board outline, component placements and body sizes
  netlist.ipc     connectivity (the WS2812 chain order, jack->converter paths)
  dump_hw_setup   the firmware's own HwSetup/UxSetup tables (the index spaces)

so the spec cannot drift from either the board or the firmware. Anything that
genuinely is not in the CAD data lives in panel/overrides.json and is listed
under "assumptions" in the output.

Usage:  tools/gen_panel_spec.py [--hw-repo ..] [--dump build-native/dump_hw_setup]
"""

import argparse
import collections
import json
import os
import re
import subprocess
import sys

# --------------------------------------------------------------------------
# BMCV.kicad_pcb
# --------------------------------------------------------------------------


def read_pcb(path):
    """Board outline, component positions and body sizes, from the PCB itself.

    Not from production/positions.csv. That export gives *pick-and-place
    centroids*, which for a footprint whose pads are not symmetric about its
    origin sit to one side - and the offset flips sign with the footprint's
    rotation. The jacks are placed on a uniform 13mm grid but alternate
    90/270 degrees, so the centroids came out at 14.03/11.97mm alternating and
    the panel was visibly wrong. The footprint's own (at ...) is the placement.

    Returns (board, positions, extents) with everything in panel space: x from
    the left edge, y from the top edge, millimetres.
    """
    src = open(path, encoding="utf-8", errors="replace").read()

    def blocks(token):
        for m in re.finditer(re.escape(token), src):
            start = m.start()
            depth, i = 0, start
            while i < len(src):
                if src[i] == "(":
                    depth += 1
                elif src[i] == ")":
                    depth -= 1
                    if depth == 0:
                        break
                i += 1
            yield src[start : i + 1]

    # Board outline: the bounding box of everything on Edge.Cuts.
    xs, ys = [], []
    for pat in (
        r"\(gr_line\s*\(start ([\-\d\.]+) ([\-\d\.]+)\)\s*\(end ([\-\d\.]+) ([\-\d\.]+)\)[\s\S]{0,200}?\(layer \"Edge\.Cuts\"",
        r"\(gr_rect\s*\(start ([\-\d\.]+) ([\-\d\.]+)\)\s*\(end ([\-\d\.]+) ([\-\d\.]+)\)[\s\S]{0,200}?\(layer \"Edge\.Cuts\"",
        r"\(arc\s*\(start ([\-\d\.]+) ([\-\d\.]+)\)[\s\S]{0,80}?\(end ([\-\d\.]+) ([\-\d\.]+)\)[\s\S]{0,200}?\(layer \"Edge\.Cuts\"",
    ):
        for a, b, c, d in re.findall(pat, src):
            xs += [float(a), float(c)]
            ys += [float(b), float(d)]
    if not xs:
        raise SystemExit("no Edge.Cuts geometry found in the PCB")
    ox, oy = min(xs), min(ys)
    board = {"width_mm": round(max(xs) - ox, 3), "height_mm": round(max(ys) - oy, 3)}

    positions, extents = {}, {}
    for blk in blocks("(footprint "):
        ref = re.search(r'\(property "Reference" "([^"]+)"', blk)
        at = re.search(r"\(at ([\-\d\.]+) ([\-\d\.]+)(?: ([\-\d\.]+))?", blk)
        if not (ref and at):
            continue
        name = ref.group(1)
        layer = re.search(r'\(layer "([^"]+)"', blk)
        positions[name] = (
            round(float(at.group(1)) - ox, 3),
            round(float(at.group(2)) - oy, 3),
            float(at.group(3) or 0.0),
            "bottom" if layer and layer.group(1).startswith("B.") else "top",
        )

        bx, by = [], []
        for a, b, c, d, lay in re.findall(
            r"\(fp_line\s*\(start ([\-\d\.]+) ([\-\d\.]+)\)\s*\(end ([\-\d\.]+) ([\-\d\.]+)\)"
            r"[\s\S]{0,160}?\(layer \"([^\"]+)\"",
            blk,
        ):
            if "Fab" in lay or "CrtYd" in lay or "SilkS" in lay:
                bx += [float(a), float(c)]
                by += [float(b), float(d)]
        if bx:
            extents[name] = {"w": round(max(bx) - min(bx), 3), "h": round(max(by) - min(by), 3)}

    return board, positions, extents


# --------------------------------------------------------------------------
# netlist.ipc
# --------------------------------------------------------------------------

# IPC-D-356: record type 327 is a component pad. Net name occupies columns
# 3..20, then refdes and -pin. Long net names are truncated by the exporter,
# but consistently, so they still compare equal to themselves.
_PAD_RE = re.compile(r"^(\S+)\s+-(\d+)")

POWER_NETS = {"GND", "+12V", "-12V", "+5V", "+3V3", "3.3V", "-5V", "AGND"}

# OPA4171 quad: input pin -> the output pin of the same amplifier. Used to
# walk *with* the signal through a buffer stage.
OPAMP_IN_TO_OUT = {2: 1, 3: 1, 5: 7, 6: 7, 9: 8, 10: 8, 12: 14, 13: 14}


class Netlist:
    def __init__(self, path):
        self.net2pins = collections.defaultdict(list)
        self.ref2pins = collections.defaultdict(list)
        with open(path, encoding="utf-8", errors="replace") as fh:
            for line in fh:
                if not line.startswith("327"):
                    continue
                net = line[3:20].strip()
                m = _PAD_RE.match(line[20:])
                if not m:
                    continue
                ref, pin = m.group(1), int(m.group(2))
                self.net2pins[net].append((ref, pin))
                self.ref2pins[ref].append((net, pin))

    def net_of(self, ref, pin):
        return next((n for n, p in self.ref2pins[ref] if p == pin), None)

    def _resistor_other_side(self, ref, net):
        if ref[0] != "R" or ref.startswith("RV") or len(self.ref2pins[ref]) != 2:
            return None
        return next((n for n, _ in self.ref2pins[ref] if n != net), None)

    def walk(self, start_net, is_goal, max_hops=12):
        """BFS from a net, stepping through series resistors and forward
        through opamp buffers. Skips ESD diodes (RV*), caps and power nets."""
        seen = {start_net}
        frontier = [(start_net, [start_net])]
        for _ in range(max_hops):
            nxt = []
            for net, path in frontier:
                hit = is_goal(net, path)
                if hit is not None and net != start_net:
                    return hit, path
                for ref, pin in self.net2pins[net]:
                    tgt = None
                    if ref.startswith(("RV", "C", "D", "Y")):
                        continue
                    if ref[0] == "R":
                        tgt = self._resistor_other_side(ref, net)
                    elif ref.startswith(("U38", "U39", "U40")) and pin in OPAMP_IN_TO_OUT:
                        tgt = self.net_of(ref, OPAMP_IN_TO_OUT[pin])
                    if tgt and tgt not in seen and tgt not in POWER_NETS:
                        seen.add(tgt)
                        nxt.append((tgt, path + [f"{ref}.{pin}", tgt]))
            frontier = nxt
        return None, None

    # -- derived hardware facts ------------------------------------------

    def led_chain(self, led_refs):
        """WS2811 data-chain order -> [designator]; index in the list is the
        LED index the firmware addresses (led_fb.c writes 0..LED_COUNT-1)."""
        din, dout = {}, {}
        for led in led_refs:
            by_pin = {p: n for n, p in self.ref2pins[led]}
            din[led], dout[led] = by_pin.get(6), by_pin.get(5)

        # DO -> (optional series resistor) -> next DIN
        nxt = {}
        for led in led_refs:
            if not dout[led]:
                continue
            reachable = {dout[led]}
            for ref, _ in self.net2pins[dout[led]]:
                other = self._resistor_other_side(ref, dout[led])
                if other:
                    reachable.add(other)
            for other in led_refs:
                if other != led and din[other] in reachable:
                    nxt[led] = other

        heads = [l for l in led_refs if l not in set(nxt.values())]
        if len(heads) != 1:
            raise SystemExit(f"LED chain: expected exactly one head, got {heads}")
        chain, cur = [], heads[0]
        while cur and cur not in chain:
            chain.append(cur)
            cur = nxt.get(cur)
        if len(chain) != len(led_refs):
            raise SystemExit(f"LED chain: walked {len(chain)} of {len(led_refs)} LEDs")
        return chain

    def dac_to_jack(self):
        """DAC buffer index (as dacadc_write() numbers them) -> jack designator.

        dac_init() lays the SPI buffer out as idx0,idx1 = CHA, idx2,idx3 = CHB
        and so on, and dacadc_dma_next() ships 6 bytes - two 3-byte commands -
        per transfer.

        The two AD5754Rs are daisy-chained, not addressed separately: the MCU
        drives U34's SDIN, U34's SDO feeds U35's SDIN, and both share one SYNC
        line. So a 6-byte burst shifts the *first* command all the way through
        U34 into U35, and the second command stays in U34. Even indices are
        therefore U35 and odd indices U34 - the opposite of what the buffer
        order suggests at a glance.
        """
        jack_re = re.compile(r"^NET-\(J(\d+)-PADT\)$")
        goal = lambda net, _path: (jack_re.match(net).group(1) if jack_re.match(net) else None)
        out_pin = {"A": 3, "B": 4, "C": 23, "D": 22}
        result = {}
        for idx in range(8):
            chip = "U35" if idx % 2 == 0 else "U34"
            letter = "ABCD"[idx // 2]
            start = self.net_of(chip, out_pin[letter])
            jack, _ = self.walk(start, goal)
            if jack is None:
                raise SystemExit(f"dac{idx}: no path from {chip} CH{letter} to a jack")
            result[idx] = f"J{jack}"
        return result

    def jack_to_adc_input(self):
        """Input jack designator -> AD7367 input name (A1/A2/B1/B2)."""
        adc_re = re.compile(r"^IN_BUFFERED_([AB][12])$")
        goal = lambda net, _p: (adc_re.match(net).group(1) if adc_re.match(net) else None)
        result = {}
        for j in range(9, 13):
            name, _ = self.walk(f"NET-(J{j}-PADT)", goal)
            if name is None:
                raise SystemExit(f"J{j}: no path to an ADC input")
            result[f"J{j}"] = name
        return result

    def tactile_to_button_net(self, refs):
        """Unlit tactile switch designator -> the MCU net it drives."""
        mcu_nets = {"BTN_MENU_2", "BTN_MENU_3", "BOOT"}
        goal = lambda net, _p: (net if net in mcu_nets else None)
        out = {}
        for sw in refs:
            for net, _pin in self.ref2pins[sw]:
                if net in POWER_NETS:
                    continue
                hit, _ = self.walk(net, goal)
                if hit:
                    out[sw] = hit
                    break
        return out


# --------------------------------------------------------------------------
# merge
# --------------------------------------------------------------------------

SEMITONE_NAMES = ["C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"]

# Ctrl button names are NOT listed here: they are shift-mode names, and the
# firmware's ui_mode.c table is the one place they are written. dump_hw_setup
# emits them as "ctrl_names".

# The first six ctrl buttons double as the parameter selector: with no shift
# mode active, tapping one sets selected_param to its id (ctrl_button.c), and
# ChannelParameters is declared in that order. The remaining three are actions,
# not parameters.
PARAM_NAMES = ["FRQ", "SHP", "MOD", "PHS", "AMP", "OFS"]


def nearest(pos, xy, candidates, tol=1.0):
    """The candidate designator co-located with xy. Every WS2811 sits directly
    behind the control it lights, so this is how LED index -> control is
    resolved without trusting any hand-written table."""
    best, best_d = None, tol
    for ref in candidates:
        if ref not in pos:
            continue
        dx, dy = pos[ref][0] - xy[0], pos[ref][1] - xy[1]
        d = (dx * dx + dy * dy) ** 0.5
        if d < best_d:
            best, best_d = ref, d
    return best


def build(pcb_board, pos, extents, net, hw_dump, overrides):
    hw = hw_dump["hw_setup"]
    ux = hw_dump["ux_setup"]
    counts = hw_dump["counts"]

    led_refs = [f"U{i}" for i in range(9, 30)]
    chain = net.led_chain(led_refs)  # LED index -> designator
    if len(chain) != counts["leds"]:
        raise SystemExit(f"chain has {len(chain)} LEDs, firmware expects {counts['leds']}")

    encoder_refs = [f"U{i}" for i in range(1, 9)]
    switch_refs = [f"SW{i}" for i in range(1, 14)]
    tactile_refs = ["SW14", "SW15", "SW16"]

    led_pos = {i: (pos[d][0], pos[d][1]) for i, d in enumerate(chain)}

    # LED -> the control it sits behind.
    led_control = {}
    for i, xy in led_pos.items():
        led_control[i] = nearest(pos, xy, encoder_refs + switch_refs)
        if led_control[i] is None:
            raise SystemExit(f"LED {i} ({chain[i]}) has no control within 1mm")

    # Control index spaces -> designator, entirely via the LED pairing.
    button_part, encoder_part = {}, {}
    for c in ux["channels"]:
        part = led_control[c["led"]]
        button_part[c["button"]] = part
        encoder_part[c["encoder"]] = part
    for c in ux["ctrl_buttons"]:
        if c["led"] >= 0:
            button_part[c["button"]] = led_control[c["led"]]
    for s in ux["scenes"]:
        button_part[s["button"]] = led_control[s["led"]]

    # The three unlit tactiles have no LED, so they come from the netlist.
    net_to_button = {  # mcu_read_buttons() order, mcp.c
        "BTN_MENU_2": 21,
        "BOOT": 22,
        "BTN_MENU_3": 23,
    }
    for sw, mcu_net in net.tactile_to_button_net(tactile_refs).items():
        button_part[net_to_button[mcu_net]] = sw

    missing = [b for b in range(counts["buttons"]) if b not in button_part]
    if missing:
        raise SystemExit(f"buttons with no physical part resolved: {missing}")

    # Roles per button, from the firmware's own tables.
    roles = collections.defaultdict(dict)
    for c in ux["channels"]:
        roles[c["button"]]["channel"] = c["id"]
    for c in ux["ctrl_buttons"]:
        roles[c["button"]]["ctrl"] = c["id"]
        roles[c["button"]]["ctrl_name"] = hw_dump["ctrl_names"][c["id"]]
        if c["id"] < len(PARAM_NAMES):
            roles[c["button"]]["param"] = c["id"]
            roles[c["button"]]["param_name"] = PARAM_NAMES[c["id"]]
    for s in ux["scenes"]:
        roles[s["button"]]["scene"] = s["id"]
    for q in ux["quantizer_semitones"]:
        roles[q["button"]]["semitone"] = q["id"]
        roles[q["button"]]["semitone_name"] = SEMITONE_NAMES[q["id"]]

    button_led = {}
    for c in ux["channels"]:
        button_led[c["button"]] = c["led"]
    for c in ux["ctrl_buttons"]:
        button_led.setdefault(c["button"], c["led"] if c["led"] >= 0 else None)
    for s in ux["scenes"]:
        button_led[s["button"]] = s["led"]

    def xy(ref):
        return [round(pos[ref][0], 3), round(pos[ref][1], 3)]

    buttons = []
    for b in range(counts["buttons"]):
        part = button_part[b]
        kind = "encoder_push" if part in encoder_refs else ("rgb_switch" if part in switch_refs else "tactile")
        buttons.append(
            {
                "index": b,
                "designator": part,
                "pos_mm": xy(part),
                "led": button_led.get(b),
                "kind": kind,
                "roles": dict(roles[b]),
            }
        )

    encoders = []
    for c in ux["channels"]:
        part = encoder_part[c["encoder"]]
        encoders.append(
            {
                "index": c["encoder"],
                "designator": part,
                "pos_mm": xy(part),
                "push_button": c["button"],
                "led": c["led"],
                "channel": c["id"],
            }
        )
    encoders.sort(key=lambda e: e["index"])

    leds = [
        {
            "index": i,
            "designator": chain[i],
            "pos_mm": xy(chain[i]),
            "attached_to": led_control[i],
        }
        for i in range(len(chain))
    ]

    # Outputs: channel -> dac index (firmware) -> jack (netlist).
    dac_jack = net.dac_to_jack()
    outputs = []
    for c in ux["channels"]:
        jack = dac_jack[c["dac_channel"]]
        outputs.append(
            {
                "index": c["id"],
                "designator": jack,
                "pos_mm": xy(jack),
                "dac_index": c["dac_channel"],
                "channel": c["id"],
            }
        )

    # Inputs: jack -> AD7367 input (netlist) -> adc index (ADDR phase, an
    # assumption) -> firmware input index (hw_setup.input_adc_idx).
    adc_name_to_idx = overrides["adc_input_order"]
    jack_adc_name = net.jack_to_adc_input()
    adc_idx_to_jack = {adc_name_to_idx[name]: jack for jack, name in jack_adc_name.items()}
    inputs = []
    for i, adc_idx in enumerate(hw["input_adc_idx"]):
        jack = adc_idx_to_jack[adc_idx]
        inputs.append(
            {
                "index": i,
                "designator": jack,
                "pos_mm": xy(jack),
                "adc_index": adc_idx,
                "adc_input": jack_adc_name[jack],
            }
        )

    board = pcb_board
    panel = dict(overrides["panel"])
    # The board size comes from Edge.Cuts, so the panel offset that centres it
    # follows rather than being typed in.
    panel["board_offset_mm"] = [
        round((panel["width_mm"] - board["width_mm"]) / 2, 3),
        round((panel["height_mm"] - board["height_mm"]) / 2, 3),
    ]

    # Slider: the axis and body come from the footprint, so only the wiper
    # travel is an assumption.
    slider = dict(overrides["slider"])
    slider_ref = slider["designator"]
    body = extents.get(slider_ref)
    if body:
        slider["body_mm"] = [body["w"], body["h"]]
        slider["axis"] = "x" if body["w"] >= body["h"] else "y"
    slider["pos_mm"] = xy(slider_ref)

    return {
        "schema": 1,
        "generated_by": "tools/gen_panel_spec.py",
        "generated_from": {
            "kicad_pcb": overrides["_kicad_pcb"],
            "netlist_ipc": overrides["_netlist_ipc"],
            "hw_setup": "tools/dump_hw_setup.c (linked against the firmware's own tables)",
        },
        "assumptions": overrides["assumptions"],
        "board": board,
        "panel": panel,
        "ranges": hw_dump["ranges"],
        "encoders": encoders,
        "buttons": buttons,
        "leds": leds,
        "slider": slider,
        "outputs": outputs,
        "inputs": inputs,
    }


# --------------------------------------------------------------------------
# SVG
# --------------------------------------------------------------------------


def render_svg(spec):
    """A plain, functional panel: correct geometry, no artwork pass. It exists
    so the layout can be eyeballed and so the web sim has a base to draw over."""
    pw = spec["panel"]["width_mm"]
    ph = spec["panel"]["height_mm"]
    ox, oy = spec["panel"]["board_offset_mm"]

    def px(p):
        return p[0] + ox, p[1] + oy

    out = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{pw}mm" height="{ph}mm" '
        f'viewBox="0 0 {pw} {ph}">',
        "<style>",
        "  .panel{fill:#22252b}.jack{fill:#111;stroke:#8a8f98;stroke-width:.4}",
        "  .enc{fill:#3b4048;stroke:#c8ccd4;stroke-width:.5}",
        "  .sw{fill:#2c3038;stroke:#c8ccd4;stroke-width:.4}",
        "  .tact{fill:#2c3038;stroke:#8a8f98;stroke-width:.4}",
        "  .slider{fill:#1a1c21;stroke:#8a8f98;stroke-width:.4}",
        "  .knob{fill:#5a626e;stroke:#c8ccd4;stroke-width:.35}",
        "  text{font-family:sans-serif;fill:#dfe3ea;text-anchor:middle;dominant-baseline:middle}",
        "  .lbl{font-size:1.9px}.sub{font-size:1.5px;fill:#9aa1ac}",
        "</style>",
        f'<rect class="panel" x="0" y="0" width="{pw}" height="{ph}" rx="1.25"/>',
    ]

    for jacks, tag in ((spec["outputs"], "OUT"), (spec["inputs"], "IN")):
        for j in jacks:
            x, y = px(j["pos_mm"])
            out.append(f'<circle class="jack" cx="{x:.3f}" cy="{y:.3f}" r="3.0"/>')
            out.append(f'<text class="sub" x="{x:.3f}" y="{y + 4.6:.3f}">{tag}{j["index"]}</text>')

    for e in spec["encoders"]:
        x, y = px(e["pos_mm"])
        out.append(f'<circle class="enc" cx="{x:.3f}" cy="{y:.3f}" r="6.0"/>')
        out.append(f'<line x1="{x:.3f}" y1="{y:.3f}" x2="{x:.3f}" y2="{y - 5:.3f}" stroke="#c8ccd4" stroke-width=".6"/>')
        out.append(f'<text class="lbl" x="{x:.3f}" y="{y + 8.6:.3f}">CH{e["channel"]}</text>')

    s = spec["slider"]
    sx, sy = px(s["pos_mm"])
    trav = s["travel_mm"]
    if s.get("axis", "x") == "x":
        out.append(f'<rect class="slider" x="{sx - trav / 2:.3f}" y="{sy - 3:.3f}" width="{trav}" height="6" rx="2"/>')
        out.append(f'<rect class="knob" x="{sx - 2:.3f}" y="{sy - 4.2:.3f}" width="4" height="8.4" rx="1.2"/>')
        out.append(f'<text class="sub" x="{sx:.3f}" y="{sy + 7.6:.3f}">SCENE</text>')
    else:
        out.append(f'<rect class="slider" x="{sx - 3:.3f}" y="{sy - trav / 2:.3f}" width="6" height="{trav}" rx="2"/>')
        out.append(f'<rect class="knob" x="{sx - 4.2:.3f}" y="{sy - 2:.3f}" width="8.4" height="4" rx="1.2"/>')
        out.append(f'<text class="sub" x="{sx:.3f}" y="{sy + trav / 2 + 3:.3f}">SCENE</text>')

    for b in spec["buttons"]:
        if b["kind"] == "encoder_push":
            continue
        x, y = px(b["pos_mm"])
        if b["kind"] == "rgb_switch":
            out.append(f'<rect class="sw" x="{x - 4.5:.3f}" y="{y - 4.5:.3f}" width="9" height="9" rx="1.4"/>')
            r = b["roles"]
            top = r.get("ctrl_name") or (f'S{r["scene"]}' if "scene" in r else "")
            bot = r.get("semitone_name", "")
            if top:
                out.append(f'<text class="lbl" x="{x:.3f}" y="{y - 1.4:.3f}">{top}</text>')
            if bot:
                out.append(f'<text class="sub" x="{x:.3f}" y="{y + 2.0:.3f}">{bot}</text>')
        else:
            out.append(f'<rect class="tact" x="{x - 3:.3f}" y="{y - 3:.3f}" width="6" height="6" rx="1"/>')
            out.append(f'<text class="sub" x="{x:.3f}" y="{y:.3f}">{b["roles"].get("ctrl_name", "")}</text>')

    out.append("</svg>")
    return "\n".join(out) + "\n"


# --------------------------------------------------------------------------
# C header
# --------------------------------------------------------------------------


def render_header(spec):
    def arr(name, rows):
        body = "\n".join(rows)
        return f"static const {name}\n{{\n{body}\n}};\n"

    lines = [
        "// Generated by tools/gen_panel_spec.py - do not edit.",
        "// Panel geometry in mm, origin at the board's top-left corner.",
        "// Regenerate with `just panel`.",
        "#ifndef BMCV_PANEL_LAYOUT_H_",
        "#define BMCV_PANEL_LAYOUT_H_",
        "",
        "#include <stdint.h>",
        "",
        "typedef struct { float x, y; } PanelPoint;",
        "",
        f'#define PANEL_BOARD_W_MM {spec["board"]["width_mm"]}f',
        f'#define PANEL_BOARD_H_MM {spec["board"]["height_mm"]}f',
        f'#define PANEL_HP {spec["panel"]["hp"]}',
        "",
    ]

    rows = [f'  {{{b["pos_mm"][0]:.3f}f, {b["pos_mm"][1]:.3f}f}}, // {b["index"]:2d} {b["designator"]}' for b in spec["buttons"]]
    lines.append(arr(f'PanelPoint panel_button_pos[{len(spec["buttons"])}] =', rows))

    leds = spec["leds"]
    rows = [f'  {{{l["pos_mm"][0]:.3f}f, {l["pos_mm"][1]:.3f}f}}, // {l["index"]:2d} {l["designator"]} @ {l["attached_to"]}' for l in leds]
    lines.append(arr(f"PanelPoint panel_led_pos[{len(leds)}] =", rows))

    encs = sorted(spec["encoders"], key=lambda e: e["index"])
    rows = [f'  {{{e["pos_mm"][0]:.3f}f, {e["pos_mm"][1]:.3f}f}}, // {e["index"]} {e["designator"]} ch{e["channel"]}' for e in encs]
    lines.append(arr(f"PanelPoint panel_encoder_pos[{len(encs)}] =", rows))

    rows = [f'  {{{o["pos_mm"][0]:.3f}f, {o["pos_mm"][1]:.3f}f}}, // ch{o["index"]} {o["designator"]} dac{o["dac_index"]}' for o in spec["outputs"]]
    lines.append(arr(f'PanelPoint panel_output_pos[{len(spec["outputs"])}] =', rows))

    rows = [f'  {{{i["pos_mm"][0]:.3f}f, {i["pos_mm"][1]:.3f}f}}, // in{i["index"]} {i["designator"]} adc{i["adc_index"]}' for i in spec["inputs"]]
    lines.append(arr(f'PanelPoint panel_input_pos[{len(spec["inputs"])}] =', rows))

    s = spec["slider"]
    lines.append(f'static const PanelPoint panel_slider_pos = {{{s["pos_mm"][0]:.3f}f, {s["pos_mm"][1]:.3f}f}};')
    lines.append(f'#define PANEL_SLIDER_TRAVEL_MM {s["travel_mm"]}f')
    lines.append("")
    lines.append("#endif /* BMCV_PANEL_LAYOUT_H_ */")
    return "\n".join(lines) + "\n"


# --------------------------------------------------------------------------


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.dirname(here)

    ap = argparse.ArgumentParser()
    ap.add_argument("--hw-repo", default=os.path.join(root, ".."), help="hardware repo with production/")
    ap.add_argument("--dump", default=os.path.join(root, "build-native", "dump_hw_setup"))
    ap.add_argument("--out-dir", default=root)
    args = ap.parse_args()

    netlist_ipc = os.path.join(args.hw_repo, "production", "netlist.ipc")
    kicad_pcb = os.path.join(args.hw_repo, "BMCV.kicad_pcb")
    for p in (netlist_ipc, kicad_pcb):
        if not os.path.exists(p):
            sys.exit(f"missing {p}\n(pass --hw-repo pointing at the BMCV hardware repo)")
    if not os.path.exists(args.dump):
        sys.exit(f"missing {args.dump}\nbuild it first: cmake --build build-native --target dump_hw_setup")

    overrides_path = os.path.join(root, "panel", "overrides.json")
    with open(overrides_path) as fh:
        overrides = json.load(fh)
    overrides["_kicad_pcb"] = os.path.relpath(kicad_pcb, root)
    overrides["_netlist_ipc"] = os.path.relpath(netlist_ipc, root)

    hw_dump = json.loads(subprocess.run([args.dump], capture_output=True, text=True, check=True).stdout)

    pcb_board, positions, extents = read_pcb(kicad_pcb)
    spec = build(pcb_board, positions, extents, Netlist(netlist_ipc), hw_dump, overrides)

    out = args.out_dir
    os.makedirs(os.path.join(out, "panel"), exist_ok=True)
    os.makedirs(os.path.join(out, "web"), exist_ok=True)

    written = []
    for path, data in (
        (os.path.join(out, "panel", "bmcv_panel.json"), json.dumps(spec, indent=2) + "\n"),
        (os.path.join(out, "web", "panel.json"), json.dumps(spec, separators=(",", ":")) + "\n"),
        (os.path.join(out, "panel", "bmcv_panel.svg"), render_svg(spec)),
        (os.path.join(out, "panel", "panel_layout.h"), render_header(spec)),
    ):
        with open(path, "w") as fh:
            fh.write(data)
        written.append(os.path.relpath(path, root))

    print(f"panel spec: {len(spec['buttons'])} buttons, {len(spec['leds'])} leds, "
          f"{len(spec['encoders'])} encoders, {len(spec['outputs'])} outputs, {len(spec['inputs'])} inputs")
    for w in written:
        print(f"  wrote {w}")
    for a in spec["assumptions"]:
        print(f"  ASSUMPTION: {a}")


if __name__ == "__main__":
    main()
