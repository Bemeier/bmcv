"""Draws what each shape mode does across SHP and MOD, into docs/images/.

One SVG per mode: a grid with SHP along the rows and MOD along the columns, so
what each knob does is the direction you read in. The samples come from
tools/dump_shapes.c, which calls the module's own shape functions - a picture of
a shape the firmware does not have is worse than no picture, which is the same
reason gen_wavetable.py plots the table it just generated.

Run with `just shape-figures`. Output is checked in; review the diff.
"""
import subprocess
import sys

OUT = "docs/images"
MODES = {
    "lfo": ("wavetable", "SHP slices the table, MOD leans where the cycle spends its time"),
    "stepped": ("stepped", "SHP is the distribution, MOD the motion"),
    "pwm": ("PWM", "SHP is the pulse width, MOD the ramps"),
}

# Matches gen_wavetable.py, so the two sets of plots read as one family.
STYLE = """  <style>
    .frame { fill: none; stroke: #d8d8d8; stroke-width: 1; }
    .axis  { stroke: #ececec; stroke-width: 1; }
    .trace { fill: none; stroke: #222; stroke-width: 1.6; stroke-linejoin: round; }
    .name  { font: 600 13px/1.2 system-ui, sans-serif; fill: #222; }
    .sub   { font: 11px/1.2 system-ui, sans-serif; fill: #777; }
    .axlab { font: 600 11px/1.2 system-ui, sans-serif; fill: #555; }
    @media (prefers-color-scheme: dark) {
      .frame { stroke: #3a3a3a; }
      .axis  { stroke: #2c2c2c; }
      .trace { stroke: #e8e8e8; }
      .name  { fill: #e8e8e8; }
      .sub   { fill: #999; }
      .axlab { fill: #aaa; }
    }
  </style>
"""


def load(text):
    grid = {}
    for line in text.splitlines():
        head, vals = line.split(":")
        mode, shp, mod = head.split()
        grid[(mode, float(shp), float(mod))] = [float(v) for v in vals.split()]
    return grid


def trace(samples, x, y, w, h, points=128):
    pts = []
    for k in range(points + 1):
        v = samples[round(k * len(samples) / points) % len(samples)]
        pts.append(f"{x + w * k / points:.1f},{y + h / 2.0 - v * (h / 2.0):.1f}")
    return " ".join(pts)


def figure(grid, mode, knobs):
    title, blurb = MODES[mode]
    pw, ph, gap = 150, 78, 14
    left, top = 74, 62
    width = left + len(knobs) * (pw + gap) + 8
    height = top + len(knobs) * (ph + gap) + 18

    out = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" '
           f'width="{width}" height="{height}">', STYLE]
    out.append(f'  <text class="name" x="{left}" y="22">{title}</text>')
    out.append(f'  <text class="sub" x="{left}" y="38">{blurb}</text>')

    for col, mod in enumerate(knobs):
        x = left + col * (pw + gap)
        out.append(f'  <text class="axlab" x="{x + pw / 2:.0f}" y="{top - 8}" '
                   f'text-anchor="middle">MOD {mod:+.1f}</text>')

    for row, shp in enumerate(knobs):
        y = top + row * (ph + gap)
        out.append(f'  <text class="axlab" x="{left - 10}" y="{y + ph / 2 + 4:.0f}" '
                   f'text-anchor="end">SHP {shp:+.1f}</text>')
        for col, mod in enumerate(knobs):
            x = left + col * (pw + gap)
            out.append(f'  <rect class="frame" x="{x}" y="{y}" width="{pw}" height="{ph}" rx="3"/>')
            out.append(f'  <line class="axis" x1="{x}" y1="{y + ph / 2}" x2="{x + pw}" y2="{y + ph / 2}"/>')
            out.append(f'  <polyline class="trace" points="{trace(grid[(mode, shp, mod)], x, y, pw, ph)}"/>')

    out.append("</svg>")
    return "\n".join(out) + "\n"


def main():
    grid = load(sys.stdin.read())
    knobs = sorted({k[1] for k in grid})
    for mode in MODES:
        path = f"{OUT}/shape-{mode}.svg"
        with open(path, "w") as f:
            f.write(figure(grid, mode, knobs))
        print(f"wrote {path}")


if __name__ == "__main__":
    main()
