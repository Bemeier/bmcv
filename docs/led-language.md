# LED language

The colours are not decoration. One fact per LED, and the same colour for the
same idea on every page - the point being that the panel teaches itself, so the
manual never has to explain a colour.

![The panel's LEDs](images/led-language.png)

- **Ctrl-page settings clamp, they do not wrap.** Each is a short list of
  unrelated states with its default at index 0, so spinning an encoder fully
  left resets it and you can do that without looking. Rolling off one end into
  the other would be the largest change on the page.
- **Setting states** are the base layer. Index 0 of every setting - disabled,
  default, neutral - is **purple**; **cyan** is continuous / level-following or
  multiplicative, **green** additive or half-way, **yellow** triggered / clocked
  / stepped, **red** reset. Destructive is **pink**, and belongs to clearing
  alone. See `HUE_STATE_*` in `color_presets.h`. The output clamp is the one
  place brightness also carries meaning, because it is two facts on one LED.
- **White is assignment, and nothing else uses it.** A short white flash every
  1.6s over an element's own colour means "you can pick this". Once something is
  held, the places it can go are steady white with a short dropout, the held
  source is steady saturated purple, and everything else goes **dark** - if it
  cannot be pressed it does not light.
- **Output level** only appears when no shift mode is active. In a shift mode
  the encoder ring shows that mode's setting, or nothing if it has none.
- **FRQ is a ratio, not a level**, so its ring codes three facts on three axes
  rather than drawing a bar. **Hue** is the kind of division: **green** straight
  (halves, quarters, octaves), **yellow** triplet, **orange** quintuplet -
  octaves are free, so 1/8 and 16 are the same green and 1/3, 3/2 and 24 are the
  same yellow. **Saturation** is how far off the grid the value sits, so a fine
  adjust washes the colour out to a pastel and a snapped ratio is pure.
  **Brightness** pulses shallowly at the channel's own output rate, which is
  what says fast from slow; anything too fast for the panel to resolve shimmers
  together at one rate instead of aliasing into a slow phantom pulse.
- **Confirmations** are a brief flash on top: purple wrote, pink cleared, cyan
  loaded. Purple is the selection colour, so a copy landing somewhere flashes
  the colour the source was held in. Red is errors only.
- **A held press dips out once as it crosses each of its thresholds** - off,
  then back on - which says "that registered" rather than pulsing away as if
  something were still in progress. At the first stage it comes back exactly as
  the page had it; where holding longer does something wider - clearing every
  scene rather than this one - it comes back brighter, in the same colour.
  Nothing commits until the release.

