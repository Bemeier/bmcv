# BMCV

Macro CV Controller for eurorack inspired by the [Emblematic Systems Catalyst](http://www.emblematic-systems.net/).

![Render][render]

## PCB

PCB made with [KiCad 8.0](https://www.kicad.org/).

## Firmware

Eight scene-crossfaded LFOs, each with frequency, shape, modulation, phase,
amplitude and offset, and a PLL that locks all eight to an incoming clock. A
crossfader blends between two saved scenes, so one slider morphs the whole
patch rather than just one parameter. Firmware source, the browser simulator,
and the VCV Rack module all live in [`BMCVFirmware/`](BMCVFirmware/) and run
the same C - see [its README](BMCVFirmware/README.md) for how the module
works and [`docs/architecture.md`](BMCVFirmware/docs/architecture.md) for how
that's arranged.

No hardware needed to try it:

- **[Simulator](https://bemeier.github.io/bmcv/)** - the full panel in the
  browser, wired to the real firmware core compiled to WebAssembly.
- **[Firmware updater](https://bemeier.github.io/bmcv/update/)** - flashes a
  module over USB-C, straight from the page. Pick a released version or a
  local `.bin`.

Both are built and deployed from `main` on every release.

## License

This work is licensed under a [Creative Commons Attribution 4.0 International License][cc]. 

[![CC BY SA 4.0][shield]][cc]

[cc]: https://creativecommons.org/licenses/by-sa/4.0
[shield]: https://licensebuttons.net/l/by-sa/4.0/88x31.png
[render]: https://raw.githubusercontent.com/Bemeier/bmcv/refs/heads/main/render.png