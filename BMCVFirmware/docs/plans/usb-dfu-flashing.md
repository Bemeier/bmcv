# Plan: USB firmware update via ROM DFU

Status: **done and working on hardware.** Kept only for the two loose ends at
the bottom; the design and the reasoning live in CHANGELOG.md and the README's
"Updating the firmware over USB" section.

## What shipped

- `sysex.c` + `test_sysex.c` - SysEx control channel on the existing USB MIDI
  endpoint. Reboot into update mode, report version.
- `fw_update.c` - marker in `.noinit`, reset, hand over to the ROM bootloader
  from the first line of `main()`.
- `web/update/` - the updater page and its DfuSe client, plus `just dfu-check`
  in `just check`.
- `scripts/where.sh` / `just where` - halts the module over SWD and resolves PC
  and LR to source lines. Built during this work and it earned its place.

Costs about 1.2 KB of flash.

## Bench log

1. **FN2 into ROM DFU** - works, enumerates as `0483:df11`.
2. **Flashing from the page** - works. Two fixes on the way: the bootloader entry
   pointer was a stack local read back after `__set_MSP` had already moved the
   stack, and the page accepted an `.elf`. Flash reads back byte-identical.
3. **The software route into DFU** - works, after being reworked from a direct
   jump to marker-and-reset. The bootloader will not initialise USB if it
   inherits a running chip, however carefully it is torn down first.
4. **Restart after flashing** - **does not work, and will not.** The bootloader
   reaches its manifest phase, drops its own USB pull-up, and stops where ST's
   reference calls `NVIC_SystemReset()`. Nothing the host can send reaches a
   device that has already left the bus. Power-cycling the case is the answer
   and the page says so.

## Loose ends

- **Windows and the Zadig step are still completely untested.** It is the first
  thing a new user hits, and the instructions on the page have never been
  followed by anyone.
- **Hosting.** The page takes an image through a file picker. GitHub Pages plus
  a CI-built manifest would let people flash a release rather than a local
  build; nothing in the page assumes local-only, so it is additive.
