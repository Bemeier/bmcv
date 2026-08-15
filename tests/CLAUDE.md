# Tests

`just test` builds and runs all of them (~10s). `just test-san` repeats under
ASan/UBSan. Both are host-compiled — no ARM toolchain, no hardware.

## Adding one

One file per subject, one executable per file, registered in two places:

1. `bmcv_add_test(test_foo)` in `tests/CMakeLists.txt`
2. `RUN_TEST(the_case_name);` inside that file's `main()`

**`main()` must `return TESTKIT_SUMMARY();`.** Four suites once called it
without returning, so they could not fail the build and everything they covered
was silently unverified. `[[nodiscard]]` prevents a repeat — do not work around
it.

`testkit.h` is the whole framework: `TEST_CASE`, `RUN_TEST`, `CHECK`,
`CHECK_NEAR`. Failures print `file:line [case] expression`.

**Name the case as the property, not the function.**
`an_edit_on_the_press_tick_still_blocks_the_reset`, not `test_ui_channel_3`.
Put the reasoning — and the bug it came from — in a comment above it.

## Which level to test at

**`fixtures/fixture.c` — prefer this.** Builds a whole wired `UxState` and
drives it through `engine_tick` from real button levels and encoder deltas, so
a test states a *gesture* rather than a state:

```c
Fixture f; fixture_init(&f);
fixture_hold(&f, ctrl_btn(&f, SHIFT_STATE_MIX), UI_T_HOLD + MS(50));
fixture_release(&f, ctrl_btn(&f, SHIFT_STATE_MIX));
fixture_press(&f, ctrl_btn(&f, CH_PARAM_AMP), MS(40));
```

It reproduces the module's real timing, including the UX rate limit, so
off-by-one-tick bugs show up here and nowhere else.

**Direct unit tests** (`test_ui_input.c`, `test_helpers.c`) where exact
threshold behaviour is the subject and going through the fixture would couple
the test to the dispatch rate.

**`sim/flows/`** for interactions across time that no unit test can state — a
clock locking, a mute ramping, a mode latching. Scripted input timelines
replayed through the CLI and diffed against committed CSV.
See `sim/CLAUDE.md`.

**`tests/pll_metrics.{c,h}`** turns a run of the sync loop into numbers —
settling, ringing, frequency pull, phase continuity — so a change to the loop
reads as a trade rather than a feeling. `test_pll.c` prints the table;
`docs/pll.md` holds the baseline to diff against. Bounds there are deliberately
loose: the printed table is the artifact, not the assertion.

## Tests that exist to stop the builds drifting

Do not delete or weaken these when they fail — they are load-bearing.

| Test | Holds together |
|---|---|
| `test_panel_spec.c` | generated panel geometry vs `hw_setup.c` |
| `test_sim_import.c` | a snapshot round-trips through the wasm decoder |
| `test_usb_descriptors.c` | the descriptor set a browser needs to bind WinUSB |
| `test_config_migrate.c` | every old FRAM record still loads |
| LED curve check in `web/smoke.mjs` | the C header vs the JS constants |
| `sim/include/layout_target.h` | ARM and host struct layouts, field by field |

## Tools built here, not tests

- `render_channel` (`just render`) — renders a channel to a WAV for listening.
- `dump_hw_setup` — emits the firmware's `HwSetup` tables for `just panel`.
