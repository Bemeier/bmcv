# Giving the DAC a clock of its own

Status: **not started.** Everything below is measured, on the `-rel` build,
against a module with eight channels patched.

## The problem

The output rate is a property of how busy the engine is.

| stepped channels | transactions/s | frames/s | engine |
|---|---|---|---|
| 0 | 18 kHz | 4.5 kHz | 4 kHz |
| 1 | 14 kHz | 3.5 kHz | |
| 2 | 10 kHz | 2.5 kHz | |
| 3 | 6 kHz | 1.5 kHz | |
| 4+ | ~engine rate | <1 kHz | |

A frame is all eight outputs. `DAC_CHANNELS` is 4 and each transaction carries
two outputs, so **frames are transactions ÷ 4** - and `dac_fps` counts
transactions, which is worth remembering before reading it as an output rate.

So at four stepped channels each output is updated under a thousand times a
second, against an engine computing four thousand values a second for it. The
interpolation in `dac_write_interpolated()` is not oversampling anything; it is
smoothing over an output that cannot keep up with the engine. `DAC_SUBSTEPS 4`
asks for four frames per tick - 16 kHz - and has never been getting it.

## Where the time actually goes

- **The wire is idle 95% of the time.** SYSCLK is 144 MHz (HSE 38.4, PLLM/4,
  PLLN 30, PLLR/2), APB2 is undivided, and SPI1's prescaler is 8 - so 18 MHz,
  and a 6-byte transaction is **2.67 us** of bus time against a 55 us cadence.
  The datasheet's ceiling is 20 MHz, so there is one prescaler step of headroom
  and no more: `/4` would be 36 MHz.
- **The rest is latency, not work.** A completed transfer sets a flag and waits
  for the main loop, and the loop is inside an engine tick. `just profile` on
  the module: the tick is 160 us with no stepped channels and **307 us with
  eight, against a 250 us period** - a load of 1.23, so there is nothing left
  between ticks at all, and `resyncs` climbs.
- **The engine's own baseline is 135.6 us** before any stepped channel exists.
  Each stepped channel adds 18.3 us. That baseline is worth its own look.

## What was tried

**Re-arming from the SPI completion interrupt.** `bmcv_handle_txrx_complete()`
called the service directly when one was due instead of setting a flag. On the
module: a stable transaction rate and a **dead output** - transfers running,
nothing latching. Reverted.

It is legal on paper. `SPI_DMATransmitReceiveCplt()` sets
`hspi->State = HAL_SPI_STATE_READY` before invoking the callback, and
`HAL_DMA_IRQHandler()` sets the channel to READY and unlocks before invoking
its own. So the state machines allow the re-arm and something else objects.
Two candidates, neither tested:

- **The SYNC pulse.** `dacadc_dma_complete()` raises CS and
  `dacadc_dma_next()` lowers it again; served from the loop that gap was tens
  of microseconds, and from the callback it is one or two. If the DAC needs
  longer to latch what was just shifted in, the output freezes exactly as
  observed.
- **The ADC conversion window.** The same transaction starts a conversion
  (CNVST, ADDR toggle) and reads the previous one. Closing that window sooner
  may be reading before the conversion is done - which would corrupt the ADC
  rather than the DAC, so this explains the symptom less well.

## The plan

**A periodic timer interrupt calling `dac_service()` directly.** It does not
re-enter HAL's own DMA callback, which is what makes it different from the
attempt above, and a fixed cadence is the thing that was wanted in the first
place rather than a side effect of loop scheduling.

1. **Add a timer.** TIM2 is the microsecond counter and TIM4 is the 303 Hz task
   poll, so there is nothing spare. A CubeMX change, and `Core/Src/main.c` is
   generated territory - see how TIM4 is wired through
   `HAL_TIM_PeriodElapsedCallback()` and do the same.
2. **Rate: start at the current target**, 15.6 us (`DAC_CHUNK_US`), and treat it
   as adjustable. `dac_service()` already takes `now_us` and does its
   bookkeeping before arming the transfer, so it is safe to call from an
   interrupt as it stands.
3. **Skip when one is in flight.** The service must not arm a transfer while the
   previous is running - `dac_poll` says whether a completion has been seen.
4. **Measure what the interrupt costs.** At 64 kHz an interrupt doing an
   interpolation and a DMA arm is not free. `just profile` plus `engine_fps` is
   how to see whether it has taken the engine's budget; if it has, fewer
   substeps is the honest answer rather than more cleverness.

Then, and only then, the two that are independent of it:

- **`ENGINE_TICK_US`.** A tick of 307 us in a 250 us period is a deadline that
  cannot be met; 2.4 kHz (417 us) puts the load at 0.75. The engine is
  dt-driven and documented as having to stay correct at any rate. What it costs
  is timing resolution on gate and trigger edges - 417 us against 250 - and a
  re-take of the baseline in `docs/pll.md`.
- **The 135.6 us baseline.** Eight wavetable channels and a PLL should not need
  half the budget at `-O2`.

## Unresolved questions

1. **Is it the SYNC pulse?** A scope on the DAC's sync line during the failing
   version would settle in a minute what the reasoning above cannot.
2. **Does the DAC need the interpolation at all** once its frame rate is above
   the engine's? The ramp exists because a step is a broadband click; at 16 kHz
   frames against a 2.4 kHz engine it would be doing what it was written for
   for the first time.
3. **Is 20 MHz worth the last prescaler step?** It would take 2.67 us of bus
   time to 2.4 us, which is nothing next to the latency - so no, unless the
   latency goes first.
