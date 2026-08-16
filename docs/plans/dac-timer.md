# Giving the DAC a clock of its own

Status: **works, and is not yet a net win.** Branch `feat/dac-timer`, validated
on the module with CH1 patched into IN3. The timer does exactly what it was
supposed to; what it uncovered is that the scheduling was never the expensive
part.

## What the module said

All on the `-rel` build, eight channels patched, stepped random on seven.

| | before | after (1 substep) |
|---|---|---|
| frames/s per output | 842 | **4032** |
| `dac_fps` vs `engine_fps` | locked equal | **decoupled** |
| `engine_fps` | 3366 | 2455 |
| `load` | 1.19 | 1.65 |
| resyncs | 16% of ticks | 30% |
| loopback, slow sine | R^2 0.9996 | R^2 0.9996 |

**The rate fix works.** `dac_fps` and `engine_fps` used to read the *same*
number - one transaction per tick, so one frame per output every four ticks -
and they no longer do. The output rate is now a number this firmware chooses.

**4 substeps is unaffordable, and not by a little.** At a 15.0us cadence one
`dac_service()` measures **8.18us** - `bmcv_profile.dac`, read off the module -
so 55% of the CPU goes into that interrupt before the DMA completion it feeds
is counted. With both at priority 0 thread mode gets nothing: the module came
up with `dac_fps 54335`, `engine_fps 0`, and `just profile` reporting **0 ticks
since boot**. The engine had never run once. `just where` found the core in
`SPI_DMATransmitReceiveCplt`, which is where it stayed.

**The cost is HAL, not the interpolation.** At 1 substep the tick inflated from
297us to 423us. Backing that out: ~6.8 services per tick at 8.18us is 56us,
leaving ~70us across ~6.8 completions - so **the completion handler costs ~10us,
more than the arm does**. Together the HAL SPI path is ~18.5us per transaction
against **2.67us of actual wire time**. Eight float lerps is at most 1-2us of
it. The bus was never the constraint and neither was the scheduling; it is
`HAL_SPI_TransmitReceive_DMA` and `SPI_DMATransmitReceiveCplt`.

**Raising the SPI clock is not a lever.** SPI2 is already at prescaler 8 = 18MHz
against a 20MHz ceiling, so `/4` is out of spec.

**What the loopback can and cannot see.** With a slow sine on CH1 both builds
read R^2 0.9996, gain 1.000 - which proves the output is alive, linear and full
amplitude, and is exactly the check that would have caught the dead-output
regression. It cannot resolve the rate difference, because a slow sine barely
moves within a tick. With near-noise on CH1 it can, and it read old 0.851
against new 0.804 - the new build slightly *worse* end to end, because output
latency is one engine tick and the ticks got longer. That is the whole problem
in one number.

## Where that leaves it

The change is structurally right and currently pays for itself with engine
budget it does not have. Three ways on, in the order they are worth doing:

1. **Cut HAL out of the interrupt path.** Arm the DMA and handle completion by
   writing the registers in `dac_adc.c`. 18.5us against 2.67us of wire time is
   most of a transaction spent walking a state machine. Plausibly 2-3us, which
   would make 4 substeps affordable *and* hand the engine back what it lost.
   Driver-level, ARM-only, behind the seam.
2. **`ENGINE_TICK_US`**, below. The plan's own next item, and the cheap one.
3. **Ship at 1 substep as it stands.** Argued against: a 28% slower engine buys
   a faster DAC whose end-to-end latency is one engine tick, so it gives back
   much of what it gained.

## What is done

- **The hardware-in-the-loop rig** - `just hil` and `just dac-loopback`, below.
  Built first, because the failure this change has already produced once was
  invisible to every number the firmware reports about itself.
- **The timer** - TIM6, hand-written in `main.c`'s USER CODE blocks, at
  `bmcv_dac_service_hz()`. `bmcv_dac_tick()` in `bmcv.c` is what it calls; the
  main loop no longer touches the DAC at all.
- **`bmcv_profile.dac`** - the service measured from inside its own interrupt,
  and printed by `just profile`. It is what turned "the engine is dead" into "the
  interrupt costs 55% of the CPU" in one reading.

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

## The rig

Neither candidate above can be told apart from the other, or from success, by
anything the module says about itself - `dac_fps` reported the dead output as
the best rate it had ever managed. So the loop is closed outside the firmware:

**`just hil EXPR...`** samples any scalar inside `bmcv` off a running module
over SWD, as TSV. The whole 2.7 KB instance comes back in one burst and the ELF
says where each field sits in it, so nothing in the script knows the struct. A
full-instance read and a 16-byte read both measure 0.13 s - the cost is the
STM32_Programmer_CLI connect, not the transfer - so reading everything is free,
and every field in a sample was read microseconds from every other.

**`just dac-loopback`** patches that into a verdict. CH1's output goes into IN3
with a patch cable; the script correlates `channels_gated_level[0]` against
`input.curr.input_state[2]` and reports `dead`, `loose` or `ok`.

It **correlates rather than traces**. Eight samples a second cannot follow an
LFO - but both fields come out of one instance read, so each (commanded,
measured) pair is internally consistent however badly the sampling aliases.
Scatter enough pairs across the waveform and they lie on a line whether or not
they were taken in order, which means the channel can be left running whatever
it was already running and no part of the rig has to control the module.

Checked against an unpatched module first: `commanded span 22.8% of range,
measured span 0.0%, verdict DEAD` - the same signature as the regression, which
is the evidence that the detector detects.

## The plan

**A periodic timer interrupt calling `dac_service()` directly.** It does not
re-enter HAL's own DMA callback, which is what makes it different from the
attempt above, and a fixed cadence is the thing that was wanted in the first
place rather than a side effect of loop scheduling.

1. **Add a timer.** TIM2 is the microsecond counter and TIM4 is the 303 Hz task
   poll - but **TIM6 is free**, a basic timer with no channels, no pins and a
   vector of its own. Not a CubeMX change: the `.ioc` already does not
   round-trip (it still describes CubeMX's TIM2 - prescaler 14400, period 32 -
   which is the timer `main.c` calls TIM4, while `main.c`'s own TIM2 has no
   `.ioc` counterpart), so TIM6 is hand-written in the USER CODE blocks
   alongside everything else that is already maintained by hand there.
2. **Rate: the current target**, 15.625 us (`DAC_CHUNK_US`), exposed as
   `bmcv_dac_service_hz()` so the timer setup reads one number rather than
   reproducing the formula. Prescaler 0 and the whole divide in the reload:
   2250 counts at 144 MHz is exact, where a 1 us prescaler would quantise a
   15.625 us period to 15 or 16 and put 2-6% of jitter on the cadence this
   change exists to make even.
3. **Skip when one is in flight.** `dac_poll` says whether a completion has been
   seen. A skipped chunk is not a stall - `dac_write_interpolated()` reads the
   clock rather than counting substeps, so the next one lands where the elapsed
   time says it should.
4. **Priority 0, the SPI DMA completion's.** Equal priority means neither can
   preempt the other, so `dac_service()` cannot be re-entered against
   `dacadc_dma_complete()` and the two need no lock. TIM4 sits at 1 and is
   preempted by both.
5. **Measure what the interrupt costs.** At 64 kHz an interrupt doing an
   interpolation and a DMA arm is not free. `bmcv_profile.dac` measures the
   service from inside itself, so `just profile` reports it directly; `load` and
   `engine_fps` say what it took from the engine. If it has taken too much,
   fewer substeps is the honest answer rather than more cleverness.

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
   version would settle in a minute what the reasoning above cannot. Worth
   noting that a fixed cadence answers it either way without proving it: armed
   on a clock, the gap between `dacadc_dma_complete()` raising SYNC and
   `dacadc_dma_next()` lowering it is the chunk period less the transfer, about
   13 us of the 15.6, on every chunk. The same holds for the ADC's conversion
   window, which is one whole chunk period and now the same one every time.
2. **Does the DAC need the interpolation at all** once its frame rate is above
   the engine's? The ramp exists because a step is a broadband click; at 16 kHz
   frames against a 2.4 kHz engine it would be doing what it was written for
   for the first time.
3. **Is 20 MHz worth the last prescaler step?** It would take 2.67 us of bus
   time to 2.4 us, which is nothing next to the latency - so no, unless the
   latency goes first.
