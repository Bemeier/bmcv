#ifndef INC_LIB_HW_SETUP_H_
#define INC_LIB_HW_SETUP_H_

#include <stdint.h>

#define N_INPUTS 4
#define N_ENCODERS 8
#define N_CHANNELS 8
#define N_BUTTONS 24
#define N_SCENES 7
#define N_CTRL_BUTTONS 9
#define N_SEMITONES 12
#define LED_COUNT 21

// Every channel is driven by exactly one encoder, so the two counts are the
// same number and a handful of tables are indexed by either. Channel-indexed
// arrays should say N_CHANNELS; this keeps the identity from silently breaking
// if a future board ever has a spare encoder.
// static_assert rather than _Static_assert because this header is also read by
// C++ (the VCV Rack plugin), where the underscored spelling does not exist.
// C23 has both.
static_assert(N_ENCODERS == N_CHANNELS, "one encoder per channel");

// A channel's sample & hold can be triggered by an input jack or by another
// channel's output, so src_trig indexes a composite space: the N_INPUTS jacks
// first, then the N_CHANNELS channel outputs. HwState.trigger_src[] has the
// same layout. Use these rather than open-coding the offset.
#define N_TRIG_SRC (N_INPUTS + N_CHANNELS)
#define TRIG_SRC_INPUT(i) (i)
#define TRIG_SRC_CHANNEL(c) (N_INPUTS + (c))

// CV range of the board, in raw converter units. These describe the analog
// front/back end, not the SPI driver, so they live here rather than in
// dac_adc.h - the logic and presentation layers need them without pulling in
// a driver header.
#define ADC_10V 8192
#define ADC_5V 4096

#define DAC_10V 32768
#define DAC_5V 16384

// Crossfader travel in raw ADC units. Calibration of a physical part, so it
// belongs with the converter ranges rather than in state.h: a host that
// synthesises a slider position needs it without knowing anything else.
#define SLIDER_MIN_VALUE 400
#define SLIDER_MAX_VALUE 7661

// Gate/trigger detection, with hysteresis: an edge is recognised at ~1.25V and
// the latch only re-arms below ~0.98V. Eurorack gates swing to 5V or more, so
// this trips well clear of noise while staying well under any valid gate's low
// level.
//
// Here for the same reason as the ranges above: the ADC driver applies them on
// hardware, but a simulator has to reproduce the same edges from its own CV
// samples, and it must not pull in a driver header to do it.
//
// The same two voltages appear in both converter domains, because the two
// converters differ in density - one voltage is a different count in each. An
// input jack is measured in ADC units; a channel output used as a trigger
// source is measured in DAC units. Derived rather than typed twice, so the
// pair cannot drift apart.
#define TRIG_THRESH 1024
#define TRIG_THRESH_LOW 800
#define TRIG_THRESH_DAC (TRIG_THRESH * (DAC_10V / ADC_10V))
#define TRIG_THRESH_LOW_DAC (TRIG_THRESH_LOW * (DAC_10V / ADC_10V))

// The engine's control period. The engine is dt-driven and correct at any rate,
// so this is not a correctness constant - it is the interval the module spaces
// its DAC updates at, and the number the other three hosts already assume
// (web/const.js TICK_US, BMCV_CONTROL_HZ in the Rack plugin). Here so that when
// the board turns out to hold a different rate, one place changes.
// 500us is 2kHz, down from 250 (4kHz) by way of 312 (3.2kHz), and the number
// was decided by the worst case rather than the typical one.
//
// The typical case fits anything: with the stepped path memoised, eight busy
// channels cost 202us. But **a moving crossfader empties both caches every
// tick** - every scene blend is a new shape/mod, so the pattern genuinely has
// to be recomputed - and that costs 359us however fast the tick is. Measured on
// the module at 312us, a fader sweep took the engine to load 1.23 and
// engine_fps to 2440, which is not a 3.2kHz engine.
//
// At 500us the same cold case is load 0.82 and engine_fps holds 2000.15, with
// 18% left for the USB link. Warm it is 0.55. So this is the rate the module
// actually delivers, rather than the rate it delivers when nobody touches it.
//
// It costs nothing on the output: DAC_CHUNK_US is ENGINE_TICK_US over
// DAC_SUBSTEPS * DAC_CHANNELS, so two substeps at 500us is a 62.5us chunk and
// 4032 frames/s - the same rate as the 4kHz engine managed, and now genuinely
// oversampled twice per tick rather than exactly once.
//
// What it does cost is timing resolution on gate and trigger edges, 500us
// against 250. Note that this is the *output* side: gates arriving on the CV
// inputs are latched in the DAC's own interrupt at 8kHz and consumed once per
// tick, so nothing is missed - see adc_read_trig_state().
#define ENGINE_TICK_US 500 // 2 kHz

typedef struct
{
  // ADC/DAC
  uint8_t input_adc_idx[N_INPUTS];
  int8_t channel_dac_idx[N_ENCODERS];

  // Buttons & Encoders
  int8_t channel_encoder_idx[N_ENCODERS];
  int8_t channel_button_idx[N_ENCODERS];
  uint8_t quantizer_button_idx[N_SEMITONES];
  int8_t ctrl_button_idx[N_CTRL_BUTTONS];
  int8_t scene_button_idx[N_SCENES];

  // LEDs
  int8_t channel_led_idx[N_ENCODERS];
  int8_t scene_button_led_idx[N_SCENES];
  uint8_t quantizer_button_led_idx[N_SEMITONES];
  int8_t ctrl_button_led_idx[N_CTRL_BUTTONS];
  uint8_t ctrl_button_color[N_CTRL_BUTTONS];
} HwSetup;

const HwSetup* HwSetup_Get(void);

#endif /* INC_LIB_HW_SETUP_H_ */
