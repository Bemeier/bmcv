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
#define ENGINE_TICK_US 250 // 4 kHz

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
