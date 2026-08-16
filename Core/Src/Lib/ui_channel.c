#include "ui_channel.h"
#include "channel.h"
#include "color_presets.h"
#include "config.h"
#include "helpers.h"
#include "hw_setup.h"
#include "stepped_table.h"
#include "ui_feedback.h"
#include "ui_input.h"
#include "ui_mode.h"
#include "ui_select.h"
#include "ui_state.h"
#include "ux_state.h"
#include <stdint.h>

// The frequency parameter steps through ratios of the beat rather than sweeping
// continuously, so the encoder snaps to this grid.
//
// The set is a 5-limit lattice: every entry is 2^k, 3*2^k or 5*2^k over the
// same, which is the same thing as saying every entry is a straight division, a
// triplet or a quintuplet. That structure is what the colour reads off, so the
// ratio is written here as a fraction and the stored value *and* the hue are
// both derived from it - one source of truth. Adding a divisor without giving
// it a colour is not expressible.
//
// Not reciprocal-symmetric, deliberately: 1/128 is 32 bars and is a real
// setting for a slow LFO, while 128x is 256Hz at 120BPM - past anything the
// panel can show and not a division anyone reaches for.
// Hand-laid out, slowest to fastest and eight to a line, because the shape of
// the set is the documentation. Exempt from the formatter, which reflows it
// into a different paragraph on every run and never reaches a fixed point - so
// `just fmt-check` could never pass on it.
// clang-format off
#define FREQ_RATIOS(X)                                                     \
  X(1, 128) X(1, 64) X(1, 32) X(1, 24) X(1, 16) X(1, 12) X(1, 10) X(1, 8)  \
  X(1, 6)   X(1, 5)  X(1, 4)  X(1, 3)  X(2, 5)  X(1, 2)  X(2, 3)  X(3, 4)  \
  X(4, 5)   X(1, 1)  X(5, 4)  X(4, 3)  X(3, 2)  X(2, 1)  X(5, 2)  X(3, 1)  \
  X(4, 1)   X(5, 1)  X(6, 1)  X(8, 1)  X(10, 1) X(12, 1) X(16, 1) X(24, 1) \
  X(32, 1)  X(64, 1)
// clang-format on

// The stored parameter is 1/f-linear either side of 1x - see the multiplier
// reconstruction in channel_compute - so the two directions are not the same
// expression. Rounded rather than truncated, which is why 2/3 and 3/2 are at
// -/+128 where the old hand-typed table had -/+127.
#define FREQ_VAL(p, q) ((int16_t) ((p) >= (q) ? (255 * ((p) - (q)) + (q) / 2) / (q) : -(255 * ((q) - (p)) + (p) / 2) / (p)))

#define FREQ_COUNT_ONE(p, q) +1
#define N_FREQ_MULTIPLIERS (0 FREQ_RATIOS(FREQ_COUNT_ONE))

#define FREQ_EMIT_VAL(p, q) FREQ_VAL(p, q),
static const int16_t quantized_multipliers[N_FREQ_MULTIPLIERS] = {FREQ_RATIOS(FREQ_EMIT_VAL)};

#define FREQ_EMIT_RATIO(p, q) {(p), (q)},
static const struct
{
  uint8_t p, q;
} freq_ratios[N_FREQ_MULTIPLIERS] = {FREQ_RATIOS(FREQ_EMIT_RATIO)};

// A division's prime limit: the largest odd prime left once the octaves are
// divided out. It is the whole of what makes one division feel different from
// another - 3/2 is a dotted eighth and aligns on a 3-beat period exactly as 3,
// 6 and 1/3 do - so it is the only thing the hue codes.
//
// Green -> yellow -> orange is monotone on the hue wheel and reads as familiar
// -> exotic. Red is deliberately not on it: it means an error and nothing else.
//
// Three classes, and the FRQ grid and the pattern lengths are both built to
// stay inside them - neither offers a division with a 7 in it, so nothing here
// falls through to straight that should not.
uint8_t ui_division_hue(uint32_t n)
{
  if (n == 0u)
    return HUE_FREQ_STRAIGHT; // no division at all; every limit below divides it

  while ((n & 1u) == 0u)
  {
    n >>= 1;
  }
  if (n % 5u == 0u)
    return HUE_FREQ_QUINTUPLET;
  if (n % 3u == 0u)
    return HUE_FREQ_TRIPLET;
  return HUE_FREQ_STRAIGHT;
}

// The two sides of a ratio multiplied: p and q are coprime, so this keeps both
// odd parts and the limit of the pair is the limit of the product.
static uint8_t ratio_hue(uint16_t p, uint16_t q) { return ui_division_hue((uint32_t) p * q); }

// The grid entry nearest a stored value.
static size_t freq_nearest(int16_t value)
{
  // The grid is sorted, so the first entry not below the value is either the
  // match or one past the nearest one below it.
  size_t idx = 0;
  while (idx + 1 < N_FREQ_MULTIPLIERS && quantized_multipliers[idx] < value)
  {
    idx++;
  }
  if (idx > 0 && (value - quantized_multipliers[idx - 1]) < (quantized_multipliers[idx] - value))
  {
    idx--;
  }
  return idx;
}

// The gap between the grid entry at idx and its neighbour on the side `value`
// falls, which is what "how far off the grid" has to be measured against: the
// grid is wildly non-uniform - 64 units between 1 and 5/4, 16320 between 1/64
// and 1/128 - so an absolute distance means nothing on its own.
static int32_t freq_gap(size_t idx, int16_t value)
{
  if (value < quantized_multipliers[idx] && idx > 0)
    return quantized_multipliers[idx] - quantized_multipliers[idx - 1];
  if (value > quantized_multipliers[idx] && idx + 1 < N_FREQ_MULTIPLIERS)
    return quantized_multipliers[idx + 1] - quantized_multipliers[idx];
  return 0;
}

UiColor ui_channel_freq_color(int16_t value)
{
  size_t idx = freq_nearest(value);
  UiColor c  = {ratio_hue(freq_ratios[idx].p, freq_ratios[idx].q), SAT_MAX, VAL_BASE};

  // Off the grid, the hue still says which ratio you are near and the colour
  // washes out to say you are not on it. Floored at SAT_MED rather than run to
  // white: white is the assignment vocabulary and nothing else may use it.
  int32_t gap = freq_gap(idx, value);
  if (gap > 0)
  {
    // Halfway to the neighbour is the furthest off-grid a value can be, so that
    // is where the wash bottoms out. The deadzone keeps a snapped value pure
    // through any rounding.
    int32_t d_pct  = (int32_t) (abs(value - quantized_multipliers[idx]) * 200) / gap; // 0..100
    int32_t washed = iclamp((d_pct - FREQ_OFFGRID_DEADZONE) * 100 / (100 - FREQ_OFFGRID_DEADZONE), 0, 100);
    c.s            = (uint8_t) (SAT_MAX - (SAT_MAX - SAT_MED) * washed / 100);
  }
  return c;
}

// One detent of fine adjust on the frequency grid.
//
// Clamped to the ends of the grid, which is not cosmetic: the step used to be
// unbounded, so twelve detents from the top entry overflowed int16_t and wrapped
// to -32768 - the fastest setting on the panel snapping straight to slower than
// the slowest.
static int16_t freq_fine_adjust(int16_t value, int16_t delta)
{
  int16_t lo = quantized_multipliers[0];
  int16_t hi = quantized_multipliers[N_FREQ_MULTIPLIERS - 1];

  // The interval being moved through is the one ahead of the value in the
  // direction of travel, so a value sitting exactly on a grid entry steps by
  // the gap it is about to cross rather than the one behind it.
  size_t idx  = freq_nearest(value);
  int32_t gap = freq_gap(idx, value);
  if (gap == 0)
  {
    size_t ahead = (delta > 0) ? idx : (idx > 0 ? idx - 1 : 0);
    if (ahead + 1 < N_FREQ_MULTIPLIERS)
      gap = quantized_multipliers[ahead + 1] - quantized_multipliers[ahead];
  }

  int32_t step = gap / FREQ_FINE_STEPS_PER_GAP;
  if (step < 1)
    step = 1;

  return (int16_t) iclamp((int32_t) value + step * delta, lo, hi);
}

static void ui_channel_param(const ChannelSetup* ch, UxState* state)
{
  ChannelConfig* chcfg = &state->engine_config->channel_state[ch->id];
  int8_t param         = (int8_t) state->engine_config->selected_param;
  int16_t delta        = enc_delta(&state->ui->in, ch->encoder);
  int8_t alt           = btn_down(&state->ui->in, ch->button);
  int8_t scene         = state->engine_state->active_scene;
  if (delta == 0)
    return;

  ux_note_channel_edit(state, ch->id);

  // Every encoder shows the parameter, not just this one: the reason to see it
  // at all is to compare this channel against the others.
  ui_show_param_display(state->ui);

  // By value: ChannelConfig is a packed FRAM record, so a pointer into it is
  // potentially unaligned.
  int16_t value = chcfg->params[scene][param];

  if (alt && param == CH_PARAM_FRQ)
  {
    // Fine-adjust on the frequency grid is a fraction of the interval it is
    // moving through, not a flat step. The grid is 1/f-linear, so a flat 32 was
    // half a gap near 1x - where two detents crossed to the next ratio - and
    // 0.2% of one between 1/64 and 1/128, where crossing took 250. A gap-
    // relative step is the same eight detents per ratio everywhere.
    value = freq_fine_adjust(value, delta);
  }
  else if (alt)
  {
    // Holding the channel button is the fine-adjust modifier. The other five
    // parameters are linear, so a flat step is the right one.
    value += 32 * delta;
  }
  else if (param == CH_PARAM_FRQ)
  {
    size_t idx = 0;
    value      = val_neighbour(value, delta, quantized_multipliers, N_FREQ_MULTIPLIERS, &idx);
  }
  else
  {
    value += delta * 256;
  }

  chcfg->params[scene][param] = value;
}

// Step one of a mode's discrete settings, one detent at a time.
//
// Clamped at both ends rather than wrapping. These are short lists of unrelated
// states, not continuous values, and rolling off one end into the other is
// never a move anyone means to make - the jump from "off" to the most extreme
// setting is the largest change on the page. Every one of these lists has its
// default at index 0, so spinning fully left is a reset you can make blind.
//
// By value rather than through a pointer, because the fields have different
// types - some of them enums, which are not int8_t and must not be aliased as
// one. No display to arm: in a shift mode the setting is what the LED shows.
static int step_setting(int current, int16_t delta, int count) { return iclamp(current + iclamp(delta, -1, 1), 0, count - 1); }

void ui_channel_update(const ChannelSetup* ch, UxState* state)
{
  const UiModeDesc* m  = ui_mode(state->ui->shift_state);
  int8_t long_pressed  = btn_released_after(&state->ui->in, ch->button, UI_T_LONG);
  int8_t pressed       = btn_ev(&state->ui->in, ch->button, BTN_EV_UP);
  int8_t pressing      = btn_down(&state->ui->in, ch->button);
  int16_t delta        = enc_delta(&state->ui->in, ch->encoder);
  ChannelConfig* chcfg = &state->engine_config->channel_state[ch->id];

  switch (m->channel_btn_action)
  {
  case CHB_SELECT:
    if (pressed)
      ui_sel_press(state, m->channel_btn_kind, ch->id, long_pressed);
    break;

  case CHB_MUTE_TOGGLE:
    // On release rather than press: easier to perform accurately, and it
    // matches every other momentary action in the UI.
    if (pressed)
      state->ui->muted[ch->id] = !state->ui->muted[ch->id];
    break;

  case CHB_RESET_PARAM:
  {
    // Only a press that spanned no encoder movement resets the param -
    // otherwise holding the button as a fine-adjust modifier would wipe the
    // value the user was just adjusting.
    uint32_t since_edit = state->hw_state->time - state->engine_state->channels_last_delta[ch->id];
    if (!pressed || btn_held(&state->ui->in, ch->button) >= since_edit)
      break;

    // A tap clears the parameter in the active scene; holding clears it in
    // every scene. The two are the same act on different scope, so they flash
    // the same colour and differ in how long it lasts.
    int8_t scene = long_pressed ? -1 : state->engine_state->active_scene;
    channel_reset_param(ch->id, state->engine_config, scene, (int8_t) state->engine_config->selected_param);
    ui_feedback_emit(state->ui, long_pressed ? FB_CLEAR_ALL : FB_CLEAR, TGT_CHANNEL, ch->id);
    return;
  }

  default:
    break;
  }

  if (delta == 0)
    return;

  switch (m->channel_enc_target)
  {
  case ENC_PARAM:
    ui_channel_param(ch, state);
    break;

  case ENC_SHAPE:
    chcfg->shape_mode = (int8_t) step_setting(chcfg->shape_mode, delta, SHAPE_MODE_COUNT);
    break;

  case ENC_QUANT:
    // While a trigger source is being picked the encoder would fight the
    // assignment for the same value.
    if (ui_sel_pending(state->ui))
      break;
    chcfg->quantize_mode = (int8_t) step_setting(chcfg->quantize_mode, delta, QUANTIZE_MODE_COUNT);
    break;

  case ENC_AMPMODE:
    if (pressing)
      break; // the button is picking a source, not modifying the encoder
    chcfg->input_amp_mode = (int8_t) step_setting(chcfg->input_amp_mode, delta, INPUT_AMP_MODE_COUNT);
    break;

  case ENC_MUTE:
    // Absolute rather than a toggle: right is always unmute and left is always
    // mute, so a row of channels can be muted by feel without checking each
    // one's state first. The button stays a toggle.
    state->ui->muted[ch->id] = delta < 0;
    break;

  case ENC_STEPPED_LENGTH:
    // Only the stepped modes have a pattern length, and the LED is dark on the
    // channels where it would do nothing - so the encoder does nothing there
    // too, rather than silently moving a setting with no visible effect.
    if (!shape_mode_is_stepped(chcfg->shape_mode))
      break;
    chcfg->st_length_idx = (int8_t) step_setting(chcfg->st_length_idx, delta, ST_LENGTH_COUNT);
    // The engine latches pattern length at the cycle wrap unless the user is
    // actively turning this channel - which is what this records.
    ux_note_channel_edit(state, ch->id);
    break;

  case ENC_CLAMP:
    chcfg->clamp_mode = (int8_t) step_setting(chcfg->clamp_mode, delta, CLAMP_MODE_COUNT);
    break;

  default:
    break;
  }
}
