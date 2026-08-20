#include "led_fb.h"
#include "hw_setup.h"
#include "led_curve.h"
#include <math.h>

// Colour conversion moved here from ws2811.c: it is presentation logic, not
// driver logic, and keeping it hardware-free makes it testable.

// Whole duty steps in, 8.8 out. Every setter that names a VAL_* constant goes
// through here: those are already whole steps and gain nothing from the
// fraction, but the framebuffer has one unit and this is it.
static uint16_t whole(uint8_t v) { return (uint16_t) v * LED_UNIT; }

void led_set_rgb(UxState* state, int16_t idx, uint8_t r, uint8_t g, uint8_t b)
{
  if (idx < 0 || idx >= LED_COUNT)
    return;
  LedRgb* led = &state->engine_state->leds[idx];
  led->r      = whole(r);
  led->g      = whole(g);
  led->b      = whole(b);
}

// Light out of an LED, in the units LED_PALETTE_REF and LED_CV_CEIL are written
// in: duty on a nominal weight-1.0 die.
static float luminance(uint16_t r, uint16_t g, uint16_t b)
{
  return (float) r * LED_W_RED + (float) g * LED_W_GREEN + (float) b * LED_W_BLUE;
}

// Store a colour scaled to the brightness it asked for.
//
// The hue arrives as a shape - which primaries, in what proportion - and the
// value arrives as a brightness. Storing the shape at that value confuses the
// two: how much light comes out then depends on which dies the hue happens to
// use, which is why a VAL_LOW purple and a VAL_LOW yellow were three times
// apart. Scaling the whole shape leaves the hue exactly where it was and makes
// the value mean what the palette always meant by it.
static void balance(LedRgb* led, uint16_t r, uint16_t g, uint16_t b, uint16_t v)
{
  float lum = luminance(r, g, b);
  if (lum <= 0.0f)
  {
    led->r = led->g = led->b = 0;
    return;
  }

  float k = (float) v * LED_PALETTE_REF / lum;

  // A dim hue can need more duty than a bright one for the same light - a red
  // runs at over twice a green - so the scale has to stop where the framebuffer
  // does. Bounded on the peak rather than clamped per primary, which would
  // flatten the hue toward white as it saturated instead of just topping out.
  float peak = (float) (r > g ? r : g);
  if ((float) b > peak)
    peak = (float) b;
  float ceiling = (float) (255 * LED_UNIT);
  if (peak * k > ceiling)
    k = ceiling / peak;

  // ...and where the brightest primary would land too far down its own curve to
  // show what it was asked for, scale until it does. See LED_MIN_ON_DUTY.
  //
  // The whole triple, not each primary: flooring them individually lifts the
  // trace of red that a saturated green carries as a rounding artefact up to
  // the same duty as the green itself, and turns it yellow. Scaling leaves
  // every ratio exactly where it was, so the hue is the hue - it is only
  // brighter.
  //
  // Which means the lift lands where the problem is. A hue on one die already
  // clears the floor and does not move; a hue split between two reaches it at
  // twice the value and gets scaled up. Yellow, the evenest split, moves most.
  const float floor_duty = LED_MIN_ON_DUTY * (float) LED_UNIT;
  const float peak_duty  = peak * k;
  if (peak_duty > 0.0f && peak_duty < floor_duty)
    k *= floor_duty / peak_duty;

  led->r = (uint16_t) ((float) r * k + 0.5f);
  led->g = (uint16_t) ((float) g * k + 0.5f);
  led->b = (uint16_t) ((float) b * k + 0.5f);
}

// The value in framebuffer units rather than whole steps. Internal: everything
// outside names a VAL_*, and the fraction only exists so that the balancing
// above has somewhere to put a hue that needs a duty between two steps.
static void set_hsv_fine(UxState* state, int16_t idx, uint8_t h, uint8_t s, uint16_t v)
{
  if (idx < 0 || idx >= LED_COUNT)
    return;
  LedRgb* led = &state->engine_state->leds[idx];

  uint16_t r, g, b;

  if (s == 0)
  {
    r = g = b = v;
    balance(led, r, g, b, v);
    return;
  }

  uint8_t region    = h / 43;
  uint8_t remainder = (h - region * 43) * 6;

  // Widened to 32 bits for the multiply and back: v is the framebuffer's own
  // unit now, up to 255 * LED_UNIT, and the products overflow 16 bits long
  // before the shift brings them back.
  uint16_t p = (uint16_t) (((uint32_t) v * (255 - s)) >> 8);
  uint16_t q = (uint16_t) (((uint32_t) v * (255 - ((s * remainder) >> 8))) >> 8);
  uint16_t t = (uint16_t) (((uint32_t) v * (255 - ((s * (255 - remainder)) >> 8))) >> 8);

  switch (region)
  {
  case 0:
    r = v, g = t, b = p;
    break;
  case 1:
    r = q, g = v, b = p;
    break;
  case 2:
    r = p, g = v, b = t;
    break;
  case 3:
    r = p, g = q, b = v;
    break;
  case 4:
    r = t, g = p, b = v;
    break;
  default:
    r = v, g = p, b = q;
    break;
  }

  balance(led, r, g, b, v);
}

void led_set_hsv(UxState* state, int16_t idx, uint8_t h, uint8_t s, uint8_t v) { set_hsv_fine(state, idx, h, s, whole(v)); }

void led_wash(UxState* state, int16_t idx, uint16_t v, uint8_t keep)
{
  if (idx < 0 || idx >= LED_COUNT)
    return;
  LedRgb* led = &state->engine_state->leds[idx];

  // `v` is a value, the same thing led_set_hsv takes, so it has to be compared
  // against light and not against duty. Against duty the marker would be a
  // different strength on every hue - hard on the ones balancing runs at a low
  // duty, invisible on the ones it runs high - which is the imbalance the
  // balancing exists to remove, reappearing one layer further up.
  float lum  = luminance(led->r, led->g, led->b);
  float want = (float) v * LED_PALETTE_REF;

  if (lum <= 0.0f)
  {
    // Nothing underneath to keep or to brighten: white, at that value.
    balance(led, v, v, v, v);
    return;
  }

  if (lum < want)
  {
    // Brightening is what balance() does - scale the shape until it puts out
    // the value asked for - so it is what does it here, rather than a second
    // copy of the same multiply that would have to repeat the ceiling guard to
    // be safe. A saturated blue at VAL_BASE is already at duty 124; doubling it
    // by hand ran to within a few hundred of wrapping the framebuffer.
    balance(led, led->r, led->g, led->b, v);
  }

  uint16_t peak = led->r > led->g ? led->r : led->g;
  if (led->b > peak)
    peak = led->b;

  // Toward the peak, which is toward neutral: the primary already at the peak
  // does not move, so this only ever adds light.
  uint32_t pull = 255u - keep;
  led->r        = (uint16_t) (led->r + (uint32_t) (peak - led->r) * pull / 255u);
  led->g        = (uint16_t) (led->g + (uint32_t) (peak - led->g) * pull / 255u);
  led->b        = (uint16_t) (led->b + (uint32_t) (peak - led->b) * pull / 255u);
}

// Perceived lightness at a magnitude, 0..1 either side of zero.
//
// Two straight lines meeting at LED_CV_KNEE at half scale. Straight in
// *perceived* space, not in duty: equal voltage steps are meant to look like
// equal steps, which a linear duty ramp never did - it spent four fifths of the
// visible range on the first quarter of the voltage and compressed everything
// above into the part of the scale nobody can read.
//
// The knee is the whole of what is left of the old two-stage idea. Instead of
// handing the range past 5V a different colour, it hands it a smaller share of
// the same ramp: the +/-5V region most patches live in gets 70% of what the eye
// can see, and 5..10V gets the rest.
static float cv_lightness(float x)
{
  if (x <= 0.5f)
    return x * (LED_CV_KNEE * 2.0f);
  return LED_CV_KNEE + (x - 0.5f) * ((1.0f - LED_CV_KNEE) * 2.0f);
}

// Luminance share handed to the primary that is *not* the polarity's own.
//
// Two jobs, at opposite ends. Around zero it runs to a half, so the two
// primaries meet and 0V reads as a neutral amber ember rather than as a red or
// a green too dim to name - which is what makes the zero crossing itself
// visible. Past half scale it rises to LED_CV_WARM, so green warms toward
// yellow-green and red toward orange: with the blue axis gone that shift is the
// only thing that says a signal has left the usual range. Both stay well short
// of the other polarity's colour.
static float cv_mix(float x)
{
  if (x < LED_CV_ZERO_SPAN)
    return 0.5f * (1.0f - x / LED_CV_ZERO_SPAN);
  if (x > 0.5f)
    return LED_CV_WARM * (x - 0.5f) * 2.0f;
  return 0.0f;
}

// Luminance -> 8.8 duty on one die. Saturated at the top of the framebuffer's
// range, which the defaults never reach - it is here because the constants are
// meant to be edited.
static uint16_t cv_duty(float luminance, float weight)
{
  float duty = luminance * LED_CV_CEIL / weight * (float) LED_UNIT;
  if (duty <= 0.0f)
    return 0;
  if (duty >= (float) (255 * LED_UNIT))
    return 255 * LED_UNIT;
  return (uint16_t) (duty + 0.5f);
}

// Bipolar level -> colour, in whichever converter domain the caller is in:
// green for positive, red for negative, and nothing else. `half_scale` is the
// count that corresponds to 5V in that domain - ADC_5V or DAC_5V - and the ramp
// runs to twice it.
//
// Blue used to ramp in above half scale, so the top of the range read as teal
// and pink. It was legible and it was the wrong two colours: everywhere else a
// level is red or green, and nothing about 7V is worth changing colour family
// over. What replaces it is the knee in cv_lightness, the warm shift in cv_mix,
// and the per-die weighting here - the last of which is the reason the ring can
// be trusted at all, since the green die is roughly twice the red one and -3V
// used to read visibly dimmer than +3V.
void led_set_bipolar(UxState* state, int16_t idx, int32_t val, int32_t half_scale)
{
  if (idx < 0 || idx >= LED_COUNT)
    return;
  if (half_scale <= 0)
    return;
  LedRgb* led = &state->engine_state->leds[idx];

  // Widened before negating and saturated on the way back: val is int32 and
  // -INT32_MIN is not representable in one, so the obvious `(val < 0) ? -val`
  // is undefined at that single input.
  int64_t wide    = (val < 0) ? -(int64_t) val : (int64_t) val;
  int32_t abs_val = (int32_t) (wide > INT32_MAX ? INT32_MAX : wide);

  // Clamped rather than wrapped. The old code masked the blue term with 0xFF,
  // so a param value past twice half scale rolled the colour over and drew a
  // near-full ring as a nearly dark one.
  float x = (float) abs_val / (float) (2 * half_scale);
  if (x > 1.0f)
    x = 1.0f;

  // The ember is where the ramp starts, not a clamp under it. Clamping put a
  // floor across the quiet end of the scale and everything below about 1.5V
  // came out the same brightness; lifting the whole ramp onto it keeps every
  // step of the range distinct and still leaves 0V lit.
  float lightness = LED_CV_FLOOR + (1.0f - LED_CV_FLOOR) * cv_lightness(x);
  float y         = powf(lightness, LED_GAMMA);

  // Luminance-preserving: warming or neutralising a colour moves light between
  // the two dies rather than adding any, so brightness stays the magnitude's to
  // carry and nothing else's.
  float mix   = cv_mix(x);
  float lead  = y * (1.0f - mix);
  float trail = y * mix;

  led->r = cv_duty(val < 0 ? lead : trail, LED_W_RED);
  led->g = cv_duty(val < 0 ? trail : lead, LED_W_GREEN);
  led->b = 0;
}

// A signed value that is not a voltage: same ramp, two hues that the voltage
// arc cannot reach.
//
// Everything about the shape is deliberately the voltage ramp's - the knee, the
// gamma, the neutral crossing, dark at zero - so that reading one teaches the
// other. Only the colour family differs, and that difference is the whole
// message: teal and pink light the blue die, which led_set_bipolar never does.
//
// Through led_set_hsv rather than onto the dies directly, because that is where
// the per-hue light balancing already lives. led_set_bipolar mixes two
// primaries and so has to weight them itself; a hue pair does not.
//
// `full_scale` is the value that reaches the top of the ramp, and `sat` is the
// caller's - the parameter ring spends saturation on how near a landmark the
// value sits, the same way the FRQ ring spends it on how near a grid ratio.
void led_set_signed(UxState* state, int16_t idx, int32_t val, int32_t full_scale, uint8_t hue_neg, uint8_t hue_pos, uint8_t sat)
{
  if (idx < 0 || idx >= LED_COUNT)
    return;
  if (full_scale <= 0)
    return;

  // Widened before negating, as in led_set_bipolar: -INT32_MIN is not
  // representable in an int32.
  int64_t wide    = (val < 0) ? -(int64_t) val : (int64_t) val;
  int32_t abs_val = (int32_t) (wide > INT32_MAX ? INT32_MAX : wide);

  float x = (float) abs_val / (float) full_scale;
  if (x > 1.0f)
    x = 1.0f;

  float lightness = LED_CV_FLOOR + (1.0f - LED_CV_FLOOR) * cv_lightness(x);
  float y         = powf(lightness, LED_GAMMA);

  // The same light at full scale as the voltage ramp puts out, so the row does
  // not jump when the parameter display decays back to the output level.
  // LED_CV_CEIL is duty on a nominal weight-1.0 die; a palette value is light
  // against LED_PALETTE_REF, so the two are that ratio apart.
  float v = y * (LED_CV_CEIL / LED_PALETTE_REF) * (float) LED_UNIT;

  float hue = (float) (val < 0 ? hue_neg : hue_pos);

  // Neutral at the crossing, for the reason the ramp has an ember there: the
  // zero itself should be visible rather than being a red or a green too dim to
  // name. Both hues are on the same side of the wheel, so the midpoint between
  // them is a blend and never a wrap.
  if (x < LED_CV_ZERO_SPAN)
  {
    float mid = 0.5f * ((float) hue_neg + (float) hue_pos);
    float t   = 1.0f - x / LED_CV_ZERO_SPAN;
    hue += (mid - hue) * t;
  }

  set_hsv_fine(state, idx, (uint8_t) (hue + 0.5f), sat, (uint16_t) v);
}

void led_set_adcr(UxState* state, int16_t idx, int16_t val) { led_set_bipolar(state, idx, val, ADC_5V); }

void led_set_dac(UxState* state, int16_t idx, int32_t val) { led_set_bipolar(state, idx, val, DAC_5V); }

void led_clear_all(UxState* state)
{
  for (int16_t i = 0; i < LED_COUNT; i++)
  {
    led_set_rgb(state, i, 0, 0, 0);
  }
}

// One primary, one frame: emit whole duty steps and carry the remainder into
// the next frame.
//
// First-order sigma-delta, which is all "accumulate and take the carry" is. A
// constant 1.5 comes out 1,2,1,2 and averages what was asked for; the error it
// leaves is shaped toward the frame rate rather than sitting at DC, so what is
// left is high-frequency noise instead of a wrong colour.
//
// It never swings more than one duty step, and every level this matters at is
// under four steps out of 255 - so the flicker is a fraction of a percent of
// full scale at ~150Hz, which is not something an eye finds. That is the whole
// safety argument: dithering the *upper* range at 300 frames a second would be
// perfectly visible.
static_assert(LED_FRAC_BITS == 8, "LedDither carries the remainder in a byte");

static uint8_t dither_one(uint16_t want, uint8_t* acc)
{
  uint32_t sum  = (uint32_t) want + *acc;
  uint32_t step = sum >> LED_FRAC_BITS;
  *acc          = (uint8_t) (sum & (LED_UNIT - 1));
  return (uint8_t) (step > 255 ? 255 : step);
}

// The framebuffer as the WS2812 wants it. Called from the flush, not from the
// renderer: the framebuffer has to keep holding the colour that was meant, or
// the simulator - which samples it at 60Hz and has no dither to average - would
// show the noise instead of the ramp.
//
// The accumulator belongs to the caller rather than to a static here, so the
// quantiser is a pure function of its inputs and a test can drive a hundred
// frames through it and check what came out.
void led_fb_quantize(const LedRgb* fb, LedDither* acc, uint8_t* out, int16_t count)
{
  for (int16_t i = 0; i < count; i++)
  {
    out[i * 3]     = dither_one(fb[i].r, &acc[i].r);
    out[i * 3 + 1] = dither_one(fb[i].g, &acc[i].g);
    out[i * 3 + 2] = dither_one(fb[i].b, &acc[i].b);
  }
}
