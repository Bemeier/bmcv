#pragma once

// The panel.
//
// Two rules decide what is drawn here and what is not.
//
// Rack's own components are used wherever one fits the real part: the jacks
// (PJ301M, 8.03mm, and the board's are on a 10mm pitch), the illuminated
// switches (VCVLightBezel, whose light fills the cap - which is what a
// backlit switch is), and the three unlit tactiles (TL1105, which is the part
// on the board). They come with Rack's shading, its plug graphics and its
// cable behaviour, and a module that draws its own versions of those looks
// like a module that could not be bothered.
//
// The rest matches web/panel.js, because the two frontends are the same
// module: the real panel artwork underneath, an encoder whose face carries its
// LED colour with four spokes and a centre cap over it, and legends drawn on
// top. Rack has no endless encoder and no horizontal fader, so those are ours
// either way.

#include "plugin.hpp"

#include "bmcv_core.hpp"

// Board millimetres -> panel pixels. The board is centred in a panel one HP
// wider than itself; PANEL_VCV_OFF_* is that offset, generated alongside the
// SVG so the two cannot disagree.
inline Vec panelPos(PanelPoint p) { return mm2px(Vec(p.x + PANEL_VCV_OFF_X_MM, p.y + PANEL_VCV_OFF_Y_MM)); }

// The artwork was exported for a panel 0.28mm narrower than Rack's, so
// anything given in its coordinates - the mounting slots, the image itself -
// shifts by the difference between the two board offsets rather than sitting
// where its own corner would put it.
inline Vec artPos(PanelPoint p)
{
	return mm2px(Vec(p.x + PANEL_VCV_OFF_X_MM - PANEL_ART_OFF_X_MM, p.y + PANEL_VCV_OFF_Y_MM - PANEL_ART_OFF_Y_MM));
}

inline float panelMm(float mm) { return mm2px(mm); }

// web/panel.js's palette, so the two frontends are the same module rather than
// two designs of it.
static const NVGcolor BMCV_EDGE = nvgRGB(0x10, 0x13, 0x17);      // every outline on the panel
static const NVGcolor BMCV_PART = nvgRGB(0x6b, 0x70, 0x78);      // encoder bodies, fader handle
static const NVGcolor BMCV_LABEL = nvgRGB(0x15, 0x18, 0x1c);     // parameter name
static const NVGcolor BMCV_LABEL_SUB = nvgRGB(0x3a, 0x40, 0x48); // shift-mode name
static const NVGcolor BMCV_HALO = nvgRGB(0xe9, 0xe9, 0xe6);      // the light rim behind a legend

/* ---- panel artwork ------------------------------------------------------ */

// The real exported panel, laid over Rack's SvgPanel at 1:1.
//
// It is a raster, and Rack's SVG renderer cannot embed one, so it is drawn
// here instead of being part of res/BMCV.svg - which is left as a plain
// rectangle of the artwork's own background colour, so a missing file degrades
// to a blank panel rather than to a black one.
//
// Placed by the artwork's *board origin*, not by its corner: it was exported
// for an 81.0mm panel and Rack's is 81.28, so aligning the corners would put
// every cutout 0.14mm off the control that sits in it.
struct PanelArt : widget::Widget
{
	void draw(const DrawArgs& args) override
	{
		std::shared_ptr<window::Image> img = APP->window->loadImage(asset::plugin(pluginInstance, "res/BMCV.png"));
		if (!img || img->handle < 0)
			return;

		Vec at = artPos({0.f, 0.f});
		float x = at.x, y = at.y;
		float w = panelMm(PANEL_ART_W_MM);
		float h = panelMm(PANEL_ART_H_MM);

		nvgBeginPath(args.vg);
		nvgRect(args.vg, x, y, w, h);
		nvgFillPaint(args.vg, nvgImagePattern(args.vg, x, y, w, h, 0.f, img->handle, 1.f));
		nvgFill(args.vg);
	}
};

/* ---- legends ------------------------------------------------------------ */

// Drawn rather than left to the artwork, so they follow hw_setup.c if a
// button's function is ever reassigned - the same reason web/panel.js draws
// them. The light rim is what keeps a legend readable where it crosses one of
// the artwork's black cutouts; nanovg has no paint-order, so it is the string
// stamped around itself.
inline void drawLegend(NVGcontext* vg, float x, float y, const char* text, float sizeMm, NVGcolor col)
{
	if (!text || !text[0])
		return;
	// Rack ships no bold DejaVu; Nunito-Bold is the panel face most of the
	// stock modules letter with, which is the point of using it here.
	std::shared_ptr<window::Font> font = APP->window->loadFont(asset::system("res/fonts/Nunito-Bold.ttf"));
	if (!font)
		font = APP->window->loadFont(asset::system("res/fonts/DejaVuSans.ttf"));
	if (!font)
		return;

	nvgFontFaceId(vg, font->handle);
	nvgFontSize(vg, panelMm(sizeMm));
	nvgTextAlign(vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);

	float r = panelMm(0.32f);
	nvgFillColor(vg, BMCV_HALO);
	for (int i = 0; i < 8; i++)
	{
		float a = (float) i * (2.f * M_PI / 8.f);
		nvgText(vg, x + std::cos(a) * r, y + std::sin(a) * r, text, NULL);
	}

	nvgFillColor(vg, col);
	nvgText(vg, x, y, text, NULL);
}

// A button's two legends, above and below the cap. The parameter name goes on
// top and the shift mode underneath; the three action buttons only latch a
// mode, so they get the lower slot alone. Not on the cap: a 6mm cap has no
// room, and the cap is the thing that lights.
struct ButtonLegend : widget::Widget
{
	const char* param = "";
	const char* mode = "";
	float capMm = 3.0f;

	void draw(const DrawArgs& args) override
	{
		float cx = box.size.x / 2.f, cy = box.size.y / 2.f;
		if (param && param[0])
			drawLegend(args.vg, cx, cy - panelMm(capMm + 1.6f), param, 2.7f, BMCV_LABEL);
		// Tighter than the parameter line above it: the ctrl row and the scene
		// row are only 11mm apart, and this legend sits between them.
		if (mode && mode[0])
			drawLegend(args.vg, cx, cy + panelMm(capMm + 2.7f), mode, 2.3f, BMCV_LABEL_SUB);
	}
};

/* ---- lamps -------------------------------------------------------------- */

// One LED, drawn the way web/leds.js draws it: the hue at full saturation,
// alpha-blended over the part at the perceptual level led_color_of computed.
//
// Not left to Rack's light layer. That layer is composited *additively*, which
// is right over the dark panels Rack's own modules wear and wrong over this
// one - adding a 30%-bright red to a light grey panel gives white-pink, not
// red, and the brighter the LED the whiter it gets. So the colour goes down in
// the base layer, where alpha blending keeps the hue, and the light layer
// carries only the bloom around the part.
struct BmcvLamp : app::ModuleLightWidget
{
	BmcvLamp()
	{
		addBaseColor(SCHEME_RED);
		addBaseColor(SCHEME_GREEN);
		addBaseColor(SCHEME_BLUE);
		bgColor = nvgRGBA(0, 0, 0, 0);
		borderColor = nvgRGBA(0, 0, 0, 0);
	}

	// Read straight from the module rather than from `color`, which Rack has
	// already screen-blended into something with its own idea of saturation.
	//
	// led_color_of folds hue and brightness into the same three numbers, so the
	// peak *is* the level and dividing by it recovers the hue at full strength.
	bool lamp(NVGcolor* out, float opacity)
	{
		// Unlinked - the module browser's preview - is drawn dark rather than
		// at Rack's default full brightness. A BMCV at rest has one LED lit and
		// twenty off; a panel showing all twenty-one at once is not what the
		// module looks like, and on a light panel it just reads as white discs.
		float c[3] = {0.f, 0.f, 0.f};
		if (!module)
			return false;
		for (int i = 0; i < 3; i++)
		{
			c[i] = math::clamp(getLight(i)->getBrightness(), 0.f, 1.f);
		}
		float peak = std::max(c[0], std::max(c[1], c[2]));
		if (peak <= 0.f)
			return false;
		*out = nvgRGBAf(c[0] / peak, c[1] / peak, c[2] / peak, peak * opacity);
		return true;
	}

	// Rack's default halo is drawn from `color` and is a wide white-ish wash.
	// Each lamp below draws its own, shaped to the part it belongs to.
	void drawHalo(const DrawArgs& args) override {}
};

// How much of a lit part's own colour reaches the eye, and how much spills
// around it. web/leds.js's BASE_OPACITY, so the two frontends agree.
static const float LAMP_OPACITY = 0.78f;
static const float HALO_OPACITY = 0.6f;

/* ---- encoders ----------------------------------------------------------- */

// Part sizes in mm, matching web/panel.js, which matched the hardware: an
// encoder body smaller than its 12mm courtyard, and a cap that sits just
// inside the artwork's 7.03mm cutout so a hairline of it still shows.
static const float ENC_R = 5.6f;
static const float ENC_CAP_R = 3.35f;

// Pixels of drag per detent, as in the web sim.
static const float BMCV_PX_PER_DETENT = 6.f;

// The knob's face: four spokes and the rim of the centre cap. Four and not
// one, because a single mark reads like an absolute pointer and these are
// endless relative encoders with no meaningful zero.
inline void drawEncoderFace(NVGcontext* vg, float cx, float cy, float angleDeg)
{
	float rad = angleDeg * (float) M_PI / 180.f;
	nvgStrokeColor(vg, BMCV_EDGE);
	nvgLineCap(vg, NVG_ROUND);

	for (int i = 0; i < 4; i++)
	{
		float a = rad + (float) i * (float) M_PI / 2.f;
		float s = std::sin(a), k = std::cos(a);
		nvgBeginPath(vg);
		nvgMoveTo(vg, cx + s * panelMm(ENC_CAP_R + 0.3f), cy - k * panelMm(ENC_CAP_R + 0.3f));
		nvgLineTo(vg, cx + s * panelMm(ENC_R - 0.5f), cy - k * panelMm(ENC_R - 0.5f));
		nvgStrokeWidth(vg, panelMm(0.55f));
		nvgStroke(vg);
	}

	nvgBeginPath(vg);
	nvgCircle(vg, cx, cy, panelMm(ENC_CAP_R));
	nvgStrokeWidth(vg, panelMm(0.4f));
	nvgStroke(vg);
}

// The rotating part. Not an app::Knob: Knob derives its step from the
// parameter's *range*, and an endless encoder's range is only as wide as it is
// so the counter never reaches the end. That would make one pixel worth sixty
// detents. The pixel-to-detent rate belongs to the gesture, not to the range.
struct EncoderRing : app::ParamWidget
{
	// The push button under the same finger, asserted while Shift is held so
	// press-and-turn works with one mouse.
	int pushParamId = -1;
	bool shifted = false;
	float accum = 0.f;

	// Cosmetic. The firmware's encoders are relative and endless, so this only
	// says that something turned - but it has to move by exactly what was
	// sent, which is why every turn goes through turn().
	float angle = 0.f;

	EncoderRing() { box.size = Vec(panelMm((ENC_R + 2.4f) * 2.f), panelMm((ENC_R + 2.4f) * 2.f)); }

	void initParamQuantity() override
	{
		app::ParamWidget::initParamQuantity();
		ParamQuantity* pq = getParamQuantity();
		if (pq)
		{
			pq->snapEnabled = true;
			pq->smoothEnabled = false;
		}
	}

	void turn(float detents)
	{
		ParamQuantity* pq = getParamQuantity();
		if (!pq)
			return;
		pq->setValue(std::round(pq->getValue() + detents));
		angle += detents * 12.f; // degrees, as in web/panel.js
	}

	void setPush(bool down)
	{
		if (!module || pushParamId < 0)
			return;
		module->params[pushParamId].setValue(down ? 1.f : 0.f);
	}

	void onDragStart(const DragStartEvent& e) override
	{
		if (e.button != GLFW_MOUSE_BUTTON_LEFT)
			return;
		accum = 0.f;
		shifted = (APP->window->getMods() & RACK_MOD_MASK) == GLFW_MOD_SHIFT;
		if (shifted)
			setPush(true);
	}

	void onDragMove(const DragMoveEvent& e) override
	{
		if (e.button != GLFW_MOUSE_BUTTON_LEFT)
			return;
		// Up is clockwise, as it is on every Rack knob and in the web sim.
		accum += -e.mouseDelta.y / BMCV_PX_PER_DETENT;
		float whole = std::trunc(accum);
		if (whole != 0.f)
		{
			accum -= whole;
			turn(whole);
		}
	}

	void onDragEnd(const DragEndEvent& e) override
	{
		if (e.button != GLFW_MOUSE_BUTTON_LEFT)
			return;
		if (shifted)
		{
			setPush(false);
			shifted = false;
		}
	}

	void onHoverScroll(const HoverScrollEvent& e) override
	{
		if (e.scrollDelta.y == 0.f)
			return;
		turn(e.scrollDelta.y > 0.f ? 1.f : -1.f);
		e.consume(this);
	}

	void draw(const DrawArgs& args) override
	{
		float c = box.size.x / 2.f;

		// Outline only: the body and its colour are drawn by the EncoderLight
		// underneath, so that the LED lands between the two and not on top of
		// the spokes.
		nvgBeginPath(args.vg);
		nvgCircle(args.vg, c, c, panelMm(ENC_R));
		nvgStrokeColor(args.vg, BMCV_EDGE);
		nvgStrokeWidth(args.vg, panelMm(0.45f));
		nvgStroke(args.vg);

		drawEncoderFace(args.vg, c, c, angle);

		app::ParamWidget::draw(args);
	}
};

// The WS2812 sits behind the encoder and lights it through the cap.
//
// This draws the knob's body as well as its colour, because the two have to be
// in that order - grey body, then the LED over it, then the spokes over that,
// exactly as web/panel.js layers them - and the body cannot be left to the
// EncoderRing above, which draws after this.
struct EncoderLight : BmcvLamp
{
	EncoderLight()
	{
		box.size = Vec(panelMm(ENC_R * 3.0f), panelMm(ENC_R * 3.0f));
	}

	void drawBackground(const DrawArgs& args) override
	{
		float c = box.size.x / 2.f;

		nvgBeginPath(args.vg);
		nvgCircle(args.vg, c, c, panelMm(ENC_R));
		nvgFillColor(args.vg, nvgTransRGBAf(BMCV_PART, 0.16f));
		nvgFill(args.vg);

		NVGcolor lit;
		if (!lamp(&lit, LAMP_OPACITY))
			return;
		nvgBeginPath(args.vg);
		nvgCircle(args.vg, c, c, panelMm(ENC_R * 0.95f));
		nvgFillColor(args.vg, lit);
		nvgFill(args.vg);
	}

	// The spill around the body, and only that: additive light cannot draw the
	// dark spokes back on top of itself, so nothing that needs to stay legible
	// belongs in this layer.
	void drawLight(const DrawArgs& args) override
	{
		NVGcolor lit;
		if (!lamp(&lit, HALO_OPACITY))
			return;

		float c = box.size.x / 2.f;
		float inner = panelMm(ENC_R * 0.94f);
		float outer = panelMm(ENC_R * 1.45f);
		NVGcolor edge = lit;
		edge.a = 0.f;

		nvgBeginPath(args.vg);
		nvgCircle(args.vg, c, c, outer);
		nvgCircle(args.vg, c, c, inner);
		nvgPathWinding(args.vg, NVG_HOLE);
		nvgFillPaint(args.vg, nvgRadialGradient(args.vg, c, c, inner, outer, lit, edge));
		nvgFill(args.vg);
	}
};

// The centre cap, on top of the ring so a plain click is a push and a drag on
// the ring around it is a turn. app::Switch in momentary mode already handles
// press on drag start and release on drag end, including release outside the
// widget, which a plain onButton handler gets wrong.
struct EncoderCap : app::Switch
{
	// The cap is a good half of the knob, and a wheel that does nothing over
	// the middle of it reads as a dead spot. web/panel.js binds the wheel to
	// both parts for the same reason.
	EncoderRing* ring = NULL;

	EncoderCap()
	{
		momentary = true;
		box.size = Vec(panelMm(ENC_CAP_R * 2.f), panelMm(ENC_CAP_R * 2.f));
	}

	void draw(const DrawArgs& args) override
	{
		ParamQuantity* pq = getParamQuantity();
		bool down = pq && pq->getValue() > 0.5f;

		float c = box.size.x / 2.f;
		nvgBeginPath(args.vg);
		nvgCircle(args.vg, c, c, panelMm(ENC_CAP_R));
		nvgFillColor(args.vg, nvgTransRGBAf(BMCV_PART, down ? 0.55f : 0.3f));
		nvgFill(args.vg);
		nvgStrokeColor(args.vg, BMCV_EDGE);
		nvgStrokeWidth(args.vg, panelMm(0.4f));
		nvgStroke(args.vg);

		app::Switch::draw(args);
	}

	void onHoverScroll(const HoverScrollEvent& e) override
	{
		if (ring)
			ring->onHoverScroll(e);
	}
};

/* ---- switches ----------------------------------------------------------- */

// The lamp inside a VCVLightBezel. Rack's bezel draws the physical rim and its
// framebuffer runs first; this paints the cap over the middle of it, so the
// colour lands on the part rather than being added to the panel around it.
struct SwitchLamp : BmcvLamp
{
	void drawBackground(const DrawArgs& args) override
	{
		NVGcolor lit;
		if (!lamp(&lit, LAMP_OPACITY))
			return;
		float c = box.size.x / 2.f;
		nvgBeginPath(args.vg);
		nvgCircle(args.vg, c, c, c * 0.94f);
		nvgFillColor(args.vg, lit);
		nvgFill(args.vg);
	}

	void drawLight(const DrawArgs& args) override
	{
		NVGcolor lit;
		if (!lamp(&lit, HALO_OPACITY))
			return;
		float c = box.size.x / 2.f;
		NVGcolor edge = lit;
		edge.a = 0.f;
		nvgBeginPath(args.vg);
		nvgCircle(args.vg, c, c, c * 1.55f);
		nvgFillPaint(args.vg, nvgRadialGradient(args.vg, c, c, c * 0.9f, c * 1.55f, lit, edge));
		nvgFill(args.vg);
	}
};

// The three centre-column tactiles, at the size of the part.
//
// TL1105 is drawn 5.2mm across and SW14-16 are 7mm switches, against 6mm for
// the lit ones below. Scaled to keep that ratio against the 7.2mm bezel those
// are drawn with, rather than to the bare part - otherwise the bigger switch
// on the board is the smaller one on the panel.
static const float TACTILE_SCALE = 8.4f / 5.2f;

struct BmcvTactile : TL1105
{
	BmcvTactile() { box.size = box.size.mult(TACTILE_SCALE); }

	void draw(const DrawArgs& args) override
	{
		nvgSave(args.vg);
		nvgScale(args.vg, TACTILE_SCALE, TACTILE_SCALE);
		TL1105::draw(args);
		nvgRestore(args.vg);
	}
};

/* ---- crossfader --------------------------------------------------------- */

// RV13 is horizontal on the real board, and Rack has no horizontal stock
// fader. The artwork draws the slot, so only the handle is drawn here - in the
// same grey the encoder bodies wear, so the two read as the same kind of part
// rather than the fader being the brightest thing on the panel.
struct BmcvCrossfader : app::SliderKnob
{
	BmcvCrossfader()
	{
		horizontal = true;
		forceLinear = true;
		box.size = Vec(panelMm(PANEL_SLIDER_TRAVEL_MM + 4.f), panelMm(11.f));
	}

	void draw(const DrawArgs& args) override
	{
		float mid = box.size.y / 2.f;
		float travel = panelMm(PANEL_SLIDER_TRAVEL_MM);
		float x0 = (box.size.x - travel) / 2.f;

		// The parameter is the handle's position from the left, so dragging
		// right raises it as it does on every other Rack fader. Which end is
		// which scene is the module's business, not the widget's.
		ParamQuantity* pq = getParamQuantity();
		float hx = x0 + (pq ? pq->getScaledValue() : 0.f) * travel;

		nvgBeginPath(args.vg);
		nvgRoundedRect(args.vg, hx - panelMm(2.f), mid - panelMm(3.3f), panelMm(4.f), panelMm(6.6f), panelMm(1.1f));
		nvgFillColor(args.vg, BMCV_PART);
		nvgFill(args.vg);
		nvgStrokeColor(args.vg, BMCV_EDGE);
		nvgStrokeWidth(args.vg, panelMm(0.4f));
		nvgStroke(args.vg);

		app::SliderKnob::draw(args);
	}
};
