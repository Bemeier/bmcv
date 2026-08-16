// BMCV as a Rack module.
//
// Everything below the widget layer is the firmware. This file allocates one
// BmcvInstance, samples Rack's ports into an InputSample, ticks the engine on
// a divider and pushes what comes out at the ports and the lights. There is no
// DSP here and there must never be any: a behaviour that exists in this file
// and not in Core/Src/Lib is a behaviour the hardware does not have.

#include "plugin.hpp"

#include "bmcv_core.hpp"
#include "widgets.hpp"

#include <cmath>
#include <cstring>

// What the engine ticks at. The board manages roughly this, and the engine is
// dt-driven so any value is *correct* - this one just makes the simulation
// match the hardware's resolution. sim_tickdiv_config rounds it to a whole
// number of host frames and reports the rate that really results.
static const float BMCV_CONTROL_HZ = 2000.f;

// How often the LED framebuffer is read. The UX pass inside the engine only
// re-renders every 8ms, so anything above ~125Hz is looking at the same bytes
// twice; anything below it drops frames the firmware meant you to see, and the
// LED language is full of 90ms dips.
static const float BMCV_LIGHT_HZ = 125.f;

struct BMCV : Module
{
	enum ParamId
	{
		ENUMS(ENCODER_PARAM, N_ENCODERS),
		ENUMS(BUTTON_PARAM, N_BUTTONS),
		SLIDER_PARAM,
		PARAMS_LEN
	};
	enum InputId
	{
		ENUMS(CV_INPUT, N_INPUTS),
		INPUTS_LEN
	};
	enum OutputId
	{
		ENUMS(CH_OUTPUT, N_CHANNELS),
		OUTPUTS_LEN
	};
	enum LightId
	{
		ENUMS(LED_LIGHT, LED_COUNT * 3),
		LIGHTS_LEN
	};

	BmcvInstance m;
	InputSample sample;
	SimTickDiv tickDiv;
	SimTrigLatch trig;

	SlotStore store;
	PresetIo io;

	// Held between control ticks so the ports carry a value on every frame.
	float outV[N_CHANNELS] = {};

	// The encoder params are free-running counters, and a patch reload or a
	// param reset can move one by thousands at once. Deltas are taken against
	// this rather than against the param's own last value, so a jump that the
	// user did not make produces no rotation at all.
	int16_t encPrev[N_ENCODERS] = {};
	bool encPrimed = false;

	int lightCounter = 0;
	int lightPeriod = 1;

	BMCV()
	{
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);

		for (int e = 0; e < N_ENCODERS; e++)
		{
			// Endless: the value is a detent count, not a position, so the range
			// only has to be wider than anyone will ever drag. int16 is what the
			// firmware reads it as.
			configParam(ENCODER_PARAM + e, -32768.f, 32767.f, 0.f,
			            string::f("Channel %d encoder", panel_encoder[e].channel));
			ParamQuantity* pq = getParamQuantity(ENCODER_PARAM + e);
			pq->snapEnabled = true;
			pq->smoothEnabled = false;
			// Both would move the counter by an arbitrary amount, which on
			// hardware is not something an encoder can do.
			pq->randomizeEnabled = false;
			pq->resetEnabled = false;
		}

		for (int b = 0; b < N_BUTTONS; b++)
		{
			configButton(BUTTON_PARAM + b, panel_button_label[b]);
		}

		configParam(SLIDER_PARAM, 0.f, 1.f, 0.f, "Scene crossfader", "%", 0.f, 100.f);

		for (int i = 0; i < N_INPUTS; i++)
		{
			configInput(CV_INPUT + i, string::f("CV %d", i));
		}
		for (int c = 0; c < N_CHANNELS; c++)
		{
			configOutput(CH_OUTPUT + c, string::f("Channel %d", c));
		}
		for (int l = 0; l < LED_COUNT; l++)
		{
			configLight(LED_LIGHT + l * 3, string::f("LED %d", l));
		}

		slot_store_init(&store, &io);

		// A placeholder until Rack tells us the real rate, which it does when the
		// module is added. Not read from APP here: a Module is also constructed
		// by the module browser, and a constructor is the wrong place to reach
		// for global engine state. It has to come before boot(), which starts
		// the instance at whatever the clock currently reads.
		setRate(48000.f, false);
		boot();
	}

	// Power-on, in the firmware's own words. Everything that is not persisted -
	// phases, mutes, the shift page - comes back at its default, as it does when
	// a module is unplugged and plugged back in.
	void boot()
	{
		std::memset(&sample, 0, sizeof(sample));
		sim_input_slider(&sample, 0.f);
		sim_trig_reset(&trig);
		// Started at the host's clock rather than at zero. A patch that has been
		// open for a minute and then resets this module would otherwise hand the
		// engine a first dt of a minute.
		bmcv_instance_init(&m, &io, tickDiv.now_us);
		std::memset(outV, 0, sizeof(outV));
		encPrimed = false;
	}

	void onReset(const ResetEvent& e) override
	{
		Module::onReset(e); // params back to default first, so priming sees them
		slot_store_clear(&store);
		boot();
	}

	void onSampleRateChange(const SampleRateChangeEvent& e) override { setRate(e.sampleRate, true); }

	// `keepClock` is the difference between a module coming up and a running
	// module having the rate changed under it. In the second case the engine's
	// clock must carry over: it reads elapsed time as an unsigned difference,
	// so a timestamp that steps back a second is a 71 minute step forward.
	void setRate(float sampleRate, bool keepClock)
	{
		if (keepClock)
			sim_tickdiv_reconfig(&tickDiv, sampleRate, BMCV_CONTROL_HZ);
		else
			sim_tickdiv_config(&tickDiv, sampleRate, BMCV_CONTROL_HZ);

		lightPeriod = std::max(1, (int) std::lround(sim_tickdiv_rate_hz(&tickDiv) / BMCV_LIGHT_HZ));
		lightCounter = 0;
	}

	void process(const ProcessArgs& args) override
	{
		const HwSetup* hw = m.hw_setup;

		// Every frame, not every tick. On hardware the gate edge is caught in
		// the ADC's DMA callback, which runs far faster than the engine loop; a
		// 1ms trigger must not be lost because the tick landed either side of it.
		for (int i = 0; i < N_INPUTS; i++)
		{
			sim_input_cv(&sample, &trig, hw, (uint8_t) i, inputs[CV_INPUT + i].getVoltage());
		}

		if (sim_tickdiv_step(&tickDiv))
		{
			readControls();

			sim_input_take_trigs(&sample, &trig);

			bmcv_instance_tick(&m, &sample, tickDiv.now_us);

			for (int c = 0; c < N_CHANNELS; c++)
			{
				// The gated level, not the raw one: mute is an output-stage gain,
				// so this is what leaves the module. The same array the firmware
				// hands to the DAC.
				outV[c] = sim_dac_to_volts(m.engine_state.channels_gated_level[c]);
			}

			if (++lightCounter >= lightPeriod)
			{
				lightCounter = 0;
				readLights();
			}
		}

		for (int c = 0; c < N_CHANNELS; c++)
		{
			outputs[CH_OUTPUT + c].setVoltage(outV[c]);
		}
	}

	void readControls()
	{
		if (!encPrimed)
		{
			for (int e = 0; e < N_ENCODERS; e++)
			{
				encPrev[e] = encoderParam(e);
			}
			encPrimed = true;
		}

		for (int e = 0; e < N_ENCODERS; e++)
		{
			int16_t p = encoderParam(e);
			// Accumulated rather than assigned, so the position the firmware sees
			// is free-running and wraps exactly as the hardware's does. input_fold
			// takes a wrap-safe delta from it.
			sample.encoder_pos[e] = (int16_t) (sample.encoder_pos[e] + (int16_t) (p - encPrev[e]));
			encPrev[e] = p;
		}

		for (int b = 0; b < N_BUTTONS; b++)
		{
			sample.button_down[b] = params[BUTTON_PARAM + b].getValue() > 0.5f ? 1 : 0;
		}

		// The parameter is where the handle sits from the left. Scene A anchors
		// at SLIDER_MAX_VALUE and is the left-hand end of the panel, so the
		// leftmost position is full scale and the two run opposite ways.
		sim_input_slider(&sample, 1.f - params[SLIDER_PARAM].getValue());
	}

	int16_t encoderParam(int e) { return (int16_t) (int) std::lround(params[ENCODER_PARAM + e].getValue()); }

	void readLights()
	{
		for (int l = 0; l < LED_COUNT; l++)
		{
			LedColor c = led_color_of(m.engine_state.leds[l]);
			// Not setBrightnessSmooth: the firmware's LED language is built out of
			// 90ms dips and short flashes, and a decay envelope on top of it would
			// smear exactly the parts that carry the meaning.
			lights[LED_LIGHT + l * 3 + 0].setBrightness(c.r);
			lights[LED_LIGHT + l * 3 + 1].setBrightness(c.g);
			lights[LED_LIGHT + l * 3 + 2].setBrightness(c.b);
		}
	}

	/* ---- persistence ----------------------------------------------------- */
	//
	// The preset slots are the whole of the module's persisted state, because
	// the module boots from one of them: bmcv_instance_init() loads
	// CONFIG_AUTOSAVE_SLOT, which the engine rewrites every 2 seconds. Saving a
	// patch is a power cut, so the live config goes into that slot first rather
	// than trusting the timer to have fired since the last edit.

	json_t* dataToJson() override
	{
		io.store(io.user, &m.engine_config, CONFIG_AUTOSAVE_SLOT);

		json_t* root = json_object();
		// The shape of this object, not of the config inside it - EngineConfig
		// carries its own version and config_validate() rejects what it cannot
		// read, so a firmware config change does not need a bump here.
		json_object_set_new(root, "version", json_integer(1));

		json_t* slots = json_array();
		for (int s = 0; s < FRAM_CONFIG_SLOTS; s++)
		{
			if (store.occupied[s])
			{
				std::string b64 = string::toBase64((const uint8_t*) &store.slots[s], sizeof(EngineConfig));
				json_array_append_new(slots, json_string(b64.c_str()));
			}
			else
			{
				json_array_append_new(slots, json_null());
			}
		}
		json_object_set_new(root, "slots", slots);
		return root;
	}

	void dataFromJson(json_t* root) override
	{
		slot_store_clear(&store);

		json_t* slots = json_object_get(root, "slots");
		if (slots && json_is_array(slots))
		{
			for (int s = 0; s < FRAM_CONFIG_SLOTS && s < (int) json_array_size(slots); s++)
			{
				json_t* j = json_array_get(slots, s);
				if (!j || !json_is_string(j))
					continue;
				std::vector<uint8_t> bytes = string::fromBase64(json_string_value(j));
				// A blob of the wrong length is from another firmware version.
				// Dropping it lands that slot on the first-boot defaults, which is
				// what a module with an unreadable FRAM record does.
				if (bytes.size() != sizeof(EngineConfig))
					continue;
				std::memcpy(&store.slots[s], bytes.data(), sizeof(EngineConfig));
				store.occupied[s] = 1;
			}
		}

		// Same path as power-on, so a restored config goes through
		// config_validate() and channel_init() exactly as a stored one does.
		boot();
	}
};

struct BMCVWidget : ModuleWidget
{
	BMCVWidget(BMCV* module)
	{
		setModule(module);
		setPanel(createPanel(asset::plugin(pluginInstance, "res/BMCV.svg")));

		// The artwork over the backdrop, before anything else, because every
		// control on this panel sits in a cutout drawn on it.
		addChild(new PanelArt);

		// Rack's own screws, in the artwork's own slots. Every module in the
		// rack has them and one without reads as unfinished.
		for (int i = 0; i < 4; i++)
		{
			addChild(createWidgetCentered<ScrewSilver>(artPos(panel_mount_pos[i])));
		}

		// Rack's own jack, and Rack's own cable behaviour with it. The board's
		// jacks are on a 10mm pitch and a PJ301M is 8.03mm, so the part that
		// every other module in the rack uses is also the one that fits.
		for (int c = 0; c < N_CHANNELS; c++)
		{
			addOutput(createOutputCentered<PJ301MPort>(panelPos(panel_output_pos[c]), module, BMCV::CH_OUTPUT + c));
		}
		for (int i = 0; i < N_INPUTS; i++)
		{
			addInput(createInputCentered<PJ301MPort>(panelPos(panel_input_pos[i]), module, BMCV::CV_INPUT + i));
		}

		// Encoders. Each is three things at the same coordinate on the real
		// board - a rotation, a push button and the WS2812 behind it - and stays
		// three widgets here, in that order: the light spills from underneath,
		// the ring turns, and the cap on top takes the click.
		for (int e = 0; e < N_ENCODERS; e++)
		{
			Vec pos = panelPos(panel_encoder_pos[e]);
			int push = panel_encoder[e].push_button;

			addChild(createLightCentered<EncoderLight>(pos, module, BMCV::LED_LIGHT + panel_encoder[e].led * 3));

			EncoderRing* ring = createParamCentered<EncoderRing>(pos, module, BMCV::ENCODER_PARAM + e);
			ring->pushParamId = BMCV::BUTTON_PARAM + push;
			addParam(ring);

			EncoderCap* cap = createParamCentered<EncoderCap>(pos, module, BMCV::BUTTON_PARAM + push);
			cap->ring = ring; // so the wheel works over the middle of the knob too
			addParam(cap);
		}

		// Switches. The six ctrl buttons and the seven scenes are backlit, which
		// is what a VCVLightBezel is: the light fills the cap rather than sitting
		// beside it. The three centre-column tactiles have led == -1 on the real
		// board and are a TL1105 here, which is the part that is on it.
		for (int b = 0; b < N_BUTTONS; b++)
		{
			PanelButtonKind kind = panel_button_kind[b];
			if (kind == PANEL_BTN_ENCODER_PUSH)
				continue; // placed with its encoder above

			bool tactile = kind == PANEL_BTN_TACTILE;
			int led = panel_button_led[b];
			Vec pos = panelPos(panel_button_pos[b]);

			if (tactile)
			{
				addParam(createParamCentered<BmcvTactile>(pos, module, BMCV::BUTTON_PARAM + b));
			}
			else
			{
				addParam(createLightParamCentered<VCVLightBezel<SwitchLamp>>(
				    pos, module, BMCV::BUTTON_PARAM + b, BMCV::LED_LIGHT + led * 3));
			}

			// Above and below the cap, never on it: the cap is what lights, and a
			// 6mm one has no room for a word anyway.
			ButtonLegend* legend = new ButtonLegend;
			legend->param = panel_button_param[b];
			legend->mode = panel_button_mode[b];
			legend->capMm = tactile ? 4.2f : 3.6f;
			legend->box.size = Vec(panelMm(14.f), panelMm(14.f));
			legend->box.pos = pos.minus(legend->box.size.div(2));
			addChild(legend);
		}

		addParam(createParamCentered<BmcvCrossfader>(panelPos(panel_slider_pos), module, BMCV::SLIDER_PARAM));
	}

};

Model* modelBMCV = createModel<BMCV, BMCVWidget>("BMCV");
