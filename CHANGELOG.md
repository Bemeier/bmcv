# CHANGELOG


## PCB

### V1

- Fix pinout WS2812
- WS2812 data uses a dedicated levelshifter
- Use one interrupt per MCP
- Disconnect sdo from DAC-2 (not needed, ic not going into tri-state, so it was fighting ADC sdo).
- Weaker pulldowns plugs & small filter on ADC inputs to improve stability when floating.
- Small RC filters on DAC outputs.
- Updated TC002 footprint slightly to also be potentially compatible with 228C series LED switches
- Switch DAC_SYNC pin to PA9
- PA8 now menu button 2 (which was previously reset)
- PA10 free
- TODO: Free up Level Shifter EN?

### V0

    Initial version