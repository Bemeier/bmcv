# CHANGELOG


## PCB

### V1

- Fix pinout WS2812
- Use one interrupt per MCP
- Disconnect sdo from DAC-2 (not needed, ic not going into tri-state, so it was fighting ADC sdo).
- Weaker pulldowns plugs & small filter on ADC inputs to improve stability when floating.
- Small RC filters on DAC outputs.

### V0

Initial version