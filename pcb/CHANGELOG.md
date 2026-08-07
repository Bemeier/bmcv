# CHANGELOG


## PCB

### V1

- Fix pinout WS2812
- WS2812 data uses a dedicated levelshifter
- Use one interrupt per MCP
- Disconnect sdo from DAC-2 (not needed, ic not going into tri-state, so it was fighting ADC sdo).
- Weaker pulldowns on and small RC filter on ADC inputs to improve stability when floating.
- Small RC filters on DAC outputs.
- Updated TC002 footprint slightly to also be potentially compatible with 228C series LED switches
- Don't control state of DAC_LDAC & DAC_CLR (tied high, ldac only via software, not using CLR)
    - PA8 now menu button 2 (which was previously reset)
    - PA9 now free & broken out
- Switched to differend (smoother) sliding potentiometer
- Added third function button
  - using PB6 (which was previously SLIDER_LED)

### V0

    Initial version