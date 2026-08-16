#ifndef INC_DRIVERS_DAC_ADC_HAL_H_
#define INC_DRIVERS_DAC_ADC_HAL_H_

#include "dac_adc.h"
#include "stm32g4xx_hal.h" // IWYU pragma: keep

typedef struct
{
  SPI_HandleTypeDef* spiHandle;

  // The DMA channels behind that SPI, reached directly.
  //
  // HAL configures them - direction, byte widths, the DMAMUX request routing -
  // and then HAL is not used to drive them, because driving them is what cost
  // the module its engine. Measured on hardware: HAL_SPI_TransmitReceive_DMA
  // plus SPI_DMATransmitReceiveCplt is ~18.5us per six-byte transaction against
  // 2.67us of actual wire time, and at the cadence the DAC wants that is most
  // of the CPU. Arming is a handful of register writes; the state machine
  // around it is the part that does not fit.
  //
  // Taken from spiHandle->hdmarx/hdmatx rather than named here, so this file
  // never learns which DMA channel the .ioc happened to assign.
  DMA_TypeDef* dma;
  DMA_Channel_TypeDef* rxch;
  DMA_Channel_TypeDef* txch;

  // DMA1->IFCR write that clears every flag for that channel. HAL's
  // ChannelIndex is already the four-bit shift for the channel's flag group.
  uint32_t rx_flags;
  uint32_t tx_flags;

  GPIO_TypeDef* csadcPortHandle;
  uint16_t csadcPin;

  GPIO_TypeDef* cnvstPortHandle;
  uint16_t cnvstPin;

  GPIO_TypeDef* addrPortHandle;
  uint16_t adrrPin;

  GPIO_TypeDef* csdacPortHandle;
  uint16_t csdacPin;

  uint8_t CH_IDX;
  uint8_t rx_buf[DAC_CHANNEL_DATA_WIDTH];
  uint8_t offset;

  uint8_t DAC_BUF[DAC_CHANNELS * DAC_CHANNEL_DATA_WIDTH];

  volatile int16_t adc_i[DAC_CHANNELS];
  int16_t adc_i_prev[DAC_CHANNELS];
  volatile int8_t trig_state[DAC_CHANNELS];
  volatile int8_t trig_flag[DAC_CHANNELS];
} DAC_ADC;

void dacadc_init(SPI_HandleTypeDef* spi);

// The DAC's transfer has finished. Called from the RX DMA channel's own
// interrupt vector rather than through HAL_DMA_IRQHandler and the SPI
// completion callback it dispatches to - that path is the ~10us half of the
// 18.5us above, most of it a FIFO drain this transfer does not need. RX
// completing *is* the frame being over: the last byte in was clocked by the
// same edge as the last byte out.
//
// Returns 0 if the interrupt was not ours, so the vector stays shareable.
uint8_t dacadc_dma_isr(void);

#endif /* INC_DRIVERS_DAC_ADC_HAL_H_ */
