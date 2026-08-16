#include "dac_adc_hal.h"
#include "main.h"
#include <stdint.h>

static DAC_ADC dacadc;

// Defined with the reasoning for them, below; both are used above their
// definitions.
static void dacadc_rx_flush(void);
static int16_t sign_extend_14bit(uint16_t val);

void dacadc_init(SPI_HandleTypeDef* spi)
{
  dacadc.spiHandle       = spi;
  dacadc.csadcPortHandle = OUT_ADC_CS_GPIO_Port;
  dacadc.csadcPin        = OUT_ADC_CS_Pin;
  dacadc.cnvstPortHandle = OUT_ADC_CNVST_GPIO_Port;
  dacadc.cnvstPin        = OUT_ADC_CNVST_Pin;
  dacadc.addrPortHandle  = OUT_ADC_ADDR_GPIO_Port;
  dacadc.adrrPin         = OUT_ADC_ADDR_Pin;
  dacadc.csdacPortHandle = OUT_DAC_SYNC_GPIO_Port;
  dacadc.csdacPin        = OUT_DAC_SYNC_Pin;

  // Set default pin levels
  HAL_GPIO_WritePin(dacadc.csadcPortHandle, dacadc.csadcPin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(dacadc.cnvstPortHandle, dacadc.cnvstPin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(dacadc.addrPortHandle, dacadc.adrrPin, GPIO_PIN_RESET);

  HAL_Delay(1); // Let things settle

  dac_init();
  // Optional: do a dummy read to check communication

  // Everything above this line still goes through HAL, and should: it runs once
  // at startup, where a state machine costs nothing and being obviously correct
  // is worth more than being quick. Past here the transfer is armed by hand.
  dacadc.dma  = dacadc.spiHandle->hdmarx->DmaBaseAddress;
  dacadc.rxch = dacadc.spiHandle->hdmarx->Instance;
  dacadc.txch = dacadc.spiHandle->hdmatx->Instance;

  // ChannelIndex is the shift HAL already worked out for this channel's group
  // of four flags; the low one of the four is the global flag.
  dacadc.rx_flags = DMA_IFCR_CGIF1 << dacadc.spiHandle->hdmarx->ChannelIndex;
  dacadc.tx_flags = DMA_IFCR_CGIF1 << dacadc.spiHandle->hdmatx->ChannelIndex;

  // Set once, so arming a transfer never has to. The peripheral end of both
  // channels is the same data register and never moves, and the widths and
  // increments HAL_DMA_Init put in CCR are already what this wants.
  dacadc.rxch->CPAR = (uint32_t) &dacadc.spiHandle->Instance->DR;
  dacadc.txch->CPAR = (uint32_t) &dacadc.spiHandle->Instance->DR;

  // The completion this driver acts on is RX, and the error is worth hearing
  // about; TX finishing early tells us nothing, so its interrupt stays off and
  // DMA1_Channel5 never fires.
  dacadc.rxch->CCR |= DMA_CCR_TCIE | DMA_CCR_TEIE;

  // Left on for the life of the module rather than raised per transfer. With
  // both DMA channels disabled the SPI has nothing feeding it and does not
  // clock, so an idle peripheral is just idle - and this is three register
  // read-modify-writes that would otherwise be on the 16kHz path.
  //
  // CR2 is OR'd, never written: HAL_SPI_Init put the FIFO reception threshold
  // in there for 8-bit frames, and clobbering it would make RXNE wait for two
  // bytes and every transaction hang on its last one.
  dacadc.spiHandle->Instance->CR2 |= SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;
  dacadc.spiHandle->Instance->CR1 |= SPI_CR1_SPE;
}

void dacadc_write(uint8_t idx, int16_t data)
{
  dacadc.DAC_BUF[idx * 3 + 1] = (data >> 8) & 0xFF;
  dacadc.DAC_BUF[idx * 3 + 2] = data & 0xFF;
}

// Drain whatever is sitting in the SPI's receive FIFO, and clear the overrun
// that leaving it there will have set.
//
// A receive DMA armed by hand has to start against an empty FIFO or it is not
// reading the frame it thinks it is, and **a desync repairs nothing by itself -
// it perpetuates**. Two stale bytes mean the channel takes those two plus four
// off the wire, reaches its count early, and leaves the frame's last two bytes
// behind for the next transfer to mistake for a header. Every frame after it is
// shifted by the same two bytes, for ever.
//
// dac_init() is what seeds it: it talks to the DACs with HAL_SPI_Transmit,
// which clocks bytes out of a two-line master and so clocks bytes *in* at the
// same time, and never reads the data register because a caller asking only to
// transmit has nowhere to put them. HAL's own transfer path resynchronised
// around that on every call, which is why this only appeared once HAL was taken
// out of the path.
//
// So it is called before arming, not once at startup. Flushing at startup fixes
// the seed and not the loop, which is exactly what was tried first and changed
// nothing. Measured on the module: SPI2->SR read 0x403 in steady state - RXNE
// set with FRLVL at two bytes - while dac_fps sat at a perfect 16129 and the
// output read flat, because adc_i was being decoded out of the DAC's echo
// bytes.
static void dacadc_rx_flush(void)
{
  SPI_TypeDef* spi = dacadc.spiHandle->Instance;

  // Byte-wide reads: with FRXTH set for 8-bit frames a 32-bit access to DR pops
  // up to four bytes at once, and the count this is draining against is in
  // bytes.
  //
  // Through uintptr_t rather than casting the address straight to uint8_t*,
  // which is the same access and warns: -Wstrict-aliasing sees a uint32_t
  // object read as a uint8_t. It is a register, not an object, and the width of
  // the access is the whole point of it.
  volatile uint8_t* dr = (volatile uint8_t*) (uintptr_t) &spi->DR;
  while (spi->SR & SPI_SR_FRLVL)
  {
    (void) *dr;
  }

  // OVR clears on a read of DR followed by a read of SR, and the loop above has
  // done the first half of that even when it ran zero times.
  (void) spi->DR;
  (void) spi->SR;
}

void dacadc_dma_next(void)
{
  dacadc.CH_IDX = (dacadc.CH_IDX + 1) % DAC_CHANNELS;
  dacadc.offset = (HAL_GPIO_ReadPin(dacadc.addrPortHandle, dacadc.adrrPin) == GPIO_PIN_SET) ? 2 : 0;
  HAL_GPIO_WritePin(dacadc.cnvstPortHandle, dacadc.cnvstPin, GPIO_PIN_RESET);
  HAL_GPIO_TogglePin(dacadc.addrPortHandle, dacadc.adrrPin);
  HAL_GPIO_WritePin(dacadc.cnvstPortHandle, dacadc.cnvstPin, GPIO_PIN_SET);

  HAL_GPIO_WritePin(dacadc.csadcPortHandle, dacadc.csadcPin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(dacadc.csdacPortHandle, dacadc.csdacPin, GPIO_PIN_RESET);

  // Both down before either count is written: CNDTR is writable only while its
  // channel is disabled, and the hardware ignores the write otherwise rather
  // than complaining about it.
  dacadc.rxch->CCR &= ~DMA_CCR_EN;
  dacadc.txch->CCR &= ~DMA_CCR_EN;

  // With both channels down, so nothing races the drain. This is what keeps
  // rx_buf[0] the first byte off the wire rather than the tail of the last
  // frame - see dacadc_rx_flush for why it belongs here and not in init.
  dacadc_rx_flush();

  dacadc.dma->IFCR = dacadc.rx_flags | dacadc.tx_flags;

  dacadc.rxch->CNDTR = DAC_CHANNEL_DATA_WIDTH;
  dacadc.rxch->CMAR  = (uint32_t) dacadc.rx_buf;
  dacadc.txch->CNDTR = DAC_CHANNEL_DATA_WIDTH;
  dacadc.txch->CMAR  = (uint32_t) &dacadc.DAC_BUF[dacadc.CH_IDX * DAC_CHANNEL_DATA_WIDTH];

  // Receive armed before transmit, always. The other order leaves a window
  // where the first byte has been clocked in with nowhere to put it, which is
  // an overrun that costs the whole frame's alignment - every ADC reading after
  // it belongs to the wrong channel.
  dacadc.rxch->CCR |= DMA_CCR_EN;
  dacadc.txch->CCR |= DMA_CCR_EN;
}

uint8_t dacadc_dma_isr(void)
{
  const uint32_t flags = dacadc.dma->ISR;

  // Neither ours nor anything we asked for. The vector is shared, so this has
  // to be answerable without guessing.
  if (!(flags & dacadc.rx_flags))
  {
    return 0;
  }

  dacadc.dma->IFCR = dacadc.rx_flags | dacadc.tx_flags;

  dacadc.rxch->CCR &= ~DMA_CCR_EN;
  dacadc.txch->CCR &= ~DMA_CCR_EN;

  // Raising SYNC is what latches the DAC, so it happens here and not a
  // microsecond later. Safe at this point because RX completing means the last
  // byte was clocked in, and the same edge clocked the last byte out - there is
  // nothing still on the wire to truncate.
  HAL_GPIO_WritePin(dacadc.csdacPortHandle, dacadc.csdacPin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(dacadc.csadcPortHandle, dacadc.csadcPin, GPIO_PIN_SET);

  // A transfer error means rx_buf holds some mixture of this frame and the
  // last, and - worse - that the FIFO is left holding however many bytes the
  // channel did not take, which would shift every frame after it. Flush before
  // reporting the completion; the timer arms a fresh one on its next tick, so
  // the recovery costs one frame and nothing downstream is told a wrong
  // voltage.
  if (flags & (DMA_ISR_TEIF1 << dacadc.spiHandle->hdmarx->ChannelIndex))
  {
    dacadc_rx_flush();
    return 1;
  }

  uint16_t adc_raw[2] = {0};
  adc_raw[0]          = ((dacadc.rx_buf[0] << 6) | (dacadc.rx_buf[1] >> 2)) & 0x3FFF;
  adc_raw[1]          = (((dacadc.rx_buf[1] & 0x03) << 12) | (dacadc.rx_buf[2] << 4) | (dacadc.rx_buf[3] >> 4)) & 0x3FFF;
  for (uint8_t ch = 0; ch < 2; ch++)
  {
    dacadc.adc_i[ch + dacadc.offset] = sign_extend_14bit(adc_raw[ch]);
    if (dacadc.trig_state[ch + dacadc.offset] < 1 && dacadc.adc_i_prev[ch + dacadc.offset] < TRIG_THRESH &&
        dacadc.adc_i[ch + dacadc.offset] >= TRIG_THRESH)
    {
      dacadc.trig_state[ch + dacadc.offset] = 1;
      dacadc.trig_flag[ch + dacadc.offset]  = 1;
    }
    else if (dacadc.adc_i[ch + dacadc.offset] < TRIG_THRESH_LOW)
    {
      dacadc.trig_state[ch + dacadc.offset] = 0;
    }
    dacadc.adc_i_prev[ch + dacadc.offset] = dacadc.adc_i[ch + dacadc.offset];
  }

  return 1;
}

uint8_t adc_read_trig_state(uint8_t channel)
{
  if (dacadc.trig_flag[channel])
  {
    dacadc.trig_flag[channel] = 0;
    return 1;
  }
  return 0;
}

void dac_init()
{
  dacadc.DAC_BUF[0]  = (0b00000000); // DAC1 CHA
  dacadc.DAC_BUF[3]  = (0b00000000); // DAC2 CHA
  dacadc.DAC_BUF[6]  = (0b00000001); // DAC1 CHB
  dacadc.DAC_BUF[9]  = (0b00000001); // DAC2 CHB
  dacadc.DAC_BUF[12] = (0b00000010); // DAC1 CHC
  dacadc.DAC_BUF[15] = (0b00000010); // DAC2 CHC
  dacadc.DAC_BUF[18] = (0b00000011); // DAC1 CHD
  dacadc.DAC_BUF[21] = (0b00000011); // DAC2 CHD

  HAL_GPIO_WritePin(dacadc.csdacPortHandle, dacadc.csdacPin, GPIO_PIN_SET);
  // control_bits |= (range_code & 0x07);  // Bits 2:0 = range select
  //  All other control bits = 0 (normal operation, Slew Rate Off, etc.)

  HAL_Delay(1);

  uint16_t control_bits = 0b0000000000000100;

  uint8_t tx_buf[6];
  tx_buf[0] = (0b00001100);
  tx_buf[1] = (control_bits >> 8) & 0xFF;
  tx_buf[2] = control_bits & 0xFF;
  tx_buf[3] = (0b00001100);
  tx_buf[4] = (control_bits >> 8) & 0xFF;
  tx_buf[5] = control_bits & 0xFF;

  HAL_GPIO_WritePin(dacadc.csdacPortHandle, dacadc.csdacPin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(dacadc.spiHandle, tx_buf, 6, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(dacadc.csdacPortHandle, dacadc.csdacPin, GPIO_PIN_SET);

  HAL_Delay(1);

  tx_buf[0] = (0b00010000);
  tx_buf[1] = 0xFF;
  tx_buf[2] = 0xFF;
  tx_buf[3] = (0b00010000);
  tx_buf[4] = 0xFF;
  tx_buf[5] = 0xFF;

  HAL_GPIO_WritePin(dacadc.csdacPortHandle, dacadc.csdacPin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(dacadc.spiHandle, tx_buf, 6, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(dacadc.csdacPortHandle, dacadc.csdacPin, GPIO_PIN_SET);

  HAL_Delay(1);

  // HAL_GPIO_WritePin(dacadc.ldacPortHandle, dacadc.ldacPin, GPIO_PIN_RESET);
}

static int16_t sign_extend_14bit(uint16_t val) { return (int16_t) ((int32_t) (val << 18) >> 18); }

int16_t get_adc(uint8_t channel) { return dacadc.adc_i[channel]; }
