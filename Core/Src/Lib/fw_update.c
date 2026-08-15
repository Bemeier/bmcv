#include "fw_update.h"

#include "hw_setup.h" // IWYU pragma: keep - LED_COUNT
#include "main.h"
#include "stm32g4xx_hal.h"
#include "usbd_core.h"
#include "ws2811.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

// STM32G4 system memory, per AN2606. The bootloader there is mask ROM: it
// cannot be erased, which is the whole reason this approach needs no bootloader
// of our own and has no way to brick the module.
#define BOOTLOADER_ADDR 0x1FFF0000UL

// What the panel shows once the ROM bootloader has the module. Every LED the
// same colour, which never happens in normal operation, so it reads as "not
// running" rather than as some mode you have not seen before.
//
// Dim on purpose: 21 LEDs at anything like full brightness is a lot of current
// to draw off the case for what may be several minutes of sitting idle.
#define DFU_LED_R 40
#define DFU_LED_G 10
#define DFU_LED_B 0

// How long to wait for the LED DMA to land, in ms. One frame of 21 LEDs is
// well under a millisecond; this only has to be long enough that a frame
// already in flight finishes.
#define DFU_LED_TIMEOUT_MS 50

// How long to stay off the bus before resetting, in ms. The reset would detach
// us anyway, but doing it explicitly while the USB stack is still ours means
// the host sees a clean disconnect rather than a device that vanished.
#define DFU_DETACH_MS 50

// The note one boot leaves for the next. Two words rather than one, the second
// the complement of the first, so that uninitialised SRAM cannot spell it by
// accident - this decides whether the module comes up as itself or disappears
// into a bootloader, and it is read before anything has had a chance to be
// initialised.
#define FW_UPDATE_MAGIC 0x0DF00DF0UL

__attribute__((section(".noinit"))) static volatile uint32_t fw_update_marker;
__attribute__((section(".noinit"))) static volatile uint32_t fw_update_marker_inv;

typedef void (*BootloaderEntry)(void);

// The bootloader's reset handler, read out before the jump and deliberately
// *not* a local.
//
// Setting MSP moves the stack out from under the caller: everything in the
// current frame is gone, and reading a spilled local afterwards reads whatever
// happens to sit near the bootloader's stack top instead. Jumping through that
// wedges the CPU - which looks exactly like a successful jump from the panel,
// because the LEDs latch and keep showing the colour set before it.
static BootloaderEntry bootloader_entry;

// Block until no LED frame is in flight, or the timeout runs out.
//
// The subtraction rather than a computed deadline is what keeps this right
// across the tick counter's 49-day wrap: unsigned arithmetic gives the true
// elapsed time either side of it, where `now < start + timeout` would fall
// straight through.
static void fw_update_wait_led_idle(void)
{
  const uint32_t start = HAL_GetTick();
  while (!ws2811_dma_completed() && (HAL_GetTick() - start) < DFU_LED_TIMEOUT_MS)
  {
  }
}

// Park the LED data line low before handing over to the bootloader.
//
// Reset left PB1 a floating input, the timer that would normally drive it does
// not exist on this path, and the ROM bootloader never touches the pin - so
// without this it floats for the whole DFU session behind a level shifter that
// is permanently enabled. The WS2811s hold their last latched colour either way
// since they only update on a valid frame, but a driven line means nothing
// downstream can be mistaken for one.
//
// Register writes only: this runs before HAL_Init, and must not depend on it.
static void fw_update_park_led_line(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // Low first, so the pin never glitches high as it becomes an output.
  HAL_GPIO_WritePin(WS_TIM3_CH4_GPIO_Port, WS_TIM3_CH4_Pin, GPIO_PIN_RESET);

  GPIO_InitTypeDef gpio = {0};
  gpio.Pin              = WS_TIM3_CH4_Pin;
  gpio.Mode             = GPIO_MODE_OUTPUT_PP;
  gpio.Pull             = GPIO_NOPULL;
  gpio.Speed            = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(WS_TIM3_CH4_GPIO_Port, &gpio);
}

void fw_update_enter_dfu(void)
{
  // Light the panel and let the DMA finish. The WS2811s latch and are powered
  // from +5V rather than by the MCU, so this colour survives the reset and is
  // what the panel shows for the whole DFU session - the only feedback there
  // is, because the ROM bootloader does not know this panel exists. Nothing can
  // show the CPY route into DFU; that one leaves the panel dark.
  //
  // The wait before the update matters as much as the one after it:
  // ws2811_update() returns without doing anything while a frame is still in
  // flight, which would leave the panel showing whatever the UX drew last.
  fw_update_wait_led_idle();
  for (uint16_t i = 0; i < LED_COUNT; i++)
  {
    ws2811_setled_rgb(i, DFU_LED_R, DFU_LED_G, DFU_LED_B);
  }
  ws2811_update();
  fw_update_wait_led_idle();

  // Leave the bus deliberately rather than just vanishing when the reset hits,
  // and give the host long enough to notice. USBD_DeInit stops the device on
  // its way through, so there is no need to stop it separately first.
  USBD_DeInit(&hUsbDeviceFS);
  HAL_Delay(DFU_DETACH_MS);

  fw_update_marker     = FW_UPDATE_MAGIC;
  fw_update_marker_inv = ~FW_UPDATE_MAGIC;

  NVIC_SystemReset(); // __NO_RETURN
}

void fw_update_check_boot(void)
{
  if (fw_update_marker != FW_UPDATE_MAGIC || fw_update_marker_inv != ~FW_UPDATE_MAGIC)
  {
    return;
  }

  // Clear it first. If anything below goes wrong, the next power cycle has to
  // come up as the firmware rather than back into this same dead end.
  fw_update_marker     = 0;
  fw_update_marker_inv = 0;

  fw_update_park_led_line();

  // Map system memory at zero. SCB->VTOR is still at its reset value here -
  // nothing has run that could have moved it - so this is also what points the
  // vector table at the bootloader's own.
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();

  bootloader_entry = (BootloaderEntry) (*(volatile uint32_t*) (BOOTLOADER_ADDR + 4));

  // Last thing that touches the stack. `bootloader_entry` is read from static
  // storage afterwards, which is the whole point of it being static.
  __set_MSP(*(volatile uint32_t*) BOOTLOADER_ADDR);
  bootloader_entry();

  // Not reached. If the jump ever did fall through, spinning here is better
  // than running on with the stack pointer pointing into the bootloader's RAM.
  while (1)
  {
  }
}
