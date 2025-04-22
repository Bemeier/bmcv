/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include "dualmpc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_midi.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define FRAM_WRITE   0x02
#define FRAM_READ    0x03
#define FRAM_WREN    0x06


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// FRAM

#define FRAM_CS_LOW()    HAL_GPIO_WritePin(SPI3_FRAM_CS_GPIO_Port, SPI3_FRAM_CS_Pin, GPIO_PIN_RESET)
#define FRAM_CS_HIGH()    HAL_GPIO_WritePin(SPI3_FRAM_CS_GPIO_Port, SPI3_FRAM_CS_Pin, GPIO_PIN_SET)

// MIDI
extern USBD_HandleTypeDef hUsbDeviceFS;
static uint8_t buffUsbReport[MIDI_EPIN_SIZE] = {0};
static uint8_t buffUsbReportNextIndex = 0;

void MIDI_addToUSBReport(uint8_t cable, uint8_t message, uint8_t param1, uint8_t param2)
{
  buffUsbReport[buffUsbReportNextIndex++] = (cable << 4) | (message >> 4);
  buffUsbReport[buffUsbReportNextIndex++] = (message);
  buffUsbReport[buffUsbReportNextIndex++] = (param1);
  buffUsbReport[buffUsbReportNextIndex++] = (param2);

  if (buffUsbReportNextIndex == MIDI_EPIN_SIZE)
  {
    while (USBD_MIDI_GetState(&hUsbDeviceFS) != MIDI_IDLE) {};
    USBD_MIDI_SendReport(&hUsbDeviceFS, buffUsbReport, MIDI_EPIN_SIZE);
    buffUsbReportNextIndex = 0;
  }
}

// DAC

#define DAC_CS_LOW()     HAL_GPIO_WritePin(GPIOA, OUT_DAC_SYNC_Pin, GPIO_PIN_RESET)
#define DAC_CS_HIGH()    HAL_GPIO_WritePin(GPIOA, OUT_DAC_SYNC_Pin, GPIO_PIN_SET)

#define DAC_LDAC_LOW()     HAL_GPIO_WritePin(GPIOA, OUT_DAC_LDAC_Pin, GPIO_PIN_RESET)
#define DAC_LDAC_HIGH()    HAL_GPIO_WritePin(GPIOA, OUT_DAC_LDAC_Pin, GPIO_PIN_SET)

#define DAC_CLR_LOW()      HAL_GPIO_WritePin(GPIOA, OUT_DAC_CLR_Pin, GPIO_PIN_RESET)
#define DAC_CLR_HIGH()     HAL_GPIO_WritePin(GPIOA, OUT_DAC_CLR_Pin, GPIO_PIN_SET)


#define ADC_CS_LOW() 		HAL_GPIO_WritePin(OUT_ADC_CS_GPIO_Port, OUT_ADC_CS_Pin, GPIO_PIN_RESET);
#define ADC_CS_HIGH() 		HAL_GPIO_WritePin(OUT_ADC_CS_GPIO_Port, OUT_ADC_CS_Pin, GPIO_PIN_SET);


// MCP
DUALMPC mpc;

uint16_t ADC_HALF_RANGE = 4096;


// WS2812
#define LED_COUNT 21
#define WS2811_BITS 24
#define RST_PERIODS 64
#define WS2811_BUF_LEN ((WS2811_BITS * LED_COUNT) + RST_PERIODS)



// Over 72
#define WS2811_1 38        //
#define WS2811_0 14        //

typedef union {
	struct {
		uint8_t r;
		uint8_t g;
		uint8_t b;
	} color;

	uint32_t data;
} WS8211_LED_DATA;

WS8211_LED_DATA ws2811_rgb_data[LED_COUNT];
uint32_t ws2811_pwm_data[WS2811_BUF_LEN];
volatile uint8_t WS_DATA_COMPLETE_FLAG;

void set_led_hsv(uint8_t h, uint8_t s, uint8_t v, WS8211_LED_DATA* led) {
    uint8_t region, remainder, p, q, t;

    if (s == 0) {
        led->color.r = v;
        led->color.g = v;
        led->color.b = v;
        return;
    }

    region = h / 43;
    remainder = (h - region * 43) * 6;

    p = (v * (255 - s)) >> 8;
    q = (v * (255 - ((s * remainder) >> 8))) >> 8;
    t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;

    switch (region) {
        case 0:
            led->color.r = v; led->color.g = t; led->color.b = p; break;
        case 1:
            led->color.r = q; led->color.g = v; led->color.b = p; break;
        case 2:
            led->color.r = p; led->color.g = v; led->color.b = t; break;
        case 3:
            led->color.r = p; led->color.g = q; led->color.b = v; break;
        case 4:
            led->color.r = t; led->color.g = p; led->color.b = v; break;
        default:
            led->color.r = v; led->color.g = p; led->color.b = q; break;
    }
}


void set_led_adc_range(int16_t val, WS8211_LED_DATA* led) {
	// int16_t only safe because we know ADC values are only 14 bits, so they won't overflow here.
	int16_t abs_val = abs(val);
	int16_t blue_range = abs_val - ADC_HALF_RANGE;
	uint8_t base_val = 255;
	if (blue_range < 0) {
		blue_range = 0;
		base_val = (abs_val / 16) & 0xFF;
	}
	led->color.b = ((blue_range / 16) & 0xFF);
	if (val > 0) {
		led->color.g = base_val;
		led->color.r =  0;
	} else {
		led->color.r = base_val;
		led->color.b = 0;
	}
}

void ws2811_init() {
	for (uint16_t bufidx = 0; bufidx < WS2811_BUF_LEN; bufidx++) {
		ws2811_pwm_data[bufidx] = 0;
	}
	WS_DATA_COMPLETE_FLAG = 1;
}

void ws2811_setled(uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
	ws2811_rgb_data[index].color.r = r;
	ws2811_rgb_data[index].color.g = g;
	ws2811_rgb_data[index].color.b = b;
}

void ws2811_commit() {
	uint16_t bufidx = 0;

	for (uint8_t led = 0; led < LED_COUNT; led++) {
		for (uint8_t bits = 0; bits < WS2811_BITS; bits++, bufidx++) {
			uint8_t byte = (bits / 8) * 8;
			uint8_t bit = 7 - (bits % 8);
			uint8_t bitIndex = byte + bit;

			if ((ws2811_rgb_data[led].data >> bitIndex) & 0x01)
				ws2811_pwm_data[bufidx] = WS2811_1;
			else
				ws2811_pwm_data[bufidx] = WS2811_0;
		}
	}
}

volatile int16_t adc_i[4] = {0};
volatile float adc_f[4] = {0};
volatile uint8_t addr = 0;
volatile uint8_t converting = 0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim3_ch4;

/* USER CODE BEGIN PV */
int slider = 0;
int state = 0;
uint32_t now = 0, next_blink = 500, next_tick = 1000, loop_count = 0;
HAL_StatusTypeDef result;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int16_t sign_extend_14bit(uint16_t val) {
    return (int16_t)((int32_t)(val << 18) >> 18);
}

float adc_to_voltage(int16_t adc_value) {
    return ((float)adc_value / 8192.0f) * 10.0f;  // Assuming full scale ±10V
}


void ADC_Init(void)
{
    // Set default pin levels
    HAL_GPIO_WritePin(OUT_ADC_CS_GPIO_Port, OUT_ADC_CS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(OUT_ADC_CNVST_GPIO_Port, OUT_ADC_CNVST_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(OUT_ADC_ADDR_GPIO_Port, OUT_ADC_ADDR_Pin, GPIO_PIN_RESET);

    HAL_Delay(1); // Let things settle

    // Optional: do a dummy read to check communication
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	if(GPIO_Pin == INT_MCP_ENC_Pin || GPIO_Pin == 0) {
		ReadEncoders(&mpc);
	}

}

//uint8_t CH_IDX = 0;
uint8_t DAC_BUF[24] = { 0 };
uint16_t DAC_DATA[8] = { 0 };


static inline void WRITE_DAC_VALUE(int idx, int16_t data) {
	DAC_BUF[idx * 3 + 1] = (data >> 8) & 0xFF;
	DAC_BUF[idx * 3 + 2] = data & 0xFF;
}

void ADC_DAC_Transaction() {
    uint8_t rx_buf[6] = {0};
    uint16_t adc_a_raw = 0;
    uint16_t adc_b_raw = 0;

    for (uint8_t CH_IDX = 0; CH_IDX < 4; CH_IDX++) {
		uint8_t offset = (HAL_GPIO_ReadPin(OUT_ADC_ADDR_GPIO_Port, OUT_ADC_ADDR_Pin) == GPIO_PIN_SET) ? 2 : 0;
		HAL_GPIO_WritePin(OUT_ADC_CNVST_GPIO_Port, OUT_ADC_CNVST_Pin, GPIO_PIN_RESET);
		HAL_GPIO_TogglePin(OUT_ADC_ADDR_GPIO_Port, OUT_ADC_ADDR_Pin);
		HAL_GPIO_WritePin(OUT_ADC_CNVST_GPIO_Port, OUT_ADC_CNVST_Pin, GPIO_PIN_SET);

		// TRANSMIT 1 channel to both DACs
		ADC_CS_LOW();
		DAC_CS_LOW();
		HAL_SPI_TransmitReceive(&hspi2, &DAC_BUF[CH_IDX*6], rx_buf, 6, HAL_MAX_DELAY); //
		DAC_CS_HIGH();
		ADC_CS_HIGH();
		adc_a_raw = ((rx_buf[0] << 6) | (rx_buf[1] >> 2)) & 0x3FFF;
		adc_b_raw = (((rx_buf[1] & 0x03) << 12) | (rx_buf[2] << 4) | (rx_buf[3] >> 4)) & 0x3FFF;
		adc_i[0+offset] = sign_extend_14bit(adc_a_raw);
		adc_i[1+offset] = sign_extend_14bit(adc_b_raw);
		adc_f[0+offset] = adc_to_voltage(adc_i[0+offset]);
		adc_f[1+offset] = adc_to_voltage(adc_i[1+offset]);
    }
}

void FRAM_WriteEnable() {
    uint8_t tx[1] = {
    	FRAM_WREN
    };

    FRAM_CS_LOW();
    HAL_SPI_Transmit(&hspi3, tx, 1, HAL_MAX_DELAY);
    FRAM_CS_HIGH();
}


void FRAM_WriteByte(uint16_t addr, uint8_t data) {
    uint8_t tx[4] = {
        FRAM_WRITE,
        (addr >> 8) & 0xFF,
        addr & 0xFF,
        data
    };

    FRAM_CS_LOW();
    HAL_SPI_Transmit(&hspi3, tx, 4, HAL_MAX_DELAY);
    FRAM_CS_HIGH();
}

uint8_t FRAM_ReadByte(uint16_t addr) {
    uint8_t tx[4] = {
        FRAM_READ,
        (addr >> 8) & 0xFF,
        addr & 0xFF,
		0
    };
    uint8_t rx[4] = { 0 };


    FRAM_CS_LOW();
    HAL_SPI_TransmitReceive(&hspi3, tx, rx, 4, HAL_MAX_DELAY);
    FRAM_CS_HIGH();

    return rx[3];
}

#define SINE_STEPS     128       // Number of points per wave cycle
#define SINE_AMPLITUDE 31000     // Max int16_t value for DAC full scale
#define CHANNEL        0         // DAC Channel A

float sine_table[SINE_STEPS];

void DAC_Init(void)
{
	DAC_BUF[ 0] = (0b00000000); // DAC1 CHA
	DAC_BUF[ 3] = (0b00000000); // DAC2 CHA
	DAC_BUF[ 6] = (0b00000001); // DAC1 CHB
	DAC_BUF[ 9] = (0b00000001); // DAC2 CHB
	DAC_BUF[12] = (0b00000010); // DAC1 CHC
	DAC_BUF[15] = (0b00000010); // DAC2 CHC
	DAC_BUF[18] = (0b00000011); // DAC1 CHD
	DAC_BUF[21] = (0b00000011); // DAC2 CHD

    // Step 1: Set all GPIOs to default state
    DAC_CS_HIGH();
    DAC_LDAC_HIGH();
    DAC_CLR_LOW();
    HAL_Delay(1);
    DAC_CLR_HIGH();

    //control_bits |= (range_code & 0x07);  // Bits 2:0 = range select
    // All other control bits = 0 (normal operation, Slew Rate Off, etc.)

    HAL_Delay(1); // Short delay to let power stabilize if needed

    uint16_t control_bits = 0b0000000000000100;

    uint8_t tx_buf[6];
    tx_buf[0] = (0b00001100);
    tx_buf[1] = (control_bits >> 8) & 0xFF;
    tx_buf[2] =  control_bits & 0xFF;
    tx_buf[3] = (0b00001100);
    tx_buf[4] = (control_bits >> 8) & 0xFF;
    tx_buf[5] =  control_bits & 0xFF;


    HAL_GPIO_WritePin(OUT_DAC_SYNC_GPIO_Port, OUT_DAC_SYNC_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, tx_buf, 6, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(OUT_DAC_SYNC_GPIO_Port, OUT_DAC_SYNC_Pin, GPIO_PIN_SET);

    HAL_Delay(1); // Short delay to let power stabilize if needed

    tx_buf[0] = (0b00010000);
    tx_buf[1] = 0xFF;
    tx_buf[2] = 0xFF;
    tx_buf[3] = (0b00010000);
    tx_buf[4] = 0xFF;
    tx_buf[5] = 0xFF;

    HAL_GPIO_WritePin(OUT_DAC_SYNC_GPIO_Port, OUT_DAC_SYNC_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, tx_buf, 6, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(OUT_DAC_SYNC_GPIO_Port, OUT_DAC_SYNC_Pin, GPIO_PIN_SET);
    HAL_Delay(1); // Short delay to let power stabilize if needed

    DAC_LDAC_LOW();

    for (int i = 0; i < SINE_STEPS; i++)
    {
        sine_table[i] = SINE_AMPLITUDE * sinf(2 * M_PI * i / SINE_STEPS);
    }

}
uint8_t val1;
uint8_t test_val;
uint8_t read_val;

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
    {
    	WS_DATA_COMPLETE_FLAG = 1;
    }
}

volatile float adc_freq_hz = 0;           // Smoothed ADC frequency estimate
volatile uint32_t last_sample_time = 0;   // Last tick (ms) of ADC sample
const float filter_alpha = 0.1f;          // Low-pass filter coefficient

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USB_Device_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(LEVEL_SHIFTER_EN_GPIO_Port, LEVEL_SHIFTER_EN_Pin, SET);
  HAL_Delay(20);

  ws2811_init();

  // Should move to init function:
  mpc.spiHandle = &hspi1;
  mpc.csPortHandle = OUT_MCP_CS_GPIO_Port;
  mpc.csPin = OUT_MCP_CS_Pin;
  mpc.resetPortHandle = OUT_MCP_RESET_GPIO_Port;
  mpc.resetPin = OUT_MCP_RESET_Pin;

  MCP23S17_Init(&mpc);

  ADC_Init();

  DAC_Init();



  for (uint8_t ledidx = 0; ledidx < LED_COUNT; ledidx++) {
	  ws2811_setled(ledidx, 255, 0, 0);
  }
  ws2811_commit();

  /*
  result = HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  if (result != HAL_OK) {
	  ws2811_encode(0xFF0000);
  }
  */

  uint16_t sine_index = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t time_ms;
  float freq = 0.2f;
  HAL_ADC_Start_IT(&hadc1);
  HAL_TIM_Base_Start_IT(&htim2);

  while (1)
  {
	time_ms = HAL_GetTick();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    //HAL_GPIO_WritePin(SLIDER_LED_GPIO_Port, SLIDER_LED_Pin, SET);
	//HAL_GPIO_TogglePin(SLIDER_LED_GPIO_Port, SLIDER_LED_Pin);
	//HAL_Delay(200);
	//HAL_GPIO_TogglePin(SLIDER_LED_GPIO_Port, SLIDER_LED_Pin);

	/*
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	slider = HAL_ADC_GetValue(&hadc1);
	*/

	test_val = 0x5A;
	FRAM_WriteEnable();
	FRAM_WriteByte(0x0010, test_val);
	read_val = FRAM_ReadByte(0x0010);

	/*
	if (read_val != test_val) {
		HAL_GPIO_TogglePin(SLIDER_LED_GPIO_Port, SLIDER_LED_Pin);
		HAL_Delay(1000);
	}
	*/


    sine_index = (sine_index + 1) % SINE_STEPS;
    for (uint8_t i = 0; i < 8; i++) {
    	int16_t val = sine_table[(sine_index + i * 16) % SINE_STEPS];
    	if (i % 2 == 0 && val > 0) {
    		val = SINE_AMPLITUDE;
    	} else if (i % 2 == 0 && val <= 0) {
    		val = -SINE_AMPLITUDE;
    	}
    	WRITE_DAC_VALUE(i, val);
    }


    ADC_DAC_Transaction();


	/*

	if (USBD_MIDI_GetState(&hUsbDeviceFS) == MIDI_IDLE) {
		uint8_t midi_value0 = (uint8_t)(fminf(fmaxf(adc_f[0], 0.0f), 5.0f) * (127.0f / 5.0f));
		uint8_t midi_value1 = (uint8_t)(fminf(fmaxf(adc_f[1], 0.0f), 5.0f) * (127.0f / 5.0f));
		uint8_t midi_value2 = (uint8_t)(fminf(fmaxf(adc_f[2], 0.0f), 5.0f) * (127.0f / 5.0f));
		uint8_t midi_value3 = (uint8_t)(fminf(fmaxf(adc_f[3], 0.0f), 5.0f) * (127.0f / 5.0f));
		MIDI_addToUSBReport(0, 0xB0, 0x10, midi_value0);
		MIDI_addToUSBReport(0, 0xB0, 0x11, midi_value1);
		MIDI_addToUSBReport(0, 0xB0, 0x12, midi_value2);
		MIDI_addToUSBReport(0, 0xB0, 0x13, midi_value3);
	    USBD_MIDI_SendReport(&hUsbDeviceFS, buffUsbReport, MIDI_EPIN_SIZE);
	    buffUsbReportNextIndex = 0;
	}
	*/


	/*
*/
	if (WS_DATA_COMPLETE_FLAG == 1) {
		WS_DATA_COMPLETE_FLAG = 0;
		float hue = sinf(2.0f * M_PI * freq * time_ms / 1000.0f) * 127.5f + 127.5f;
	    //set_led_hsv(hue, 255, 255, &ws2811_rgb_data[0]);
		//set_led_adc_range(adc_i[0], &ws2811_rgb_data[0]);
		set_led_adc_range(slider, &ws2811_rgb_data[0]);
	    ws2811_commit();

		result = HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_4, ws2811_pwm_data, WS2811_BUF_LEN);
		if (result != HAL_OK) {
			HAL_GPIO_TogglePin(SLIDER_LED_GPIO_Port, SLIDER_LED_Pin);
			HAL_Delay(100);
		}
	}
	/*
	*/

    /*
	HAL_Delay(500);
	HAL_GPIO_TogglePin(SLIDER_LED_GPIO_Port, SLIDER_LED_Pin);

	now = uwTick;
	*/

	/*
	int newState = HAL_GPIO_ReadPin (GPIOC, IN_BTN_MCU2_Pin);
	if (newState != state) {
		HAL_GPIO_TogglePin(SLIDER_LED_GPIO_Port, SLIDER_LED_Pin);
		state = newState;
	}

		HAL_GPIO_TogglePin(SLIDER_LED_GPIO_Port, SLIDER_LED_Pin);
		HAL_Delay(100+(slider/5));
		HAL_GPIO_TogglePin(SLIDER_LED_GPIO_Port, SLIDER_LED_Pin);
		HAL_Delay(200);

	if (!(now % 10)) { // Just call every 10th loop
		ws2812_demos_tick(&ws2812);
	}

	if (now >= next_blink) { // Every 500 ms
		HAL_GPIO_TogglePin(SLIDER_LED_GPIO_Port, SLIDER_LED_Pin);
		next_blink = now + 500;
	}

	if (now >= next_tick) {
		loop_count = 0;
		next_tick = now + 1000;
	}
	++loop_count;
	*/
  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 30;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV6;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV256;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_32;
  hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_4;
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 14400-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 2;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 72;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  __HAL_LINKDMA(&htim3, hdma[TIM_DMA_ID_CC4], hdma_tim3_ch4);
  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 143;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMAMUX_OVR_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMAMUX_OVR_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMAMUX_OVR_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, OUT_MCP_CS_Pin|SPI3_FRAM_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OUT_MCP_RESET_Pin|OUT_ADC_ADDR_Pin|SLIDER_LED_Pin|LEVEL_SHIFTER_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OUT_ADC_CS_Pin|OUT_ADC_CNVST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, OUT_DAC_CLR_Pin|OUT_DAC_LDAC_Pin|OUT_DAC_SYNC_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : IN_BTN_MCU2_Pin IN_BTN_MCU5_Pin IN_BTN_MCU4_Pin */
  GPIO_InitStruct.Pin = IN_BTN_MCU2_Pin|IN_BTN_MCU5_Pin|IN_BTN_MCU4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT_SW_Pin */
  GPIO_InitStruct.Pin = BOOT_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IN_BTN_MCU3_Pin */
  GPIO_InitStruct.Pin = IN_BTN_MCU3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IN_BTN_MCU3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INT_MCP_BTN_Pin INT_MCP_ENC_Pin */
  GPIO_InitStruct.Pin = INT_MCP_BTN_Pin|INT_MCP_ENC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT_MCP_CS_Pin OUT_DAC_CLR_Pin OUT_DAC_LDAC_Pin OUT_DAC_SYNC_Pin
                           SPI3_FRAM_CS_Pin */
  GPIO_InitStruct.Pin = OUT_MCP_CS_Pin|OUT_DAC_CLR_Pin|OUT_DAC_LDAC_Pin|OUT_DAC_SYNC_Pin
                          |SPI3_FRAM_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT_MCP_RESET_Pin OUT_ADC_CS_Pin OUT_ADC_CNVST_Pin OUT_ADC_ADDR_Pin */
  GPIO_InitStruct.Pin = OUT_MCP_RESET_Pin|OUT_ADC_CS_Pin|OUT_ADC_CNVST_Pin|OUT_ADC_ADDR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : INT_ADC_BUSY_Pin */
  GPIO_InitStruct.Pin = INT_ADC_BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INT_ADC_BUSY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SLIDER_LED_Pin LEVEL_SHIFTER_EN_Pin */
  GPIO_InitStruct.Pin = SLIDER_LED_Pin|LEVEL_SHIFTER_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RESET_SW_Pin IN_BTN_MCU1_Pin */
  GPIO_InitStruct.Pin = RESET_SW_Pin|IN_BTN_MCU1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

volatile uint8_t mcp_busy = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) { // ~ 100 Hz
    	ReadButtons(&mpc);

    	ReadEncoders(&mpc);

        uint32_t now = HAL_GetTick();  // Time in ms
        uint32_t dt = now - last_sample_time;

        if (dt > 0) {
            float current_freq = 1000.0f / dt;  // Convert ms to Hz
            adc_freq_hz = (1.0f - filter_alpha) * adc_freq_hz + filter_alpha * current_freq;
        }

        last_sample_time = now;
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {


        slider = HAL_ADC_GetValue(hadc);
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
