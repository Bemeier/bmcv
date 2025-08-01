/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h" // IWYU pragma: keep

    /* Private includes ----------------------------------------------------------*/
    /* USER CODE BEGIN Includes */

    /* USER CODE END Includes */

    /* Exported types ------------------------------------------------------------*/
    /* USER CODE BEGIN ET */

    /* USER CODE END ET */

    /* Exported constants --------------------------------------------------------*/
    /* USER CODE BEGIN EC */

    /* USER CODE END EC */

    /* Exported macro ------------------------------------------------------------*/
    /* USER CODE BEGIN EM */

    /* USER CODE END EM */

    void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

    /* Exported functions prototypes ---------------------------------------------*/
    void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IN_BTN_MCU2_Pin GPIO_PIN_13
#define IN_BTN_MCU2_GPIO_Port GPIOC
#define IN_BTN_MCU5_Pin GPIO_PIN_14
#define IN_BTN_MCU5_GPIO_Port GPIOC
#define IN_BTN_MCU4_Pin GPIO_PIN_15
#define IN_BTN_MCU4_GPIO_Port GPIOC
#define IN_BTN_MCU3_Pin GPIO_PIN_0
#define IN_BTN_MCU3_GPIO_Port GPIOA
#define ADC1_IN2_SLIDER_Pin GPIO_PIN_1
#define ADC1_IN2_SLIDER_GPIO_Port GPIOA
#define INT_MCP_BTN_Pin GPIO_PIN_2
#define INT_MCP_BTN_GPIO_Port GPIOA
#define INT_MCP_BTN_EXTI_IRQn EXTI2_IRQn
#define INT_MCP_ENC_Pin GPIO_PIN_3
#define INT_MCP_ENC_GPIO_Port GPIOA
#define INT_MCP_ENC_EXTI_IRQn EXTI3_IRQn
#define OUT_MCP_CS_Pin GPIO_PIN_4
#define OUT_MCP_CS_GPIO_Port GPIOA
#define SPI1_MCP_SCK_Pin GPIO_PIN_5
#define SPI1_MCP_SCK_GPIO_Port GPIOA
#define SPI1_MCP_MISO_Pin GPIO_PIN_6
#define SPI1_MCP_MISO_GPIO_Port GPIOA
#define SPI1_MCP_MOSI_Pin GPIO_PIN_7
#define SPI1_MCP_MOSI_GPIO_Port GPIOA
#define OUT_MCP_RESET_Pin GPIO_PIN_0
#define OUT_MCP_RESET_GPIO_Port GPIOB
#define WS_TIM3_CH4_Pin GPIO_PIN_1
#define WS_TIM3_CH4_GPIO_Port GPIOB
#define OUT_ADC_CS_Pin GPIO_PIN_2
#define OUT_ADC_CS_GPIO_Port GPIOB
#define OUT_ADC_CNVST_Pin GPIO_PIN_10
#define OUT_ADC_CNVST_GPIO_Port GPIOB
#define OUT_ADC_ADDR_Pin GPIO_PIN_11
#define OUT_ADC_ADDR_GPIO_Port GPIOB
#define INT_ADC_BUSY_Pin GPIO_PIN_12
#define INT_ADC_BUSY_GPIO_Port GPIOB
#define INT_ADC_BUSY_EXTI_IRQn EXTI15_10_IRQn
#define SPI2_DAC_ADC_SCK_Pin GPIO_PIN_13
#define SPI2_DAC_ADC_SCK_GPIO_Port GPIOB
#define SPI2_DAC_ADC_MISO_Pin GPIO_PIN_14
#define SPI2_DAC_ADC_MISO_GPIO_Port GPIOB
#define SPI2_DAC_ADC_MOSI_Pin GPIO_PIN_15
#define SPI2_DAC_ADC_MOSI_GPIO_Port GPIOB
#define MENU_BTN_2_Pin GPIO_PIN_8
#define MENU_BTN_2_GPIO_Port GPIOA
#define EXTRA_PA9_Pin GPIO_PIN_9
#define EXTRA_PA9_GPIO_Port GPIOA
#define OUT_DAC_SYNC_Pin GPIO_PIN_10
#define OUT_DAC_SYNC_GPIO_Port GPIOA
#define SPI3_FRAM_CS_Pin GPIO_PIN_15
#define SPI3_FRAM_CS_GPIO_Port GPIOA
#define SPI3_FRAM_SCK_Pin GPIO_PIN_3
#define SPI3_FRAM_SCK_GPIO_Port GPIOB
#define SPI3_FRAM_MISO_Pin GPIO_PIN_4
#define SPI3_FRAM_MISO_GPIO_Port GPIOB
#define SPI3_FRAM_MOSI_Pin GPIO_PIN_5
#define SPI3_FRAM_MOSI_GPIO_Port GPIOB
#define MENU_BTN_3_Pin GPIO_PIN_6
#define MENU_BTN_3_GPIO_Port GPIOB
#define LEVEL_SHIFTER_EN_Pin GPIO_PIN_7
#define LEVEL_SHIFTER_EN_GPIO_Port GPIOB
#define MENU_BTN_BOOT_Pin GPIO_PIN_8
#define MENU_BTN_BOOT_GPIO_Port GPIOB
#define IN_BTN_MCU1_Pin GPIO_PIN_9
#define IN_BTN_MCU1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define MIDI_IN_PORTS_NUM 0x01  // Specify input ports number of your device
#define MIDI_OUT_PORTS_NUM 0x01 // Specify output ports number of your device
#define LEDS 24
    /* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
