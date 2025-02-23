/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

#include "stm32f1xx_ll_adc.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_i2c.h"
#include "stm32f1xx_ll_iwdg.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_spi.h"
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "projdefs.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/**
 * Check if a timeout in [ms] has occurred
 *
 * @param startTick The tick to compare to now to check for a timeout
 * @param timeout The timeout span in milliseconds
 * @retval 1 Timeout has occurred
 * @retval 0 Timeout has not occurred
 */
static inline uint8_t timeout(uint32_t startTick, uint16_t timeout_ms) {
  return ((HAL_GetTick() - startTick) >= pdMS_TO_TICKS(timeout_ms));
}
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_RED_Pin LL_GPIO_PIN_0
#define LED_RED_GPIO_Port GPIOA
#define LED_GREEN_Pin LL_GPIO_PIN_1
#define LED_GREEN_GPIO_Port GPIOA
#define LED_BLUE_Pin LL_GPIO_PIN_2
#define LED_BLUE_GPIO_Port GPIOA
#define BATT_VOLT_Pin LL_GPIO_PIN_3
#define BATT_VOLT_GPIO_Port GPIOA
#define BMI270_CS_Pin LL_GPIO_PIN_4
#define BMI270_CS_GPIO_Port GPIOA
#define BMI270_SCL_Pin LL_GPIO_PIN_5
#define BMI270_SCL_GPIO_Port GPIOA
#define BMI270_SDO_Pin LL_GPIO_PIN_6
#define BMI270_SDO_GPIO_Port GPIOA
#define BMI270_SDA_Pin LL_GPIO_PIN_7
#define BMI270_SDA_GPIO_Port GPIOA
#define BMI270_INT1_Pin LL_GPIO_PIN_0
#define BMI270_INT1_GPIO_Port GPIOB
#define BMI270_INT1_EXTI_IRQn EXTI0_IRQn
#define CHG_INT_Pin LL_GPIO_PIN_1
#define CHG_INT_GPIO_Port GPIOB
#define CHG_INT_EXTI_IRQn EXTI1_IRQn
#define BMI270_INT2_Pin LL_GPIO_PIN_2
#define BMI270_INT2_GPIO_Port GPIOB
#define BMI270_INT2_EXTI_IRQn EXTI2_IRQn
#define ZERO_BUTTON_Pin LL_GPIO_PIN_10
#define ZERO_BUTTON_GPIO_Port GPIOB
#define HOLD_BUTTON_Pin LL_GPIO_PIN_11
#define HOLD_BUTTON_GPIO_Port GPIOB
#define ST7735S_CS_Pin LL_GPIO_PIN_12
#define ST7735S_CS_GPIO_Port GPIOB
#define ST7735S_SCL_Pin LL_GPIO_PIN_13
#define ST7735S_SCL_GPIO_Port GPIOB
#define ST7735S_MOSI_Pin LL_GPIO_PIN_15
#define ST7735S_MOSI_GPIO_Port GPIOB
#define BATT_EN_Pin LL_GPIO_PIN_8
#define BATT_EN_GPIO_Port GPIOA
#define FTDI_TX_Pin LL_GPIO_PIN_9
#define FTDI_TX_GPIO_Port GPIOA
#define FTDI_RX_Pin LL_GPIO_PIN_10
#define FTDI_RX_GPIO_Port GPIOA
#define ST7735S_DC_Pin LL_GPIO_PIN_11
#define ST7735S_DC_GPIO_Port GPIOA
#define ST7735S_RST_Pin LL_GPIO_PIN_12
#define ST7735S_RST_GPIO_Port GPIOA
#define DEBUG_SWDIO_Pin LL_GPIO_PIN_13
#define DEBUG_SWDIO_GPIO_Port GPIOA
#define DEBUG_SWCLK_Pin LL_GPIO_PIN_14
#define DEBUG_SWCLK_GPIO_Port GPIOA
#define ST7735S_BL_Pin LL_GPIO_PIN_15
#define ST7735S_BL_GPIO_Port GPIOA
#define DEBUG_OUT_Pin LL_GPIO_PIN_4
#define DEBUG_OUT_GPIO_Port GPIOB
#define CHG_SCL_Pin LL_GPIO_PIN_6
#define CHG_SCL_GPIO_Port GPIOB
#define CHG_SDA_Pin LL_GPIO_PIN_7
#define CHG_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
