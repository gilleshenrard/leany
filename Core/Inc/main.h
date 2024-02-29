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
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_iwdg.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_spi.h"
#include "stm32f1xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ADXL_CS_Pin LL_GPIO_PIN_4
#define ADXL_CS_GPIO_Port GPIOA
#define ADXL_SCL_Pin LL_GPIO_PIN_5
#define ADXL_SCL_GPIO_Port GPIOA
#define ADXL_SDO_Pin LL_GPIO_PIN_6
#define ADXL_SDO_GPIO_Port GPIOA
#define ADXL_SDA_Pin LL_GPIO_PIN_7
#define ADXL_SDA_GPIO_Port GPIOA
#define ADXL_INT1_Pin LL_GPIO_PIN_0
#define ADXL_INT1_GPIO_Port GPIOB
#define ZERO_BUTTON_Pin LL_GPIO_PIN_10
#define ZERO_BUTTON_GPIO_Port GPIOB
#define HOLD_BUTTON_Pin LL_GPIO_PIN_11
#define HOLD_BUTTON_GPIO_Port GPIOB
#define SSD1306_CS_Pin LL_GPIO_PIN_12
#define SSD1306_CS_GPIO_Port GPIOB
#define SSD1306_D0_Pin LL_GPIO_PIN_13
#define SSD1306_D0_GPIO_Port GPIOB
#define SSD1306_D1_Pin LL_GPIO_PIN_15
#define SSD1306_D1_GPIO_Port GPIOB
#define SSD1306_DC_Pin LL_GPIO_PIN_9
#define SSD1306_DC_GPIO_Port GPIOA
#define SSD1306_RES_Pin LL_GPIO_PIN_10
#define SSD1306_RES_GPIO_Port GPIOA
#define DEBUG_SWDIO_Pin LL_GPIO_PIN_13
#define DEBUG_SWDIO_GPIO_Port GPIOA
#define DEBUG_SWCLK_Pin LL_GPIO_PIN_14
#define DEBUG_SWCLK_GPIO_Port GPIOA
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
