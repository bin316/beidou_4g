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
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

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
#define SENSOR_INT1_Pin GPIO_PIN_0
#define SENSOR_INT1_GPIO_Port GPIOA
#define SENSOR_INT1_EXTI_IRQn EXTI0_IRQn
#define SENSOR_NPWR_Pin GPIO_PIN_1
#define SENSOR_NPWR_GPIO_Port GPIOA
#define BD_TX_Pin GPIO_PIN_2
#define BD_TX_GPIO_Port GPIOA
#define BD_RX_Pin GPIO_PIN_3
#define BD_RX_GPIO_Port GPIOA
#define BD_NRST_Pin GPIO_PIN_4
#define BD_NRST_GPIO_Port GPIOA
#define LTE_PWR_Pin GPIO_PIN_0
#define LTE_PWR_GPIO_Port GPIOB
#define BD_PWR_Pin GPIO_PIN_1
#define BD_PWR_GPIO_Port GPIOB
#define LTE_PWK_Pin GPIO_PIN_8
#define LTE_PWK_GPIO_Port GPIOA
#define LTE_TX_Pin GPIO_PIN_9
#define LTE_TX_GPIO_Port GPIOA
#define LTE_RX_Pin GPIO_PIN_10
#define LTE_RX_GPIO_Port GPIOA
#define TEST_Pin GPIO_PIN_5
#define TEST_GPIO_Port GPIOB
#define SENSOR_SCL_Pin GPIO_PIN_6
#define SENSOR_SCL_GPIO_Port GPIOB
#define SENSOR_SDA_Pin GPIO_PIN_7
#define SENSOR_SDA_GPIO_Port GPIOB
#define LED_BOOT_Pin GPIO_PIN_3
#define LED_BOOT_GPIO_Port GPIOH

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
