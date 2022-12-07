/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#define PRESSR_COLD_Pin GPIO_PIN_0
#define PRESSR_COLD_GPIO_Port GPIOC
#define PRESSR_HOT_Pin GPIO_PIN_1
#define PRESSR_HOT_GPIO_Port GPIOC
#define LEAK1_Pin GPIO_PIN_2
#define LEAK1_GPIO_Port GPIOC
#define LEAK1_EXTI_IRQn EXTI2_IRQn
#define LEAK2_Pin GPIO_PIN_3
#define LEAK2_GPIO_Port GPIOC
#define LEAK2_EXTI_IRQn EXTI3_IRQn
#define CHK_12V_Pin GPIO_PIN_0
#define CHK_12V_GPIO_Port GPIOA
#define ZB_RES_Pin GPIO_PIN_1
#define ZB_RES_GPIO_Port GPIOA
#define ZB_TX_Pin GPIO_PIN_2
#define ZB_TX_GPIO_Port GPIOA
#define ZB_RX_Pin GPIO_PIN_3
#define ZB_RX_GPIO_Port GPIOA
#define LED_HOT_OPN_Pin GPIO_PIN_4
#define LED_HOT_OPN_GPIO_Port GPIOA
#define LED_HOT_CLS_Pin GPIO_PIN_5
#define LED_HOT_CLS_GPIO_Port GPIOA
#define LED_COLD_OPN_Pin GPIO_PIN_6
#define LED_COLD_OPN_GPIO_Port GPIOA
#define LED_COLD_CLS_Pin GPIO_PIN_7
#define LED_COLD_CLS_GPIO_Port GPIOA
#define HOT_DIR_Pin GPIO_PIN_4
#define HOT_DIR_GPIO_Port GPIOC
#define HOT_EN_Pin GPIO_PIN_5
#define HOT_EN_GPIO_Port GPIOC
#define COLD_DIR_Pin GPIO_PIN_0
#define COLD_DIR_GPIO_Port GPIOB
#define COLD_EN_Pin GPIO_PIN_1
#define COLD_EN_GPIO_Port GPIOB
#define TEST1_Pin GPIO_PIN_2
#define TEST1_GPIO_Port GPIOB
#define RS485_TX_Pin GPIO_PIN_10
#define RS485_TX_GPIO_Port GPIOB
#define RS485_RX_Pin GPIO_PIN_11
#define RS485_RX_GPIO_Port GPIOB
#define COLD_CLS_Pin GPIO_PIN_12
#define COLD_CLS_GPIO_Port GPIOB
#define COLD_OPN_Pin GPIO_PIN_13
#define COLD_OPN_GPIO_Port GPIOB
#define HOT_CLS_Pin GPIO_PIN_14
#define HOT_CLS_GPIO_Port GPIOB
#define HOT_OPN_Pin GPIO_PIN_15
#define HOT_OPN_GPIO_Port GPIOB
#define KEY_HOT_Pin GPIO_PIN_6
#define KEY_HOT_GPIO_Port GPIOC
#define KEY_COLD_Pin GPIO_PIN_7
#define KEY_COLD_GPIO_Port GPIOC
#define ZB_RUN_Pin GPIO_PIN_8
#define ZB_RUN_GPIO_Port GPIOC
#define ZB_NET_Pin GPIO_PIN_9
#define ZB_NET_GPIO_Port GPIOC
#define ZB_AT_HEX_Pin GPIO_PIN_8
#define ZB_AT_HEX_GPIO_Port GPIOA
#define COUNT_COLD_Pin GPIO_PIN_10
#define COUNT_COLD_GPIO_Port GPIOC
#define COUNT_COLD_EXTI_IRQn EXTI15_10_IRQn
#define COUNT_HOT_Pin GPIO_PIN_11
#define COUNT_HOT_GPIO_Port GPIOC
#define COUNT_HOT_EXTI_IRQn EXTI15_10_IRQn
#define COUNT_FILTER_Pin GPIO_PIN_12
#define COUNT_FILTER_GPIO_Port GPIOC
#define COUNT_FILTER_EXTI_IRQn EXTI15_10_IRQn
#define LED_CHK_Pin GPIO_PIN_2
#define LED_CHK_GPIO_Port GPIOD
#define RS485_SEND_Pin GPIO_PIN_3
#define RS485_SEND_GPIO_Port GPIOB
#define PWR_HOT_Pin GPIO_PIN_4
#define PWR_HOT_GPIO_Port GPIOB
#define PWR_COLD_Pin GPIO_PIN_5
#define PWR_COLD_GPIO_Port GPIOB
#define OVR_COLD_Pin GPIO_PIN_8
#define OVR_COLD_GPIO_Port GPIOB
#define OVR_HOT_Pin GPIO_PIN_9
#define OVR_HOT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
