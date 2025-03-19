/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.h
  * @author  MCD Application Team
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019-2021 STMicroelectronics.
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
#include "stm32wbxx_hal.h"
#include "app_conf.h"
#include "app_entry.h"
#include "app_common.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32wbxx_nucleo.h"
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
#define MCU_CHG_nDISABLE_Pin GPIO_PIN_2
#define MCU_CHG_nDISABLE_GPIO_Port GPIOC
#define MCU_CHG_INT_Pin GPIO_PIN_3
#define MCU_CHG_INT_GPIO_Port GPIOC
#define MCU_CHG_nPGD_Pin GPIO_PIN_0
#define MCU_CHG_nPGD_GPIO_Port GPIOA
#define LED_RED_2_Pin GPIO_PIN_10
#define LED_RED_2_GPIO_Port GPIOA
#define LED_GREEN_2_Pin GPIO_PIN_11
#define LED_GREEN_2_GPIO_Port GPIOA
#define LED_GREEN_1_Pin GPIO_PIN_12
#define LED_GREEN_1_GPIO_Port GPIOA
#define LED_RED_1_Pin GPIO_PIN_15
#define LED_RED_1_GPIO_Port GPIOA
#define MCU_SWITCH_INT_Pin GPIO_PIN_12
#define MCU_SWITCH_INT_GPIO_Port GPIOC
#define MCU_KILL_Pin GPIO_PIN_0
#define MCU_KILL_GPIO_Port GPIOD
#define MCU_WIRELESS_PWR_nEN_Pin GPIO_PIN_5
#define MCU_WIRELESS_PWR_nEN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
