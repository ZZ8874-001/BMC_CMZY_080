/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define HSE_DIS_Pin GPIO_PIN_1
#define HSE_DIS_GPIO_Port GPIOA
#define RGB_USART2_TX_Pin GPIO_PIN_2
#define RGB_USART2_TX_GPIO_Port GPIOA
#define COMP_BTTN_Pin GPIO_PIN_3
#define COMP_BTTN_GPIO_Port GPIOA
#define COMP_PACK_Pin GPIO_PIN_4
#define COMP_PACK_GPIO_Port GPIOA
#define BUZ_TIM1_CH1_Pin GPIO_PIN_7
#define BUZ_TIM1_CH1_GPIO_Port GPIOA
#define SUP_5V_EN_Pin GPIO_PIN_0
#define SUP_5V_EN_GPIO_Port GPIOB
#define BQ_BOOT_Pin GPIO_PIN_1
#define BQ_BOOT_GPIO_Port GPIOB
#define BQ_ALT_EXTI8_Pin GPIO_PIN_8
#define BQ_ALT_EXTI8_GPIO_Port GPIOA
#define BQ_IIC2_SCL_Pin GPIO_PIN_9
#define BQ_IIC2_SCL_GPIO_Port GPIOA
#define BQ_IIC1_SDA_Pin GPIO_PIN_10
#define BQ_IIC1_SDA_GPIO_Port GPIOA
#define IND_ROHT_Pin GPIO_PIN_3
#define IND_ROHT_GPIO_Port GPIOB
#define IND_NORM_Pin GPIO_PIN_4
#define IND_NORM_GPIO_Port GPIOB
#define WAKE_EXTI5_Pin GPIO_PIN_5
#define WAKE_EXTI5_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
