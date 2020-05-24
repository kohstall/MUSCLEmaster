/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f4xx_hal.h"

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
#define LD_1_Pin GPIO_PIN_3
#define LD_1_GPIO_Port GPIOE
#define LD_2_Pin GPIO_PIN_4
#define LD_2_GPIO_Port GPIOE
#define M0_TEMP_Pin GPIO_PIN_5
#define M0_TEMP_GPIO_Port GPIOA
#define EN_GATE_Pin GPIO_PIN_14
#define EN_GATE_GPIO_Port GPIOE
#define M0_DC_CAL_Pin GPIO_PIN_15
#define M0_DC_CAL_GPIO_Port GPIOE
#define debug1_out_Pin GPIO_PIN_12
#define debug1_out_GPIO_Port GPIOD
#define debug2_out_Pin GPIO_PIN_13
#define debug2_out_GPIO_Port GPIOD
#define debug1_in_Pin GPIO_PIN_14
#define debug1_in_GPIO_Port GPIOD
#define ROT0_nCS_Pin GPIO_PIN_15
#define ROT0_nCS_GPIO_Port GPIOD
#define ROT0_I_W_Pin GPIO_PIN_8
#define ROT0_I_W_GPIO_Port GPIOC
#define ROT0_I_W_EXTI_IRQn EXTI9_5_IRQn
#define PWRGD_Pin GPIO_PIN_4
#define PWRGD_GPIO_Port GPIOD
#define nOCTW_Pin GPIO_PIN_5
#define nOCTW_GPIO_Port GPIOD
#define nFAULT_Pin GPIO_PIN_6
#define nFAULT_GPIO_Port GPIOD
#define nSCS_Pin GPIO_PIN_7
#define nSCS_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

#define AS_ODD 0x8000
#define AS_READ 0x4000
#define AS_EVEN 0x0000
#define AS_WRITE 0x0000
#define AS_DATA_MASK 0x3FFF
#define AS_ADDR_ZPOSM 0x0016
#define AS_ADDR_ZPOSL 0x8017
#define AS_ADDR_SETTINGS1 0x0018
#define AS_ADDR_SETTINGS2 0x8019

#define USE_HAL_TIM_REGISTER_CALLBACKS 1

#define PI 3.141592

#define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })
#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

#define MOTOR_POLES 6


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
