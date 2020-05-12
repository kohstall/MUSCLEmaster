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
#define ASENSE_Pin GPIO_PIN_1
#define ASENSE_GPIO_Port GPIOC
#define BSENSE_Pin GPIO_PIN_2
#define BSENSE_GPIO_Port GPIOC
#define CSENSE_Pin GPIO_PIN_3
#define CSENSE_GPIO_Port GPIOC
#define I_Pin GPIO_PIN_1
#define I_GPIO_Port GPIOA
#define M0_SO1_Pin GPIO_PIN_2
#define M0_SO1_GPIO_Port GPIOA
#define M0_SO2_Pin GPIO_PIN_3
#define M0_SO2_GPIO_Port GPIOA
#define TEMP_Pin GPIO_PIN_4
#define TEMP_GPIO_Port GPIOA
#define M0_TEMP_Pin GPIO_PIN_5
#define M0_TEMP_GPIO_Port GPIOA
#define STRAIN0_Pin GPIO_PIN_4
#define STRAIN0_GPIO_Port GPIOC
#define STRAIN1_Pin GPIO_PIN_5
#define STRAIN1_GPIO_Port GPIOC
#define VBUS_S_Pin GPIO_PIN_0
#define VBUS_S_GPIO_Port GPIOB
#define M0_AH_Pin GPIO_PIN_9
#define M0_AH_GPIO_Port GPIOE
#define EN_GATE_Pin GPIO_PIN_14
#define EN_GATE_GPIO_Port GPIOE
#define M0_DC_CAL_Pin GPIO_PIN_15
#define M0_DC_CAL_GPIO_Port GPIOE
#define ROT0_A_U_Pin GPIO_PIN_6
#define ROT0_A_U_GPIO_Port GPIOC
#define ROT0_A_U_EXTI_IRQn EXTI9_5_IRQn
#define GENERATOR_Pin GPIO_PIN_7
#define GENERATOR_GPIO_Port GPIOC
#define PWRGD_Pin GPIO_PIN_4
#define PWRGD_GPIO_Port GPIOD
#define nOCTW_Pin GPIO_PIN_5
#define nOCTW_GPIO_Port GPIOD
#define nFAULT_Pin GPIO_PIN_6
#define nFAULT_GPIO_Port GPIOD
#define nSCS_Pin GPIO_PIN_7
#define nSCS_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
