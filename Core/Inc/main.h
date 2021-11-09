/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Exported functions prototypes ---------------8-----------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void USER_UART_IRQHandler(UART_HandleTypeDef *huart);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
//uint8_t numTrays = 2;
//uint8_t maxSelsPerTray = 16;

#define numTrays 2
#define maxSelsPerTray 16



#define Tray0_Sels 4
#define Tray1_Sels 4
#define Tray2_Sels 0
#define Tray3_Sels 0
#define Tray4_Sels 0
#define Tray5_Sels 0
#define Tray6_Sels 0
#define Tray7_Sels 0




#define SW_NREST_MAST_Pin GPIO_PIN_6
#define SW_NREST_MAST_Port GPIOB

#define ST_OUT_DI2C_EN_0_Pin GPIO_PIN_11
#define ST_OUT_DI2C_EN_0_Port GPIOC

#define ST_OUT_DI2C_EN_1_Pin GPIO_PIN_12
#define ST_OUT_DI2C_EN_1_Port GPIOC

#define ST_OUT_DI2C_EN_2_Pin GPIO_PIN_4
#define ST_OUT_DI2C_EN_2_Port GPIOB

#define ST_OUT_DI2C_EN_3_Pin GPIO_PIN_5
#define ST_OUT_DI2C_EN_3_Port GPIOB

#define ST_OUT_DI2C_EN_4_Pin GPIO_PIN_13
#define ST_OUT_DI2C_EN_4_Port GPIOC

#define ST_OUT_DI2C_EN_5_Pin GPIO_PIN_14
#define ST_OUT_DI2C_EN_5_Port GPIOC

#define ST_OUT_DI2C_EN_6_Pin GPIO_PIN_15
#define ST_OUT_DI2C_EN_6_Port GPIOC

#define ST_OUT_DI2C_EN_7_Pin GPIO_PIN_4
#define ST_OUT_DI2C_EN_7_Port GPIOA

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
