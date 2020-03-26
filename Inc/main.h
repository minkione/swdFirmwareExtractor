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
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOC
#define TARGET_PWR_Pin GPIO_PIN_5
#define TARGET_PWR_GPIO_Port GPIOA
#define TARGET_RESET_Pin GPIO_PIN_6
#define TARGET_RESET_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_7
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_0
#define SWCLK_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#include <stdint.h>


#ifndef NULL
#define NULL ((void*) 0)
#endif

#define likely(x)       __builtin_expect((x),1)
#define unlikely(x)     __builtin_expect((x),0)

#define MAX_READ_ATTEMPTS (100u)

/* all times in milliseconds */
/* minimum wait time between reset deassert and attack */
#define DELAY_JITTER_MS_MIN (20u)
/* increment per failed attack */
#define DELAY_JITTER_MS_INCREMENT (1u)
/* maximum wait time between reset deassert and attack */
#define DELAY_JITTER_MS_MAX (50u)

/* flash readout statistics */
typedef struct {
    uint32_t numAttempts;
    uint32_t numSuccess;
    uint32_t numFailure;
} extractionStatistics_t;

void printExtractionStatistics( void );

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
