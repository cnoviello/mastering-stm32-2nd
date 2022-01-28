/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "nucleo_hal_bsp.h"
#include "cmsis_os.h"
#include <string.h>
#include <task.h>

/* Private variables ---------------------------------------------------------*/
extern UART_HandleTypeDef hlpuart1;

/* Private function prototypes -----------------------------------------------*/
void blinkThread(void *argument);
void UARTThread(void  *argument);

/* Definitions for blinkThread and UARTThread */
osThreadId_t blinkThreadID;
const osThreadAttr_t blinkThread_attr = {
  .name = "blinkThread",
  .stack_size = 128 * 4, /* In bytes */
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t UARTThreadID;
const osThreadAttr_t UARTThread_attr = {
  .name = "UARTThread",
  .stack_size = 128 * 4, /* In bytes */
  .priority = (osPriority_t) osPriorityAboveNormal,
};

int main(void) {
  HAL_Init();

  Nucleo_BSP_Init();

  /* Init scheduler */
  osKernelInitialize();

  /* Creation of blinkThread */
  blinkThreadID = osThreadNew(blinkThread, NULL, &blinkThread_attr);
  /* Creation of UARTThread */
  UARTThreadID = osThreadNew(UARTThread, NULL, &UARTThread_attr);

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  while (1);
}

void blinkThread(void *argument) {
  while(1) {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    osDelay(500);
  }
}

void UARTThread(void *argument) {
  while(1) {
    HAL_UART_Transmit(&hlpuart1, (uint8_t *)"UARTThread\r\n", strlen("UARTThread\r\n"), HAL_MAX_DELAY);
  }
}

#ifdef DEBUG

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed portCHAR *pcTaskName ) {
  asm("BKPT #0");
}

#endif
/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

