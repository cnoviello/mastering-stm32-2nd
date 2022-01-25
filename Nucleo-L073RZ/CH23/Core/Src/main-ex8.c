/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"
#include "retarget.h"
#include <cmsis_os.h>
#include <nucleo_hal_bsp.h>
#include <string.h>
#include <stdio.h>
#include <task.h>

/* Private variables ---------------------------------------------------------*/
extern UART_HandleTypeDef huart2;

/* Private function prototypes -----------------------------------------------*/
void blinkThread(void *argument);
void threadsDumpThread(void *argument);

/* Definitions for blinkThread and threadsDumpThread */
osThreadId_t blinkThreadID;
const osThreadAttr_t blinkThread_attr = {
  .name = "blinkThread",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4, /* In bytes */
};

osThreadId_t dumpThreadID;
const osThreadAttr_t dumpThread_attr = {
  .name = "dumpThread",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4, /* In bytes */
};

int main(void) {
  HAL_Init();

  Nucleo_BSP_Init();
  RetargetInit(&huart2);

  osKernelInitialize();

  blinkThreadID = osThreadNew(blinkThread, NULL, &blinkThread_attr);
  dumpThreadID = osThreadNew(threadsDumpThread, NULL, &dumpThread_attr);

  osKernelStart();

  /* Infinite loop */
  while (1);
}

void threadsDumpThread(void *argument) {
 TaskStatus_t *pxTaskStatusArray = NULL;
 char *pcBuf = NULL;
 char *pcStatus;
 uint32_t ulTotalRuntime;

  while(1) {
    if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET) {
      /* Allocate the message buffer. */
      pcBuf = pvPortMalloc(100 * sizeof(char));

      /* Allocate an array index for each task. */
      pxTaskStatusArray = pvPortMalloc( uxTaskGetNumberOfTasks() * sizeof( TaskStatus_t ) );

      if( pcBuf != NULL && pxTaskStatusArray != NULL ) {
        /* Generate the (binary) data. */
        uxTaskGetSystemState( pxTaskStatusArray, uxTaskGetNumberOfTasks(), &ulTotalRuntime );

        sprintf(pcBuf, "         LIST OF RUNNING THREADS(%lu)         \r\n-----------------------------------------\r\n", uxTaskGetNumberOfTasks());
        HAL_UART_Transmit(&huart2, (uint8_t*)pcBuf, strlen(pcBuf), HAL_MAX_DELAY);

        for(uint16_t i = 0; i < uxTaskGetNumberOfTasks(); i++ )
        {
          sprintf(pcBuf, "Thread: %s\r\n", pxTaskStatusArray[i].pcTaskName);
          HAL_UART_Transmit(&huart2, (uint8_t*)pcBuf, strlen(pcBuf), HAL_MAX_DELAY);

          sprintf(pcBuf, "Thread ID: %lu\r\n", pxTaskStatusArray[i].xTaskNumber);
          HAL_UART_Transmit(&huart2, (uint8_t*)pcBuf, strlen(pcBuf), HAL_MAX_DELAY);

          switch (pxTaskStatusArray[i].eCurrentState) {
          case eRunning:
            pcStatus = "RUNNING";
            break;
          case eReady:
            pcStatus = "READY";
            break;
          case eBlocked:
            pcStatus = "BLOCKED";
            break;
          case eSuspended:
            pcStatus = "SUSPENDED";
            break;
          case eDeleted:
            pcStatus = "DELETED";
            break;

          default: /* Should not get here, but it is included
                      to prevent static checking errors. */
            pcStatus = "UNKNOWN";
            break;
          }

          sprintf(pcBuf, "\tStatus: %s\r\n", pcStatus);
          HAL_UART_Transmit(&huart2, (uint8_t*)pcBuf, strlen(pcBuf), HAL_MAX_DELAY);

          sprintf(pcBuf, "\tStack watermark number: %d\r\n", pxTaskStatusArray[i].usStackHighWaterMark);
          HAL_UART_Transmit(&huart2, (uint8_t*)pcBuf, strlen(pcBuf), HAL_MAX_DELAY);

          sprintf(pcBuf, "\tPriority: %lu\r\n", pxTaskStatusArray[i].uxCurrentPriority);
          HAL_UART_Transmit(&huart2, (uint8_t*)pcBuf, strlen(pcBuf), HAL_MAX_DELAY);

          sprintf(pcBuf, "\tRun-time time: %lu\r\n", pxTaskStatusArray[i].ulRunTimeCounter);
          HAL_UART_Transmit(&huart2, (uint8_t*)pcBuf, strlen(pcBuf), HAL_MAX_DELAY);

          float data = (float)(((float)pxTaskStatusArray[i].ulRunTimeCounter)/ulTotalRuntime)*100;
          sprintf(pcBuf, "\tRun-time time in percentage: %lu%%\r\n", (uint32_t)data);
          HAL_UART_Transmit(&huart2, (uint8_t*)pcBuf, strlen(pcBuf), HAL_MAX_DELAY);
        }

        vPortFree(pcBuf);
        vPortFree(pxTaskStatusArray);
      }
    }
    osDelay(50);
  }
}

void blinkThread(void *argument) {
  while(1) {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    osDelay(500);
  }
}

#ifdef DEBUG

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed portCHAR *pcTaskName ) {
  UNUSED(pxTask);
  UNUSED(pcTaskName);
  asm("BKPT #0");
}

#endif

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

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
