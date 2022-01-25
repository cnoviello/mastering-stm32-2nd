/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include <cmsis_os.h>
#include <nucleo_hal_bsp.h>
#include <string.h>
#include "FreeRTOSConfig.h"
#include "retarget.h"
#include <task.h>

/* Private variables ---------------------------------------------------------*/
extern UART_HandleTypeDef huart2;

/* Private function prototypes -----------------------------------------------*/
void blinkThread(void *argument);

/* Definitions for blinkThread and UARTThread */
osThreadId_t blinkThreadID;
const osThreadAttr_t blinkThread_attr = {
  .name = "blinkThread",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4, /* In bytes */
};

volatile uint8_t suspendBlink = 0;
extern volatile uint8_t ucSleepModeCalled;
extern void SystemClock_Config(void);


static volatile uint32_t sleepTime = 50;

int main(void) {
  HAL_Init();
  Nucleo_BSP_Init();
  RetargetInit(&huart2);

  /* Init scheduler */
  osKernelInitialize();

  blinkThreadID = osThreadNew(blinkThread, NULL, &blinkThread_attr);

  osKernelStart();

  /* Infinite loop */
  while (1);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == B1_Pin) {
    osThreadFlagsSet(blinkThreadID, 0x1);
    if(suspendBlink) {
      BaseType_t xYieldRequired;

      // Resume the suspended task.
      xYieldRequired = xTaskResumeFromISR((TaskHandle_t)blinkThreadID);

      // We should switch context so the ISR returns to a different task.
      // NOTE:  How this is done depends on the port you are using.  Check
      // the documentation and examples for your port.
      portYIELD_FROM_ISR( xYieldRequired );
    }
  }
}

void blinkThread(void *argument) {
  UNUSED(argument);
  uint32_t flags;

  while (1) {
    flags = osThreadFlagsWait(0x1, osFlagsWaitAll, 1);
    if (flags == 0x1) {
      ucSleepModeCalled = 0;
      if (suspendBlink) {
        suspendBlink = 0;
        sleepTime = 50;
      } else if (sleepTime == 50) {
        sleepTime = 500;
      } else if (sleepTime == 500) {
        suspendBlink = 1;
        osThreadSuspend(blinkThreadID);
      }
    }
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    osDelay(sleepTime);
  }
}

#if configUSE_TICKLESS_IDLE == 2

void preSLEEP(uint32_t xModifiableIdleTime) {
  UNUSED(xModifiableIdleTime);

  HAL_SuspendTick();
  __enable_irq();
  __disable_irq();
}

void postSLEEP(uint32_t xModifiableIdleTime) {
  UNUSED(xModifiableIdleTime);

  HAL_ResumeTick();
}

void preSTOP() {
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
  __HAL_RCC_GPIOA_CLK_DISABLE();
  HAL_SuspendTick();
  __enable_irq();
  __disable_irq();
}

void postSTOP() {
  SystemClock_Config();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  HAL_ResumeTick();
}


#endif

#ifdef DEBUG

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed portCHAR *pcTaskName) {
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

