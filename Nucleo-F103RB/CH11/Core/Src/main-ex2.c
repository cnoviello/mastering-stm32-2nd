/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "main.h"
#include <nucleo_hal_bsp.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_up;

int main(void) {
  uint8_t data[] = {0xFF, 0x0};

  HAL_Init();

  Nucleo_BSP_Init();

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 31999; //64MHz/32000 = 2000Hz
  htim1.Init.Period = 999;      //2000HZ / 1000 = 2Hz = 0.5s

  __TIM1_CLK_ENABLE();

  hdma_tim1_up.Instance = DMA1_Channel5;
  hdma_tim1_up.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_tim1_up.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_tim1_up.Init.MemInc = DMA_MINC_ENABLE;
  hdma_tim1_up.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_tim1_up.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_tim1_up.Init.Mode = DMA_CIRCULAR;
  hdma_tim1_up.Init.Priority = DMA_PRIORITY_LOW;

  HAL_TIM_Base_Init(&htim1);
  HAL_DMA_Init(&hdma_tim1_up);

  HAL_TIM_Base_Start(&htim1);
  HAL_DMA_Start(&hdma_tim1_up, (uint32_t)data, (uint32_t)&GPIOA->ODR, 2);
  __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_UPDATE);

  while (1);
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
