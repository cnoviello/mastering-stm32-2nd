/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"
#include <nucleo_hal_bsp.h>
#include <string.h>
#include <math.h>
#include "main.h"

#define PI    3.14159
#define SAMPLES 200

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
TIM_HandleTypeDef htim6;
DMA_HandleTypeDef hdma_dac1_ch1;

/* Private function prototypes -----------------------------------------------*/
static void MX_DAC1_Init(void);
static void MX_TIM6_Init(void);

int main(void) {
  uint32_t IV[SAMPLES], value;

  HAL_Init();
  Nucleo_BSP_Init();

  /* Initialize all configured peripherals */
  MX_TIM6_Init();
  MX_DAC1_Init();

  for (uint16_t i = 0; i < SAMPLES; i++) {
    value = (uint16_t) rint((sinf(((2*PI)/SAMPLES)*i)+1)*2048);
    IV[i] = value < 4096 ? value : 4095;
  }

  HAL_DAC_Init(&hdac1);
  HAL_TIM_Base_Start(&htim6);
  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)IV, SAMPLES, DAC_ALIGN_12B_R);

  while(1);
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* DAC1 DMA Init */
  /* DAC1_CH1 Init */
  hdma_dac1_ch1.Instance = DMA1_Channel1;
  hdma_dac1_ch1.Init.Request = DMA_REQUEST_DAC1_CHANNEL1;
  hdma_dac1_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_dac1_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_dac1_ch1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_dac1_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma_dac1_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma_dac1_ch1.Init.Mode = DMA_CIRCULAR;
  hdma_dac1_ch1.Init.Priority = DMA_PRIORITY_LOW;
  if (HAL_DMA_Init(&hdma_dac1_ch1) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_LINKDMA(&hdac1,DMA_Handle1,hdma_dac1_ch1);


  /* USER CODE END DAC1_Init 2 */

}

/* TIM6 init function */
void MX_TIM6_Init(void) {
  TIM_MasterConfigTypeDef sMasterConfig;

  __HAL_RCC_TIM6_CLK_ENABLE();

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 7999;
  HAL_TIM_Base_Init(&htim6);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig);
}

/* USER CODE BEGIN 4 */
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


/* USER CODE BEGIN 4 */

/* USER CODE END 4 */


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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
