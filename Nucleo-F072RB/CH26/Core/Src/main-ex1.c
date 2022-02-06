#include "ioLibrary_Driver/Ethernet/socket.h"
#include <nucleo_hal_bsp.h>
#include "stm32f0xx_hal.h"
#include "ioLibrary_Driver/Ethernet/wizchip_conf.h"
#include "retarget-tcp.h"
#include <stdio.h>

extern SPI_HandleTypeDef hspi1;

void Error_Handler(void);

#define MAX_HTTPSOCK  6
#define DATA_BUF_SIZE 2048
uint8_t socknumlist[] = {2, 3, 4, 5, 6, 7};
volatile uint8_t RX_BUF[DATA_BUF_SIZE];
volatile uint8_t TX_BUF[DATA_BUF_SIZE];

void cs_sel() {
  HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_RESET); //CS LOW
}

void cs_desel() {
  HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_SET); //CS HIGH
}

uint8_t spi_rb(void) {
  uint8_t rbuf;
  HAL_SPI_Receive(&hspi1, &rbuf, 1, HAL_MAX_DELAY);
  return rbuf;
}

void spi_wb(uint8_t b) {
  HAL_SPI_Transmit(&hspi1, &b, 1, HAL_MAX_DELAY);
}

void spi_rb_burst(uint8_t *buf, uint16_t len) {
  HAL_SPI_Receive_DMA(&hspi1, buf, len);
  while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_RX);
}

void spi_wb_burst(uint8_t *buf, uint16_t len) {
  HAL_SPI_Transmit_DMA(&hspi1, buf, len);
  while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
}


void IO_LIBRARY_Init(void) {
  uint8_t bufSize[] = {2, 2, 2, 2};

  reg_wizchip_cs_cbfunc(cs_sel, cs_desel);
  reg_wizchip_spi_cbfunc(spi_rb, spi_wb);
  reg_wizchip_spiburst_cbfunc(spi_rb_burst, spi_wb_burst);

  wizchip_init(bufSize, bufSize);
  wiz_NetInfo netInfo = { .mac  = {0x00, 0x08, 0xdc, 0xab, 0xcd, 0xef}, // Mac address
                          .ip   = {192, 168, 2, 165},         // IP address
                          .sn   = {255, 255, 255, 0},         // Subnet mask
                          .gw   = {192, 168, 2, 1}};          // Gateway address
  wizchip_setnetinfo(&netInfo);
  wizchip_setinterruptmask(IK_SOCK_0);
}

int main(void) {
  char buf[20];

  HAL_Init();
  Nucleo_BSP_Init();

  IO_LIBRARY_Init();
  RetargetInit(0);

  while(1) {
    if(printf("Write your name: ")) {
      scanf("%s", buf);
      printf("\r\nYou wrote: %s\r\n", buf);
    }
    HAL_Delay(1000);
  }
}


void Error_Handler(void) {
  asm("BKPT #0");
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
