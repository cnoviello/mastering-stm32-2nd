#include <nucleo_hal_bsp.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "stm32f4xx_hal.h"
#include "fatfs.h"
#include "ioLibrary_Driver/Ethernet/wizchip_conf.h"
#include "ioLibrary_Driver/Internet/DHCP/dhcp.h"
#include "ioLibrary_Driver/Internet/httpServer/httpParser.h"
#include "ioLibrary_Driver/Internet/httpServer/httpServer.h"

#include "config.h"

#include <cmsis_os2.h>
#include <task.h>

#define MAX_HTTPSOCK  7
#define DHCP_SOCK 0
#define DATA_BUF_SIZE 2048

ADC_HandleTypeDef hadc1;
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_adc1;
UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim2;

void Error_Handler(void);
void MX_ADC1_Init(void);
void SetupW5500Thread(void *argument);
FRESULT scan_files (TCHAR* path);


uint8_t socknumlist[] = {0, 1, 2, 3, 4, 5, 6, 8};
uint8_t RX_BUF[DATA_BUF_SIZE];
uint8_t TX_BUF[DATA_BUF_SIZE];
uint16_t adcConv[100], _adcConv[200];
uint8_t convComplete;

/* Definitions for blinkThread */
osThreadId_t w5500ThreadID;
const osThreadAttr_t w5500Thread_attr = {
  .name = "w5500Thread",
  .stack_size = 1024 * 4, /* In bytes */
  .priority = (osPriority_t) osPriorityNormal,
};
osSemaphoreId_t adcSemID;

#if defined(_USE_SDCARD_) && !defined(OS_USE_SEMIHOSTING)
FATFS diskHandle;
#endif

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
  uint8_t runApplication = 0, dhcpRetry = 0, phyLink = 0, bufSize[] = {2, 2, 2, 2, 2};
  wiz_NetInfo netInfo;

  reg_wizchip_cs_cbfunc(cs_sel, cs_desel);
  reg_wizchip_spi_cbfunc(spi_rb, spi_wb);
  reg_wizchip_spiburst_cbfunc(spi_rb_burst, spi_wb_burst);
  reg_wizchip_cris_cbfunc(vPortEnterCritical, vPortExitCritical);

  wizchip_init(bufSize, bufSize);

  ReadNetCfgFromFile(&netInfo);

  /* Wait until the ETH cable is plugged in */
  do {
    ctlwizchip(CW_GET_PHYLINK, (void*) &phyLink);
    osDelay(10);
  } while(phyLink == PHY_LINK_OFF);

  if(netInfo.dhcp == NETINFO_DHCP) { /* DHCP Mode */
    DHCP_init(DHCP_SOCK, RX_BUF);

    while(!runApplication) {
      switch(DHCP_run()) {
      case DHCP_IP_LEASED:
      case DHCP_IP_ASSIGN:
      case DHCP_IP_CHANGED:
        getIPfromDHCP(netInfo.ip);
        getGWfromDHCP(netInfo.gw);
        getSNfromDHCP(netInfo.sn);
        getDNSfromDHCP(netInfo.dns);
        runApplication = 1;
        break;
      case DHCP_FAILED:
        dhcpRetry++;
        if(dhcpRetry > MAX_DHCP_RETRY)
        {
          netInfo.dhcp = NETINFO_STATIC;
          DHCP_stop();      // if restart, recall DHCP_init()
#ifdef DEBUG
          printf(">> DHCP %d Failed\r\n", dhcpRetry);
#endif
          dhcpRetry = 0;
          asm("BKPT #0");
        }
        break;
      default:
        break;
      }
    }
  }
  wizchip_setnetinfo(&netInfo);
  wizchip_setinterruptmask(IK_SOCK_0);
}

int main(void) {#ifdef OS_USE_SEMIHOSTING
  initialise_monitor_handles();
#endif

  HAL_Init();
  Nucleo_BSP_Init();

  /* Init scheduler */
  osKernelInitialize();
  adcSemID = osSemaphoreNew(1, 1, NULL);

#if defined(_USE_SDCARD_) && !defined(OS_USE_SEMIHOSTING)
  SD_SPI_Configure(SD_CS_GPIO_Port, SD_CS_Pin, &hspi1);
  MX_FATFS_Init();

  if(f_mount(&diskHandle, "0:", 1) != FR_OK) {
#ifdef DEBUG
    asm("BKPT #0");
#else
    while(1) {
      HAL_Delay(500);
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    }
#endif //#ifdef DEBUG
  }

#ifdef DEBUG
  /* Prints the SD content over the UART */
  TCHAR buff[256];
  strcpy(buff, (char*)L"/");

#endif //#ifdef DEBUG

#endif //#if defined(_USE_SDCARD_) && !defined(OS_USE_SEMIHOSTING)

  w5500ThreadID = osThreadNew(SetupW5500Thread, 0, &w5500Thread_attr);

  /* Start scheduler */
  osKernelStart();
  /* Never coming here, but just in case... */
  while(1);
}

void SetupW5500Thread(void *argument) {
  UNUSED(argument);

  /* Configure the W5500 module */
  IO_LIBRARY_Init();

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)_adcConv, 200);

  /* Configure the HTTP server */
  httpServer_init(TX_BUF, RX_BUF, MAX_HTTPSOCK, socknumlist);
  reg_httpServer_cbfunc(NVIC_SystemReset, NULL);

  /* Start processing sockets */
  while(1) {
    for(uint8_t i = 0; i < MAX_HTTPSOCK; i++)
      httpServer_run(i);
    /* We just delay for 1ms so that other threads with the same
     * or lower priority can be executed */
    osDelay(1);
  }
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
  UNUSED(hadc);

  if(osSemaphoreAcquire(adcSemID, 0) == osOK) {
	  memcpy(adcConv, _adcConv, sizeof(uint16_t)*100);
	  osSemaphoreRelease(adcSemID);
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  UNUSED(hadc);

  if(osSemaphoreAcquire(adcSemID, 0) == osOK) {
      memcpy(adcConv, _adcConv+100, sizeof(uint16_t)*100);
      osSemaphoreRelease(adcSemID);
  }
}


#if defined(DEBUG) && !defined(OS_USE_SEMIHOSTING)
FRESULT scan_files (TCHAR* path) {
    FRESULT res;
    DIR dir;
    UINT i;
    static FILINFO fno;
    TCHAR *fname;

    res = f_opendir(&dir, path); /* Open the directory */
    if (res == FR_OK) {
        for (;;) {
            /* Read a directory item */
            res = f_readdir(&dir, &fno);
            /* Break on error or end of directory */
            if (res != FR_OK || fno.fname[0] == 0) break;
#if _USE_LFN > 0
            fname = fno.fname;
#endif
            if (fno.fattrib & AM_DIR) { /* It is a directory */
                i = strlen(path);
                sprintf(&path[i], "/%s", fname);
                /* Scan directory recursively */
                res = scan_files(path);
                if (res != FR_OK) break;
                path[i] = 0;
            } else { /* It is a file. */
                printf("%s/%s\r\n", path, fname);
            }
        }
        f_closedir(&dir);
    }

    return res;
}
#endif //#ifdef DEBUG


void Error_Handler(void) {
#ifdef DEBUG
  asm("BKPT #0");
#else
  while(1) {
    HAL_Delay(250);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  }
#endif
}

#ifdef DEBUG

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed portCHAR *pcTaskName ) {
  UNUSED(pxTask);UNUSED(pcTaskName);

  Error_Handler();
}

#endif


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
