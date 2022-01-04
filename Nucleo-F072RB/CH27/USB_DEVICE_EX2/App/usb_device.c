/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usb_device.c
  * @version        : v2.0_Cube
  * @brief          : This file implements the USB Device
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "../../USB_DEVICE_EX2/App/usb_device.h"

#include "usbd_core.h"
#include "usbd_customhid.h"

#include "../../USB_DEVICE_EX2/App/usbd_custom_hid_if.h"
#include "../../USB_DEVICE_EX2/App/usbd_desc.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t B1StatusChanged = 0;
/* USER CODE END PV */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USB Device Core handle declaration. */
USBD_HandleTypeDef hUsbDeviceFS;

/*
 * -- Insert your variables declaration here --
 */
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*
 * -- Insert your external function declaration here --
 */
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * Init USB device Library, add supported class and start the library
  * @retval None
  */
void MX_USB_DEVICE_Init(void)
{
  /* USER CODE BEGIN USB_DEVICE_Init_PreTreatment */

  /* USER CODE END USB_DEVICE_Init_PreTreatment */

  /* Init Device Library, add supported class and start the library. */
  if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CUSTOM_HID) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_CUSTOM_HID_RegisterInterface(&hUsbDeviceFS, &USBD_CustomHID_fops_FS) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_Start(&hUsbDeviceFS) != USBD_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN USB_DEVICE_Init_PostTreatment */
  uint8_t report[4], reportLen;

//  while(1) {
//	  if(B1StatusChanged) {
//		  B1StatusChanged = 0;
//		  USBD_CustomHID_fops_FS.InEvent(report, &reportLen);
//	      USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, report, reportLen);
//	  }
//	  HAL_Delay(200);
//  }

  /* USER CODE END USB_DEVICE_Init_PostTreatment */
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if(GPIO_Pin == B1_Pin) {
		  uint8_t report[4], reportLen;
		  USBD_CustomHID_fops_FS.GetData(report, &reportLen);
	      USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, report, reportLen);
		  HAL_Delay(200);
	}
}


/**
  * @}
  */

/**
  * @}
  */

