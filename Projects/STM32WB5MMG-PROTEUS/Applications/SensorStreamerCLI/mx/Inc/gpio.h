/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
//#include "main.h"

/* USER CODE BEGIN Includes */
#include "mx.h"
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */
/* STBC02 SWSEL, CHG, CEN*/
void MX_GPIO_PE1_Init(void);
void MX_GPIO_PE0_Init(void);
void MX_GPIO_PD12_Init(void);
/* IIS3DWB and ISM330DHCX */
void MX_GPIO_PE3_Init(void);
void MX_GPIO_PA15_Init(void);
void MX_GPIO_PE4_Init(void);
void MX_GPIO_PA4_Init(void);
void MX_GPIO_PB13_Init(void);
void MX_GPIO_PD2_Init(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
