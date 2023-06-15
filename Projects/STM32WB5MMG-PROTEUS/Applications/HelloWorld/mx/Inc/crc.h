/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    crc.h
  * @brief   This file contains all the function prototypes for
  *          the crc.c file
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
#ifndef __CRC_H__
#define __CRC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include "mx.h"
/* USER CODE END Includes */

extern CRC_HandleTypeDef hcrc;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_CRC_Init(void);
void HAL_CRC_MspInit(CRC_HandleTypeDef* hcrc);
void HAL_CRC_MspDeInit(CRC_HandleTypeDef* hcrc);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CRC_H__ */


