/**
  ******************************************************************************
  * @file    ipcc.h
  * @brief   This file contains all the function prototypes for
  *          the ipcc.c file
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
  *
  ******************************************************************************
  */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __IPCC_H__
#define __IPCC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include "mx.h"
/* USER CODE END Includes */

/**
 * It is used to initialize the IPCC.
 */

extern IPCC_HandleTypeDef hipcc;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_IPCC_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __IPCC_H__ */


