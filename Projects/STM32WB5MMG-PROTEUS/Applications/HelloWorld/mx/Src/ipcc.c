/**
  ******************************************************************************
  * @file    ipcc.c
  * @brief   This file provides code for the configuration
  *          of The inter-processor communication controller (IPCC).
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


/* Includes ------------------------------------------------------------------*/
#include "ipcc.h"

/* USER CODE BEGIN 0 */
#define Error_Handler sys_error_handler
void sys_error_handler(void);
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure IPCC                                                              */
/*----------------------------------------------------------------------------*/

IPCC_HandleTypeDef hipcc;

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */





void MX_IPCC_Init(void){

  /* USER CODE BEGIN IPCC_Init 0 */

  /* USER CODE END IPCC_Init 0 */

  /* USER CODE BEGIN IPCC_Init 1 */

  /* USER CODE END IPCC_Init 1 */

  hipcc.Instance = IPCC;
  if (HAL_IPCC_Init(&hipcc) != HAL_OK)
  {
	Error_Handler();
  }

  HAL_NVIC_SetPriority(IPCC_C1_RX_IRQn, 8, 0);
  HAL_NVIC_SetPriority(IPCC_C1_TX_IRQn, 8, 0);

  /* USER CODE BEGIN IPCC_Init 2 */

  /* USER CODE END IPCC_Init 2 */

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */


