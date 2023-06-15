/**
 ******************************************************************************
 * @file    ABusIF.c
 * @author  SRA - MCD
 * @version 1.1.0
 * @date    10-Dec-2021
 *
 * @brief
 *
 * <DESCRIPTIOM>
 *
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
 *
 ******************************************************************************
 */

#include "ABusIF.h"

int32_t ABusIFNullRW(void *pxSensor, uint8_t nRegAddr, uint8_t *pnData, uint16_t nSize)
{
  UNUSED(pxSensor);
  UNUSED(nRegAddr);
  UNUSED(pnData);
  UNUSED(nSize);

  return 0;
}
