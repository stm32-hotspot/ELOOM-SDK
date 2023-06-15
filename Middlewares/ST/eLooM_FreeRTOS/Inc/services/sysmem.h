/**
 ******************************************************************************
 * @file    sysmem.h
 * @author  STMicroelectronics - AIS - MCD Team
 * @version 3.0.4
 * @date    May 20, 2022
 *
 * @brief   System memory management.
 *
 * This file declares API function to alloc and release block of memory.
 * It remaps the eLooM functions into the ones provided by FreeRTOS.
 * The application can use its own memory allocation strategy.
 *
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file in
 * the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 ******************************************************************************
 */
#ifndef INCLUDE_SERVICES_SYSMEM_H_
#define INCLUDE_SERVICES_SYSMEM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "services/systp.h"
#include "services/systypes.h"
#include "FreeRTOS.h"
#include "task.h"

static inline  void *SysAlloc(size_t nSize);
static inline void SysFree(void *pvData);

/**
 * Allocate a block of memory of a specific size.
 *
 * @param nSize [IN] specifies the size in byte of the requested memory.
 * @return a pointer to the allocated memory if success, NULL otherwise.
 */
static inline
void *SysAlloc(size_t nSize) {
  return pvPortMalloc(nSize);
}

/**
 * Release a block of memory.
 *
 * @param pvData [IN] specifies the start of teh block of memory to release.
 */
static inline
void SysFree(void *pvData) {
  vPortFree(pvData);
}

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_SERVICES_SYSMEM_H_ */
