/**
 ******************************************************************************
 * @file    main.c
 * @author  SRA - GPM
 * 
 * 
 * @brief   Main program body.
 *
 * Main program body.
 *
 *********************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file in
 * the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *********************************************************************************
 */

#include <stdio.h>
#include "services/sysinit.h"
#include "task.h"


// Forward function declaration
// ----------------------------



int main()
{
  // System initialization.
  SysInit(FALSE);

  vTaskStartScheduler();

  while (1);
}
