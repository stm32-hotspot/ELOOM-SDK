/**
 ******************************************************************************
 * @file    App.c
 * @author  SRA - GPM
 * 
 * 
 * @brief   Define the Application main entry points
 *
 * ## Introduction
 *
 * This file is the main entry point for the user code.
 *
 * The framework `weak` functions are redefined in this file and they link
 * the application specific code with the framework:
 * - SysLoadApplicationContext(): it is the first application defined function
 *   called by the framework. Here we define all managed tasks. A managed task
 *   implements one or more application specific feature.
 * - SysOnStartApplication(): this function is called by the framework
 *   when the system is initialized (all managed task objects have been
 *   initialized), and before the INIT task release the control. Here we
 *   link the application objects according to the application design.
 *
 * The execution time  between the two above functions is called
 * *system initialization*. During this period only the INIT task is running.
 *
 * Each managed task will be activated in turn to initialize its hardware
 * resources, if any - MyTask_vtblHardwareInit() - and its software
 * resources - MyTask_vtblOnCreateTask().
 *
 * ## About this demo
 *
 * This is the most simple eLooM application. It shows how to setup an eLooM project.
 * It create one managed task (::HelloWorldTask_t). The task display
 * a greeting message in the debug log every second.
 *
 * ## How to use the demo
 *
 * connect the board through the ST-Link. Open a terminal
 * like [Tera Term](http://www.teraterm.org) to display the debug log using these parameters:
 * - Speed       : 115200
 * - Data        : 8 bit
 * - Parity      : none
 * - Stop bits   : 1
 * - Flow control: none
 *
 * Build the project and program the board.
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

#include "services/sysdebug.h"
#include "services/ApplicationContext.h"
#include "AppPowerModeHelper.h"
#include "HelloWorldTask.h"

#include "mx.h"

/**
 * HelloWorld task object.
 */
static AManagedTask *spHElloWorldObj = NULL;

/* eLooM framework entry points definition */
/*******************************************/

sys_error_code_t SysLoadApplicationContext(ApplicationContext *pAppContext)
{
  assert_param(pAppContext);
  sys_error_code_t xRes = SYS_NO_ERROR_CODE;

  // Allocate the task objects
  spHElloWorldObj = HelloWorldTaskAlloc(&MX_GPIO_UB1InitParams);

  // Add the task object to the context.
  xRes = ACAddTask(pAppContext, spHElloWorldObj);

  return xRes;
}

sys_error_code_t SysOnStartApplication(ApplicationContext *pAppContext) {
  UNUSED(pAppContext);

  return SYS_NO_ERROR_CODE;
}

/*IApplicationErrorDelegate *SysGetErrorDelegate(void)
 * {
  // Install the application error manager delegate.
  static IApplicationErrorDelegate *spErrDelegate = NULL;
  if (spErrDelegate == NULL)
  {
    spErrDelegate = AEMAlloc();
  }

  return spErrDelegate;
}*/

IAppPowerModeHelper *SysGetPowerModeHelper(void)
{
  // Install the application power mode helper.
  static IAppPowerModeHelper *spPowerModeHelper = NULL;
  if (spPowerModeHelper == NULL) {
    spPowerModeHelper = AppPowerModeHelperAlloc();
  }

  return spPowerModeHelper;
}

/* Private function definition */
/*******************************/

