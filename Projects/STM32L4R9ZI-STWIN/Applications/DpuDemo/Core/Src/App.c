/**
 ******************************************************************************
 * @file    App.c
 * @author  SRA - GPM
 * 
 * 
 *
 * @brief   Define the Application main entry points
 *
 * The framework `weak` function are redefined in this file and they link
 * the application specific code with the framework.
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

#include "services/ApplicationContext.h"
#include "AppPowerModeHelper.h"
#include "HelloWorldTask.h"
#include "ProcessTask.h"
#include "SPIBusTask.h"
#include "ISM330DHCXTask.h"
#include "SCLIUtilTask.h"
#include "mx.h"



/**
 * HelloWorld task object.
 */
static AManagedTask *spHelloWorldObj = NULL;

/**
 * Processing task. It process the data data coming from the sensor using a DPU.
 */
static AManagedTaskEx *spProcessTask = NULL;

/**
 * SPI bus task object.
 */
static AManagedTaskEx *spSPIBusObj = NULL;

/**
 * Sensor task object.
 */
static AManagedTaskEx *spISM330DHCXObj = NULL;
/**
 * Utility task object.
 */
static AManagedTaskEx *spUtilObj = NULL;


/* eLooM framework entry points definition */
/*******************************************/

sys_error_code_t SysLoadApplicationContext(ApplicationContext *pAppContext)
{
  assert_param(pAppContext);
  sys_error_code_t xRes = SYS_NO_ERROR_CODE;

  // Allocate the task objects
  spSPIBusObj = SPIBusTaskAlloc(&MX_SPI3InitParams);
  spISM330DHCXObj = ISM330DHCXTaskAlloc(&MX_GPIO_PE8InitParams, NULL, &MX_GPIO_PF13InitParams);
  spUtilObj = SCLIUtilTaskAlloc(&MX_GPIO_PE1InitParams, NULL);
  spProcessTask = ProcessTaskAlloc();
  spHelloWorldObj = HelloWorldTaskAlloc(&MX_GPIO_PE0InitParams);

  // Add the task object to the context.
  xRes = ACAddTask(pAppContext, (AManagedTask*)spSPIBusObj);
  xRes = ACAddTask(pAppContext, (AManagedTask*)spISM330DHCXObj);
  xRes = ACAddTask(pAppContext, (AManagedTask*)spUtilObj);
  xRes = ACAddTask(pAppContext, (AManagedTask*)spProcessTask);
  xRes = ACAddTask(pAppContext, spHelloWorldObj);

  return xRes;
}


sys_error_code_t SysOnStartApplication(ApplicationContext *pAppContext) {
  UNUSED(pAppContext);

  /* Disable the automatic low power mode timer */
  SCLIUtilTaskSetAutoLowPowerModePeriod((SCLIUtilTask_t*)spUtilObj, 0);

  /* connect the sensors task to the SPI bus. */
  SPIBusTaskConnectDevice((SPIBusTask*)spSPIBusObj, (SPIBusIF*)ISM330DHCXTaskGetSensorIF((ISM330DHCXTask*)spISM330DHCXObj));
  /* for simplicity, in this demo, the configuration of the sensors is done in the virtual function
   * ProcessTask::OnEnterTaskControlLoop() -> ProcessTask_vtblOnEnterTaskControlLoop()
   */

  /* connect the HelloWord task object to the Process task.*/
  ProcessTaskAddDPUListener((ProcessTask_t*)spProcessTask, HelloWorldTaskGetDataEvtListenerIF((HelloWorldTask*)spHelloWorldObj));

  return SYS_NO_ERROR_CODE;
}

/*IApplicationErrorDelegate *SysGetErrorDelegate(void)
 * {
  // Install the application error manager delegate.
  static IApplicationErrorDelegate *s_pxErrDelegate = NULL;
  if (s_pxErrDelegate == NULL)
  {
    s_pxErrDelegate = AEMAlloc();
  }

  return s_pxErrDelegate;
}*/

IAppPowerModeHelper *SysGetPowerModeHelper(void)
{
  // Install the application power mode helper.
  static IAppPowerModeHelper *s_pxPowerModeHelper = NULL;
  if (s_pxPowerModeHelper == NULL) {
    s_pxPowerModeHelper = AppPowerModeHelperAlloc();
  }

  return s_pxPowerModeHelper;
}



