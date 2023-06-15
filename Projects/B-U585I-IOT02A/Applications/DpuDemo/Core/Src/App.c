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

#include "services/sysdebug.h"
#include "services/SysTimestamp.h"
#include "services/ApplicationContext.h"
#include "AppPowerModeHelper.h"
#include "HelloWorldTask.h"
#include "ProcessTask.h"
//#include "UtilTask.h"
#include "mx.h"
#include "ISM330DHCXTask.h"
#include "I2CBusTask.h"

/**
 * Application managed task.
 */
static AManagedTask *sHelloWorldObj = NULL;

/**
 * Processing task. It process the data data coming from the sensor using a DPU.
 */
static AManagedTaskEx *sProcessTask = NULL;

/**
 * I2C bus task object
 */

static AManagedTaskEx *sI2CBusObj = NULL;

/**
 * Sensor task object.
 */
static AManagedTaskEx *sISM330DHCXObj = NULL;

/**
 * Utility task object.
 */
//static AManagedTaskEx *spUtilObj = NULL;

/**
 * specifies the map (PM_APP, PM_SM). It re-map the state of teh application into the state of the Sensor Manager.
 */
//static EPowerMode spAppPMState2SMPMStateMap[] = {
//    E_POWER_MODE_STATE1,
//    E_POWER_MODE_SLEEP_1,
//    E_POWER_MODE_SENSORS_ACTIVE,
//    E_POWER_MODE_RESERVED,
//    E_POWER_MODE_SENSORS_ACTIVE,
//    E_POWER_MODE_SENSORS_ACTIVE
//};


/* Private functions declaration */
/*********************************/

/**
 * Re-map the PM State Machine of the Sensor Manager managed tasks used in the application according to the following map:
 *
 * | App State                      | Sensor Manager State         |
 * | :----------------------------- | ---------------------------: |
 * | E_POWER_MODE_STATE1            | E_POWER_MODE_STATE1          |
 * | E_POWER_MODE_SLEEP_1           | E_POWER_MODE_SLEEP_1         |
 * | E_POWER_MODE_X_CUBE_AI_ACTIVE  | E_POWER_MODE_SENSORS_ACTIVE  |
 * | E_POWER_MODE_NEAI_ACTIVE       | E_POWER_MODE_SENSORS_ACTIVE  |
 *
 * @param pPMState2PMStateMap [IN] specifies the map (PM_APP, PM_SM).
 * @return SYS_NO_ERROR_CODE if success, an error code otherwise.
 */
//static sys_error_code_t SensorManagerStateMachineRemap(EPowerMode *pPMState2PMStateMap);


/* eLooM framework entry points definition */
/*******************************************/

sys_error_code_t SysLoadApplicationContext(ApplicationContext *pAppContext)
{
  assert_param(pAppContext);
  sys_error_code_t xRes = SYS_NO_ERROR_CODE;

  // Allocate the task objects
  sI2CBusObj = I2CBusTaskAlloc(&MX_I2C2InitParams);
  sISM330DHCXObj = ISM330DHCXTaskAlloc(&MX_GPIO_PE11InitParams, NULL, NULL);
  //  sUtilObj = SCLIUtilTaskAlloc(&MX_GPIO_PE1InitParams, NULL);
  sProcessTask = ProcessTaskAlloc();
  sHelloWorldObj = HelloWorldTaskAlloc(&MX_GPIO_PC13InitParams);

  // Add the task object to the context.
  xRes = ACAddTask(pAppContext, sHelloWorldObj);
  xRes = ACAddTask(pAppContext, (AManagedTask*)sI2CBusObj);
  xRes = ACAddTask(pAppContext, (AManagedTask*)sISM330DHCXObj);
  xRes = ACAddTask(pAppContext, (AManagedTask*)sProcessTask);

  /* Add the task object to the context. */
  return xRes;
}

sys_error_code_t SysOnStartApplication(ApplicationContext *pAppContext)
{
  UNUSED(pAppContext);

  /* connect the sensors task to the SPI bus. */
  I2CBusTaskConnectDevice((I2CBusTask*) sI2CBusObj, (I2CBusIF*)ISM330DHCXTaskGetSensorIF((ISM330DHCXTask*)sISM330DHCXObj));

  /* connect the HelloWord task object to the Process task.*/
  ProcessTaskAddDPUListener((ProcessTask_t*)sProcessTask, HelloWorldTaskGetDataEvtListenerIF((HelloWorldTask*)sHelloWorldObj));

  SysTsStart(SysGetTimestampSrv(), TRUE);
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



