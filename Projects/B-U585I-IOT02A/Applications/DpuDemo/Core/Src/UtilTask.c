/**
 ******************************************************************************
 * @file    UtilTask.c
 * @author  SRA - GPM
 * 
 * 
 *
 * @brief  UtilTask_t definition.
 *
 *
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

#include "UtilTask.h"
#include "UtilTask_vtbl.h"
#include "app_messages_parser.h"
#include "mx.h"
#include "services/sysdebug.h"
#include "services/sysmem.h"


#ifndef UTIL_TASK_CFG_STACK_DEPTH
#define UTIL_TASK_CFG_STACK_DEPTH              TX_MINIMUM_STACK*4
#endif

#ifndef UTIL_TASK_CFG_PRIORITY
#define UTIL_TASK_CFG_PRIORITY                 (7)
#endif

#ifndef UTIL_TASK_CFG_IN_QUEUE_ITEM_SIZE
#define UTIL_TASK_CFG_IN_QUEUE_ITEM_SIZE       sizeof(struct utilMessage_t)
#endif

#ifndef UTIL_TASK_CFG_IN_QUEUE_ITEM_COUNT
#define UTIL_TASK_CFG_IN_QUEUE_ITEM_COUNT      10
#endif

#ifndef UTIL_TASK_CFG_LP_TIMER_PERIOD_MS
#define UTIL_TASK_CFG_LP_TIMER_PERIOD_MS       300000
#endif

#define SYS_DEBUGF(level, message)             SYS_DEBUGF3(SYS_DBG_UTIL, level, message)

#if defined(DEBUG) || defined (SYS_DEBUG)
#define sTaskObj                               sUtilTaskObj
#endif

/**
 * Class object declaration. The class object encapsulates members that are shared between
 * all instance of the class.
 */
typedef struct _UtilTaskClass_t
{
  /**
   * UtilTask class virtual table.
   */
  AManagedTaskEx_vtbl vtbl;

  /**
   * UtilTask class (PM_STATE, ExecuteStepFunc) map. The map is implemente with an array and
   * the key is the index. Number of items of this array must be equal to the number of PM state
   * of the application. If the managed task does nothing in a PM state, then set to NULL the
   * relative entry in the map.
   */
  pExecuteStepFunc_t p_pm_state2func_map[];
} UtilTaskClass_t;

/* Private member function declaration */
/***************************************/

/**
 * Execute one step of the task control loop while the system is in STATE1.
 *
 * @param _this [IN] specifies a pointer to a task object.
 * @return SYS_NO_EROR_CODE if success, a task specific error code otherwise.
 */
static sys_error_code_t UtilTaskExecuteStepState1(AManagedTask *_this);

/**
 * Execute one step of the task control loop while the system is in SENSORS_ACTIVE.
 *
 * @param _this [IN] specifies a pointer to a task object.
 * @return SYS_NO_EROR_CODE if success, a task specific error code otherwise.
 */
static sys_error_code_t UtilTaskExecuteStepSensorsActive(AManagedTask *_this);

/**
 * Execute one step of the task control loop while the system is in STARTING.
 *
 * @param _this [IN] specifies a pointer to a task object.
 * @return SYS_NO_EROR_CODE if success, a task specific error code otherwise.
 */
static sys_error_code_t UtilTaskExecuteStepStarting(AManagedTask *_this);

/**
 * Callback function called when the software timer expires.
 *
 * @param xTimer [IN] specifies the handle of the expired timer.
 */
static VOID UtilTaskSwTimerCallbackFunction(ULONG timer);

/**
 * Initialize the BatteryCharger
 *
 */
static void PB_PWR_Init(void);

/* Inline function forward declaration */
/***************************************/

#if defined (__GNUC__) || defined (__ICCARM__)
/* Inline function defined inline in the header file UtilTask.h must be declared here as extern function. */
#endif

/**
 * The only instance of the task object.
 */
static UtilTask_t sTaskObj;

/**
 * The class object.
 */
static const UtilTaskClass_t sTheClass = {
    /* Class virtual table */
    {
        UtilTask_vtblHardwareInit,
        UtilTask_vtblOnCreateTask,
        UtilTask_vtblDoEnterPowerMode,
        UtilTask_vtblHandleError,
        UtilTask_vtblOnEnterTaskControlLoop,
        UtilTask_vtblForceExecuteStep,
        UtilTask_vtblOnEnterPowerMode
    },

    /* class (PM_STATE, ExecuteStepFunc) map */
    {
        UtilTaskExecuteStepState1,
        NULL,
        UtilTaskExecuteStepSensorsActive,
        UtilTaskExecuteStepStarting,
    }
};

/* Public API definition */
/*************************/

AManagedTaskEx* UtilTaskAlloc(const void *p_mx_drv_cfg)
{
  /* In this application there is only one Keyboard task,
   * so this allocator implement the singleton design pattern.
   */

  /* Initialize the super class */
  AMTInitEx(&sTaskObj.super);

  sTaskObj.super.vptr = &sTheClass.vtbl;
  sTaskObj.p_mx_drv_cfg = p_mx_drv_cfg;

  return (AManagedTaskEx*) &sTaskObj;
}

SMUtilityDriver_t* GetSMUtilityDriver(void)
{
  return (SMUtilityDriver_t*) sTaskObj.p_driver;
}

/* AManagedTask virtual functions definition */
/*********************************************/

sys_error_code_t UtilTask_vtblHardwareInit(AManagedTask *_this, void *p_params)
{
  assert_param(_this != NULL);
  sys_error_code_t res = SYS_NO_ERROR_CODE;
  UtilTask_t *p_obj = (UtilTask_t*) _this;
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };

  p_obj->p_driver = SMUtilityDriverAlloc();
  if(p_obj->p_driver == NULL)
  {
    SYS_DEBUGF(SYS_DBG_LEVEL_SEVERE, ("UTIL task: unable to alloc driver object.\r\n"));
    res = SYS_GET_LAST_LOW_LEVEL_ERROR_CODE();
  }
  else
  {
    SMUtilityDriverParams_t cfg_params = { .p_mx_tim_cfg = (void*) p_obj->p_mx_drv_cfg };
    res = IDrvInit(p_obj->p_driver, &cfg_params);
    if(SYS_IS_ERROR_CODE(res))
    {
      SYS_DEBUGF(SYS_DBG_LEVEL_SEVERE, ("UTIL task: error during driver initialization\r\n"));
    }
  }

  if(!SYS_IS_ERROR_CODE(res))
  {
    /* Initialize the LED and User Button */
    // configure LED2
    __HAL_RCC_GPIOH_CLK_ENABLE();
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = LED2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

    // configure LED1
    __HAL_RCC_GPIOH_CLK_ENABLE();
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = LED1_Pin;
    HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

    // configure the push button
    __HAL_RCC_GPIOE_CLK_ENABLE();
    GPIO_InitStruct.Pin = USER_BUTTON_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(USER_BUTTON_EXTI_IRQn, 15, 0);

    // configure the BatteryCharger
    PB_PWR_Init();
    // BSP_BC_Init();
  }

  return res;
}

sys_error_code_t UtilTask_vtblOnCreateTask(AManagedTask *_this, tx_entry_function_t *pTaskCode, CHAR **pName,
VOID **pvStackStart,
                                           ULONG *pStackDepth, UINT *pPriority, UINT *pPreemptThreshold, ULONG *pTimeSlice, ULONG *pAutoStart, ULONG *pParams)
{
  assert_param(_this != NULL);
  sys_error_code_t res = SYS_NO_ERROR_CODE;
  UtilTask_t *p_obj = (UtilTask_t*) _this;

  // Create task specific sw resources.

  uint16_t nItemSize = UTIL_TASK_CFG_IN_QUEUE_ITEM_SIZE;
  VOID *pvQueueItemsBuff = SysAlloc(UTIL_TASK_CFG_IN_QUEUE_ITEM_COUNT * nItemSize);
  if(pvQueueItemsBuff == NULL)
  {
    res = SYS_UTIL_TASK_INIT_ERROR_CODE;
    SYS_SET_SERVICE_LEVEL_ERROR_CODE(res);
    return res;
  }

  if(TX_SUCCESS != tx_queue_create(&p_obj->in_queue, "UTIL_Q", nItemSize / 4, pvQueueItemsBuff, UTIL_TASK_CFG_IN_QUEUE_ITEM_COUNT * nItemSize))
  {
    res = SYS_UTIL_TASK_INIT_ERROR_CODE;
    SYS_SET_SERVICE_LEVEL_ERROR_CODE(res);
    return res;
  }

  /* create the software timer*/
  if(TX_SUCCESS
      != tx_timer_create(&p_obj->auto_low_power_timer, "UTIL_T", UtilTaskSwTimerCallbackFunction, 0, AMT_MS_TO_TICKS(UTIL_TASK_CFG_LP_TIMER_PERIOD_MS), 0,
                         TX_NO_ACTIVATE))
  {
    res = SYS_UTIL_TASK_INIT_ERROR_CODE;
    SYS_SET_SERVICE_LEVEL_ERROR_CODE(res);
    return res;
  }

  /* set the (PM_STATE, ExecuteStepFunc) map from the class object.  */
  _this->m_pfPMState2FuncMap = sTheClass.p_pm_state2func_map;

  *pTaskCode = AMTExRun;
  *pName = "UTIL";
  *pvStackStart = NULL; // allocate the task stack in the system memory pool.
  *pStackDepth = UTIL_TASK_CFG_STACK_DEPTH;
  *pParams = (ULONG) _this;
  *pPriority = UTIL_TASK_CFG_PRIORITY;
  *pPreemptThreshold = UTIL_TASK_CFG_PRIORITY;
  *pTimeSlice = TX_NO_TIME_SLICE;
  *pAutoStart = TX_AUTO_START;

  return res;
}

sys_error_code_t UtilTask_vtblDoEnterPowerMode(AManagedTask *_this, const EPowerMode active_power_mode, const EPowerMode new_power_mode)
{
  assert_param(_this != NULL);
  sys_error_code_t res = SYS_NO_ERROR_CODE;
  UtilTask_t *p_obj = (UtilTask_t*) _this;

  /* propagate the call to the driver object. */
  IDrvDoEnterPowerMode(p_obj->p_driver, active_power_mode, new_power_mode);

  if(new_power_mode == E_POWER_MODE_STATE1)
  {
    /* turn on the USER LED */
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

    struct utilMessage_t msg = { .msgId = APP_MESSAGE_ID_UTIL, .nCmdID = UTIL_CMD_ID_START_LP_TIMER };

    if(TX_SUCCESS != tx_queue_front_send(&p_obj->in_queue, &msg, AMT_MS_TO_TICKS(150)))
    {
      res = SYS_TASK_QUEUE_FULL_ERROR_CODE;
    }
  }
  else if(new_power_mode == E_POWER_MODE_SENSORS_ACTIVE || (new_power_mode == E_POWER_MODE_STARTING))
  {
    if(TX_SUCCESS != tx_timer_deactivate(&p_obj->auto_low_power_timer))
    {
      SYS_SET_SERVICE_LEVEL_ERROR_CODE(SYS_UTIL_TASK_LP_TIMER_ERROR_CODE);
      res = SYS_UTIL_TASK_LP_TIMER_ERROR_CODE;
    }
  }
  else if(new_power_mode == E_POWER_MODE_SLEEP_1)
  {
    /* turn off the USER LED */
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

    if(TX_SUCCESS != tx_timer_deactivate(&p_obj->auto_low_power_timer))
    {
      SYS_SET_SERVICE_LEVEL_ERROR_CODE(SYS_UTIL_TASK_LP_TIMER_ERROR_CODE);
      res = SYS_UTIL_TASK_LP_TIMER_ERROR_CODE;
    }
  }

  SYS_DEBUGF(SYS_DBG_LEVEL_VERBOSE, ("UTIL: -> %d\r\n", (uint8_t)new_power_mode));

  return res;
}

sys_error_code_t UtilTask_vtblHandleError(AManagedTask *_this, SysEvent error)
{
  assert_param(_this != NULL);
  sys_error_code_t res = SYS_NO_ERROR_CODE;
  /*  UtilTask_t *p_obj = (UtilTask_t*)_this; */

  return res;
}

sys_error_code_t UtilTask_vtblOnEnterTaskControlLoop(AManagedTask *_this)
{
  assert_param(_this);
  sys_error_code_t xRes = SYS_NO_ERROR_CODE;
  /*  UtilTask_t *p_obj = (UtilTask_t*)_this; */

  SYS_DEBUGF(SYS_DBG_LEVEL_VERBOSE, ("UTIL: start.\r\n"));

  /* enable the user button */
  HAL_NVIC_EnableIRQ(USER_BUTTON_EXTI_IRQn);
//  UtilTaskSetLED(NULL, TRUE);

  return xRes;
}

/* AManagedTaskEx virtual functions definition */
/***********************************************/

sys_error_code_t UtilTask_vtblForceExecuteStep(AManagedTaskEx *_this, EPowerMode active_power_mode)
{
  assert_param(_this != NULL);
  sys_error_code_t res = SYS_NO_ERROR_CODE;
  UtilTask_t *p_obj = (UtilTask_t*) _this;

  struct utilMessage_t msg = { .msgId = APP_REPORT_ID_FORCE_STEP };

  if(active_power_mode == E_POWER_MODE_STATE1)
  {
    if(TX_SUCCESS != tx_queue_front_send(&p_obj->in_queue, &msg, AMT_MS_TO_TICKS(100)))
    {
      res = SYS_TASK_QUEUE_FULL_ERROR_CODE;
    }
  }
  else if(active_power_mode == E_POWER_MODE_SENSORS_ACTIVE || (active_power_mode == E_POWER_MODE_STARTING))
  {
    tx_thread_wait_abort(&_this->m_xThaskHandle);
  }
  else
  {
    tx_thread_resume(&_this->m_xThaskHandle);
  }

  return res;
}

sys_error_code_t UtilTask_vtblOnEnterPowerMode(AManagedTaskEx *_this, const EPowerMode active_power_mode, const EPowerMode new_power_mode)
{
  assert_param(_this != NULL);
  sys_error_code_t res = SYS_NO_ERROR_CODE;
  /*  UtilTask_t *p_obj = (UtilTask_t*)_this; */

  return res;
}

/* Private function definition */
/*******************************/

static sys_error_code_t UtilTaskExecuteStepState1(AManagedTask *_this)
{
  assert_param(_this != NULL);
  sys_error_code_t res = SYS_NO_ERROR_CODE;
  UtilTask_t *p_obj = (UtilTask_t*) _this;

  struct utilMessage_t msg = { 0 };

  AMTExSetInactiveState((AManagedTaskEx*) _this, TRUE);
  if(TX_SUCCESS == tx_queue_receive(&p_obj->in_queue, &msg, TX_WAIT_FOREVER))
  {
    AMTExSetInactiveState((AManagedTaskEx*) _this, FALSE);
    if(msg.msgId == APP_REPORT_ID_FORCE_STEP)
    {
      __NOP();
    }
    else if(msg.msgId == APP_MESSAGE_ID_UTIL)
    {
      if(msg.nCmdID == UTIL_CMD_ID_START_LP_TIMER)
      {
        if(TX_SUCCESS != tx_timer_deactivate(&p_obj->auto_low_power_timer))
        {
          if(TX_SUCCESS != tx_timer_activate(&p_obj->auto_low_power_timer))
          {
            SYS_SET_SERVICE_LEVEL_ERROR_CODE(SYS_UTIL_TASK_LP_TIMER_ERROR_CODE);
            res = SYS_UTIL_TASK_LP_TIMER_ERROR_CODE;
          }
        }
      }
    }
  }

  return res;
}

static sys_error_code_t UtilTaskExecuteStepSensorsActive(AManagedTask *_this)
{
  assert_param(_this);
  sys_error_code_t res = SYS_NO_ERROR_CODE;
  /*UtilTask_t *p_obj = (UtilTask_t*)_this;*/

  AMTExSetInactiveState((AManagedTaskEx*) _this, TRUE);
  tx_thread_sleep(AMT_MS_TO_TICKS(1000));
  AMTExSetInactiveState((AManagedTaskEx*) _this, FALSE);
  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

  return res;
}

static sys_error_code_t UtilTaskExecuteStepStarting(AManagedTask *_this)
{
  assert_param(_this);
  sys_error_code_t res = SYS_NO_ERROR_CODE;
  /*UtilTask_t *p_obj = (UtilTask_t*)_this;*/

  AMTExSetInactiveState((AManagedTaskEx*) _this, TRUE);
  tx_thread_sleep(AMT_MS_TO_TICKS(1000));
  AMTExSetInactiveState((AManagedTaskEx*) _this, FALSE);
  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

  return res;
}

static VOID UtilTaskSwTimerCallbackFunction(ULONG timer)
{
  SYS_DEBUGF(SYS_DBG_LEVEL_VERBOSE, ("UTIL: lp timer expired.\r\n"));

  SysEvent event;
  event.nRawEvent = SYS_PM_MAKE_EVENT(SYS_PM_EVT_SRC_LP_TIMER, SYS_PM_EVT_PARAM_ENTER_LP);
  SysPostPowerModeEvent(event);
}

/**
 * @brief  Initialize the Power button PWR.
 * @param  None
 * @retval None
 */
static void PB_PWR_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* Configure Button pin as input with External interrupt */
  GPIO_InitStruct.Pin = BUTTON_PWR_Pin;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;

  HAL_GPIO_Init(BUTTON_PWR_GPIO_Port, &GPIO_InitStruct);

  /* Enable and set Button EXTI Interrupt to the lowest priority */
  HAL_NVIC_SetPriority((IRQn_Type) BUTTON_PWR_EXTI_IRQn, 3, 0x00);
  HAL_NVIC_EnableIRQ((IRQn_Type) BUTTON_PWR_EXTI_IRQn);

}

/* CubeMX integration */
/**********************/

void Util_USR_EXTI_Callback(uint16_t pin)
{
  /* anti debounch */
  static uint32_t t_start = 0;
  if(HAL_GetTick() - t_start > 1000)
  {
    if(pin == USER_BUTTON_Pin)
    {
      /* generate the system event.*/
      SysEvent evt = { .nRawEvent = SYS_PM_MAKE_EVENT(SYS_PM_EVT_SRC_PB, SYS_PM_EVT_PARAM_SHORT_PRESS) };
      SysPostPowerModeEvent(evt);
      /* don't check the error code. For the moment we assume that we can loose a USER BUTTON PRessed event.*/
    }
  }

  t_start = HAL_GetTick();
}

void Util_PWR_EXTI_Callback(uint16_t nPin)
{

  /* anti debounch */
  static uint32_t t_start = 0;
  if(HAL_GetTick() - t_start > 1000)
  {
    if(nPin == BUTTON_PWR_Pin)
    {
//      BSP_BC_CmdSend(SHIPPING_MODE_ON);
    }
  }

  t_start = HAL_GetTick();
}

