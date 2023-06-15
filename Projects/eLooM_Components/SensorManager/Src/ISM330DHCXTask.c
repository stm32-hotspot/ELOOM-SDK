/**
 ******************************************************************************
 * @file    ISM330DHCXTask.c
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

#include "ISM330DHCXTask.h"
#include "ISM330DHCXTask_vtbl.h"
#include "SMMessageParser.h"
#include "SensorCommands.h"
#include "SensorManager.h"
#include "SensorRegister.h"
#include "events/IDataEventListener.h"
#include "events/IDataEventListener_vtbl.h"
#include "services/SysTimestamp.h"
#include "ism330dhcx_reg.h"
#include "timers.h"
#include <string.h>
#include <stdlib.h>
#include "services/sysdebug.h"
#include "mx.h"

#ifndef ISM330DHCX_TASK_CFG_STACK_DEPTH
#define ISM330DHCX_TASK_CFG_STACK_DEPTH              120
#endif

#ifndef ISM330DHCX_TASK_CFG_PRIORITY
#define ISM330DHCX_TASK_CFG_PRIORITY                 (tskIDLE_PRIORITY)
#endif

#ifndef ISM330DHCX_TASK_CFG_IN_QUEUE_LENGTH
#define ISM330DHCX_TASK_CFG_IN_QUEUE_LENGTH          20
#endif

#define ISM330DHCX_TASK_CFG_IN_QUEUE_ITEM_SIZE       sizeof(SMMessage)

#ifndef ISM330DHCX_TASK_CFG_TIMER_PERIOD_MS
#define ISM330DHCX_TASK_CFG_TIMER_PERIOD_MS          1000
#endif
#ifndef ISM330DHCX_TASK_CFG_MLC_TIMER_PERIOD_MS
#define ISM330DHCX_TASK_CFG_MLC_TIMER_PERIOD_MS      500
#endif

#define ISM330DHCX_TAG_ACC                           (0x02)

#define SYS_DEBUGF(level, message)                   SYS_DEBUGF3(SYS_DBG_ISM330DHCX, level, message)

#if defined(DEBUG) || defined (SYS_DEBUG)
#define sTaskObj                                     sISM330DHCXTaskObj
#endif

#ifndef HSD_USE_DUMMY_DATA
#define HSD_USE_DUMMY_DATA 0
#endif

#if (HSD_USE_DUMMY_DATA == 1)
static int16_t dummyDataCounter_acc = 0;
static int16_t dummyDataCounter_gyro = 0;
#endif

/**
 *  ISM330DHCXTask internal structure.
 */
struct _ISM330DHCXTask
{
  /**
   * Base class object.
   */
  AManagedTaskEx super;

  // Task variables should be added here.

  /**
   * IRQ GPIO configuration parameters.
   */
  const MX_GPIOParams_t *pIRQConfig;

  /**
   * MLC GPIO configuration parameters.
   */
  const MX_GPIOParams_t *pMLCConfig;

  /**
   * SPI CS GPIO configuration parameters.
   */
  const MX_GPIOParams_t *pCSConfig;

  /**
   * Bus IF object used to connect the sensor task to the specific bus.
   */
  ABusIF *p_sensor_bus_if;

  /**
   * Implements the accelerometer ISensor interface.
   */
  ISensor_t acc_sensor_if;

  /**
   * Implements the gyroscope ISensor interface.
   */
  ISensor_t gyro_sensor_if;

  /**
   * Implements the mlc ISensor interface.
   */
  ISensor_t mlc_sensor_if;

  /**
   * Implements the ISensorMlc interface.
   */
  ISensorMlc_t sensor_mlc_if;

  /**
   * Specifies accelerometer sensor capabilities.
   */
  const SensorDescriptor_t *acc_sensor_descriptor;

  /**
   * Specifies accelerometer sensor configuration.
   */
  SensorStatus_t acc_sensor_status;

  EMData_t data_acc;
  /**
   * Specifies gyroscope sensor capabilities.
   */
  const SensorDescriptor_t *gyro_sensor_descriptor;

  /**
   * Specifies gyroscope sensor configuration.
   */
  SensorStatus_t gyro_sensor_status;

  EMData_t data_gyro;
  /**
   * Specifies mlc sensor capabilities.
   */
  const SensorDescriptor_t *mlc_sensor_descriptor;

  /**
   * Specifies mlc sensor configuration.
   */
  SensorStatus_t mlc_sensor_status;

  EMData_t data;
  /**
   * Specifies the sensor ID for the accelerometer subsensor.
   */
  uint8_t acc_id;

  /**
   * Specifies the sensor ID for the gyroscope subsensor.
   */
  uint8_t gyro_id;

  /**
   * Specifies the sensor ID for the mlc subsensor.
   */
  uint8_t mlc_id;

  /**
   * Specifies mlc status.
   */
  boolean_t mlc_enable;

  /**
   * Synchronization object used to send command to the task.
   */
  QueueHandle_t in_queue;

#ifdef ISM330DHCX_FIFO_ENABLED
  /**
   * Buffer to store the data read from the sensor FIFO.
   * It is reused also to save data from the faster subsensor
   */
  uint8_t p_fast_sensor_data_buff[ISM330DHCX_MAX_SAMPLES_PER_IT * 7];

  /**
   * Buffer to store the data from the slower subsensor
   */
  uint8_t p_slow_sensor_data_buff[ISM330DHCX_MAX_SAMPLES_PER_IT / 2 * 6];
#else
  /**
    * Buffer to store the data read from the sensor FIFO.
    * It is reused also to save data from the faster subsensor
    */
  uint8_t p_acc_sample[6];

  /**
    * Buffer to store the data from the slower subsensor
    */
  uint8_t p_gyro_sample[6];

  /**
    * Save acc data ready status
    */
  uint8_t acc_drdy;

  /**
    * Save gyro data ready status
    */
  uint8_t gyro_drdy;
#endif

  /**
   * Buffer to store the data from mlc
   */
  uint8_t p_mlc_sensor_data_buff[9];

  /**
   * Specifies the FIFO level
   */
  uint16_t fifo_level;

  /**
   * Specifies the FIFO watermark level (it depends from ODR)
   */
  uint16_t samples_per_it;

  /**
   * If both subsensors are active, specifies the amount of ACC samples in the FIFO
   */
  uint16_t acc_samples_count;

  /**
   * If both subsensors are active, specifies the amount of GYRO samples in the FIFO
   */
  uint16_t gyro_samples_count;

  /**
   * ::IEventSrc interface implementation for this class.
   */
  IEventSrc *p_acc_event_src;

  /**
   * ::IEventSrc interface implementation for this class.
   */
  IEventSrc *p_gyro_event_src;

  /**
   * ::IEventSrc interface implementation for this class.
   */
  IEventSrc *p_mlc_event_src;

  /**
   * Software timer used to generate the read command
   */
  TimerHandle_t read_timer;

  /**
   * Timer period used to schedule the read command
   */
  uint16_t ism330dhcx_task_cfg_timer_period_ms;

  /**
   * Software timer used to generate the mlc read command
   */
  TimerHandle_t mlc_timer;

  /**
   * Used to update the instantaneous ODR.
   */
  double prev_timestamp;
};

/**
 * Class object declaration
 */
typedef struct _ISM330DHCXTaskClass
{
  /**
   * ISM330DHCXTask class virtual table.
   */
  AManagedTaskEx_vtbl vtbl;

  /**
   * Accelerometer IF virtual table.
   */
  ISensor_vtbl acc_sensor_if_vtbl;

  /**
   * Gyro IF virtual table.
   */
  ISensor_vtbl gyro_sensor_if_vtbl;

  /**
   * mlc IF virtual table.
   */
  ISensor_vtbl mlc_sensor_if_vtbl;

  /**
   * SensorMlc IF virtual table.
   */
  ISensorMlc_vtbl sensor_mlc_if_vtbl;

  /**
   * Specifies accelerometer sensor capabilities.
   */
  SensorDescriptor_t acc_class_descriptor;

  /**
   * Specifies gyroscope sensor capabilities.
   */
  SensorDescriptor_t gyro_class_descriptor;

  /**
   * Specifies mlc sensor capabilities.
   */
  SensorDescriptor_t mlc_class_descriptor;

  /**
   * ISM330DHCXTask (PM_STATE, ExecuteStepFunc) map.
   */
  pExecuteStepFunc_t p_pm_state2func_map[3];
} ISM330DHCXTaskClass_t;

/* Private member function declaration */

/**
 * Execute one step of the task control loop while the system is in RUN mode.
 *
 * @param _this [IN] specifies a pointer to a task object.
 * @return SYS_NO_EROR_CODE if success, a task specific error code otherwise.
 */
static sys_error_code_t ISM330DHCXTaskExecuteStepState1(AManagedTask *_this);

/**
 * Execute one step of the task control loop while the system is in SENSORS_ACTIVE mode.
 *
 * @param _this [IN] specifies a pointer to a task object.
 * @return SYS_NO_EROR_CODE if success, a task specific error code otherwise.
 */
static sys_error_code_t ISM330DHCXTaskExecuteStepDatalog(AManagedTask *_this);

/**
 * Initialize the sensor according to the actual parameters.
 *
 * @param _this [IN] specifies a pointer to a task object.
 * @return SYS_NO_EROR_CODE if success, a task specific error code otherwise.
 */
static sys_error_code_t ISM330DHCXTaskSensorInit(ISM330DHCXTask *_this);

/**
 * Read the data from the sensor.
 *
 * @param _this [IN] specifies a pointer to a task object.
 * @return SYS_NO_EROR_CODE if success, a task specific error code otherwise.
 */
static sys_error_code_t ISM330DHCXTaskSensorReadData(ISM330DHCXTask *_this);

/**
 * Read the data from the mlc.
 *
 * @param _this [IN] specifies a pointer to a task object.
 * @return SYS_NO_EROR_CODE if success, a task specific error code otherwise.
 */
static sys_error_code_t ISM330DHCXTaskSensorReadMLC(ISM330DHCXTask *_this);

/**
 * Register the sensor with the global DB and initialize the default parameters.
 *
 * @param _this [IN] specifies a pointer to a task object.
 * @return SYS_NO_ERROR_CODE if success, an error code otherwise
 */
static sys_error_code_t ISM330DHCXTaskSensorRegister(ISM330DHCXTask *_this);

/**
 * Initialize the default parameters.
 *
 * @param _this [IN] specifies a pointer to a task object.
 * @return SYS_NO_ERROR_CODE if success, an error code otherwise
 */
static sys_error_code_t ISM330DHCXTaskSensorInitTaskParams(ISM330DHCXTask *_this);

/**
 * Private implementation of sensor interface methods for ISM330DHCX sensor
 */
static sys_error_code_t ISM330DHCXTaskSensorSetODR(ISM330DHCXTask *_this, SMMessage report);
static sys_error_code_t ISM330DHCXTaskSensorSetFS(ISM330DHCXTask *_this, SMMessage report);
static sys_error_code_t ISM330DHCXTaskSensorSetFifoWM(ISM330DHCXTask *_this, SMMessage report);
static sys_error_code_t ISM330DHCXTaskSensorEnable(ISM330DHCXTask *_this, SMMessage report);
static sys_error_code_t ISM330DHCXTaskSensorDisable(ISM330DHCXTask *_this, SMMessage report);

/**
 * Check if the sensor is active. The sensor is active if at least one of the sub sensor is active.
 * @param _this [IN] specifies a pointer to a task object.
 * @return TRUE if the sensor is active, FALSE otherwise.
 */
static boolean_t ISM330DHCXTaskSensorIsActive(const ISM330DHCXTask *_this);

static sys_error_code_t ISM330DHCXTaskEnterLowPowerMode(const ISM330DHCXTask *_this);

static sys_error_code_t ISM330DHCXTaskConfigureIrqPin(const ISM330DHCXTask *_this, boolean_t LowPower);
static sys_error_code_t ISM330DHCXTaskConfigureMLCPin(const ISM330DHCXTask *_this, boolean_t LowPower);

/**
 * Callback function called when the software timer expires.
 *
 * @param timer [IN] specifies the handle of the expired timer.
 */
static void ISM330DHCXTaskTimerCallbackFunction(TimerHandle_t timer);

/**
 * Callback function called when the mlc software timer expires.
 *
 * @param timer [IN] specifies the handle of the expired timer.
 */
static void ISM330DHCXTaskMLCTimerCallbackFunction(TimerHandle_t timer);

/**
 * Given a interface pointer it return the instance of the object that implement the interface.
 *
 * @param p_if [IN] specifies a sensor interface implemented by the task object.
 * @return the instance of the task object that implements the given interface.
 */
static inline ISM330DHCXTask* ISM330DHCXTaskGetOwnerFromISensorIF(ISensor_t *p_if);

/**
 * Given a interface pointer it return the instance of the object that implement the interface.
 *
 * @param p_if [IN] specifies a sensorMlc interface implemented by the task object.
 * @return the instance of the task object that implements the given interface.
 */
static sys_error_code_t ISM330DHCXTaskCreateTimer(ISM330DHCXTask *_this);
static inline ISM330DHCXTask* ISM330DHCXTaskGetOwnerFromISensorMlcIF(ISensorMlc_t *p_if);

/**
 * Interrupt callback
 */
void ISM330DHCXTask_EXTI_Callback(uint16_t Pin);
void INT2_DHCX_EXTI_Callback(uint16_t Pin);

/**
 * Internal function used to update ISM330DHCX properties from ucf
 */
static sys_error_code_t ISM330DHCX_XL_ODR_From_UCF(ISM330DHCXTask *_this);
static sys_error_code_t ISM330DHCX_XL_FS_From_UCF(ISM330DHCXTask *_this);
static sys_error_code_t ISM330DHCX_GY_ODR_From_UCF(ISM330DHCXTask *_this);
static sys_error_code_t ISM330DHCX_GY_FS_From_UCF(ISM330DHCXTask *_this);

/* Inline function forward declaration */

/**
 * Private function used to post a report into the front of the task queue.
 * Used to resume the task when the required by the INIT task.
 *
 * @param this [IN] specifies a pointer to the task object.
 * @param pReport [IN] specifies a report to send.
 * @return SYS_NO_EROR_CODE if success, SYS_APP_TASK_REPORT_LOST_ERROR_CODE.
 */
static inline sys_error_code_t ISM330DHCXTaskPostReportToFront(ISM330DHCXTask *_this, SMMessage *pReport);

/**
 * Private function used to post a report into the back of the task queue.
 *
 * @param this [IN] specifies a pointer to the task object.
 * @param pReport [IN] specifies a report to send.
 * @return SYS_NO_EROR_CODE if success, SYS_APP_TASK_REPORT_LOST_ERROR_CODE.
 */
static inline sys_error_code_t ISM330DHCXTaskPostReportToBack(ISM330DHCXTask *_this, SMMessage *pReport);

#if defined (__GNUC__)
/* Inline function defined inline in the header file ISM330DHCXTask.h must be declared here as extern function. */
#endif

/* Objects instance */
/********************/

/**
 * The only instance of the task object.
 */
static ISM330DHCXTask sTaskObj;

/**
 * The class object.
 */
static const ISM330DHCXTaskClass_t sTheClass =
{
    /* class virtual table */
    {
        ISM330DHCXTask_vtblHardwareInit,
        ISM330DHCXTask_vtblOnCreateTask,
        ISM330DHCXTask_vtblDoEnterPowerMode,
        ISM330DHCXTask_vtblHandleError,
        ISM330DHCXTask_vtblOnEnterTaskControlLoop,
        ISM330DHCXTask_vtblForceExecuteStep,
        ISM330DHCXTask_vtblOnEnterPowerMode },

    /* class::acc_sensor_if_vtbl virtual table */
    {
        ISM330DHCXTask_vtblAccGetId,
        ISM330DHCXTask_vtblAccGetEventSourceIF,
        ISM330DHCXTask_vtblAccGetDataInfo,
        ISM330DHCXTask_vtblAccGetODR,
        ISM330DHCXTask_vtblAccGetFS,
        ISM330DHCXTask_vtblAccGetSensitivity,
        ISM330DHCXTask_vtblSensorSetODR,
        ISM330DHCXTask_vtblSensorSetFS,
        ISM330DHCXTask_vtblSensorSetFifoWM,
        ISM330DHCXTask_vtblSensorEnable,
        ISM330DHCXTask_vtblSensorDisable,
        ISM330DHCXTask_vtblSensorIsEnabled,
        ISM330DHCXTask_vtblAccGetDescription,
        ISM330DHCXTask_vtblAccGetStatus },

    /* class::gyro_sensor_if_vtbl virtual table */
    {
        ISM330DHCXTask_vtblGyroGetId,
        ISM330DHCXTask_vtblGyroGetEventSourceIF,
        ISM330DHCXTask_vtblGyroGetDataInfo,
        ISM330DHCXTask_vtblGyroGetODR,
        ISM330DHCXTask_vtblGyroGetFS,
        ISM330DHCXTask_vtblGyroGetSensitivity,
        ISM330DHCXTask_vtblSensorSetODR,
        ISM330DHCXTask_vtblSensorSetFS,
        ISM330DHCXTask_vtblSensorSetFifoWM,
        ISM330DHCXTask_vtblSensorEnable,
        ISM330DHCXTask_vtblSensorDisable,
        ISM330DHCXTask_vtblSensorIsEnabled,
        ISM330DHCXTask_vtblGyroGetDescription,
        ISM330DHCXTask_vtblGyroGetStatus },

    /* class::mlc_sensor_if_vtbl virtual table */
    {
        ISM330DHCXTask_vtblMlcGetId,
        ISM330DHCXTask_vtblMlcGetEventSourceIF,
        ISM330DHCXTask_vtblMlcGetDataInfo,
        ISM330DHCXTask_vtblMlcGetODR,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL,
        ISM330DHCXTask_vtblSensorEnable,
        ISM330DHCXTask_vtblSensorDisable,
        ISM330DHCXTask_vtblSensorIsEnabled,
        ISM330DHCXTask_vtblMlcGetDescription,
        ISM330DHCXTask_vtblMlcGetStatus },

    /* class::mlc_sensor_if_vtbl virtual table */
    {
        ISM330DHCXTask_vtblSensorMlcLoadUcf,
        ISM330DHCXTask_vtblSensorMlcIsEnabled },

    /* ACCELEROMETER DESCRIPTOR */
    {
        "ism330dhcx",
        COM_TYPE_ACC,
        {
            12.5,
            26,
            52,
            104,
            208,
            416,
            833,
            1666,
            3332,
            6667,
            COM_END_OF_LIST_FLOAT, },
        {
            2,
            4,
            8,
            16,
            COM_END_OF_LIST_FLOAT, },
        {
            "acc", },
        "g" },

    /* GYROSCOPE DESCRIPTOR */
    {
        "ism330dhcx",
        COM_TYPE_GYRO,
        {
            12.5,
            26,
            52,
            104,
            208,
            416,
            833,
            1666,
            3332,
            6667,
            COM_END_OF_LIST_FLOAT, },
        {
            125,
            250,
            500,
            1000,
            2000,
            4000,
            COM_END_OF_LIST_FLOAT, },
        {
            "gyro", },
        "mdps" },

    /* MLC DESCRIPTOR */
    {
        "ism330dhcx",
        COM_TYPE_MLC,
        {
            1,
            COM_END_OF_LIST_FLOAT, },
        {
            1,
            COM_END_OF_LIST_FLOAT, },
        {
            "mlc", },
        "out", },
    /* class (PM_STATE, ExecuteStepFunc) map */
    {
        ISM330DHCXTaskExecuteStepState1,
        NULL,
        ISM330DHCXTaskExecuteStepDatalog, } };

/* Public API definition */

ISourceObservable* ISM330DHCXTaskGetAccSensorIF(ISM330DHCXTask *_this)
{
  return (ISourceObservable*) &(_this->acc_sensor_if);
}

ISourceObservable* ISM330DHCXTaskGetGyroSensorIF(ISM330DHCXTask *_this)
{
  return (ISourceObservable*) &(_this->gyro_sensor_if);
}

ISourceObservable* ISM330DHCXTaskGetMlcSensorIF(ISM330DHCXTask *_this)
{
  return (ISourceObservable*) &(_this->mlc_sensor_if);
}

ISensorMlc_t* ISM330DHCXTaskGetSensorMlcIF(ISM330DHCXTask *_this)
{
  return (ISensorMlc_t*) &(_this->sensor_mlc_if);
}

AManagedTaskEx* ISM330DHCXTaskAlloc(const void *pIRQConfig, const void *pMLCConfig, const void *pCSConfig)
{
  /* This allocator implements the singleton design pattern. */

  /* Initialize the super class */
  AMTInitEx(&sTaskObj.super);

  sTaskObj.super.vptr = &sTheClass.vtbl;
  sTaskObj.acc_sensor_if.vptr = &sTheClass.acc_sensor_if_vtbl;
  sTaskObj.gyro_sensor_if.vptr = &sTheClass.gyro_sensor_if_vtbl;
  sTaskObj.mlc_sensor_if.vptr = &sTheClass.mlc_sensor_if_vtbl;
  sTaskObj.sensor_mlc_if.vptr = &sTheClass.sensor_mlc_if_vtbl;
  sTaskObj.acc_sensor_descriptor = &sTheClass.acc_class_descriptor;
  sTaskObj.gyro_sensor_descriptor = &sTheClass.gyro_class_descriptor;
  sTaskObj.mlc_sensor_descriptor = &sTheClass.mlc_class_descriptor;

  sTaskObj.pIRQConfig = (MX_GPIOParams_t*) pIRQConfig;
  sTaskObj.pMLCConfig = (MX_GPIOParams_t*) pMLCConfig;
  sTaskObj.pCSConfig = (MX_GPIOParams_t*) pCSConfig;

  return (AManagedTaskEx*) &sTaskObj;
}

ABusIF* ISM330DHCXTaskGetSensorIF(ISM330DHCXTask *_this)
{
  assert_param(_this != NULL);

  return _this->p_sensor_bus_if;
}

IEventSrc* ISM330DHCXTaskGetAccEventSrcIF(ISM330DHCXTask *_this)
{
  assert_param(_this != NULL);

  return _this->p_acc_event_src;
}

IEventSrc* ISM330DHCXTaskGetGyroEventSrcIF(ISM330DHCXTask *_this)
{
  assert_param(_this != NULL);

  return _this->p_gyro_event_src;
}

IEventSrc* ISM330DHCXTaskGetMlcEventSrcIF(ISM330DHCXTask *_this)
{
  assert_param(_this != NULL);

  return _this->p_mlc_event_src;
}

/* AManagedTaskEx virtual functions definition */

sys_error_code_t ISM330DHCXTask_vtblHardwareInit(AManagedTask *_this, void *pParams)
{
  assert_param(_this != NULL);
  sys_error_code_t res = SYS_NO_ERROR_CODE;
  ISM330DHCXTask *p_obj = (ISM330DHCXTask*) _this;

  /* Configure CS Pin */
  if(p_obj->pCSConfig != NULL)
  {
    p_obj->pCSConfig->p_mx_init_f();
  }

  return res;
}

sys_error_code_t ISM330DHCXTask_vtblOnCreateTask(AManagedTask *_this, TaskFunction_t *pTaskCode, const char **pName, unsigned short *pStackDepth,
                                                 void **pParams, UBaseType_t *pPriority)
{
  assert_param(_this != NULL);
  sys_error_code_t res = SYS_NO_ERROR_CODE;
  ISM330DHCXTask *p_obj = (ISM330DHCXTask*) _this;

  /* Create task specific sw resources. */

  p_obj->in_queue = xQueueCreate(ISM330DHCX_TASK_CFG_IN_QUEUE_LENGTH, ISM330DHCX_TASK_CFG_IN_QUEUE_ITEM_SIZE);
  if(p_obj->in_queue == NULL)
  {
    res = SYS_TASK_HEAP_OUT_OF_MEMORY_ERROR_CODE;
    SYS_SET_SERVICE_LEVEL_ERROR_CODE(res);
    return res;
  }

#ifdef DEBUG
  vQueueAddToRegistry(p_obj->in_queue, "ISM330DHCX_Q");
#endif

  /* create the software timer */
  res = ISM330DHCXTaskCreateTimer(p_obj);
  if(SYS_IS_ERROR_CODE(res))
  {
    return res;
  }

  /* Alloc the bus interface (SPI if the task is given the CS Pin configuration param, I2C otherwise) */
  if(p_obj->pCSConfig != NULL)
  {
    p_obj->p_sensor_bus_if = SPIBusIFAlloc(ISM330DHCX_ID, p_obj->pCSConfig->port, p_obj->pCSConfig->pin, 0);
    if(p_obj->p_sensor_bus_if == NULL)
    {
      res = SYS_TASK_HEAP_OUT_OF_MEMORY_ERROR_CODE;
      SYS_SET_SERVICE_LEVEL_ERROR_CODE(res);
    }
  }
  else
  {
    p_obj->p_sensor_bus_if = I2CBusIFAlloc(ISM330DHCX_ID, ISM330DHCX_I2C_ADD_H, 0);
    if(p_obj->p_sensor_bus_if == NULL)
    {
      res = SYS_TASK_HEAP_OUT_OF_MEMORY_ERROR_CODE;
      SYS_SET_SERVICE_LEVEL_ERROR_CODE(res);
    }
  }

  if(!SYS_IS_ERROR_CODE(res))
  {
    /* Initialize the EventSrc interface, take the ownership of the interface. */
    p_obj->p_acc_event_src = DataEventSrcAlloc();
    if(p_obj->p_acc_event_src == NULL)
    {
      SYS_SET_SERVICE_LEVEL_ERROR_CODE(SYS_OUT_OF_MEMORY_ERROR_CODE);
      res = SYS_OUT_OF_MEMORY_ERROR_CODE;
    }
    else
    {
      IEventSrcInit(p_obj->p_acc_event_src);

      p_obj->p_gyro_event_src = DataEventSrcAlloc();
      if(p_obj->p_gyro_event_src == NULL)
      {
        SYS_SET_SERVICE_LEVEL_ERROR_CODE(SYS_OUT_OF_MEMORY_ERROR_CODE);
        res = SYS_OUT_OF_MEMORY_ERROR_CODE;
      }
      else
      {
        IEventSrcInit(p_obj->p_gyro_event_src);

        p_obj->p_mlc_event_src = DataEventSrcAlloc();
        if(p_obj->p_mlc_event_src == NULL)
        {
          SYS_SET_SERVICE_LEVEL_ERROR_CODE(SYS_OUT_OF_MEMORY_ERROR_CODE);
          res = SYS_OUT_OF_MEMORY_ERROR_CODE;
        }
        else
        {
          IEventSrcInit(p_obj->p_mlc_event_src);

#ifdef ISM330DHCX_FIFO_ENABLED
          memset(p_obj->p_fast_sensor_data_buff, 0, sizeof(p_obj->p_fast_sensor_data_buff));
          memset(p_obj->p_slow_sensor_data_buff, 0, sizeof(p_obj->p_slow_sensor_data_buff));
#else
          memset(p_obj->p_acc_sample, 0, sizeof(p_obj->p_acc_sample));
          memset(p_obj->p_gyro_sample, 0, sizeof(p_obj->p_gyro_sample));
          p_obj->acc_drdy = 0;
          p_obj->gyro_drdy = 0;
#endif
          memset(p_obj->p_mlc_sensor_data_buff, 0, sizeof(p_obj->p_mlc_sensor_data_buff));
          p_obj->acc_id = 0;
          p_obj->gyro_id = 1;
          p_obj->mlc_enable = FALSE;
          p_obj->prev_timestamp = 0.0f;
          p_obj->acc_samples_count = 0;
          p_obj->gyro_samples_count = 0;
          p_obj->fifo_level = 0;
          p_obj->samples_per_it = 0;
          _this->m_pfPMState2FuncMap = sTheClass.p_pm_state2func_map;

          *pTaskCode = AMTExRun;
          *pName = "ISM330DHCX";
          *pStackDepth = ISM330DHCX_TASK_CFG_STACK_DEPTH;
          *pParams = _this;
          *pPriority = ISM330DHCX_TASK_CFG_PRIORITY;

          res = ISM330DHCXTaskSensorInitTaskParams(p_obj);
          if(SYS_IS_ERROR_CODE(res))
          {
            SYS_SET_SERVICE_LEVEL_ERROR_CODE(SYS_OUT_OF_MEMORY_ERROR_CODE);
            res = SYS_OUT_OF_MEMORY_ERROR_CODE;
          }
          else
          {
            res = ISM330DHCXTaskSensorRegister(p_obj);
            if(SYS_IS_ERROR_CODE(res))
            {
              SYS_DEBUGF(SYS_DBG_LEVEL_VERBOSE, ("ISM330DHCX: unable to register with DB\r\n"));
              sys_error_handler();
            }
          }
        }
      }
    }
  }
  return res;
}

sys_error_code_t ISM330DHCXTask_vtblDoEnterPowerMode(AManagedTask *_this, const EPowerMode ActivePowerMode, const EPowerMode NewPowerMode)
{
  assert_param(_this != NULL);
  sys_error_code_t res = SYS_NO_ERROR_CODE;
  ISM330DHCXTask *p_obj = (ISM330DHCXTask*) _this;
  stmdev_ctx_t *p_sensor_drv = (stmdev_ctx_t*) &p_obj->p_sensor_bus_if->m_xConnector;

  if(NewPowerMode == E_POWER_MODE_SENSORS_ACTIVE)
  {
    if(ISM330DHCXTaskSensorIsActive(p_obj))
    {
      SMMessage report =
      {
          .sensorMessage.messageId = SM_MESSAGE_ID_SENSOR_CMD,
          .sensorMessage.nCmdID = SENSOR_CMD_ID_INIT };

      if(xQueueSendToBack(p_obj->in_queue, &report, pdMS_TO_TICKS(100)) != pdTRUE)
      {
        res = SYS_APP_TASK_REPORT_LOST_ERROR_CODE;
        SYS_SET_SERVICE_LEVEL_ERROR_CODE(SYS_APP_TASK_REPORT_LOST_ERROR_CODE);
      }

      // reset the variables for the time stamp computation.
      p_obj->prev_timestamp = 0.0f;
    }

    SYS_DEBUGF(SYS_DBG_LEVEL_VERBOSE, ("ISM330DHCX: -> SENSORS_ACTIVE\r\n"));
  }
  else if(NewPowerMode == E_POWER_MODE_STATE1)
  {
    if(ActivePowerMode == E_POWER_MODE_SENSORS_ACTIVE)
    {
      /* Deactivate the sensor */
      ism330dhcx_xl_data_rate_set(p_sensor_drv, ISM330DHCX_XL_ODR_OFF);
      ism330dhcx_gy_data_rate_set(p_sensor_drv, ISM330DHCX_GY_ODR_OFF);
      ism330dhcx_fifo_gy_batch_set(p_sensor_drv, ISM330DHCX_GY_NOT_BATCHED);
      ism330dhcx_fifo_xl_batch_set(p_sensor_drv, ISM330DHCX_XL_NOT_BATCHED);
      ism330dhcx_fifo_mode_set(p_sensor_drv, ISM330DHCX_BYPASS_MODE);
      p_obj->samples_per_it = 0;
      xQueueReset(p_obj->in_queue);
      if(p_obj->pIRQConfig == NULL)
      {
        xTimerStop(p_obj->read_timer, pdMS_TO_TICKS(100));
      }
      else
      {
        ISM330DHCXTaskConfigureIrqPin(p_obj, TRUE);
      }
      if(p_obj->pMLCConfig == NULL)
      {
        xTimerStop(p_obj->mlc_timer, pdMS_TO_TICKS(100));
      }
      else
      {
        ISM330DHCXTaskConfigureMLCPin(p_obj, TRUE);
      }
    }
    SYS_DEBUGF(SYS_DBG_LEVEL_VERBOSE, ("ISM330DHCX: -> STATE1\r\n"));
  }
  else if(NewPowerMode == E_POWER_MODE_SLEEP_1)
  {
    /* the MCU is going in stop so I put the sensor in low power
     from the INIT task */
    res = ISM330DHCXTaskEnterLowPowerMode(p_obj);
    if(SYS_IS_ERROR_CODE(res))
    {
      sys_error_handler();
    }
    if(p_obj->pIRQConfig != NULL)
    {
      ISM330DHCXTaskConfigureIrqPin(p_obj, TRUE);
    }
    if(p_obj->pMLCConfig != NULL)
    {
      ISM330DHCXTaskConfigureMLCPin(p_obj, TRUE);
    }
    /* notify the bus */
    if(p_obj->p_sensor_bus_if->m_pfBusCtrl != NULL)
    {
      p_obj->p_sensor_bus_if->m_pfBusCtrl(p_obj->p_sensor_bus_if, E_BUS_CTRL_DEV_NOTIFY_POWER_MODE, 0);
    }

    if(p_obj->pIRQConfig == NULL)
    {
      xTimerStop(p_obj->read_timer, pdMS_TO_TICKS(100));
    }
    if(p_obj->pMLCConfig == NULL)
    {
      xTimerStop(p_obj->mlc_timer, pdMS_TO_TICKS(100));
    }

    SYS_DEBUGF(SYS_DBG_LEVEL_VERBOSE, ("ISM330DHCX: -> SLEEP_1\r\n"));
  }

  return res;
}

sys_error_code_t ISM330DHCXTask_vtblHandleError(AManagedTask *_this, SysEvent Error)
{
  assert_param(_this != NULL);
  sys_error_code_t res = SYS_NO_ERROR_CODE;

  return res;
}

sys_error_code_t ISM330DHCXTask_vtblOnEnterTaskControlLoop(AManagedTask *_this)
{
  assert_param(_this != NULL);
  sys_error_code_t res = SYS_NO_ERROR_CODE;

  SYS_DEBUGF(SYS_DBG_LEVEL_VERBOSE, ("ISM330DHCX: start.\r\n"));

  // At this point all system has been initialized.
  // Execute task specific delayed one time initialization.

  return res;
}

sys_error_code_t ISM330DHCXTask_vtblForceExecuteStep(AManagedTaskEx *_this, EPowerMode ActivePowerMode)
{
  assert_param(_this != NULL);
  sys_error_code_t res = SYS_NO_ERROR_CODE;
  ISM330DHCXTask *p_obj = (ISM330DHCXTask*) _this;

  SMMessage report =
  {
      .internalMessageFE.messageId = SM_MESSAGE_ID_FORCE_STEP,
      .internalMessageFE.nData = 0 };

  if((ActivePowerMode == E_POWER_MODE_STATE1) || (ActivePowerMode == E_POWER_MODE_SENSORS_ACTIVE))
  {
    if(AMTExIsTaskInactive(_this))
    {
      res = ISM330DHCXTaskPostReportToFront(p_obj, (SMMessage*) &report);
    }
    else
    {
      // do nothing and wait for the step to complete.
      //      _this->m_xStatus.nDelayPowerModeSwitch = 0;
    }
  }
  else
  {
    if(eTaskGetState(_this->m_xThaskHandle) == eSuspended)
    {
      vTaskResume(_this->m_xThaskHandle);
    }
  }

  return res;
}

sys_error_code_t ISM330DHCXTask_vtblOnEnterPowerMode(AManagedTaskEx *_this, const EPowerMode ActivePowerMode, const EPowerMode NewPowerMode)
{
  assert_param(_this != NULL);
  sys_error_code_t res = SYS_NO_ERROR_CODE;
  //  ISM330DHCXTask *p_obj = (ISM330DHCXTask*)_this;

  return res;
}

// ISensor virtual functions definition
// *******************************************

uint8_t ISM330DHCXTask_vtblAccGetId(ISourceObservable *_this)
{
  assert_param(_this != NULL);
  ISM330DHCXTask *p_if_owner = (ISM330DHCXTask*) ((uint32_t) _this - offsetof(ISM330DHCXTask, acc_sensor_if));
  uint8_t res = p_if_owner->acc_id;

  return res;
}

uint8_t ISM330DHCXTask_vtblGyroGetId(ISourceObservable *_this)
{
  assert_param(_this != NULL);
  ISM330DHCXTask *p_if_owner = (ISM330DHCXTask*) ((uint32_t) _this - offsetof(ISM330DHCXTask, gyro_sensor_if));
  uint8_t res = p_if_owner->gyro_id;

  return res;
}

uint8_t ISM330DHCXTask_vtblMlcGetId(ISourceObservable *_this)
{
  assert_param(_this != NULL);
  ISM330DHCXTask *p_if_owner = (ISM330DHCXTask*) ((uint32_t) _this - offsetof(ISM330DHCXTask, mlc_sensor_if));
  uint8_t res = p_if_owner->mlc_id;

  return res;
}

IEventSrc* ISM330DHCXTask_vtblAccGetEventSourceIF(ISourceObservable *_this)
{
  assert_param(_this != NULL);
  ISM330DHCXTask *p_if_owner = (ISM330DHCXTask*) ((uint32_t) _this - offsetof(ISM330DHCXTask, acc_sensor_if));

  return p_if_owner->p_acc_event_src;
}

IEventSrc* ISM330DHCXTask_vtblGyroGetEventSourceIF(ISourceObservable *_this)
{
  assert_param(_this != NULL);
  ISM330DHCXTask *p_if_owner = (ISM330DHCXTask*) ((uint32_t) _this - offsetof(ISM330DHCXTask, gyro_sensor_if));
  return p_if_owner->p_gyro_event_src;
}

IEventSrc* ISM330DHCXTask_vtblMlcGetEventSourceIF(ISourceObservable *_this)
{
  assert_param(_this != NULL);
  ISM330DHCXTask *p_if_owner = (ISM330DHCXTask*) ((uint32_t) _this - offsetof(ISM330DHCXTask, mlc_sensor_if));
  return p_if_owner->p_mlc_event_src;
}

sys_error_code_t ISM330DHCXTask_vtblAccGetODR(ISourceObservable *_this, float *p_measured, float *p_nominal)
{
  assert_param(_this != NULL);
  /*get the object implementing the ISourceObservable IF */
  ISM330DHCXTask *p_if_owner = (ISM330DHCXTask*) ((uint32_t) _this - offsetof(ISM330DHCXTask, acc_sensor_if));
  sys_error_code_t res = SYS_NO_ERROR_CODE;

  /* parameter validation */
  if((p_measured == NULL) || (p_nominal == NULL))
  {
    res = SYS_INVALID_PARAMETER_ERROR_CODE;
    SYS_SET_SERVICE_LEVEL_ERROR_CODE(SYS_INVALID_PARAMETER_ERROR_CODE);
  }
  else
  {
    *p_measured = p_if_owner->acc_sensor_status.MeasuredODR;
    *p_nominal = p_if_owner->acc_sensor_status.ODR;
  }

  return res;
}

float ISM330DHCXTask_vtblAccGetFS(ISourceObservable *_this)
{
  assert_param(_this != NULL);
  ISM330DHCXTask *p_if_owner = (ISM330DHCXTask*) ((uint32_t) _this - offsetof(ISM330DHCXTask, acc_sensor_if));
  float res = p_if_owner->acc_sensor_status.FS;

  return res;
}

float ISM330DHCXTask_vtblAccGetSensitivity(ISourceObservable *_this)
{
  assert_param(_this != NULL);
  ISM330DHCXTask *p_if_owner = (ISM330DHCXTask*) ((uint32_t) _this - offsetof(ISM330DHCXTask, acc_sensor_if));
  float res = p_if_owner->acc_sensor_status.Sensitivity;

  return res;
}

EMData_t ISM330DHCXTask_vtblAccGetDataInfo(ISourceObservable *_this)
{
  assert_param(_this != NULL);
  ISM330DHCXTask *p_if_owner = (ISM330DHCXTask*) ((uint32_t) _this - offsetof(ISM330DHCXTask, acc_sensor_if));
  EMData_t res = p_if_owner->data_acc;

  return res;
}

sys_error_code_t ISM330DHCXTask_vtblGyroGetODR(ISourceObservable *_this, float *p_measured, float *p_nominal)
{
  assert_param(_this != NULL);
  /*get the object implementing the ISourceObservable IF */
  ISM330DHCXTask *p_if_owner = (ISM330DHCXTask*) ((uint32_t) _this - offsetof(ISM330DHCXTask, gyro_sensor_if));
  sys_error_code_t res = SYS_NO_ERROR_CODE;

  /* parameter validation */
  if((p_measured == NULL) || (p_nominal == NULL))
  {
    res = SYS_INVALID_PARAMETER_ERROR_CODE;
    SYS_SET_SERVICE_LEVEL_ERROR_CODE(SYS_INVALID_PARAMETER_ERROR_CODE);
  }
  else
  {
    *p_measured = p_if_owner->gyro_sensor_status.MeasuredODR;
    *p_nominal = p_if_owner->gyro_sensor_status.ODR;
  }

  return res;
}

float ISM330DHCXTask_vtblGyroGetFS(ISourceObservable *_this)
{
  assert_param(_this != NULL);
  ISM330DHCXTask *p_if_owner = (ISM330DHCXTask*) ((uint32_t) _this - offsetof(ISM330DHCXTask, gyro_sensor_if));
  float res = p_if_owner->gyro_sensor_status.FS;

  return res;
}

float ISM330DHCXTask_vtblGyroGetSensitivity(ISourceObservable *_this)
{
  assert_param(_this != NULL);
  ISM330DHCXTask *p_if_owner = (ISM330DHCXTask*) ((uint32_t) _this - offsetof(ISM330DHCXTask, gyro_sensor_if));
  float res = p_if_owner->gyro_sensor_status.Sensitivity;

  return res;
}

EMData_t ISM330DHCXTask_vtblGyroGetDataInfo(ISourceObservable *_this)
{
  assert_param(_this != NULL);
  ISM330DHCXTask *p_if_owner = (ISM330DHCXTask*) ((uint32_t) _this - offsetof(ISM330DHCXTask, gyro_sensor_if));
  EMData_t res = p_if_owner->data_gyro;

  return res;
}

sys_error_code_t ISM330DHCXTask_vtblMlcGetODR(ISourceObservable *_this, float *p_measured, float *p_nominal)
{
  assert_param(_this != NULL);
  /*get the object implementing the ISourceObservable IF */
  ISM330DHCXTask *p_if_owner = (ISM330DHCXTask*) ((uint32_t) _this - offsetof(ISM330DHCXTask, mlc_sensor_if));
  sys_error_code_t res = SYS_NO_ERROR_CODE;

  /* parameter validation */
  if((p_measured == NULL) || (p_nominal == NULL))
  {
    res = SYS_INVALID_PARAMETER_ERROR_CODE;
    SYS_SET_SERVICE_LEVEL_ERROR_CODE(SYS_INVALID_PARAMETER_ERROR_CODE);
  }
  else
  {
    *p_measured = p_if_owner->mlc_sensor_status.MeasuredODR;
    *p_nominal = p_if_owner->mlc_sensor_status.ODR;
  }

  return res;
}

EMData_t ISM330DHCXTask_vtblMlcGetDataInfo(ISourceObservable *_this)
{
  assert_param(_this != NULL);
  ISM330DHCXTask *p_if_owner = (ISM330DHCXTask*) ((uint32_t) _this - offsetof(ISM330DHCXTask, mlc_sensor_if));
  EMData_t res = p_if_owner->data;

  return res;
}

sys_error_code_t ISM330DHCXTask_vtblSensorSetODR(ISensor_t *_this, float ODR)
{
  assert_param(_this != NULL);
  sys_error_code_t res = SYS_NO_ERROR_CODE;
  ISM330DHCXTask *p_if_owner = ISM330DHCXTaskGetOwnerFromISensorIF(_this);

  EPowerMode log_status = AMTGetTaskPowerMode((AManagedTask*) p_if_owner);
  uint8_t sensor_id = ISourceGetId((ISourceObservable*) _this);

  if((log_status == E_POWER_MODE_SENSORS_ACTIVE) && ISensorIsEnabled(_this))
  {
    res = SYS_INVALID_FUNC_CALL_ERROR_CODE;
  }
  else
  {
    /* Set a new command message in the queue */
    SMMessage report =
    {
        .sensorMessage.messageId = SM_MESSAGE_ID_SENSOR_CMD,
        .sensorMessage.nCmdID = SENSOR_CMD_ID_SET_ODR,
        .sensorMessage.nSensorId = sensor_id,
        .sensorMessage.nParam = (uint32_t) ODR };
    res = ISM330DHCXTaskPostReportToBack(p_if_owner, (SMMessage*) &report);
  }

  return res;
}

sys_error_code_t ISM330DHCXTask_vtblSensorSetFS(ISensor_t *_this, float FS)
{
  assert_param(_this != NULL);
  sys_error_code_t res = SYS_NO_ERROR_CODE;
  ISM330DHCXTask *p_if_owner = ISM330DHCXTaskGetOwnerFromISensorIF(_this);

  EPowerMode log_status = AMTGetTaskPowerMode((AManagedTask*) p_if_owner);
  uint8_t sensor_id = ISourceGetId((ISourceObservable*) _this);

  if((log_status == E_POWER_MODE_SENSORS_ACTIVE) && ISensorIsEnabled(_this))
  {
    res = SYS_INVALID_FUNC_CALL_ERROR_CODE;
  }
  else
  {
    /* Set a new command message in the queue */
    SMMessage report =
    {
        .sensorMessage.messageId = SM_MESSAGE_ID_SENSOR_CMD,
        .sensorMessage.nCmdID = SENSOR_CMD_ID_SET_FS,
        .sensorMessage.nSensorId = sensor_id,
        .sensorMessage.nParam = (uint32_t) FS };
    res = ISM330DHCXTaskPostReportToBack(p_if_owner, (SMMessage*) &report);
  }

  return res;

}

sys_error_code_t ISM330DHCXTask_vtblSensorSetFifoWM(ISensor_t *_this, uint16_t fifoWM)
{
  assert_param(_this != NULL);
  sys_error_code_t res = SYS_NO_ERROR_CODE;
  ISM330DHCXTask *p_if_owner = ISM330DHCXTaskGetOwnerFromISensorIF(_this);

  EPowerMode log_status = AMTGetTaskPowerMode((AManagedTask*) p_if_owner);
  uint8_t sensor_id = ISourceGetId((ISourceObservable*) _this);

  if((log_status == E_POWER_MODE_SENSORS_ACTIVE) && ISensorIsEnabled(_this))
  {
    res = SYS_INVALID_FUNC_CALL_ERROR_CODE;
  }
  else
  {
    /* Set a new command message in the queue */
    SMMessage report =
    {
        .sensorMessage.messageId = SM_MESSAGE_ID_SENSOR_CMD,
        .sensorMessage.nCmdID = SENSOR_CMD_ID_SET_FIFO_WM,
        .sensorMessage.nSensorId = sensor_id,
        .sensorMessage.nParam = (uint16_t) fifoWM };
    res = ISM330DHCXTaskPostReportToBack(p_if_owner, (SMMessage*) &report);
  }

  return res;
}

sys_error_code_t ISM330DHCXTask_vtblSensorEnable(ISensor_t *_this)
{
  assert_param(_this != NULL);
  sys_error_code_t res = SYS_NO_ERROR_CODE;
  ISM330DHCXTask *p_if_owner = ISM330DHCXTaskGetOwnerFromISensorIF(_this);

  EPowerMode log_status = AMTGetTaskPowerMode((AManagedTask*) p_if_owner);
  uint8_t sensor_id = ISourceGetId((ISourceObservable*) _this);

  if((log_status == E_POWER_MODE_SENSORS_ACTIVE) && ISensorIsEnabled(_this))
  {
    res = SYS_INVALID_FUNC_CALL_ERROR_CODE;
  }
  else
  {
    /* Set a new command message in the queue */
    SMMessage report =
    {
        .sensorMessage.messageId = SM_MESSAGE_ID_SENSOR_CMD,
        .sensorMessage.nCmdID = SENSOR_CMD_ID_ENABLE,
        .sensorMessage.nSensorId = sensor_id };
    res = ISM330DHCXTaskPostReportToBack(p_if_owner, (SMMessage*) &report);
  }

  return res;
}

sys_error_code_t ISM330DHCXTask_vtblSensorDisable(ISensor_t *_this)
{
  assert_param(_this != NULL);
  sys_error_code_t res = SYS_NO_ERROR_CODE;
  ISM330DHCXTask *p_if_owner = ISM330DHCXTaskGetOwnerFromISensorIF(_this);

  EPowerMode log_status = AMTGetTaskPowerMode((AManagedTask*) p_if_owner);
  uint8_t sensor_id = ISourceGetId((ISourceObservable*) _this);

  if((log_status == E_POWER_MODE_SENSORS_ACTIVE) && ISensorIsEnabled(_this))
  {
    res = SYS_INVALID_FUNC_CALL_ERROR_CODE;
  }
  else
  {
    /* Set a new command message in the queue */
    SMMessage report =
    {
        .sensorMessage.messageId = SM_MESSAGE_ID_SENSOR_CMD,
        .sensorMessage.nCmdID = SENSOR_CMD_ID_DISABLE,
        .sensorMessage.nSensorId = sensor_id };
    res = ISM330DHCXTaskPostReportToBack(p_if_owner, (SMMessage*) &report);
  }

  return res;
}

boolean_t ISM330DHCXTask_vtblSensorIsEnabled(ISensor_t *_this)
{
  assert_param(_this != NULL);
  boolean_t res = FALSE;
  ISM330DHCXTask *p_if_owner = ISM330DHCXTaskGetOwnerFromISensorIF(_this);

  if(ISourceGetId((ISourceObservable*) _this) == p_if_owner->acc_id)
  {
    res = p_if_owner->acc_sensor_status.IsActive;
  }
  else if(ISourceGetId((ISourceObservable*) _this) == p_if_owner->gyro_id)
  {
    res = p_if_owner->gyro_sensor_status.IsActive;
  }
  else if(ISourceGetId((ISourceObservable*) _this) == p_if_owner->mlc_id)
  {
    res = p_if_owner->mlc_sensor_status.IsActive;
  }

  return res;
}

SensorDescriptor_t ISM330DHCXTask_vtblAccGetDescription(ISensor_t *_this)
{
  assert_param(_this != NULL);
  ISM330DHCXTask *p_if_owner = ISM330DHCXTaskGetOwnerFromISensorIF(_this);
  return *p_if_owner->acc_sensor_descriptor;
}

SensorDescriptor_t ISM330DHCXTask_vtblGyroGetDescription(ISensor_t *_this)
{
  assert_param(_this != NULL);
  ISM330DHCXTask *p_if_owner = ISM330DHCXTaskGetOwnerFromISensorIF(_this);
  return *p_if_owner->gyro_sensor_descriptor;
}

SensorDescriptor_t ISM330DHCXTask_vtblMlcGetDescription(ISensor_t *_this)
{
  assert_param(_this != NULL);
  ISM330DHCXTask *p_if_owner = ISM330DHCXTaskGetOwnerFromISensorIF(_this);
  return *p_if_owner->mlc_sensor_descriptor;
}

SensorStatus_t ISM330DHCXTask_vtblAccGetStatus(ISensor_t *_this)
{
  assert_param(_this != NULL);
  ISM330DHCXTask *p_if_owner = ISM330DHCXTaskGetOwnerFromISensorIF(_this);
  return p_if_owner->acc_sensor_status;
}

SensorStatus_t ISM330DHCXTask_vtblGyroGetStatus(ISensor_t *_this)
{
  assert_param(_this != NULL);
  ISM330DHCXTask *p_if_owner = ISM330DHCXTaskGetOwnerFromISensorIF(_this);
  return p_if_owner->gyro_sensor_status;
}

SensorStatus_t ISM330DHCXTask_vtblMlcGetStatus(ISensor_t *_this)
{
  assert_param(_this != NULL);
  ISM330DHCXTask *p_if_owner = ISM330DHCXTaskGetOwnerFromISensorIF(_this);
  return p_if_owner->mlc_sensor_status;
}

boolean_t ISM330DHCXTask_vtblSensorMlcIsEnabled(ISensorMlc_t *_this)
{
  assert_param(_this != NULL);
  ISM330DHCXTask *p_if_owner = ISM330DHCXTaskGetOwnerFromISensorMlcIF(_this);
  return p_if_owner->mlc_enable;
}

sys_error_code_t ISM330DHCXTask_vtblSensorMlcLoadUcf(ISensorMlc_t *_this, uint32_t size, const char *ucf)
{
  assert_param(_this != NULL);
  sys_error_code_t res = SYS_NO_ERROR_CODE;
  ISM330DHCXTask *p_if_owner = ISM330DHCXTaskGetOwnerFromISensorMlcIF(_this);
  stmdev_ctx_t *p_sensor_drv = (stmdev_ctx_t*) &p_if_owner->p_sensor_bus_if->m_xConnector;

  uint32_t i;
  char ucf_reg[3];
  char ucf_data[3];
  long reg;
  long data;

  for(i = 0; i < size / 4u; i++)
  {
    ucf_reg[0] = ucf[4u * i];
    ucf_reg[1] = ucf[4u * i + 1u];
    ucf_reg[2] = '\0';
    ucf_data[0] = ucf[4u * i + 2u];
    ucf_data[1] = ucf[4u * i + 3u];
    ucf_data[2] = '\0';

    reg = strtol(ucf_reg, NULL, 16);
    data = strtol(ucf_data, NULL, 16);

    res = ism330dhcx_write_reg(p_sensor_drv, (uint8_t) reg, (uint8_t*) &data, 1u);
    if(res != SYS_NO_ERROR_CODE)
    {
      break;
    }
  }

  if(!SYS_IS_ERROR_CODE(res))
  {
    p_if_owner->mlc_enable = TRUE;
    p_if_owner->mlc_sensor_status.IsActive = TRUE;

    ISM330DHCX_XL_ODR_From_UCF(p_if_owner);
    ISM330DHCX_XL_FS_From_UCF(p_if_owner);
    ISM330DHCX_GY_ODR_From_UCF(p_if_owner);
    ISM330DHCX_GY_FS_From_UCF(p_if_owner);
  }
  return res;
}

/* Private function definition */
// ***************************
static sys_error_code_t ISM330DHCXTaskExecuteStepState1(AManagedTask *_this)
{
  assert_param(_this != NULL);
  sys_error_code_t res = SYS_NO_ERROR_CODE;
  ISM330DHCXTask *p_obj = (ISM330DHCXTask*) _this;
  SMMessage report =
  {
      0 };

  AMTExSetInactiveState((AManagedTaskEx*) _this, TRUE);
  if(pdTRUE == xQueueReceive(p_obj->in_queue, &report, portMAX_DELAY))
  {
    AMTExSetInactiveState((AManagedTaskEx*) _this, FALSE);

    switch(report.messageID)
    {
      case SM_MESSAGE_ID_FORCE_STEP:
        {
          // do nothing. I need only to resume.
          __NOP();
          break;
        }
      case SM_MESSAGE_ID_SENSOR_CMD:
        {
          switch(report.sensorMessage.nCmdID)
          {
            case SENSOR_CMD_ID_SET_ODR:
              res = ISM330DHCXTaskSensorSetODR(p_obj, report);
              break;
            case SENSOR_CMD_ID_SET_FS:
              res = ISM330DHCXTaskSensorSetFS(p_obj, report);
              break;
            case SENSOR_CMD_ID_SET_FIFO_WM:
              res = ISM330DHCXTaskSensorSetFifoWM(p_obj, report);
              break;
            case SENSOR_CMD_ID_ENABLE:
              res = ISM330DHCXTaskSensorEnable(p_obj, report);
              break;
            case SENSOR_CMD_ID_DISABLE:
              res = ISM330DHCXTaskSensorDisable(p_obj, report);
              break;
            default:
              /* unwanted report */
              res = SYS_APP_TASK_UNKNOWN_REPORT_ERROR_CODE;
              SYS_SET_SERVICE_LEVEL_ERROR_CODE(SYS_APP_TASK_UNKNOWN_REPORT_ERROR_CODE)
              ;

              SYS_DEBUGF(SYS_DBG_LEVEL_WARNING, ("ISM330DHCX: unexpected report in Run: %i\r\n", report.messageID));
              break;
          }
          break;
        }
      default:
        {
          /* unwanted report */
          res = SYS_APP_TASK_UNKNOWN_REPORT_ERROR_CODE;
          SYS_SET_SERVICE_LEVEL_ERROR_CODE(SYS_APP_TASK_UNKNOWN_REPORT_ERROR_CODE);

          SYS_DEBUGF(SYS_DBG_LEVEL_WARNING, ("ISM330DHCX: unexpected report in Run: %i\r\n", report.messageID));
          break;
        }
    }
  }

  return res;
}

static sys_error_code_t ISM330DHCXTaskExecuteStepDatalog(AManagedTask *_this)
{
  assert_param(_this != NULL);
  sys_error_code_t res = SYS_NO_ERROR_CODE;
  ISM330DHCXTask *p_obj = (ISM330DHCXTask*) _this;
  SMMessage report =
  {
      0 };

  AMTExSetInactiveState((AManagedTaskEx*) _this, TRUE);
  if(pdTRUE == xQueueReceive(p_obj->in_queue, &report, portMAX_DELAY))
  {
    AMTExSetInactiveState((AManagedTaskEx*) _this, FALSE);

    switch(report.messageID)
    {
      case SM_MESSAGE_ID_FORCE_STEP:
        {
          // do nothing. I need only to resume.
          __NOP();
          break;
        }

      case SM_MESSAGE_ID_DATA_READY:
        {
          if(p_obj->pIRQConfig == NULL)
          {
            if(pdPASS != xTimerReset(p_obj->read_timer, pdMS_TO_TICKS(100)))
              res = SYS_UNDEFINED_ERROR_CODE;
          }

          res = ISM330DHCXTaskSensorReadData(p_obj);
          if(!SYS_IS_ERROR_CODE(res))
          {
#if ISM330DHCX_FIFO_ENABLED
            if(p_obj->fifo_level != 0)
            {
#endif
              // notify the listeners...
              double timestamp = report.sensorDataReadyMessage.fTimestamp;
              double delta_timestamp = timestamp - p_obj->prev_timestamp;
              p_obj->prev_timestamp = timestamp;

              DataEvent_t evt_acc, evt_gyro;

#ifdef ISM330DHCX_FIFO_ENABLED
              if((p_obj->acc_sensor_status.IsActive) && (p_obj->gyro_sensor_status.IsActive)) /* Read both ACC and GYRO */
              {
                /* update measuredODR */
                p_obj->acc_sensor_status.MeasuredODR = (float) p_obj->acc_samples_count / (float) delta_timestamp;
                p_obj->gyro_sensor_status.MeasuredODR = (float) p_obj->gyro_samples_count / (float) delta_timestamp;

                if(p_obj->acc_sensor_status.ODR > p_obj->gyro_sensor_status.ODR) /* Acc is faster than Gyro */
                {
                  /* Create a bidimensional data interleaved [m x 3], m is the number of samples in the sensor queue:
                   * [X0, Y0, Z0]
                   * [X1, Y1, Z1]
                   * ...
                   * [Xm-1, Ym-1, Zm-1]
                   */
                  EMD_Init(&p_obj->data_acc, p_obj->p_fast_sensor_data_buff, E_EM_INT16, E_EM_MODE_INTERLEAVED, 2, p_obj->acc_samples_count, 3);
                  DataEventInit((IEvent*) &evt_acc, p_obj->p_acc_event_src, &p_obj->data_acc, timestamp, p_obj->acc_id);

                  EMD_Init(&p_obj->data_gyro, p_obj->p_slow_sensor_data_buff, E_EM_INT16, E_EM_MODE_INTERLEAVED, 2, p_obj->gyro_samples_count, 3);
                  DataEventInit((IEvent*) &evt_gyro, p_obj->p_gyro_event_src, &p_obj->data_gyro, timestamp, p_obj->gyro_id);

                  IEventSrcSendEvent(p_obj->p_acc_event_src, (IEvent*) &evt_acc, NULL);
                  IEventSrcSendEvent(p_obj->p_gyro_event_src, (IEvent*) &evt_gyro, NULL);
                }
                else
                {
                  /* Create a bidimensional data interleaved [m x 3], m is the number of samples in the sensor queue:
                   * [X0, Y0, Z0]
                   * [X1, Y1, Z1]
                   * ...
                   * [Xm-1, Ym-1, Zm-1]
                   */
                  EMD_Init(&p_obj->data_acc, p_obj->p_slow_sensor_data_buff, E_EM_INT16, E_EM_MODE_INTERLEAVED, 2, p_obj->acc_samples_count, 3);
                  DataEventInit((IEvent*) &evt_acc, p_obj->p_acc_event_src, &p_obj->data_acc, timestamp, p_obj->acc_id);

                  EMD_Init(&p_obj->data_gyro, p_obj->p_fast_sensor_data_buff, E_EM_INT16, E_EM_MODE_INTERLEAVED, 2, p_obj->gyro_samples_count, 3);
                  DataEventInit((IEvent*) &evt_gyro, p_obj->p_gyro_event_src, &p_obj->data_gyro, timestamp, p_obj->gyro_id);

                  IEventSrcSendEvent(p_obj->p_acc_event_src, (IEvent*) &evt_acc, NULL);
                  IEventSrcSendEvent(p_obj->p_gyro_event_src, (IEvent*) &evt_gyro, NULL);
                }

              }
              else /* Only 1 out of 2 is active */
              {
                if(p_obj->acc_sensor_status.IsActive)
                {
                  /* update measuredODR */
                  p_obj->acc_sensor_status.MeasuredODR = (float) p_obj->acc_samples_count / (float) delta_timestamp;

                  /* Create a bidimensional data interleaved [m x 3], m is the number of samples in the sensor queue:
                   * [X0, Y0, Z0]
                   * [X1, Y1, Z1]
                   * ...
                   * [Xm-1, Ym-1, Zm-1]
                   */
                  EMD_Init(&p_obj->data_acc, p_obj->p_fast_sensor_data_buff, E_EM_INT16, E_EM_MODE_INTERLEAVED, 2, p_obj->acc_samples_count, 3);
                  DataEventInit((IEvent*) &evt_acc, p_obj->p_acc_event_src, &p_obj->data_acc, timestamp, p_obj->acc_id);

                  IEventSrcSendEvent(p_obj->p_acc_event_src, (IEvent*) &evt_acc, NULL);
                }
                else if(p_obj->gyro_sensor_status.IsActive)
                {
                  /* update measuredODR */
                  p_obj->gyro_sensor_status.MeasuredODR = (float) p_obj->gyro_samples_count / (float) delta_timestamp;

                  /* Create a bidimensional data interleaved [m x 3], m is the number of samples in the sensor queue:
                   * [X0, Y0, Z0]
                   * [X1, Y1, Z1]
                   * ...
                   * [Xm-1, Ym-1, Zm-1]
                   */
                  EMD_Init(&p_obj->data_gyro, p_obj->p_fast_sensor_data_buff, E_EM_INT16, E_EM_MODE_INTERLEAVED, 2, p_obj->gyro_samples_count, 3);
                  DataEventInit((IEvent*) &evt_gyro, p_obj->p_gyro_event_src, &p_obj->data_gyro, timestamp, p_obj->gyro_id);

                  IEventSrcSendEvent(p_obj->p_gyro_event_src, (IEvent*) &evt_gyro, NULL);
                }
                else
                {
                  res = SYS_INVALID_PARAMETER_ERROR_CODE;
                }
              }
#else
          if(p_obj->acc_sensor_status.IsActive && p_obj->acc_drdy)
          {
            /* update measuredODR */
            p_obj->acc_sensor_status.MeasuredODR = (float)p_obj->acc_samples_count / (float)delta_timestamp;

            EMD_Init(&p_obj->data_acc, p_obj->p_acc_sample, E_EM_INT16, E_EM_MODE_INTERLEAVED, 2, p_obj->acc_samples_count, 3);
            DataEventInit((IEvent*) &evt_acc, p_obj->p_acc_event_src, &p_obj->data_acc, timestamp, p_obj->acc_id);

            IEventSrcSendEvent(p_obj->p_acc_event_src, (IEvent*) &evt_acc, NULL);
            p_obj->acc_drdy = 0;
          }
          if(p_obj->gyro_sensor_status.IsActive && p_obj->gyro_drdy)
          {
            /* update measuredODR */
            p_obj->gyro_sensor_status.MeasuredODR = (float)p_obj->gyro_samples_count / (float)delta_timestamp;

            EMD_Init(&p_obj->data_gyro, p_obj->p_gyro_sample, E_EM_INT16, E_EM_MODE_INTERLEAVED, 2, p_obj->gyro_samples_count, 3);
            DataEventInit((IEvent*) &evt_gyro, p_obj->p_gyro_event_src, &p_obj->data_gyro, timestamp, p_obj->gyro_id);

            IEventSrcSendEvent(p_obj->p_gyro_event_src, (IEvent*) &evt_gyro, NULL);
            p_obj->gyro_drdy = 0;
          }
#endif
              SYS_DEBUGF(SYS_DBG_LEVEL_ALL, ("ISM330DHCX: ts = %f\r\n", (float)timestamp));
#if ISM330DHCX_FIFO_ENABLED
            }
#endif
          }
          if(p_obj->pIRQConfig == NULL)
          {
            if(pdPASS != xTimerStart(p_obj->read_timer, pdMS_TO_TICKS(100)))
            {
              res = SYS_UNDEFINED_ERROR_CODE;
            }
          }
          break;
        }

      case SM_MESSAGE_ID_DATA_READY_MLC:
        {
          if(p_obj->pMLCConfig == NULL)
          {
            if(pdPASS != xTimerReset(p_obj->mlc_timer, pdMS_TO_TICKS(100)))
              res = SYS_UNDEFINED_ERROR_CODE;
          }
          res = ISM330DHCXTaskSensorReadMLC(p_obj);
          if(!SYS_IS_ERROR_CODE(res))
          {
            // notify the listeners...
            double timestamp = report.sensorDataReadyMessage.fTimestamp;

            if(p_obj->mlc_enable)
            {
              EMD_Init(&p_obj->data, p_obj->p_mlc_sensor_data_buff, E_EM_UINT8, E_EM_MODE_INTERLEAVED, 2, 1, 9);

              DataEvent_t evt;

              DataEventInit((IEvent*) &evt, p_obj->p_mlc_event_src, &p_obj->data, timestamp, p_obj->mlc_id);
              IEventSrcSendEvent(p_obj->p_mlc_event_src, (IEvent*) &evt, NULL);
            }
            else
            {
              res = SYS_INVALID_PARAMETER_ERROR_CODE;
            }
          }
          if(p_obj->pMLCConfig == NULL)
          {
            if(pdPASS != xTimerStart(p_obj->mlc_timer, pdMS_TO_TICKS(100)))
            {
              res = SYS_UNDEFINED_ERROR_CODE;
            }
          }
          break;
        }

      case SM_MESSAGE_ID_SENSOR_CMD:
        {
          switch(report.sensorMessage.nCmdID)
          {
            case SENSOR_CMD_ID_INIT:
              res = ISM330DHCXTaskSensorInit(p_obj);
              if(!SYS_IS_ERROR_CODE(res))
              {
                if(p_obj->pIRQConfig == NULL)
                {
                  if(pdPASS != xTimerStart(p_obj->read_timer, pdMS_TO_TICKS(100)))
                    res = SYS_UNDEFINED_ERROR_CODE;
                }
                else
                {
                  ISM330DHCXTaskConfigureIrqPin(p_obj, FALSE);
                }
              }
              if(!SYS_IS_ERROR_CODE(res))
              {
                if(p_obj->mlc_sensor_status.IsActive == true)
                {
                  if(p_obj->pMLCConfig == NULL)
                  {
                    if(pdPASS != xTimerStart(p_obj->mlc_timer, pdMS_TO_TICKS(100)))
                    {
                      res = SYS_UNDEFINED_ERROR_CODE;
                    }
                  }
                  else
                  {
                    ISM330DHCXTaskConfigureMLCPin(p_obj, FALSE);
                  }
                }
              }
              break;
            case SENSOR_CMD_ID_SET_ODR:
              res = ISM330DHCXTaskSensorSetODR(p_obj, report);
              break;
            case SENSOR_CMD_ID_SET_FS:
              res = ISM330DHCXTaskSensorSetFS(p_obj, report);
              break;
            case SENSOR_CMD_ID_SET_FIFO_WM:
              res = ISM330DHCXTaskSensorSetFifoWM(p_obj, report);
              break;
            case SENSOR_CMD_ID_ENABLE:
              res = ISM330DHCXTaskSensorEnable(p_obj, report);
              break;
            case SENSOR_CMD_ID_DISABLE:
              res = ISM330DHCXTaskSensorDisable(p_obj, report);
              break;
            default:
              {
                /* unwanted report */
                res = SYS_APP_TASK_UNKNOWN_REPORT_ERROR_CODE;
                SYS_SET_SERVICE_LEVEL_ERROR_CODE(SYS_APP_TASK_UNKNOWN_REPORT_ERROR_CODE);

                SYS_DEBUGF(SYS_DBG_LEVEL_WARNING, ("ISM330DHCX: unexpected report in Datalog: %i\r\n", report.messageID));
                break;
              }
          }
          break;
        }

      default:
        {
          /* unwanted report */
          res = SYS_APP_TASK_UNKNOWN_REPORT_ERROR_CODE;
          SYS_SET_SERVICE_LEVEL_ERROR_CODE(SYS_APP_TASK_UNKNOWN_REPORT_ERROR_CODE);

          SYS_DEBUGF(SYS_DBG_LEVEL_WARNING, ("ISM330DHCX: unexpected report in Datalog: %i\r\n", report.messageID));
          break;
        }
    }
  }

  return res;
}

static inline sys_error_code_t ISM330DHCXTaskPostReportToFront(ISM330DHCXTask *_this, SMMessage *pReport)
{
  assert_param(_this != NULL);
  assert_param(pReport);
  sys_error_code_t res = SYS_NO_ERROR_CODE;

  if(SYS_IS_CALLED_FROM_ISR())
  {
    if(pdTRUE != xQueueSendToFrontFromISR(_this->in_queue, pReport, NULL))
    {
      res = SYS_APP_TASK_REPORT_LOST_ERROR_CODE;
      // this function is private and the caller will ignore this return code.
    }
  }
  else
  {
    if(pdTRUE != xQueueSendToFront(_this->in_queue, pReport, pdMS_TO_TICKS(100)))
    {
      res = SYS_APP_TASK_REPORT_LOST_ERROR_CODE;
      // this function is private and the caller will ignore this return code.
    }
  }

  return res;
}

static inline sys_error_code_t ISM330DHCXTaskPostReportToBack(ISM330DHCXTask *_this, SMMessage *pReport)
{
  assert_param(_this != NULL);
  assert_param(pReport);
  sys_error_code_t res = SYS_NO_ERROR_CODE;

  if(SYS_IS_CALLED_FROM_ISR())
  {
    if(pdTRUE != xQueueSendToBackFromISR(_this->in_queue, pReport, NULL))
    {
      res = SYS_APP_TASK_REPORT_LOST_ERROR_CODE;
      // this function is private and the caller will ignore this return code.
    }
  }
  else
  {
    if(pdTRUE != xQueueSendToBack(_this->in_queue, pReport, pdMS_TO_TICKS(100)))
    {
      res = SYS_APP_TASK_REPORT_LOST_ERROR_CODE;
      // this function is private and the caller will ignore this return code.
    }
  }

  return res;
}

static sys_error_code_t ISM330DHCXTaskSensorInit(ISM330DHCXTask *_this)
{
  assert_param(_this != NULL);
  sys_error_code_t res = SYS_NO_ERROR_CODE;
  stmdev_ctx_t *p_sensor_drv = (stmdev_ctx_t*) &_this->p_sensor_bus_if->m_xConnector;

  /* Setup acc and gyro */
  uint8_t reg0 = 0;
  ism330dhcx_odr_xl_t ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_OFF;
  ism330dhcx_bdr_xl_t ism330dhcx_bdr_xl = ISM330DHCX_XL_NOT_BATCHED;
  ism330dhcx_odr_g_t ism330dhcx_odr_g = ISM330DHCX_GY_ODR_OFF;
  ism330dhcx_bdr_gy_t ism330dhcx_bdr_gy = ISM330DHCX_GY_NOT_BATCHED;
  int32_t ret_val = 0;

  ism330dhcx_pin_int1_route_t int1_route =
  {
      0 };
  ism330dhcx_pin_int2_route_t int2_route =
  {
      0 };

  ret_val = ism330dhcx_reset_set(p_sensor_drv, 1);
  do
  {
    ism330dhcx_reset_get(p_sensor_drv, &reg0);
  }
  while(reg0);

  /* check if the sensor is using the SPI IF, and in case disable the I2C IF*/
  if(_this->pCSConfig != NULL)
  {
    ret_val = ism330dhcx_i2c_interface_set(p_sensor_drv, ISM330DHCX_I2C_DISABLE);
  }

  ret_val = ism330dhcx_device_id_get(p_sensor_drv, (uint8_t*) &reg0);
  if(!ret_val)
  {
    ABusIFSetWhoAmI(_this->p_sensor_bus_if, reg0);
  }

  SYS_DEBUGF(SYS_DBG_LEVEL_VERBOSE, ("ISM330DHCX: sensor - I am 0x%x.\r\n", reg0));

  /* AXL FS */
  if(_this->acc_sensor_status.FS < 3.0f)
  {
    ism330dhcx_xl_full_scale_set(p_sensor_drv, ISM330DHCX_2g);
  }
  else if(_this->acc_sensor_status.FS < 5.0f)
  {
    ism330dhcx_xl_full_scale_set(p_sensor_drv, ISM330DHCX_4g);
  }
  else if(_this->acc_sensor_status.FS < 9.0f)
  {
    ism330dhcx_xl_full_scale_set(p_sensor_drv, ISM330DHCX_8g);
  }
  else
    ism330dhcx_xl_full_scale_set(p_sensor_drv, ISM330DHCX_16g);

  /* GYRO FS */
  if(_this->gyro_sensor_status.FS < 126.0f)
  {
    ism330dhcx_gy_full_scale_set(p_sensor_drv, ISM330DHCX_125dps);
  }
  else if(_this->gyro_sensor_status.FS < 251.0f)
  {
    ism330dhcx_gy_full_scale_set(p_sensor_drv, ISM330DHCX_250dps);
  }
  else if(_this->gyro_sensor_status.FS < 501.0f)
  {
    ism330dhcx_gy_full_scale_set(p_sensor_drv, ISM330DHCX_500dps);
  }
  else if(_this->gyro_sensor_status.FS < 1001.0f)
  {
    ism330dhcx_gy_full_scale_set(p_sensor_drv, ISM330DHCX_1000dps);
  }
  else if(_this->gyro_sensor_status.FS < 2001.0f)
  {
    ism330dhcx_gy_full_scale_set(p_sensor_drv, ISM330DHCX_2000dps);
  }
  else
    ism330dhcx_gy_full_scale_set(p_sensor_drv, ISM330DHCX_4000dps);

  if(_this->acc_sensor_status.ODR < 13.0f)
  {
    ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_12Hz5;
    ism330dhcx_bdr_xl = ISM330DHCX_XL_BATCHED_AT_12Hz5;
  }
  else if(_this->acc_sensor_status.ODR < 27.0f)
  {
    ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_26Hz;
    ism330dhcx_bdr_xl = ISM330DHCX_XL_BATCHED_AT_26Hz;
  }
  else if(_this->acc_sensor_status.ODR < 53.0f)
  {
    ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_52Hz;
    ism330dhcx_bdr_xl = ISM330DHCX_XL_BATCHED_AT_52Hz;
  }
  else if(_this->acc_sensor_status.ODR < 105.0f)
  {
    ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_104Hz;
    ism330dhcx_bdr_xl = ISM330DHCX_XL_BATCHED_AT_104Hz;
  }
  else if(_this->acc_sensor_status.ODR < 209.0f)
  {
    ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_208Hz;
    ism330dhcx_bdr_xl = ISM330DHCX_XL_BATCHED_AT_208Hz;
  }
  else if(_this->acc_sensor_status.ODR < 417.0f)
  {
    ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_416Hz;
    ism330dhcx_bdr_xl = ISM330DHCX_XL_BATCHED_AT_417Hz;
  }
  else if(_this->acc_sensor_status.ODR < 834.0f)
  {
    ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_833Hz;
    ism330dhcx_bdr_xl = ISM330DHCX_XL_BATCHED_AT_833Hz;
  }
  else if(_this->acc_sensor_status.ODR < 1667.0f)
  {
    ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_1666Hz;
    ism330dhcx_bdr_xl = ISM330DHCX_XL_BATCHED_AT_1667Hz;
  }
  else if(_this->acc_sensor_status.ODR < 3333.0f)
  {
    ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_3332Hz;
    ism330dhcx_bdr_xl = ISM330DHCX_XL_BATCHED_AT_3333Hz;
  }
  else
  {
    ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_6667Hz;
    ism330dhcx_bdr_xl = ISM330DHCX_XL_BATCHED_AT_6667Hz;
  }

  if(_this->gyro_sensor_status.ODR < 13.0f)
  {
    ism330dhcx_odr_g = ISM330DHCX_GY_ODR_12Hz5;
    ism330dhcx_bdr_gy = ISM330DHCX_GY_BATCHED_AT_12Hz5;
  }
  else if(_this->gyro_sensor_status.ODR < 27.0f)
  {
    ism330dhcx_odr_g = ISM330DHCX_GY_ODR_26Hz;
    ism330dhcx_bdr_gy = ISM330DHCX_GY_BATCHED_AT_26Hz;
  }
  else if(_this->gyro_sensor_status.ODR < 53.0f)
  {
    ism330dhcx_odr_g = ISM330DHCX_GY_ODR_52Hz;
    ism330dhcx_bdr_gy = ISM330DHCX_GY_BATCHED_AT_52Hz;
  }
  else if(_this->gyro_sensor_status.ODR < 105.0f)
  {
    ism330dhcx_odr_g = ISM330DHCX_GY_ODR_104Hz;
    ism330dhcx_bdr_gy = ISM330DHCX_GY_BATCHED_AT_104Hz;
  }
  else if(_this->gyro_sensor_status.ODR < 209.0f)
  {
    ism330dhcx_odr_g = ISM330DHCX_GY_ODR_208Hz;
    ism330dhcx_bdr_gy = ISM330DHCX_GY_BATCHED_AT_208Hz;
  }
  else if(_this->gyro_sensor_status.ODR < 417.0f)
  {
    ism330dhcx_odr_g = ISM330DHCX_GY_ODR_416Hz;
    ism330dhcx_bdr_gy = ISM330DHCX_GY_BATCHED_AT_417Hz;
  }
  else if(_this->gyro_sensor_status.ODR < 834.0f)
  {
    ism330dhcx_odr_g = ISM330DHCX_GY_ODR_833Hz;
    ism330dhcx_bdr_gy = ISM330DHCX_GY_BATCHED_AT_833Hz;
  }
  else if(_this->gyro_sensor_status.ODR < 1667.0f)
  {
    ism330dhcx_odr_g = ISM330DHCX_GY_ODR_1666Hz;
    ism330dhcx_bdr_gy = ISM330DHCX_GY_BATCHED_AT_1667Hz;
  }
  else if(_this->gyro_sensor_status.ODR < 3333.0f)
  {
    ism330dhcx_odr_g = ISM330DHCX_GY_ODR_3332Hz;
    ism330dhcx_bdr_gy = ISM330DHCX_GY_BATCHED_AT_3333Hz;
  }
  else
  {
    ism330dhcx_odr_g = ISM330DHCX_GY_ODR_6667Hz;
    ism330dhcx_bdr_gy = ISM330DHCX_GY_BATCHED_AT_6667Hz;
  }

  if(_this->acc_sensor_status.IsActive)
  {
    ism330dhcx_xl_data_rate_set(p_sensor_drv, ism330dhcx_odr_xl);
    ism330dhcx_fifo_xl_batch_set(p_sensor_drv, ism330dhcx_bdr_xl);
  }
  else
  {
    ism330dhcx_xl_data_rate_set(p_sensor_drv, ISM330DHCX_XL_ODR_OFF);
    ism330dhcx_fifo_xl_batch_set(p_sensor_drv, ISM330DHCX_XL_NOT_BATCHED);
    _this->acc_sensor_status.IsActive = false;
  }

  if(_this->gyro_sensor_status.IsActive)
  {
    ism330dhcx_gy_data_rate_set(p_sensor_drv, ism330dhcx_odr_g);
    ism330dhcx_fifo_gy_batch_set(p_sensor_drv, ism330dhcx_bdr_gy);
  }
  else
  {
    ism330dhcx_gy_data_rate_set(p_sensor_drv, ISM330DHCX_GY_ODR_OFF);
    ism330dhcx_fifo_gy_batch_set(p_sensor_drv, ISM330DHCX_GY_NOT_BATCHED);
    _this->gyro_sensor_status.IsActive = false;
  }

#if ISM330DHCX_FIFO_ENABLED

  uint16_t ism330dhcx_wtm_level = 0;
  uint16_t ism330dhcx_wtm_level_acc;
  uint16_t ism330dhcx_wtm_level_gyro;

  if(_this->samples_per_it == 0)
  {
    /* Calculation of watermark and samples per int*/
    ism330dhcx_wtm_level_acc = ((uint16_t) _this->acc_sensor_status.ODR * (uint16_t) ISM330DHCX_MAX_DRDY_PERIOD);
    ism330dhcx_wtm_level_gyro = ((uint16_t) _this->gyro_sensor_status.ODR * (uint16_t) ISM330DHCX_MAX_DRDY_PERIOD);

    if(_this->acc_sensor_status.IsActive && _this->gyro_sensor_status.IsActive) /* Both subSensor is active */
    {
      if(ism330dhcx_wtm_level_acc > ism330dhcx_wtm_level_gyro)
      {
        ism330dhcx_wtm_level = ism330dhcx_wtm_level_acc;
      }
      else
      {
        ism330dhcx_wtm_level = ism330dhcx_wtm_level_gyro;
      }
    }
    else /* Only one subSensor is active */
    {
      if(_this->acc_sensor_status.IsActive)
      {
        ism330dhcx_wtm_level = ism330dhcx_wtm_level_acc;
      }
      else
      {
        ism330dhcx_wtm_level = ism330dhcx_wtm_level_gyro;
      }
    }

    if(ism330dhcx_wtm_level > ISM330DHCX_MAX_WTM_LEVEL)
    {
      ism330dhcx_wtm_level = ISM330DHCX_MAX_WTM_LEVEL;
    }
    else if(ism330dhcx_wtm_level < ISM330DHCX_MIN_WTM_LEVEL)
    {
      ism330dhcx_wtm_level = ISM330DHCX_MIN_WTM_LEVEL;
    }
    _this->samples_per_it = ism330dhcx_wtm_level;
  }

  /* Setup int for FIFO */
  ism330dhcx_fifo_watermark_set(p_sensor_drv, _this->samples_per_it);

  if(_this->pIRQConfig != NULL)
  {
    int1_route.int1_ctrl.int1_fifo_th = 1;
  }
  else
  {
    int1_route.int1_ctrl.int1_fifo_th = 0;
  }
  ism330dhcx_pin_int1_route_set(p_sensor_drv, &int1_route);

  ism330dhcx_fifo_mode_set(p_sensor_drv, ISM330DHCX_STREAM_MODE);

#else

  _this->samples_per_it = 1;
  if (_this->pIRQConfig != NULL)
  {
    if (_this->acc_sensor_status.IsActive && _this->gyro_sensor_status.IsActive) /* Both subSensor is active */
    {
      int1_route.int1_ctrl.int1_drdy_xl = 1;
      int1_route.int1_ctrl.int1_drdy_g = 1;
    }
    else if (_this->acc_sensor_status.IsActive)
    {
      int1_route.int1_ctrl.int1_drdy_xl = 1;
    }
    else
    {
      int1_route.int1_ctrl.int1_drdy_g = 1;
    }
  }
  else
  {
    int1_route.int1_ctrl.int1_drdy_xl = 0;
    int1_route.int1_ctrl.int1_drdy_g = 0;
  }
  ism330dhcx_pin_int1_route_set(p_sensor_drv, &int1_route);

#endif /* ISM330DHCX_FIFO_ENABLED */

  /* Setup mlc */
  if(_this->mlc_enable)
  {
    ism330dhcx_pin_int1_route_get(p_sensor_drv, &int1_route);
    ism330dhcx_pin_int2_route_get(p_sensor_drv, &int2_route);

    if(int1_route.mlc_int1.int1_mlc1 == 1 || int1_route.md1_cfg.int1_emb_func == 1)
    {
      int1_route.mlc_int1.int1_mlc1 = 0;
      int1_route.md1_cfg.int1_emb_func = 0;
      ism330dhcx_pin_int1_route_set(p_sensor_drv, &int1_route);
    }

    if(int2_route.mlc_int2.int2_mlc1 == 0 || int2_route.md2_cfg.int2_emb_func == 0)
    {
      int2_route.mlc_int2.int2_mlc1 = 1;
      int2_route.md2_cfg.int2_emb_func = 1;
      ism330dhcx_pin_int2_route_set(p_sensor_drv, &int2_route);
    }
  }

  _this->ism330dhcx_task_cfg_timer_period_ms =
      (uint16_t)(_this->acc_sensor_status.ODR < _this->gyro_sensor_status.ODR ? _this->acc_sensor_status.ODR : _this->gyro_sensor_status.ODR);
#if ISM330DHCX_FIFO_ENABLED
  _this->ism330dhcx_task_cfg_timer_period_ms = (uint16_t) ((1000.0f / _this->ism330dhcx_task_cfg_timer_period_ms) * (((float) (_this->samples_per_it)) / 2.0f));
#else
  _this->ism330dhcx_task_cfg_timer_period_ms = (uint16_t)(1000.0f/_this->ism330dhcx_task_cfg_timer_period_ms);
#endif

  return res;
}

static sys_error_code_t ISM330DHCXTaskSensorReadData(ISM330DHCXTask *_this)
{
  assert_param(_this != NULL);
  sys_error_code_t res = SYS_NO_ERROR_CODE;
  stmdev_ctx_t *p_sensor_drv = (stmdev_ctx_t*) &_this->p_sensor_bus_if->m_xConnector;

#if ISM330DHCX_FIFO_ENABLED
  uint8_t reg[2];
  uint16_t i;

  /* Check FIFO_WTM_IA and fifo level. We do not use PID in order to avoid reading one register twice */
  ism330dhcx_read_reg(p_sensor_drv, ISM330DHCX_FIFO_STATUS1, reg, 2);

  _this->fifo_level = ((reg[1] & 0x03) << 8) + reg[0];

  if(((reg[1]) & 0x80) && (_this->fifo_level >= _this->samples_per_it))
  {
    ism330dhcx_read_reg(p_sensor_drv, ISM330DHCX_FIFO_DATA_OUT_TAG, _this->p_fast_sensor_data_buff, _this->samples_per_it * 7);

#if (HSD_USE_DUMMY_DATA == 1)
    int16_t * p16 = (int16_t *)(_this->p_fast_sensor_data_buff);

    for (i = 0; i < _this->samples_per_it; i++)
    {
      p16 = (int16_t *)(&_this->p_fast_sensor_data_buff[i*7]+1);
      if((_this->p_fast_sensor_data_buff[i*7]>>3) == ISM330DHCX_TAG_ACC)
      {
        *p16++ = dummyDataCounter_acc++;
        *p16++ = dummyDataCounter_acc++;
        *p16++ = dummyDataCounter_acc++;
      }
      else
      {
        *p16++ = dummyDataCounter_gyro++;
        *p16++ = dummyDataCounter_gyro++;
        *p16++ = dummyDataCounter_gyro++;
      }
    }
#endif
    if((_this->acc_sensor_status.IsActive) && (_this->gyro_sensor_status.IsActive))
    {
      /* Read both ACC and GYRO */

      uint32_t odr_acc = (uint32_t) _this->acc_sensor_status.ODR;
      uint32_t odr_gyro = (uint32_t) _this->gyro_sensor_status.ODR;

      int16_t *p16_src = (int16_t*) _this->p_fast_sensor_data_buff;
      int16_t *p_acc, *p_gyro;

      _this->acc_samples_count = 0;
      _this->gyro_samples_count = 0;

      if(odr_acc > odr_gyro) /* Acc is faster than Gyro */
      {
        p_acc = (int16_t*) _this->p_fast_sensor_data_buff;
        p_gyro = (int16_t*) _this->p_slow_sensor_data_buff;
      }
      else
      {
        p_acc = (int16_t*) _this->p_slow_sensor_data_buff;
        p_gyro = (int16_t*) _this->p_fast_sensor_data_buff;
      }

      uint8_t *p_tag = (uint8_t*) p16_src;

      for(i = 0; i < _this->samples_per_it; i++)
      {
        if(((*p_tag) >> 3) == ISM330DHCX_TAG_ACC)
        {
          p16_src = (int16_t*) (p_tag + 1);
          *p_acc++ = *p16_src++;
          *p_acc++ = *p16_src++;
          *p_acc++ = *p16_src++;
          _this->acc_samples_count++;
        }
        else
        {
          p16_src = (int16_t*) (p_tag + 1);
          *p_gyro++ = *p16_src++;
          *p_gyro++ = *p16_src++;
          *p_gyro++ = *p16_src++;
          _this->gyro_samples_count++;
        }
        p_tag += 7;
      }
    }
    else /* 1 subsensor active only --> simply drop TAGS */
    {
      int16_t *p16_src = (int16_t*) _this->p_fast_sensor_data_buff;
      int16_t *p16_dest = (int16_t*) _this->p_fast_sensor_data_buff;
      for(i = 0; i < _this->samples_per_it; i++)
      {
        p16_src = (int16_t*) &((uint8_t*) (p16_src))[1];
        *p16_dest++ = *p16_src++;
        *p16_dest++ = *p16_src++;
        *p16_dest++ = *p16_src++;
      }
      if(_this->acc_sensor_status.IsActive)
      {
        _this->acc_samples_count = _this->samples_per_it;
      }
      else
      {
        _this->gyro_samples_count = _this->samples_per_it;
      }
    }

  }
  else
  {
    _this->fifo_level = 0;
    res = SYS_BASE_ERROR_CODE;
  }
#else
  if((_this->acc_sensor_status.IsActive) && (_this->gyro_sensor_status.IsActive))
  {
    ism330dhcx_status_reg_t val;
    uint32_t odr_acc = (uint32_t) _this->acc_sensor_status.ODR;
    uint32_t odr_gyro = (uint32_t) _this->gyro_sensor_status.ODR;

    if(odr_acc != odr_gyro)
    {
      /* Need to read which sensor generated the INT in case of different ODR; */
      ism330dhcx_status_reg_get(p_sensor_drv, &val);
    }
    else
    {
      /* Manually set the variable to read both sensors (avoid to loose time with a read) */
      val.xlda = 1U;
      val.gda  = 1U;
    }

    if(val.xlda == 1U)
    {
      ism330dhcx_read_reg(p_sensor_drv, ISM330DHCX_OUTX_L_A, _this->p_acc_sample, 6);
      _this->acc_samples_count = 1U;
      _this->acc_drdy = 1;
    }
    if(val.gda == 1U)
    {
      ism330dhcx_read_reg(p_sensor_drv, ISM330DHCX_OUTX_L_G, _this->p_gyro_sample, 6);
      _this->gyro_samples_count = 1U;
      _this->gyro_drdy = 1;
    }
  }
  else if(_this->acc_sensor_status.IsActive)
  {
    ism330dhcx_read_reg(p_sensor_drv, ISM330DHCX_OUTX_L_A, _this->p_acc_sample, _this->samples_per_it * 6);
    _this->acc_samples_count = 1U;
    _this->acc_drdy = 1;
  }
  else if(_this->gyro_sensor_status.IsActive)
  {
    ism330dhcx_read_reg(p_sensor_drv, ISM330DHCX_OUTX_L_G, _this->p_acc_sample, _this->samples_per_it * 6);
    _this->gyro_samples_count = 1U;
    _this->gyro_drdy = 1;
  }
  else
  {
  }
  _this->fifo_level = 1;
#endif /* ISM330DHCX_FIFO_ENABLED */

  return res;
}

static sys_error_code_t ISM330DHCXTaskSensorReadMLC(ISM330DHCXTask *_this)
{
  assert_param(_this != NULL);
  sys_error_code_t res = SYS_NO_ERROR_CODE;
  stmdev_ctx_t *p_sensor_drv = (stmdev_ctx_t*) &_this->p_sensor_bus_if->m_xConnector;
  uint8_t ii;

  if(_this->mlc_enable)
  {
    for(ii = 0; ii < 8; ii++)
    {
      ism330dhcx_mem_bank_set(p_sensor_drv, ISM330DHCX_EMBEDDED_FUNC_BANK);
      ism330dhcx_read_reg(p_sensor_drv, ISM330DHCX_MLC0_SRC + ii, (uint8_t*) (&_this->p_mlc_sensor_data_buff[ii]), 1);
      ism330dhcx_mem_bank_set(p_sensor_drv, ISM330DHCX_USER_BANK);
    }
    ism330dhcx_mlc_status_get(p_sensor_drv, (ism330dhcx_mlc_status_mainpage_t*) &_this->p_mlc_sensor_data_buff[8]);
  }

  return res;
}

static sys_error_code_t ISM330DHCXTaskSensorRegister(ISM330DHCXTask *_this)
{
  assert_param(_this != NULL);
  sys_error_code_t res = SYS_NO_ERROR_CODE;

  ISensor_t *acc_if = (ISensor_t*) ISM330DHCXTaskGetAccSensorIF(_this);
  ISensor_t *gyro_if = (ISensor_t*) ISM330DHCXTaskGetGyroSensorIF(_this);
  ISensor_t *mlc_if = (ISensor_t*) ISM330DHCXTaskGetMlcSensorIF(_this);

  _this->acc_id = SMAddSensor(acc_if);
  _this->gyro_id = SMAddSensor(gyro_if);
  _this->mlc_id = SMAddSensor(mlc_if);

  return res;
}

static sys_error_code_t ISM330DHCXTaskSensorInitTaskParams(ISM330DHCXTask *_this)
{
  assert_param(_this != NULL);
  sys_error_code_t res = SYS_NO_ERROR_CODE;

  /* ACCELEROMETER STATUS */
  _this->acc_sensor_status.IsActive = TRUE;
  _this->acc_sensor_status.FS = 16.0f;
  _this->acc_sensor_status.Sensitivity = 0.0000305f * _this->acc_sensor_status.FS;
  _this->acc_sensor_status.ODR = 6667.0f;
  _this->acc_sensor_status.MeasuredODR = 0.0f;
#ifdef ISM330DHCX_FIFO_ENABLED
  EMD_Init(&_this->data_acc, _this->p_fast_sensor_data_buff, E_EM_INT16, E_EM_MODE_INTERLEAVED, 2, 1, 3);
#else
  EMD_Init(&_this->data_acc, _this->p_acc_sample, E_EM_INT16, E_EM_MODE_INTERLEAVED, 2, 1, 3);
#endif

  /* GYROSCOPE STATUS */
  _this->gyro_sensor_status.IsActive = TRUE;
  _this->gyro_sensor_status.FS = 4000.0f;
  _this->gyro_sensor_status.Sensitivity = 0.035f * _this->gyro_sensor_status.FS;
  _this->gyro_sensor_status.ODR = 6667.0f;
  _this->gyro_sensor_status.MeasuredODR = 0.0f;
#ifdef ISM330DHCX_FIFO_ENABLED
  EMD_Init(&_this->data_gyro, _this->p_slow_sensor_data_buff, E_EM_INT16, E_EM_MODE_INTERLEAVED, 2, 1, 3);
#else
  EMD_Init(&_this->data_gyro, _this->p_gyro_sample, E_EM_INT16, E_EM_MODE_INTERLEAVED, 2, 1, 3);
#endif

  /* MLC STATUS */
  _this->mlc_sensor_status.IsActive = FALSE;
  _this->mlc_sensor_status.FS = 1.0f;
  _this->mlc_sensor_status.Sensitivity = 1.0f;
  _this->mlc_sensor_status.ODR = 1.0f;
  _this->mlc_sensor_status.MeasuredODR = 0.0f;
  EMD_Init(&_this->data, _this->p_mlc_sensor_data_buff, E_EM_UINT8, E_EM_MODE_INTERLEAVED, 2, 1, 9);

  return res;
}

static sys_error_code_t ISM330DHCXTaskSensorSetODR(ISM330DHCXTask *_this, SMMessage report)
{
  assert_param(_this != NULL);
  sys_error_code_t res = SYS_NO_ERROR_CODE;

  stmdev_ctx_t *p_sensor_drv = (stmdev_ctx_t*) &_this->p_sensor_bus_if->m_xConnector;
  float ODR = (float) report.sensorMessage.nParam;
  uint8_t id = report.sensorMessage.nSensorId;

  /* Changing ODR must disable MLC sensor: MLC can work properly only when setup from UCF */
  _this->mlc_enable = FALSE;
  _this->mlc_sensor_status.IsActive = FALSE;

  if(id == _this->acc_id)
  {
    if(ODR < 1.0f)
    {
      ism330dhcx_xl_data_rate_set(p_sensor_drv, ISM330DHCX_XL_ODR_OFF);
      /* Do not update the model in case of ODR = 0 */
      ODR = _this->acc_sensor_status.ODR;
    }
    else if(ODR < 13.0f)
    {
      ODR = 12.5f;
    }
    else if(ODR < 27.0f)
    {
      ODR = 26.0f;
    }
    else if(ODR < 53.0f)
    {
      ODR = 52.0f;
    }
    else if(ODR < 105.0f)
    {
      ODR = 104.0f;
    }
    else if(ODR < 209.0f)
    {
      ODR = 208.0f;
    }
    else if(ODR < 417.0f)
    {
      ODR = 416.0f;
    }
    else if(ODR < 834.0f)
    {
      ODR = 833.0f;
    }
    else if(ODR < 1667.0f)
    {
      ODR = 1666.0f;
    }
    else if(ODR < 3333.0f)
    {
      ODR = 3332.0f;
    }
    else
    {
      ODR = 6667;
    }

    if(!SYS_IS_ERROR_CODE(res))
    {
      _this->acc_sensor_status.ODR = ODR;
      _this->acc_sensor_status.MeasuredODR = 0.0f;
    }
  }
  else if(id == _this->gyro_id)
  {
    if(ODR < 1.0f)
    {
      ism330dhcx_gy_data_rate_set(p_sensor_drv, ISM330DHCX_GY_ODR_OFF);
      /* Do not update the model in case of ODR = 0 */
      ODR = _this->gyro_sensor_status.ODR;
    }
    else if(ODR < 13.0f)
    {
      ODR = 12.5f;
    }
    else if(ODR < 27.0f)
    {
      ODR = 26.0f;
    }
    else if(ODR < 53.0f)
    {
      ODR = 52.0f;
    }
    else if(ODR < 105.0f)
    {
      ODR = 104.0f;
    }
    else if(ODR < 209.0f)
    {
      ODR = 208.0f;
    }
    else if(ODR < 417.0f)
    {
      ODR = 416.0f;
    }
    else if(ODR < 834.0f)
    {
      ODR = 833.0f;
    }
    else if(ODR < 1667.0f)
    {
      ODR = 1666.0f;
    }
    else if(ODR < 3333.0f)
    {
      ODR = 3332.0f;
    }
    else
    {
      ODR = 6667.0f;
    }

    if(!SYS_IS_ERROR_CODE(res))
    {
      _this->gyro_sensor_status.ODR = ODR;
      _this->gyro_sensor_status.MeasuredODR = 0.0f;
    }
  }
  else
  {
    res = SYS_INVALID_PARAMETER_ERROR_CODE;
  }

  return res;
}

static sys_error_code_t ISM330DHCXTaskSensorSetFS(ISM330DHCXTask *_this, SMMessage report)
{
  assert_param(_this != NULL);
  sys_error_code_t res = SYS_NO_ERROR_CODE;

  stmdev_ctx_t *p_sensor_drv = (stmdev_ctx_t*) &_this->p_sensor_bus_if->m_xConnector;
  float FS = (float) report.sensorMessage.nParam;
  uint8_t id = report.sensorMessage.nSensorId;

  /* Changing FS must disable MLC sensor: MLC can work properly only when setup from UCF */
  _this->mlc_enable = FALSE;
  _this->mlc_sensor_status.IsActive = FALSE;

  if(id == _this->acc_id)
  {
    if(FS < 3.0f)
    {
      ism330dhcx_xl_full_scale_set(p_sensor_drv, ISM330DHCX_2g);
      FS = 2.0f;
    }
    else if(FS < 5.0f)
    {
      ism330dhcx_xl_full_scale_set(p_sensor_drv, ISM330DHCX_4g);
      FS = 4.0f;
    }
    else if(FS < 9.0f)
    {
      ism330dhcx_xl_full_scale_set(p_sensor_drv, ISM330DHCX_8g);
      FS = 8.0f;
    }
    else
    {
      ism330dhcx_xl_full_scale_set(p_sensor_drv, ISM330DHCX_16g);
      FS = 16.0f;
    }

    if(!SYS_IS_ERROR_CODE(res))
    {
      _this->acc_sensor_status.FS = FS;
      _this->acc_sensor_status.Sensitivity = 0.0000305f * _this->acc_sensor_status.FS;
    }
  }
  else if(id == _this->gyro_id)
  {
    if(FS < 126.0f)
    {
      ism330dhcx_gy_full_scale_set(p_sensor_drv, ISM330DHCX_125dps);
      FS = 125.0f;
    }
    else if(FS < 251.0f)
    {
      ism330dhcx_gy_full_scale_set(p_sensor_drv, ISM330DHCX_250dps);
      FS = 250.0f;
    }
    else if(FS < 501.0f)
    {
      ism330dhcx_gy_full_scale_set(p_sensor_drv, ISM330DHCX_500dps);
      FS = 500.0f;
    }
    else if(FS < 1001.0f)
    {
      ism330dhcx_gy_full_scale_set(p_sensor_drv, ISM330DHCX_1000dps);
      FS = 1000.0f;
    }
    else if(FS < 2001.0f)
    {
      ism330dhcx_gy_full_scale_set(p_sensor_drv, ISM330DHCX_2000dps);
      FS = 2000.0f;
    }
    else
    {
      ism330dhcx_gy_full_scale_set(p_sensor_drv, ISM330DHCX_4000dps);
      FS = 4000.0f;
    }

    if(!SYS_IS_ERROR_CODE(res))
    {
      _this->gyro_sensor_status.FS = FS;
      _this->gyro_sensor_status.Sensitivity = 0.035f * _this->gyro_sensor_status.FS;
    }
  }
  else
  {
    res = SYS_INVALID_PARAMETER_ERROR_CODE;
  }

  return res;
}

static sys_error_code_t ISM330DHCXTaskSensorSetFifoWM(ISM330DHCXTask *_this, SMMessage report)
{
  assert_param(_this != NULL);
  sys_error_code_t res = SYS_NO_ERROR_CODE;

  stmdev_ctx_t *p_sensor_drv = (stmdev_ctx_t*) &_this->p_sensor_bus_if->m_xConnector;
  uint16_t ism330dhcx_wtm_level = report.sensorMessage.nParam;
  uint8_t id = report.sensorMessage.nSensorId;

  if((id == _this->acc_id) || (id == _this->gyro_id))
  {
    ism330dhcx_wtm_level = ism330dhcx_wtm_level + _this->samples_per_it;
//    if(_this->samples_per_it < ism330dhcx_wtm_level) /* keep the higher value  */
//    {
//      if (_this->acc_sensor_status.IsActive && _this->gyro_sensor_status.IsActive) /* Both subSensor is active */
//      {
//        ism330dhcx_wtm_level = ism330dhcx_wtm_level*2;    /* acc and gyro share the FIFO, so size should be double*/
//      }

    if(ism330dhcx_wtm_level > ISM330DHCX_MAX_WTM_LEVEL)
    {
      ism330dhcx_wtm_level = ISM330DHCX_MAX_WTM_LEVEL;
    }

    SYS_DEBUGF(SYS_DBG_LEVEL_VERBOSE, ("**** ISM330DHCX: REAL FIFO WM: %d\r\n", ism330dhcx_wtm_level));

    _this->samples_per_it = ism330dhcx_wtm_level;

    /* Set fifo in continuous / stream mode*/
    ism330dhcx_fifo_mode_set(p_sensor_drv, ISM330DHCX_STREAM_MODE);

    /* Setup int for FIFO */
    ism330dhcx_fifo_watermark_set(p_sensor_drv, _this->samples_per_it);

  }
  else
  {
    res = SYS_INVALID_PARAMETER_ERROR_CODE;
  }

  return res;
}

static sys_error_code_t ISM330DHCXTaskSensorEnable(ISM330DHCXTask *_this, SMMessage report)
{
  assert_param(_this != NULL);
  sys_error_code_t res = SYS_NO_ERROR_CODE;

  uint8_t id = report.sensorMessage.nSensorId;

  if(id == _this->acc_id)
  {
    _this->acc_sensor_status.IsActive = TRUE;
  }
  else if(id == _this->gyro_id)
  {
    _this->gyro_sensor_status.IsActive = TRUE;
  }
  else
  {
    res = SYS_INVALID_PARAMETER_ERROR_CODE;
  }

  return res;
}

static sys_error_code_t ISM330DHCXTaskSensorDisable(ISM330DHCXTask *_this, SMMessage report)
{
  assert_param(_this != NULL);
  sys_error_code_t res = SYS_NO_ERROR_CODE;
  stmdev_ctx_t *p_sensor_drv = (stmdev_ctx_t*) &_this->p_sensor_bus_if->m_xConnector;

  uint8_t id = report.sensorMessage.nSensorId;

  if(id == _this->acc_id)
  {
    _this->acc_sensor_status.IsActive = FALSE;
    ism330dhcx_xl_data_rate_set(p_sensor_drv, ISM330DHCX_XL_ODR_OFF);
  }
  else if(id == _this->gyro_id)
  {
    _this->gyro_sensor_status.IsActive = FALSE;
    ism330dhcx_gy_data_rate_set(p_sensor_drv, ISM330DHCX_GY_ODR_OFF);
  }
  else
  {
    res = SYS_INVALID_PARAMETER_ERROR_CODE;
  }

  return res;
}

static boolean_t ISM330DHCXTaskSensorIsActive(const ISM330DHCXTask *_this)
{
  assert_param(_this != NULL);
  return (_this->acc_sensor_status.IsActive || _this->gyro_sensor_status.IsActive);
}

static sys_error_code_t ISM330DHCXTaskEnterLowPowerMode(const ISM330DHCXTask *_this)
{
  assert_param(_this != NULL);
  sys_error_code_t res = SYS_NO_ERROR_CODE;
  stmdev_ctx_t *p_sensor_drv = (stmdev_ctx_t*) &_this->p_sensor_bus_if->m_xConnector;

  ism330dhcx_odr_xl_t ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_OFF;
  ism330dhcx_bdr_xl_t ism330dhcx_bdr_xl = ISM330DHCX_XL_NOT_BATCHED;
  ism330dhcx_odr_g_t ism330dhcx_odr_g = ISM330DHCX_GY_ODR_OFF;
  ism330dhcx_bdr_gy_t ism330dhcx_bdr_gy = ISM330DHCX_GY_NOT_BATCHED;

  ism330dhcx_xl_data_rate_set(p_sensor_drv, ism330dhcx_odr_xl);
  ism330dhcx_fifo_xl_batch_set(p_sensor_drv, ism330dhcx_bdr_xl);
  ism330dhcx_gy_data_rate_set(p_sensor_drv, ism330dhcx_odr_g);
  ism330dhcx_fifo_gy_batch_set(p_sensor_drv, ism330dhcx_bdr_gy);

  return res;
}

static sys_error_code_t ISM330DHCXTaskConfigureIrqPin(const ISM330DHCXTask *_this, boolean_t LowPower)
{
  assert_param(_this != NULL);
  sys_error_code_t res = SYS_NO_ERROR_CODE;

  if(!LowPower)
  {
    /* Configure INT Pin */
    _this->pIRQConfig->p_mx_init_f();
  }
  else
  {
    GPIO_InitTypeDef GPIO_InitStruct =
    {
        0 };

    // first disable the IRQ to avoid spurious interrupt to wake the MCU up.
    HAL_NVIC_DisableIRQ(_this->pIRQConfig->irq_n);
    HAL_NVIC_ClearPendingIRQ(_this->pIRQConfig->irq_n);
    // then reconfigure the PIN in analog high impedance to reduce the power consumption.
    GPIO_InitStruct.Pin = _this->pIRQConfig->pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(_this->pIRQConfig->port, &GPIO_InitStruct);
  }

  return res;
}

static sys_error_code_t ISM330DHCXTaskConfigureMLCPin(const ISM330DHCXTask *_this, boolean_t LowPower)
{
  assert_param(_this != NULL);
  sys_error_code_t res = SYS_NO_ERROR_CODE;

  if(!LowPower)
  {
    /* Configure MLC INT Pin */
    _this->pMLCConfig->p_mx_init_f();
  }
  else
  {
    GPIO_InitTypeDef GPIO_InitStruct =
    {
        0 };

    // first disable the IRQ to avoid spurious interrupt to wake the MCU up.
    HAL_NVIC_DisableIRQ(_this->pMLCConfig->irq_n);
    HAL_NVIC_ClearPendingIRQ(_this->pMLCConfig->irq_n);
    // then reconfigure the PIN in analog high impedance to reduce the power consumption.
    GPIO_InitStruct.Pin = _this->pMLCConfig->pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(_this->pMLCConfig->port, &GPIO_InitStruct);
  }

  return res;
}

static inline ISM330DHCXTask* ISM330DHCXTaskGetOwnerFromISensorIF(ISensor_t *p_if)
{
  assert_param(p_if != NULL);
  ISM330DHCXTask *p_if_owner = NULL;

  /* check if the virtual function has been called from the mlc sensor IF  */
  p_if_owner = (ISM330DHCXTask*) ((uint32_t) p_if - offsetof(ISM330DHCXTask, mlc_sensor_if));
  if(!(p_if_owner->gyro_sensor_if.vptr == &sTheClass.gyro_sensor_if_vtbl) || !(p_if_owner->super.vptr == &sTheClass.vtbl))
  {
    /* then the virtual function has been called from the gyro IF  */
    p_if_owner = (ISM330DHCXTask*) ((uint32_t) p_if - offsetof(ISM330DHCXTask, gyro_sensor_if));
  }
  if(!(p_if_owner->acc_sensor_if.vptr == &sTheClass.acc_sensor_if_vtbl) || !(p_if_owner->super.vptr == &sTheClass.vtbl))
  {
    /* then the virtual function has been called from the acc IF  */
    p_if_owner = (ISM330DHCXTask*) ((uint32_t) p_if - offsetof(ISM330DHCXTask, acc_sensor_if));
  }

  return p_if_owner;
}

static inline ISM330DHCXTask* ISM330DHCXTaskGetOwnerFromISensorMlcIF(ISensorMlc_t *p_if)
{
  assert_param(p_if != NULL);
  ISM330DHCXTask *p_if_owner = NULL;
  p_if_owner = (ISM330DHCXTask*) ((uint32_t) p_if - offsetof(ISM330DHCXTask, sensor_mlc_if));

  return p_if_owner;
}

static void ISM330DHCXTaskTimerCallbackFunction(TimerHandle_t timer)
{
  SMMessage report;
  report.sensorDataReadyMessage.messageId = SM_MESSAGE_ID_DATA_READY;
  report.sensorDataReadyMessage.fTimestamp = SysTsGetTimestampF(SysGetTimestampSrv());

  if(sTaskObj.in_queue != NULL)
  {
    if(pdTRUE != xQueueSendToBackFromISR(sTaskObj.in_queue, &report, NULL))
    {
      // unable to send the message. Signal the error
      sys_error_handler();
    }
  }
}

static void ISM330DHCXTaskMLCTimerCallbackFunction(TimerHandle_t timer)
{
  SMMessage report;
  report.sensorDataReadyMessage.messageId = SM_MESSAGE_ID_DATA_READY_MLC;
  report.sensorDataReadyMessage.fTimestamp = SysTsGetTimestampF(SysGetTimestampSrv());

  if(sTaskObj.in_queue != NULL)
  {
    if(pdTRUE != xQueueSendToBackFromISR(sTaskObj.in_queue, &report, NULL))
    {
      // unable to send the message. Signal the error
      sys_error_handler();
    }
  }
}

static sys_error_code_t ISM330DHCXTaskCreateTimer(ISM330DHCXTask *_this)
{
  sys_error_code_t res = SYS_NO_ERROR_CODE;

  /* create the software timer used to read ISM330DHCX data */
  _this->read_timer = xTimerCreate("ISM330DHCX_T", pdMS_TO_TICKS(ISM330DHCX_TASK_CFG_TIMER_PERIOD_MS), pdFALSE, _this, ISM330DHCXTaskTimerCallbackFunction);
  if(_this->read_timer == NULL)
  {
    res = SYS_TASK_HEAP_OUT_OF_MEMORY_ERROR_CODE;
    SYS_SET_SERVICE_LEVEL_ERROR_CODE(res);
  }

  /* create the software timer used to read ISM330DHCX data */
  _this->mlc_timer = xTimerCreate("ISM330DHCX_MLC_T", pdMS_TO_TICKS(ISM330DHCX_TASK_CFG_MLC_TIMER_PERIOD_MS), pdFALSE, _this,
                                  ISM330DHCXTaskMLCTimerCallbackFunction);
  if(_this->mlc_timer == NULL)
  {
    res = SYS_TASK_HEAP_OUT_OF_MEMORY_ERROR_CODE;
    SYS_SET_SERVICE_LEVEL_ERROR_CODE(res);
  }

  return res;
}

/* CubeMX integration */
// ******************
/**
 * Interrupt callback
 */
void ISM330DHCXTask_EXTI_Callback(uint16_t Pin)
{
  SMMessage report;
  report.sensorDataReadyMessage.messageId = SM_MESSAGE_ID_DATA_READY;
  report.sensorDataReadyMessage.fTimestamp = SysTsGetTimestampF(SysGetTimestampSrv());

  if(sTaskObj.in_queue != NULL)
  {
    if(pdTRUE != xQueueSendToBackFromISR(sTaskObj.in_queue, &report, NULL))
    {
      // unable to send the report. Signal the error
      sys_error_handler();
    }
  }
}

void INT2_DHCX_EXTI_Callback(uint16_t Pin)
{
  SMMessage report;
  report.sensorDataReadyMessage.messageId = SM_MESSAGE_ID_DATA_READY_MLC;
  report.sensorDataReadyMessage.fTimestamp = SysTsGetTimestampF(SysGetTimestampSrv());

  if(sTaskObj.in_queue != NULL)
  {
    if(pdTRUE != xQueueSendToBackFromISR(sTaskObj.in_queue, &report, NULL))
    {
      // unable to send the report. Signal the error
      sys_error_handler();
    }
  }
}

static sys_error_code_t ISM330DHCX_XL_ODR_From_UCF(ISM330DHCXTask *_this)
{
  assert_param(_this != NULL);
  sys_error_code_t res = SYS_NO_ERROR_CODE;
  stmdev_ctx_t *p_sensor_drv = (stmdev_ctx_t*) &_this->p_sensor_bus_if->m_xConnector;

  /* Read ODR set from UCF, then set to 0 to switch off the sensor */
  float Odr = 0.0f;
  ism330dhcx_odr_xl_t ism330dhcx_odr_xl = ISM330DHCX_XL_ODR_OFF;
  ism330dhcx_xl_data_rate_get(p_sensor_drv, &ism330dhcx_odr_xl);
  ism330dhcx_xl_data_rate_set(p_sensor_drv, ISM330DHCX_XL_ODR_OFF);

  _this->acc_sensor_status.IsActive = TRUE;

  /* Update only the model */
  switch(ism330dhcx_odr_xl)
  {
    case ISM330DHCX_XL_ODR_OFF:
      _this->acc_sensor_status.IsActive = FALSE;
      /* Do not update the model in case of ODR = 0 */
      Odr = _this->acc_sensor_status.ODR;
      break;

    case ISM330DHCX_XL_ODR_12Hz5:
      Odr = 12.5f;
      break;

    case ISM330DHCX_XL_ODR_26Hz:
      Odr = 26.0f;
      break;

    case ISM330DHCX_XL_ODR_52Hz:
      Odr = 52.0f;
      break;

    case ISM330DHCX_XL_ODR_104Hz:
      Odr = 104.0f;
      break;

    case ISM330DHCX_XL_ODR_208Hz:
      Odr = 208.0f;
      break;

    case ISM330DHCX_XL_ODR_416Hz:
      Odr = 416.0f;
      break;

    case ISM330DHCX_XL_ODR_833Hz:
      Odr = 833.0f;
      break;

    case ISM330DHCX_XL_ODR_1666Hz:
      Odr = 1666.0f;
      break;

    case ISM330DHCX_XL_ODR_3332Hz:
      Odr = 3332.0f;
      break;

    case ISM330DHCX_XL_ODR_6667Hz:
      Odr = 6667.0f;
      break;

    default:
      break;
  }

  if(!SYS_IS_ERROR_CODE(res))
  {
    _this->acc_sensor_status.ODR = Odr;
    _this->acc_sensor_status.MeasuredODR = 0.0f;
  }

  return res;
}

static sys_error_code_t ISM330DHCX_XL_FS_From_UCF(ISM330DHCXTask *_this)
{
  assert_param(_this != NULL);
  sys_error_code_t res = SYS_NO_ERROR_CODE;
  stmdev_ctx_t *p_sensor_drv = (stmdev_ctx_t*) &_this->p_sensor_bus_if->m_xConnector;

  float FullScale = 2.0;
  ism330dhcx_fs_xl_t fs_xl;
  ism330dhcx_xl_full_scale_get(p_sensor_drv, &fs_xl);

  switch(fs_xl)
  {
    case ISM330DHCX_2g:
      FullScale = 2.0;
      break;

    case ISM330DHCX_4g:
      FullScale = 4.0;
      break;

    case ISM330DHCX_8g:
      FullScale = 8.0;
      break;

    case ISM330DHCX_16g:
      FullScale = 16.0;
      break;

    default:
      break;
  }

  if(!SYS_IS_ERROR_CODE(res))
  {
    _this->acc_sensor_status.FS = FullScale;
  }

  return res;
}

static sys_error_code_t ISM330DHCX_GY_ODR_From_UCF(ISM330DHCXTask *_this)
{
  assert_param(_this != NULL);
  sys_error_code_t res = SYS_NO_ERROR_CODE;
  stmdev_ctx_t *p_sensor_drv = (stmdev_ctx_t*) &_this->p_sensor_bus_if->m_xConnector;

  /* Read ODR set from UCF, then set to 0 to switch off the sensor */
  float Odr = 0.0f;
  ism330dhcx_odr_g_t ism330dhcx_odr_g = ISM330DHCX_GY_ODR_OFF;
  ism330dhcx_gy_data_rate_get(p_sensor_drv, &ism330dhcx_odr_g);
  ism330dhcx_gy_data_rate_set(p_sensor_drv, ISM330DHCX_GY_ODR_OFF);

  _this->gyro_sensor_status.IsActive = TRUE;

  /* Update only the model */
  switch(ism330dhcx_odr_g)
  {
    case ISM330DHCX_GY_ODR_OFF:
      _this->gyro_sensor_status.IsActive = FALSE;
      /* Do not update the model in case of ODR = 0 */
      Odr = _this->gyro_sensor_status.ODR;
      break;

    case ISM330DHCX_GY_ODR_12Hz5:
      Odr = 12.5f;
      break;

    case ISM330DHCX_GY_ODR_26Hz:
      Odr = 26.0f;
      break;

    case ISM330DHCX_GY_ODR_52Hz:
      Odr = 52.0f;
      break;

    case ISM330DHCX_GY_ODR_104Hz:
      Odr = 104.0f;
      break;

    case ISM330DHCX_GY_ODR_208Hz:
      Odr = 208.0f;
      break;

    case ISM330DHCX_GY_ODR_416Hz:
      Odr = 416.0f;
      break;

    case ISM330DHCX_GY_ODR_833Hz:
      Odr = 833.0f;
      break;

    case ISM330DHCX_GY_ODR_1666Hz:
      Odr = 1666.0f;
      break;

    case ISM330DHCX_GY_ODR_3332Hz:
      Odr = 3332.0f;
      break;

    case ISM330DHCX_GY_ODR_6667Hz:
      Odr = 6667.0f;
      break;

    default:
      break;
  }

  if(!SYS_IS_ERROR_CODE(res))
  {
    _this->gyro_sensor_status.ODR = Odr;
    _this->gyro_sensor_status.MeasuredODR = 0.0f;
  }

  return res;
}

static sys_error_code_t ISM330DHCX_GY_FS_From_UCF(ISM330DHCXTask *_this)
{
  assert_param(_this != NULL);
  sys_error_code_t res = SYS_NO_ERROR_CODE;
  stmdev_ctx_t *p_sensor_drv = (stmdev_ctx_t*) &_this->p_sensor_bus_if->m_xConnector;

  float FullScale = 125;
  ism330dhcx_fs_g_t fs_g;
  ism330dhcx_gy_full_scale_get(p_sensor_drv, &fs_g);

  switch(fs_g)
  {
    case ISM330DHCX_125dps:
      FullScale = 125;
      break;

    case ISM330DHCX_250dps:
      FullScale = 250;
      break;

    case ISM330DHCX_500dps:
      FullScale = 500;
      break;

    case ISM330DHCX_1000dps:
      FullScale = 1000;
      break;

    case ISM330DHCX_2000dps:
      FullScale = 2000;
      break;

    case ISM330DHCX_4000dps:
      FullScale = 4000;
      break;

    default:
      break;
  }

  if(!SYS_IS_ERROR_CODE(res))
  {
    _this->gyro_sensor_status.FS = FullScale;
  }

  return res;
}

