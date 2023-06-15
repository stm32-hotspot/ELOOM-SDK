/**
 ******************************************************************************
 * @file    StreamerCLI.h
 * @author  SRA - GPM
 * 
 * 
 *
 * @brief
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
#ifndef INC_APPCONTROLLER_H_
#define INC_APPCONTROLLER_H_

#ifdef __cplusplus
extern "C" {
#endif


#include "services/AManagedTaskEx.h"
#include "services/AManagedTaskEx_vtbl.h"
#include "services/CircularBuffer.h"
#include "events/IDataEventListener.h"
#include "events/IDataEventListener_vtbl.h"
#include "queue.h"
#include "timers.h"
#include "usbd_def.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "SensorManager.h"

#ifndef SCLI_TASK_CFG_MAX_IN_LENGTH
#define SCLI_TASK_CFG_MAX_IN_LENGTH                   128
#endif
#ifndef SCLI_TASK_CFG_MAX_OUT_LENGTH
#define SCLI_TASK_CFG_MAX_OUT_LENGTH                  512
#endif
#define SCLI_TASK_CFG_DEF_DATA_ELEMENT_BUFFER_COUNT   (256)
#ifndef SCLI_TASK_CFG_BLOCK_ON_DATA_LOST
#define SCLI_TASK_CFG_BLOCK_ON_DATA_LOST              0
#endif

/**
 * Create  type name for struct _CBHandle_t.
 */
typedef struct _CBHandle_t CBHandle_t;

/**
 * Describe the struct used to manage the Circular Buffer.
 */
struct _CBHandle_t
{
	/**
	 * Pointer to the CircularBuffer object.
	 */
  CircularBuffer *p_circular_buffer;

  /**
   * Point at the CBItem used by the data producer (a sensor).
   */
  CBItem *p_producer_data_buff;

  /**
   * Point at the CBItem used by the data consumer (the StreamerCLI).
   */
  CBItem *p_consumer_data_buff;

  /**
   * Index of the next free position in the p_producer_data_buff. It is used by the data producer.
   */
  uint32_t producer_element_idx;
};

/**
 * Function prototype to set a configuration for the SensorManager.This function configures the sensors
 * available through the SensorManager and it specifies the active sensor used by the SensorListener
 * to visualize the sensor data.
 *
 * @param p_sm [IN] specifies a pointer to the application SensorManager object.
 * @param p_active_sensor_id [OUT] specifies the
 * @return SYS_NO_ERROR_CODE if success, an application error code otherwise.
 */
typedef sys_error_code_t (*SensorsConfigF)(SensorManager_t *p_sm, uint16_t *p_active_sensor_id);

/**
 * Create  type name for struct _StreamerCLI_t.
 */
typedef struct _StreamerCLI_t StreamerCLI_t;

/**
 *  StreamerCLI_t internal structure.
 */
struct _StreamerCLI_t {
  /**
   * Base class object.
   */
  AManagedTaskEx super;

  /* Task variables should be added here. */

  /**
   * Task input message queue. The task receives message of type struct CtrlMessage_t in this queue.
   * This is one of the way the task expose its services at application level.
   */
  QueueHandle_t in_queue;

  /**
   * Input buffer to process the commands from the console.
   */
  char in_string[SCLI_TASK_CFG_MAX_IN_LENGTH];

  /**
   * Output buffer to process assembly the command answer to sent to the console.
   */
  char out_string[SCLI_TASK_CFG_MAX_OUT_LENGTH];

  /**
   * index of the insertion point in the in_buffer.
   */
  uint16_t in_index;

  /**
   * Specifies the duration in ms of the next phase.
   */
  TickType_t timer_period_ms;

  /**
   * Sensor interface. This is or the sensor selected during the start of an execution phase,
   * or NULL.
   */
  ISourceObservable *p_selected_sensor;

  /**
   * Specifies the listener interface to receive the sensor event.
   */
  IDataEventListener_t sensor_listener;

  /**
   * CircularBuffer handle used to decouple the sensor producing data from teh StreamerCLI
   * that consume the data.
   */
  CBHandle_t cb;

  /**
   * Specifies the number of items to allocate for the circular buffer.
   */
  uint16_t cb_items;

  /**
   * Specifies the number of element for each Circular Buffer Item. Its semantic depends on the number of dimension of the data.
   */
  uint32_t cbi_elements;

  /**
   * It depends on the data format of the enabled sensor, and the number of elements we want to print each time.
   */
  EMData_t out_data_format;

  /**
   * Specifies a software timer used to stop the streamer phase.
   */
  TimerHandle_t stop_timer;

  /**
   * Function used to set the default sensor configuration.
   */
  SensorsConfigF def_sensors_config_f;

};


/* Public API declaration */
/**************************/

/**
 * Allocate an instance of StreamerCLI_t.
 *
 * @return a pointer to the generic object ::AManagedTaskEx if success,
 * or NULL if out of memory error occurs.
 */
AManagedTaskEx *StreamerCLIAlloc(void);

/**
 * Get the task input queue. The application can use the services exported by the task by sending message
 * of type struct CtrlMessage_t to this queue.
 *
 * @param _this [IN] specifies a pointer to a task object.
 * @return an handle to the task input message queue.
 */
static inline QueueHandle_t StreamerCLIGetInQueue(StreamerCLI_t *_this);

/**
 * The application can specifies the default sensors configuration by providing its own configuration function.
 *
 * @param _this [IN] specifies a pointer to a task object.
 * @param def_sensors_config_f [IN] specify a pointer to sensors configuration function.
 * @return SYS_NO_ERROR_CODE
 */
sys_error_code_t StreamerCLISetDefSensorsConfig(StreamerCLI_t *_this, SensorsConfigF def_sensors_config_f);

/**
 *
 *
 * @param _this
 * @param cb_items
 * @param cbi_elements
 * @return
 */
sys_error_code_t StreamerCLISetBufferSize(StreamerCLI_t *_this, uint16_t cb_items, uint32_t cbi_elements);


/* Inline functions definition */
/*******************************/

SYS_DEFINE_STATIC_INLINE
QueueHandle_t StreamerCLIGetInQueue(StreamerCLI_t *_this)
{
  assert_param(_this != NULL);
  return _this->in_queue;
}

#ifdef __cplusplus
}
#endif

#endif /* INC_APPCONTROLLER_H_ */
