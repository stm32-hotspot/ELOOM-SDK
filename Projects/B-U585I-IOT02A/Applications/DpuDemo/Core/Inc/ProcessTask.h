/**
 ******************************************************************************
 * @file    ProcessTask.h
 * @author  SRA - GPM
 * 
 * 
 *
 * @brief
 *
 *
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file in
 * the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 ******************************************************************************
 */
#ifndef PROCESSTASK_H_
#define PROCESSTASK_H_

#ifdef __cplusplus
extern "C" {
#endif


#include "services/AManagedTaskEx.h"
#include "services/AManagedTaskEx_vtbl.h"
#include "Dummy_DPU2.h"
#include "Dummy_DPU2_vtbl.h"
#include "T1toT2DataBuilder.h"
#include "T1toT2DataBuilder_vtbl.h"

#include "queue.h"

/**
 * Create  type name for _ProcessTask_t.
 */
typedef struct _ProcessTask_t ProcessTask_t;

/**
 *  ProcessTask_t internal structure.
 */
struct _ProcessTask_t {
  /**
   * Base class object.
   */
  AManagedTaskEx super;

  /* Task variables should be added here. */

  /**
   * Task input message queue. The task receives message of type struct AIMessage_t in this queue.
   * This is one of the way the task expose its services at application level.
   */
  QueueHandle_t in_queue;

  /**
   * Digital processing Unit specialized for nothing.
   */
  Dummy_DPU2_t *p_dpu;

  /**
   * Data builder object to connect the DPU to teh sensor.
   */
//  Int16toFloatDataBuilder_t data_builder;
//  DefDataBuilder_t data_builder;
  T1toT2DataBuilder_t data_builder;

  /**
   * Data buffer used by the DPU but allocated by the task. The size of the buffer depend
   * on many parameters like:
   * - the type of the data used as input by the DPU
   * - the length of the signal
   * - the number of signals to manage in a circular way in order to decouple the data producer from the data process.
   *
   * The DPU uses this buffer to store one or more input signals to be processed, depending on the size of the buffer.
   * The correct size in byte for one signal is computed by the DPU with the method ADPU2_GetInDataPayloadSize().
   */
  void *p_dpu_in_buff;

  /**
   * Data buffer used by the DPU but allocated by the task. It is used by the DPU to store the result of the processing.
   * The correct size in byte for one signal is computed by the DPU with the method ADPU2_GetOutDataPayloadSize().
   */
  void *p_dpu_out_buff;
};


/* Public API declaration */
/**************************/

/**
 * Allocate an instance of ProcessTask_t.
 *
 * @return a pointer to the generic object ::AManagedTaskEx if success,
 * or NULL if out of memory error occurs.
 */
AManagedTaskEx *ProcessTaskAlloc(void);

/**
 * Get the task input queue. The application can use the services exported by the task by sending message
 * of type struct ProcMessage_t to this queue. The command ID for all supported command are defined in
 * ProcMessagesDef.h
 *
 * @param _this [IN] specifies a pointer to a task object.
 * @return an handle to the task input message queue.
 */
static inline QueueHandle_t ProcessTaskGetInQueue(ProcessTask_t *_this);

/**
 * Connect a sensor to the task as data source.
 *
 * @param _this [IN] specifies a pointer to a task object.
 * @param p_sensor [IN] specifies a pointer to the sensor to be connected.
 * @return SYS_NO_ERROR_CODE if success, and error code otherwise.
 */
sys_error_code_t ProcessTaskAttachToDataSrc(ProcessTask_t *_this, ISourceObservable *p_sensor);

/**
 * Add a process listener to the AiDPU owned by the task.
 *
 * @param _this [IN] specifies a pointer to a task object.
 * @param p_listener [IN] specifies the ::IDataEventListener_t object to add.
 * @return SYS_NO_ERROR_CODE if success, an error code otherwise.
 */
sys_error_code_t ProcessTaskAddDPUListener(ProcessTask_t *_this, IDataEventListener_t *p_listener);

/**
 * Remove a process listener to the AiDPU owned by the task.
 *
 * @param _this [IN] specifies a pointer to a task object.
 * @param p_listener [IN] specifies the ::IDataEventListener_t object to remove.
 * @return SYS_NO_ERROR_CODE if success, an error code otherwise.
 */
sys_error_code_t ProcessTaskRemoveDPUListener(ProcessTask_t *_this, IDataEventListener_t *p_listener);


/* Inline functions definition */
/*******************************/

SYS_DEFINE_STATIC_INLINE
QueueHandle_t ProcessTaskGetInQueue(ProcessTask_t *_this)
{
  assert_param(_this != NULL);
  return _this->in_queue;
}


#ifdef __cplusplus
}
#endif

#endif /* PROCESSTASK_H_ */
