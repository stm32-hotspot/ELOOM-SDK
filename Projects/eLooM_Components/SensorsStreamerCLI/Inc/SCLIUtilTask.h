/**
 ******************************************************************************
 * @file    SCLIUtilTask.h
 * @author  SRA - GPM
 * 
 * 
 *
 * @brief  Utility task declaration.
 *
 * The Utility task is in export minor functionalities of the application:
 * - Time stamp for the Sensor Manager
 * - Manage teh user button and the user LED1
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
#ifndef INC_SCLIUTILTASK_H_
#define INC_SCLIUTILTASK_H_

#ifdef __cplusplus
extern "C" {
#endif


#include "services/AManagedTaskEx.h"
#include "services/AManagedTaskEx_vtbl.h"
#include "queue.h"
#include "timers.h"

#define SCLIUTIL_CMD_ID_START_LP_TIMER                    ((uint16_t)0x0001)              ///< START Low Power timer command ID.
#define SCLIUTIL_CMD_ID_STOP_LP_TIMER                     ((uint16_t)0x0002)              ///< STOP Low Power timer command ID.
#define SCLIUTIL_CMD_ID_RESET_LP_TIMER                    ((uint16_t)0x0003)              ///< RESET Low Power timer command ID.


/**
 * Create  type name for _SCLIUtilTask_t.
 */
typedef struct _SCLIUtilTask_t SCLIUtilTask_t;

/**
 *  SCLIUtilTask_t internal structure.
 */
struct _SCLIUtilTask_t {
  /**
   * Base class object.
   */
  AManagedTaskEx super;

  /* Task variables should be added here. */

  /**
   * HAL driver configuration parameters for the hw GPIO used to blink a LED during the data streaming.
   * The actual type is MX_GPIOParams_t.
   */
  const void *p_mx_led_cfg;

  /**
   * HAL driver configuration parameters for the hw GPIO used to control button to start and stop the data streaming.
   */
  const void *p_mx_button_cfg;

  /**
   * Input queue used by other task to request services.
   */
  QueueHandle_t in_queue;

  /**
   * Software timer used to generate a transaction into low-power mode.
   */
  TimerHandle_t auto_low_power_timer;

  /**
   * specifies a timeout in ms. If the timeout is zero then the timer is disabled.
   */
  uint32_t timeout_ms;
};


/* Public API declaration */
/**************************/

/**
 * Allocate an instance of SCLIUtilTask.
 *
 * @param p_mx_led_drv_cfg [IN] specifies a ::MX_GPIOParams_t instance declared in the mx.h file. It can be NULL.
 * @param p_mx_button_drv_cfg [IN] specifies a ::MX_GPIOParams_t instance declared in the mx.h file. It can be NULL.
 * @return a pointer to the generic object ::AManagedTaskEx if success,
 * or NULL if out of memory error occurs.
 */
AManagedTaskEx *SCLIUtilTaskAlloc(const void *p_mx_led_drv_cfg, const void *p_mx_button_drv_cfg);

/**
 * Set teh period of the automatic low power mode (software) timer.
 * If the period is greater than zero then the timer is enabled. If the period is set to zero then
 * the timer is disabled. The timer is started when the system enter in STATE_1 PM. When the timer
 * elapse then the system goes in SLEEP_1.
 *
 * @param _this [IN] specifies a pointer to the task object.
 * @param timeout_ms [IN] specifies a timeout in ms. If the timeout is zero then the timer is disabled.
 * @return
 */
sys_error_code_t SCLIUtilTaskSetAutoLowPowerModePeriod(SCLIUtilTask_t *_this, uint32_t timeout_ms);


/* Inline functions definition */
/*******************************/


#ifdef __cplusplus
}
#endif

#endif /* INC_SCLIUTILTASK_H_ */
