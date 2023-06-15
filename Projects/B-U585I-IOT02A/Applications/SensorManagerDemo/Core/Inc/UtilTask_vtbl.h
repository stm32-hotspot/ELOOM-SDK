/**
 ******************************************************************************
 * @file    UtilTask_vtbl.h
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
#ifndef INC_UTILTASK_VTBL_H_
#define INC_UTILTASK_VTBL_H_

#ifdef __cplusplus
extern "C" {
#endif

/* AManagedTask virtual functions */
sys_error_code_t UtilTask_vtblHardwareInit(AManagedTask *_this, void *p_params); /*!< @sa AMTHardwareInit */
sys_error_code_t UtilTask_vtblOnCreateTask(AManagedTask *_this, TaskFunction_t *p_task_code, const char **p_name, unsigned short *p_stack_depth, void **p_params, UBaseType_t *p_priority); /*!< @sa AMTOnCreateTask */
sys_error_code_t UtilTask_vtblDoEnterPowerMode(AManagedTask *_this, const EPowerMode active_power_mode, const EPowerMode new_power_mode); /*!< @sa AMTDoEnterPowerMode */
sys_error_code_t UtilTask_vtblHandleError(AManagedTask *_this, SysEvent error); /*!< @sa AMTHandleError */
sys_error_code_t UtilTask_vtblOnEnterTaskControlLoop(AManagedTask *this); ///< @sa AMTOnEnterTaskControlLoop

/* AManagedTaskEx virtual functions */
sys_error_code_t UtilTask_vtblForceExecuteStep(AManagedTaskEx *_this, EPowerMode active_power_mode); /*!< @sa AMTExForceExecuteStep */
sys_error_code_t UtilTask_vtblOnEnterPowerMode(AManagedTaskEx *_this, const EPowerMode active_power_mode, const EPowerMode new_power_mode); /*!< @sa AMTExOnEnterPowerMode */

#ifdef __cplusplus
}
#endif

#endif /* INC_UTILTASK_VTBL_H_ */
