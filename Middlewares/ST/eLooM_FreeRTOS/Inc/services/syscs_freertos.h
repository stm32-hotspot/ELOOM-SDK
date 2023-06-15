/**
 ******************************************************************************
 * @file    syscs_freertos.h
 * @author  STMicroelectronics - AIS - MCD Team
 * @version M.m.b
 * @date    May 27, 2022
 *
 * @brief
 *
 *
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file in
 * the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 ******************************************************************************
 */
#ifndef ELOOM_INC_SERVICES_SYSCS_FREERTOS_H_
#define ELOOM_INC_SERVICES_SYSCS_FREERTOS_H_

#ifdef __cplusplus
extern "C" {
#endif

/* MISRA messages linked to FreeRTOS include are ignored */
/*cstat -MISRAC2012-* */
#include "FreeRTOS.h"
#include "task.h"
/*cstat +MISRAC2012-* */


/* Public API declaration */
/**************************/

#define SYS_DECLARE_CS(cs)                     UBaseType_t cs=0

#define SYS_ENTER_CRITICAL(cs)                 \
    if (SYS_IS_CALLED_FROM_ISR()) {            \
      (cs) = taskENTER_CRITICAL_FROM_ISR(); \
    }                                          \
    else {                                     \
      taskENTER_CRITICAL();                    \
    }

#define SYS_EXIT_CRITICAL(cs)                  \
    if (SYS_IS_CALLED_FROM_ISR()) {            \
      taskEXIT_CRITICAL_FROM_ISR((cs));        \
    }                                          \
    else {                                     \
      taskEXIT_CRITICAL();                     \
    }


/* Inline functions definition */
/*******************************/


#ifdef __cplusplus
}
#endif

#endif /* ELOOM_INC_SERVICES_SYSCS_FREERTOS_H_ */
