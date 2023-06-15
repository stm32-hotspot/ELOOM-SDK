/**
 ******************************************************************************
 * @file    sysconfig.h
 * @author  SRA - GPM
 * 
 * 
 * @brief   Global System configuration file
 *
 * This file include some configuration parameters grouped here for user
 * convenience. This file override the default configuration value, and it is
 * used in the "Preinclude file" section of the "compiler > prepocessor"
 * options.
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

#ifndef SYSCONFIG_H_
#define SYSCONFIG_H_

#include "SensorManager_conf.h"

// Drivers configuration
// *********************


// Other hardware configuration
// ****************************

#define SYS_DBG_AUTO_START_TA4                    0

// Services configuration
// **********************

// files syslowpower.h, SysDefPowerModeHelper.c
#define SYS_CFG_USE_DEFAULT_PM_HELPER             0
#define SYS_CFG_DEF_PM_HELPER_STANDBY             0  ///< if defined to 1 then the MCU goes in STANDBY mode when the system enters in SLEEP_1.

// file SysTimestamp.c
#define SYS_TS_CFG_ENABLE_SERVICE                 1
/**
 * Configuration parameter for the timer used for the eLooM timestamp service.
 * Valid value are:
 * - SYS_TS_USE_SW_TSDRIVER to use the RTOS tick
 * - The configuration structure for an hardware timer. It must be compatible with SysTimestamp_t type.
 */
//#define SYS_TS_CFG_TSDRIVER_PARAMS                &MX_TIM5InitParams
#define SYS_TS_CFG_TSDRIVER_PARAMS                SYS_TS_USE_SW_TSDRIVER

//#define SYS_TS_CFG_TSDRIVER_FREQ_HZ               SystemCoreClock ///< hardware timer clock frequency in Hz
#define SYS_TS_CFG_TSDRIVER_FREQ_HZ               configTICK_RATE_HZ ///< FreeRTOS clock frequency in Hz

// Tasks configuration
// *******************

// file IManagedTask.h
#define MT_ALLOWED_ERROR_COUNT                    0x2

// file sysinit.c
#define INIT_TASK_CFG_ENABLE_BOOT_IF              0
#define INIT_TASK_CFG_STACK_SIZE                  (configMINIMAL_STACK_SIZE*6)

// file HelloWorldTask.c
#define HW_TASK_CFG_STACK_DEPTH                   360
#define HW_TASK_CFG_PRIORITY                      (tskIDLE_PRIORITY+1)

// file ProcessTask.c
#define PRC_TASK_CFG_STACK_DEPTH                  (configMINIMAL_STACK_SIZE*12)
#define PRC_TASK_CFG_PRIORITY                     (tskIDLE_PRIORITY+2)


// file SCLIUtilTask.c
#define SCLIUTIL_TASK_CFG_STACK_DEPTH             ((configMINIMAL_STACK_SIZE*2) + (configMINIMAL_STACK_SIZE/2))
#define SCLIUTIL_TASK_CFG_PRIORITY                (tskIDLE_PRIORITY+1)


#endif /* SYSCONFIG_H_ */
