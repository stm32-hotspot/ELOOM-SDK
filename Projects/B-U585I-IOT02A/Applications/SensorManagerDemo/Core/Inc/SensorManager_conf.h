/**
 ******************************************************************************
 * @file    SensorManager_conf.h
 * @author  SRA - GPM
 * @brief   Global System configuration file
 *
 * This file include some configuration parameters grouped here for user
 * convenience. This file override the default configuration value, and it is
 * used in the "Preinclude file" section of the "compiler > prepocessor"
 * options.
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
 *                             
 *
 ******************************************************************************
 */

#ifndef SENSORMANAGERCONF_H_
#define SENSORMANAGERCONF_H_


#define IIS2DLPC_FIFO_ENABLED 1
#define IIS2ICLX_FIFO_ENABLED 1
#define IIS3DWB_FIFO_ENABLED 1
#define ILPS22QS_FIFO_ENABLED 1
#define ISM330DHCX_FIFO_ENABLED 1
#define LPS22DF_FIFO_ENABLED	0
#define LPS22HH_FIFO_ENABLED	1

//#define HSD_USE_DUMMY_DATA 1

//#define EXTERNAL_IIS3DWB 1

// file ISM330DHCXTask.c
#define ISM330DHCX_TASK_CFG_STACK_DEPTH           ((configMINIMAL_STACK_SIZE*4) + (configMINIMAL_STACK_SIZE/2))
#define ISM330DHCX_TASK_CFG_PRIORITY              (tskIDLE_PRIORITY+4)

// file IIS3DWBTask.c
#define IIS3DWB_TASK_CFG_STACK_DEPTH              ((configMINIMAL_STACK_SIZE*4) + (configMINIMAL_STACK_SIZE/2))
#define IIS3DWB_TASK_CFG_PRIORITY                 (tskIDLE_PRIORITY+4)

// file SPIBusTask.c
#define SPIBUS_TASK_CFG_STACK_DEPTH               ((configMINIMAL_STACK_SIZE*2) + (configMINIMAL_STACK_SIZE/2))
#define SPIBUS_TASK_CFG_PRIORITY                  (tskIDLE_PRIORITY+4)

// file HTS221Task.c
#define HTS221_TASK_CFG_STACK_DEPTH               ((configMINIMAL_STACK_SIZE*4) + (configMINIMAL_STACK_SIZE/2))
#define HTS221_TASK_CFG_PRIORITY                  (tskIDLE_PRIORITY+4)

// file I2CBusTask.c
#define I2CBUS_TASK_CFG_STACK_DEPTH               ((configMINIMAL_STACK_SIZE*2) + (configMINIMAL_STACK_SIZE/2))
#define I2CBUS_TASK_CFG_PRIORITY                  (tskIDLE_PRIORITY+4)



#endif /* SENSORMANAGERCONF_H_ */
