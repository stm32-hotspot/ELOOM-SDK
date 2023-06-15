/**
 ******************************************************************************
 * @file    SPIBusIF.h
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
#ifndef SPIBUSIF_H_
#define SPIBUSIF_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "ABusIF.h"
#include "FreeRTOS.h"
#include "semphr.h"


/**
 * Create a type name for _SPIBusIF.
 */
typedef struct _SPIBusIF SPIBusIF;

/**
 * Specifies the SPI interface for a generic sensor.
 */
struct _SPIBusIF {
  /**
   * The bus connector encapsulates the function pointer to read and write in the bus,
   * and it is compatible with the the ST universal sensor driver.
   */
  ABusIF super;

  /**
   * Slave select GPIO Port
   */
  GPIO_TypeDef* m_pxSSPinPort;

  /**
   * Slave select GPIO Pin.
   */
  uint16_t m_nSSPin;

  /**
   * Address auto-increment (Multi-byte read/write).
   */
  uint8_t m_nAutoInc;

  /**
   * Synchronization object used to synchronize the sensor with the bus.
   */
  SemaphoreHandle_t m_xSyncObj;
};


// Public API declaration
// **********************

/**
 * Initialize a sensor object. It must be called once before using the sensor.
 *
 * @param _this [IN] specifies a sensor object.
 * @param nWhoAmI [IN] specifies the sensor ID. It can be zero.
 * @param pxSSPinPort [IN] specifies the port PIN of the Slave Select line.
 * @param nSSPin [IN] specifies the pin number of the Slave Select line.
 * @param nAutoInc [IN] specifies the SPI address auto-increment to allow multiple data read/write.
 * @return SYS_NO_EROR_CODE if success, an error code otherwise.
 */
ABusIF *SPIBusIFAlloc(uint8_t nWhoAmI, GPIO_TypeDef* pxSSPinPort, uint16_t nSSPin, uint8_t nAutoInc);

sys_error_code_t SPIBusIFWaitIOComplete(SPIBusIF *_this);
sys_error_code_t SPIBusIFNotifyIOComplete(SPIBusIF *_this);


// Inline function definition
// **************************


#ifdef __cplusplus
}
#endif

#endif /* SPIBUSIF_H_ */
