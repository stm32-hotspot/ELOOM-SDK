/**
 ******************************************************************************
 * @file    SPIBusIF.c
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

#include "SPIBusIF.h"

// Private functions declaration
// *****************************


// Private variables
// *****************


// Public API implementation.
// **************************

ABusIF *SPIBusIFAlloc(uint8_t nWhoAmI, GPIO_TypeDef *pxSSPinPort, uint16_t nSSPin, uint8_t nAutoInc)
{
  SPIBusIF *_this = NULL;

  _this = pvPortMalloc(sizeof(SPIBusIF));
  if(_this != NULL) {
    ABusIFInit(&_this->super, nWhoAmI);

    _this->m_pxSSPinPort = pxSSPinPort;
    _this->m_nSSPin = nSSPin;
    _this->m_nAutoInc = nAutoInc;

    // initialize the software resources
    _this->m_xSyncObj = xSemaphoreCreateBinary();
    if (_this->m_xSyncObj == NULL){
        vPortFree(_this);
        _this = NULL;
      }
      else {
        ABusIFSetHandle(&_this->super, _this);
#ifdef DEBUG
        vQueueAddToRegistry(_this->m_xSyncObj, "SPI_IP_S");
#endif
      }
  }

  return (ABusIF *)_this;
}

sys_error_code_t SPIBusIFWaitIOComplete(SPIBusIF *_this)
{
  assert_param(_this);
  sys_error_code_t xRes = SYS_NO_ERROR_CODE;

  if (_this->m_xSyncObj != NULL){
    if (pdTRUE != xSemaphoreTake(_this->m_xSyncObj, portMAX_DELAY)) {
      SYS_SET_SERVICE_LEVEL_ERROR_CODE(SYS_UNDEFINED_ERROR_CODE);
      xRes = SYS_UNDEFINED_ERROR_CODE;
    }
  }
  else {
    SYS_SET_SERVICE_LEVEL_ERROR_CODE(SYS_INVALID_FUNC_CALL_ERROR_CODE);
    xRes = SYS_INVALID_FUNC_CALL_ERROR_CODE;
  }

  return xRes;
}

sys_error_code_t SPIBusIFNotifyIOComplete(SPIBusIF *_this)
{
  assert_param(_this);
  sys_error_code_t xRes = SYS_NO_ERROR_CODE;

  if (_this->m_xSyncObj != NULL){
    if (pdTRUE != xSemaphoreGive(_this->m_xSyncObj)) {
      SYS_SET_SERVICE_LEVEL_ERROR_CODE(SYS_UNDEFINED_ERROR_CODE);
      xRes = SYS_UNDEFINED_ERROR_CODE;
    }
  }
  else {
    SYS_SET_SERVICE_LEVEL_ERROR_CODE(SYS_INVALID_FUNC_CALL_ERROR_CODE);
    xRes = SYS_INVALID_FUNC_CALL_ERROR_CODE;
  }

  return xRes;
}


// Private functions definition
// ****************************


