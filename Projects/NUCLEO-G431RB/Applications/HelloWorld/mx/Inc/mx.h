/**
 ******************************************************************************
 * @file    mx.h
 * @author  SRA - GPM
 * 
 * 
 *
 * @brief Project peripherals configuration files.
 *
 * This files declares the peripherals configuration parameters that can be used
 * in the Low Level API of an eLooM driver - IDrvInit(). It creates a link
 * between the initialization code generated by CubeMX (see the .ioc file
 * in the tools folder) and the eLooM framework.
 *
 * This example show how a managed task allocates an SPI driver that must be
 * connected to the SPI3 (that has been configured using CubeMX):
 * ~~~{.c}
 * p_task->p_driver = SPIMasterDriverAlloc();
 * if (p_task->p_driver != NULL)
 * {
 *   res = IDrvInit((IDriver*)p_task->p_driver, (void*)&MX_SPI3InitParams);
 *   if (SYS_IS_ERROR_CODE(res)) {
 *     SYS_DEBUGF(SYS_DBG_LEVEL_SEVERE, ("SPIBus task: error during driver initialization\r\n"));
 *   }
 * }
 * ~~~
 *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */
#ifndef INCLUDE_MX_MX_H_
#define INCLUDE_MX_MX_H_

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Driver initialization parameters */
/************************************/

/**
 * GPIO Configuration parameters.
 */
typedef struct _MX_GPIOParams_t
{
  void (*p_mx_init_f)(void); /*!< MX GPIO initialization function */
  IRQn_Type irq_n; /*!< External interrupt number. */
  uint32_t pin;
  GPIO_TypeDef *port;
} MX_GPIOParams_t;

/**
 * TIM configuration parameters.
 */
typedef struct _MX_TIMParams_t
{
  TIM_HandleTypeDef *p_tim; /*!< HAL TIM handle */
  IRQn_Type irq_n; /*!< External interrupt number. */
  void (*p_mx_init_f)(void); /*!< MX TIM initialization function */
} MX_TIMParams_t;

/**
 * I2C configuration parameters.
 */
typedef struct _MX_I2CParams_t
{
  I2C_HandleTypeDef *p_i2c_handle; /*!< HAL I2C handle */
  IRQn_Type i2c_ev_irq_n; /*!< I2C EV interrupt number. */
  IRQn_Type i2c_er_irq_n; /*!< I2C ER interrupt number. */
  IRQn_Type i2c_dma_rx_irq_n; /*!< DMA channel rx interrupt number. */
  IRQn_Type i2c_dma_tx_irq_n; /*!< DMA channel tx interrupt number. */
  void (*p_mx_init_f)(void); /*!< MX I2C initialization function */
  void (*p_mx_dma_init_f)(void); /*!< MX DMA initialization function */
} MX_I2CParams_t;

/**
 * (USER_BUTTON_1) configuration parameters.
 */
extern const MX_GPIOParams_t MX_GPIO_UB1InitParams;

/**
 * (LD2) configuration parameters.
 */
extern const MX_GPIOParams_t MX_GPIO_LD2InitParams;

/**
 * TIM7 configuration parameters.
 */
extern const MX_TIMParams_t MX_TIM7InitParams;

/**
 * TIM16 configuration parameters.
 */
extern const MX_TIMParams_t MX_TIM16InitParams;

/**
 * I2C1 configuration parameters.
 */
extern const MX_I2CParams_t MX_I2C1InitParams;

/**
 * ISM330DHCX INT1 GPIO configuration parameters.
 */
extern const MX_GPIOParams_t MX_GPIO_INT1_DHCXInitParams;

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_MX_MX_H_ */
