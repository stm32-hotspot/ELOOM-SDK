/**
 ******************************************************************************
 * @file    mx.c
 * @author  SRA - GPM
 * 
 * @date    Apr 9, 2021
 *
 * @brief Project peripherals configuration files.
 *
 * This files defines the peripherals configuration parameters that can be used
 * in the Low Level API of an eLooM driver - IDrvInit(). It creates a link
 * between the initialization code generated by CubeMX (see the .ioc file
 * in the tools folder) and the eLooM framework.
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

#include "mx.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"
#include "spi.h"
#include "i2c.h"
#include "dma.h"

/**
 * SPI3 initialization parameters.
 */
const MX_SPIParams_t MX_SPI3InitParams = {
    .p_mx_init_f = MX_SPI3_Init,
    .p_spi_handle  = &hspi3,
    .p_mx_dma_init_f = MX_DMA_InitCustom,
    .spi_dma_rx_irq_n = DMA1_Channel1_IRQn,
    .spi_dma_tx_irq_n = DMA1_Channel2_IRQn
};

/**
 * I2C2 initialization parameters.
 */
const MX_I2CParams_t MX_I2C2InitParams = {
    .p_mx_init_f = MX_I2C2_Init,
    .p_i2c_handle = &hi2c2,
    .p_mx_dma_init_f = MX_DMA_InitCustom,
    .i2c_er_irq_n = I2C2_ER_IRQn,
    .i2c_ev_irq_n = I2C2_EV_IRQn,
    .i2c_dma_rx_irq_n = DMA1_Channel3_IRQn,
    .i2c_dma_tx_irq_n = DMA1_Channel4_IRQn
};

/**
 * TIM5 initialization parameters.
 */
const MX_TIMParams_t MX_TIM5InitParams = {
    .p_mx_init_f = MX_TIM5_Init,
    .p_tim = &htim5,
    .irq_n = TIM5_IRQn
};

/**
 * PE0 (USER_BUTTON) Initialization parameters.
 */
const MX_GPIOParams_t MX_GPIO_PE0InitParams = {
    MX_GPIO_PE0_Init,
    USER_BUTTON_EXTI_IRQn,
    GPIO_PIN_0,
    GPIOE
};

/**
 * PE1 (LED_1) Initialization parameters.
 */
const MX_GPIOParams_t MX_GPIO_PE1InitParams = {
    MX_GPIO_PE1_Init,
    UsageFault_IRQn, /* not used. */
    LED1_Pin,
    LED1_GPIO_Port
};

/**
 * PF13 Initialization parameters.
 */
const MX_GPIOParams_t MX_GPIO_PF13InitParams = {
    MX_GPIO_PF13_Init,
    UsageFault_IRQn, /*!< NOT USED */
    GPIO_PIN_13,
    GPIOF
};

/**
 * PE8 Initialization parameters.
 */
const MX_GPIOParams_t MX_GPIO_PE8InitParams = {
    MX_GPIO_PE8_Init,
    EXTI9_5_IRQn,
    GPIO_PIN_8,
    GPIOE
};

/**
 * PF5 Initialization parameters.
 */
const MX_GPIOParams_t MX_GPIO_PF5InitParams = {
    MX_GPIO_PF5_Init,
    UsageFault_IRQn, /*!< NOT USED */
    GPIO_PIN_5,
    GPIOF
};

/**
 * PE14 Initialization parameters.
 */
const MX_GPIOParams_t MX_GPIO_PE14InitParams = {
    MX_GPIO_PE14_Init,
    EXTI15_10_IRQn,
    GPIO_PIN_14,
    GPIOE
};

/**
 * PG6 Initialization parameters.
 */
const MX_GPIOParams_t MX_GPIO_PG6InitParams = {
    MX_GPIO_PG6_Init,
    EXTI9_5_IRQn,
    GPIO_PIN_6,
    GPIOG
};