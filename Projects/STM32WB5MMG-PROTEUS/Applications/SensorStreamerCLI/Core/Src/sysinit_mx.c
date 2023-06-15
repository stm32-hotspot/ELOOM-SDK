/**
 ******************************************************************************
 * @file    sysinit_mx.c
 * @author  SRA - GPM
 * 
 * 
 *
 * @brief
 *
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
 *
 ******************************************************************************
 */

#include "services/systp.h"
#include "services/syserror.h"
#include "mx.h"
#include "stm32wbxx_ll_hsem.h"


//Select the SystemClock_Config
//#define SystemClock_Config
#define SystemClock_Config_MX SystemClock_Config

//Remap the HAL error handler function to eLooM default error handler function.
#define Error_Handler sys_error_handler


/**
 * This type group together the components of the clock three to be modified during
 * a power mode change.
 */
typedef struct _system_clock_t{
  uint32_t latency;
  RCC_OscInitTypeDef osc;
  RCC_ClkInitTypeDef clock;
  RCC_PeriphCLKInitTypeDef periph_clock;
} system_clock_t;

/**
 * It is used to save and restore the system clock during the power mode switch.
 */
static system_clock_t system_clock;


// Private member function declaration
// ***********************************


// Public API definition
// *********************

void SystemClock_Config_MX(void) {

  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_HIGH);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_LSI1|RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS|RCC_PERIPHCLK_RFWAKEUP
                              |RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_LPUART1
                              |RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInitStruct.RFWakeUpClockSelection = RCC_RFWKPCLKSOURCE_HSE_DIV1024;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSE;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Smps */

  /**
   *  Configure and enable SMPS
   *
   *  The SMPS configuration is not yet supported by CubeMx
   *  when SMPS output voltage is set to 1.4V, the RF output power is limited to 3.7dBm
   *  the SMPS output voltage shall be increased for higher RF output power
   */

  #if CFG_USE_SMPS == 1

  LL_PWR_SMPS_SetStartupCurrent(LL_PWR_SMPS_STARTUP_CURRENT_80MA);
  LL_PWR_SMPS_SetOutputVoltageLevel(LL_PWR_SMPS_OUTPUT_VOLTAGE_1V40);
  LL_PWR_SMPS_Enable();

  #endif

  /* USER CODE END Smps */

  /**
    * This prevents the CPU2 to disable the HSI48 oscillator when
    * it does not use anymore the RNG IP
    */

  LL_HSEM_1StepLock( HSEM, 5 );

}

void SystemClock_Backup(void)
{
  HAL_RCC_GetOscConfig(&(system_clock.osc));
  HAL_RCC_GetClockConfig(&(system_clock.clock), &(system_clock.latency));
  HAL_RCCEx_GetPeriphCLKConfig(&(system_clock.periph_clock));
}

/**
  * @brief  Restore original clock parameters
  * @retval Process result
  *         @arg SMPS_OK or SMPS_KO
  */
void SystemClock_Restore(void)
{
  /* SEQUENCE:
   *   1. Set PWR regulator to SCALE1_BOOST
   *   2. PLL ON
   *   3. Set SYSCLK source to PLL
   *
   * NOTE:
   *   Do not change or update the base-clock source (e.g. MSI and LSE)
   */

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
    sys_error_handler();
  }

//  if (__HAL_RCC_GET_SYSCLK_SOURCE() != RCC_CFGR_SWS_PLL) {
//    if (HAL_RCC_OscConfig(&(system_clock.osc)) != HAL_OK) {
//      sys_error_handler();
//    }
//  }

  if (HAL_RCC_ClockConfig(&(system_clock.clock), system_clock.latency) != HAL_OK) {
    sys_error_handler();
  }
}

void SysPowerConfig() {

// This function is called in the early step of the system initialization.
// All the PINs used by the application are reconfigured later by the application tasks.

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pins : PA2 PA1 PA0 PA10
                           PA8 PA9 PA7 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_1|GPIO_PIN_0|GPIO_PIN_10
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_7|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC3 PC2 PC10 PC11
                           PC12 PC6 PC13 PC5
                           PC9 PC7 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_6|GPIO_PIN_13|GPIO_PIN_5
                          |GPIO_PIN_9|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB9 PB8 PB7 PB4
                           PB3 PB14 PB15 PB6
                           PB12 PB11 PB10 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_8|GPIO_PIN_7|GPIO_PIN_4
                          |GPIO_PIN_3|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_6
                          |GPIO_PIN_12|GPIO_PIN_11|GPIO_PIN_10|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 PD14 PD13
                           PD7 PD3 PD4 PD9
                           PD15 PD10 PD5 PD6
                           PD11 PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_14|GPIO_PIN_13
                          |GPIO_PIN_12|GPIO_PIN_7|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_9|GPIO_PIN_15|GPIO_PIN_10|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{

  /* Set Interrupt Group Priority */
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* HSEM_IRQn interrupt configuration */
  __HAL_RCC_HSEM_CLK_ENABLE();
  HAL_NVIC_SetPriority(HSEM_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(HSEM_IRQn);

  // MemoryManagement_IRQn interrupt configuration
  HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);

  /*Configure the internal voltage reference buffer voltage scale */
  HAL_SYSCFG_VREFBUF_VoltageScalingConfig(SYSCFG_VREFBUF_VOLTAGE_SCALE1);
  /* Enable the Internal Voltage Reference buffer */
  HAL_SYSCFG_EnableVREFBUF();
  /* Configure the internal voltage reference buffer high impedance mode */
  HAL_SYSCFG_VREFBUF_HighImpedanceConfig(SYSCFG_VREFBUF_HIGH_IMPEDANCE_DISABLE);

}

// Private function definition
// ***************************
