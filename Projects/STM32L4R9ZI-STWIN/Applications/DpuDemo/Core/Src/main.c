/**
 ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
 *
  * This fine defines the main() function and few other functions to integrate
  * the low layer of the firmware with the HAL and the error management.
  * Normally a developer does not need to modify this file.
  * The main application entry points, instead, are defined in the file App.c
 *
  ******************************************************************************
 * @attention
 *
  * Copyright (c) 2023 STMicroelectronics..
 * All rights reserved.
 *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/

#include <stdio.h>
#include "mx.h"
#include "services/sysinit.h"
#include "task.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  // System initialization

  SysInit(FALSE);

  vTaskStartScheduler();

  while (1);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  printf("Wrong parameters value: file %s on line %u\r\n", file, (unsigned int)line);
  sys_error_handler();
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

