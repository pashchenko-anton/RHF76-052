/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

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

uint8_t isDIO_0_Flag = 0;

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
  /* USER CODE BEGIN 1 */
	uint8_t LTX_BUF[255]="4321";
	uint8_t LRX_BUF[255]={0};
	uint8_t LTX_BUF_SIZE = 4;
	uint8_t LRX_BUF_SIZE = 0;

	uint8_t isReceived = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */


  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */


  /* USER CODE BEGIN SysInit */
	clockInit();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
	initSPI1();
	SX1276Init();
	SX1276_Init();

//	Delay(0xFFFFF);
	SX1276_SendTXBUF(LTX_BUF, LTX_BUF_SIZE);
	SX1276_CleanBuffer();
//	GPIOA->ODR &= ~GPIO_ODR_OD1;
//	GPIOA->ODR &= ~GPIO_ODR_OD2;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  if(isDIO_0_Flag)
//	  {
//		  isReceived = SX1276_ReadRXBUF(LRX_BUF, &LRX_BUF_SIZE);
//		  if(isReceived)
//		  {
//			  GPIOA->ODR |= GPIO_ODR_OD1;
//			  GPIOA->ODR |= GPIO_ODR_OD2;
////			  GPIOA->ODR &= ~GPIO_ODR_OD1;
////			  GPIOA->ODR &= ~GPIO_ODR_OD2;
//			  SX1276_SendTXBUF(LTX_BUF, LTX_BUF_SIZE);
//			  SX1276_CleanBuffer();
//			  isReceived = 0;
////			  GPIOA->ODR |= GPIO_ODR_OD1;
////			  GPIOA->ODR |= GPIO_ODR_OD2;
//			  GPIOA->ODR &= ~GPIO_ODR_OD1;
//			  GPIOA->ODR &= ~GPIO_ODR_OD2;
//		  }
//		  isDIO_0_Flag = 0;
//  	  }

	  Delay(0xFFFFFF);
	  SX1276_SendTXBUF(LTX_BUF, LTX_BUF_SIZE);
	  SX1276_CleanBuffer();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */

/* USER CODE BEGIN 4 */
//interrupt from DIO0 SX1276
void EXTI4_15_IRQHandler(void){

	isDIO_0_Flag = 1;
	EXTI->PR |= EXTI_PR_PR10;

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
