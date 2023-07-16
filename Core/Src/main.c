/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "main.h"
#include "tim.h"
#include "gpio.h"

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
int SensorPin1;
int SensorPin2;
int SensorPin3;
int SensorPin4;
int SensorPin5;
int SensorPin6;
int SensorPin7;

int error;

enum Direction
{
	FORWARD,
	BACKWARD
};

void SetSpeed
(float RightDuty,
	enum Direction RightDirection,
	float LeftDuty,
	enum Direction LeftDirection)
{
	int CounterPeriod, Compare;
	CounterPeriod = __HAL_TIM_GET_AUTORELOAD(&htim5);
	Compare = (int)(CounterPeriod * LeftDuty);
	switch (LeftDirection)
	{
	case FORWARD: 
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, Compare);
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
		break;
	case BACKWARD:
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, Compare);
		break;
	}
	Compare = (int)(CounterPeriod * RightDuty);
	switch (RightDirection)
	{
	case FORWARD: 
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, Compare);
		break;
	case BACKWARD:
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, Compare);
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);
		break;
	}
}

void ReadSensorValue()
{
	if (SensorPin1 == 0 && SensorPin2 == 0 && SensorPin3 == 0 && SensorPin4 == 1)
	{
		error = 2;
	}
	else if(SensorPin1 == 0 && SensorPin2 == 0 && SensorPin3 == 1 && SensorPin4 == 1)
	{
		error = 1;
	}
	else if(SensorPin1 == 0 && SensorPin2 == 1 && SensorPin3 == 1 && SensorPin4 == 0)
	{
		error = 0;
	}
	else if(SensorPin1 == 1 && SensorPin2 == 1 && SensorPin3 == 0 && SensorPin4 == 0)
	{
		error = -1;
	}
	else if(SensorPin1 == 1 && SensorPin2 == 0 && SensorPin3 == 0 & SensorPin4 ==0)
	{
		error = -2;
	}
	else if(SensorPin1 == 0 && SensorPin2 == 0 && SensorPin3 == 0 && SensorPin4 == 0)
	{
		error = 404;
	}
}


void carForward()
{
	ReadSensorValue();
	switch(error)
	{
		case 2:
			do
			{
				SetSpeed(0.5, FORWARD, 0.3, FORWARD);
			}while(error != 0);
			break;
		case 1:
			do
			{
				SetSpeed(0.45, FORWARD, 0.35, FORWARD);
			}while(error != 0);
			break;
		case 0: SetSpeed(0.4, FORWARD, 0.4, FORWARD);break;
		case -1:
			do
			{
				SetSpeed(0.35, FORWARD, 0.45, FORWARD);
			}while(error != 0);
			break;
		case -2:
			do
			{
				SetSpeed(0.3, FORWARD, 0.5, FORWARD);
			}while(error != 0);
			break;
//		case 100:
//			do
//			{
//				if(error == 100)
//			}while(error != 0);
//			break;
		case 404: SetSpeed(0,FORWARD, 0, FORWARD);break;
	}
}

//void carBACKWARD()
//{

//}

//void carTurnRight()
//{
//	
//}

//void carTurnLeft()
//{

//}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim5);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
	SensorPin1 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);
	SensorPin2 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);
	SensorPin3 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
	SensorPin4 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
    /* USER CODE END WHILE */
	
    /* USER CODE BEGIN 3 */
		carForward();
//		if(SensorPin1 == 1)
//		{
//			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
//		}
//		else {
//			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
//		}
//		if(SensorPin2 == 1)
//		{
//			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
//		}
//		else {
//			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
//		}
//		if(SensorPin3 == 1)
//		{
//			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
//		}
//		else {
//			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
//		}
//		if(SensorPin4 == 1)
//		{
//			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
//		}
//		else {
//			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
//		}
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
	   ex: printf("error parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
