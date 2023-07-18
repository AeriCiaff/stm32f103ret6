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
#include "usart.h"
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

unsigned char ForwardSensor;
unsigned char BackwardSensor;

float initialMotorSpeed = 0.4; //初始速度
float Kp; //比例系数
float Ki; //积分系数
float Kd; //微分系数
float expectVal; //期望值
float realVal; //实际值
float currentValue; //当前误差
char error; //实际误差
float controlIncrementalValue; //控制增量
float controlIncrementalVal; //返回的控制增量
float controlVal; //控制量
float lastVal = 0; //上次误差
float nextLastVal = 0; //上上次误差
float P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;

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

void ReadSensor(void)
{
	ForwardSensor = (
		(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) << 7) |
		(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) << 6) |
		(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) << 5) |
		(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) << 4) |
		(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) << 3) |
		(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) << 2) |
		(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)  << 1) |
		(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)  << 0)
	);
}

void CalculateError(void)
{
	switch (ForwardSensor)
	{
		// 1个的情况
		case 0b10000000:
			error = -4; 
			break;
		case 0b01000000:
			error = -3;
			break;
		case 0b00100000:
			error = -2;
			break;
		case 0b00010000:
			error = -1;
			break;
		case 0b00001000:
			error = 1;
			break;
		case 0b00000100:
			error = 2;
			break;
		case 0b00000010:
			error = 3;
			break;
		case 0b00000001:
			error = 4;
			break;
		// 2个的情况
		case 0b11000000:
			error = -3;
			break;
		case 0b01100000:
			error = -2;
			break;
		case 0b00110000:
			error = -1;
			break;
		case 0b00011000:
			error = 0;
			break;
		case 0b00001100:
			error = 1;
			break;
		case 0b00000110:
			error = 2;
			break;
		case 0b00000011:
			error = 3;
			break;
		// 3个的情况
		case 0b11100000:
			error = -2;
			break;
		case 0b00000111:
			error = 2;
			break;
		//大左转
		case 0b00001111:
			error = 100;
			break;
		//大右转
		case 0b11110000:
			error = 101;
			break;
		//停
		case 0b11111111:
			error = 102;
			break;
	}
}

float incrementalPID(float expectValue, float realValue)//增量式PID（期望值，实际值）
{
	currentValue = expectValue - realValue;//计算当前误差
	//计算控制增量
	controlIncrementalValue = Kp * (currentValue - lastVal) + Ki * currentValue + Kd * ((currentValue - lastVal) - (lastVal - nextLastVal));
	nextLastVal = lastVal;//更新上上次误差
	lastVal = currentValue;//更新上次误差
	return controlIncrementalValue;
}

void calculateControlValue()//计算控制量
{
	controlIncrementalVal = incrementalPID(0, error); //调用函数获得控制增量
	controlVal = controlVal + controlIncrementalVal;//计算控制量
}

void motor_control()
{
	//计算影响之后电机的转速
	float leftMotorSpeed = initialMotorSpeed - controlVal;
	float rightMotorSpeed = initialMotorSpeed + controlVal;
	
	SetSpeed(rightMotorSpeed,FORWARD,leftMotorSpeed,FORWARD);
	
}

void forward()
{
	SetSpeed(0.4,FORWARD,0.4,FORWARD);
}

void reverse()
{
	SetSpeed(0.4,BACKWARD,0.4,BACKWARD);
}

void right()
{
	SetSpeed(0.2,FORWARD,0.4,FORWARD);
}

void left()
{
	SetSpeed(0.4,FORWARD,0.2,FORWARD);
}

void sharpRightTurn()
{
	SetSpeed(0.4,BACKWARD,0.4,FORWARD);
}

void sharpLeftTurn()
{
	SetSpeed(0.4,FORWARD,0.4,BACKWARD);
}

void stop()
{
	SetSpeed(0,FORWARD,0,FORWARD);
}

void Go(void)
{
	ReadSensor();// 读取前向八路灰度传感器的情况
	CalculateError();// 判断误差error
	switch(error)
	{
		//大左转
		case 100:
			if(error == 100)
			{
				forward();
				HAL_Delay(1000);
				stop();
				do{
					ReadSensor();
					sharpLeftTurn();
				}while(error != 0 || error != 1);
			}
			break;
		//大右转
		case 101:
			if(error == 100)
			{
				forward();
				HAL_Delay(1000);
				stop();
				do{
					ReadSensor();
					sharpRightTurn();
				}while(error != 0 || error != 1);
			}
			break;
		//停车
		case 102:
			do{
				ReadSensor();
				stop();
			}while(error != 0 || error != 1);
			break;
		default:
			calculateControlValue();//计算控制量
			motor_control();
	}
}
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
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim5);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//	unsigned char *arr[] = {"-4","-3","-2","-1","0","1","2","3","4"};
//	unsigned char **p = &arr[4];
	while (1)
	{
//		HAL_UART_Transmit_IT(&huart5,p[error],2);
//		HAL_Delay(500);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		Go();
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
