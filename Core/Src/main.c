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
#include "HCSR04p.h"
#include "L298N.h"
#include "Button.h"
#include "RingBuffer.h"
#include "complex_parser.h"
#include "utils.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HCSR04p_TRIGGER_TIMER htim1
#define HCSR04p_ECHO_TIMER htim1
#define HCSR04p_TRIG_CHANNEL TIM_CHANNEL_3
#define HCSR04p_START_CHANNEL TIM_CHANNEL_1
#define HCSR04p_STOP_CHANNEL TIM_CHANNEL_2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t Distance_u16;
float Distance_f;
HCSR04p_t HCSR04p_front;
float HoldDistance_value = 0.0;
float* ptrHoldDistance_value = &HoldDistance_value;

bool ReadDist = false;
bool* ptrReadDist = &ReadDist;

bool RobotEnable = false;
bool* ptrRobotEnable = &RobotEnable;


Button_t BlueKey;

uint8_t Msg[30];
uint8_t Length;

uint8_t HC05_Command[RING_BUFFER_SIZE];
HAL_StatusTypeDef Status_RX, Status_TX;

bool Motor_Action_Flag = false;
bool* ptrMotor_Action_Flag = &Motor_Action_Flag;
Motor_t LeftMotor, RightMotor;

RingBuffer_t RX_Buffer;
uint8_t RX_Temp;
uint8_t RX_Lines;
uint8_t Recevied_Data[RING_BUFFER_SIZE];

PID_t Distance_PID;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void Hold_Distance(float* Distance);
void Robot_Operation();
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
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HCSR04p_Init(&HCSR04p_front, &HCSR04p_TRIGGER_TIMER, HCSR04p_TRIG_CHANNEL, &HCSR04p_ECHO_TIMER, HCSR04p_START_CHANNEL, HCSR04p_STOP_CHANNEL);

  Button_Init(&BlueKey, B1_GPIO_Port, B1_Pin, 20);

  L298N_MotorInit(&LeftMotor, LeftMotor_FWD_GPIO_Port, LeftMotor_FWD_Pin, LeftMotor_BWD_GPIO_Port, LeftMotor_BWD_Pin);
  L298N_MotorInit(&RightMotor, RightMotor_FWD_GPIO_Port, RightMotor_FWD_Pin, RightMotor_BWD_GPIO_Port, RightMotor_BWD_Pin);
  PID_Init(&Distance_PID, 100.0, 1.0, 10.0);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); //Left motor speed
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); //Right motor speed

  Status_RX = HAL_UART_Receive_IT(&huart1, &RX_Temp, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  Button_Task(&BlueKey); //Check button state

	  if(RX_Lines > 0)
	  {
		  Parser_TakeLine(&RX_Buffer, Recevied_Data);
		  RX_Lines--;
		  Parser_Parse(Recevied_Data);
	  }
	  if(RobotEnable)
	  {
		  Robot_Operation();
	  }
	  else
	  {
		  Move_Stop(&LeftMotor, &RightMotor);
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM1_CC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim == HCSR04p_front.htim_echo)
	{
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{
			HCSR04p_InteruptHandler(&HCSR04p_front);
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		if(Status_RX != HAL_OK)
		{
			UartLog("Error while receiving data\n\r");
		}
		else
		{
			if(RB_OK == RB_Write(&RX_Buffer, RX_Temp))
			{
				HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

				if(RX_Temp == END_LINE)
				{
					RX_Lines++;
				}

			}

		}

		Status_RX = HAL_UART_Receive_IT(&huart1, &RX_Temp, 1);
	}
}

void Robot_Operation()
{
	HCSR04p_ReadFloat(&HCSR04p_front, &Distance_f);
	L298N_MotorTask(&LeftMotor, &RightMotor, &Distance_PID);
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
