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
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_STATE 	12
#define MIN_STATE	0
#define TIMEBUZZER 1000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char 					uart_buf[50];
int 					uart_buf_len;
int						state;
int						ledState;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void MainLoop(void);
void SysTick_Handler(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
enum ledStates {RED,GREEN};
volatile bool timExpired = true;
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

  SysTick_Config(SystemCoreClock / 1000);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

  CAN_TX_filter_init();
  state = 8;
  CAN_SendState(state);

  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  {
	  Error_Handler();
  }
  if (HAL_TIM_Base_Start_IT(&htim7) != HAL_OK)
    {
  	  Error_Handler();
    }

  //Testfunction leds
    HAL_GPIO_WritePin(GPIOB, GreenLed_Pin, RESET);
    HAL_GPIO_WritePin(GPIOB, RedLed_Pin, SET);
    for(int i = 0; i<6;i++)
    {
		HAL_GPIO_TogglePin(GPIOB, GreenLed_Pin);
		HAL_GPIO_TogglePin(GPIOB, RedLed_Pin);
		HAL_Delay(500);
    }
    ledState = RED;
    HAL_GPIO_WritePin(GPIOB, GreenLed_Pin, RESET);
    HAL_GPIO_WritePin(GPIOB, RedLed_Pin, SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  MainLoop();
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */

void MainLoop(void)
{
	static uint32_t stoptime;
	if(ledState != RED && timExpired==1)
	{
		ledState = RED;
		HAL_GPIO_WritePin(GPIOB, RedLed_Pin, SET);
		HAL_GPIO_WritePin(GPIOB, GreenLed_Pin, RESET);
		HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, RESET);
	}
	else if(ledState != GREEN && timExpired==0)
	{
		ledState = GREEN;
		HAL_GPIO_WritePin(GPIOB, RedLed_Pin, RESET);
		HAL_GPIO_WritePin(GPIOB, GreenLed_Pin, SET);
		stoptime = HAL_GetTick();
		HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, SET);
	}

	if(HAL_GetTick() > (stoptime+TIMEBUZZER))
	{
		HAL_GPIO_WritePin(GPIOA, Buzzer_Pin, RESET);
	}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim2)
	{
		CAN_SendState(state);
	}
	if(htim == &htim7)
	{
		timExpired = 1;
	}
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

    if(GPIO_Pin == ButtonDown_Pin)
    {
        if(state > MIN_STATE)
        {
            state = state - 1;
        }
    }
    else if(GPIO_Pin == ButtonUp_Pin) {
        if(state < MAX_STATE)
        {
            state = state + 1;
        }
    } else {
        __NOP();
    }

    uart_buf_len = sprintf(uart_buf, "state : %lu\n", state);
    UART_Send(uart_buf, uart_buf_len);
    CAN_SendState(state);
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
//	uint32_t error = HAL_CAN_GetError(&hcan1);
//
//	    // Handle specific errors here
//	    if (error & HAL_CAN_ERROR_EWG) {
//	        // Handle Error Warning
//	    	hcan1.Instance->ESR = 0;  // Reset the Error Status Register (ESR)
//	        __HAL_RCC_CAN1_FORCE_RESET();
//	        HAL_Delay(1);
//	        __HAL_RCC_CAN1_RELEASE_RESET();
//	    }
//	    if (error & HAL_CAN_ERROR_EPV) {
//	        // Handle Error Passive
//	    	hcan1.Instance->ESR = 0;  // Reset the Error Status Register (ESR)
//	    	__HAL_RCC_CAN1_FORCE_RESET();
//	    	HAL_Delay(1);
//	    	__HAL_RCC_CAN1_RELEASE_RESET();
//	    }
//	    if (error & HAL_CAN_ERROR_BOF) {
//	        // Handle Bus-Off
//	    	hcan1.Instance->ESR = 0;  // Reset the Error Status Register (ESR)
//	    	__HAL_RCC_CAN1_FORCE_RESET();
//	    	HAL_Delay(1);
//	    	__HAL_RCC_CAN1_RELEASE_RESET();
//	    }
//	    if (error & HAL_CAN_ERROR_STF) {
//	        // Handle Stuff Error
//	    	hcan1.Instance->ESR = 0;  // Reset the Error Status Register (ESR)
//	    	__HAL_RCC_CAN1_FORCE_RESET();
//	    	HAL_Delay(1);
//	    	__HAL_RCC_CAN1_RELEASE_RESET();
//	    }
//	    if (error & HAL_CAN_ERROR_FOR) {
//	        // Handle Form Error
//	    	hcan1.Instance->ESR = 0;  // Reset the Error Status Register (ESR)
//	    	__HAL_RCC_CAN1_FORCE_RESET();
//	    	HAL_Delay(1);
//	    	__HAL_RCC_CAN1_RELEASE_RESET();
//	    }
//	    if (error & HAL_CAN_ERROR_ACK) {
//	        // Handle Acknowledge Error
//	    	hcan1.Instance->ESR = 0;  // Reset the Error Status Register (ESR)
//	    	__HAL_RCC_CAN1_FORCE_RESET();
//	    	HAL_Delay(1);
//	    	__HAL_RCC_CAN1_RELEASE_RESET();
//	    }
//	    if (error & HAL_CAN_ERROR_BR) {
//	        // Handle Bit Rate Error
//	    	hcan1.Instance->ESR = 0;  // Reset the Error Status Register (ESR)
//	    	__HAL_RCC_CAN1_FORCE_RESET();
//	    	HAL_Delay(1);
//	    	__HAL_RCC_CAN1_RELEASE_RESET();
//	    }
//	    if (error & HAL_CAN_ERROR_BD) {
//	        // Handle Bit Dominant Error
//	    	hcan1.Instance->ESR = 0;  // Reset the Error Status Register (ESR)
//	    	__HAL_RCC_CAN1_FORCE_RESET();
//	    	HAL_Delay(1);
//	    	__HAL_RCC_CAN1_RELEASE_RESET();
//	    }
//	    if (error & HAL_CAN_ERROR_CRC) {
//	        // Handle CRC Error
//	    	hcan1.Instance->ESR = 0;  // Reset the Error Status Register (ESR)
//	    	__HAL_RCC_CAN1_FORCE_RESET();
//	    	HAL_Delay(1);
//	    	__HAL_RCC_CAN1_RELEASE_RESET();
//	    }
//	    else
//	    {
//		    __disable_irq();
//		    //Perform a system reset
//		    NVIC_SystemReset();
//		    while (1)
//		    {
//		    }
//	    }
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
