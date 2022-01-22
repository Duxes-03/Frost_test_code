/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include"stdint.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include"fonts.h"
#include"ssd1306.h"
#include"test.h"

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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef enum STATE

{
	IDLE=1,
	ACTIVE,
	INACTIVE,
	WARNING,
}STATE;

STATE state = IDLE ;

uint32_t Tim_Ticks, Temp_variable=1,Current_ldr_adc1,Historical_ldr_adc1,
Current_Weight_adc2,Historical_Weight_adc2,elapsed_ticks = 7200;

// function prototypes

void Check_ldr_adc1();
void Check_weighingsensor_adc2();
void Func_idle ();
void Func_active();
void Func_inactive();
void Func_warning();
void Battery();
void Func_buzzer();


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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_I2C1_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  SSD1306_Init();
  SSD1306_Clear();
  SSD1306_GotoXY(0, 0);
  SSD1306_Puts("FROST SMART", &Font_11x18, 1);
  SSD1306_GotoXY(0, 30);
   SSD1306_Puts("WATER BOTTLE", &Font_11x18, 1);
  SSD1306_UpdateScreen();
  /* USER CODE END 2 */
  HAL_TIM_Base_Start_IT(&htim7);      // start the timer
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */
      switch(state)
      {
      case 1:
        Func_idle();                   // idle state
          break;
      case 2:
    	  Func_active();              // active state
    	  break;
      case 3:
    	  Func_inactive();           // inactive state
    	  break;
      case 4:
    	  Func_warning();           // warning state
    	  break;
      }
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}


//*********** function definition **************
void Check_ldr_adc1()
{
	HAL_ADC_Start(&hadc1);                           // start adc1(LDR)

	Current_ldr_adc1 = HAL_ADC_GetValue(&hadc1);    // current value as historical value and new value as current value
	Historical_ldr_adc1 = Current_ldr_adc1;
	Current_ldr_adc1 = HAL_ADC_GetValue(&hadc1);

	if(Tim_Ticks == Temp_variable )
	{
		Temp_variable++;
		Current_ldr_adc1 = HAL_ADC_GetValue(&hadc1);
	}

}
void Check_weighingsensor_adc2()
{
	HAL_ADC_Start(&hadc2);

	Current_Weight_adc2 = HAL_ADC_GetValue(&hadc2);      // ADC_2 (10 BIT), Value 676=3.3V
	Historical_Weight_adc2 = Current_Weight_adc2;
	Current_Weight_adc2 = HAL_ADC_GetValue(&hadc2);

	if(Current_Weight_adc2 <= 0)                        // if no weight is present on DOC display warning
	{
		SSD1306_Puts("Please place water bottle on DOC", &Font_11x18, 1);
		HAL_GPIO_WritePin(GPIOD, Red_Pin, GPIO_PIN_SET);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(GPIOD, Red_Pin, GPIO_PIN_RESET);
		Func_buzzer();

	}

}
void Func_buzzer()
{
		HAL_GPIO_WritePin(GPIOD, Buzz_output_Pin, GPIO_PIN_SET);     // setting buzzer
		HAL_Delay(1000);
		HAL_GPIO_WritePin(GPIOD, Buzz_output_Pin, GPIO_PIN_RESET);    // resetting buzzer

}
void Battery()
{
	if(HAL_GPIO_ReadPin(GPIOC, Battery_input_Pin) == GPIO_PIN_SET)    // checking battery condition
    {
		SSD1306_Puts("Battery full", &Font_11x18, 1);
		HAL_GPIO_WritePin(GPIOD, Green_Pin, GPIO_PIN_SET);           // LED ON
		HAL_Delay(1000);
		HAL_GPIO_WritePin(GPIOD, Green_Pin, GPIO_PIN_RESET);         // LED off
		Func_buzzer();
    }
	else if(HAL_GPIO_ReadPin(GPIOC, Battery_input_Pin) == GPIO_PIN_RESET)
	{
		SSD1306_Puts("Battery LOW", &Font_11x18, 1);
		HAL_GPIO_WritePin(GPIOD, Red_Pin, GPIO_PIN_SET);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(GPIOD, Red_Pin, GPIO_PIN_RESET);
		Func_buzzer();
	}
}

void Func_idle()
{
	Check_ldr_adc1();                     // checking sensor
	Check_weighingsensor_adc2();          // checking weighing sensor (potentiometer)

	// if LDR <=36 (LDR is high), LDR >36 (LDR is LOW)
   //  if bottle is present on doc then that condition is taken as " low "
   //  ie. current is < 676(3.3V)  when transition happens from low to high ie. active state
   // transition from high to low is taken as idle
    if(Current_ldr_adc1 <= 36 && ( Historical_ldr_adc1 < 36 || Historical_ldr_adc1 > 36) &&
    		Historical_Weight_adc2 > 676 && Current_Weight_adc2 < 676 )
    {
         state = ACTIVE;
         SSD1306_Puts("Active state", &Font_11x18, 1);
    }
    // current Low and historical high (transition from high to low) bottle is back to DOC
    if( Current_Weight_adc2  < 676 && Historical_Weight_adc2 > 676 )
    {
		state = IDLE;
	    SSD1306_Puts("IDLE state", &Font_11x18, 1);
    }

    // if LDR is high, check the battery condition (calling battery function)
    if(Current_ldr_adc1 <= 36 && ( Historical_ldr_adc1 < 36 || Historical_ldr_adc1 > 36))
    {
    	       Battery();

	}
}

void Func_active()
{
	// if LDR is high only
    if(Current_ldr_adc1 <= 36 && ( Historical_ldr_adc1 < 36 || Historical_ldr_adc1 > 36))
    {
    	Check_weighingsensor_adc2();   // checks the weight and moves to warning
    	state = WARNING;

	}
}
void Func_inactive()
{
     // if not bottle is present on DOC for 2 hours then display warning
	if(Current_Weight_adc2 <= 0 && Tim_Ticks == elapsed_ticks)
	{
		elapsed_ticks = elapsed_ticks+7200;
		SSD1306_Puts("No bottle found on DOC", &Font_11x18, 1);
		HAL_GPIO_WritePin(GPIOD, Red_Pin, GPIO_PIN_SET);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(GPIOD, Red_Pin, GPIO_PIN_RESET);
		Func_buzzer();

	}
}
void Func_warning()
{
	// if water level is same for 1 day then display the warning
	Check_weighingsensor_adc2();
    if(Current_Weight_adc2 == (Historical_Weight_adc2 && (Tim_Ticks > 86400 )))
    {
		SSD1306_Puts("Please clean the water bottle", &Font_11x18, 1);
		HAL_GPIO_WritePin(GPIOD, Yellow_Pin, GPIO_PIN_SET);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(GPIOD, Yellow_Pin, GPIO_PIN_RESET);
		Func_buzzer();
    }
	// if No water present in bottle then display the warning
    if(Current_Weight_adc2 > 0 && Current_Weight_adc2 <= 20 )
    {
		SSD1306_Puts("Please fill the water", &Font_11x18, 1);
		HAL_GPIO_WritePin(GPIOD, Blue_Pin, GPIO_PIN_SET);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(GPIOD, Blue_Pin, GPIO_PIN_RESET);
		Func_buzzer();
    }
    // Display warning every 2 hours if user have not drunk water
    if(Current_Weight_adc2 > 676 && Tim_Ticks == elapsed_ticks )
    {
    	elapsed_ticks = elapsed_ticks+7200;

		SSD1306_Puts("Please Drink water", &Font_11x18, 1);
		HAL_GPIO_WritePin(GPIOD, Yellow_Pin, GPIO_PIN_SET);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(GPIOD, Yellow_Pin, GPIO_PIN_RESET);
		Func_buzzer();
    }


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

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_10B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 16000;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1000;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Battery_output_GPIO_Port, Battery_output_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, Green_Pin|Red_Pin|Yellow_Pin|Blue_Pin
                          |Buzz_output_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Battery_input_Pin */
  GPIO_InitStruct.Pin = Battery_input_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Battery_input_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Battery_output_Pin */
  GPIO_InitStruct.Pin = Battery_output_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Battery_output_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Green_Pin Red_Pin Yellow_Pin Blue_Pin
                           Buzz_output_Pin */
  GPIO_InitStruct.Pin = Green_Pin|Red_Pin|Yellow_Pin|Blue_Pin
                          |Buzz_output_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
    HAL_IncTick();
  }
  else if (htim->Instance == TIM7) {

	  Tim_Ticks++;
	  Tim_Ticks = Tim_Ticks % 86400;

     HAL_IncTick();
   }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
