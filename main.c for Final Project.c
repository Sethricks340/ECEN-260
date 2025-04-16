/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
	#define I2C_ADDR 0x27 // I2C address of the PCF8574
	#define RS_BIT 0 // Register select bit
	#define EN_BIT 2 // Enable bit
	#define BL_BIT 3 // Backlight bit
	#define D4_BIT 4 // Data 4 bit
	#define D5_BIT 5 // Data 5 bit
	#define D6_BIT 6 // Data 6 bit
	#define D7_BIT 7 // Data 7 bit
	#define LCD_ROWS 2 // Number of rows on the LCD
	#define LCD_COLS 16 // Number of columns on the LCD
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
//Initialize ADCs
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

//Initialize I2C and timers
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */
//The backlight state is on
	uint8_t backlight_state = 1;

	 //initialize all the directions
	 int DOWN = 0;
	 int RIGHT = 0;
	 int LEFT = 0;
	 int UP = 0;
	 //initialize if lcd is ready
	 uint8_t update_lcd = 0;
	 char lcd_buffer[2][16]; // Buffer to hold the LCD messages for each row
	 //servo motor status
	 int UP_servo = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM16_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//Function to write a nibble to the LCD
	void lcd_write_nibble(uint8_t nibble, uint8_t rs) {
	 uint8_t data = nibble << D4_BIT;
	 data |= rs << RS_BIT;
	 data |= backlight_state << BL_BIT; // Include backlight state in data
	 data |= 1 << EN_BIT;
	 HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDR << 1, &data, 1, 100);
	 HAL_Delay(1);
	 data &= ~(1 << EN_BIT);
	 HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDR << 1, &data, 1, 100);
	}
//Function to send a command to the LCD
	void lcd_send_cmd(uint8_t cmd) {
	 uint8_t upper_nibble = cmd >> 4;
	 uint8_t lower_nibble = cmd & 0x0F;
	 lcd_write_nibble(upper_nibble, 0);
	 lcd_write_nibble(lower_nibble, 0);
	 if (cmd == 0x01 || cmd == 0x02) {
	 HAL_Delay(2);
	 }
	}
//Function to send data to the LCD
	void lcd_send_data(uint8_t data) {
	 uint8_t upper_nibble = data >> 4;
	 uint8_t lower_nibble = data & 0x0F;
	 lcd_write_nibble(upper_nibble, 1);
	 lcd_write_nibble(lower_nibble, 1);
	}
	void lcd_init() {
	 HAL_Delay(50);
	 lcd_write_nibble(0x03, 0);
	 HAL_Delay(5);
	 lcd_write_nibble(0x03, 0);
	 HAL_Delay(1);
	 lcd_write_nibble(0x03, 0);
	 HAL_Delay(1);
	 lcd_write_nibble(0x02, 0);
	 lcd_send_cmd(0x28);
	 lcd_send_cmd(0x0C);
	 lcd_send_cmd(0x06);
	 lcd_send_cmd(0x01);
	 HAL_Delay(2);
	}
//Function to write a string to the LCD
	void lcd_write_string(char *str) {
	 while (*str) {
	 lcd_send_data(*str++);
	 }
	}
	//Function to set the cursor
	void lcd_set_cursor(uint8_t row, uint8_t column) {
	 uint8_t address;
	 switch (row) {
	 case 0:
	 address = 0x00;
	 break;
	 case 1:
	 address = 0x40;
	 break;
	 default:
	 address = 0x00;
	 }
	 address += column;
	 lcd_send_cmd(0x80 | address);
	}
	//Function to clear the LCD
	void lcd_clear(void) {
	lcd_send_cmd(0x01);
	 HAL_Delay(2);
	}
	//Function to turn the backlight on and off
	void lcd_backlight(uint8_t state) {
	 if (state) {
	 backlight_state = 1;
	 } else {
	 backlight_state = 0;
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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_TIM16_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	  HAL_TIM_Base_Start_IT(&htim16); // Start Timer16
	  // I2C pull-up resistors
	  GPIOB->PUPDR |= 0b01 << (8*2);
	  GPIOB->PUPDR |= 0b01 << (9*2);
	  // Initialize the LCD
	  lcd_init();
	  lcd_backlight(1); // Turn on backlight

	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);  // Start PWM on TIM2, Channel 2
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	  while (1)
	  {
	    if (update_lcd)
	    {
	      update_lcd = 0; // Clear the flag
	      lcd_set_cursor(0, 0); //cursor on first row, first column
	      lcd_write_string(lcd_buffer[0]); //write the first message
	      lcd_set_cursor(1, 0); //cursor on second row, first column
	      lcd_write_string(lcd_buffer[1]); //write the second message
	    }

	    //move the first motor, so magnet moves up
	    if (UP){
			HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 1);
			HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, 1);
			HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
	    }

	    //move the first motor, so magnet moves down
	    else if (DOWN){
			HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);
			HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, 1);
			HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 1);
	    }

	    //turn off the first motor
	    else{
			HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, 0);
			HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, 0);
			HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, 0);
	    }

	    //move the second motor, so magnet moves left
	    if (LEFT){
			HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 1);
			HAL_GPIO_WritePin(ENABLE2_GPIO_Port, ENABLE2_Pin, 1);
			HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 0);
	    }

	    //move the second motor, so magnet moves right
	    else if (RIGHT){
			HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 0);
			HAL_GPIO_WritePin(ENABLE2_GPIO_Port, ENABLE2_Pin, 1);
			HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 1);
	    }

	    //turn off the second motor
	    else{
			HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, 0);
			HAL_GPIO_WritePin(ENABLE2_GPIO_Port, ENABLE2_Pin, 0);
			HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, 0);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

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
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1400;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 7999;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 999;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ENABLE_Pin|IN1_Pin|IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ENABLE2_GPIO_Port, ENABLE2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, IN4_Pin|IN3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ENABLE_Pin IN1_Pin IN2_Pin */
  GPIO_InitStruct.Pin = ENABLE_Pin|IN1_Pin|IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ENABLE2_Pin */
  GPIO_InitStruct.Pin = ENABLE2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ENABLE2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IN4_Pin IN3_Pin */
  GPIO_InitStruct.Pin = IN4_Pin|IN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SERVO_BUTTON_Pin */
  GPIO_InitStruct.Pin = SERVO_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SERVO_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
	// Callback: timer has rolled over
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  //if the clock that was triggerd is clock 16
  if (htim == &htim16)
  {
    int ADC_RANGE = 4096; // 2^12 (12-bit resolution)
    // Start ADC Conversions
    HAL_ADC_Start(&hadc2);
    HAL_ADC_Start(&hadc3);
    // Wait for ADC conversions to complete
    HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
    HAL_ADC_PollForConversion(&hadc3, HAL_MAX_DELAY);
    // Read ADC values
    uint16_t xjoy_measurement = HAL_ADC_GetValue(&hadc2);
    uint16_t yjoy_measurement = HAL_ADC_GetValue(&hadc3);
    // Convert ADC levels to a fraction of total
    float xjoy_value = ((float)xjoy_measurement) / ADC_RANGE;
    float yjoy_value = ((float)yjoy_measurement) / ADC_RANGE;

    //joystick is turned to the left
    if (yjoy_value < 0.3)
    {
      RIGHT = 1;
      LEFT = 0;
      //message to be sent to the display
      snprintf(lcd_buffer[1], 16, "LEFT        ");
    }
    //joystick is turned to the right
    else if (yjoy_value > 0.5)
    {
      RIGHT = 0;
      LEFT = 1;
      //message to be sent to the display
      snprintf(lcd_buffer[1], 16, "RIGHT         ");
    }
    //joystick is in the middle (not left or right)
    else
    {
	  RIGHT = 0;
	  LEFT = 0;
      //message to be sent to the display
      snprintf(lcd_buffer[1], 16, "NA           ");
    }

    //joystick is turned up
    if (xjoy_value < 0.3)
    {
      UP = 1;
      DOWN = 0;
      //message to be sent to the display
      snprintf(lcd_buffer[0], 16, "DOWN           ");
    }
    //joystick is turned down
    else if (xjoy_value > 0.5)
    {
      UP = 0;
      DOWN = 1;
      //message to be sent to the display
      snprintf(lcd_buffer[0], 16, "UP         ");
    }

    //joystick is in the middle (not up or down)
    else
    {
	  UP = 0;
	  DOWN = 0;
      //message to be sent to the display
      snprintf(lcd_buffer[0], 16, "NA           ");
    }

    //message ready to be sent to the LCD
    update_lcd = 1; // Set flag to update the LCD
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	//if the servo button is pressed
	if (GPIO_Pin == SERVO_BUTTON_Pin){

		//if the motor is not already up
		if(!UP_servo){
			//move servo motor to the up position
			TIM2->CCR2 = 99;
			//The servo arm is now up
			UP_servo = 1;
		}

		//if the motor is already up, move it down
		else{
			//move servo motor to the down position
			TIM2->CCR2 = 50;
			//The servo arm is now down
			UP_servo = 0;
		}
	}
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
