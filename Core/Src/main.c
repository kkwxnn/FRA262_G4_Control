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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
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
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

//microsecond timer
uint64_t _micros = 0;
int64_t currentTime;

//Motor drive
typedef struct _QEIStructure
{
	uint32_t data[2]; //position data contenter
	uint64_t timestamp[2];

	float QEIPosition; //step
	float QEIVelocity; //step/sec
}QEIStructireTypedef;
QEIStructireTypedef QEIData = {0};

float duty = 0;
int direction = 0;
int R_EN = 1;
int L_EN = 1;

//PID
int16_t position ;
float setposition = 0;
float errorposition = 0;
float u_position = 0;
float pre_errorposition = 0;

float integral = 0;
float derivative = 0;

float velocity = 0;
float setvelocity = 0;
float sumsetvelocity = 0;
float errorvelocity = 0;

float Kp = 0;
float Ki = 0;
float Kd = 0;

//Joystick
typedef struct LocationStructure
{
	float L1[2];
	float L2[2];
	float L3[2];
}Location;
Location PickTray;
Location PlaceTray;

typedef struct ButtonStructure
{
	int last;
	int current;
	int flag;
}Button;
Button GetPositionButton;
Button ResetButton;
Button FineButton;
Button RoughButton;

int XYSwitch[2];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
inline uint64_t micros();
void QEIEncoderPositionVelocity_Update();
void JoystickControl();
void JoystickPinUpdate();
void JoystickLocationState();
float PIDcal(float setposition, int16_t position);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM5_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1|TIM_CHANNEL_2);

  HAL_TIM_Base_Start(&htim1); //Start Timer1
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, L_EN);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, R_EN);

  HAL_TIM_Base_Start(&htim5); //Start Timer5

  HAL_ADC_Start_DMA(&hadc1, XYSwitch, 2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //Joystick
	  JoystickControl();
	  JoystickPinUpdate();
	  JoystickLocationState();

	  //QEI
	  position = __HAL_TIM_GET_COUNTER(&htim3);

	  static uint64_t timestamp1 = 0;
	  currentTime = micros();
	  if(currentTime > timestamp1)
	  {
		  timestamp1 = currentTime + 1000;
		  QEIEncoderPositionVelocity_Update();
		  velocity = QEIData.QEIVelocity;
	  }

	  //PWM & Motor drive
	  static uint64_t timestamp2 = 0;
	  	  if (HAL_GetTick()>= timestamp2)
	  	  {
	  		  timestamp2 = HAL_GetTick()+10;
	  		  duty = PIDcal(setposition, position);
	  		  if (duty >= 0)
	  		  {
	  			  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
	  			  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,duty);
	  		  }
	  		  else if (duty < 0)
	  		  {
	  			  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
	  			  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,-1*duty);
	  		  }
	  	  }
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = QEI_PERIOD-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 83;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeroidElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim5)
	{
		_micros += UINT32_MAX;
	}
}


uint64_t micros()
{
	return __HAL_TIM_GET_COUNTER(&htim5)+_micros;
}


void QEIEncoderPositionVelocity_Update()
{
	QEIData.timestamp[0] = micros();
	uint32_t counterPosition = __HAL_TIM_GET_COUNTER(&htim3);
	QEIData.data[0] = counterPosition;

	QEIData.QEIPosition = counterPosition % 1600;

	int32_t diffPosition = QEIData.data[0]-QEIData.data[1];
	float difftime = (QEIData.timestamp[0]-QEIData.timestamp[1]);

	if(diffPosition > QEI_PERIOD>>1) diffPosition -= QEI_PERIOD;
	if(diffPosition < -(QEI_PERIOD>>1)) diffPosition += QEI_PERIOD;

	QEIData.QEIVelocity = (diffPosition * 1000000)/difftime;

	QEIData.data[1] = QEIData.data[0];
	QEIData.timestamp[1] = QEIData.timestamp[0];
}

float PIDcal(float setposition, int16_t position)
{
	errorposition = setposition - position;

	integral = integral + errorposition;
	derivative = errorposition - pre_errorposition;
	u_position = Kp*errorposition + Ki*integral + Kd*derivative;

	pre_errorposition = errorposition;

	return u_position;
}

void JoystickPinUpdate()
{
	  GetPositionButton.current = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);
	  if (GetPositionButton.last == 1 && GetPositionButton.current == 0)
	  {
		  GetPositionButton.flag = 1;
	  }
	  else
	  {
		  GetPositionButton.flag = 0;
	  }
	  GetPositionButton.last = GetPositionButton.current;

	  ResetButton.current = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
	  if (ResetButton.last == 1 && ResetButton.current == 0)
	  {
		ResetButton.flag = 1;
	  }
	  else
	  {
		  ResetButton.flag = 0;
	  }
	  ResetButton.last = ResetButton.current;

	  FineButton.current = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);
	  if (FineButton.last == 1 && FineButton.current == 0)
	  {
		  FineButton.flag = 1;
	  }
	  else
	  {
		  FineButton.flag = 0;
	  }
	  FineButton.last = FineButton.current;

	  RoughButton.current = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
	  if (RoughButton.last == 1 && RoughButton.current == 0)
	  {
		  RoughButton.flag = 1;
	  }
	  else
	  {
		  RoughButton.flag = 0;
	  }
	  RoughButton.last = RoughButton.current;
}

void JoystickControl()
{
	static float setposition_f = 0;
	static int JoySpeed = 0;

	if(RoughButton.flag == 1)
	{
		JoySpeed = 0;
		RoughButton.flag = 0;
	}
	else if(FineButton.flag == 1)
	{
		JoySpeed = 1;
		FineButton.flag = 0;
	}

	switch(JoySpeed)
	{
	case 0:
		if(XYSwitch[1] > 2150)
		{
			setposition_f += XYSwitch[1]*0.00002;
		}
		else if(XYSwitch[1] < 2000)
		{
			setposition_f -= (4096-XYSwitch[1])*0.00002;
		}
		setposition = (int)round(setposition_f);
		break;

	case 1:
		if(XYSwitch[1] > 2150)
		{
			setposition_f += XYSwitch[1]*0.0000005;
		}
		else if(XYSwitch[1] < 2000)
		{
			setposition_f -= (4096-XYSwitch[1])*0.0000005;
		}
		setposition = (int)round(setposition_f);
		break;
	}
}

void JoystickLocationState()
{
	static int state = 0;
	switch(state)
	{
	case 0:
		PickTray.L1[1] = 0;
		PickTray.L2[1] = 0;
		PickTray.L3[1] = 0;
		PlaceTray.L1[1] = 0;
		PlaceTray.L2[1] = 0;
		PlaceTray.L3[1] = 0;
		if (GetPositionButton.flag == 1)
		{
			PickTray.L1[1] = position;
			GetPositionButton.flag = 0;
			state = 1;
		}
		break;
	case 1:
		if (GetPositionButton.flag == 1)
		{
			PickTray.L2[1] = position;
			GetPositionButton.flag = 0;
			state = 2;
		}
		else if (ResetButton.flag == 1)
		{
			ResetButton.flag = 0;
			state = 0;
		}
		break;
	case 2:
		if (GetPositionButton.flag == 1)
		{
			PickTray.L3[1] = position;
			GetPositionButton.flag = 0;
			state = 3;
		}
		else if (ResetButton.flag == 1)
		{
			ResetButton.flag = 0;
			state = 0;
		}
		break;
	case 3:
		if (GetPositionButton.flag == 1)
		{
			PlaceTray.L1[1] = position;
			GetPositionButton.flag = 0;
			state = 4;
		}
		else if (ResetButton.flag == 1)
		{
			ResetButton.flag = 0;
			state = 0;
		}
		break;
	case 4:
		if (GetPositionButton.flag == 1)
		{
			PlaceTray.L2[1] = position;
			GetPositionButton.flag = 0;
			state = 5;
		}
		else if (ResetButton.flag == 1)
		{
			ResetButton.flag = 0;
			state = 0;
		}
		break;
	case 5:
		if (GetPositionButton.flag == 1)
		{
			PlaceTray.L3[1] = position;
			GetPositionButton.flag = 0;
			state = 6;
		}
		else if (ResetButton.flag == 1)
		{
			ResetButton.flag = 0;
			state = 0;
		}
		break;
	case 6:
		if (ResetButton.flag == 1)
		{
			ResetButton.flag = 0;
			state = 0;
		}
		break;
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
