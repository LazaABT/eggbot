/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LIMIT_PIN (1)
#define RX_BUFF_SIZE (64)
#define TX_BUFF_SIZE (16)
#define CMD_BUFFER_SIZE (16)
#define STEPPER_X_MAX (200*16)
#define STEPPER_Y_MAX (850)
#define SERVO_UP_POSITION (4600)
#define SERVO_DOWN_POSITION (5300)
#define STEPS_FROM_ZERO (20)
#define STEPPER_PULSE_WIDTH (0.000002)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define min(a,b) a>b?b:a
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void set_speed(uint16_t x_speed, uint16_t y_speed);
void move_steppers(int16_t xrel, int16_t yrel);
void zero_steppers();
void send_message(uint8_t* msg, int len);
int process_command();
void command_buffer_overflow();
void report_position();
void report_max_position();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


uint8_t command_buffer[CMD_BUFFER_SIZE], command_buffer_pos = 0;
uint8_t rx_buffer[RX_BUFF_SIZE];
uint8_t tx_buffer[TX_BUFF_SIZE];

int16_t stepper_x_count = 0, stepper_x_goal = 0, stepper_x_direction = 1, stepper_x_speed = 100;
int16_t stepper_y_count = 0, stepper_y_goal = 0, stepper_y_direction = 1, stepper_y_speed = 100;
uint8_t stepper_x_moving = 0, stepper_y_moving = 0;
uint8_t zeroed = 0, limit_flag = 0, ignore_limit = 0, zeroing = 0;

volatile int tmp = 0;

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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

//  volatile HAL_StatusTypeDef tmp = HAL_TIM_Base_Start(&htim2);


  htim4.Instance->CCR1 = (int)(SERVO_UP_POSITION);
  HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_1);
  HAL_UART_Receive_DMA(&huart1, rx_buffer, RX_BUFF_SIZE);
  limit_flag = 0;

  int current_active = RX_BUFF_SIZE-1, new_active = RX_BUFF_SIZE-1;
  while (1)
  {
	  new_active = (2*RX_BUFF_SIZE - hdma_usart1_rx.Instance->CNDTR-1)%RX_BUFF_SIZE;
	  while (current_active != new_active)
	  {
		  current_active = (current_active + 1)%RX_BUFF_SIZE;
		  command_buffer[command_buffer_pos]=rx_buffer[current_active];
		  command_buffer_pos += 1;
		  if (process_command())
		  {
			  command_buffer_pos = 0;
		  }

	  }


//	  HAL_Delay(1);
////	  if (HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin))
////	  {
//		  count = 0;
//		  count3 = 0;
//		  htim4.Instance->CCR1 = (int)(pos * 3000.0 + 3000.0)-1;
//		  pos = pos + 0.02;
//		  if (pos > 1.5)
//			  pos = -0.6;
////		  HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
////		  HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
//		  HAL_Delay(100);
////		  while (HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin))
////		  {}
////		  HAL_Delay(10);

//	  }
//	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
//	  HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 49999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfigOC.Pulse = 47;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 29999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 47;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 59999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2999;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(YDIR_GPIO_Port, YDIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LD1_Pin|LD2_Pin|XDIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BTN_Pin */
  GPIO_InitStruct.Pin = BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : YDIR_Pin */
  GPIO_InitStruct.Pin = YDIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(YDIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD2_Pin XDIR_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD2_Pin|XDIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */


void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  UNUSED(htim);
  if (htim == &htim2)
	{
	  stepper_x_count += stepper_x_direction;
	  stepper_x_count %= STEPPER_X_MAX;
	  if (limit_flag && (!ignore_limit) && zeroing)
	  {
		  HAL_TIM_PWM_Stop_IT(htim, TIM_CHANNEL_1);
		  stepper_x_moving = 0;
		  return;
	  }
	  if (stepper_x_count == stepper_x_goal)
	  {
		  HAL_TIM_PWM_Stop_IT(htim, TIM_CHANNEL_1);
		  stepper_x_moving = 0;
		  return;
	  }
	}
  else if (htim == &htim3)
	{
	  stepper_y_count += stepper_y_direction;
	  if (limit_flag && (!ignore_limit) && zeroing)
	  {
		  HAL_TIM_PWM_Stop_IT(htim, TIM_CHANNEL_1);
		  stepper_y_moving = 0;
		  return;
	  }
	  if (stepper_y_count == stepper_y_goal)
	  {
		  HAL_TIM_PWM_Stop_IT(htim, TIM_CHANNEL_1);
		  stepper_y_moving = 0;
		  return;
	  }
	}

}

void set_speed(uint16_t x_speed, uint16_t y_speed)
{
	if (x_speed<1 || y_speed<1 || x_speed > 30000 || y_speed > 30000)
	{
		send_message("as\xff\xff",5);
		return;
	}
	stepper_x_speed = x_speed;
	stepper_y_speed = y_speed;
	tx_buffer[0] = 'a';
	tx_buffer[1] = 's';
	*(uint16_t*)(tx_buffer+2) = x_speed;
	*(uint16_t*)(tx_buffer+4) = y_speed;
	send_message(tx_buffer,6);
}

void report_position()
{
	tx_buffer[0] = 'a';
	tx_buffer[1] = 'r';
	tx_buffer[2] = 'a';
	*(uint16_t*)(tx_buffer+3) = stepper_x_count;
	*(uint16_t*)(tx_buffer+5) = stepper_y_count;
	send_message(tx_buffer,7);
}

void report_max_position()
{
	tx_buffer[0] = 'a';
	tx_buffer[1] = 'r';
	tx_buffer[2] = 'm';
	*(uint16_t*)(tx_buffer+3) = STEPPER_X_MAX;
	*(uint16_t*)(tx_buffer+5) = STEPPER_Y_MAX;
	send_message(tx_buffer,7);
}

void command_buffer_overflow()
{
	send_message("O",1);
	//Overflow message
}


int process_command()
{
	switch (command_buffer[0])
	{
	case 's':
		if (command_buffer_pos < 5)
		{
//			send_message("ai",2);
			return 0;
		}
		uint16_t speed_x, speed_y;
		speed_x =*(uint16_t*)(command_buffer+1);
		speed_y =*(uint16_t*)(command_buffer+3);
		set_speed(speed_x,speed_y);

	//set movement speed (sXXYY\r) -> (asXXYY) report exact movement speed (asFF in case of error)
		break;
	case 'm':
		if (command_buffer_pos < 5)
		{
//			send_message("ai",2);
			return 0;
		}
		int16_t rel_x, rel_y;
		rel_x =*(int16_t*)(command_buffer+1);
		rel_y =*(int16_t*)(command_buffer+3);
		move_steppers(rel_x,rel_y);
	//move relative  (mXXYY\r) -> (am) ack after done (amFFFF in case of error)
		break;
	case 'r':
		if (command_buffer_pos < 2)
		{
//			send_message("ai",2);
			return 0;
		}
		if (command_buffer[1] == 'a')
		{
			report_position();
		}
		else if (command_buffer[1] == 'm')
		{
			report_max_position();
		}
		else
		{
			send_message("ai",2);
			return;
		}
	//report absolute position (ra\r) -> (araXXYY)  FFFF in case of error
	//report max position (rm\r) -> (armXXYY)
		break;
	case 'z':
		if (command_buffer_pos < 1)
		{
//			send_message("ai",2);
			return 0;
		}
		zero_steppers();
	//zero out (z\r) -> (az)
		break;
	case 'p':
		if (command_buffer_pos < 2)
		{
//			send_message("ai",2);
			return 0;
		}
		if (command_buffer[1] == 'u')
		{
			htim4.Instance->CCR1 = (int)(SERVO_UP_POSITION);
			send_message("apu",3);
		}
		else if (command_buffer[1] == 'd')
		{
			while (htim4.Instance->CCR1 < (SERVO_DOWN_POSITION))
			{
				htim4.Instance->CCR1 = min(htim4.Instance->CCR1 + 30, SERVO_DOWN_POSITION);
				HAL_Delay(20);
			}
			send_message("apd",3);
		}
		else
		{
			send_message("ai",2);
		}
	//lift pen (pu\r) -> (apu)
	//drop pen (pd\r) -> (apu)
		break;
	default:
		send_message("ai",2);
		break;
	}
	return 1;
}

void zero_steppers()
{
	//calculate based on speed for both x and y
	zeroing = 1;
	limit_flag = 0;
	stepper_y_goal = -1;
	stepper_y_direction = 0;
	HAL_GPIO_WritePin(YDIR_GPIO_Port, YDIR_Pin, GPIO_PIN_RESET);
	htim3.Instance->ARR = 39999;
	htim3.Instance->PSC = 1;
	htim3.Instance->CCR1 = 47;
	stepper_y_moving = 1;
	ignore_limit = 0;
	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
	while (stepper_y_moving) {}

	HAL_GPIO_WritePin(YDIR_GPIO_Port, YDIR_Pin, GPIO_PIN_SET);
	stepper_y_count = - STEPS_FROM_ZERO;
	stepper_y_goal = 0;
	stepper_y_direction = 1;
	stepper_y_moving = 1;
	ignore_limit = 1;
	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
	while (stepper_y_moving) {}
	limit_flag = 0;
	zeroed = 1;
	ignore_limit = 0;
	zeroing = 0;
	send_message("az",2);
}

void move_steppers(int16_t xrel, int16_t yrel)
{
	//calculate based on speed for both x and y
	int16_t xnew, ynew;
	double speed,time,prescale,count,xspeed,yspeed;
	xnew = stepper_x_count + xrel;
	ynew = stepper_y_count + yrel;
	if (ynew >= STEPPER_Y_MAX || ynew < 0 || !zeroed)
	{
		send_message("am\xFF\xFF\xFF\xFF",6);
		return;
	}
	if (xrel ==0 && yrel == 0 )
	{
		send_message("am\x00\x00\x00\x00",6);
		return;
	}
	stepper_x_goal = (xnew + STEPPER_X_MAX) % STEPPER_X_MAX;
	stepper_y_goal = ynew;
	speed =  1.0*xrel*stepper_x_speed +  1.0*yrel*stepper_y_speed;
	speed /=  1.0*(xrel + yrel);
	time = sqrt(1.0*xrel*xrel + 1.0*yrel*yrel)/speed;
	if (xrel != 0)
	{
		xspeed = abs(xrel)/time;
		xspeed = 24e6/xspeed;
		prescale = ceil(xspeed / 65500);
		count = round(xspeed / prescale);
		htim2.Instance->ARR = (uint16_t)count-1;
		htim2.Instance->PSC = (uint16_t)prescale-1;
	}

	if (yrel != 0)
	{
		yspeed = abs(yrel)/time;
		yspeed = 24e6/yspeed;
		prescale = ceil(yspeed / 65500);
		count = round(yspeed / prescale);
		htim3.Instance->ARR = (uint16_t)count-1;
		htim3.Instance->PSC = (uint16_t)prescale-1;
	}
	if (xrel > 0)
	{
		stepper_x_direction = 1;
		HAL_GPIO_WritePin(XDIR_GPIO_Port, XDIR_Pin, GPIO_PIN_SET);
	}
	else
	{
		stepper_x_direction = -1;
		HAL_GPIO_WritePin(XDIR_GPIO_Port, XDIR_Pin, GPIO_PIN_RESET);
	}

	if (yrel > 0)
	{
		stepper_y_direction = 1;
		HAL_GPIO_WritePin(YDIR_GPIO_Port, YDIR_Pin, GPIO_PIN_SET);
	}
	else
	{
		stepper_y_direction = -1;
		HAL_GPIO_WritePin(YDIR_GPIO_Port, YDIR_Pin, GPIO_PIN_RESET);
	}
	if (xrel != 0)
	{
		stepper_x_moving = 1;
		HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);
	}
	if (yrel != 0)
	{
		stepper_y_moving = 1;
		HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
	}
	while (stepper_x_moving || stepper_y_moving) {}

	tx_buffer[0] = 'a';
	tx_buffer[1] = 'm';
	*(int16_t*)(tx_buffer+2) = xrel;
	*(int16_t*)(tx_buffer+4) = yrel;
	send_message(tx_buffer,6);
}

void send_message(uint8_t* msg, int len)
{
	if (!msg) return;
	while (hdma_usart1_tx.State != HAL_DMA_STATE_READY)
	{}
	HAL_UART_AbortTransmit(&huart1);
	HAL_UART_Transmit_DMA(&huart1, msg, len);

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == LIMIT_PIN)
	{
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		limit_flag = 1;
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
