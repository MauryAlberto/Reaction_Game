/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "lcd16x2_i2c.h"
#include <time.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define TIM_FREQ 48000000
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t game_mode = 0;
uint8_t game_init = 0;
uint8_t game_reset = 0;
uint8_t button_left = 0;
uint8_t button_right = 0;
uint8_t button_up = 0;
uint8_t button_down = 0;
uint8_t button_check = 0;
uint8_t total_points = 0;
uint8_t previous_points = 0;
uint8_t LEDG = 0;
uint8_t LEDR = 0;
uint8_t lock = 1;
uint8_t score_board_lock = 1;
uint8_t display_score_board = 0;
uint8_t toggle = 1;
char curr_score[20] = "Final Score: 0";
char prev_sore[20] = "Last Score:  0";
char* direction;
int Mariomelody[] = {
659, 659, 659, 0, 523, 659, 784, 392, 523, 0, 392, 0, 330, 0, 440, 0, 494, 0, 466, 440, 392, 659, 784, 880, 698, 784, 0, 659, 0, 523, 587, 494, 0, 523, 0, 392, 0, 330, 0, 440, 0, 494, 0, 466, 440, 392, 659, 784, 880, 698, 784, 0
};
 MarionoteDurations[] = {
150, 300, 150, 150, 150, 300, 600, 600, 300, 150, 150, 300, 300, 150, 150, 150, 150, 150, 150, 300, 200, 200, 200, 300, 150, 150, 150, 150, 150, 150, 150, 150, 300, 300, 150, 150, 300, 300, 150, 150, 150, 150, 150, 150, 300, 200, 200, 200, 300, 150, 150, 150
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM14_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void start_game(void);
int presForFrequency(int frequency);
void play_song(int *tone, int* duration, int* pause, int size);
void noTone(void);
//void button_check_func(char* direction);
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
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  lcd16x2_i2c_clear();
  lcd16x2_i2c_init(&hi2c1);
  lcd16x2_i2c_printf("Reaction Game");
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(game_init){
      score_board_lock = 0;
      noTone();
      lcd16x2_i2c_clear();
      lcd16x2_i2c_1stLine();
      lcd16x2_i2c_printf("Starting in 3...");
      lcd16x2_i2c_2ndLine();
      lcd16x2_i2c_printf("Score: 0");
      HAL_Delay(1000);
      if(game_reset){
        continue;
      }
      lcd16x2_i2c_1stLine();
      lcd16x2_i2c_printf("Starting in 2...");
      HAL_Delay(1000);
      if(game_reset){
        continue;
      }
      lcd16x2_i2c_1stLine();
      lcd16x2_i2c_printf("Starting in 1...");
      HAL_Delay(1000);
      if(game_reset){
        continue;
      }
      lcd16x2_i2c_1stLine();
      lcd16x2_i2c_printf("      GO!!      ");
      HAL_Delay(1000);
      if(game_reset){
        continue;
      }
      game_init = 0;
      start_game();
    }else if(game_reset){
      lcd16x2_i2c_clear();
      lcd16x2_i2c_1stLine();
      lcd16x2_i2c_printf("Reaction Game   ");
      game_reset = 0;
      score_board_lock = 1;
    }else if(display_score_board){
      if(toggle){
        lcd16x2_i2c_1stLine();
        lcd16x2_i2c_printf(curr_score);
        lcd16x2_i2c_2ndLine();
        lcd16x2_i2c_printf(prev_sore);
        toggle = 0;
        display_score_board = 0;
      }else{
        lcd16x2_i2c_clear();
        lcd16x2_i2c_1stLine();
        lcd16x2_i2c_printf("Reaction Game   ");
        toggle = 1;
        display_score_board = 0;
      }
    }else{
      play_song(Mariomelody, MarionoteDurations, NULL, sizeof(Mariomelody)/sizeof(Mariomelody[0]));
      noTone();
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

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x10805D88;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim3.Init.Period = 12000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
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
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 8000-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 1500-1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB11 PB12 PB13 PB6
                           PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_6
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void start_game(void){//start the game
  char num_to_char[2];
  char full_string[] = "Final Score: ";
  char full_string2[] = "Last Score:  ";
  char random_directions[4][6] = {"LEFT", "RIGHT", " UP", "DOWN"};//initialize an array of pointers to a char array
  char* direction2 = (char*)malloc(6 * sizeof(char));//allocate memory for a single char (6 bytes for 1 word because each char is a byte long)
  direction = direction2;//set direction2 to the global variable definition of direction due to some weird error
  char* previous;//previous stores the previous value of direction so that no direction is repeated more than once in a row
  uint8_t random_index = 0;//variable to store a random index to random_directions
  for(int j = 0; j < 15; j++){//for - loop used to iterate through 15 random directions
    if(game_reset){//check if the user reset the game
      break;//break from the for - loop and return to main
    }
    srand(TIM3->CNT);//seed srand with a new value by taking the input from a continuous timer
    random_index = rand() % 4;//retrieve a random index
    previous = direction;//store the current direction into previous
    direction = random_directions[random_index];//assign direction a new random direction
    while(strcmp(previous, direction) == 0){// prevents a direction from repeating more than once in a row
      random_index = rand() % 4;//retrieve a new random index
      direction = random_directions[random_index];//get new char array pointer
    }
    lcd16x2_i2c_1stLine();
    lcd16x2_i2c_printf("               ");
    lcd16x2_i2c_setCursor(0, 6);//got to row 0 column 6 on the LCD display
    lcd16x2_i2c_printf(direction);//print direction
    HAL_Delay(1000);//wait 1 second for the user to respond
    if(lock){
      LEDR = 1;
      __HAL_TIM_SET_PRESCALER(&htim1, presForFrequency(500));
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
      HAL_TIM_Base_Start_IT(&htim14);
    }
    lock = 1;
  }
  game_reset = 1;//reset the game
  free(direction2);//free up the allocate memory
  lcd16x2_i2c_clear();
  lcd16x2_i2c_1stLine();
  itoa(total_points, num_to_char, 10);
  strcat(full_string, num_to_char);
  strcpy(curr_score, full_string);


  itoa(previous_points, num_to_char, 10);
  strcat(full_string2, num_to_char);
  strcpy(prev_sore, full_string2);

  lcd16x2_i2c_printf(full_string);
  lcd16x2_i2c_2ndLine();
  lcd16x2_i2c_printf(full_string2);

  previous_points = total_points;
  total_points = 0;

  HAL_Delay(2000);
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin){
  char num_to_char[2];
  char full_string[10] = "Score: ";
  if(GPIO_Pin == GPIO_PIN_11 && score_board_lock){
    display_score_board = 1;
  }else if(GPIO_Pin == GPIO_PIN_12){
    game_init = 1;
  }else if (GPIO_Pin == GPIO_PIN_13){
    game_reset = 1;
    game_init = 0;
    game_mode = 0;
    total_points = 0;
    score_board_lock = 1;
  }else if(GPIO_Pin == GPIO_PIN_4 && lock){//button left
    if((strcmp(direction, "LEFT") == 0)){
        lock = 0;
        LEDG = 1;
        total_points++;
        itoa(total_points, num_to_char, 10);
        strcat(full_string, num_to_char);
        lcd16x2_i2c_2ndLine();
        lcd16x2_i2c_printf(full_string);
        __HAL_TIM_SET_PRESCALER(&htim1, presForFrequency(1000));
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
        HAL_TIM_Base_Start_IT(&htim14);
      }else if(lock){
        lock = 0;
        LEDR = 1;
        __HAL_TIM_SET_PRESCALER(&htim1, presForFrequency(500));
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
        HAL_TIM_Base_Start_IT(&htim14);
      }
  }else if(GPIO_Pin == GPIO_PIN_10 && lock){//button up
    if((strcmp(direction, " UP") == 0)){
        lock = 0;
        LEDG = 1;
        total_points++;
        itoa(total_points, num_to_char, 10);
        strcat(full_string, num_to_char);
        lcd16x2_i2c_2ndLine();
        lcd16x2_i2c_printf(full_string);
        __HAL_TIM_SET_PRESCALER(&htim1, presForFrequency(1000));
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
        HAL_TIM_Base_Start_IT(&htim14);
    }else if(lock){
      lock = 0;
      LEDR = 1;
      __HAL_TIM_SET_PRESCALER(&htim1, presForFrequency(500));
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
      HAL_TIM_Base_Start_IT(&htim14);
    }
  }else if(GPIO_Pin == GPIO_PIN_6 && lock){//button right
    if((strcmp(direction, "RIGHT") == 0)){
        lock = 0;
        LEDG = 1;
        total_points++;
        itoa(total_points, num_to_char, 10);
        strcat(full_string, num_to_char);
        lcd16x2_i2c_2ndLine();
        lcd16x2_i2c_printf(full_string);
        __HAL_TIM_SET_PRESCALER(&htim1, presForFrequency(1000));
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
        HAL_TIM_Base_Start_IT(&htim14);
      }else if(lock){
        lock = 0;
        LEDR = 1;
        __HAL_TIM_SET_PRESCALER(&htim1, presForFrequency(500));
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
        HAL_TIM_Base_Start_IT(&htim14);
      }
  }else if(GPIO_Pin == GPIO_PIN_7 && lock){//button down
    if((strcmp(direction, "DOWN") == 0)){
      lock = 0;
      LEDG = 1;
      total_points++;
      itoa(total_points, num_to_char, 10);
      strcat(full_string, num_to_char);
      lcd16x2_i2c_2ndLine();
      lcd16x2_i2c_printf(full_string);
      __HAL_TIM_SET_PRESCALER(&htim1, presForFrequency(1000));
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
      HAL_TIM_Base_Start_IT(&htim14);
      }else if(lock){
        lock = 0;
        LEDR = 1;
        __HAL_TIM_SET_PRESCALER(&htim1, presForFrequency(500));
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
        HAL_TIM_Base_Start_IT(&htim14);
      }
  }
  return;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){//timer 14 is used as the period of the ON state of both LEDs

  if(htim == &htim14 && LEDG){//check if the timer 14 is completed and if the green LED is on
    LEDG = 0;//reset LEDG ON signal to 0
    __HAL_TIM_SET_PRESCALER(&htim1, presForFrequency(0));
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);//turn off Green LED
    HAL_TIM_Base_Stop_IT(&htim14);//stop timer 14
  }else if(htim == &htim14 && LEDR){//check if the timer 14 is completed and if the green LED is on
    LEDR = 0;//rest LEDR ON signal to 0
    __HAL_TIM_SET_PRESCALER(&htim1, presForFrequency(0));
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);//turn off Red LED
    HAL_TIM_Base_Stop_IT(&htim14);//stop timer 14
  }


}

int presForFrequency(int frequency){
  if(frequency == 0){
    return 0;
  }else{
    return (TIM_FREQ/(1000*frequency));
  }
}

void noTone(void){
  __HAL_TIM_SET_PRESCALER(&htim1, 0);
}

void play_song(int *tone, int* duration, int* pause, int size){
  for(int i = 0; i < size; i++){
    if(game_init || game_reset || display_score_board){
      break;
    }
    int pres = presForFrequency(tone[i]);
    int dur = duration[i];
    int pauseBetweenTones = 0;
    if(pause != NULL){
      pauseBetweenTones = pause[i] - duration[i];
    }
    __HAL_TIM_SET_PRESCALER(&htim1, pres-1);
    HAL_Delay(dur);
    noTone();
    HAL_Delay(pauseBetweenTones);
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
