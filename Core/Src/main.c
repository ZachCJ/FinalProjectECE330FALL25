/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body Final Project SP2025
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
#include "seg7.h"

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
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM7_Init(void);


/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char ramp = 0;
char RED_BRT = 0;
char GREEN_BRT = 0;
char BLUE_BRT = 0;
char RED_STEP = 1;
char GREEN_STEP = 2;
char BLUE_STEP = 3;
char DIM_Enable = 0;
char Music_ON = 0;
int TONE = 0;
int COUNT = 0;
int INDEX = 0;
int Note = 0;
int Save_Note = 0;
int Vibrato_Depth = 1;
int Vibrato_Rate = 40;
int Vibrato_Count = 0;
char Animate_On = 0;
char Message_Length = 0;
char *Message_Pointer;
char *Save_Pointer;
int Delay_msec = 0;
int Delay_counter = 0;


/* HELLO ECE-330L */
char Message[] =
		{SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,
		CHAR_H,CHAR_E,CHAR_L,CHAR_L,CHAR_O,SPACE,CHAR_E,CHAR_C,CHAR_E,DASH,CHAR_3,CHAR_3,CHAR_0,CHAR_L,
		SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,SPACE};

/* Declare array for Song */
Music Song[100];
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
  MX_TIM7_Init();
  //MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  /********************************************************************
   * PWR->CR |= ???;  //Enable Real Time Clock (RTC) Register Access  *
   * RCC->BDCR |= ???;  //Set clock source for RTC                    *
   * RCC->BDCR |= ???; //Enable RTC									  *
   ********************************************************************/

  /*** Configure GPIOs ***/
  GPIOD->MODER = 0x55555555; // set all Port D pins to outputs
  GPIOA->MODER |= 0x000000FF; // Port A mode register - make A0 to A3 analog pins
  GPIOE->MODER |= 0x55555555; // Port E mode register - make E0 to E15 outputs
  GPIOC->MODER |= 0x0; // Port C mode register - all inputs
  GPIOE->ODR = 0xFFFF; // Set all Port E pins high

  /*** Configure ADC1 ***/
  RCC->APB2ENR |= 1<<8;  // Turn on ADC1 clock by forcing bit 8 to 1 while keeping other bits unchanged
  ADC1->SMPR2 |= 1; // 15 clock cycles per sample
  ADC1->CR2 |= 1;        // Turn on ADC1 by forcing bit 0 to 1 while keeping other bits unchanged

  /*****************************************************************************************************
  These commands are handled as part of the MX_TIM7_Init() function and don't need to be enabled
  RCC->AHB1ENR |= 1<<5; // Enable clock for timer 7
  __enable_irq(); // Enable interrupts
  NVIC_EnableIRQ(TIM7_IRQn); // Enable Timer 7 Interrupt in the NVIC controller
  *******************************************************************************************************/

  TIM7->PSC = 199; //250Khz timer clock prescaler value, 250Khz = 50Mhz / 200
  TIM7->ARR = 1; // Count to 1 then generate interrupt (divide by 2), 125Khz interrupt rate to increment byte counter for 78Hz PWM
  TIM7->DIER |= 1; // Enable timer 7 interrupt
  TIM7->CR1 |= 1; // Enable timer counting

  /* Jeopardy Song */
  Song[0].note = A4;
  Song[0].size = quarter;
  Song[0].tempo = 1400;
  Song[0].space = 10;
  Song[0].end = 0;

  Song[1].note = D5;
  Song[1].size = quarter;
  Song[1].tempo = 1400;
  Song[1].space = 10;
  Song[1].end = 0;

  Song[2].note = A4;
  Song[2].size = quarter;
  Song[2].tempo = 1400;
  Song[2].space = 10;
  Song[2].end = 0;

  Song[3].note = D4;
  Song[3].size = quarter;
  Song[3].tempo = 1400;
  Song[3].space = 10;
  Song[3].end = 0;

  Song[4].note = A4;
  Song[4].size = quarter;
  Song[4].tempo = 1400;
  Song[4].space = 10;
  Song[4].end = 0;

  Song[5].note = D5;
  Song[5].size = quarter;
  Song[5].tempo = 1400;
  Song[5].space = 10;
  Song[5].end = 0;

  Song[6].note = A4;
  Song[6].size = quarter;
  Song[6].tempo = 1400;
  Song[6].space = 10;
  Song[6].end = 0;

  Song[7].note = rest;
  Song[7].size = quarter;
  Song[7].tempo = 1400;
  Song[7].space = 10;
  Song[7].end = 0;

  Song[8].note = A4;
  Song[8].size = quarter;
  Song[8].tempo = 1400;
  Song[8].space = 10;
  Song[8].end = 0;

  Song[9].note = D5;
  Song[9].size = quarter;
  Song[9].tempo = 1400;
  Song[9].space = 10;
  Song[9].end = 0;

  Song[10].note = A4;
  Song[10].size = quarter;
  Song[10].tempo = 1400;
  Song[10].space = 10;
  Song[10].end = 0;

  Song[11].note = D5;
  Song[11].size = quarter;
  Song[11].tempo = 1400;
  Song[11].space = 10;
  Song[11].end = 0;

  Song[12].note = Fs5_Gb5;
  Song[12].size = quarter;
  Song[12].tempo = 1400;
  Song[12].space = 100;
  Song[12].end = 0;

  Song[13].note = rest;
  Song[13].size = _8th;
  Song[13].tempo = 1400;
  Song[13].space = 10;
  Song[13].end = 0;

  Song[14].note = E5;
  Song[14].size = _8th;
  Song[14].tempo = 1400;
  Song[14].space = 10;
  Song[14].end = 0;

  Song[15].note = D5;
  Song[15].size = _8th;
  Song[15].tempo = 1400;
  Song[15].space = 10;
  Song[15].end = 0;

  Song[16].note = Cs5_Db5;
  Song[16].size = _8th;
  Song[16].tempo = 1400;
  Song[16].space = 10;
  Song[16].end = 0;

  Song[17].note = B4;
  Song[17].size = _8th;
  Song[17].tempo = 1400;
  Song[17].space = 10;
  Song[17].end = 0;

  Song[18].note = As4_Bb4;
  Song[18].size = _8th;
  Song[18].tempo = 1400;
  Song[18].space = 10;
  Song[18].end = 0;

  Song[19].note = A4;
  Song[19].size = quarter;
  Song[19].tempo = 1400;
  Song[19].space = 10;
  Song[19].end = 0;

  Song[20].note = D5;
  Song[20].size = quarter;
  Song[20].tempo = 1400;
  Song[20].space = 10;
  Song[20].end = 0;

  Song[21].note = A4;
  Song[21].size = quarter;
  Song[21].tempo = 1400;
  Song[21].space = 10;
  Song[21].end = 0;

  Song[22].note = Fs4_Gb4;
  Song[22].size = _8th;
  Song[22].tempo = 1400;
  Song[22].space = 10;
  Song[22].end = 0;

  Song[23].note = G4;
  Song[23].size = _8th;
  Song[23].tempo = 1400;
  Song[23].space = 10;
  Song[23].end = 0;

  Song[24].note = A4;
  Song[24].size = quarter;
  Song[24].tempo = 1400;
  Song[24].space = 10;
  Song[24].end = 0;

  Song[25].note = D5;
  Song[25].size = quarter;
  Song[25].tempo = 1400;
  Song[25].space = 10;
  Song[25].end = 0;

  Song[26].note = A4;
  Song[26].size = quarter;
  Song[26].tempo = 1400;
  Song[26].space = 10;
  Song[26].end = 0;

  Song[27].note = rest;
  Song[27].size = quarter;
  Song[27].tempo = 1400;
  Song[27].space = 10;
  Song[27].end = 0;

  Song[28].note = D5;
  Song[28].size = quarter;
  Song[28].tempo = 1400;
  Song[28].space = 100;
  Song[28].end = 0;

  Song[29].note = rest;
  Song[29].size = _8th;
  Song[29].tempo = 1400;
  Song[29].space = 10;
  Song[29].end = 0;

  Song[30].note = B4;
  Song[30].size = _8th;
  Song[30].tempo = 1400;
  Song[30].space = 10;
  Song[30].end = 0;

  Song[31].note = A4;
  Song[31].size = quarter;
  Song[31].tempo = 1400;
  Song[31].space = 100;
  Song[31].end = 0;

  Song[32].note = G4;
  Song[32].size = quarter;
  Song[32].tempo = 1400;
  Song[32].space = 100;
  Song[32].end = 0;

  Song[33].note = Fs4_Gb4;
  Song[33].size = quarter;
  Song[33].tempo = 1400;
  Song[33].space = 100;
  Song[33].end = 0;

  Song[34].note = E4;
  Song[34].size = quarter;
  Song[34].tempo = 1400;
  Song[34].space = 100;
  Song[34].end = 0;

  Song[35].note = D4;
  Song[35].size = quarter;
  Song[35].tempo = 1400;
  Song[35].space = 100;
  Song[35].end = 0;

  Song[36].note = rest;
  Song[36].size = quarter;
  Song[36].tempo = 1400;
  Song[36].space = 10;
  Song[36].end = 0;

  Song[37].note = C5;
  Song[37].size = quarter;
  Song[37].tempo = 1400;
  Song[37].space = 10;
  Song[37].end = 0;

  Song[38].note = F5;
  Song[38].size = quarter;
  Song[38].tempo = 1400;
  Song[38].space = 10;
  Song[38].end = 0;

  Song[39].note = C5;
  Song[39].size = quarter;
  Song[39].tempo = 1400;
  Song[39].space = 10;
  Song[39].end = 0;

  Song[40].note = F4;
  Song[40].size = _8th;
  Song[40].tempo = 1400;
  Song[40].space = 10;
  Song[40].end = 0;

  Song[41].note = F4;
  Song[41].size = _8th;
  Song[41].tempo = 1400;
  Song[41].space = 10;
  Song[41].end = 0;

  Song[42].note = C5;
  Song[42].size = quarter;
  Song[42].tempo = 1400;
  Song[42].space = 10;
  Song[42].end = 0;

  Song[43].note = F5;
  Song[43].size = quarter;
  Song[43].tempo = 1400;
  Song[43].space = 10;
  Song[43].end = 0;

  Song[44].note = C5;
  Song[44].size = quarter;
  Song[44].tempo = 1400;
  Song[44].space = 10;
  Song[44].end = 0;

  Song[45].note = rest;
  Song[45].size = quarter;
  Song[45].tempo = 1400;
  Song[45].space = 10;
  Song[45].end = 0;

  Song[46].note = C5;
  Song[46].size = quarter;
  Song[46].tempo = 1400;
  Song[46].space = 10;
  Song[46].end = 0;

  Song[47].note = F5;
  Song[47].size = quarter;
  Song[47].tempo = 1400;
  Song[47].space = 10;
  Song[47].end = 0;

  Song[48].note = C5;
  Song[48].size = quarter;
  Song[48].tempo = 1400;
  Song[48].space = 10;
  Song[48].end = 0;

  Song[49].note = F5;
  Song[49].size = quarter;
  Song[49].tempo = 1400;
  Song[49].space = 10;
  Song[49].end = 0;

  Song[50].note = A5;
  Song[50].size = quarter;
  Song[50].tempo = 1400;
  Song[50].space = 0;
  Song[50].end = 0;

  Song[51].note = A5;
  Song[51].size = _8th;
  Song[51].tempo = 1400;
  Song[51].space = 10;
  Song[51].end = 0;

  Song[52].note = G5;
  Song[52].size = _8th;
  Song[52].tempo = 1400;
  Song[52].space = 10;
  Song[52].end = 0;

  Song[53].note = F5;
  Song[53].size = _8th;
  Song[53].tempo = 1400;
  Song[53].space = 10;
  Song[53].end = 0;

  Song[54].note = E5;
  Song[54].size = _8th;
  Song[54].tempo = 1400;
  Song[54].space = 10;
  Song[54].end = 0;

  Song[55].note = D5;
  Song[55].size = _8th;
  Song[55].tempo = 1400;
  Song[55].space = 10;
  Song[55].end = 0;

  Song[56].note = Cs5_Db5;
  Song[56].size = _8th;
  Song[56].tempo = 1400;
  Song[56].space = 10;
  Song[56].end = 0;

  Song[57].note = C5;
  Song[57].size = quarter;
  Song[57].tempo = 1400;
  Song[57].space = 10;
  Song[57].end = 0;

  Song[58].note = F5;
  Song[58].size = quarter;
  Song[58].tempo = 1400;
  Song[58].space = 10;
  Song[58].end = 0;

  Song[59].note = C5;
  Song[59].size = quarter;
  Song[59].tempo = 1400;
  Song[59].space = 10;
  Song[59].end = 0;

  Song[60].note = A4;
  Song[60].size = _8th;
  Song[60].tempo = 1400;
  Song[60].space = 10;
  Song[60].end = 0;

  Song[61].note = As4_Bb4;
  Song[61].size = _8th;
  Song[61].tempo = 1400;
  Song[61].space = 10;
  Song[61].end = 0;

  Song[62].note = C5;
  Song[62].size = quarter;
  Song[62].tempo = 1400;
  Song[62].space = 10;
  Song[62].end = 0;

  Song[63].note = F5;
  Song[63].size = quarter;
  Song[63].tempo = 1400;
  Song[63].space = 10;
  Song[63].end = 0;

  Song[64].note = C5;
  Song[64].size = quarter;
  Song[64].tempo = 1400;
  Song[64].space = 10;
  Song[64].end = 0;

  Song[65].note = rest;
  Song[65].size = _16th;
  Song[65].tempo = 1400;
  Song[65].space = 10;
  Song[65].end = 0;

  Song[66].note = C5;
  Song[66].size = _16th;
  Song[66].tempo = 1400;
  Song[66].space = 10;
  Song[66].end = 0;

  Song[67].note = D5;
  Song[67].size = _16th;
  Song[67].tempo = 1400;
  Song[67].space = 10;
  Song[67].end = 0;

  Song[68].note = E5;
  Song[68].size = _16th;
  Song[68].tempo = 1400;
  Song[68].space = 10;
  Song[68].end = 0;

  Song[69].note = F5;
  Song[69].size = quarter;
  Song[69].tempo = 1400;
  Song[69].space = 100;
  Song[69].end = 0;

  Song[70].note = rest;
  Song[70].size = _8th;
  Song[70].tempo = 1400;
  Song[70].space = 10;
  Song[70].end = 0;

  Song[71].note = D5;
  Song[71].size = _8th;
  Song[71].tempo = 1400;
  Song[71].space = 10;
  Song[71].end = 0;

  Song[72].note = C5;
  Song[72].size = quarter;
  Song[72].tempo = 1400;
  Song[72].space = 100;
  Song[72].end = 0;

  Song[73].note = As4_Bb4;
  Song[73].size = quarter;
  Song[73].tempo = 1400;
  Song[73].space = 100;
  Song[73].end = 0;

  Song[74].note = A4;
  Song[74].size = quarter;
  Song[74].tempo = 1400;
  Song[74].space = 100;
  Song[74].end = 0;

  Song[75].note = rest;
  Song[75].size = quarter;
  Song[75].tempo = 1400;
  Song[75].space = 100;
  Song[75].end = 0;

  Song[76].note = G4;
  Song[76].size = quarter;
  Song[76].tempo = 1400;
  Song[76].space = 100;
  Song[76].end = 0;

  Song[77].note = rest;
  Song[77].size = quarter;
  Song[77].tempo = 1400;
  Song[77].space = 100;
  Song[77].end = 0;

  Song[78].note = F4;
  Song[78].size = quarter;
  Song[78].tempo = 1400;
  Song[78].space = 100;
  Song[78].end = 0;

  Song[99].note = rest;
  Song[99].size = quarter;
  Song[99].tempo = 1400;
  Song[99].space = 10;
  Song[99].end = 1;


  Save_Note = Song[0].note;  // Needed for vibrato effect
  INDEX = 0;
  Music_ON = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  Message_Pointer = &Message[0];
  Save_Pointer = &Message[0];
  Message_Length = sizeof(Message)/sizeof(Message[0]);
  Delay_msec = 200;
  Animate_On = 0;


  while (1)
  {

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}






static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
