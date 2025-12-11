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
static void MX_RTC_Init(void);
static void MX_TIM7_Init(void);
void RTC_Init(void);
void RTC_Set_Date(uint32_t new_date);
void RTC_Set_Time(uint32_t new_time);
void RTC_Set_Alarm(void);
void Systik_Handler(void);
uint32_t time_format(uint32_t x);
void date_message_gen(uint32_t x);
extern void Seven_Segment(unsigned int HexValue);


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
uint32_t alarm;
int alarm_enabled = 0;
int toggleDateTime = 0;//Default Time Display State
int toggleMode = 0; //Default Display State
uint32_t set_time = 0; //Default time
uint32_t set_date = 0;
int display_index = 0;

/* HELLO ECE-330L */
char Message[35];

/* Declare array for Song */
Music Song[5];
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
  MX_RTC_Init();
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

  /* Initialize the first index*/
  Song[0].note = A4;
  Song[0].size = quarter;
  Song[0].tempo = 2400;
  Song[0].space = 100;
  Song[0].end = 0;

  Song[1].note = C4;
  Song[1].size = quarter;
  Song[1].tempo = 2400;
  Song[1].space = 100;
  Song[1].end = 0;

  Song[2].note = A4;
  Song[2].size = quarter;
  Song[2].tempo = 2400;
  Song[2].space = 100;
  Song[2].end = 0;

  Song[3].note = C4;
  Song[3].size = quarter;
  Song[3].tempo = 2400;
  Song[3].space = 100;
  Song[3].end = 0;

  Song[4].note = A4;
  Song[4].size = quarter;
  Song[4].tempo = 2400;
  Song[4].space = 100;
  Song[4].end = 0;

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
	  //Clock Set Time Mode
	  if(toggleMode == 1){
		  RCC->CR &= ~(0b1 << 8);//Allows write access
		  RCC->BDCR &= ~(0b1 << 15);//Disables clock
		  Seven_Segment(time_format(set_time));
		  GPIOD->ODR = 0x1 << display_index; //Tells what number is being edited
		  //Filtering invalid values for time
		  if(!((GPIOC->IDR & 0xF) > 9)){
		  	  set_time &= ~(0xF << (display_index * 4));
			  set_time |= (GPIOC->IDR & 0xF) << (display_index * 4);//Reads the first four switches to set the time
		  }

		  //PC11 Button Changes index
		  if(!(GPIOC->IDR & (1 << 11))){
			  (display_index == 5) ? (display_index = 0) : (display_index++);//Increments the display index
			  HAL_Delay(20);
		  	  while(!(GPIOC->IDR & (1 << 11)));//Wait for button release
		  }

	  //Clock Set Date Mode
	  } else if(toggleMode == 2){
		  RCC->CR &= ~(0b1 << 8);//Allows write access
		  RCC->BDCR &= ~(0b1 << 15);//Disables clock
		  date_message_gen(set_date);
		  Animate_On = 2;

		  GPIOD->ODR = 0x1 << display_index; //Tells what number is being edited

		  if(!((GPIOC->IDR & 0xF) > 9)){
			  switch(display_index){
				  case 0://Day One's
					  set_date &= ~0xF;
					  set_date |= (GPIOC->IDR & 0xF);
					  break;
				  case 1://Day Ten's
					  set_date &= ~0x30;
					  set_date |= (GPIOC->IDR & 0x3) << 4;
					  break;
				  case 2://Month One's
					  set_date &= ~0xF00;
					  set_date |= (GPIOC->IDR & 0xF) << 8;
					  break;
				  case 3://Month Ten's
					  set_date &= ~0x1000;
					  set_date |= (GPIOC->IDR & 0x1) << 12;
					  break;
				  case 4://Weekday
					  set_date &= ~0xE000;
					  set_date |= (GPIOC->IDR & 0x7) << 13;
					  break;
				  case 5://Year One's
					  set_date &= ~0xF0000;
					  set_date |= (GPIOC->IDR & 0xF) << 16;
					  break;
				  case 6://Year Ten's
					  set_date &= ~0xF00000;
					  set_date |= (GPIOC->IDR & 0xF) << 20;
					  break;
			  }
		  }

		  //PC11 Button changes index
		  if(!(GPIOC->IDR & (1 << 11))){
			  (display_index == 6) ? (display_index = 0) : (display_index++);//Increments the display index
			  if(display_index >= 4){
				  Message_Pointer = &Message[17];
			  } else {
				  Message_Pointer = &Message[12];
			  }
			  HAL_Delay(20);
			  while(!(GPIOC->IDR & (1 << 11)));//Wait for button release
		  }


	  //Clock Set Alarm Mode
	  } else if(toggleMode == 3){
		  RCC->CR &= ~(0b1 << 8);//Allows write access
		  RCC->BDCR &= ~(0b1 << 15);//Disables clock
		  Seven_Segment(time_format(set_time));
		  GPIOD->ODR = 0x1 << display_index; //Tells what number is being edited
		  //Filtering invalid values for time
		  if(!((GPIOC->IDR & 0xF) > 9)){
		  	  set_time &= ~(0xF << (display_index * 4));
			  set_time |= (GPIOC->IDR & 0xF) << (display_index * 4);//Reads the first four switches to set the time
		  }

		  //PC11 Button Changes index
		  if(!(GPIOC->IDR & (1 << 11))){
			  (display_index == 5) ? (display_index = 0) : (display_index++);//Increments the display index
			  HAL_Delay(20);
		  	  while(!(GPIOC->IDR & (1 << 11)));//Wait for button release
		  }

	  //Clock Display Time & Date Mode
	  } else {
		  GPIOD->ODR = 0;
		  RCC->BDCR |= (0b1 << 15);//Makes sure the clock is active
		  RCC->CR |= (0b1 << 8);//Disables write access

		  //Displaying time
		  if(toggleDateTime == 0){
	 		  Seven_Segment(time_format(RTC->TR)); //Sends the clock data to the time register
	 	  	  Animate_On = 0; //Makes sure the time isn't a scrolling message

	 	  //Displaying Date
		  } else if(toggleDateTime == 1){
			  date_message_gen(RTC->DR); //Puts the date into the Message array to display
			  Animate_On = 1; //Activates the scrolling message
	  	  }
		  //In this mode PC11 toggles the date and time
		  if(!(GPIOC->IDR & (1 << 11))){
			  (toggleDateTime == 0) ? (toggleDateTime = 1) : (toggleDateTime = 0); // toggles the bit to display date or time
		  	  HAL_Delay(15);
		  	  while(!(GPIOC->IDR & (1 << 11)));//Wait for button release
		  }
	  }
	  //PC11 Button toggles the clock mode
	  if(!(GPIOC->IDR & (1 <<10 ))){
		  if(toggleMode == 1){
			  RTC_Set_Time(set_time);
		  } else {
			  set_time = RTC->TR; //log the time
		  }
		  if(toggleMode == 2){
			  RTC_Set_Date(set_date);
		  } else {
			  set_date = RTC->DR; //log the time
			  Message_Pointer = &Message[12];
 		  }
		  (toggleMode == 2) ? (toggleMode = 0) : (toggleMode++);//Increments the mode
		  display_index = 0; //reset display index
		  HAL_Delay(15);
		  while(!(GPIOC->IDR & (1 << 10)));//Wait for button release
	  }

	  //Checks switch 15 to enable alarm
	 if((GPIOC->IDR & (0b1 << 15)) == 0b1 << 15){
		 RTC->CR &= RTC_CR_ALRAE; //Enable Alarm
	 } else {
		 RTC->CR &= ~RTC_CR_ALRAE; //Disable alarm
	 }


    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

uint32_t time_format(uint32_t x)
{
	uint32_t time = x & 0xFF; //Seconds
	time |= 0xA << 8;
	time |= (x & 0xFF << 8) << 4; //Minutes
	time |= 0xA << 20;
	time |= (x & 0xFF << 16) << 8; //Hours
	return time;
}

void date_message_gen(uint32_t x){
	Message[0] = SPACE;
	Message[1] = SPACE;
	Message[2] = SPACE;
	Message[3] = SPACE;
	Message[4] = SPACE;
	Message[5] = SPACE;
	Message[6] = SPACE;
	Message[7] = SPACE;
	Message[8] = SPACE;
	Message[9] = SPACE;
	Message[10] = SPACE;
	Message[11] = SPACE;

	Message[13] = (x & 0x3); //Day one's position
	Message[12] = (x & 0x30) >> 4; //Day ten's position
	Message[14] = SPACE;
	Message[16] = (x & (0xF << 8)) >> 8;//Month one's position
	Message[15] = (x & (0x1 << 12)) >> 12;//Month ten's position
	Message[17] = SPACE;
	Message[18] = (x & (0x7 << 13)) >> 13;//Weekday
	Message[19] = SPACE;
	Message[23] = (x & (0xF << 16)) >> 16;//Year one's position
	Message[22] = (x & (0xF << 20)) >> 20;//Year ten's position
	Message[21] = 0;
	Message[20] = 2;

	Message[24] = SPACE;
	Message[25] = SPACE;
	Message[26] = SPACE;
	Message[27] = SPACE;
	Message[28] = SPACE;
	Message[29] = SPACE;
	Message[30] = SPACE;
	Message[31] = SPACE;
	Message[32] = SPACE;
	Message[33] = SPACE;
	Message[34] = SPACE;
	Message[35] = SPACE;
}


/*Initialize the RTC Clock*/
void MX_RTC_Init(void)
{
	PWR->CR |= (0b1 << 8); // enable real time clock (RTC) register access
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;//Enabling write access
	RTC->ISR |= RTC_ISR_INIT;

	RCC->BDCR &= ~(0b11 << 8); // clear bits 9:8
	RCC->BDCR |= (0b10 << 8); //select LSI oscillator clock as 10

	RCC->BDCR |= (0b1 << 15); // enable RTC

	while (!(RTC->ISR & RTC_ISR_INITF));

	RTC->PRER = 0x102; // set lower portion to 258
	RTC->PRER |= 0x007F0000; // set upper portion to 127

	RTC->CR &= ~RTC_CR_FMT; // set to 24-hr clock

	RTC->ISR &= ~RTC_ISR_INIT;
}

/*Sets the date off a given date*/
void RTC_Set_Date(uint32_t new_date)
{
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;//Enabling write access
	RTC->ISR |= RTC_ISR_INIT; //Enter initialization mode

	RCC->BDCR &= ~(0b11 << 8); // clear bits 9:8
	RCC->BDCR |= (0b10 << 8); //select LSI oscillator clock as 10

	RCC->BDCR |= (0b1 << 15); // enable RTC

	while (!(RTC->ISR & RTC_ISR_INITF));

	RTC->PRER = 0x102; // set lower portion to 258
	RTC->PRER |= 0x007F0000; // set upper portion to 127

	RTC->DR |= new_date;
	RTC->CR &= ~RTC_CR_FMT; // set to 24-hr clock

	RTC->ISR ^= RTC_ISR_INIT; //Exit initialization mode
}


void RTC_Set_Time(uint32_t new_time)
{
	PWR->CR |= (0b1 << 8); // enable real time clock (RTC) register access
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;//Enabling write access
	RTC->ISR |= RTC_ISR_INIT;

	RCC->BDCR &= ~(0b11 << 8); // clear bits 9:8
	RCC->BDCR |= (0b10 << 8); //select LSI oscillator clock as 10

	RCC->BDCR |= (0b1 << 15); // enable RTC

	while (!(RTC->ISR & RTC_ISR_INITF));

	RTC->PRER = 0x102; // set lower portion to 258
	RTC->PRER |= 0x007F0000; // set upper portion to 127

	RTC->TR = new_time;
	RTC->CR &= ~RTC_CR_FMT; // set to 24-hr clock

	RTC->ISR &= ~RTC_ISR_INIT;
}

void RTC_Set_Alarm(void)
{

	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;//Enabling write access
	RTC->ISR |= RTC_ISR_INIT; // Put the clock into initialization mode
	RTC->CR &= ~RTC_CR_ALRAE; // clears ALRAE to disable alarm A

	while (!(RTC->ISR & RTC_ISR_ALRAWF));

	RTC->ALRMAR = alarm;

	RTC->CR |= (1<< 12); // enable alarm interrupt (ALRAIE)
	RTC->CR |= RTC_CR_ALRAE; // enable alarm A (ALRAE)
}

void Systik_Handler(void)
{
	//check if alarm A is triggered
	if(RTC->ISR & (1 << 8)){
		if(alarm_enabled){
			Music_ON = 1; // start music
			INDEX = 0; //reset song if needed
		}
		RTC->ISR &= ~(1 << 8); // clear alarm A flag
	}

	HAL_IncTick(); // keep HAL timing working, increments tick counter
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
