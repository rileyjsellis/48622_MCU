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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* Note: although we can leave it, and SystemState will
 * count from 0, I feel far better leaving this explicit.
 * - Riley
 */

typedef enum {
	STATE_A = 0,
	STATE_B = 1,
	STATE_C = 2
}  SystemState;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//ALL LEDs
#define LED1_PORT GPIOB
#define LED1_PIN 4
#define LED2_PORT GPIOB
#define LED2_PIN 5
#define LED3_PORT GPIOB
#define LED3_PIN 3

#define BUTTON1_PIN 4
#define BUTTON2_PIN	13

#define BUTTON1 0
#define BUTTON2 1

//TIMER
#define TIMER_SEL TIM2
#define TIMER_IRQn TIM2_IRQn

//Potentiometer
#define POT_PORT GPIOA
#define POT_PIN 0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

volatile SystemState currentState = STATE_A;

volatile uint32_t ledBitPos[2] = {LED1_PIN,LED2_PIN};

volatile uint32_t counter[2] = {0};

volatile uint32_t debugCounter[4] = {0};
volatile uint32_t debugEXTIcr3 = 0;
volatile uint32_t lastDebounceTime = 0;

volatile uint32_t buttonPressed[2]= {0};

volatile uint32_t uartEnabled = 1;
volatile uint32_t ledToggle = 0;
volatile uint32_t ledOn = 0;
volatile uint32_t ledOff = 1;

volatile uint32_t blink = 0;

volatile uint32_t adcValue = 0;

volatile uint32_t whichLedToggled = 1;

volatile uint32_t cState = 0;       // 0 = OFF, 1 = ON
volatile uint32_t ledBlinkCounter = 0;  // Count OFF states (each full flash = 1)
volatile uint32_t ledDone = 0;        // Flag when 3 flashes complete

//for code review
volatile int32_t activity = 3; //start on 3, can edit in live expressions
volatile int32_t Potcounter = 0;
volatile int32_t hal_delay = 1;

//potentiometer
volatile int32_t adc_value = 0;

//for servo, 47 as minimum is explained below
volatile int32_t pwm_val = 470; //for activity 2 sweeping!
volatile int32_t direction = 0; // to be treated as boolean

volatile int32_t counter_test = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* Declare all functions here, rather than relying on
 * implicitly declarations from compiler:
 * - Riley */

void ConfigureGpioOutput(GPIO_TypeDef* port, uint32_t pin);
void ConfigureAdc(void);
void ConfigureButtonEXTI(void);
void ConfigureTimer(TIM_TypeDef *tim, uint32_t prescaler,
		uint32_t arr, IRQn_Type irqNum, uint8_t priority);

void HandleStateA(void);
void HandleStateB(void);
void HandleStateC(void);

int MapLinear(int x, int in_min, int in_max, int out_min, int out_max);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t ButtonPressed(uint8_t buttonIndex){
	if (buttonPressed[buttonIndex]) {
		buttonPressed[buttonIndex] = 0;
		return 1; //button press is new and valid
	}
	return 0; // No new press
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	  if (htim->Instance == TIM2){
		  // Toggle GPIO Pin Using HAL Libraries.
		  if (activity == 3){
			  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
		  }
		  else {
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,GPIO_PIN_RESET);
		  }
	  }
	  counter_test++;
}

void HandleStateA(void) {
	// To do: Display LCD SID
		//displayLcdSid(); // shows SID and course name

	// To do: Transmit UART if uartEnabled
    if (uartEnabled) {
        //transmitUart(adcValue); // send UART string every 500ms
    }

    if (ButtonPressed(BUTTON1)) {
		currentState = STATE_C;
		debugCounter[2];
		return;
	}
    if (ButtonPressed(BUTTON2)) {
        currentState = STATE_B;
        counter[0] = 0;
        return;
    }
}

void HandleStateB(void) {
	// ADC updates adcValue in ISR
	// led_period[1] (for LED3 PWM blink) already updated in ADC ISR

    //displayLcdAdc(adcValue);      // shows ADC value
    //updateLed3Blinking(adcValue); // update blink rate
    //rotateServo(adcValue);        // update PWM for servo

    if (buttonPressed[BUTTON1]) {
        ledToggle = 1; //Switch between LED1 and LED2 blinking
    }
    else{
    	ledToggle = 0;
    }
    if (ButtonPressed(BUTTON2)) {
        currentState = STATE_A;
        debugCounter[2];
        return;
    }
}

void HandleStateC(void) {
	 static uint8_t started = 0;

	if (!started) {
		started = 1;
		cState = 0;
		ledBlinkCounter = 0;
		ledDone = 0;

		LED3_PORT->BSRR = (1 << (LED3_PIN + 16)); // Ensure LED3 OFF
	}

	if (ledDone) {
		started = 0;
		currentState = STATE_A;
	}
}

/*LINEAR MAPPING
 * this exists for the input ADC value to be mapped to the
 * minimum and maximum of the servo motor PWM, 180 degrees rotation.
 */
int MapLinear(int x, int inMin, int inMax, int outMin, int outMax){
	return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

void TIM2_IRQHandler(void){
    TIM2->SR &= ~TIM_SR_UIF; // Clear update interrupt flag

    static uint16_t period = 1000;

    // --- LED1/2 handling (State B) ---
    if (currentState == STATE_B) {

    	ledOn= ledToggle;
		ledOff = !ledOff;
		whichLedToggled = ledOn + 1;


    	// set one LED to OFF
		GPIOB->BSRR = (1 << (ledBitPos[ledOff] + 16)); //whichever LED is off at the time

		//counter and
		if (counter[0] > period / 2 ){
			GPIOB->BSRR = (1 << ledBitPos[ledOn]); //LED blinks on
			blink = 1;
		}
		if (counter[0] < period / 2){
			GPIOB->BSRR = (1 << (ledBitPos[ledOn] + 16)); //LED blinks off
			blink = 0;
		}
    }

    // --- LED3 3x flash handling (State C) ---
    else if (currentState == STATE_C) {
    	//Start of state
    	if (cState == 0) {
			LED3_PORT->BSRR = (1 << LED3_PIN); // ON
			counter[1] = 0;
			cState = 1;
		}

    	//Phase 1 - Blinking Logic
    	if (cState == 1){
    		if (counter[1] > period / 2 ){
				GPIOB->BSRR = (1 << LED3_PIN); //LED blinks on
			}
			if (counter[1] <= period / 2){
				GPIOB->BSRR = (1 << (LED3_PIN + 16)); //LED blinks off
			}
			if (counter[1] <= period){
				counter[1] = 0;
				ledBlinkCounter++;
		        if (ledBlinkCounter >= 3) {
		        	ledDone = 1;
				}
			}

    	}
    }

    // Reset counter when full period completes
	counter[0] = (counter[0] + 1) % period;
}

void ADC1_COMP_IRQHandler(void){

	adcValue = ADC1->DR;

	ADC1->ISR |= (1 << 2); //clear the interrupt flag

	ADC1->CR |= 1 << 2; //start new ADC conversion

}

void ConfigureGpioOutput(GPIO_TypeDef* port, uint32_t pin) {

    // 1. Enable GPIO clock (based on port address offset from GPIOA)

    uint32_t portIndex = ((uint32_t)port - (uint32_t)GPIOA) / 0x400;
    RCC->IOPENR |= (1 << portIndex);

    // 2. Set pin as general purpose output (MODER = 01)

    port->MODER &= ~(3 << (pin * 2)); // clear mode bits
    port->MODER |=  (1 << (pin * 2)); // set to output mode

    // 3. Set push-pull, low speed, no pull

    port->OTYPER &= ~(1 << pin);      // push-pull
    port->OSPEEDR &= ~(3 << (pin * 2)); // low speed
    port->PUPDR &= ~(3 << (pin * 2)); // no pull-up/down
}

void ConfigureAdc(void){

	// 1. Enable PORT

	RCC->IOPENR |= (1 << 0); //GPIOA

	POT_PORT->MODER |= (3 << (POT_PIN * 2)); // Set MODER to 11
	POT_PORT->PUPDR &= ~(3 << (POT_PIN *2)); // No pull-up/down

	// 2. Enable ADC Clock

	RCC->APBENR2 |= (1 << 20); //ADC Clock Enable //RCC_APBENR2_ADCEN ?

	// 3. Configure ADC Parameters

	ADC1->CFGR1 &= ~(3 << 3); // 12-bit resolution 0b00
	ADC1->CFGR2 &= ~(3UL << 30); // Asynchronous clock mode 0b00
	//3UL to avoid compiler warnings near signed boundary
	ADC->CCR &= ~(0xF << 18); // 0b0000, Prescaler = 1

	ADC1->CHSELR |= (1 << POT_PIN); // Selecting pin

	ADC1->SMPR &= ~(7 << 4); // Clears bits 6:4 (SMP2[2:0])
	ADC1->SMPR |= (3 << 4); // Set bits 6:4 to 0b011 = 12.5 cycles 0b011
	ADC1->SMPR |= (1 << 8); // Set channel 0 to use SMP2 time

	// 4. Enable ADC Interrupt & Start

	ADC1->IER |= (1 << 2); // End-of-conversation interrupt
	ADC1->CR |= (1 << 0); // Enable ADC

	while (!(ADC1->ISR & (1 << 0))){}; // Wait for ready

	NVIC_SetPriority(ADC1_COMP_IRQn, 2);
	NVIC_EnableIRQ(ADC1_COMP_IRQn);

	ADC1->CR |= 1 << 2; // Start first conversion
}

void ConfigureButtonEXTI(void) {
    RCC->IOPENR |= RCC_IOPENR_GPIOCEN;
    RCC->APBENR2 |= RCC_APBENR2_SYSCFGEN;

    // PC4 and PC13 as input with pull-up
    GPIOC->MODER &= ~((3 << (4 * 2)) | (3 << (13 * 2)));
    GPIOC->PUPDR &= ~((3 << (4 * 2)) | (3 << (13 * 2)));
    GPIOC->PUPDR |=  ((1 << (4 * 2)) | (1 << (13 * 2))); // Pull-up for both

    // Map EXTI4 to PC4
    EXTI->EXTICR[1] &= ~(0xF << 0);        // Bits 3:0 for EXTI4
    EXTI->EXTICR[1] |=  (0x2 << 0);        // Port C = 0b0010

    // Map EXTI13 to PC13
    EXTI->EXTICR[3] &= ~(0xF << 8);
    EXTI->EXTICR[3] |=  (0x2 << 8); // Port C = 0b0010

    // Falling edge trigger only
    EXTI->RTSR1 |= (1 << 4) | (1 << 13);  // Enable rising
    EXTI->FTSR1 |= (1 << 4) | (1 << 13);    // Enable falling

    // Unmask both EXTI lines
    EXTI->IMR1 |= (1 << 4) | (1 << 13);

    // Disable event generation
    EXTI->EMR1 &= ~((1 << 4) | (1 << 13));

    NVIC_SetPriority(EXTI4_15_IRQn, 1);
    NVIC_EnableIRQ(EXTI4_15_IRQn);
}

void EXTI4_15_IRQHandler(void){

	//uint32_t now = HAL_GetTick(); //assume 200ms.

	//BUTTON 1
	// only read once to change state
	if (EXTI->FPR1 & (1 << 13)) {
		EXTI->FPR1 = (1 << 13);  // Clear flag
		buttonPressed[BUTTON2] = 1;
	}
	if (EXTI->RPR1 & (1 << 13)) {
		EXTI->RPR1 = (1 << 13);  // Clear flag
		buttonPressed[BUTTON2] = 0;
	}

	// BUTTON 2
	// must sustain long press and read once.
	// State A, read once to visit C
	// State B, sustained press to toggle LEDs.

	if ((EXTI->FPR1 & (1 << 4))) {
		buttonPressed[BUTTON1] = 0;
		debugCounter[0]++;
		//lastDebounceTime = now;
	}
	EXTI->FPR1 = (1 << 4);  // Clear flag

	if ((EXTI->RPR1 & (1 << 4))) {
		buttonPressed[BUTTON1] = 1;
		debugCounter[1]++;
		//lastDebounceTime = now;
	}
	EXTI->RPR1 = (1 << 4);  // Clear flag

}


void ConfigureTimer(TIM_TypeDef *tim, uint32_t prescaler,
		uint32_t arr, IRQn_Type irqNum, uint8_t priority){

	// 1. Enable timer clock
	if (tim == TIM1)   RCC->APBENR2 |= RCC_APBENR2_TIM1EN;
	else if (tim == TIM2) RCC->APBENR1 |= RCC_APBENR1_TIM2EN;
	else if (tim == TIM3) RCC->APBENR1 |= RCC_APBENR1_TIM3EN;
	else if (tim == TIM14) RCC->APBENR2 |= RCC_APBENR2_TIM14EN;
	// Add more as needed

	// 2. Set prescaler and ARR
	tim->PSC = prescaler;
	tim->ARR = arr;
	tim->CNT = 0;

	// 3. Enable update interrupt
	tim->DIER |= TIM_DIER_UIE;

	// 4. Set NVIC
	NVIC_SetPriority(irqNum, priority);
	NVIC_EnableIRQ(irqNum);

	// 5. Start the timer
	tim->CR1 &= ~TIM_CR1_DIR; // Upcounting
	tim->CR1 |= TIM_CR1_CEN;
}

void InitAll(){

	// GPIO outputs
	ConfigureGpioOutput(LED1_PORT, LED1_PIN); // LED1
	ConfigureGpioOutput(LED2_PORT, LED2_PIN); // LED2
	ConfigureGpioOutput(LED3_PORT, LED3_PIN); // LED3

	// EXTI lines
	ConfigureButtonEXTI();

	// ADC
	ConfigureAdc(); //PA0 Potentiometer

	// TIM
	ConfigureTimer(TIM2, 16 - 1, 1000 - 1, TIM2_IRQn, 3); //All LEDs
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  InitAll();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  /* USER CODE BEGIN 3 */
	  	 HAL_ADC_Start(&hadc1); //for ADC potentiometer

	  	 // Read ADC value from PA0 (0-4095)
	  	 adc_value = HAL_ADC_GetValue(&hadc1);

	  	 /* Relevant Values
	  	  * PWM Period is 50Hz, 20ms with resolution of 2000 ticks.
	  	  * 20ms/20000 = 0.001ms = 1us per tick.
	  	  */

	  	//RILEY's NOTE, these DUTY CYCLE UPPER AND LOWER
	  	// are what exactly work for 180 degrees on my own SERVO, we don't need to alter them
	  	// if it's showing up weirdly on your own servo.

	  	 /* DUTY CYCLE LOWER: 0 DEGREES
	  	  * 0.61ms = 610us
	  	  * datasheet: 0.05 of PWM Period
	  	  * actual: ~0.03 of PWM Period
	  	  */
	  	 int32_t serv_min = 610;

	  	 /* DUTY CYCLE UPPER: 180 DEGREES
	  	  * 2.60ms = 2450us
	  	  * datasheet: 0.10 of PWM Period
	  	  * actual: ~0.13 of PWM Period for this servo
	  	  */
	  	 int32_t serv_max = 2600;

	  	 //POTENTIOMETER MIN & MAX
	  	 int32_t pot_min = 70;
	  	 int32_t pot_max = 4095;

	    	  /* ----------------------------
	    	  *  ACTIVITY 3: MCU 3
	    	  *  Potentiometer to Control a Servo Motor
	    	  *  ----------------------------
	    	  */

	  	 pwm_val = MapLinear(adc_value,pot_min,pot_max,serv_min,serv_max);
	  	 //time for servo to react to new input
	  	 Potcounter++;

	  	 /* HAL_TIM_SET_COMPARE, setting CCR to the new PWM duty cycle value
	  	  * Set the TIM Capture Compare Register value on runtime
	  	  * without calling another time ConfigChannel function
	  	  */
	  	 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_val);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

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
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 499;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
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
  sConfigOC.Pulse = 0;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
