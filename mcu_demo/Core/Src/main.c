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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
int main(void){
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
  /* USER CODE BEGIN 2 */
  InitAll();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
    /* USER CODE END WHILE */

	/* Switch is the most ideal way to ensure that we remain
	 * in a clear state system. default state exists as error
	 * correction.
	 * - Riley
	 */

	 switch (currentState){

	 	 case STATE_A: HandleStateA(); break;

	 	 case STATE_B: HandleStateB(); break;

	 	 case STATE_C: HandleStateC(); break;

	 	 default:	currentState = STATE_A; break;
	 }

	/* USER CODE BEGIN 3 */
	/* USER CODE END 3 */
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
//TEST
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
