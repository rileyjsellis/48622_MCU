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

//errors were reached without these, they are used within lcd and uart functionality.
#include <stdio.h>
#include <string.h>
#include "i2c_lcd.h"
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

typedef enum {
    PC4_MODE_UNKNOWN,
    PC4_MODE_UART,
    PC4_MODE_GPIO
} Pc4Mode;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* *************
 *  USART (TX/RX)
 * *************
 */

// TX PA_2 (USART2_TX)
// RX PA_15 (USART2_RX)

/* *************
 *  LCD (I2C)
 * *************
 */
// SDA = PB_7
// SCL = PB_6
#define LCD_REFRESH_PERIOD 50 //ms

/* *********
 *  ALL LEDs
 * *********
 */
#define LED1_PORT GPIOB
#define LED1_PIN 4 //CN9_D5
#define LED2_PORT GPIOB
#define LED2_PIN 5 //CN9_D4
#define LED3_PORT GPIOB
#define LED3_PIN 3 //CN9_D3
#define LED4_PORT GPIOB
#define LED4_PIN 2 //CN10, in alignment with CN9_D8

/* *********
 *  BUTTONS
 * *********
 */
#define BUTTON1_PIN 4 //OFFBOARD PC_4, CN9_D1, CN10_ aligned with CN9's TX/D1
#define BUTTON2_PIN	13 //ONBOARD, PC_13, CN7 aligned with gap in CN8 and CN6
#define BUTTON1 0
#define BUTTON2 1

/* *********
 *  TIMERS
 * *********/
#define TIMER_SEL TIM2
#define TIMER_IRQn TIM2_IRQn

/* *********
 *  ADC POT
 * *********/
#define POT_PORT GPIOA
#define POT_PIN 0

#define ADC_AVG_WINDOW 30  // Average the last 10 values only
#define POT_ADC_MAX 4095
#define POT_ADC_MIN 80 //Potentiometer doesn't reach zero, so this is for linear mapping.

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
I2C_LCD_HandleTypeDef lcd1;

volatile SystemState currentState = STATE_A;

volatile uint32_t ledBitPos[2] = {LED1_PIN,LED2_PIN};

volatile uint32_t counter[2] = {0};

volatile uint32_t debugCounter[4] = {0};
volatile uint32_t debugEXTIcr3 = 0;
volatile uint32_t lastDebounceTime = 0;

volatile uint32_t lastPressTime[2] = {0,0};
volatile uint32_t lastReleaseTime[2] = {0,0};
const uint32_t debounceDelay = 30; //ms
volatile uint32_t buttonState[2]= {0};

volatile uint32_t selectedLed = 0;
volatile uint32_t ledOn = 0;
volatile uint32_t ledOff = 1;

volatile uint32_t blink = 0;

volatile uint32_t adcValue = 0;

volatile uint32_t cState = 0;       // 0 = OFF, 1 = ON
volatile uint32_t ledBlinkCounter = 0;  // Count OFF states (each full flash = 1)
volatile uint32_t ledDone = 0;        // Flag when 3 flashes complete

//for code review
volatile int32_t activity = 3; //start on 3, can edit in live expressions
volatile int32_t Potcounter = 0;
volatile int32_t hal_delay = 1;

//potentiometer

volatile int32_t latestAdcValue = 0;
volatile int32_t adcNewValueFlag = 0;
volatile uint32_t adcHistory[ADC_AVG_WINDOW] = {0};  // Stores the last 10 readings
volatile uint8_t adcHistoryIndex = 0;  // Tracks where to insert the new value

volatile uint32_t variableBlinkingFrequency = 0; //for led3
volatile uint32_t led3counter = 0;

//for servo, 47 as minimum is explained below
volatile int32_t pwm_val = 470; //for activity 2 sweeping!
volatile int32_t direction = 0; // to be treated as boolean

volatile int32_t counter_test = 0;


/**************************
 *** ALL UART VARIABLES ***
 **************************/
volatile uint32_t uartEnabled = 1;
volatile Pc4Mode pc4CurrentMode = PC4_MODE_UNKNOWN;

//transmit data variable
volatile char txData [] = "Autumn2025 MX1 SID: 14057208, ADC Reading: XXXX\r\n";
volatile char rxData [] = "";

volatile int32_t isTransmitting = 1;
volatile int32_t uartCounter = 0;
volatile int8_t uartTransmitFlag = 0;

/*************************
 *** ALL LCD VARIABLES ***
 *************************/

volatile int8_t lcdUpdateFlag = 0;
volatile int32_t lcdCounter = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
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

void ConfigureTxForUart(void);
void ConfigureTxForGpio(void);

uint32_t LinearMap(uint32_t x, uint32_t inMin, uint32_t inMax,
		uint32_t outMin, uint32_t outMax);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ****************************
 * ******* BUTTON LOGIC *******
 * ****************************/

// This function is called when button PRESS matters,
// not button being HELD.
// For swapping between states.
uint8_t ButtonWasPressed(uint8_t buttonIndex){
	if (buttonState[buttonIndex]) {
		buttonState[buttonIndex] = 0; //clears press flag
		return 1; //returns TRUE, recent button press HAS occurred.
	}
	return 0; // No new press
}

void handleButtonInterrupt(uint8_t button, uint8_t isFalling, uint32_t pin){
    if (isFalling) {
        buttonState[button] = 1;  // Button is now pressed
        EXTI->FPR1 = (1 << pin);   // Clear falling edge flag
    } else {
        buttonState[button] = 0;  // Button is now released
        EXTI->RPR1 = (1 << pin);   // Clear rising edge flag
    }
}

void EXTI4_15_IRQHandler(void){

    if (EXTI->FPR1 & (1 << 13)) handleButtonInterrupt(BUTTON2, 1, 13);
    if (EXTI->RPR1 & (1 << 13)) handleButtonInterrupt(BUTTON2, 0, 13);

    if (EXTI->FPR1 & (1 << 4)) handleButtonInterrupt(BUTTON1, 1, 4);
    if (EXTI->RPR1 & (1 << 4)) handleButtonInterrupt(BUTTON1, 0, 4);
}

/* ****************************
 * ******* USART LOGIC ********
 * ****************************/

void TransmitWithADC(void){
	char messageWithAdc [100];
	if (!uartTransmitFlag) return; //immediate exit if no transmission

	sprintf(messageWithAdc, "Autumn2025 EMS SID: 14057208, ADC Reading: %lu\r\n", adcValue);

	if(HAL_UART_Transmit(&huart2, (uint8_t*)messageWithAdc, strlen(messageWithAdc), HAL_MAX_DELAY) != HAL_OK)
		 Error_Handler();
	//Riley's note:
	// 'uint8_t*) included to remove warning. signedness differed.
	// removed warning without changing variable type.

	uartTransmitFlag = 0; //clear flag after successful transmit

}

void ConfigureTxForUart(void) {
    if (pc4CurrentMode == PC4_MODE_UART) return; // already UART, skip

    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_4);  // Release PC4 from any previous GPIO mode

    MX_USART2_UART_Init(); // CubeMX-generated USART1 initializer (re-init UART and PC4 alternate function)

    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE); // Enable RX interrupt

    pc4CurrentMode = PC4_MODE_UART;
}

void ConfigureTxForGpio(void) {
    if (pc4CurrentMode == PC4_MODE_GPIO) return; // already GPIO, skip

    HAL_UART_DeInit(&huart2);  // Disable UART, free PC4

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    pc4CurrentMode = PC4_MODE_GPIO;
}

/* **************************
 * ******* LCD LOGIC ********
 * **************************/

void OutputLCD(const char* line1, const char* line2) {
	if(!lcdUpdateFlag)return; //prevents endless refresh
    lcd_clear(&lcd1);

    lcd_gotoxy(&lcd1, 0, 0);
    lcd_puts(&lcd1, line1);

    lcd_gotoxy(&lcd1, 0, 1);
    lcd_puts(&lcd1, line2);
    lcdUpdateFlag = 0;
}

/* ****************************
 * ***** STATE MACHINE ********
 * ****************************/

void HandleStateA(void) {

	// UART Transmitting
	ConfigureTxForUart(); //UART ON
	TransmitWithADC();

	// LCD STATE A
	OutputLCD("SID: 14057208", "MECHATRONICS 1");

	// UART Receiving (only in State A)
	HAL_UART_Receive_IT(&huart2, rxData, sizeof(rxData));
	// UART Transmitting
	TransmitWithADC();

    if (ButtonWasPressed(BUTTON1)) {
		currentState = STATE_C;
		debugCounter[2];
		return;
	}
    if (ButtonWasPressed(BUTTON2)) {
        currentState = STATE_B;
        counter[0] = 0;
        return;
    }
}

void HandleStateB(void) {


	ServoMove();

	//LCD ALL FUNCTIONALITY
	char adcText[17];
	snprintf(adcText, sizeof(adcText), "ADC:%4lu STATE B", adcValue);

	OutputLCD(adcText, "MECHATRONICS 1");

    if (buttonState[BUTTON1]) {
        selectedLed = 1; //LED2 blinks while button is held
    }
    else{
    	selectedLed = 0; //LED1 blinks otherwise
    }
    if (ButtonWasPressed(BUTTON2)) {
        currentState = STATE_A;
        return;
    }
}

void HandleStateC(void) {
	 static uint8_t started = 0;

	 ConfigureTxForGpio(); //UART OFF

	 OutputLCD("", ""); //LCD

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

/* ****************************
 * ***** SERVO MOVING  ********
 * ****************************/

void ServoMove(void){

	//HAVE A FLAG FOR TIMING HOW OFTEN THE SERVO IS ALLOWED TO RECIEVE A NEW COMMAND

	//RILEY's NOTE, these DUTY CYCLE UPPER AND LOWER
	// are what exactly work for 180 degrees on my own SERVO, we don't need to alter them
	// if it's showing up weirdly on your own servo.

	 // SERVO PULSE WIDTH LIMITS (tested)
	 const int32_t serv_min = 610; // 0 degrees, ~0.61ms
	 const int32_t serv_max = 2600; // 180 degrees, ~2.6ms

	 pwm_val = LinearMap(adcValue,POT_ADC_MAX,0,serv_min,serv_max);

	 // Safety if values end up beyond
	 if (pwm_val < serv_min) pwm_val = serv_min;
	 if (pwm_val > serv_max) pwm_val = serv_max;

	 //time for servo to react to new input
	 Potcounter++;

	 /* HAL_TIM_SET_COMPARE, setting CCR to the new PWM duty cycle value
	  * Set the TIM Capture Compare Register value on runtime
	  * without calling another time ConfigChannel function
	  */
	 //RILEY's DEBUG, this needs to be called (along with most of WHILE LOOP) in TIM3 Handler for servo functionality.
	 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_val);

}
/*LINEAR MAPPING
 * this exists for the input ADC value to be mapped to the
 * minimum and maximum of the servo motor PWM, 180 degrees rotation.
 */
uint32_t LinearMap(uint32_t x, uint32_t inMin, uint32_t inMax, uint32_t outMin, uint32_t outMax){
    if (inMax == inMin) return outMin; // prevent divide-by-zero

    if (x < inMin) x = inMin;
    if (x > inMax) x = inMax;

    //uint64_t avoids overflow during multiplication
    return ((uint64_t)(x - inMin) * (outMax - outMin)) / (inMax - inMin) + outMin;
}

void TIM2_IRQHandler(void){
    TIM2->SR &= ~TIM_SR_UIF; // Clear update interrupt flag

    /* ***********************
	 * LCD Counter for Refresh
	 * ***********************
	 */
	lcdCounter++;
	if (lcdCounter > LCD_REFRESH_PERIOD -1){
		lcdUpdateFlag = 1;
		lcdCounter = 0;
	}

	/* *****************************
	 * UART Counter for Transmission
	 * ******************************/
	uartCounter++;
	if (uartCounter > 499 //specified output frequency 500ms.
			&& currentState == STATE_A
			&& isTransmitting){
		uartTransmitFlag = 1;
		uartCounter = 0;
	}

    static uint16_t period = 1000;

    // --- LED1/2 handling (State B) ---
    if (currentState == STATE_B) {

    	ledOn = selectedLed;
		ledOff = !ledOff;


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


		// --- LED3 ADC BLINKING PERIOD

		variableBlinkingFrequency = LinearMap(adcValue,0,POT_ADC_MAX,200,1000);

		if(led3counter > variableBlinkingFrequency / 2){
			GPIOB->BSRR = (1 << LED3_PIN); //ON
		}
		if(led3counter < variableBlinkingFrequency / 2){
			GPIOB->BSRR = (1 << (LED3_PIN + 16)); //LED OFF
		}

		led3counter = (led3counter + 1) % variableBlinkingFrequency;
    }
    else {
    	GPIOB->BSRR = (1 << (LED1_PIN + 16)); //LED blinks off
		GPIOB->BSRR = (1 << (LED2_PIN + 16)); //LED blinks off
		GPIOB->BSRR = (1 << (LED3_PIN + 16)); //LED blinks off
    }

    // --- LED4 3x flash handling (State C) ---
    if (currentState == STATE_C) {
    	//Start of state
    	if (cState == 0) {
			LED4_PORT->BSRR = (1 << LED4_PIN); // ON
			counter[1] = 0;
			cState = 1;
		}

    	//Phase 1 - Blinking Logic
    	if (cState == 1){
    		if (counter[1] > period / 2 ){
				GPIOB->BSRR = (1 << LED4_PIN); //LED blinks on
			}
			if (counter[1] <= period / 2){
				GPIOB->BSRR = (1 << (LED4_PIN + 16)); //LED blinks off
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

/*
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
*/


/* *********************************
 * ***** POTENTIOMETER ADC  ********
 * *********************************/

void ADC1_COMP_IRQHandler(void){

	uint32_t localAdcValue = ADC1->DR; //take value
	adcNewValueFlag = 1;

	if (localAdcValue < POT_ADC_MIN){ //if at fluctuating low point, make minimum
		localAdcValue = POT_ADC_MIN;
	}

	latestAdcValue = localAdcValue;

	ADC1->ISR |= (1 << 2); //clear the interrupt flag

	ADC1->CR |= 1 << 2; //start new ADC conversion

}

void GetAdcValue(void){

	if (!adcNewValueFlag) return;
	adcNewValueFlag = 1;

	// Store latest value into the buffer at the current index
	adcHistory[adcHistoryIndex] = latestAdcValue;

	// Move to next index, wrapping around at the end (circular buffer)
	adcHistoryIndex = (adcHistoryIndex + 1) % ADC_AVG_WINDOW;

	// Calculate average of the 10 stored values
	uint32_t sum = 0;
	for(uint8_t i = 0; i < ADC_AVG_WINDOW; i++) {
		sum += adcHistory[i];
	}
	uint32_t averagedValue = sum / ADC_AVG_WINDOW;

	// Map averaged result to desired range
	adcValue = LinearMap(averagedValue,POT_ADC_MIN,POT_ADC_MAX,0,POT_ADC_MAX);


	//RILEY's NOTE, potential EXTRA FOR SERVO, a DEADBAND to stop servo jitter.

}

void InitAdcHistory(void) {
    // Fill the history buffer with the first valid ADC value
    for (uint8_t i = 0; i < ADC_AVG_WINDOW; i++) {
        adcHistory[i] = latestAdcValue;
    }
}

void configure_LCD(void){
	MX_I2C1_Init(); // CubeMX-generated

	lcd1.hi2c = &hi2c1;
	lcd1.address = 0x27 << 1;  // (confirm your backpack's address, usually 0x27 or 0x3F)
	lcd_init(&lcd1);
	lcd_clear(&lcd1);
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
	InitAdcHistory(); //sets average to current value

	// TIM
	ConfigureTimer(TIM2, 16 - 1, 1000 - 1, TIM2_IRQn, 3); //All LEDs
	//TIM3 for servo done in .ioc
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  InitAll();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){

	  GetAdcValue(); // Averaged out value.

	  switch (currentState){

	  	  case STATE_A: HandleStateA(); break;

	  	  case STATE_B: HandleStateB(); break;

	  	  case STATE_C: HandleStateC(); break;

	  	  default:	currentState = STATE_A; break;
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
  hi2c1.Init.Timing = 0x00503D58;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// This function has been moved to main.c
// Purpose is called when key press triggers interrupt.
// - Riley
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UART_RxCpltCallback can be implemented in the user file.
   */

  //Echoing user input for the purpose of debugging
  char toggleMessage [] = "UART output toggled with E key. \r\n";

  //strcasecmp means the key will activate for 'e' and 'E'
  // const char to remove warning as strcasecmp expects const char.
  // - Riley
  if (!strcasecmp((const char*)rxData, "e")){
      isTransmitting = !isTransmitting;
	  HAL_UART_Transmit(&huart2, (uint8_t*)toggleMessage, strlen(toggleMessage), HAL_MAX_DELAY);
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
