/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define __FPU_PRESENT             1U
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
//#include "arm_math.h"
//#include "arm_math.h"

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

CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */

static const uint8_t IMU_ADDR = 0x68 << 1;
static const uint8_t REG_ACCEL_H = 0x3B;
static const uint8_t REG_ACCEL_L = 0x3C;
static const uint8_t REG_POWER = 0x6B;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM9_Init(void);
static void MX_SPI2_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void delayMs( int delay);
void playSound(uint16_t periode, uint16_t volume, uint16_t cycles);
//void TIM8_IRQHandler(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
//void HAL_TIM_IRQHandler(TIM_HandleTypeDef *htim);
//void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *htim);
void calc_lookup(float *lookup);
void myDelay(void);
void DMAUSARTTransferComplete(DMA_HandleTypeDef *hdma);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// --- MOTOR SPECIFIC PARAMETERS
float phase0 = 0.6; // angle motor winding A to encoder 0 [radians of electrical phase]

// --- SYSTEM SPECIFIC PARAMETERS

int pwm = 2048;

// --- CONTROL PARAMETERS

float amp = 0.05;  // amp
int run_motor = 1;
int direction = 1;
float phase_shift = PI/2;

float stiffness = 0;

// --- INITIALIZE OTHER GLOBALS
int16_t EncVal;
int16_t last_EncVal;
float phase = 0;
int int_phase = 0;
float velocity = 0;
float av_velocity = 0;
int pwmA = 0;
int pwmB = 0;
int pwmC = 0;

int skip_update = 0;
int skip_update_high_v = 0;

//uint16_t tim12_counter = 5;
uint32_t tim12_counter = 4000000000;

// --- LOOKUPS

float lookup[210];




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//ls /dev/tty*
	//screen /dev/tty.usbmodem14203 115200          --- stop: control a \

	//Drivers/CMSIS/DSP/Include
	//ARM_MATH_CM4



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
  MX_TIM9_Init();
  MX_SPI2_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_TIM8_Init();
  MX_TIM13_Init();
  MX_TIM12_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  calc_lookup(lookup);

	uint8_t buf[300];
	uint8_t plot[300];

	char ch='q';
	HAL_StatusTypeDef ret;

	int16_t accel16;
	uint8_t accel8l;
	uint8_t accel8h;

  // --- SET STATUS LEDS
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
  HAL_TIM_OC_Start(&htim9, TIM_CHANNEL_2);

  HAL_TIM_OC_Start(&htim12, TIM_CHANNEL_1);
  HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1);

  // --- ENABLE DRV
  HAL_GPIO_WritePin(EN_GATE_GPIO_Port, EN_GATE_Pin, 1);


  SCB->CPACR |= 0xf00000;

  int i=0;
	uint32_t i_fast = 0;
	uint32_t i_slow = 0;
	uint32_t fast2slow = 2000;

	int blink_duration = 100;



  //  buf[0] = 0x6B;
  //  HAL_I2C_Master_Transmit(&hi2c2, IMU_ADDR, buf, 1, HAL_MAX_DELAY);
  //  HAL_I2C_Master_Transmit(&hi2c2, IMU_ADDR, 0x00, 1, HAL_MAX_DELAY);
  //  HAL_Delay(2);


	// --- MOTOR DRIVER ----------------------------------------------------
	//EN_GATE
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);

	playSound( 3, 100, 20);

	HAL_GPIO_WritePin(EN_GATE_GPIO_Port, EN_GATE_Pin, GPIO_PIN_SET);

	// --- ADC --------------------------------------
	ADC_ChannelConfTypeDef adcChannel;

	//adcChannel.Channel = ADC_CHANNEL_8;
	adcChannel.Rank = 1;
	adcChannel.SamplingTime = ADC_SAMPLETIME_15CYCLES;//5mus //ADC_SAMPLETIME_480CYCLES;// 20mus
	adcChannel.Offset = 0;
	//HAL_ADC_ConfigChannel(&hadc2, &adcChannel);



	uint32_t g_ADCValue8;
	uint32_t g_ADCValue14;
	uint32_t g_ADCValue15;
	int g_MeasurementNumber;

	//see https://visualgdb.com/tutorials/arm/stm32/adc/
	//uint32_t a_val;
	//a_val = HAL_ADC_GetValue(&hadc2)
	//HAL_ADC_Start(&hadc2);


	// --- I2C2 IMU ------------------------------------------------
	//see: https://www.youtube.com/watch?v=isOekyygpR8
	//b1101000
	char accel_char[20];

	buf[0] = 0x6B; //power register
	buf[1] = 0x00; //switch on
	ret = HAL_I2C_Master_Transmit(&hi2c2, IMU_ADDR, buf, 2, HAL_MAX_DELAY);
	if (ret != HAL_OK){
		strcpy((char*)buf, "Error IMU T\r\n");
	} else {
		buf[0] = 0x00;
	}

	buf[0] = 0x3B;
	ret = HAL_I2C_Master_Transmit(&hi2c2, IMU_ADDR, buf, 1, HAL_MAX_DELAY);
	if (ret != HAL_OK){
		strcpy((char*)buf, "Error IMU T\r\n");
	} else {
		ret = HAL_I2C_Master_Receive(&hi2c2, IMU_ADDR, buf, 1, HAL_MAX_DELAY);
		if (ret != HAL_OK){
			strcpy((char*)buf, "Error IMU R\r\n");
		} else {
			accel8l = (int8_t)buf[0];
			sprintf((char*)accel_char, "%u m\r\n", (int)accel8l);
			//itoa(buf[0], accel_char, 10);
		}

	}

  //  	buf[0] = 0x42;
  //		ret = HAL_I2C_Master_Transmit(&hi2c2, IMU_ADDR, buf, 1, HAL_MAX_DELAY);
  //		if (ret != HAL_OK){
  //			strcpy((char*)buf, "Error IMU T\r\n");
  //		} else {
  //			ret = HAL_I2C_Master_Receive(&hi2c2, IMU_ADDR, buf, 1, HAL_MAX_DELAY);
  //			if (ret != HAL_OK){
  //				strcpy((char*)buf, "Error IMU R\r\n");
  //			} else {
  //				accel8l = (int8_t)buf[0];
  //				//sprintf((char*)buf, "%u m\r\n", (int)accel8l);
  //				//itoa(buf[0], accel_char, 10);
  //			}
  //
  //		}
  //
  //		//who am i WORKS
  //
  //		buf[0] = 0x75;
  //				ret = HAL_I2C_Master_Transmit(&hi2c2, IMU_ADDR, buf, 1, HAL_MAX_DELAY);
  //				if (ret != HAL_OK){
  //					strcpy((char*)buf, "Error IMU T\r\n");
  //				} else {
  //					ret = HAL_I2C_Master_Receive(&hi2c2, IMU_ADDR, buf, 1, HAL_MAX_DELAY);
  //					if (ret != HAL_OK){
  //						strcpy((char*)buf, "Error IMU R\r\n");
  //					} else {
  //						accel8l = (int8_t)buf[0];
  //						//sprintf((char*)buf, "%u m\r\n", (int)accel8l);
  //						//itoa(buf[0], accel_char, 10);
  //					}
  //
  //				}

	// --- TIMERS ----------------------------------------------------
	TIM9->CCR1 = blink_duration;
	TIM9->CCR2 = blink_duration;

	// --- GPIO ----------------------------------------------------

	GPIOE->BSRR = GPIO_PIN_4; //switches LD2




	playSound( 2, 100, 40);
	playSound( 1, 100, 80);
	HAL_Delay(100); // So the system stops vibrating






	// --- ROTATION SENSOR INIT ----------------------------------------------------
	HAL_TIM_Encoder_Start_IT(&htim8, TIM_CHANNEL_ALL );

	// --- ROTATION SENSOR SETTINGS ----------------------------------------------------

	//ROT0_nCS_GPIO_Port->BSRR = (uint32_t)ROT0_nCS_Pin << 16U;

	uint16_t address = 0x0000;
	uint16_t value = 0x0000;

	// --- set ABI and enable PWM
	//TODO: Error handling
	address = AS_ADDR_SETTINGS1 | AS_WRITE ;
	value = 0x0080 | AS_ODD;
	HAL_GPIO_WritePin(ROT0_nCS_GPIO_Port, ROT0_nCS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, (uint16_t *)&address, 1, 1);
	HAL_GPIO_WritePin(ROT0_nCS_GPIO_Port, ROT0_nCS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ROT0_nCS_GPIO_Port, ROT0_nCS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, (uint16_t *)&value, 1, 1);
	HAL_GPIO_WritePin(ROT0_nCS_GPIO_Port, ROT0_nCS_Pin, GPIO_PIN_SET);

	// --- set steps 2000steps 500 pulses
	address = AS_ADDR_SETTINGS2 | AS_WRITE ;
	value = 0x0020 | AS_ODD;
	HAL_GPIO_WritePin(ROT0_nCS_GPIO_Port, ROT0_nCS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, (uint16_t *)&address, 1, 1);
	HAL_GPIO_WritePin(ROT0_nCS_GPIO_Port, ROT0_nCS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ROT0_nCS_GPIO_Port, ROT0_nCS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, (uint16_t *)&value, 1, 1);
	HAL_GPIO_WritePin(ROT0_nCS_GPIO_Port, ROT0_nCS_Pin, GPIO_PIN_SET);

	// --- read angle
	address = 0x3FFE | AS_READ ;
	uint16_t angle = 0x0000;
	HAL_GPIO_WritePin(ROT0_nCS_GPIO_Port, ROT0_nCS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, (uint16_t *)&address, 1, 1);//this is the value
	HAL_GPIO_WritePin(ROT0_nCS_GPIO_Port, ROT0_nCS_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(ROT0_nCS_GPIO_Port, ROT0_nCS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Receive(&hspi2, (uint16_t *)&angle, 1, 1);
	HAL_GPIO_WritePin(ROT0_nCS_GPIO_Port, ROT0_nCS_Pin, GPIO_PIN_SET);


	// --- ROTATION SENSOR 0 POINT SETTING ----------------------------------------------------
	angle &= AS_DATA_MASK;
	EncVal = (uint16_t) ((float)angle /16384.0 * 2000.0);
	last_EncVal = EncVal;
	TIM8->CNT = EncVal;


	// --- Calibrate phase0   ------ TODO also this requires that we are not yet sensitive to value change
//	TIM1->CCR1 = 50;
//	HAL_Delay(500);
//	EncVal = TIM8->CNT;//takes 200ns
//	TIM1->CCR1 = 0;
//	phase = (float) EncVal * 0.02199 ;

	HAL_DMA_RegisterCallback(&hdma_usart3_tx, HAL_DMA_XFER_CPLT_CB_ID, &DMAUSARTTransferComplete);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */




  while (1)
  {
  	// -------------------------------------------------------------
		// --- FAST PROCESS ----------------------------------------------------
		// -------------------------------------------------------------
  	//HAL_Delay(1);
  	//debug2_out_GPIO_Port->BSRR = debug2_out_Pin; //takes 60ns == 5 clock cycles
  	//debug2_out_GPIO_Port->BSRR = (uint32_t)debug2_out_Pin << 16U;

  	debug1_out_GPIO_Port->BSRR = debug1_out_Pin; //takes 60ns == 5 clock cycles
  	debug1_out_GPIO_Port->BSRR = (uint32_t)debug1_out_Pin << 16U;
  	debug1_out_GPIO_Port->BSRR = debug1_out_Pin; //takes 60ns == 5 clock cycles
		debug1_out_GPIO_Port->BSRR = (uint32_t)debug1_out_Pin << 16U;


		// 3measurements take 25mus --- one just 5mus --- 7 take 50mus

		// --- ADC MEASUREMENTS
		uint32_t g_ADCValue1_4=5;
		uint32_t g_ADCValue1_5=5;

		// --- VBUS
		adcChannel.Channel = ADC_CHANNEL_8;
		HAL_ADC_ConfigChannel(&hadc2, &adcChannel);
		HAL_ADC_Start(&hadc2);
	  if (HAL_ADC_PollForConversion(&hadc2, 1) == HAL_OK)
		{
				g_ADCValue8 = HAL_ADC_GetValue(&hadc2);
				g_MeasurementNumber++;
		}//takes several microseconds

	  // --- STRAIN
	  adcChannel.Channel = ADC_CHANNEL_14;
		HAL_ADC_ConfigChannel(&hadc2, &adcChannel);
		HAL_ADC_Start(&hadc2);
		if (HAL_ADC_PollForConversion(&hadc2, 1) == HAL_OK)
		{
				g_ADCValue14 = HAL_ADC_GetValue(&hadc2);
				g_MeasurementNumber++;
		}//takes several microseconds

		// --- STRAIN
		adcChannel.Channel = ADC_CHANNEL_15;
		HAL_ADC_ConfigChannel(&hadc2, &adcChannel);
		HAL_ADC_Start(&hadc2);
		if (HAL_ADC_PollForConversion(&hadc2, 1) == HAL_OK){
				g_ADCValue15 = HAL_ADC_GetValue(&hadc2);
		}

		uint32_t g_ADCValue1_1=5;
		adcChannel.Channel = ADC_CHANNEL_1;
		HAL_ADC_ConfigChannel(&hadc1, &adcChannel);
		HAL_ADC_Start(&hadc1);
		if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK){
				g_ADCValue1_1 = HAL_ADC_GetValue(&hadc1);
		}

		uint32_t g_ADCValue1_2=5;
		adcChannel.Channel = ADC_CHANNEL_2;
		HAL_ADC_ConfigChannel(&hadc1, &adcChannel);
		HAL_ADC_Start(&hadc1);
		if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK){
				g_ADCValue1_2 = HAL_ADC_GetValue(&hadc1);
		}

		// --- MOTOR TEMP 105 is room temp rising

		adcChannel.Channel = ADC_CHANNEL_4;
		HAL_ADC_ConfigChannel(&hadc1, &adcChannel);
		HAL_ADC_Start(&hadc1);
		if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK){
				g_ADCValue1_4 = HAL_ADC_GetValue(&hadc1);
		}

		// --- BOARD TEMP 1280 is room temp rising
		adcChannel.Channel = ADC_CHANNEL_5;
		HAL_ADC_ConfigChannel(&hadc1, &adcChannel);
		HAL_ADC_Start(&hadc1);
		if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK){
				g_ADCValue1_5 = HAL_ADC_GetValue(&hadc1);
		}

//
//
//
//	  HAL_TIM_IC_CaptureCallback(&htim8);// TODO thisis ahack so it get's going eventually when at standstill

//
//  	TIM1->CCR1 = 0;
//  	TIM1->CCR2 = 0;
//  	TIM1->CCR3 = 20;



	  // -------------------------------------------------------------
	  // --- SLOW PROCESS ----------------------------------------------------
	  // -------------------------------------------------------------

	  if (i_fast%fast2slow == 0){

	  	// --- GPIO ----------------------------------------------------
	  	HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_4);

			// --- UART ----------------------------------------------------


			HAL_UART_Receive_IT(&huart3, (uint8_t *)&ch, 1);


			switch(ch){
				case 'w':
					amp *= 2;
					break;
				case 's':
					amp /= 2;
					break;
				case 'a':
					phase_shift += 0.05;
					break;
				case 'd':
					phase_shift -= 0.05;
					break;
				case 't':
					run_motor = 1;
					break;
				case 'g':
					run_motor = 0;
					break;
				case 'h':
					direction = 1;
					break;
				case 'f':
					direction = -1;
					break;
				case 'r':
					direction *= -1;
					break;
				case 'p':
					playSound( 1, 20, 100);
					break;
				case 'u':
					stiffness += 0.001;
					break;
				case 'j':
					stiffness -= 0.001;
					break;
				default:
					ch='q';
			}

			//                   0---------1---------2---------3---------4---------5---------6---------7---------8---------9---------0---------1---------2---------3---------4---------5---------6---------7---------8---------9---------0---------1---------2---------3---------4---------5
			sprintf((char*)buf, "%c#%d %d A %d %d T %d %d                                                                                                                               \r\n",
					ch, (int)(amp*100), (int)(phase_shift*100),
					g_ADCValue1_4, g_ADCValue1_5,
					(int)(stiffness*1000), (int)(1000*av_velocity));

			buf[150] = '|';
			buf[100] = '.';
			buf[50] = '|';
			buf[100 + max(-50, min(50, (int)av_velocity))] = 'v';

			//HAL_UART_Transmit_IT(&huart3, buf, strlen((char*)buf)); //WORKS but replaced by DMA below
			huart3.Instance->CR3 |= USART_CR3_DMAT; //enabel dma as we disable in callback so uart can be used for something else
			HAL_DMA_Start_IT(&hdma_usart3_tx, (uint32_t)buf, (uint32_t)&huart3.Instance->DR, strlen(buf));

			ch='q';

			i_slow++;
	  }

	  i_fast++;
	  //HAL_Delay(5);

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
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
  sConfig.Channel = ADC_CHANNEL_2;
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
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
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
  sConfig.Channel = ADC_CHANNEL_8;
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
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  htim1.Init.Prescaler = 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2048;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  htim2.Init.Period = 4294967295;
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
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  //HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn); // this didn't seem to be necessary


  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 2;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 4096;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 2000;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 4;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 65535;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 0;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 0;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim13, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LD_1_Pin|LD_2_Pin|EN_GATE_Pin|M0_DC_CAL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11|debug1_out_Pin|debug2_out_Pin|ROT0_nCS_Pin 
                          |nSCS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD_1_Pin LD_2_Pin EN_GATE_Pin M0_DC_CAL_Pin */
  GPIO_InitStruct.Pin = LD_1_Pin|LD_2_Pin|EN_GATE_Pin|M0_DC_CAL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD11 debug1_out_Pin debug2_out_Pin ROT0_nCS_Pin 
                           nSCS_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_11|debug1_out_Pin|debug2_out_Pin|ROT0_nCS_Pin 
                          |nSCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : debug1_in_Pin */
  GPIO_InitStruct.Pin = debug1_in_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(debug1_in_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ROT0_I_W_Pin */
  GPIO_InitStruct.Pin = ROT0_I_W_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ROT0_I_W_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PWRGD_Pin nOCTW_Pin nFAULT_Pin */
  GPIO_InitStruct.Pin = PWRGD_Pin|nOCTW_Pin|nFAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void delayMs(int delay){
  int i;
  for(;delay>0; delay--){
    //for (i=0; i<3195; i++);
  	for (i=0; i<1; i++);
  }
}
// --- INTERRUPT  ----------------------------------------------------
// --- this was used to try interrupt - now the ROT0_A_U_Pin is configured for rot sensor
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
//	if (GPIO_Pin == ROT0_A_U_Pin){
//		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_3);
//	}
//	else{
//		__NOP();
//	}
//}

void myDelay(void){
	HAL_Delay(1);

}

void playSound(uint16_t periode, uint16_t volume, uint16_t cycles){
	// TODO disable interrupt for the duration of sound
	//HAL_NVIC_DisableIRQ(TIM8_UP_TIM13_IRQn);
	//HAL_Delay(1000);

	for (uint16_t i=0; i<cycles; i++){
		TIM1->CCR1 = 0; //takes<150ns
		TIM1->CCR2 = volume; //takes<150ns
		HAL_Delay(periode);
		TIM1->CCR1 = volume; //takes<150ns
		TIM1->CCR2 = 0; //takes<150ns
		HAL_Delay(periode);
	}

	//HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if (GPIO_Pin == ROT0_I_W_Pin){
		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_3);
		TIM8->CNT = 0;     //TODO: there is acually some stepping happening under the I pulse so we have to distinguish between step from right and step from left
	}
	else{
		__NOP();
	}
}



//this is it
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	// see https://community.st.com/s/question/0D50X00009XkWUpSAN/encoder-mode-and-rotary-encoder

	//debug2_out_GPIO_Port->BSRR = debug2_out_Pin; //takes 60ns == 5 clock cycles
	//debug2_out_GPIO_Port->BSRR = (uint32_t)debug2_out_Pin << 16U;
	//HAL_GPIO_TogglePin(debug2_out_GPIO_Port, debug2_out_Pin);
	if(htim->Instance == TIM8){

		if (skip_update){ //TODO somehow the Callback is triggered at this strange 25% duty cycle so we just look at every second update to get a constant frequency
			skip_update = 0;
		}
		else{

			skip_update = 1;

			if (abs(av_velocity) > 5 &&  skip_update_high_v == 1){
				skip_update_high_v = 0;
			}
			else {

				debug1_out_GPIO_Port->BSRR = debug1_out_Pin; //takes 60ns == 5 clock cycles

				skip_update_high_v = 1;



				EncVal = TIM8->CNT;//takes 200ns

				//tim12_counter = TIM12->CNT;
				tim12_counter = TIM2->CNT;
				if (tim12_counter > 10000){ // TODO fix the issue that this gets almost never called when velocity is super low.
					//TIM12->CNT = 0;
					TIM2->CNT = 0;
					int EncDiff = EncVal-last_EncVal;
					if (EncDiff > 1000){ // if jump is more than a half rotation it's most likely the 0 crossing
						EncDiff -= 2000;
					}
					else if (EncDiff < -1000){
						EncDiff += 2000;
					}
					velocity = (float)(EncDiff) / (float)tim12_counter;
					velocity *= 10500; // /2000 steps/rotation / 21000000 counts/sec  //TODO velocity seems too high by factor of 2 or 3 maybe same clock frequency issue that we actually run at 42 MHz. !!! TODO check clock frequency  // TODO divided by 10 as well
					av_velocity = 0.95 * av_velocity + 0.05 * velocity;
					last_EncVal = EncVal;
				}



				//HAL_GPIO_TogglePin(debug2_out_GPIO_Port, debug2_out_Pin);
				//debug2_out_GPIO_Port->BSRR = debug2_out_Pin; //takes 60ns == 5 clock cycles
				//debug2_out_GPIO_Port->BSRR = (uint32_t)debug2_out_Pin << 16U;
				// --- get angle from encoder 0...2000



				phase = (float) EncVal * 0.02199 ; //(float) EncVal / 2000.0 * 2*PI * 7 ; //takes 1500ns
				phase -= phase0;




				float u0 = 0.5773; //0.5 * 2.0 / 1.73205;// maximal possible U on one coil thanks to wankel //takes<200ns
				float modified_amp = amp + stiffness * av_velocity * direction; // TODO the abs allows same stiffness to make it softer for both directions - without a signchange is needed BUT turnaround is super aggressive now :( SAME issue with direction - super forceful reverse but sign identical --- looks like v needs to direct also the phase !!!!
				//u0 *= amp;  //takes<200ns
				u0 *= modified_amp;  //takes<200ns
				u0 *= run_motor;  //takes<200ns

				if (direction == 1){
					phase -= phase_shift;  //takes<200ns
				}
				else {
					phase += phase_shift;
				}

	//



				phase *= 100;
				int_phase = (int) phase;
				int_phase = int_phase % 628;
				if (int_phase < 0) {
					int_phase += 628;
				}

				float uA = 0;
				float uB = 0;
				float uC = 0;


		//
		//    uA = lookup[1]; //takes<32000ns !!!!!!!!!!!!!! with the fast implement it's just 2000ns !!!!!
		//    			uB = lookup[2]; // takes 3mus
		//    			uC = 0;

				// ---- lookup  this optimized routine brings roundtrip down to 5mus

				if  (int_phase < 210)	{ //0...209
					uA = lookup[int_phase]; //takes<32000ns !!!!!!!!!!!!!! with the fast implement it's just 2000ns !!!!!
					uB = lookup[210 - 1 - int_phase]; // takes 3mus
					uC = 0;
				}
			 else if  (int_phase < 420){	 //210...419
					uA = 0; //takes<32000ns !!!!!!!!!!!!!! with the fast implement it's just 2000ns !!!!!
					uB = lookup[int_phase - 210]; // takes 3mus
					uC = lookup[420 - 1 - int_phase];
			 }
			 else	{  //420...629
					uA = lookup[630 - 1 - int_phase]; //takes<32000ns !!!!!!!!!!!!!! with the fast implement it's just 2000ns !!!!!
					uB = 0; // takes 3mus
					uC = lookup[int_phase - 420];
				}


				pwmA = (uint16_t) (pwm * u0 * uA); //takes<2s00ns
				pwmB = (uint16_t) (pwm * u0 * uB); //takes<200ns
				pwmC = (uint16_t) (pwm * u0 * uC); //takes<200ns

				// ---- end lookup

				debug1_out_GPIO_Port->BSRR = (uint32_t)debug1_out_Pin << 16U;

				// --- MOTOR DRIVER ----------------------------------------------------
				// --- PWM pulses 0...2048
				TIM1->CCR1 = pwmA; //takes<150ns
				TIM1->CCR2 = pwmB; //takes<150ns
				TIM1->CCR3 = pwmC; //takes<150ns
			}
		}
	}


	//counterISR++;

}

void calc_lookup(float *lookup){
	// TODO plug in a higher order harmonic and see if system gets more energy efficient or more silent
	for (int i=0; i<210; i++){
	    lookup[i] = cos((float)i/100.0) + cos((float)i/100.0-1.047);
	}
}

void DMAUSARTTransferComplete(DMA_HandleTypeDef *hdma){
	huart3.Instance->CR3 &= ~USART_CR3_DMAT;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
