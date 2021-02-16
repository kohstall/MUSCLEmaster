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

//Big todo
// - reliable programming and rieadout of angle sensor
// - booting up from switch on without reset
// - reliable play button when writing code
// - analog channels work on one board only
// - increase encoder steps to get better resolution in v and a


/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


//todo
// - pressing p seems to pause entire program

#define __FPU_PRESENT             1U
#define USE_HAL_TIM_REGISTER_CALLBACKS 1
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
//#include "arm_math.h"
//#include "arm_math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DB_TIMING 0
#define FOC_ALG 1

#define PI 3.14159f
#define PI2 6.28318f

#define INV_SQRT_3 0.57735f //allowed great speed up!

//#define USE_HAL_TIM_REGISTER_CALLBACKS 1    // TODO work out other callback for encoder
#define ENC_STEPS 4000 // edge changes on A and B outputs - Note that ARR in TIM8 needs to be set to ENC_STEPS-1
#define ENC_STEPS_HALF 2000 // to be set equal to  ENC_STEPS / 2
#define ENC_RESOLUTION 16384 // 14 bit resolution for angle reading via SPI
#define ENC_TOLERANCE 2

#define N_PHASES 3

#define PWM_STEPS 4096
#define PWM_1PERCENT 41 // set this to 1% of the PWM_STEP
#define AMP_LIMIT 0.9f



#define BUF_LEN 400
#define BUF_ADD_LEN 200

#define DB1H debug1_out_GPIO_Port->BSRR = debug1_out_Pin
#define DB1L debug1_out_GPIO_Port->BSRR = debug1_out_Pin << 16U

#define ANALOG_SAMPLES_BITSHIFT 5
#define ANALOG_SAMPLES_N 32 // must be 2^ANALOG_SAMPLES_BITSHIFT

#define FAST_PER_SLOW 8

#define FOC_PHASE_LIM 0.3f

#define PWM_F 2048.0f



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc3;

CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
static const uint8_t IMU_ADDR = 0x68 << 1;
static const uint8_t REG_ACCEL_H = 0x3B;
static const uint8_t REG_ACCEL_L = 0x3C;
static const uint8_t REG_POWER = 0x6B;


CAN_HandleTypeDef hcan1;
CAN_TxHeaderTypeDef pHeader;
CAN_RxHeaderTypeDef pRxHeader;
uint32_t TxMailbox;
uint8_t tx_msg[6];
uint8_t rx_msg[4];
//uint8_t rx_character_armed = 0;
char rx_character_buffered = '.';
char rx_character = '.';
uint8_t rx_control_0 = 0 ;
uint16_t rx_control_1 = 0;
uint8_t rx_mode_0 = 0;
uint8_t rx_mode_1 = 0;
uint8_t rx_intent = 0;
//unsigned int r : 32;
CAN_FilterTypeDef sFilterConfig;

uint32_t can_pending_before = 22;
uint32_t can_pending_after = 22;


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
static void MX_ADC3_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
void playSound(uint32_t periode, uint32_t volume, uint32_t cycles);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_CaptureHalfCpltCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_TriggerHalfCpltCallback(TIM_HandleTypeDef *htim);
void calc_lookup(float *lookup);
void calc_sin_lookup(float *sin_lookup);
void calc_cos_lookup(float *cos_lookup);
void delay_SPI(void);
void DMAUSARTTransferComplete(DMA_HandleTypeDef *hdma);
void EncoderStepCallback(void);
void step_through_pole_angles(void);
void step_through_pwm_percent(void);
void explore_limits(void);
void set_pwm_off(void);
void calc_v(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void update_pwm(void);
void fast_control_task(void);
void slow_control_task(void);
void keyboard_intake(void);
void print_task(void);
void print_prep_task(int fast_control_task_counter);
void timing_party(void);




/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// --- MOTOR SPECIFIC PARAMETERS


////--ESC_ID = 0; // test
//float phase0 = 0.6f;  //$$$$$$$$$$$$$ SPECIFIC $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//#define  N_POLES 20 // 7(14 magnets, 12 coils) //20//(40 magnets, 36 coils) //$$$$$$$$$$$$$ SPECIFIC $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//#define CAN_ID 0x10
//#define INVERT 0

//--ESC_ID = 1; // hip pitch
//float phase0 = 0.0f;
//#define  N_POLES 21 //black motor
//#define CAN_ID 0x11
//#define INVERT 0

//--ESC_ID = 2; // knee
//float phase0 = 3.9f;//backcalc after correction: =3.5-->56=enc_val //angle_enc=53 for ABC = 0 - for 20 poles 2pi is 18degree=100angle_enc --> 53 is 1.06pi
//#define  N_POLES 20 //7(14 magnets, 12 coils) //21//(42 magnets, 36 coils) //$$$$$$$$$$$$$ SPECIFIC $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//#define CAN_ID 0x12
//#define INVERT 0

//--ESC_ID = 3; // ankle pitch
//float phase0 = 0.4f;  //$$$$$$$$$$$$$ SPECIFIC $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//#define  N_POLES 20 //7(14 magnets, 12 coils) //21//(42 magnets, 36 coils) //$$$$$$$$$$$$$ SPECIFIC $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//#define CAN_ID 0x13
//#define INVERT 0

//--ESC_ID = 100; // test rig for muscle mp with 5045
float phase0 = 1.2f;  //$$$$$$$$$$$$$ SPECIFIC $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
#define  N_POLES 7 //7(14 magnets, 12 coils) //21//(42 magnets, 36 coils) //$$$$$$$$$$$$$ SPECIFIC $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
#define CAN_ID 0x100
#define INVERT 0

//--ESC_ID = 100; // clavicle yaw
//float phase0 = -1.232f;  // set to at ABC=0 angle*2*pi/2000*N_poles=angle*0.0220 = (1944-2000)*0.022=-1.232//$$$$$$$$$$$$$ SPECIFIC $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//#define  N_POLES 7 //7(14 magnets, 12 coils) //21//(42 magnets, 36 coils) //$$$$$$$$$$$$$ SPECIFIC $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//#define CAN_ID 0x163 //first number is left right 1=left 2=right 3=center; 2nd number is joint (1=toe 2=ankle 3=knee 4=hip 5=torso 6=clavicle 7=shoulder 8=elbow 9=wrist ...); 3rd number is xyz (1=x=roll, 2=y=pitch, 3=z=yaw)
//#define INVERT 1

//--ESC_ID = 100; // shoulder yaw
//float phase0 = 0f;  //$$$$$$$$$$$$$ SPECIFIC $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//#define  N_POLES 7 //7(14 magnets, 12 coils) //21//(42 magnets, 36 coils) //$$$$$$$$$$$$$ SPECIFIC $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//#define CAN_ID 0x173
//#define INVERT 0



// --- SYSTEM SPECIFIC PARAMETERS

enum Control_method {sinusoidal = 0, trapezoidal = 1, freerun = 2} ;
control_method = sinusoidal;

uint8_t CONVERT=0;

int pwm = 2048; // this is half of PWM_STEPS? todo check
float pwm_float = 2048.0f;
// --- CONTROL PARAMETERS

float amp = 0.01f;  // amp //todo check if f needs to be appended
bool sw_enable_pwm = true;
//int direction = 1;
float phase_shift = PI/2 ; //todo 0.1f is an empirical correction and is to be replaced by FOC

float stiffness = 0;

float pos_amp = 100.0f;
float pos_freq = 0.5f;
float pos_amp_limit = 0.2f;
int32_t pos_offset = 0;
float P_gain = 0.0005f;

// --- INITIALIZE OTHER GLOBALS
int32_t EncVal;
int32_t last_EncVal;
int32_t last_EncVal_v;

int32_t rotation_counter = 0;
//int32_t rotation_counter_abs = 0; // TODO if useful
bool counter0ing_at0crossing = true;
float phase = 0.0f;
int field_phase_int = 0;
float vel = 0.0f;
float av_vel = 0.0f;
int pwmA = 0;
int pwmB = 0;
int pwmC = 0;

int skip_update = 0;
int skip_update_high_v = 0;


uint16_t adc1_buf[8];
uint16_t adc2_buf[8];
uint16_t adc3_buf[8];

uint16_t val_SO1_buf_index = 0;
uint16_t val_SO1_buf[200];

int32_t pole_phase_int;

float field_phase_shift = 0;
float field_phase_shift_pihalf = 0;
uint32_t field_amplitude = 0;

//uint16_t tim12_counter = 5;
uint32_t tim12_counter = 4000000000;

uint8_t buf_msgs[100];
uint8_t buf_msg[50];

bool print2uart = true;

uint16_t pole_angles[N_PHASES * N_POLES];
uint16_t pole_angle_by_amp[20];
float av_start_angle;

bool normal_operation_enabled = true;

uint8_t wave_mode = 0;

uint8_t mode_of_operation = 0; // 0=startup 1=standard
//enum mode_of_operation{ STARTUP, STANDARD};

uint8_t mode_of_control = 0; // 0=open 1=position

int32_t Enc_Val_total_lim_m = 0;
int32_t Enc_Val_total_lim_p = 0;

uint32_t last_tim5_cnt = 0 ;


// --- LOOKUPS

float lookup[210];
float sin_lookup[628];
float cos_lookup[628];
float amp_harmonic = 1.0f;

uint32_t acc_STRAIN0 = 0; //last number refers to rank

uint32_t val_I = 0;
uint32_t val_ASENSE = 0;
uint32_t val_STRAIN0 = 0; //last number refers to rank
uint32_t val_M0_TEMP = 0;

uint32_t val_SO1 = 0;
uint32_t val_BSENSE = 0;
uint32_t val_STRAIN1 = 0;
uint32_t val_TEMP = 0;
//uint32_t val_VBUS = 0; //TODO this value is not read out correctly - always comes as 0

uint32_t val_SO2 = 0;
uint32_t val_CSENSE = 0;


uint32_t analog_samples_counter = 0;
uint32_t fast_control_task_counter = 0;
uint32_t slow_control_task_counter = 0;
uint32_t prep_counter = 0;


float direct_component = 0.0f;
float direct_component_lp = 0.0f;
float quadrature_component = 0.0f;
float quadrature_component_lp = 0.0f;
float A_mean= 2040.0f;
float B_mean= 2005.0f;
float C_mean= 2005.0f;
int32_t component_counter = 0;
int32_t direct_component_sum = 0;
int32_t quadrature_component_sum = 0;

float FOC_phase_shift = 0.0f;

float generic_gain = 1.0f;
float generic_n = 0.0f;

float p = 0.0f;
float a;
float b;

char ch='.';

char buf[BUF_LEN]; //todo switch to char
char buf_add[BUF_ADD_LEN];

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	// --- STARTING SERIAL OUTPUT IN TERMINAL
	//ls /dev/tty*
	//screen /dev/tty.usbmodem14203 115200          --- stop: control a \
	//screen /dev/tty.usbmodem14103 115200          --- stop: control a \

	//Drivers/CMSIS/DSP/Include
	//ARM_MATH_CM4



	/* USER CODE END 1 */


	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	// todo transfer init part of code here

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
	MX_ADC3_Init();
	MX_USB_OTG_FS_PCD_Init();
	MX_RTC_Init();
	MX_TIM6_Init();
	MX_TIM3_Init();
	MX_TIM5_Init();
	/* USER CODE BEGIN 2 */

	calc_lookup(lookup);
	calc_sin_lookup(sin_lookup);
	calc_cos_lookup(cos_lookup);


	//uint8_t plot[300];


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
	HAL_GPIO_WritePin(EN_GATE_GPIO_Port, EN_GATE_Pin, 1); //todo maybe redundant since its also done below


	SCB->CPACR |= 0xf00000;

	int i=0;
	uint32_t i_fast = 0;
	uint32_t i_slow = 0;
	uint32_t fast2slow = 100;

	int blink_duration = 100;

	// --- MOTOR DRIVER ----------------------------------------------------
	//EN_GATE
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4);

	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);

	playSound( 3, 100, 20);

	HAL_GPIO_WritePin(EN_GATE_GPIO_Port, EN_GATE_Pin, GPIO_PIN_SET);


	// --- used for heartbeat of microcontroller
	HAL_TIM_Base_Start_IT(&htim3);

	// --- 32bit timer used to measure time in10mus
	HAL_TIM_Base_Start(&htim5);

	// --- ADC --------------------------------------
	ADC_ChannelConfTypeDef adcChannel;



	// --- I2C2 IMU ------------------------------------------------
	//see: https://www.youtube.com/watch?v=isOekyygpR8

	//  buf[0] = 0x6B;
	//  HAL_I2C_Master_Transmit(&hi2c2, IMU_ADDR, buf, 1, HAL_MAX_DELAY);
	//  HAL_I2C_Master_Transmit(&hi2c2, IMU_ADDR, 0x00, 1, HAL_MAX_DELAY);
	//  HAL_Delay(2);

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
	//TODO: Error handling

	uint8_t spi_address_8[2];
	uint8_t spi_value_8[2];

	//todo UGLY BUG - Ugly FIX: now i just send the init below twice because somehow the communication of the first transaction does not seem to work-- i sse on the MISO signal the lines just pulls up to 0.5fV instead of 3V but it works fine for the next transmission so it gets initialized correctly if i sent it twice



	// --- set ABI and enable PWM
	spi_address_8[1]= 0x00;//
	spi_address_8[0]= 0x18;//00000000 00011000
	spi_value_8[1]= 0x80;
	spi_value_8[0]= 0x80;  //10000000 10000000
	delay_SPI();
	HAL_GPIO_WritePin(ROT0_nCS_GPIO_Port, ROT0_nCS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, (uint8_t *)spi_address_8, 1, 1);// The HAL function here takes only 8bit only - still the "Size amount of data" is 1 because we set spi to 16 bit in Config
	HAL_GPIO_WritePin(ROT0_nCS_GPIO_Port, ROT0_nCS_Pin, GPIO_PIN_SET);
	delay_SPI();
	HAL_GPIO_WritePin(ROT0_nCS_GPIO_Port, ROT0_nCS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, (uint8_t *)spi_value_8, 1, 1);// The HAL function here takes only 8bit only - still the "Size amount of data" is 1 because we set spi to 16 bit in Config
	HAL_GPIO_WritePin(ROT0_nCS_GPIO_Port, ROT0_nCS_Pin, GPIO_PIN_SET);

	// --- set steps 2000steps 500 pulses //todo this sometimes seems not to work as I get 4000 steps
	spi_address_8[1]= 0x80;
	spi_address_8[0]= 0x19; //00001000 00011001
	//address = AS_ADDR_SETTINGS2 | AS_WRITE ; // 0x8019
	//value = 0x0020 | AS_ODD; // 0x8020
	//value = 0x00E0 | AS_ODD;
	spi_value_8[1]= 0x00;
	spi_value_8[0]= 0x00;  //was 0x80 and 0x20 10000000 00100000 to get 2000 pulses --> changed to 4000
	delay_SPI();
	HAL_GPIO_WritePin(ROT0_nCS_GPIO_Port, ROT0_nCS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, (uint8_t *)spi_address_8, 1, 1);// The HAL function here takes only 8bit only - still the "Size amount of data" is 1 because we set spi to 16 bit in Config
	HAL_GPIO_WritePin(ROT0_nCS_GPIO_Port, ROT0_nCS_Pin, GPIO_PIN_SET);
	delay_SPI();
	HAL_GPIO_WritePin(ROT0_nCS_GPIO_Port, ROT0_nCS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, (uint8_t *)spi_value_8, 1, 1);// The HAL function here takes only 8bit only - still the "Size amount of data" is 1 because we set spi to 16 bit in Config
	HAL_GPIO_WritePin(ROT0_nCS_GPIO_Port, ROT0_nCS_Pin, GPIO_PIN_SET);

	// --- read angle
	HAL_Delay(1);

	uint8_t angle8[2];
	uint16_t angle;

	//for (int i=0; i<4; i++)
	spi_address_8[1]= 0x7F;
	spi_address_8[0]= 0xFE;
	delay_SPI();
	HAL_GPIO_WritePin(ROT0_nCS_GPIO_Port, ROT0_nCS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, (uint8_t *)spi_address_8, 1, 1);// The HAL function here takes only 8bit only - still the "Size amount of data" is 1 because we set spi to 16 bit in Config
	HAL_GPIO_WritePin(ROT0_nCS_GPIO_Port, ROT0_nCS_Pin, GPIO_PIN_SET);
	delay_SPI();
	HAL_GPIO_WritePin(ROT0_nCS_GPIO_Port, ROT0_nCS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Receive(&hspi2, (uint8_t *)&angle8, 1, 1);
	HAL_GPIO_WritePin(ROT0_nCS_GPIO_Port, ROT0_nCS_Pin, GPIO_PIN_SET);

	angle = (uint16_t) angle8[0] | (uint16_t) angle8[1] << 8U;
	angle &= AS_DATA_MASK;


	// --- ROTATION SENSOR 0 POINT SETTING ----------------------------------------------------
	//angle &= AS_DATA_MASK;
	EncVal = (uint32_t) ((float)angle /16384.0f * ENC_STEPS);
	last_EncVal = EncVal;
	last_EncVal_v = EncVal;
	TIM8->CNT = EncVal;


	// --- Calibrate phase0   ------ TODO also this requires that we are not yet sensitive to value change
	//	TIM1->CCR1 = 50;
	//	HAL_Delay(500);
	//	EncVal = TIM8->CNT;//takes 200ns
	//	TIM1->CCR1 = 0;
	//	phase = (float) EncVal * 0.02199f ;

	// --- UART DMA
	HAL_DMA_RegisterCallback(&hdma_usart3_tx, HAL_DMA_XFER_CPLT_CB_ID, &DMAUSARTTransferComplete);



	//HAL_TIM_RegisterCallback(&htim8, HAL_TIM_IC_CAPTURE_CB_ID, &EncoderStepCallback );

	// --- ADC DMA
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_buf, 30); // this is the only one working // the length must be multiple of channels otherwise I observed mess in order - even like 2 of one and lots of mess
	HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_buf, 30); // TODO enabling this only leads to no change all values stay zero
	HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc3_buf, 30); // TODO enabling this breaks transmission entirely


	HAL_ADCEx_InjectedStart (&hadc1);
	HAL_ADCEx_InjectedStart (&hadc2);
	HAL_ADCEx_InjectedStart (&hadc3); // again this seems to break  the full loop


	//
	//

	// --- WELCOME
	sprintf((char*)buf, "\r\n\r\nWELCOME TO MUSCLEmaster \r\n\r\nangle: %d EncVal %d \r\nangle: %u EncVal %u \r\n\r\n",
			angle, EncVal ,
			angle, EncVal );
	huart3.Instance->CR3 |= USART_CR3_DMAT; //enabel dma as we disable in callback so uart can be used for something else
	HAL_DMA_Start_IT(&hdma_usart3_tx, (uint32_t)buf, (uint32_t)&huart3.Instance->DR, strlen(buf));

	HAL_Delay(10); //some delay needed othwise the first print statement in while will overwrite


	//HAL_TIM_Base_Start(&htim6);
	//HAL_TIM_Base_Start(&htim3);
	//HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_1);

	//--- CAN interface
	pHeader.DLC =6;
	pHeader.IDE = CAN_ID_STD;
	pHeader.RTR = CAN_RTR_DATA;
	pHeader.StdId = 0x001;

	sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	sFilterConfig.FilterIdHigh = CAN_ID<<5;
	sFilterConfig.FilterIdLow = 0;
	sFilterConfig.FilterMaskIdHigh = 0xFFFF;
	sFilterConfig.FilterMaskIdLow = 0xFFFF;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
	sFilterConfig.FilterActivation = ENABLE;

	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);



	mode_of_operation = 1;

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{

		if (analog_samples_counter >= ANALOG_SAMPLES_N){
			fast_control_task();
			print_prep_task(prep_counter);

			analog_samples_counter = 0;
			fast_control_task_counter ++;
			prep_counter ++;
		}

		if(fast_control_task_counter >= FAST_PER_SLOW){
			slow_control_task();

			fast_control_task_counter = 0;
			slow_control_task_counter ++;
		}



		static uint32_t last_ui_task_cnt = 0;
		uint32_t t_since_last_ui_task = TIM5->CNT - last_ui_task_cnt; //TIM5 100kHz = 10mus
		if (t_since_last_ui_task > 2000000000){
			t_since_last_ui_task -= 0xFFFFFFFF; // TODO needs to be checked
		}
		if(t_since_last_ui_task > 20000){ //5Hz
			last_ui_task_cnt = TIM5->CNT;
			uint32_t time10mus = TIM5->CNT;

			keyboard_intake();

			if (print2uart){
				print_task();
			}
			prep_counter = 0;
			ch='.';

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
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
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
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
	PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Enables the Clock Security System
	 */
	HAL_RCC_EnableCSS();
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

	ADC_MultiModeTypeDef multimode = {0};
	ADC_ChannelConfTypeDef sConfig = {0};
	ADC_InjectionConfTypeDef sConfigInjected = {0};

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 4;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure the ADC multi-mode
	 */
	multimode.Mode = ADC_TRIPLEMODE_INJECSIMULT;
	multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_5CYCLES;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = 2;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Rank = 3;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 4;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
	 */
	sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
	sConfigInjected.InjectedRank = 1;
	sConfigInjected.InjectedNbrOfConversion = 4;
	sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
	sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISINGFALLING;
	sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_CC4;
	sConfigInjected.AutoInjectedConv = DISABLE;
	sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
	sConfigInjected.InjectedOffset = 0;
	if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
	 */
	sConfigInjected.InjectedChannel = ADC_CHANNEL_11;
	sConfigInjected.InjectedRank = 2;
	if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
	 */
	sConfigInjected.InjectedChannel = ADC_CHANNEL_14;
	sConfigInjected.InjectedRank = 3;
	if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
	 */
	sConfigInjected.InjectedChannel = ADC_CHANNEL_5;
	sConfigInjected.InjectedRank = 4;
	if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
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
	ADC_InjectionConfTypeDef sConfigInjected = {0};

	/* USER CODE BEGIN ADC2_Init 1 */

	/* USER CODE END ADC2_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc2.Instance = ADC2;
	hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
	hadc2.Init.Resolution = ADC_RESOLUTION_12B;
	hadc2.Init.ScanConvMode = ENABLE;
	hadc2.Init.ContinuousConvMode = DISABLE;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.NbrOfConversion = 4;
	hadc2.Init.DMAContinuousRequests = ENABLE;
	hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	if (HAL_ADC_Init(&hadc2) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_12;
	sConfig.Rank = 2;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_15;
	sConfig.Rank = 3;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = 4;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
	 */
	sConfigInjected.InjectedChannel = ADC_CHANNEL_2;
	sConfigInjected.InjectedRank = 1;
	sConfigInjected.InjectedNbrOfConversion = 4;
	sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
	sConfigInjected.AutoInjectedConv = DISABLE;
	sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
	sConfigInjected.InjectedOffset = 0;
	if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
	 */
	sConfigInjected.InjectedChannel = ADC_CHANNEL_12;
	sConfigInjected.InjectedRank = 2;
	if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
	 */
	sConfigInjected.InjectedChannel = ADC_CHANNEL_15;
	sConfigInjected.InjectedRank = 3;
	if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
	 */
	sConfigInjected.InjectedChannel = ADC_CHANNEL_4;
	sConfigInjected.InjectedRank = 4;
	if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
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
	ADC_InjectionConfTypeDef sConfigInjected = {0};

	/* USER CODE BEGIN ADC3_Init 1 */

	/* USER CODE END ADC3_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc3.Instance = ADC3;
	hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
	hadc3.Init.Resolution = ADC_RESOLUTION_12B;
	hadc3.Init.ScanConvMode = ENABLE;
	hadc3.Init.ContinuousConvMode = DISABLE;
	hadc3.Init.DiscontinuousConvMode = DISABLE;
	hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc3.Init.NbrOfConversion = 4;
	hadc3.Init.DMAContinuousRequests = ENABLE;
	hadc3.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	if (HAL_ADC_Init(&hadc3) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_13;
	sConfig.Rank = 2;
	if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = 3;
	if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_13;
	sConfig.Rank = 4;
	if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
	 */
	sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
	sConfigInjected.InjectedRank = 1;
	sConfigInjected.InjectedNbrOfConversion = 4;
	sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
	sConfigInjected.AutoInjectedConv = DISABLE;
	sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
	sConfigInjected.InjectedOffset = 0;
	if (HAL_ADCEx_InjectedConfigChannel(&hadc3, &sConfigInjected) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
	 */
	sConfigInjected.InjectedChannel = ADC_CHANNEL_13;
	sConfigInjected.InjectedRank = 2;
	if (HAL_ADCEx_InjectedConfigChannel(&hadc3, &sConfigInjected) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
	 */
	sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
	sConfigInjected.InjectedRank = 3;
	if (HAL_ADCEx_InjectedConfigChannel(&hadc3, &sConfigInjected) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
	 */
	sConfigInjected.InjectedChannel = ADC_CHANNEL_13;
	sConfigInjected.InjectedRank = 4;
	if (HAL_ADCEx_InjectedConfigChannel(&hadc3, &sConfigInjected) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC3_Init 2 */

	/* USER CODE END ADC3_Init 2 */

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
	hcan1.Init.Prescaler = 3;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_10TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
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
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void)
{

	/* USER CODE BEGIN RTC_Init 0 */

	/* USER CODE END RTC_Init 0 */

	/* USER CODE BEGIN RTC_Init 1 */

	/* USER CODE END RTC_Init 1 */
	/** Initialize RTC Only
	 */
	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv = 127;
	hrtc.Init.SynchPrediv = 255;
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	if (HAL_RTC_Init(&hrtc) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN RTC_Init 2 */

	/* USER CODE END RTC_Init 2 */

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
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 4095;
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
	if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
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
	sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
	sConfigOC.Pulse = 1900;
	if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
	htim2.Init.Prescaler = 7;
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
	htim3.Init.Prescaler = 83;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 1000;
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
	if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
	htim5.Init.Prescaler = 839;
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
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void)
{

	/* USER CODE BEGIN TIM6_Init 0 */

	/* USER CODE END TIM6_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM6_Init 1 */

	/* USER CODE END TIM6_Init 1 */
	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 83;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 1000;
	htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM6_Init 2 */

	/* USER CODE END TIM6_Init 2 */

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
	htim8.Init.Period = 3999;
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
	htim9.Init.Prescaler = 167;
	htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim9.Init.Period = 1000;
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
	htim12.Init.Prescaler = 7;
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
 * @brief USB_OTG_FS Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_OTG_FS_PCD_Init(void)
{

	/* USER CODE BEGIN USB_OTG_FS_Init 0 */

	/* USER CODE END USB_OTG_FS_Init 0 */

	/* USER CODE BEGIN USB_OTG_FS_Init 1 */

	/* USER CODE END USB_OTG_FS_Init 1 */
	hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
	hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
	hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
	hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
	hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
	if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USB_OTG_FS_Init 2 */

	/* USER CODE END USB_OTG_FS_Init 2 */

}

/** 
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) 
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	/* DMA2_Stream1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
	/* DMA2_Stream2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, LD_1_Pin|LD_2_Pin|EN_GATE_Pin|M0_DC_CAL_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11|debug1_out_Pin|debug2_out_Pin|ROT0_nCS_Pin
			|nSCS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : LD_1_Pin LD_2_Pin EN_GATE_Pin M0_DC_CAL_Pin */
	GPIO_InitStruct.Pin = LD_1_Pin|LD_2_Pin|EN_GATE_Pin|M0_DC_CAL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pin : PC13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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

void set_pwm_off(void){
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
}

void step_through_pole_angles(void){
	normal_operation_enabled = false;
	set_pwm_off();
	HAL_Delay(100);
	uint32_t step_through_amp = 5 * PWM_1PERCENT;
	for (uint8_t pole = 0; pole < N_POLES ; pole++){
		for (uint8_t ABC = 0; ABC < N_PHASES ; ABC++){
			set_pwm_off();
			if (ABC==0){
				TIM1->CCR1 = step_through_amp;
			}
			else if (ABC==1){
				if (INVERT){
					TIM1->CCR3 = step_through_amp;
				}
				else{
					TIM1->CCR2 = step_through_amp;
				}

			}
			else {
				if (INVERT){
					TIM1->CCR2 = step_through_amp;
				}
				else{
					TIM1->CCR3 = step_through_amp;
				}
			}
			HAL_Delay(200);
			pole_angles[pole * N_PHASES + ABC]=TIM8->CNT;



			uint8_t buf[300];
			buf[0] = '\0';
			sprintf((char*)buf_msg, "[step_through_pole_angles] pole: %d ABC: %d angle: %d \r\n", pole, ABC, TIM8->CNT);
			if (strlen(buf_msg) + strlen(buf_msgs) < 100){
				strcat(buf_msgs, buf_msg);
			}
			else {
				buf_msgs[0] = '#';
			}
			if (buf_msgs[0] != '\0'){
				strcat(buf, buf_msgs);
				buf_msgs[0] = '\0';
			}
			//HAL_UART_Transmit_IT(&huart3, buf, strlen((char*)buf)); //WORKS but replaced by DMA below
			huart3.Instance->CR3 |= USART_CR3_DMAT; //enabel dma as we disable in callback so uart can be used for something else
			HAL_DMA_Start_IT(&hdma_usart3_tx, (uint32_t)buf, (uint32_t)&huart3.Instance->DR, strlen(buf));

		}
	}
	set_pwm_off();
	normal_operation_enabled = true;

	float sum = 0.0f;
	float enc_steps_per_A2B = (float)ENC_STEPS / (float)(N_POLES * N_PHASES);
	float enc_steps_per_A2A = (float)ENC_STEPS / (float)N_POLES;
	for (uint8_t i = 0; i < N_POLES * N_PHASES ; i++){
		float reduced_pole_angle = pole_angles[i] - i * enc_steps_per_A2B ;//should be 95.238=ENC_STEPS/21 = ENC_STEPS/ (N_POLES * N_PHASES)
		if (reduced_pole_angle > -ENC_STEPS_HALF){
			sum += reduced_pole_angle;
		}
		else{
			sum += reduced_pole_angle + ENC_STEPS;
		}
		av_start_angle = sum / (float)(N_POLES * N_PHASES);
		while(av_start_angle > enc_steps_per_A2A){
			av_start_angle -= enc_steps_per_A2A;
		}
		//float av_angle_first_A =

	}


}

void step_through_pwm_percent(void){
	normal_operation_enabled = false;
	set_pwm_off();
	HAL_Delay(100);
	for (uint8_t percent = 0; percent < 10 ; percent++){
		TIM1->CCR1 = percent * PWM_1PERCENT;
		HAL_Delay(200);
		pole_angle_by_amp[percent]=TIM8->CNT;
	}
	set_pwm_off();
	normal_operation_enabled = true;
}

void explore_limits(void){
	amp = 0;
	HAL_Delay(100);
	for (int8_t dir=-1;dir<2; dir+=2){
		HAL_Delay(500);
		amp= dir * 0.1f;
		for (int32_t i = 0; i<50; i++){
			HAL_Delay(100);
			uint32_t val_I = HAL_ADCEx_InjectedGetValue (&hadc1, 1);
			if (val_I > 2100 || val_I < 1980){
				amp=0;
				uint32_t EncVal_lim = TIM8->CNT;
				if (dir==-1){
					Enc_Val_total_lim_m = EncVal_lim + rotation_counter * ENC_STEPS;
				}
				else{
					Enc_Val_total_lim_p = EncVal_lim + rotation_counter * ENC_STEPS;

				}

				break;
			}
		}
	}

	amp = 0.01f;
}



void delay_SPI(void){
	int g =0;
	for(int i=0; i<20; i++){
		g++;
	}
}


void playSound(uint32_t periode, uint32_t volume, uint32_t cycles){
	// TODO disable interrupt for the duration of sound
	//HAL_NVIC_DisableIRQ(TIM8_UP_TIM13_IRQn);
	//HAL_Delay(1000);
	normal_operation_enabled = false;
	set_pwm_off();
	HAL_Delay(10);

	for (uint32_t i=0; i<cycles; i++){
		TIM1->CCR1 = 0; //takes<150ns
		TIM1->CCR2 = volume; //takes<150ns
		HAL_Delay(periode);
		TIM1->CCR1 = volume; //takes<150ns
		TIM1->CCR2 = 0; //takes<150ns
		HAL_Delay(periode);
	}
	set_pwm_off();
	normal_operation_enabled = true;

	//HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
}


void calc_lookup(float *lookup){
	for (int i=0; i<210; i++){
		// --- vanilla
		//lookup[i] = cos((float)i/100.0f) + cos((float)i/100.0f-1.047f);

		// --- harmonic
		//lookup[i] = cos((float)i/100.0f)       + amp_harmonic * cos( (float)i/100.0f       * 3.0f)    +  cos((float)i/100.0f-1.047f) + amp_harmonic * cos(((float)i/100.0f-1.047f)* 3.0f) ;// the harmonic tends to fully cancel out

		// --- power law
		lookup[i] = pow( cos((float)i/100.0f) + cos((float)i/100.0f-1.047f),amp_harmonic)/ pow(amp_harmonic,0.5f); //looks like 1.0 is already best in terms of overtones
	}
}

void calc_sin_lookup(float *sin_lookup){
	for (int i=0; i<628; i++){
		sin_lookup[i] = sin((float)i/100.0f);
	}
}

void calc_cos_lookup(float *cos_lookup){
	for (int i=0; i<628; i++){
		cos_lookup[i] = cos((float)i/100.0f);
	}
}

void DMAUSARTTransferComplete(DMA_HandleTypeDef *hdma){
	huart3.Instance->CR3 &= ~USART_CR3_DMAT;
}





// I find 80Hz at 25V this would be 192kV arrg
void calc_v(void){
	int16_t tim2_counter = TIM2->CNT; //has prescalor of 8-1 ==> 10.5MHz ==> will always be around 10500 for heartbeat 1ms
	TIM2->CNT = 0;

	int16_t EncDiff = EncVal-last_EncVal_v; // will be 200 for 100Hz rotation or 2 for 1Hz rotation
	last_EncVal_v = EncVal;

	if (EncDiff > ENC_STEPS_HALF){ // if jump is more than a half rotation it's most likely the 0 crossing
		EncDiff -= ENC_STEPS;
	}
	else if (EncDiff < -ENC_STEPS_HALF){
		EncDiff += ENC_STEPS;
	}

	vel = (float)(EncDiff) / (float)tim2_counter; //[steps/counts]
	vel *= 10500000/ENC_STEPS; // /ENC_STEPS steps/round * 21000000 counts/sec --> [round/sec]  //TODO vel seems too high by factor of 2 or 3 maybe same clock frequency issue that we actually run at 42 MHz. !!! TODO check clock frequency  // TODO divided by 10 as well
	av_vel = 0.95f * av_vel + 0.05f * vel;

}


// --- Callback when Encoder fires the I at zero point
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if (GPIO_Pin == ROT0_I_W_Pin){
		//HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_3);
		uint16_t encoder_belief = TIM8->CNT;

		if (counter0ing_at0crossing){
			TIM8->CNT = 0;     //TODO: this could lead to an offset of 1 since the CNT value may not be set yet and get incremented thereafter if this interrupt is executed before the CNT increment.
			counter0ing_at0crossing = false;
			sprintf((char*)buf_msg, "[EXTI_Callback] EncVal at FIRST ZERO: %d \r\n", encoder_belief);
			if (strlen(buf_msg) + strlen(buf_msgs) < 100){
				strcat(buf_msgs, buf_msg);
			}
			else {
				buf_msgs[0] = '#';
			}
		}
		val_SO1_buf_index = 0;

		if (encoder_belief > ENC_TOLERANCE && encoder_belief < ENC_STEPS - ENC_TOLERANCE){
			sprintf((char*)buf_msg, "[EXTI_Callback] EncVal at ZERO MISMATCH: %d \r\n", encoder_belief);
			if (strlen(buf_msg) + strlen(buf_msgs) < 100){
				strcat(buf_msgs, buf_msg);
			}
			else {
				buf_msgs[0] = '#';
			}
		}
	}
	else{
		__NOP();
	}
}



// -----------------------------------------------------------
// MAIN UPDATE STEP interrupt triggered by timer 1 channel 4 towards end of each pwm cycle
// -----------------------------------------------------------
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim != &htim1){
		return;
	}

#if DB_TIMING
	DB1H;
#endif

	//timing_party();

	// --- get current encoder position
	last_EncVal = EncVal;
	EncVal = TIM8->CNT;//both lines 200ns

	// --- determine whether 0 crossing happened and adjust rotation_counter accordingly
	if (EncVal - last_EncVal > ENC_STEPS_HALF) {
		rotation_counter--;
	}
	else if (last_EncVal - EncVal > ENC_STEPS_HALF){
		rotation_counter++;
	}// both statements 300ns

	// --- accumulate analog readings till we have enough samples which is a flag for the heart beat (= all MCU internal control loops)
	if (analog_samples_counter < ANALOG_SAMPLES_N ){  // TODO: if n_samples >= 32
		acc_STRAIN0 += HAL_ADCEx_InjectedGetValue (&hadc1, 3);
		analog_samples_counter ++;
	}//200ns when not entering presumably

	// --- calculate the phase with respect to a pole cycle in 100x int
	pole_phase_int = (int)((PI2 * N_POLES / ENC_STEPS * (float) EncVal - phase0 + PI2) * 100.0f) % 628 ; //400ns when consolidated in one line



#if FOC_ALG

	register int32_t A = HAL_ADCEx_InjectedGetValue (&hadc1, 1);//500ns
	register int32_t B = HAL_ADCEx_InjectedGetValue (&hadc2, 1);//500ns
	register int32_t C = HAL_ADCEx_InjectedGetValue (&hadc3, 1);//500ns


	//	register float mean_lp = 0.00001f;
	//      A_mean = (1-mean_lp) * A_mean + mean_lp * A;
	//			B_mean = (1-mean_lp) * B_mean + mean_lp * B;
	//	A_mean = 2040.0f;
	//	B_mean = 2005.0f;
	//	C_mean = 2005.0f;

	// --- Park transform
	a = 0.7f * ((float)A-A_mean);
	b = INV_SQRT_3 * (a + 2.0f * ((float)B-B_mean)); //200ns thanks to precalc of SQRT

	// -- Clark transform
	direct_component = a * cos_lookup[pole_phase_int] + b * sin_lookup[pole_phase_int];
	quadrature_component = -a * sin_lookup[pole_phase_int] + b * cos_lookup[pole_phase_int]; //300ns

	// --- low pass filter
	register float lp = 0.001f;
	direct_component_lp = (1-lp) * direct_component_lp + lp * direct_component;
	quadrature_component_lp = (1-lp) * quadrature_component_lp + lp * quadrature_component;//with register 240 without register 380ns for the 3 lines


	static float direct_component_lp_integral = 0.0f;
	direct_component_lp_integral += direct_component_lp;//150ns for 2lines

	// --- PI controller
	FOC_phase_shift = 0.005f* generic_gain * direct_component_lp + 0.00001f  * direct_component_lp_integral; //220ns//starts oscillating at I = 0.00008f and alternatively at P = 0.03f


	if (FOC_phase_shift > FOC_PHASE_LIM){
		FOC_phase_shift = FOC_PHASE_LIM;
	}
	else if (FOC_phase_shift < -FOC_PHASE_LIM){
		FOC_phase_shift = -FOC_PHASE_LIM;
	}//350ns for checks

	if (abs(av_vel) < 1.0f){
		FOC_phase_shift = 0.0f;
		direct_component_lp_integral = 0.0f;
	}//220ns


#else if //empirical mean
	FOC_phase_shift = 0.1f;
#endif


#if DB_TIMING
	DB1L;
#endif


	update_pwm();

}




// -----------------------------------------------------------
// called from MAIN UPDATE STEP to calc and write pwm values to FETdriver
// -----------------------------------------------------------
void update_pwm(void){

#if DB_TIMING
	DB1H;
#endif

	//register int32_t field_phase_int;
	register float u0 = 0.5773f; //0.5f * 2.0f / 1.73205f;// maximal possible U on one coil thanks to wankel //takes<200ns
	register float modified_amp = amp + stiffness * av_vel;// * direction; // TODO the abs allows same stiffness to make it softer for both directions - without a signchange is needed BUT turnaround is super aggressive now :( SAME issue with direction - super forceful reverse but sign identical --- looks like v needs to direct also the phase !!!!

	if (modified_amp > AMP_LIMIT){
		modified_amp = AMP_LIMIT;
	}
	if (modified_amp < -AMP_LIMIT){
		modified_amp = -AMP_LIMIT;
	}

	if (modified_amp > 0){
		field_phase_int = pole_phase_int - (int)((phase_shift + FOC_phase_shift) * 100.0f);
		u0 *= modified_amp;  //takes<200ns
	}
	else {
		field_phase_int = pole_phase_int + (int)((phase_shift + FOC_phase_shift) * 100.0f);
		u0 *= -modified_amp;  //takes<200ns
	}

	if (!sw_enable_pwm){
		u0 = 0;
	}

	if (field_phase_int < 0) {
		field_phase_int += 628;
	}
	else if (field_phase_int >= 628) {
		field_phase_int -= 628;
	}//150ns

	register float uA = 0;
	register float uB = 0;
	register float uC = 0;

	if (control_method != freerun ){
		if (control_method == sinusoidal ){

			if  (field_phase_int < 210)	{
				uA = lookup[field_phase_int]; //took<32000ns - with lookup implement it's just 2000ns
				uB = lookup[210 - 1 - field_phase_int]; //
				uC = 0;
			}
			else if  (field_phase_int < 420){	 //210...419
				uA = 0;
				uB = lookup[field_phase_int - 210];
				uC = lookup[420 - 1 - field_phase_int];
			}
			else	{  //420...629
				uA = lookup[630 - 1 - field_phase_int];
				uB = 0;
				uC = lookup[field_phase_int - 420];
			}
		}//400ns

		else if (control_method == trapezoidal){
			if  (field_phase_int < 105-52)	{
				uA = 1;
				uB = 0;
				uC = 0;
			}
			else if  (field_phase_int < 210-52)	{
				uA = 1;
				uB = 1;
				uC = 0;
			}
			else if  (field_phase_int < 315-52)	{
				uA = 0;
				uB = 1;
				uC = 0;
			}
			else if  (field_phase_int < 420-52)	{
				uA = 0;
				uB = 1;
				uC = 1;
			}
			else if  (field_phase_int < 525-52)	{
				uA = 0;
				uB = 0;
				uC = 1;
			}
			else if  (field_phase_int < 630-52)	{
				uA = 1;
				uB = 0;
				uC = 1;
			}
			else 	{ //same as first half phase
				uA = 1;
				uB = 0;
				uC = 0;
			}
		}

		pwmA = (uint16_t) (PWM_F * u0 * uA); //180ns
		pwmB = (uint16_t) (PWM_F * u0 * uB); //180ns
		pwmC = (uint16_t) (PWM_F * u0 * uC); //180ns

		// --- send out PWM pulses 0...2048
		if (normal_operation_enabled){
			TIM1->CCR1 = pwmA; //takes<150ns
			if (INVERT){
				TIM1->CCR3 = pwmB; //takes<150ns
				TIM1->CCR2 = pwmC; //takes<150ns
			}
			else {
				TIM1->CCR2 = pwmB; //takes<150ns
				TIM1->CCR3 = pwmC; //takes<150ns
			}

		}//300ns
	}

	else{ // NOTE this mode is still experimental
		if  (field_phase_int < 105)	{
			uA = 1;
			pwmA = (uint16_t) (pwm * u0 * uA); //takes<2s00ns
			TIM1->CCR1 = pwmA; //takes<150ns
			//			SET_BIT(TIM1->CCMR1, TIM_CCMR1_OC1CE);
			//			CLEAR_BIT(TIM1->CCMR1, TIM_CCMR1_OC1CE);
			//
			//			SET_BIT(TIM1->CCMR1, TIM_CCMR1_OC2CE);
			//			CLEAR_BIT(TIM1->CCMR1, TIM_CCMR1_OC2CE);
			//			SET_BIT(TIM1->CCMR2, TIM_CCMR2_OC3CE);
			//			CLEAR_BIT(TIM1->CCMR2, TIM_CCMR2_OC3CE);

			CLEAR_BIT(TIM1->CCMR1, TIM_CR2_OIS2N);
			SET_BIT(TIM1->CCMR1, TIM_CR2_OIS3N);
		}
		else if  (field_phase_int < 210)	{
			uB = 1;
			pwmB = (uint16_t) (pwm * u0 * uB); //takes<2s00ns
			TIM1->CCR2 = pwmB; //takes<150ns

			CLEAR_BIT(TIM1->CCMR1, TIM_CR2_OIS1N);
			SET_BIT(TIM1->CCMR1, TIM_CR2_OIS3N);
		}
		else if  (field_phase_int < 315)	{
			uB = 1;
			pwmB = (uint16_t) (pwm * u0 * uB); //takes<2s00ns
			TIM1->CCR2 = pwmB; //takes<150ns

			SET_BIT(TIM1->CCMR1, TIM_CR2_OIS1N);
			CLEAR_BIT(TIM1->CCMR1, TIM_CR2_OIS3N);
		}
		else if  (field_phase_int < 420)	{
			uC = 1;
			pwmC = (uint16_t) (pwm * u0 * uC); //takes<2s00ns
			TIM1->CCR3 = pwmC; //takes<150ns

			SET_BIT(TIM1->CCMR1, TIM_CR2_OIS1N);
			CLEAR_BIT(TIM1->CCMR1, TIM_CR2_OIS2N);
		}
		else if  (field_phase_int < 525)	{
			uC = 1;
			pwmC = (uint16_t) (pwm * u0 * uC); //takes<2s00ns
			TIM1->CCR3 = pwmC; //takes<150ns

			CLEAR_BIT(TIM1->CCMR1, TIM_CR2_OIS1N);
			SET_BIT(TIM1->CCMR1, TIM_CR2_OIS2N);
		}
		else 	{
			uA = 1;
			pwmA = (uint16_t) (pwm * u0 * uA); //takes<2s00ns
			TIM1->CCR1 = pwmA; //takes<150ns

			SET_BIT(TIM1->CCMR1, TIM_CR2_OIS2N);
			CLEAR_BIT(TIM1->CCMR1, TIM_CR2_OIS3N);
		}

	}

#if DB_TIMING
	DB1L;
#endif

}

void fast_control_task(void){
	val_STRAIN0 = acc_STRAIN0 >> ANALOG_SAMPLES_BITSHIFT;
	acc_STRAIN0 = 0;

	calc_v();


	if (mode_of_control == 1){
		float t = (float)((TIM5->CNT - last_tim5_cnt) / 100) / 1000.0f;

		int32_t desired_EncVal = pos_offset + pos_amp * sin(6.28f * pos_freq * t);

		//int32_t desired_EncVal = 0;//TIM5->CNT / 100;

		//		if ((TIM5->CNT/50000)%2 == 1){
		//			desired_EncVal = 10000;
		//		}
		//		else{
		//			desired_EncVal = 0;
		//		}


		int32_t Enc_Val_total = EncVal + rotation_counter * ENC_STEPS;
		float raw_amp = (float)(Enc_Val_total - desired_EncVal) * P_gain; //oscillates for P_gain > 0.005f
		float raw_amp_check = raw_amp;
		//		if (raw_amp < 0.0f){
		//			raw_amp = -raw_amp;
		//			direction = -1;
		//		}
		//		else{
		//			direction = 1;
		//		}
		if (raw_amp > pos_amp_limit){
			raw_amp = pos_amp_limit;
		}
		if (raw_amp < - pos_amp_limit){
			raw_amp = - pos_amp_limit;
		}
		amp = raw_amp;

		if (buf_msgs[0] == '\0'){
			sprintf((char*)buf_msg, "[HEART] raw_a: %d %d %d Enc_tot: %d a: %d f: %d lim: %d off: %d g: %d\r\n",
					(int)((float)(Enc_Val_total - desired_EncVal) * 0.0005f*1000.0f),
					(int)(raw_amp*1000),
					(int)(raw_amp_check*1000),
					(int)Enc_Val_total,
					(int)(pos_amp),
					(int)(pos_freq*1000),
					(int)(pos_amp_limit * 1000),
					(int)(pos_offset),
					(int)(P_gain*1000000));
			strcat(buf_msgs, buf_msg);
		}
	}
	else{
		last_tim5_cnt = TIM5->CNT;
	}

}

void slow_control_task(void){

}

void keyboard_intake(void){

	HAL_UART_Receive_IT(&huart3, (uint8_t *)&ch, 1);

	if (rx_character_buffered != '.'){
		ch = rx_character_buffered;
		//rx_character_armed = 0;
		rx_character_buffered = '.';
	}//since beginning of slow 1000ns


	switch(ch){
	case 'w':
		amp *= 2;
		break;
	case 's':
		amp /= 2;
		break;
	case 'a':
		phase_shift -= 0.05f;
		break;
	case 'd':
		phase_shift += 0.05f;
		break;
	case 'q':
		phase0 -= 0.05f;
		break;
	case 'e':
		phase0 += 0.05f;
		break;
	case 't':
		sw_enable_pwm = true;
		break;
	case 'g':
		sw_enable_pwm = false;
		break;
	case 'h':
		amp = abs(amp); //positive should be clockwise == EncVal increases positive :)
		break;
	case 'f':
		amp = -abs(amp);
		break;
	case 'r':
		amp = -amp;
		break;
	case 'z':
		playSound( 1, 20, 100);
		break;
	case 'u':
		stiffness += 0.001f;
		break;
	case 'j':
		stiffness -= 0.001f;
		break;
	case 'p':
		//print2uart = false;
		print2uart = !print2uart;
		break;
	case 'o':
		//HAL_GPIO_WritePin(EN_GATE_GPIO_Port, EN_GATE_Pin, 1);
		EN_GATE_GPIO_Port->BSRR = (uint32_t)EN_GATE_Pin << 16U;
		break;
	case 'l':
		HAL_GPIO_WritePin(EN_GATE_GPIO_Port, EN_GATE_Pin, 1);
		EN_GATE_GPIO_Port->BSRR = EN_GATE_Pin ;
		break;
	case 'X':
		step_through_pole_angles();
		break;
	case 'P':
		step_through_pwm_percent();
		break;
	case 'i':
		mode_of_control = 1;
		break;
	case 'k':
		mode_of_control = 0;
		amp = 0.05f;
		break;

		// pos control
	case 'W':
		pos_amp *= 2;
		break;
	case 'S':
		pos_amp *= 0.5f;
		break;
	case 'D':
		pos_freq *= 2;
		break;
	case 'A':
		pos_freq *= 0.5f;
		break;
	case 'C':
		if (CONVERT){
			CONVERT = 0;
		}
		else {
			CONVERT = 1;
		}
		break;
	case 'R':
		pos_amp_limit *= 2;
		break;
	case 'F':
		pos_amp_limit *= 0.5f;
	case 'E':
		pos_offset += 200;
		break;
	case 'Q':
		pos_offset -= 200;
		break;
	case 'M':
		P_gain *= 2;
		break;
	case 'N':
		P_gain *= 0.5f;
		break;
	case 'T':
		control_method = sinusoidal;
		break;
	case 'G':
		control_method = trapezoidal;
		break;
	case 'B':
		control_method = freerun;
		break;
	case 'L':
		explore_limits();
		break;
	case 'I':
		amp_harmonic += 0.1f;
		calc_lookup(lookup);
		break;
	case 'K':
		amp_harmonic -= 0.1f;
		calc_lookup(lookup);
		break;
	case 'm':
		generic_gain *= 2;
		break;
	case 'n':
		generic_gain *= 0.5f;
		break;
	case 'v':
		generic_n += PI/6;
		break;
	case 'b':
		generic_n -= PI/6;
		break;

	default:
		ch='.';
	}//for case 200ns

}

void print_prep_task(int fast_control_task_counter){

	int pos = strlen(buf);
	int left  = BUF_LEN - pos;
	int n;
#define ADD_VAL(fmt, val)                 \
		n = snprintf(buf+pos, left, fmt, val);  \
		pos += n;                               \
		left -= n;

	switch (fast_control_task_counter){
		case 0:
			sprintf(buf, "tx: %c %4d %4d %4d %4d ", ch, tx_msg[0],rx_msg[1],rx_msg[2],rx_msg[3]);//70000ns
			break;
		case 1:
			ADD_VAL(" p0:%4.2f", phase0);//70000ns
			break;
		case 2:
			ADD_VAL(" ps:%4.2f", phase_shift);//70000ns f takes much longer than int
			break;
		case 3:
			ADD_VAL(" a:%4.2f", amp);//70000ns f takes much longer than int
			break;
		case 4:
			ADD_VAL(" E:%5d", EncVal);//70000ns f takes much longer than int
			break;
		case 5:
			ADD_VAL(" c:%5d", rotation_counter);//70000ns f takes much longer than int
			break;
		case 6:
			ADD_VAL(" v:%6.2f", vel);//70000ns f takes much longer than int
			break;
		case 7:
			ADD_VAL(" pi:%4d", field_phase_int);//70000ns f takes much longer than int
			break;
		case 8:
			ADD_VAL(" FOC:%4.2f", FOC_phase_shift);//70000ns f takes much longer than int
			break;
		case 9:
			ADD_VAL(" gg:%6.3f", generic_gain);//70000ns f takes much longer than int
			break;
		case 10:
			ADD_VAL(" gn:%6.3f", generic_n);//70000ns f takes much longer than int
			break;
		case 11:
			if (CONVERT){
				float SO0 = ((float)val_I - 2040.0f) * 0.134f; // 3.3[V]/4095[ticks] /20[gain]/0.0003[ohm] = 0.134
				float SO1 = ((float)val_SO1 - 2002.0f) * 0.189f; // 3.3[V]/4095[ticks] /20[gain]/0.0003[ohm] = 0.134 //TODO verify SPI setting in DRV8301 the factor sqrt(2) comes out of thin air
				float SO2 = ((float)val_SO2 - 2002.0f) * 0.189f; // 3.3[V]/4095[ticks] /20[gain]/0.0003[ohm] = 0.134
				sprintf(buf_add, " I:%5.2fA SO1:%5.2fA SO2:%5.2fA", SO0, SO1, SO2); strcat(buf, buf_add);

				float I_tot = sqrt((SO0*SO0 + SO1*SO1 + SO2*SO2)/1.5f); //see colab - the factor 1.5 allows to extract the distance from center of triangle to tip
				sprintf(buf_add, " It:%5.2fA", I_tot); strcat(buf, buf_add);
			}
			else{
				sprintf(buf_add, " I:%4d SO1:%4d SO2:%4d", val_I, val_SO1, val_SO2); strcat(buf, buf_add);
			}
			break;
		case 12:
			ADD_VAL(" ps:%4.2f", phase_shift);//70000ns f takes much longer than int
			break;
		case 13:
			if (val_TEMP > 1900){
				sprintf(buf_add, "* >50C on ESC"); strcat(buf, buf_add);
			}

			if (val_M0_TEMP > 1900){
				sprintf(buf_add, "* >50C on MOTOR"); strcat(buf, buf_add);
			}

			if (val_STRAIN0 < 2170){
				sprintf(buf_add, "* -100N force"); strcat(buf, buf_add);
			}
			break;

		case 20:
			sprintf(buf_add, " \r\n"); strcat(buf, buf_add);
			break;
	}

#if 0

	sprintf((char*)buf_add, " dcl:%6.2f qcl:%6.2f", direct_component_lp, quadrature_component_lp); strcat(buf, buf_add);

	sprintf((char*)buf_add, " dc:%6.2f qc:%6.2f", direct_component, quadrature_component); strcat(buf, buf_add);

	sprintf((char*)buf_add, " a:%6.2f b:%6.2f", a, b); strcat(buf, buf_add);

	//sprintf((char*)buf_add, " s:%4.3f", stiffness); strcat(buf, buf_add);

	//sprintf((char*)buf_add, " h:%4.3f", amp_harmonic); strcat(buf, buf_add);

	//sprintf((char*)buf_add, " d:%2d", direction); strcat(buf, buf_add);

	sprintf((char*)buf_add, " A:%4d B:%4d C:%4d", val_ASENSE, val_BSENSE, val_CSENSE); strcat(buf, buf_add);

	if (CONVERT){
		float STRAIN0 = ((float)val_STRAIN0 - 2235.0f) * 1.678f; // 3.3/4095/0.00048[gain see page 114] = 1.678
		float STRAIN1 = ((float)val_STRAIN1 - 2235.0f) * 1.678f;
		sprintf((char*)buf_add, " S0:%5.1fN S1:%4dN", STRAIN0, val_STRAIN1); strcat(buf, buf_add);
	}
	else{
		//sprintf((char*)buf_add, " S0:%4d S1:%4d", val_STRAIN0, val_STRAIN1); strcat(buf, buf_add);
	}

	//sprintf((char*)buf_add, " S0s:%4d", val_STRAIN0); strcat(buf, buf_add);

	////sprintf((char*)buf_add, " TM:%4d TC:%4d V:%4d", val_TEMP, val_M0_TEMP, val_VBUS); strcat(buf, buf_add);
	//sprintf((char*)buf_add, " TM:%4d TC:%4d", val_TEMP, val_M0_TEMP); strcat(buf, buf_add);

	//sprintf((char*)buf_add, " ADC1: %4d %4d %4d %4d %4d %4d %4d %4d", adc1_buf[0], adc1_buf[1], adc1_buf[2], adc1_buf[3], adc1_buf[4], adc1_buf[5], adc1_buf[6], adc1_buf[7]); strcat(buf, buf_add);
	//sprintf((char*)buf_add, " ADC2: %4d %4d %4d %4d %4d %4d %4d %4d", adc2_buf[0], adc2_buf[1], adc2_buf[2], adc2_buf[3], adc2_buf[4], adc2_buf[5], adc2_buf[6], adc2_buf[7]); strcat(buf, buf_add);
	//sprintf((char*)buf_add, " ADC3: %4d %4d %4d %4d %4d %4d %4d %4d", adc3_buf[0], adc3_buf[1], adc3_buf[2], adc3_buf[3], adc3_buf[4], adc3_buf[5], adc3_buf[6], adc3_buf[7]); strcat(buf, buf_add);

	sprintf((char*)buf_add, " p:%5d m:%5d", Enc_Val_total_lim_p, Enc_Val_total_lim_m); strcat(buf, buf_add);

	//			sprintf((char*)buf, "%c# AI %d %d %d %d A1 %d %d %d %d %d            \r\n",
	//								ch, //(int)(amp*100), (int)(phase_shift*100),
	//								//(int)(stiffness*1000), (int)(1000*av_vel),
	//								val_I, val_ASENSE, val_STRAIN0, val_M0_TEMP,
	//								adc1_buf[0], adc1_buf[1], adc1_buf[2], adc1_buf[3], adc1_buf[4]);
	//								//val_SO1, val_BSENSE, val_STRAIN1, val_TEMP, val_SO2, val_CSENSE); //        %d %d %d %d A2 %d %d


	//			buf[150] = '|';
	//			buf[100] = '.';
	//			buf[50] = '|';
	//			buf[100 + max(-50, min(50, (int)av_vel))] = 'v';

	//2L //1ms = 1000000ns
#endif

}

void print_task(void){


	if (buf_msgs[0] != '\0'){
		strcat(buf, buf_msgs);
		buf_msgs[0] = '\0';
	}

		//                   0---------1---------2---------3---------4---------5---------6---------7---------8---------9---------0---------1---------2---------3---------4---------5---------6---------7---------8---------9---------0---------1---------2---------3---------4---------5



	huart3.Instance->CR3 |= USART_CR3_DMAT; //enabel dma as we disable in callback so uart can be used for something else
	HAL_DMA_Start_IT(&hdma_usart3_tx, (uint32_t)buf, (uint32_t)&huart3.Instance->DR, strlen(buf));


}



// -----------------------------------------------------------
// Just a helpful util to check processing times for different operations
// -----------------------------------------------------------
void timing_party(){
	DB1L;
	DB1H;
	DB1L;
	DB1H;
	DB1L;
	DB1H;
	DB1L;
	DB1H;

	if (0){

		DB1L;//40ns
		int t11 = 2345;//10 (subtract 40 from the 60 below and devide by 2 because we do 2 operations)
		int t12 = 1234;//10
		DB1H;//60ns
		int t13 = t11+t12;//40

		DB1L;//80ns
		int t21 = 2345;//0
		int t22 = 1234;//0
		DB1H;//40ns
		int t23 = t21/t22;//60

		DB1L;//100
		float t31 = 2345;//20
		float t32 = 1234;//20
		DB1H;//80
		float t33 = t31+t32;//160

		DB1L;//200
		float t41 = 2345;//20
		float t42 = 1234;//20
		DB1H;//80
		float t43 = t41/t42;//110

		DB1L;//150
		float t51 = 2345;//20
		int t52 = 1234;//10
		DB1H;//70
		float t53 = t51/(float)t52;//140

		DB1L;//180
		DB1H;
		DB1L;
		DB1H;



		DB1L;
		int8_t t71 = 2345;//10
		int8_t t72 = 1234;//10
		DB1H;//50
		int8_t t73 = t71+t72;//40

		DB1L;//80
		int16_t t61 = 2345;//10
		int16_t t62 = 1234;//
		DB1H;//
		int16_t t63 = t61+t62;//40


		DB1L;
		int32_t t81 = 2345;//10
		int32_t t82 = 1234;//
		DB1H;//
		int32_t t83 = t81+t82;//40


		DB1L;
		int64_t t91 = 2345;//30
		int64_t t92 = 1234;//
		DB1H;//100
		int64_t t93 = t91+t92;//60 <<<<<<<<<<<< all int have same speed but 64 is much slower


		DB1L;//100
		DB1H;
		DB1L;
		DB1H;

		DB1L;
		double r91 = 2345;//70
		double r92 = 1234;//70
		DB1H;//180
		double r93 = r91+r92;//860

		DB1L;//900
		float r81 = 2345;//20
		//int t52 = 1234;//
		DB1H;//60
		float r83 = 3.1234 * r81;//950 <<<<<<<<<<<<<<< super slow

		DB1L;//1000
		float r71 = 2345;//
		//int t52 = 1234;//
		DB1H;//60
		float r73 = 3.1234f * r71; // 80 <<<<<<<<<<<< super important to put f behind floating point number

		DB1L;//120
		DB1H;
		DB1L;
		DB1H;

		DB1L;
		int e11 = 123;
		int e12 = 234;
		int e13 =0;
		DB1H;//140
		e13 = e11 + e12;
		e13 = e13 + 345;
		DB1L;//100
		e13 = e11 + e12 + 345;// <<<<<<one line is faster than 2 lines as above !!!
		DB1H;//80
		e13 = e11 + e12 + 345 + 456 + 567; //  <<<<<NO additional time for writing out all the numbers !!!!!
		DB1L;//80
		e13 = e11 + e12;
		e13 += 345; //+= just as = _+
		DB1H;//110
		e13 = e11 + e12;//40
		e13 = e13 + PWM_1PERCENT;//30
		DB1L;//110
		e13 = e11 + e12; //40
		e13 = e13 + skip_update; //100 <<<<<<<<getting a variable takes much longer than having define !!!!

		DB1L;//180
		DB1H;
		DB1L;
		DB1H;
	}

	if (0){

		float c;
		DB1L;
		c = cos(0.1f);
		DB1H;//60
		c = cos(0.1);
		DB1L;//60
		c = sqrt(0.1f);
		DB1H;//60
		c = pow(0.1f,0.5f);
		DB1L;//60
		c = pow(0.1f,0.5);
		DB1H;//60
		c = sin_lookup[4];
		DB1L;//60
		//c = cos(c);
		DB1H;//10000ns >>>>>>>>>>>>>>> Doing calc on variable as opposed to fix takes forever
		c = sin_lookup[(int)(c*100)];
		DB1L;//300ns
		c = sin_lookup[(int)(c*100.0)];
		DB1H;//1100ns
		c = sin_lookup[(int)(c*100.0f)];
		DB1L;//180ns

		DB1H;

		DB1L;//60
		DB1H;
		DB1L;
		DB1H;
	}

	if (0){
		int ii;
		float ff;

		DB1L;
		ii = 1;
		ii = 2;
		ii = 3;//10
		DB1H;//80

		DB1L;
		for (int i=0; i<3; i++){
			ii = i;
		}
		DB1H;//450     //<<<<<<<<<<<<<<< What is happening here??? loop takes for ever

		DB1L;
		for (float i=0.0f; i<1.0f; i+=0.3f){
			ff = i;
		}
		DB1H;//800

		//	DB1L;
		//	for (float i=0.0f; i<1.0f; i+=0.3f){
		//		ff = sin(i);
		//	}
		//	DB1H;

		DB1L;
		for (int i=0; i<1; i++){
			ff = sin(0.1f);
		}
		DB1H;//400 in next measurement only 250ns --- how can this be faster than the assignment loop - maybe because we always assign same.



		DB1L;
		for (int i=0; i<3; i++){
			ff = sin_lookup[i];
		}
		DB1H;//550  <<<<<<<<<< Lookup is slower than calc --- double check because we do only same calc up there

		DB1L;
		for (int i=0; i<1; i++){
			ff = sin(0.1f*(float)i);
		}
		DB1H;//2000ns !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! for one calc only

		DB1L;
		DB1H;
		DB1L;
		DB1H;
	}


	if (0) {

		DB1L;
		float r = 2345;//30
		float rr = 1234;//30
		DB1H;//100
		float rrr = r+rr;//40

		DB1L;//80
		register float r1 = 2345;//0
		register float rr1 = 1234;//0
		DB1H;//40
		register float rrr1 = r1+rr1;//0 <<<<<<<<<<<<,OK this is not resolvable anymore

		DB1L;//40
		float r2 = 2345;//20
		float rr2 = 1234;//20
		DB1H;//100
		float rrr2 = r2+rr2;//40

		DB1L;//80
		DB1H;
		DB1L;
		DB1H;

	}

	if (1) {

		DB1L;
		bool v = true;//
		int ff = 1;
		DB1H;//100
		if (v){
			int ff = 0;
		}

		DB1L;//80
		int vv = 1;
		int fff = 1;
		DB1H;//40
		fff *= vv;//0 <<<<<<<<<<<<,OK this is not resolvable anymore



		DB1L;//80
		DB1H;
		DB1L;
		DB1H;

	}

	// --- MORE LEARINGS
	// - avoid fmod - 4000ns

	DB1L;
	DB1H;
	DB1L;
	DB1H;

	DB1L;
	DB1H;
	DB1L;
	DB1H;



}

// -----------------------------------------------------------
// --- USEFUL LINES
// -----------------------------------------------------------

// --- read analog
//		uint32_t val_I = HAL_ADCEx_InjectedGetValue (&hadc1, 1);
//					uint32_t val_ASENSE = HAL_ADCEx_InjectedGetValue (&hadc1, 2);
//					uint32_t val_STRAIN0 = HAL_ADCEx_InjectedGetValue (&hadc1, 3); //last number refers to rank
//					uint32_t val_M0_TEMP = HAL_ADCEx_InjectedGetValue (&hadc1, 4);
//
//					uint32_t val_SO1 = HAL_ADCEx_InjectedGetValue (&hadc2, 1);
//					uint32_t val_BSENSE = HAL_ADCEx_InjectedGetValue (&hadc2, 2);
//					uint32_t val_STRAIN1 = HAL_ADCEx_InjectedGetValue (&hadc2, 3);
//					uint32_t val_TEMP = HAL_ADCEx_InjectedGetValue (&hadc2, 4);
//					uint32_t val_VBUS = HAL_ADCEx_InjectedGetValue (&hadc2, 5); //TODO this value is not read out correctly - always comes as 0
//
//					uint32_t val_SO2 = HAL_ADCEx_InjectedGetValue (&hadc3, 1);
//					uint32_t val_CSENSE = HAL_ADCEx_InjectedGetValue (&hadc3, 2);



// -- read angle
		//		if (0){
		//			//			// --- read angle
		//			//uint8_t spi_address_8[2];
		//			//uint8_t angle8[2];
		//			spi_address_8[1]= 0x7F;
		//			spi_address_8[0]= 0xFE;
		//			//address8 = {0xFE, 0x7F};
		//			//address = 0x3FFE | AS_READ ;
		//			delay_SPI();
		//			HAL_GPIO_WritePin(ROT0_nCS_GPIO_Port, ROT0_nCS_Pin, GPIO_PIN_RESET);
		//			HAL_SPI_Transmit(&hspi2, (uint8_t *)spi_address_8, 1, 1);// The HAL function here takes only 8bit only - still the "Size amount of data" is 1 because we set spi to 16 bit in Config
		//			HAL_GPIO_WritePin(ROT0_nCS_GPIO_Port, ROT0_nCS_Pin, GPIO_PIN_SET);
		//			delay_SPI();
		//			HAL_GPIO_WritePin(ROT0_nCS_GPIO_Port, ROT0_nCS_Pin, GPIO_PIN_RESET);
		//			HAL_SPI_Receive(&hspi2, (uint8_t *)&angle8, 1, 1);
		//			HAL_GPIO_WritePin(ROT0_nCS_GPIO_Port, ROT0_nCS_Pin, GPIO_PIN_SET);
		//			angle = (uint16_t) angle8[0] | (uint16_t) angle8[1] << 8U;
		//			angle &= AS_DATA_MASK;
		//		}



// -----------------------------------------------------------
// --- possible enhancements
// -----------------------------------------------------------
//phase_int1000 = EncVal * N_POLES * 1000 /

//uint16_t convertVal2Temp(uint16_t val_T){
//	if (T < 584){
//		return 0 + 10*
//	}
//}

// -----------------------------------------------------------
//--- SERIOUS OUTTAKES
// -----------------------------------------------------------

#if 0

////called every second step of the quadrature encoder was used for pwm update in past
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){ // see https://community.st.com/s/question/0D50X00009XkWUpSAN/encoder-mode-and-rotary-encoder

	if(htim->Instance == TIM8){
	}
}

// --- HEARTBEAT (1ms)  of the microcontroller - was used for position control
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim3){
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc){
	debug2_out_GPIO_Port->BSRR = (uint32_t)debug2_out_Pin;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	debug2_out_GPIO_Port->BSRR = (uint32_t)debug2_out_Pin << 16U;
}

void myDelay(void){
	HAL_Delay(1);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM3){
		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_3);
	}
}


#endif


// -----------------------------------------------------------
//--- OUTTAKES
// -----------------------------------------------------------

//void HAL_TIM_IRQHandler(TIM_HandleTypeDef *htim);
//void HAL_TIM_IRQHandler(TIM_HandleTypeDef *htim);
//void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *htim);

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

//HAL_UART_Transmit_IT(&huart3, buf, strlen((char*)buf)); //WORKS but replaced by DMA below

	//	adcChannel.Channel = ADC_CHANNEL_8;
	//	adcChannel.Rank = 1;
	//	adcChannel.SamplingTime = ADC_SAMPLETIME_15CYCLES;//5mus //ADC_SAMPLETIME_480CYCLES;// 20mus
	//	adcChannel.Offset = 0;
	//	HAL_ADC_ConfigChannel(&hadc2, &adcChannel);

	//	uint32_t g_ADCValue8;
	//	uint32_t g_ADCValue14;
	//	uint32_t g_ADCValue15;
	//	int g_MeasurementNumber;

	//	see https://visualgdb.com/tutorials/arm/stm32/adc/
	//	uint32_t a_val;
	//	a_val = HAL_ADC_GetValue(&hadc2)
	//	HAL_ADC_Start(&hadc2);

		//HAL_CAN_AddTxMessage(&hcan1, &pHeader, &a, &TxMailbox);

		// --- GPIO ----------------------------------------------------
		//HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_4);


//--mybest but still super slow FOC alg
//else if (FOC_ALG == 2){ // this method needs to have trigger when we get to next cycle and needs averaging over a few cycles
//			static float current_phase;
//			static float last_phase;
//			last_phase = current_phase;
//			current_phase = (float) EncVal * 0.0031415f * N_POLES ; //(float) EncVal / ENC_STEPS * 2*PI * N_POLES ; //takes 1500ns
//			current_phase -= phase0;
//			current_phase += 6.28f;
//			current_phase = fmod(current_phase, 6.28f);
//
//			int32_t val_I = HAL_ADCEx_InjectedGetValue (&hadc1, 1);
//
//
//			if (last_phase - current_phase > 1 || last_phase - current_phase < -1){
//				direct_component = direct_component_sum/component_counter;
//				quadrature_component = quadrature_component_sum/component_counter;
//
//
//				float lp = 0.05f;
//				direct_component_lp = (1-lp) * direct_component_lp + lp * direct_component;
//				quadrature_component_lp = (1-lp) * quadrature_component_lp + lp * quadrature_component;
//
//				static float direct_component_lp_integral = 0.0f;
//				direct_component_lp_integral += direct_component_lp;
//
//				FOC_phase_shift = 0.01f* generic_gain * direct_component_lp + 0.00001f  * direct_component_lp_integral; //starts oscillating at I = 0.00008f and alternatively at P = 0.03f
//
//				if (FOC_phase_shift > 0.3f){
//					FOC_phase_shift = 0.3f;
//				}
//				else if (FOC_phase_shift < -0.3f){
//					FOC_phase_shift = -0.3f;
//				}
//
//				if (abs(av_vel) < 1){
//					FOC_phase_shift = 0.0f;
//					//direct_component_integral = 0.0f;
//				}
//
//				direct_component_sum = 0.0f;
//				quadrature_component_sum = 0.0f;
//				component_counter = 0;
//			}
//
//			if (current_phase < 1.571){
//				direct_component_sum += val_I;
//				quadrature_component_sum += val_I;
//			}
//			else if (current_phase < 3.142){
//				direct_component_sum -= val_I;
//				quadrature_component_sum += val_I;
//			}
//			else if (current_phase < 4.712){
//				direct_component_sum -= val_I;
//				quadrature_component_sum -= val_I;
//			}
//			else {
//				direct_component_sum += val_I;
//				quadrature_component_sum -= val_I;
//			}
//			component_counter++;
//
//		}

// --- wrong way of doing FOC
//if (0){ //it is nonsense to have a exp decay window for an oscillatory motion --- this is why this only worked for super slow lp
//	phase = (float) EncVal * 0.0031415 * N_POLES ; //(float) EncVal / ENC_STEPS * 2*PI * N_POLES ; //takes 1500ns
//			phase -= phase0;
//			phase += 6.28;
//			phase = fmod(phase, 6.28);
//	int8_t cos_contribution = 0;
//	int8_t sin_contribution = 0;
//
//	if (phase < 1.571){
//		cos_contribution = +1;
//		sin_contribution = +1;
//	}
//	else if (phase < 3.142){
//		cos_contribution = -1;
//		sin_contribution = +1;
//	}
//	else if (phase < 4.712){
//		cos_contribution = -1;
//		sin_contribution = -1;
//	}
//	else {
//		cos_contribution = +1;
//		sin_contribution = -1;
//	}
//
//	int32_t val_I = HAL_ADCEx_InjectedGetValue (&hadc1, 1);
//
//	//float lp =  1.0f / 40000.0f / 10.0f * av_vel * (float)N_POLES; //making the 1/e time 10 times the pass of a pole
//	//float lp =  0.00003f; // for 2Hz
//	float lp =  0.0003f; // for 2Hz
//	direct_component = (1-lp) * direct_component + lp * (cos_contribution * val_I);
//	quadrature_component = (1-lp) * quadrature_component + lp * (sin_contribution * val_I);
//
//	static float direct_component_integral = 0.0f;
//	direct_component_integral += direct_component;
//
//	FOC_phase_shift = 0.01f* generic_gain * direct_component + 0.00001f  * direct_component_integral; //starts oscillating at I = 0.00008f and alternatively at P = 0.03f
//
//	if (FOC_phase_shift > 0.3f){
//		FOC_phase_shift = 0.3f;
//	}
//	else if (FOC_phase_shift < -0.3f){
//		FOC_phase_shift = -0.3f;
//	}
//
//	if (av_vel < 6 && av_vel > -6){
//		FOC_phase_shift = 0.0f;
//		direct_component_integral = 0.0f;
//	}
//}

// --- another wrong way of doing FOC
// --- phase calc
//		if (val_SO1_buf_index < 72){
//			val_SO1_buf[val_SO1_buf_index] = HAL_ADCEx_InjectedGetValue (&hadc1, 1);
//			val_SO1_buf_index++;
//		}
//		if (val_SO1_buf_index == 72){  // some hints that this takes 10mus
//
//			int32_t cos_part = 0;
//			int32_t sin_part = 0;
//
//			for (int i=0; i< 72; i++){
//					if (i<18){
//						cos_part += val_SO1_buf[i];
//						sin_part += val_SO1_buf[i];}
//					else if (i<36){
//						cos_part -= val_SO1_buf[i];
//						sin_part += val_SO1_buf[i];}
//					else if (i<54){
//						cos_part -= val_SO1_buf[i];
//						sin_part -= val_SO1_buf[i];}
//					else{
//						cos_part += val_SO1_buf[i];
//						sin_part -= val_SO1_buf[i];}
//			}
//			field_amplitude = cos_part*cos_part + sin_part*sin_part;
//			field_phase_shift = (float) cos_part / (float) sin_part;
//			field_phase_shift_pihalf = (float) sin_part / (float) cos_part;
//
//			val_SO1_buf_index++;
//		}

//void append_msg((char*)msg, uint8_t n){
//	if (strlen(buf_msg) + strlen(buf_msgs) < 100){
//		strncat(buf_msgs, buf_msg, n);
//	}
//	else {
//		buf_msgs[0] = '#';
//	}
//}


//void EncoderStepCallback(TIM_HandleTypeDef *htim){
//	if(htim->Instance == TIM8){
//	HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_3);
//	debug1_out_GPIO_Port->BSRR = debug1_out_Pin; //takes 60ns == 5 clock cycles
//				debug1_out_GPIO_Port->BSRR = debug1_out_Pin << 16U; //takes 60ns == 5 clock cycles
//				debug1_out_GPIO_Port->BSRR = debug1_out_Pin; //takes 60ns == 5 clock cycles
//							debug1_out_GPIO_Port->BSRR = debug1_out_Pin << 16U; //takes 60ns == 5 clock cycles
//							debug1_out_GPIO_Port->BSRR = debug1_out_Pin; //takes 60ns == 5 clock cycles
//										debug1_out_GPIO_Port->BSRR = debug1_out_Pin << 16U; //takes 60ns == 5 clock cycles
//
//}}


//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim1){
//
//	debug1_out_GPIO_Port->BSRR = debug1_out_Pin; //takes 60ns == 5 clock cycles
//	debug1_out_GPIO_Port->BSRR = debug1_out_Pin << 16U; //takes 60ns == 5 clock cycles
//}




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
