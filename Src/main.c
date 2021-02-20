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
/*
 * next steps
 * close torque loop
 * perfect stiffness
 * find limits routine --> set to absolute position and limits
 * implement software bumpers
 *
 */
//
// --- code
//
// --- architecture
// - injection triggers sensing that take way too long (8prescalor*4*(3+5+15)cycles)=8mus
// - as above may have adverse effect on force reading
//
// --- features
// - force control
// - more self check
// - CAN comm
// - read IMU
// - program DRV (max current)
//
// --- more
// - (shelved) V sense
//
// --- reliability
// - booting up from switch on without reset
// - (good for now) reliable programming and readout of angle sensor
// - (good for now) reliable play button when writing code
//
// --- design
// - lp of ABCsense with 1mus rise 600kHz may not be ideal - needs to be MUCH slower to average out
// - lp in current sense is different for A than for B and C !
//
// --- further digging
// - I sense is right at end of PWM cycle but I only see B compromised when moving it away from there?



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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/*****************************************************************************
 *                                                                           *
 *    CONTROL PANEL
 *                                                                           *
 *****************************************************************************
 */
#define DB_TIMING 1
#define I_CALIB_ENABLED 1

/*****************************************************************************
 *                                                                           *
 *    SYSTEM CONSTANTS
 *                                                                           *
 *****************************************************************************
 */

// --- ENCODER
#define ENC_STEPS 4000 // edge changes on A and B outputs - Note that ARR in TIM8 needs to be set to ENC_STEPS-1
#define ENC_STEPS_F 4000.0f // edge changes on A and B outputs - Note that ARR in TIM8 needs to be set to ENC_STEPS-1
#define ENC_STEPS_HALF 2000 // to be set equal to  ENC_STEPS / 2
#define ENC_RESOLUTION 16384 // 14 bit resolution for angle reading via SPI
#define ENC_TOLERANCE 2

// --- MOTOR
#define N_PHASES 3

// --- PWM
#define PWM_STEPS 4096
#define PWM_STEPS_F 4096.0f
#define PWM_1PERCENT 41 // set this to 1% of the PWM_STEP
#define AMP_LIMIT 0.95f //0.98f is absolute min so I measurement can work 85 would also free up F measurement - swinging not considered

// --- ADCs
#define ADC1_BUF_LEN 8
#define ADC2_BUF_LEN 4
#define ADC3_BUF_LEN 4

#define RANK_U 1
#define RANK_T 2
#define RANK_F 3
#define RANK_I 4

#define RANK_CONT_U 1
#define RANK_CONT_T 2
#define RANK_CONT_F 3
#define RANK_CONT_I 4
#define RANK_CONT_TMCU 5
#define RANK_CONT_Vref 6
#define RANK_CONT_Vbat 7 // V mcu supply
#define RANK_CONT_Vbus 8 // V actual battery

#define CONVERT_VBUS_INT2V 0.0165f // 603 for 10V

// --- ROUTINE PARAMS
#define ANALOG_SAMPLES_BITSHIFT 5
#define ANALOG_SAMPLES_N 32 // must be 2^ANALOG_SAMPLES_BITSHIFT

#define ANALOG_SAMPLES_AV_BITSHIFT 6
#define ANALOG_SAMPLES_AV_N 64 // must be 2^ANALOG_SAMPLES_AV_BITSHIFT

#define FAST_PER_SLOW 8

#define FOC_PHASE_LIM 0.3f

#define OMEGAENC_MISSING_UPDATE_MAX 100

#define LP_OMEGA_ENC_DOT_CONST 0.5f

#define LP_OMEGA_ENC_CONST 0.5f

//#define LP_TEMP 0.1f

#define I_CALIB_N 128

// ---BUFFERS
#define BUF_LEN 400
#define BUF_ADD_LEN 200

/*****************************************************************************
 *                                                                           *
 *    FOREVER CONSTANTS
 *                                                                           *
 *****************************************************************************
 */
#define PI 3.14159f
#define PI2 6.28318f

#define INV_SQRT_3 0.57735f

#define WANKEL_ADVANTAGE 1.1547f // WANKEL! sqrt(3)/1.5

/*****************************************************************************
 *                                                                           *
 *    MACROS
 *                                                                           *
 *****************************************************************************
 */

#define DB1H debug1_out_GPIO_Port->BSRR = debug1_out_Pin
#define DB1L debug1_out_GPIO_Port->BSRR = debug1_out_Pin << 16U

#define DB2H debug2_out_GPIO_Port->BSRR = debug2_out_Pin;
#define DB2L debug2_out_GPIO_Port->BSRR = debug2_out_Pin << 16U;




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
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;

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
static void MX_TIM2_Init(void);
static void MX_ADC3_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_RTC_Init(void);
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
//void calc_omega(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void update_pwm(void);
void fast_control_task(void);
void slow_control_task(void);
void keyboard_intake(void);
void print_task(void);
void print_prep_task(int fast_control_task_counter);
void timing_party(void);
float omega_division(int32_t delta_EncVal, int32_t delta_t);
int32_t encoder_jump_comp(int32_t delta_EncVal);
float convert2SI_strain(uint32_t strain);




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

////--ESC_ID = 100; // test rig for muscle mp with 5045
//float phase0 = 1.2f;  //$$$$$$$$$$$$$ SPECIFIC $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//int32_t  N_POLES = 7; //<<<see calib routine //7(14 magnets, 12 coils) //21//(42 magnets, 36 coils) //$$$$$$$$$$$$$ SPECIFIC $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//#define CAN_ID 0x100
//int32_t INVERT = 0; //<<<see calib routine
//#define DIFF_FORCE 0
//#define I_LIM 50.0f //amps
//#define I_LIM_MAX_COUNT 100 // max number of encounters in fast loop
//#define VBUS_MAX 30.0f
//#define VBUS_MIN 8.0f

//--ESC_ID = 100; // right biceps 5045
float phase0 = 5.84f;  //$$$$$$$$$$$$$ SPECIFIC $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
int32_t  N_POLES = 7; //<<<see calib routine //7(14 magnets, 12 coils) //21//(42 magnets, 36 coils) //$$$$$$$$$$$$$ SPECIFIC $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
#define CAN_ID 0x100
int32_t INVERT = 0; //<<<see calib routine
#define DIFF_FORCE 0
#define I_LIM 100.0f //amps
#define T_ESC_LIM 1500
#define T_MOTOR_LIM 1500
#define I_LIM_MAX_COUNT 100 // max number of encounters in fast loop
#define T_LIM_MAX_COUNT 100 // max number of encounters in fast loop
#define VBUS_MAX 30.0f
#define VBUS_MIN 8.0f
float STRAIN0_NperINT = 2.0f;
int32_t STRAIN0_OFFSET = 648;

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

/*****************************************************************************
 *                                                                           *
 *    CONTROL PARAMETERS
 *                                                                           *
 *****************************************************************************
 */

enum Current_mode {sinusoidal = 0, trapezoidal = 1, freerun = 2} ;
current_mode = sinusoidal;

enum Control_mode {voltage_control = 0, position_control = 1, force_control = 2};
control_mode = voltage_control;

bool FOC_enabled = false; //<<< choose

bool normal_pwm_update = true;

bool convert2SI = false;

bool print2uart = true;

bool counter0ing_at0crossing = true;

bool sw_enable_pwm = false;

float amp = 0.0f;
float phase_shift = PI/2.0f ;
float stiffness = -0.00; // -0.0024 makes motor invisible

float pos_amp = 100.0f;
float pos_freq = 0.5f;
float pos_amp_limit = 0.2f;
int32_t pos_offset = 0;
float P_gain = 0.0005f;

float generic_fac = 1.0f;
float generic_add = 0.0f;

/*****************************************************************************
 *                                                                           *
 *    SYSTEM VARIABLES
 *                                                                           *
 *****************************************************************************
 */

int32_t init_EncVal;
int32_t rotation_counter = 0;

int32_t Enc_Val_total_lim_m = 0;
int32_t Enc_Val_total_lim_p = 0;

int32_t last_EncVal_pwm;
int32_t last_EncVal_omegaEnc;

int32_t omegaEnc_missing_update_counter = 0; //reseet in Enc interrupt and incremented in fast_task

float omegaEnc = 0.0f;
float lp_omegaEnc = 0.0f;
float omegaEncDot = 0.0;
float lp_omegaEncDot = 0.0f;

uint16_t adc1_buf[ADC1_BUF_LEN];
uint16_t adc2_buf[ADC2_BUF_LEN];
uint16_t adc3_buf[ADC3_BUF_LEN];

int32_t pole_phase_int;

uint32_t acc_STRAIN0 = 0;
uint32_t fast_STRAIN0 = 0;
uint32_t acc_STRAIN1 = 0;
uint32_t fast_STRAIN1 = 0;
uint32_t acc_Vbus = 0;
uint32_t fast_Vbus = 600;
uint32_t acc_ESC_TEMP = 0;
uint32_t av_ESC_TEMP = 0;
uint32_t acc_MOT_TEMP = 0;
uint32_t av_MOT_TEMP = 0;

float acc_I_tot_squared = 0.0f;
float fast_I_tot = 0.0f;
float acc_u0 = 0.0f;
float fast_u0 = 0.0f;

float fast_P_consumed = 0.0f;

float FOC_phase_shift = 0.0f;

uint32_t analog_samples_counter = 0;
uint32_t fast_control_task_counter = 0;
uint32_t slow_control_task_counter = 0;
uint32_t prep_counter = 0;

float A_mean= 2040.0f;
float B_mean= 2004.0f;
float C_mean= 2004.0f;

char ch='.';

int32_t sys_err = 0;

/*****************************************************************************
 *                                                                           *
 *    LOOKUP TABLES
 *                                                                           *
 *****************************************************************************
 */

float lookup[210];
float sin_lookup[628];
float cos_lookup[628];

char buf_msgs[100];
char buf_msg[50];
uint8_t dat_buf[10];
char buf[BUF_LEN + 1];
char buf_add[BUF_ADD_LEN + 1];
//uint8_t plot[300];



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	/*****************************************************************************
	 *****************************************************************************
	 ***                                                                       ***
	 ***    MAIN
	 ***                                                                       ***
	 *****************************************************************************
	 *****************************************************************************
	 */

	// --- STARTING SERIAL OUTPUT IN TERMINAL
	//ls /dev/tty*
	//screen /dev/tty.usbmodem14203 115200          --- stop: control a
	//screen /dev/tty.usbmodem14103 115200          --- stop: control a
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
  MX_TIM2_Init();
  MX_ADC3_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_RTC_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

	/*****************************************************************************
	 *                                                                           *
	 *    LOOKUPS
	 *                                                                           *
	 *****************************************************************************
	 */

	calc_lookup(lookup);
	calc_sin_lookup(sin_lookup);
	calc_cos_lookup(cos_lookup);


	/*****************************************************************************
	 *                                                                           *
	 *    TIMERS
	 *                                                                           *
	 *****************************************************************************
	 */
	HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_Base_Start(&htim5);
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
	HAL_TIM_OC_Start(&htim9, TIM_CHANNEL_2); //seems to work just like pwm above

	/*
	 * TIM1 mosfet control
	 * TIM2 0 time measurement for omega calculation
	 * TIM5 839 => 10mus=100kHz steps position control in fast task
	 * TIM8 encoder
	 * TIM9 LEDs
	 * 1 =  84MHz:   2 3 4 5 6 7           12 13 14
	 * 2 = 168MHz: 1             8 9 10 11
	 */


	/*****************************************************************************
	 *                                                                           *
	 *    LEDS
	 *                                                                           *
	 *****************************************************************************
	 */

	GPIOE->BSRR = GPIO_PIN_3; //switches LD1 on
	GPIOE->BSRR = GPIO_PIN_3 << 16U; //switches LD1 off

	GPIOE->BSRR = GPIO_PIN_4; //switches LD2 on
	GPIOE->BSRR = GPIO_PIN_4 << 16U; //switches LD2 off

	TIM9->CCR1 = 300; //switches LD3 to 30%
	TIM9->CCR2 = 600; //switches LD4 to 60%

	SCB->CPACR |= 0xf00000; //todo understand

	/*****************************************************************************
	 *                                                                           *
	 *    DRV DRIVER
	 *                                                                           *
	 *****************************************************************************
	 */
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);

	HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4);

	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);//todo check OC versus IC

	// --- ENABLE DRV
	HAL_GPIO_WritePin(EN_GATE_GPIO_Port, EN_GATE_Pin, GPIO_PIN_SET);





	/*****************************************************************************
	 *                                                                           *
	 *    IMU
	 *                                                                           *
	 *****************************************************************************
	 */
	//see: https://www.youtube.com/watch?v=isOekyygpR8

	//  dat_buf[0] = 0x6B;
	//  HAL_I2C_Master_Transmit(&hi2c2, IMU_ADDR, dat_buf, 1, HAL_MAX_DELAY);
	//  HAL_I2C_Master_Transmit(&hi2c2, IMU_ADDR, 0x00, 1, HAL_MAX_DELAY);
	//  HAL_Delay(2);

	//b1101000
	HAL_StatusTypeDef ret;

	int16_t accel16;
	uint8_t accel8l;
	uint8_t accel8h;

	char accel_char[20];

	dat_buf[0] = 0x6B; //power register
	dat_buf[1] = 0x00; //switch on
	ret = HAL_I2C_Master_Transmit(&hi2c2, IMU_ADDR, dat_buf, 2, HAL_MAX_DELAY);
	if (ret != HAL_OK){
		strcpy((char*)dat_buf, "Error IMU T\r\n");
	} else {
		dat_buf[0] = 0x00;
	}

	dat_buf[0] = 0x3B;
	ret = HAL_I2C_Master_Transmit(&hi2c2, IMU_ADDR, dat_buf, 1, HAL_MAX_DELAY);
	if (ret != HAL_OK){
		strcpy((char*)dat_buf, "Error IMU T\r\n");
	} else {
		ret = HAL_I2C_Master_Receive(&hi2c2, IMU_ADDR, dat_buf, 1, HAL_MAX_DELAY);
		if (ret != HAL_OK){
			strcpy((char*)dat_buf, "Error IMU R\r\n");
		} else {
			accel8l = (int8_t)dat_buf[0];
			sprintf((char*)accel_char, "%u m\r\n", (int)accel8l);
			//itoa(dat_buf[0], accel_char, 10);
		}

	}

	//  	dat_buf[0] = 0x42;
	//		ret = HAL_I2C_Master_Transmit(&hi2c2, IMU_ADDR, dat_buf, 1, HAL_MAX_DELAY);
	//		if (ret != HAL_OK){
	//			strcpy((char*)dat_buf, "Error IMU T\r\n");
	//		} else {
	//			ret = HAL_I2C_Master_Receive(&hi2c2, IMU_ADDR, dat_buf, 1, HAL_MAX_DELAY);
	//			if (ret != HAL_OK){
	//				strcpy((char*)dat_buf, "Error IMU R\r\n");
	//			} else {
	//				accel8l = (int8_t)dat_buf[0];
	//				//sprintf((char*)dat_buf, "%u m\r\n", (int)accel8l);
	//				//itoa(dat_buf[0], accel_char, 10);
	//			}
	//
	//		}
	//
	//		//who am i WORKS
	//
	//		dat_buf[0] = 0x75;
	//				ret = HAL_I2C_Master_Transmit(&hi2c2, IMU_ADDR, dat_buf, 1, HAL_MAX_DELAY);
	//				if (ret != HAL_OK){
	//					strcpy((char*)dat_buf, "Error IMU T\r\n");
	//				} else {
	//					ret = HAL_I2C_Master_Receive(&hi2c2, IMU_ADDR, dat_buf, 1, HAL_MAX_DELAY);
	//					if (ret != HAL_OK){
	//						strcpy((char*)dat_buf, "Error IMU R\r\n");
	//					} else {
	//						accel8l = (int8_t)dat_buf[0];
	//						//sprintf((char*)dat_buf, "%u m\r\n", (int)accel8l);
	//						//itoa(dat_buf[0], accel_char, 10);
	//					}
	//
	//				}


	/*****************************************************************************
	 *                                                                           *
	 *    ROTATION SENSOR
	 *                                                                           *
	 *****************************************************************************
	 */
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
	init_EncVal = (uint16_t) ((float)angle /16384.0f * ENC_STEPS_F);
	last_EncVal_omegaEnc = init_EncVal;
	last_EncVal_pwm = init_EncVal;
	TIM8->CNT = init_EncVal;


	// --- Calibrate phase0   ------ TODO also this requires that we are not yet sensitive to value change
	//	TIM1->CCR1 = 50;
	//	HAL_Delay(500);
	//	EncVal = TIM8->CNT;//takes 200ns
	//	TIM1->CCR1 = 0;
	//	phase = (float) EncVal * 0.02199f ;

	/*****************************************************************************
	 *                                                                           *
	 *    UART DMA
	 *                                                                           *
	 *****************************************************************************
	 */
	HAL_DMA_RegisterCallback(&hdma_usart3_tx, HAL_DMA_XFER_CPLT_CB_ID, &DMAUSARTTransferComplete);


	/*****************************************************************************
	 *                                                                           *
	 *    ADC DMA and INJECTED
	 *                                                                           *
	 *****************************************************************************
	 */
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_buf, ADC1_BUF_LEN); // the length must be multiple of channels otherwise I observed mess in order - even like 2 of one and lots of mess
	HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_buf, ADC2_BUF_LEN); // TODO enabling this only leads to no change all values stay zero
	HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc3_buf, ADC3_BUF_LEN); // TODO enabling this breaks transmission entirely

	HAL_ADCEx_InjectedStart (&hadc1);
	HAL_ADCEx_InjectedStart (&hadc2);
	HAL_ADCEx_InjectedStart (&hadc3);

	/*****************************************************************************
	 *                                                                           *
	 *    CAN COMMUNICATION
	 *                                                                           *
	 *****************************************************************************
	 */
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


	/*****************************************************************************
	 *                                                                           *
	 *    SYSTEM CALIBRATION
	 *                                                                           *
	 *****************************************************************************
	 */

	sprintf(buf, "\r\n\r\nWELCOME TO MUSCLEmaster \r\n\r\nangle: %d init_EncVal %d \r\nangle: %u EncVal %u \r\n\r\n",
			(int)angle, (int)init_EncVal ,
			(int)angle, (int)init_EncVal );
	huart3.Instance->CR3 |= USART_CR3_DMAT; //enabel dma as we disable in callback so uart can be used for something else
	HAL_DMA_Start_IT(&hdma_usart3_tx, (uint32_t)buf, (uint32_t)&huart3.Instance->DR, strlen(buf));

	HAL_Delay(10); //some delay needed othwise the first print statement in while will overwrite

	// --- find current sense offsets
	//float acc_I_A = 0.0f;
#if I_CALIB_ENABLED
	A_mean = 0;
	B_mean = 0;
	C_mean = 0;
	for (int i=0; i<I_CALIB_N; i++){
		A_mean += HAL_ADCEx_InjectedGetValue (&hadc1, RANK_I);
		B_mean += HAL_ADCEx_InjectedGetValue (&hadc2, RANK_I);
		C_mean += HAL_ADCEx_InjectedGetValue (&hadc3, RANK_I);
		HAL_Delay(1);
	}
	A_mean /= I_CALIB_N;
	B_mean /= I_CALIB_N;
	C_mean /= I_CALIB_N;

	sprintf(buf, "I_mean: %5d %5d %5d \n", (int)A_mean, (int)B_mean, (int)C_mean );
	huart3.Instance->CR3 |= USART_CR3_DMAT; //enabel dma as we disable in callback so uart can be used for something else
	HAL_DMA_Start_IT(&hdma_usart3_tx, (uint32_t)buf, (uint32_t)&huart3.Instance->DR, strlen(buf));
	HAL_Delay(10);
#endif

	/*****************************************************************************
	 *                                                                           *
	 *    SYSTEM CHECK
	 *                                                                           *
	 *****************************************************************************
	 */

	if (adc1_buf[RANK_CONT_Vbus-1]*CONVERT_VBUS_INT2V > VBUS_MAX){
		sys_err |= 1 << 0U;
	}
	if (adc1_buf[RANK_CONT_Vbus-1]*CONVERT_VBUS_INT2V < VBUS_MIN){
		sys_err |= 1 << 1U;
	}

	if (A_mean < 2030.0f || A_mean > 2060.0f ||
			B_mean < 1990.0f || B_mean > 2020.0f ||
			C_mean < 1990.0f || C_mean > 2020.0f ){
		sys_err |= 1 << 2U;
	}

	if (sys_err == 0){
		GPIOE->BSRR = GPIO_PIN_3; //switches LD1 on
	}


	/*****************************************************************************
	 *                                                                           *
	 *    WELCOME
	 *                                                                           *
	 *****************************************************************************
	 */

	sprintf(buf, "\r\n\r\nWELCOME TO MUSCLEmaster \r\n\r\n angle: %d EncVal %d \r\n error %d \r\n",
			(int)angle, (int)init_EncVal ,
			(int)sys_err);
	huart3.Instance->CR3 |= USART_CR3_DMAT; //enabel dma as we disable in callback so uart can be used for something else
	HAL_DMA_Start_IT(&hdma_usart3_tx, (uint32_t)buf, (uint32_t)&huart3.Instance->DR, strlen(buf));

	HAL_Delay(10); //some delay needed othwise the first print statement in while will overwrite


	/*****************************************************************************
	 *                                                                           *
	 *    SYSTEM START
	 *                                                                           *
	 *****************************************************************************
	 */
	playSound( 3, 100, 20);
	playSound( 2, 100, 40);
	playSound( 1, 100, 80);
	HAL_Delay(100); // So the system stops vibrating

	sw_enable_pwm = true;
	amp = 0.01f;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		/*****************************************************************************
		 *****************************************************************************
		 ***                                                                       ***
		 ***    WHILE
		 ***                                                                       ***
		 *****************************************************************************
		 *****************************************************************************
		 */

		// --- calling fast control task
		if (analog_samples_counter >= ANALOG_SAMPLES_N){
			fast_control_task();
			print_prep_task(prep_counter);

			analog_samples_counter = 0;
			fast_control_task_counter ++;
			prep_counter ++;
		}

		// --- calling slow control task
		if(fast_control_task_counter >= FAST_PER_SLOW){
			slow_control_task();

			fast_control_task_counter = 0;
			slow_control_task_counter ++;
		}

		// --- calling print task
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
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 8;
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
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 3;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 4;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 5;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_VBAT;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_11;
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
  sConfigInjected.InjectedChannel = ADC_CHANNEL_5;
  sConfigInjected.InjectedRank = 2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_14;
  sConfigInjected.InjectedRank = 3;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
  sConfigInjected.InjectedRank = 4;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
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
  hadc2.Init.ContinuousConvMode = ENABLE;
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
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 3;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 4;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_12;
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
  sConfigInjected.InjectedChannel = ADC_CHANNEL_4;
  sConfigInjected.InjectedRank = 2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_15;
  sConfigInjected.InjectedRank = 3;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_2;
  sConfigInjected.InjectedRank = 4;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
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
  hadc3.Init.ContinuousConvMode = ENABLE;
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
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
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
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_13;
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
  sConfigInjected.InjectedRank = 2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc3, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
  sConfigInjected.InjectedRank = 3;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc3, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
  */
  sConfigInjected.InjectedRank = 4;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
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
  sConfigOC.Pulse = 4095-1120;
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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;
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
  htim5.Init.Period = 0xFFFFFFFF;
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
  htim9.Init.Prescaler = 41999;
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
  sConfigOC.Pulse = 300;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 600;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

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

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF9_TIM13;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

	normal_pwm_update = false;
	set_pwm_off();
	HAL_Delay(100);

	const uint32_t poles_max = 25;
	uint32_t EncVal[poles_max];
	uint32_t step_through_amp = 5 * PWM_1PERCENT;
	uint8_t ABC = 0;


	// --- DETERMINE INVERT
	for (ABC = 0; ABC < N_PHASES ; ABC++){
		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_4);
		set_pwm_off();
		if (ABC==0){
			TIM1->CCR1 = step_through_amp;
		}
		else if (ABC==1){
			TIM1->CCR2 = step_through_amp;
		}
		else {
			TIM1->CCR3 = step_through_amp;
		}
		HAL_Delay(200);
		EncVal[ABC] = TIM8->CNT;
	}
	int32_t delta_EncVal = EncVal[2] - EncVal[1];
	if (delta_EncVal > ENC_STEPS_HALF) {
		delta_EncVal - ENC_STEPS;
	}
	else if (delta_EncVal < -ENC_STEPS_HALF){
		delta_EncVal + ENC_STEPS;
	}
	if (delta_EncVal < 0){
		INVERT = 1; //SET MOTOR SPECIFIC VALUE
	}
	else{
		INVERT = 0; //SET MOTOR SPECIFIC VALUE
	}

	set_pwm_off();
	TIM1->CCR1 = step_through_amp;
	HAL_Delay(200);

	// --- DETERMINE N_POLES
	uint8_t BCA = 0;
	int32_t counter = 0;
	bool done = false;

	for (int u = 0 ; u < poles_max; u++){
	//while (!done){
		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_4);
		for (BCA = 0; BCA < N_PHASES ; BCA++){
			set_pwm_off();
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_4);
			if (BCA==0){
				if(INVERT){
					TIM1->CCR3 = step_through_amp;
				}
				else {
					TIM1->CCR2 = step_through_amp;
				}
			}
			else if (BCA==1){
				if(INVERT){
					TIM1->CCR2 = step_through_amp;
				}
				else{
					TIM1->CCR3 = step_through_amp;
				}
			}
			else {
				TIM1->CCR1 = step_through_amp;
			}
			HAL_Delay(200);
		}
		EncVal[counter] = TIM8->CNT;


		int32_t low_bracket = EncVal[counter] - 50; // note can be negative //todo still a problem when it's right around 0!
		int32_t high_bracket = EncVal[counter] + 50; // note can be >ENC_STEPS


		char local_buf[300];
		local_buf[0] = '\0';
		sprintf(buf_msg, "[step_through_pole_angles] array: %d counter: %d Enc: %d \r\n", (int)EncVal[counter], (int)counter, (int)TIM8->CNT);
		if (strlen(buf_msg) + strlen(buf_msgs) < 100){
			strcat(buf_msgs, buf_msg);
		}
		else {
			buf_msgs[0] = '#';
		}
		if (buf_msgs[0] != '\0'){
			strcat(local_buf, buf_msgs);
			buf_msgs[0] = '\0';
		}
		//HAL_UART_Transmit_IT(&huart3, local_buf, strlen((char*)local_buf)); //WORKS but replaced by DMA below
		huart3.Instance->CR3 |= USART_CR3_DMAT; //enabel dma as we disable in callback so uart can be used for something else
		HAL_DMA_Start_IT(&hdma_usart3_tx, (uint32_t)local_buf, (uint32_t)&huart3.Instance->DR, strlen(local_buf));


		if (counter > 0 && high_bracket > EncVal[0] && low_bracket < EncVal[0]){
			N_POLES = counter ; //SET MOTOR SPECIFIC VALUE
			//done = true;
			break;
		}
		counter ++;
	}
	set_pwm_off();

	// --- DETERMINE phase0
	int32_t ENCpPOLE_1000 = ((int32_t)ENC_STEPS * 1000) / (int32_t)N_POLES;
	int32_t acc_Enc0_1000 = 0;
 	for (int i = 0; i < N_POLES ; i++){
		acc_Enc0_1000 += ((int32_t)EncVal[i]*1000) % ENCpPOLE_1000;
	}
	phase0 = (float) acc_Enc0_1000 / 1000.0f / (float)ENC_STEPS * PI2; //SET MOTOR SPECIFIC VALUE // * (float) N_POLES/ (float) N_POLES cancels out
	//todo this calculation gets into trouble when phase shift is close to 0 because the mod operation may give vastly different results !!!

	char local_buf[300];
	local_buf[0] = '\0';
	sprintf(buf_msg, "MOTOR CHARACTERISTIC: INVERT: %d N_POLES: %d phase0: %d \r\n", (int)INVERT, (int)N_POLES, (int)(phase0*100.0f));
	if (strlen(buf_msg) + strlen(buf_msgs) < 100){
		strcat(buf_msgs, buf_msg);
	}
	else {
		buf_msgs[0] = '#';
	}
	if (buf_msgs[0] != '\0'){
		strcat(local_buf, buf_msgs);
		buf_msgs[0] = '\0';
	}
	//HAL_UART_Transmit_IT(&huart3, local_buf, strlen((char*)local_buf)); //WORKS but replaced by DMA below
	huart3.Instance->CR3 |= USART_CR3_DMAT; //enabel dma as we disable in callback so uart can be used for something else
	HAL_DMA_Start_IT(&hdma_usart3_tx, (uint32_t)local_buf, (uint32_t)&huart3.Instance->DR, strlen(local_buf));
	HAL_Delay(200);


	set_pwm_off();
	normal_pwm_update = true;


}

void step_through_pwm_percent(void){
	uint16_t pole_angle_by_amp[20];
	normal_pwm_update = false;
	set_pwm_off();
	HAL_Delay(100);
	for (uint8_t percent = 0; percent < 10 ; percent++){
		TIM1->CCR1 = percent * PWM_1PERCENT;
		HAL_Delay(200);
		pole_angle_by_amp[percent]=TIM8->CNT;
	}
	set_pwm_off();
	normal_pwm_update = true;
}

void explore_limits(void){
	amp = 0;
	HAL_Delay(100);
	for (int8_t dir=-1;dir<2; dir+=2){
		HAL_Delay(500);
		amp= dir * 0.1f;
		for (int32_t i = 0; i<50; i++){
			HAL_Delay(100);
			uint32_t val_I = HAL_ADCEx_InjectedGetValue (&hadc1, RANK_I);
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
	normal_pwm_update = false;
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
	normal_pwm_update = true;

	//HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
}


void calc_lookup(float *lookup){
	for (int i=0; i<210; i++){
		// --- vanilla
		lookup[i] = 0.5773f * (cos((float)i/100.0f) + cos((float)i/100.0f-1.047f));

		// --- harmonic
		//float amp_harmonic = 1.0f;
		//lookup[i] = 0.5773f * (cos((float)i/100.0f)       + amp_harmonic * cos( (float)i/100.0f       * 3.0f)    +  cos((float)i/100.0f-1.047f) + amp_harmonic * cos(((float)i/100.0f-1.047f)* 3.0f)) ;// the harmonic tends to fully cancel out

		// --- power law
		//lookup[i] = 0.5773f * (pow( cos((float)i/100.0f) + cos((float)i/100.0f-1.047f),amp_harmonic)/ pow(amp_harmonic,0.5f)); //looks like 1.0 is already best in terms of overtones
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
		//val_SO1_buf_index = 0;

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

/*********************************************************************************************
 *
 *  ###  #       #  ##   ##
 *  #  # #       #  # # # #
 *  ###  #   #   #  #  #  #
 *  #     # # # #   #     #
 *  #      #   #    #     #
 *
 *********************************************************************************************
 */


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
	register int32_t EncVal = TIM8->CNT;
	register int32_t delta_EncVal = (int32_t)EncVal - (int32_t)last_EncVal_pwm;
	last_EncVal_pwm = EncVal;




	// --- determine whether 0 crossing happened and adjust rotation_counter accordingly
	if (delta_EncVal > ENC_STEPS_HALF) {
		rotation_counter--;
	}
	else if (delta_EncVal < -ENC_STEPS_HALF){
		rotation_counter++;
	}// both statements 300ns



	// --- calculate the phase with respect to a pole cycle in 100x int
	pole_phase_int = (int)((PI2 * N_POLES / ENC_STEPS * (float) EncVal - phase0 + PI2) * 100.0f) % 628 ; //400ns when consolidated in one line


	register int32_t A = HAL_ADCEx_InjectedGetValue (&hadc1, RANK_I);//500ns
	register int32_t B = HAL_ADCEx_InjectedGetValue (&hadc2, RANK_I);//500ns
	register int32_t C = HAL_ADCEx_InjectedGetValue (&hadc3, RANK_I);//500ns

	register float I_A = ((float)A - A_mean) * 0.134f;
	register float I_B = ((float)B - B_mean) * 0.189f; // 3.3[V]/4095[ticks] /20[gain]/0.0003[ohm] = 0.134 //TODO verify SPI setting in DRV8301 the factor sqrt(2) comes out of thin air
	register float I_C = ((float)C - C_mean) * 0.189f;



	// --- accumulate analog readings till we have enough samples which is a flag for the heart beat (= all MCU internal control loops)
	if (analog_samples_counter < ANALOG_SAMPLES_N ){  // TODO: if n_samples >= 32
		acc_I_tot_squared += (I_A * I_A + I_B * I_B + I_C * I_C) / 1.5f; //todo check 1.5
		acc_STRAIN0 += HAL_ADCEx_InjectedGetValue (&hadc1, RANK_F);
		acc_Vbus += adc1_buf[RANK_CONT_Vbus-1];
#if DIFF_FORCE
		acc_STRAIN1 += HAL_ADCEx_InjectedGetValue (&hadc2, RANK_F);
#endif
		analog_samples_counter ++;
	}//200ns when not entering presumably

	if (FOC_enabled){

		register float a;
		register float b;

		register float direct_component = 0.0f;
		register float quadrature_component = 0.0f;

		static float direct_component_lp = 0.0f;
		static float quadrature_component_lp = 0.0f;

		//	register float mean_lp = 0.00001f;
		//      A_mean = (1-mean_lp) * A_mean + mean_lp * A;
		//			B_mean = (1-mean_lp) * B_mean + mean_lp * B;
		//	A_mean = 2040.0f;
		//	B_mean = 2005.0f;
		//	C_mean = 2005.0f;

		// --- Park transform
		//a = 0.7f * ((float)A-A_mean);
		//b = INV_SQRT_3 * (a + 2.0f * ((float)B-B_mean)); //200ns thanks to precalc of SQRT
		a = I_B; // a and b derived from B and C since they have same DAC (A is on external DAC which may behave differently -- adjust phaseshift accordingly!)
		b = INV_SQRT_3 * (a + 2.0f * I_C); //200ns thanks to precalc of SQRT

		// -- Clark transform
		register uint32_t poleB_phase_int = (pole_phase_int - 209 + 628) % 628; //
		direct_component = a * cos_lookup[poleB_phase_int] + b * sin_lookup[poleB_phase_int];
		quadrature_component = -a * sin_lookup[poleB_phase_int] + b * cos_lookup[poleB_phase_int]; //300ns

		// --- low pass filter
		register float lp = 0.001f;
		direct_component_lp = (1-lp) * direct_component_lp + lp * direct_component;
		quadrature_component_lp = (1-lp) * quadrature_component_lp + lp * quadrature_component;//with register 240 without register 380ns for the 3 lines


		static float direct_component_lp_integral = 0.0f;
		direct_component_lp_integral += direct_component_lp;//150ns for 2lines

		register float direct_component_lp_integral_max = 0.4f / 0.00001f;
		if (direct_component_lp_integral > direct_component_lp_integral_max){
			direct_component_lp_integral = direct_component_lp_integral_max;
		}
		if (direct_component_lp_integral < -direct_component_lp_integral_max){
			direct_component_lp_integral = -direct_component_lp_integral_max;
		}


		// --- PI controller
		FOC_phase_shift = 0.005f * direct_component_lp + 0.00001f  * direct_component_lp_integral; //220ns//starts oscillating at I = 0.00008f and alternatively at P = 0.03f


		if (FOC_phase_shift > FOC_PHASE_LIM){
			FOC_phase_shift = FOC_PHASE_LIM;
		}
		else if (FOC_phase_shift < -FOC_PHASE_LIM){
			FOC_phase_shift = -FOC_PHASE_LIM;
		}//350ns for checks

		//	if (abs(lp_omega) < 1.0f){
		//		FOC_phase_shift = 0.0f;
		//		direct_component_lp_integral = 0.0f;
		//	}//220ns
	}
	else {
		FOC_phase_shift = 0.1f;//empirical good mean of correction
	}


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
	register int32_t field_phase_int = 0;

	// --- stiffness motor
	register float u0 = amp + stiffness * omegaEnc / (float)fast_Vbus * 603.0f ;// * direction; // TODO the abs allows same stiffness to make it softer for both directions - without a signchange is needed BUT turnaround is super aggressive now :( SAME issue with direction - super forceful reverse but sign identical --- looks like v needs to direct also the phase !!!!

	// -- higher order terms
	//register float modified_amp = amp + stiffness * omega + generic_add * 0.000000001f * omega * omega * omega;//

	// -- invisible motor
	//register float modified_amp = amp + stiffness * lp_omegaEnc + lp_omegaEncDot * 0.00003f * 0.1f * generic_add;// * direction; // TODO the abs allows same stiffness to make it softer for both directions - without a signchange is needed BUT turnaround is super aggressive now :( SAME issue with direction - super forceful reverse but sign identical --- looks like v needs to direct also the phase !!!!
	//AMAZING invisible motor kind of works at generic_add = -8 BUT super unstable at <-9....with both in lp 0.1 it is stable all the way to -25

	// --- signed u0 becomes abs(u0) and direction is encoded in field_phase_int
	if (u0 > 0){
		field_phase_int = pole_phase_int - (int32_t)((phase_shift + FOC_phase_shift) * 100.0f);
	}
	else {
		field_phase_int = pole_phase_int + (int32_t)((phase_shift + FOC_phase_shift) * 100.0f);
		u0 = -u0;
	}

	// --- clamp u0
	if (u0 > AMP_LIMIT){
		u0 = AMP_LIMIT;
	}

	if (!sw_enable_pwm){
		u0 = 0;
	}

	acc_u0 += u0;

	if (field_phase_int < 0) {
		field_phase_int += 628;
	}
	else if (field_phase_int >= 628) {
		field_phase_int -= 628;
	}//150ns

	register float uA = 0.0f;
	register float uB = 0.0f;
	register float uC = 0.0f;

	register uint16_t pwmA = 0;
	register uint16_t pwmB = 0;
	register uint16_t pwmC = 0;

	if (!normal_pwm_update){
#if DB_TIMING
		DB1L;
#endif
		return;
	}

	if (current_mode != freerun ){
		if (current_mode == sinusoidal ){

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

		else if (current_mode == trapezoidal){
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

		pwmA = (uint16_t) (PWM_STEPS_F * u0 * uA); //180ns
		pwmB = (uint16_t) (PWM_STEPS_F * u0 * uB); //180ns
		pwmC = (uint16_t) (PWM_STEPS_F * u0 * uC); //180ns

		// --- send out PWM pulses 0...2048

		TIM1->CCR1 = pwmA; //takes<150ns
		if (INVERT){
			TIM1->CCR3 = pwmB; //takes<150ns
			TIM1->CCR2 = pwmC; //takes<150ns
		}
		else {
			TIM1->CCR2 = pwmB; //takes<150ns
			TIM1->CCR3 = pwmC; //takes<150ns
		}//300ns for both
	}

	else{ // NOTE this mode is still experimental
		if  (field_phase_int < 105)	{
			uA = 1;
			pwmA = (uint16_t) (PWM_STEPS_F * u0 * uA); //takes<2s00ns
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
			pwmB = (uint16_t) (PWM_STEPS_F * u0 * uB); //takes<2s00ns
			TIM1->CCR2 = pwmB; //takes<150ns

			CLEAR_BIT(TIM1->CCMR1, TIM_CR2_OIS1N);
			SET_BIT(TIM1->CCMR1, TIM_CR2_OIS3N);
		}
		else if  (field_phase_int < 315)	{
			uB = 1;
			pwmB = (uint16_t) (PWM_STEPS_F * u0 * uB); //takes<2s00ns
			TIM1->CCR2 = pwmB; //takes<150ns

			SET_BIT(TIM1->CCMR1, TIM_CR2_OIS1N);
			CLEAR_BIT(TIM1->CCMR1, TIM_CR2_OIS3N);
		}
		else if  (field_phase_int < 420)	{
			uC = 1;
			pwmC = (uint16_t) (PWM_STEPS_F * u0 * uC); //takes<2s00ns
			TIM1->CCR3 = pwmC; //takes<150ns

			SET_BIT(TIM1->CCMR1, TIM_CR2_OIS1N);
			CLEAR_BIT(TIM1->CCMR1, TIM_CR2_OIS2N);
		}
		else if  (field_phase_int < 525)	{
			uC = 1;
			pwmC = (uint16_t) (PWM_STEPS_F * u0 * uC); //takes<2s00ns
			TIM1->CCR3 = pwmC; //takes<150ns

			CLEAR_BIT(TIM1->CCMR1, TIM_CR2_OIS1N);
			SET_BIT(TIM1->CCMR1, TIM_CR2_OIS2N);
		}
		else 	{
			uA = 1;
			pwmA = (uint16_t) (PWM_STEPS_F * u0 * uA); //takes<2s00ns
			TIM1->CCR1 = pwmA; //takes<150ns

			SET_BIT(TIM1->CCMR1, TIM_CR2_OIS2N);
			CLEAR_BIT(TIM1->CCMR1, TIM_CR2_OIS3N);
		}

	}

#if DB_TIMING
	DB1L;
#endif

}

////called every second step of the quadrature encoder was used for pwm update in past

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){ // see https://community.st.com/s/question/0D50X00009XkWUpSAN/encoder-mode-and-rotary-encoder

	if(htim->Instance == TIM8){
		omegaEnc_missing_update_counter = 0;

		static float omegaEnc_last = 0.0f;
		static uint32_t last_t = 0;
		register uint32_t t_now = TIM2->CNT;
		register int32_t delta_t;
		if (t_now > last_t){
			delta_t = t_now - last_t;
		}
		else {
			delta_t = t_now - last_t; //todo correct statement
		}

		if (delta_t > 84000){
			register int32_t EncVal = TIM8->CNT;
			register int32_t delta_EncVal = (int32_t)EncVal - (int32_t)last_EncVal_omegaEnc;
			last_EncVal_omegaEnc = EncVal;
			last_t = t_now;

			delta_EncVal = encoder_jump_comp(delta_EncVal);

			omegaEnc =  omega_division(delta_EncVal, delta_t);

			lp_omegaEnc = (1.0f - LP_OMEGA_ENC_CONST) * lp_omegaEnc + LP_OMEGA_ENC_CONST * omegaEnc;

			omegaEncDot = (omegaEnc - omegaEnc_last) / (float)delta_t * 84000000.0f;

			lp_omegaEncDot = (1.0f - LP_OMEGA_ENC_DOT_CONST) * lp_omegaEncDot + LP_OMEGA_ENC_DOT_CONST * omegaEncDot;
			//alternative todo calc from t directly to save compute

			omegaEnc_last = omegaEnc;
			// todo must be set to 0 if not updated for long otherwise it maintains value from last update forever
		}

	}
}


float omega_division(int32_t delta_EncVal, int32_t delta_t){
	return PI2 * 84000000.0f / ENC_STEPS_F * (float)(delta_EncVal) / (float)delta_t;
}

int32_t encoder_jump_comp(int32_t delta_EncVal){
	if (delta_EncVal > ENC_STEPS_HALF){ // if jump is more than a half rotation it's most likely the 0 crossing
		return delta_EncVal - ENC_STEPS;
	}
	else if (delta_EncVal < -ENC_STEPS_HALF){
		return delta_EncVal + ENC_STEPS;
	}
	else {
		return delta_EncVal;
	}
}

/*********************************************************************************************
 *
 *  ###  ###  ###
 *  #  #  #   #  #
 *  ###   #   #  #
 *  #     #   #  #
 *  #    ###  ###
 *
 *********************************************************************************************
 */


void fast_control_task(void){
#if DB_TIMING
	DB2H;
#endif

	// --- averaging the quantities read in pwm update
	fast_STRAIN0 = acc_STRAIN0 >> ANALOG_SAMPLES_BITSHIFT;
	acc_STRAIN0 = 0;
	fast_Vbus = acc_Vbus >> ANALOG_SAMPLES_BITSHIFT;
	acc_Vbus = 0;
#if DIFF_FORCE
	fast_STRAIN1 = acc_STRAIN1 >> ANALOG_SAMPLES_BITSHIFT;
	acc_STRAIN1 = 0;
#endif

	fast_I_tot = sqrt(acc_I_tot_squared/(float)ANALOG_SAMPLES_N);
	acc_I_tot_squared = 0.0f;
	fast_u0 = acc_u0 / (float)ANALOG_SAMPLES_N;
	acc_u0 = 0.0f;

	// --- deriving quantities
	fast_P_consumed = fast_I_tot * fast_u0 * (float)fast_Vbus * CONVERT_VBUS_INT2V  / WANKEL_ADVANTAGE;

	// sw current limit switch off
	static uint32_t I_lim_exceeded_counter = 0;
	static uint32_t T_ESC_lim_exceeded_counter = 0;
	static uint32_t T_MOTOR_lim_exceeded_counter = 0;
	if (fast_I_tot > I_LIM){
		I_lim_exceeded_counter++;
	}
	else {
		I_lim_exceeded_counter = 0;
	}
	if (I_lim_exceeded_counter > I_LIM_MAX_COUNT){
		sw_enable_pwm = false;
	}
	if (av_ESC_TEMP > T_ESC_LIM){
		T_ESC_lim_exceeded_counter++;
	}
	else {
		T_ESC_lim_exceeded_counter = 0;
	}
	if (T_ESC_lim_exceeded_counter > T_LIM_MAX_COUNT){
		sw_enable_pwm = false;
	}
	if (av_MOT_TEMP > T_MOTOR_LIM){
		T_MOTOR_lim_exceeded_counter++;
	}
	else {
		T_MOTOR_lim_exceeded_counter = 0;
	}
	if (T_MOTOR_lim_exceeded_counter > T_LIM_MAX_COUNT){
		sw_enable_pwm = false;
	}

	//lp_ESC_TEMP = (uint32_t)((1.0f - LP_TEMP) * (float)lp_ESC_TEMP + LP_TEMP * (float)HAL_ADCEx_InjectedGetValue (&hadc1, RANK_T));
	//lp_MOT_TEMP = (uint32_t)((1.0f - LP_TEMP) * (float)lp_MOT_TEMP + LP_TEMP * (float)HAL_ADCEx_InjectedGetValue (&hadc2, RANK_T));

	//lp_ESC_TEMP = (1.0f - LP_TEMP) * lp_ESC_TEMP + LP_TEMP * (float)HAL_ADCEx_InjectedGetValue (&hadc1, RANK_T);
	//lp_MOT_TEMP = (1.0f - LP_TEMP) * lp_MOT_TEMP + LP_TEMP * (float)HAL_ADCEx_InjectedGetValue (&hadc2, RANK_T);

	static uint32_t analog_samples_av_counter = 0;
	if (analog_samples_av_counter < ANALOG_SAMPLES_AV_N){
		acc_ESC_TEMP += HAL_ADCEx_InjectedGetValue (&hadc1, RANK_T);
		acc_MOT_TEMP += HAL_ADCEx_InjectedGetValue (&hadc2, RANK_T);
		analog_samples_av_counter++;
	}
	else {
		av_ESC_TEMP = acc_ESC_TEMP >> ANALOG_SAMPLES_AV_BITSHIFT;
		acc_ESC_TEMP = 0;
		av_MOT_TEMP = acc_MOT_TEMP >> ANALOG_SAMPLES_AV_BITSHIFT;
		acc_MOT_TEMP = 0;
		analog_samples_av_counter = 0;
	}

	//calc_omega();

	// --- correct for missing update in omecaEnc for very small omega
	if (omegaEnc_missing_update_counter > OMEGAENC_MISSING_UPDATE_MAX){
		omegaEnc = 0.0f;
		omegaEncDot = 0.0f;
	}
	omegaEnc_missing_update_counter ++;

	static uint32_t last_tim5_cnt = 0 ;
	if (control_mode == position_control){

		float t = (float)((TIM5->CNT - last_tim5_cnt) ) / 100000.0f;

		int32_t desired_EncVal = pos_offset + pos_amp * sin(6.28f * pos_freq * t);

		//int32_t desired_EncVal = 0;//TIM5->CNT / 100;

		//		if ((TIM5->CNT/50000)%2 == 1){
		//			desired_EncVal = 10000;
		//		}
		//		else{
		//			desired_EncVal = 0;
		//		}


		int32_t Enc_Val_total = (int32_t)TIM8->CNT + rotation_counter * ENC_STEPS;
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
			sprintf(buf_msg, "[HEART] raw_a: %d %d %d Enc_tot: %d a: %d f: %d lim: %d off: %d g: %d\r\n",
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
#if DB_TIMING
	DB2L;
#endif
}

void slow_control_task(void){
	//can communication update

}

float convert2SI_strain0(uint32_t strain){
	return (float) ((int32_t)strain - STRAIN0_OFFSET) * STRAIN0_NperINT;
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
		stiffness += 0.0001f;
		break;
	case 'j':
		stiffness -= 0.0001f;
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
		control_mode = position_control;
		break;
	case 'k':
		control_mode = voltage_control;
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
		if (convert2SI){
			convert2SI = false;
		}
		else {
			convert2SI = true;
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
		current_mode = sinusoidal;
		break;
	case 'G':
		current_mode = trapezoidal;
		break;
	case 'B':
		current_mode = freerun;
		break;
	case 'L':
		explore_limits();
		break;

	case 'm':
		generic_fac *= 2.0f;
		break;
	case 'n':
		generic_fac *= 0.5f;
		break;
	case 'v':
		generic_add += 1.0f;
		break;
	case 'b':
		generic_add -= 1.0f;
		break;
	case 'Z':
		FOC_enabled = !FOC_enabled;
		break;


	default:
		ch='.';
	}//for case 200ns

}

void print_prep_task(int fast_control_task_counter){
#if DB_TIMING
	DB2H;
	DB2L;
	DB2H;
#endif

	int pos = strlen(buf);
	int left  = BUF_LEN - pos;
	int nn;
#define ADD_VAL(fmt, val)                    \
		nn = snprintf(buf+pos, left, fmt, val);  \
		pos += nn;                               \
		left -= nn;


	switch (fast_control_task_counter){
	case 0:
		sprintf(buf, "tx: %c %4d %4d %4d %4d ", ch, tx_msg[0],rx_msg[1],rx_msg[2],rx_msg[3]);//70000ns
		break;
	case 1:
		//ADD_VAL(" a:%5d", (int)HAL_ADCEx_InjectedGetValue (&hadc1, RANK_U));
		//ADD_VAL(" b:%5d", (int)HAL_ADCEx_InjectedGetValue (&hadc2, RANK_U));
		//ADD_VAL(" c:%5d", (int)HAL_ADCEx_InjectedGetValue (&hadc3, RANK_U));
		//ADD_VAL(" p0:%4.2f", phase0);
		//ADD_VAL(" ps:%4.2f", phase_shift);
		break;
	case 2:
		//ADD_VAL(" TM:%4d", (int) av_MOT_TEMP);
		ADD_VAL(" ENC:%4d", (int) TIM8->CNT);
		ADD_VAL(" pi:%4d", pole_phase_int);
		break;
	case 3:
		//ADD_VAL(" a:%4.2f", amp);
		//ADD_VAL(" a:%5d", (int)(amp*100.0f));
		//ADD_VAL(" A:%5d", (int)HAL_ADCEx_InjectedGetValue (&hadc1, RANK_I)- (int)A_mean);
		//ADD_VAL(" B:%5d", (int)HAL_ADCEx_InjectedGetValue (&hadc2, RANK_I)- (int)B_mean);
		//ADD_VAL(" C:%5d", (int)HAL_ADCEx_InjectedGetValue (&hadc3, RANK_I)- (int)C_mean);
		ADD_VAL(" P:%4d", (int)N_POLES);
		ADD_VAL(" I:%4d", (int)INVERT);
		ADD_VAL(" p0:%4d", (int)(phase0*100.0f));
		break;
	case 4:
		//ADD_VAL(" v:%6.2f", omega);
		ADD_VAL(" oE:%5d", (int)(omegaEnc*100.0f));
		ADD_VAL(" od:%5d", (int)(omegaEncDot*1.0f));

		//sprintf(buf_add, " E:%5d", (int)TIM8->CNT); strcat(buf, buf_add);
		break;
	case 5:
		//ADD_VAL(" c:%5d", (int)rotation_counter);
		ADD_VAL(" s:%5d", (int)(stiffness*10000.0f));
		break;
	case 6:
		break;
	case 7:
		//ADD_VAL(" ADC:%5d", HAL_ADC_GetValue(&hadc1));
		ADD_VAL(" T_MCU:%5d", adc1_buf[RANK_CONT_TMCU-1]);
		//ADD_VAL(" Vref:%5d", adc1_buf[RANK_CONT_Vref-1]);
		//ADD_VAL(" Vbat:%5d", adc1_buf[RANK_CONT_Vbat-1]);
		ADD_VAL(" Vbus:%5d", adc1_buf[RANK_CONT_Vbus-1]);
		ADD_VAL(" fVbus:%5d",  (int)fast_Vbus);



		break;
	case 8:
		//ADD_VAL(" dc:%4d", (int)(direct_component*100.0f));
		//ADD_VAL(" qc:%4d", (int)(quadrature_component*100.0f));
		ADD_VAL(" FOC:%4d", (int)(FOC_phase_shift*100.0f));
		break;
	case 9:
		ADD_VAL(" gf:%4d", (int)(generic_fac*100.0f));
		break;
	case 10:
		ADD_VAL(" ga:%4d", (int) generic_add);
		break;
	case 11:
		ADD_VAL(" P:%5d", (int) (fast_P_consumed*100.0f));

		if (convert2SI){
			//float SO0 = ((float)val_I - 2040.0f) * 0.134f; // 3.3[V]/4095[ticks] /20[gain]/0.0003[ohm] = 0.134
			//float SO1 = ((float)val_SO1 - 2002.0f) * 0.189f; // 3.3[V]/4095[ticks] /20[gain]/0.0003[ohm] = 0.134 //TODO verify SPI setting in DRV8301 the factor sqrt(2) comes out of thin air
			//float SO2 = ((float)val_SO2 - 2002.0f) * 0.189f; // 3.3[V]/4095[ticks] /20[gain]/0.0003[ohm] = 0.134
			//sprintf(buf_add, " I:%5.2fA SO1:%5.2fA SO2:%5.2fA", SO0, SO1, SO2); strcat(buf, buf_add);

			//float I_tot = sqrt((SO0*SO0 + SO1*SO1 + SO2*SO2)/1.5f); //see colab - the factor 1.5 allows to extract the distance from center of triangle to tip
			//sprintf(buf_add, " It:%5.2fA", I_tot); strcat(buf, buf_add);
		}
		else{
			//sprintf(buf_add, " I:%4d SO1:%4d SO2:%4d", val_I, val_SO1, val_SO2); strcat(buf, buf_add);
		}
		break;
	case 12:
		ADD_VAL(" TM:%4d", (int) av_MOT_TEMP);
		break;
	case 13:
		ADD_VAL(" TE:%4d", (int) av_ESC_TEMP);
		break;
	case 14:
		ADD_VAL(" It:%4d", (int) (fast_I_tot*100.0f));
		break;
	case 15:
		ADD_VAL(" F0:%4dN", (int) (convert2SI_strain0(fast_STRAIN0)));
		break;
	case 16:
		ADD_VAL(" ps:%4.2f", phase_shift);
		break;




	case 19:
		if (av_ESC_TEMP > 1900){
			sprintf(buf_add, "* >50C on ESC"); strcat(buf, buf_add);
		}

		if (av_MOT_TEMP > 1900){
			sprintf(buf_add, "* >50C on MOTOR"); strcat(buf, buf_add);
		}

		if (convert2SI_strain0(fast_STRAIN0) < -100){
			sprintf(buf_add, "* -100N force"); strcat(buf, buf_add);
		}
		if (convert2SI_strain0(fast_STRAIN0) > 100){
			sprintf(buf_add, "* 100N force"); strcat(buf, buf_add);
		}
		break;

	case 20:
		sprintf(buf_add, " \r\n"); strcat(buf, buf_add);
		break;
	}

#if 0

	sprintf(buf_add, " dcl:%6.2f qcl:%6.2f", direct_component_lp, quadrature_component_lp); strcat(buf, buf_add);

	sprintf((char*)buf_add, " dc:%6.2f qc:%6.2f", direct_component, quadrature_component); strcat(buf, buf_add);

	sprintf((char*)buf_add, " a:%6.2f b:%6.2f", a, b); strcat(buf, buf_add);

	//sprintf((char*)buf_add, " s:%4.3f", stiffness); strcat(buf, buf_add);

	//sprintf((char*)buf_add, " h:%4.3f", amp_harmonic); strcat(buf, buf_add);

	//sprintf((char*)buf_add, " d:%2d", direction); strcat(buf, buf_add);

	sprintf((char*)buf_add, " A:%4d B:%4d C:%4d", val_ASENSE, val_BSENSE, val_CSENSE); strcat(buf, buf_add);

	if (convert2SI){
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
	//								//(int)(stiffness*1000), (int)(1000*lp_omega),
	//								val_I, val_ASENSE, val_STRAIN0, val_M0_TEMP,
	//								adc1_buf[0], adc1_buf[1], adc1_buf[2], adc1_buf[3], adc1_buf[4]);
	//								//val_SO1, val_BSENSE, val_STRAIN1, val_TEMP, val_SO2, val_CSENSE); //        %d %d %d %d A2 %d %d


	//			buf[150] = '|';
	//			buf[100] = '.';
	//			buf[50] = '|';
	//			buf[100 + max(-50, min(50, (int)lp_omega))] = 'v';

	//2L //1ms = 1000000ns
#endif

#if DB_TIMING
	DB2L;
	DB2H;
	DB2L;
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
