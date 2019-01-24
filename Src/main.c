/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2018 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "stm32xxx_hal.h"

#include "vl53l1_api.h"
#include "X-NUCLEO-53L1A1.h"

#include "motor_control.h"

#include "app_mems-library.h"
#include "DemoSerial.h"
#include "MotionFX_Manager.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ALGO_FREQ    100U /* Algorithm frequency [Hz] */
#define ALGO_PERIOD  10   /* Algorithm period [ms] */
#define MOTIONFX_ENGINE_DELTATIME  0.01f

#define FROM_MG_TO_G                    0.001f
#define FROM_G_TO_MG                    1000.0f
#define FROM_MDPS_TO_DPS                0.001f
#define FROM_DPS_TO_MDPS                1000.0f
#define FROM_MGAUSS_TO_UT50             (0.1f / 50.0f)
#define FROM_UT50_TO_MGAUSS             500.0f

/* Extern variables ----------------------------------------------------------*/
volatile uint8_t DataLoggerActive = 0;
extern int UseLSI;
extern volatile uint32_t SensorsEnabled; /* This "redundant" line is here to fulfil MISRA C-2012 rule 8.4 */
volatile uint32_t SensorsEnabled = 0xFFFFFFFF;
TIM_HandleTypeDef AlgoTimHandle;
extern uint8_t Enabled6X;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;

volatile static int uart6TXReady = 1;
volatile static int uart2TXReady = 1;
volatile static int uartRXReady = 1;

VL53L1_Dev_t devCenter;
VL53L1_Dev_t devLeft;
VL53L1_Dev_t devRight;
VL53L1_DEV Dev = &devCenter;
int status;
volatile int IntCount;
#define isAutonomousExample 1  /* Allow to select either autonomous ranging or fast ranging example */
#define isInterrupt 0 /* If isInterrupt = 1 then device working in interrupt mode, else device working in polling mode */

static int RtcSynchPrediv;
static RTC_HandleTypeDef RtcHandle;
static IKS01A2_MOTION_SENSOR_Axes_t AccValue;
static IKS01A2_MOTION_SENSOR_Axes_t GyrValue;
static IKS01A2_MOTION_SENSOR_Axes_t MagValue;
static IKS01A2_MOTION_SENSOR_Axes_t MagOffset;
static volatile uint8_t SensorReadRequest = 0;
static volatile uint8_t MagCalRequest = 0;

#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)) || (defined (USE_STM32L1XX_NUCLEO)))
static uint32_t MagTimeStamp = 0;
#endif

static uint8_t MagCalStatus = 0;

static volatile MFX_output_t data_out;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */

int uartSendChar(int ch);
int uartReceiveChar(void);

extern void motor_init(void);
extern bool BSP_MotorControl_SetMaxSpeed(uint8_t deviceId, uint16_t newMaxSpeed);
extern bool BSP_MotorControl_SetMinSpeed(uint8_t deviceId, uint16_t newMinSpeed);

VL53L1_RangingMeasurementData_t* MeasureSensors(void);
MFX_output_t run_mems_process();
void printf_rtc_timestamp(void);

static void RTC_Config(void);
static void RTC_TimeStampConfig(void);
static void Init_Sensors(void);
// static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM_ALGO_Init(void);
static RTC_TimeTypeDef RTC_Handler(TMsg *Msg);
static MFX_output_t FX_Data_Handler(TMsg *Msg);
static void Accelero_Sensor_Handler(TMsg *Msg, uint32_t Instance);
static void Gyro_Sensor_Handler(TMsg *Msg, uint32_t Instance);
static void Magneto_Sensor_Handler(TMsg *Msg, uint32_t Instance);
static void Pressure_Sensor_Handler(TMsg *Msg, uint32_t Instance);
static void Humidity_Sensor_Handler(TMsg *Msg, uint32_t Instance);
static void Temperature_Sensor_Handler(TMsg *Msg, uint32_t Instance);

void writeOutLSM6DSLRegs(void);
void writeHexAsBits(uint8_t value);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == VL53L1X_INT_Pin) {
		IntCount++;
	}
}

int main(void); /* This "redundant" line is here to fulfil MISRA C-2012 rule 8.4 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	uint16_t wordData;
	uint8_t ToFSensor = 1; // 0=Left, 1=Center(default), 2=Right
	uint8_t newI2C = 0x52;

	char lib_version[35];
	int lib_version_len;
	float ans_float;
	TMsg msg_dat;

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
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_ADC1_Init();
	MX_USART6_UART_Init();
	MX_CRC_Init();
	MX_I2C1_Init();

	__HAL_RCC_CRC_CLK_ENABLE()
	;

	/* USER CODE BEGIN 2 */

	setvbuf(stdin, NULL, _IONBF, 0);
	setvbuf(stdout, NULL, _IONBF, 0);
	setvbuf(stderr, NULL, _IONBF, 0);

	printf_rtc_timestamp();
	printf("Starting, initializing sensors ...");

	XNUCLEO53L1A1_Init();

	/* Timer for algorithm synchronization initialization */
	MX_TIM_ALGO_Init();

	/* IKS91A2 initialize (disabled) sensors */
	Init_Sensors();
	MX_X_CUBE_MEMS1_Init();

	/* Sensor Fusion API initialization function */
	MotionFX_manager_init();

	if (Enabled6X == 1U) {
		MotionFX_enable_6X(MFX_ENGINE_ENABLE);
		MotionFX_enable_9X(MFX_ENGINE_DISABLE);
		MotionFX_manager_stop_9X();
		MotionFX_manager_start_6X();
	} else {
		MotionFX_enable_6X(MFX_ENGINE_DISABLE);
		MotionFX_enable_9X(MFX_ENGINE_ENABLE);
		MotionFX_manager_stop_6X();
		MotionFX_manager_start_9X();
	}

	/* OPTIONAL */
	/* Get library version */
	MotionFX_manager_get_version(lib_version, &lib_version_len);

	/* RTC Initialization */
	RTC_Config();
	RTC_TimeStampConfig();

	/* LED Blink */
	BSP_LED_On(LED2);
	HAL_Delay(500);
	BSP_LED_Off(LED2);

	/* Enable magnetometer calibration */
	MotionFX_manager_MagCal_start(ALGO_PERIOD);

	/* Test if calibration data are available */
	MFX_MagCal_output_t mag_cal_test;
	MotionFX_MagCal_getParams(&mag_cal_test);

	/* If calibration data are available load HI coeficients */
	if (mag_cal_test.cal_quality == MFX_MAGCALGOOD) {
		ans_float = (mag_cal_test.hi_bias[0] * FROM_UT50_TO_MGAUSS);
		MagOffset.x = (int32_t) ans_float;
		ans_float = (mag_cal_test.hi_bias[1] * FROM_UT50_TO_MGAUSS);
		MagOffset.y = (int32_t) ans_float;
		ans_float = (mag_cal_test.hi_bias[2] * FROM_UT50_TO_MGAUSS);
		MagOffset.z = (int32_t) ans_float;

		MagCalStatus = 1;
	}

	/* An example here below shows how to manage multi-sensor operation.
	 In this example the sensors range sequentially. Several sensors range simultanously is also possible */

	/* Reset the 3 ToF sensors on the expansion board */
	for (ToFSensor = 0; ToFSensor < 3; ToFSensor++) {
		status = XNUCLEO53L1A1_ResetId(ToFSensor, 0);
	}

	/* Bring the sensors out of the reset stage one by one and set the new I2C address */
	for (ToFSensor = 0; ToFSensor < 3; ToFSensor++) {
		switch (ToFSensor) {
		case 0:
			Dev = &devLeft;
			break;
		case 1:
			Dev = &devCenter;
			break;
		case 2:
			Dev = &devRight;
			break;
		}

		status = XNUCLEO53L1A1_ResetId(ToFSensor, 1);
		Dev->comms_speed_khz = 400;
		Dev->I2cHandle = &hi2c1;
		Dev->comms_type = 1;
		Dev->I2cDevAddr = 0x52; /* default ToF sensor I2C address*/
		VL53L1_RdWord(Dev, 0x010F, &wordData);
		printf("VL53L1X: %02X", wordData);
		newI2C = Dev->I2cDevAddr + (ToFSensor + 1) * 2;
		status = VL53L1_SetDeviceAddress(Dev, newI2C);
		Dev->I2cDevAddr = newI2C;
		VL53L1_RdWord(Dev, 0x010F, &wordData);
		printf("VL53L1X: %02X", wordData);

		/* Device Initialization and setting */
		status = VL53L1_WaitDeviceBooted(Dev);
		status = VL53L1_DataInit(Dev);
		status = VL53L1_StaticInit(Dev);
		status = VL53L1_SetDistanceMode(Dev, VL53L1_DISTANCEMODE_LONG);
		status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(Dev, 50000);
		status = VL53L1_SetInterMeasurementPeriodMilliSeconds(Dev, 100);
	}

	printf_rtc_timestamp();
	printf("Sensors initialized.");

	printf_rtc_timestamp();
	printf("Initialing motors ...");

	motor_init();

	printf_rtc_timestamp();
	printf("Motors initialized.");

	printf_rtc_timestamp();
	printf("Console ready.");

	IKS01A2_MOTION_SENSOR_Enable_6D_Orientation(0, 0);

	IKS01A2_MOTION_SENSOR_SetOutputDataRate(0, MOTION_ACCELERO, 416);
	IKS01A2_MOTION_SENSOR_SetFullScale(0, MOTION_ACCELERO, LSM6DSL_2g);

	IKS01A2_MOTION_SENSOR_SetOutputDataRate(0, MOTION_GYRO, 416);
	IKS01A2_MOTION_SENSOR_SetFullScale(0, MOTION_GYRO, LSM6DSL_500dps);

	IKS01A2_MOTION_SENSOR_SetOutputDataRate(2, MOTION_MAGNETO, 416);
	IKS01A2_MOTION_SENSOR_SetFullScale(2, MOTION_MAGNETO,
	LSM303AGR_MAG_SENSITIVITY_FS_50GAUSS);

	// Lower datarates for non-critical sensors
	IKS01A2_ENV_SENSOR_SetOutputDataRate(0, ENV_TEMPERATURE, 0.1);
	IKS01A2_ENV_SENSOR_SetOutputDataRate(0, ENV_HUMIDITY, 0.1);
	IKS01A2_ENV_SENSOR_SetOutputDataRate(1, ENV_PRESSURE, 0.1);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	char *readline = malloc(100 * sizeof(char));
	VL53L1_RangingMeasurementData_t *RangeData;

	VL53L1_RangingMeasurementData_t RangeDataN;

	while (1) {
		uartRXReady = 1;
		fgets(readline, 100, stdin);

		printf_rtc_timestamp();
		printf("READ -> %s ", readline);

		if ((strncmp(readline, "f", 1) == 0)) {

			printf_rtc_timestamp();
			printf("Running forward");

			BSP_MotorControl_SetMaxSpeed(0, 20);
			BSP_MotorControl_Run(0, BACKWARD);

			BSP_MotorControl_SetMaxSpeed(1, 20);
			BSP_MotorControl_Run(1, FORWARD);

		} else if ((strncmp(readline, "b", 1) == 0)) {

			printf_rtc_timestamp();
			printf("Running backward");

			BSP_MotorControl_SetMaxSpeed(0, 20);
			BSP_MotorControl_Run(0, FORWARD);

			BSP_MotorControl_SetMaxSpeed(1, 20);
			BSP_MotorControl_Run(1, BACKWARD);

		} else if ((strncmp(readline, "r", 1) == 0)) {

			RangeData = MeasureSensors();

			for (ToFSensor = 0; ToFSensor < 3; ToFSensor = ToFSensor + 2) {

				RangeDataN = (*(RangeData + ToFSensor));
				printf_rtc_timestamp();

				if (RangeDataN.RangeStatus == 0) {
					printf(
							"TOFS = %d STAT = %d RNG = %d SGN = %0.2f AMB = %0.2f",
							ToFSensor, RangeDataN.RangeStatus,
							RangeDataN.RangeMilliMeter,
							(RangeDataN.SignalRateRtnMegaCps / 65536.0),
							(RangeDataN.AmbientRateRtnMegaCps) / 65336.0);
				} else
					printf("%d %d 0 0.0 0.0", ToFSensor,
							RangeDataN.RangeStatus);

			}

		} else if ((strncmp(readline, "s", 1) == 0)) {

			// MX_X_CUBE_MEMS1_Process();

			for(int i = 0; i<1000; i++) {
				data_out = run_mems_process();
				HAL_Delay(10);
			}

			/* Acquire data from enabled sensors and fill Msg stream */
			RTC_Handler(&msg_dat);
			Accelero_Sensor_Handler(&msg_dat, IKS01A2_LSM6DSL_0);
			Gyro_Sensor_Handler(&msg_dat, IKS01A2_LSM6DSL_0);
			Magneto_Sensor_Handler(&msg_dat, IKS01A2_LSM303AGR_MAG_0);
			Humidity_Sensor_Handler(&msg_dat, IKS01A2_HTS221_0);
			Temperature_Sensor_Handler(&msg_dat, IKS01A2_HTS221_0);
			Pressure_Sensor_Handler(&msg_dat, IKS01A2_LPS22HB_0);

			/* Sensor Fusion specific part */
			data_out = FX_Data_Handler(&msg_dat);

			printf_rtc_timestamp();
			if (Enabled6X) {
				printf(
						"6X ACC XYZ: %8.2f %8.2f %8.2f GYR XYZ: %8.2f %8.2f %8.2f HDG = %8.2f HDGe = %8.2f",
						data_out.linear_acceleration_6X[0],
						data_out.linear_acceleration_6X[1],
						data_out.linear_acceleration_6X[2],
						data_out.rotation_6X[0], data_out.rotation_6X[1],
						data_out.rotation_6X[2], data_out.heading_6X,
						data_out.headingErr_6X);
			} else {
				printf(
						"9X ACC XYZ: %8.2f %8.2f %8.2f GYR XYZ: %8.2f %8.2f %8.2f HDG = %8.2f HDGe = %8.2f",
						data_out.linear_acceleration_9X[0],
						data_out.linear_acceleration_9X[1],
						data_out.linear_acceleration_9X[2],
						data_out.rotation_9X[0], data_out.rotation_9X[1],
						data_out.rotation_9X[2], data_out.heading_9X,
						data_out.headingErr_9X);
			}

		} else if ((strncmp(readline, "t", 1) == 0)) {

			// Stop motors

			printf_rtc_timestamp();
			printf("Stop");

			BSP_MotorControl_SetMaxSpeed(0, 0);
			BSP_MotorControl_Run(0, FORWARD);

			BSP_MotorControl_SetMaxSpeed(1, 0);
			BSP_MotorControl_Run(1, BACKWARD);

		}

		else {

			printf_rtc_timestamp();
			printf("NOK -> %s", readline);
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
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE()
	;
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* Private functions --------------------------------- */

void printf_rtc_timestamp(void) {
	RTC_TimeTypeDef stimestructure;
	TMsg msg_dat;

	stimestructure = RTC_Handler(&msg_dat);
	printf("T %d:%d:%d:%lu ", stimestructure.Hours, stimestructure.Minutes, stimestructure.Seconds, stimestructure.SubSeconds);
}

MFX_output_t run_mems_process() {

	TMsg msg_dat;

	// MX_X_CUBE_MEMS1_Process();

	/* Acquire data from enabled sensors and fill Msg stream */
	RTC_Handler(&msg_dat);
	Accelero_Sensor_Handler(&msg_dat, IKS01A2_LSM6DSL_0);
	Gyro_Sensor_Handler(&msg_dat, IKS01A2_LSM6DSL_0);
	Magneto_Sensor_Handler(&msg_dat, IKS01A2_LSM303AGR_MAG_0);
	Humidity_Sensor_Handler(&msg_dat, IKS01A2_HTS221_0);
	Temperature_Sensor_Handler(&msg_dat, IKS01A2_HTS221_0);
	Pressure_Sensor_Handler(&msg_dat, IKS01A2_LPS22HB_0);

	/* Sensor Fusion specific part */
	data_out = FX_Data_Handler(&msg_dat);

	return data_out;
}

VL53L1_RangingMeasurementData_t* MeasureSensors(void) {
	static VL53L1_RangingMeasurementData_t RangingData;

	uint8_t ToFSensor;

	static VL53L1_RangingMeasurementData_t RangeData[3];

	for (ToFSensor = 0; ToFSensor < 3; ToFSensor++) {
		switch (ToFSensor) {
		case 0:
			Dev = &devLeft;
			break;
		case 1:
			Dev = &devCenter;
			break;
		case 2:
			Dev = &devRight;
			break;
		}

		status = VL53L1_StartMeasurement(Dev);
		status = VL53L1_WaitMeasurementDataReady(Dev);
		if (!status) {
			status = VL53L1_GetRangingMeasurementData(Dev, &RangingData);

			RangeData[ToFSensor] = RangingData;

			status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
		}
	}

	return RangeData;
}

/**
 * @brief  Initialize all sensors
 * @param  None
 * @retval None
 */
static void Init_Sensors(void) {
	(void) IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM6DSL_0,
	MOTION_ACCELERO | MOTION_GYRO);
	(void) IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM303AGR_MAG_0, MOTION_MAGNETO);
	(void) IKS01A2_ENV_SENSOR_Init(IKS01A2_HTS221_0,
	ENV_TEMPERATURE | ENV_HUMIDITY);
	(void) IKS01A2_ENV_SENSOR_Init(IKS01A2_LPS22HB_0, ENV_PRESSURE);

	/* Start enabled sensors */
	if ((SensorsEnabled & PRESSURE_SENSOR) == PRESSURE_SENSOR) {
		(void) IKS01A2_ENV_SENSOR_Enable(IKS01A2_LPS22HB_0, ENV_PRESSURE);
	}
	if ((SensorsEnabled & TEMPERATURE_SENSOR) == TEMPERATURE_SENSOR) {
		(void) IKS01A2_ENV_SENSOR_Enable(IKS01A2_HTS221_0, ENV_TEMPERATURE);
	}
	if ((SensorsEnabled & HUMIDITY_SENSOR) == HUMIDITY_SENSOR) {
		(void) IKS01A2_ENV_SENSOR_Enable(IKS01A2_HTS221_0, ENV_HUMIDITY);
	}
	if ((SensorsEnabled & ACCELEROMETER_SENSOR) == ACCELEROMETER_SENSOR) {
		(void) IKS01A2_MOTION_SENSOR_Enable(IKS01A2_LSM6DSL_0, MOTION_ACCELERO);
	}
	if ((SensorsEnabled & GYROSCOPE_SENSOR) == GYROSCOPE_SENSOR) {
		(void) IKS01A2_MOTION_SENSOR_Enable(IKS01A2_LSM6DSL_0, MOTION_GYRO);
	}
	if ((SensorsEnabled & MAGNETIC_SENSOR) == MAGNETIC_SENSOR) {
		(void) IKS01A2_MOTION_SENSOR_Enable(IKS01A2_LSM303AGR_MAG_0,
		MOTION_MAGNETO);
	}

	(void) HAL_TIM_Base_Start_IT(&AlgoTimHandle);
	DataLoggerActive = 1;
}

/**
 * @brief  GPIO init function.
 * @param  None
 * @retval None
 * @details GPIOs initialized are User LED(PA5) and User Push Button(PC1)
 */
//static void MX_GPIO_Init(void)
//{
//  /* Initialize LED */
//  BSP_LED_Init(LED2);
//
//  /* Initialize push button */
//  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
//}
/**
 * @brief  CRC init function.
 * @param  None
 * @retval None
 */
static void MX_CRC_Init(void) {
	__CRC_CLK_ENABLE()
	;
}

/**
 * @brief  TIM_ALGO init function.
 * @param  None
 * @retval None
 * @details This function intializes the Timer used to synchronize the algorithm.
 */
static void MX_TIM_ALGO_Init(void) {
#if (defined (USE_STM32F4XX_NUCLEO))
#define CPU_CLOCK  84000000U
#elif (defined (USE_STM32L0XX_NUCLEO))
#define CPU_CLOCK  32000000U
#elif (defined (USE_STM32L1XX_NUCLEO))
#define CPU_CLOCK  32000000U
#elif (defined (USE_STM32L4XX_NUCLEO))
#define CPU_CLOCK  80000000U
#else
#error Not supported platform
#endif

#define TIM_CLOCK  2000U

	const uint32_t prescaler = CPU_CLOCK / TIM_CLOCK - 1U;
	const uint32_t tim_period = TIM_CLOCK / ALGO_FREQ - 1U;

	TIM_ClockConfigTypeDef s_clock_source_config;
	TIM_MasterConfigTypeDef s_master_config;

	AlgoTimHandle.Instance = TIM_ALGO;
	AlgoTimHandle.Init.Prescaler = prescaler;
	AlgoTimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	AlgoTimHandle.Init.Period = tim_period;
	AlgoTimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	(void) HAL_TIM_Base_Init(&AlgoTimHandle);

	s_clock_source_config.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	(void) HAL_TIM_ConfigClockSource(&AlgoTimHandle, &s_clock_source_config);

	s_master_config.MasterOutputTrigger = TIM_TRGO_RESET;
	s_master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	(void) HAL_TIMEx_MasterConfigSynchronization(&AlgoTimHandle,
			&s_master_config);
}

/**
 * @brief  Handles the time+date getting/sending
 * @param  Msg the time+date part of the stream
 * @retval None
 */
static RTC_TimeTypeDef RTC_Handler(TMsg *Msg) {
	uint8_t sub_sec;
	uint32_t ans_uint32;
	int32_t ans_int32;
	RTC_DateTypeDef sdatestructureget;
	RTC_TimeTypeDef stimestructure;

	(void) HAL_RTC_GetTime(&RtcHandle, &stimestructure, FORMAT_BIN);
	(void) HAL_RTC_GetDate(&RtcHandle, &sdatestructureget, FORMAT_BIN);

	/* To be MISRA C-2012 compliant the original calculation:
	 sub_sec = ((((((int)RtcSynchPrediv) - ((int)stimestructure.SubSeconds)) * 100) / (RtcSynchPrediv + 1)) & 0xFF);
	 has been split to separate expressions */
	ans_int32 = (RtcSynchPrediv - (int32_t) stimestructure.SubSeconds) * 100;
	ans_int32 /= RtcSynchPrediv + 1;
	ans_uint32 = (uint32_t) ans_int32 & 0xFFU;
	sub_sec = (uint8_t) ans_uint32;

	Msg->Data[3] = (uint8_t) stimestructure.Hours;
	Msg->Data[4] = (uint8_t) stimestructure.Minutes;
	Msg->Data[5] = (uint8_t) stimestructure.Seconds;
	Msg->Data[6] = sub_sec;

	return stimestructure;
}

/**
 * @brief  Sensor Fusion data handler
 * @param  Msg the Sensor Fusion data part of the stream
 * @retval None
 */
static MFX_output_t FX_Data_Handler(TMsg *Msg) {
#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)) || (defined (USE_STM32L1XX_NUCLEO)))
	MFX_input_t data_in;
	MFX_input_t *pdata_in = &data_in;
	MFX_output_t data_out;
	MFX_output_t *pdata_out = &data_out;
#elif (defined (USE_STM32L0XX_NUCLEO))
	MFX_CM0P_input_t data_in;
	MFX_CM0P_input_t *pdata_in = &data_in;
	MFX_CM0P_output_t data_out;
	MFX_CM0P_output_t *pdata_out = &data_out;
#else
#error Not supported platform
#endif

	if ((SensorsEnabled & ACCELEROMETER_SENSOR) == ACCELEROMETER_SENSOR) {
		if ((SensorsEnabled & GYROSCOPE_SENSOR) == GYROSCOPE_SENSOR) {
			if ((SensorsEnabled & MAGNETIC_SENSOR) == MAGNETIC_SENSOR) {
				data_in.gyro[0] = (float) GyrValue.x * FROM_MDPS_TO_DPS;
				data_in.gyro[1] = (float) GyrValue.y * FROM_MDPS_TO_DPS;
				data_in.gyro[2] = (float) GyrValue.z * FROM_MDPS_TO_DPS;

				data_in.acc[0] = (float) AccValue.x * FROM_MG_TO_G;
				data_in.acc[1] = (float) AccValue.y * FROM_MG_TO_G;
				data_in.acc[2] = (float) AccValue.z * FROM_MG_TO_G;

				data_in.mag[0] = (float) MagValue.x * FROM_MGAUSS_TO_UT50;
				data_in.mag[1] = (float) MagValue.y * FROM_MGAUSS_TO_UT50;
				data_in.mag[2] = (float) MagValue.z * FROM_MGAUSS_TO_UT50;

				/* Run Sensor Fusion algorithm */
				BSP_LED_On(LED2);
				MotionFX_manager_run(pdata_in, pdata_out,
				MOTIONFX_ENGINE_DELTATIME);
				BSP_LED_Off(LED2);

				if (Enabled6X == 1U) {
					(void) memcpy(&Msg->Data[55],
							(void *) pdata_out->quaternion_6X,
							4U * sizeof(float));
					(void) memcpy(&Msg->Data[71],
							(void *) pdata_out->rotation_6X,
							3U * sizeof(float));
					(void) memcpy(&Msg->Data[83],
							(void *) pdata_out->gravity_6X, 3U * sizeof(float));
					(void) memcpy(&Msg->Data[95],
							(void *) pdata_out->linear_acceleration_6X,
							3U * sizeof(float));

#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)) || (defined (USE_STM32L1XX_NUCLEO)))
					(void) memcpy(&Msg->Data[107],
							(void *) &(pdata_out->heading_6X), sizeof(float));
					(void) memcpy(&Msg->Data[111],
							(void *) &(pdata_out->headingErr_6X),
							sizeof(float));
#elif (defined (USE_STM32L0XX_NUCLEO))
					(void)memset(&Msg->Data[107], 0, sizeof(float));
					(void)memset(&Msg->Data[111], 0, sizeof(float));
#else
#error Not supported platform
#endif
				} else {
					(void) memcpy(&Msg->Data[55],
							(void *) pdata_out->quaternion_9X,
							4U * sizeof(float));
					(void) memcpy(&Msg->Data[71],
							(void *) pdata_out->rotation_9X,
							3U * sizeof(float));
					(void) memcpy(&Msg->Data[83],
							(void *) pdata_out->gravity_9X, 3U * sizeof(float));
					(void) memcpy(&Msg->Data[95],
							(void *) pdata_out->linear_acceleration_9X,
							3U * sizeof(float));

#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)) || (defined (USE_STM32L1XX_NUCLEO)))
					(void) memcpy(&Msg->Data[107],
							(void *) &(pdata_out->heading_9X), sizeof(float));
					(void) memcpy(&Msg->Data[111],
							(void *) &(pdata_out->headingErr_9X),
							sizeof(float));
#elif (defined (USE_STM32L0XX_NUCLEO))
					(void)memset(&Msg->Data[107], 0, sizeof(float));
					(void)memset(&Msg->Data[111], 0, sizeof(float));
#else
#error Not supported platform
#endif
				}
			}
		}
	}

//	printf("ROT XYZ: %0.2f %0.2f %0.2f ACC XYZ: %0.2f %0.2f %0.2f HDG = %0.2f HDGe = %0.2f \r\n",
//	       pdata_out->rotation_9X[0], pdata_out->rotation_9X[1], pdata_out->rotation_9X[2],
//		   pdata_out->linear_acceleration_9X[0], pdata_out->linear_acceleration_9X[1],
//		   pdata_out->linear_acceleration_9X[2],
//		   pdata_out->heading_9X, pdata_out->headingErr_9X);

	data_out = *pdata_out;

	return data_out;
}

/**
 * @brief  Handles the ACC axes data getting/sending
 * @param  Msg the ACC part of the stream
 * @param  Instance the device instance
 * @retval None
 */
static void Accelero_Sensor_Handler(TMsg *Msg, uint32_t Instance) {
	if ((SensorsEnabled & ACCELEROMETER_SENSOR) == ACCELEROMETER_SENSOR) {

		(void) IKS01A2_MOTION_SENSOR_GetAxes(Instance, MOTION_ACCELERO,
				&AccValue);
		Serialize_s32(&Msg->Data[19], (int32_t) AccValue.x, 4);
		Serialize_s32(&Msg->Data[23], (int32_t) AccValue.y, 4);
		Serialize_s32(&Msg->Data[27], (int32_t) AccValue.z, 4);
	}
}

/**
 * @brief  Handles the GYR axes data getting/sending
 * @param  Msg the GYR part of the stream
 * @param  Instance the device instance
 * @retval None
 */
static void Gyro_Sensor_Handler(TMsg *Msg, uint32_t Instance) {
	if ((SensorsEnabled & GYROSCOPE_SENSOR) == GYROSCOPE_SENSOR) {
		(void) IKS01A2_MOTION_SENSOR_GetAxes(Instance, MOTION_GYRO, &GyrValue);

		Serialize_s32(&Msg->Data[31], GyrValue.x, 4);
		Serialize_s32(&Msg->Data[35], GyrValue.y, 4);
		Serialize_s32(&Msg->Data[39], GyrValue.z, 4);
	}
}

/**
 * @brief  Handles the MAG axes data getting/sending
 * @param  Msg the MAG part of the stream
 * @param  Instance the device instance
 * @retval None
 */
static void Magneto_Sensor_Handler(TMsg *Msg, uint32_t Instance) {
	float ans_float;
#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)) || (defined (USE_STM32L1XX_NUCLEO)))
	MFX_MagCal_input_t mag_data_in;
	MFX_MagCal_output_t mag_data_out;
#elif (defined (USE_STM32L0XX_NUCLEO))
	MFX_CM0P_MagCal_input_t mag_data_in;
	MFX_CM0P_MagCal_output_t mag_data_out;
#else
#error Not supported platform
#endif

	if ((SensorsEnabled & MAGNETIC_SENSOR) == MAGNETIC_SENSOR) {
		(void) IKS01A2_MOTION_SENSOR_GetAxes(Instance, MOTION_MAGNETO,
				&MagValue);

		if (MagCalStatus == 0U) {
			mag_data_in.mag[0] = (float) MagValue.x * FROM_MGAUSS_TO_UT50;
			mag_data_in.mag[1] = (float) MagValue.y * FROM_MGAUSS_TO_UT50;
			mag_data_in.mag[2] = (float) MagValue.z * FROM_MGAUSS_TO_UT50;

#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)) || (defined (USE_STM32L1XX_NUCLEO)))
			mag_data_in.time_stamp = (int) MagTimeStamp;
			MagTimeStamp += (uint32_t) ALGO_PERIOD;
#endif

			MotionFX_manager_MagCal_run(&mag_data_in, &mag_data_out);

#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)) || (defined (USE_STM32L1XX_NUCLEO)))
			if (mag_data_out.cal_quality == MFX_MAGCALGOOD)
#elif (defined (USE_STM32L0XX_NUCLEO))
					if (mag_data_out.cal_quality == MFX_CM0P_MAGCALGOOD)
#else
#error Not supported platform
#endif
					{
				MagCalStatus = 1;

				ans_float = (mag_data_out.hi_bias[0] * FROM_UT50_TO_MGAUSS);
				MagOffset.x = (int32_t) ans_float;
				ans_float = (mag_data_out.hi_bias[1] * FROM_UT50_TO_MGAUSS);
				MagOffset.y = (int32_t) ans_float;
				ans_float = (mag_data_out.hi_bias[2] * FROM_UT50_TO_MGAUSS);
				MagOffset.z = (int32_t) ans_float;

				/* Disable magnetometer calibration */
				MotionFX_manager_MagCal_stop(ALGO_PERIOD);
			}
		}

		MagValue.x = (int32_t) (MagValue.x - MagOffset.x);
		MagValue.y = (int32_t) (MagValue.y - MagOffset.y);
		MagValue.z = (int32_t) (MagValue.z - MagOffset.z);

		Serialize_s32(&Msg->Data[43], MagValue.x, 4);
		Serialize_s32(&Msg->Data[47], MagValue.y, 4);
		Serialize_s32(&Msg->Data[51], MagValue.z, 4);
	}
}

/**
 * @brief  Handles the PRESS sensor data getting/sending.
 * @param  Msg the PRESS part of the stream
 * @param  Instance the device instance
 * @retval None
 */
static void Pressure_Sensor_Handler(TMsg *Msg, uint32_t Instance) {
	float press_value;

	if ((SensorsEnabled & PRESSURE_SENSOR) == PRESSURE_SENSOR) {
		(void) IKS01A2_ENV_SENSOR_GetValue(Instance, ENV_PRESSURE,
				&press_value);
		(void) memcpy(&Msg->Data[7], (void *) &press_value, sizeof(float));
	}
}

/**
 * @brief  Handles the TEMP axes data getting/sending
 * @param  Msg the TEMP part of the stream
 * @param  Instance the device instance
 * @retval None
 */
static void Temperature_Sensor_Handler(TMsg *Msg, uint32_t Instance) {
	float temp_value;

	if ((SensorsEnabled & TEMPERATURE_SENSOR) == TEMPERATURE_SENSOR) {
		(void) IKS01A2_ENV_SENSOR_GetValue(Instance, ENV_TEMPERATURE,
				&temp_value);
		(void) memcpy(&Msg->Data[11], (void *) &temp_value, sizeof(float));
	}
}

/**
 * @brief  Handles the HUM axes data getting/sending
 * @param  Msg the HUM part of the stream
 * @param  Instance the device instance
 * @retval None
 */
static void Humidity_Sensor_Handler(TMsg *Msg, uint32_t Instance) {
	float hum_value;

	if ((SensorsEnabled & HUMIDITY_SENSOR) == HUMIDITY_SENSOR) {
		(void) IKS01A2_ENV_SENSOR_GetValue(Instance, ENV_HUMIDITY, &hum_value);
		(void) memcpy(&Msg->Data[15], (void *) &hum_value, sizeof(float));
		;
	}
}

/**
 * @brief  Configures the RTC
 * @param  None
 * @retval None
 */
static void RTC_Config(void) {
	/*##-1- Configure the RTC peripheral #######################################*/
	/* Check if LSE can be used */
	RCC_OscInitTypeDef rcc_osc_init_struct;

	/*##-2- Configure LSE as RTC clock soucre ###################################*/
	rcc_osc_init_struct.OscillatorType = RCC_OSCILLATORTYPE_LSI
			| RCC_OSCILLATORTYPE_LSE;
	rcc_osc_init_struct.PLL.PLLState = RCC_PLL_NONE;
	rcc_osc_init_struct.LSEState = RCC_LSE_ON;
	rcc_osc_init_struct.LSIState = RCC_LSI_OFF;
	if (HAL_RCC_OscConfig(&rcc_osc_init_struct) != HAL_OK) {
		/* LSE not available, we use LSI */
		UseLSI = 1;
		RtcHandle.Init.AsynchPrediv = RTC_ASYNCH_PREDIV_LSI;
		RtcHandle.Init.SynchPrediv = RTC_SYNCH_PREDIV_LSI;
		RtcSynchPrediv = RTC_SYNCH_PREDIV_LSI;
	} else {
		/* We use LSE */
		UseLSI = 0;
		RtcHandle.Init.AsynchPrediv = RTC_ASYNCH_PREDIV_LSE;
		RtcHandle.Init.SynchPrediv = RTC_SYNCH_PREDIV_LSE;
		RtcSynchPrediv = RTC_SYNCH_PREDIV_LSE;
	}
	RtcHandle.Instance = RTC;

	/* Configure RTC prescaler and RTC data registers */
	/* RTC configured as follow:
	 - Hour Format    = Format 12
	 - Asynch Prediv  = Value according to source clock
	 - Synch Prediv   = Value according to source clock
	 - OutPut         = Output Disable
	 - OutPutPolarity = High Polarity
	 - OutPutType     = Open Drain
	 */
	RtcHandle.Init.HourFormat = RTC_HOURFORMAT_12;
	RtcHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
	RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	RtcHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;

	if (HAL_RTC_Init(&RtcHandle) != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}
}

/**
 * @brief  Configures the current time and date
 * @param  None
 * @retval None
 */
static void RTC_TimeStampConfig(void) {
	RTC_DateTypeDef sdatestructure;
	RTC_TimeTypeDef stimestructure;

	/* Configure the Date */
	/* Set Date: Monday January 1st 2001 */
	sdatestructure.Year = 0x01;
	sdatestructure.Month = RTC_MONTH_JANUARY;
	sdatestructure.Date = 0x01;
	sdatestructure.WeekDay = RTC_WEEKDAY_MONDAY;

	if (HAL_RTC_SetDate(&RtcHandle, &sdatestructure, FORMAT_BCD) != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}

	/* Configure the Time */
	/* Set Time: 00:00:00 */
	stimestructure.Hours = 0x00;
	stimestructure.Minutes = 0x00;
	stimestructure.Seconds = 0x00;
	stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
	stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

	if (HAL_RTC_SetTime(&RtcHandle, &stimestructure, FORMAT_BCD) != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}
}

/**
 * @brief  Configures the current date
 * @param  y the year value to be set
 * @param  m the month value to be set
 * @param  d the day value to be set
 * @param  dw the day-week value to be set
 * @retval None
 */
void RTC_DateRegulate(uint8_t y, uint8_t m, uint8_t d, uint8_t dw) {
	RTC_DateTypeDef sdatestructure;

	sdatestructure.Year = y;
	sdatestructure.Month = m;
	sdatestructure.Date = d;
	sdatestructure.WeekDay = dw;

	if (HAL_RTC_SetDate(&RtcHandle, &sdatestructure, FORMAT_BIN) != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}
}

/**
 * @brief  Configures the current time
 * @param  hh the hour value to be set
 * @param  mm the minute value to be set
 * @param  ss the second value to be set
 * @retval None
 */
void RTC_TimeRegulate(uint8_t hh, uint8_t mm, uint8_t ss) {
	RTC_TimeTypeDef stimestructure;

	stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
	stimestructure.Hours = hh;
	stimestructure.Minutes = mm;
	stimestructure.Seconds = ss;
	stimestructure.SubSeconds = 0;
	stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

	if (HAL_RTC_SetTime(&RtcHandle, &stimestructure, FORMAT_BIN) != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}
}

/** @brief Sends a character to serial port
 * @param ch Character to send
 * @retval Character sent
 */
int uartSendChar(int ch) {
//	while ((uart2TXReady == 0) | (uart6TXReady == 0)) {
//		;
//	}

//	while ((uart2TXReady == 0)) {
//		;
//	}

	uart2TXReady = 0;
	uart6TXReady = 0;

//	HAL_UART_Transmit_DMA(&huart6, (uint8_t *)&ch, 1);

//	while (uart6TXReady == 0) {
//		;
//	}

	while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY)
	{

	}


	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&ch, 1);

	// HAL_UART_Transmit(&huart2, (uint8_t *) &ch, 1, 100);

	while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY)
	{

	}


//	while (uart2TXReady == 0) {
//		;
//	}

	return ch;
}

/** @brief Receives a character from serial port
 * @param None
 * @retval Character received
 */
int uartReceiveChar(void) {
	uint8_t ch;

//	while (uartRXReady == 0) {
//		;
//	}

	uartRXReady = 0;

	while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY)
	{

		//data_out = run_mems_process();
		//HAL_Delay(10);

	}

	HAL_UART_Receive_DMA(&huart2, &ch, 1);

//	while (uartRXReady == 0) {
//
//		// Most of time is spend here, so
//		// run MEMS updates
	while (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY)
	{
		//data_out = run_mems_process();
		//HAL_Delay(10);
	}

	//HAL_UART_DMAStop()

//	}

	return ch;
}

/** @brief putchar call for standard output implementation
 * @param ch Character to print
 * @retval Character printed
 */
int __io_putchar(int ch) {
	uartSendChar(ch);

	return 0;
}

/** @brief getchar call for standard input implementation
 * @param None
 * @retval Character acquired from standard input
 */
int __io_getchar(void) {
	return uartReceiveChar();
}

/**
 * @brief  Tx Transfer completed callback
 * @param  UartHandle: UART handle.
 * @note   This example shows a simple way to report end of DMA Tx transfer, and
 *         you can add your own implementation.
 * @retval None
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle) {
	/* Set transmission flag: trasfer complete*/

	if (UartHandle->Instance == USART2)
		uart2TXReady = 1;
	if (UartHandle->Instance == USART6)
		uart6TXReady = 1;
}

/**
 * @brief  Rx Transfer completed callback
 * @param  UartHandle: UART handle
 * @note   This example shows a simple way to report end of DMA Rx transfer, and
 *         you can add your own implementation.
 * @retval None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) {
	/* Set transmission flag: transfer complete*/
	uartRXReady = 1;
}

/* End Private functions --------------------------------- */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */

	for (;;) {
		BSP_LED_On(LED2);
		HAL_Delay(100);
		BSP_LED_Off(LED2);
		HAL_Delay(100);
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
void assert_failed(uint8_t *file, uint32_t line) {
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number */
	printf("Wrong parameters value: file %s on line %lu\r\n", file, line);
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
