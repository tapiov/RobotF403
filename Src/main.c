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
#include <math.h>

#include "mems.h"
#include "range.h"

#include "stm32xxx_hal.h"

#include "vl53l1_api.h"
#include "X-NUCLEO-53L1A1.h"

#include "motor_control.h"

#include "DemoSerial.h"
#include "MotionFX_Manager.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Extern variables ----------------------------------------------------------*/

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

I2C_HandleTypeDef hi2c1;

static MFX_output_t data_out;
RTC_TimeTypeDef stimestruct;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */

void motor_init(void);

extern void MeasureSensors(VL53L1_RangingMeasurementData_t (*RangeData)[3]);
extern MFX_output_t run_mems_process();

void MX_CRC_Init(void);

extern void mems_init(void);
//extern MFX_output_t FX_Data_Handler(IKS01A2_MOTION_SENSOR_Axes_t AccValue,
//		IKS01A2_MOTION_SENSOR_Axes_t GyrValue,
//		IKS01A2_MOTION_SENSOR_Axes_t MagValue);
//extern void Accelero_Sensor_Handler(uint32_t Instance);
//extern void Gyro_Sensor_Handler(uint32_t Instance);
//extern void Magneto_Sensor_Handler(uint32_t Instance);
//extern void Pressure_Sensor_Handler(uint32_t Instance);
//extern void Humidity_Sensor_Handler(uint32_t Instance);
//extern void Temperature_Sensor_Handler(uint32_t Instance);

extern void writeOutLSM6DSLRegs(void);
extern void writeHexAsBits(uint8_t value);

extern void Print_TimeStamp(void);
extern void MX_TIM_ALGO_Init(void);
extern void RTC_Config(void);
extern void RTC_TimeStampConfig(void);

void range_init(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int main(void); /* This "redundant" line is here to fulfil MISRA C-2012 rule 8.4 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

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

	Print_TimeStamp();
	printf("Starting, initializing sensors ...\n");

	XNUCLEO53L1A1_Init();

	struct devStore devs;

	range_init();

	/* Timer for algorithm synchronization initialization */
	MX_TIM_ALGO_Init();

	// MEMS init
	mems_init();

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

	Print_TimeStamp();
	printf("Sensors initialized.\n");

	Print_TimeStamp();
	printf("Initialing motors ...\n");

	motor_init();

	Print_TimeStamp();
	printf("Motors initialized.\n");

	Print_TimeStamp();
	printf("Console ready.\n");

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	char *readline = malloc(100 * sizeof(char));
	VL53L1_RangingMeasurementData_t *RangeData;

	VL53L1_RangingMeasurementData_t RangeDataN;

	uint8_t seconds = 0;
	uint8_t seconds_last = 0;

	MFX_output_t start_data_out;

	start_data_out = run_mems_process();

	// 1. Execute turn for 5 sec, record headings with ranges
	// Output data to term

	// 2. try exact degree (90, 180, 45) turns using gyro

	HAL_Delay(3000);



	while (1) {


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

/**
 * @brief  CRC init function.
 * @param  None
 * @retval None
 */
void MX_CRC_Init(void) {
__CRC_CLK_ENABLE()
;
}

/** @brief Sends a character to serial port
 * @param ch Character to send
 * @retval Character sent
 */

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

// Code storage

//if ((strncmp(readline, "f", 1) == 0)) {
//	Print_TimeStamp();
//	printf("Running forward");
//
//	BSP_MotorControl_SetMaxSpeed(0, 20);
//	BSP_MotorControl_Run(0, BACKWARD);
//
//	BSP_MotorControl_SetMaxSpeed(1, 20);
//	BSP_MotorControl_Run(1, FORWARD);
//} else if ((strncmp(readline, "b", 1) == 0)) {
//	Print_TimeStamp();
//	printf("Running backward");
//
//	BSP_MotorControl_SetMaxSpeed(0, 20);
//	BSP_MotorControl_Run(0, FORWARD);
//
//	BSP_MotorControl_SetMaxSpeed(1, 20);
//	BSP_MotorControl_Run(1, BACKWARD);
//} else if ((strncmp(readline, "r", 1) == 0)) {
//	RangeData = MeasureSensors();
//
//	for (ToFSensor = 0; ToFSensor < 3; ToFSensor = ToFSensor + 2) {
//		RangeDataN = (*(RangeData + ToFSensor));
//		Print_TimeStamp();
//
//		if (RangeDataN.RangeStatus == 0) {
//			printf(
//					"TOFS = %d STAT = %d RNG = %d SGN = %0.2f AMB = %0.2f ",
//					ToFSensor, RangeDataN.RangeStatus,
//					RangeDataN.RangeMilliMeter,
//					(RangeDataN.SignalRateRtnMegaCps / 65536.0),
//					(RangeDataN.AmbientRateRtnMegaCps) / 65336.0);
//		} else {
//			printf("TOFS = %d STAT = %d RNG = 0 SGN = 0.00 AMB = 0.00 ",
//					ToFSensor, RangeDataN.RangeStatus);
//		}
//	}
//} else if ((strncmp(readline, "s", 1) == 0)) {
//
//	/* Sensor Fusion specific part */
//	while (RTC_GetReadRequest() == 0) {
//	}
//	data_out = run_mems_process();
//	RTC_ResetReadRequest();
//
//	seconds_last = seconds;
//	seconds = stimestructure.Seconds;
//
//	if (seconds > seconds_last) {
//		Print_TimeStamp();
//
////							printf(
////									"ST ACC XYZ: %8.6f %8.6f %8.6f ROT XYZ: %8.5f %8.5f %8.5f",
////									start_data_out.linear_acceleration_6X[0],
////									start_data_out.linear_acceleration_6X[1],
////									start_data_out.linear_acceleration_6X[2],
////									start_data_out.rotation_6X[0], data_out.rotation_6X[1],
////									start_data_out.rotation_6X[2]);
////
////							if (Enabled6X) {
////								printf(
////										"6X ACC XYZ: %8.6f %8.6f %8.6f ROT XYZ: %8.5f %8.5f %8.5f",
////										data_out.linear_acceleration_6X[0],
////										data_out.linear_acceleration_6X[1],
////										data_out.linear_acceleration_6X[2],
////										data_out.rotation_6X[0], data_out.rotation_6X[1],
////										data_out.rotation_6X[2]);
////							} else {
////								printf(
////										"9X ACC XYZ: %8.6f %8.6f %8.6f ROT XYZ: %8.5f %8.5f %8.5f",
////										data_out.linear_acceleration_9X[0],
////										data_out.linear_acceleration_9X[1],
////										data_out.linear_acceleration_9X[2],
////										data_out.rotation_9X[0], data_out.rotation_9X[1],
////										data_out.rotation_9X[2]);
////							}
//
//		printf("ST ROT X: %8.5f", start_data_out.rotation_6X[0]);
//
//		if (Enabled6X) {
//			printf("6X ROT X: %8.5f", data_out.rotation_6X[0]);
//		} else {
//			printf(
//					"9X ACC XYZ: %8.6f %8.6f %8.6f ROT XYZ: %8.5f %8.5f %8.5f",
//					data_out.linear_acceleration_9X[0],
//					data_out.linear_acceleration_9X[1],
//					data_out.linear_acceleration_9X[2],
//					data_out.rotation_9X[0], data_out.rotation_9X[1],
//					data_out.rotation_9X[2]);
//		}
//
//		printf("\n");
//
//		RangeData = MeasureSensors();
//
//		for (ToFSensor = 2; ToFSensor < 3; ToFSensor = ToFSensor + 2) {
//			RangeDataN = (*(RangeData + ToFSensor));
//			Print_TimeStamp();
//
////								if (RangeDataN.RangeStatus == 0) {
////									printf(
////											"TOFS = %d STAT = %d RNG = %d SGN = %0.2f AMB = %0.2f ",
////											ToFSensor, RangeDataN.RangeStatus,
////											RangeDataN.RangeMilliMeter,
////											(RangeDataN.SignalRateRtnMegaCps / 65536.0),
////											(RangeDataN.AmbientRateRtnMegaCps) / 65336.0);
////								} else {
////									printf("TOFS = %d STAT = %d RNG = 0 SGN = 0.00 AMB = 0.00 ",
////											ToFSensor, RangeDataN.RangeStatus);
////								}
//
//			if (RangeDataN.RangeStatus == 0) {
//				printf("RNG = %d ", RangeDataN.RangeMilliMeter);
//			} else {
//				printf(
//						"TOFS = %d STAT = %d RNG = 0 SGN = 0.00 AMB = 0.00 ",
//						ToFSensor, RangeDataN.RangeStatus);
//			}
//		}
//
//		printf("\n");
//	}
//
//	// Is there room forward?
//	if (((*(RangeData + 2)).RangeMilliMeter > 400) && (moving)) {
//		robot_move(FORWARD, 30);
//		moving = 1;
//		turning = 0;
//
//		if (rprint++ == 1) {
//			Print_TimeStamp();
//			printf("Running forward\n");
//		}
//	}
//	if ((((*(RangeData + 2)).RangeMilliMeter < 400)) && (moving)) {
//		robot_stop();
//		moving = 0;
//		turning = 1;
//
//		if (rprint++ == 2) {
//			Print_TimeStamp();
//			printf("Too close - Stop\n");
//		}
//	}
//
//	if ((fabsf(data_out.rotation_6X[0] - start_data_out.rotation_6X[0])
//			< 90) && (turning)) {
//		robot_turn(RIGHT, 40);
//		moving = 0;
//		turning = 1;
//
//		printf(" - %f - \n",
//				(fabsf(
//						data_out.rotation_6X[0]
//								- start_data_out.rotation_6X[0])));
//
//		if (rprint++ == 3) {
//			Print_TimeStamp();
//			printf("Turning right\n");
//		}
//	}
//	if ((fabsf(data_out.rotation_6X[0] - start_data_out.rotation_6X[0])
//			> 90) && (turning)) {
//		robot_stop();
//		start_data_out = data_out;
//		moving = 1;
//		turning = 0;
//
//		if (rprint++ == 4) {
//			Print_TimeStamp();
//			printf("Turn finished - Stop\n");
//		}
//
//		rprint = 1;
//	}
//}
//}
//else if ((strncmp(readline, "t", 1) == 0)) {
//// Stop motors
//
//Print_TimeStamp();
//printf("Stop");
//
//BSP_MotorControl_SetMaxSpeed(0, 0);
//BSP_MotorControl_Run(0, FORWARD);
//
//BSP_MotorControl_SetMaxSpeed(1, 0);
//BSP_MotorControl_Run(1, BACKWARD);
//} else {
//Print_TimeStamp();
//printf("NOK -> %s", readline);
//}


