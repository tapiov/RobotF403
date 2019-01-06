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

#include "stm32xxx_hal.h"

#include "vl53l1_api.h"
#include "X-NUCLEO-53L1A1.h"

#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#include "motor_control.h"

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

/* USER CODE BEGIN PV */

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;
VL53L1_Dev_t devCenter;
VL53L1_Dev_t devLeft;
VL53L1_Dev_t devRight;
VL53L1_DEV Dev = &devCenter;
int status;
volatile int IntCount;
#define isAutonomousExample 1  /* Allow to select either autonomous ranging or fast ranging example */
#define isInterrupt 0 /* If isInterrupt = 1 then device working in interrupt mode, else device working in polling mode */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */

extern void motor_init(void);
extern bool BSP_MotorControl_SetMaxSpeed(uint8_t deviceId, uint16_t newMaxSpeed);
extern bool BSP_MotorControl_SetMinSpeed(uint8_t deviceId, uint16_t newMinSpeed);

VL53L1_RangingMeasurementData_t* MeasureSensors(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == VL53L1X_INT_Pin) {
		IntCount++;
	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	uint16_t wordData;
	uint8_t ToFSensor = 1; // 0=Left, 1=Center(default), 2=Right
	static VL53L1_RangingMeasurementData_t RangingData;
	uint8_t newI2C = 0x52;

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
	/* USER CODE BEGIN 2 */

	printf("\r\nConsole ready ... \r\n");

	XNUCLEO53L1A1_Init();

	setvbuf(stdin, NULL, _IONBF, 0);
	setvbuf(stdout, NULL, _IONBF, 0);
	setvbuf(stderr, NULL, _IONBF, 0);

	printf("\r\nConsole ready ... \r\n");

	uint8_t *readline = malloc(100 * sizeof(uint8_t));
	VL53L1_RangingMeasurementData_t *RangeData;

	motor_init();

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
			printf("VL53L1X: %02X\n\r", wordData);
			newI2C = Dev->I2cDevAddr + (ToFSensor + 1) * 2;
			status = VL53L1_SetDeviceAddress(Dev, newI2C);
			Dev->I2cDevAddr = newI2C;
			VL53L1_RdWord(Dev, 0x010F, &wordData);
			printf("VL53L1X: %02X\n\r", wordData);

			/* Device Initialization and setting */
			status = VL53L1_WaitDeviceBooted(Dev);
			status = VL53L1_DataInit(Dev);
			status = VL53L1_StaticInit(Dev);
			status = VL53L1_SetDistanceMode(Dev, VL53L1_DISTANCEMODE_LONG);
			status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(Dev, 50000);
			status = VL53L1_SetInterMeasurementPeriodMilliSeconds(Dev, 100);
		}

	/* USER CODE END 2 */


	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	VL53L1_RangingMeasurementData_t RangeDataN;

	while (1) {

		RangeData = MeasureSensors();

		// Timestamp, 10 numbers
		printf("%-10lu ", HAL_GetTick());

		for (ToFSensor = 0; ToFSensor < 3; ToFSensor++) {

			RangeDataN = (*(RangeData+ToFSensor));

			if (RangeDataN.RangeStatus == 0) {
				printf("%-6d|%-6d|%-6d|%-6.2f|%-6.2f|", ToFSensor, RangeDataN.RangeStatus, RangeDataN.RangeMilliMeter,
						(RangeDataN.SignalRateRtnMegaCps / 65536.0), (RangeDataN.AmbientRateRtnMegaCps) / 65336.0);
			} else
				printf("%-6d|%-6d|%-6d|%-6.2f|%-6.2f|", ToFSensor, RangeDataN.RangeStatus, 0, 0.0, 0.0);

		}

		printf("\n\r");

//	  // fgets(readline,100,stdin);
//	  // printf("OK -> %s",readline);
//
//	  if ((strncmp(readline, "f",1) == 0)) {
//			BSP_MotorControl_SetMaxSpeed(0, 20);
//			BSP_MotorControl_Run(0, BACKWARD);
//
//			BSP_MotorControl_SetMaxSpeed(1, 20);
//			BSP_MotorControl_Run(1, FORWARD);
//
//			HAL_Delay(1000);
//
//	  } else if ((strncmp(readline,"b",1) == 0)) {
//			BSP_MotorControl_SetMaxSpeed(0, 20);
//			BSP_MotorControl_Run(0, FORWARD);
//
//			BSP_MotorControl_SetMaxSpeed(1, 20);
//			BSP_MotorControl_Run(1, BACKWARD);
//
//			HAL_Delay(1000);
//
//	  } else {
//		  printf("NOK -> %s",readline);
//	  }

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
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
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
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
