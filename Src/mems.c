/**
 ******************************************************************************
 * File Name          :  stmicroelectronics_x-cube-mems1_1_0_0.c
 * Description        : This file provides code for the configuration
 *                      of the STMicroelectronics.X-CUBE-MEMS1.1.0.0 instances.
 ******************************************************************************
 *
 * COPYRIGHT 2018 STMicroelectronics
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *        http://www.st.com/software_license_agreement_liberty_v2
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ******************************************************************************
 */

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "mems.h"
#include "motion_fx.h"

#include "iks01a2_motion_sensors.h"
#include "iks01a2_env_sensors.h"
#include "stm32f4_nucleo_f401re.h"
#include "math.h"

/* Private typedef -----------------------------------------------------------*/
typedef struct displayFloatToInt_s {
	int8_t sign; /* 0 means positive, 1 means negative*/
	uint32_t out_int;
	uint32_t out_dec;
} displayFloatToInt_t;

/* Private define ------------------------------------------------------------*/
#define MAX_BUF_SIZE 256

/* Private variables ---------------------------------------------------------*/
uint8_t PushButtonDetected = 0;
IKS01A2_MOTION_SENSOR_Capabilities_t MotionCapabilities[IKS01A2_MOTION_INSTANCES_NBR];
IKS01A2_ENV_SENSOR_Capabilities_t EnvCapabilities[IKS01A2_ENV_INSTANCES_NBR];
char dataOut[MAX_BUF_SIZE];

/* Private function prototypes -----------------------------------------------*/
void floatToInt(float in, displayFloatToInt_t *out_value, int32_t dec_prec);
IKS01A2_MOTION_SENSOR_Axes_t Accelero_Sensor_Handler(uint32_t Instance);
IKS01A2_MOTION_SENSOR_Axes_t Gyro_Sensor_Handler(uint32_t Instance);
IKS01A2_MOTION_SENSOR_Axes_t Magneto_Sensor_Handler(uint32_t Instance);
float Temp_Sensor_Handler(uint32_t Instance);
float Hum_Sensor_Handler(uint32_t Instance);
float Press_Sensor_Handler(uint32_t Instance);
void MX_IKS01A2_DataLogTerminal_Init(void);
void MX_IKS01A2_DataLogTerminal_Process(void);

void mems_init(void);
MFX_output_t run_mems_process();
extern RTC_TimeTypeDef RTC_Handler();

void mems_init(void) {

	TIM_HandleTypeDef *htim5;

	displayFloatToInt_t out_value_odr;
	int i;

	float ans_float;

	IKS01A2_MOTION_SENSOR_Axes_t MagOffset;
	volatile uint8_t MagCalRequest = 0;

	/* IKS91A2 initialize (disabled) sensors */
	(void) IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM6DSL_0,
	MOTION_ACCELERO | MOTION_GYRO);
	(void) IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM303AGR_MAG_0,
	MOTION_MAGNETO);
	(void) IKS01A2_ENV_SENSOR_Init(IKS01A2_HTS221_0,
	ENV_TEMPERATURE | ENV_HUMIDITY);
	(void) IKS01A2_ENV_SENSOR_Init(IKS01A2_LPS22HB_0, ENV_PRESSURE);

	/* Start enabled sensors */
	if ((SensorsEnabled & PRESSURE_SENSOR) == PRESSURE_SENSOR) {
		(void) IKS01A2_ENV_SENSOR_Enable(IKS01A2_LPS22HB_0,
		ENV_PRESSURE);
	}
	if ((SensorsEnabled & TEMPERATURE_SENSOR) == TEMPERATURE_SENSOR) {
		(void) IKS01A2_ENV_SENSOR_Enable(IKS01A2_HTS221_0,
		ENV_TEMPERATURE);
	}
	if ((SensorsEnabled & HUMIDITY_SENSOR) == HUMIDITY_SENSOR) {
		(void) IKS01A2_ENV_SENSOR_Enable(IKS01A2_HTS221_0,
		ENV_HUMIDITY);
	}
	if ((SensorsEnabled & ACCELEROMETER_SENSOR) == ACCELEROMETER_SENSOR) {
		(void) IKS01A2_MOTION_SENSOR_Enable(IKS01A2_LSM6DSL_0,
		MOTION_ACCELERO);
	}
	if ((SensorsEnabled & GYROSCOPE_SENSOR) == GYROSCOPE_SENSOR) {
		(void) IKS01A2_MOTION_SENSOR_Enable(IKS01A2_LSM6DSL_0,
		MOTION_GYRO);
	}
	if ((SensorsEnabled & MAGNETIC_SENSOR) == MAGNETIC_SENSOR) {
		(void) IKS01A2_MOTION_SENSOR_Enable(IKS01A2_LSM303AGR_MAG_0,
		MOTION_MAGNETO);
	}

	(void) HAL_TIM_Base_Start_IT(htim5);

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

	/* Initialize LED */
	BSP_LED_Init(LED2);

	/* Initialize button */
	BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

	IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM6DSL_0, MOTION_GYRO);

	IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM303AGR_ACC_0, MOTION_ACCELERO);

	IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM303AGR_MAG_0, MOTION_MAGNETO);

	for (i = 0; i < IKS01A2_MOTION_INSTANCES_NBR; i++) {
		IKS01A2_MOTION_SENSOR_GetCapabilities(i, &MotionCapabilities[i]);
		snprintf(dataOut, MAX_BUF_SIZE,
				"\r\nMotion Sensor Instance %d capabilities: \r\n ACCELEROMETER: %d\r\n GYROSCOPE: %d\r\n MAGNETOMETER: %d\r\n LOW POWER: %d\r\n",
				i, MotionCapabilities[i].Acc, MotionCapabilities[i].Gyro,
				MotionCapabilities[i].Magneto, MotionCapabilities[i].LowPower);
		printf("%s", dataOut);
		floatToInt(MotionCapabilities[i].AccMaxOdr, &out_value_odr, 3);
		snprintf(dataOut, MAX_BUF_SIZE,
				" MAX ACC ODR: %d.%03d Hz, MAX ACC FS: %d\r\n",
				(int) out_value_odr.out_int, (int) out_value_odr.out_dec,
				(int) MotionCapabilities[i].AccMaxFS);
		printf("%s", dataOut);
		floatToInt(MotionCapabilities[i].GyroMaxOdr, &out_value_odr, 3);
		snprintf(dataOut, MAX_BUF_SIZE,
				" MAX GYRO ODR: %d.%03d Hz, MAX GYRO FS: %d\r\n",
				(int) out_value_odr.out_int, (int) out_value_odr.out_dec,
				(int) MotionCapabilities[i].GyroMaxFS);
		printf("%s", dataOut);
		floatToInt(MotionCapabilities[i].MagMaxOdr, &out_value_odr, 3);
		snprintf(dataOut, MAX_BUF_SIZE,
				" MAX MAG ODR: %d.%03d Hz, MAX MAG FS: %d\r\n",
				(int) out_value_odr.out_int, (int) out_value_odr.out_dec,
				(int) MotionCapabilities[i].MagMaxFS);
		printf("%s", dataOut);
	}

	IKS01A2_ENV_SENSOR_Init(IKS01A2_HTS221_0, ENV_TEMPERATURE | ENV_HUMIDITY);

	IKS01A2_ENV_SENSOR_Init(IKS01A2_LPS22HB_0, ENV_TEMPERATURE | ENV_PRESSURE);

	for (i = 0; i < IKS01A2_ENV_INSTANCES_NBR; i++) {
		IKS01A2_ENV_SENSOR_GetCapabilities(i, &EnvCapabilities[i]);
		snprintf(dataOut, MAX_BUF_SIZE,
				"\r\nEnvironmental Sensor Instance %d capabilities: \r\n TEMPERATURE: %d\r\n PRESSURE: %d\r\n HUMIDITY: %d\r\n LOW POWER: %d\r\n",
				i, EnvCapabilities[i].Temperature, EnvCapabilities[i].Pressure,
				EnvCapabilities[i].Humidity, EnvCapabilities[i].LowPower);
		printf("%s", dataOut);
		floatToInt(EnvCapabilities[i].TempMaxOdr, &out_value_odr, 3);
		snprintf(dataOut, MAX_BUF_SIZE, " MAX TEMP ODR: %d.%03d Hz\r\n",
				(int) out_value_odr.out_int, (int) out_value_odr.out_dec);
		printf("%s", dataOut);
		floatToInt(EnvCapabilities[i].PressMaxOdr, &out_value_odr, 3);
		snprintf(dataOut, MAX_BUF_SIZE, " MAX PRESS ODR: %d.%03d Hz\r\n",
				(int) out_value_odr.out_int, (int) out_value_odr.out_dec);
		printf("%s", dataOut);
		floatToInt(EnvCapabilities[i].HumMaxOdr, &out_value_odr, 3);
		snprintf(dataOut, MAX_BUF_SIZE, " MAX HUM ODR: %d.%03d Hz\r\n",
				(int) out_value_odr.out_int, (int) out_value_odr.out_dec);
		printf("%s", dataOut);
	}

	// Mag cal
	/* Enable magnetometer calibration */

	uint32_t MagTimeStamp = 0;
	uint8_t MagCalStatus = 0;

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

	/* Sensor Fusion API initialization function */
	MotionFX_manager_init();

	if (Enabled6X == 1) {
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

}

/**
 * @brief  BSP Push Button callback
 * @param  Button Specifies the pin connected EXTI line
 * @retval None.
 */
void BSP_PB_Callback(Button_TypeDef Button) {
	/* Wait until the Push Button is released */
	while (BSP_PB_GetState(BUTTON_KEY) == (uint32_t) GPIO_PIN_RESET)
		;

	PushButtonDetected = 1;
}

/**
 * @brief  Splits a float into two integer values.
 * @param  in the float value as input
 * @param  out_value the pointer to the output integer structure
 * @param  dec_prec the decimal precision to be used
 * @retval None
 */
void floatToInt(float in, displayFloatToInt_t *out_value, int32_t dec_prec) {
	if (in >= 0.0f) {
		out_value->sign = 0;
	} else {
		out_value->sign = 1;
		in = -in;
	}

	out_value->out_int = (int32_t) in;
	in = in - (float) (out_value->out_int);
	out_value->out_dec = (int32_t) trunc(in * pow(10, dec_prec));
}

/**
 * @brief  Handles the accelerometer axes data getting/sending
 * @param  Instance the device instance
 * @retval None
 */
IKS01A2_MOTION_SENSOR_Axes_t Accelero_Sensor_Handler(uint32_t Instance) {
	float odr;
	int32_t fullScale;
	IKS01A2_MOTION_SENSOR_Axes_t acceleration;
	displayFloatToInt_t out_value;
	uint8_t whoami;
	int verbose = 0;

	IKS01A2_MOTION_SENSOR_GetAxes(Instance, MOTION_ACCELERO, &acceleration);

	if (verbose == 1) {
		if (IKS01A2_MOTION_SENSOR_GetAxes(Instance, MOTION_ACCELERO,
				&acceleration)) {
			snprintf(dataOut, MAX_BUF_SIZE, "\r\nACC[%d]: Error\r\n",
					(int) Instance);
		} else {
			snprintf(dataOut, MAX_BUF_SIZE,
					"\r\nACC_X[%d]: %d, ACC_Y[%d]: %d, ACC_Z[%d]: %d\r\n",
					(int) Instance, (int) acceleration.x, (int) Instance,
					(int) acceleration.y, (int) Instance, (int) acceleration.z);
		}

		printf("%s", dataOut);
		return acceleration;

		if (IKS01A2_MOTION_SENSOR_ReadID(Instance, &whoami)) {
			snprintf(dataOut, MAX_BUF_SIZE, "WHOAMI[%d]: Error\r\n",
					(int) Instance);
		} else {
			snprintf(dataOut, MAX_BUF_SIZE, "WHOAMI[%d]: 0x%x\r\n",
					(int) Instance, (int) whoami);
		}

		printf("%s", dataOut);

		if (IKS01A2_MOTION_SENSOR_GetOutputDataRate(Instance, MOTION_ACCELERO,
				&odr)) {
			snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: ERROR\r\n",
					(int) Instance);
		} else {
			floatToInt(odr, &out_value, 3);
			snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: %d.%03d Hz\r\n",
					(int) Instance, (int) out_value.out_int,
					(int) out_value.out_dec);
		}

		printf("%s", dataOut);

		if (IKS01A2_MOTION_SENSOR_GetFullScale(Instance, MOTION_ACCELERO,
				&fullScale)) {
			snprintf(dataOut, MAX_BUF_SIZE, "FS[%d]: ERROR\r\n",
					(int) Instance);
		} else {
			snprintf(dataOut, MAX_BUF_SIZE, "FS[%d]: %d g\r\n", (int) Instance,
					(int) fullScale);
		}

		printf("%s", dataOut);
	}

	return acceleration;

}

/**
 * @brief  Handles the gyroscope axes data getting/sending
 * @param  Instance the device instance
 * @retval None
 */
IKS01A2_MOTION_SENSOR_Axes_t Gyro_Sensor_Handler(uint32_t Instance) {
	float odr;
	int32_t fullScale;
	IKS01A2_MOTION_SENSOR_Axes_t angular_velocity;
	displayFloatToInt_t out_value;
	uint8_t whoami;
	int verbose = 0;

	IKS01A2_MOTION_SENSOR_GetAxes(Instance, MOTION_GYRO, &angular_velocity);

	if (verbose == 1) {

		if (IKS01A2_MOTION_SENSOR_GetAxes(Instance, MOTION_GYRO,
				&angular_velocity)) {
			snprintf(dataOut, MAX_BUF_SIZE, "\r\nGYR[%d]: Error\r\n",
					(int) Instance);
		} else {
			snprintf(dataOut, MAX_BUF_SIZE,
					"\r\nGYR_X[%d]: %d, GYR_Y[%d]: %d, GYR_Z[%d]: %d\r\n",
					(int) Instance, (int) angular_velocity.x, (int) Instance,
					(int) angular_velocity.y, (int) Instance,
					(int) angular_velocity.z);
		}

		printf("%s", dataOut);
		return angular_velocity;

		if (IKS01A2_MOTION_SENSOR_ReadID(Instance, &whoami)) {
			snprintf(dataOut, MAX_BUF_SIZE, "WHOAMI[%d]: Error\r\n",
					(int) Instance);
		} else {
			snprintf(dataOut, MAX_BUF_SIZE, "WHOAMI[%d]: 0x%x\r\n",
					(int) Instance, (int) whoami);
		}

		printf("%s", dataOut);

		if (IKS01A2_MOTION_SENSOR_GetOutputDataRate(Instance, MOTION_GYRO,
				&odr)) {
			snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: ERROR\r\n",
					(int) Instance);
		} else {
			floatToInt(odr, &out_value, 3);
			snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: %d.%03d Hz\r\n",
					(int) Instance, (int) out_value.out_int,
					(int) out_value.out_dec);
		}

		printf("%s", dataOut);

		if (IKS01A2_MOTION_SENSOR_GetFullScale(Instance, MOTION_GYRO,
				&fullScale)) {
			snprintf(dataOut, MAX_BUF_SIZE, "FS[%d]: ERROR\r\n",
					(int) Instance);
		} else {
			snprintf(dataOut, MAX_BUF_SIZE, "FS[%d]: %d dps\r\n",
					(int) Instance, (int) fullScale);
		}

		printf("%s", dataOut);
	}

	return angular_velocity;

}

/**
 * @brief  Handles the magneto axes data getting/sending
 * @param  Instance the device instance
 * @retval None
 */
IKS01A2_MOTION_SENSOR_Axes_t Magneto_Sensor_Handler(uint32_t Instance) {
	float ans_float;

	MFX_MagCal_input_t mag_data_in;
	MFX_MagCal_output_t mag_data_out;
	IKS01A2_MOTION_SENSOR_Axes_t MagValue;

	IKS01A2_MOTION_SENSOR_Axes_t MagOffset;

	uint8_t MagCalStatus = 0;
	uint32_t MagTimeStamp = 0;

	if ((SensorsEnabled & MAGNETIC_SENSOR) == MAGNETIC_SENSOR) {
		(void) IKS01A2_MOTION_SENSOR_GetAxes(Instance, MOTION_MAGNETO,
				&MagValue);

		if (MagCalStatus == 0U) {
			mag_data_in.mag[0] = (float) MagValue.x * FROM_MGAUSS_TO_UT50;
			mag_data_in.mag[1] = (float) MagValue.y * FROM_MGAUSS_TO_UT50;
			mag_data_in.mag[2] = (float) MagValue.z * FROM_MGAUSS_TO_UT50;

			mag_data_in.time_stamp = (int) MagTimeStamp;
			MagTimeStamp += (uint32_t) ALGO_PERIOD;

			MotionFX_manager_MagCal_run(&mag_data_in, &mag_data_out);

			if (mag_data_out.cal_quality == MFX_MAGCALGOOD) {
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

		return MagValue;
	}
}

/**
 * @brief  Handles the temperature data getting/sending
 * @param  Instance the device instance
 * @retval None
 */
float Temp_Sensor_Handler(uint32_t Instance) {
	float odr;
	float temperature = 0;
	displayFloatToInt_t out_value;
	uint8_t whoami;
	int verbose = 0;

	IKS01A2_ENV_SENSOR_GetValue(Instance, ENV_TEMPERATURE, &temperature);

	if (verbose == 1) {

		if (IKS01A2_ENV_SENSOR_GetValue(Instance, ENV_TEMPERATURE,
				&temperature)) {
			snprintf(dataOut, MAX_BUF_SIZE, "\r\nTemp[%d]: Error\r\n",
					(int) Instance);
		} else {
			floatToInt(temperature, &out_value, 2);
			snprintf(dataOut, MAX_BUF_SIZE, "\r\nTemp[%d]: %d.%02d degC\r\n",
					(int) Instance, (int) out_value.out_int,
					(int) out_value.out_dec);
		}

		printf("%s", dataOut);
		return temperature;

		if (verbose == 1) {
			if (IKS01A2_ENV_SENSOR_ReadID(Instance, &whoami)) {
				snprintf(dataOut, MAX_BUF_SIZE, "WHOAMI[%d]: Error\r\n",
						(int) Instance);
			} else {
				snprintf(dataOut, MAX_BUF_SIZE, "WHOAMI[%d]: 0x%x\r\n",
						(int) Instance, (int) whoami);
			}

			printf("%s", dataOut);

			if (IKS01A2_ENV_SENSOR_GetOutputDataRate(Instance, ENV_TEMPERATURE,
					&odr)) {
				snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: Error\r\n",
						(int) Instance);
			} else {
				floatToInt(odr, &out_value, 3);
				snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: %d.%03d Hz\r\n",
						(int) Instance, (int) out_value.out_int,
						(int) out_value.out_dec);
			}

			printf("%s", dataOut);
		}
	}

	return temperature;

}

/**
 * @brief  Handles the pressure sensor data getting/sending
 * @param  Instance the device instance
 * @retval None
 */
float Press_Sensor_Handler(uint32_t Instance) {
	float odr;
	float pressure;
	displayFloatToInt_t out_value;
	uint8_t whoami;
	int verbose = 0;

	IKS01A2_ENV_SENSOR_GetValue(Instance, ENV_PRESSURE, &pressure);

	if (verbose == 1) {

		if (IKS01A2_ENV_SENSOR_GetValue(Instance, ENV_PRESSURE, &pressure)) {
			snprintf(dataOut, MAX_BUF_SIZE, "\r\nPress[%d]: Error\r\n",
					(int) Instance);
		} else {
			floatToInt(pressure, &out_value, 2);
			snprintf(dataOut, MAX_BUF_SIZE, "\r\nPress[%d]: %d.%02d hPa\r\n",
					(int) Instance, (int) out_value.out_int,
					(int) out_value.out_dec);
		}

		printf("%s", dataOut);
		return pressure;

		if (IKS01A2_ENV_SENSOR_ReadID(Instance, &whoami)) {
			snprintf(dataOut, MAX_BUF_SIZE, "WHOAMI[%d]: Error\r\n",
					(int) Instance);
		} else {
			snprintf(dataOut, MAX_BUF_SIZE, "WHOAMI[%d]: 0x%x\r\n",
					(int) Instance, (int) whoami);
		}

		printf("%s", dataOut);

		if (IKS01A2_ENV_SENSOR_GetOutputDataRate(Instance, ENV_PRESSURE,
				&odr)) {
			snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: Error\r\n",
					(int) Instance);
		} else {
			floatToInt(odr, &out_value, 3);
			snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: %d.%03d Hz\r\n",
					(int) Instance, (int) out_value.out_int,
					(int) out_value.out_dec);
		}

		printf("%s", dataOut);
	}

	return pressure;

}

/**
 * @brief  Handles the humidity data getting/sending
 * @param  Instance the device instance
 * @retval None
 */
float Hum_Sensor_Handler(uint32_t Instance) {
	float odr;
	float humidity;
	displayFloatToInt_t out_value;
	uint8_t whoami;
	int verbose = 0;

	IKS01A2_ENV_SENSOR_GetValue(Instance, ENV_HUMIDITY, &humidity);

	if (verbose == 1) {

		if (IKS01A2_ENV_SENSOR_GetValue(Instance, ENV_HUMIDITY, &humidity)) {
			snprintf(dataOut, MAX_BUF_SIZE, "\r\nHum[%d]: Error\r\n",
					(int) Instance);
		} else {
			floatToInt(humidity, &out_value, 2);
			snprintf(dataOut, MAX_BUF_SIZE, "\r\nHum[%d]: %d.%02d %%\r\n",
					(int) Instance, (int) out_value.out_int,
					(int) out_value.out_dec);
		}

		printf("%s", dataOut);
		return humidity;

		if (IKS01A2_ENV_SENSOR_ReadID(Instance, &whoami)) {
			snprintf(dataOut, MAX_BUF_SIZE, "WHOAMI[%d]: Error\r\n",
					(int) Instance);
		} else {
			snprintf(dataOut, MAX_BUF_SIZE, "WHOAMI[%d]: 0x%x\r\n",
					(int) Instance, (int) whoami);
		}

		printf("%s", dataOut);

		if (IKS01A2_ENV_SENSOR_GetOutputDataRate(Instance, ENV_HUMIDITY,
				&odr)) {
			snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: Error\r\n",
					(int) Instance);
		} else {
			floatToInt(odr, &out_value, 3);
			snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: %d.%03d Hz\r\n",
					(int) Instance, (int) out_value.out_int,
					(int) out_value.out_dec);
		}

		printf("%s", dataOut);
	}

	return humidity;

}

/**
 * @brief  Sensor Fusion data handler
 * @param  Msg the Sensor Fusion data part of the stream
 * @retval None
 */
MFX_output_t FX_Data_Handler(IKS01A2_MOTION_SENSOR_Axes_t AccValue,
		IKS01A2_MOTION_SENSOR_Axes_t GyrValue,
		IKS01A2_MOTION_SENSOR_Axes_t MagValue) {
	MFX_input_t data_in;
	MFX_input_t *pdata_in = &data_in;
	MFX_output_t data_out;
	MFX_output_t *pdata_out = &data_out;

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
			}
		}
	}

	data_out = *pdata_out;

	return data_out;
}

MFX_output_t run_mems_process() {
	IKS01A2_MOTION_SENSOR_Axes_t AccValue;
	IKS01A2_MOTION_SENSOR_Axes_t GyrValue;
	IKS01A2_MOTION_SENSOR_Axes_t MagValue;
	float Temperature = 0;
	float Pressure = 0;
	float Humidity = 0;

	TMsg msg_dat;

	MFX_output_t data_out;

	/* Acquire data from enabled sensors and fill Msg stream */
	RTC_Handler(&msg_dat);
	AccValue = Accelero_Sensor_Handler(IKS01A2_LSM6DSL_0);
	GyrValue = Gyro_Sensor_Handler(IKS01A2_LSM6DSL_0);
	MagValue = Magneto_Sensor_Handler(IKS01A2_LSM303AGR_MAG_0);
	Humidity = Hum_Sensor_Handler(IKS01A2_HTS221_0);
	Temperature = Temp_Sensor_Handler(IKS01A2_HTS221_0);
	Pressure = Press_Sensor_Handler(IKS01A2_LPS22HB_0);

	/* Sensor Fusion specific part */
	data_out = FX_Data_Handler(AccValue, GyrValue, MagValue);

	data_out.temperature = Temperature;
	data_out.pressure = Pressure;
	data_out.humidity = Humidity;

	return data_out;
}


#ifdef __cplusplus
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
