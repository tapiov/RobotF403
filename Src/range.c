/*
 * range.c
 *
 *  Created on: Feb 25, 2019
 *      Author: tapio
 */

#include "range.h"


// Static vars
	static VL53L1_Dev_t devCenter;
	static VL53L1_Dev_t devLeft;
	static VL53L1_Dev_t devRight;
	static VL53L1_Dev_t Dev;

/* An example here below shows how to manage multi-sensor operation.
 In this example the sensors range sequentially. Several sensors range simultanously is also possible */

/* Reset the 3 ToF sensors on the expansion board */

void range_init() {

	I2C_HandleTypeDef hi2c1;

	uint16_t wordData;
	uint8_t ToFSensor = 1; // 0=Left, 1=Center(default), 2=Right
	uint8_t newI2C = 0x52;

	uint8_t VL53L1_Status;

	for (ToFSensor = 0; ToFSensor < 3; ToFSensor++) {
		VL53L1_Status = XNUCLEO53L1A1_ResetId(ToFSensor, 0);
	}

	/* Bring the sensors out of the reset stage one by one and set the new I2C address */
	for (ToFSensor = 0; ToFSensor < 3; ToFSensor++) {
		switch (ToFSensor) {
		case 0:
			Dev = devCenter;
			break;
		case 1:
			Dev = devLeft;
			break;
		case 2:
			Dev = devRight;
			break;
		}

		VL53L1_Status = XNUCLEO53L1A1_ResetId(ToFSensor, 1);
		Dev.comms_speed_khz = 400;
		Dev.I2cHandle = &hi2c1;
		Dev.comms_type = 1;
		Dev.I2cDevAddr = 0x52; /* default ToF sensor I2C address*/
		VL53L1_RdWord(&Dev, 0x010F, &wordData);
		printf("VL53L1X: %02X\n", wordData);
		newI2C = Dev.I2cDevAddr + (ToFSensor + 1) * 2;
		VL53L1_Status = VL53L1_SetDeviceAddress(&Dev, newI2C);
		Dev.I2cDevAddr = newI2C;
		VL53L1_RdWord(&Dev, 0x010F, &wordData);
		printf("VL53L1X: %02X\n", wordData);

		/* Device Initialization and setting */
		VL53L1_Status = VL53L1_WaitDeviceBooted(&Dev);
		VL53L1_Status = VL53L1_DataInit(&Dev);
		VL53L1_Status = VL53L1_StaticInit(&Dev);
		VL53L1_Status = VL53L1_SetDistanceMode(&Dev, VL53L1_DISTANCEMODE_LONG);
		VL53L1_Status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(&Dev,
				50000);
		VL53L1_Status = VL53L1_SetInterMeasurementPeriodMilliSeconds(&Dev, 100);
	}
}

void MeasureSensors(VL53L1_RangingMeasurementData_t (*RangeData)[3]) {

	uint8_t ToFSensor;

	VL53L1_RangingMeasurementData_t RangingData;

	VL53L1_Dev_t devCenter;
	VL53L1_Dev_t devLeft;
	VL53L1_Dev_t devRight;
	VL53L1_Dev_t Dev;

	int VL53L1_Status;

	for (ToFSensor = 0; ToFSensor < 3; ToFSensor++) {

		switch (ToFSensor) {
		case 0:
			Dev = devCenter;
			break;
		case 1:
			Dev = devLeft;
			break;
		case 2:
			Dev = devRight;
			break;
		}

		VL53L1_Status = VL53L1_StartMeasurement(&Dev);
		VL53L1_Status = VL53L1_WaitMeasurementDataReady(&Dev);
		if (!VL53L1_Status) {
			VL53L1_Status = VL53L1_GetRangingMeasurementData(&Dev, &RangingData);

			*RangeData[ToFSensor] = RangingData;

			VL53L1_Status = VL53L1_ClearInterruptAndStartMeasurement(&Dev);
		}
	}
}

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

