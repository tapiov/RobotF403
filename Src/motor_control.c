/*
 * motor_control.c
 *
 *  Created on: 29 Nov 2018
 *      Author: tapio
 */


#include "stdio.h"
#include "string.h"

#include "l6206.h"
#include "x_nucleo_ihmxx.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

static volatile uint16_t gLastError;

/* Private function prototypes -----------------------------------------------*/

static void MyFlagInterruptHandler(void);
void Error_Handler(uint16_t error);

/* Private functions ---------------------------------------------------------*/

void motor_init(void)
{
	deviceParams_t initDeviceParameters =
	{
		L6206_CONF_PARAM_PARALLE_BRIDGES,
		{ L6206_CONF_PARAM_FREQ_PWM1A, L6206_CONF_PARAM_FREQ_PWM2A, L6206_CONF_PARAM_FREQ_PWM1B, L6206_CONF_PARAM_FREQ_PWM2B },
		{ 100, 100, 100, 100 },
		{ FORWARD, FORWARD, BACKWARD, FORWARD },
		{ INACTIVE, INACTIVE, INACTIVE, INACTIVE },
		{ FALSE, FALSE }
	};

	//----- Init of the Motor control library
	/* Set the L6208 library to use 2 device */
	BSP_MotorControl_SetNbDevices(BSP_MOTOR_CONTROL_BOARD_ID_L6206, 2);
	/* When BSP_MotorControl_Init is called with NULL pointer,                  */
	/* the L6206 parameters are set with the predefined values from file        */
	/* l6206_target_config.h, otherwise the parameters are set using the        */
	/* initDeviceParameters structure values.                                   */
	BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_L6206, &initDeviceParameters);

	/* Select the configuration with paralleling of brigde input IN1A with IN2A,
	   and with paralleling of brigde input IN1B with IN2B with the use of one
	   bidirectionnal motor */
	BSP_MotorControl_SetDualFullBridgeConfig(PARALLELING_NONE___1_BIDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B);

	/* Attach the function MyFlagInterruptHandler (defined below) to the flag interrupt */
	BSP_MotorControl_AttachFlagInterrupt(MyFlagInterruptHandler);

	/* Attach the function Error_Handler (defined below) to the error Handler*/
	BSP_MotorControl_AttachErrorHandler(Error_Handler);

	/* Set PWM Frequency of bridge A inputs to 10000 Hz */
	BSP_MotorControl_SetBridgeInputPwmFreq(0, 10000);

	/* Set PWM Frequency of bridge B inputs to 10000 Hz */
	BSP_MotorControl_SetBridgeInputPwmFreq(1, 10000);
}

void robot_move(motorDir_t dir, int speed)
{
	if (dir == FORWARD) {
		//printf_rtc_timestamp();
		//printf("Running forward\n");

		BSP_MotorControl_SetMaxSpeed(0, speed);
		BSP_MotorControl_Run(0, BACKWARD);

		BSP_MotorControl_SetMaxSpeed(1, speed);
		BSP_MotorControl_Run(1, FORWARD);
	} else if (dir == BACKWARD) {
		//printf_rtc_timestamp();
		//printf("Running backward\n");

		BSP_MotorControl_SetMaxSpeed(0, 20);
		BSP_MotorControl_Run(0, FORWARD);

		BSP_MotorControl_SetMaxSpeed(1, 20);
		BSP_MotorControl_Run(1, BACKWARD);
	}
}

void robot_turn(turnDir_t turn, int speed)
{
	if (turn == RIGHT) {
		//printf_rtc_timestamp();
		//printf("Turning right\n");

		BSP_MotorControl_SetMaxSpeed(0, speed);
		BSP_MotorControl_Run(0, BACKWARD);

		BSP_MotorControl_SetMaxSpeed(1, speed);
		BSP_MotorControl_Run(1, BACKWARD);
	} else if (turn == LEFT) {
		//printf_rtc_timestamp();
		//printf("Turning left\n");

		BSP_MotorControl_SetMaxSpeed(0, speed);
		BSP_MotorControl_Run(0, FORWARD);

		BSP_MotorControl_SetMaxSpeed(1, speed);
		BSP_MotorControl_Run(1, FORWARD);
	}
}

void robot_stop(void)
{
	//printf_rtc_timestamp();
	//printf("Stop\n");

	BSP_MotorControl_SetMaxSpeed(0, 0);
	BSP_MotorControl_Run(0, FORWARD);

	BSP_MotorControl_SetMaxSpeed(1, 0);
	BSP_MotorControl_Run(1, BACKWARD);
}

/**
 * @brief  This function is the User handler for the flag interrupt
 * @param  None
 * @retval None
 */
void MyFlagInterruptHandler(void)
{
	/* Code to be customised */
	/************************/
	/* Get the state of bridge A */
	uint16_t bridgeState = BSP_MotorControl_CmdGetStatus(0);

	if (bridgeState == 0) {
		if (BSP_MotorControl_GetDeviceState(0) != INACTIVE) {
			/* Bridge A was disabling due to overcurrent or over temperature */
			/* When  motor was running */
			Error_Handler(0XBAD0);
		}
	}
}

///**
//  * @brief  This function is executed in case of error occurrence.
//  * @param  error number of the error
//  * @retval None
//  */
//void Error_Handler(uint16_t error)
//{
//  /* Backup error number */
//  gLastError = error;
//
//  /* Infinite loop */
//  while(1)
//  {
//  }
//}


#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
//void assert_failed(uint8_t* file, uint32_t line)
//{
//	/* User can add his own implementation to report the file name and line number,
//	   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//
//	/* Infinite loop */
//	while (1) {
//	}
//}
#endif

/**
 * @}
 */

