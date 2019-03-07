/**
 ******************************************************************************
 * File Name          :  stmicroelectronics_x-cube-mems1_1_0_0.h
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MEMS_H
#define __MEMS_H
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "nucleo_f401re_bus.h"
#include "nucleo_f401re_errno.h"

#include "MotionFX_Manager.h"
#include "serial_protocol.h"

/* Exported Functions --------------------------------------------------------*/
void MX_X_CUBE_MEMS1_Init(void);
void MX_X_CUBE_MEMS1_Process(void);

#define Enabled6X 										1

#define IKS01A2_I2C_Init BSP_I2C1_Init
#define IKS01A2_I2C_DeInit BSP_I2C1_DeInit
#define IKS01A2_I2C_ReadReg BSP_I2C1_ReadReg
#define IKS01A2_I2C_WriteReg BSP_I2C1_WriteReg
#define IKS01A2_GetTick BSP_GetTick

// For IKS01A2 from main.h
#define STORE_CALIB_FLASH

/* Definition for TIMx clock resources : Timer used for algorithm */
#define TIM_ALGO                          TIM5
#define TIM_ALGO_CLK_ENABLE               __TIM5_CLK_ENABLE
#define TIM_ALGO_CLK_DISABLE              __TIM5_CLK_DISABLE

/* Definition for TIMx's NVIC */
#define TIM_ALGO_IRQn                     TIM5_IRQn
#define TIM_ALGO_IRQHandler               TIM5_IRQHandler

/* Enable sensor masks */

// All sensors on
#define SensorsEnabled							0xffffffff

#define PRESSURE_SENSOR                         0x00000001U
#define TEMPERATURE_SENSOR                      0x00000002U
#define HUMIDITY_SENSOR                         0x00000004U
#define UV_SENSOR                               0x00000008U /* for future use */
#define ACCELEROMETER_SENSOR                    0x00000010U
#define GYROSCOPE_SENSOR                        0x00000020U
#define MAGNETIC_SENSOR                         0x00000040U

#define ALGO_FREQ    100U /* Algorithm frequency [Hz] */
#define ALGO_PERIOD  10   /* Algorithm period [ms] */
#define MOTIONFX_ENGINE_DELTATIME  0.01f

#define FROM_MG_TO_G                    0.001f
#define FROM_G_TO_MG                    1000.0f
#define FROM_MDPS_TO_DPS                0.001f
#define FROM_DPS_TO_MDPS                1000.0f
#define FROM_MGAUSS_TO_UT50             (0.1f / 50.0f)
#define FROM_UT50_TO_MGAUSS             500.0f

#define LPS25HB_UNICLEO_ID_ONBOARD 1
#define LPS25HB_UNICLEO_ID_DIL 2
#define LPS22HB_UNICLEO_ID 3
#define HTS221_UNICLEO_ID 1
#define LSM6DS0_UNICLEO_ID 1
#define LSM6DS3_UNICLEO_ID 2
#define LSM6DSL_UNICLEO_ID 3
#define LIS3MDL_UNICLEO_ID 1
#define LSM303AGR_UNICLEO_ID_MAG 2

#define FW_ID "4"
#define FW_VERSION "2.3.0"
#define LIB_VERSION "2.1.0"
#define EXPANSION_BOARD "IKS01A2"

#define DATA_TX_LEN  MIN(4, DATABYTE_LEN)

#ifdef __cplusplus
}
#endif

#endif /* __MEMS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
