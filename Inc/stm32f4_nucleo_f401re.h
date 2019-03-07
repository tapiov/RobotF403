/**
 ******************************************************************************
 * @file  : nucleo_f401re.h
 * @brief : header file for the BSP Common driver
 ******************************************************************************
 * This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * Copyright (c) 2018 STMicroelectronics International N.V.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __NUCLEO_F401RE_H
#define __NUCLEO_F401RE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdio.h>

/** @addtogroup BSP
 * @{
 */

/** @addtogroup NUCLEO_F401RE
 * @{
 */

/** @addtogroup NUCLEO_F401RE_LOW_LEVEL
 * @{
 */

/** @defgroup NUCLEO_F401RE_LOW_LEVEL_Exported_Types NUCLEO_F401RE LOW LEVEL Exported Types
 * @{
 */

typedef enum {
	COM1 = 0U,
} COM_TypeDef;
#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
typedef struct {
	pUART_CallbackTypeDef pMspUartInitCb;
	pUART_CallbackTypeDef pMspUartDeInitCb;
} BSP_UART_Cb_t;
#endif /* (USE_HAL_UART_REGISTER_CALLBACKS == 1) */

/**
 * @}
 */

/** @defgroup NUCLEO_F401RE_LOW_LEVEL_Exported_Constants NUCLEO_F401RE LOW LEVEL Exported Constants
 * @{
 */
/**
 * @brief Define for NUCLEO_F401RE board
 */
/* Common Error codes */
#define BSP_ERROR_NONE                    0
#define BSP_ERROR_NO_INIT                -1
#define BSP_ERROR_WRONG_PARAM            -2
#define BSP_ERROR_BUSY                   -3
#define BSP_ERROR_PERIPH_FAILURE         -4
#define BSP_ERROR_COMPONENT_FAILURE      -5
#define BSP_ERROR_UNKNOWN_FAILURE        -6
#define BSP_ERROR_UNKNOWN_COMPONENT      -7
#define BSP_ERROR_BUS_FAILURE            -8
#define BSP_ERROR_CLOCK_FAILURE          -9
#define BSP_ERROR_MSP_FAILURE            -10
#define BSP_ERROR_FEATURE_NOT_SUPPORTED  -11

/** @defgroup NUCLEO_F401RE_LOW_LEVEL_LED NUCLEO_F401RE LOW LEVEL LED
 * @{
 */

#define LED2_GPIO_PORT                    GPIOA
#define LED2_GPIO_CLK_ENABLE()            __HAL_RCC_GPIOA_CLK_ENABLE()
#define LED2_GPIO_CLK_DISABLE()           __HAL_RCC_GPIOA_CLK_DISABLE()
#define LED2_GPIO_PIN                     GPIO_PIN_5
/**
 * @}
 */

/** @defgroup NUCLEO_F401RE_LOW_LEVEL_BUTTON NUCLEO_F401RE LOW LEVEL BUTTON
 * @{
 */
/* Button state */
#define BUTTON_RELEASED                   0U
#define BUTTON_PRESSED                    1U

/** @defgroup NUCLEO_F401RE_LOW_LEVEL_COM NUCLEO_F401RE LOW LEVEL COM
 * @{
 */
/**
 * @brief Definition for COM portx, connected to USART2
 */
#define COMn                             1U
#define COM1_UART                        USART2

#define COM_POLL_TIMEOUT                 1000

#define USE_COM_LOG                      1U

#define UartHandle huart2

typedef enum {
	COM_STOPBITS_1 =   UART_STOPBITS_1,
	COM_STOPBITS_2 =   UART_STOPBITS_2,
} COM_StopBitsTypeDef;

typedef enum {
	COM_PARITY_NONE =  UART_PARITY_NONE,
	COM_PARITY_EVEN =  UART_PARITY_EVEN,
	COM_PARITY_ODD  =  UART_PARITY_ODD,
} COM_ParityTypeDef;

typedef enum {
	COM_HWCONTROL_NONE    =  UART_HWCONTROL_NONE,
	COM_HWCONTROL_RTS     =  UART_HWCONTROL_RTS,
	COM_HWCONTROL_CTS     =  UART_HWCONTROL_CTS,
	COM_HWCONTROL_RTS_CTS =  UART_HWCONTROL_RTS_CTS,
} COM_HwFlowCtlTypeDef;

typedef struct {
	uint32_t BaudRate;
	uint32_t WordLength;
	COM_StopBitsTypeDef StopBits;
	COM_ParityTypeDef Parity;
	COM_HwFlowCtlTypeDef HwFlowCtl;
} COM_InitTypeDef;
/**
 * @}
 */

/** @defgroup NUCLEO_F401RE_LOW_LEVEL_Exported_Functions NUCLEO_F401RE LOW LEVEL Exported Functions
 * @{
 */
/* Exported Functions --------------------------------------------------------*/

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
#ifdef __cplusplus
}
#endif

#endif /* __NUCLEO_F401RE__H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
