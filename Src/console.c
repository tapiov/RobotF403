/**
 ******************************************************************************
 * @file    console.c
 * @author  Central LAB
 * @version V2.2.0
 * @date    29-September-2015
 * @brief   This file provides implementation of standard input/output
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *        http://www.st.com/software_license_agreement_liberty_v2
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

#include <stdio.h>
#include <stdlib.h>

#include "main.h"

int cuiGetInteger(const char* message);
int uartSendChar(int ch);
int uartReceiveChar(void);

volatile static int uart6TXReady = 1;
volatile static int uart2TXReady = 1;
volatile static int uartRXReady = 1;

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;

/** @brief Asks user to input an integer number and returns its value
 * @param message A message to be prompted on the console when asking value
 * @retval The integer acquired from console
 */
int cuiGetInteger(const char* message)
{
	int ret, value;
	char buf[32];
	do {
		printf("%s", message);
		fflush(stdout);
		ret = scanf("%u", &value);
		if (ret != 1) {
			/* In case of non-matching characters, eat stdin as IAR and
			 * TrueStudio won't flush it as Keil does */
			scanf("%s", buf);
			printf("\r\nPlease insert a valid integer number\r\n");
		}
	} while (ret != 1);

	return value;
}

/** @brief Sends a character to serial port
 * @param ch Character to send
 * @retval Character sent
 */
int uartSendChar(int ch)
{
	while ((uart2TXReady == 0) | (uart6TXReady == 0)) {
		;
	}

	uart2TXReady = 0;
	uart6TXReady = 1;

	// HAL_UART_Transmit_DMA(&huart6, (uint8_t *)&ch, 1);
	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&ch, 1);

	while ((uart2TXReady == 0) | (uart6TXReady == 0)) {
		;
	}

	return ch;
}

/** @brief Receives a character from serial port
 * @param None
 * @retval Character received
 */
int uartReceiveChar(void)
{
	uint8_t ch;

	while (uartRXReady == 0) {
		;
	}

	uartRXReady = 0;

	HAL_UART_Receive_DMA(&huart6, &ch, 1);

	while (uartRXReady == 0) {
		;
	}

	return ch;
}

/** @brief putchar call for standard output implementation
 * @param ch Character to print
 * @retval Character printed
 */
int __io_putchar(int ch)
{
	uartSendChar(ch);

	return 0;
}

/** @brief getchar call for standard input implementation
 * @param None
 * @retval Character acquired from standard input
 */
int __io_getchar(void)
{
	return uartReceiveChar();
}

/**
 * @brief  Tx Transfer completed callback
 * @param  UartHandle: UART handle.
 * @note   This example shows a simple way to report end of DMA Tx transfer, and
 *         you can add your own implementation.
 * @retval None
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	/* Set transmission flag: trasfer complete*/

	if (UartHandle->Instance == USART2) uart2TXReady = 1;
	if (UartHandle->Instance == USART6) uart6TXReady = 1;
}

/**
 * @brief  Rx Transfer completed callback
 * @param  UartHandle: UART handle
 * @note   This example shows a simple way to report end of DMA Rx transfer, and
 *         you can add your own implementation.
 * @retval None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	/* Set transmission flag: trasfer complete*/
	uartRXReady = 1;
}

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/

