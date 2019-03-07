/*
 * rtc.c
 *
 *  Created on: Feb 3, 2019
 *      Author: tapio
 */

#include "mems.h"

// Prototypes
void MX_TIM_ALGO_Init(void);
RTC_TimeTypeDef RTC_Handler(void);
uint8_t RTC_GetReadRequest(void);
void RTC_ResetReadRequest(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

RTC_TimeTypeDef RTC_TimeStamp(void);
RTC_TimeTypeDef RTC_Handler(void);
void RTC_Config(void);
void RTC_TimeStampConfig(void);

// Read request flag
static volatile uint8_t SensorReadRequest = 0;

TIM_HandleTypeDef htim5;
static int RtcSynchPrediv;
static RTC_HandleTypeDef RtcHandle;

/**
 * @brief  TIM_ALGO init function.
 * @param  None
 * @retval None
 * @details This function intializes the Timer used to synchronize the algorithm.
 */
void MX_TIM_ALGO_Init(void) {

#define CPU_CLOCK  84000000U
#define TIM_CLOCK  2000U

	const uint32_t prescaler = CPU_CLOCK / TIM_CLOCK - 1U;
	const uint32_t tim_period = TIM_CLOCK / ALGO_FREQ - 1U;

	TIM_ClockConfigTypeDef s_clock_source_config;
	TIM_MasterConfigTypeDef s_master_config;

	htim5.Instance = TIM5;
	htim5.Init.Prescaler = prescaler;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = tim_period;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	(void) HAL_TIM_Base_Init(&htim5);

	s_clock_source_config.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	(void) HAL_TIM_ConfigClockSource(&htim5, &s_clock_source_config);

	s_master_config.MasterOutputTrigger = TIM_TRGO_RESET;
	s_master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	(void) HAL_TIMEx_MasterConfigSynchronization(&htim5,
			&s_master_config);
}

/**
 * @brief  Handles the time+date getting/sending
 * @param  Msg the time+date part of the stream
 * @retval None
 */
RTC_TimeTypeDef RTC_Handler() {
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

	stimestructure.SubSeconds = sub_sec;

	return stimestructure;
}

/**
 * @brief  Returns the SensorReadRequest value
 * @param  None
 * @retval uint8_t SensorReadRequest
 */
uint8_t RTC_GetReadRequest(void) {
	return SensorReadRequest;
}

/**
 * @brief  Set the SensorReadRequest value
 * @param  uint8_t new value for SensorReadRequest
 * @retval None
 */
void RTC_ResetReadRequest(void) {
	NVIC_DisableIRQ(TIM_ALGO_IRQn);
	SensorReadRequest = 0;
	NVIC_EnableIRQ(TIM_ALGO_IRQn);
}

/**
 * @brief  Return the current RTC Timestamp
 * @param  None
 * @retval RTC_TimeTypeDef stimestructure
 */
RTC_TimeTypeDef RTC_TimeStamp(void) {
	RTC_TimeTypeDef stimestructure;

	stimestructure = RTC_Handler();
	return stimestructure;
}

/**
 * @brief  Print the RTC Timestamp HH:MM:SS:sss
 * @param  None
 * @retval None
 */
void Print_TimeStamp(void) {
	RTC_TimeTypeDef stimestructure;
	static char dataOut[20];

	stimestructure = RTC_Handler();

	snprintf(dataOut, 20, "T %2d:%2d:%2d:%3lu ", stimestructure.Hours,
			stimestructure.Minutes, stimestructure.Seconds,
			stimestructure.SubSeconds);

	printf(dataOut);
}

/**
 * @brief  Configures the RTC
 * @param  None
 * @retval None
 */
void RTC_Config(void) {
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
		RtcHandle.Init.AsynchPrediv = RTC_ASYNCH_PREDIV_LSI;
		RtcHandle.Init.SynchPrediv = RTC_SYNCH_PREDIV_LSI;
		RtcSynchPrediv = RTC_SYNCH_PREDIV_LSI;
	} else {
		/* We use LSE */
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
	RtcHandle.Init.HourFormat = RTC_HOURFORMAT_24;
	RtcHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
	RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	RtcHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;

	if (HAL_RTC_Init(&RtcHandle) != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}

	// Time start 00:00:00
	RTC_TimeRegulate(00, 00, 00);

	// Date start Tue 1.1.2019
	RTC_DateRegulate(19, 1, 1, 2);

}

/**
 * @brief  Configures the current time and date
 * @param  None
 * @retval None
 */
void RTC_TimeStampConfig(void) {
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
	stimestructure.TimeFormat = RTC_HOURFORMAT_24;
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

	stimestructure.TimeFormat = RTC_HOURFORMAT_24;
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

/**
 * @brief  Period elapsed callback
 * @param  htim pointer to a TIM_HandleTypeDef structure that contains
 *              the configuration information for TIM module.
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM_ALGO) {
		SensorReadRequest = 1;
	}
}

