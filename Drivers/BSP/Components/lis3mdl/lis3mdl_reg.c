/*
 ******************************************************************************
 * @file    lis3mdl_reg.c
 * @author  MEMS Software Solution Team
 * @brief   LIS3MDL driver file
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
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
 */

#include "lis3mdl_reg.h"

/**
 * @addtogroup  lis3mdl
 * @brief  This file provides a set of functions needed to drive the
 *         lis3mdl enanced inertial module.
 * @{
 */

/**
 * @addtogroup  interfaces_functions
 * @brief  This section provide a set of functions used to read and write
 *         a generic register of the device.
 * @{
 */

/**
 * @brief  Read generic device register
 *
 * @param  lis3mdl_ctx_t* ctx: read / write interface definitions
 * @param  uint8_t reg: register to read
 * @param  uint8_t* data: pointer to buffer that store the data read
 * @param  uint16_t len: number of consecutive register to read
 *
 */
int32_t lis3mdl_read_reg(lis3mdl_ctx_t *ctx, uint8_t reg, uint8_t *data,
			 uint16_t len)
{
	int32_t ret;
	ret = ctx->read_reg(ctx->handle, reg, data, len);
	return ret;
}

/**
 * @brief  Write generic device register
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  uint8_t reg: register to write
 * @param  uint8_t* data: pointer to data to write in register reg
 * @param  uint16_t len: number of consecutive register to write
 *
 */
int32_t lis3mdl_write_reg(lis3mdl_ctx_t *ctx, uint8_t reg, uint8_t *data,
			  uint16_t len)
{
	int32_t ret;
	ret = ctx->write_reg(ctx->handle, reg, data, len);
	return ret;
}

/**
 * @}
 */

/**
 * @addtogroup  data_generation_c
 * @brief   This section group all the functions concerning data generation
 * @{
 */

/**
 * @brief  data_rate: [set]  Output data rate selection.
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  lis3mdl_om_t: change the values of om in reg CTRL_REG1
 *
 */
int32_t lis3mdl_data_rate_set(lis3mdl_ctx_t *ctx, lis3mdl_om_t val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	ret = lis3mdl_read_reg(ctx, LIS3MDL_CTRL_REG1, &reg.byte, 1);
	if (ret == 0) {
		reg.ctrl_reg1.om = (uint8_t)val;
		ret = lis3mdl_write_reg(ctx, LIS3MDL_CTRL_REG1, &reg.byte, 1);
		if (ret == 0) {
			/* set mode also for z axis, ctrl_reg4 -> omz */
			ret = lis3mdl_read_reg(ctx, LIS3MDL_CTRL_REG4, &reg.byte, 1);
			if (ret == 0) {
				reg.ctrl_reg4.omz = (uint8_t)(((uint8_t)val >> 4) & 0x03U);
				ret = lis3mdl_write_reg(ctx, LIS3MDL_CTRL_REG4, &reg.byte, 1);
			}
		}
	}

	return ret;
}

/**
 * @brief  data_rate: [get]  Output data rate selection.
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  lis3mdl_om_t: Get the values of om in reg CTRL_REG1
 *
 */
int32_t lis3mdl_data_rate_get(lis3mdl_ctx_t *ctx, lis3mdl_om_t *val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	/* z axis, ctrl_reg4 -> omz is alligned with x/y axis ctrl_reg1 -> om*/
	ret = lis3mdl_read_reg(ctx, LIS3MDL_CTRL_REG1, &reg.byte, 1);
	switch (reg.ctrl_reg1.om) {
	case LIS3MDL_LP_Hz625:
		*val = LIS3MDL_LP_Hz625;
		break;
	case LIS3MDL_LP_1kHz:
		*val = LIS3MDL_LP_1kHz;
		break;
	case LIS3MDL_MP_560Hz:
		*val = LIS3MDL_MP_560Hz;
		break;
	case LIS3MDL_HP_300Hz:
		*val = LIS3MDL_HP_300Hz;
		break;
	case LIS3MDL_UHP_155Hz:
		*val = LIS3MDL_UHP_155Hz;
		break;
	case LIS3MDL_LP_1Hz25:
		*val = LIS3MDL_LP_1Hz25;
		break;
	case LIS3MDL_LP_2Hz5:
		*val = LIS3MDL_LP_2Hz5;
		break;
	case LIS3MDL_LP_5Hz:
		*val = LIS3MDL_LP_5Hz;
		break;
	case LIS3MDL_LP_10Hz:
		*val = LIS3MDL_LP_10Hz;
		break;
	case LIS3MDL_LP_20Hz:
		*val = LIS3MDL_LP_20Hz;
		break;
	case LIS3MDL_LP_40Hz:
		*val = LIS3MDL_LP_40Hz;
		break;
	case LIS3MDL_LP_80Hz:
		*val = LIS3MDL_LP_80Hz;
		break;
	case LIS3MDL_MP_1Hz25:
		*val = LIS3MDL_MP_1Hz25;
		break;
	case LIS3MDL_MP_2Hz5:
		*val = LIS3MDL_MP_2Hz5;
		break;
	case LIS3MDL_MP_5Hz:
		*val = LIS3MDL_MP_5Hz;
		break;
	case LIS3MDL_MP_10Hz:
		*val = LIS3MDL_MP_10Hz;
		break;
	case LIS3MDL_MP_20Hz:
		*val = LIS3MDL_MP_20Hz;
		break;
	case LIS3MDL_MP_40Hz:
		*val = LIS3MDL_MP_40Hz;
		break;
	case LIS3MDL_MP_80Hz:
		*val = LIS3MDL_MP_80Hz;
		break;
	case LIS3MDL_HP_1Hz25:
		*val = LIS3MDL_HP_1Hz25;
		break;
	case LIS3MDL_HP_2Hz5:
		*val = LIS3MDL_HP_2Hz5;
		break;
	case LIS3MDL_HP_5Hz:
		*val = LIS3MDL_HP_5Hz;
		break;
	case LIS3MDL_HP_10Hz:
		*val = LIS3MDL_HP_10Hz;
		break;
	case LIS3MDL_HP_20Hz:
		*val = LIS3MDL_HP_20Hz;
		break;
	case LIS3MDL_HP_40Hz:
		*val = LIS3MDL_HP_40Hz;
		break;
	case LIS3MDL_HP_80Hz:
		*val = LIS3MDL_HP_80Hz;
		break;
	case LIS3MDL_UHP_1Hz25:
		*val = LIS3MDL_UHP_1Hz25;
		break;
	case LIS3MDL_UHP_2Hz5:
		*val = LIS3MDL_UHP_2Hz5;
		break;
	case LIS3MDL_UHP_5Hz:
		*val = LIS3MDL_UHP_5Hz;
		break;
	case LIS3MDL_UHP_10Hz:
		*val = LIS3MDL_UHP_10Hz;
		break;
	case LIS3MDL_UHP_20Hz:
		*val = LIS3MDL_UHP_20Hz;
		break;
	case LIS3MDL_UHP_40Hz:
		*val = LIS3MDL_UHP_40Hz;
		break;
	case LIS3MDL_UHP_80Hz:
		*val = LIS3MDL_UHP_80Hz;
		break;
	default:
		*val = LIS3MDL_DATA_RATE_ND;
		break;
	}

	return ret;
}

/**
 * @brief  temperature_meas: [set]  Temperature sensor enable.
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  uint8_t val: change the values of temp_en in reg CTRL_REG1
 *
 */
int32_t lis3mdl_temperature_meas_set(lis3mdl_ctx_t *ctx, uint8_t val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	ret = lis3mdl_read_reg(ctx, LIS3MDL_CTRL_REG1, &reg.byte, 1);
	if (ret == 0) {
		reg.ctrl_reg1.temp_en = val;
		ret = lis3mdl_write_reg(ctx, LIS3MDL_CTRL_REG1, &reg.byte, 1);
	}
	return ret;
}

/**
 * @brief  temperature_meas: [get]  Temperature sensor enable.
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  uint8_t: change the values of temp_en in reg CTRL_REG1
 *
 */
int32_t lis3mdl_temperature_meas_get(lis3mdl_ctx_t *ctx, uint8_t *val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	ret = lis3mdl_read_reg(ctx, LIS3MDL_CTRL_REG1, &reg.byte, 1);
	*val = (uint8_t)reg.ctrl_reg1.temp_en;

	return ret;
}

/**
 * @brief  full_scale: [set]  Full-scale configuration.
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  lis3mdl_fs_t: change the values of fs in reg CTRL_REG2
 *
 */
int32_t lis3mdl_full_scale_set(lis3mdl_ctx_t *ctx, lis3mdl_fs_t val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	ret = lis3mdl_read_reg(ctx, LIS3MDL_CTRL_REG2, &reg.byte, 1);
	if (ret == 0) {
		reg.ctrl_reg2.fs = (uint8_t)val;
		ret = lis3mdl_write_reg(ctx, LIS3MDL_CTRL_REG2, &reg.byte, 1);
	}
	return ret;
}

/**
 * @brief  full_scale: [get]  Full-scale configuration.
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  lis3mdl_fs_t: Get the values of fs in reg CTRL_REG2
 *
 */
int32_t lis3mdl_full_scale_get(lis3mdl_ctx_t *ctx, lis3mdl_fs_t *val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	ret = lis3mdl_read_reg(ctx, LIS3MDL_CTRL_REG2, &reg.byte, 1);
	switch (reg.ctrl_reg2.fs) {
	case LIS3MDL_4_GAUSS:
		*val = LIS3MDL_4_GAUSS;
		break;
	case LIS3MDL_8_GAUSS:
		*val = LIS3MDL_8_GAUSS;
		break;
	case LIS3MDL_12_GAUSS:
		*val = LIS3MDL_12_GAUSS;
		break;
	case LIS3MDL_16_GAUSS:
		*val = LIS3MDL_16_GAUSS;
		break;
	default:
		*val = LIS3MDL_FS_ND;
		break;
	}

	return ret;
}

/**
 * @brief  operating_mode: [set]  Operating mode selection.
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  lis3mdl_md_t: change the values of md in reg CTRL_REG3
 *
 */
int32_t lis3mdl_operating_mode_set(lis3mdl_ctx_t *ctx, lis3mdl_md_t val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	ret = lis3mdl_read_reg(ctx, LIS3MDL_CTRL_REG3, &reg.byte, 1);
	if (ret == 0) {
		reg.ctrl_reg3.md = (uint8_t)val;
		ret = +lis3mdl_write_reg(ctx, LIS3MDL_CTRL_REG3, &reg.byte, 1);
	}

	return ret;
}

/**
 * @brief  operating_mode: [get]  Operating mode selection.
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  lis3mdl_md_t: Get the values of md in reg CTRL_REG3
 *
 */
int32_t lis3mdl_operating_mode_get(lis3mdl_ctx_t *ctx, lis3mdl_md_t *val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	ret = lis3mdl_read_reg(ctx, LIS3MDL_CTRL_REG3, &reg.byte, 1);
	switch (reg.ctrl_reg3.md) {
	case LIS3MDL_CONTINUOUS_MODE:
		*val = LIS3MDL_CONTINUOUS_MODE;
		break;
	case LIS3MDL_SINGLE_TRIGGER:
		*val = LIS3MDL_SINGLE_TRIGGER;
		break;
	case LIS3MDL_POWER_DOWN:
		*val = LIS3MDL_POWER_DOWN;
		break;
	default:
		*val = LIS3MDL_POWER_MODE_ND;
		break;
	}
	return ret;
}

/**
 * @brief  fast_low_power: [set]  If this bit is ‘1’, device is set in
 *                                low power to 0.625 Hz.
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  uint8_t val: change the values of lp in reg CTRL_REG3
 *
 */
int32_t lis3mdl_fast_low_power_set(lis3mdl_ctx_t *ctx, uint8_t val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	ret = lis3mdl_read_reg(ctx, LIS3MDL_CTRL_REG3, &reg.byte, 1);
	if (ret == 0) {
		reg.ctrl_reg3.lp = val;
		ret = lis3mdl_write_reg(ctx, LIS3MDL_CTRL_REG3, &reg.byte, 1);
	}
	return ret;
}

/**
 * @brief  fast_low_power: [get]  If this bit is ‘1’, device is set in
 *                                low power to 0.625 Hz.
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  uint8_t: change the values of lp in reg CTRL_REG3
 *
 */
int32_t lis3mdl_fast_low_power_get(lis3mdl_ctx_t *ctx, uint8_t *val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	ret = lis3mdl_read_reg(ctx, LIS3MDL_CTRL_REG3, &reg.byte, 1);
	*val = (uint8_t)reg.ctrl_reg3.lp;

	return ret;
}

/**
 * @brief  block_data_update: [set] Blockdataupdate.
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  uint8_t val: change the values of bdu in reg CTRL_REG5
 *
 */
int32_t lis3mdl_block_data_update_set(lis3mdl_ctx_t *ctx, uint8_t val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	ret = lis3mdl_read_reg(ctx, LIS3MDL_CTRL_REG5, &reg.byte, 1);
	if (ret == 0) {
		reg.ctrl_reg5.bdu = val;
		ret = lis3mdl_write_reg(ctx, LIS3MDL_CTRL_REG5, &reg.byte, 1);
	}
	return ret;
}

/**
 * @brief  block_data_update: [get] Blockdataupdate.
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  uint8_t: change the values of bdu in reg CTRL_REG5
 *
 */
int32_t lis3mdl_block_data_update_get(lis3mdl_ctx_t *ctx, uint8_t *val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	ret = lis3mdl_read_reg(ctx, LIS3MDL_CTRL_REG5, &reg.byte, 1);
	*val = (uint8_t)reg.ctrl_reg5.bdu;

	return ret;
}

/**
 * @brief  high_part_cycle: [set]  fast_read allows reading the
 *                                 high part of DATA OUT only in order
 *                                 to increase reading efficiency.
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  uint8_t val: change the values of fast_read in reg CTRL_REG5
 *
 */
int32_t lis3mdl_high_part_cycle_set(lis3mdl_ctx_t *ctx, uint8_t val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	ret = lis3mdl_read_reg(ctx, LIS3MDL_CTRL_REG5, &reg.byte, 1);
	if (ret == 0) {
		reg.ctrl_reg5.fast_read = val;
		ret = lis3mdl_write_reg(ctx, LIS3MDL_CTRL_REG5, &reg.byte, 1);
	}

	return ret;
}

/**
 * @brief  high_part_cycle: [get]  fast_read allows reading the
 *                                 high part of DATA OUT only in order
 *                                 to increase reading efficiency.
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  uint8_t: change the values of fast_read in reg CTRL_REG5
 *
 */
int32_t lis3mdl_high_part_cycle_get(lis3mdl_ctx_t *ctx, uint8_t *val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	ret = lis3mdl_read_reg(ctx, LIS3MDL_CTRL_REG5, &reg.byte, 1);
	*val = (uint8_t)reg.ctrl_reg5.fast_read;

	return ret;
}

/**
 * @brief  mag_data_ready: [get]  Magnetic set of data available.
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  uint8_t: change the values of zyxda in reg STATUS_REG
 *
 */
int32_t lis3mdl_mag_data_ready_get(lis3mdl_ctx_t *ctx, uint8_t *val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	ret = lis3mdl_read_reg(ctx, LIS3MDL_STATUS_REG, &reg.byte, 1);
	*val = (uint8_t)reg.status_reg.zyxda;

	return ret;
}
/**
 * @brief  mag_data_ovr: [get]  Magnetic set of data overrun.
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  uint8_t: change the values of zyxor in reg STATUS_REG
 *
 */
int32_t lis3mdl_mag_data_ovr_get(lis3mdl_ctx_t *ctx, uint8_t *val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	ret = lis3mdl_read_reg(ctx, LIS3MDL_STATUS_REG, &reg.byte, 1);
	*val = (uint8_t)reg.status_reg.zyxor;

	return ret;
}
/**
 * @brief  magnetic_raw: [get]  Magnetic output value.
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  uint8_t * : buffer that stores data read
 *
 */
int32_t lis3mdl_magnetic_raw_get(lis3mdl_ctx_t *ctx, uint8_t *buff)
{
	return lis3mdl_read_reg(ctx, LIS3MDL_OUT_X_L, (uint8_t *)buff, 6);
}
/**
 * @brief  temperature_raw: [get]  Temperature output value.
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  uint8_t * : buffer that stores data read
 *
 */
int32_t lis3mdl_temperature_raw_get(lis3mdl_ctx_t *ctx, uint8_t *buff)
{
	return lis3mdl_read_reg(ctx, LIS3MDL_TEMP_OUT_L, (uint8_t *)buff, 2);
}
/**
 * @}
 */

/**
 * @addtogroup  common
 * @brief   This section group common usefull functions
 * @{
 */

/**
 * @brief  device_id: [get] DeviceWhoamI.
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  uint8_t * : buffer that stores data read
 *
 */
int32_t lis3mdl_device_id_get(lis3mdl_ctx_t *ctx, uint8_t *buff)
{
	return lis3mdl_read_reg(ctx, LIS3MDL_WHO_AM_I, (uint8_t *)buff, 1);
}
/**
 * @brief  self_test: [set] self_test
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  uint8_t val: change the values of st in reg CTRL_REG1
 *
 */
int32_t lis3mdl_self_test_set(lis3mdl_ctx_t *ctx, uint8_t val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	ret = lis3mdl_read_reg(ctx, LIS3MDL_CTRL_REG1, &reg.byte, 1);
	if (ret == 0) {
		reg.ctrl_reg1.st = (uint8_t)val;
		ret = lis3mdl_write_reg(ctx, LIS3MDL_CTRL_REG1, &reg.byte, 1);
	}
	return ret;
}

/**
 * @brief  self_test: [get] self_test
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  uint8_t: change the values of st in reg CTRL_REG1
 *
 */
int32_t lis3mdl_self_test_get(lis3mdl_ctx_t *ctx, uint8_t *val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	ret = lis3mdl_read_reg(ctx, LIS3MDL_CTRL_REG1, &reg.byte, 1);
	*val = (uint8_t)reg.ctrl_reg1.st;

	return ret;
}

/**
 * @brief  reset: [set]  Software reset. Restore the default
 *                       values in user registers.
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  uint8_t val: change the values of soft_rst in reg CTRL_REG2
 *
 */
int32_t lis3mdl_reset_set(lis3mdl_ctx_t *ctx, uint8_t val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	ret = lis3mdl_read_reg(ctx, LIS3MDL_CTRL_REG2, &reg.byte, 1);
	if (ret == 0) {
		reg.ctrl_reg2.soft_rst = val;
		ret = lis3mdl_write_reg(ctx, LIS3MDL_CTRL_REG2, &reg.byte, 1);
	}
	return ret;
}

/**
 * @brief  reset: [get]  Software reset. Restore the default values
 *                       in user registers.
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  uint8_t: change the values of soft_rst in reg CTRL_REG2
 *
 */
int32_t lis3mdl_reset_get(lis3mdl_ctx_t *ctx, uint8_t *val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	ret = lis3mdl_read_reg(ctx, LIS3MDL_CTRL_REG2, &reg.byte, 1);
	*val = (uint8_t)reg.ctrl_reg2.soft_rst;

	return ret;
}

/**
 * @brief  boot: [set]  Reboot memory content. Reload the
 *                      calibration parameters.
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  uint8_t val: change the values of reboot in reg CTRL_REG2
 *
 */
int32_t lis3mdl_boot_set(lis3mdl_ctx_t *ctx, uint8_t val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	ret = lis3mdl_read_reg(ctx, LIS3MDL_CTRL_REG2, &reg.byte, 1);
	if (ret == 0) {
		reg.ctrl_reg2.reboot = val;
		ret = lis3mdl_write_reg(ctx, LIS3MDL_CTRL_REG2, &reg.byte, 1);
	}

	return ret;
}

/**
 * @brief  boot: [get]  Reboot memory content.
 *                      Reload the calibration parameters.
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  uint8_t: change the values of reboot in reg CTRL_REG2
 *
 */
int32_t lis3mdl_boot_get(lis3mdl_ctx_t *ctx, uint8_t *val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	ret = lis3mdl_read_reg(ctx, LIS3MDL_CTRL_REG2, &reg.byte, 1);
	*val = (uint8_t)reg.ctrl_reg2.reboot;

	return ret;
}

/**
 * @brief  data_format: [set]  Big/Little Endian data selection.
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  lis3mdl_ble_t: change the values of ble in reg CTRL_REG4
 *
 */
int32_t lis3mdl_data_format_set(lis3mdl_ctx_t *ctx, lis3mdl_ble_t val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	ret = lis3mdl_read_reg(ctx, LIS3MDL_CTRL_REG4, &reg.byte, 1);
	if (ret == 0) {
		reg.ctrl_reg4.ble = (uint8_t)val;
		ret = lis3mdl_write_reg(ctx, LIS3MDL_CTRL_REG4, &reg.byte, 1);
	}
	return ret;
}

/**
 * @brief  data_format: [get]  Big/Little Endian data selection.
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  lis3mdl_ble_t: Get the values of ble in reg CTRL_REG4
 *
 */
int32_t lis3mdl_data_format_get(lis3mdl_ctx_t *ctx, lis3mdl_ble_t *val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	ret = lis3mdl_read_reg(ctx, LIS3MDL_CTRL_REG4, &reg.byte, 1);
	switch (reg.ctrl_reg4.ble) {
	case LIS3MDL_LSB_AT_LOW_ADD:
		*val = LIS3MDL_LSB_AT_LOW_ADD;
		break;
	case LIS3MDL_MSB_AT_LOW_ADD:
		*val = LIS3MDL_MSB_AT_LOW_ADD;
		break;
	default:
		*val = LIS3MDL_DATA_FMT_ND;
		break;
	}
	return ret;
}

/**
 * @brief  status: [get] Statusregister
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  lis3mdl_status_reg_t: Registers STATUS_REG
 *
 */
int32_t lis3mdl_status_get(lis3mdl_ctx_t *ctx, lis3mdl_status_reg_t *val)
{
	return lis3mdl_read_reg(ctx, LIS3MDL_STATUS_REG, (uint8_t *)val, 1);
}
/**
 * @}
 */

/**
 * @addtogroup  interrupts
 * @brief   This section group all the functions that manage interrupts
 * @{
 */

/**
 * @brief  int_config: [set]  Interrupt configuration register
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  lis3mdl_int_cfg_t: Registers INT_CFG
 *
 */
int32_t lis3mdl_int_config_set(lis3mdl_ctx_t *ctx, lis3mdl_int_cfg_t *val)
{
	return lis3mdl_read_reg(ctx, LIS3MDL_INT_CFG, (uint8_t *)val, 1);
}

/**
 * @brief  int_config: [get]  Interrupt configuration register
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  lis3mdl_int_cfg_t: Registers INT_CFG
 *
 */
int32_t lis3mdl_int_config_get(lis3mdl_ctx_t *ctx, lis3mdl_int_cfg_t *val)
{
	return lis3mdl_read_reg(ctx, LIS3MDL_INT_CFG, (uint8_t *)val, 1);
}
/**
 * @brief  int_generation: [set]  Interrupt enable on INT pin.
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  uint8_t val: change the values of ien in reg INT_CFG
 *
 */
int32_t lis3mdl_int_generation_set(lis3mdl_ctx_t *ctx, uint8_t val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	ret = lis3mdl_read_reg(ctx, LIS3MDL_INT_CFG, &reg.byte, 1);
	if (ret == 0) {
		reg.int_cfg.ien = val;
		ret = lis3mdl_write_reg(ctx, LIS3MDL_INT_CFG, &reg.byte, 1);
	}

	return ret;
}

/**
 * @brief  int_generation: [get]  Interrupt enable on INT pin.
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  uint8_t: change the values of ien in reg INT_CFG
 *
 */
int32_t lis3mdl_int_generation_get(lis3mdl_ctx_t *ctx, uint8_t *val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	ret = lis3mdl_read_reg(ctx, LIS3MDL_INT_CFG, &reg.byte, 1);
	*val = (uint8_t)reg.int_cfg.ien;

	return ret;
}

/**
 * @brief   int_notification_mode: [set]  Interrupt request to the
 *                                        INT_SOURCE (25h) register
 *                                        mode (pulsed / latched)
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  lis3mdl_lir_t: change the values of lir in reg INT_CFG
 *
 */
int32_t lis3mdl_int_notification_mode_set(lis3mdl_ctx_t *ctx,
					  lis3mdl_lir_t val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	ret = lis3mdl_read_reg(ctx, LIS3MDL_INT_CFG, &reg.byte, 1);
	if (ret == 0) {
		reg.int_cfg.lir = (uint8_t)val;
		ret = lis3mdl_write_reg(ctx, LIS3MDL_INT_CFG, &reg.byte, 1);
	}
	return ret;
}

/**
 * @brief   int_notification_mode: [get]  Interrupt request to the
 *                                        INT_SOURCE (25h) register
 *                                        mode (pulsed / latched)
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  lis3mdl_lir_t: Get the values of lir in reg INT_CFG
 *
 */
int32_t lis3mdl_int_notification_mode_get(lis3mdl_ctx_t *ctx,
					  lis3mdl_lir_t *val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	ret = lis3mdl_read_reg(ctx, LIS3MDL_INT_CFG, &reg.byte, 1);
	switch (reg.int_cfg.lir) {
	case LIS3MDL_INT_PULSED:
		*val = LIS3MDL_INT_PULSED;
		break;
	case LIS3MDL_INT_LATCHED:
		*val = LIS3MDL_INT_LATCHED;
		break;
	default:
		*val = LIS3MDL_INT_MOODE_ND;
		break;
	}

	return ret;
}

/**
 * @brief  int_polarity: [set]  Interrupt active-high/low.
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  lis3mdl_iea_t: change the values of iea in reg INT_CFG
 *
 */
int32_t lis3mdl_int_polarity_set(lis3mdl_ctx_t *ctx, lis3mdl_iea_t val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	ret = lis3mdl_read_reg(ctx, LIS3MDL_INT_CFG, &reg.byte, 1);
	if (ret == 0) {
		reg.int_cfg.iea = (uint8_t)val;
		ret = lis3mdl_write_reg(ctx, LIS3MDL_INT_CFG, &reg.byte, 1);
	}

	return ret;
}

/**
 * @brief  int_polarity: [get]  Interrupt active-high/low.
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  lis3mdl_iea_t: Get the values of iea in reg INT_CFG
 *
 */
int32_t lis3mdl_int_polarity_get(lis3mdl_ctx_t *ctx, lis3mdl_iea_t *val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	ret = lis3mdl_read_reg(ctx, LIS3MDL_INT_CFG, &reg.byte, 1);
	switch (reg.int_cfg.iea) {
	case LIS3MDL_ACTIVE_HIGH:
		*val = LIS3MDL_ACTIVE_HIGH;
		break;
	case LIS3MDL_ACTIVE_LOW:
		*val = LIS3MDL_ACTIVE_LOW;
		break;
	default:
		*val = LIS3MDL_POLARITY_ND;
		break;
	}
	return ret;
}

/**
 * @brief  int_on_z_ax: [set]  Enable interrupt generation on Z-axis.
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  uint8_t val: change the values of zien in reg INT_CFG
 *
 */
int32_t lis3mdl_int_on_z_ax_set(lis3mdl_ctx_t *ctx, uint8_t val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	ret = lis3mdl_read_reg(ctx, LIS3MDL_INT_CFG, &reg.byte, 1);
	if (ret == 0) {
		reg.int_cfg.zien = val;
		ret = lis3mdl_write_reg(ctx, LIS3MDL_INT_CFG, &reg.byte, 1);
	}
	return ret;
}

/**
 * @brief  int_on_z_ax: [get]  Enable interrupt generation on Z-axis.
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  uint8_t: change the values of zien in reg INT_CFG
 *
 */
int32_t lis3mdl_int_on_z_ax_get(lis3mdl_ctx_t *ctx, uint8_t *val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	ret = lis3mdl_read_reg(ctx, LIS3MDL_INT_CFG, &reg.byte, 1);
	*val = (uint8_t)reg.int_cfg.zien;

	return ret;
}

/**
 * @brief  int_on_y_ax: [set]  Enable interrupt generation on Y-axis.
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  uint8_t val: change the values of yien in reg INT_CFG
 *
 */
int32_t lis3mdl_int_on_y_ax_set(lis3mdl_ctx_t *ctx, uint8_t val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	ret = lis3mdl_read_reg(ctx, LIS3MDL_INT_CFG, &reg.byte, 1);
	if (ret == 0) {
		reg.int_cfg.yien = val;
		ret = lis3mdl_write_reg(ctx, LIS3MDL_INT_CFG, &reg.byte, 1);
	}
	return ret;
}

/**
 * @brief  int_on_y_ax: [get]  Enable interrupt generation on Y-axis.
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  uint8_t: change the values of yien in reg INT_CFG
 *
 */
int32_t lis3mdl_int_on_y_ax_get(lis3mdl_ctx_t *ctx, uint8_t *val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	ret = lis3mdl_read_reg(ctx, LIS3MDL_INT_CFG, &reg.byte, 1);
	*val = (uint8_t)reg.int_cfg.yien;

	return ret;
}

/**
 * @brief  int_on_x_ax: [set]  Enable interrupt generation on X-axis.
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  uint8_t val: change the values of xien in reg INT_CFG
 *
 */
int32_t lis3mdl_int_on_x_ax_set(lis3mdl_ctx_t *ctx, uint8_t val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	ret = lis3mdl_read_reg(ctx, LIS3MDL_INT_CFG, &reg.byte, 1);
	if (ret == 0) {
		reg.int_cfg.xien = val;
		ret = lis3mdl_write_reg(ctx, LIS3MDL_INT_CFG, &reg.byte, 1);
	}

	return ret;
}

/**
 * @brief  int_on_x_ax: [get]  Enable interrupt generation on X-axis.
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  uint8_t: change the values of xien in reg INT_CFG
 *
 */
int32_t lis3mdl_int_on_x_ax_get(lis3mdl_ctx_t *ctx, uint8_t *val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	ret = lis3mdl_read_reg(ctx, LIS3MDL_INT_CFG, &reg.byte, 1);
	*val = (uint8_t)reg.int_cfg.xien;

	return ret;
}

/**
 * @brief  int_source: [get]  Interrupt source register
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  lis3mdl_int_src_t: Register INT_SRC
 *
 */
int32_t lis3mdl_int_source_get(lis3mdl_ctx_t *ctx, lis3mdl_int_src_t *val)
{
	return lis3mdl_read_reg(ctx, LIS3MDL_INT_SRC, (uint8_t *)val, 1);
}
/**
 * @brief   interrupt_event_flag: [get]   Interrupt active flag.
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  uint8_t: change the values of int in reg INT_SRC
 *
 */
int32_t lis3mdl_interrupt_event_flag_get(lis3mdl_ctx_t *ctx, uint8_t *val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	ret = lis3mdl_read_reg(ctx, LIS3MDL_INT_SRC, &reg.byte, 1);
	*val = (uint8_t)reg.int_src.int_;

	return ret;
}
/**
 * @brief   int_mag_over_range_flag: [get]  Internal measurement range
 *                                          overflow on magnetic value.
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  uint8_t: change the values of mroi in reg INT_SRC
 *
 */
int32_t lis3mdl_int_mag_over_range_flag_get(lis3mdl_ctx_t *ctx, uint8_t *val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	ret = lis3mdl_read_reg(ctx, LIS3MDL_INT_SRC, &reg.byte, 1);
	*val = (uint8_t)reg.int_src.mroi;

	return ret;
}
/**
 * @brief  int_neg_z_flag: [get]  Value on Z-axis exceeds the threshold
 *                                on the negative side.
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  uint8_t: change the values of nth_z in reg INT_SRC
 *
 */
int32_t lis3mdl_int_neg_z_flag_get(lis3mdl_ctx_t *ctx, uint8_t *val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	ret = lis3mdl_read_reg(ctx, LIS3MDL_INT_SRC, &reg.byte, 1);
	*val = (uint8_t)reg.int_src.nth_z;

	return ret;
}
/**
 * @brief  int_neg_y_flag: [get]  Value on Y-axis exceeds the
 *                                threshold on the negative side.
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  uint8_t: change the values of nth_y in reg INT_SRC
 *
 */
int32_t lis3mdl_int_neg_y_flag_get(lis3mdl_ctx_t *ctx, uint8_t *val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	ret = lis3mdl_read_reg(ctx, LIS3MDL_INT_SRC, &reg.byte, 1);
	*val = (uint8_t)reg.int_src.nth_y;

	return ret;
}
/**
 * @brief  int_neg_x_flag: [get]  Value on X-axis exceeds the
 *                                threshold on the negative side.
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  uint8_t: change the values of nth_x in reg INT_SRC
 *
 */
int32_t lis3mdl_int_neg_x_flag_get(lis3mdl_ctx_t *ctx, uint8_t *val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	ret = lis3mdl_read_reg(ctx, LIS3MDL_INT_SRC, &reg.byte, 1);
	*val = (uint8_t)reg.int_src.nth_x;

	return ret;
}
/**
 * @brief  int_pos_z_flag: [get]  Value on Z-axis exceeds the
 *                                threshold on the positive side.
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  uint8_t: change the values of pth_z in reg INT_SRC
 *
 */
int32_t lis3mdl_int_pos_z_flag_get(lis3mdl_ctx_t *ctx, uint8_t *val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	ret = lis3mdl_read_reg(ctx, LIS3MDL_INT_SRC, &reg.byte, 1);
	*val = (uint8_t)reg.int_src.pth_z;

	return ret;
}
/**
 * @brief  int_pos_y_flag: [get]  Value on Y-axis exceeds the
 *                                threshold on the positive side.
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  uint8_t: change the values of pth_y in reg INT_SRC
 *
 */
int32_t lis3mdl_int_pos_y_flag_get(lis3mdl_ctx_t *ctx, uint8_t *val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	ret = lis3mdl_read_reg(ctx, LIS3MDL_INT_SRC, &reg.byte, 1);
	*val = (uint8_t)reg.int_src.pth_y;

	return ret;
}
/**
 * @brief  int_pos_x_flag: [get]  Value on X-axis exceeds the
 *                                threshold on the positive side.
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  uint8_t: change the values of pth_x in reg INT_SRC
 *
 */
int32_t lis3mdl_int_pos_x_flag_get(lis3mdl_ctx_t *ctx, uint8_t *val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	ret = lis3mdl_read_reg(ctx, LIS3MDL_INT_SRC, &reg.byte, 1);
	*val = (uint8_t)reg.int_src.pth_x;

	return ret;
}
/**
 * @brief  int_threshold: [set]  User-defined threshold value
 *                              for pressure interrupt event
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  uint8_t * : buffer that contains data to write
 *
 */
int32_t lis3mdl_int_threshold_set(lis3mdl_ctx_t *ctx, uint8_t *buff)
{
	return lis3mdl_read_reg(ctx, LIS3MDL_INT_THS_L, buff, 2);
}

/**
 * @brief  int_threshold: [get]  User-defined threshold value for
 *                              pressure interrupt event
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  uint8_t * : buffer that stores data read
 *
 */
int32_t lis3mdl_int_threshold_get(lis3mdl_ctx_t *ctx, uint8_t *buff)
{
	return lis3mdl_read_reg(ctx, LIS3MDL_INT_THS_L, buff, 2);
}
/**
 * @}
 */

/**
 * @addtogroup  serial_interface
 * @brief   This section group all the functions concerning
 *          serial interface management
 * @{
 */

/**
 * @brief  spi_mode: [set]  SPI Serial Interface Mode selection.
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  lis3mdl_sim_t: change the values of sim in reg CTRL_REG3
 *
 */
int32_t lis3mdl_spi_mode_set(lis3mdl_ctx_t *ctx, lis3mdl_sim_t val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	ret = lis3mdl_read_reg(ctx, LIS3MDL_CTRL_REG3, &reg.byte, 1);
	if (ret == 0) {
		reg.ctrl_reg3.sim = (uint8_t)val;
		ret = lis3mdl_write_reg(ctx, LIS3MDL_CTRL_REG3, &reg.byte, 1);
	}
	return ret;
}

/**
 * @brief  spi_mode: [get]  SPI Serial Interface Mode selection.
 *
 * @param  lis3mdl_ctx_t *ctx: read / write interface definitions
 * @param  lis3mdl_sim_t: Get the values of sim in reg CTRL_REG3
 *
 */
int32_t lis3mdl_spi_mode_get(lis3mdl_ctx_t *ctx, lis3mdl_sim_t *val)
{
	lis3mdl_reg_t reg;
	int32_t ret;

	ret = lis3mdl_read_reg(ctx, LIS3MDL_CTRL_REG3, &reg.byte, 1);
	switch (reg.ctrl_reg3.sim) {
	case LIS3MDL_SPI_4_WIRE:
		*val = LIS3MDL_SPI_4_WIRE;
		break;
	case LIS3MDL_SPI_3_WIRE:
		*val = LIS3MDL_SPI_3_WIRE;
		break;
	default:
		*val = LIS3MDL_SPI_MODE_ND;
		break;
	}
	return ret;
}

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
