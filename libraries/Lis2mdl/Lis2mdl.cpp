/*
 * Copyright (c) 2023 EdgeImpulse Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an "AS
 * IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language
 * governing permissions and limitations under the License.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include "Lis2mdl.h"
#include "../I2c/I2c.h"


static int16_t data_raw_magnetic[3];
static int16_t lis2_data_raw_temperature;
static float magnetic_mG[3];
static float lis2_temperature_degC;
static uint8_t lis2_whoamI, lis2_rst;
static lis2mdl_mode_t lis2mdl_mode = {0};

/**
  * @defgroup    LIS2MDL
  * @brief       This file provides a set of functions needed to drive the
  *              lis2mdl enhanced inertial module.
  * @{
  *
  */

/**
  * @defgroup    LIS2MDL_Interfaces_Functions
  * @brief       This section provide a set of functions used to read and
  *              write a generic register of the device.
  *              MANDATORY: return 0 -> no Error.
  * @{
  *
  */

/**
  * @brief  Read generic device register
  *
  * @param  ctx   read / write interface definitions(ptr)
  * @param  reg   register to read
  * @param  data  pointer to buffer that store the data read(ptr)
  * @param  len   number of consecutive register to read
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_read_reg(stmdev_ctx_t* ctx, uint8_t reg, uint8_t* data, uint16_t len) {
    return (int32_t) i2c_read_regs(LIS2MDL_I2C_ADD, reg, data, len);
}

/**
  * @brief  Write generic device register
  *
  * @param  ctx   read / write interface definitions(ptr)
  * @param  reg   register to write
  * @param  data  pointer to data to write in register reg(ptr)
  * @param  len   number of consecutive register to write
  * @retval       interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_write_reg(stmdev_ctx_t* ctx, uint8_t reg, uint8_t* data, uint16_t len) {
    return (int32_t) i2c_write_regs(LIS2MDL_I2C_ADD, reg, data, len);
}

/**
  * @}
  *
  */

  /**
  * @defgroup    LIS2MDL_Sensitivity
  * @brief       These functions convert raw-data into engineering units.
  * @{
  *
  */
float_t lis2mdl_from_lsb_to_mgauss(int16_t lsb) {
    return ((float_t)lsb * 1.5f);
}

float_t lis2mdl_from_lsb_to_celsius(int16_t lsb) {
    return (((float_t)lsb / 8.0f) + 25.0f);
}

/**
  * @}
  *
  */

/**
  * @defgroup    LIS2MDL_data_generation
  * @brief       This section group all the functions concerning
  *              data generation
  * @{
  *
  */

/**
  * @brief  These registers comprise a 3 group of 16-bit number and represent
  *         hard-iron offset in order to compensate environmental effects.
  *         Data format is the same of output data raw: two’s complement
  *         with 1LSb = 1.5mG. These values act on the magnetic output data
  *         value in order to delete the environmental offset.[set]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  buff  buffer that contains data to write
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_mag_user_offset_set(stmdev_ctx_t *ctx, uint8_t *buff) {
    int32_t ret;
    ret = lis2mdl_write_reg(ctx, LIS2MDL_OFFSET_X_REG_L, buff, 6);
    return ret;
}

/**
  * @brief  These registers comprise a 3 group of 16-bit number and represent
  *         hard-iron offset in order to compensate environmental effects.
  *         Data format is the same of output data raw: two’s complement
  *         with 1LSb = 1.5mG. These values act on the magnetic output data
  *         value in order to delete the environmental offset.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  buff  that stores data read
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_mag_user_offset_get(stmdev_ctx_t *ctx, uint8_t *buff) {
    int32_t ret;
    ret = lis2mdl_read_reg(ctx, LIS2MDL_OFFSET_X_REG_L, buff, 6);
    return ret;
}

/**
  * @brief  Operating mode selection.[set]
  *
  * @param  ctx    read / write interface definitions.(ptr)
  * @param  val    change the values of md in reg CFG_REG_A
  * @retval        interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_operating_mode_set(stmdev_ctx_t *ctx, lis2mdl_md_t val) {
    lis2mdl_cfg_reg_a_t reg;
    int32_t ret;

    ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_A, (uint8_t*)&reg, 1);

    if(ret == 0){
        reg.md = (uint8_t)val;
        ret = lis2mdl_write_reg(ctx, LIS2MDL_CFG_REG_A, (uint8_t*)&reg, 1);
    }

    return ret;
}

/**
  * @brief  Operating mode selection.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   Get the values of md in reg CFG_REG_A.(ptr)
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_operating_mode_get(stmdev_ctx_t *ctx, lis2mdl_md_t *val) {
    lis2mdl_cfg_reg_a_t reg;
    int32_t ret;

    ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_A, (uint8_t*)&reg, 1);
    switch (reg.md){
        case LIS2MDL_POWER_DOWN:
          *val = LIS2MDL_POWER_DOWN;
          break;
        case LIS2MDL_CONTINUOUS_MODE:
          *val = LIS2MDL_CONTINUOUS_MODE;
          break;
        case LIS2MDL_SINGLE_TRIGGER:
          *val = LIS2MDL_SINGLE_TRIGGER;
          break;
        default:
          *val = LIS2MDL_POWER_DOWN;
          break;
    }

    return ret;
}

/**
  * @brief  Output data rate selection.[set]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of odr in reg CFG_REG_A
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_data_rate_set(stmdev_ctx_t *ctx, lis2mdl_odr_t val) {
    lis2mdl_cfg_reg_a_t reg;
    int32_t ret;

    ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_A, (uint8_t*)&reg, 1);

    if(ret == 0){
        reg.odr = (uint8_t)val;
        ret = lis2mdl_write_reg(ctx, LIS2MDL_CFG_REG_A, (uint8_t*)&reg, 1);
    }

    return ret;
}

/**
  * @brief  Output data rate selection.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   Get the values of odr in reg CFG_REG_A.(ptr)
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_data_rate_get(stmdev_ctx_t *ctx, lis2mdl_odr_t *val) {
    lis2mdl_cfg_reg_a_t reg;
    int32_t ret;

    ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_A, (uint8_t*)&reg, 1);
    switch (reg.odr){
        case LIS2MDL_ODR_10Hz:
          *val = LIS2MDL_ODR_10Hz;
          break;
        case LIS2MDL_ODR_20Hz:
          *val = LIS2MDL_ODR_20Hz;
          break;
        case LIS2MDL_ODR_50Hz:
          *val = LIS2MDL_ODR_50Hz;
          break;
        case LIS2MDL_ODR_100Hz:
          *val = LIS2MDL_ODR_100Hz;
          break;
        default:
          *val = LIS2MDL_ODR_10Hz;
          break;
    }
    return ret;
}

/**
  * @brief  Enables high-resolution/low-power mode.[set]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of lp in reg CFG_REG_A
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_power_mode_set(stmdev_ctx_t *ctx, lis2mdl_lp_t val) {
    lis2mdl_cfg_reg_a_t reg;
    int32_t ret;

    ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_A, (uint8_t*)&reg, 1);

    if(ret == 0){
        reg.lp = (uint8_t)val;
        ret = lis2mdl_write_reg(ctx, LIS2MDL_CFG_REG_A, (uint8_t*)&reg, 1);
    }

    return ret;
}

/**
  * @brief  Enables high-resolution/low-power mode.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   Get the values of lp in reg CFG_REG_A.(ptr)
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_power_mode_get(stmdev_ctx_t *ctx, lis2mdl_lp_t *val) {
    lis2mdl_cfg_reg_a_t reg;
    int32_t ret;

    ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_A, (uint8_t*)&reg, 1);
    switch (reg.lp){
        case LIS2MDL_HIGH_RESOLUTION:
          *val = LIS2MDL_HIGH_RESOLUTION;
          break;
        case LIS2MDL_LOW_POWER:
          *val = LIS2MDL_LOW_POWER;
          break;
        default:
          *val = LIS2MDL_HIGH_RESOLUTION;
          break;
    }
    return ret;
}

/**
  * @brief  Enables the magnetometer temperature compensation.[set]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of comp_temp_en in reg CFG_REG_A
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_offset_temp_comp_set(stmdev_ctx_t *ctx, uint8_t val) {
    lis2mdl_cfg_reg_a_t reg;
    int32_t ret;

    ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_A, (uint8_t*)&reg, 1);

    if(ret == 0){
        reg.comp_temp_en = val;
        ret = lis2mdl_write_reg(ctx, LIS2MDL_CFG_REG_A, (uint8_t*)&reg, 1);
    }

    return ret;
}

/**
  * @brief  Enables the magnetometer temperature compensation.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of comp_temp_en in reg CFG_REG_A.(ptr)
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_offset_temp_comp_get(stmdev_ctx_t *ctx, uint8_t *val) {
    lis2mdl_cfg_reg_a_t reg;
    int32_t ret;

    ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_A, (uint8_t*)&reg, 1);
    *val = reg.comp_temp_en;

    return ret;
}

/**
  * @brief  Low-pass bandwidth selection.[set]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of lpf in reg CFG_REG_B
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_low_pass_bandwidth_set(stmdev_ctx_t *ctx, lis2mdl_lpf_t val) {
    lis2mdl_cfg_reg_b_t reg;
    int32_t ret;

    ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_B, (uint8_t*)&reg, 1);

    if(ret == 0){
        reg.lpf = (uint8_t)val;
        ret = lis2mdl_write_reg(ctx, LIS2MDL_CFG_REG_B, (uint8_t*)&reg, 1);
    }

    return ret;
}

/**
  * @brief  Low-pass bandwidth selection.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   Get the values of lpf in reg CFG_REG_B.(ptr)
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_low_pass_bandwidth_get(stmdev_ctx_t *ctx, lis2mdl_lpf_t *val) {
    lis2mdl_cfg_reg_b_t reg;
    int32_t ret;

    ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_B, (uint8_t*)&reg, 1);
    switch (reg.lpf){
        case LIS2MDL_ODR_DIV_2:
          *val = LIS2MDL_ODR_DIV_2;
          break;
        case LIS2MDL_ODR_DIV_4:
          *val = LIS2MDL_ODR_DIV_4;
          break;
        default:
          *val = LIS2MDL_ODR_DIV_2;
          break;
    }
    return ret;
}

/**
  * @brief  Reset mode.[set]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of set_rst in reg CFG_REG_B
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_set_rst_mode_set(stmdev_ctx_t *ctx, lis2mdl_set_rst_t val) {
    lis2mdl_cfg_reg_b_t reg;
    int32_t ret;

    ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_B, (uint8_t*)&reg, 1);

    if(ret == 0){
        reg.set_rst = (uint8_t)val;
        ret = lis2mdl_write_reg(ctx, LIS2MDL_CFG_REG_B, (uint8_t*)&reg, 1);
    }

    return ret;
}

/**
  * @brief  Reset mode.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   Get the values of set_rst in reg CFG_REG_B.(ptr)
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_set_rst_mode_get(stmdev_ctx_t *ctx, lis2mdl_set_rst_t *val) {
    lis2mdl_cfg_reg_b_t reg;
    int32_t ret;

    ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_B, (uint8_t*)&reg, 1);
    switch (reg.set_rst){
        case LIS2MDL_SET_SENS_ODR_DIV_63:
          *val = LIS2MDL_SET_SENS_ODR_DIV_63;
          break;
        case LIS2MDL_SENS_OFF_CANC_EVERY_ODR:
          *val = LIS2MDL_SENS_OFF_CANC_EVERY_ODR;
          break;
        case LIS2MDL_SET_SENS_ONLY_AT_POWER_ON:
          *val = LIS2MDL_SET_SENS_ONLY_AT_POWER_ON;
          break;
        default:
          *val = LIS2MDL_SET_SENS_ODR_DIV_63;
          break;
    }
    return ret;
}

/**
  * @brief  Enables offset cancellation in single measurement mode.
  *         The OFF_CANC bit must be set to 1 when enabling offset
  *         cancellation in single measurement mode this means a
  *         call function: set_rst_mode(SENS_OFF_CANC_EVERY_ODR)
  *         is need.[set]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of off_canc_one_shot in reg CFG_REG_B
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_set_rst_sensor_single_set(stmdev_ctx_t *ctx, uint8_t val) {
    lis2mdl_cfg_reg_b_t reg;
    int32_t ret;

    ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_B, (uint8_t*)&reg, 1);

    if(ret == 0){
        reg.off_canc_one_shot = val;
        ret = lis2mdl_write_reg(ctx, LIS2MDL_CFG_REG_B, (uint8_t*)&reg, 1);
    }

    return ret;
}

/**
  * @brief  Enables offset cancellation in single measurement mode.
  *         The OFF_CANC bit must be set to 1 when enabling offset
  *         cancellation in single measurement mode this means a
  *         call function: set_rst_mode(SENS_OFF_CANC_EVERY_ODR)
  *         is need.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of off_canc_one_shot in reg CFG_REG_B.(ptr)
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_set_rst_sensor_single_get(stmdev_ctx_t *ctx, uint8_t *val) {
    lis2mdl_cfg_reg_b_t reg;
    int32_t ret;

    ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_B, (uint8_t*)&reg, 1);
    *val = reg.off_canc_one_shot;

    return ret;
}

/**
  * @brief  Blockdataupdate.[set]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of bdu in reg CFG_REG_C
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_block_data_update_set(stmdev_ctx_t *ctx, uint8_t val) {
    lis2mdl_cfg_reg_c_t reg;
    int32_t ret;

    ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_C, (uint8_t*)&reg, 1);

    if(ret == 0){
        reg.bdu = val;
        ret = lis2mdl_write_reg(ctx, LIS2MDL_CFG_REG_C, (uint8_t*)&reg, 1);
    }

    return ret;
}

/**
  * @brief  Blockdataupdate.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of bdu in reg CFG_REG_C.(ptr)
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_block_data_update_get(stmdev_ctx_t *ctx, uint8_t *val) {
    lis2mdl_cfg_reg_c_t reg;
    int32_t ret;

    ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_C, (uint8_t*)&reg, 1);
    *val = reg.bdu;

    return ret;
}

/**
  * @brief  Magnetic set of data available.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of zyxda in reg STATUS_REG.(ptr)
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_mag_data_ready_get(stmdev_ctx_t *ctx, uint8_t *val) {
    lis2mdl_status_reg_t reg;
    int32_t ret;

    ret = lis2mdl_read_reg(ctx, LIS2MDL_STATUS_REG, (uint8_t*)&reg, 1);
    *val = reg.zyxda;

    return ret;
}

/**
  * @brief  Magnetic set of data overrun.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of zyxor in reg STATUS_REG.(ptr)
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_mag_data_ovr_get(stmdev_ctx_t *ctx, uint8_t *val) {
    lis2mdl_status_reg_t reg;
    int32_t ret;

    ret = lis2mdl_read_reg(ctx, LIS2MDL_STATUS_REG, (uint8_t*)&reg, 1);
    *val = reg.zyxor;

    return ret;
}

/**
  * @brief  Magnetic output value.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  buff  that stores data read
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_magnetic_raw_get(stmdev_ctx_t *ctx, uint8_t *buff) {
    int32_t ret;
    ret = lis2mdl_read_reg(ctx, LIS2MDL_OUTX_L_REG, buff, 6);
    return ret;
}

/**
  * @brief  Temperature output value.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  buff  that stores data read
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_temperature_raw_get(stmdev_ctx_t *ctx, uint8_t *buff) {
    int32_t ret;
    ret = lis2mdl_read_reg(ctx, LIS2MDL_TEMP_OUT_L_REG, buff, 2);
    return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup    LIS2MDL_common
  * @brief       This section group common usefull functions
  * @{
  *
  */

/**
  * @brief  DeviceWhoamI.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  buff  that stores data read
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_device_id_get(stmdev_ctx_t *ctx, uint8_t *buff) {
    int32_t ret;
    ret = lis2mdl_read_reg(ctx, LIS2MDL_WHO_AM_I, buff, 1);
    return ret;
}

/**
  * @brief  Software reset. Restore the default values in user registers.[set]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of soft_rst in reg CFG_REG_A
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_reset_set(stmdev_ctx_t *ctx, uint8_t val) {
    lis2mdl_cfg_reg_a_t reg;
    int32_t ret;

    ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_A, (uint8_t*)&reg, 1);

    if(ret == 0){
        reg.soft_rst = val;
        ret = lis2mdl_write_reg(ctx, LIS2MDL_CFG_REG_A, (uint8_t*)&reg, 1);
    }

    return ret;
}

/**
  * @brief  Software reset. Restore the default values in user registers.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of soft_rst in reg CFG_REG_A.(ptr)
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_reset_get(stmdev_ctx_t *ctx, uint8_t *val) {
    lis2mdl_cfg_reg_a_t reg;
    int32_t ret;

    ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_A, (uint8_t*)&reg, 1);
    *val = reg.soft_rst;

    return ret;
}

/**
  * @brief  Reboot memory content. Reload the calibration parameters.[set]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of reboot in reg CFG_REG_A
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_boot_set(stmdev_ctx_t *ctx, uint8_t val) {
    lis2mdl_cfg_reg_a_t reg;
    int32_t ret;

    ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_A, (uint8_t*)&reg, 1);

    if(ret == 0){
        reg.reboot = val;
        ret = lis2mdl_write_reg(ctx, LIS2MDL_CFG_REG_A, (uint8_t*)&reg, 1);
    }

    return ret;
}

/**
  * @brief  Reboot memory content. Reload the calibration parameters.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of reboot in reg CFG_REG_A.(ptr)
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_boot_get(stmdev_ctx_t *ctx, uint8_t *val) {
    lis2mdl_cfg_reg_a_t reg;
    int32_t ret;

    ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_A, (uint8_t*)&reg, 1);
    *val = reg.reboot;

    return ret;
}

/**
  * @brief  Selftest.[set]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of self_test in reg CFG_REG_C
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_self_test_set(stmdev_ctx_t *ctx, uint8_t val) {
    lis2mdl_cfg_reg_c_t reg;
    int32_t ret;

    ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_C, (uint8_t*)&reg, 1);

    if(ret == 0){
        reg.self_test = val;
        ret = lis2mdl_write_reg(ctx, LIS2MDL_CFG_REG_C, (uint8_t*)&reg, 1);
    }

    return ret;
}

/**
  * @brief  Selftest.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of self_test in reg CFG_REG_C.(ptr)
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_self_test_get(stmdev_ctx_t *ctx, uint8_t *val) {
    lis2mdl_cfg_reg_c_t reg;
    int32_t ret;

    ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_C, (uint8_t*)&reg, 1);
    *val = reg.self_test;

    return ret;
}

/**
  * @brief  Big/Little Endian data selection.[set]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of ble in reg CFG_REG_C
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_data_format_set(stmdev_ctx_t *ctx, lis2mdl_ble_t val) {
    lis2mdl_cfg_reg_c_t reg;
    int32_t ret;

    ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_C, (uint8_t*)&reg, 1);

    if(ret == 0){
        reg.ble = (uint8_t)val;
        ret = lis2mdl_write_reg(ctx, LIS2MDL_CFG_REG_C, (uint8_t*)&reg, 1);
    }

    return ret;
}

/**
  * @brief  Big/Little Endian data selection.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   Get the values of ble in reg CFG_REG_C.(ptr)
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_data_format_get(stmdev_ctx_t *ctx, lis2mdl_ble_t *val) {
    lis2mdl_cfg_reg_c_t reg;
    int32_t ret;

    ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_C, (uint8_t*)&reg, 1);
    switch (reg.ble){
        case LIS2MDL_LSB_AT_LOW_ADD:
          *val = LIS2MDL_LSB_AT_LOW_ADD;
          break;
        case LIS2MDL_MSB_AT_LOW_ADD:
          *val = LIS2MDL_MSB_AT_LOW_ADD;
          break;
        default:
          *val = LIS2MDL_LSB_AT_LOW_ADD;
          break;
    }
    return ret;
}

/**
  * @brief  Info about device status.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   registers STATUS_REG.(ptr)
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_status_get(stmdev_ctx_t *ctx, lis2mdl_status_reg_t *val) {
    int32_t ret;
    ret = lis2mdl_read_reg(ctx, LIS2MDL_STATUS_REG, (uint8_t*) val, 1);
    return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup    LIS2MDL_interrupts
  * @brief       This section group all the functions that manage interrupts
  * @{
  *
  */

/**
  * @brief  The interrupt block recognition checks data after/before the
  *         hard-iron correction to discover the interrupt.[set]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of int_on_dataoff in reg CFG_REG_B
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_offset_int_conf_set(stmdev_ctx_t *ctx, lis2mdl_int_on_dataoff_t val) {
    lis2mdl_cfg_reg_b_t reg;
    int32_t ret;

    ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_B, (uint8_t*)&reg, 1);

    if(ret == 0){
        reg.int_on_dataoff = (uint8_t)val;
        ret = lis2mdl_write_reg(ctx, LIS2MDL_CFG_REG_B, (uint8_t*)&reg, 1);
    }

    return ret;
}

/**
  * @brief  The interrupt block recognition checks data after/before the
  *         hard-iron correction to discover the interrupt.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   Get the values of int_on_dataoff in reg CFG_REG_B.(ptr)
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_offset_int_conf_get(stmdev_ctx_t *ctx, lis2mdl_int_on_dataoff_t *val) {
    lis2mdl_cfg_reg_b_t reg;
    int32_t ret;

    ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_B, (uint8_t*)&reg, 1);
    switch (reg.int_on_dataoff){
        case LIS2MDL_CHECK_BEFORE:
          *val = LIS2MDL_CHECK_BEFORE;
          break;
        case LIS2MDL_CHECK_AFTER:
          *val = LIS2MDL_CHECK_AFTER;
          break;
        default:
          *val = LIS2MDL_CHECK_BEFORE;
          break;
    }
    return ret;
}

/**
  * @brief  Data-ready signal on INT_DRDY pin.[set]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of drdy_on_pin in reg CFG_REG_C
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_drdy_on_pin_set(stmdev_ctx_t *ctx, uint8_t val) {
    lis2mdl_cfg_reg_c_t reg;
    int32_t ret;

    ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_C, (uint8_t*)&reg, 1);

    if(ret == 0){
        reg.drdy_on_pin = val;
        ret = lis2mdl_write_reg(ctx, LIS2MDL_CFG_REG_C, (uint8_t*)&reg, 1);
    }

    return ret;
}

/**
  * @brief  Data-ready signal on INT_DRDY pin.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of drdy_on_pin in reg CFG_REG_C.(ptr)
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_drdy_on_pin_get(stmdev_ctx_t *ctx, uint8_t *val) {
    lis2mdl_cfg_reg_c_t reg;
    int32_t ret;

    ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_C, (uint8_t*)&reg, 1);
    *val = reg.drdy_on_pin;

    return ret;
}

/**
  * @brief  Interrupt signal on INT_DRDY pin.[set]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of int_on_pin in reg CFG_REG_C
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_int_on_pin_set(stmdev_ctx_t *ctx, uint8_t val) {
    lis2mdl_cfg_reg_c_t reg;
    int32_t ret;

    ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_C, (uint8_t*)&reg, 1);

    if(ret == 0){
        reg.int_on_pin = val;
        ret = lis2mdl_write_reg(ctx, LIS2MDL_CFG_REG_C, (uint8_t*)&reg, 1);
    }

    return ret;
}

/**
  * @brief  Interrupt signal on INT_DRDY pin.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of int_on_pin in reg CFG_REG_C.(ptr)
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_int_on_pin_get(stmdev_ctx_t *ctx, uint8_t *val) {
    lis2mdl_cfg_reg_c_t reg;
    int32_t ret;

    ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_C, (uint8_t*)&reg, 1);
    *val = reg.int_on_pin;

    return ret;
}

/**
  * @brief  Interrupt generator configuration register.[set]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   registers INT_CRTL_REG.(ptr)
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_int_gen_conf_set(stmdev_ctx_t *ctx, lis2mdl_int_crtl_reg_t *val) {
    int32_t ret;
    ret = lis2mdl_write_reg(ctx, LIS2MDL_INT_CRTL_REG, (uint8_t*) val, 1);
    return ret;
}

/**
  * @brief  Interrupt generator configuration register.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   registers INT_CRTL_REG.(ptr)
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_int_gen_conf_get(stmdev_ctx_t *ctx, lis2mdl_int_crtl_reg_t *val) {
    int32_t ret;
    ret = lis2mdl_read_reg(ctx, LIS2MDL_INT_CRTL_REG, (uint8_t*) val, 1);
    return ret;
}

/**
  * @brief  Interrupt generator source register.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   registers INT_SOURCE_REG.(ptr)
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_int_gen_source_get(stmdev_ctx_t *ctx, lis2mdl_int_source_reg_t *val) {
    int32_t ret;
    ret = lis2mdl_read_reg(ctx, LIS2MDL_INT_SOURCE_REG, (uint8_t*) val, 1);
    return ret;
}

/**
  * @brief  User-defined threshold value for xl interrupt event on generator.
  *         Data format is the same of output data raw: two’s complement with
  *         1LSb = 1.5mG.[set]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  buff  that contains data to write
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_int_gen_treshold_set(stmdev_ctx_t *ctx, uint8_t *buff) {
    int32_t ret;
    ret = lis2mdl_write_reg(ctx, LIS2MDL_INT_THS_L_REG, buff, 2);
    return ret;
}

/**
  * @brief  User-defined threshold value for xl interrupt event on generator.
  *         Data format is the same of output data raw: two’s complement with
  *         1LSb = 1.5mG.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  buff  that stores data read
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_int_gen_treshold_get(stmdev_ctx_t *ctx, uint8_t *buff) {
    int32_t ret;
    ret = lis2mdl_read_reg(ctx, LIS2MDL_INT_THS_L_REG, buff, 2);
    return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup    LIS2MDL_serial_interface
  * @brief       This section group all the functions concerning serial
  *              interface management
  * @{
  *
  */

/**
  * @brief  SPI Serial Interface Mode selection.[set]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      change the values of 4wspi in reg CFG_REG_C
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_spi_mode_set(stmdev_ctx_t *ctx, lis2mdl_sim_t val) {
    lis2mdl_cfg_reg_c_t reg;
    int32_t ret;

    ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_C, (uint8_t*)&reg, 1);

    if(ret == 0){
        reg._4wspi = (uint8_t)val;
        ret = lis2mdl_write_reg(ctx, LIS2MDL_CFG_REG_C, (uint8_t*)&reg, 1);
    }

    return ret;
}

/**
  * @brief  SPI Serial Interface Mode selection.[get]
  *
  * @param  ctx      read / write interface definitions
  * @param  val      Get the values of 4wspi in reg CFG_REG_C
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_spi_mode_get(stmdev_ctx_t *ctx, lis2mdl_sim_t *val) {
    lis2mdl_cfg_reg_c_t reg;
    int32_t ret;

    ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_C, (uint8_t*)&reg, 1);
    switch (reg._4wspi){
        case LIS2MDL_SPI_4_WIRE:
          *val = LIS2MDL_SPI_4_WIRE;
          break;
        case LIS2MDL_SPI_3_WIRE:
          *val = LIS2MDL_SPI_3_WIRE;
          break;
        default:
          *val = LIS2MDL_SPI_3_WIRE;
          break;
    }
    return ret;
}

/**
  * @brief  Enable/Disable I2C interface.[set]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   change the values of i2c_dis in reg CFG_REG_C
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_i2c_interface_set(stmdev_ctx_t *ctx, lis2mdl_i2c_dis_t val) {
    lis2mdl_cfg_reg_c_t reg;
    int32_t ret;

    ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_C, (uint8_t*)&reg, 1);

    if(ret == 0){
        reg.i2c_dis = (uint8_t)val;
        ret = lis2mdl_write_reg(ctx, LIS2MDL_CFG_REG_C, (uint8_t*)&reg, 1);
    }

    return ret;
}

/**
  * @brief  Enable/Disable I2C interface.[get]
  *
  * @param  ctx   read / write interface definitions.(ptr)
  * @param  val   Get the values of i2c_dis in reg CFG_REG_C.(ptr)
  * @retval       interface status.(MANDATORY: return 0 -> no Error)
  *
  */
int32_t lis2mdl_i2c_interface_get(stmdev_ctx_t *ctx, lis2mdl_i2c_dis_t *val) {
    lis2mdl_cfg_reg_c_t reg;
    int32_t ret;

    ret = lis2mdl_read_reg(ctx, LIS2MDL_CFG_REG_C, (uint8_t*)&reg, 1);
    switch (reg.i2c_dis){
        case LIS2MDL_I2C_ENABLE:
          *val = LIS2MDL_I2C_ENABLE;
          break;
        case LIS2MDL_I2C_DISABLE:
          *val = LIS2MDL_I2C_DISABLE;
          break;
        default:
          *val = LIS2MDL_I2C_ENABLE;
          break;
    }
    return ret;
}

/**
 * @brief  LIS2MDL init
 * @param  None
 * @retval true if ok
 */
bool lis2mdl_init(void) {
    i2c_init();
    /* Check device ID */
    lis2mdl_device_id_get(nullptr, &lis2_whoamI);

    if (lis2_whoamI != LIS2MDL_ID) {
        return false;
    }

    /* Restore default configuration */
    lis2mdl_reset_set(nullptr, PROPERTY_ENABLE);

    do {
        lis2mdl_reset_get(nullptr, &lis2_rst);
    } while (lis2_rst);

    /* Enable Block Data Update */
    lis2mdl_block_data_update_set(nullptr, PROPERTY_ENABLE);
    /* Set Output Data Rate */
    lis2mdl_data_rate_set(nullptr, LIS2MDL_ODR_10Hz);
    /* Set / Reset sensor mode */
    lis2mdl_set_rst_mode_set(nullptr, LIS2MDL_SENS_OFF_CANC_EVERY_ODR);
    /* Enable temperature compensation */
    lis2mdl_offset_temp_comp_set(nullptr, PROPERTY_ENABLE);
    /* Set Low-pass bandwidth to ODR/4 */
    //lis2mdl_low_pass_bandwidth_set(nullptr, LIS2MDL_ODR_DIV_4);
    /* Set device in continuous mode */
    lis2mdl_operating_mode_set(nullptr, LIS2MDL_CONTINUOUS_MODE);
    /* Enable interrupt generation on new data ready */
    lis2mdl_drdy_on_pin_set(nullptr, PROPERTY_ENABLE);
    return true;
}

/**
  * @brief  Convert magnetic data to [uT] 
  * @param  axis axis data
  * @retval result in [uT] 
  */
float magnetic_convert_to_ut(float axis) {
    return axis * 0.1;
}

/**
 * @brief  LIS2MDL get magnetic [mG]
 * @param  [out] axis_x pointer to x coordinate
 * @param  [out] axis_y pointer to x coordinate
 * @param  [out] axis_z pointer to x coordinate
 * @retval None
 */
void lis2mdl_get_magnetic(float* axis_x, float* axis_y, float* axis_z) {
    uint8_t reg;
    if ((axis_x == 0) || (axis_y == 0) || (axis_z == 0)) {
        return;
    }
    /* Read output only if new value is available */
    lis2mdl_mag_data_ready_get(nullptr, &reg);

    if (reg) {
        /* Read magnetic field data */
        memset(data_raw_magnetic, 0x00, 3 * sizeof(int16_t));
        lis2mdl_magnetic_raw_get(nullptr, (uint8_t*) data_raw_magnetic);
        magnetic_mG[0] = lis2mdl_from_lsb_to_mgauss( data_raw_magnetic[0]);
        magnetic_mG[1] = lis2mdl_from_lsb_to_mgauss( data_raw_magnetic[1]);
        magnetic_mG[2] = lis2mdl_from_lsb_to_mgauss( data_raw_magnetic[2]);
        *axis_x = magnetic_mG[0];
        *axis_y = magnetic_mG[1];
        *axis_z = magnetic_mG[2];
        /* Read temperature data */
        memset(&lis2_data_raw_temperature, 0x00, sizeof(int16_t));
        lis2mdl_temperature_raw_get(nullptr, (uint8_t*) &lis2_data_raw_temperature);
        lis2_temperature_degC =
        lis2mdl_from_lsb_to_celsius(lis2_data_raw_temperature);
    }
}


/**
 * @brief  get mesure pheriod [ns]
 * @param  [in] rate measure rate from lis2mdl_odr_t
 * @retval measure pheriod [ns] 0 if off, 0xffffffffffffffff in other caces
 */
uint64_t lis2mdl_get_pheriod_ns(lis2mdl_odr_t rate) {
    switch (rate) {
        case LIS2MDL_ODR_10Hz:
        return 1000000000 / 10;
        case LIS2MDL_ODR_20Hz:
        return 1000000000 / 20;
        case LIS2MDL_ODR_50Hz:
        return 1000000000 / 50;
        case LIS2MDL_ODR_100Hz:
        return 1000000000 / 100;
        default:
        break;
    }
    return 0xffffffffffffffff;
}

/**
 * @brief  get lis2mdl current mode
 * @param  [out] mode from lis2mdl_mode_t
 * @retval none
 */
void lis2mdl_get_mode(lis2mdl_mode_t* mode) {
    memcpy(mode, &lis2mdl_mode, sizeof(lis2mdl_mode_t));
}

/**
 * @brief  set lis2mdl current mode
 * @param  [in] mode from lis2mdl_mode_t
 * @retval none
 */
void lis2mdl_set_mode(lis2mdl_mode_t* mode) {

    memcpy(&lis2mdl_mode, mode, sizeof(lis2mdl_mode_t));
    lis2mdl_data_rate_set(nullptr, mode->data_rate);
    if (lis2mdl_mode.enable != false) {
        lis2mdl_enable();
    } else {
        lis2mdl_disable();
    }
}


/**
 * @brief  enable lis2mdl
 * @param  None
 * @retval None
 */
void lis2mdl_enable(void) {
    lis2mdl_mode.enable = true;
}

/**
 * @brief  disable lis2mdl
 * @param  None
 * @retval None
 */
void lis2mdl_disable(void) {
    lis2mdl_mode.enable = false;
}

/**
  * @}
  *
  */

