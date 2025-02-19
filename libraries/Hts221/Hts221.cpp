/* The Clear BSD License
 *
 * Copyright (c) 2025 EdgeImpulse Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <string.h>
#include <stdint.h>
#include "Hts221.h"
#include "../I2c/I2c.h"
#include "unistd.h"

hts221_mode_t hts221_mode = {0};
typedef struct {
    float x0;
    float y0;
    float x1;
    float y1;
} lin_t;

static uint8_t hts221_whoamI = 0;

static int16_t hts221_data_raw_humidity;
static int16_t hts221_data_raw_temperature;
static float hts221_humidity_perc;
static float hts221_temperature_degC;

static lin_t lin_hum;
static lin_t lin_temp;

/**
  * @defgroup  HTS221
  * @brief     This file provides a set of functions needed to drive the
  *            hts221 enhanced inertial module.
  * @{
  *
  */

/**
  * @defgroup  HTS221_interfaces_functions
  * @brief     This section provide a set of functions used to read and write
  *            a generic register of the device.
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
int32_t hts221_read_reg(uint8_t reg, uint8_t* data, uint16_t len) {
    return (int32_t) i2c_read_regs(HTS221_I2C_ADDRESS, reg, data, len);
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
int32_t hts221_write_reg(uint8_t reg, uint8_t* data, uint16_t len) {
    return (int32_t) i2c_write_regs(HTS221_I2C_ADDRESS, reg, data, len);
}

/**
  * @}
  *
  */

/**
  * @defgroup  HTS221_Data_Generation
  * @brief     This section group all the functions concerning data generation
  * @{
  *
  */

/**
  * @brief  The numbers of averaged humidity samples.[set]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     change the values of avgh in reg AV_CONF
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_humidity_avg_set(hts221_avgh_t val) {
    hts221_av_conf_t reg;
    int32_t ret;

    ret = hts221_read_reg(HTS221_AV_CONF, (uint8_t*) &reg, 1);

    if(ret == 0){
      reg.avgh = (uint8_t)val;
      ret = hts221_write_reg(HTS221_AV_CONF, (uint8_t*) &reg, 1);
    }

    return ret;
}

/**
  * @brief  The numbers of averaged humidity samples.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     Get the values of avgh in reg AV_CONF
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_humidity_avg_get(hts221_avgh_t *val) {
    hts221_av_conf_t reg;
    int32_t ret;

    ret = hts221_read_reg(HTS221_AV_CONF, (uint8_t*) &reg, 1);

    switch (reg.avgh) {
        case HTS221_H_AVG_4:
          *val = HTS221_H_AVG_4;
          break;
        case HTS221_H_AVG_8:
          *val = HTS221_H_AVG_8;
          break;
        case HTS221_H_AVG_16:
          *val = HTS221_H_AVG_16;
          break;
        case HTS221_H_AVG_32:
          *val = HTS221_H_AVG_32;
          break;
        case HTS221_H_AVG_64:
          *val = HTS221_H_AVG_64;
          break;
        case HTS221_H_AVG_128:
          *val = HTS221_H_AVG_128;
          break;
        case HTS221_H_AVG_256:
          *val = HTS221_H_AVG_256;
          break;
        case HTS221_H_AVG_512:
          *val = HTS221_H_AVG_512;
          break;
        default:
          *val = HTS221_H_AVG_ND;
          break;
    }

    return ret;
}

/**
  * @brief  The numbers of averaged temperature samples.[set]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     change the values of avgt in reg AV_CONF
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_temperature_avg_set(hts221_avgt_t val) {
    hts221_av_conf_t reg;
    int32_t ret;

    ret = hts221_read_reg(HTS221_AV_CONF, (uint8_t*) &reg, 1);

    if(ret == 0){
        reg.avgt = (uint8_t)val;
        ret = hts221_write_reg(HTS221_AV_CONF, (uint8_t*) &reg, 1);
    }

    return ret;
}

/**
  * @brief  The numbers of averaged temperature samples.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     Get the values of avgt in reg AV_CONF
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_temperature_avg_get(hts221_avgt_t *val) {
    hts221_av_conf_t reg;
    int32_t ret;

    ret = hts221_read_reg(HTS221_AV_CONF, (uint8_t*) &reg, 1);

    switch (reg.avgh) {
        case HTS221_T_AVG_2:
          *val = HTS221_T_AVG_2;
          break;
        case HTS221_T_AVG_4:
          *val = HTS221_T_AVG_4;
          break;
        case HTS221_T_AVG_8:
          *val = HTS221_T_AVG_8;
          break;
        case HTS221_T_AVG_16:
          *val = HTS221_T_AVG_16;
          break;
        case HTS221_T_AVG_32:
          *val = HTS221_T_AVG_32;
          break;
        case HTS221_T_AVG_64:
          *val = HTS221_T_AVG_64;
          break;
        case HTS221_T_AVG_128:
          *val = HTS221_T_AVG_128;
          break;
        case HTS221_T_AVG_256:
          *val = HTS221_T_AVG_256;
          break;
        default:
          *val = HTS221_T_AVG_ND;
          break;
    }

    return ret;
}

/**
  * @brief  Output data rate selection.[set]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     change the values of odr in reg CTRL_REG1
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_data_rate_set(hts221_odr_t val) {
    hts221_ctrl_reg1_t reg;
    int32_t ret;

    ret = hts221_read_reg(HTS221_CTRL_REG1, (uint8_t*) &reg, 1);

    if(ret == 0){
        reg.odr = (uint8_t)val;
        ret = hts221_write_reg(HTS221_CTRL_REG1, (uint8_t*) &reg, 1);
    }

    return ret;
}

/**
  * @brief  Output data rate selection.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     Get the values of odr in reg CTRL_REG1
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_data_rate_get(hts221_odr_t *val) {
    hts221_ctrl_reg1_t reg;
    int32_t ret;

    ret = hts221_read_reg(HTS221_CTRL_REG1, (uint8_t*) &reg, 1);

    switch (reg.odr) {
        case HTS221_ONE_SHOT:
          *val = HTS221_ONE_SHOT;
          break;
        case HTS221_ODR_1Hz:
          *val = HTS221_ODR_1Hz;
          break;
        case HTS221_ODR_7Hz:
          *val = HTS221_ODR_7Hz;
          break;
        case HTS221_ODR_12Hz5:
          *val = HTS221_ODR_12Hz5;
          break;
        default:
          *val = HTS221_ODR_ND;
          break;
    }

    return ret;
}

/**
  * @brief  Block data update.[set]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     change the values of bdu in reg CTRL_REG1
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_block_data_update_set(uint8_t val) {
    hts221_ctrl_reg1_t reg;
    int32_t ret;

    ret = hts221_read_reg(HTS221_CTRL_REG1, (uint8_t*) &reg, 1);

    if(ret == 0){
        reg.bdu = val;
        ret = hts221_write_reg(HTS221_CTRL_REG1, (uint8_t*) &reg, 1);
    }

    return ret;
}

/**
  * @brief  Block data update.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     change the values of bdu in reg CTRL_REG1
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_block_data_update_get(uint8_t *val) {
    hts221_ctrl_reg1_t reg;
    int32_t ret;

    ret = hts221_read_reg(HTS221_CTRL_REG1, (uint8_t*) &reg, 1);
    *val = reg.bdu;

    return ret;
}

/**
  * @brief  One-shot mode. Device perform a single measure.[set]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     change the values of one_shot in reg CTRL_REG2
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_one_shoot_trigger_set(uint8_t val) {
    hts221_ctrl_reg2_t reg;
    int32_t ret;

    ret = hts221_read_reg(HTS221_CTRL_REG2, (uint8_t*) &reg, 1);

    if(ret == 0){
        reg.one_shot = val;
        ret = hts221_write_reg(HTS221_CTRL_REG2, (uint8_t*) &reg, 1);
    }

    return ret;
}

/**
  * @brief  One-shot mode. Device perform a single measure.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     change the values of one_shot in reg CTRL_REG2
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_one_shoot_trigger_get(uint8_t *val) {
    hts221_ctrl_reg2_t reg;
    int32_t ret;

    ret = hts221_read_reg(HTS221_CTRL_REG2, (uint8_t*) &reg, 1);
    *val = reg.one_shot;

    return ret;
}

/**
  * @brief  Temperature data available.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     change the values of t_da in reg STATUS_REG
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_temp_data_ready_get(uint8_t *val) {
    hts221_status_reg_t reg;
    int32_t ret;

    ret = hts221_read_reg(HTS221_STATUS_REG, (uint8_t*) &reg, 1);
    *val = reg.t_da;

    return ret;
}

/**
  * @brief  Humidity data available.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     change the values of h_da in reg STATUS_REG
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_hum_data_ready_get(uint8_t *val) {
    hts221_status_reg_t reg;
    int32_t ret;

    ret = hts221_read_reg(HTS221_STATUS_REG, (uint8_t*) &reg, 1);
    *val = reg.h_da;

    return ret;
}

/**
  * @brief  Humidity output value[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  buff    buffer that stores data read
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_humidity_raw_get(uint8_t *buff) {
    int32_t ret;
    uint8_t * pnt = (uint8_t *) buff;
    ret = hts221_read_reg(HTS221_HUMIDITY_OUT_L, &pnt[0], 1);
    ret = hts221_read_reg(HTS221_HUMIDITY_OUT_H, &pnt[1], 1);
    return ret;
}

/**
  * @brief  Temperature output value[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  buff    buffer that stores data read
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_temperature_raw_get(uint8_t *buff) {
    int32_t ret;
    uint8_t * pnt = (uint8_t *) buff;
    ret = hts221_read_reg(HTS221_TEMP_OUT_L, &pnt[0], 1);
    ret = hts221_read_reg(HTS221_TEMP_OUT_H, &pnt[1], 1);
    return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  HTS221_common
  * @brief     This section group common usefull functions
  * @{
  *
  */

/**
  * @brief  Device Who amI.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  buff    buffer that stores data read
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_device_id_get(uint8_t *buff) {
    int32_t ret;
    ret = hts221_read_reg(HTS221_WHO_AM_I, buff, 1);
    return ret;
}

/**
  * @brief  Switch device on/off.[set]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     change the values of pd in reg CTRL_REG1
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_power_on_set(uint8_t val) {
    hts221_ctrl_reg1_t reg;
    int32_t ret;

    ret = hts221_read_reg(HTS221_CTRL_REG1, (uint8_t*) &reg, 1);

    if(ret == 0){
        reg.pd = val;
        ret = hts221_write_reg(HTS221_CTRL_REG1, (uint8_t*) &reg, 1);
    }
    return ret;
}

/**
  * @brief  Switch device on/off.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     change the values of pd in reg CTRL_REG1
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_power_on_get(uint8_t *val) {
    hts221_ctrl_reg1_t reg;
    int32_t mm_error;

    mm_error = hts221_read_reg(HTS221_CTRL_REG1, (uint8_t*) &reg, 1);
    *val = reg.pd;

    return mm_error;
}

/**
  * @brief  Heater enable / disable.[set]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     change the values of heater in reg CTRL_REG2
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_heater_set(uint8_t val) {
    hts221_ctrl_reg2_t reg;
    int32_t ret;

    ret = hts221_read_reg(HTS221_CTRL_REG2, (uint8_t*) &reg, 1);

    if(ret == 0){
        reg.heater = val;
        ret = hts221_write_reg(HTS221_CTRL_REG2, (uint8_t*) &reg, 1);
    }

    return ret;
}

/**
  * @brief  Heater enable / disable.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     change the values of heater in reg CTRL_REG2
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_heater_get(uint8_t *val) {
    hts221_ctrl_reg2_t reg;
    int32_t ret;

    ret = hts221_read_reg(HTS221_CTRL_REG2, (uint8_t*) &reg, 1);
    *val = reg.heater;

    return ret;
}

/**
  * @brief  Reboot memory content. Reload the calibration parameters.[set]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     change the values of boot in reg CTRL_REG2
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_boot_set(uint8_t val) {
    hts221_ctrl_reg2_t reg;
    int32_t ret;

    ret = hts221_read_reg(HTS221_CTRL_REG2, (uint8_t*) &reg, 1);

    if(ret == 0){
        reg.boot = val;
        ret = hts221_write_reg(HTS221_CTRL_REG2, (uint8_t*) &reg, 1);
    }

    return ret;
}

/**
  * @brief  Reboot memory content. Reload the calibration parameters.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     change the values of boot in reg CTRL_REG2
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_boot_get(uint8_t *val) {
    hts221_ctrl_reg2_t reg;
    int32_t ret;

    ret = hts221_read_reg(HTS221_CTRL_REG2, (uint8_t*) &reg, 1);
    *val = reg.boot;

    return ret;
}

/**
  * @brief  Info about device status.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     Registers STATUS_REG
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_status_get(hts221_status_reg_t *val) {
    int32_t ret;
    ret = hts221_read_reg(HTS221_STATUS_REG, (uint8_t*) val, 1);
    return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  HTS221_interrupts
  * @brief   This section group all the functions that manage interrupts
  * @{
  *
  */

/**
  * @brief  Data-ready signal on INT_DRDY pin.[set]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     change the values of drdy in reg CTRL_REG3
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_drdy_on_int_set(uint8_t val) {
    hts221_ctrl_reg3_t reg;
    int32_t ret;

    ret = hts221_read_reg(HTS221_CTRL_REG3, (uint8_t*) &reg, 1);

    if(ret == 0){
        reg.drdy = val;
        ret = hts221_write_reg(HTS221_CTRL_REG3, (uint8_t*) &reg, 1);
    }

    return ret;
}

/**
  * @brief  Data-ready signal on INT_DRDY pin.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     change the values of drdy in reg CTRL_REG3
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_drdy_on_int_get(uint8_t *val) {
    hts221_ctrl_reg3_t reg;
    int32_t ret;

    ret = hts221_read_reg(HTS221_CTRL_REG3, (uint8_t*) &reg, 1);
    *val = reg.drdy;

    return ret;
}

/**
  * @brief  Push-pull/open drain selection on interrupt pads.[set]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     change the values of pp_od in reg CTRL_REG3
  *
  */
int32_t hts221_pin_mode_set(hts221_pp_od_t val) {
    hts221_ctrl_reg3_t reg;
    int32_t ret;

    ret = hts221_read_reg(HTS221_CTRL_REG3, (uint8_t*) &reg, 1);

    if(ret == 0){
        reg.pp_od = (uint8_t)val;
        ret = hts221_write_reg(HTS221_CTRL_REG3, (uint8_t*) &reg, 1);
    }

    return ret;
}

/**
  * @brief  Push-pull/open drain selection on interrupt pads.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     Get the values of pp_od in reg CTRL_REG3
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_pin_mode_get(hts221_pp_od_t *val) {
    hts221_ctrl_reg3_t reg;
    int32_t ret;

    ret = hts221_read_reg(HTS221_CTRL_REG3, (uint8_t*) &reg, 1);

    switch (reg.pp_od) {
        case HTS221_PUSH_PULL:
          *val = HTS221_PUSH_PULL;
          break;
        case HTS221_OPEN_DRAIN:
          *val = HTS221_OPEN_DRAIN;
          break;
        default:
          *val = HTS221_PIN_MODE_ND;
          break;
    }

    return ret;
}

/**
  * @brief  Interrupt active-high/low.[set]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     change the values of drdy_h_l in reg CTRL_REG3
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_int_polarity_set(hts221_drdy_h_l_t val) {
    hts221_ctrl_reg3_t reg;
    int32_t ret;

    ret = hts221_read_reg(HTS221_CTRL_REG3, (uint8_t*) &reg, 1);

    if(ret == 0){
        reg.drdy_h_l = (uint8_t)val;
        ret = hts221_write_reg(HTS221_CTRL_REG3, (uint8_t*) &reg, 1);
    }

    return ret;
}

/**
  * @brief  Interrupt active-high/low.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     Get the values of drdy_h_l in reg CTRL_REG3
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_int_polarity_get(hts221_drdy_h_l_t *val) {
    hts221_ctrl_reg3_t reg;
    int32_t ret;

    ret = hts221_read_reg(HTS221_CTRL_REG3, (uint8_t*) &reg, 1);

    switch (reg.drdy_h_l) {
        case HTS221_ACTIVE_HIGH:
          *val = HTS221_ACTIVE_HIGH;
          break;
        case HTS221_ACTIVE_LOW:
          *val = HTS221_ACTIVE_LOW;
          break;
        default:
          *val = HTS221_ACTIVE_ND;
          break;
    }

    return ret;
}

/**
  * @}
  *
  */

/**
  * @defgroup  HTS221_calibration
  * @brief     This section group all the calibration coefficients need
  *            for reading data
  * @{
  *
  */

/**
  * @brief  First calibration point for Rh Humidity.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     buffer that stores data read
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_hum_rh_point_0_get(float_t *val) {
    uint8_t coeff;
    int32_t ret;

    ret = hts221_read_reg(HTS221_H0_RH_X2, &coeff, 1);
    *val = coeff / 2.0f;

    return ret;
}

/**
  * @brief  Second calibration point for Rh Humidity.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     buffer that stores data read
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_hum_rh_point_1_get(float_t *val) {
    uint8_t coeff;
    int32_t ret;

    ret = hts221_read_reg(HTS221_H1_RH_X2, &coeff, 1);
    *val = coeff / 2.0f;

    return ret;
}

/**
  * @brief  First calibration point for degC temperature.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  buff    buffer that stores data read
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_temp_deg_point_0_get(float_t *val) {
    hts221_t1_t0_msb_t reg;
    uint8_t coeff_h, coeff_l;
    int32_t ret;

    ret = hts221_read_reg(HTS221_T0_DEGC_X8, &coeff_l, 1);

    if(ret == 0){
        ret = hts221_read_reg(HTS221_T1_T0_MSB, (uint8_t*) &reg, 1);
        coeff_h = reg.t0_msb;
        *val = ((coeff_h << 8) + coeff_l) / 8.0f;
    }

    return ret;
}

/**
  * @brief  Second calibration point for degC temperature.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     buffer that stores data read
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_temp_deg_point_1_get(float_t *val) {
    hts221_t1_t0_msb_t reg;
    uint8_t coeff_h, coeff_l;
    int32_t ret;

    ret = hts221_read_reg(HTS221_T1_DEGC_X8, &coeff_l, 1);

    if(ret == 0){
        ret = hts221_read_reg(HTS221_T1_T0_MSB, (uint8_t*) &reg, 1);
        coeff_h = reg.t1_msb;
        *val = ((coeff_h << 8) + coeff_l) / 8.0f;
    }

    return ret;
}

/**
  * @brief  First calibration point for humidity in LSB.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     buffer that stores data read
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_hum_adc_point_0_get(float_t *val) {
    uint8_t coeff_p[2];
    int16_t coeff;
    int32_t ret;
    ret = hts221_read_reg(HTS221_H0_T0_OUT_L, &coeff_p[0], 1);
    ret = hts221_read_reg(HTS221_H0_T0_OUT_H, &coeff_p[1], 1);
    coeff = (coeff_p[1] << 8) + coeff_p[0];
    *val = coeff * 1.0f;
    return ret;
}

/**
  * @brief  Second calibration point for humidity in LSB.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     buffer that stores data read
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_hum_adc_point_1_get(float_t *val) {
    uint8_t coeff_p[2];
    int16_t coeff;
    int32_t ret;    
    ret = hts221_read_reg(HTS221_H1_T0_OUT_L, &coeff_p[0], 1);
    ret = hts221_read_reg(HTS221_H1_T0_OUT_H, &coeff_p[1], 1);
    coeff = (coeff_p[1] << 8) + coeff_p[0];
    *val = coeff * 1.0f;
    return ret;
}

/**
  * @brief  First calibration point for temperature in LSB.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     buffer that stores data read
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_temp_adc_point_0_get(float_t *val) {
    uint8_t coeff_p[2];
    int16_t coeff;
    int32_t ret;    
    ret = hts221_read_reg(HTS221_T0_OUT_L, &coeff_p[0], 1);
    ret = hts221_read_reg(HTS221_T0_OUT_H, &coeff_p[1], 1);
    coeff = (coeff_p[1] << 8) + coeff_p[0];
    *val = coeff * 1.0f;
    return ret;
}

/**
  * @brief  Second calibration point for temperature in LSB.[get]
  *
  * @param  ctx     read / write interface definitions
  * @param  val     buffer that stores data read
  * @retval         interface status (MANDATORY: return 0 -> no Error)
  *
  */
int32_t hts221_temp_adc_point_1_get(float_t *val) {
    uint8_t coeff_p[2];
    int16_t coeff;
    int32_t ret;
    ret = hts221_read_reg(HTS221_T1_OUT_L, &coeff_p[0], 1);
    ret = hts221_read_reg(HTS221_T1_OUT_H, &coeff_p[1], 1);    
    coeff = (coeff_p[1] << 8) + coeff_p[0];
    *val = coeff * 1.0f;
    return ret;
}

static float linear_interpolation(lin_t *lin, int16_t x) {    
  return ((lin->y1 - lin->y0) * x + ((lin->x1 * lin->y0) -
                                     (lin->x0 * lin->y1)))
         / (lin->x1 - lin->x0);
}

/**
 * @brief  HTS221 init
 * @param  None
 * @retval None
 */
void hts221_init(void) {
    i2c_init();

    hts221_device_id_get(&hts221_whoamI); 

    /* Read humidity calibration coefficient */
    hts221_hum_adc_point_0_get(&lin_hum.x0);
    hts221_hum_rh_point_0_get(&lin_hum.y0);
    hts221_hum_adc_point_1_get(&lin_hum.x1);
    hts221_hum_rh_point_1_get(&lin_hum.y1);
    /* Read temperature calibration coefficient */
    hts221_temp_adc_point_0_get(&lin_temp.x0);
    hts221_temp_deg_point_0_get(&lin_temp.y0);
    hts221_temp_adc_point_1_get(&lin_temp.x1);
    hts221_temp_deg_point_1_get(&lin_temp.y1);
    /* Enable Block Data Update */
    hts221_block_data_update_set(PROPERTY_ENABLE);
    /* Set Output Data Rate */
    hts221_data_rate_set(HTS221_ODR_1Hz);
    /* Device power on */
    hts221_power_on_set(PROPERTY_ENABLE);
}

/**
 * @brief  HTS221 get humidity and temperature
 * @param  [Out] humidity pointer to humidity
 * @param  [Out] temperature pointer to temperature
 * @retval None
 */
void hts221_get_hym_temp(float* humidity, float* temperature) {
    /* Read output only if new value is available */
    //hts221_init();
    hts221_reg_t reg;
    if ((humidity == 0) || (temperature == 0)) {
        return;
    }
    hts221_status_get(&reg.status_reg);

    if (reg.status_reg.h_da) {
        /* Read humidity data */
        memset(&hts221_data_raw_humidity, 0x00, sizeof(int16_t));
        hts221_humidity_raw_get((uint8_t*) &hts221_data_raw_humidity);
        hts221_humidity_perc = linear_interpolation(&lin_hum, hts221_data_raw_humidity);
        //hts221_humidity_perc = 0.0 - hts221_humidity_perc;
        if (hts221_humidity_perc < 0) {
            hts221_humidity_perc = 0;
        }

        if (hts221_humidity_perc > 100) {
            hts221_humidity_perc = 100;
        }

        reg.status_reg.h_da = 0;
        *humidity = hts221_humidity_perc;
    }

    if (reg.status_reg.t_da) {
        /* Read temperature data */
        memset(&hts221_data_raw_temperature, 0x00, sizeof(int16_t));
        hts221_temperature_raw_get((uint8_t*) &hts221_data_raw_temperature);
        hts221_temperature_degC = linear_interpolation(&lin_temp, hts221_data_raw_temperature);

        reg.status_reg.t_da = 0;
        *temperature = hts221_temperature_degC;
    }
}



/**
 * @brief  get mesure pheriod [ns]
 * @param  [in] rate measure rate from hts221_odr_t
 * @retval measure pheriod [ns] 0 if off, 0xffffffffffffffff in other caces
 */
uint64_t hts221_get_pheriod_ns(hts221_odr_t rate) {
    switch (rate) {
        case HTS221_ONE_SHOT:
        return 0;
        case HTS221_ODR_1Hz:
        return 1000000000 / 1;
        case HTS221_ODR_7Hz:
        return 1000000000 / 7;
        case HTS221_ODR_12Hz5:
        return 1000000000 / 125;
        default:
        break;
    }
    return 0xffffffffffffffff;
}

/**
 * @brief  get hts221 current mode
 * @param  [out] mode from hts221_mode_t
 * @retval none
 */
void hts221_get_mode(hts221_mode_t* mode) {
    memcpy(mode, &hts221_mode, sizeof(hts221_mode_t));
}

/**
 * @brief  set hts221 current mode
 * @param  [in] mode from hts221_mode_t
 * @retval none
 */
void hts221_set_mode(hts221_mode_t* mode) {

    memcpy(&hts221_mode, mode, sizeof(hts221_mode_t));
    hts221_data_rate_set(mode->data_rate);
    hts221_humidity_avg_set(mode->hts221_avgh);
    hts221_temperature_avg_set(mode->hts221_avgt);
    if (hts221_mode.enable != false) {
        hts221_enable();
    } else {
        hts221_disable();
    }
}

/**
 * @brief  enable hts221
 * @param  None
 * @retval None
 */
void hts221_enable(void) {
    /* Device power on */
    hts221_power_on_set(PROPERTY_ENABLE);
    hts221_mode.enable = true;
}

/**
 * @brief  disable hts221
 * @param  None
 * @retval None
 */
void hts221_disable(void) {
    /* Device power off */
    hts221_power_on_set(PROPERTY_DISABLE);
    hts221_mode.enable = false;
}

/**
  * @}
  *
  */

