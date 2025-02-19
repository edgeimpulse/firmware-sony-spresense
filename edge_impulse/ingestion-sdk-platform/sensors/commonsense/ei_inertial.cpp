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

/* Include ----------------------------------------------------------------- */
#include "ei_inertial.h"
#include "Lsm6dso32.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"

/* Constant defines -------------------------------------------------------- */
#define CONVERT_G_TO_MS2 (9.80665f)
#define ACC_RAW_SCALING  (32767.5f)

#define ACC_SCALE_FACTOR (2.0f * CONVERT_G_TO_MS2) / ACC_RAW_SCALING

#define CONVERT_ADC_GYR (float)(250.0f / 32768.0f)

/* Private variables ------------------------------------------------------- */
static float imu_data[INERTIAL_AXIS_SAMPLED];
static stmdev_ctx_t dev_ctx;

/**
 * @brief      Setup payload header
 *
 * @return     true
 */
bool ei_inertial_init(void)
{
    uint8_t tmp_read = 0x00;
    uint8_t retries = 10;

    /* Check device ID */
    lsm6dso32_device_id_get(&dev_ctx, &tmp_read);

    if (tmp_read == LSM6DSO32_ID) {

        /* Restore default configuration */
        lsm6dso32_reset_set(&dev_ctx, PROPERTY_ENABLE);

        do {
            lsm6dso32_reset_get(&dev_ctx, &tmp_read);
            ei_sleep(10);
        } while ((tmp_read) && (--retries));

        /* Disable I3C interface */
        lsm6dso32_i3c_disable_set(&dev_ctx, LSM6DSO32_I3C_DISABLE);

        /* Enable Block Data Update */
        lsm6dso32_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

        /* Set full scale */
        lsm6dso32_xl_full_scale_set(&dev_ctx, LSM6DSO32_4g); // it's 2g !
        lsm6dso32_gy_full_scale_set(&dev_ctx, LSM6DSO32_250dps);

        /* Set ODR (Output Data Rate) and power mode*/
        lsm6dso32_xl_data_rate_set(&dev_ctx, LSM6DSO32_XL_ODR_208Hz_NORMAL_MD);
        lsm6dso32_gy_data_rate_set(&dev_ctx, LSM6DSO32_GY_ODR_208Hz_NORMAL_MD);

        if (ei_add_sensor_to_fusion_list(inertial_sensor) == false) {
            ei_printf("ERR: failed to register Inertial LSM6dso sensor!\r\n");
            return false;
        }
    }
    else {
        ei_printf("IMU missing or not working correctly\r\n");
        return false;
    }

    return true;
}

/**
 * @brief 
 * 
 * @param n_samples 
 * @return float* 
 */
float *ei_fusion_inertial_read_data(int n_samples)
{
    lsm6dso32_reg_t reg;
    static int16_t data_raw_read[3] = { 0 };
    uint8_t i;

    memset(imu_data, 0, sizeof(imu_data));

    /* Read output only if new data is available */
    lsm6dso32_status_reg_get(&dev_ctx, &reg.status_reg);

    if (reg.status_reg.xlda) {
        /* Read acceleration data */
        lsm6dso32_acceleration_raw_get(&dev_ctx, (uint8_t *)data_raw_read);

        for (i = 0; i < 3; i++) {
            //imu_data[i] = lsm6dso32_from_fs4_to_mg(data_raw_read[i]);
            imu_data[i] = data_raw_read[i] * ACC_SCALE_FACTOR;
        }
    }

    if ((n_samples > 3) && (reg.status_reg.gda)) {

        lsm6dso32_angular_rate_raw_get(&dev_ctx, (uint8_t *)data_raw_read);

        for (i = 0; i < 3; i++) {
            imu_data[3 + i] = data_raw_read[i] * CONVERT_ADC_GYR;
        }
    }

    return imu_data;
}
