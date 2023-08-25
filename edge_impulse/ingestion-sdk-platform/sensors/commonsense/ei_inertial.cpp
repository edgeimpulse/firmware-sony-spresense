/* Edge Impulse ingestion SDK
 * Copyright (c) 2023 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
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
