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
#include "ei_magnetometer.h"
#include "Lis2mdl.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"

#include "ei_config_types.h"

/* Constant defines -------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */
static float lis_data[MAGNETIC_AXIS_SAMPLED];

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool ei_magnetometer_init(void)
{
    bool initialised = false;
    uint8_t lis2_whoamI = 0;

    /* Check device ID */
    lis2_whoamI = 0;
    lis2mdl_device_id_get(nullptr, &lis2_whoamI);

    if (lis2_whoamI == LIS2MDL_ID) {
        (void)lis2mdl_init(); // we already checked if sensor is present

        /* Set Output Data Rate - different from default */
        lis2mdl_data_rate_set(nullptr, LIS2MDL_ODR_100Hz);

        ei_add_sensor_to_fusion_list(magnetic_sensor);
        initialised = true;
    }
    else {
        ei_printf("LIS2MDL magnetometer not found!\n");
    }

    return initialised;
}

/**
 * @brief 
 * 
 * @param n_samples 
 * @return float* 
 */
float *ei_fusion_magnetometer_read_data(int n_samples)
{
    uint8_t reg;
    static int16_t data_raw_magnetic[3];

    /* Read output only if new value is available */
    lis2mdl_mag_data_ready_get(nullptr, &reg);

    if (reg) {
        /* Read magnetic field data */
        memset(data_raw_magnetic, 0x00, MAGNETIC_AXIS_SAMPLED * sizeof(int16_t));
        memset(&lis_data, 0x00, sizeof(lis_data));

        lis2mdl_magnetic_raw_get(nullptr, (uint8_t *)data_raw_magnetic);

        for (uint8_t i = 0; i < MAGNETIC_AXIS_SAMPLED; i++) {
            lis_data[i] = lis2mdl_from_lsb_to_mgauss(data_raw_magnetic[i]) * 0.1;
        }
    }

    return lis_data;
}
