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
#include "ei_environmentsensor.h"
#include "Hts221.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"

#include "ei_config_types.h"

/* Constant defines -------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */
static float htps_data[ENV_AXIS_SAMPLED];

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool ei_environment_init(void)
{
    bool initialised = false;
    uint8_t hts221_whoamI = 0;

    hts221_device_id_get(&hts221_whoamI);

    if (hts221_whoamI == HTS221_ID) {
        /* call drivr init */
        hts221_init();

        /* Enable Block Data Update */
        hts221_block_data_update_set(PROPERTY_ENABLE);
        /* Set Output Data Rate */
        hts221_data_rate_set(HTS221_ODR_12Hz5);

        hts221_humidity_avg_set(HTS221_H_AVG_4);
        hts221_temperature_avg_set(HTS221_T_AVG_2);

        /* Device power on */
        hts221_enable();

        ei_add_sensor_to_fusion_list(environment_sensor);
        initialised = true;
    }
    else {
        ei_printf("HTS221 sensor not found!\n");
    }

    return initialised;
}

/**
 * @brief 
 * 
 * @param n_samples 
 * @return float* 
 */
float *ei_fusion_environment_read_data(int n_samples)
{
    /* first humidity then temperature */
    hts221_get_hym_temp(&htps_data[1], &htps_data[0]);

    return htps_data;
}
