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
