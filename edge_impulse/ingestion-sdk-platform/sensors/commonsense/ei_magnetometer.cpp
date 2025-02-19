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
