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
#include "ei_pressure.h"
#include "Lps22hh.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"

#include "ei_config_types.h"

/* Constant defines -------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */
static float lps_data[PRESSURE_AXIS_SAMPLED];

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool ei_pressure_init(void)
{
    bool initialised = false;
    uint8_t lps22_whoamI = 0;

    /* Check device ID */
    lps22_whoamI = 0;
    lps22hh_device_id_get(nullptr, &lps22_whoamI);

    if (lps22_whoamI == LPS22HH_ID) {
        (void)lps22hh_init(); // we already checked if sensor is present

        /* Set Output Data Rate - different from default */
        lps22hh_data_rate_set(nullptr, LPS22HH_75_Hz_LOW_NOISE);

        ei_add_sensor_to_fusion_list(pressure_sensor);
        initialised = true;
    }
    else {
        ei_printf("LPS22HH pressure sensor not found!\n");
    }

    return initialised;
}

/**
 * @brief 
 * 
 * @param n_samples 
 * @return float* 
 */
float *ei_fusion_pressure_read_data(int n_samples)
{
    lps22hh_reg_t reg;
    static uint32_t data_raw_pressure = 0;
    static int16_t lps22_data_raw_temperature = 0;

    memset(&lps_data, 0x00, sizeof(lps_data));

    /* Read output only if new value is available */
    lps22hh_read_reg(nullptr, LPS22HH_STATUS, (uint8_t *)&reg, 1);

    if (reg.status.p_da) {
        lps22hh_pressure_raw_get(nullptr, (uint8_t *)&data_raw_pressure);
        lps_data[0] = (lps22hh_from_lsb_to_hpa(data_raw_pressure) / 10.0);
    }

    if (reg.status.t_da) {
        lps22hh_temperature_raw_get(nullptr, (uint8_t *)&lps22_data_raw_temperature);
        lps_data[1] = lps22hh_from_lsb_to_celsius(lps22_data_raw_temperature);
    }

    return lps_data;
}
