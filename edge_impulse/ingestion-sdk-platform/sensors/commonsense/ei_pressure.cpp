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
