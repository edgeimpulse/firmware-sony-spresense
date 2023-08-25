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
#include "ei_gas_sensor.h"
#include "Hts221.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "sensirion_common.h"
#include "sensirion_i2c.h"
#include "sensirion_i2c_hal.h"
#include "sensirion_voc_algorithm.h"
#include "sgp40_i2c.h"

#include "ei_config_types.h"

/* Constant defines -------------------------------------------------------- */
#define PROCESS_GAS   0 // enable to perform voc algorithm
#define USE_REAL_DATA 1 // enable to read temp and hum and use them

static uint16_t default_rh = 0x8000;
static uint16_t default_temp = 0x6666;

VocAlgorithmParams voc_algorithm_params;

/* Private variables ------------------------------------------------------- */
static float gas_data;

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool ei_gas_init(void)
{
    bool initialised = false;
    uint16_t serial_number[3];
    uint8_t serial_number_size = 3;

    sensirion_i2c_hal_init();

    if (sgp40_get_serial_number(serial_number, serial_number_size) == 0) {
        VocAlgorithm_init(&voc_algorithm_params);

        ei_add_sensor_to_fusion_list(gas_sensor);
        initialised = true;
    }

    if (!initialised) {
        ei_printf("sgp40 gas sensor not found!\n");
    }

    return initialised;
}

/**
 * @brief 
 * 
 * @param n_samples 
 * @return float* 
 */
float *ei_fusion_gas_read_data(int n_samples)
{
    int16_t error;
    uint16_t sraw_voc;

#if USE_REAL_DATA == 1
    float temp;
    float hum;
    uint16_t hum_tick;
    uint16_t temp_tick;

    hts221_get_hym_temp(&hum, &temp);
    hum_tick = (uint16_t)((hum * 65535) / 100 + 0.5);
    temp_tick = (uint16_t)(((temp + 45) * 65535) / 175);

    error = sgp40_measure_raw_signal(hum_tick, temp_tick, &sraw_voc);
#else
    error = sgp40_measure_raw_signal(default_rh, default_temp, &sraw_voc);
#endif

    if (error == 0) {
#if PROCESS_GAS == 1
        int32_t voc_index = 0;
        VocAlgorithm_process(&voc_algorithm_params, sraw_voc, &voc_index);
        gas_data = voc_index;
#else
        gas_data = sraw_voc;
#endif
    }
    else {
        gas_data = 0.0;
    }

    return &gas_data;
}
