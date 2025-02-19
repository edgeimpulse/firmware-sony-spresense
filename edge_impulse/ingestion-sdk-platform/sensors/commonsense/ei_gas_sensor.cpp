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
