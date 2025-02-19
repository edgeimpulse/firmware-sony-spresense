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
#include "ei_proximity.h"
#include "VL53L1X_api.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"

#include "ei_config_types.h"

/* Constant defines -------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */
static float prox_data[PROXIMITY_AXIS_SAMPLED];
static uint16_t dev = 0;

static bool ei_vl53l1_init(void);
static bool ei_vl53l1_start_measure(void);

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool ei_proximity_init(void)
{
    bool initialised = false;
    uint8_t byteData;
    int status;
    uint8_t sensorState = 0;
    uint8_t retries = 10;

    do {
        status += vl53l1x_boot_state(dev, &sensorState);
        ei_sleep(20);
    } while ((sensorState == 0) && (--retries));

    if (retries) {
        ei_add_sensor_to_fusion_list(proximity_sensor);
        initialised = true;
    }
    else {
        ei_printf("ERR: proximity sensor not initialised\r\n");
    }

    return initialised;
}

/**
 * @brief 
 * 
 * @param n_samples 
 * @return float* 
 */
float *ei_fusion_proximity_read_data(int n_samples)
{
    static uint8_t first_range = 1;
    VL53L1X_Result_t Results = { 0 };
    int status = 0;
    uint8_t dataReady = 0;

    ei_vl53l1_start_measure(); // restart

    while (dataReady == 0) {
        status = vl53l1x_check_for_data_ready(dev, &dataReady);
        ei_sleep(1);
    }

    dataReady = 0;

    /* Get the data the new way */
    status += vl53l1x_get_result(dev, &Results);

    /* trigger next ranging */
    status += vl53l1x_clear_interrupt(dev);
    if (first_range) {
        /* very first measurement shall be ignored
         * thus requires twice call
         */
        status += vl53l1x_clear_interrupt(dev);
        first_range = 0;
    }

    prox_data[0] = Results.Distance;

    return prox_data;
}

/**
 * @brief 
 * 
 */
static bool ei_vl53l1_init(void)
{
    uint8_t byteData;
    int status;
    uint8_t sensorState = 0;

    status = VL53L1_RdByte(dev, 0x010F, &byteData);
    status += VL53L1_RdByte(dev, 0x0110, &byteData);
    status += VL53L1_RdByte(dev, 0x0111, &byteData);

    return (true);
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
static bool ei_vl53l1_start_measure(void)
{
    int status;

    status = vl53l1x_sensor_init(dev); // we already checked if sensor is present
    status += vl53l1x_set_timing_budgetIn_ms(dev, 1);
    status += vl53l1x_set_distance_mode(dev, 2); /* 1=short, 2=long */
    status += vl53l1x_set_inter_measurementIn_ms(dev, 1);
    status += vl53l1x_start_ranging(dev);

    return (status == 0); //
}
