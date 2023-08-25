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
