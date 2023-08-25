/* Edge Impulse ingestion SDK
 * Copyright (c) 2020 EdgeImpulse Inc.
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
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include <stdint.h>
#include <stdlib.h>

#include "Apds9250.h"
#include "ei_light.h"

/* Constant defines -------------------------------------------------------- */

/* Private variables ------------------------------------------------------- */
static Apds9250 g_apds9250;

static float light_data[LIGHT_AXIS_SAMPLED];

/**
 * @brief      Setup payload header
 *
 * @return     true
 */
bool ei_light_init(void)
{
    if (g_apds9250.begin() == true) {
        g_apds9250.setMode(modeColorSensor);
        g_apds9250.setResolution(res18bit);
        g_apds9250.setGain(gain1);
        g_apds9250.setMeasurementRate(rate100ms);

        if (ei_add_sensor_to_fusion_list(light_sensor) == false) {
            ei_printf("ERR: failed to register APDS-9250 light sensor!\r\n");
            return false;
        }
    }
    else {
        ei_printf("ERR: APDS-9250 light sensor not present!\r\n");
        return false;
    }

    return true;
}

/**
 * @brief 
 * 
 * @param n_samples 
 * @return float* 
 */
float *ei_fusion_light_read_data(int n_samples)
{
    uint32_t red = 0;
    uint32_t green = 0;
    uint32_t blue = 0;
    uint32_t ir = 0;

    g_apds9250.getAll(&red, &green, &blue, &ir);

    light_data[0] = red;
    light_data[1] = green;
    light_data[2] = blue;
    light_data[3] = ir;

    return light_data;
}
