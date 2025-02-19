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
