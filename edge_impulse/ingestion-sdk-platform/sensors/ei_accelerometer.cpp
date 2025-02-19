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
#include <stdint.h>
#include <stdlib.h>
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"

#include "ei_config_types.h"
#include "ei_accelerometer.h"
#include "ei_device_sony_spresense.h"
#include "ei_board_ctrl.h"

/* Constant defines -------------------------------------------------------- */
#define CONVERT_G_TO_MS2    9.80665f

/* Private variables ------------------------------------------------------- */
static float imu_data[N_AXIS_SAMPLED];

/**
 * @brief      Setup payload header
 *
 * @return     true
 */
bool ei_accelerometer_init(void)
{
    if (spresense_init_acc() == true) {
        if(ei_add_sensor_to_fusion_list(accelerometer_sensor) == false) {
            ei_printf("ERR: failed to register Inertial KX126 sensor!\r\n");
            return false;
        }
    }
    else {
        ei_printf("Accelerometer (KX126) missing or not working correctly\r\n");
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
float *ei_fusion_accelerometer_read_data(int n_samples)
{    
    memset(imu_data, 0, sizeof(imu_data));

    if (n_samples >= 3){    /* acc */
        spresense_getAcc(imu_data);

        for (int i = 0; i < 3; i++) {            
            imu_data[i] *= CONVERT_G_TO_MS2;
        }
    }

#if 0
    if (n_samples >= 6){    /* gyr */

    }

    if (n_samples >= 9){    /* mag */

    }
#endif

    return imu_data;
}
