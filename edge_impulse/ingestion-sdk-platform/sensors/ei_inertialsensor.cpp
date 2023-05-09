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
#include <stdint.h>
#include <stdlib.h>
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"

#include "ei_config_types.h"
#include "ei_inertialsensor.h"
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
bool ei_inertial_init(void)
{
    if (spresense_init_acc() == true) {
        if(ei_add_sensor_to_fusion_list(imu_sensor) == false) {
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
float *ei_fusion_inertial_read_data(int n_samples)
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
