/* Edge Impulse ingestion SDK
 * Copyright (c) 2021 EdgeImpulse Inc.
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

/* Includes ---------------------------------------------------------------- */
#include <cstdlib>
#include <cstdio>
#include <string.h>
#include "stdlib_missing.h"
#include "ei_device_sony_spresense.h"
#include "ei_at_handlers.h"
#include "ei_run_impulse.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "ei_board.h"
#include "inference/ei_run_impulse.h"
#include "ei_camera_driver_sony.h"

#ifdef COMMONSENSE
#include "commonsense/ei_inertial.h"
#include "commonsense/ei_environmentsensor.h"
#include "commonsense/ei_pressure.h"
#include "commonsense/ei_light.h"
#include "commonsense/ei_magnetometer.h"
#include "commonsense/ei_proximity.h"
#include "commonsense/ei_gas_sensor.h"
#else
#include "ei_accelerometer.h"
#endif

#include "ei_adc.h"

/**
 * @brief Init sensors, load config and run command handler
 * 
 * @return int 
 */
int ei_main(void) 
{
    EiDeviceSonySpresense* dev = static_cast<EiDeviceSonySpresense*>(EiDeviceInfo::get_device());
    EiCameraSony* cam = static_cast<EiCameraSony*>(EiCamera::get_camera());
    ATServer *at;
    char data = 0xFF;

    ei_printf("Hello from Edge Impulse Device SDK.\n"
        "Compiled on %s %s\n", __DATE__, __TIME__);

    at = ei_at_init(dev);
    ei_printf("Type AT+HELP to see a list of commands.\n");    
    
#ifdef COMMONSENSE
    // Init of commonsense sensors
    ei_inertial_init();
    ei_environment_init();
    ei_pressure_init();
    ei_light_init();
    ei_magnetometer_init();
    ei_proximity_init();
    ei_gas_init();
#else
    // kx inertial
    ei_accelerometer_init();
#endif

    ei_adc_init();

    at->print_prompt();
    
    /* Intialize configuration */
    while (1) {
        data = spresense_getchar();

        if (data != 0) {
            if (is_inference_running() && data == 'b') {
                ei_stop_impulse();
                at->print_prompt();
                continue;
            }

            if(cam->is_ingestion_stream_active() && data =='b') {
                dev->stop_stream();
                at->print_prompt();
                continue;
            }

            at->handle(data);
        }

        if (cam->is_ingestion_stream_active()) {
            cam->run_stream();
        }

        if (is_inference_running() == true) {
            ei_run_impulse();
        }
    }
}
