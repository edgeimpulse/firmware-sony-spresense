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
