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

#ifndef EI_INERTIAL_SENSOR
#define EI_INERTIAL_SENSOR

/* Include ----------------------------------------------------------------- */
#include "firmware-sdk/ei_fusion.h"

/** Number of axis used and sample data format */
typedef float sample_format_t;

#define N_AXIS_SAMPLED			3
#define SIZEOF_N_AXIS_SAMPLED	(sizeof(sample_format_t) * N_AXIS_SAMPLED)

/* Function prototypes ----------------------------------------------------- */
bool ei_inertial_init(void);
float *ei_fusion_inertial_read_data(int n_samples);

static const ei_device_fusion_sensor_t imu_sensor = {
    "Inertial",
    // number of sensor module axis
    N_AXIS_SAMPLED,
    // sampling frequencies
    { 62.5f, 100.0f , 250.0f, 500.0f},
    // axis name and units payload (must be same order as read in)    
    { {"accX", "m/s2"}, {"accY", "m/s2"}, {"accZ", "m/s2"} },
    // reference to read data function
    &ei_fusion_inertial_read_data,
    0
};

#endif