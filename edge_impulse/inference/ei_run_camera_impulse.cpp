/* Edge Impulse ingestion SDK
 * Copyright (c) 2022 EdgeImpulse Inc.
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

#include "model-parameters/model_metadata.h"
#if defined(EI_CLASSIFIER_SENSOR) && (EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_CAMERA)
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "ei_device_sony_spresense.h"
#include "ei_camera_driver_sony.h"
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "firmware-sdk/jpeg/encode_as_jpg.h"
#include "ei_run_impulse.h"

#include "malloc.h" //for memalign
#include <cstdio>

#define DWORD_ALIGN_PTR(a)   ((a & 0x3) ?(((uintptr_t)a + 0x4) & ~(uintptr_t)0x3) : a)

typedef enum {
    INFERENCE_STOPPED,
    INFERENCE_WAITING,
    INFERENCE_SAMPLING,
    INFERENCE_DATA_READY
} inference_state_t;

static inference_state_t state = INFERENCE_STOPPED;
static uint64_t last_inference_ts = 0;
static bool debug_mode = false;
static bool continuous_mode = false;
static bool use_max_baud = false;
static uint8_t *snapshot_buf = nullptr;
static uint32_t snapshot_buf_size;
static ei_device_snapshot_resolutions_t snapshot_resolution;
static bool resize_required = false;
static bool crop_required = false;
static uint32_t inference_delay;

/**
 * @brief
 *
 * @param offset
 * @param length
 * @param out_ptr
 * @return int
 */
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr)
{
    // we already have a RGB888 buffer, so recalculate offset into pixel index
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix + 2];

        // go to the next pixel
        out_ptr_ix++;
        pixel_ix+=3;
        pixels_left--;
    }

    // and done!
    return 0;
}

/**
 * @brief
 *
 */
void ei_run_impulse(void)
{
    switch(state) {
        case INFERENCE_STOPPED:
            // nothing to do
            return;
        case INFERENCE_WAITING:
            if(ei_read_timer_ms() < (last_inference_ts + inference_delay)) {
                return;
            }
            state = INFERENCE_DATA_READY;
            break;
        case INFERENCE_SAMPLING:
        case INFERENCE_DATA_READY:
            if(continuous_mode == true) {
                state = INFERENCE_WAITING;
            }
            break;
        default:
            break;
    }

    // if we have to resize, then allocate bigger buffer
    // (resize means camera can't get big enough spanshot)
    if(resize_required) {
        snapshot_buf = (uint8_t*)memalign(32, EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT * 3);
    }
    else {
        snapshot_buf = (uint8_t*)memalign(32, snapshot_buf_size);
    }

    // check if allocation was succesful
    if(snapshot_buf == nullptr) {
        ei_printf("ERR: Failed to allocate snapshot buffer!\n");
        ei_printf("Trying to allocate %lu!\n", snapshot_buf_size);
        return;
    }

    EiCameraSony *camera = static_cast<EiCameraSony*>(EiCameraSony::get_camera());

    ei_printf("Taking photo...\n");

    bool isOK = camera->get_stream(snapshot_buf, snapshot_buf_size);

    if (!isOK) {
        return;
    }

    if (resize_required || crop_required) {
        ei::image::processing::crop_and_interpolate_rgb888(
            snapshot_buf,
            snapshot_resolution.width,
            snapshot_resolution.height,
            snapshot_buf,
            EI_CLASSIFIER_INPUT_WIDTH,
            EI_CLASSIFIER_INPUT_HEIGHT); // bytes per pixel
    }

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    // print and discard JPEG buffer before inference to free some memory
    if (debug_mode) {
        ei_printf("Begin output\n");
        ei_printf("Framebuffer: ");

        int ret = encode_rgb888_signal_as_jpg_and_output_base64(&signal, EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT);
        ei_printf("\r\n");

        if(ret != 0) {
            ei_printf("ERR: Failed to encode frame as JPEG (%d)\n", ret);
        }
    }

    // run the impulse: DSP, neural network and the Anomaly algorithm
    ei_impulse_result_t result = { 0 };

    EI_IMPULSE_ERROR ei_error = run_classifier(&signal, &result, false);
    if (ei_error != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run impulse (%d)\n", ei_error);
        ei_free(snapshot_buf);
        return;
    }
    ei_free(snapshot_buf);

    display_results(&ei_default_impulse, &result);

    if (debug_mode) {
        ei_printf("\r\n----------------------------------\r\n");
        ei_printf("End output\n");
    }

    if(continuous_mode == false) {
        last_inference_ts = ei_read_timer_ms();
        ei_printf("Starting inferencing in %ld seconds...\n", inference_delay / 1000);
    }
}

/**
 * @brief
 *
 * @param continuous
 * @param debug
 * @param use_max_uart_speed
 */
void ei_start_impulse(bool continuous, bool debug, bool use_max_uart_speed)
{
    EiDeviceSonySpresense* dev = static_cast<EiDeviceSonySpresense*>(EiDeviceInfo::get_device());
    EiCameraSony* cam = static_cast<EiCameraSony*>(EiCamera::get_camera());

    debug_mode = debug;
    continuous_mode = (debug) ? true : continuous;
    use_max_baud = use_max_uart_speed;

    if (cam->is_camera_present() == false) {
        ei_printf("ERR: Failed to start inference, camera is missing!\n");
        return;
    }

    if(EI_CLASSIFIER_INPUT_WIDTH < 96 || EI_CLASSIFIER_INPUT_HEIGHT < 64) {
        ei_printf("Image resolution too small should be at least 96x64\r\n");
        return;
    }

    snapshot_resolution = cam->search_resolution(EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT);
    if(cam->set_resolution(snapshot_resolution) == false) {
        ei_printf("ERR: Failed to set snapshot resolution (%ux%u)!\n", EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT);
        return;
    }

    if(snapshot_resolution.width > EI_CLASSIFIER_INPUT_WIDTH || snapshot_resolution.height > EI_CLASSIFIER_INPUT_HEIGHT) {
        crop_required = true;
        resize_required = false;
    }
    else if(snapshot_resolution.width < EI_CLASSIFIER_INPUT_WIDTH || snapshot_resolution.height < EI_CLASSIFIER_INPUT_HEIGHT) {
        crop_required = false;
        resize_required = true;
    }
    else {
        crop_required = false;
        resize_required = false;
    }

    snapshot_buf_size = snapshot_resolution.width * snapshot_resolution.height * 3;
    cam->init(snapshot_resolution.width, snapshot_resolution.height);

    if (cam->start_stream(snapshot_resolution.width, snapshot_resolution.height, e_inference_stream) == false) {
        ei_printf("Error in starting stream\n");
        return;
    }

    // summary of inferencing settings (from model_metadata.h)
    ei_printf("Inferencing settings:\n");
    ei_printf("\tImage resolution: %dx%d\n", EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT);
    ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    ei_printf("\tNo. of classes: %d\n", sizeof(ei_classifier_inferencing_categories) / sizeof(ei_classifier_inferencing_categories[0]));

    if(continuous_mode == true) {
        inference_delay = 0;
        state = INFERENCE_DATA_READY;
    }
    else {
        inference_delay = 2000;
        last_inference_ts = ei_read_timer_ms();
        state = INFERENCE_WAITING;
        ei_printf("Starting inferencing in %ld seconds...\n", inference_delay / 1000);
    }

    if (debug_mode || use_max_baud) {
        ei_printf("OK\r\n");
        ei_sleep(100);
        dev->set_max_data_output_baudrate();
        ei_sleep(100);
    }
}

/**
 * @brief
 *
 */
void ei_stop_impulse(void)
{
    EiDeviceSonySpresense* dev = static_cast<EiDeviceSonySpresense*>(EiDeviceInfo::get_device());
    EiCameraSony* cam = static_cast<EiCameraSony*>(EiCamera::get_camera());

    cam->stop_stream();

    if (debug_mode || use_max_baud) {
        ei_printf("\r\nOK\r\n");
        ei_sleep(100);
        dev->set_default_data_output_baudrate();
        ei_sleep(100);
    }

    ei_printf("Inferencing stopped by user\r\n");

    state = INFERENCE_STOPPED;
}

/**
 * @brief
 *
 * @return true
 * @return false
 */
bool is_inference_running(void)
{
    return (state != INFERENCE_STOPPED);
}

#endif /* defined(EI_CLASSIFIER_SENSOR) && EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_CAMERA */
