/* Edge Impulse inferencing library
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

// set to 1 to generate and send a test image
#define SEND_TEST_IMAGE 0

#if !SEND_TEST_IMAGE
#include "firmware-sdk/ei_camera_interface.h"
#include "firmware-sdk/ei_device_info_lib.h"
#endif

#if EI_PORTING_SONY_SPRESENSE
#include "malloc.h" //for memalign
#endif

#include <memory>

#include "edge-impulse-sdk/dsp/ei_utils.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "firmware-sdk/at_base64_lib.h"
#include "firmware-sdk/ei_device_interface.h"
#include "firmware-sdk/ei_image_lib.h"
#include "edge-impulse-sdk/dsp/EiProfiler.h"

// *********************************** AT cmd functions ***************

static void respond_and_change_to_max_baud()
{
    auto device = EiDeviceInfo::get_device();
    // sleep a little to let the daemon attach on the new baud rate...
    ei_printf("\r\nOK");
    ei_sleep(100);
    device->set_max_data_output_baudrate();
    ei_sleep(100);
}

static void change_to_normal_baud()
{
    auto device = EiDeviceInfo::get_device();
    // lower baud rate
    ei_printf("\r\nOK\r\n");
    ei_sleep(100);
    device->set_default_data_output_baudrate();
    // sleep a little to let the daemon attach on baud rate 115200 again...
    ei_sleep(100);
}

static bool ei_camera_take_snapshot_encode_and_output_no_init(size_t width, size_t height)
{
    // rgb888 packed, 3B color depth
    uint32_t size = width * height * 3;

#if EI_PORTING_SONY_SPRESENSE
    // 32 BYTE aligned (for Sony, maybe others too?  Monster vector moves in our future?)
    auto image_p = std::unique_ptr<uint8_t, decltype(free) *> { reinterpret_cast<uint8_t *>(
                                                                    memalign(32, size)),
                                                                free };
#else // more portable version
    std::unique_ptr<uint8_t[]> image_p(new uint8_t[size]);
#endif

    if (!image_p) {
        ei_printf("Take snapshot: Out of memory\n");
        return false;
    }

    auto image = image_p.get();

#if SEND_TEST_IMAGE
    uint32_t counter = 0;
    for (int i = 0; i < size; i += 3) {
        image[i] = counter & 0xff;
        image[i + 1] = counter >> 8;
        image[i + 2] = counter >> 16;
        counter += 100;
    }
#else
    auto camera = EiCamera::get_camera();
    bool isOK = camera->ei_camera_capture_rgb888_packed_big_endian(image, size, width, height);
    if (!isOK) {
        return false;
    }
#endif

    base64_encode(reinterpret_cast<char *>(image), size, ei_putc);

    return true;
}

extern bool
ei_camera_take_snapshot_output_on_serial(size_t width, size_t height, bool use_max_baudrate)
{
    auto camera = EiCamera::get_camera();

    if (!camera->init()) {
        ei_printf("Failed to init camera\n");
        return false;
    }

    if( use_max_baudrate ) {
        respond_and_change_to_max_baud();
    }

    bool isOK = ei_camera_take_snapshot_encode_and_output_no_init(width, height);
    camera->deinit();

    if( use_max_baudrate ) {
        change_to_normal_baud();
    }
    else {
        ei_printf("\r\nOK\r\n");
    }

    return isOK;
}

extern bool ei_camera_start_snapshot_stream(size_t width, size_t height, bool use_max_baudrate)
{
    bool isOK = true;
    ei_printf("Starting snapshot stream...\n");
    auto camera = EiCamera::get_camera();

    if (!camera->init()) {
        ei_printf("Failed to init camera\n");
        return false;
    }

    if( use_max_baudrate ) {
        respond_and_change_to_max_baud();
    }

    while (!ei_user_invoke_stop_lib()) {
        isOK &=
            ei_camera_take_snapshot_encode_and_output_no_init(width, height);
        ei_printf("\r\n");
    }
    camera->deinit();

    if( use_max_baudrate ) {
        change_to_normal_baud();
    }

    return isOK;
}

// ------------------------------------- Util functions --------------------------------

// Clamp out of range values
#define CLAMP(t) (((t) > 255) ? 255 : (((t) < 0) ? 0 : (t)))

// Color space conversion for RGB
#define GET_R_FROM_YUV(y, u, v) ((298 * y + 409 * v + 128) >> 8)
#define GET_G_FROM_YUV(y, u, v) ((298 * y - 100 * u - 208 * v + 128) >> 8)
#define GET_B_FROM_YUV(y, u, v) ((298 * y + 516 * u + 128) >> 8)

extern void YUV422toRGB888(
    unsigned char *rgb_out,
    unsigned const char *yuv_in,
    unsigned int in_size_B,
    YUV_OPTIONS opts)
{
    int in_size_pixels = in_size_B / 4;
    yuv_in += in_size_B - 1;

    int rgb_end = TEST_BIT_MASK(opts, PAD_4B) ? 2 * in_size_B : (6 * in_size_B) / 4;
    rgb_out += rgb_end - 1;

    // Going backwards probably looks strange, but
    // This allows us to do the algorithm in place!
    // User needs to put the YUV image into a larger buffer than necessary
    // But going backwards means we don't overwrite the YUV bytes
    //  until we don't need them anymore
    for (unsigned int i = 0; i < in_size_pixels; ++i) {
        int y2 = *yuv_in-- - 16;
        int v = *yuv_in-- - 128;
        int y0 = *yuv_in-- - 16;
        int u0 = *yuv_in-- - 128;

        if (TEST_BIT_MASK(opts, BIG_ENDIAN_ORDER)) {
            *rgb_out-- = CLAMP(GET_B_FROM_YUV(y2, u0, v));
            *rgb_out-- = CLAMP(GET_G_FROM_YUV(y2, u0, v));
            *rgb_out-- = CLAMP(GET_R_FROM_YUV(y2, u0, v));
            if (TEST_BIT_MASK(opts, PAD_4B)) {
                *rgb_out-- = 0;
            }

            *rgb_out-- = CLAMP(GET_B_FROM_YUV(y0, u0, v));
            *rgb_out-- = CLAMP(GET_G_FROM_YUV(y0, u0, v));
            *rgb_out-- = CLAMP(GET_R_FROM_YUV(y0, u0, v));
            if (TEST_BIT_MASK(opts, PAD_4B)) {
                *rgb_out-- = 0;
            }
        } // else TODO
    }
}
