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

#include <vector>

#include "firmware-sdk/ei_camera_interface.h"
#include "firmware-sdk/ei_device_interface.h"
#include "firmware-sdk/ei_image_lib.h"
#include "edge-impulse-sdk/dsp/image/image.hpp"

#include <errno.h>
#include <fcntl.h>
#include <nuttx/video/video.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define IMAGE_YUV_SIZE (320 * 240 * 2) /* QVGA YUV422 */

// image must be 32b word aligned
static bool
camera_prepare(int fd, uint8_t *image, uint32_t image_size_B, uint16_t hsize, uint16_t vsize)
{
    int ret;
    struct v4l2_format fmt = { 0 };
    struct v4l2_requestbuffers req = { 0 };
    struct v4l2_buffer buf = { 0 };
    uint32_t TYPE = V4L2_BUF_TYPE_STILL_CAPTURE;

    /* VIDIOC_REQBUFS initiate user pointer I/O */

    req.type = TYPE;
    req.memory = V4L2_MEMORY_USERPTR;
    req.count = 1;
    req.mode = V4L2_BUF_MODE_FIFO;

    ret = ioctl(fd, VIDIOC_REQBUFS, (unsigned long)&req);
    if (ret < 0) {
        ei_printf("Failed to VIDIOC_REQBUFS: errno = %d\n", errno);
        return false;
    }

    /* VIDIOC_S_FMT set format */

    fmt.type = TYPE;
    fmt.fmt.pix.width = hsize;
    fmt.fmt.pix.height = vsize;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;

    ret = ioctl(fd, VIDIOC_S_FMT, (unsigned long)&fmt);
    if (ret < 0) {
        ei_printf("Failed to VIDIOC_S_FMT: errno = %d\n", errno);
        return ret;
    }

    /* VIDIOC_QBUF enqueue buffer */

    memset(&buf, 0, sizeof(v4l2_buffer_t));
    buf.type = TYPE;
    buf.memory = V4L2_MEMORY_USERPTR;
    buf.index = 0;
    buf.m.userptr = (unsigned long)image;
    buf.length = image_size_B;

    ret = ioctl(fd, VIDIOC_QBUF, (unsigned long)&buf);
    if (ret) {
        ei_printf("Fail QBUF %d\n", errno);
        return false;
    }

    /* VIDIOC_STREAMON start stream */

    // I think the final arg is actually ignored here, but just copying the example
    ret = ioctl(fd, VIDIOC_STREAMON, (unsigned long)&TYPE);
    if (ret < 0) {
        ei_printf("Failed to VIDIOC_STREAMON: errno = %d\n", errno);
        return false;
    }

    return true;
}

/****************************************************************************
 * Name: get_camimage()
 *
 * Description:
 *   DQBUF camera frame buffer from video driver with taken picture data.
 ****************************************************************************/

static bool get_camimage(int fd, struct v4l2_buffer *v4l2_buf, enum v4l2_buf_type buf_type)
{
    int ret;

    /* VIDIOC_DQBUF acquires captured data. */

    memset(v4l2_buf, 0, sizeof(v4l2_buffer_t));
    v4l2_buf->type = buf_type;
    v4l2_buf->memory = V4L2_MEMORY_USERPTR;

    ret = ioctl(fd, VIDIOC_DQBUF, (unsigned long)v4l2_buf);
    if (ret) {
        printf("Fail DQBUF %d\n", errno);
        return false;
    }

    return true;
}

/****************************************************************************
 * Name: release_camimage()
 *
 * Description:
 *   Re-QBUF to set used frame buffer into video driver.
 ****************************************************************************/

static bool release_camimage(int fd, struct v4l2_buffer *v4l2_buf)
{
    int ret;

    /* VIDIOC_QBUF sets buffer pointer into video driver again. */

    ret = ioctl(fd, VIDIOC_QBUF, (unsigned long)v4l2_buf);
    if (ret) {
        printf("Fail QBUF %d\n", errno);
        return false;
    }
    return true;
}

class EiCameraSony : public EiCamera {
public:
    uint16_t get_min_width() override { return 96; }
    uint16_t get_min_height() override { return 64; }
    // see README, need to close and re open for certain operations
    bool init() override
    {

        //lazy init the handle
        int ret = video_initialize("/dev/video");
        if (ret != 0) {
            ei_printf("ERROR: Failed to initialize video: errno = %d\n", errno);
            return false;
        }

        v_fd = open("/dev/video", 0);
        if (v_fd < 0) {
            ei_printf("ERROR: Failed to open video.errno = %d\n", errno);
            v_fd = NULL;
            return false;
        }
        return true;
    }

    bool deinit() override
    {
        close(v_fd);
        video_uninitialize();
        v_fd = NULL; //we check v_fd before using, so it's cool to be square, err, NULL!
        return true;
    }

    bool ei_camera_capture_rgb888_packed_big_endian(
        uint8_t *image,
        unsigned int image_size_B, // using raw types b/c of stdint.h int32 mismatch!
        uint16_t hsize,
        uint16_t vsize) override
    {
        // check buffer size.  RGB packed, 3B per pixel
        if (image_size_B < hsize * vsize * 3) {
            ei_printf("Buffer for image capture too small\n");
            return false;
        }
        // 2B per pixel in YUV land
        uint32_t yuv_buf_size = hsize * vsize * 2;
        bool isOK = captureYUV422(image, yuv_buf_size, hsize, vsize);

        if (!isOK) {
            ei_printf("Could not capture image\n");
            return false;
        }

        using namespace ei::image::processing;

        // In place conversion!
        yuv422_to_rgb888(image, image, yuv_buf_size, BIG_ENDIAN_ORDER);

        return true;
    }

    // Note, image must be aligned on a 32 BYTE boundary!!
    bool captureYUV422(uint8_t *image, uint32_t image_size_B, uint16_t hsize, uint16_t vsize)
    {
        if (!v_fd) {
            ei_printf("ERR: Init camera before capturing\n");
            return false;
        }

        int ret;
        struct v4l2_buffer v4l2_buf;
        bool isOK;

        isOK = camera_prepare(v_fd, image, image_size_B, hsize, vsize);
        if (!isOK) {
            return false;
        }

        // Start still capture.  This is only used for V4L2_BUF_TYPE_STILL_CAPTURE
        ret = ioctl(v_fd, VIDIOC_TAKEPICT_START, 0);
        if (ret < 0) {
            ei_printf("Failed to start taking picture\n");
            return false;
        }

        // This writes the buffer assigned in camera_prepare()
        isOK = get_camimage(v_fd, &v4l2_buf, V4L2_BUF_TYPE_STILL_CAPTURE);
        if (!isOK) {
            return false;
        }

        isOK = release_camimage(v_fd, &v4l2_buf);
        if (!isOK) {
            return false;
        }

        // Stop still capture
        ret = ioctl(v_fd, VIDIOC_TAKEPICT_STOP, false);
        if (ret < 0) {
            ei_printf("Failed to stop taking picture\n");
            return false;
        }

        return true;
    }
    // nuttx camera handle
    int v_fd = NULL;
};

EiCamera *EiCamera::get_camera()
{
    static EiCameraSony camera;
    return &camera;
}