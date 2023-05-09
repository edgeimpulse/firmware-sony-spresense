/*
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
/* Includes ---------------------------------------------------------------- */
#include "ei_camera_driver_sony.h"
#include "firmware-sdk/ei_image_lib.h"
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"

#include <nuttx/video/video.h>

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Constants */
#define VIDEO_BUFNUM       (3)
#define STILL_BUFNUM       (1)

struct v_buffer
{
  uint8_t *start;
  uint32_t length;
};

typedef struct v_buffer v_buffer_t;

/* Private variables ------------------------------------------------------- */
static v_buffer *local_buffer_stream = NULL;

struct v_buffer *buffers_video = NULL;
struct v_buffer *buffers_still = NULL;

ei_device_snapshot_resolutions_t EiCameraSony::resolutions_isx012[] = {
        { .width = 96, .height = 64 },
        //{ .width = 96, .height = 96 },
        { .width = 160, .height = 120 },
        { .width = 320, .height = 240 },
#if defined(EI_CLASSIFIER_SENSOR) && EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_CAMERA
        { .width = EI_CLASSIFIER_INPUT_WIDTH, .height = EI_CLASSIFIER_INPUT_HEIGHT },
#endif
    };

ei_device_snapshot_resolutions_t EiCameraSony::resolutions_isx019[] = {
    { .width = 160, .height = 120 },
    { .width = 320, .height = 240 },
#if defined(EI_CLASSIFIER_SENSOR) && EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_CAMERA
    { .width = EI_CLASSIFIER_INPUT_WIDTH, .height = EI_CLASSIFIER_INPUT_HEIGHT },
#endif
};

/* Private functions ------------------------------------------------------- */
static int camera_prepare(int fd, enum v4l2_buf_type type, uint32_t buf_mode, uint32_t pixformat, uint16_t hsize, uint16_t vsize, FAR struct v_buffer **vbuf, uint8_t buffernum);
static bool get_camimage(int fd, struct v4l2_buffer *v4l2_buf, enum v4l2_buf_type buf_type);
static bool release_camimage(int fd, struct v4l2_buffer *v4l2_buf);
static const char *get_imgsensor_name(int fd);
static bool start_stillcapture(int v_fd, enum v4l2_buf_type capture_type);
static bool stop_stillcapture(int v_fd, enum v4l2_buf_type capture_type);
static void free_video_buffers(void);
static void local_base64_encode(const char *input, size_t input_size);

/* Public functions ------------------------------------------------------- */
void EiCameraSony::get_resolutions(ei_device_snapshot_resolutions_t **res, uint8_t *res_num)
{
    if (connected_camera == e_ISX012) {
        *res = &EiCameraSony::resolutions_isx012[0];
        *res_num = sizeof(EiCameraSony::resolutions_isx012) / sizeof(ei_device_snapshot_resolutions_t);
    }
    else if (connected_camera == e_ISX019) {
        *res = &EiCameraSony::resolutions_isx019[0];
        *res_num = sizeof(EiCameraSony::resolutions_isx019) / sizeof(ei_device_snapshot_resolutions_t);
    }
    else {

    }
}

/**
 * @brief Get the min resolution object
 * 
 * @return ei_device_snapshot_resolutions_t 
 */
ei_device_snapshot_resolutions_t EiCameraSony::get_min_resolution(void)
{
    lazy_init();
    if (connected_camera == e_ISX012) {
        return EiCameraSony::resolutions_isx012[0];
    }
    else if (connected_camera == e_ISX019) {
        return EiCameraSony::resolutions_isx019[0];
    }
    else {
        // not sure
        return EiCameraSony::resolutions_isx012[0];
    }

}

/**
 * @brief Construct a new Ei Camera Sony:: Ei Camera Sony object
 * 
 */
EiCameraSony::EiCameraSony()
{    
    connected_camera = e_max_camera;
}

/**
 * @brief 
 * 
 * @return EiCamera* 
 */
EiCamera *EiCamera::get_camera(void)
{
    static EiCameraSony camera;

    return &camera;
}

/**
 * @brief 
 * 
 * @param width 
 * @param height 
 * @return true 
 * @return false 
 */
bool EiCameraSony::init(uint16_t width, uint16_t height)
{
    #if 0
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
    #endif

    lazy_init();

    if (connected_camera != e_no_camera) {
        this->output_height = height;
        this->output_width = width;
    }
    else {
        return false;
    }
    
    
    return true;
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool EiCameraSony::deinit(void)
{
    #if 0
    //close(v_fd);
    //video_uninitialize();
    //v_fd = NULL; //we check v_fd before using, so it's cool to be square, err, NULL!
    #endif
    return true;
}

/**
 * @brief 
 * 
 * @param image 
 * @param image_size 
 * @return true 
 * @return false 
 */
bool EiCameraSony::ei_camera_capture_rgb888_packed_big_endian(uint8_t *image, uint32_t image_size)
{
    uint16_t height = this->output_height;
    uint16_t width = this->output_width;    

    // check buffer size.  RGB packed, 3B per pixel
    if (image_size < height * width * 3) {
        ei_printf("Buffer for image capture too small\n");
        return false;
    }
    // 2B per pixel in YUV land
    uint32_t yuv_buf_size = height * width * 2;
    bool isOK = captureYUV422(image, yuv_buf_size, width, height);

    if (!isOK) {
        ei_printf("Could not capture image\n");
        return false;
    }

    using namespace ei::image::processing;

    // In place conversion!
    if (yuv422_to_rgb888(image, image, yuv_buf_size, BIG_ENDIAN_ORDER) != ei::EIDSP_OK) {
        ei_printf("Error in yuv422_to_rgb888\n");
        return false;
    }

    return true;
}

/**
 * @brief 
 * 
 * @param image 
 * @param image_size_B 
 * @param width 
 * @param height 
 * @return true 
 * @return false 
 */
bool EiCameraSony::captureYUV422(uint8_t *image, uint32_t image_size_B, uint16_t width, uint16_t height)
{
    int ret;
    struct v4l2_buffer v4l2_buf;
    bool isOK;
    v4l2_buf_type capture_type = V4L2_BUF_TYPE_STILL_CAPTURE;

    if (!v_fd) {
        ei_printf("ERR: Init camera before capturing\n");
        return false;
    }

    uint32_t buffer_size = (width * height * 3);

    buffers_still = (v_buffer_t*)ei_malloc(sizeof(v_buffer_t));

    buffers_still->start = image;
    buffers_still->length = buffer_size;
    
    ret = camera_prepare
    (v_fd, capture_type,
     V4L2_BUF_MODE_FIFO,
     V4L2_PIX_FMT_UYVY,
     width, height,
     &buffers_still,
     STILL_BUFNUM);

    if (ret != 0) {
        ei_printf("camera_prepare NOK\n");
        ei_free(buffers_still);
        return false;
    }

    // Start still capture.  This is only used for V4L2_BUF_TYPE_STILL_CAPTURE
    isOK = start_stillcapture(v_fd, capture_type);
    if (!isOK) {
         ei_printf("start_stillcapture NOK\n");
         ei_free(buffers_still);
        return false;
    }

    // This writes the buffer assigned in camera_prepare()
    isOK = get_camimage(v_fd, &v4l2_buf, V4L2_BUF_TYPE_STILL_CAPTURE);
    if (!isOK) {
        ei_printf("get_camimage NOK\n");
        ei_free(buffers_still);
        return false;
    }

    isOK = release_camimage(v_fd, &v4l2_buf);
    if (!isOK) {
        ei_printf("release_camimage NOK\n");
        ei_free(buffers_still);
        return false;
    }

    // Stop still capture
    isOK = stop_stillcapture(v_fd, capture_type);
    if (!isOK) {
        ei_printf("Failed to stop taking picture\n");
        ei_free(buffers_still);
        return false;
    }

    ei_free(buffers_still);

    return true;
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool EiCameraSony::is_ingestion_stream_active(void)
{
    return (stream_type == e_ingestion_stream);
}

/**
 * @brief 
 * 
 * @param width_to_set 
 * @param height_to_set 
 * @return true 
 * @return false 
 */
bool EiCameraSony::start_stream(uint32_t width_to_set, uint32_t height_to_set, t_stream_type requested_stream)
{
    uint8_t i;
    int ret;
        
    buffers_video = (v_buffer_t*)ei_malloc(sizeof(v_buffer_t) * VIDEO_BUFNUM);
    this->stream_buffer_size = (width_to_set * height_to_set * 2);  // for capture

    v_buffer_t* p_buffer = buffers_video;
    for (i = 0; i < VIDEO_BUFNUM; i++) {
        p_buffer->length = this->stream_buffer_size;
        p_buffer->start = (uint8_t*)memalign(32, this->stream_buffer_size);
        p_buffer++;
    }

    if (init(width_to_set, height_to_set) == false) {
        ei_printf("Error in start_stream\n");
        free_video_buffers();
        return false;
    }
    
    ret = camera_prepare
    (v_fd, V4L2_BUF_TYPE_VIDEO_CAPTURE,
     V4L2_BUF_MODE_RING,
     V4L2_PIX_FMT_UYVY,
     width_to_set, height_to_set,
     &buffers_video,
     VIDEO_BUFNUM);

    if (ret != 0) {
        ei_printf("camera_prepare NOK %d\n", ret);
        free_video_buffers();
        return false;
    }

    if (requested_stream == e_ingestion_stream) {
        local_buffer_stream = (v_buffer_t*)ei_malloc(sizeof(v_buffer_t));
        local_buffer_stream->length = width_to_set * height_to_set * 3;
        local_buffer_stream->start = (uint8_t*)memalign(32, local_buffer_stream->length);
    }
    
    stream_type = requested_stream;

    return true;
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool EiCameraSony::stop_stream(void)
{
    bool isOk = true;
    int ret;

    uint32_t type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (stream_type == false) {
        ei_printf("Nothing to do!\n");
        return false;
    }
        
    ret = ioctl(v_fd, VIDIOC_STREAMOFF, (uintptr_t)&type);
    if (ret < 0) {
        ei_printf("Failed to stop taking picture\n");
        isOk = false;
    }
    ioctl(v_fd, VIDIOC_CANCEL_DQBUF, (uintptr_t)&type);

    // close
    deinit();
    free_video_buffers();

    if (stream_type == e_ingestion_stream) {
        ei_free(local_buffer_stream->start);
        ei_free(local_buffer_stream);
        
        local_buffer_stream = NULL;
        local_buffer_stream->start = NULL;
    }
    
    stream_type = e_no_stream;
    
    return isOk;
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool EiCameraSony::run_stream(void)
{    
    bool isOK;
    struct v4l2_buffer v_buf;

    if (stream_type == e_no_stream) {
        return false;
    }    
    
    if (local_buffer_stream == nullptr) {
        ei_printf("Error in allocating buffer for stream\n");
        free_video_buffers();
        return false;
    }

    isOK = get_camimage(v_fd, &v_buf, V4L2_BUF_TYPE_VIDEO_CAPTURE);
    if (!isOK) {
        ei_printf("get_camimage error\n");
        free_video_buffers();
        return false;
    }

    isOK = release_camimage(v_fd, &v_buf);
    if (!isOK) {
        ei_printf("release_camimage error\n");
        free_video_buffers();
        return false;
    }
    using namespace ei::image::processing;

    // In place conversion!
    if (yuv422_to_rgb888(local_buffer_stream->start, (unsigned char*)v_buf.m.userptr, (this->stream_buffer_size), BIG_ENDIAN_ORDER) == ei::EIDSP_OK) {
        local_base64_encode((char*)local_buffer_stream->start, local_buffer_stream->length);
    }
    else {
        ei_printf("yuv422_to_rgb888 error\n");
    }
    ei_printf("\r\n");

    return true;
}

/**
 * @brief Returns the stream in rgb888 format
 * 
 * @param image 
 * @param image_size 
 * @return true 
 * @return false 
 */
bool EiCameraSony::get_stream(uint8_t *image, uint32_t image_size)
{
    bool isOK;
    struct v4l2_buffer v_buf;

    if (stream_type == e_no_stream) {
        return false;
    }    
    
    if (image == nullptr) {
        ei_printf("Error in passed buffer for stream\n");        
        return false;
    }

    isOK = get_camimage(v_fd, &v_buf, V4L2_BUF_TYPE_VIDEO_CAPTURE);
    if (!isOK) {
        ei_printf("get_camimage error\n");
        free_video_buffers();
        return false;
    }

    isOK = release_camimage(v_fd, &v_buf);
    if (!isOK) {
        ei_printf("release_camimage error\n");
        free_video_buffers();
        return false;
    }
    using namespace ei::image::processing;

    // In place conversion!
    if (yuv422_to_rgb888(image, (unsigned char*)v_buf.m.userptr, (this->stream_buffer_size), BIG_ENDIAN_ORDER) != ei::EIDSP_OK) {        
        ei_printf("yuv422_to_rgb888 error\n");
        return false;
    }
    
    return true;
}

/**
 * @brief 
 * 
 */
void EiCameraSony::lazy_init(void)
{
    const char *sensor;

    if (connected_camera == e_max_camera) {
        //lazy init the handle
        int ret = video_initialize("/dev/video");
        if (ret != 0) {
            connected_camera = e_no_camera;
            ei_printf("video_initialize\n");
        }

        v_fd = open("/dev/video", 0);
        if (v_fd < 0) {
            v_fd = NULL;
            connected_camera = e_no_camera;
            ei_printf("open\n");
        }

        if (connected_camera == e_max_camera) {
            sensor = get_imgsensor_name(v_fd);
            if (strncmp(sensor, "ISX012", strlen("ISX012")) == 0) {
                connected_camera = e_ISX012;
            }
            else if (strncmp(sensor, "ISX019", strlen("ISX019")) == 0) {
                connected_camera = e_ISX019;
            }
            else {
                connected_camera = e_no_camera;

                close(v_fd);
                video_uninitialize();
                v_fd = NULL; //we check v_fd before using, so it's cool to be square, err, NULL!
            }
        }
    }
}
/* Private functions ------------------------------------------------------- */
/**
 * @brief 
 * 
 * @param fd 
 * @param type 
 * @param buf_mode 
 * @param pixformat 
 * @param hsize 
 * @param vsize 
 * @param vbuf 
 * @param buffernum 
 * @return int 
 */
static int camera_prepare(int fd, enum v4l2_buf_type type,
                          uint32_t buf_mode, uint32_t pixformat,
                          uint16_t hsize, uint16_t vsize,
                          FAR struct v_buffer **vbuf,
                          uint8_t buffernum)
{
    int ret;
    int cnt;
    struct v4l2_format fmt = { 0 };
    struct v4l2_requestbuffers req = { 0 };
    struct v4l2_buffer buf = { 0 };

    /* VIDIOC_S_FMT set format */
    fmt.type                = type;
    fmt.fmt.pix.width       = hsize;
    fmt.fmt.pix.height      = vsize;
    fmt.fmt.pix.field       = V4L2_FIELD_ANY;
    fmt.fmt.pix.pixelformat = pixformat;

    ret = ioctl(fd, VIDIOC_S_FMT, (uintptr_t)&fmt);
    if (ret < 0) {
        ei_printf("Failed to VIDIOC_S_FMT: errno = %d\n", errno);      
        return ret;
    }
    
    if (V4L2_BUF_TYPE_VIDEO_CAPTURE == type) {
        // set framerate
        struct v4l2_streamparm frame_param;
        memset(&frame_param, 0, sizeof(frame_param));
        frame_param.type = type;
        frame_param.parm.capture.timeperframe.denominator = 30;    /// 30 fps ?
        frame_param.parm.capture.timeperframe.numerator = 1;
        ret = ioctl(fd, VIDIOC_S_PARM, (uintptr_t)&frame_param);
        if (ret < 0) {
            ret = -errno;
            ei_printf("Failed to set framerate: %d\n", ret);
            return ret;
        }
    }

    /* VIDIOC_REQBUFS initiate user pointer I/O */
    req.type   = type;
    req.memory = V4L2_MEMORY_USERPTR;
    req.count  = buffernum;
    req.mode   = buf_mode;

    ret = ioctl(fd, VIDIOC_REQBUFS, (uintptr_t)&req);
    if (ret < 0) {
        ei_printf("Failed to VIDIOC_REQBUFS: errno = %d\n", errno);
        return ret; 
    }

    /* VIDIOC_QBUF enqueue buffer */
    for (cnt = 0; cnt < buffernum; cnt++) {
        memset(&buf, 0, sizeof(v4l2_buffer_t));
        buf.type = type;
        buf.memory = V4L2_MEMORY_USERPTR;
        buf.index = cnt;
        buf.m.userptr = (uintptr_t)(*vbuf)[cnt].start;
        buf.length = (*vbuf)[cnt].length;

        ret = ioctl(fd, VIDIOC_QBUF, (uintptr_t)&buf);
        if (ret) {
            ei_printf("Failed to VIDIOC_QBUF: errno = %d\n", errno);
            return ERROR;
        }
    }

    /* VIDIOC_STREAMON start stream */
    ret = ioctl(fd, VIDIOC_STREAMON, (uintptr_t)&type);
    if (ret < 0) {
        ei_printf("Failed to VIDIOC_STREAMON: errno = %d\n", errno);
        return ret;
    }

    return 0;
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
        ei_printf("Fail DQBUF %d\n", errno);
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
        ei_printf("Fail QBUF %d\n", errno);
        return false;
    }
    return true;
}

/**
 * @brief Get the imgsensor name object
 * 
 * @param fd 
 * @return const char* 
 */
static const char *get_imgsensor_name(int fd)
{
  static struct v4l2_capability cap;

  ioctl(fd, VIDIOC_QUERYCAP, (uintptr_t)&cap);

  return (FAR const char *)cap.driver;
}

/**
 * @brief 
 * 
 * @param v_fd 
 * @param capture_type 
 * @return int 
 */
static bool start_stillcapture(int v_fd, enum v4l2_buf_type capture_type)
{
  int ret;

  if (capture_type == V4L2_BUF_TYPE_STILL_CAPTURE)
    {
      ret = ioctl(v_fd, VIDIOC_TAKEPICT_START, 0);
      if (ret < 0)
        {
          ei_printf("Failed to start taking picture\n");
          return false;
        }
    }

  return true;
}

/**
 * @brief 
 * 
 * @param v_fd 
 * @param capture_type 
 * @return int 
 */
static bool stop_stillcapture(int v_fd, enum v4l2_buf_type capture_type)
{
  int ret;

  if (capture_type == V4L2_BUF_TYPE_STILL_CAPTURE)
    {
      ret = ioctl(v_fd, VIDIOC_TAKEPICT_STOP, false);
      if (ret < 0)
        {
          ei_printf("Failed to stop taking picture\n");
          return false;
        }
    }

  return true;
}

/**
 * @brief 
 * 
 */
static void free_video_buffers(void)
{
    uint8_t i;

    v_buffer_t* p_buffer = buffers_video;

    for (i = 0; i < VIDEO_BUFNUM; i++) {
        ei_free(p_buffer->start);
        p_buffer++;
    }

    ei_free(buffers_video);
}

static const char *base64_chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                                  "abcdefghijklmnopqrstuvwxyz"
                                  "0123456789+/";

/**
 * @brief 
 * 
 * @param input 
 * @param input_size 
 * @param output 
 * @param output_size 
 */
static void local_base64_encode(const char *input, size_t input_size)
{
    int i = 0;
    int j = 0;
    unsigned char char_array_3[3];
    unsigned char char_array_4[4];

    while (input_size--) {
        char_array_3[i++] = *(input++);
        if (i == 3) {
            char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
            char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
            char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
            char_array_4[3] = char_array_3[2] & 0x3f;

            for (i = 0; (i < 4); i++) {
                ei_putchar(base64_chars[char_array_4[i]]);
            }
            i = 0;
        }
    }

    if (i) {
        for (j = i; j < 3; j++) {
            char_array_3[j] = '\0';
        }

        char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
        char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
        char_array_4[3] = char_array_3[2] & 0x3f;

        for (j = 0; (j < i + 1); j++) {
            ei_putchar(base64_chars[char_array_4[j]]);
        }

        while ((i++ < 3)) {
            ei_putchar('=');
        }
    }
}
