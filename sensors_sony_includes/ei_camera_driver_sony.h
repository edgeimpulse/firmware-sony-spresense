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
#ifndef EI_CAMERA_DRIVER_SONY_H
#define EI_CAMERA_DRIVER_SONY_H


#include "firmware-sdk/ei_camera_interface.h"
#include <stdint.h>
#include <stddef.h>

typedef enum
{
    e_no_camera,
    e_ISX012,
    e_ISX019,
    e_max_camera
}t_sony_connected_camera;

typedef enum
{
    e_no_stream,
    e_ingestion_stream,
    e_inference_stream
}t_stream_type;

class EiCameraSony : public EiCamera {
private:
    uint16_t output_width;
    uint16_t output_height;
    static ei_device_snapshot_resolutions_t resolutions_isx012[];
    static ei_device_snapshot_resolutions_t resolutions_isx019[];
    t_sony_connected_camera connected_camera;

    bool stream_do_resize;
    uint8_t *stream_buffer;
    uint32_t stream_buffer_size;
    t_stream_type stream_type;

    // nuttx camera handle
    int v_fd = NULL;
    void lazy_init(void);
public:
    EiCameraSony();
    
    ei_device_snapshot_resolutions_t get_min_resolution(void);

    void get_resolutions(ei_device_snapshot_resolutions_t **res, uint8_t *res_num);

    // see README, need to close and re open for certain operations
    bool init(uint16_t width, uint16_t height) override;

    bool deinit(void) override;

    bool set_resolution(const ei_device_snapshot_resolutions_t res)
    {
        // Nothing to do...?
        return true;
    }

    bool ei_camera_capture_rgb888_packed_big_endian(
        uint8_t *image,
        uint32_t image_size) override; // using raw types b/c of stdint.h int32 mismatch!


    // Note, image must be aligned on a 32 BYTE boundary!!
    bool captureYUV422(uint8_t *image, uint32_t image_size_B, uint16_t width, uint16_t height);

    /**
     * @brief 
     * 
     * @return true 
     * @return false 
     */
    bool is_camera_present(void)
    {
        lazy_init();
        return ((connected_camera == e_ISX012) || (connected_camera == e_ISX019));
    }

    bool start_stream(uint32_t width_to_set, uint32_t height_to_set, t_stream_type requested_stream);
    bool stop_stream(void);
    bool get_stream(uint8_t *image, uint32_t image_size);
    bool run_stream(void);
    bool is_ingestion_stream_active(void);

};

#endif