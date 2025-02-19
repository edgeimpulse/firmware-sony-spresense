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