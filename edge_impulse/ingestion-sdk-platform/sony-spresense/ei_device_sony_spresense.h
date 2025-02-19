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

#ifndef EI_DEVICE_SONY_SPRESENSE
#define EI_DEVICE_SONY_SPRESENSE

#include "firmware-sdk/ei_device_info_lib.h"
#include "firmware-sdk/ei_device_memory.h"
#include "ei_camera_driver_sony.h"
#include "firmware-sdk/ei_fusion.h"

class EiDeviceSonySpresense : public EiDeviceInfo {
private:
    EiDeviceSonySpresense() = delete;
    static const int sensors_count = 4;
    ei_device_sensor_t sensors[sensors_count] = {0};
    EiState state;
    EiCameraSony* camera;
    bool is_sampling;
    float sample_interval_ms;
    
public:
    EiDeviceSonySpresense(EiDeviceMemory *mem);
    ~EiDeviceSonySpresense();
    void init_device_id(void);
    void clear_config(void);
    uint32_t get_data_output_baudrate(void) override;
    void set_max_data_output_baudrate(void) override;
    void set_default_data_output_baudrate(void) override;
    
    void set_state(EiState state) override;
    EiState get_state(void);

    bool is_camera_present(void);
    EiSnapshotProperties get_snapshot_list(void);

    bool get_sensor_list(const ei_device_sensor_t **sensor_list, size_t *sensor_list_size) override;

    bool start_sample_thread(void (*sample_read_cb)(void), float sample_interval_ms) override;
    bool stop_sample_thread(void) override;

    bool stop_stream(void);
    void (*sample_read_callback)(void);

    bool get_is_sampling(void) {
        return is_sampling;
    }

#if MULTI_FREQ_ENABLED == 1
	bool start_multi_sample_thread(void (*sample_multi_read_cb)(uint8_t), float* multi_sample_interval_ms, uint8_t num_fusioned) override;		
#endif
};

unsigned int local_sample_thread(void);
#if MULTI_FREQ_ENABLED == 1
unsigned int local_multi_sample_thread(void);
#endif

#endif /* EI_DEVICE_TEMPLATE_H_ */
