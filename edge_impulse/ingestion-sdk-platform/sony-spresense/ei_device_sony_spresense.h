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

#ifndef EI_DEVICE_SONY_SPRESENSE
#define EI_DEVICE_SONY_SPRESENSE

#include "firmware-sdk/ei_device_info_lib.h"
#include "firmware-sdk/ei_device_memory.h"
#include "ei_camera_driver_sony.h"
#include "firmware-sdk/ei_fusion.h"

class EiDeviceSonySpresense : public EiDeviceInfo {
private:
    EiDeviceSonySpresense() = delete;
    static const int sensors_count = 1;
    ei_device_sensor_t sensors[sensors_count];
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
};

unsigned int local_sample_thread(void);

#endif /* EI_DEVICE_TEMPLATE_H_ */
