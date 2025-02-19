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
#include <string>

#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "firmware-sdk/ei_device_info_lib.h"
#include "firmware-sdk/ei_device_memory.h"

#include "ei_device_sony_spresense.h"
#include "ei_flash_memory.h"
#include "ei_board.h"
#include "ei_microphone.h"
#include "ei_timer.h"

#define DEFAULT_BAUD    (115200u)
#define MAX_BAUD_RATE   (921600u)


/** Sensors */
typedef enum
{
    MICROPHONE = 0,
    MICROPHONE_2CH,
    //MICROPHONE_3CH,
    MICROPHONE_4CH,
    MAX_USED_SENSOR
} used_sensors_t;

/******
 *
 * @brief EdgeImpulse Device structure and information
 *
 ******/

EiDeviceSonySpresense::EiDeviceSonySpresense(EiDeviceMemory* mem)
{
    EiDeviceInfo::memory = mem;

    init_device_id();
    load_config();

    device_type = "SONY_SPRESENSE"; /* TODO-MODIFY update the device name */

        /* Calculate number of bytes available on flash for sampling, reserve 1 block for header + overhead */
    uint32_t available_bytes = ((mem->get_available_sample_blocks() - 1) * mem->block_size);

    sensors[MICROPHONE].name = "Microphone";
    sensors[MICROPHONE].start_sampling_cb = &ei_microphone_1ch_start;
    sensors[MICROPHONE].max_sample_length_s = available_bytes / (48000 * 2);
    sensors[MICROPHONE].frequencies[0] = 16000.0f;
    sensors[MICROPHONE].frequencies[1] = 48000.0f;

    sensors[MICROPHONE_2CH].name = "Microphone 2 channels";
    sensors[MICROPHONE_2CH].start_sampling_cb = &ei_microphone_2ch_start;
    sensors[MICROPHONE_2CH].max_sample_length_s = available_bytes / (48000 * 4);
    sensors[MICROPHONE_2CH].frequencies[0] = 16000.0f;
    sensors[MICROPHONE_2CH].frequencies[1] = 48000.0f;

    // do not expose for now
    //sensors[MICROPHONE_3CH].name = "Microphone 3 channels";
    //sensors[MICROPHONE_3CH].start_sampling_cb = &ei_microphone_3ch_start;
    //sensors[MICROPHONE_3CH].max_sample_length_s = available_bytes / (16000 * 6);
    //sensors[MICROPHONE_3CH].frequencies[0] = 16000.0f;

    sensors[MICROPHONE_4CH].name = "Microphone 4 channels";
    sensors[MICROPHONE_4CH].start_sampling_cb = &ei_microphone_4ch_start;
    sensors[MICROPHONE_4CH].max_sample_length_s = available_bytes / (48000 * 8);
    sensors[MICROPHONE_4CH].frequencies[0] = 16000.0f;
    sensors[MICROPHONE_4CH].frequencies[1] = 48000.0f;

    /* Init camera instance */
    camera = static_cast<EiCameraSony*>(EiCamera::get_camera());
}

EiDeviceSonySpresense::~EiDeviceSonySpresense()
{

}

EiDeviceInfo* EiDeviceInfo::get_device(void)
{
    /* Initializing EdgeImpulse classes here in order for
     * Flash memory to be initialized before mainloop start
     */
    static EiFlashMemory memory(sizeof(EiConfig));
    static EiDeviceSonySpresense dev(&memory);

    return &dev;
}

/**
 * @brief 
 * 
 */
void EiDeviceSonySpresense::init_device_id(void)
{
    char temp[32];

    spresense_get_chip_id(temp);

    device_id = std::string(temp);
}

/**
 * @brief 
 * 
 */
void EiDeviceSonySpresense::clear_config(void)
{
    EiDeviceInfo::clear_config();

    init_device_id();
    save_config();
}

/**
 * @brief 
 * 
 * @return uint32_t 
 */
uint32_t EiDeviceSonySpresense::get_data_output_baudrate(void)
{
    return MAX_BAUD_RATE;
}

/**
 * @brief      Set output baudrate to max
 *
 */
void EiDeviceSonySpresense::set_max_data_output_baudrate(void)
{
    spresense_uart_flush();
    spresense_set_uart(MAX_BAUD_RATE);    
}

/**
 * @brief      Set output baudrate to default
 *
 */
void EiDeviceSonySpresense::set_default_data_output_baudrate(void)
{
    spresense_uart_flush();
    spresense_set_uart(DEFAULT_BAUD);
}

void EiDeviceSonySpresense::set_state(EiState state)
{
    this->state = state;

    if (state == eiStateFinished) {
        spresense_ledcontrol(LEDALL, 0);
        ei_sleep(100);
        spresense_ledcontrol(LED1, 1);
        ei_sleep(100);
        spresense_ledcontrol(LED2, 1);
        ei_sleep(100);
        spresense_ledcontrol(LED3, 1);
        ei_sleep(100);
        spresense_ledcontrol(LED4, 1);
        ei_sleep(200);
        spresense_ledcontrol(LEDALL, 0);
        ei_sleep(200);
        spresense_ledcontrol(LEDALL, 1);
        ei_sleep(200);
        spresense_ledcontrol(LEDALL, 0);

        this->state = eiStateIdle;
    }
}

EiState EiDeviceSonySpresense::get_state(void)
{
    return this->state;
}

EiSnapshotProperties EiDeviceSonySpresense::get_snapshot_list(void)
{
    ei_device_snapshot_resolutions_t *res = NULL;
    uint8_t res_num = 0;

    EiSnapshotProperties props = {
        .has_snapshot = false,
        .support_stream = false,
        .color_depth = "",
        .resolutions_num = 0,
        .resolutions = res
    };

    if(this->camera->is_camera_present() == true) {
        this->camera->get_resolutions(&res, &res_num);
        props.has_snapshot = true;
        props.support_stream = true;
        props.color_depth = "RGB";
        props.resolutions_num = res_num;
        props.resolutions = res;
    }


    return props;
}

/**
 * @brief 
 * 
 * @param sample_read_cb 
 * @param sample_interval_ms 
 * @return true 
 * @return false 
 */
bool EiDeviceSonySpresense::start_sample_thread(void (*sample_read_cb)(void), float sample_interval_ms)
{
    int ret = ei_timer_init(&local_sample_thread, (uint32_t)sample_interval_ms * 1000);

    if (ret != 0) {
        ei_printf("Error in ei_timer_init %d", ret);
        return false;
    }

    this->sample_interval_ms = sample_interval_ms;
    this->is_sampling = true;
    this->sample_read_callback = sample_read_cb;

    return true;
}

#if MULTI_FREQ_ENABLED == 1
/**
 * @brief 
 * 
 * @param sample_multi_read_cb 
 * @param multi_sample_interval_ms 
 * @param num_fusioned 
 * @return true 
 * @return false 
 */
bool EiDeviceSonySpresense::start_multi_sample_thread(void (*sample_multi_read_cb)(uint8_t), float* multi_sample_interval_ms, uint8_t num_fusioned)
{
    uint8_t i;
    uint8_t flag = 0;

    this->sample_multi_read_callback = sample_multi_read_cb;
    this->fusioning = num_fusioned;
    this->multi_sample_interval.clear();

    for (i = 0; i < num_fusioned; i++){
        this->multi_sample_interval.push_back(1.f/multi_sample_interval_ms[i]*1000.f);
    }

    this->sample_interval = ei_fusion_calc_multi_gcd(this->multi_sample_interval.data(), this->fusioning);

    /* force first reading */
    for (i = 0; i < this->fusioning; i++){
            flag |= (1<<i);
    }
    this->sample_multi_read_callback(flag);

    this->actual_timer = 0;
    // start thread !

    int ret = ei_timer_init(&local_multi_sample_thread, (uint32_t)this->sample_interval * 1000);

    if (ret != 0) {
        ei_printf("Error in ei_timer_init %d", ret);
        return false;
    }

    this->sample_interval_ms = sample_interval;
    this->is_sampling = true;

    return true;
}
#endif

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool EiDeviceSonySpresense::stop_sample_thread(void)
{
    this->is_sampling = false;
    ei_timer_stop();

    return true;
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool EiDeviceSonySpresense::is_camera_present()
{
    return this->camera->is_camera_present();
}

/**
 * @brief 
 * 
 * @param sensor_list 
 * @param sensor_list_size 
 * @return true 
 * @return false 
 */
bool EiDeviceSonySpresense::get_sensor_list(const ei_device_sensor_t **sensor_list, size_t *sensor_list_size)
{
    *sensor_list      = sensors;
    *sensor_list_size = MAX_USED_SENSOR;

    return false;
}

/**
 * @brief Wrapper for camera stop_stream
 * 
 * @return true 
 * @return false 
 */
bool EiDeviceSonySpresense::stop_stream(void)
{
    if (camera->stop_stream() == true) {
        ei_printf("OK\n");
        this->set_default_data_output_baudrate();
        ei_sleep(100);
        ei_printf("Snapshot streaming stopped by user\n");
        ei_printf("OK\n");
        return true;
    }
    else {
        ei_printf("Error in stop streaming\n");

        ei_printf("OK\n");
        this->set_default_data_output_baudrate();
        ei_sleep(100);        
        ei_printf("OK\n");
        return false;
    }
}

/* Private functions ------------------------------------------------------- */
/**
 * @brief 
 * 
 * @return unsigned int 
 */
unsigned int local_sample_thread(void)
{
    EiDeviceSonySpresense* dev = static_cast<EiDeviceSonySpresense*>(EiDeviceSonySpresense::get_device());
    unsigned int next_interval = 0;

    if (dev != nullptr) {
        dev->sample_read_callback();
    }

    if (dev->get_is_sampling() == true) {
        next_interval =  (dev->get_sample_interval_ms() * 1000);
    }
    else {

    }

    return next_interval;
}

#if MULTI_FREQ_ENABLED == 1
/**
 * @brief 
 * 
 * @return unsigned int 
 */
unsigned int local_multi_sample_thread(void)
{
    unsigned int next_interval = 0;
    EiDeviceSonySpresense *dev = static_cast<EiDeviceSonySpresense*>(EiDeviceInfo::get_device());

    uint8_t flag = 0;
    uint8_t i = 0;
    
    dev->actual_timer += dev->get_sample_interval();  /* update actual time */

    for (i = 0; i < dev->get_fusioning(); i++){
        if (((uint32_t)(dev->actual_timer % (uint32_t)dev->multi_sample_interval.at(i))) == 0) {   /* check if period of sensor is a multiple of actual time*/
            flag |= (1<<i);                                                                     /* if so, time to sample it! */
        }
    }

    if (dev->sample_multi_read_callback != nullptr){
        dev->sample_multi_read_callback(flag);        
    }

    if (dev->get_is_sampling() == true) {
        next_interval =  (dev->get_sample_interval() * 1000);
    }
    else {

    }

    return next_interval;
}
#endif
