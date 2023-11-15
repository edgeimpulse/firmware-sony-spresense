/* Edge Impulse ingestion SDK
 * Copyright (c) 2021 EdgeImpulse Inc.
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

/* Include ----------------------------------------------------------------- */
#include <stdint.h>
#include <stdlib.h>

#include "ei_microphone.h"
#include "ei_device_sony_spresense.h"
#include "ei_flash_memory.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"
#include "edge-impulse-sdk/dsp/numpy.hpp"
#include "ei_board_ctrl.h"

#include "ei_config_types.h"
#include "sensor_aq_mbedtls_hs256.h"
#include "firmware-sdk/sensor-aq/sensor_aq_none.h"

/* Audio sampling config */
//#define AUDIO_SAMPLING_FREQUENCY            16000
//#define AUDIO_SAMPLES_PER_MS                (AUDIO_SAMPLING_FREQUENCY / 1000)
//#define AUDIO_DSP_SAMPLE_LENGTH_MS          16
//#define AUDIO_DSP_SAMPLE_RESOLUTION         (sizeof(short))
//#define AUDIO_DSP_SAMPLE_BUFFER_SIZE        1600//(AUDIO_SAMPLES_PER_MS * AUDIO_DSP_SAMPLE_LENGTH_MS * AUDIO_DSP_SAMPLE_RESOLUTION)
#define AUDIO_DSP_SAMPLE_BUFFER_SIZE        4800

/** Status and control struct for inferencing struct */
typedef struct {
    int16_t *buffers[2];
    uint8_t buf_select;
    uint8_t buf_ready;
    uint32_t buf_count;
    uint32_t n_samples;
} inference_t;

/* Dummy functions for sensor_aq_ctx type */
static size_t ei_write(const void* buffer, size_t size, size_t count, EI_SENSOR_AQ_STREAM* stream)
{
    (void)buffer;
    (void)size;
    (void)stream;

    return count;
}

static int ei_seek(EI_SENSOR_AQ_STREAM* stream, long int offset, int origin)
{
    (void)stream;
    (void)offset;
    (void)origin;

    return 0;
}

/* Private variables ------------------------------------------------------- */
static bool record_ready = false;
static uint32_t headerOffset;
static uint32_t samples_required;
static uint32_t current_sample;
static uint32_t cbor_current_sample;
static uint8_t n_audio_channels = 4;

static inference_t inference;

static unsigned char ei_mic_ctx_buffer[1024];
static sensor_aq_signing_ctx_t ei_mic_signing_ctx;
static sensor_aq_mbedtls_hs256_ctx_t ei_mic_hs_ctx;
static sensor_aq_ctx ei_mic_ctx = {
    { ei_mic_ctx_buffer, 1024 },
    &ei_mic_signing_ctx,
    &ei_write,
    &ei_seek,
    NULL,
};

static int16_t audio_buffer[AUDIO_DSP_SAMPLE_BUFFER_SIZE];

static bool ei_microphone_record(uint32_t sample_length_ms, uint32_t start_delay_ms, bool print_start_messages);
static void audio_buffer_callback(void *buffer, uint32_t n_bytes);
static bool create_header(sensor_aq_payload_info *payload);
static void audio_buffer_inference_callback(void *buffer, uint32_t n_bytes);
static void get_dsp_data(void (*callback)(void *buffer, uint32_t n_bytes));

/* Public functions -------------------------------------------------------- */
/**
 * @brief      Set the PDM mic to +34dB
 */
void ei_microphone_init(void)
{
    
}

/**
 * @brief 
 * 
 * @param n_samples 
 * @return true 
 * @return false 
 */
bool ei_microphone_inference_start(uint32_t n_samples, uint8_t n_channels_inference, uint32_t freq)
{
    int retval = 0;

    if (n_channels_inference > EI_AUDIO_N_CHANNELS) {
        ei_printf("Wrong number of channels\r\n");
        return false;
    }

    n_audio_channels = n_channels_inference;

    inference.buffers[0] = (int16_t *)ei_malloc(n_samples * sizeof(int16_t) * n_audio_channels);

    if (inference.buffers[0] == NULL) {
        ei_printf("Can't allocate first audio buffer\r\n");
        return false;
    }

    inference.buffers[1] = (int16_t *)ei_malloc(n_samples * sizeof(int16_t) * n_audio_channels);

    if (inference.buffers[1] == NULL) {
        ei_free(inference.buffers[0]);
        ei_printf("Can't allocate second audio buffer\r\n");
        return false;
    }

    inference.buf_select = 0;
    inference.buf_count = 0;
    inference.n_samples = n_samples;    // this represents the number of samples per channel
    inference.buf_ready = 0;

    retval = spresense_startStopAudio(true, n_audio_channels, freq);
    if (retval != 0) {
        ei_printf("Error in spresense_startStopAudio: %d\r\n", retval);
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
bool ei_microphone_inference_record(void)
{
    bool recorded = false;

    get_dsp_data(&audio_buffer_inference_callback);
    if (inference.buf_ready == 1) {

        recorded = true;
        inference.buf_ready = 0;
        inference.buf_count = 0;
    }

    return recorded;
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool ei_microphone_inference_is_complete(void)
{
    return record_ready;
}

/**
 * @brief      Reset buffer counters for non-continuous inferecing
 */
void ei_microphone_inference_reset_buffers(void)
{
    inference.buf_ready = 0;
    inference.buf_count = 0;
    record_ready = false;
}

/**
 * Get raw audio signal data
 */
int ei_microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr)
{    
    ei::numpy::int16_to_float(&inference.buffers[inference.buf_select ^ 1][offset], out_ptr, length);
    record_ready = false;

    return 0;
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool ei_microphone_inference_end(void)
{
    record_ready = false;
    spresense_startStopAudio(false, n_audio_channels, 0);   // when stopping freq not important

    ei_free(inference.buffers[0]);
    ei_free(inference.buffers[1]);
    return true;
}

bool ei_microphone_1ch_start(void)
{
    n_audio_channels = 1;
    return ei_microphone_sample_start();
}

bool ei_microphone_2ch_start(void)
{
    n_audio_channels = 2;
    return ei_microphone_sample_start();
}

bool ei_microphone_3ch_start(void)
{
    n_audio_channels = 3;
    return ei_microphone_sample_start();
}

bool ei_microphone_4ch_start(void)
{
    n_audio_channels = 4;
    return ei_microphone_sample_start();
}

/**
 * Sample raw data
 */
bool ei_microphone_sample_start(void)
{
    EiDeviceSonySpresense* dev = static_cast<EiDeviceSonySpresense*>(EiDeviceInfo::get_device());
    EiFlashMemory* mem = static_cast<EiFlashMemory*>(dev->get_memory());
    // this sensor does not have settable interval...

    ei_printf("Sampling settings:\n");
    ei_printf("\tInterval: %.5f ms.\n", dev->get_sample_interval_ms());
    ei_printf("\tLength: %lu ms.\n", dev->get_sample_length_ms());
    ei_printf("\tName: %s\n", dev->get_sample_label().c_str());
    ei_printf("\tHMAC Key: %s\n", dev->get_sample_hmac_key().c_str());
    ei_printf("\tFile name: %s\n", dev->get_sample_label().c_str());

    char filename[256];
    int fn_r = snprintf(filename, 256, "/fs/%s", dev->get_sample_label().c_str());
    if (fn_r <= 0) {
        ei_printf("ERR: Failed to allocate file name\n");
        return false;
    }
    ei_printf("\tFile name: %s\n", filename);

    samples_required = (uint32_t)(((float)dev->get_sample_length_ms()) / dev->get_sample_interval_ms());

    /* Round to even number of samples for word align flash write */
    if (samples_required & 1) {
        samples_required++;
    }

    current_sample = 0;
    cbor_current_sample = 0;

    bool r = ei_microphone_record(dev->get_sample_length_ms(), (((samples_required <<1)/ mem->block_size) * mem->block_erase_time), true);
    if (!r) {
        return r;
    }
    record_ready = false;
    dev->set_state(eiStateSampling);

    while (record_ready == false) {
        get_dsp_data(&audio_buffer_callback);
    };

    spresense_startStopAudio(false, n_audio_channels, 0);   // when stopping freq not important

    /* Write end of cbor + dummy */
    const uint8_t end_of_cbor[] = {0xff, 0xff, 0xff, 0xff};
    mem->write_sample_data((uint8_t*)end_of_cbor, headerOffset + cbor_current_sample, 4);

    int ctx_err =
        ei_mic_ctx.signature_ctx->finish(ei_mic_ctx.signature_ctx, ei_mic_ctx.hash_buffer.buffer);
    if (ctx_err != 0) {
        ei_printf("Failed to finish signature (%d)\n", ctx_err);
        return false;
    }

    mem->close_sample_file();

    ei_printf("Done sampling, total bytes collected: %lu\n", current_sample);
    ei_printf("[1/1] Uploading file to Edge Impulse...\n");
    ei_printf("Not uploading file, not connected to WiFi. Used buffer, from=0, to=%lu.\n", cbor_current_sample + 1 + headerOffset);
    ei_printf("OK\n");

    return true;
}

/* Private functions ------------------------------------------------------- */

/**
 * @brief 
 * 
 * @param sample_length_ms 
 * @param start_delay_ms 
 * @param print_start_messages 
 * @return true 
 * @return false 
 */
static bool ei_microphone_record(uint32_t sample_length_ms, uint32_t start_delay_ms, bool print_start_messages)
{
    EiDeviceSonySpresense* dev = static_cast<EiDeviceSonySpresense*>(EiDeviceSonySpresense::get_device());
    EiFlashMemory* mem = static_cast<EiFlashMemory*>(dev->get_memory());
    uint32_t freq = roundf(1.0/(dev->get_sample_interval_ms()))*1000;
    
    sensor_aq_payload_info payload = {
        dev->get_device_id().c_str(),
        dev->get_device_type().c_str(),
        dev->get_sample_interval_ms(),
        { { "audio", "wav" } }
    };

    if(n_audio_channels > 1) {
        payload.sensors[1] = { "audio2", "wav"};
    }
    if(n_audio_channels > 2) {
        payload.sensors[2] = { "audio3", "wav"};
    }
    if(n_audio_channels > 3) {
        payload.sensors[3] = { "audio4", "wav"};
    }

    dev->set_state(eiStateErasingFlash);

    if (print_start_messages) {
        ei_printf(
            "Starting in %lu ms... (or until all flash was erased)\n",
            start_delay_ms < 2000 ? 2000 : start_delay_ms);
    }
        
    if (!spresense_startStopAudio(true, n_audio_channels, freq)) {
        ei_printf("\r\nERR: Missing DSP binary. Follow steps here https://developer.sony.com/develop/spresense/docs/arduino_tutorials_en.html#_install_dsp_files\r\n");
        return false;
    }

    if (start_delay_ms < 2000) {
        ei_sleep(2000 - start_delay_ms);
    }
    
    if (mem->erase_sample_data(0, mem->block_size) != 0) {
        ei_printf("Error in erase_sample_data\n");
        spresense_startStopAudio(false, n_audio_channels, 0);   // when stopping freq not important
        return false;
    }
    mem->close_sample_file();

	mem->openSampleFile(true);

    create_header(&payload);

    if (print_start_messages) {
        ei_printf("Sampling...\n");
    }

    return true;
}

/**
 * Convert to CBOR int16 format and write to buf
*/
static void write_value_to_cbor_buffer(uint8_t *buf, int16_t value)
{
    uint8_t datatype;
    uint16_t sample;
    if(value < 0) {
        datatype = 0x39;
        /* Convert from 2's complement */
        sample = (uint16_t)~value + 1;
    }
    else {
        datatype = 0x19;
        sample = value;
    }
    buf[0] = datatype;
    buf[1] = sample >> 8;
    buf[2] = sample & 0xFF;
}

/**
 * @brief      Ingestion audio callback, write audio samples to memory
 *             Signal record_ready when all needed samples are there
 * @param      buffer   Pointer to source buffer
 * @param[in]  n_bytes  Number of bytes to write
 */
static void audio_buffer_callback(void *buffer, uint32_t n_bytes)
{
    EiDeviceSonySpresense* dev = static_cast<EiDeviceSonySpresense*>(EiDeviceInfo::get_device());
    EiFlashMemory* mem = static_cast<EiFlashMemory*>(dev->get_memory());

    /* The number of bytes is divided over the number of sampled audio channels */
    n_bytes /= n_audio_channels;

    uint32_t n_samples = n_bytes / 2;
    int16_t *sbuffer = (int16_t *)buffer;
    uint32_t sbuf_ptr = 0;

    /* Calculate the cbor buffer length: header + 3 bytes per audio channel */
    uint32_t cbor_length = n_samples + ((n_samples + n_bytes) * n_audio_channels);
    uint8_t *cbor_buf = (uint8_t *)ei_malloc(cbor_length);

    if(cbor_buf == NULL) {
        ei_printf("ERR: memory allocation error\r\n");
        return;
    }

    for (int i = 0; i < n_samples; i++) {
        uint32_t cval_ptr = i * ((n_audio_channels * 3) + 1);

        cbor_buf[cval_ptr] = 0x80 + n_audio_channels;
        for (int y = 0; y < n_audio_channels; y++) {
            write_value_to_cbor_buffer(&cbor_buf[cval_ptr + 1 + (3 * y)], sbuffer[sbuf_ptr + y]);
        }
        sbuf_ptr += n_audio_channels;
    }

    mem->write_sample_data((uint8_t*)cbor_buf, headerOffset + cbor_current_sample, cbor_length);

    ei_free(cbor_buf);

    cbor_current_sample += cbor_length;
    current_sample += n_bytes;
    if (current_sample >= (samples_required << 1)) {
        record_ready = true;
    }
}

/**
 * @brief      Inference audio callback, store samples in ram buffer
 *             Signal when buffer is full, and swap buffers
 * @param      buffer   Pointer to source buffer
 * @param[in]  n_bytes  Number of bytes to write
 */
static void audio_buffer_inference_callback(void *buffer, uint32_t n_bytes)
{
    int16_t *samples = (int16_t *)buffer;

    for (uint32_t i = 0; i < (n_bytes >> 1); i++) {
        inference.buffers[inference.buf_select][inference.buf_count++] = samples[i];

        if (inference.buf_count >= (inference.n_samples * n_audio_channels)) {  // n_samples is per channel

            inference.buf_select ^= 1;
            inference.buf_count = 0;
            inference.buf_ready = 1;
        }
    }    
}

/**
 * @brief      Check DSP semaphores, when ready get sample buffer that belongs
 *             to the semaphore.
 * @param[in]  callback  Callback needs to handle the audio samples
 */
static void get_dsp_data(void (*callback)(void *buffer, uint32_t n_bytes))
{
    size_t length;

    if (spresense_getAudio((char *)&audio_buffer[0], &length)) {
        callback((void *)&audio_buffer[0], length);
    }
}

/**
 *
 * @param payload
 * @return
 */
static bool create_header(sensor_aq_payload_info *payload)
{
    int ret;
    EiDeviceSonySpresense* dev = static_cast<EiDeviceSonySpresense*>(EiDeviceInfo::get_device());
    EiFlashMemory* mem = static_cast<EiFlashMemory*>(dev->get_memory());

    sensor_aq_init_mbedtls_hs256_context(&ei_mic_signing_ctx, &ei_mic_hs_ctx, dev->get_sample_hmac_key().c_str());

    ret = sensor_aq_init(&ei_mic_ctx, payload, NULL, true);

    if (ret != AQ_OK) {
        ei_printf("sensor_aq_init failed (%d)\n", ret);
        return false;
    }

    // then we're gonna find the last byte that is not 0x00 in the CBOR buffer.
    // That should give us the whole header
    size_t end_of_header_ix = 0;
    for (size_t ix = ei_mic_ctx.cbor_buffer.len - 1; ix != 0; ix--) {
        if (((uint8_t *)ei_mic_ctx.cbor_buffer.ptr)[ix] != 0x0) {
            end_of_header_ix = ix;
            break;
        }
    }

    if (end_of_header_ix == 0) {
        ei_printf("Failed to find end of header\n");
        return false;
    }

    // Write to blockdevice
    ret = mem->write_sample_data((uint8_t*)ei_mic_ctx.cbor_buffer.ptr, 0, end_of_header_ix);
    if ((size_t)ret != end_of_header_ix) {
        ei_printf("Failed to write to header blockdevice (%d)\n", ret);
        return false;
    }

    headerOffset = end_of_header_ix;

    return true;
}
