/*
 * Copyright (c) 2022 Edge Impulse Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an "AS
 * IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language
 * governing permissions and limitations under the License.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
 /* Include ----------------------------------------------------------------- */
#include "model-parameters/model_metadata.h"
#if defined(EI_CLASSIFIER_SENSOR) && ((EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_FUSION) || (EI_CLASSIFIER_SENSOR == EI_CLASSIFIER_SENSOR_ACCELEROMETER))
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "firmware-sdk/ei_fusion.h"
#include "inference/ei_run_impulse.h"
#include "ei_device_sony_spresense.h"
#include "model-parameters/model_variables.h"

typedef enum {
    INFERENCE_STOPPED,
    INFERENCE_WAITING,
    INFERENCE_SAMPLING,
    INFERENCE_DATA_READY
} inference_state_t;

static int print_results;
static uint16_t samples_per_inference;
static inference_state_t state = INFERENCE_STOPPED;
static uint64_t last_inference_ts = 0;
static bool continuous_mode = false;
static bool debug_mode = false;
static float samples_circ_buff[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
static int samples_wr_index = 0;

/**
 * @brief Called for each single sample
 */
bool samples_callback(const void *raw_sample, uint32_t raw_sample_size)
{
    if(state != INFERENCE_SAMPLING) {
        // stop collecting samples if we are not in SAMPLING state
        return true;
    }

    float *sample = (float *)raw_sample;

    for(int i = 0; i < (int)(raw_sample_size / sizeof(float)); i++) {
        samples_circ_buff[samples_wr_index++] = sample[i];
        if(samples_wr_index > EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
            /* start from beginning of the circular buffer */
            samples_wr_index = 0;
        }
        if(samples_wr_index >= samples_per_inference) {                
            state = INFERENCE_DATA_READY;
            ei_printf("Recording done\n");
            return true;
        }
    }

    return false;
}

/**
 * @brief
 */
static void display_results(ei_impulse_result_t* result)
{
    float max = 0.0f;
    size_t max_ix = 0;

    ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
        result->timing.dsp, result->timing.classification, result->timing.anomaly);
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {            
        ei_printf("    %s: \t", result->classification[ix].label);
        ei_printf_float(result->classification[ix].value);
        ei_printf("\r\n");
    }
#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("    anomaly score: ");
    ei_printf_float(result->anomaly);
    ei_printf("\r\n");
#endif

    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {       
        if (result->classification[ix].value > max) {
            max = result->classification[ix].value;
            max_ix = ix;
        }
    }
}

/**
 * @brief
 */
void ei_run_impulse(void)
{
    EiDeviceSonySpresense* dev = static_cast<EiDeviceSonySpresense*>(EiDeviceSonySpresense::get_device());

    switch(state) {
        case INFERENCE_STOPPED:
            // nothing to do
            return;
        case INFERENCE_WAITING:
            if(ei_read_timer_ms() < (last_inference_ts + 2000)) {
                return;
            }
            ei_printf("Recording\n");
            state = INFERENCE_SAMPLING;
            ei_fusion_sample_start(&samples_callback, EI_CLASSIFIER_INTERVAL_MS);
            dev->set_state(eiStateSampling);
            return;
        case INFERENCE_SAMPLING:
            //dev->sample_thread();
            return;
        case INFERENCE_DATA_READY:
            dev->set_state(eiStateIdle);
            // nothing to do, just continue to inference provcessing below
            break;
        default:
            break;
    }

    signal_t signal;

    // shift circular buffer, so the newest data will be the first
    // if samples_wr_index is 0, then roll is immediately returning
    numpy::roll(samples_circ_buff, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, (-samples_wr_index));
    /* reset wr index, the oldest data will be overwritten */
    samples_wr_index = 0;

    // Create a data structure to represent this window of data
    int err = numpy::signal_from_buffer(samples_circ_buff, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    if (err != 0) {
        ei_printf("ERR: signal_from_buffer failed (%d)\n", err); 
    }

    // run the impulse: DSP, neural network and the Anomaly algorithm
    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR ei_error;
    if(continuous_mode == true) {
        ei_error = run_classifier_continuous(&signal, &result, debug_mode);
    }
    else {
        ei_error = run_classifier(&signal, &result, debug_mode);
    }

    if (ei_error != EI_IMPULSE_OK) {
        ei_printf("Failed to run impulse (%d)", ei_error);
        return;
    }

    if(continuous_mode == true) {
        if(++print_results >= (EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW >> 1)) {
            display_results(&result);
            print_results = 0;
        }
    }
    else {
        display_results(&result);
    }

    if(continuous_mode == true) {
        state = INFERENCE_SAMPLING;
    }
    else {
        ei_printf("Starting inferencing in 2 seconds...\n");
        last_inference_ts = ei_read_timer_ms();
        state = INFERENCE_WAITING;
    }
}

/**
 * @brief
 */
void ei_start_impulse(bool continuous, bool debug, bool use_max_uart_speed)
{
    EiDeviceInfo *dev = EiDeviceInfo::get_device();

    const char *axis_name = EI_CLASSIFIER_FUSION_AXES_STRING;
    
    if (!ei_connect_fusion_list(axis_name, AXIS_FORMAT)) {
        ei_printf("ERR: Failed to find sensor '%s' in the sensor list\n", axis_name);
        return;
    }

    if (continuous == true) {
        ei_printf("Error no continuous classification available for current model\r\n");
        return;
    }

    continuous_mode = continuous;
    debug_mode = debug;

    // summary of inferencing settings (from model_metadata.h)
    ei_printf("Inferencing settings:\n");
    ei_printf("\tInterval: ");
    ei_printf_float((float)EI_CLASSIFIER_INTERVAL_MS);
    ei_printf(" ms.");
    ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    ei_printf("\tSample length: ");
    ei_printf_float((EI_CLASSIFIER_RAW_SAMPLE_COUNT * EI_CLASSIFIER_INTERVAL_MS));
    ei_printf(" ms.");
    ei_printf("\tNo. of classes: %d\n", sizeof(ei_classifier_inferencing_categories) /
                                            sizeof(ei_classifier_inferencing_categories[0]));
    ei_printf("Starting inferencing, press 'b' to break\n");

    dev->set_sample_length_ms(EI_CLASSIFIER_RAW_SAMPLE_COUNT * EI_CLASSIFIER_INTERVAL_MS);
    dev->set_sample_interval_ms(EI_CLASSIFIER_INTERVAL_MS);

    if (continuous == true) {
        samples_per_inference = EI_CLASSIFIER_SLICE_SIZE * EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME;
        // In order to have meaningful classification results, continuous inference has to run over
        // the complete model window. So the first iterations will print out garbage.
        // We now use a fixed length moving average filter of half the slices per model window and
        // only print when we run the complete maf buffer to prevent printing the same classification multiple times.
        print_results = -(EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW);
        run_classifier_init();
        state = INFERENCE_SAMPLING;
    }
    else {
        samples_per_inference = EI_CLASSIFIER_RAW_SAMPLE_COUNT * EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME;
        // it's time to prepare for sampling
        ei_printf("Starting inferencing in 2 seconds...\n");
        last_inference_ts = ei_read_timer_ms();
        state = INFERENCE_WAITING;
    }
}

/**
 * @brief
 */
void ei_stop_impulse(void) 
{
    EiDeviceInfo *dev = EiDeviceInfo::get_device();

    if(state != INFERENCE_STOPPED) {
        state = INFERENCE_STOPPED;
        ei_printf("Inferencing stopped by user\r\n");
        dev->set_state(eiStateFinished);
        /* reset samples buffer */
        samples_wr_index = 0;
        run_classifier_deinit();
    }
}

/**
 * @brief
 */
bool is_inference_running(void)
{
    return (state != INFERENCE_STOPPED);
}

#endif
