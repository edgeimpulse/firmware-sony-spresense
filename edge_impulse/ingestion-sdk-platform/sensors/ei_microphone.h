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

#ifndef EI_MICROPHONE_H
#define EI_MICROPHONE_H

/* Include ----------------------------------------------------------------- */
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

/* Function prototypes ----------------------------------------------------- */
extern void ei_microphone_init(void);

extern bool ei_microphone_inference_start(uint32_t n_samples, uint8_t n_channels_inference, uint32_t freq);
extern bool ei_microphone_inference_is_complete(void);
extern bool ei_microphone_inference_record(void);
extern void ei_microphone_inference_reset_buffers(void);
extern bool ei_microphone_inference_end(void);

bool ei_microphone_1ch_start(void);
bool ei_microphone_2ch_start(void);
bool ei_microphone_3ch_start(void);
bool ei_microphone_4ch_start(void);

extern bool ei_microphone_sample_start(void);
extern int ei_microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr);

#endif
