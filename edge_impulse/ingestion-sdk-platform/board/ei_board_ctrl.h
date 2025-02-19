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
#ifndef EI_BOARD_CTRL__H_
#define EI_BOARD_CTRL__H_

/* Include ----------------------------------------------------------------- */
#include <stdint.h>

/* Global constants -------------------------------------------------------- */
#define EI_AUDIO_N_CHANNELS     4   /* For Sony can be 1, 2, 4, 6, 8 */

extern bool spresense_init_acc(void);
extern int spresense_getAcc(float acc_val[3]);

extern int spresense_startStopAudio(bool start, uint8_t n_channels, uint32_t freq);
extern void spresense_pauseAudio(bool pause);
extern void spresense_closeAudio(void);
extern bool spresense_getAudio(char *audio_buffer, unsigned int* size);

extern uint32_t spresense_readFromFile(const char *name, uint8_t *buf, uint32_t pos, uint32_t length);
extern int spresense_writeToFile(const char *name, const uint8_t *buf, uint32_t length);
extern int spresense_closeFile(const char *name);
extern int spresense_openFile(const char *name, bool write);
extern int spresense_deleteFile(const char *name);
extern void spresense_rewind(void);

#endif