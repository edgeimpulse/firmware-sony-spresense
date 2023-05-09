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
#ifndef EI_BOARD_CTRL__H_
#define EI_BOARD_CTRL__H_

#include <stdint.h>

extern bool spresense_init_acc(void);
extern int spresense_getAcc(float acc_val[3]);

extern int spresense_setupAudio(void);
extern bool spresense_startStopAudio(bool start);
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