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

#ifndef EI_TIMER_H_
#define EI_TIMER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern int ei_timer_init(unsigned int (*isr)(void), uint32_t interval);
extern void ei_timer_stop(void);

#ifdef __cplusplus
}
#endif 

#endif
