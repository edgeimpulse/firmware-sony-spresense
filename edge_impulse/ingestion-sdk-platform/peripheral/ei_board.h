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

#ifndef EI_BOARD_H_
#define EI_BOARD_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif


/** Led definition */
typedef enum
{
    LED1 = 0, LED2, LED3, LED4, LEDALL
} tEiLeds;

extern void spresense_ledcontrol(tEiLeds led, bool on_off);
extern void spresense_set_uart(uint32_t baud_rate);
extern void spresense_disable_uart_irq(void);
extern void spresense_board_init(void);
extern void spresense_get_chip_id(char* str_id);
extern void spresense_putchar(char cChar);
extern void spresense_puts(const char *str);
extern char spresense_getchar(void);
extern void spresense_uart_flush(void);
extern void spresense_time_cb(uint32_t *sec, uint32_t *nano);

#ifdef __cplusplus
}
#endif 

#endif