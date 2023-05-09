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

/* Includes ---------------------------------------------------------------- */
#include <stdio.h>
#include <sys/boardctl.h>
#include <hardware/cxd5602_memorymap.h>
#include "ei_board.h"

/* Extern reference -------------------------------------------------------- */
extern int ei_main();

/* Registers read/write macro's ------------------------------------------- */

#define CONSOLE_BASE    CXD56_UART1_BASE

extern "C" {

// Declared weak in Arduino.h to allow user redefinitions.
int atexit(void (*func)()) __attribute__((weak));
int atexit(void (* /*func*/ )()) { return 0; }

// Weak empty variant initialization function.
// May be redefined by variant files.
void initVariant() __attribute__((weak));
void initVariant() { }

#if defined(CONFIG_HAVE_CXX) && defined(CONFIG_HAVE_CXXINITIALIZE)
typedef void (*initializer_t)(void);
extern initializer_t _sinit;
extern initializer_t _einit;
extern uint32_t _stext;
extern uint32_t _etext;

static void up_cxxinitialize(void)
{
    initializer_t *initp;

    /* Visit each entry in the initialization table */

    for (initp = &_sinit; initp != &_einit; initp++) {
        initializer_t initializer = *initp;

        /* Make sure that the address is non-NULL and lies in the text region
         * defined by the linker script.  Some toolchains may put NULL values
         * or counts in the initialization table.
         */

        if ((void *)initializer > (void *)&_stext && (void *)initializer < (void *)&_etext) {
            initializer();
        }
    }
}
#endif
}

/**
 * @brief Main application function
 *
 */
extern "C" int spresense_main(void)
{
#if defined(CONFIG_HAVE_CXX) && defined(CONFIG_HAVE_CXXINITIALIZE)
    up_cxxinitialize();
#endif
    spresense_board_init();

    ei_main();

    return 0;
}

/* Private functions ------------------------------------------------------- */
