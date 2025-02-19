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
#include <stdio.h>
#include <sys/boardctl.h>
#include <hardware/cxd5602_memorymap.h>
#include "ei_board.h"
#include "I2c.h"

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
    i2c_init();

    ei_main();

    return 0;
}

/* Private functions ------------------------------------------------------- */
