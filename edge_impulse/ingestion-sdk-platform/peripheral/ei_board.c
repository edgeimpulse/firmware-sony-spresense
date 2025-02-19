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
#include "ei_board.h"
#include <arch/cxd56xx/pin.h>
#include <arch/board/cxd56_gpioif.h>
#include <chip/hardware/cxd5602_memorymap.h>
#include <chip/cxd56_uart.h>
#include <arch/chip/scu.h>
#include <arch/chip/adc.h>
#include <sys/boardctl.h>
#include <fcntl.h>
#include <stdio.h>
#include <stddef.h>
#include <time.h>


/* Constants */
#define putreg32(v,a)   (*(volatile uint32_t *)(a) = (v))
#define getreg32(a)     (*(volatile uint32_t *)(a))

#define CONSOLE_BASE    CXD56_UART1_BASE

#define BOARD_FCLKOUT_FREQUENCY   (100000000)

#define ADC_RANGE_MIN             0.0f
#define ADC_RANGE_MAX             5.0f

/* UART clocking ***********************************************************/
#define BOARD_UART1_BASEFREQ        BOARD_FCLKOUT_FREQUENCY

/* Private variables ------------------------------------------------------- */


/* Private functions ------------------------------------------------------- */
static float adc_map(int16_t x, int16_t in_min, int16_t in_max,
                     float out_min, float out_max);

/* Public functions ------------------------------------------------------- */
/**
 * @brief Led handler
 * 
 * @param led 
 * @param on_off 
 */
void spresense_ledcontrol(tEiLeds led, bool on_off)
{
    switch (led)
    {
        case LED1: board_gpio_write(PIN_I2S1_DATA_OUT, on_off); break;
        case LED2: board_gpio_write(PIN_I2S1_DATA_IN, on_off);  break;
        case LED3: board_gpio_write(PIN_I2S1_LRCK, on_off);     break;
        case LED4: board_gpio_write(PIN_I2S1_BCK, on_off);      break;
        default: //all leds
        {
            board_gpio_write(PIN_I2S1_DATA_OUT, on_off);
            board_gpio_write(PIN_I2S1_DATA_IN, on_off);
            board_gpio_write(PIN_I2S1_LRCK, on_off);
            board_gpio_write(PIN_I2S1_BCK, on_off);
            break;
        }
    }
}

/**
 * @brief Set baud rate for UART
 * 
 * @param baud_rate 
 */
void spresense_set_uart(uint32_t baud_rate)
{    
    cxd56_uart_reset(0);
    cxd56_setbaud(CONSOLE_BASE, BOARD_UART1_BASEFREQ, baud_rate);
}

/**
 * @brief IRQ is attached to NUTTX handler. Disable to you use Uart directly.
 */
void spresense_disable_uart_irq(void)
{
    putreg32((uint32_t)0x0, CONSOLE_BASE + CXD56_UART_IMSC);
}

/**
 * @brief Hardware initializazion
 * 
 */
void spresense_board_init(void)
{
    boardctl(BOARDIOC_INIT, 0);

    spresense_disable_uart_irq();
    spresense_set_uart(115200);

    //init_acc();
}

/**
 * @brief read micro chip id
 * 
 * @param str_id 
 */
void spresense_get_chip_id(char* str_id)
{
    uint8_t raw_id[16] = {0};    

    if (str_id != NULL) {
        boardctl(BOARDIOC_UNIQUEID, (uintptr_t)raw_id);
        sprintf(str_id, "%X:%X:%X:%X:%X", raw_id[0], raw_id[1], raw_id[2], raw_id[3], raw_id[4]);
    }
}

/**
 * @brief Write a byte straight to the UART DR register
 *
 * @param byte
 */
void spresense_putchar(char cChar)
{
    while ((getreg32(CONSOLE_BASE + CXD56_UART_FR) & UART_FLAG_TXFF));      // exit when 
    putreg32((uint32_t)cChar, CONSOLE_BASE + CXD56_UART_DR);
    while ( (getreg32(CONSOLE_BASE + CXD56_UART_FR) & (UART_FLAG_TXFE)) == 0);    // exit when is empty
}

/**
 * @brief 
 * 
 * @param str 
 */
void spresense_puts(const char *str)
{
    up_puts(str);
    while ( (getreg32(CONSOLE_BASE + CXD56_UART_FR) & (UART_FLAG_TXFE)) == 0);    // exit when is empty
}

/**
 * @brief Get a char directly from the UART
 *
 * @return char
 */
char spresense_getchar(void)
{
    uint32_t reg = 0;
    reg = getreg32(CONSOLE_BASE + CXD56_UART_FR);
    //if ((UART_FR_RXFE & reg) && !(UART_FR_RXFF & reg)) {
    if ((UART_FLAG_RXFE & reg)) {
        return 0;
    }
    else {
        return (char)getreg32(CONSOLE_BASE + CXD56_UART_DR);
    }
}

/**
 * @brief 
 * 
 */
void spresense_uart_flush(void)
{
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
}

/**
 * @brief Get current time from spresense lib
 *
 * @param sec
 * @param nano
 */
void spresense_time_cb(uint32_t *sec, uint32_t *nano)
{
    struct timespec cur_time;
    clock_gettime(CLOCK_MONOTONIC, &cur_time);
    *(sec) = cur_time.tv_sec;
    *(nano)= cur_time.tv_nsec;
}

#define ADC_CHANNEL     "/dev/hpadc0"
int adc_fd = 0;

/**
 * @brief 
 * 
 */
uint32_t spresense_adc_init(void)
{
    int ret;
    int errval = 0;

    if (!adc_fd) {

        adc_fd = open(ADC_CHANNEL, O_RDONLY);
        if (adc_fd < 0) {
        printf("open error:%d,%d\n", adc_fd, errno);
        return -ENODEV;
        }

        ret = ioctl(adc_fd, SCUIOC_SETFIFOMODE, 1);
        if (ret < 0) {
            errval = errno;
            printf("ioctl(SETFIFOMODE) failed: %d\n", errval);
            return 1; // maybe an error code ?
        }

        if (ioctl(adc_fd, ANIOC_CXD56_FIFOSIZE, 2) < 0) {
            printf("ioctl(FIFOSIZE) failed: %d\n", errno);
            return 2;
        }

        ret = ioctl(adc_fd, ANIOC_CXD56_START, 0);
        if (ret < 0) {
            errval = errno;
            return 3;
        }
    }

    return 0;
}

/**
 * @brief 
 * 
 * @return uint16_t 
 */
float spresense_adc_read(void)
{
    int16_t adc_read = -1;
    float value = 0.0;
    ssize_t nbytes;

    if (adc_fd) {
        nbytes = read(adc_fd, &adc_read, sizeof(int16_t));
        value = adc_map(adc_read, INT16_MIN, INT16_MAX, ADC_RANGE_MIN, ADC_RANGE_MAX);       
    }

    return value;
}

/* Private functions ------------------------------------------------------- */
/**
 * @brief 
 * 
 * @param x 
 * @param in_min 
 * @param in_max 
 * @param out_min 
 * @param out_max 
 * @return float 
 */
static float adc_map(int16_t x, int16_t in_min, int16_t in_max,
                     float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
