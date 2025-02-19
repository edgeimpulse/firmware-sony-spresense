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

#ifndef I2C_H
#define I2C_H

#include <stdint.h>

// return value
#define I2C_SUCCESS         (0) // success
#define I2C_DATA_TOO_LONG   (1) // data too long to fit in transmit buffer
#define I2C_NACK_ON_ADDRESS (2) // received NACK on transmit of address
#define I2C_NACK_ON_DATA    (3) // received NACK on transmit of data
#define I2C_OTHER_ERROR     (4) // other error

// I2C frequence supported
#define I2C_FREQ_100KHZ     (100000)    // standard mode
#define I2C_FREQ_400KHZ     (400000)    // fast mode
#define I2C_FREQ_1MHZ       (1000000)   // fast mode plus


/**
 * @brief  I2C init
 * @param  None
 * @retval None
 */
void i2c_init(void);

/**
 * @brief  Set I2C freq
 * @param  [in] freq Frequency from "I2C frequence supported"
 * @retval None
 */
void i2c_set_freq(uint32_t freq);

/**
 * @brief  Write device register via I2C
 * @param  [in] i2c_addr I2C devices address
 * @param  [in] reg_addr I2C devices register address
 * @param  [in] data dete wich should be write in register
 * @retval None
 */
uint8_t i2c_write_reg(uint8_t i2c_addr, uint8_t reg_addr, uint8_t data);

/**
 * @brief  Read device register via I2C
 * @param  [in] i2c_addr I2C devices address
 * @param  [in] reg_addr I2C devices register address
 * @param  [out] data dete wich should be read from register
 * @retval None
 */
uint8_t i2c_read_reg(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *data);

/**
 * @brief  Write device registers via I2C
 * @param  [in] i2c_addr I2C devices address
 * @param  [in] reg_addr I2C devices register address
 * @param  [in] data dete wich should be read from register
 * @param  [in] size size of data to write
 * @retval None
 */
uint8_t i2c_write_regs(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *data, uint16_t size);

/**
 * @brief  Read device registers via I2C
 * @param  [in] i2c_addr I2C devices address
 * @param  [in] reg_addr I2C devices register address
 * @param  [out] data dete wich should be read from register
 * @param  [in] size size of data to read
 * @retval I2C_SUCCESS (0) if ok
 */
uint8_t i2c_read_regs(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *data, uint16_t size);

/**
 * @brief  Read device data via I2C
 * @param  [in] i2c_addr I2C devices address
 * @param  [out] data dete wich should be read from register
 * @param  [in] size size of data to read
 * @retval I2C_SUCCESS (0) if ok
 */
uint8_t i2c_read_data(uint8_t i2c_addr, uint8_t *data, uint16_t size);

/**
 * @brief  Write device data via I2C
 * @param  [in] i2c_addr I2C devices address
 * @param  [in] data dete wich should be read from register
 * @param  [in] size size of data to write
 * @retval None
 */
uint8_t i2c_write_data(uint8_t i2c_addr, uint8_t *data, uint16_t size);

/**
 * @brief  Write device registers via I2C with 16 bit addres
 * @param  [in] i2c_addr I2C devices address
 * @param  [in] reg_addr I2C devices register address
 * @param  [in] data dete wich should be read from register
 * @param  [in] size size of data to write
 * @retval None
 */
uint8_t i2c_write_regs_16addr(uint8_t i2c_addr, uint16_t reg_addr, uint8_t *data, uint16_t size);

/**
 * @brief  Read device registers via I2C with 16 bit addres
 * @param  [in] i2c_addr I2C devices address
 * @param  [in] reg_addr I2C devices register address
 * @param  [out] data dete wich should be read from register
 * @param  [in] size size of data to read
 * @retval I2C_SUCCESS (0) if ok
 */
uint8_t i2c_read_regs_16addr(uint8_t i2c_addr, uint16_t reg_addr, uint8_t *data, uint16_t size);

/**
 * @brief  Check if device present or ready
 * @param  [in] i2c_addr I2C devices address
 * @retval true if ok or ready
 */
bool i2c_is_bevice_present (uint8_t i2c_addr);


#endif // I2C_H
