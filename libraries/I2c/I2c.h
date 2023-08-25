/*
 * Copyright (c) 2023 EdgeImpulse Inc.
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
