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

#include "I2c.h"
#include <sdk/config.h>

#include <string.h>
#include <stdio.h>
extern "C" {
#include <cxd56_i2c.h>
#include <cxd56_pmic.h>
}
#include <errno.h>

// buffer
#define BUFFER_LENGTH       (32)
#define I2C_TX_BUF_LEN      BUFFER_LENGTH
#define I2C_RX_BUF_LEN      BUFFER_LENGTH

uint8_t _tx_buf[I2C_TX_BUF_LEN];
uint8_t _tx_buf_index = 0;
uint8_t _tx_buf_len = 0;
uint8_t _rx_buf[I2C_RX_BUF_LEN];
uint8_t _rx_buf_index = 0;
uint8_t _rx_buf_len = 0;
uint32_t _freq = I2C_FREQ_100KHZ;

FAR struct i2c_master_s* _dev;



/**
 * @brief  I2C init
 * @param  None
 * @retval None
 */
void i2c_init(void) {
    static bool isInit = false;
    if (isInit == false) {
        isInit = true;
        memset(_tx_buf, 0, sizeof(_tx_buf));
        memset(_rx_buf, 0, sizeof(_rx_buf));

        _dev = cxd56_i2cbus_initialize(0);
        if (_dev == 0){
        }
    }
}

/**
 * @brief  Set I2C freq
 * @param  [in] freq Frequency from "I2C frequence supported"
 * @retval None
 */
void i2c_set_freq(uint32_t freq) {
    _freq = freq;
}

static uint8_t i2c_request(uint8_t address, uint8_t quantity, uint8_t sendStop) {
    struct i2c_msg_s msg;
    unsigned int flags = 0;
    int ret;

    if (!_dev) return 0;

    // clamp to buffer length
    if (quantity > I2C_RX_BUF_LEN) {
        quantity = I2C_RX_BUF_LEN;
    }


    if (!sendStop) {
        flags |= I2C_M_NOSTOP;
    }

    // Setup for the transfer
    msg.frequency = _freq;
    msg.addr      = address;
    msg.flags     = (flags | I2C_M_READ);
    msg.buffer    = _rx_buf;
    msg.length    = quantity;

    // Then perform the transfer.
    ret = I2C_TRANSFER(_dev, &msg, 1);
    if (ret < 0) {
        return 0;
    }

    // set rx buffer iterator vars
    _rx_buf_index = 0;
    _rx_buf_len = quantity;

    return quantity;
}

static uint8_t i2c_transmit(uint8_t _tx_address, bool sendStop) {
    struct i2c_msg_s msg;
    unsigned int flags = 0;
    int ret;

    if (!sendStop) {
        flags |= I2C_M_NOSTOP;
    }

    // Setup for the transfer
    msg.frequency = _freq;
    msg.addr      = _tx_address;
    msg.flags     =  flags;
    msg.buffer    = _tx_buf;
    msg.length    = _tx_buf_len;

    // Then perform the transfer.
    ret = I2C_TRANSFER(_dev, &msg, 1);
     _tx_buf_len = 0;

    if (ret == -ENODEV) {
        // device not found
        return I2C_NACK_ON_ADDRESS;
    }
    return (ret < 0) ? I2C_OTHER_ERROR : I2C_SUCCESS;
}

/**
 * @brief  Write device register via I2C
 * @param  [in] i2c_addr I2C devices address
 * @param  [in] reg_addr I2C devices register address
 * @param  [in] data dete wich should be write in register
 * @retval None
 */
uint8_t i2c_write_reg(uint8_t i2c_addr, uint8_t reg_addr, uint8_t data) {
    _tx_buf[0] = reg_addr;
    _tx_buf[1] = data;
    _tx_buf_len = 2;
    
    return i2c_transmit(i2c_addr, true);
 }

/**
 * @brief  Read device register via I2C
 * @param  [in] i2c_addr I2C devices address
 * @param  [in] reg_addr I2C devices register address
 * @param  [out] data dete wich should be read from register
 * @retval None
 */
uint8_t i2c_read_reg(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *data) {
    _tx_buf[0] = reg_addr;
    _tx_buf_len = 1;

    if (data == nullptr) {
        return I2C_OTHER_ERROR;
    }
    if (i2c_transmit(i2c_addr, false) != I2C_SUCCESS) {
        return I2C_OTHER_ERROR;
    }

    i2c_request(i2c_addr, 1, true);

    *data = _rx_buf[0];
    return I2C_SUCCESS;
}

/**
 * @brief  Write device registers via I2C
 * @param  [in] i2c_addr I2C devices address
 * @param  [in] reg_addr I2C devices register address
 * @param  [in] data dete wich should be read from register
 * @param  [in] size size of data to write
 * @retval None
 */
uint8_t i2c_write_regs(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *data, uint16_t size) {
    if ((data == nullptr) || (size == 0)) {
        return I2C_OTHER_ERROR;
    }
    _tx_buf[0] = reg_addr;
    memcpy(&_tx_buf[1], data, size);
    _tx_buf_len = size + 1;
    
    return i2c_transmit(i2c_addr, true);
 }

/**
 * @brief  Read device registers via I2C
 * @param  [in] i2c_addr I2C devices address
 * @param  [in] reg_addr I2C devices register address
 * @param  [out] data dete wich should be read from register
 * @param  [in] size size of data to read
 * @retval I2C_SUCCESS (0) if ok
 */
uint8_t i2c_read_regs(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *data, uint16_t size) {
    if ((data == nullptr) || (size == 0)) {
        return I2C_OTHER_ERROR;
    }
    _tx_buf[0] = reg_addr;
    _tx_buf_len = 1;

    if (i2c_transmit(i2c_addr, false) != I2C_SUCCESS) {
        return I2C_OTHER_ERROR;
    }

    uint16_t read = i2c_request(i2c_addr, size, true);

    memcpy(data, _rx_buf, size);

    return I2C_SUCCESS;
}

/**
 * @brief  Read device data via I2C
 * @param  [in] i2c_addr I2C devices address
 * @param  [out] data dete wich should be read from register
 * @param  [in] size size of data to read
 * @retval I2C_SUCCESS (0) if ok
 */
uint8_t i2c_read_data(uint8_t i2c_addr, uint8_t *data, uint16_t size) {

    if ((data == nullptr) || (size == 0)) {
        return I2C_OTHER_ERROR;
    }
    uint16_t read = i2c_request(i2c_addr, size, true);

    memcpy(data, _rx_buf, size);

    return I2C_SUCCESS;
}

/**
 * @brief  Write device data via I2C
 * @param  [in] i2c_addr I2C devices address
 * @param  [in] data dete wich should be read from register
 * @param  [in] size size of data to write
 * @retval None
 */
uint8_t i2c_write_data(uint8_t i2c_addr, uint8_t *data, uint16_t size) {
    if ((data == nullptr) || (size == 0)) {
        return I2C_OTHER_ERROR;
    }
    memcpy(_tx_buf, data, size);
    _tx_buf_len = size;
    
    return i2c_transmit(i2c_addr, true);
}

/**
 * @brief  Write device registers via I2C with 16 bit addres
 * @param  [in] i2c_addr I2C devices address
 * @param  [in] reg_addr I2C devices register address
 * @param  [in] data dete wich should be read from register
 * @param  [in] size size of data to write
 * @retval None
 */
uint8_t i2c_write_regs_16addr(uint8_t i2c_addr, uint16_t reg_addr, uint8_t *data, uint16_t size) {
    if ((data == nullptr) || (size == 0)) {
        return I2C_OTHER_ERROR;
    }
    uint8_t reg [2] = {0};
    memcpy(&reg, &reg_addr, 2);
    _tx_buf [0] = reg[1];
    _tx_buf [1] = reg[0];
    _tx_buf_len = 2;
    memcpy(&_tx_buf[2], data, size);
    _tx_buf_len += size;
    
    return i2c_transmit(i2c_addr, true);
}

/**
 * @brief  Read device registers via I2C with 16 bit addres
 * @param  [in] i2c_addr I2C devices address
 * @param  [in] reg_addr I2C devices register address
 * @param  [out] data dete wich should be read from register
 * @param  [in] size size of data to read
 * @retval I2C_SUCCESS (0) if ok
 */
uint8_t i2c_read_regs_16addr(uint8_t i2c_addr, uint16_t reg_addr, uint8_t *data, uint16_t size) {
    if ((data == nullptr) || (size == 0)) {
        return I2C_OTHER_ERROR;
    }
    uint8_t reg [2] = {0};
    memcpy(&reg, &reg_addr, 2);
    _tx_buf [0] = reg[1];
    _tx_buf [1] = reg[0];
    _tx_buf_len = 2;

    if (i2c_transmit(i2c_addr, false) != I2C_SUCCESS) {
        return I2C_OTHER_ERROR;
    }
    uint16_t read = i2c_request(i2c_addr, size, true);
    memcpy(data, _rx_buf, size);

    return I2C_SUCCESS;
}

/**
 * @brief  Check if device present or ready
 * @param  [in] i2c_addr I2C devices address
 * @retval true if ok or ready
 */
bool i2c_is_bevice_present (uint8_t i2c_addr) {
    _tx_buf_len = 0;
    if (i2c_transmit(i2c_addr, true) != I2C_SUCCESS) {
        return false;
    }
    return true;
}
