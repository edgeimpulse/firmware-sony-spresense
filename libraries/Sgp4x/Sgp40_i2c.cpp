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


#include <string.h>
#include "sgp40_i2c.h"
#include "sensirion_common.h"
#include "sensirion_i2c.h"
#include "sensirion_i2c_hal.h"
#include "../I2c/I2c.h"

#define SGP40_I2C_ADDRESS 0x59

sgp40_mode_t sgp40_mode = {0};

/**
 * sgp40_measure_raw_signal() - This command starts/continues the VOC
 * measurement mode
 *
 * @param relative_humidity Leaves humidity compensation disabled by sending the
 * default value 0x8000 (50%RH) or enables humidity compensation when sending
 * the relative humidity in ticks (ticks = %RH * 65535 / 100)
 *
 * @param temperature Leaves humidity compensation disabled by sending the
 * default value 0x6666 (25 degC) or enables humidity compensation when sending
 * the temperature in ticks (ticks = (degC + 45) * 65535 / 175)
 *
 * @param sraw_voc u16 unsigned integer directly provides the raw signal
 * SRAW_VOC in ticks which is proportional to the logarithm of the resistance of
 * the sensing element.
 *
 * @return 0 on success, an error code otherwise
 */
int16_t sgp40_measure_raw_signal(uint16_t relative_humidity, uint16_t temperature, uint16_t* sraw_voc) {
    if (sraw_voc == nullptr) {
        return ~NO_ERROR;
    }
    int16_t error;
    uint8_t buffer[8];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0x260F);

    offset = sensirion_i2c_add_uint16_t_to_buffer(&buffer[0], offset,
                                                  relative_humidity);
    offset =
        sensirion_i2c_add_uint16_t_to_buffer(&buffer[0], offset, temperature);

    error = sensirion_i2c_write_data(SGP40_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }

    sensirion_i2c_hal_sleep_usec(30000);

    error = sensirion_i2c_read_data_inplace(SGP40_I2C_ADDRESS, &buffer[0], 2);
    if (error) {
        return error;
    }
    *sraw_voc = sensirion_common_bytes_to_uint16_t(&buffer[0]);
    return NO_ERROR;
}

/**
 * sgp40_execute_self_test() - This command triggers the built-in self-test
 * checking for integrity of the hotplate and MOX material and returns the
 * result of this test as 2 bytes
 *
 * @param test_result 0xD4 00: all tests passed successfully or 0x4B 00: one or
 * more tests have failed
 *
 * @return 0 on success, an error code otherwise
 */
int16_t sgp40_execute_self_test(uint16_t* test_result) {
    if (test_result == nullptr) {
        return ~NO_ERROR;
    }
    int16_t error;
    uint8_t buffer[3];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0x280E);

    error = sensirion_i2c_write_data(SGP40_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }

    sensirion_i2c_hal_sleep_usec(320000);

    error = sensirion_i2c_read_data_inplace(SGP40_I2C_ADDRESS, &buffer[0], 2);
    if (error) {
        return error;
    }
    *test_result = sensirion_common_bytes_to_uint16_t(&buffer[0]);
    return NO_ERROR;
}

/**
 * sgp40_turn_heater_off() - This command turns the hotplate off and stops the
 * measurement. Subsequently, the sensor enters the idle mode.
 *
 * @return 0 on success, an error code otherwise
 */
int16_t sgp40_turn_heater_off(void) {
    int16_t error;
    uint8_t buffer[2];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0x3615);

    error = sensirion_i2c_write_data(SGP40_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }
    sensirion_i2c_hal_sleep_usec(1000);
    return NO_ERROR;
}

/**
 * sgp40_get_serial_number() - This command provides the decimal serial number
 * of the SGP40 chip by returning 3x2 bytes.
 *
 * @param serial_number 48-bit unique serial number
 *
 * @return 0 on success, an error code otherwise
 */
int16_t sgp40_get_serial_number(uint16_t* serial_number, uint8_t serial_number_size) {
    if (serial_number == nullptr) {
        return ~NO_ERROR;
    }
    int16_t error;
    uint8_t buffer[9];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0x3682);

    error = sensirion_i2c_write_data(SGP40_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }

    sensirion_i2c_hal_sleep_usec(1000);

    error = sensirion_i2c_read_data_inplace(SGP40_I2C_ADDRESS, &buffer[0], 6);
    if (error) {
        return error;
    }

    serial_number[0] = sensirion_common_bytes_to_uint16_t(&buffer[0]);
    serial_number[1] = sensirion_common_bytes_to_uint16_t(&buffer[2]);
    serial_number[2] = sensirion_common_bytes_to_uint16_t(&buffer[4]);

    return NO_ERROR;
}

/**
 * @brief  Calculate humidity tics from humidity [%]
 * @param  [in] rh humidity from 0 to 100 [%]
 * @retval humidity in tics
 */
uint16_t sgp40_get_rh_tick(float rh) {
    float result = (rh * 65535.0) / 100.0;
    return (uint16_t) result;
}

/**
 * @brief  Calculate temerature tics from temerature [degC]
 * @param  [in] t temerature from [degC]
 * @retval temerature in tics
 */
uint16_t sgp40_get_t_tick(float t) {
    float result = ((t + 45.0) * 65535.0) / 175.0;
    return (uint16_t) result;
}

/**
 * @brief  sgp40 init
 * @param  none
 * @retval true if ok
 */
bool sgp40_init(void) {
    int16_t error = 0;
    uint16_t serial_number[3];
    uint8_t serial_number_size = 3;
    i2c_init();
    error = sgp40_get_serial_number(serial_number, serial_number_size);

    if (error) {
        return false;
    }
    return true;
}

/**
 * @brief  get mesure pheriod [ns]
 * @param  [in] rate measure rate from sgp40_period_t
 * @retval measure pheriod [ns], 0xffffffffffffffff in other caces
 */
uint64_t sgp40_get_pheriod_ns(sgp40_period_t rate) {
    switch (rate) {
        case SGP40_1S:
        return 1 * 1000000000;
        case SGP40_2S:
        return 2 * 1000000000;
        case SGP40_5S:
        return 5 * 1000000000;
        case SGP40_10S:
        return 10 * 1000000000;
        default:
        break;
    }
    return 0xffffffffffffffff;
}

/**
 * @brief  get sgp40 current mode
 * @param  [out] mode from sgp40_mode_t
 * @retval none
 */
void sgp40_get_mode(sgp40_mode_t* mode) {
    if (mode == nullptr) {
        return;
    }
    memcpy(mode, &sgp40_mode, sizeof(sgp40_mode_t));
}

/**
 * @brief  set sgp40 current mode
 * @param  [in] mode from sgp40_mode_t
 * @retval none
 */
void sgp40_set_mode(sgp40_mode_t* mode) {
    if (mode == nullptr) {
        return;
    }
    memcpy(&sgp40_mode, mode, sizeof(sgp40_mode_t));
    if (sgp40_mode.enable != false) {
        sgp40_enable();
    } else {
        sgp40_disable();
    }
}

/**
 * @brief  enable sgp40
 * @param  None
 * @retval None
 */
void sgp40_enable(void) {
    sgp40_mode.enable = true;
}

/**
 * @brief  disable sgp40
 * @param  None
 * @retval None
 */
void sgp40_disable(void) {
    sgp40_mode.enable = false;
}
