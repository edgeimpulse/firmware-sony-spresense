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


#ifndef SGP40_I2C_H
#define SGP40_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "sensirion_config.h"
#include <stdint.h>

#define SGP40_DEFAULT_HUMIDITY 0x8000 // 50%RH
#define SGP40_DEFAULT_TEMPERATURE 0x6666 // 25 degC

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
int16_t sgp40_measure_raw_signal(uint16_t relative_humidity, uint16_t temperature, uint16_t* sraw_voc);

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
int16_t sgp40_execute_self_test(uint16_t* test_result);

/**
 * sgp40_turn_heater_off() - This command turns the hotplate off and stops the
 * measurement. Subsequently, the sensor enters the idle mode.
 *
 * @return 0 on success, an error code otherwise
 */
int16_t sgp40_turn_heater_off(void);

/**
 * sgp40_get_serial_number() - This command provides the decimal serial number
 * of the SGP40 chip by returning 3x2 bytes.
 *
 * @param serial_number 48-bit unique serial number
 *
 * @return 0 on success, an error code otherwise
 */
int16_t sgp40_get_serial_number(uint16_t* serial_number, uint8_t serial_number_size);

/**
 * @brief  Calculate humidity tics from humidity [%]
 * @param  [in] rh humidity from 0 to 100 [%]
 * @retval humidity in tics
 */
uint16_t sgp40_get_rh_tick(float rh);

/**
 * @brief  Calculate temerature tics from temerature [degC]
 * @param  [in] t temerature from [degC]
 * @retval temerature in tics
 */
uint16_t sgp40_get_t_tick(float t);

typedef enum {
    SGP40_1S,
    SGP40_2S,
    SGP40_5S,
    SGP40_10S,
    SGP40_ENUM
} sgp40_period_t;

typedef enum {
    SGP40_SOURCE_DEFAULT,
    SGP40_SOURCE_HTS221,
    SGP40_SOURCE_ENUM
} sgp40_source_t;

/**
 * @brief  sgp40 init
 * @param  none
 * @retval true if ok
 */
bool sgp40_init(void);

typedef struct {
    bool enable;
    sgp40_source_t sgp40_source;
    sgp40_period_t sgp40_period;
} sgp40_mode_t;

/**
 * @brief  get mesure pheriod [ns]
 * @param  [in] rate measure rate from sgp40_period_t
 * @retval measure pheriod [ns] 0 if off, 0xffffffffffffffff in other caces
 */
uint64_t sgp40_get_pheriod_ns(sgp40_period_t rate);

/**
 * @brief  get sgp40 current mode
 * @param  [out] mode from sgp40_mode_t
 * @retval none
 */
void sgp40_get_mode(sgp40_mode_t* mode);

/**
 * @brief  set sgp40 current mode
 * @param  [in] mode from sgp40_mode_t
 * @retval none
 */
void sgp40_set_mode(sgp40_mode_t* mode);

/**
 * @brief  enable sgp40
 * @param  None
 * @retval None
 */
void sgp40_enable(void);

/**
 * @brief  disable sgp40
 * @param  None
 * @retval None
 */
void sgp40_disable(void);

#ifdef __cplusplus
}
#endif

#endif /* SGP40_I2C_H */
