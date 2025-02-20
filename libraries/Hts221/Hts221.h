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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HTS221_REGS_H
#define HTS221_REGS_H

#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
//#include <math.h>
#ifndef float_t
#define float_t float
#endif
/** @addtogroup HTS221
  * @{
  *
  */

/** @defgroup STMicroelectronics sensors common types
  * @{
  *
  */

#ifndef MEMS_SHARED_TYPES
#define MEMS_SHARED_TYPES

typedef struct{
  uint8_t bit0       : 1;
  uint8_t bit1       : 1;
  uint8_t bit2       : 1;
  uint8_t bit3       : 1;
  uint8_t bit4       : 1;
  uint8_t bit5       : 1;
  uint8_t bit6       : 1;
  uint8_t bit7       : 1;
} bitwise_t;

#define PROPERTY_DISABLE                (0U)
#define PROPERTY_ENABLE                 (1U)

/** @addtogroup  Interfaces_Functions
  * @brief       This section provide a set of functions used to read and
  *              write a generic register of the device.
  *              MANDATORY: return 0 -> no Error.
  * @{
  *
  */

typedef int32_t (*stmdev_write_ptr)(void *, uint8_t, uint8_t*, uint16_t);
typedef int32_t (*stmdev_read_ptr) (void *, uint8_t, uint8_t*, uint16_t);

typedef struct {
  /** Component mandatory fields **/
  stmdev_write_ptr  write_reg;
  stmdev_read_ptr   read_reg;
  /** Customizable optional pointer **/
  void *handle;
} stmdev_ctx_t;

/**
  * @}
  *
  */

#endif /* MEMS_SHARED_TYPES */

#ifndef MEMS_UCF_SHARED_TYPES
#define MEMS_UCF_SHARED_TYPES

/** @defgroup    Generic address-data structure definition
  * @brief       This structure is useful to load a predefined configuration
  *              of a sensor.
    *              You can create a sensor configuration by your own or using
    *              Unico / Unicleo tools available on STMicroelectronics
    *              web site.
  *
  * @{
  *
  */

typedef struct {
  uint8_t address;
  uint8_t data;
} ucf_line_t;

/**
  * @}
  *
  */

#endif /* MEMS_UCF_SHARED_TYPES */

/**
  * @}
  *
  */

/** @defgroup HTS221_Infos
  * @{
  *
  */

/** I2C Device Address 8 bit format  **/
#define HTS221_I2C_ADDRESS         0x5fU

/** Device Identification (Who am I) **/
#define HTS221_ID                  0xBCU

/**
  * @}
  *
  */

#define HTS221_WHO_AM_I            0x0FU
#define HTS221_AV_CONF             0x10U
typedef struct {
  uint8_t avgh                 : 3;
  uint8_t avgt                 : 3;
  uint8_t not_used_01          : 2;
} hts221_av_conf_t;

#define HTS221_CTRL_REG1           0x20U
typedef struct {
  uint8_t odr                  : 2;
  uint8_t bdu                  : 1;
  uint8_t not_used_01          : 4;
  uint8_t pd                   : 1;
} hts221_ctrl_reg1_t;

#define HTS221_CTRL_REG2           0x21U
typedef struct {
  uint8_t one_shot             : 1;
  uint8_t heater               : 1;
  uint8_t not_used_01          : 5;
  uint8_t boot                 : 1;
} hts221_ctrl_reg2_t;

#define HTS221_CTRL_REG3           0x22U
typedef struct {
  uint8_t not_used_01          : 2;
  uint8_t drdy                 : 1;
  uint8_t not_used_02          : 3;
  uint8_t pp_od                : 1;
  uint8_t drdy_h_l             : 1;
} hts221_ctrl_reg3_t;

#define HTS221_STATUS_REG          0x27U
typedef struct {
  uint8_t t_da                 : 1;
  uint8_t h_da                 : 1;
  uint8_t not_used_01          : 6;
} hts221_status_reg_t;

#define HTS221_HUMIDITY_OUT_L      0x28U
#define HTS221_HUMIDITY_OUT_H      0x29U
#define HTS221_TEMP_OUT_L          0x2AU
#define HTS221_TEMP_OUT_H          0x2BU
#define HTS221_H0_RH_X2            0x30U
#define HTS221_H1_RH_X2            0x31U
#define HTS221_T0_DEGC_X8          0x32U
#define HTS221_T1_DEGC_X8          0x33U
#define HTS221_T1_T0_MSB           0x35U
typedef struct {
  uint8_t t0_msb               : 2;
  uint8_t t1_msb               : 2;
  uint8_t not_used_01          : 4;
} hts221_t1_t0_msb_t;

#define HTS221_H0_T0_OUT_L         0x36U
#define HTS221_H0_T0_OUT_H         0x37U
#define HTS221_H1_T0_OUT_L         0x3AU
#define HTS221_H1_T0_OUT_H         0x3BU
#define HTS221_T0_OUT_L            0x3CU
#define HTS221_T0_OUT_H            0x3DU
#define HTS221_T1_OUT_L            0x3EU
#define HTS221_T1_OUT_H            0x3FU

/**
  * @defgroup HTS221_Register_Union
  * @brief    This union group all the registers that has a bitfield
  *           description.
  *           This union is useful but not need by the driver.
  *
  *           REMOVING this union you are compliant with:
  *           MISRA-C 2012 [Rule 19.2] -> " Union are not allowed "
  *
  * @{
  *
  */
typedef union{
  hts221_av_conf_t        av_conf;
  hts221_ctrl_reg1_t      ctrl_reg1;
  hts221_ctrl_reg2_t      ctrl_reg2;
  hts221_ctrl_reg3_t      ctrl_reg3;
  hts221_status_reg_t     status_reg;
  hts221_t1_t0_msb_t      t1_t0_msb;
  bitwise_t               bitwise;
  uint8_t                 byte;
} hts221_reg_t;

/**
  * @}
  *
  */

int32_t hts221_read_reg(uint8_t reg, uint8_t* data,
                        uint16_t len);
int32_t hts221_write_reg(uint8_t reg, uint8_t* data,
                         uint16_t len);

typedef enum {
  HTS221_H_AVG_4    = 0,
  HTS221_H_AVG_8    = 1,
  HTS221_H_AVG_16   = 2,
  HTS221_H_AVG_32   = 3,
  HTS221_H_AVG_64   = 4,
  HTS221_H_AVG_128  = 5,
  HTS221_H_AVG_256  = 6,
  HTS221_H_AVG_512  = 7,
  HTS221_H_AVG_ND   = 8,
  HTS221_H_AVG_ENUM
} hts221_avgh_t;
int32_t hts221_humidity_avg_set(hts221_avgh_t val);
int32_t hts221_humidity_avg_get(hts221_avgh_t *val);

typedef enum {
  HTS221_T_AVG_2   = 0,
  HTS221_T_AVG_4   = 1,
  HTS221_T_AVG_8   = 2,
  HTS221_T_AVG_16  = 3,
  HTS221_T_AVG_32  = 4,
  HTS221_T_AVG_64  = 5,
  HTS221_T_AVG_128 = 6,
  HTS221_T_AVG_256 = 7,
  HTS221_T_AVG_ND  = 8,
  HTS221_T_AVG_ENUM
} hts221_avgt_t;
int32_t hts221_temperature_avg_set(hts221_avgt_t val);
int32_t hts221_temperature_avg_get(hts221_avgt_t *val);

typedef enum {
  HTS221_ONE_SHOT  = 0,
  HTS221_ODR_1Hz   = 1,
  HTS221_ODR_7Hz   = 2,
  HTS221_ODR_12Hz5 = 3,
  HTS221_ODR_ND    = 4,
  HTS221_ENUM
} hts221_odr_t;
int32_t hts221_data_rate_set(hts221_odr_t val);
int32_t hts221_data_rate_get(hts221_odr_t *val);

int32_t hts221_block_data_update_set(uint8_t val);
int32_t hts221_block_data_update_get(uint8_t *val);

int32_t hts221_one_shoot_trigger_set(uint8_t val);
int32_t hts221_one_shoot_trigger_get(uint8_t *val);

int32_t hts221_temp_data_ready_get(uint8_t *val);

int32_t hts221_hum_data_ready_get(uint8_t *val);

int32_t hts221_humidity_raw_get(uint8_t *buff);

int32_t hts221_temperature_raw_get(uint8_t *buff);

int32_t hts221_device_id_get(uint8_t *buff);

int32_t hts221_power_on_set(uint8_t val);

int32_t hts221_power_on_get(uint8_t *val);

int32_t hts221_heater_set(uint8_t val);
int32_t hts221_heater_get(uint8_t *val);

int32_t hts221_boot_set(uint8_t val);
int32_t hts221_boot_get(uint8_t *val);

int32_t hts221_status_get(hts221_status_reg_t *val);

int32_t hts221_drdy_on_int_set(uint8_t val);
int32_t hts221_drdy_on_int_get(uint8_t *val);

typedef enum {
  HTS221_PUSH_PULL   = 0,
  
  HTS221_OPEN_DRAIN  = 1,
  HTS221_PIN_MODE_ND = 2,
} hts221_pp_od_t;
int32_t hts221_pin_mode_set(hts221_pp_od_t val);
int32_t hts221_pin_mode_get(hts221_pp_od_t *val);

typedef enum {
  HTS221_ACTIVE_HIGH = 0,
  HTS221_ACTIVE_LOW  = 1,
  HTS221_ACTIVE_ND   = 2,
} hts221_drdy_h_l_t;
int32_t hts221_int_polarity_set(hts221_drdy_h_l_t val);
int32_t hts221_int_polarity_get(hts221_drdy_h_l_t *val);

int32_t hts221_hum_rh_point_0_get(float_t *val);
int32_t hts221_hum_rh_point_1_get(float_t *val);

int32_t hts221_temp_deg_point_0_get(float_t *val);
int32_t hts221_temp_deg_point_1_get(float_t *val);

int32_t hts221_hum_adc_point_0_get(float_t *val);
int32_t hts221_hum_adc_point_1_get(float_t *val);

int32_t hts221_temp_adc_point_0_get(float_t *val);
int32_t hts221_temp_adc_point_1_get(float_t *val);

/**
 * @brief  HTS221 init
 * @param  None
 * @retval None
 */
void hts221_init(void);

/**
 * @brief  HTS221 get humidity and temperature
 * @param  [Out] humidity pointer to humidity
 * @param  [Out] temperature pointer to temperature
 * @retval None
 */
void hts221_get_hym_temp(float* humidity, float* temperature);


typedef struct {
    bool enable;
    hts221_odr_t data_rate;
    hts221_avgh_t hts221_avgh;
    hts221_avgt_t hts221_avgt;
} hts221_mode_t;

/**
 * @brief  get mesure pheriod [ns]
 * @param  [in] rate measure rate from hts221_odr_t
 * @retval measure pheriod [ns] 0 if off, 0xffffffffffffffff in other caces
 */
uint64_t hts221_get_pheriod_ns(hts221_odr_t rate);

/**
 * @brief  get hts221 current mode
 * @param  [out] mode from hts221_mode_t
 * @retval none
 */
void hts221_get_mode(hts221_mode_t* mode);

/**
 * @brief  set hts221 current mode
 * @param  [in] mode from hts221_mode_t
 * @retval none
 */
void hts221_set_mode(hts221_mode_t* mode);

/**
 * @brief  enable hts221
 * @param  None
 * @retval None
 */
void hts221_enable(void);

/**
 * @brief  disable hts221
 * @param  None
 * @retval None
 */
void hts221_disable(void);

/**
  * @}
  *
  */

#ifdef __cplusplus
}
#endif

#endif /*HTS221_REGS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
