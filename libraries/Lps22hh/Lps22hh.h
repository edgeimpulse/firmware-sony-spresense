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
#ifndef LPS22HH_REGS_H
#define LPS22HH_REGS_H

#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#ifndef float_t
#define float_t float
#endif

/** @addtogroup LPS22HH
  * @{
  *
  */

/** @defgroup STMicroelectronics sensors common types
  * @{
  *
  */

#ifndef MEMS_SHARED_TYPES
#define MEMS_SHARED_TYPES

typedef struct {
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

/** @defgroup LPS22HH_Infos
  * @{
  *
  */

/** I2C Device Address 8 bit **/

#define LPS22HH_I2C_ADD                         0x5dU

/** I2C Device Address 8 bit format  if SA0=0 -> B9 if SA0=1 -> BB **/
#define LPS22HH_I2C_ADD_H                       0xBBU
#define LPS22HH_I2C_ADD_L                       0xB9U

/** Device Identification (Who am I) **/
#define LPS22HH_ID                              0xB3U

/**
  * @}
  *
  */

#define LPS22HH_INTERRUPT_CFG                   0x0BU
typedef struct {
    uint8_t pe                              : 2;  /* ple + phe */
    uint8_t lir                             : 1;
    uint8_t diff_en                         : 1;
    uint8_t reset_az                        : 1;
    uint8_t autozero                        : 1;
    uint8_t reset_arp                       : 1;
    uint8_t autorefp                        : 1;
} lps22hh_interrupt_cfg_t;

#define LPS22HH_THS_P_L                         0x0CU
typedef struct {
    uint8_t ths                             : 8;
} lps22hh_ths_p_l_t;

#define LPS22HH_THS_P_H                         0x0DU
typedef struct {
    uint8_t ths                             : 7;
    uint8_t not_used_01                     : 1;
} lps22hh_ths_p_h_t;

#define LPS22HH_IF_CTRL                         0x0EU
typedef struct {
    uint8_t i2c_disable                     : 1;
    uint8_t i3c_disable                     : 1;
    uint8_t pd_dis_int1                     : 1;
    uint8_t sdo_pu_en                       : 1;
    uint8_t sda_pu_en                       : 1;
    uint8_t not_used_01                     : 2;
    uint8_t int_en_i3c                      : 1;
} lps22hh_if_ctrl_t;

#define LPS22HH_WHO_AM_I                        0x0FU
#define LPS22HH_CTRL_REG1                       0x10U
typedef struct {
    uint8_t sim                             : 1;
    uint8_t bdu                             : 1;
    uint8_t lpfp_cfg                        : 2;  /* en_lpfp + lpfp_cfg */
    uint8_t odr                             : 3;
    uint8_t not_used_01                     : 1;
} lps22hh_ctrl_reg1_t;

#define LPS22HH_CTRL_REG2                       0x11U
typedef struct {
    uint8_t one_shot                        : 1;
    uint8_t low_noise_en                    : 1;
    uint8_t swreset                         : 1;
    uint8_t not_used_01                     : 1;
    uint8_t if_add_inc                      : 1;
    uint8_t pp_od                           : 1;
    uint8_t int_h_l                         : 1;
    uint8_t boot                            : 1;
} lps22hh_ctrl_reg2_t;

#define LPS22HH_CTRL_REG3                       0x12U
typedef struct {
    uint8_t int_s                           : 2;
    uint8_t drdy                            : 1;
    uint8_t int_f_ovr                       : 1;
    uint8_t int_f_wtm                       : 1;
    uint8_t int_f_full                      : 1;
    uint8_t not_used_01                     : 2;
} lps22hh_ctrl_reg3_t;

#define LPS22HH_FIFO_CTRL                       0x13U
typedef struct {
    uint8_t f_mode                          : 3;  /* f_mode + trig_modes */
    uint8_t stop_on_wtm                     : 1;
    uint8_t not_used_01                     : 4;
} lps22hh_fifo_ctrl_t;

#define LPS22HH_FIFO_WTM                        0x14U
typedef struct {
    uint8_t wtm                             : 7;
    uint8_t not_used_01                     : 1;
} lps22hh_fifo_wtm_t;

#define LPS22HH_REF_P_L                         0x15U
#define LPS22HH_REF_P_H                         0x16U
#define LPS22HH_RPDS_L                          0x18U
#define LPS22HH_RPDS_H                          0x19U
#define LPS22HH_INT_SOURCE                      0x24U
typedef struct {
    uint8_t ph                              : 1;
    uint8_t pl                              : 1;
    uint8_t ia                              : 1;
    uint8_t not_used_01                     : 4;
    uint8_t boot_on                         : 1;
} lps22hh_int_source_t;

#define LPS22HH_FIFO_STATUS1                    0x25U
#define LPS22HH_FIFO_STATUS2                    0x26U
typedef struct {
    uint8_t not_used_01                     : 5;
    uint8_t fifo_full_ia                    : 1;
    uint8_t fifo_ovr_ia                     : 1;
    uint8_t fifo_wtm_ia                     : 1;
} lps22hh_fifo_status2_t;

#define LPS22HH_STATUS                          0x27U
typedef struct {
    uint8_t p_da                            : 1;
    uint8_t t_da                            : 1;
    uint8_t not_used_01                     : 2;
    uint8_t p_or                            : 1;
    uint8_t t_or                            : 1;
    uint8_t not_used_02                     : 2;
} lps22hh_status_t;

#define LPS22HH_PRESS_OUT_XL                    0x28U
#define LPS22HH_PRESS_OUT_L                     0x29U
#define LPS22HH_PRESS_OUT_H                     0x2AU
#define LPS22HH_TEMP_OUT_L                      0x2BU
#define LPS22HH_TEMP_OUT_H                      0x2CU
#define LPS22HH_FIFO_DATA_OUT_PRESS_XL          0x78U
#define LPS22HH_FIFO_DATA_OUT_PRESS_L           0x79U
#define LPS22HH_FIFO_DATA_OUT_PRESS_H           0x7AU
#define LPS22HH_FIFO_DATA_OUT_TEMP_L            0x7BU
#define LPS22HH_FIFO_DATA_OUT_TEMP_H            0x7CU

/**
  * @defgroup LPS22HH_Register_Union
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
typedef union {
    lps22hh_interrupt_cfg_t        interrupt_cfg;
    lps22hh_if_ctrl_t              if_ctrl;
    lps22hh_ctrl_reg1_t            ctrl_reg1;
    lps22hh_ctrl_reg2_t            ctrl_reg2;
    lps22hh_ctrl_reg3_t            ctrl_reg3;
    lps22hh_fifo_ctrl_t            fifo_ctrl;
    lps22hh_fifo_wtm_t             fifo_wtm;
    lps22hh_int_source_t           int_source;
    lps22hh_fifo_status2_t         fifo_status2;
    lps22hh_status_t               status;
    bitwise_t                      bitwise;
    uint8_t                        byte;
} lps22hh_reg_t;

/**
  * @}
  *
  */

int32_t lps22hh_read_reg(stmdev_ctx_t *ctx, uint8_t reg, uint8_t* data, uint16_t len);
int32_t lps22hh_write_reg(stmdev_ctx_t *ctx, uint8_t reg, uint8_t* data, uint16_t len);

extern float_t lps22hh_from_lsb_to_hpa(int32_t lsb);
extern float_t lps22hh_from_lsb_to_celsius(int16_t lsb);

int32_t lps22hh_autozero_rst_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps22hh_autozero_rst_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps22hh_autozero_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps22hh_autozero_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps22hh_pressure_snap_rst_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps22hh_pressure_snap_rst_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps22hh_pressure_snap_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps22hh_pressure_snap_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps22hh_block_data_update_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps22hh_block_data_update_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
    LPS22HH_POWER_DOWN          = 0x00,
    LPS22HH_ONE_SHOOT           = 0x08,
    LPS22HH_1_Hz                = 0x01,
    LPS22HH_10_Hz               = 0x02,
    LPS22HH_25_Hz               = 0x03,
    LPS22HH_50_Hz               = 0x04,
    LPS22HH_75_Hz               = 0x05,
    LPS22HH_1_Hz_LOW_NOISE      = 0x11,
    LPS22HH_10_Hz_LOW_NOISE     = 0x12,
    LPS22HH_25_Hz_LOW_NOISE     = 0x13,
    LPS22HH_50_Hz_LOW_NOISE     = 0x14,
    LPS22HH_75_Hz_LOW_NOISE     = 0x15,
    LPS22HH_100_Hz              = 0x06,
    LPS22HH_200_Hz              = 0x07,
    LPS22HH_ODR_ENUM
} lps22hh_odr_t;
int32_t lps22hh_data_rate_set(stmdev_ctx_t *ctx, lps22hh_odr_t val);
int32_t lps22hh_data_rate_get(stmdev_ctx_t *ctx, lps22hh_odr_t *val);

int32_t lps22hh_pressure_ref_set(stmdev_ctx_t *ctx, uint8_t *buff);
int32_t lps22hh_pressure_ref_get(stmdev_ctx_t *ctx, uint8_t *buff);

int32_t lps22hh_pressure_offset_set(stmdev_ctx_t *ctx, uint8_t *buff);
int32_t lps22hh_pressure_offset_get(stmdev_ctx_t *ctx, uint8_t *buff);

typedef struct{
    lps22hh_int_source_t    int_source;
    lps22hh_fifo_status2_t  fifo_status2;
    lps22hh_status_t        status;
} lps22hh_all_sources_t;
int32_t lps22hh_all_sources_get(stmdev_ctx_t *ctx, lps22hh_all_sources_t *val);

int32_t lps22hh_status_reg_get(stmdev_ctx_t *ctx, lps22hh_status_t *val);

int32_t lps22hh_press_flag_data_ready_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps22hh_temp_flag_data_ready_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps22hh_pressure_raw_get(stmdev_ctx_t *ctx, uint8_t *buff);

int32_t lps22hh_temperature_raw_get(stmdev_ctx_t *ctx, uint8_t *buff);

int32_t lps22hh_fifo_pressure_raw_get(stmdev_ctx_t *ctx, uint8_t *buff);

int32_t lps22hh_fifo_temperature_raw_get(stmdev_ctx_t *ctx, uint8_t *buff);

int32_t lps22hh_device_id_get(stmdev_ctx_t *ctx, uint8_t *buff);

int32_t lps22hh_reset_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps22hh_reset_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps22hh_auto_increment_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps22hh_auto_increment_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps22hh_boot_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps22hh_boot_get(stmdev_ctx_t *ctx, uint8_t *val);

typedef enum {
    LPS22HH_LPF_ODR_DIV_2    = 0,
    LPS22HH_LPF_ODR_DIV_9    = 2,
    LPS22HH_LPF_ODR_DIV_20   = 3,
} lps22hh_lpfp_cfg_t;
int32_t lps22hh_lp_bandwidth_set(stmdev_ctx_t *ctx, lps22hh_lpfp_cfg_t val);
int32_t lps22hh_lp_bandwidth_get(stmdev_ctx_t *ctx, lps22hh_lpfp_cfg_t *val);

typedef enum {
    LPS22HH_I2C_ENABLE    = 0,
    LPS22HH_I2C_DISABLE   = 1,
} lps22hh_i2c_disable_t;
int32_t lps22hh_i2c_interface_set(stmdev_ctx_t *ctx, lps22hh_i2c_disable_t val);
int32_t lps22hh_i2c_interface_get(stmdev_ctx_t *ctx, lps22hh_i2c_disable_t *val);

typedef enum {
    LPS22HH_I3C_ENABLE                 = 0x00,
    LPS22HH_I3C_ENABLE_INT_PIN_ENABLE  = 0x10,
    LPS22HH_I3C_DISABLE                = 0x11,
} lps22hh_i3c_disable_t;
int32_t lps22hh_i3c_interface_set(stmdev_ctx_t *ctx, lps22hh_i3c_disable_t val);
int32_t lps22hh_i3c_interface_get(stmdev_ctx_t *ctx, lps22hh_i3c_disable_t *val);

typedef enum {
    LPS22HH_PULL_UP_DISCONNECT    = 0,
    LPS22HH_PULL_UP_CONNECT       = 1,
} lps22hh_pu_en_t;
int32_t lps22hh_sdo_sa0_mode_set(stmdev_ctx_t *ctx, lps22hh_pu_en_t val);
int32_t lps22hh_sdo_sa0_mode_get(stmdev_ctx_t *ctx, lps22hh_pu_en_t *val);
int32_t lps22hh_sda_mode_set(stmdev_ctx_t *ctx, lps22hh_pu_en_t val);
int32_t lps22hh_sda_mode_get(stmdev_ctx_t *ctx, lps22hh_pu_en_t *val);

typedef enum {
    LPS22HH_SPI_4_WIRE  = 0,
    LPS22HH_SPI_3_WIRE  = 1,
} lps22hh_sim_t;
int32_t lps22hh_spi_mode_set(stmdev_ctx_t *ctx, lps22hh_sim_t val);
int32_t lps22hh_spi_mode_get(stmdev_ctx_t *ctx, lps22hh_sim_t *val);

typedef enum {
    LPS22HH_INT_PULSED   = 0,
    LPS22HH_INT_LATCHED  = 1,
} lps22hh_lir_t;
int32_t lps22hh_int_notification_set(stmdev_ctx_t *ctx, lps22hh_lir_t val);
int32_t lps22hh_int_notification_get(stmdev_ctx_t *ctx, lps22hh_lir_t *val);

typedef enum {
    LPS22HH_PUSH_PULL   = 0,
    LPS22HH_OPEN_DRAIN  = 1,
} lps22hh_pp_od_t;
int32_t lps22hh_pin_mode_set(stmdev_ctx_t *ctx, lps22hh_pp_od_t val);
int32_t lps22hh_pin_mode_get(stmdev_ctx_t *ctx, lps22hh_pp_od_t *val);

typedef enum {
    LPS22HH_ACTIVE_HIGH = 0,
    LPS22HH_ACTIVE_LOW  = 1,
} lps22hh_int_h_l_t;
int32_t lps22hh_pin_polarity_set(stmdev_ctx_t *ctx, lps22hh_int_h_l_t val);
int32_t lps22hh_pin_polarity_get(stmdev_ctx_t *ctx, lps22hh_int_h_l_t *val);

int32_t lps22hh_pin_int_route_set(stmdev_ctx_t *ctx, lps22hh_ctrl_reg3_t *val);
int32_t lps22hh_pin_int_route_get(stmdev_ctx_t *ctx, lps22hh_ctrl_reg3_t *val);

typedef enum {
    LPS22HH_NO_THRESHOLD  = 0,
    LPS22HH_POSITIVE      = 1,
    LPS22HH_NEGATIVE      = 2,
    LPS22HH_BOTH          = 3,
} lps22hh_pe_t;
int32_t lps22hh_int_on_threshold_set(stmdev_ctx_t *ctx, lps22hh_pe_t val);
int32_t lps22hh_int_on_threshold_get(stmdev_ctx_t *ctx, lps22hh_pe_t *val);

int32_t lps22hh_int_treshold_set(stmdev_ctx_t *ctx, uint16_t buff);
int32_t lps22hh_int_treshold_get(stmdev_ctx_t *ctx, uint16_t *buff);

typedef enum {
    LPS22HH_BYPASS_MODE            = 0,
    LPS22HH_FIFO_MODE              = 1,
    LPS22HH_STREAM_MODE            = 2,
    LPS22HH_DYNAMIC_STREAM_MODE    = 3,
    LPS22HH_BYPASS_TO_FIFO_MODE    = 5,
    LPS22HH_BYPASS_TO_STREAM_MODE  = 6,
    LPS22HH_STREAM_TO_FIFO_MODE    = 7,
} lps22hh_f_mode_t;
int32_t lps22hh_fifo_mode_set(stmdev_ctx_t *ctx, lps22hh_f_mode_t val);
int32_t lps22hh_fifo_mode_get(stmdev_ctx_t *ctx, lps22hh_f_mode_t *val);

int32_t lps22hh_fifo_stop_on_wtm_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps22hh_fifo_stop_on_wtm_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps22hh_fifo_watermark_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps22hh_fifo_watermark_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps22hh_fifo_data_level_get(stmdev_ctx_t *ctx, uint8_t *buff);

int32_t lps22hh_fifo_src_get(stmdev_ctx_t *ctx, lps22hh_fifo_status2_t *val);

int32_t lps22hh_fifo_full_flag_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps22hh_fifo_ovr_flag_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps22hh_fifo_wtm_flag_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps22hh_fifo_ovr_on_int_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps22hh_fifo_ovr_on_int_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps22hh_fifo_threshold_on_int_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps22hh_fifo_threshold_on_int_get(stmdev_ctx_t *ctx, uint8_t *val);

int32_t lps22hh_fifo_full_on_int_set(stmdev_ctx_t *ctx, uint8_t val);
int32_t lps22hh_fifo_full_on_int_get(stmdev_ctx_t *ctx, uint8_t *val);

/**
 * @brief  LPS22HH init
 * @param  None
 * @retval true if ok
 */
bool lps22hh_init (void);

/**
 * @brief  LPS22HH get pressure in hPa
 * @param  None
 * @retval pressure [hPa] or 0 if data doesn`t ready
 */
float lps22hh_get_pressure_hpa (void);


typedef struct {
    bool enable;
    lps22hh_odr_t data_rate;
} lps22hh_mode_t;

/**
 * @brief  get mesure pheriod [ns]
 * @param  [in] rate measure rate from lps22hh_odr_t
 * @retval measure pheriod [ns] 0 if off, 0xffffffffffffffff in other caces
 */
uint64_t lps22hh_get_pheriod_ns(lps22hh_odr_t rate);

/**
 * @brief  get lps22hh current mode
 * @param  [out] mode from lps22hh_mode_t
 * @retval none
 */
void lps22hh_get_mode(lps22hh_mode_t* mode);

/**
 * @brief  set lps22hh current mode
 * @param  [in] mode from lps22hh_mode_t
 * @retval none
 */
void lps22hh_set_mode(lps22hh_mode_t* mode);

/**
 * @brief  enable lps22hh
 * @param  None
 * @retval None
 */
void lps22hh_enable(void);

/**
 * @brief  disable lps22hh
 * @param  None
 * @retval None
 */
void lps22hh_disable(void);


/**
  * @}
  *
  */

#ifdef __cplusplus
}
#endif

#endif /*LPS22HH_REGS_H */

