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


/**
 * @file  vl53l1x_api.h
 * @brief Functions definition
 */

#ifndef _API_H_
#define _API_H_

#include "vl53l1_platform.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define VL53L1X_IMPLEMENTATION_VER_MAJOR       3
#define VL53L1X_IMPLEMENTATION_VER_MINOR       3
#define VL53L1X_IMPLEMENTATION_VER_SUB         0
#define VL53L1X_IMPLEMENTATION_VER_REVISION  0000

typedef int8_t VL53L1X_ERROR;

#define SOFT_RESET											0x0000
#define VL53L1_I2C_SLAVE__DEVICE_ADDRESS					0x0001
#define VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND        0x0008
#define ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS 		0x0016
#define ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS 	0x0018
#define ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS 	0x001A
#define ALGO__PART_TO_PART_RANGE_OFFSET_MM					0x001E
#define MM_CONFIG__INNER_OFFSET_MM							0x0020
#define MM_CONFIG__OUTER_OFFSET_MM 							0x0022
#define GPIO_HV_MUX__CTRL									0x0030
#define GPIO__TIO_HV_STATUS       							0x0031
#define SYSTEM__INTERRUPT_CONFIG_GPIO 						0x0046
#define PHASECAL_CONFIG__TIMEOUT_MACROP     				0x004B
#define RANGE_CONFIG__TIMEOUT_MACROP_A_HI   				0x005E
#define RANGE_CONFIG__VCSEL_PERIOD_A        				0x0060
#define RANGE_CONFIG__VCSEL_PERIOD_B						0x0063
#define RANGE_CONFIG__TIMEOUT_MACROP_B_HI  					0x0061
#define RANGE_CONFIG__TIMEOUT_MACROP_B_LO  					0x0062
#define RANGE_CONFIG__SIGMA_THRESH 							0x0064
#define RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS			0x0066
#define RANGE_CONFIG__VALID_PHASE_HIGH      				0x0069
#define VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD				0x006C
#define SYSTEM__THRESH_HIGH 								0x0072
#define SYSTEM__THRESH_LOW 									0x0074
#define SD_CONFIG__WOI_SD0                  				0x0078
#define SD_CONFIG__INITIAL_PHASE_SD0        				0x007A
#define ROI_CONFIG__USER_ROI_CENTRE_SPAD					0x007F
#define ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE		0x0080
#define SYSTEM__SEQUENCE_CONFIG								0x0081
#define VL53L1_SYSTEM__GROUPED_PARAMETER_HOLD 				0x0082
#define SYSTEM__INTERRUPT_CLEAR       						0x0086
#define SYSTEM__MODE_START                 					0x0087
#define VL53L1_RESULT__RANGE_STATUS							0x0089
#define VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0		0x008C
#define RESULT__AMBIENT_COUNT_RATE_MCPS_SD					0x0090
#define VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0				0x0096
#define VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0 	0x0098
#define VL53L1_RESULT__OSC_CALIBRATE_VAL					0x00DE
#define VL53L1_FIRMWARE__SYSTEM_STATUS                      0x00E5
#define VL53L1_IDENTIFICATION__MODEL_ID                     0x010F
#define VL53L1_ROI_CONFIG__MODE_ROI_CENTRE_SPAD				0x013E

/****************************************
 * PRIVATE define do not edit
 ****************************************/

/**
 *  @brief defines SW Version
 */
typedef struct
{
    uint8_t major; /*!< major number */
    uint8_t minor; /*!< minor number */
    uint8_t build; /*!< build number */
    uint32_t revision; /*!< revision number */
} VL53L1X_Version_t;

/**
 *  @brief defines packed reading results type
 */
typedef struct
{
    uint8_t Status; /*!< ResultStatus */
    uint16_t Distance; /*!< ResultDistance */
    uint16_t Ambient; /*!< ResultAmbient */
    uint16_t SigPerSPAD;/*!< ResultSignalPerSPAD */
    uint16_t NumSPADs; /*!< ResultNumSPADs */
} VL53L1X_Result_t;

/**
 * @brief This function returns the SW driver version
 */
VL53L1X_ERROR vl53l1x_get_sw_version (VL53L1X_Version_t *pVersion);

/**
 * @brief This function sets the sensor I2C address used in case multiple devices application, default address 0x52
 */
VL53L1X_ERROR vl53l1x_set_i2c_address (uint16_t, uint8_t new_address);

/**
 * @brief This function loads the 135 bytes default values to initialize the sensor.
 * @param dev Device address
 * @return 0:success, != 0:failed
 */
VL53L1X_ERROR vl53l1x_sensor_init (uint16_t dev);

/**
 * @brief This function clears the interrupt, to be called after a ranging data reading
 * to arm the interrupt for the next data ready event.
 */
VL53L1X_ERROR vl53l1x_clear_interrupt (uint16_t dev);

/**
 * @brief This function programs the interrupt polarity\n
 * 1=active high (default), 0=active low
 */
VL53L1X_ERROR vl53l1x_setInterrupt_polarity (uint16_t dev, uint8_t IntPol);

/**
 * @brief This function returns the current interrupt polarity\n
 * 1=active high (default), 0=active low
 */
VL53L1X_ERROR vl53l1x_getInterrupt_polarity (uint16_t dev, uint8_t *pIntPol);

/**
 * @brief This function starts the ranging distance operation\n
 * The ranging operation is continuous. The clear interrupt has to be done after each get data to allow the interrupt to raise when the next data is ready\n
 * 1=active high (default), 0=active low, use SetInterruptPolarity() to change the interrupt polarity if required.
 */
VL53L1X_ERROR vl53l1x_start_ranging (uint16_t dev);

/**
 * @brief This function stops the ranging.
 */
VL53L1X_ERROR vl53l1x_stop_ranging (uint16_t dev);

/**
 * @brief This function checks if the new ranging data is available by polling the dedicated register.
 * @param : isDataReady==0 -> not ready; isDataReady==1 -> ready
 */
VL53L1X_ERROR vl53l1x_check_for_data_ready (uint16_t dev, uint8_t *isDataReady);

/**
 * @brief This function programs the timing budget in ms.
 * Predefined values = 15, 20, 33, 50, 100(default), 200, 500.
 */
VL53L1X_ERROR vl53l1x_set_timing_budgetIn_ms (uint16_t dev, uint16_t TimingBudgetInMs);

/**
 * @brief This function returns the current timing budget in ms.
 */
VL53L1X_ERROR vl53l1x_get_timing_budgetIn_ms (uint16_t dev, uint16_t *pTimingBudgetInMs);

/**
 * @brief This function programs the distance mode (1=short, 2=long(default)).
 * Short mode max distance is limited to 1.3 m but better ambient immunity.\n
 * Long mode can range up to 4 m in the dark with 200 ms timing budget.
 */
VL53L1X_ERROR vl53l1x_set_distance_mode (uint16_t dev, uint16_t DistanceMode);

/**
 * @brief This function returns the current distance mode (1=short, 2=long).
 */
VL53L1X_ERROR vl53l1x_get_distance_mode (uint16_t dev, uint16_t *pDistanceMode);

/**
 * @brief This function programs the Intermeasurement period in ms\n
 * Intermeasurement period must be >/= timing budget. This condition is not checked by the API,
 * the customer has the duty to check the condition. Default = 100 ms
 */
VL53L1X_ERROR vl53l1x_set_inter_measurementIn_ms (uint16_t dev,
        uint32_t InterMeasurementInMs);

/**
 * @brief This function returns the Intermeasurement period in ms.
 */
VL53L1X_ERROR vl53l1x_get_inter_measurementIn_ms (uint16_t dev, uint16_t * pIM);

/**
 * @brief This function returns the boot state of the device (1:booted, 0:not booted)
 */
VL53L1X_ERROR vl53l1x_boot_state (uint16_t dev, uint8_t *state);

/**
 * @brief This function returns the sensor id, sensor Id must be 0xEEAC
 */
VL53L1X_ERROR vl53l1x_get_sensorId (uint16_t dev, uint16_t *id);

/**
 * @brief This function returns the distance measured by the sensor in mm
 */
VL53L1X_ERROR vl53l1x_get_distance (uint16_t dev, uint16_t *distance);

/**
 * @brief This function returns the returned signal per SPAD in kcps/SPAD.
 * With kcps stands for Kilo Count Per Second
 */
VL53L1X_ERROR vl53l1x_get_signal_per_spad (uint16_t dev, uint16_t *signalPerSp);

/**
 * @brief This function returns the ambient per SPAD in kcps/SPAD
 */
VL53L1X_ERROR vl53l1x_get_ambient_per_spad (uint16_t dev, uint16_t *amb);

/**
 * @brief This function returns the returned signal in kcps.
 */
VL53L1X_ERROR vl53l1x_get_signal_rate (uint16_t dev, uint16_t *signalRate);

/**
 * @brief This function returns the current number of enabled SPADs
 */
VL53L1X_ERROR vl53l1x_get_spad_nb (uint16_t dev, uint16_t *spNb);

/**
 * @brief This function returns the ambient rate in kcps
 */
VL53L1X_ERROR vl53l1x_get_ambient_rate (uint16_t dev, uint16_t *ambRate);

/**
 * @brief This function returns the ranging status error \n
 * (0:no error, 1:sigma failed, 2:signal failed, ..., 7:wrap-around)
 */
VL53L1X_ERROR vl53l1x_get_range_status (uint16_t dev, uint8_t *rangeStatus);

/**
 * @brief This function returns measurements and the range status in a single read access
 */
VL53L1X_ERROR vl53l1x_get_result (uint16_t dev, VL53L1X_Result_t *pResult);

/**
 * @brief This function programs the offset correction in mm
 * @param OffsetValue:the offset correction value to program in mm
 */
VL53L1X_ERROR vl53l1x_set_offset (uint16_t dev, int16_t OffsetValue);

/**
 * @brief This function returns the programmed offset correction value in mm
 */
VL53L1X_ERROR vl53l1x_get_offset (uint16_t dev, int16_t *Offset);

/**
 * @brief This function programs the xtalk correction value in cps (Count Per Second).\n
 * This is the number of photons reflected back from the cover glass in cps.
 */
VL53L1X_ERROR vl53l1x_set_xtalk (uint16_t dev, uint16_t XtalkValue);

/**
 * @brief This function returns the current programmed xtalk correction value in cps
 */
VL53L1X_ERROR vl53l1x_get_xtalk (uint16_t dev, uint16_t *Xtalk);

/**
 * @brief This function programs the threshold detection mode\n
 * Example:\n
 * vl53l1x_set_distance_threshold(dev,100,300,0,1): Below 100 \n
 * vl53l1x_set_distance_threshold(dev,100,300,1,1): Above 300 \n
 * vl53l1x_set_distance_threshold(dev,100,300,2,1): Out of window \n
 * vl53l1x_set_distance_threshold(dev,100,300,3,1): In window \n
 * @param   dev : device address
 * @param  	ThreshLow(in mm) : the threshold under which one the device raises an interrupt if Window = 0
 * @param 	ThreshHigh(in mm) :  the threshold above which one the device raises an interrupt if Window = 1
 * @param   Window detection mode : 0=below, 1=above, 2=out, 3=in
 * @param   IntOnNoTarget = 1 (No longer used - just use 1)
 */
VL53L1X_ERROR vl53l1x_set_distance_threshold (uint16_t dev, uint16_t ThreshLow,
        uint16_t ThreshHigh, uint8_t Window, uint8_t IntOnNoTarget);

/**
 * @brief This function returns the window detection mode (0=below; 1=above; 2=out; 3=in)
 */
VL53L1X_ERROR vl53l1x_get_distance_threshold_window (uint16_t dev, uint16_t *window);

/**
 * @brief This function returns the low threshold in mm
 */
VL53L1X_ERROR vl53l1x_get_distance_threshold_low (uint16_t dev, uint16_t *low);

/**
 * @brief This function returns the high threshold in mm
 */
VL53L1X_ERROR vl53l1x_get_distance_threshold_high (uint16_t dev, uint16_t *high);

/**
 * @brief This function programs the ROI (Region of Interest)\n
 * The ROI position is centered, only the ROI size can be reprogrammed.\n
 * The smallest acceptable ROI size = 4\n
 * @param X:ROI Width; Y=ROI Height
 */
VL53L1X_ERROR vl53l1x_set_roi (uint16_t dev, uint16_t X, uint16_t Y);

/**
 *@brief This function returns width X and height Y
 */
VL53L1X_ERROR VL53L1X_GetROI_XY (uint16_t dev, uint16_t *ROI_X, uint16_t *ROI_Y);

/**
 *@brief This function programs the new user ROI center, please to be aware that there is no check in this function.
 *if the ROI center vs ROI size is out of border the ranging function return error #13
 */
VL53L1X_ERROR vl53l1x_set_roi_center (uint16_t dev, uint8_t ROICenter);

/**
 *@brief This function returns the current user ROI center
 */
VL53L1X_ERROR vl53l1x_get_roi_center (uint16_t dev, uint8_t *ROICenter);

/**
 * @brief This function programs a new signal threshold in kcps (default=1024 kcps\n
 */
VL53L1X_ERROR vl53l1x_set_signal_threshold (uint16_t dev, uint16_t signal);

/**
 * @brief This function returns the current signal threshold in kcps
 */
VL53L1X_ERROR vl53l1x_get_signal_threshold (uint16_t dev, uint16_t *signal);

/**
 * @brief This function programs a new sigma threshold in mm (default=15 mm)
 */
VL53L1X_ERROR vl53l1x_set_sigma_threshold (uint16_t dev, uint16_t sigma);

/**
 * @brief This function returns the current sigma threshold in mm
 */
VL53L1X_ERROR vl53l1x_get_sigma_threshold (uint16_t dev, uint16_t *signal);

/**
 * @brief This function performs the temperature calibration.
 * It is recommended to call this function any time the temperature might have changed by more than 8 deg C
 * without sensor ranging activity for an extended period.
 */
VL53L1X_ERROR vl53l1x_start_temperature_update (uint16_t dev);

typedef enum {
    VL53_DISTANCE_SHORT, // Short mode max distance is limited to 1.3 m but better ambient immunity
    VL53_DISTANCE_LONG, // Long mode can range up to 4 m in the dark with 200 ms timing budget
    VL53_DISTANCE_ENUM
} vl53l1x_distance_t;

typedef enum {
    VL53_PERIOD_1S,
    VL53_PERIOD_2S,
    VL53_PERIOD_5S,
    VL53_PERIOD_10S,
    VL53_PERIOD_ENUM
}vl53l1x_period_t;

/**
 * @brief  vl53l1x init
 * @param  none
 * @retval non
 */
void vl53l1x_init(void);

typedef struct {
    bool enable;
    vl53l1x_distance_t vl53l1x_distance;
    vl53l1x_period_t vl53l1x_period;
} vl53l1x_mode_t;

/**
 * @brief  get mesure pheriod [ns]
 * @param  [in] rate measure rate from vl53l1x_period_t
 * @retval measure pheriod [ns] 0 if off, 0xffffffffffffffff in other caces
 */
uint64_t vl53l1x_get_pheriod_ns(vl53l1x_period_t rate);

/**
 * @brief  get vl53l1x current mode
 * @param  [out] mode from vl53l1x_mode_t
 * @retval none
 */
void vl53l1x_get_mode(vl53l1x_mode_t* mode);

/**
 * @brief  set vl53l1x current mode
 * @param  [in] mode from vl53l1x_mode_t
 * @retval none
 */
void vl53l1x_set_mode(vl53l1x_mode_t* mode);

/**
 * @brief  enable vl53l1x
 * @param  None
 * @retval None
 */
void vl53l1x_enable(void);

/**
 * @brief  disable vl53l1x
 * @param  None
 * @retval None
 */
void vl53l1x_disable(void);

#ifdef __cplusplus
}
#endif

#endif
