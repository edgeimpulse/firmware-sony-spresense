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

#include <stdint.h>

#ifndef APDS9250_H
#define APDS9250_H

#define APDS9250_REG_MAIN_CTRL           0x00
#define APDS9250_REG_LS_MEAS_RATE        0x04
#define APDS9250_REG_LS_GAIN             0x05
#define APDS9250_REG_PART_ID             0x06
#define APDS9250_REG_MAIN_STATUS         0x07
#define APDS9250_REG_LS_DATA_IR_0        0x0A
#define APDS9250_REG_LS_DATA_IR_1        0x0B
#define APDS9250_REG_LS_DATA_IR_2        0x0C
#define APDS9250_REG_LS_DATA_GREEN0      0x0D
#define APDS9250_REG_LS_DATA_GREEN1      0x0E
#define APDS9250_REG_LS_DATA_GREEN2      0x0F
#define APDS9250_REG_LS_DATA_BLUE0       0x10
#define APDS9250_REG_LS_DATA_BLUE1       0x11
#define APDS9250_REG_LS_DATA_BLUE2       0x12
#define APDS9250_REG_LS_DATA_RED0        0x13
#define APDS9250_REG_LS_DATA_RED1        0x14
#define APDS9250_REG_LS_DATA_RED2        0x15
#define APDS9250_REG_INT_CFG             0x19
#define APDS9250_REG_INT_PERSISTENCE     0x1A
#define APDS9250_REG_LS_THRES_UP0        0x21
#define APDS9250_REG_LS_THRES_UP1        0x22
#define APDS9250_REG_LS_THRES_UP2        0x23
#define APDS9250_REG_LS_THRES_LOW0       0x24
#define APDS9250_REG_LS_THRES_LOW1       0x25
#define APDS9250_REG_LS_THRES_LOW2       0x26
#define APDS9250_REG_LS_THRES_VAR        0x27

typedef enum _APDS9250Mode {
    modeAmbientLightSensor,
    modeColorSensor
} APDS9250Mode;

typedef enum _APDS9250Resolution {
    res20bit = 0,
    res19bit,
    res18bit,
    res17bit,
    res16bit,
    res13bit    
} APDS9250Resolution;

typedef enum _APDS9250Rate {
    rate25ms = 0,
    rate50ms,
    rate100ms,
    rate200ms,
    rate500ms,
    rate1000ms,
    rate2000ms
} APDS9250Rate;

typedef enum _APDS9250Gain {
    gain1 = 0,
    gain3,
    gain6,
    gain9,
    gain18    
} APDS9250Gain;

typedef enum _APDSInterruptChannel {
    channelInfrared = 0,
    channelGreen = 1,
    channelALS = 1,
    channelRed = 2,
    channelBlue = 3
} APDSInterruptChannel;

typedef enum _APDSInterruptMode {
    modeThreshold,
    modeVariation
} APDSInterruptMode;

typedef enum _APDSInterruptVariationCount {
    variation8samples,
    variation16samples,
    variation32samples,
    variation64samples,
    variation1024samples,
} APDSVariationCount;

class Apds9250 {
public:
    Apds9250() : irq_pin((uint32_t) -1), current_res(res18bit) {}

    /**
     * @brief  APDS9250 begin
     * @param  None
     * @retval true if Ok
     */
    bool begin();

    /**
     * @brief  APDS9250 ensble
     * @param  None
     * @retval true if Ok
     */
    bool enable();

    /**
     * @brief  APDS9250 disable
     * @param  None
     * @retval true if Ok
     */
    bool disable();

    /**
     * @brief  APDS9250 reset
     * @param  None
     * @retval true if Ok
     */
    bool reset();
    
    /**
     * @brief  APDS9250 set mode
     * @param  [in] mode mode from APDS9250Mode
     * @retval true if Ok
     */
    bool setMode(APDS9250Mode mode);

    /**
     * @brief  APDS9250 set resolution
     * @param  [in] resolution resolution from APDS9250Resolution
     * @retval true if Ok
     */
    bool setResolution(APDS9250Resolution resolution);

    /**
     * @brief  APDS9250 set measurment rate
     * @param  [in] rate rate from APDS9250Rate
     * @retval true if Ok
     */
    bool setMeasurementRate(APDS9250Rate rate);

    /**
     * @brief  APDS9250 set gain
     * @param  [in] gain gain from APDS9250Gain
     * @retval true if Ok
     */
    bool setGain(APDS9250Gain gain);
    
    /**
     * @brief  APDS9250 get mode
     * @param  None
     * @retval mode from APDS9250Mode
     */
    APDS9250Mode getMode();

    /**
     * @brief  APDS9250 get mode
     * @param  None
     * @retval resolution from APDS9250Resolution
     */
    APDS9250Resolution getResolution();
        
    /**
     * @brief  APDS9250 get mode
     * @param  None
     * @retval rate from APDS9250Rate
     */
    APDS9250Rate getMeasurementRate();
        
    /**
     * @brief  APDS9250 get mode
     * @param  None
     * @retval gain from APDS9250Gain
     */
    APDS9250Gain getGain();
    
    /**
     * @brief  APDS9250 get status register
     * @param  None
     * @retval register value
     */
    uint8_t getStatusRegister();
    
    /**
     * @brief  APDS9250 get red color value
     * @param  None
     * @retval red color value
     */
    uint32_t getRed();
        
    /**
     * @brief  APDS9250 get green color value
     * @param  None
     * @retval green color value
     */
    uint32_t getGreen();
        
    /**
     * @brief  APDS9250 get blue color value
     * @param  None
     * @retval blue color value
     */
    uint32_t getBlue();
        
    /**
     * @brief  APDS9250 get IR value
     * @param  None
     * @retval IR value
     */
    uint32_t getInfrared();

    /**
     * @brief  APDS9250 get ambient light value
     * @param  None
     * @retval ambient light value
     */
    inline uint32_t getAmbientLight() { return getGreen(); }
    
    /**
     * @brief  APDS9250 get all colors values
     * @param  [out] red red color value
     * @param  [out] green green color value
     * @param  [out] blue blue color value
     * @param  [out] ir ir color value
     * @retval true if Ok
     */
    bool getAll(uint32_t* red, uint32_t* green, uint32_t* blue, uint32_t* ir);
    bool setInterruptThreshold(uint32_t upper, uint32_t lower, APDSVariationCount count);
    
    /**
     * @brief  APDS9250 converting light value from green and IR
     * @param  [out] green_value green color value
     * @param  [out] ir_value ir color value
     * @retval true if Ok
     */    
    uint32_t ConvertLightData(uint32_t green_value, uint32_t ir_value );
    
#if 0
    bool enableInterrupt(uint32_t pin, void* function, APDSInterruptChannel channel, APDSInterruptMode mode);
    bool disableInterrupt();
#endif
    
private:
    uint32_t irq_pin;
    APDS9250Resolution current_res;
    uint32_t resolution_mask;

    uint32_t getResolutionMask(APDS9250Resolution res) {
        switch (res) {
            case res20bit: return 0xFFFFF;
            case res19bit: return 0x7FFFF;
            case res18bit: return 0x3FFFF;
            case res17bit: return 0x1FFFF;
            case res16bit: return 0xFFFF;
            case res13bit: return 0x1FFF;            
        }

        return 0xFFFFFFFF;
    }
    
    uint8_t readRegister(uint8_t reg);
    void readRegisters(uint8_t reg, uint8_t* values, uint8_t count);
    void writeRegister(uint8_t reg, uint8_t value);
    uint32_t regTo24bits(uint8_t* values);
};

#endif // APDS9250_H