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

#include "Apds9250.h"
#include "../I2c/I2c.h"

#define APDS9250_ADDRESS 0x52

void Apds9250::readRegisters(uint8_t reg, uint8_t* values, uint8_t count) {
    i2c_read_regs(APDS9250_ADDRESS, reg, values, count);
}

uint8_t Apds9250::readRegister(uint8_t reg) {
    static uint8_t data;
    i2c_read_regs (APDS9250_ADDRESS, reg, &data, 1);
    return data;    
}

void Apds9250::writeRegister(uint8_t reg, uint8_t value) {
    i2c_write_regs(APDS9250_ADDRESS, reg, &value, 1);
}

bool Apds9250::begin() {
    
    // Read PART_ID
    uint8_t part = readRegister(APDS9250_REG_PART_ID);
    if (part >> 4 != 0x0B)
        return false;
    
    if (!setMode(modeAmbientLightSensor))
        return false;

    resolution_mask = getResolutionMask(current_res);
    return enable();
}

bool Apds9250::enable() {
    uint8_t reg = readRegister(APDS9250_REG_MAIN_CTRL);
    reg |= 0x02;
    writeRegister(APDS9250_REG_MAIN_CTRL, reg);
    return true;
}

bool Apds9250::disable() {
    uint8_t reg = readRegister(APDS9250_REG_MAIN_CTRL);
    reg &= ~0x02;
    writeRegister(APDS9250_REG_MAIN_CTRL, reg);
    return true;
}

bool Apds9250::reset() {
    uint8_t reg = readRegister(APDS9250_REG_MAIN_CTRL);
    reg |= 0x10;
    writeRegister(APDS9250_REG_MAIN_CTRL, reg);
    return true;
}

bool Apds9250::setMode(APDS9250Mode mode) {
    uint8_t reg = readRegister(APDS9250_REG_MAIN_CTRL);
    reg &= ~0x04;
    reg |= (mode == modeAmbientLightSensor) ? 0 : 0x04;
    writeRegister(APDS9250_REG_MAIN_CTRL, reg);
    return true;
}

bool Apds9250::setResolution(APDS9250Resolution resolution) {
    uint8_t reg = readRegister(APDS9250_REG_LS_MEAS_RATE);
    reg &= ~0x70;
    reg |= ((uint8_t) resolution) << 4;
    writeRegister(APDS9250_REG_LS_MEAS_RATE, reg);

    current_res = resolution;
    resolution_mask = getResolutionMask(current_res);
    return true;
}

bool Apds9250::setMeasurementRate(APDS9250Rate rate) {
    uint8_t reg = readRegister(APDS9250_REG_LS_MEAS_RATE);
    reg &= ~0x07;
    reg |= ((uint8_t) rate);
    writeRegister(APDS9250_REG_LS_MEAS_RATE, reg);
    return true;
}

bool Apds9250::setGain(APDS9250Gain gain) {
    uint8_t reg = (uint8_t) gain;
    writeRegister(APDS9250_REG_LS_GAIN, reg);
    return true;
}

APDS9250Mode Apds9250::getMode() {
    uint8_t reg = readRegister(APDS9250_REG_MAIN_CTRL);
    reg = (reg >> 2) & 0x01;
    return (APDS9250Mode) reg;
}

APDS9250Resolution Apds9250::getResolution() {
    uint8_t reg = readRegister(APDS9250_REG_LS_MEAS_RATE);
    reg = (reg >> 4) & 0x07;
    return (APDS9250Resolution) reg;
}

APDS9250Rate Apds9250::getMeasurementRate() {
    uint8_t reg = readRegister(APDS9250_REG_LS_MEAS_RATE);
    reg = reg & 0x07;
    return (APDS9250Rate) reg;
}

APDS9250Gain Apds9250::getGain() {
    uint8_t reg = readRegister(APDS9250_REG_LS_GAIN);
    reg = reg & 0x07;
    return (APDS9250Gain) reg;    
}
    
uint8_t Apds9250::getStatusRegister() {
    uint8_t reg = readRegister(APDS9250_REG_MAIN_STATUS);
    return reg;
}

uint32_t Apds9250::regTo24bits(uint8_t* values) {
    uint32_t val;
    
    val = values[0];
    val |= values[1] << 8;
    val |= values[2] << 16;
    
    return val & resolution_mask;
}
    
uint32_t Apds9250::getRed() {
    uint32_t val;
    uint8_t values[3];
    
    readRegisters(APDS9250_REG_LS_DATA_RED0, values, 3);

    return regTo24bits(values);
}

uint32_t Apds9250::getGreen() {
    uint32_t val;
    uint8_t values[3];
    
    readRegisters(APDS9250_REG_LS_DATA_GREEN0, values, 3);

    return regTo24bits(values);
}

uint32_t Apds9250::getBlue() {
    uint32_t val;
    uint8_t values[3];
    
    readRegisters(APDS9250_REG_LS_DATA_BLUE0, values, 3);

    return regTo24bits(values);        
}
    
uint32_t Apds9250::getInfrared() {
    uint32_t val;
    uint8_t values[3];
    
    readRegisters(APDS9250_REG_LS_DATA_IR_0, values, 3);

    return regTo24bits(values);    
}

bool Apds9250::getAll(uint32_t* red, uint32_t* green, uint32_t* blue, uint32_t* ir) {
#if 1 //read all color regs
    uint8_t values[12];
    
    readRegisters(APDS9250_REG_LS_DATA_IR_0, values, 12);

    if (ir)
        *ir = regTo24bits(&values[0]);

    if (green)
        *green = regTo24bits(&values[3]);
    
    if (blue)
        *blue = regTo24bits(&values[6]);
    
    if (red)
        *red = regTo24bits(&values[9]);

#else // read color reg by reg
    uint32_t color = 0;
    uint8_t *pnt = (uint8_t*) &color;

    if (ir)
        pnt[0] = readRegister(APDS9250_REG_LS_DATA_IR_0);
        pnt[1] = readRegister(APDS9250_REG_LS_DATA_IR_1);
        pnt[2] = readRegister(APDS9250_REG_LS_DATA_IR_2);
        *ir = color;

    if (green)
        pnt[0] = readRegister(APDS9250_REG_LS_DATA_GREEN0);
        pnt[1] = readRegister(APDS9250_REG_LS_DATA_GREEN1);
        pnt[2] = readRegister(APDS9250_REG_LS_DATA_GREEN2);
        *green = color;
    
    if (blue)
        pnt[0] = readRegister(APDS9250_REG_LS_DATA_BLUE0);
        pnt[1] = readRegister(APDS9250_REG_LS_DATA_BLUE1);
        pnt[2] = readRegister(APDS9250_REG_LS_DATA_BLUE2);
        *blue = color;
    
    if (red)
        pnt[0] = readRegister(APDS9250_REG_LS_DATA_RED0);
        pnt[1] = readRegister(APDS9250_REG_LS_DATA_RED1);
        pnt[2] = readRegister(APDS9250_REG_LS_DATA_RED2);
        *red = color;
#endif
    
    return true;
}
        
bool Apds9250::setInterruptThreshold(uint32_t upper, uint32_t lower, APDSVariationCount count) {
    writeRegister(APDS9250_REG_LS_THRES_UP0, upper & 0xFF);
    writeRegister(APDS9250_REG_LS_THRES_UP1, (upper >> 8) & 0xFF);
    writeRegister(APDS9250_REG_LS_THRES_UP2, (upper >> 16) & 0xFF);

    writeRegister(APDS9250_REG_LS_THRES_LOW0, lower & 0xFF);
    writeRegister(APDS9250_REG_LS_THRES_LOW1, (lower >> 8) & 0xFF);
    writeRegister(APDS9250_REG_LS_THRES_LOW2, (lower >> 16) & 0xFF);

    uint8_t reg = (uint8_t) count;
    reg = reg & 0x07;
    writeRegister(APDS9250_REG_LS_THRES_VAR, reg);

    return true;
}

uint32_t Apds9250::ConvertLightData(uint32_t green_value, uint32_t ir_value ) {

    uint32_t factor = ir_value > green_value ? 35 : 46;

    uint32_t lux = ((green_value * factor) / 3) / 100;
    return lux;
}

#if 0
bool Apds9250::enableInterrupt(uint32_t pin, void* function, APDSInterruptChannel channel, APDSInterruptMode mode) {
    if (irq_pin != (uint32_t) -1)
        disableInterrupt();

    pinMode(pin, INPUT);
    
    uint8_t reg = readRegister(APDS9250_REG_INT_CFG);

    // Clear other values
    reg &= ~0x3C;

    reg |= ((uint8_t) channel) << 4;
    reg |= ((uint8_t) mode) << 3;

    writeRegister(APDS9250_REG_INT_CFG, reg);

    // Attach interrupt to pin
    attachInterrupt(pin, function, FALLING);

    // Enable interrupt
    reg = readRegister(APDS9250_REG_INT_CFG);
    reg |= 0x04;
    writeRegister(APDS9250_REG_INT_CFG, reg);

    reg = readRegister(APDS9250_REG_INT_CFG);
    Serial.println(reg, HEX);

    irq_pin = pin;
    return true;
}

bool Apds9250::disableInterrupt() {
    uint8_t reg = readRegister(APDS9250_REG_INT_CFG);
    reg &= ~0x04;
    writeRegister(APDS9250_REG_INT_CFG, reg);

    detachInterrupt(irq_pin);
    irq_pin = (uint32_t) -1;
    return true;
}
#endif
