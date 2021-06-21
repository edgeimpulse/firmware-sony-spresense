/*****************************************************************************
  KX122.cpp
 Copyright (c) 2018 ROHM Co.,Ltd.
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
******************************************************************************/
#include <Wire.h>

#include "KX122.h"
#include "ei_device_sony_spresense.h"

KX122::KX122(int slave_address)
{
  _device_address = slave_address;
}

char KX122::init(void)
{
  char rc;
  unsigned char reg;
  unsigned char gsel;
  int i;

  rc = read(KX122_WHO_AM_I + 1, &reg, sizeof(reg));
  if (rc != 0) {
    ei_printf("Can't access KX122");
    return (rc);
  } 
  ei_printf("KX122_WHO_AMI Register Value = 0x%X\r\n", reg);
  
  if (reg != KX122_WAI_VAL) {
    ei_printf("Can't find KX122");
    return (-1);
  }

  reg = KX122_CNTL1_VAL;
  rc = write(KX122_CNTL1, &reg, sizeof(reg));
  if (rc != 0) {
    ei_printf("Can't write KX122 CNTL1 register at first");
    return (rc);
  }

  reg = KX122_ODCNTL_VAL;
  rc = write(KX122_ODCNTL, &reg, sizeof(reg));
  if (rc != 0) {
    ei_printf("Can't write KX122 ODCNTL register");
    return (rc);
  }

  rc = read(KX122_CNTL1, &reg, sizeof(reg));
  if (rc != 0) {
    ei_printf("Can't read KX122 CNTL1 register");
    return (rc);
  }
  gsel = reg & KX122_CNTL1_GSELMASK;

  reg |= KX122_CNTL1_PC1;
  rc = write(KX122_CNTL1, &reg, sizeof(reg));
  if (rc != 0) {
    ei_printf("Can't write KX122 CNTL1 register at second");
    return (rc);
  }
  
  switch(gsel) {
    case KX122_CNTL1_GSEL_2G : _g_sens = 16384; break;
    case KX122_CNTL1_GSEL_4G : _g_sens = 8192;  break;
    case KX122_CNTL1_GSEL_8G : _g_sens = 4096;  break;
    default: break;
  }
}

char KX122::get_rawval(unsigned char *data)
{
  char rc;

  rc = read(KX122_XOUT_L, data, 6);
  if (rc != 0) {
    ei_printf("Can't get KX122 accel value");
  }

  return (rc);
}

char KX122::get_val(float *data)
{
  char rc;
  unsigned char val[6];
  signed short acc[3];

  rc = get_rawval(val);
  if (rc != 0) {
    return (rc);
  }

  acc[0] = ((signed short)val[1] << 8) | (val[0]);
  acc[1] = ((signed short)val[3] << 8) | (val[2]);
  acc[2] = ((signed short)val[5] << 8) | (val[4]);

  // Convert LSB to g
  data[0] = (float)acc[0] / _g_sens;
  data[1] = (float)acc[1] / _g_sens;
  data[2] = (float)acc[2] / _g_sens;

  return (rc);  
}

char KX122::write(unsigned char memory_address, unsigned char *data, unsigned char size)
{
  char rc;
  unsigned int cnt;

  Wire.beginTransmission((uint8_t)_device_address);
  Wire.write(memory_address);
  Wire.write(data, size);
  rc = Wire.endTransmission(true);
  return (rc);
}

char KX122::read(unsigned char memory_address, unsigned char *data, int size)
{
  char rc;
  unsigned char cnt;

  Wire.beginTransmission((uint8_t)_device_address);
  Wire.write(memory_address);
  rc = Wire.endTransmission(false);
  if (rc != 0) {
    ei_printf("Read failed!\r\n");
    return (rc);
  }

  Wire.requestFrom((int)_device_address, (int)size, (int)true);
  cnt = 0;
  while(Wire.available()) {
    data[cnt] = Wire.read();
    ei_printf("%d data: %X\r\n", cnt, data[cnt]);
    cnt++;
  }

  return (0);
}