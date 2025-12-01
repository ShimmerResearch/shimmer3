/*
 * Copyright (c) 2013, Shimmer Research, Ltd.
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *    * Neither the name of Shimmer Research, Ltd. nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *    * You may not use or distribute this Software or any derivative works
 *      in any form for commercial purposes with the exception of commercial
 *      purposes when used in conjunction with Shimmer products purchased
 *      from Shimmer or their designated agent or with permission from
 *      Shimmer.
 *      Examples of commercial purposes would be running business
 *      operations, licensing, leasing, or selling the Software, or
 *      distributing the Software for use with commercial products.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @author Sam O'Mahony
 * @date October, 2017
 */

#include "lsm303ahtr.h"

#include "../5xx_HAL/hal_I2C.h"
#include "msp430.h"
#include <math.h>

#include "Util/shimmer_util.h"

uint8_t last_lsm303ah_mag_data[7] = { 0, 0, 0, 0, 0, 0, 0 };

void LSM303AHTR_accelInit(uint8_t samplingRate, uint8_t range, uint8_t lowPower, uint8_t highresolution)
{
  uint8_t i2c_buffer[2], highFreq;

  if (samplingRate > 7 && !lowPower)
  {
    highFreq = 1;
    switch (samplingRate)
    {
      case 8:
        //1600 Hz => ODR[3:0] 0101 = 5
        samplingRate = 0x05;
        break;
      case 9:
        //3200 Hz => ODR[3:0] 0110 = 6
        samplingRate = 0x06;
        break;
      case 10:
        //6400 Hz => ODR[3:0] 0111 = 7
        samplingRate = 0x07;
        break;
      default:
        break;
    }
  }
  else
  {
    highFreq = 0;
  }

  //Configure Accelerometer
  I2C_Set_Slave_Address(LSM303AHTR_ACCEL_ADDR);
  //write CTRL1_A register
  i2c_buffer[0] = CTRL1_A;
  i2c_buffer[1] = (samplingRate << 4) + (range << 2) + (highFreq << 1);
  I2C_Write_Packet_To_Sensor(i2c_buffer, 2);
}

void LSM303AHTR_magInit(uint8_t samplingrate)
{
  uint8_t i2c_buffer[2];

  //Configure Magnetometer
  I2C_Set_Slave_Address(LSM303AHTR_MAG_ADDR);
  //write CFG_REG_A_M register
  i2c_buffer[0] = CFG_REG_A_M;
  i2c_buffer[1] = samplingrate << 2;
  I2C_Write_Packet_To_Sensor(i2c_buffer, 2);
}

void LSM303AHTR_getAccel(uint8_t *buf)
{
  I2C_Set_Slave_Address(LSM303AHTR_ACCEL_ADDR);
  *buf = OUT_X_L_A;
  I2C_Read_Packet_From_Sensor(buf, 6);
}

void LSM303AHTR_getMag(uint8_t *buf)
{
  I2C_Set_Slave_Address(LSM303AHTR_MAG_ADDR);
  *buf = STATUS_REG_M;
  I2C_Read_Packet_From_Sensor(buf, 1);
  if (buf[0] & 0x08)
  {
    *buf = OUTX_L_REG_M;
    I2C_Read_Packet_From_Sensor(buf, 6);
    memcpy(last_lsm303ah_mag_data, buf, 6);
    last_lsm303ah_mag_data[6] = 1;
  }
  else
  {
    if (last_lsm303ah_mag_data[6] == 1)
    {
      memcpy(buf, last_lsm303ah_mag_data, 6);
    }
  }
}

void LSM303AHTR_sleep(void)
{
  uint8_t i2c_buffer[2];

  I2C_Set_Slave_Address(LSM303AHTR_ACCEL_ADDR);
  //write CTRL1_A register
  i2c_buffer[0] = CTRL1_A;
  i2c_buffer[1] = 0; //power down mode -> ODR = 0000
  I2C_Write_Packet_To_Sensor(i2c_buffer, 2);

  I2C_Set_Slave_Address(LSM303AHTR_MAG_ADDR);
  //write CFG_REG_A_M register
  i2c_buffer[0] = CFG_REG_A_M;
  i2c_buffer[1] = 0x02; //idle mode
  I2C_Write_Packet_To_Sensor(i2c_buffer, 2);
}
