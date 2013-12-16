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
 * @author Mike Healy
 * @date December, 2013
 */

#include "msp430.h"
#include "hal_I2C.h"
#include "mpu9150.h"

//configure I2C
void MPU9150_init(void) {
   uint8_t buf[2];

   P8OUT |= BIT4;             //set SW_I2C high to power on all I2C chips
   __delay_cycles(24000000);  //wait 1s (assuming 24MHz MCLK) to allow for power ramp up

   I2C_Master_Init(S_MCLK,24000000,400000);  //Source from SMCLK, which is running @ 24MHz. 4kHz desired BRCLK
                                             //which is max for this part
   //put in I2C pass through mode so that mag can be accessed
   I2C_Set_Slave_Address(MPU9150_ADDR);

   *buf = USER_CTRL;
   buf[1] = 0;    //ensure I2C_MST_EN is 0
   I2C_Write_Packet_To_Sensor(buf, 2);

   *buf = INT_PIN_CFG;
   buf[1] = 0x02; //set I2C_BYPASS_EN to 1
   I2C_Write_Packet_To_Sensor(buf, 2);
}

uint8_t MPU9150_getId(void) {
   uint8_t buf;
   I2C_Set_Slave_Address(MPU9150_ADDR);
   buf = MPU9150_WHO_AM_I;
   I2C_Read_Packet_From_Sensor(&buf, 1);
   return buf;
}

void MPU9150_wake(uint8_t wakeup) {
   uint8_t buf[2];

   I2C_Set_Slave_Address(MPU9150_ADDR);
   *buf = MPU9150_PWR_MGMT_1;
   I2C_Read_Packet_From_Sensor(buf, 1);

   if(wakeup) {
      //wakeup
      buf[1] = buf[0] & 0xBF;
   } else {
      //go back to sleep
      buf[1] = buf[0] | 0x40;
   }
   *buf = MPU9150_PWR_MGMT_1;
   I2C_Write_Packet_To_Sensor(buf,2);
}

void MPU9150_getGyro(uint8_t *buf) {
   I2C_Set_Slave_Address(MPU9150_ADDR);
   *buf = GYRO_XOUT_H;
   I2C_Read_Packet_From_Sensor(buf, 6);
}

void MPU9150_getAccel(uint8_t *buf) {
   I2C_Set_Slave_Address(MPU9150_ADDR);
   *buf = ACCEL_XOUT_H;
   I2C_Read_Packet_From_Sensor(buf, 6);
}

void MPU9150_setGyroSensitivity(uint8_t val) {
   uint8_t buf[2];

   I2C_Set_Slave_Address(MPU9150_ADDR);
   *buf = GYRO_CONFIG;
   I2C_Read_Packet_From_Sensor(buf, 1);

   if(val>0 && val<4) {
      buf[1] = (buf[0]&0xE7) | (val<<3);
   } else {
      buf[1] = (buf[0]&0xE7);
   }

   *buf = GYRO_CONFIG;
   I2C_Write_Packet_To_Sensor(buf,2);
}

void MPU9150_setAccelRange(uint8_t val) {
   uint8_t buf[2];

   I2C_Set_Slave_Address(MPU9150_ADDR);
   *buf = ACCEL_CONFIG;
   I2C_Read_Packet_From_Sensor(buf, 1);

   if(val>0 && val<4) {
      buf[1] = (buf[0]&0xE7) | (val<<3);
   } else {
      buf[1] = (buf[0]&0xE7);
   }

   *buf = ACCEL_CONFIG;
   I2C_Write_Packet_To_Sensor(buf,2);
}

void MPU9150_setSamplingRate(uint8_t sampleRateDiv) {
   uint8_t buf[2];

   I2C_Set_Slave_Address(MPU9150_ADDR);
   *buf = MPU9150_SMPLRT_DIV;

   buf[1] = sampleRateDiv;

   I2C_Write_Packet_To_Sensor(buf,2);
}

uint8_t MPU9150_getMagId(void) {
   uint8_t buf;
   I2C_Set_Slave_Address(MPU9150_MAG_ADDR);
   buf = WIA;
   I2C_Read_Packet_From_Sensor(&buf, 1);
   return buf;
}

//Set the mag to single measurement mode
//can take between 7.3ms to 9ms before data is ready
void MPU9150_startMagMeasurement(void) {
   uint8_t buf[2];

   I2C_Set_Slave_Address(MPU9150_MAG_ADDR);
   *buf = CNTL;
   buf[1] = 0x01;    //single measurement mode
   I2C_Write_Packet_To_Sensor(buf,2);
}

//put x, y and z mag values in buf (little endian)
//-4096 to 4095
//if values are 32767 they are not valid
//either due to data read error or magnetic sensor overflow
void MPU9150_getMag(uint8_t *buf) {
   uint8_t status;

   I2C_Set_Slave_Address(MPU9150_MAG_ADDR);
   *buf = MAG_XOUT_L;
   I2C_Read_Packet_From_Sensor(buf, 6);

   //check status register
   status = ST2;
   I2C_Read_Packet_From_Sensor(&status, 1);
   if(status) {
      //either a read error or mag sensor overflow occurred
      buf[0] = 0xFF;
      buf[1] = 0x7F;
      buf[2] = 0xFF;
      buf[3] = 0x7F;
      buf[4] = 0xFF;
      buf[5] = 0x7F;
   }

}

//read the x, y and z mag sensitivity adjustment values
void MPU9150_getMagSensitivityAdj(uint8_t *buf) {
   uint8_t localbuf[2];

   //ensure starting from power down mode
   I2C_Set_Slave_Address(MPU9150_MAG_ADDR);
   *localbuf = CNTL;
   buf[1] = 0x00;    //powerdown mode
   I2C_Write_Packet_To_Sensor(localbuf,2);
   __delay_cycles(2400);   //100us (assuming 24MHz clock)

   //set to fuse ROM mode
   *localbuf = CNTL;
   localbuf[1] = 0x0F;     //Fuse ROM mode
   I2C_Write_Packet_To_Sensor(localbuf,2);

   //read sensitivity adjustment data for the 3 axes
   *buf = ASAX;
   I2C_Read_Packet_From_Sensor(buf, 3);

   //return to power down mode
   I2C_Set_Slave_Address(MPU9150_MAG_ADDR);
   *localbuf = CNTL;
   localbuf[1] = 0x00;     //powerdown mode
   I2C_Write_Packet_To_Sensor(localbuf,2);
   __delay_cycles(2400);   //100us (assuming 24MHz clock)
}
