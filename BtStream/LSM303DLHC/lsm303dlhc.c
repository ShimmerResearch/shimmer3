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
#include "lsm303dlhc.h"


//configure I2C
void LSM303DLHC_init(void) {
   P8OUT |= BIT4;                            //set SW_I2C high to power on all I2C chips
   __delay_cycles(24000000);                 //wait 1s (assuming 24MHz MCLK) to allow for power ramp up

   I2C_Master_Init(S_MCLK,24000000,400000);  //Source from SMCLK, which is running @ 24MHz. 4kHz desired BRCLK
}


void LSM303DLHC_accelInit(uint8_t samplingrate, uint8_t range, uint8_t lowpower, uint8_t highresolution) {
   uint8_t i2c_buffer[2];

   if(samplingrate > 9) samplingrate = 5;
   if(range > 3) range = 0;
   if(lowpower > 1) lowpower = 0;
   if(highresolution > 1) highresolution = 0;

   //Configure Accel
   I2C_Set_Slave_Address(LSM303DHLC_ACCEL_ADDR);
   //write CTRL_REG1_A register
   i2c_buffer[0] = CTRL_REG1_A;
   i2c_buffer[1] = (samplingrate << 4) + (lowpower << 3) + 0x07;
   I2C_Write_Packet_To_Sensor(i2c_buffer,2);
   //write CTRL_REG4_A register
   i2c_buffer[0] = CTRL_REG4_A;
   i2c_buffer[1] = (range << 4) + (highresolution << 3);
   I2C_Write_Packet_To_Sensor(i2c_buffer,2);
}


void LSM303DLHC_magInit(uint8_t samplingrate, uint8_t gain) {
   uint8_t i2c_buffer[2];

   if(samplingrate > 7) samplingrate = 6;
   if(gain<1 || gain>7) gain = 1;

   //Configure Mag
   I2C_Set_Slave_Address(LSM303DHLC_MAG_ADDR);
   //write CRA_REG_M register
   i2c_buffer[0] = CRA_REG_M;
   i2c_buffer[1] = samplingrate << 2;
   I2C_Write_Packet_To_Sensor(i2c_buffer,2);
   //write CRB_REG_M register
   i2c_buffer[0] = CRB_REG_M;
   i2c_buffer[1] = gain << 5;
   I2C_Write_Packet_To_Sensor(i2c_buffer,2);
   //write MR_REG_M register
   i2c_buffer[0] = MR_REG_M;
   i2c_buffer[1] = 0x00; //continuous-conversion mode
   I2C_Write_Packet_To_Sensor(i2c_buffer,2);
}


void LSM303DLHC_getAccel(uint8_t *buf) {
   I2C_Set_Slave_Address(LSM303DHLC_ACCEL_ADDR);
   // need to assert MSB of sub-address in order to read multiple bytes.
   // See section 5.1.2 of LSM303DLHC datasheet (April 2011, Doc ID 018771 Rev1) for details
   *buf = OUT_X_L_A | 0x80;
   I2C_Read_Packet_From_Sensor(buf, 6);
}


void LSM303DLHC_getMag(uint8_t *buf) {
   I2C_Set_Slave_Address(LSM303DHLC_MAG_ADDR);
   *buf = OUT_X_H_M;
   I2C_Read_Packet_From_Sensor(buf, 6);
}


void LSM303DLHC_sleep(void) {
   uint8_t i2c_buffer[2];

   I2C_Set_Slave_Address(LSM303DHLC_ACCEL_ADDR);
   //write CTRL_REG1_A register
   i2c_buffer[0] = CTRL_REG1_A;
   i2c_buffer[1] = 0;      //power down mode, 3 axes disabled
   I2C_Write_Packet_To_Sensor(i2c_buffer,2);

   I2C_Set_Slave_Address(LSM303DHLC_MAG_ADDR);
   //write MR_REG_M register
   i2c_buffer[0] = MR_REG_M;
   i2c_buffer[1] = 0x02;   //sleep mode
   I2C_Write_Packet_To_Sensor(i2c_buffer,2);
}
