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
#include "../5xx_HAL/hal_I2C.h"
#include "bmp180.h"

//configure I2C
void BMP180_init(void) {
   P8OUT |= BIT4;             //set SW_I2C high to power on all I2C chips
   __delay_cycles(24000000);  //wait 1s (assuming 24MHz MCLK) to allow for power ramp up

   I2C_Master_Init(S_MCLK,24000000,400000);  //Source from SMCLK, which is running @ 24MHz. 4kHz desired BRCLK
                                             //max is 3.4MHz
}

uint8_t BMP180_getId(void) {
   uint8_t buf;
   I2C_Set_Slave_Address(BMP180_ADDR);
   buf = ID;
   I2C_Read_Packet_From_Sensor(&buf, 1);
   return buf;
}

void BMP180_startTempMeasurement(void) {
   uint8_t buf[2];

   I2C_Set_Slave_Address(BMP180_ADDR);
   buf[0] = CTRL_MEAS;
   buf[1] = 0x2E;
   I2C_Write_Packet_To_Sensor(buf,2);
}

void BMP180_getTemp(uint8_t *buf) {
   I2C_Set_Slave_Address(BMP180_ADDR);
   *buf = OUT_MSB;
   I2C_Read_Packet_From_Sensor(buf, 2);
}

void BMP180_startPressMeasurement(uint8_t oss) {
   uint8_t buf[2];

   I2C_Set_Slave_Address(BMP180_ADDR);
   buf[0] = CTRL_MEAS;
   buf[1] = ((oss&0x03)<<6) + 0x34;
   I2C_Write_Packet_To_Sensor(buf,2);
}

void BMP180_getPress(uint8_t *buf) {
   I2C_Set_Slave_Address(BMP180_ADDR);
   *buf = OUT_MSB;
   I2C_Read_Packet_From_Sensor(buf, 3);
}

void BMP180_getCalibCoeff(uint8_t *res) {
   I2C_Set_Slave_Address(BMP180_ADDR);
   *res = AC1_MSB;
   I2C_Read_Packet_From_Sensor(res, 22);
}
