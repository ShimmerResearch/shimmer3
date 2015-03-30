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

#ifndef BMP180_H
#define BMP180_H

#include <stdint.h>

#define BMP180_ADDR  0x77 //7 bit address I2C address of the BMP180

//registers
#define OUT_XLSB     0xF8
#define OUT_LSB      0xF7
#define OUT_MSB      0xF6
#define CTRL_MEAS    0xF4
#define SOFT_RESET   0xE0
#define ID           0xD0
//calibration coefficients
#define AC1_MSB      0xAA
#define AC1_LSB      0xAB
#define AC2_MSB      0xAC
#define AC2_LSB      0xAD
#define AC3_MSB      0xAE
#define AC3_LSB      0xAF
#define AC4_MSB      0xB0
#define AC4_LSB      0xB1
#define AC5_MSB      0xB2
#define AC5_LSB      0xB3
#define AC6_MSB      0xB4
#define AC6_LSB      0xB5
#define B1_MSB       0xB6
#define B1_LSB       0xB7
#define B2_MSB       0xB8
#define B2_LSB       0xB9
#define MB_MSB       0xBA
#define MB_LSB       0xBB
#define MC_MSB       0xBC
#define MC_LSB       0xBD
#define MD_MSB       0xBE
#define MD_LSB       0xBF


//initialise the I2C for use with the BMP180
void BMP180_init(void);

//returns the ID
//will always be 0x55
//useful for checking communication
uint8_t BMP180_getId(void);

//Initiate temperature measurement
//Need to wait 4.5ms before reading value
void BMP180_startTempMeasurement(void);

//read temperature
//16-bit value returned in buf
//big endian
void BMP180_getTemp(uint8_t *buf);

//Initiate temperature measurement
//if OSS = 0 need to wait 4.5ms before reading value
//if OSS = 1 need to wait 7.5ms before reading value
//if OSS = 2 need to wait 13.5ms before reading value
//if OSS = 3 need to wait 25.5ms before reading value
void BMP180_startPressMeasurement(uint8_t oss);

//read pressure
//19-bit (in 3 bytes) value returned in *buf
//big endian
void BMP180_getPress(uint8_t *buf);

//read calibration Coefficients
//AC1 to MD
//11x16-bit values returned in res
//big endian
void BMP180_getCalibCoeff(uint8_t *res);

#endif //BMP180_H
