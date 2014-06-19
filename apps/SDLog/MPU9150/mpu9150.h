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

#ifndef MPU9150_H
#define MPU9150_H

#include <stdint.h>

#define MPU9150_ADDR       0x68  //7 bit address I2C address of the MPU9150 accel and gyro
#define MPU9150_MAG_ADDR   0x0C  //7 bit address I2C address of the MPU9150 mag

//registers
#define MPU9150_SMPLRT_DIV 0x19
#define GYRO_CONFIG        0x1B
#define ACCEL_CONFIG       0x1C
#define INT_PIN_CFG        0x37
#define USER_CTRL          0x6A
#define ACCEL_XOUT_H       0x3B
#define ACCEL_XOUT_L       0x3C
#define ACCEL_YOUT_H       0x3D
#define ACCEL_YOUT_L       0x3E
#define ACCEL_ZOUT_H       0x3F
#define ACCEL_ZOUT_L       0x40
#define GYRO_XOUT_H        0x43
#define GYRO_XOUT_L        0x44
#define GYRO_YOUT_H        0x45
#define GYRO_YOUT_L        0x46
#define GYRO_ZOUT_H        0x47
#define GYRO_ZOUT_L        0x48
#define MPU9150_PWR_MGMT_1 0x6B
#define MPU9150_WHO_AM_I   0x75

//mag registers
#define WIA                0x00
#define MAG_XOUT_L         0x03
#define MAG_XOUT_H         0x04
#define MAG_YOUT_L         0x05
#define MAG_YOUT_H         0x06
#define MAG_ZOUT_L         0x07
#define MAG_ZOUT_H         0x08
#define ST2                0x09
#define CNTL               0x0A
#define ASAX               0x10
#define ASAY               0x11
#define ASAZ               0x12

void MPU9150_init(void);

//returns the ID
//will always be 0x68
//useful for checking communication
uint8_t MPU9150_getId(void);


//if wakeup is 0 puts MPU9150 to sleep
//else wakes it up
void MPU9150_wake(uint8_t wakeup);

//put x, y and z gyro values into buf (big endian)
void MPU9150_getGyro(uint8_t *buf);

//put x, y and z accel values into buf (big endian)
void MPU9150_getAccel(uint8_t *buf);

//val = sensitivity to set
//0 = ±250°/s
//1 = ±500°/s
//2 = ±1000°/s
//3 = ±2000°/s
//else ±250°/s
void MPU9150_setGyroSensitivity(uint8_t val);

//val = sensitivity to set
//0 = ±2G
//1 = ±4G
//2 = ±8G
//3 = ±16G
//else ±2G
void MPU9150_setAccelRange(uint8_t val);

//Set the sampling rate for the gyro and accel
//Sampling Rate (Hz) = 8000/(sampleRateDiv+1)
//Note, max output rate for accel is 1kHz
void MPU9150_setSamplingRate(uint8_t sampleRateDiv);

//returns the ID of the mag
//will always be 0x48
//useful for checking communication
uint8_t MPU9150_getMagId(void);

//Set the mag to single measurement mode
//can take between 7.3ms to 9ms before data is ready
void MPU9150_startMagMeasurement(void);

//put x, y and z mag values in buf (little endian)
//-4096 to 4095
//if values are 32767 they are not valid
//either due to data read error or magnetic sensor overflow
void MPU9150_getMag(uint8_t *buf);

//read the x, y and z mag sensitivity adjustment values
void MPU9150_getMagSensitivityAdj(uint8_t *buf);

#endif //MPU9150_H
