/*
 * Copyright (c) 2022, Shimmer Research, Ltd.
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
 * @author Prakash Rajiah
 * @date January , 2021
 *
 * @author Sam O'Mahony
 * @date August, 2021
 *
 * @author Mark Nolan
 * @date December, 2021
 */

#ifndef ICM20948_H
#define ICM20948_H

#include <stdint.h>

// 7 bit address I2C address of the ICM20948 accel and gyro
// NOTE: 0x68 when AD0 is low , 0x69 when AD1 is high - change as required
#define ICM20948_ADDR           (0x69)
#define AK09916_MAG_ADDR	    (0x0C)
#define ICM20948_WHO_AM_I_VAL   (0xEA)

//user banks
#define USER_BANK_SEL	(0x7F)
#define USER_BANK_0		(0x00)
#define USER_BANK_1		(0x10)
#define USER_BANK_2		(0x20)
#define USER_BANK_3		(0x30)

//registers
#define ICM20948_GYRO_SMPLRT_DIV        0x00
#define ICM20948_ACCEL_SMPLRT_DIV_1     0x10
#define ICM20948_ACCEL_SMPLRT_DIV_2     0x11

#define GYRO_CONFIG_1           0x01
#define GYRO_CONFIG_2           0x02
#define ICM_ACCEL_CONFIG        0x14
#define ICM_INT_PIN_CFG         0x0F
#define ICM_USER_CTRL           0x03
#define ICM_ACCEL_XOUT_H        0x2D
#define ICM_ACCEL_XOUT_L        0x2E
#define ICM_ACCEL_YOUT_H        0x2F
#define ICM_ACCEL_YOUT_L        0x30
#define ICM_ACCEL_ZOUT_H        0x31
#define ICM_ACCEL_ZOUT_L        0x32

#define ICM_GYRO_XOUT_H         0x33
#define ICM_GYRO_XOUT_L         0x34
#define ICM_GYRO_YOUT_H         0x35
#define ICM_GYRO_YOUT_L         0x36
#define ICM_GYRO_ZOUT_H         0x37
#define ICM_GYRO_ZOUT_L         0x38

#define ICM20948_PWR_MGMT_1     0x06
#define ICM20948_WHO_AM_I_REG   0x00

//mag registers
#define WIA2                    0x01
#define ICM_ST1                 0x10
#define ICM_MAG_XOUT_L          0x11
#define ICM_MAG_XOUT_H          0x12
#define ICM_MAG_YOUT_L          0x13
#define ICM_MAG_YOUT_H          0x14
#define ICM_MAG_ZOUT_L          0x15
#define ICM_MAG_ZOUT_H          0x16
#define ICM_ST2                 0x18
#define CNTL2                   0x31
#define CNTL3                   0x32

#define SAMPLING_TIMER_TICKS_100Hz   328 // ceil(32768/100Hz) = 328. i.e., 99.90244Hz
#define SAMPLING_TIMER_TICKS_512Hz   64 // 32768/512Hz = 64

enum ICM_MAG_ALTERNATIVE_READ_IDX
{
    ICM_MAG_IDX_ST1,
    ICM_MAG_IDX_XOUT_L,
    ICM_MAG_IDX_XOUT_H,
    ICM_MAG_IDX_YOUT_L,
    ICM_MAG_IDX_YOUT_H,
    ICM_MAG_IDX_ZOUT_L,
    ICM_MAG_IDX_ZOUT_H,
    ICM_MAG_IDX_TMPS,
    ICM_MAG_IDX_ST2,
    ICM_MAG_RD_SIZE
};

typedef enum AK09916_OP_MODE
{
    AK09916_PWR_DOWN           = 0x00,
    AK09916_TRIGGER_MODE       = 0x01,
    AK09916_CONT_MODE_10HZ     = 0x02,
    AK09916_CONT_MODE_20HZ     = 0x04,
    AK09916_CONT_MODE_50HZ     = 0x06,
    AK09916_CONT_MODE_100HZ    = 0x08
} AK09916_opMode;

void ICM20948_init(void);

//returns the ID
//will always be 0xEA for ICM20948
//useful for checking communication
uint8_t ICM20948_getId(void);

//if wakeup is 0 puts ICM20948 to sleep
//else wakes it up
void ICM20948_wake(uint8_t wakeup);

//put x, y and z gyro values into buf (big endian)
void ICM20948_getGyro(uint8_t *buf);

void ICM20948_getAccelAndGyro(uint8_t *buf);

//put x, y and z accel values into buf (big endian)
void ICM20948_getAccel(uint8_t *buf);

//val = sensitivity to set
//0 = 250 dps
//1 = 500 dps
//2 = 1000 dps
//3 = 2000 dps
//else 250 dps
void ICM20948_setGyroSensitivity(uint8_t val);

//val = sensitivity to set
//0 = 2G
//1 = 4G
//2 = 8G
//3 = 16G
//else 2G
void ICM20948_setAccelRange(uint8_t val);

uint8_t ICM20948_convertSampleRateDivFromMPU9X50(uint8_t sampleRateDivider,
                                                 uint8_t lpfState);
//Set the sampling rate for the gyro
//Sampling Rate (Hz) = 8000/(sampleRateDiv+1)
//Note, max output rate for accel is 1kHz
void ICM20948_setGyroSamplingRate(uint8_t sampleRateDiv);

//Set the sampling rate for the Accel
//Sampling Rate (Hz) = 8000/(sampleRateDiv+1)
//Note, max output rate for accel is 1.125kHz
void ICM20948_setAccelSamplingRate(uint16_t sampleRateDiv);

//returns the ID of the mag
//will always be 0x48
//useful for checking communication
uint8_t ICM20948_getMagId(void);

void ICM20948_setMagSamplingRateFromShimmerRate(uint16_t samplingRateTicks);

void ICM20948_setMagMode(AK09916_opMode opMode);

uint8_t ICM20948_isMagDataRdy(void);

//put x, y and z mag values in buf (little endian)
//-4096 to 4095
//if values are 32767 they are not valid
//either due to data read error or magnetic sensor overflow
void ICM20948_getMag(uint8_t *buf);

uint8_t ICM20948_isMagSampleSkipEnabled(void);
uint8_t ICM20948_hasTimeoutPeriodPassed(uint64_t currentSampleTsTicks);
uint8_t ICM20948_getMagAndStatus(uint64_t currentSampleTsTicks, uint8_t *buf);

//read the x, y and z mag sensitivity adjustment values
void ICM20948_getMagSensitivityAdj(uint8_t *buf);

#endif //ICM20948_H
