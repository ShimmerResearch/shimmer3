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
 * @date June, 2017
 */

#ifndef LSM303AHTR_H
#define LSM303AHTR_H

#include <stdint.h>

#define LSM303AHTR_ACCEL_ADDR  0x1D // 7 bit address of the LSM303DHLC's accelerometer
#define LSM303AHTR_MAG_ADDR    0x1E // 7 bit address of the LSM303DHLC's magnetometer

#define LSM303AHTR_IN_USE      0x02

//registers
/* Reserved 0x00 - 0x0B */
#define Module_8bit_A          0x0C
/* Reserved 0x0D - 0x0E */
#define WHO_AM_I_A             0x0F
/* Reserved 0x10 - 0x1F */
#define CTRL1_A                0x20
#define CTRL2_A                0x21
#define CTRL3_A                0x22
#define CTRL4_A                0x23
#define CTRL5_A                0x24
#define FIFO_CTRL_A            0x25
#define OUT_T_A                0x26
#define STATUS_A               0x27
#define OUT_X_L_A              0x28
#define OUT_X_H_A              0x29
#define OUT_Y_L_A              0x2A
#define OUT_Y_H_A              0x2B
#define OUT_Z_L_A              0x2C
#define OUT_Z_H_A              0x2D
#define FIFO_THS_A             0x2E
#define FIFO_SRC_A             0x2F
#define FIFO_SAMPLES_A         0x30
#define TAP_6D_THS_A           0x31
#define INT_DUR_A              0x32
#define WAKE_UP_THS_A          0x33
#define WAKE_UP_DUR_A          0x34
#define FREE_FALL_A            0x35
#define STATUS_DUP_A           0x36
#define WAKE_UP_SRC__A         0x37
#define TAP_SRC_A              0x38
#define SIXD_SRC_A             0x39
#define STEP_COUNTER_MINTHS_A  0x3A
#define STEP_COUNTER_L_A       0x3B
#define STEP_COUNTER_H_A       0x3C
#define FUNC_CK_GATE_A         0x3D
#define FUNC_SRC_A             0x3E
#define FUNC_CTRL_A            0x3F
/* Reserved 0x40 - 0x44 */
#define OFFSET_X_REG_L_M       0x45
#define OFFSET_X_REG_H_M       0x46
#define OFFSET_Y_REG_L_M       0x47
#define OFFSET_Y_REG_H_M       0x48
#define OFFSET_Z_REG_L_M       0x49
#define OFFSET_Z_REG_H_M       0x4A
/* Reserved 0x4B - 0x4C */
#define WHO_AM_I_M             0x4F
/* Reserved 0x50 - 0x5F */
#define CFG_REG_A_M            0x60
#define CFG_REG_B_M            0x61
#define CFG_REG_C_M            0x62
#define INT_CTRL_REG_M         0x63
#define INT_SOURCE_REG_M       0x64
#define INT_THS_L_REG_M        0x65
#define STATUS_REG_M           0x67
#define OUTX_L_REG_M           0x68
#define OUTX_H_REG_M           0x69
#define OUTY_L_REG_M           0x6A
#define OUTY_H_REG_M           0x6B
#define OUTZ_L_REG_M           0x6C
#define OUTZ_H_REG_M           0x6D
/* Reserved 0x6E - 0x6F */

//initialize the I2C for use with the LSM303AHTR
void LSM303AHTR_init(void);

//initialize the accelerometer
void LSM303AHTR_accelInit(uint8_t samplingrate, uint8_t range, uint8_t lowpower, uint8_t highresolution);

//initialize the magnetometer
void LSM303AHTR_magInit(uint8_t samplingrate);

//put x, y and z accel values into buf
void LSM303AHTR_getAccel(uint8_t *buf);

//put x, y and z mag values into buf
void LSM303AHTR_getMag(uint8_t *buf);

//powers down the LSM303AHTR
void LSM303AHTR_sleep(void);

#endif //LSM303AHTR_H
