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
 */
/*
 * @author Weibo Pan
 * @date Mar, 2014
 *
 * @edited Sam O'Mahony
 * @date January, 2018
 */
#ifndef SHIMMER_BTSD_H
#define SHIMMER_BTSD_H

#include <stdint.h>

//these are defined in the Makefile for BtStream (TinyOS)
#define DEVICE_VER 3 //Represents SR30. 0-3 for shimmer1 to shimmer3
#define FW_IDENTIFIER \
  3 //Two byte firmware identifier number:  3 for BTSD, 2 for SDLog, 1 for BTStream,
#define FW_VER_MAJOR 1 //Major version number: 0-65535
#define FW_VER_MINOR 0 //Minor version number: 0-255
#define FW_VER_REL   2 //internal version number: 0-255

typedef uint8_t bool;
#define TRUE  (1)
#define FALSE (0)

typedef uint8_t error_t;
#define SUCCESS                       (1)
#define FAIL                          (0)

//SENSORS0
#define SENSOR_A_ACCEL                0x80
#define SENSOR_MPU9X50_ICM20948_GYRO  0x40
#define SENSOR_LSM303XXXX_MAG         0x20
#define SENSOR_EXG1_24BIT             0x10
#define SENSOR_EXG2_24BIT             0x08
#define SENSOR_GSR                    0x04
#define SENSOR_EXT_A7                 0x02
#define SENSOR_EXT_A6                 0x01
//SENSORS1
#define SENSOR_STRAIN                 0x80
//#define SDH_SENSOR_HR                0x40
#define SENSOR_VBATT                  0x20
#define SENSOR_LSM303XXXX_ACCEL       0x10
#define SENSOR_EXT_A15                0x08
#define SENSOR_INT_A1                 0x04
#define SENSOR_INT_A12                0x02
#define SENSOR_INT_A13                0x01
//SENORS2
#define SENSOR_INT_A14                0x80
#define SENSOR_MPU9X50_ICM20948_ACCEL 0x40
#define SENSOR_MPU9X50_ICM20948_MAG   0x20
#define SENSOR_EXG1_16BIT             0x10
#define SENSOR_EXG2_16BIT             0x08
#define SENSOR_BMPX80_PRESSURE        0x04

//#define RESPONSE_PACKET_SIZE     131   //biggest possibly required  (daughter card mem read + 1 byte for ack)
#define DATA_PACKET_SIZE \
  84 //3 + (MAX_NUM_CHANNELS * 2) + 1 + 6 (+1 as BMPX80
     //pressure requires 3 bytes, +6 for 4 (3 byte) ExG
     //channels plus 2 status bytes instead of
     //4xinternalADC)

//Config byte masks
//Config Byte0
#define LSM303DLHC_ACCEL_SAMPLING_RATE        0xF0
#define LSM303DLHC_ACCEL_RANGE                0x0C
#define LSM303DLHC_ACCEL_LOW_POWER_MODE       0x02
#define LSM303DLHC_ACCEL_HIGH_RESOLUTION_MODE 0x01
//Config Byte1
#define MPU9150_SAMPLING_RATE                 0xFF
//Config Byte2
#define LSM303DLHC_MAG_GAIN                   0xE0
#define LSM303DLHC_MAG_SAMPLING_RATE          0x1C
#define MPU9150_GYRO_RANGE                    0x03
//Config Byte3
#define MPU9150_ACCEL_RANGE                   0xC0
#define BMPX80_PRESSURE_RESOLUTION            0x30
#define GSR_RANGE                             0x0E
#define EXP_POWER_ENABLE                      0x01
//Unused bits 3-0

//ADC initialisation mask
#define MASK_A_ACCEL                          0x0001
#define MASK_VBATT                            0x0002
#define MASK_EXT_A7                           0x0004
#define MASK_EXT_A6                           0x0008
#define MASK_EXT_A15                          0x0010
#define MASK_INT_A1                           0x0020
#define MASK_INT_A12                          0x0040
#define MASK_INT_A13                          0x0080
#define MASK_INT_A14                          0x0100
#define MASK_MSP_TEMP                         0x0200
#define MASK_STRAIN                           0x0180 //uses ADC13 and ADC14

//LSM303DLHC Accel Range
//Corresponds to the FS field of the LSM303DLHC's CTRL_REG4_A register
//and the AFS_SEL field of the MPU9150's ACCEL_CONFIG register
#define ACCEL_2G                              0x00
#define ACCEL_4G                              0x01
#define ACCEL_8G                              0x02
#define ACCEL_16G                             0x03

//LSM303DLHC Accel Sampling Rate
//Corresponds to the ODR field of the LSM303DLHC's CTRL_REG1_A register
#define LSM303DLHC_ACCEL_POWER_DOWN           0x00
#define LSM303DLHC_ACCEL_1HZ                  0x01
#define LSM303DLHC_ACCEL_10HZ                 0x02
#define LSM303DLHC_ACCEL_25HZ                 0x03
#define LSM303DLHC_ACCEL_50HZ                 0x04
#define LSM303DLHC_ACCEL_100HZ                0x05
#define LSM303DLHC_ACCEL_200HZ                0x06
#define LSM303DLHC_ACCEL_400HZ                0x07
#define LSM303DLHC_ACCEL_1_620KHZ             0x08 //1.620kHz in Low-power mode only
#define LSM303DLHC_ACCEL_1_344kHz \
  0x09 //1.344kHz in normal mode, 5.376kHz in low-power mode

//LSM303DLHC Mag gain
#define LSM303DLHC_MAG_1_3G            0x01 //+/-1.3 Gauss
#define LSM303DLHC_MAG_1_9G            0x02 //+/-1.9 Gauss
#define LSM303DLHC_MAG_2_5G            0x03 //+/-2.5 Gauss
#define LSM303DLHC_MAG_4_0G            0x04 //+/-4.0 Gauss
#define LSM303DLHC_MAG_4_7G            0x05 //+/-4.7 Gauss
#define LSM303DLHC_MAG_5_6G            0x06 //+/-5.6 Gauss
#define LSM303DLHC_MAG_8_1G            0x07 //+/-8.1 Gauss

//LSM303DLHC Mag sampling rate
#define LSM303DLHC_MAG_0_75HZ          0x00 //0.75 Hz
#define LSM303DLHC_MAG_1_5HZ           0x01 //1.5 Hz
#define LSM303DLHC_MAG_3HZ             0x02 //3.0 Hz
#define LSM303DLHC_MAG_7_5HZ           0x03 //7.5 Hz
#define LSM303DLHC_MAG_15HZ            0x04 //15 Hz
#define LSM303DLHC_MAG_30HZ            0x05 //30 Hz
#define LSM303DLHC_MAG_75HZ            0x06 //75 Hz
#define LSM303DLHC_MAG_220HZ           0x07 //220 Hz

//calibration info
#define S_ACCEL_A                      0
#define S_GYRO                         1
#define S_MAG                          2
#define S_ACCEL_D                      3
//#define S_ECG                     3
//#define S_EMG                     4

//MPU9150/MPU9250 Gyro range
#define MPU9X50_GYRO_250DPS            0x00 //+/-250 dps
#define MPU9X50_GYRO_500DPS            0x01 //+/-500 dps
#define MPU9X50_GYRO_1000DPS           0x02 //+/-1000 dps
#define MPU9X50_GYRO_2000DPS           0x03 //+/-2000 dps

//#digital accel_range
#define RANGE_2G                       0
#define RANGE_4G                       1
#define RANGE_8G                       2
#define RANGE_16G                      3

//#mag_gain
#define LSM303_MAG_13GA                1
#define LSM303_MAG_19GA                2
#define LSM303_MAG_25GA                3
#define LSM303_MAG_40GA                4
#define LSM303_MAG_47GA                5
#define LSM303_MAG_56GA                6
#define LSM303_MAG_81GA                7

//choice of clock
#define TCXO_CLOCK                     (255765.625)
#define MSP430_CLOCK                   (32768.0)

//BMP Pressure oversampling ratio
#define BMPX80_OSS_1                   0x00
#define BMPX80_OSS_2                   0x01
#define BMPX80_OSS_4                   0x02
#define BMPX80_OSS_8                   0x03

#define CALIB_SYNC_SOURCE_STORECCONFIG 1
#define CALIB_SYNC_SOURCE_INFOMEM      2
#define CALIB_SYNC_SOURCE_SD_FILE      3
#define CALIB_SYNC_SOURCE_CALIBRAM     4

#endif
