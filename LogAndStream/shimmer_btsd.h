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
#define DEVICE_VER            3      //Represents SR30. 0-3 for shimmer1 to shimmer3
#define FW_IDENTIFIER         3      //Two byte firmware identifier number:  3 for BTSD, 2 for SDLog, 1 for BTStream,
#define FW_VER_MAJOR          0      //Major version number: 0-65535
#define FW_VER_MINOR          16     //Minor version number: 0-255
#define FW_VER_REL            11     //internal version number: 0-255

typedef uint8_t bool;
#define TRUE    (1)
#define FALSE   (0)

typedef uint8_t error_t;
#define SUCCESS (1)
#define FAIL    (0)

//SENSORS0
#define SENSOR_A_ACCEL                 0x80
#define SENSOR_MPU9X50_ICM20948_GYRO   0x40
#define SENSOR_LSM303XXXX_MAG              0x20
#define SENSOR_EXG1_24BIT              0x10
#define SENSOR_EXG2_24BIT              0x08
#define SENSOR_GSR                     0x04
#define SENSOR_EXT_A7                  0x02
#define SENSOR_EXT_A6                  0x01
//SENSORS1
#define SENSOR_STRAIN                  0x80
//#define SDH_SENSOR_HR                0x40
#define SENSOR_VBATT                   0x20
#define SENSOR_LSM303XXXX_ACCEL        0x10
#define SENSOR_EXT_A15                 0x08
#define SENSOR_INT_A1                  0x04
#define SENSOR_INT_A12                 0x02
#define SENSOR_INT_A13                 0x01
//SENORS2
#define SENSOR_INT_A14                 0x80
#define SENSOR_MPU9X50_ICM20948_ACCEL  0x40
#define SENSOR_MPU9X50_ICM20948_MAG    0x20
#define SENSOR_EXG1_16BIT              0x10
#define SENSOR_EXG2_16BIT              0x08
#define SENSOR_BMPX80_PRESSURE         0x04


//#define RESPONSE_PACKET_SIZE     131   //biggest possibly required  (daughter card mem read + 1 byte for ack)
#define MAX_NUM_CHANNELS         45    //3xanalogAccel + 3xdigiGyro + 3xdigiMag +
                                       //3xLSM303DLHCAccel + 3xMPU9150Accel + 3xMPU9150MAG +
                                       //BMPX80TEMP + BMPX80PRESS + batteryVoltage +
                                       //3xexternalADC + 4xinternalADC + (ExG)
#define DATA_PACKET_SIZE         84    //3 + (MAX_NUM_CHANNELS * 2) + 1 + 6 (+1 as BMPX80
                                       //pressure requires 3 bytes, +6 for 4 (3 byte) ExG
                                       //channels plus 2 status bytes instead of
                                       //4xinternalADC)

// Channel contents
#define X_A_ACCEL                         0x00
#define Y_A_ACCEL                         0x01
#define Z_A_ACCEL                         0x02
#define VBATT                             0x03
#define X_LSM303DLHC_ACCEL                0x04
#define Y_LSM303DLHC_ACCEL                0x05
#define Z_LSM303DLHC_ACCEL                0x06
#define X_LSM303DLHC_MAG                  0x07
#define Y_LSM303DLHC_MAG                  0x08
#define Z_LSM303DLHC_MAG                  0x09
#define X_MPU9150_GYRO                    0x0A
#define Y_MPU9150_GYRO                    0x0B
#define Z_MPU9150_GYRO                    0x0C
#define EXTERNAL_ADC_7                    0x0D
#define EXTERNAL_ADC_6                    0x0E
#define EXTERNAL_ADC_15                   0x0F
#define INTERNAL_ADC_1                    0x10
#define INTERNAL_ADC_12                   0x11
#define INTERNAL_ADC_13                   0x12
#define INTERNAL_ADC_14                   0x13
#define X_MPU9150_ACCEL                   0x14
#define Y_MPU9150_ACCEL                   0x15
#define Z_MPU9150_ACCEL                   0x16
#define X_MPU9150_MAG                     0x17
#define Y_MPU9150_MAG                     0x18
#define Z_MPU9150_MAG                     0x19
#define BMPX80_TEMP                       0x1A
#define BMPX80_PRESSURE                   0x1B
#define GSR_RAW                           0x1C
#define EXG_ADS1292R_1_STATUS             0x1D
#define EXG_ADS1292R_1_CH1_24BIT          0x1E
#define EXG_ADS1292R_1_CH2_24BIT          0x1F
#define EXG_ADS1292R_2_STATUS             0x20
#define EXG_ADS1292R_2_CH1_24BIT          0x21
#define EXG_ADS1292R_2_CH2_24BIT          0x22
#define EXG_ADS1292R_1_CH1_16BIT          0x23
#define EXG_ADS1292R_1_CH2_16BIT          0x24
#define EXG_ADS1292R_2_CH1_16BIT          0x25
#define EXG_ADS1292R_2_CH2_16BIT          0x26
#define STRAIN_HIGH                       0x27
#define STRAIN_LOW                        0x28


// Infomem contents
//#define NV_NUM_CONFIG_BYTES             100
// Infomem contents
#define NV_NUM_SETTINGS_BYTES             34
#define NV_NUM_CALIBRATION_BYTES          84
#define NV_NUM_SD_BYTES                   37
#define NV_TOTAL_NUM_CONFIG_BYTES         384//NV_NUM_SETTINGS_BYTES + NV_NUM_CALIBRATION_BYTES + NV_NUM_SD_BYTES
#define NV_NUM_RWMEM_BYTES                512
#define NV_NUM_BYTES_SYNC_CENTER_NODE_ADDRS 126

#define NV_SAMPLING_RATE                  0
#define NV_BUFFER_SIZE                    2
#define NV_SENSORS0                       3
#define NV_SENSORS1                       4
#define NV_SENSORS2                       5
#define NV_CONFIG_SETUP_BYTE0             6 //sensors setting bytes
#define NV_CONFIG_SETUP_BYTE1             7
#define NV_CONFIG_SETUP_BYTE2             8
#define NV_CONFIG_SETUP_BYTE3             9
#define NV_EXG_ADS1292R_1_CONFIG1         10// exg bytes, not implemented yet
#define NV_EXG_ADS1292R_1_CONFIG2         11
#define NV_EXG_ADS1292R_1_LOFF            12
#define NV_EXG_ADS1292R_1_CH1SET          13
#define NV_EXG_ADS1292R_1_CH2SET          14
#define NV_EXG_ADS1292R_1_RLD_SENS        15
#define NV_EXG_ADS1292R_1_LOFF_SENS       16
#define NV_EXG_ADS1292R_1_LOFF_STAT       17
#define NV_EXG_ADS1292R_1_RESP1           18
#define NV_EXG_ADS1292R_1_RESP2           19
#define NV_EXG_ADS1292R_2_CONFIG1         20
#define NV_EXG_ADS1292R_2_CONFIG2         21
#define NV_EXG_ADS1292R_2_LOFF            22
#define NV_EXG_ADS1292R_2_CH1SET          23
#define NV_EXG_ADS1292R_2_CH2SET          24
#define NV_EXG_ADS1292R_2_RLD_SENS        25
#define NV_EXG_ADS1292R_2_LOFF_SENS       26
#define NV_EXG_ADS1292R_2_LOFF_STAT       27
#define NV_EXG_ADS1292R_2_RESP1           28
#define NV_EXG_ADS1292R_2_RESP2           29
#define NV_BT_COMMS_BAUD_RATE             30
#define NV_DERIVED_CHANNELS_0             31
#define NV_DERIVED_CHANNELS_1             32
#define NV_DERIVED_CHANNELS_2             33
#define NV_A_ACCEL_CALIBRATION            34
#define NV_MPU9150_GYRO_CALIBRATION       55
#define NV_LSM303DLHC_MAG_CALIBRATION     76
#define NV_LSM303DLHC_ACCEL_CALIBRATION   97       //97->117
#define NV_CALIBRATION_END                117
#define NV_DERIVED_CHANNELS_3             118
#define NV_DERIVED_CHANNELS_4             119
#define NV_DERIVED_CHANNELS_5             120
#define NV_DERIVED_CHANNELS_6             121
#define NV_DERIVED_CHANNELS_7             122

#define NV_SENSORS3                       (128+0)
#define NV_SENSORS4                       (128+1)
#define NV_CONFIG_SETUP_BYTE4             (128+2)
#define NV_CONFIG_SETUP_BYTE5             (128+3)
#define NV_CONFIG_SETUP_BYTE6             (128+4)
#define NV_MPL_ACCEL_CALIBRATION          (128+5)    //+21
#define NV_MPL_MAG_CALIBRATION            (128+26)   //+21
#define NV_MPL_GYRO_CALIBRATION           (128+47)   //+12
#define NV_SD_SHIMMER_NAME                (128+59)   // +12 bytes
#define NV_SD_EXP_ID_NAME                 (128+71)   // +12 bytes
#define NV_SD_CONFIG_TIME                 (128+83)   // +4 bytes
#define NV_SD_MYTRIAL_ID                  (128+87)   // 1 byte
#define NV_SD_NSHIMMER                    (128+88)   // 1 byte
#define NV_SD_TRIAL_CONFIG0               (128+89)
#define NV_SD_TRIAL_CONFIG1               (128+90)
#define NV_SD_BT_INTERVAL                 (128+91)
#define NV_EST_EXP_LEN_MSB                (128+92)  // 2bytes
#define NV_EST_EXP_LEN_LSB                (128+93)
#define NV_MAX_EXP_LEN_MSB                (128+94)  // 2bytes
#define NV_MAX_EXP_LEN_LSB                (128+95)
#define NV_MAC_ADDRESS                    (128+96)   // 6bytes
#define NV_SD_CONFIG_DELAY_FLAG           (128+102)
#define NV_BT_SET_PIN                     (128+103)
#define NV_TEMP_PRES_CALIBRATION          (128+104) // +22 bytes, till 128+125

#define NV_CENTER                         (128+128+0)
#define NV_NODE0                          (128+128+6)

//Config byte masks
//Config Byte0
#define LSM303DLHC_ACCEL_SAMPLING_RATE          0xF0
#define LSM303DLHC_ACCEL_RANGE                  0x0C
#define LSM303DLHC_ACCEL_LOW_POWER_MODE         0x02
#define LSM303DLHC_ACCEL_HIGH_RESOLUTION_MODE   0x01
//Config Byte1
#define MPU9150_SAMPLING_RATE                   0xFF
//Config Byte2
#define LSM303DLHC_MAG_GAIN                     0xE0
#define LSM303DLHC_MAG_SAMPLING_RATE            0x1C
#define MPU9150_GYRO_RANGE                      0x03
//Config Byte3
#define MPU9150_ACCEL_RANGE                     0xC0
#define BMPX80_PRESSURE_RESOLUTION              0x30
#define GSR_RANGE                               0x0E
#define EXP_POWER_ENABLE                        0x01
//Unused bits 3-0


//ADC initialisation mask
#define MASK_A_ACCEL       0x0001
#define MASK_VBATT         0x0002
#define MASK_EXT_A7        0x0004
#define MASK_EXT_A6        0x0008
#define MASK_EXT_A15       0x0010
#define MASK_INT_A1        0x0020
#define MASK_INT_A12       0x0040
#define MASK_INT_A13       0x0080
#define MASK_INT_A14       0x0100
#define MASK_MSP_TEMP      0x0200
#define MASK_STRAIN        0x0180   //uses ADC13 and ADC14


//LSM303DLHC Accel Range
//Corresponds to the FS field of the LSM303DLHC's CTRL_REG4_A register
//and the AFS_SEL field of the MPU9150's ACCEL_CONFIG register
#define ACCEL_2G      0x00
#define ACCEL_4G      0x01
#define ACCEL_8G      0x02
#define ACCEL_16G     0x03

//LSM303DLHC Accel Sampling Rate
//Corresponds to the ODR field of the LSM303DLHC's CTRL_REG1_A register
#define LSM303DLHC_ACCEL_POWER_DOWN 0x00
#define LSM303DLHC_ACCEL_1HZ        0x01
#define LSM303DLHC_ACCEL_10HZ       0x02
#define LSM303DLHC_ACCEL_25HZ       0x03
#define LSM303DLHC_ACCEL_50HZ       0x04
#define LSM303DLHC_ACCEL_100HZ      0x05
#define LSM303DLHC_ACCEL_200HZ      0x06
#define LSM303DLHC_ACCEL_400HZ      0x07
#define LSM303DLHC_ACCEL_1_620KHZ   0x08 //1.620kHz in Low-power mode only
#define LSM303DLHC_ACCEL_1_344kHz   0x09 //1.344kHz in normal mode, 5.376kHz in low-power mode

//LSM303DLHC Mag gain
#define LSM303DLHC_MAG_1_3G         0x01 //+/-1.3 Gauss
#define LSM303DLHC_MAG_1_9G         0x02 //+/-1.9 Gauss
#define LSM303DLHC_MAG_2_5G         0x03 //+/-2.5 Gauss
#define LSM303DLHC_MAG_4_0G         0x04 //+/-4.0 Gauss
#define LSM303DLHC_MAG_4_7G         0x05 //+/-4.7 Gauss
#define LSM303DLHC_MAG_5_6G         0x06 //+/-5.6 Gauss
#define LSM303DLHC_MAG_8_1G         0x07 //+/-8.1 Gauss

//LSM303DLHC Mag sampling rate
#define LSM303DLHC_MAG_0_75HZ       0x00 //0.75 Hz
#define LSM303DLHC_MAG_1_5HZ        0x01 //1.5 Hz
#define LSM303DLHC_MAG_3HZ          0x02 //3.0 Hz
#define LSM303DLHC_MAG_7_5HZ        0x03 //7.5 Hz
#define LSM303DLHC_MAG_15HZ         0x04 //15 Hz
#define LSM303DLHC_MAG_30HZ         0x05 //30 Hz
#define LSM303DLHC_MAG_75HZ         0x06 //75 Hz
#define LSM303DLHC_MAG_220HZ        0x07 //220 Hz


//calibration info
#define S_ACCEL_A                   0
#define S_GYRO                      1
#define S_MAG                       2
#define S_ACCEL_D                   3
//#define S_ECG                     3
//#define S_EMG                     4

//MPU9150 Gyro range
#define MPU9150_GYRO_250DPS         0x00 //+/-250 dps
#define MPU9150_GYRO_500DPS         0x01 //+/-500 dps
#define MPU9150_GYRO_1000DPS        0x02 //+/-1000 dps
#define MPU9150_GYRO_2000DPS        0x03 //+/-2000 dps

//#digital accel_range
#define RANGE_2G                    0
#define RANGE_4G                    1
#define RANGE_8G                    2
#define RANGE_16G                   3

//#mag_gain
#define LSM303_MAG_13GA             1
#define LSM303_MAG_19GA             2
#define LSM303_MAG_25GA             3
#define LSM303_MAG_40GA             4
#define LSM303_MAG_47GA             5
#define LSM303_MAG_56GA             6
#define LSM303_MAG_81GA             7



//SD Log file header format
#define SDHEAD_LEN                  256// 0-255

#define SDH_SAMPLE_RATE_0           (0)
#define SDH_SAMPLE_RATE_1           (1)
#define SDH_BUFFER_SIZE             (2)
#define SDH_SENSORS0                (3)
#define SDH_SENSORS1                (4)
#define SDH_SENSORS2                (5)
#define SDH_SENSORS3                (6)
#define SDH_SENSORS4                (7)
#define SDH_CONFIG_SETUP_BYTE0      (8)//sensors setting bytes
#define SDH_CONFIG_SETUP_BYTE1      (9)
#define SDH_CONFIG_SETUP_BYTE2      (10)
#define SDH_CONFIG_SETUP_BYTE3      (11)
#define SDH_CONFIG_SETUP_BYTE4      (12)
#define SDH_CONFIG_SETUP_BYTE5      (13)
#define SDH_CONFIG_SETUP_BYTE6      (14)
#define SDH_TRIAL_CONFIG0           (16)
#define SDH_TRIAL_CONFIG1           (17)
#define SDH_BROADCAST_INTERVAL      (18)
#define SDH_BT_COMMS_BAUD_RATE      (19)
#define SDH_EST_EXP_LEN_MSB         (20)
#define SDH_EST_EXP_LEN_LSB         (21)
#define SDH_MAX_EXP_LEN_MSB         (22)
#define SDH_MAX_EXP_LEN_LSB         (23)
#define SDH_MAC_ADDR                (24)
#define SDH_SHIMMERVERSION_BYTE_0   (30)
#define SDH_SHIMMERVERSION_BYTE_1   (31)
#define SDH_MYTRIAL_ID              (32)
#define SDH_NSHIMMER                (33)
#define SDH_FW_VERSION_TYPE_0       (34)
#define SDH_FW_VERSION_TYPE_1       (35)
#define SDH_FW_VERSION_MAJOR_0      (36)
#define SDH_FW_VERSION_MAJOR_1      (37)
#define SDH_FW_VERSION_MINOR        (38)
#define SDH_FW_VERSION_INTERNAL     (39)
#define SDH_DERIVED_CHANNELS_0      (40)
#define SDH_DERIVED_CHANNELS_1      (41)
#define SDH_DERIVED_CHANNELS_2      (42)
#define SDH_RTC_DIFF_0              (44)
#define SDH_RTC_DIFF_1              (45)
#define SDH_RTC_DIFF_2              (46)
#define SDH_RTC_DIFF_3              (47)
#define SDH_RTC_DIFF_4              (48)
#define SDH_RTC_DIFF_5              (49)
#define SDH_RTC_DIFF_6              (50)
#define SDH_RTC_DIFF_7              (51)
#define SDH_CONFIG_TIME_0           (52)
#define SDH_CONFIG_TIME_1           (53)
#define SDH_CONFIG_TIME_2           (54)
#define SDH_CONFIG_TIME_3           (55)
#define SDH_EXG_ADS1292R_1_CONFIG1        (56)
#define SDH_EXG_ADS1292R_1_CONFIG2        (57)
#define SDH_EXG_ADS1292R_1_LOFF           (58)
#define SDH_EXG_ADS1292R_1_CH1SET         (59)
#define SDH_EXG_ADS1292R_1_CH2SET         (60)
#define SDH_EXG_ADS1292R_1_RLD_SENS       (61)
#define SDH_EXG_ADS1292R_1_LOFF_SENS      (62)
#define SDH_EXG_ADS1292R_1_LOFF_STAT      (63)
#define SDH_EXG_ADS1292R_1_RESP1          (64)
#define SDH_EXG_ADS1292R_1_RESP2          (65)
#define SDH_EXG_ADS1292R_2_CONFIG1        (66)
#define SDH_EXG_ADS1292R_2_CONFIG2        (67)
#define SDH_EXG_ADS1292R_2_LOFF           (68)
#define SDH_EXG_ADS1292R_2_CH1SET         (69)
#define SDH_EXG_ADS1292R_2_CH2SET         (70)
#define SDH_EXG_ADS1292R_2_RLD_SENS       (71)
#define SDH_EXG_ADS1292R_2_LOFF_SENS      (72)
#define SDH_EXG_ADS1292R_2_LOFF_STAT      (73)
#define SDH_EXG_ADS1292R_2_RESP1          (74)
#define SDH_EXG_ADS1292R_2_RESP2          (75)
#define SDH_LSM303DLHC_ACCEL_CALIBRATION  (76)//0x4c
#define SDH_MPU9150_GYRO_CALIBRATION      (97) //0x61
#define SDH_LSM303DLHC_MAG_CALIBRATION    (118)//0x76
#define SDH_A_ACCEL_CALIBRATION           (139)//0x8b
#define SDH_TEMP_PRES_CALIBRATION         (160)
#define SDH_LSM303DLHC_ACCEL_CALIB_TS     (182)//+8
#define SDH_MPU9150_GYRO_CALIB_TS         (190) //+8
#define SDH_LSM303DLHC_MAG_CALIB_TS       (198) //+8
#define SDH_A_ACCEL_CALIB_TS              (206) //+8
#define SDH_DAUGHTER_CARD_ID_BYTE0        (214) //+3
#define SDH_DERIVED_CHANNELS_3            (217)
#define SDH_DERIVED_CHANNELS_4            (218)
#define SDH_DERIVED_CHANNELS_5            (219)
#define SDH_DERIVED_CHANNELS_6            (220)
#define SDH_DERIVED_CHANNELS_7            (221)
#define BMP280_XTRA_CALIB_BYTES           (222)
#define SDH_MY_LOCALTIME_5TH              (251)
#define SDH_MY_LOCALTIME                  (252)  //252-255


//SENSORS0
#define SDH_SENSOR_A_ACCEL           0x80
#define SDH_SENSOR_MPU9150_GYRO        0x40
#define SDH_SENSOR_LSM303DLHC_MAG      0x20
#define SDH_SENSOR_EXG1_24BIT          0x10
#define SDH_SENSOR_EXG2_24BIT          0x08
#define SDH_SENSOR_GSR                 0x04
#define SDH_SENSOR_EXTCH7              0x02
#define SDH_SENSOR_EXTCH6              0x01
//SENSORS1
#define SDH_SENSOR_STRAIN              0x80
//#define SDH_SENSOR_HR                0x40
#define SDH_SENSOR_VBATT               0x20
#define SDH_SENSOR_LSM303DLHC_ACCEL    0x10
#define SDH_SENSOR_EXTCH15             0x08
#define SDH_SENSOR_INTCH1              0x04
#define SDH_SENSOR_INTCH12             0x02
#define SDH_SENSOR_INTCH13             0x01
//SENSORS2
#define SDH_SENSOR_INTCH14             0x80
#define SDH_SENSOR_MPU9150_ACCEL       0x40
#define SDH_SENSOR_MPU9150_MAG         0x20
#define SDH_SENSOR_EXG1_16BIT          0x10
#define SDH_SENSOR_EXG2_16BIT          0x08
#define SDH_SENSOR_BMPX80_PRES         0x04
//#define SDH_SENSOR_BMPX80_TEMP       0x02
//SENSORS3
#define SDH_SENSOR_MSP430_TEMP         0x01
#define SDH_SENSOR_TCXO                0x80

//SDH_TRIAL_CONFIG0
#define SDH_SDERROR_EN                 0x01
#define SDH_IAMMASTER                  0x02
#define SDH_TIME_SYNC                  0x04
//#define SDH_TIME_STAMP                 0x08// not used now, reserved as 1
#define SDH_BLUETOOTH_DISABLE          0x08
#define SDH_RWCERROR_EN                0x10// when 0, won't flash error. when 1, will flash error if RTC isn't set (RTC_offset == 0)
#define SDH_USER_BUTTON_ENABLE         0x20
#define SDH_SET_PMUX                   0x40// not used now, reserved as 0
#define SDH_RTC_SET_BY_BT              0x80

//SDH_TRIAL_CONFIG1
#define SDH_BATT_CRITICAL_CUTOFF       0x01
#define SDH_TCXO                       0x10
#define SDH_SINGLETOUCH                0x80

//choice of clock
#define TCXO_CLOCK      (255765.625)
#define MSP430_CLOCK    (32768.0)

// sd card write buffer size
#define SDBUFF_SIZE_MAX 4096 //4095  255
#define SDBUFF_SIZE     512 //4095  512

// BATTERY
#define BATT_INTERVAL   1966080  // 19660800 = 10min interval
#define BATT_INTERVAL_D 65535

#define MAX_NODES       20
#define MAX_CHARS       13
#define UINT32_LEN      11 // 10+1, where the last byte should be 0x00
#define UINT64_LEN      21 // 20+1, where the last byte should be 0x00
#define RESPONSE_PACKET_SIZE 133

//BMP Pressure oversampling ratio
#define BMPX80_OSS_1                0x00
#define BMPX80_OSS_2                0x01
#define BMPX80_OSS_4                0x02
#define BMPX80_OSS_8                0x03

//BtStream specific extension to range values : should SDLog keep it?
#define GSR_AUTORANGE               0x04

#define CALIB_SYNC_SOURCE_STORECCONFIG       1
#define CALIB_SYNC_SOURCE_INFOMEM            2
#define CALIB_SYNC_SOURCE_SD_FILE            3
#define CALIB_SYNC_SOURCE_CALIBRAM           4

typedef enum
{
   TASK_SETUP_DOCK          = (0x00000001UL << 0U),
   TASK_BATT_READ           = (0x00000001UL << 1U),
   TASK_DOCK_PROCESS_CMD    = (0x00000001UL << 2U),
   TASK_DOCK_RESPOND        = (0x00000001UL << 3U),
   TASK_BT_PROCESS_CMD      = (0x00000001UL << 4U),
   TASK_CFGCH               = (0x00000001UL << 5U),
   TASK_BT_RESPOND          = (0x00000001UL << 6U),
   TASK_RCCENTERR1          = (0x00000001UL << 7U),
   TASK_RCNODER10           = (0x00000001UL << 8U),
   TASK_SAMPLE_MPU9150_MAG  = (0x00000001UL << 9U),
   TASK_SAMPLE_BMPX80_PRESS = (0x00000001UL << 10U),
   TASK_STREAMDATA          = (0x00000001UL << 11U),
   TASK_SDLOG_CFG_UPDATE    = (0x00000001UL << 12U),
//   TASK_STOPSENSING         = (0x00000001UL << 13U),
   TASK_STARTSENSING        = (0x00000001UL << 14U),
   TASK_WR2SD               = (0x00000001UL << 15U),
   TASK_FACTORY_TEST        = (0x00000001UL << 16U)
} TASK_FLAGS;
#define TASK_SIZE    32

#endif
