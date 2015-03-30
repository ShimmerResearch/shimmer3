/*
 * Copyright (c) 2015, Shimmer Research, Ltd.
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
 * @author Weibo Pan
 * @date March, 2015
 *
 * @modifed Mark Nolan
 * @date June, 2014
 *
 */
#ifndef SHIMMER_SD_H
#define SHIMMER_SD_H


//these are defined in the Makefile for BtStream (TinyOS)
#define DEVICE_VER         3     //Represents shimmer3. 0-3 for shimmer1 to shimmer3
#define FW_IDENTIFIER      2     //Two byte firmware identifier number: always 2 for SDLog
#define FW_VER_MAJOR       0     //Maor version number: 0-65535
#define FW_VER_MINOR       9     //Minor version number: 0-255
#define FW_VER_REL         0    //internal version number: 0-255
// Packet Types
#define DATA_PACKET                       0x00
#define ACK_COMMAND_PROCESSED             0xFF
#define ROUTINE_COMMUNICATION             0xE0   //'0'

// UART COMMANDS
//================= WP uart 3.0: command names ==============
#define UART_STEP_WAIT4_CMD   4
#define UART_STEP_WAIT4_LEN   3
#define UART_STEP_WAIT4_DATA  2
#define UART_STEP_WAIT4_CRC   1
#define UART_STEP_WAIT4_NONE  0

#define UART_RXBUF_START      0
#define UART_RXBUF_CMD        1
#define UART_RXBUF_LEN        2
#define UART_RXBUF_COMP       3
#define UART_RXBUF_PROP       4  // data in rxbuf starts from byte 3
#define UART_RXBUF_DATA       5  // data in rxbuf starts from byte 3
#define UART_DATA_LEN_MAX     138// max case: '$' + get + length + comp_shimmer+ prop_infomem
#define UART_RSP_PACKET_SIZE  138// + info_len + info_loc*2 + 128bytes data + crc*2 = 138


#define UART_SET                    0x01
#define UART_RESPONSE               0x02
#define UART_GET                    0x03
#define UART_BAD_CMD_RESPONSE       0xfc //252
#define UART_BAD_ARG_RESPONSE       0xfd //253
#define UART_BAD_CRC_RESPONSE       0xfe //254
#define UART_ACK_RESPONSE           0xff //255
//================= WP uart 3.0: components names ==============
#define UART_COMP_SHIMMER           0x01
#define UART_COMP_BAT               0x02 // this is seen as a sensor
#define UART_COMP_DAUGHTER_CARD     0x03
#define UART_COMP_D_ACCEL           0x04
#define UART_COMP_GSR               0x05
//================= WP uart 3.0: property names ==============
// component == UART_COMP_SHIMMER:
#define UART_PROP_ENABLE            0x00 // this is for all sensors
#define UART_PROP_SAMPLE_RATE       0x01
#define UART_PROP_MAC               0x02
#define UART_PROP_VER               0x03
#define UART_PROP_RWC_CFG_TIME      0x04
#define UART_PROP_CURR_LOCAL_TIME   0x05
#define UART_PROP_INFOMEM           0x06
// component == UART_COMP_BAT:
//#define UART_PROP_SAMPLE_RATE       0x01
#define UART_PROP_VALUE             0x02
//#define UART_PROP_DIVIDER           0x05
// component == UART_COMP_DAUGHTER_CARD:
#define UART_PROP_CARD_ID           0x02
#define UART_PROP_CARD_MEM          0x03
// component == UART_COMP_D_ACCEL:
//#define UART_PROP_ENABLE            0x00
//#define UART_PROP_SAMPLE_RATE       0x01
#define UART_PROP_DATA_RATE         0x02
#define UART_PROP_RANGE             0x03
#define UART_PROP_LP_MODE           0x04
#define UART_PROP_HR_MODE           0x05
#define UART_PROP_FREQ_DIVIDER      0x06
#define UART_PROP_CALIBRATION       0x07
// component == UART_COMP_GSR:
//#define UART_PROP_ENABLE            0x00
//#define UART_PROP_SAMPLE_RATE       0x01
//#define UART_PROP_RANGE             0x03
//#define UART_PROP_DIVIDER           0x05
//== new uart ends ==
#define CBUF_SIZE                   27
#define CBUF_PARAM_LEN_MAX          CBUF_SIZE-6
// UART OLD COMMANDS
#define UART_CMD_MAC 1
#define UART_CMD_VER 2
#define UART_CMD_BAT 3
#define UART_CMD_MEM 4
#define UART_CMD_RTC 5
#define UART_CMD_RCT 6
#define UART_CMD_RDT 7
#define UART_CMD_TIM 8

//SENSORS0
#define SENSOR_A_ACCEL                    0x80
#define SENSOR_MPU9150_GYRO               0x40
#define SENSOR_LSM303DLHC_MAG             0x20
#define SENSOR_EXG1_24BIT                 0x10
#define SENSOR_EXG2_24BIT                 0x08
#define SENSOR_GSR                        0x04
#define SENSOR_EXT_A7                     0x02
#define SENSOR_EXT_A6                     0x01
//SENSORS1
#define SENSOR_STRAIN                     0x80   //higher priority than SENSOR_INT_A13 and SENSOR_INT_A14
#define SENSOR_VBATT                      0x20
#define SENSOR_LSM303DLHC_ACCEL           0x10
#define SENSOR_EXT_A15                    0x08
#define SENSOR_INT_A1                     0x04
#define SENSOR_INT_A12                    0x02
#define SENSOR_INT_A13                    0x01
//SENORS2
#define SENSOR_INT_A14                    0x80
#define SENSOR_MPU9150_ACCEL              0x40
#define SENSOR_MPU9150_MAG                0x20
#define SENSOR_EXG1_16BIT                 0x10
#define SENSOR_EXG2_16BIT                 0x08
#define SENSOR_BMP180_PRESSURE            0x04
#define SENSOR_MPU9150_TEMP               0x02

//SENSORS3
#define SENSOR_MPU9150_MPL_QUAT_6DOF      0x80
#define SENSOR_MPU9150_MPL_QUAT_9DOF      0x40
#define SENSOR_MPU9150_MPL_EULER_6DOF     0x20
#define SENSOR_MPU9150_MPL_EULER_9DOF     0x10
#define SENSOR_MPU9150_MPL_HEADING        0x08
#define SENSOR_MPU9150_MPL_PEDOMETER      0x04
#define SENSOR_MPU9150_MPL_TAP            0x02
#define SENSOR_MPU9150_MPL_MOTION_ORIENT  0x01

//SENSORS4
#define SENSOR_MPU9150_GYRO_CAL           0x80
#define SENSOR_MPU9150_ACCEL_CAL          0x40
#define SENSOR_MPU9150_MAG_CAL            0x20
#define SENSOR_MPU9150_MPL_QUAT_6DOF_RAW  0x10

#define MAX_COMMAND_ARG_SIZE              21       //maximum number of arguments for any command sent to SR30 (calibration data)
//--#define RESPONSE_PACKET_SIZE          85      //biggest possibly required (4 x kinematic calibration responses)
//--#define MAX_NUM_CHANNELS              29       //3xanalogAccel + 3xdigiGyro + 3xdigiMag + 3xLSM303DLHCAccel + 3xMPU9150Accel + batteryVoltage + 3xexternalADC + 4xinternalADC
#define DATA_PACKET_SIZE                  80      //4 + (MAX_NUM_CHANNELS * 2)


// Infomem contents
#define NV_NUM_SETTINGS_BYTES             39
#define NV_NUM_CALIBRATION_BYTES          138+2 // 138 bytes plus buffer of 2 (just in case)
#define NV_TOTAL_NUM_CONFIG_BYTES         384


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
#define NV_LSM303DLHC_ACCEL_CALIBRATION   97 //97->117

#define NV_SENSORS3                       128+0
#define NV_SENSORS4                       128+1
#define NV_CONFIG_SETUP_BYTE4             128+2
#define NV_CONFIG_SETUP_BYTE5             128+3
#define NV_CONFIG_SETUP_BYTE6             128+4
#define NV_MPL_ACCEL_CALIBRATION          128+5 //+21
#define NV_MPL_MAG_CALIBRATION            128+26 //+21
#define NV_MPL_GYRO_CALIBRATION           128+47 //+12
#define NV_SD_SHIMMER_NAME                128+59   // +12 bytes
#define NV_SD_EXP_ID_NAME                 128+71   // +12 bytes
#define NV_SD_CONFIG_TIME                 128+83   // +4 bytes
#define NV_SD_MYTRIAL_ID                  128+87   // 1 byte
#define NV_SD_NSHIMMER                    128+88   // 1 byte
#define NV_SD_TRIAL_CONFIG0               128+89
#define NV_SD_TRIAL_CONFIG1               128+90
#define NV_SD_BT_INTERVAL                 128+91
#define NV_EST_EXP_LEN_MSB                128+92 // 2bytes
#define NV_EST_EXP_LEN_LSB                128+93
#define NV_MAX_EXP_LEN_MSB                128+94 // 2bytes
#define NV_MAX_EXP_LEN_LSB                128+95
#define NV_MAC_ADDRESS                    128+96 // 6bytes
#define NV_SD_CONFIG_DELAY_FLAG           128+102

#define NV_CENTER                         128+128+0
#define NV_NODE0                          128+128+6

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
#define BMP180_PRESSURE_RESOLUTION              0x30
#define GSR_RANGE                               0x0E
#define EXP_POWER_ENABLE                        0x01
//Unused bits 3-0

//Config Byte4
#define MPU9150_MPL_DMP               0x80
#define MPU9150_MPL_USE_LSM303_MAG    0x40
#define MPU9150_MPL_LPF               0x38
#define MPU9150_MOT_CAL_CFG           0x07
//Config Byte5
#define MPU9150_MPL_SAMPLING_RATE     0xE0
#define MPU9150_MAG_SAMPLING_RATE     0x1C
#define MPU9150_MPL_MAG_MIX           0x03
//Config Byte6
#define MPU9150_MPL_SENS_FUSION       0x80
#define MPU9150_MPL_GYRO_CAL_TC       0x40
#define MPU9150_MPL_VECT_COMP_CAL     0x20
#define MPU9150_MPL_MAG_DIST_CAL      0x10
#define MPU9150_MPL_ENABLE            0x08

//ADC initialisation mask
#define MASK_A_ACCEL       0x0001
#define MASK_VBATT         0x0002
#define MASK_EXT_A7        0x0004
#define MASK_EXT_A6        0x0008
#define MASK_EXT_A15       0x0010
#define MASK_INT_A1        0x0020
#define MASK_GSR           0x0020   //uses ADC1
#define MASK_INT_A12       0x0040
#define MASK_INT_A13       0x0080
#define MASK_INT_A14       0x0100
#define MASK_STRAIN        0x0180   //uses ADC13 and ADC14

//LSM303DLHC Accel Range
//Corresponds to the FS field of the LSM303DLHC's CTRL_REG4_A register
//and the AFS_SEL field of the MPU9150's ACCEL_CONFIG register
#define ACCEL_2G                    0x00
#define ACCEL_4G                    0x01
#define ACCEL_8G                    0x02
#define ACCEL_16G                   0x03

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
#define S_MPL_ACCEL                 4
#define S_MPL_MAG                   5
#define S_MPL_GYRO                  6

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

// MPU Low Pass filter cut-off
#define MPU9150_LPF_256HZ_NOLPF2    0x00
#define MPU9150_LPF_188HZ           0x01
#define MPU9150_LPF_98HZ            0x02
#define MPU9150_LPF_42HZ            0x03
#define MPU9150_LPF_20HZ            0x04
#define MPU9150_LPF_10HZ            0x05
#define MPU9150_LPF_5HZ             0x06
#define MPU9150_LPF_2500HZ_NOLPF    0x07

// On-the-fly gyro calibration settings
#define MPL_MOT_CAL_OFF             0x00
#define MPL_MOT_CAL_FAST_NO_MOT     0x01
#define MPL_MOT_CAL_MOT_NO_MOT_1S   0x02
#define MPL_MOT_CAL_MOT_NO_MOT_2S   0x03
#define MPL_MOT_CAL_MOT_NO_MOT_5S   0x04
#define MPL_MOT_CAL_MOT_NO_MOT_10S  0x05
#define MPL_MOT_CAL_MOT_NO_MOT_30S  0x06
#define MPL_MOT_CAL_MOT_NO_MOT_60S  0x07

#define MPL_RATE_10HZ               0x00
#define MPL_RATE_20HZ               0x01
#define MPL_RATE_40HZ               0x02
#define MPL_RATE_50HZ               0x03
#define MPL_RATE_100HZ              0x04

//Pansenti 9DOF MagMix
#define GYRO_ONLY                   0x00
#define MAG_ONLY                    0x01
#define GYRO_AND_MAG                0x02
#define GYRO_AND_SOME_MAG           0x03


//SD Log file header format
#define SDHEAD_LEN                        256 //(0-255)

#define SDH_SAMPLE_RATE_0                 0
#define SDH_SAMPLE_RATE_1                 1
#define SDH_SENSORS0                      3
#define SDH_SENSORS1                      4
#define SDH_SENSORS2                      5
#define SDH_SENSORS3                      6
#define SDH_SENSORS4                      7
#define SDH_CONFIG_SETUP_BYTE0            8 //sensors setting bytes
#define SDH_CONFIG_SETUP_BYTE1            9
#define SDH_CONFIG_SETUP_BYTE2            10
#define SDH_CONFIG_SETUP_BYTE3            11
#define SDH_CONFIG_SETUP_BYTE4            12
#define SDH_CONFIG_SETUP_BYTE5            13
#define SDH_CONFIG_SETUP_BYTE6            14
#define SDH_TRIAL_CONFIG0                 16
#define SDH_TRIAL_CONFIG1                 17
#define SDH_BROADCAST_INTERVAL            18
#define SDH_BT_COMMS_BAUD_RATE            19
#define SDH_EST_EXP_LEN_MSB               20
#define SDH_EST_EXP_LEN_LSB               21
#define SDH_MAX_EXP_LEN_MSB               22
#define SDH_MAX_EXP_LEN_LSB               23
#define SDH_MAC_ADDR                      24 //24-29, 6 bytes
#define SDH_SHIMMERVERSION_BYTE_0         30
#define SDH_SHIMMERVERSION_BYTE_1         31
#define SDH_MYTRIAL_ID                    32
#define SDH_NSHIMMER                      33
#define SDH_FW_VERSION_TYPE_0             34
#define SDH_FW_VERSION_TYPE_1             35
#define SDH_FW_VERSION_MAJOR_0            36
#define SDH_FW_VERSION_MAJOR_1            37
#define SDH_FW_VERSION_MINOR              38
#define SDH_FW_VERSION_INTERNAL           39 //46-51
#define SDH_DERIVED_CHANNELS_0            40
#define SDH_DERIVED_CHANNELS_1            41
#define SDH_DERIVED_CHANNELS_2            42
#define SDH_RTC_DIFF_0                    44
#define SDH_RTC_DIFF_1                    45
#define SDH_RTC_DIFF_2                    46
#define SDH_RTC_DIFF_3                    47
#define SDH_RTC_DIFF_4                    48
#define SDH_RTC_DIFF_5                    49
#define SDH_RTC_DIFF_6                    50
#define SDH_RTC_DIFF_7                    51
#define SDH_CONFIG_TIME_0                 52
#define SDH_CONFIG_TIME_1                 53
#define SDH_CONFIG_TIME_2                 54
#define SDH_CONFIG_TIME_3                 55
#define SDH_EXG_ADS1292R_1_CONFIG1        56
#define SDH_EXG_ADS1292R_1_CONFIG2        57
#define SDH_EXG_ADS1292R_1_LOFF           58
#define SDH_EXG_ADS1292R_1_CH1SET         59
#define SDH_EXG_ADS1292R_1_CH2SET         60
#define SDH_EXG_ADS1292R_1_RLD_SENS       61
#define SDH_EXG_ADS1292R_1_LOFF_SENS      62
#define SDH_EXG_ADS1292R_1_LOFF_STAT      63
#define SDH_EXG_ADS1292R_1_RESP1          64
#define SDH_EXG_ADS1292R_1_RESP2          65
#define SDH_EXG_ADS1292R_2_CONFIG1        66
#define SDH_EXG_ADS1292R_2_CONFIG2        67
#define SDH_EXG_ADS1292R_2_LOFF           68
#define SDH_EXG_ADS1292R_2_CH1SET         69
#define SDH_EXG_ADS1292R_2_CH2SET         70
#define SDH_EXG_ADS1292R_2_RLD_SENS       71
#define SDH_EXG_ADS1292R_2_LOFF_SENS      72
#define SDH_EXG_ADS1292R_2_LOFF_STAT      73
#define SDH_EXG_ADS1292R_2_RESP1          74
#define SDH_EXG_ADS1292R_2_RESP2          75
#define SDH_LSM303DLHC_ACCEL_CALIBRATION  76 //0x4c
#define SDH_MPU9150_GYRO_CALIBRATION      97 //0x61
#define SDH_LSM303DLHC_MAG_CALIBRATION    118//0x76
#define SDH_A_ACCEL_CALIBRATION           139//0x8b
#define SDH_TEMP_PRES_CALIBRATION         160
#define SDH_MY_LOCALTIME_5TH              251   //252-255
#define SDH_MY_LOCALTIME_L                252   //252-255

#define SDH_MPL_ACCEL_CALIBRATION         182
#define SDH_MPL_MAG_CALIBRATION           203
#define SDH_MPL_GYRO_CALIBRATION          224

//SENSORS0
#define SDH_SENSOR_A_ACCEL             0x80
#define SDH_SENSOR_MPU9150_GYRO        0x40
#define SDH_SENSOR_LSM303DLHC_MAG      0x20
#define SDH_SENSOR_EXG1_24BIT          0x10
#define SDH_SENSOR_EXG2_24BIT          0x08
#define SDH_SENSOR_GSR                 0x04
#define SDH_SENSOR_EXTCH7              0x02
#define SDH_SENSOR_EXTCH6              0x01
//SENSORS1
#define SDH_SENSOR_STRAIN              0x80
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
#define SDH_SENSOR_BMP180_PRES         0x04
#define SDH_SENSOR_MPU9150_TEMP        0x02
//SENSORS3
#define SDH_SENSOR_MPL_QUAT_6DOF       0x80
#define SDH_SENSOR_MPL_QUAT_9DOF       0x40
#define SDH_SENSOR_MPL_EULER_6DOF      0x20
#define SDH_SENSOR_MPL_EULER_9DOF      0x10
#define SDH_SENSOR_MPL_HEADING         0x08
#define SDH_SENSOR_MPL_PEDOMETER       0x04
#define SDH_SENSOR_MPL_TAP             0x02
#define SDH_SENSOR_MPL_MOTION_ORIENT   0x01
//SENSORS4
#define SDH_SENSOR_MPU9150_GYRO_CAL    0x80
#define SDH_SENSOR_MPU9150_ACCEL_CAL   0x40
#define SDH_SENSOR_MPU9150_MAG_CAL     0x20
#define SDH_SENSOR_MPL_QUAT_6DOF_RAW   0x10

//#define SDH_SENSOR_MSP430_TEMP       0x01
#define SDH_SENSOR_TCXO                0x80

//SDH_TRIAL_CONFIG0
#define SDH_IAMMASTER                  0x02
#define SDH_TIME_SYNC                  0x04
#define SDH_TIME_STAMP                 0x08
#define SDH_GYRO_BUTTON_ENABLE         0x10
#define SDH_USER_BUTTON_ENABLE         0x20
#define SDH_SET_PMUX                   0x40
#define SDH_SET_5V_REG                 0x80
//SDH_TRIAL_CONFIG1
#define SDH_SINGLETOUCH                0x80
#define SDH_ACCEL_LPM                  0x40
#define SDH_ACCEL_HRM                  0x20
#define SDH_TCXO                       0x10

//choice of clock
#define TCXO_CLOCK      255765.625
#define MSP430_CLOCK    32768

// BT routine communication
// all node time must *2 in use
// all center time must *4 in use
#define RC_AHD          3
#define RC_WINDOW_N     13
#define RC_WINDOW_C     27
#define RC_INT_N        27
#define RC_INT_C        54//240
#define RC_CLK_N        16384   //16384=2hz;//32768=1hz;8192=4hz
#define RC_CLK_C        8192   //16384=2hz;//32768=1hz;8192=4hz
#define RC_FACTOR_N     32768/RC_CLK_N   //16384=2hz;//32768=1hz;8192=4hz
#define RC_FACTOR_C     32768/RC_CLK_C   //16384=2hz;//32768=1hz;8192=4hz


//routine communication response text - ack:flag:time4:time3:time2:time1
#define RCT_SIZE        10
#define RCT_ACK         0
#define RCT_FLG         1
#define RCT_TIME        2
#define RCT_TIME_SIZE   8

// new sync
#define SYNC_PERIOD              32768
#define SYNC_FACTOR              1//32768/SYNC_PERIOD
#define SYNC_BOOT                3
#define SYNC_CD                  2
#define SYNC_EXTEND              4// should have SYNC_BOOT>SYNC_CD
//below were params in sdlog.cfg
#define SYNC_T_EACHNODE_C        12
#define SYNC_WINDOW_C            800
#define SYNC_WINDOW_N            50
#define SYNC_NODE_REBOOT         17
#define SYNC_TRANS_IN_ONE_COMM   50//50
#define SYNC_NEXT2MATCH          2

// sd card write buffer size
#define SDBUFF_SIZE     512

// BATTERY
#define BATT_HIGH       0x01
#define BATT_MID        0x02
#define BATT_LOW        0x04
#define BATT_INTERVAL   65535    //can use 19660800 for 10min interval
#define BATT_INTERVAL_D 65535

#define MAX_NODES       20
#define MAX_CHARS       13
#define UINT32_LEN      11 // 10+1, where the last byte should be 0x00
#define UINT64_LEN      21 // 20+1, where the last byte should be 0x00
#define RESPONSE_PACKET_SIZE 256//133

//BtStream specific extension to range values : should SDLog keep it?
#define GSR_AUTORANGE   0x04


#endif
