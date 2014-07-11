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


#ifndef SHIMMER_H
#define SHIMMER_H

//these are defined in the Makefile for BtStream (TinyOS)
#define DEVICE_VER      3     //Represents shimmer3
#define FW_IDENTIFIER   1     //Two byte firmware identifier number
#define FW_VER_MAJOR    0     //Major version number: 0-65535
#define FW_VER_MINOR    4     //Minor version number: 0-255
#define FW_VER_REL      0     //Release candidate version number: 0-255


// Packet Types
#define DATA_PACKET                                   0x00
#define INQUIRY_COMMAND                               0x01
#define INQUIRY_RESPONSE                              0x02
#define GET_SAMPLING_RATE_COMMAND                     0x03
#define SAMPLING_RATE_RESPONSE                        0x04
#define SET_SAMPLING_RATE_COMMAND                     0x05
#define TOGGLE_LED_COMMAND                            0x06
#define START_STREAMING_COMMAND                       0x07  //maintain compatibility with Shimmer2/2r BtStream
#define SET_SENSORS_COMMAND                           0x08
#define SET_LSM303DLHC_ACCEL_RANGE_COMMAND            0x09
#define LSM303DLHC_ACCEL_RANGE_RESPONSE               0x0A
#define GET_LSM303DLHC_ACCEL_RANGE_COMMAND            0x0B
#define SET_CONFIG_SETUP_BYTES_COMMAND                0x0E
#define CONFIG_SETUP_BYTES_RESPONSE                   0x0F
#define GET_CONFIG_SETUP_BYTES_COMMAND                0x10
#define SET_A_ACCEL_CALIBRATION_COMMAND               0x11
#define A_ACCEL_CALIBRATION_RESPONSE                  0x12
#define GET_A_ACCEL_CALIBRATION_COMMAND               0x13
#define SET_MPU9150_GYRO_CALIBRATION_COMMAND          0x14
#define MPU9150_GYRO_CALIBRATION_RESPONSE             0x15
#define GET_MPU9150_GYRO_CALIBRATION_COMMAND          0x16
#define SET_LSM303DLHC_MAG_CALIBRATION_COMMAND        0x17
#define LSM303DLHC_MAG_CALIBRATION_RESPONSE           0x18
#define GET_LSM303DLHC_MAG_CALIBRATION_COMMAND        0x19
#define SET_LSM303DLHC_ACCEL_CALIBRATION_COMMAND      0x1A
#define LSM303DLHC_ACCEL_CALIBRATION_RESPONSE         0x1B
#define GET_LSM303DLHC_ACCEL_CALIBRATION_COMMAND      0x1C
#define STOP_STREAMING_COMMAND                        0x20  //maintain compatibility with Shimmer2/2r BtStream
#define SET_GSR_RANGE_COMMAND                         0x21
#define GSR_RANGE_RESPONSE                            0x22
#define GET_GSR_RANGE_COMMAND                         0x23
#define DEPRECATED_GET_DEVICE_VERSION_COMMAND         0x24  //maintain compatibility with Shimmer2/2r BtStream
                                                            //deprecated because 0x24 ('$' ASCII) as a command
                                                            //is problematic if remote config is enabled in
                                                            //RN42 Bluetooth module. Replaced with 0x3F command
#define DEVICE_VERSION_RESPONSE                       0x25  //maintain compatibility with Shimmer2/2r BtStream
#define GET_ALL_CALIBRATION_COMMAND                   0x2C
#define ALL_CALIBRATION_RESPONSE                      0x2D
#define GET_FW_VERSION_COMMAND                        0x2E  //maintain compatibility with Shimmer2/2r BtStream
#define FW_VERSION_RESPONSE                           0x2F  //maintain compatibility with Shimmer2/2r BtStream
#define SET_CHARGE_STATUS_LED_COMMAND                 0x30
#define CHARGE_STATUS_LED_RESPONSE                    0x31
#define GET_CHARGE_STATUS_LED_COMMAND                 0x32
#define BUFFER_SIZE_RESPONSE                          0x35
#define GET_BUFFER_SIZE_COMMAND                       0x36
#define SET_LSM303DLHC_MAG_GAIN_COMMAND               0x37
#define LSM303DLHC_MAG_GAIN_RESPONSE                  0x38
#define GET_LSM303DLHC_MAG_GAIN_COMMAND               0x39
#define SET_LSM303DLHC_MAG_SAMPLING_RATE_COMMAND      0x3A
#define LSM303DLHC_MAG_SAMPLING_RATE_RESPONSE         0x3B
#define GET_LSM303DLHC_MAG_SAMPLING_RATE_COMMAND      0x3C
#define UNIQUE_SERIAL_RESPONSE                        0x3D
#define GET_UNIQUE_SERIAL_COMMAND                     0x3E
#define GET_DEVICE_VERSION_COMMAND                    0x3F
#define SET_LSM303DLHC_ACCEL_SAMPLING_RATE_COMMAND    0x40
#define LSM303DLHC_ACCEL_SAMPLING_RATE_RESPONSE       0x41
#define GET_LSM303DLHC_ACCEL_SAMPLING_RATE_COMMAND    0x42
#define SET_LSM303DLHC_ACCEL_LPMODE_COMMAND           0x43
#define LSM303DLHC_ACCEL_LPMODE_RESPONSE              0x44
#define GET_LSM303DLHC_ACCEL_LPMODE_COMMAND           0x45
#define SET_LSM303DLHC_ACCEL_HRMODE_COMMAND           0x46
#define LSM303DLHC_ACCEL_HRMODE_RESPONSE              0x47
#define GET_LSM303DLHC_ACCEL_HRMODE_COMMAND           0x48
#define SET_MPU9150_GYRO_RANGE_COMMAND                0x49
#define MPU9150_GYRO_RANGE_RESPONSE                   0x4A
#define GET_MPU9150_GYRO_RANGE_COMMAND                0x4B
#define SET_MPU9150_SAMPLING_RATE_COMMAND             0x4C
#define MPU9150_SAMPLING_RATE_RESPONSE                0x4D
#define GET_MPU9150_SAMPLING_RATE_COMMAND             0x4E
#define SET_MPU9150_ACCEL_RANGE_COMMAND               0x4F
#define MPU9150_ACCEL_RANGE_RESPONSE                  0x50
#define GET_MPU9150_ACCEL_RANGE_COMMAND               0x51
#define SET_BMP180_PRES_OVERSAMPLING_RATIO_COMMAND    0x52
#define BMP180_PRES_OVERSAMPLING_RATIO_RESPONSE       0x53
#define GET_BMP180_PRES_OVERSAMPLING_RATIO_COMMAND    0x54
#define BMP180_CALIBRATION_COEFFICIENTS_RESPONSE      0x58
#define GET_BMP180_CALIBRATION_COEFFICIENTS_COMMAND   0x59
#define RESET_TO_DEFAULT_CONFIGURATION_COMMAND        0x5A
#define RESET_CALIBRATION_VALUE_COMMAND               0x5B
#define MPU9150_MAG_SENS_ADJ_VALS_RESPONSE            0x5C
#define GET_MPU9150_MAG_SENS_ADJ_VALS_COMMAND         0x5D
#define SET_INTERNAL_EXP_POWER_ENABLE_COMMAND         0x5E
#define INTERNAL_EXP_POWER_ENABLE_RESPONSE            0x5F
#define GET_INTERNAL_EXP_POWER_ENABLE_COMMAND         0x60
#define SET_EXG_REGS_COMMAND                          0x61
#define EXG_REGS_RESPONSE                             0x62
#define GET_EXG_REGS_COMMAND                          0x63
#define DAUGHTER_CARD_ID_RESPONSE                     0x65
#define GET_DAUGHTER_CARD_ID_COMMAND                  0x66
#define SET_DAUGHTER_CARD_MEM_COMMAND                 0x67
#define DAUGHTER_CARD_MEM_RESPONSE                    0x68
#define GET_DAUGHTER_CARD_MEM_COMMAND                 0x69
#define SET_BT_COMMS_BAUD_RATE                        0x6A     //11 allowable options: 0=115.2K(default), 1=1200, 2=2400, 3=4800,
                                                               //4=9600, 5=19.2K, 6=38.4K, 7=57.6K, 8=230.4K, 9=460.8K, 10=921.6K
                                                               //Need to disconnect BT connection before change is active
#define BT_COMMS_BAUD_RATE_RESPONSE                   0x6B
#define GET_BT_COMMS_BAUD_RATE                        0x6C
//0x70 to 0x87 and 0xE0 reserved for Log+Stream
#define ACK_COMMAND_PROCESSED                         0xFF


//SENSORS0
#define SENSOR_A_ACCEL           0x80
#define SENSOR_MPU9150_GYRO      0x40
#define SENSOR_LSM303DLHC_MAG    0x20
#define SENSOR_EXG1_24BIT        0x10
#define SENSOR_EXG2_24BIT        0x08
#define SENSOR_GSR               0x04
#define SENSOR_EXT_A7            0x02
#define SENSOR_EXT_A6            0x01
//SENSORS1
#define SENSOR_BRIDGE_AMP        0x80     //higher priority than SENSOR_INT_A13 and SENSOR_INT_A14
#define SENSOR_VBATT             0x20
#define SENSOR_LSM303DLHC_ACCEL  0x10
#define SENSOR_EXT_A15           0x08
#define SENSOR_INT_A1            0x04
#define SENSOR_INT_A12           0x02
#define SENSOR_INT_A13           0x01
//SENORS2
#define SENSOR_INT_A14           0x80
#define SENSOR_MPU9150_ACCEL     0x40
#define SENSOR_MPU9150_MAG       0x20
#define SENSOR_EXG1_16BIT        0x10
#define SENSOR_EXG2_16BIT        0x08
#define SENSOR_BMP180_PRESSURE   0x04


#define MAX_COMMAND_ARG_SIZE     131   //maximum number of arguments for any command sent
                                       //(daughter card mem write)
#define RESPONSE_PACKET_SIZE     131   //biggest possibly required  (daughter card mem read + 1 byte for ack)
#define MAX_NUM_CHANNELS         28    //3xanalogAccel + 3xdigiGyro + 3xdigiMag + 
                                       //3xLSM303DLHCAccel + 3xMPU9150Accel + 3xMPU9150MAG +
                                       //BMP180TEMP + BMP180PRESS + batteryVoltage + 
                                       //3xexternalADC + 4xinternalADC
#define DATA_PACKET_SIZE         66    //3 + (MAX_NUM_CHANNELS * 2) + 1 + 6 (+1 as BMP180 
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
#define BMP180_TEMP                       0x1A
#define BMP180_PRESSURE                   0x1B
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
#define BRIDGE_AMP_HIGH                   0x27
#define BRIDGE_AMP_LOW                    0x28


// Infomem contents
#define NV_NUM_SETTINGS_BYTES             31
#define NV_NUM_CALIBRATION_BYTES          84
#define NV_TOTAL_NUM_CONFIG_BYTES         115

#define NV_SAMPLING_RATE                  0
#define NV_BUFFER_SIZE                    2
#define NV_SENSORS0                       3
#define NV_SENSORS1                       4
#define NV_SENSORS2                       5
#define NV_CONFIG_SETUP_BYTE0             6
#define NV_CONFIG_SETUP_BYTE1             7
#define NV_CONFIG_SETUP_BYTE2             8
#define NV_CONFIG_SETUP_BYTE3             9
#define NV_EXG_ADS1292R_1_CONFIG1         10
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
#define NV_A_ACCEL_CALIBRATION            31
#define NV_MPU9150_GYRO_CALIBRATION       52
#define NV_LSM303DLHC_MAG_CALIBRATION     73
#define NV_LSM303DLHC_ACCEL_CALIBRATION   94


//Config byte masks
//Config Byte0
#define LSM303DLHC_ACCEL_SAMPLING_RATE          0xF0
#define LSM303DLHC_ACCEL_RANGE                  0x0C
#define LSM303DLHC_ACCEL_LOW_POWER_MODE         0x02
#define LSM303DLHC_ACCEL_HIGH_RESOLUTION_MODE   0x01
//Config Byte1
//MPU9150_SAMPLING_RATE                         0xFF
//Config Byte2
#define LSM303DLHC_MAG_GAIN                     0xE0
#define LSM303DLHC_MAG_SAMPLING_RATE            0x1C
#define MPU9150_GYRO_RANGE                      0x03
//Config Byte3
#define MPU9150_ACCEL_RANGE                     0xC0
#define BMP180_PRESSURE_RESOLUTION              0x30
#define GSR_RANGE                               0x0E
#define INT_EXP_POWER_ENABLE                    0x01


//ADC initialisation mask
#define MASK_A_ACCEL 	0x0001
#define MASK_VBATT   	0x0002
#define MASK_EXT_A7  	0x0004
#define MASK_EXT_A6  	0x0008
#define MASK_EXT_A15 	0x0010
#define MASK_INT_A1  	0x0020
#define MASK_INT_A12 	0x0040
#define MASK_INT_A13 	0x0080
#define MASK_INT_A14 	0x0100
#define MASK_GSR     	0x0020   //uses ADC1
#define MASK_BRIDGE_AMP	0x0180   //uses ADC13 and ADC14


//LSM303DLHC Accel Range
//Corresponds to the FS field of the LSM303DLHC's CTRL_REG4_A register
//and the AFS_SEL field of the MPU9150's ACCEL_CONFIG register
#define ACCEL_2G     0x00
#define ACCEL_4G     0x01
#define ACCEL_8G     0x02
#define ACCEL_16G    0x03

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

//MPU9150 Gyro range
#define MPU9150_GYRO_250DPS         0x00 //+/-250 dps
#define MPU9150_GYRO_500DPS         0x01 //+/-500 dps
#define MPU9150_GYRO_1000DPS        0x02 //+/-1000 dps
#define MPU9150_GYRO_2000DPS        0x03 //+/-2000 dps

//BMP Pressure oversampling ratio
#define BMP180_OSS_1                0x00
#define BMP180_OSS_2                0x01
#define BMP180_OSS_4                0x02
#define BMP180_OSS_8                0x03

//BtStream specific extension to range values
#define GSR_AUTORANGE               0x04

#endif
