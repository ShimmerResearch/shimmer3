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
 * @date Mar, 20143
 */
#ifndef SHIMMER_BTSD_H
#define SHIMMER_BTSD_H


//these are defined in the Makefile for BtStream (TinyOS)
#define DEVICE_VER            3      //Represents SR30. 0-3 for shimmer1 to shimmer3
#define FW_IDENTIFIER         3      //Two byte firmware identifier number:  3 for BTSD, 2 for SDLog, 1 for BTStream,
#define FW_VER_MAJOR          0      //Maor version number: 0-65535
#define FW_VER_MINOR          6      //Minor version number: 0-255
#define FW_VER_REL            2     //internal version number: 0-255

typedef uint8_t bool;
#define TRUE 1
#define FALSE 0

typedef uint8_t error_t;
#define SUCCESS 1
#define FAIL 0

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
#define CBUF_SIZE             27
#define CBUF_PARAM_LEN_MAX    CBUF_SIZE-6
// UART OLD COMMANDS
#define UART_CMD_MAC 1
#define UART_CMD_VER 2
#define UART_CMD_BAT 3
#define UART_CMD_MEM 4
#define UART_CMD_RTC 5
#define UART_CMD_RCT 6
#define UART_CMD_RDT 7
#define UART_CMD_TIM 8


// Packet Types// Packet Types
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
#define SET_DAUGHTER_CARD_ID_COMMAND                  0x64
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
#define SET_DERIVED_CHANNEL_BYTES                     0x6D
#define DERIVED_CHANNEL_BYTES_RESPONSE                0x6E
#define GET_DERIVED_CHANNEL_BYTES                     0x6F
#define START_SDBT_COMMAND                            0x70
#define STATUS_RESPONSE                               0x71
#define GET_STATUS_COMMAND                            0x72
#define SET_TRIAL_CONFIG_COMMAND                      0x73
#define TRIAL_CONFIG_RESPONSE                         0x74
#define GET_TRIAL_CONFIG_COMMAND                      0x75
#define SET_CENTER_COMMAND                            0x76
#define CENTER_RESPONSE                               0x77
#define GET_CENTER_COMMAND                            0x78
#define SET_SHIMMERNAME_COMMAND                       0x79
#define SHIMMERNAME_RESPONSE                          0x7a
#define GET_SHIMMERNAME_COMMAND                       0x7b
#define SET_EXPID_COMMAND                             0x7c
#define EXPID_RESPONSE                                0x7d
#define GET_EXPID_COMMAND                             0x7e
#define SET_MYID_COMMAND                              0x7F
#define MYID_RESPONSE                                 0x80
#define GET_MYID_COMMAND                              0x81
#define SET_NSHIMMER_COMMAND                          0x82
#define NSHIMMER_RESPONSE                             0x83
#define GET_NSHIMMER_COMMAND                          0x84
#define SET_CONFIGTIME_COMMAND                        0x85
#define CONFIGTIME_RESPONSE                           0x86
#define GET_CONFIGTIME_COMMAND                        0x87
#define DIR_RESPONSE                                  0x88
#define GET_DIR_COMMAND                               0x89
#define INSTREAM_CMD_RESPONSE                         0x8A
#define SET_CRC_COMMAND                               0x8B
#define SET_INFOMEM_COMMAND                           0x8C
#define INFOMEM_RESPONSE                              0x8D
#define GET_INFOMEM_COMMAND                           0x8E
#define SET_RWC_COMMAND                               0x8F
#define RWC_RESPONSE                                  0x90
#define GET_RWC_COMMAND                               0x91
#define START_LOGGING_COMMAND                         0x92
#define STOP_LOGGING_COMMAND                          0x93
#define VBATT_RESPONSE                                0x94
#define GET_VBATT_COMMAND                             0x95
#define DUMMY_COMMAND                                 0x96
#define STOP_SDBT_COMMAND                             0x97
//#define ROUTINE_COMMUNICATION                         0xE0
#define ACK_COMMAND_PROCESSED                         0xFF




//SENSORS0
#define SENSOR_A_ACCEL                 0x80
#define SENSOR_MPU9150_GYRO            0x40
#define SENSOR_LSM303DLHC_MAG          0x20
#define SENSOR_EXG1_24BIT              0x10
#define SENSOR_EXG2_24BIT              0x08
#define SENSOR_GSR                     0x04
#define SENSOR_EXT_A7                  0x02
#define SENSOR_EXT_A6                  0x01
//SENSORS1
#define SENSOR_STRAIN                  0x80
//#define SDH_SENSOR_HR                0x40
#define SENSOR_VBATT                   0x20
#define SENSOR_LSM303DLHC_ACCEL        0x10
#define SENSOR_EXT_A15                 0x08
#define SENSOR_INT_A1                  0x04
#define SENSOR_INT_A12                 0x02
#define SENSOR_INT_A13                 0x01
//SENORS2
#define SENSOR_INT_A14                 0x80
#define SENSOR_MPU9150_ACCEL           0x40
#define SENSOR_MPU9150_MAG             0x20
#define SENSOR_EXG1_16BIT              0x10
#define SENSOR_EXG2_16BIT              0x08
#define SENSOR_BMP180_PRESSURE         0x04


#define MAX_COMMAND_ARG_SIZE     131   //maximum number of arguments for any command sent
                                       //(daughter card mem write)
//#define RESPONSE_PACKET_SIZE     131   //biggest possibly required  (daughter card mem read + 1 byte for ack)
#define MAX_NUM_CHANNELS         28    //3xanalogAccel + 3xdigiGyro + 3xdigiMag +
                                       //3xLSM303DLHCAccel + 3xMPU9150Accel + 3xMPU9150MAG +
                                       //BMP180TEMP + BMP180PRESS + batteryVoltage +
                                       //3xexternalADC + 4xinternalADC
#define DATA_PACKET_SIZE         84    //3 + (MAX_NUM_CHANNELS * 2) + 1 + 6 (+1 as BMP180
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
#define NV_LSM303DLHC_ACCEL_CALIBRATION   97  //97->117
#define NV_CALIBRATION_END               117

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

#define NV_NODE0                          128+128+0


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

#define SDH_SAMPLE_RATE_0           0
#define SDH_SAMPLE_RATE_1           1
#define SDH_BUFFER_SIZE             2
#define SDH_SENSORS0                3
#define SDH_SENSORS1                4
#define SDH_SENSORS2                5
#define SDH_SENSORS3                6
#define SDH_SENSORS4                7
#define SDH_CONFIG_SETUP_BYTE0      8 //sensors setting bytes
#define SDH_CONFIG_SETUP_BYTE1      9
#define SDH_CONFIG_SETUP_BYTE2      10
#define SDH_CONFIG_SETUP_BYTE3      11
#define SDH_CONFIG_SETUP_BYTE4      12
#define SDH_CONFIG_SETUP_BYTE5      13
#define SDH_CONFIG_SETUP_BYTE6      14
#define SDH_TRIAL_CONFIG0           16
#define SDH_TRIAL_CONFIG1           17
#define SDH_BROADCAST_INTERVAL      18
#define SDH_BT_COMMS_BAUD_RATE      19
#define SDH_EST_EXP_LEN_MSB         20
#define SDH_EST_EXP_LEN_LSB         21
#define SDH_MAX_EXP_LEN_MSB         22
#define SDH_MAX_EXP_LEN_LSB         23
#define SDH_MAC_ADDR                24
#define SDH_SHIMMERVERSION_BYTE_0   30
#define SDH_SHIMMERVERSION_BYTE_1   31
#define SDH_MYTRIAL_ID              32
#define SDH_NSHIMMER                33
#define SDH_FW_VERSION_TYPE_0       34
#define SDH_FW_VERSION_TYPE_1       35
#define SDH_FW_VERSION_MAJOR_0      36
#define SDH_FW_VERSION_MAJOR_1      37
#define SDH_FW_VERSION_MINOR        38
#define SDH_FW_VERSION_INTERNAL     39
#define SDH_DERIVED_CHANNELS_0      40
#define SDH_DERIVED_CHANNELS_1      41
#define SDH_DERIVED_CHANNELS_2      42
#define SDH_RTC_DIFF_0              44
#define SDH_RTC_DIFF_1              45
#define SDH_RTC_DIFF_2              46
#define SDH_RTC_DIFF_3              47
#define SDH_RTC_DIFF_4              48
#define SDH_RTC_DIFF_5              49
#define SDH_RTC_DIFF_6              50
#define SDH_RTC_DIFF_7              51
#define SDH_CONFIG_TIME_0           52
#define SDH_CONFIG_TIME_1           53
#define SDH_CONFIG_TIME_2           54
#define SDH_CONFIG_TIME_3           55
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
#define SDH_MY_LOCALTIME_5TH              251
#define SDH_MY_LOCALTIME                  252   //252-255

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
#define SDH_SENSOR_BMP180_PRES         0x04
//#define SDH_SENSOR_BMP180_TEMP       0x02
//SENSORS3
#define SDH_SENSOR_MSP430_TEMP         0x01
#define SDH_SENSOR_TCXO                0x80

//SDH_TRIAL_CONFIG0
#define SDH_IAMMASTER                  0x02
#define SDH_TIME_SYNC                  0x04
#define SDH_TIME_STAMP                 0x08// not used now, reserved as 1
#define SDH_RWCERROR_EN                0x10// when 0, won't flash error. when 1, will flash error if RTC isn't set (RTC_offset == 0)
//#define SDH_GYRO_BUTTON_ENABLE         0x10
#define SDH_USER_BUTTON_ENABLE         0x20
#define SDH_SET_PMUX                   0x40// not used now, reserved as 0
#define SDH_RTC_SET_BY_BT              0x80
//SDH_TRIAL_CONFIG1
#define SDH_SINGLETOUCH                0x80
//#define SDH_ACCEL_LPM                  0x40//config has this bit
//#define SDH_ACCEL_HRM                  0x20//config has this bit
#define SDH_TCXO                       0x10
//#define SDH_EXP_POWER                  0x08//config has this bit
//#define SDH_MONITOR                    0x04

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
#define RCT_SIZE        6
#define RCT_ACK         0
#define RCT_FLG         1
#define RCT_TIME        2
#define RCT_TIME_SIZE   4

// sd card write buffer size
#define SDBUFF_SIZE_MAX 4096 //4095  255
#define SDBUFF_SIZE     512 //4095  512

// BATTERY
#define BATT_HIGH       0
#define BATT_MID        1
#define BATT_LOW        2
#define BATT_INTERVAL   1966080
#define BATT_INTERVAL_D 65536

#define MAX_CHARS       13
#define UINT32_LEN      11 // 10+1, where the last byte should be 0x00
#define UINT64_LEN      21 // 20+1, where the last byte should be 0x00
#define RESPONSE_PACKET_SIZE 133

//BMP Pressure oversampling ratio
#define BMP180_OSS_1                0x00
#define BMP180_OSS_2                0x01
#define BMP180_OSS_4                0x02
#define BMP180_OSS_8                0x03

//BtStream specific extension to range values : should SDLog keep it?
#define GSR_AUTORANGE               0x04

#endif
