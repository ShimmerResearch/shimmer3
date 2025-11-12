/*
 * shimmer_bt_comms.h
 *
 *  Created on: 22 Jun 2022
 *      Author: Mark Nolan
 */

#ifndef SHIMMER3_COMMON_SOURCE_BLUETOOTH_SD_SHIMMER_BT_COMMS_H_
#define SHIMMER3_COMMON_SOURCE_BLUETOOTH_SD_SHIMMER_BT_COMMS_H_

#include <stdint.h>
#include "../../shimmer_btsd.h"
#include "../5xx_HAL/hal_CRC.h"

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
#define SET_BMPX80_PRES_OVERSAMPLING_RATIO_COMMAND    0x52
#define BMPX80_PRES_OVERSAMPLING_RATIO_RESPONSE       0x53
#define GET_BMPX80_PRES_OVERSAMPLING_RATIO_COMMAND    0x54
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
#define SET_CALIB_DUMP_COMMAND                        0x98
#define RSP_CALIB_DUMP_COMMAND                        0x99
#define GET_CALIB_DUMP_COMMAND                        0x9A
#define UPD_CALIB_DUMP_COMMAND                        0x9B
#define UPD_SDLOG_CFG_COMMAND                         0x9C
#define BMP280_CALIBRATION_COEFFICIENTS_RESPONSE      0x9F
#define GET_BMP280_CALIBRATION_COEFFICIENTS_COMMAND   0xA0
#define GET_BT_VERSION_STR_COMMAND                    0xA1
#define BT_VERSION_STR_RESPONSE                       0xA2
#define SET_INSTREAM_RESPONSE_ACK_PREFIX_STATE        0xA3
#define PRESSURE_CALIBRATION_COEFFICIENTS_RESPONSE    0xA6
#define GET_PRESSURE_CALIBRATION_COEFFICIENTS_COMMAND 0xA7

#define SET_FACTORY_TEST                              0xA8
#define RESET_BT_ERROR_COUNTS                         0xB6

#if !USE_OLD_SD_SYNC_APPROACH
#define SET_SD_SYNC_COMMAND                           0xE0
#define SD_SYNC_RESPONSE                              0xE1
#endif
#define ACK_COMMAND_PROCESSED                         0xFF

//#define BT_RX_COMMS_TIMEOUT_TICKS                     3277U /* 32768*0.1s = 3276.8  */
#define BT_RX_COMMS_TIMEOUT_TICKS                     328U /* 32768*0.01s = 327.68  */

enum
{
  PRESSURE_SENSOR_BMP180 = 0,
  PRESSURE_SENSOR_BMP280 = 1,
  PRESSURE_SENSOR_BMP390 = 2
};

#if BT_DMA_USED_FOR_RX
uint8_t Dma2ConversionDone(void);
void resetBtRxVariablesOnConnect(void);
void resetBtRxBuff(void);
#else
void processBtUartBuf(void);
void handleBtRxTimeout(void);
uint8_t hasBtRxTimeoutOccurred(void);
void processStartRnCmdResponse(void);
uint8_t wasStartBtCmdModeSentAndReponseReceived(void);
//uint8_t waitingForRnBtCmdResponse(void);
//uint8_t isExpectedRnBtCmdResponseInFifo(void);
void clearBytesFromBtRxBuf(uint16_t numBytes);
uint8_t getRxByteAtIndex(uint16_t index);
uint8_t processStatusString(void);
uint8_t isBtRxBufLike(char statStrCheck[], uint8_t numCharTolerance);
uint8_t processRnCmdResponse(void);
uint8_t isFullRN4678CmdResponseReceived(void);
void clearBtCmdTxRxBuffsAndProceed(void);
uint8_t processShimmerBtCmd(void);
void readActionAndArgBytes(uint8_t numArgs);
uint8_t isNewLineDetectedInBtRxBuf(void);
uint8_t isShimmerBtCmd(uint8_t data);
void updateNumBytesInBtRxBufWhenLastProcessed(void);
uint16_t getNumBytesInBtRxBufWhenLastProcessed(void);
uint8_t areUnprocessedBytesInBtRxBuff(void);
#endif

void btCommsProtocolInit(uint8_t (*newBtCmdToProcessCb)(void),
                         void (*handleBtRfCommStateChangeCb)(uint8_t),
                         void (*setMacIdCb)(uint8_t *),
                         uint8_t * actionPtr,
                         uint8_t * argsPtr);
void triggerBtRfCommStateChangeCallback(bool state);
void triggerShimmerErrorState(void);
uint8_t getBtVerStrLen(void);
char * getBtVerStrPtr(void);

void setBtCrcMode(COMMS_CRC_MODE btCrcModeNew);
COMMS_CRC_MODE getBtCrcMode(void);

#endif /* SHIMMER3_COMMON_SOURCE_BLUETOOTH_SD_SHIMMER_BT_COMMS_H_ */
