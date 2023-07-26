/*
 * shimmer_boards.h
 *
 *  Created on: 2 May 2023
 *      Author: MarkNolan
 */

#ifndef SHIMMER3_COMMON_SOURCE_SHIMMER_BOARDS_SHIMMER_BOARDS_H_
#define SHIMMER3_COMMON_SOURCE_SHIMMER_BOARDS_SHIMMER_BOARDS_H_

#include <stdint.h>

enum SR_BOARD_CODES
{
    EXP_BRD_BR_AMP              = 8,
    EXP_BRD_GSR                 = 14,
    SHIMMER3_IMU                = 31,
    EXP_BRD_PROTO3_MINI         = 36,
    EXP_BRD_EXG                 = 37,
    EXP_BRD_PROTO3_DELUXE       = 38,
    EXP_BRD_ADXL377_ACCEL_200G  = 44,
    EXP_BRD_EXG_UNIFIED         = 47,
    EXP_BRD_GSR_UNIFIED         = 48,
    EXP_BRD_BR_AMP_UNIFIED      = 49,
    EXP_BRD_H3LIS331DL_ACCEL_HIGH_G     = 55,
    SHIMMER_GQ_LR               = 56,
    SHIMMER_GQ_SR               = 57,
    SHIMMER_ECG_MD              = 59
};

enum WR_ACCEL_AND_MAG_IN_USE
{
    WR_ACCEL_AND_MAG_NONE_IN_USE,
    WR_ACCEL_AND_MAG_LSM303DLHC_IN_USE,
    WR_ACCEL_AND_MAG_LSM303AHTR_IN_USE,
    WR_ACCEL_AND_MAG_ICM20948_IN_USE
};

enum GYRO_IN_USE
{
    GYRO_NONE_IN_USE,
    GYRO_MPU9X50_IN_USE,
    GYRO_ICM20948_IN_USE
};

uint8_t isAds1292Present(uint8_t srId);
uint8_t isRn4678PresentAndCmdModeSupport(uint8_t srId, uint8_t srRev, uint8_t srRevSpecial);
uint8_t isSubstitutionNeededForWrAccel(uint8_t srId, uint8_t srRevMajor, uint8_t srRevMinor);
uint8_t are2ndGenImuSensorsPresent(void);
uint8_t are2ndGenSensorsPresentAndUnknownBoard(uint8_t srId);
uint8_t areGsrControlsPinsReversed(uint8_t srId, uint8_t srRevMajor, uint8_t srRevMinor);
void parseDaughterCardId(uint8_t srId);
char* getDaughtCardIdStrPtr(void);

void setWrAccelAndMagInUse(uint8_t wr_accel_and_mag_in_use);
uint8_t isWrAccelInUseLsm303dlhc(void);
uint8_t isWrAccelInUseLsm303ahtr(void);
uint8_t isWrAccelInUseIcm20948(void);

void setGyroInUse(uint8_t gyro_in_use);
uint8_t isGyroInUseMpu9x50(void);
uint8_t isGyroInUseIcm20948(void);

void setEepromIsPresent(uint8_t eeprom_is_preset);
uint8_t isEepromIsPresent(void);

uint8_t isLnAccelKxtc9_2050Present(void);

#endif /* SHIMMER3_COMMON_SOURCE_SHIMMER_BOARDS_SHIMMER_BOARDS_H_ */
