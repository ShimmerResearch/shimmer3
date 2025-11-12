/*
 * shimmer_boards.h
 *
 *  Created on: 2 May 2023
 *      Author: MarkNolan
 */

#ifndef LOG_AND_STREAM_COMMON_BOARDS_SHIMMER_BOARDS_H_
#define LOG_AND_STREAM_COMMON_BOARDS_SHIMMER_BOARDS_H_

#include <stdint.h>

#include "../EEPROM/shimmer_eeprom.h"

//#include "CAT24C16/CAT24C16.h"

#ifndef __weak
#define __weak __attribute__((weak))
#endif /* __weak */

enum SR_HW_IDS
{
  HW_ID_SHIMMER3 = 3,
  HW_ID_SHIMMER3R = 10
};

enum SR_BOARD_CODES
{
  EXP_BRD_BR_AMP = 8,
  EXP_BRD_GSR = 14,
  SHIMMER3_IMU = 31,
  EXP_BRD_PROTO3_MINI = 36,
  EXP_BRD_EXG = 37,
  EXP_BRD_PROTO3_DELUXE = 38,
  EXP_BRD_ADXL377_ACCEL_200G = 44,
  EXP_BRD_EXG_UNIFIED = 47,
  EXP_BRD_GSR_UNIFIED = 48,
  EXP_BRD_BR_AMP_UNIFIED = 49,
  EXP_BRD_H3LIS331DL_ACCEL_HIGH_G = 55,
  SHIMMER_GQ_LR = 56,
  SHIMMER_GQ_SR = 57,
  SHIMMER4_SDK = 58,
  SHIMMER_ECG_MD = 59
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

void ShimBrd_init(void);
void ShimBrd_resetDaughterCardId(void);
void ShimBrd_setHwId(uint8_t hwIdToSet);
void ShimBrd_setDaugherCardIdPage(uint8_t *pagePtr);
void ShimBrd_setDaugherCardIdMemory(uint8_t index, uint8_t *pagePtr, uint8_t len);
uint8_t ShimBrd_isAds1292Present(void);
#if defined(SHIMMER3)
uint8_t ShimBrd_isRn4678PresentAndCmdModeSupport(void);
uint8_t ShimBrd_isSubstitutionNeededForWrAccel(void);
uint8_t ShimBrd_are2ndGenImuSensorsPresent(void);
uint8_t ShimBrd_are2ndGenSensorsPresentAndUnknownBoard(void);
uint8_t ShimBrd_areGsrControlsPinsReversed(void);
#endif
uint8_t ShimBrd_areADS1292RClockLinesTied(void);
void ShimBrd_parseDaughterCardId(void);
shimmer_expansion_brd *ShimBrd_getDaughtCardId(void);
uint8_t *ShimBrd_getDaughtCardIdPtr(void);
char *ShimBrd_getDaughtCardIdStrPtr(void);
uint8_t ShimBrd_isDaughterCardIdSet(void);

#if defined(SHIMMER3)
void ShimBrd_setWrAccelAndMagInUse(uint8_t wr_accel_and_mag_in_use);
uint8_t ShimBrd_isWrAccelInUseLsm303dlhc(void);
uint8_t ShimBrd_isWrAccelInUseLsm303ahtr(void);
uint8_t ShimBrd_isWrAccelInUseIcm20948(void);

void ShimBrd_setGyroInUse(uint8_t gyro_in_use);
uint8_t ShimBrd_isGyroInUseMpu9x50(void);
uint8_t ShimBrd_isGyroInUseIcm20948(void);
#endif

#if defined(SHIMMER3)
uint8_t ShimBrd_isLnAccelKxtc9_2050Present(void);
#endif

uint8_t ShimBrd_isAdxl371Present(void);
uint8_t ShimBrd_isLis3mdlPresent(void);
uint8_t isAds7028Present(void);
uint8_t ShimBrd_isI2c4Supported(void);
uint8_t ShimBrd_isBoardSr48_6_0(void);
uint8_t ShimBrd_isI2cOnPPGControlledByAdcChip(void);
uint8_t ShimBrd_areMcuAdcsUsedForSensing(void);

uint8_t ShimBrd_isBoardSrNumber(uint8_t exp_brd_id, uint8_t exp_brd_major, uint8_t exp_brd_minor);
uint8_t ShimBrd_isHwId(uint8_t hwIdToCheck);
uint8_t ShimBrd_isExpBrdId(uint8_t expIdToCheck);
uint8_t ShimBrd_checkCorrectStateForBoot0(void);

__weak uint8_t ShimBrd_doesDeviceSupportBle(void);
__weak uint8_t ShimBrd_doesDeviceSupportBtClassic(void);

#endif /* LOG_AND_STREAM_COMMON_BOARDS_SHIMMER_BOARDS_H_ */
