/*
 * shimmer_boards.c
 *
 *  Created on: 2 May 2023
 *      Author: MarkNolan
 */

#include "shimmer_boards.h"

#include <stdio.h>
#include <string.h>

#include "../EEPROM/shimmer_eeprom.h"

#if defined(SHIMMER3)
#include "../BMPX80/bmpX80.h"
#endif

uint8_t hwId;
daughter_card_id_page daughterCardIdPage;
char daughtCardIdStr[26];
#if defined(SHIMMER3)
uint8_t wrAccelAndMagInUse, gyroInUse;
#endif

void ShimBrd_init(void)
{
  ShimBrd_resetDaughterCardId();
  memset(daughtCardIdStr, 0, sizeof(daughtCardIdStr));

  ShimBrd_setHwId(0);
#if defined(SHIMMER3)
  ShimBrd_setWrAccelAndMagInUse(WR_ACCEL_AND_MAG_NONE_IN_USE);
  ShimBrd_setGyroInUse(GYRO_NONE_IN_USE);
#endif
}

void ShimBrd_resetDaughterCardId(void)
{
  memset(daughterCardIdPage.raw, 0, sizeof(daughterCardIdPage.raw));
}

void ShimBrd_setHwId(uint8_t hwIdToSet)
{
  hwId = hwIdToSet;
}

void ShimBrd_setDaugherCardIdPage(uint8_t *pagePtr)
{
  ShimBrd_setDaugherCardIdMemory(0, pagePtr, sizeof(daughterCardIdPage.raw));
}

void ShimBrd_setDaugherCardIdMemory(uint8_t index, uint8_t *pagePtr, uint8_t len)
{
  memcpy(&daughterCardIdPage.raw[index], pagePtr, len);
}

uint8_t ShimBrd_isAds1292Present(void)
{
  uint8_t srId = daughterCardIdPage.expansion_brd.exp_brd_id;
  return (srId == EXP_BRD_EXG) || (srId == EXP_BRD_EXG_UNIFIED)
      || (srId == SHIMMER_ECG_MD) || (srId == SHIMMER4_SDK);
}

#if defined(SHIMMER3)
uint8_t ShimBrd_isRn4678PresentAndCmdModeSupport(void)
{
  uint8_t srId = daughterCardIdPage.expansion_brd.exp_brd_id;
  uint8_t srRevMajor = daughterCardIdPage.expansion_brd.exp_brd_major;
  uint8_t srRevMinor = daughterCardIdPage.expansion_brd.exp_brd_minor;

  /* Checking EEPROM here to rule out older sensors in factory test which
   * don't have EEPROM fitted */
  return (ShimEeprom_isPresent() && hwId == HW_ID_SHIMMER3
      && ((srId == SHIMMER3_IMU && srRevMajor >= 10)
          || (srId == EXP_BRD_EXG_UNIFIED && srRevMajor >= 6)
          || (srId == EXP_BRD_GSR_UNIFIED && srRevMajor >= 5)
          || (srId == EXP_BRD_BR_AMP_UNIFIED && srRevMajor >= 3)
          || (srId == EXP_BRD_PROTO3_DELUXE && srRevMajor >= 4)
          || (srId == EXP_BRD_PROTO3_MINI && srRevMajor >= 4)
          || (srId == EXP_BRD_ADXL377_ACCEL_200G && srRevMajor >= 3)
          /* This case is added so a sensor under-going factory test before
           * it's SR number has been written to it will set the RN4678 in the
           * correct mode in order for the FW to boot properly. This will
           * conflict with any older sensors that have the TCXO placed */
          || (srId == 0xFF && srRevMajor == 0xFF && srRevMinor == 0xFF)));
}

uint8_t ShimBrd_isSubstitutionNeededForWrAccel(void)
{
  uint8_t srId = daughterCardIdPage.expansion_brd.exp_brd_id;
  uint8_t srRevMajor = daughterCardIdPage.expansion_brd.exp_brd_major;
  uint8_t srRevMinor = daughterCardIdPage.expansion_brd.exp_brd_minor;

  return (hwId == HW_ID_SHIMMER3 && ShimBrd_isGyroInUseIcm20948()
      && ((srId == SHIMMER3_IMU && srRevMajor == 9 && srRevMinor == 1)
          || (srId == EXP_BRD_EXG_UNIFIED && srRevMajor == 5 && srRevMinor == 1)
          || (srId == EXP_BRD_GSR_UNIFIED && srRevMajor == 4 && srRevMinor == 1)
          || (srId == EXP_BRD_GSR_UNIFIED && srRevMajor == 4 && srRevMinor == 2)
          || (srId == EXP_BRD_BR_AMP_UNIFIED && srRevMajor == 3 && srRevMinor == 1)
          || (srId == EXP_BRD_PROTO3_DELUXE && srRevMajor == 3 && srRevMinor == 1)
          || (srId == EXP_BRD_PROTO3_MINI && srRevMajor == 3 && srRevMinor == 1)
          || (srId == EXP_BRD_ADXL377_ACCEL_200G && srRevMajor == 2 && srRevMinor == 1)));
}

uint8_t ShimBrd_are2ndGenImuSensorsPresent(void)
{
  return (hwId == HW_ID_SHIMMER3 && ShimBrd_isWrAccelInUseLsm303ahtr() && isBmp280InUse());
}

uint8_t ShimBrd_are2ndGenSensorsPresentAndUnknownBoard(void)
{
  uint8_t srId = daughterCardIdPage.expansion_brd.exp_brd_id;

  return (ShimBrd_are2ndGenImuSensorsPresent()
      && !(srId == SHIMMER_ECG_MD || srId == SHIMMER3_IMU || srId == EXP_BRD_EXG_UNIFIED
          || srId == EXP_BRD_GSR_UNIFIED || srId == EXP_BRD_BR_AMP_UNIFIED));
}

uint8_t ShimBrd_areGsrControlsPinsReversed(void)
{
  uint8_t srId = daughterCardIdPage.expansion_brd.exp_brd_id;
  uint8_t srRevMajor = daughterCardIdPage.expansion_brd.exp_brd_major;
  uint8_t srRevMinor = daughterCardIdPage.expansion_brd.exp_brd_minor;

  return (hwId == HW_ID_SHIMMER3 && srId == EXP_BRD_GSR_UNIFIED
      && srRevMajor == 4 && srRevMinor == 1);
}
#endif

uint8_t ShimBrd_areADS1292RClockLinesTied(void)
{
  uint8_t srId = daughterCardIdPage.expansion_brd.exp_brd_id;
  uint8_t srRevMajor = daughterCardIdPage.expansion_brd.exp_brd_major;

  return (hwId == HW_ID_SHIMMER3 && srId == EXP_BRD_EXG_UNIFIED && srRevMajor >= 4)
      || (hwId == HW_ID_SHIMMER3R && srId == EXP_BRD_EXG_UNIFIED);
}

void ShimBrd_parseDaughterCardId(void)
{
  memset(daughtCardIdStr, 0x00, sizeof(daughtCardIdStr));

  uint8_t srId = daughterCardIdPage.expansion_brd.exp_brd_id;
  switch (srId)
  {
    case SHIMMER3_IMU:
      sprintf(daughtCardIdStr, "Shimmer3 IMU");
      break;
    case EXP_BRD_GSR:
    case EXP_BRD_GSR_UNIFIED:
      sprintf(daughtCardIdStr, "Shimmer3 GSR+");
      break;
    case EXP_BRD_EXG:
    case EXP_BRD_EXG_UNIFIED:
      sprintf(daughtCardIdStr, "Shimmer3 ExG");
      break;
    case EXP_BRD_BR_AMP:
    case EXP_BRD_BR_AMP_UNIFIED:
      sprintf(daughtCardIdStr, "Shimmer3 Bridge Amplifier");
      break;
    case EXP_BRD_PROTO3_MINI:
      sprintf(daughtCardIdStr, "Shimmer3 Proto3 Mini");
      break;
    case EXP_BRD_PROTO3_DELUXE:
      sprintf(daughtCardIdStr, "Shimmer3 Proto3 Deluxe");
      break;
    case EXP_BRD_ADXL377_ACCEL_200G:
      sprintf(daughtCardIdStr, "Shimmer3 200G Accel");
      break;
    case EXP_BRD_H3LIS331DL_ACCEL_HIGH_G:
      sprintf(daughtCardIdStr, "Shimmer3 100G Accel");
      break;
    case SHIMMER4_SDK:
      sprintf(daughtCardIdStr, "Shimmer4 SDK");
      break;
    case SHIMMER_ECG_MD:
      sprintf(daughtCardIdStr, "Shimmer3 ECG MD");
      break;
    default:
      sprintf(daughtCardIdStr, "SR%d", srId);
      break;
  }
}

shimmer_expansion_brd *ShimBrd_getDaughtCardId(void)
{
  return &daughterCardIdPage.expansion_brd;
}

uint8_t *ShimBrd_getDaughtCardIdPtr(void)
{
  return &daughterCardIdPage.raw[0];
}

char *ShimBrd_getDaughtCardIdStrPtr(void)
{
  return &daughtCardIdStr[0];
}

uint8_t ShimBrd_isDaughterCardIdSet(void)
{
  return (daughterCardIdPage.expansion_brd.exp_brd_id != 0x00
      && daughterCardIdPage.expansion_brd.exp_brd_id != 0xFF);
}

#if defined(SHIMMER3)
void ShimBrd_setWrAccelAndMagInUse(uint8_t wr_accel_and_mag_in_use)
{
  wrAccelAndMagInUse = wr_accel_and_mag_in_use;
}

uint8_t ShimBrd_isWrAccelInUseLsm303dlhc(void)
{
  return wrAccelAndMagInUse == WR_ACCEL_AND_MAG_LSM303DLHC_IN_USE;
}

uint8_t ShimBrd_isWrAccelInUseLsm303ahtr(void)
{
  return wrAccelAndMagInUse == WR_ACCEL_AND_MAG_LSM303AHTR_IN_USE;
}

uint8_t ShimBrd_isWrAccelInUseIcm20948(void)
{
  return wrAccelAndMagInUse == WR_ACCEL_AND_MAG_ICM20948_IN_USE;
}

void ShimBrd_setGyroInUse(uint8_t gyro_in_use)
{
  gyroInUse = gyro_in_use;
}

uint8_t ShimBrd_isGyroInUseMpu9x50(void)
{
  return gyroInUse == GYRO_MPU9X50_IN_USE;
}

uint8_t ShimBrd_isGyroInUseIcm20948(void)
{
  return gyroInUse == GYRO_ICM20948_IN_USE;
}
#endif

#if defined(SHIMMER3)
uint8_t ShimBrd_isLnAccelKxtc9_2050Present(void)
{
  //Assuming here that if BMP280 and LSM303AHTR/ICM20948 present, infer low-noise accel is KXTC9-2050 and not KXRB5-2042
  return ((ShimBrd_isWrAccelInUseLsm303ahtr() || ShimBrd_isWrAccelInUseIcm20948())
      && isBmp280InUse());
}
#endif

uint8_t ShimBrd_isAdxl371Present(void)
{
  return (ShimBrd_isDaughterCardIdSet() && hwId == HW_ID_SHIMMER3R
      && (daughterCardIdPage.expansion_brd.exp_brd_id == SHIMMER3_IMU
          || ShimBrd_isBoardSrNumber(EXP_BRD_GSR_UNIFIED, 6, 0)
          || ShimBrd_isBoardSrNumber(EXP_BRD_GSR_UNIFIED, 7, 0)
          || ShimBrd_isBoardSrNumber(EXP_BRD_GSR_UNIFIED, 7, 1)
          || ShimBrd_isBoardSrNumber(EXP_BRD_EXG_UNIFIED, 7, 0)
          || ShimBrd_isBoardSrNumber(EXP_BRD_EXG_UNIFIED, 7, 1)
          || ShimBrd_isBoardSrNumber(EXP_BRD_BR_AMP_UNIFIED, 4, 0)
          || ShimBrd_isBoardSrNumber(EXP_BRD_PROTO3_DELUXE, 4, 0)));
}

uint8_t ShimBrd_isLis3mdlPresent(void)
{
  return (ShimBrd_isDaughterCardIdSet() && hwId == HW_ID_SHIMMER3R
      && (ShimBrd_isBoardSrNumber(SHIMMER3_IMU, 11, 0)
          || ShimBrd_isBoardSrNumber(EXP_BRD_GSR_UNIFIED, 6, 0)
          || ShimBrd_isBoardSrNumber(EXP_BRD_GSR_UNIFIED, 7, 0)
          || ShimBrd_isBoardSrNumber(EXP_BRD_GSR_UNIFIED, 7, 1)
          || ShimBrd_isBoardSrNumber(EXP_BRD_EXG_UNIFIED, 7, 0)
          || ShimBrd_isBoardSrNumber(EXP_BRD_EXG_UNIFIED, 7, 1)
          || ShimBrd_isBoardSrNumber(EXP_BRD_BR_AMP_UNIFIED, 4, 0)
          || ShimBrd_isBoardSrNumber(EXP_BRD_PROTO3_DELUXE, 4, 0)));
}

uint8_t isAds7028Present(void)
{
  return (ShimBrd_isDaughterCardIdSet() && hwId == HW_ID_SHIMMER3R
      && !ShimBrd_isBoardSrNumber(EXP_BRD_GSR_UNIFIED, 6, 0));
}

uint8_t ShimBrd_isI2c4Supported(void)
{
  return (ShimBrd_isDaughterCardIdSet() && hwId == HW_ID_SHIMMER3R
      && (daughterCardIdPage.expansion_brd.exp_brd_id == EXP_BRD_GSR_UNIFIED
          || daughterCardIdPage.expansion_brd.exp_brd_id == EXP_BRD_PROTO3_DELUXE));
}

uint8_t ShimBrd_isBoardSr48_6_0(void)
{
  return ShimBrd_isDaughterCardIdSet()
      && ShimBrd_isBoardSrNumber(EXP_BRD_GSR_UNIFIED, 6, 0);
}

uint8_t ShimBrd_isI2cOnPPGControlledByAdcChip(void)
{
  return ShimBrd_isDaughterCardIdSet()
      && (ShimBrd_isBoardSrNumber(EXP_BRD_GSR_UNIFIED, 7, 0)
          || ShimBrd_isBoardSrNumber(EXP_BRD_GSR_UNIFIED, 7, 1));
}

uint8_t ShimBrd_areMcuAdcsUsedForSensing(void)
{
  return (ShimBrd_isHwId(HW_ID_SHIMMER3) || ShimBrd_isHwId(SHIMMER4_SDK)
      || ShimBrd_isBoardSr48_6_0());
}

uint8_t ShimBrd_isBoardSrNumber(uint8_t exp_brd_id, uint8_t exp_brd_major, uint8_t exp_brd_minor)
{
  return (ShimBrd_isDaughterCardIdSet() //&& hwId == HW_ID_SHIMMER3R
      && (daughterCardIdPage.expansion_brd.exp_brd_id == exp_brd_id
          && daughterCardIdPage.expansion_brd.exp_brd_major == exp_brd_major
          && daughterCardIdPage.expansion_brd.exp_brd_minor == exp_brd_minor));
}

uint8_t ShimBrd_isHwId(uint8_t hwIdToCheck)
{
  return hwId == hwIdToCheck;
}

uint8_t ShimBrd_isExpBrdId(uint8_t expIdToCheck)
{
  return (ShimBrd_isDaughterCardIdSet()
      && (daughterCardIdPage.expansion_brd.exp_brd_id == expIdToCheck));
}

uint8_t ShimBrd_checkCorrectStateForBoot0(void)
{
  /* nBOOT0 Default = 1. Early S3R boards required nBoot to remain unchanged
   * (SR48-6-0). The SR48-7-0 and SR47-7-0 had inverted nBoot0 circuitry. An ECO
   * removed this and the modified boards were updated to SR47-7-1 and SR48-7-1.
   * Future boards rely on firmware setting nBoot0 to 0 with no external
   * inversion circuitry. */
  if (ShimBrd_isDaughterCardIdSet() && hwId == HW_ID_SHIMMER3R
      && (ShimBrd_isBoardSrNumber(EXP_BRD_GSR_UNIFIED, 6, 0)
          || ShimBrd_isBoardSrNumber(EXP_BRD_GSR_UNIFIED, 7, 0)
          || ShimBrd_isBoardSrNumber(EXP_BRD_EXG_UNIFIED, 7, 0)))
  {
    /* MCU comes with Default nBOOT0 = 1 */
    return 1;
  }
  else
  {
    /* We've changed direction to match Shimmer3 for backwards compatibility
     * with docks */
    return 0;
  }
}

__weak uint8_t ShimBrd_doesDeviceSupportBle(void)
{
  return 0;
}

__weak uint8_t ShimBrd_doesDeviceSupportBtClassic(void)
{
  return 0;
}
