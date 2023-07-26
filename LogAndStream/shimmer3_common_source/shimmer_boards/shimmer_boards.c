/*
 * shimmer_boards.c
 *
 *  Created on: 2 May 2023
 *      Author: MarkNolan
 */

#include <stdio.h>
#include <string.h>
#include "shimmer_boards.h"
#include "../BMPX80/bmpX80.h"

char daughtCardIdStr[25];
uint8_t wrAccelAndMagInUse, gyroInUse, eepromIsPresent;

uint8_t isAds1292Present(uint8_t srId)
{
    return (srId == EXP_BRD_EXG)
            || (srId == EXP_BRD_EXG_UNIFIED)
            || (srId == SHIMMER_ECG_MD);
}

uint8_t isRn4678PresentAndCmdModeSupport(uint8_t srId, uint8_t srRev, uint8_t srRevSpecial)
{
    /* Checking EEPROM here to rule out older sensors in factory test which
     * don't have EEPROM fitted */
    return (isEepromIsPresent()
            && ((srId == SHIMMER3_IMU && srRev >= 10)
                    || (srId == EXP_BRD_EXG_UNIFIED && srRev >= 6)
                    || (srId == EXP_BRD_GSR_UNIFIED && srRev >= 5)
                    || (srId == EXP_BRD_BR_AMP_UNIFIED && srRev >= 3)
                    || (srId == EXP_BRD_PROTO3_DELUXE && srRev >= 4)
                    || (srId == EXP_BRD_PROTO3_MINI && srRev >= 4)
                    || (srId == EXP_BRD_ADXL377_ACCEL_200G && srRev >= 3)
                    /* This case is added so a sensor under-going factory test before
                     * it's SR number has been written to it will set the RN4678 in the
                     * correct mode in order for the FW to boot properly. This will
                     * conflict with any older sensors that have the TCXO placed */
                    || (srId == 0xFF && srRev == 0xFF && srRevSpecial == 0xFF)));
}

uint8_t isSubstitutionNeededForWrAccel(uint8_t srId, uint8_t srRevMajor, uint8_t srRevMinor)
{
    return (isGyroInUseIcm20948()
                && ((srId == SHIMMER3_IMU && srRevMajor == 9 && srRevMinor == 1)
                    || (srId == EXP_BRD_EXG_UNIFIED && srRevMajor == 5 && srRevMinor == 1)
                    || (srId == EXP_BRD_GSR_UNIFIED && srRevMajor == 4 && srRevMinor == 1)
                    || (srId == EXP_BRD_GSR_UNIFIED && srRevMajor == 4 && srRevMinor == 2)
                    || (srId == EXP_BRD_BR_AMP_UNIFIED && srRevMajor == 3 && srRevMinor == 1)
                    || (srId == EXP_BRD_PROTO3_DELUXE && srRevMajor == 3 && srRevMinor == 1)
                    || (srId == EXP_BRD_PROTO3_MINI && srRevMajor == 3 && srRevMinor == 1)
                    || (srId == EXP_BRD_ADXL377_ACCEL_200G && srRevMajor == 2 && srRevMinor == 1)));
}

uint8_t are2ndGenImuSensorsPresent(void)
{
    return (isWrAccelInUseLsm303ahtr() && isBmp280InUse());
}

uint8_t are2ndGenSensorsPresentAndUnknownBoard(uint8_t srId)
{
    return (are2ndGenImuSensorsPresent()
            && !(srId == SHIMMER_ECG_MD
                    || srId == SHIMMER3_IMU
                    || srId == EXP_BRD_EXG_UNIFIED
                    || srId == EXP_BRD_GSR_UNIFIED
                    || srId == EXP_BRD_BR_AMP_UNIFIED));
}

uint8_t areGsrControlsPinsReversed(uint8_t srId, uint8_t srRevMajor, uint8_t srRevMinor)
{
    return (srId == EXP_BRD_GSR_UNIFIED && srRevMajor == 4 && srRevMinor == 1);
}

void parseDaughterCardId(uint8_t srId)
{
    memset(daughtCardIdStr, 0x00, sizeof(daughtCardIdStr));

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
    case SHIMMER_ECG_MD:
        sprintf(daughtCardIdStr, "Shimmer3 ECG MD");
        break;
    default:
        sprintf(daughtCardIdStr, "SR%d", srId);
        break;
    }
}

char* getDaughtCardIdStrPtr(void)
{
    return &daughtCardIdStr[0];
}

void setWrAccelAndMagInUse(uint8_t wr_accel_and_mag_in_use)
{
    wrAccelAndMagInUse = wr_accel_and_mag_in_use;
}

uint8_t isWrAccelInUseLsm303dlhc(void)
{
    return wrAccelAndMagInUse == WR_ACCEL_AND_MAG_LSM303DLHC_IN_USE;
}

uint8_t isWrAccelInUseLsm303ahtr(void)
{
    return wrAccelAndMagInUse == WR_ACCEL_AND_MAG_LSM303AHTR_IN_USE;
}

uint8_t isWrAccelInUseIcm20948(void)
{
    return wrAccelAndMagInUse == WR_ACCEL_AND_MAG_ICM20948_IN_USE;
}

void setGyroInUse(uint8_t gyro_in_use)
{
    gyroInUse = gyro_in_use;
}

uint8_t isGyroInUseMpu9x50(void)
{
    return gyroInUse == GYRO_MPU9X50_IN_USE;
}

uint8_t isGyroInUseIcm20948(void)
{
    return gyroInUse == GYRO_ICM20948_IN_USE;
}

void setEepromIsPresent(uint8_t eeprom_is_preset)
{
    eepromIsPresent = eeprom_is_preset;
}

uint8_t isEepromIsPresent(void)
{
    return eepromIsPresent;
}

uint8_t isLnAccelKxtc9_2050Present(void)
{
    // Assuming here that if BMP280 and LSM303AHTR/ICM20948 present, infer low-noise accel is KXTC9-2050 and not KXRB5-2042
    return ((isWrAccelInUseLsm303ahtr() || isWrAccelInUseIcm20948()) && isBmp280InUse());
}
