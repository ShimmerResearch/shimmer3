/*
 * i2c.c
 *
 *  Created on: 24 Mar 2025
 *      Author: MarkNolan
 */

#include "i2c.h"

#include "Shimmer_Driver/shimmer_driver_include.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define PRES_TS_EN 0

uint8_t preSampleMpuMag, preSampleBmpPress, bmpPressFreq, bmpPressCount,
    sampleBmpTemp, sampleBmpTempFreq, preSampleMpuMag, mpuMagFreq, mpuMagCount;

uint8_t bmpTempCurrentVal[BMPX80_TEMP_BUFF_SIZE],
    bmpPresCurrentVal[BMPX80_PRESS_BUFF_SIZE], bmpVal[BMPX80_PACKET_SIZE];

/*variables for ICM20948 Accel/Gyro*/
uint8_t icm20948AccelGyroBuf[12] = { 0 };
bool isIcm20948AccelEn = false;
bool isIcm20948GyroEn = false;

#if PRES_TS_EN
uint16_t bmpTempInterval, bmpPresInterval;
uint64_t bmpTempStartTs, bmpPresStartTs;
uint64_t bmpTempSampleTs, bmpPresSampleTs;
#endif

extern uint16_t clk_30, clk_45, clk_55, clk_64, clk_75, clk_133, clk_90, clk_120,
    clk_135, clk_225, clk_255, clk_432, clk_1000, clk_2500, clk_90_45, clk_90_64,
    clk_90_75, clk_133_90, clk_135_90, clk_225_90, clk_255_90, clk_432_90;

void I2C_varsInit(void)
{
  preSampleMpuMag = 0;
  preSampleBmpPress = 0;
  sampleBmpTemp = 0;
}

void I2C_start(uint8_t controlExpBrd)
{
  /* set SW_I2C high to power on all I2C chips */
  Board_setI2cPower(1);
  if (controlExpBrd)
  {
    Board_setExpansionBrdPower(1);
    __delay_cycles(480000); //20ms
  }
  else
  {
    /* wait 1s (assuming 24MHz MCLK) to allow for power ramp up */
    //__delay_cycles(24000000);
    /* wait 100ms (assuming 24MHz MCLK) to allow for power ramp up */
    __delay_cycles(2400000);
  }

  /* Source from SMCLK, which is running @ 24MHz. 4kHz desired BRCLK which
   * is max for this part */
  I2C_Master_Init(S_MCLK, 24000000, 400000);
}

void I2C_stop(uint8_t controlExpBrd)
{
  /* wait 10ms (assuming 24MHz MCLK) to any on-going I2C comms to finish */
  _delay_cycles(240000);
  I2C_Disable();
  Board_setI2cPower(0);
  if (controlExpBrd)
  {
    /* 5ms delay needed to ensure no writes pending but this is handled above */
    Board_setExpansionBrdPower(0);
  }
}

void I2C_startSensing(void)
{
  uint8_t ICMsampleRateDiv = 0;
  gConfigBytes *storedConfigPtr = ShimConfig_getStoredConfig();

  if (sensing.nbrI2cChans)
  {
    I2C_start(0);
  }

  if (storedConfigPtr->chEnGyro || storedConfigPtr->chEnAltAccel
      || storedConfigPtr->chEnAltMag
      || (ShimBrd_isWrAccelInUseIcm20948()
          && (storedConfigPtr->chEnWrAccel || storedConfigPtr->chEnMag)))
  {
    if (ShimBrd_isGyroInUseIcm20948())
    {
      ICM20948_init();
      ICM20948_wake(1);
      volatile uint8_t icm_id = ICM20948_getId();    //should be 0xEA
      volatile uint8_t mag_id = ICM20948_getMagId(); //should be 0x09
    }
    else if (ShimBrd_isGyroInUseMpu9x50())
    {
      MPU9150_init();
      MPU9150_wake(1);
      volatile uint8_t mpu_id = MPU9150_getId();
    }

    if (storedConfigPtr->chEnGyro || storedConfigPtr->chEnAltAccel
        || (ShimBrd_isWrAccelInUseIcm20948() && storedConfigPtr->chEnWrAccel))
    {
      if (ShimBrd_isGyroInUseIcm20948())
      {
        ICMsampleRateDiv
            = ICM20948_convertSampleRateDivFromMPU9X50(storedConfigPtr->gyroRate, 0);

        ICM20948_setGyroSamplingRate(ICMsampleRateDiv);
      }
      else if (ShimBrd_isGyroInUseMpu9x50())
      {
        MPU9150_setSamplingRate(storedConfigPtr->gyroRate);
      }
      if (storedConfigPtr->chEnGyro)
      {
        if (ShimBrd_isGyroInUseIcm20948())
        {
          ICM20948_setGyroSensitivity(ShimConfig_gyroRangeGet());
        }
        else if (ShimBrd_isGyroInUseMpu9x50())
        {
          MPU9150_setGyroSensitivity(ShimConfig_gyroRangeGet()); //This needs to go after the wake?
        }
      }
      if (storedConfigPtr->chEnAltAccel
          || (ShimBrd_isWrAccelInUseIcm20948() && storedConfigPtr->chEnWrAccel))
      {
        uint8_t wrAccelRange = storedConfigPtr->altAccelRange;
        if (ShimBrd_isGyroInUseIcm20948())
        {
          if (ShimBrd_isWrAccelInUseIcm20948())
          {
            //Use setting design for WR accel - re-map to suit the ICM-20948
            wrAccelRange = storedConfigPtr->wrAccelRange;
            switch (wrAccelRange)
            {
              case 2:
                wrAccelRange = 1; //+-4g
                break;
              case 3:
                wrAccelRange = 2; //+-8g
                break;
              case 1:
                wrAccelRange = 3; //+-16g
                break;
              default:
                break;
            }
          }
          ICM20948_setAccelRange(wrAccelRange);
        }
        else if (ShimBrd_isGyroInUseMpu9x50())
        {
          MPU9150_setAccelRange(wrAccelRange);
        }
      }
    }
    else
    {
      //For some reason it seems necessary to power on the gyro/accel core before trying to access the mag
      //followed by one other I2C command (read or write)
      //No idea why
      //timing delays or other I2C commands to gyro/accel core do not seem to have the same effect?!?
      //Only relevant first time mag is accessed after powering up MPU9150
      if (ShimBrd_isGyroInUseIcm20948())
      {
        ICM20948_wake(0);
      }
      else if (ShimBrd_isGyroInUseMpu9x50())
      {
        MPU9150_wake(0);
      }
    }
    if (storedConfigPtr->chEnAltMag
        || (ShimBrd_isWrAccelInUseIcm20948() && storedConfigPtr->chEnMag))
    {
      if (ShimBrd_isGyroInUseIcm20948())
      {
        ICM20948_setMagSamplingRateFromShimmerRate(storedConfigPtr->samplingRateTicks);
      }
      else if (ShimBrd_isGyroInUseMpu9x50())
      {
        if (storedConfigPtr->samplingRateTicks >= clk_120)
        {
          //max of approx. 3ms to sample everything + 9ms between starting mag to data ready
          //so 12ms in total (394 ticks of 32768Hz clock = 12.024ms) (3070 ticks of 255765.625Hz clock = 12.024ms)
          //so there is time to get the mag sampled before the readings need to start each sample period
          preSampleMpuMag = 1;
        }
        else
        {
          MPU9150_startMagMeasurement();
          preSampleMpuMag = 0;
          mpuMagCount = mpuMagFreq = (clk_90 / storedConfigPtr->samplingRateTicks) + 1;
        }
      }
    }
  }

  if (!ShimBrd_isWrAccelInUseIcm20948()
      && (storedConfigPtr->chEnMag || storedConfigPtr->chEnWrAccel))
  {
    if (storedConfigPtr->chEnWrAccel)
    {
      if (ShimBrd_isWrAccelInUseLsm303dlhc())
      {
        LSM303DLHC_accelInit(storedConfigPtr->wrAccelRate, //sampling rate
            storedConfigPtr->wrAccelRange,                 //range
            storedConfigPtr->wrAccelLpModeLsb,             //low power mode
            storedConfigPtr->wrAccelHrMode); //high resolution mode
      }
      else
      {
        LSM303AHTR_accelInit(storedConfigPtr->wrAccelRate, //sampling rate
            storedConfigPtr->wrAccelRange,                 //range
            storedConfigPtr->wrAccelLpModeLsb,             //low power mode
            storedConfigPtr->wrAccelHrMode); //high resolution mode
      }
    }
    if (storedConfigPtr->chEnMag)
    {
      if (ShimBrd_isWrAccelInUseLsm303dlhc())
      {
        LSM303DLHC_magInit(storedConfigPtr->magRate, //sampling rate
            storedConfigPtr->magRange);              //gain
      }
      else
      {
        LSM303AHTR_magInit(storedConfigPtr->magRate); //sampling rate
      }
    }
  }

  if (storedConfigPtr->chEnPressureAndTemperature)
  {
    uint16_t bmpX80SamplingTimeTicks = getBmpX80SamplingTimeInTicks();
    if (storedConfigPtr->samplingRateTicks >= (clk_30 + bmpX80SamplingTimeTicks))
    {
      preSampleBmpPress = 1;
      bmpPressFreq = 1; //required for the calculation of sampleBmpTempFreq below
    }
    else
    {
      //sample, then check each sample period if ready to read. Start new sample immediately
#if PRES_TS_EN
      bmpTempStartTs = RTC_get64();
#endif

      BMPX80_startTempMeasurement();

      preSampleBmpPress = 0;
      bmpPressCount = bmpPressFreq
          = (bmpX80SamplingTimeTicks / storedConfigPtr->samplingRateTicks) + 1;
    }
    //only need to sample temp once a second at most
    if (storedConfigPtr->samplingRateTicks >= clk_2500)
    {
      //less than 4Hz
      //so every second sample must be temp
      sampleBmpTemp = sampleBmpTempFreq = 1;
    }
    else
    {
      sampleBmpTemp = sampleBmpTempFreq
          = (uint8_t) ((round(ShimConfig_freqDiv(storedConfigPtr->samplingRateTicks)) - 1)
              / bmpPressFreq);
    }
#if PRES_TS_EN
    bmpPresInterval = bmpX80SamplingTimeTicks;
    //TODO Set as 5.5ms here for BMP280 but datasheet recommends to set same as pressure channel
    bmpTempInterval = (isBmp180InUse() ? clk_45 : clk_55);
#endif
    memset(bmpVal, 0, BMPX80_PACKET_SIZE);
  }
}

void I2C_stopSensing(void)
{
  gConfigBytes *storedConfigPtr = ShimConfig_getStoredConfig();

  if (storedConfigPtr->chEnGyro || storedConfigPtr->chEnAltAccel
      || storedConfigPtr->chEnAltMag
      || (ShimBrd_isWrAccelInUseIcm20948()
          && (storedConfigPtr->chEnWrAccel || storedConfigPtr->chEnMag)))
  {
    if (ShimBrd_isGyroInUseIcm20948())
    {
      ICM20948_setMagMode(AK09916_PWR_DOWN);
      ICM20948_wake(0);
    }
    else if (ShimBrd_isGyroInUseMpu9x50())
    {
      MPU9150_wake(0);
    }
  }

  if (!ShimBrd_isWrAccelInUseIcm20948()
      && (storedConfigPtr->chEnMag || storedConfigPtr->chEnWrAccel))
  {
    if (ShimBrd_isWrAccelInUseLsm303dlhc())
    {
      LSM303DLHC_sleep();
    }
    else
    {
      LSM303AHTR_sleep();
    }
  }

  I2C_stop(0);
}

void I2C_configureChannels(void)
{
  uint8_t *channel_contents_ptr = &sensing.cc[sensing.ccLen];
  uint8_t nbr_i2c_chans = 0;
  gConfigBytes *storedConfigPtr = ShimConfig_getStoredConfig();

  //Digi Gyro
  if (storedConfigPtr->chEnGyro)
  {
    *channel_contents_ptr++ = X_GYRO;
    *channel_contents_ptr++ = Y_GYRO;
    *channel_contents_ptr++ = Z_GYRO;
    nbr_i2c_chans += 3;
    sensing.ptr.gyro = sensing.dataLen;
    sensing.dataLen += 6;
  }
  //Digi Accel
  if (storedConfigPtr->chEnWrAccel)
  {
    *channel_contents_ptr++ = X_WR_ACCEL;
    *channel_contents_ptr++ = Y_WR_ACCEL;
    *channel_contents_ptr++ = Z_WR_ACCEL;
    nbr_i2c_chans += 3;
    sensing.ptr.accel2 = sensing.dataLen;
    sensing.dataLen += 6;
  }
  //Mag
  if (storedConfigPtr->chEnMag)
  {
    *channel_contents_ptr++ = X_MAG;

    if (ShimBrd_isWrAccelInUseLsm303dlhc())
    {
      *channel_contents_ptr++ = Z_MAG;
      *channel_contents_ptr++ = Y_MAG;
    }
    else
    {
      *channel_contents_ptr++ = Y_MAG;
      *channel_contents_ptr++ = Z_MAG;
    }
    nbr_i2c_chans += 3;
    sensing.ptr.mag1 = sensing.dataLen;
    sensing.dataLen += 6;
  }
  //Digi Accel
  if (storedConfigPtr->chEnAltAccel)
  {
    *channel_contents_ptr++ = X_ALT_ACCEL;
    *channel_contents_ptr++ = Y_ALT_ACCEL;
    *channel_contents_ptr++ = Z_ALT_ACCEL;
    nbr_i2c_chans += 3;
    sensing.ptr.accel3 = sensing.dataLen;
    sensing.dataLen += 6;
  }
  //Digi Accel
  if (storedConfigPtr->chEnAltMag)
  {
    *channel_contents_ptr++ = X_ALT_MAG;
    *channel_contents_ptr++ = Y_ALT_MAG;
    *channel_contents_ptr++ = Z_ALT_MAG;
    nbr_i2c_chans += 3;
    sensing.ptr.mag2 = sensing.dataLen;
    sensing.dataLen += 6;
  }

  if (storedConfigPtr->chEnPressureAndTemperature)
  {
    *channel_contents_ptr++ = BMP_TEMPERATURE;
    *channel_contents_ptr++ = BMP_PRESSURE;
    nbr_i2c_chans += 2; //PRES & TEMP, ON/OFF together
    sensing.ptr.pressure = sensing.dataLen;
    sensing.dataLen += BMPX80_PACKET_SIZE;
  }

  isIcm20948AccelEn = false;
  isIcm20948GyroEn = false;
  if (ShimBrd_isGyroInUseIcm20948() && storedConfigPtr->chEnGyro)
  {
    isIcm20948GyroEn = true;
  }
  if (storedConfigPtr->chEnAltAccel
      || (ShimBrd_isWrAccelInUseIcm20948() && storedConfigPtr->chEnWrAccel))
  {
    isIcm20948AccelEn = true;
  }

  sensing.ccLen += nbr_i2c_chans;
  sensing.nbrI2cChans = nbr_i2c_chans;
}

//TODO need to check all sections of code that intentionally try to repeat the last sample and make sure they reference the correct data
void I2C_pollSensors(void)
{
  gConfigBytes *storedConfigPtr = ShimConfig_getStoredConfig();
  PACKETBufferTypeDef *packetBufPtr = ShimSens_getPacketBuffAtWrIdx();
  uint8_t *dataBufPtr = ShimSens_getDataBuffAtWrIdx();
  static uint8_t mpu9150MagBuf[6U] = { 0 };
  static uint8_t mpu9150AccelBuf[6U] = { 0 };
  static uint8_t mpu9150GyroBuf[6U] = { 0 };

  //Pre-read the 9-axis chip in-case it is needed for substition on the LSM303 channels
  if (isIcm20948AccelEn && isIcm20948GyroEn)
  {
    ICM20948_getAccelAndGyro(&icm20948AccelGyroBuf[0]);
  }
  else if (isIcm20948GyroEn)
  {
    ICM20948_getGyro(&icm20948AccelGyroBuf[6U]);
  }
  else if (isIcm20948AccelEn)
  {
    ICM20948_getAccel(&icm20948AccelGyroBuf[0]);
  }

  if (storedConfigPtr->chEnGyro)
  {
    if (ShimBrd_isGyroInUseIcm20948())
    {
      dataBufPtr[sensing.ptr.gyro + 0U] = icm20948AccelGyroBuf[0U + 6U];
      dataBufPtr[sensing.ptr.gyro + 1U] = icm20948AccelGyroBuf[1U + 6U];
      dataBufPtr[sensing.ptr.gyro + 2U] = icm20948AccelGyroBuf[2U + 6U];
      dataBufPtr[sensing.ptr.gyro + 3U] = icm20948AccelGyroBuf[3U + 6U];
      dataBufPtr[sensing.ptr.gyro + 4U] = icm20948AccelGyroBuf[4U + 6U];
      dataBufPtr[sensing.ptr.gyro + 5U] = icm20948AccelGyroBuf[5U + 6U];
    }
    else if (ShimBrd_isGyroInUseMpu9x50())
    {
      MPU9150_getGyro(&mpu9150GyroBuf[0]);
      memcpy(&dataBufPtr[sensing.ptr.gyro], &mpu9150GyroBuf[0], 6);
    }
  }

  //needs to be static to carry sample forward between samples
  static uint8_t icm20948MagBuf[ICM_MAG_RD_SIZE] = { 0 };
  uint8_t icm20948MagRdy = 0;
  if (storedConfigPtr->chEnAltMag
      || (ShimBrd_isWrAccelInUseIcm20948() && storedConfigPtr->chEnMag))
  {
    if (ICM20948_isMagSampleSkipEnabled())
    {
      /* This system tries to avoid lock-up scenario in the ICM20948 Mag
       * (AK09916) in-which we see a 0.1 ms worth of repeated data samples
       * if the chip was being read from too often. */
      if (icm20948MagRdy = ICM20948_hasTimeoutPeriodPassed(packetBufPtr->timestampTicks))
      {
        icm20948MagRdy = ICM20948_getMagAndStatus(
            packetBufPtr->timestampTicks, &icm20948MagBuf[0]);
      }
    }
    else
    {
      /* Original approach in-which the status 1 register is read first
       * before reading the remaining bytes. This approach was found to
       * work fine <512Hz but after that it would cause packet loss due
       * to the length of time the I2C operations take to finish. */
      if (icm20948MagRdy = ICM20948_isMagDataRdy())
      {
        ICM20948_getMag(&icm20948MagBuf[1]);
      }
    }
  }

  if (storedConfigPtr->chEnWrAccel)
  {
    if (ShimBrd_isWrAccelInUseIcm20948())
    {
      //Swap byte order to immitate LSM303 chip
      dataBufPtr[sensing.ptr.accel2 + 0U] = icm20948AccelGyroBuf[1U];
      dataBufPtr[sensing.ptr.accel2 + 1U] = icm20948AccelGyroBuf[0U];

      //Invert sign of uncalibrated Y-axis to match LSM303 chip placement
      int16_t signInvertBuffer
          = -((int16_t) ((icm20948AccelGyroBuf[3U] << 8) | icm20948AccelGyroBuf[2U]));
      dataBufPtr[sensing.ptr.accel2 + 2U] = (signInvertBuffer >> 8) & 0xFF;
      dataBufPtr[sensing.ptr.accel2 + 3U] = signInvertBuffer & 0xFF;

      dataBufPtr[sensing.ptr.accel2 + 4U] = icm20948AccelGyroBuf[5U];
      dataBufPtr[sensing.ptr.accel2 + 5U] = icm20948AccelGyroBuf[4U];
    }
    else
    {
      if (ShimBrd_isWrAccelInUseLsm303dlhc())
      {
        LSM303DLHC_getAccel(&dataBufPtr[sensing.ptr.accel2]);
      }
      else
      {
        LSM303AHTR_getAccel(&dataBufPtr[sensing.ptr.accel2]);
      }
    }
  }

  if (storedConfigPtr->chEnMag)
  {
    if (ShimBrd_isWrAccelInUseIcm20948())
    {
      dataBufPtr[sensing.ptr.mag1 + 0U] = icm20948MagBuf[ICM_MAG_IDX_XOUT_L];
      dataBufPtr[sensing.ptr.mag1 + 1U] = icm20948MagBuf[ICM_MAG_IDX_XOUT_H];

      dataBufPtr[sensing.ptr.mag1 + 2U] = icm20948MagBuf[ICM_MAG_IDX_YOUT_L];
      dataBufPtr[sensing.ptr.mag1 + 3U] = icm20948MagBuf[ICM_MAG_IDX_YOUT_H];

      //Invert sign of uncalibrated Z-axis to match LSM303 chip placement
      int16_t signInvertBuffer = -((int16_t) ((icm20948MagBuf[ICM_MAG_IDX_ZOUT_H] << 8)
          | icm20948MagBuf[ICM_MAG_IDX_ZOUT_L]));
      dataBufPtr[sensing.ptr.mag1 + 4U] = signInvertBuffer & 0xFF;
      dataBufPtr[sensing.ptr.mag1 + 5U] = (signInvertBuffer >> 8) & 0xFF;
    }
    else
    {
      if (ShimBrd_isWrAccelInUseLsm303dlhc())
      {
        LSM303DLHC_getMag(&dataBufPtr[sensing.ptr.mag1]);
      }
      else
      {
        LSM303AHTR_getMag(&dataBufPtr[sensing.ptr.mag1]);
      }
    }
  }
  if (storedConfigPtr->chEnAltAccel)
  {
    if (ShimBrd_isGyroInUseIcm20948())
    {
      dataBufPtr[sensing.ptr.accel3 + 0U] = icm20948AccelGyroBuf[0U];
      dataBufPtr[sensing.ptr.accel3 + 1U] = icm20948AccelGyroBuf[1U];
      dataBufPtr[sensing.ptr.accel3 + 2U] = icm20948AccelGyroBuf[2U];
      dataBufPtr[sensing.ptr.accel3 + 3U] = icm20948AccelGyroBuf[3U];
      dataBufPtr[sensing.ptr.accel3 + 4U] = icm20948AccelGyroBuf[4U];
      dataBufPtr[sensing.ptr.accel3 + 5U] = icm20948AccelGyroBuf[5U];
    }
    else if (ShimBrd_isGyroInUseMpu9x50())
    {
      MPU9150_getAccel(&mpu9150AccelBuf[0]);
      memcpy(&dataBufPtr[sensing.ptr.accel3], &mpu9150AccelBuf[0], 6);
    }
  }
  if (storedConfigPtr->chEnAltMag)
  {
    if (ShimBrd_isGyroInUseIcm20948())
    {
      dataBufPtr[sensing.ptr.mag2 + 0U] = icm20948MagBuf[ICM_MAG_IDX_XOUT_L];
      dataBufPtr[sensing.ptr.mag2 + 1U] = icm20948MagBuf[ICM_MAG_IDX_XOUT_H];
      dataBufPtr[sensing.ptr.mag2 + 2U] = icm20948MagBuf[ICM_MAG_IDX_YOUT_L];
      dataBufPtr[sensing.ptr.mag2 + 3U] = icm20948MagBuf[ICM_MAG_IDX_YOUT_H];
      dataBufPtr[sensing.ptr.mag2 + 4U] = icm20948MagBuf[ICM_MAG_IDX_ZOUT_L];
      dataBufPtr[sensing.ptr.mag2 + 5U] = icm20948MagBuf[ICM_MAG_IDX_ZOUT_H];
    }
    else if (ShimBrd_isGyroInUseMpu9x50())
    {
      if (preSampleMpuMag)
      {
        MPU9150_getMag(&mpu9150MagBuf[0]);
      }
      else if (!mpuMagCount--)
      {
        MPU9150_getMag(&mpu9150MagBuf[0]);
        mpuMagCount = mpuMagFreq;
        MPU9150_startMagMeasurement();
      }
      else
      {
        //Mag not ready, repeat last sample
      }
      memcpy(&dataBufPtr[sensing.ptr.mag2], &mpu9150MagBuf[0], 6);
    }
  }
  if (storedConfigPtr->chEnPressureAndTemperature)
  {
    if (preSampleBmpPress)
    {
      if (sampleBmpTemp == sampleBmpTempFreq)
      {
        sampleBmpTemp = 0;
#if PRES_TS_EN
        bmpTempSampleTs = RTC_get64();
        if ((bmpTempSampleTs - bmpTempStartTs > bmpTempInterval)
            && (bmpTempStartTs > bmpPresStartTs))
        {
          BMPX80_getTemp(bmpTempCurrentVal);
          if (abs(*(int16_t *) bmpTempCurrentVal - *(int16_t *) (bmpVal + 2)) < 30)
          { //wrong
          }
          else
          {
            memcpy(bmpVal, bmpTempCurrentVal, BMPX80_TEMP_BUFF_SIZE);
          }
        }
#else
        BMPX80_getTemp(bmpVal);
#endif
      }
      else
      {
#if PRES_TS_EN
        bmpPresSampleTs = RTC_get64();
        if ((bmpPresSampleTs - bmpPresStartTs > bmpPresInterval)
            && (bmpPresStartTs > bmpTempStartTs))
        {
          BMPX80_getPress(bmpPresCurrentVal);
          if (abs(*(int16_t *) bmpPresCurrentVal - *(int16_t *) (bmpVal)) < 30)
          { //wrong
          }
          else
          {
            memcpy(bmpVal + BMPX80_TEMP_BUFF_SIZE, bmpPresCurrentVal, BMPX80_PRESS_BUFF_SIZE);
          }
        }
#else
        BMPX80_getPress(bmpVal + BMPX80_TEMP_BUFF_SIZE);
#endif
        sampleBmpTemp++;
      }
    }
    else if (!bmpPressCount--)
    {
      if (sampleBmpTemp == sampleBmpTempFreq)
      {
        sampleBmpTemp = 0;
#if PRES_TS_EN
        bmpTempSampleTs = RTC_get64();
        if ((bmpTempSampleTs - bmpTempStartTs > bmpTempInterval)
            && (bmpTempStartTs > bmpPresStartTs))
        {
          BMPX80_getTemp(bmpTempCurrentVal);
          if (abs(*(int16_t *) bmpTempCurrentVal - *(int16_t *) (bmpVal + BMPX80_TEMP_BUFF_SIZE)) < 30)
          { //wrong
          }
          else
          {
            memcpy(bmpVal, bmpTempCurrentVal, BMPX80_TEMP_BUFF_SIZE);
          }
        }
        bmpPresStartTs = RTC_get64();
#else
        BMPX80_getTemp(bmpVal);
#endif
        BMPX80_startPressMeasurement(ShimConfig_configBytePressureOversamplingRatioGet());
      }
      else
      {
#if PRES_TS_EN
        bmpPresSampleTs = RTC_get64();
        if ((bmpPresSampleTs - bmpPresStartTs > bmpPresInterval)
            && (bmpPresStartTs > bmpTempStartTs))
        {
          BMPX80_getPress(bmpPresCurrentVal);
          if (abs(*(int16_t *) bmpPresCurrentVal - *(int16_t *) (bmpVal)) < 30)
          { //wrong
          }
          else
          {
            memcpy(bmpVal + BMPX80_TEMP_BUFF_SIZE, bmpPresCurrentVal, BMPX80_PRESS_BUFF_SIZE);
          }
        }
#else
        BMPX80_getPress(bmpVal + BMPX80_TEMP_BUFF_SIZE);
#endif
        if (++sampleBmpTemp == sampleBmpTempFreq)
        {
#if PRES_TS_EN
          bmpTempStartTs = RTC_get64();
#endif
          BMPX80_startTempMeasurement();
        }
        else
        {
#if PRES_TS_EN
          bmpPresStartTs = RTC_get64();
#endif
          BMPX80_startPressMeasurement(ShimConfig_configBytePressureOversamplingRatioGet());
        }
      }
      bmpPressCount = bmpPressFreq;
    }
    memcpy(&dataBufPtr[sensing.ptr.pressure], bmpVal, BMPX80_PACKET_SIZE);
  }

  ShimSens_i2cCompleteCb();
}

void detectI2cSlaves(void)
{
  i2cSlaveDiscover();

  //Identify the presence of different sensors
  setBmpInUse((i2cSlavePresent(BMP280_ADDR)) ? BMP280_IN_USE : BMP180_IN_USE);

  if (i2cSlavePresent(LSM303AHTR_ACCEL_ADDR))
  {
    ShimBrd_setWrAccelAndMagInUse(WR_ACCEL_AND_MAG_LSM303AHTR_IN_USE);
  }
  else if (i2cSlavePresent(LSM303DHLC_ACCEL_ADDR))
  {
    ShimBrd_setWrAccelAndMagInUse(WR_ACCEL_AND_MAG_LSM303DLHC_IN_USE);
  }
  else
  {
    ShimBrd_setWrAccelAndMagInUse(WR_ACCEL_AND_MAG_NONE_IN_USE);
  }

  ShimEeprom_setIsPresent(i2cSlavePresent(CAT24C16_ADDR));

  if (i2cSlavePresent(ICM20948_ADDR))
  {
    ShimBrd_setGyroInUse(GYRO_ICM20948_IN_USE);
  }
  else if (i2cSlavePresent(MPU9150_ADDR))
  {
    ShimBrd_setGyroInUse(GYRO_MPU9X50_IN_USE);
  }
  else
  {
    ShimBrd_setGyroInUse(GYRO_NONE_IN_USE);
  }
}

void BMPX80_startMeasurement(void)
{
  if (sampleBmpTemp == sampleBmpTempFreq)
  {
#if PRES_TS_EN
    bmpTempStartTs = RTC_get64();
#endif
    BMPX80_startTempMeasurement();
  }
  else
  {
#if PRES_TS_EN
    bmpPresStartTs = RTC_get64();
#endif
    BMPX80_startPressMeasurement(ShimConfig_configBytePressureOversamplingRatioGet());
  }
}

uint16_t getBmpX80SamplingTimeInTicks(void)
{
  uint8_t bmpX80Precision = ShimConfig_configBytePressureOversamplingRatioGet();
  uint16_t bmpX80SamplingTimeTicks = 0;

  //Setting == 0: BMP180 Ultra low power = 4.5ms (max),   BMP280 Ultra low power res = 6.4ms (max)
  //Setting == 1: BMP180 Standard = 7.5ms (max),          BMP280 Standard res = 13.3ms (max)
  //Setting == 2: BMP180 High res = 13.5ms (max),         BMP280 high res = 22.5ms (max)
  //Setting == 3: BMP180 Ultra high res = 25.5ms (max),   BMP280 Ultra high res = 43.2ms (max)

  switch (bmpX80Precision)
  {
    case 0:
      bmpX80SamplingTimeTicks = (isBmp180InUse() ? clk_45 : clk_64);
      break;
    case 1:
      bmpX80SamplingTimeTicks = (isBmp180InUse() ? clk_75 : clk_133);
      break;
    case 2:
      bmpX80SamplingTimeTicks = (isBmp180InUse() ? clk_135 : clk_225);
      break;
    case 3:
    default:
      bmpX80SamplingTimeTicks = (isBmp180InUse() ? clk_255 : clk_432);
      break;
  }
  return bmpX80SamplingTimeTicks;
}

uint16_t getBmpX80SamplingTimeDiffFrom9msInTicks(void)
{
  uint8_t bmpX80Precision = ShimConfig_configBytePressureOversamplingRatioGet();
  uint16_t bmpX80SamplingTimeTicks = 0;

  switch (bmpX80Precision)
  {
    case 0:
      bmpX80SamplingTimeTicks = (isBmp180InUse() ? clk_90_45 : clk_90_64);
      break;
    case 1:
      //Note here that the BMP180 takes <9ms to sample for this setting but the BMP280 takes >9ms
      bmpX80SamplingTimeTicks = (isBmp180InUse() ? clk_90_75 : clk_133_90);
      break;
    case 2:
      bmpX80SamplingTimeTicks = (isBmp180InUse() ? clk_135_90 : clk_225_90);
      break;
    case 3:
    default:
      bmpX80SamplingTimeTicks = (isBmp180InUse() ? clk_255_90 : clk_432_90);
      break;
  }
  return bmpX80SamplingTimeTicks;
}

uint8_t isPreSampleMpuMagEn(void)
{
  return preSampleMpuMag;
}

uint8_t isPreSampleMpuPressEn(void)
{
  return preSampleBmpPress;
}
