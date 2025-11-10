/*
 * hal_FactoryTest.c
 *
 *  Created on: 14 Aug 2024
 *      Author: MarkNolan
 */

#include "hal_FactoryTest.h"

#include <stdio.h>

#include "msp430.h"

#include "../../i2c.h"

#include "shimmer_driver_include.h"

#include "log_and_stream_externs.h"
#include "shimmer_definitions.h"
#include "version.h"

char *buffer;

extern void BlinkTimerStart(void);
extern void BlinkTimerStop(void);

void hal_run_factory_test(factory_test_t factoryTestToRun, char *bufPtr)
{
  buffer = bufPtr;

  if (factoryTestToRun == FACTORY_TEST_MAIN || factoryTestToRun == FACTORY_TEST_ICS)
  {
    print_shimmer_model();
    ShimFactoryTest_sendReport("\r\n");
  }

  if (factoryTestToRun == FACTORY_TEST_MAIN || factoryTestToRun == FACTORY_TEST_ICS)
  {
    sd_card_test();
    ShimFactoryTest_sendReport("\r\n");

    bt_module_test();
    ShimFactoryTest_sendReport("\r\n");

    I2C_test();
    ShimFactoryTest_sendReport("\r\n");

    SPI_test();

    if (factoryTestToRun == FACTORY_TEST_MAIN)
    {
      ShimFactoryTest_sendReport("\r\n");
    }
  }

  if (factoryTestToRun == FACTORY_TEST_MAIN || factoryTestToRun == FACTORY_TEST_LEDS)
  {
    led_test();
  }
}

void print_shimmer_model(void)
{
  ShimFactoryTest_sendReport("Shimmer model:\r\n");
  if (ShimBrd_isDaughterCardIdSet())
  {
    sprintf(buffer, " - PASS: %s", ShimBrd_getDaughtCardIdStrPtr());
    ShimFactoryTest_sendReport(buffer);
    shimmer_expansion_brd *daughterCardId = ShimBrd_getDaughtCardId();
    sprintf(buffer, " (SR%d-%d-%d)\r\n", daughterCardId->exp_brd_id,
        daughterCardId->exp_brd_major, daughterCardId->exp_brd_minor);
    ShimFactoryTest_sendReport(buffer);
  }
  else
  {
    ShimFactoryTest_sendReport(" - FAIL: not set\r\n");
  }
}

void led_test(void)
{
  //Stop watch dog timer while running LED test
  WDTCTL = WDTPW | WDTHOLD;

  BlinkTimerStop();

  ShimFactoryTest_sendReport("LED test:\r\n");

  Board_ledOff(LED_ALL);
  ShimFactoryTest_sendReport(" - All LEDs off\r\n");
  __delay_cycles(DELAY_BETWEEN_LED_CHANGES_TICKS);

  ShimFactoryTest_sendReport(" - Lower Green LED on\r\n");
  Board_ledOn(LED_LWR_GREEN);
  __delay_cycles(DELAY_BETWEEN_LED_CHANGES_TICKS);
  Board_ledOff(LED_ALL);

  ShimFactoryTest_sendReport(" - Lower Yellow LED on\r\n");
  Board_ledOn(LED_LWR_YELLOW);
  __delay_cycles(DELAY_BETWEEN_LED_CHANGES_TICKS);
  Board_ledOff(LED_ALL);

  ShimFactoryTest_sendReport(" - Lower Red LED on\r\n");
  Board_ledOn(LED_LWR_RED);
  __delay_cycles(DELAY_BETWEEN_LED_CHANGES_TICKS);
  Board_ledOff(LED_ALL);

  ShimFactoryTest_sendReport(" - Upper Green LED on\r\n");
  Board_ledOn(LED_UPR_GREEN);
  __delay_cycles(DELAY_BETWEEN_LED_CHANGES_TICKS);
  Board_ledOff(LED_ALL);

  ShimFactoryTest_sendReport(" - Upper Blue LED on\r\n");
  Board_ledOn(LED_UPR_BLUE);
  __delay_cycles(DELAY_BETWEEN_LED_CHANGES_TICKS);
  Board_ledOff(LED_ALL);

  ShimFactoryTest_sendReport(" - All LEDs off\r\n");
  __delay_cycles(DELAY_BETWEEN_LED_CHANGES_TICKS);

  ShimFactoryTest_sendReport(" - All LEDs on\r\n");
  Board_ledOn(LED_ALL);
  __delay_cycles(DELAY_BETWEEN_LED_CHANGES_TICKS);

  BlinkTimerStart();

  //Reset Watchdog timer
  WDTCTL = WDTPW + WDTSSEL__ACLK + WDTCNTCL + WDTIS_4;
}

void sd_card_test(void)
{
  uint8_t sdWasOff = 0;

  ShimFactoryTest_sendReport("SD Card:\r\n");
  if (P4IN & BIT1)
  {
    ShimFactoryTest_sendReport(" - FAIL: SD Card not detected\r\n");
  }
  else
  {
    ShimFactoryTest_sendReport(" - PASS: SD card detected\r\n");

    if (P2IN & BIT3)
    {
      ShimFactoryTest_sendReport(
          " - FAIL: Shimmer is docked so SD card test can not be "
          "performed\r\n");
    }
    else
    {
      //ReadWriteSDTest();

      if (!shimmerStatus.sdBadFile)
      {
        if (!shimmerStatus.sdPowerOn)
        {
          sdWasOff = 1;
          Board_setSdPower(1);
        }

        ShimSd_test1();

        if (sdWasOff)
        {
          Board_setSdPower(0);
        }
      }
      sprintf(buffer, " - %s: SD card read/write test\r\n",
          shimmerStatus.sdBadFile ? "FAIL" : "PASS");
      ShimFactoryTest_sendReport(buffer);
    }
  }
}

void bt_module_test(void)
{
  //if (stat.isBtPoweredOn)
  //{
  ShimFactoryTest_sendReport("BT Module:\r\n");

  ShimFactoryTest_sendReport(" - MAC ID: ");
  memcpy(&buffer[0], ShimBt_macIdStrPtrGet(), 12);
  sprintf(&buffer[12], "\r\n");
  ShimFactoryTest_sendReport(buffer);

  sprintf(buffer, " - %s", ShimBt_getBtVerStrPtr());
  ShimFactoryTest_sendReport(buffer);

  if (strstr(buffer, "RN4678") != NULL && strstr(buffer, "V1.23") == NULL)
  {
    ShimFactoryTest_sendReport(" - FAIL: incorrect BT firmware version\r\n");
  }
  else
  {
    ShimFactoryTest_sendReport(" - PASS\r\n");
  }
  //}
  //else
  //{
  //    ShimFactoryTest_sendReport(" - FAIL\r\n");
  //}
  //return stat.isBtPoweredOn;

  gEepromBtSettings *eepromBtSetting = ShimEeprom_getRadioDetails();
  ShimFactoryTest_sendReport(" - Counts:\r\n");
  sprintf(buffer, "   - BT data-rate test blockages = %d\r\n", eepromBtSetting->btCntDataRateTestBlockage);
  ShimFactoryTest_sendReport(buffer);
  sprintf(buffer, "   - BT disconnects while streaming = %d\r\n", eepromBtSetting->btCntDisconnectWhileStreaming);
  ShimFactoryTest_sendReport(buffer);
  sprintf(buffer, "   - BT RTS Lockups = %d\r\n", eepromBtSetting->btCntRtsLockup);
  ShimFactoryTest_sendReport(buffer);
  sprintf(buffer, "   - BT unsolicited reboots = %d\r\n", eepromBtSetting->btCntUnsolicitedReboot);
  ShimFactoryTest_sendReport(buffer);
}

void I2C_test(void)
{
  ShimFactoryTest_sendReport("I2C:\r\n");

  I2C_start(1);

  uint8_t eeprom_result = CAT24C16_test();
  sprintf(buffer, " - %s: CAT24C16\r\n", eeprom_result ? "FAIL" : "PASS");
  ShimFactoryTest_sendReport(buffer);

  i2cSlaveDiscover();

  //There is no self test feature implemented yet in S3 but we can at least indicate what chips are detected.
  if (i2cSlavePresent(LSM303DHLC_ACCEL_ADDR))
  {
    ShimFactoryTest_sendReport(
        " - LSM303DHLC detected (self-test not implemented yet)\r\n");
  }
  if (i2cSlavePresent(LSM303AHTR_ACCEL_ADDR))
  {
    ShimFactoryTest_sendReport(
        " - LSM303AH detected (self-test not implemented yet)\r\n");
  }
  if (i2cSlavePresent(ICM20948_ADDR))
  {
    ShimFactoryTest_sendReport(
        " - ICM20948 detected (self-test not implemented yet)\r\n");
  }
  if (i2cSlavePresent(MPU9150_ADDR))
  {
    ShimFactoryTest_sendReport(
        " - MPU9x50 detected (self-test not implemented yet)\r\n");
  }
  if (i2cSlavePresent(BMP180_ADDR))
  {
    ShimFactoryTest_sendReport(
        " - BMP180 detected (self-test not implemented yet)\r\n");
  }
  if (i2cSlavePresent(BMP280_ADDR))
  {
    ShimFactoryTest_sendReport(
        " - BMP280 detected (self-test not implemented yet)\r\n");
  }

  I2C_stop(1);
}

void SPI_test(void)
{
  uint8_t ads1292RTestResult = 0;

  ShimFactoryTest_sendReport("SPI:\r\n");

  if (ShimBrd_isAds1292Present())
  {
    if (ShimFactoryTest_getTarget() == PRINT_TO_DOCK_UART)
    {
      ShimFactoryTest_sendReport(
          "- FAIL: ADS1292R test will not work from dock\r\n");
    }
    else
    {
      if (shimmerStatus.docked)
      {
        DockUart_deinit();
      }

      EXG_init();

      ads1292RTestResult = EXG_self_test();

      sprintf(buffer, " - %s: ADS1292R Chip1 detect\r\n",
          (ads1292RTestResult & 0x01) ? "FAIL" : "PASS");
      ShimFactoryTest_sendReport(buffer);

      sprintf(buffer, " - %s: ADS1292R Chip2 detect\r\n",
          (ads1292RTestResult & 0x02) ? "FAIL" : "PASS");
      ShimFactoryTest_sendReport(buffer);

      EXG_powerOff();

      if (shimmerStatus.docked)
      {
        DockUart_init();
      }
    }
  }
  else
  {
    ShimFactoryTest_sendReport(
        "- ADS1292R test not applicable for this model\r\n");
  }
}

void ShimFactoryTest_sendReportImpl(const char *str, factory_test_target_t factoryTestTarget)
{
  switch (factoryTestTarget)
  {
    case PRINT_TO_DOCK_UART:
      DockUart_writeBlocking((uint8_t *) str, strlen(str));
      break;
    case PRINT_TO_BT_UART:
      ShimBt_writeToTxBufAndSend((uint8_t *) str, strlen(str), SHIMMER_CMD);
      while (ShimBt_getUsedSpaceInBtTxBuf() > 0)
        ;
      break;
    default:
      break;
  }
}
