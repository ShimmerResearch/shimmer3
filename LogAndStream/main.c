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
 * @author Weibo
 * @date May, 2015
 *
 * @modifed Sam O'Mahony
 * @date January, 2018
 *
 * @modifed Mark Nolan, Ramesh Chhetri
 * @date March, 2022
 */

/***********************************************************************************
 Data Buffer Format:
      Packet Type |TimeStamp|Achan1|Achan2| ... |AchanX|Dchan1|Dchan2| ... |DchanX|
 Byte:     0      |   1-3   |Achan1|Achan2| ... |AchanX|Dchan1|Dchan2| ... |DchanX|

 Log file Format:
       SD Header|TimeStamp1|Achan1 data1| ... |DchanX data1|TimeStamp2|Achan2 data2|...
 Byte:  0-255   |  256-258 |   259-260  | ... |            |          | |...

 ***********************************************************************************/

#include <ctype.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "msp430.h"

#include "adc.h"
#include "i2c.h"
#include "spi.h"

#include "Shimmer_Driver/5xx_HAL/hal_FactoryTest.h"
#include "Shimmer_Driver/shimmer_driver_include.h"
#include "log_and_stream_globals.h"
#include "shimmer_btsd.h"

void Init(void);
void handleIfDockedStateOnBoot(void);
void sleepWhenNoTask(void);
void checkSetupDock(void);
void ProcessHwRevision(void);
void InitialiseBt(void);
void InitialiseBtAfterBoot(void);
void stopSensingWrapup(void);
void BlinkTimerStart(void);
void BlinkTimerStop(void);
void SampleTimerStart(void);
void SampleTimerStop(void);
char *HAL_GetUID(void);
void DockSdPowerCycle();
void SetupDock(void);
uint8_t CheckSdInslot(void);
uint8_t checkIfBattReadNeeded(void);
void BtStartDone();
void BtStart(void);
void BtStop(uint8_t isCalledFromMain);
void TB0Start();
void TB0Stop();
void ChargeStatusTimerStart(void);
void ChargeStatusTimerStop(void);
void ClkAssignment();
uint16_t FreqProd(uint16_t samplingFreq);
float samplingClockFreqGet(void);
void setSamplingClkSource(float samplingClock);
void triggerShimmerErrorState(void);

void eepromRead(uint16_t dataAddr, uint16_t dataSize, uint8_t *dataBuf);
void eepromWrite(uint16_t dataAddr, uint16_t dataSize, uint8_t *dataBuf);
void eepromReadWrite(uint16_t dataAddr, uint16_t dataSize, uint8_t *dataBuf, enum EEPROM_RW eepromRW);
void updateBtDetailsInEeprom(void);

#define TIMEOUT_100_MS    (3277)

/* should be 0 */
#define PRESS2UNDOCK      0
#define IS_SUPPORTED_TCXO 0

uint64_t buttonPressTs, buttonReleaseTs, buttonLastReleaseTs;

uint8_t watchDogWasOnDuringBtStart;

FRESULT ff_result;

/*battery evaluation vars*/
uint32_t battLastTs64;
float clockFreq;
uint16_t clk_30, clk_45, clk_55, clk_64, clk_75, clk_133, clk_90, clk_120,
    clk_135, clk_225, clk_255, clk_432, clk_1000, clk_2500, clk_90_45, clk_90_64,
    clk_90_75, clk_133_90, clk_135_90, clk_225_90, clk_255_90, clk_432_90;

uint8_t all0xff[7U], all0x00[7U];

char *dierecord;

/* variables used for delayed undock-start */
uint8_t undockEvent;
uint64_t time_newUnDockEvent;

//bluetooth variables
uint8_t rnx_radio_eeprom[CAT24C16_PAGE_SIZE];

void main(void)
{
  Init();

  while (1)
  {
    ShimTask_manage();
  }
}

void Init(void)
{
  //Disable the GPIO power-on default high-impedance mode to activate
  //previously configured port settings
  //PM5CTL0 &= ~LOCKLPM5;

  LogAndStream_init();
  shimmerStatus.initialising = 1; /* led flag, in initialisation period */

  Board_init();

  setBootStage(BOOT_STAGE_START);

  ShimBrd_setHwId(DEVICE_VER);

  //Set Vcore to accommodate for max. allowed system speed
  SetVCore(PMMCOREV_3);

  //Start 32.768kHz XTAL as ACLK
  LFXT_Start(XT1DRIVE_0);

  //Start 24MHz XTAL as MCLK and SMCLK
  XT2_Start(XT2DRIVE_2); //XT2DRIVE_2 or XTDRIVE_3 for 24MHz (userguide section 5.4.7)
  UCSCTL4 |= SELS_5 + SELM_5; //SMCLK=MCLK=XT2

  SFRIFG1 = 0;    //clear interrupt flag register
  SFRIE1 |= OFIE; //enable oscillator fault interrupt enable

  buttonLastReleaseTs = 0;

  battLastTs64 = 0;

  setSamplingClkSource((float) MSP430_CLOCK);

  I2C_varsInit();
  SPI_varsInit();
  ADC_varsInit();

  setBmpInUse(BMP180_IN_USE);
  ShimBrd_setWrAccelAndMagInUse(WR_ACCEL_AND_MAG_NONE_IN_USE);
  ShimBrd_setGyroInUse(GYRO_NONE_IN_USE);
  ShimBrd_setEepromIsPresent(0);

  /* variables used for delayed undock-start */
  undockEvent = 0;
  time_newUnDockEvent = 0;

  memset(all0xff, 0xff, sizeof(all0xff) / sizeof(all0xff[0]));
  memset(all0x00, 0x00, sizeof(all0x00) / sizeof(all0x00[0]));

  /* Globally enable interrupts */
  _enable_interrupts();

  BlinkTimerStart();

  /* Dock uart setup */
  UCA0_isrInit();
  UART_init(ShimDock_rxCallback);

  shimmerStatus.sdInserted = (bool) (!(P4IN & BIT1));

  handleIfDockedStateOnBoot();

  P2IFG &= ~BIT3; //clear flag
  P2IE |= BIT3;   //enable interrupt

  //RwcCheck();
  RTC_init(0);

  //enable switch1 interrupt
  Button_init();
  Button_interruptEnable();

  dierecord = (char *) 0x01A0A;

  setBootStage(BOOT_STAGE_I2C);
  I2C_start(1);
  detectI2cSlaves();
  I2C_stop(1);
  /* I2C slave discover leaves the I2C bus configuration in the wrong state so
   * just reinitialising here. */
  I2C_start(0);
  loadBmpCalibration();
  I2C_stop(0);
  ShimSdHead_saveBmpCalibrationToSdHeader();

  /* Read Daughter card ID*/
  eepromRead(DAUGHT_CARD_ID, CAT24C16_PAGE_SIZE, ShimBrd_getDaughtCardIdPtr());
  ProcessHwRevision();

  ShimSens_checkOnDefault();

  setBootStage(BOOT_STAGE_BLUETOOTH);
  InitialiseBt();

  setBootStage(BOOT_STAGE_CONFIGURATION);
  /* Calibration needs to be loaded after the chips have been detected in
   * order to know which default calib to set for attached chips.
   * It also needs to be loaded after the BT is initialised so that the
   * MAC ID can be used for default Shimmer name and calibration file names.*/
  ShimConfig_loadSensorConfigAndCalib();

  UART_setState(shimmerStatus.docked);

  RwcCheck();

  SetupDock();

  /* Take initial measurement to update LED state */
  manageReadBatt(1);

  /* Initialise Watchdog status timer */
  ChargeStatusTimerStart();

  shimmerStatus.initialising = 0;
  setBootStage(BOOT_STAGE_END);
}

void handleIfDockedStateOnBoot(void)
{
#if TEST_UNDOCKED
  if (0)
  {
#else
  if (BOARD_IS_DOCKED)
  {
#endif
    ShimBatt_setBatteryInterval(BATT_INTERVAL_DOCKED);
    P2IES |= BIT3; //look for falling edge
    shimmerStatus.docked = 1;
    shimmerStatus.sdlogReady = 0;
    if (CheckSdInslot())
    {
      DockSdPowerCycle();
    }
  }
  else
  {
    ShimBatt_setBatteryInterval(BATT_INTERVAL_UNDOCKED);
    P2IES &= ~BIT3; //look for rising edge
    shimmerStatus.docked = 0;
    P6OUT |= BIT0; //DETECT_N set to high
    if (CheckSdInslot())
    {
      shimmerStatus.sdlogReady = 1;
    }
    else
    {
      shimmerStatus.sdlogReady = 0;
    }
  }
}

void sleepWhenNoTask(void)
{
  __bis_SR_register(LPM3_bits + GIE); /* ACLK remains active */
}

void checkSetupDock(void)
{
  if (!shimmerStatus.configuring && !sensing.inSdWr
      && (BOARD_IS_DOCKED || ((RTC_get64() - time_newUnDockEvent) > TIMEOUT_100_MS)))
  {
    if (shimmerStatus.docked != (BOARD_IS_DOCKED >> 3))
    {
      shimmerStatus.docked = (BOARD_IS_DOCKED >> 3);
      SetupDock();
    }

    if (shimmerStatus.docked != (BOARD_IS_DOCKED >> 3))
    {
      ShimTask_set(TASK_SETUP_DOCK);
    }
    undockEvent = 0;
  }
  else
  {
    ShimTask_set(TASK_SETUP_DOCK);
  }

  RwcCheck();
}

void ProcessHwRevision(void)
{
  shimmer_expansion_brd *expBrd = ShimBrd_getDaughtCardId();

  ShimBrd_parseDaughterCardId();

  if (ShimBrd_isEepromIsPresent())
  {
    //Some board batches don't have the LSM303AHTR placed, in these
    //cases, the ICM-20948's channels are used instead
    if (ShimBrd_isSubstitutionNeededForWrAccel())
    {
      ShimBrd_setWrAccelAndMagInUse(WR_ACCEL_AND_MAG_ICM20948_IN_USE);
    }
    else
    {
      //If the hardware is any board other then the above, overwrite the
      //special rev with 171 (used as a failsafe) to let Consensys know
      //what sensors are on-board
      if (ShimBrd_are2ndGenSensorsPresentAndUnknownBoard())
      {
        expBrd->exp_brd_minor = 171;
      }
    }
  }
  else
  {
    //The EEPROM is not present on the SR31-6-0 so the firmware needs mock
    //the version number so that Consensys knows which sensors are on-board
    if (ShimBrd_are2ndGenImuSensorsPresent())
    {
      expBrd->exp_brd_id = SHIMMER3_IMU;
      expBrd->exp_brd_major = 6;
      expBrd->exp_brd_minor = 0;
    }
  }

  Board_initForRevision();
}

void InitialiseBt(void)
{
  //This is the start of all BT initialisation
  /* The RN4678 operational mode need to be set before BT_init() is called so
   * that the pins are set correctly prior to communication with the module */
  if (ShimBrd_isRn4678PresentAndCmdModeSupport())
  {
    setRn4678OperationalMode(RN4678_OP_MODE_APPLICATION);
  }
  else
  {
    setRn4678OperationalMode(RN4678_OP_MODE_NOT_USED);
  }

  ShimBt_btCommsProtocolInit();
  ShimSdSync_init(InitialiseBtAfterBoot, BtStop);
  BT_init();
  BT_rn4xDisableRemoteConfig(1);
  BT_setRadioMode(SLAVE_MODE); //slave mode for center&node
  BT_setGetMacAddress(1);
  BT_setGetVersion(1);

#if BT_ENABLE_BLE_FOR_LOGANDSTREAM_AND_RN4678
  /* BLE isn't compatible with the standard "1234" passkey that Shimmer3 has
   * always used for Classic Bluetooth so we're just disabling the passkey
   * altogether here */
  BT_setAuthentication(2U);
  setBleDeviceInformation(
      ShimBrd_getDaughtCardIdStrPtr(), FW_VER_MAJOR, FW_VER_MINOR, FW_VER_REL);
#endif

  BT_setUpdateBaudDuringBoot(1);
  //BT_useSpecificAdvertisingName(1U);

  /* Read previous baud rate from the EEPROM if it is present */
  uint8_t initialBaudRate = BAUD_115200;
  memset(rnx_radio_eeprom, 0xFF, sizeof(rnx_radio_eeprom) / sizeof(rnx_radio_eeprom[0]));
  if (ShimBrd_isEepromIsPresent())
  {
    //Read Bluetooth configuration parameters from EEPROM
    /* Variable to help initialise BT radio RN42/4678 type information to EEPROM */
    eepromRead(RNX_TYPE_EEPROM_ADDRESS, CAT24C16_PAGE_SIZE, rnx_radio_eeprom);

    if (rnx_radio_eeprom[RN4678_BAUD_RATE_IDX] <= BAUD_1000000)
    {
      initialBaudRate = rnx_radio_eeprom[RN4678_BAUD_RATE_IDX];
    }
  }

  uint8_t reset_cnt = 50U; //50 * 100ms = 5s per baud rate attempt
  uint8_t failCount = 0U;
  uint8_t baudIndex = 0;
  uint8_t baudsTried[BAUD_1000000 + 1U] = { 0 };

  /* Try the inital baud rate first */
  baudsTried[initialBaudRate] = 1U;
  setBtBaudRateToUse(initialBaudRate);
  BtStart();

  /* Try the baud that's stored in the EEPROM firstly, if that fails try
   * 115200, 1000000 or 460800 and then all other bauds. If they all fail, soft-reset */
  while (!shimmerStatus.btPowerOn)
  {
    _delay_cycles(2400000); //100ms

    if (!(reset_cnt--))
    {
      failCount++;

      if (failCount == sizeof(baudsTried))
      {
        //// software POR reset
        //PMMCTL0 = PMMPW + PMMSWPOR + (PMMCTL0 & 0x0003);

        setBootStage(BOOT_STAGE_BLUETOOTH_FAILURE);
        while (1)
        {
          __bis_SR_register(LPM3_bits + GIE); /* ACLK remains active */
        }
      }

      /* Baud rate is likely 115200, 1000000 or 460800 so try them first */
      if (baudsTried[BAUD_115200] != 1U)
      {
        initialBaudRate = BAUD_115200;
      }
      else if (baudsTried[BAUD_460800] != 1U)
      {
        initialBaudRate = BAUD_460800;
      }
      else if (baudsTried[BAUD_1000000] != 1U)
      {
        initialBaudRate = BAUD_1000000;
      }
      else
      {
        for (baudIndex = 0; baudIndex < sizeof(baudsTried); baudIndex++)
        {
          if (baudsTried[baudIndex] != 1U)
          {
            initialBaudRate = baudIndex;
            break;
          }
        }
      }

      BT_rst_MessageProgress();
      ShimBt_clearBtTxBuf(1U);

      baudsTried[initialBaudRate] = 1U;
      setBtBaudRateToUse(initialBaudRate);
      BtStart();

      reset_cnt = 50U;
    }
  }

  if (ShimBrd_isEepromIsPresent()
      && (rnx_radio_eeprom[RNX_RADIO_TYPE_IDX] != getBtHwVersion()
          || rnx_radio_eeprom[RN4678_BAUD_RATE_IDX] == 0xFF
          || (isBtDeviceRn4678() && rnx_radio_eeprom[RN4678_BAUD_RATE_IDX] != getCurrentBtBaudRate())
          || (isBtDeviceRn41orRN42() && rnx_radio_eeprom[RN4678_BAUD_RATE_IDX] != BAUD_115200)))
  {
    updateBtDetailsInEeprom();
  }

  if (ShimConfig_getStoredConfig()->btCommsBaudRate != getCurrentBtBaudRate())
  {
    ShimConfig_getStoredConfig()->btCommsBaudRate = getCurrentBtBaudRate();
    InfoMem_write(NV_BT_COMMS_BAUD_RATE,
        &ShimConfig_getStoredConfig()->rawBytes[NV_BT_COMMS_BAUD_RATE], 1);

    ShimTask_set(TASK_SDLOG_CFG_UPDATE);
  }
}

void InitialiseBtAfterBoot(void)
{
  BT_init();
  BT_rn4xDisableRemoteConfig(1);
  BT_setUpdateBaudDuringBoot(1);
  BtStart();
}

void stopSensingWrapup(void)
{
  ShimTask_clear(TASK_SAMPLE_BMPX80_PRESS);
  ShimTask_clear(TASK_SAMPLE_MPU9150_MAG);
}

//Switch SW1, BT_RTS and BT connect/disconnect
uint64_t button_two_release_td64, button_press_release_td64;
#pragma vector = PORT1_VECTOR

__interrupt void Port1_ISR(void)
{
  //Context save interrupt flag before calling interrupt vector.
  //Reading interrupt vector generator will automatically clear IFG flag
  //buttonsPressed = PAIFG & BUTTON_ALL;

  switch (__even_in_range(P1IV, P1IV_P1IFG7))
  {
    //BT Connect/Disconnect
    case P1IV_P1IFG0:
      //PIO2_CONNECT interrupt (Pin 19 on RN42)
      //High: when connected
      //Low: otherwise

      //RADIO_STATUS interrupt (P1_5 on RN4678)
      //High: Powered On and not connected
      //Low: Connected to peer device

      if (((P1IN & BIT0) & isBtDeviceRn41orRN42())
          || ((!(P1IN & BIT0)) & isBtDeviceRn4678()))
      { //BT is connected
        /* BLE relies on this pin to know when the UART service is open/not */
        if (!areBtStatusStringsEnabled() || isRn4678ConnectionBle())
        {
          ShimBt_handleBtRfCommStateChange(TRUE);
        }
      }
      else
      { //BT is disconnected
        if (!areBtStatusStringsEnabled() || shimmerStatus.btConnected)
        {
          /* Fail-safe incase the FW has missed the BT DISCONNECT/RFCOM_CLOSE status strings */
          ShimBt_handleBtRfCommStateChange(FALSE);
        }
      }
      __bic_SR_register_on_exit(LPM3_bits);
      break;

      //BT RTS
    case P1IV_P1IFG3:
      if (isBtModuleOverflowPinHigh())
      {
        P1IES |= BIT3; //look for falling edge
        BT_rtsInterrupt(1);
      }
      else
      {
        P1IES &= ~BIT3;     //look for rising edge
        BT_rtsInterrupt(0); //when 0, can call sendNextChar();
      }
      break;

      //ExG chip2 data ready
    case P1IV_P1IFG4:
      EXG_dataReadyChip2();
      break;

      //BUTTON_SW1
    case P1IV_P1IFG6:
      if (!BOARD_IS_BTN_RELEASED)
      {                 //button pressed
        P1IES &= ~BIT6; //select rising edge trigger, wait for release
        shimmerStatus.buttonPressed = 1;
        buttonPressTs = RTC_get64();

        Board_ledOn(LED_LWR_GREEN);
      }
      else
      {                //button released
        P1IES |= BIT6; //select fall edge trigger, wait for press
        shimmerStatus.buttonPressed = 0;
        buttonReleaseTs = RTC_get64();
        button_press_release_td64 = buttonReleaseTs - buttonPressTs;
        button_two_release_td64 = buttonReleaseTs - buttonLastReleaseTs;
        if (button_press_release_td64 >= 163840)
        { //long button press: 5s
        }
        else if ((button_two_release_td64 > 16384) && !shimmerStatus.configuring
            && !shimmerStatus.btConnected)
        { //&& (button_press_release_td64>327)
          buttonLastReleaseTs = buttonReleaseTs;
#if PRESS2UNDOCK
          if (shimmerStatus.docked)
          {
            shimmerStatus.docked = 0;
          }
          else
          {
            shimmerStatus.docked = 1;
          }
          ShimTask_set(TASK_SETUP_DOCK);
          if (!shimmerStatus.sensing)
          {
            __bic_SR_register_on_exit(LPM3_bits);
          }
#else
          if (ShimConfig_getStoredConfig()->userButtonEnable)
          {
            //toggles sensing and refresh BT timers (for the centre)
            if (shimmerStatus.sensing)
            {
              shimmerStatus.sdlogCmd = SD_LOG_CMD_STATE_STOP;
              ShimTask_setStopSensing();
              ShimBt_instreamStatusRespPendingSet(1);
            }
            else
            {
              if (shimmerStatus.sdlogReady && !shimmerStatus.sdBadFile)
              {
                shimmerStatus.sdlogCmd = SD_LOG_CMD_STATE_START;
                ShimTask_setStartSensing();
              }

              shimmerStatus.sensing = 1;
              ShimBt_instreamStatusRespSend(); //only send cmd, not starting yet till START_BTSD_CMD received
              shimmerStatus.sensing = 0;
              __bic_SR_register_on_exit(LPM3_bits);
            }
          }
#endif
        }
        else
        {
          _NOP();
        }
      }
      break;
    default:
      break;
  }
}

#pragma vector = PORT2_VECTOR

__interrupt void Port2_ISR(void)
{
  //Context save interrupt flag before calling interrupt vector.
  //Reading interrupt vector generator will automatically clear IFG flag
  //buttonsPressed = PAIFG & BUTTON_ALL;

  switch (__even_in_range(P2IV, P2IV_P2IFG7))
  {
    //ExG chip1 data ready
    case P2IV_P2IFG0:
      EXG_dataReadyChip1();
      if (adsClockTiedGet())
      {
        EXG_dataReadyChip2();
      }
      break;

      //EXP_DETECT_N
    case P2IV_P2IFG1:
      //TODO: Debounce this
      //see slaa513 for example using multiple time bases on a single timer module
      if (P2IN & BIT1)
      {
        //card not inserted
        P2IES |= BIT1; //look for falling edge
      }
      else
      {
        //card inserted
        P2IES &= ~BIT1; //look for rising edge
      }
      break;

      //dock_detect_N
    case P2IV_P2IFG3:
      ShimTask_set(TASK_SETUP_DOCK);
      /* Reset to battery "charging"/"checking" LED indication on docking change */
      ShimBatt_resetBatteryChargingStatus();
      if (!undockEvent)
      {
        if (!BOARD_IS_DOCKED) //undocked
        {
          undockEvent = 1;
          ShimBatt_setBattCritical(0);
          time_newUnDockEvent = RTC_get64();
        }
        //see slaa513 for example using multiple time bases on a single timer module
        if (!shimmerStatus.sensing)
        {
          __bic_SR_register_on_exit(LPM3_bits);
        }
      }
      if (BOARD_IS_DOCKED)
      {
        P2IES |= BIT3; //look for falling edge
        shimmerStatus.sdlogReady = 0;
      }
      else
      {
        P2IES &= ~BIT3; //look for rising edge
        if (CheckSdInslot() && !shimmerStatus.sdBadFile)
        {
          shimmerStatus.sdlogReady = 1;
        }
        else
        {
          shimmerStatus.sdlogReady = 0;
        }
      }
      break;
      //Default case
    default:
      break;
  }
}

/**
 *** Charge Status Timer for WatchDog
 **/
void ChargeStatusTimerStart(void)
{
  TB0CCR3 = GetTB0() + 3277;
  TB0CCTL3 = CCIE;
  /*
   * Set up the WatchDog timer Control Register:
   *
   * WDTPW - Watchdog timer password
   * WDTSSEL__ACLK - clock source
   * WDTCNTCL - Timer clear
   * WDTIS_4 - timer interval select - 1s at 32768Hz
   *
   */
  WDTCTL = WDTPW + WDTSSEL__ACLK + WDTCNTCL + WDTIS_4;
}

void ChargeStatusTimerStop(void)
{
  TB0CCTL3 = 0;
  Board_ledOff(LED_ALL);
}

//Blink Timer
//USING TB0 with CCR1
void BlinkTimerStart(void)
{
  shimmerStatus.timerBlinkEnabled = 1;
  TB0Start();
  TB0CCTL3 = CCIE;
  //clk_1000 = 100.0 ms = 0.1s
  TB0CCR3 = GetTB0() + clk_1000;
}

void BlinkTimerStop(void)
{
  shimmerStatus.timerBlinkEnabled = 0;
  TB0CCTL3 &= ~CCIE;
}

#pragma vector = TIMER0_B1_VECTOR

__interrupt void TIMER0_B1_ISR(void)
{
  switch (__even_in_range(TB0IV, 14))
  {
    case 0:
      break; //No interrupt
    case 2:  //TB0CCR1
      //MPU9150 mag
      TB0CCR1 = GetTB0() + ShimConfig_getStoredConfig()->samplingRateTicks;
      if (ShimTask_set(TASK_SAMPLE_MPU9150_MAG))
      {
        __bic_SR_register_on_exit(LPM3_bits);
      }
      break;
    case 4: //TB0CCR2
      //Bmp180 press
      TB0CCR2 = GetTB0() + ShimConfig_getStoredConfig()->samplingRateTicks;
      if (ShimTask_set(TASK_SAMPLE_BMPX80_PRESS))
      {
        __bic_SR_register_on_exit(LPM3_bits);
      }
      break;
    case 6: //TB0CCR3
      if (!(WDTCTL & WDTHOLD))
      {
        //Reset Watchdog timer
        WDTCTL = WDTPW + WDTSSEL__ACLK + WDTCNTCL + WDTIS_4;
      }

      //for LED blink usage details, please check shimmer user manual
      //clk_1000 = 100.0 ms = 0.1s
      TB0CCR3 += clk_1000;

      LogAndStream_blinkTimerCommon();

      if (!shimmerStatus.initialising && checkIfBattReadNeeded())
      {
        __bic_SR_register_on_exit(LPM3_bits);
      }

      break;
    case 8:
      break; //TB0CCR4
    case 10:
      break; //reserved
    case 12:
      break; //reserved
    case 14:
      break; //TBIFG overflow handler
  }
}

uint8_t checkIfBattReadNeeded(void)
{
  uint64_t batt_td, batt_my_local_time_64;
  batt_my_local_time_64 = RTC_get64();
  batt_td = batt_my_local_time_64 - battLastTs64;

  if ((batt_td > ShimBatt_getBatteryIntervalTicks()))
  { //10 mins = 19660800
    battLastTs64 = batt_my_local_time_64;
    if (!shimmerStatus.sensing && ShimTask_set(TASK_BATT_READ))
    {
      return 1;
    }
  }
  return 0;
}

//BT start Timer
void BtStartDone()
{
  shimmerStatus.btPowerOn = 1;
  if (!shimmerStatus.sensing)
  {
    shimmerStatus.configuring = 0;
  }
}

void BtStart(void)
{
  if (!shimmerStatus.btPowerOn)
  {
    /* Long delays starting BT, need to disable WDT */
    if (!(WDTCTL & WDTHOLD))
    {
      watchDogWasOnDuringBtStart = 1U;
      WDTCTL = WDTPW | WDTHOLD;
    }
    else
    {
      watchDogWasOnDuringBtStart = 0;
    }

    BT_startDone_cb(BtStartDone);

    if (!shimmerStatus.sensing)
    {
      shimmerStatus.configuring = 1;
    }
    ShimBt_resetBtRxBuff();

    shimmerStatus.btInSyncMode = shimmerStatus.sdSyncEnabled;

    BT_start();
  }
}

void BtStop(uint8_t isCalledFromMain)
{
  ShimBt_clearBtTxBuf(isCalledFromMain);

  ShimTask_clear(TASK_RCNODER10);

  DMA2_disable(); //dma2 for bt disabled

  updateBtConnectionStatusInterruptDirection();
  shimmerStatus.btConnected = 0;
  shimmerStatus.btPowerOn = 0;
  shimmerStatus.btInSyncMode = 0;
  BT_disable();
  BT_rst_MessageProgress();

  if (watchDogWasOnDuringBtStart)
  {
    //Reset Watchdog timer
    WDTCTL = WDTPW + WDTSSEL__ACLK + WDTCNTCL + WDTIS_4;
  }
}

//Sample Timer
void SampleTimerStart(void)
{
  uint16_t val_tb0 = GetTB0();
  uint16_t baseClockOffset = val_tb0 + ShimConfig_getStoredConfig()->samplingRateTicks;
  uint8_t bmpX80Precision = ShimConfig_getStoredConfig()->pressureOversamplingRatioLsb;

  if (isPreSampleMpuMagEn() || isPreSampleMpuPressEn())
  {
    if (isPreSampleMpuPressEn())
    {
      uint16_t bmpX80SamplingTimeInTicks = getBmpX80SamplingTimeInTicks();
      uint16_t bmpX80SamplingTimeDiffFrom9msInTicks
          = getBmpX80SamplingTimeDiffFrom9msInTicks();

      //When max BMPX80 sampling time is less than 9ms
      if (bmpX80Precision == 0 || (bmpX80Precision == 1 && isBmp180InUse()))
      {
        if (isPreSampleMpuMagEn())
        {
          TB0CCR0 = baseClockOffset + clk_90; //9ms
          TB0CCR1 = baseClockOffset;
          TB0CCR2 = bmpX80SamplingTimeDiffFrom9msInTicks;
          TB0CCTL1 = CCIE;
        }
        else
        {
          TB0CCR0 = baseClockOffset + bmpX80SamplingTimeInTicks;
          TB0CCR2 = baseClockOffset;
          TB0CCTL1 = 0;
        }
        TB0CCTL2 = CCIE;
      }
      //When max BMPX80 sampling time is greater than 9ms
      else if ((bmpX80Precision == 1 && !isBmp180InUse())
          || bmpX80Precision == 2 || bmpX80Precision == 3)
      {
        TB0CCR0 = baseClockOffset + bmpX80SamplingTimeInTicks;
        TB0CCR2 = baseClockOffset;
        TB0CCTL2 = CCIE;

        if (isPreSampleMpuMagEn())
        {
          TB0CCR1 = baseClockOffset + bmpX80SamplingTimeDiffFrom9msInTicks;
          TB0CCTL1 = CCIE;
        }
        else
        {
          TB0CCTL1 = 0;
        }
      }
    }
    else
    {
      TB0CCR0 = baseClockOffset + clk_90; //9ms
      TB0CCR1 = baseClockOffset;
      TB0CCTL1 = CCIE;
      TB0CCTL2 = 0;
    }
  }
  else
  {
    TB0CCTL1 = 0;
    TB0CCTL2 = 0;
    TB0CCR0 = baseClockOffset;
  }
  TB0CCTL0 = CCIE;
  shimmerStatus.timerSamplingEnabled = 1;
  TB0Start();
}

void SampleTimerStop(void)
{
  shimmerStatus.timerSamplingEnabled = 0;
  TB0Stop();
  TB0CCTL0 &= ~CCIE;
  TB0CCTL1 &= ~CCIE;
  TB0CCTL2 &= ~CCIE;
}

#pragma vector = TIMER0_B0_VECTOR

__interrupt void TIMER0_B0_ISR(void)
{
  uint16_t timer_b0 = GetTB0();
  TB0CCR0 = timer_b0 + ShimConfig_getStoredConfig()->samplingRateTicks;

  //start ADC conversion
  if (sensing.nbrAdcChans)
  {
    ShimSens_saveTimestampToPacket();
    DMA0_enable();
    ADC_startConversion();
  }
  else
  {
    //no analog channels, so go straight to digital
    ShimSens_gatherData();
    __bic_SR_register_on_exit(LPM3_bits);
  }
}

char *HAL_GetUID(void)
{
  return dierecord;
}

void DockSdPowerCycle()
{
  _delay_cycles(2880000);
  SdPowerOff(); //SW_FLASH set low, SdPowerOff();

  P5SEL &= ~(BIT4 + BIT5);
  P5OUT &= ~(BIT4 + BIT5); //FLASH_SOMI and FLASH_SCLK set low
  P5DIR |= BIT4;           //FLASH_SOMI set as output
  P3SEL &= ~BIT7;
  P3OUT &= ~BIT7;          //FLASH_SIMO set low
  P4OUT &= ~BIT0;          //FLASH_CS_N set low
  P6OUT &= ~(BIT6 + BIT7); //ADC6_FLASHDAT2 and ADC7_FLASHDAT1 set low
  P6DIR |= BIT6 + BIT7;    //ADC6_FLASHDAT2 and ADC7_FLASHDAT1 set as output

  //60ms as taken from TinyOS driver (SDP.nc powerCycle() function)
  _delay_cycles(2880000);

  P5DIR &= ~(BIT4 + BIT5); //FLASH_SOMI and FLASH_SCLK set as input
  P3DIR &= ~BIT7;          //FLASH_SIMO set as input
  P4DIR &= ~BIT0;          //FLASH_CS_N set as input
  P6DIR &= ~(BIT6 + BIT7); //ADC6_FLASHDAT2 and ADC7_FLASHDAT1 set as input

  SdPowerOn();            //SW_FLASH set high, SdPowerOn();
  _delay_cycles(1200000); //give SD card time to power back up
  P6OUT &= ~BIT0;         //DETECT_N set low
}

void SetupDock()
{
  shimmerStatus.configuring = 1;
  if (shimmerStatus.docked)
  {
    ShimBatt_setBatteryInterval(BATT_INTERVAL_DOCKED);
    ShimBatt_resetBatteryCriticalCount();

    shimmerStatus.sdlogCmd = 0;
    shimmerStatus.sdlogReady = 0;
    sensing.isFileCreated = 0;
    if (CheckSdInslot())
    {
      DockSdPowerCycle();
    }
    if (!shimmerStatus.sensing)
    {
      DockUart_enable();
    }
    ShimBt_instreamStatusRespSend();
  }
  else
  {
    ShimBatt_setBatteryInterval(BATT_INTERVAL_UNDOCKED);
    if (!shimmerStatus.sensing)
    {
      DockUart_disable();
    }
    P6OUT |= BIT0;
    ShimBt_instreamStatusRespSend();
    SdPowerOff();
    if (CheckSdInslot() && !shimmerStatus.sensing && !shimmerStatus.sdBadFile)
    {
      _delay_cycles(2880000);
      SdPowerOn();
      LogAndStream_syncConfigAndCalibOnSd();
    }
    else
    {
      LogAndStream_setSdInfoSyncDelayed(1);
    }
  }
  shimmerStatus.configuring = 0;
}

uint8_t CheckSdInslot(void)
{
  //Check if card is inserted and enable interrupt for SD_DETECT_N
  shimmerStatus.sdInserted = (P4IN & BIT1) ? 0 : 1;
  ff_result = ShimSd_mount(shimmerStatus.sdInserted);
  return shimmerStatus.sdInserted;
}

void TB0Start()
{
  if (shimmerStatus.timerSamplingEnabled || shimmerStatus.timerBlinkEnabled)
  {
    if (clockFreq == (float) MSP430_CLOCK)
    {
      TB0CTL = TBSSEL_1 + MC_2;
      TB0EX0 = 0;
    }
#if IS_SUPPORTED_TCXO
    else
    {
      TB0CTL = TBSSEL_0 + MC_2 + ID__8; //use TBSSEL_0 for tcxo, ID__8:divider=8//+ TBCLR
      TB0EX0 = TBIDEX__8; //divider: 8 - Note TCXO_CLOCK will change...
    }
#endif
  }
}

void TB0Stop()
{
  if (!shimmerStatus.timerSamplingEnabled && !shimmerStatus.timerBlinkEnabled)
  {
    TB0CTL = MC_0; //StopTb0()
  }
}

void ClkAssignment()
{
  clk_30 = FreqProd(30);
  clk_45 = FreqProd(45);
  clk_55 = FreqProd(55);
  clk_64 = FreqProd(64);
  clk_75 = FreqProd(75);
  clk_90 = FreqProd(90);
  clk_120 = FreqProd(120);
  clk_133 = FreqProd(133);
  clk_135 = FreqProd(135);
  clk_225 = FreqProd(225);
  clk_255 = FreqProd(255);
  clk_432 = FreqProd(432);
  clk_1000 = FreqProd(1000);
  clk_2500 = FreqProd(2500);

  clk_90_45 = clk_90 - clk_45;
  clk_90_64 = clk_90 - clk_64;
  clk_90_75 = clk_90 - clk_75;

  clk_133_90 = clk_133 - clk_90;
  clk_135_90 = clk_135 - clk_90;
  clk_225_90 = clk_225 - clk_90;
  clk_255_90 = clk_255 - clk_90;
  clk_432_90 = clk_432 - clk_90;
}

uint16_t FreqProd(uint16_t samplingFreq)
{ //e.g. 7.5 ms: num_in=75, .25s:num_in=2500
  return (uint16_t) ceil(samplingClockFreqGet() * ((float) (samplingFreq / 10000.0)));
}

float samplingClockFreqGet(void)
{
  return clockFreq;
}

void eepromRead(uint16_t dataAddr, uint16_t dataSize, uint8_t *dataBuf)
{
  eepromReadWrite(dataAddr, dataSize, dataBuf, EEPROM_READ);
}

void eepromWrite(uint16_t dataAddr, uint16_t dataSize, uint8_t *dataBuf)
{
  eepromReadWrite(dataAddr, dataSize, dataBuf, EEPROM_WRITE);
}

void eepromReadWrite(uint16_t dataAddr, uint16_t dataSize, uint8_t *dataBuf, enum EEPROM_RW eepromRW)
{
  bool timer_was_stopped = FALSE;

  //Spool up EEPROM and required timing peripherals
  if (TB0CTL == MC_0) //Timer is stopped
  {
    BlinkTimerStart();
    timer_was_stopped = TRUE;
  }

  CAT24C16_init();

  //EEPROM needs to be updated with latest bt baud rate, configure here
  if (eepromRW == EEPROM_READ)
  {
    CAT24C16_read(dataAddr, dataSize, dataBuf);
  }
  else
  {
    CAT24C16_write(dataAddr, dataSize, dataBuf);
  }

  //Wind down EEPROM and required timing peripherals
  CAT24C16_powerOff();
  if (timer_was_stopped == TRUE)
  {
    BlinkTimerStop();
  }
}

void updateBtDetailsInEeprom(void)
{
  uint8_t btFwVerAndBaudRate[2];
  btFwVerAndBaudRate[0] = (uint8_t) getBtHwVersion();
  if (isBtDeviceRn41orRN42())
  {
    btFwVerAndBaudRate[1] = BAUD_115200;
  }
  else
  {
    btFwVerAndBaudRate[1] = getCurrentBtBaudRate();
  }
  eepromWrite(RNX_TYPE_EEPROM_ADDRESS + RNX_RADIO_TYPE_IDX, 2U, &btFwVerAndBaudRate[0]);
}

void setSamplingClkSource(float samplingClock)
{
  clockFreq = (float) samplingClock;
  if (clockFreq == (float) TCXO_CLOCK)
  {
    P4OUT |= BIT6;
    _delay_cycles(2400000); //100ms delay for tcxo
    P4SEL |= BIT7;
  }
  else if (clockFreq == (float) MSP430_CLOCK)
  {
    P4OUT &= ~BIT6;
    P4SEL &= ~BIT7;
  }
  ClkAssignment();
}

void triggerShimmerErrorState(void)
{
  while (1)
  {
    Board_ledOff(LED_ALL);
    _delay_cycles(24000000);
    Board_ledOn(LED_LWR_YELLOW);
    _delay_cycles(12000000);
    Board_ledOn(LED_LWR_RED);
    _delay_cycles(12000000);
    Board_ledOn(LED_UPR_BLUE);
    _delay_cycles(12000000);
    Board_ledOn(LED_UPR_GREEN);
    _delay_cycles(12000000);
  }
}

/*
 * TRAP ISR
 *
 * Put all unused ISR vectors here
 *
 * no COMP_B_VECTOR,TIMER2_A0_VECTOR , TIMER2_A1_VECTOR,
 * USCI_B0_VECTOR:i2c
 * USCI_A0_VECTOR:dock/exp_uart
 * USCI_A1_VECTOR:bt_uart
 * TIMER1_A0_VECTOR: msp430_clock.h
 * RTC_VECTOR,
 */
#pragma vector = WDT_VECTOR, SYSNMI_VECTOR, TIMER0_A0_VECTOR, UNMI_VECTOR, \
        USCI_B1_VECTOR, TIMER1_A1_VECTOR

__interrupt void TrapIsr(void)
{
  /*
   * this is a trap ISR - check for the interrupt cause here by
   * checking the interrupt flags, if necessary also clear the interrupt
   * flag
   */
}
