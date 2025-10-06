/*
 * Copyright (c) 2021, Shimmer Research, Ltd.
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
 * code has evolved from TinyOS RovingNetworksP.nc:
 * https://github.com/tinyos/tinyos-main/tree/master/tos/platforms/shimmer/chips/bluetooth/RovingNetworksP.nc
 *
 * @author Mike Healy
 * @date December, 2013
 *
 * @author Weibo Pan
 * @date August, 2014
 *
 * @author Sam O'Mahony
 * @date August, 2021
 *
 * @author Mark Nolan
 * @date December, 2021
 */

#include "../RN4X/RN4X.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../5xx_HAL/hal_Board.h"
#include "../5xx_HAL/hal_DMA.h"
#include "../msp430_clock/msp430_clock.h"
#include "msp430.h"

#include "log_and_stream_externs.h"
#include "log_and_stream_includes.h"
#include "version.h"

uint8_t starting;
void (*runSetCommands_cb)(void);
void (*baudRateChange_cb)(void);
uint8_t (*ShimBt_writeToTxBufAndSend)(uint8_t *buf, uint8_t len, btResponseType responseType);

uint8_t btRebootRequired; //boolean for checking if new commands are added
volatile uint8_t txOverflow;
uint8_t txie_reg;
uint8_t receiveBuffer[8];

char expectedCommandResponse[62U], advertisingName[17], newAutoMaster[13],
    pinCode[17], newSvcClass[5], newDevClass[5], newSvcName[17],
    inquiryScanWindow[5], pagingTime[5], btMode[2], rn4678FastMode[5],
    rn4678BleConnectionParameters[20], rn4678BleSwRevision[5],
    rn4678BleManufacturer[8], rn4xRemoteConfigurationTimer[4],
    bleCompleteLocalName[21];
char commandbuf[32];

rn4678TxPower_et rn4678TxPower;
rn42TxPowerPreAug2012_et rn42TxPowerPreAug2012;
rn42TxPowerPostAug2012_et rn42TxPowerPostAug2012;

BT_SET_COMMAND_STAGES_t bt_setcommands_step;
uint8_t command_received, bt_setcommands_start;
uint8_t bt_runmastercommands_step, bt_runmastercommands_start;
uint8_t bt_getmac_step, bt_getmac_start;
uint8_t bt_setbaudrate_step, useSpecificAdvertisingName;
uint8_t charsReceived;
btOperatingMode radioMode;

uint8_t discoverable, authenticate, encrypt, resetDefaultsRequest, setSvcClassRequest,
    setDevClassRequest, setSvcNameRequest, getMacAddress, getVersion;
volatile uint8_t waitForInitialBoot, updateBaudDuringBoot;
volatile uint8_t waitForReturnNewLine, waitForMacAddress, waitForBtFwVersion, waitForStartCmd;

//master mode stuff
uint8_t deviceConn;
char targetBt[16];
uint8_t doesBtConfigNeedUpdating, btStatusStringsAreEnabled;

uint8_t (*dataAvailableFuncPtr)(uint8_t data) = 0;

uint8_t slowRate;

rn4678OperationalMode rn4678OpMode;

rn4678ConnectionType rn4678ConnectionState;

char *btRxFullResponse;

//global bluetooth variables
volatile enum BT_FIRMWARE_VERSION btFwVer;
volatile uint8_t command_mode_active;

volatile uint8_t rn4678ClassicBtSampleSetBufferSize;
#if BT_FLUSH_TX_BUF_IF_RN4678_RTS_LOCK_DETECTED
volatile uint8_t rn4678RtsLockDetected;
#endif

char *daughtCardIdStrPtrForBle;

const char *const hex = "0123456789ABCDEF";

const char *const rn4678TxPower_str[]
    = { [RN4678_TX_POWER_MINUS_20_DBM] = "0\0", [RN4678_TX_POWER_MINUS_7_DBM] = "1\0", [RN4678_TX_POWER_MINUS_2_DBM] = "2\0", [RN4678_TX_POWER_0_DBM] = "3\0", [RN4678_TX_POWER_PLUS_2_DBM] = "4\0" };

const char *const rn42TxPowerPreAug2012_set_str[]
    = { [RN42_TX_POWER_PRE_AUG_2012_PLUS_12_DBM] = "0004\0",
        [RN42_TX_POWER_PRE_AUG_2012_PLUS_6_DBM] = "0000\0",
        [RN42_TX_POWER_PRE_AUG_2012_PLUS_2_DBM] = "FFFC\0",
        [RN42_TX_POWER_PRE_AUG_2012_0_DBM] = "FFF8\0",
        [RN42_TX_POWER_PRE_AUG_2012_MINUS_5_DBM] = "FFF4\0",
        [RN42_TX_POWER_PRE_AUG_2012_MINUS_10_DBM] = "FFF0\0",
        [RN42_TX_POWER_PRE_AUG_2012_MINUS_20_DBM] = "FFE8\0" };

const char *const rn42TxPowerPreAug2012_get_str[]
    = { [RN42_TX_POWER_PRE_AUG_2012_PLUS_12_DBM] = "4\x0d\x0a\0",
        [RN42_TX_POWER_PRE_AUG_2012_PLUS_6_DBM] = "0\x0d\x0a\0",
        [RN42_TX_POWER_PRE_AUG_2012_PLUS_2_DBM] = "fffc\x0d\x0a\0",
        [RN42_TX_POWER_PRE_AUG_2012_0_DBM] = "fff8\x0d\x0a\0",
        [RN42_TX_POWER_PRE_AUG_2012_MINUS_5_DBM] = "fff4\x0d\x0a\0",
        [RN42_TX_POWER_PRE_AUG_2012_MINUS_10_DBM] = "fff0\x0d\x0a\0",
        [RN42_TX_POWER_PRE_AUG_2012_MINUS_20_DBM] = "ffe8\x0d\x0a\0" };

const char *const rn42PostAug2012TxPower_set_str[]
    = { [RN42_TX_POWER_POST_AUG_2012_PLUS_16_DBM] = "0010\0",
        [RN42_TX_POWER_POST_AUG_2012_PLUS_12_DBM] = "000C\0",
        [RN42_TX_POWER_POST_AUG_2012_PLUS_8_DBM] = "0008\0",
        [RN42_TX_POWER_POST_AUG_2012_PLUS_4_DBM] = "0004\0",
        [RN42_TX_POWER_POST_AUG_2012_0_DBM] = "0000\0",
        [RN42_TX_POWER_POST_AUG_2012_MINUS_4_DBM] = "FFFC\0",
        [RN42_TX_POWER_POST_AUG_2012_MINUS_8_DBM] = "FFF8\0",
        [RN42_TX_POWER_POST_AUG_2012_MINUS_12_DBM] = "FFF4\0" };

const char *const rn42PostAug2012TxPower_get_str[] = { [RN42_TX_POWER_POST_AUG_2012_PLUS_16_DBM]
  = "TX Power=10\x0d\x0a\0",
  [RN42_TX_POWER_POST_AUG_2012_PLUS_12_DBM] = "TX Power=C\x0d\x0a\0",
  [RN42_TX_POWER_POST_AUG_2012_PLUS_8_DBM] = "TX Power=8\x0d\x0a\0",
  [RN42_TX_POWER_POST_AUG_2012_PLUS_4_DBM] = "TX Power=4\x0d\x0a\0",
  [RN42_TX_POWER_POST_AUG_2012_0_DBM] = "TX Power=0\x0d\x0a\0",
  [RN42_TX_POWER_POST_AUG_2012_MINUS_4_DBM] = "TX Power=fffC\x0d\x0a\0",
  [RN42_TX_POWER_POST_AUG_2012_MINUS_8_DBM] = "TX Power=fff8\x0d\x0a\0",
  [RN42_TX_POWER_POST_AUG_2012_MINUS_12_DBM] = "TX Power=fff4\x0d\x0a\0" };

void BT_startDone_cb(void (*cb)(void))
{
  runSetCommands_cb = cb;
}

void BT_baudRateChange_cb(void (*cb)(void))
{
  baudRateChange_cb = cb;
}

/* TODO create common boot sequence for both BT modules */
void initRN1(void)
{
  //if (isBtDeviceRn42())
  //{
  //take RN4X out of reset
  setBtModuleReset(0);
  //}
  //else if (isBtDeviceRn4678())
  //{
  //    // hold RN4678 in reset until fully configured
  //    setBtModuleReset(1);
  //}
}

void initRN2(void)
{
  /* RN42 and RN4678
   * P1.3 = RADIO_RTS
   * P1.0 = RADIO_STATUS
   * P2.2 = RADIO_CTS
   */
  //if (isBtDeviceRn42())
  //{
  setBtRtsInterruptState(1);

  //can assume initially low as module is in reset so watch for low to high transition
  setBtConnectionStatusInterruptIsEnabled(1);

  //toggling cts wakes it up
#if BT_CTS_CONTROL_ENABLED
  P2DIR |= BIT2;
#endif
  setIsBtClearToSend(1);
  //setIsBtClearToSend(0);
  //}
  //else if (isBtDeviceRn4678())
  //{
  //#if BT_CTS_CONTROL_ENABLED
  //P2DIR |= BIT2;
  //#endif
  //setIsBtClearToSend(1);
  //
  //// take RN4678 out of reset
  //setBtModuleReset(0);
  //}
}

void initRN3(void)
{
  //if (isBtDeviceRn42())
  //{
  //    setIsBtClearToSend(1);
  //}
  //else if (isBtDeviceRn4678())
  //{
  //    setBtRtsInterruptState(1);
  //
  //    setBtConnectionStatusInterruptIsEnabled(1);
  //}
}

void setupUART(uint8_t baudRate)
{
  UCA1CTL1 |= UCSWRST;  //**Put state machine in reset**
  UCA1CTL1 |= UCSSEL_2; //SMCLK

  switch (baudRate)
  {
    case BAUD_1200:
      UCA1BR0 = 226;                         //24MHz 1200
      UCA1BR1 = 4;                           //24MHz 1200
      UCA1MCTL = UCBRS_0 + UCBRF_0 + UCOS16; //Modln UCBRSx=0, UCBRFx=0, over sampling
      break;
    case BAUD_2400:
      UCA1BR0 = 113;                         //24MHz 2400
      UCA1BR1 = 2;                           //24MHz 2400
      UCA1MCTL = UCBRS_0 + UCBRF_0 + UCOS16; //Modln UCBRSx=0, UCBRFx=0, over sampling
      break;
    case BAUD_4800:
      UCA1BR0 = 56;                          //24MHz 4800
      UCA1BR1 = 1;                           //24MHz 4800
      UCA1MCTL = UCBRS_0 + UCBRF_8 + UCOS16; //Modln UCBRSx=0, UCBRFx=8, over sampling
      break;
    case BAUD_9600:
      UCA1BR0 = 156;                         //24MHz 9600
      UCA1BR1 = 0;                           //24MHz 9600
      UCA1MCTL = UCBRS_0 + UCBRF_4 + UCOS16; //Modln UCBRSx=0, UCBRFx=4, over sampling
      break;
    case BAUD_19200:
      UCA1BR0 = 78;                          //24MHz 19200
      UCA1BR1 = 0;                           //24MHz 19200
      UCA1MCTL = UCBRS_0 + UCBRF_2 + UCOS16; //Modln UCBRSx=0, UCBRFx=2, over sampling
      break;
    case BAUD_38400:
      UCA1BR0 = 39;                          //24MHz 38400
      UCA1BR1 = 0;                           //24MHz 38400
      UCA1MCTL = UCBRS_0 + UCBRF_1 + UCOS16; //Modln UCBRSx=0, UCBRFx=1, over sampling
      break;
    case BAUD_57600:
      UCA1BR0 = 26;                          //24MHz 57600
      UCA1BR1 = 0;                           //24MHz 57600
      UCA1MCTL = UCBRS_0 + UCBRF_1 + UCOS16; //Modln UCBRSx=0, UCBRFx=1, over sampling
      break;
    case BAUD_230400:
      UCA1BR0 = 6;                           //24MHz 230400
      UCA1BR1 = 0;                           //24MHz 230400
      UCA1MCTL = UCBRS_0 + UCBRF_8 + UCOS16; //Modln UCBRSx=0, UCBRFx=8, over sampling
      break;
    case BAUD_460800:
      UCA1BR0 = 3;                           //24MHz 460800
      UCA1BR1 = 0;                           //24MHz 460800
      UCA1MCTL = UCBRS_0 + UCBRF_4 + UCOS16; //Modln UCBRSx=0, UCBRFx=4, over sampling
      break;
    case BAUD_921600:
      UCA1BR0 = 26;                 //24MHz 921600
      UCA1BR1 = 0;                  //24MHz 921600
      UCA1MCTL = UCBRS_0 + UCBRF_0; //Modln UCBRSx=0, UCBRFx=0, no over sampling
      //(cannot use over sampling as max error would be > 50%)
      break;
    case BAUD_1000000:
      UCA1BR0 = 24;                 //24MHz 1000000
      UCA1BR1 = 0;                  //24MHz 1000000
      UCA1MCTL = UCBRS_0 + UCBRF_0; //Modln UCBRSx=0, UCBRFx=0, no over sampling
      break;
    default:
      UCA1BR0 = 13;                          //24MHz 115200
      UCA1BR1 = 0;                           //24MHz 115200
      UCA1MCTL = UCBRS_0 + UCBRF_0 + UCOS16; //Modln UCBRSx=0, UCBRFx=0, over sampling
      break;
  }

  UCA1CTL1 &= ~UCSWRST; //**Initialize USCI state machine**
  UCA1IFG = 0;          //reset interrupts
  UCA1IE |= UCTXIE;     //Enable USCI_A1 TX interrupt

  if (baudRate == BAUD_1200 || baudRate == BAUD_2400)
  {
    slowRate = 1;
  }
  else
  {
    slowRate = 0;
  }
}

void disableUART(void)
{
  UCA1CTL1 |= UCSWRST; //Put state machine in reset
  UCA1IE &= ~UCTXIE;   //Disable USCI_A1 TX interrupt only
}

void disableRN(void)
{
  setBtModuleReset(1); //hold in reset
  disableUART();
  setBtRtsInterruptState(0);
  setBtConnectionStatusInterruptIsEnabled(0);
  DMA2AndCtsDisable();
}

void writeCommandNoRsp(char *cmd)
{
  return writeCommandNoRspWithCmdLen(cmd, strlen(cmd));
}

void writeCommandNoRspWithCmdLen(char *cmd, uint8_t cmdLen)
{
  memcpy(commandbuf, cmd, cmdLen);
  charsReceived = 0;
  ShimBt_writeToTxBufAndSend((uint8_t *) commandbuf, cmdLen, BT_SETUP);
}

//write data to be transmitted to the Bluetooth module
//returns 0 if fails, else 1
//will only fail if a previous BT_write is still in progress
void writeCommand(char *cmd, char *response)
{
  writeCommandWithCmdLen(cmd, strlen(cmd), response);
}

void writeCommandWithCmdLen(char *cmd, uint8_t cmdLen, char *response)
{
  strcpy(expectedCommandResponse, response);
  setDmaWaitingForResponse(strlen(expectedCommandResponse));

  writeCommandNoRspWithCmdLen(cmd, cmdLen);
}

//this one is awkward. we need to send one command at a time and wait until the
//response is received until sending the next command
//There are a couple of possible approaches
//- use a stepped switch statement and call this function from the receive ISR
//  once the response is received. But results in overly long ISR, from sending
//  the next command from within the ISR and then "hoping" that the ISR exits
//  before the response comes back (not a problem, but bad design)
//  (see TestProject5 for this approach)
//- block on a variable between each step and reset variable in ISR
//  poor design, but this function will only need to be run rarely
//  (see TestProject6 for this approach)
//- put MSP430 in low power mode while waiting for response from RN42
//  best option powerwise, but need to ensure no other interrupt will start
//  processor executing while waiting for response. Also need to take out of low
//  power mode to send commands and return to LPM afterwards
void runSetCommands(void)
{
  if (!bt_setcommands_start)
  {
    BT_rst_MessageProgress();
    command_received = 1;
    bt_setcommands_start = 1;
    DMA2AndCtsDisable();
    setRebootRequired(0);
  }
  if (command_received)
  {
    command_received = 0;
    if (bt_setcommands_step == WAIT_FOR_BOOT)
    {
      bt_setcommands_step++;
      //if (isBtDeviceRn4678())
      //{
      //    BT_setWaitForInitialBoot(1);
      //#if BT_DMA_USED_FOR_RX
      //    setDmaWaitingForResponse(BT_STAT_STR_LEN_RN4678_REBOOT); /* %REBOOT% */
      //#endif
      //    return;
      //}

      /* Have to disable msp430_clock early in the flow because the
       * millisecond interrupt fires so often that characters are missed
       * from the UART communication if it left on. */
      msp430_clock_disable();
    }

    if (bt_setcommands_step == CMD_MODE_START)
    {
      bt_setcommands_step++;

      //if (isBtDeviceRn4678())
      //{
      //    /* Wait until module has rebooted before enabling connection
      //    interrupt */ setBtConnectionStatusInterruptIsEnabled(1);
      //}

      if (!isRnCommandModeActive())
      {
        btCmdModeStart();
        return; //wait until response is received
      }
    }

    if (bt_setcommands_step == GET_VERSION)
    {
      bt_setcommands_step++;
      if (getVersion)
      {
        BT_setGetVersion(0);
        BT_setWaitForVersion(1U);
        setDmaWaitingForResponse(BT_VER_RESPONSE_SMALLEST);
        writeCommandNoRsp("V\r");
        return;
      }
    }

    if (bt_setcommands_step == MODULE_QUIET_1)
    {
      bt_setcommands_step++;
      cmdQuietModeEnter();
      return;
    }

    //reset factory defaults
    if (bt_setcommands_step == RESET_FACTORY_DEFAULTS)
    {
      bt_setcommands_step++;
      if (resetDefaultsRequest)
      {
        sprintf(commandbuf, "SF,1\r");
        setRebootRequired(1);
        writeCommandBufAndExpectAok();
        return;
      }
    }

    //default is slave (== 0), otherwise set mode
    if (bt_setcommands_step == GET_OPERATING_MODE)
    {
      bt_setcommands_step++;
      setDmaWaitForReturnNewLine();
      writeCommandNoRsp("GM\r");
      return;
    }

    if (bt_setcommands_step == SET_OPERATING_MODE)
    {
      bt_setcommands_step++;
      parseRnGetResponse("M", btRxFullResponse);
      if (doesBtConfigNeedUpdating)
      {
        doesBtConfigNeedUpdating = 0;
        sprintf(commandbuf, "SM,%d\r", radioMode);
        writeCommandBufAndExpectAok();
        return;
      }
    }

    if (bt_setcommands_step == GET_MAC_ADDRESS)
    {
      bt_setcommands_step++;
      if (getMacAddress)
      {
        BT_setGetMacAddress(0);
        BT_setWaitForMacAddress(1U);
        uint8_t expRespLen = 14U;
        if (isBtDeviceRn4678())
        {
          //Allow for read of "CMD> " string, 5 chars long
          expRespLen += RN4X_CMD_LEN;
        }
        setDmaWaitingForResponse(expRespLen);

        sprintf(commandbuf, "GB\r");
        writeCommandNoRsp(commandbuf);
        return; //wait until response is received
      }
    }

    if (bt_setcommands_step == SET_MASTER_MAC)
    {
      bt_setcommands_step++;
      //NOTE: main.c doesn't seem to call this command
      if (radioMode == AUTO_CONNECT_MASTER_MODE)
      {
        sprintf(commandbuf, "SR,%s\r", newAutoMaster);
        setRebootRequired(1);
        writeCommandBufAndExpectAok();
        return;
      }
    }

    if (bt_setcommands_step == GET_AUTHENTICATION)
    {
      bt_setcommands_step++;
      setDmaWaitForReturnNewLine();
      writeCommandNoRsp("GA\r");
      return;
    }

    if (bt_setcommands_step == SET_AUTHENTICATION)
    {
      bt_setcommands_step++;
      parseRnGetResponse("A", btRxFullResponse);
      if (doesBtConfigNeedUpdating)
      {
        doesBtConfigNeedUpdating = 0;
        sprintf(commandbuf, "SA,%d\r", authenticate);
        writeCommandBufAndExpectAok();
        return;
      }
    }

    if (bt_setcommands_step == ENABLE_ENCRYPTION)
    {
      bt_setcommands_step++;
      //device default is off
      if (encrypt)
      {
        //Command does not appear to be used in main.c

        //Given this is never set, and the default for both RN42 and
        //RN4678 is 0x1101 - this should operate as normal
        //The SE,X command sets the UUID profile type: SPP, Android, iPhone etc...
        sprintf(commandbuf, "SE,1\r");
        setRebootRequired(1);
        writeCommandBufAndExpectAok();
        return;
      }
    }

    if (bt_setcommands_step == GET_ADVERTISING_NAME)
    {
      bt_setcommands_step++;
      setDmaWaitForReturnNewLine();
      writeCommandNoRsp("GN\r");
      return;
    }

    if (bt_setcommands_step == SET_ADVERTISING_NAME)
    {
      bt_setcommands_step++;
      parseRnGetResponse("N", btRxFullResponse);
      if (doesBtConfigNeedUpdating)
      {
        doesBtConfigNeedUpdating = 0;
        BT_useSpecificAdvertisingName(0U);
        //LogAndStream used S-, SDLog used SN
        sprintf(commandbuf, "S-,%s\r", advertisingName);
        //sprintf(commandbuf, "SN,%s\r", advertisingName);
        writeCommandBufAndExpectAok();
        return;
      }
    }

    if (bt_setcommands_step == GET_PIN)
    {
      bt_setcommands_step++;
      setDmaWaitForReturnNewLine();
      writeCommandNoRsp("GP\r");
      return;
    }

    if (bt_setcommands_step == SET_PIN)
    {
      bt_setcommands_step++;
      //default is none
      if (doesBtConfigNeedUpdating)
      {
        doesBtConfigNeedUpdating = 0;
        parseRnGetResponse("P", btRxFullResponse);
        sprintf(commandbuf, "SP,%s\r", pinCode);
        writeCommandBufAndExpectAok();
        return;
      }
    }

    if (bt_setcommands_step == SET_SVC_CLASS_REQUEST)
    {
      bt_setcommands_step++;
      if (setSvcClassRequest)
      {
        sprintf(commandbuf, "SC,%s\r", newSvcClass);
        setRebootRequired(1);
        writeCommandBufAndExpectAok();
        return;
      }
    }

    if (bt_setcommands_step == SET_DEV_CLASS_REQUEST)
    {
      bt_setcommands_step++;
      if (setDevClassRequest)
      {
        sprintf(commandbuf, "SD,%s\r", newDevClass);
        setRebootRequired(1);
        writeCommandBufAndExpectAok();
        return;
      }
    }

    if (bt_setcommands_step == SET_SVC_NAME_REQUEST)
    {
      bt_setcommands_step++;
      if (setSvcNameRequest)
      {
        sprintf(commandbuf, "SS,%s\r", newSvcName);
        setRebootRequired(1);
        writeCommandBufAndExpectAok();
        return;
      }
    }
    if (bt_setcommands_step == RN42_GET_REMOTE_CONFIGURATION_TIMER)
    {
      bt_setcommands_step++;
      if (isBtDeviceRn41orRN42())
      {
        setDmaWaitForReturnNewLine();
        writeCommandNoRsp("GT\r");
        return;
      }
    }

    if (bt_setcommands_step == RN42_SET_REMOTE_CONFIGURATION_TIMER)
    {
      bt_setcommands_step++;
      if (isBtDeviceRn41orRN42())
      {
        parseRnGetResponse("T", btRxFullResponse);
        //RN4678 has a very different configuration purpose here, for BLE only
        //RN42 uses this command to configure "remote configuration timer".

        //Note: Currently, the only reference to this command is BT_disableRemoteConfig(1)
        //which ultimately sets "ST,0" - thus disabling any remote configuration anyway
        if (doesBtConfigNeedUpdating)
        {
          doesBtConfigNeedUpdating = 0;
          sprintf(commandbuf, "ST,%s\r", rn4xRemoteConfigurationTimer);
          writeCommandBufAndExpectAok();
          return;
        }
      }
    }

    if (bt_setcommands_step == GET_INQUIRY_SCAN_WINDOW)
    {
      bt_setcommands_step++;
      setDmaWaitForReturnNewLine();
      writeCommandNoRsp("GI\r");
      return;
    }

    if (bt_setcommands_step == SET_INQUIRY_SCAN_WINDOW)
    {
      bt_setcommands_step++;
      parseRnGetResponse("I", btRxFullResponse);
      if (doesBtConfigNeedUpdating)
      {
        sprintf(commandbuf, "SI,%s\r", inquiryScanWindow);
        writeCommandBufAndExpectAok();
        return;
      }
    }

    if (bt_setcommands_step == GET_PAGING_TIME)
    {
      bt_setcommands_step++;
      setDmaWaitForReturnNewLine();
      writeCommandNoRsp("GJ\r");
      return;
    }

    if (bt_setcommands_step == SET_PAGING_TIME)
    {
      bt_setcommands_step++;
      parseRnGetResponse("J", btRxFullResponse);
      if (doesBtConfigNeedUpdating)
      {
        doesBtConfigNeedUpdating = 0;
        sprintf(commandbuf, "SJ,%s\r", pagingTime);
        writeCommandBufAndExpectAok();
        return;
      }
    }

    if (bt_setcommands_step == GET_BT_POWER_LEVEL)
    {
      bt_setcommands_step++;
      setDmaWaitForReturnNewLine();
      /* GY seems to be working in RN42 v4.77 but broken in RN42 v6.15.
       * This could also be the case with other versions. Therefore we
       * must fetch the TX Power from the "Other Settings" using 'O' */
      if (btFwVer == RN42_V6_15)
      {
        writeCommandNoRsp("O\r");
      }
      else
      {
        writeCommandNoRsp("GY\r");
      }
      return;
    }

    if (bt_setcommands_step == SET_BT_POWER_LEVEL)
    {
      /* For the RN42 v6.15, we need to parse through each line of the
       * "other settings" until we get to TX Power. Note, after factory
       * reset, tx power always reports 0 until it is changed. */
      if (btFwVer == RN42_V6_15)
      {
        if (strstr(btRxFullResponse, RN42_OTHER_SETTINGS_TX_POWER))
        {
          parseRnGetResponse("Y", btRxFullResponse);
          setDmaWaitForReturnNewLine();
          return;
        }
        /* Role swith is the last line in the "Other Settings "*/
        else if (strstr(btRxFullResponse, RN42_OTHER_SETTINGS_ROLE_SWCH))
        {
          bt_setcommands_step++;
        }
        else
        {
          setDmaWaitForReturnNewLine();
          return;
        }
      }
      else
      {
        bt_setcommands_step++;
        parseRnGetResponse("Y", btRxFullResponse);
      }

      if (doesBtConfigNeedUpdating)
      {
        doesBtConfigNeedUpdating = 0;
        sprintf(commandbuf, "SY,%s\r", BT_getDesiredRnTxPowerForBtVerSetCmd());
        writeCommandBufAndExpectAok();
        return;
      }
    }

    if (isBtDeviceRn41orRN42())
    {
      /* Skip to next stage */
      if (bt_setcommands_step == SET_BT_POWER_LEVEL + 1)
      {
        bt_setcommands_step = GET_STAT_STR_PREFIX_AND_SUFFIX;
      }
    }
    else
    {
      if (bt_setcommands_step == RN4678_GET_BT_MODE)
      {
        bt_setcommands_step++;
        setDmaWaitForReturnNewLine();
        writeCommandNoRsp("GG\r");
        return;
      }

      if (bt_setcommands_step == RN4678_SET_BT_MODE)
      {
        bt_setcommands_step++;
        parseRnGetResponse("G", btRxFullResponse);
        if (doesBtConfigNeedUpdating)
        {
          doesBtConfigNeedUpdating = 0;
          sprintf(commandbuf, "SG,%s\r", btMode);
          writeCommandBufAndExpectAok();
          return;
        }
      }

      if (bt_setcommands_step == RN4678_GET_FAST_MODE)
      {
        bt_setcommands_step++;
        setDmaWaitForReturnNewLine();
        writeCommandNoRsp("GQ\r");
        return;
      }

      if (bt_setcommands_step == RN4678_SET_FAST_MODE)
      {
        bt_setcommands_step++;
        parseRnGetResponse("Q", btRxFullResponse);
        if (doesBtConfigNeedUpdating)
        {
          sprintf(commandbuf, "SQ,%s\r", rn4678FastMode);
          writeCommandBufAndExpectAok();
          return;
        }
      }

      /* Skipping BLE setup for the moment if sync is enabled to reduce
       * initialisation time while sensing */
      if (!ShimBt_isBleCurrentlyEnabled() || shimmerStatus.btInSyncMode)
      {
        /* Skip to next stage */
        if (bt_setcommands_step == RN4678_SET_FAST_MODE + 1)
        {
          bt_setcommands_step = GET_STAT_STR_PREFIX_AND_SUFFIX;
        }
      }
      else
      {
        /* BLE commands for RN4678 functionality */
        /* https://ww1.microchip.com/downloads/en/DeviceDoc/RN4678-Bluetooth-Dual-Mode-Module-Command-Reference-User-Guide-DS50002506C.pdf */
        if (bt_setcommands_step == RN4678_GET_MODEL_STRING)
        {
          bt_setcommands_step++;

          setDmaWaitForReturnNewLine();
          writeCommandNoRsp("GDM\r");
          return;
        }

        if (bt_setcommands_step == RN4678_SET_MODEL_STRING)
        {
          bt_setcommands_step++;
          parseRnGetResponse("DM", btRxFullResponse);
          //Sets the model string in BLE Device Information Service.
          if (doesBtConfigNeedUpdating)
          {
            doesBtConfigNeedUpdating = 0;
            sprintf(commandbuf, "SDM,%s\r", daughtCardIdStrPtrForBle);
            writeCommandBufAndExpectAok();
            return;
          }
        }

        if (bt_setcommands_step == RN4678_GET_MANUFACTURER_STRING)
        {
          bt_setcommands_step++;

          setDmaWaitForReturnNewLine();
          writeCommandNoRsp("GDN\r");
          return;
        }

        if (bt_setcommands_step == RN4678_SET_MANUFACTURER_STRING)
        {
          bt_setcommands_step++;
          parseRnGetResponse("DN", btRxFullResponse);
          //Sets the manufacturer string in BLE Device Information service
          if (doesBtConfigNeedUpdating)
          {
            doesBtConfigNeedUpdating = 0;
            sprintf(commandbuf, "SDN,%s\r", rn4678BleManufacturer);
            writeCommandBufAndExpectAok();
            return;
          }
        }

        if (bt_setcommands_step == RN4678_GET_SOFTWARE_REVISION_STRING)
        {
          bt_setcommands_step++;

          setDmaWaitForReturnNewLine();
          writeCommandNoRsp("GDR\r");
          return;
        }

        if (bt_setcommands_step == RN4678_SET_SOFTWARE_REVISION_STRING)
        {
          bt_setcommands_step++;
          parseRnGetResponse("DR", btRxFullResponse);
          if (doesBtConfigNeedUpdating)
          {
            //Sets the software revision of the firmware. This can accept a maximum of 4 characters
            doesBtConfigNeedUpdating = 0;
            sprintf(commandbuf, "SDR,%s\r", rn4678BleSwRevision);
            writeCommandBufAndExpectAok();
            return;
          }
        }

        if (bt_setcommands_step == RN4678_GET_CONNECTION_PARAMETERS)
        {
          bt_setcommands_step++;
          setDmaWaitForReturnNewLine();
          writeCommandNoRsp("GT\r");
          return;
        }

        if (bt_setcommands_step == RN4678_SET_CONNECTION_PARAMETERS)
        {
          bt_setcommands_step++;
          parseRnGetResponse("T", btRxFullResponse);
          if (doesBtConfigNeedUpdating)
          {
            doesBtConfigNeedUpdating = 0;
            sprintf(commandbuf, "ST,%s\r", rn4678BleConnectionParameters);
            writeCommandBufAndExpectAok();
            return;
          }
        }
      }
    }

    /* Get current status string prefix and suffix */
    if (bt_setcommands_step == GET_STAT_STR_PREFIX_AND_SUFFIX)
    {
      bt_setcommands_step++;
#if BT_RN42_STAT_STR_ENABLED
      btStatusStringsAreEnabled = 1;
#else
      btStatusStringsAreEnabled = isBtDeviceRn4678();
#endif

      setDmaWaitForReturnNewLine();
      writeCommandNoRsp("GO\r");
      return;
    }

    /* Check if the status string suffix and prefix are correctly set */
    if (bt_setcommands_step == UPDATE_STAT_STR_PREFIX_AND_SUFFIX)
    {
      bt_setcommands_step++;
      parseRnGetResponse("O", btRxFullResponse);

      if (doesBtConfigNeedUpdating)
      {
        doesBtConfigNeedUpdating = 0;

        uint8_t cmdLen = 3;
        sprintf(commandbuf, "SO,");

        if (areBtStatusStringsEnabled())
        {
          if (isBtDeviceRn41orRN42())
          {
            /* Set Prefix = % -> "SO,%\r" */
            commandbuf[cmdLen++] = '%';
            commandbuf[cmdLen++] = '\r';
          }
          else if (isBtDeviceRn4678())
          {
            /* Set Prefix&Suffix = % -> "SO,%,%\r" */
            commandbuf[cmdLen++] = '%';
            commandbuf[cmdLen++] = ',';
            commandbuf[cmdLen++] = '%';
            commandbuf[cmdLen++] = '\r';
          }
        }
        else
        {
          /* Disable status strings */
          if (isBtDeviceRn41orRN42())
          {
            /* "SO, \r" */
            commandbuf[cmdLen++] = ' ';
            commandbuf[cmdLen++] = '\r';
          }
          else if (isBtDeviceRn4678())
          {
            /* "SO,,\r" */
            commandbuf[cmdLen++] = ',';
            commandbuf[cmdLen++] = '\r';
          }
        }
        /* sprintf isn't liking the "%" characters and so it better to pass in the length */
        writeCommandBufAndExpectAokWithCmdLen(cmdLen);
        return;
      }
    }

    if (bt_setcommands_step == UPDATE_BAUD_RATE_1)
    {
      bt_setcommands_step++;
      if (updateBaudDuringBoot)
      {
        uint8_t defaultBaud = getDefaultBaudForBtVersion();
        if (ShimBt_getBtBaudRateToUse() == defaultBaud)
        {
          BT_setUpdateBaudDuringBoot(0);
        }
        else
        {
          ShimBt_setBtBaudRateToUse(defaultBaud);
          sendBaudRateUpdateToBtModule();
          return;
        }
      }
    }

    if (bt_setcommands_step == UPDATE_BAUD_RATE_2)
    {
      bt_setcommands_step++;
      if (updateBaudDuringBoot && isBtDeviceRn4678())
      {
        /* BT module rebooted here so no need to do it again */
        setRebootRequired(0);
        sprintf(commandbuf, "R,1\r");
        writeCommand(commandbuf, "Rebooting\r\n");
        return;
      }
    }

    if (bt_setcommands_step == UPDATE_BAUD_RATE_3)
    {
      bt_setcommands_step++;
      if (updateBaudDuringBoot)
      {
        BT_setUpdateBaudDuringBoot(0U);

        //change MSP430 UART to use new baud rate
        setupUART(ShimBt_getBtBaudRateToUse());

        /* RN42 automatically exists command mode after the
         * temporary baud rate is updated and we're rebooting RN4678 so
         * it CMD mode needs to be entered again. */
        setRnCommandModeActive(0);

        if (isBtDeviceRn4678())
        {
          /* Set DMA to listen for %REBOOT% after power cycle */
          BT_setWaitForInitialBoot(1);
          setDmaWaitingForResponse(BT_STAT_STR_LEN_RN4678_REBOOT);
          return;
        }
      }
    }

    /* When the temporary baud rate is updated in the RN4X, the BT module
     * automatically exits command mode but it isn't clear from the user
     * manuals whether this is effectively the same as a reboot (which is
     * required if any set commands have been called up to this point in the
     * boot sequence). Therefore we must reenter command mode for the RN4X
     * here before calling a reboot. */
    if (bt_setcommands_step == REENTER_CMD_MODE)
    {
      bt_setcommands_step++;
      if (!isRnCommandModeActive())
      {
        /* Experimentally found that a delay is needed after the baud
         * rate is changed before we can call the start command.
         * Arbitrarily choosing 100ms.
         * We need to use the msp430_clock library instead of
         * delay_cycles to avoid packet loss in SDLog */
        msp430_clock_enable();
        msp430_register_timer_cb(btCmdModeStartAfterRn4xTempBaudChange, 100, 0); //100 ms
        return;
      }
    }

    if (bt_setcommands_step == REBOOT)
    {
      bt_setcommands_step++;
      if (btRebootRequired)
      {
        sprintf(commandbuf, "R,1\r");
        /* Reboot is needed after set commands for RN42 */
        if (btFwVer == RN41_V4_77 || btFwVer == RN42_V4_77 || !areBtStatusStringsEnabled())
        {
          writeCommand(commandbuf, "Reboot!\r\n");
        }
        else if (btFwVer == RN42_V6_15)
        {
          BT_setWaitForInitialBoot(1);
          writeCommand(commandbuf, "Reboot!\r\n%REBOOT");
        }
        else if (isBtDeviceRn4678())
        {
          writeCommand(commandbuf, "Rebooting\r\n%REBOOT%");
        }
        setRnCommandModeActive(0);
        setRebootRequired(0);
        return;
      }
    }

    if (!ShimBt_isBleCurrentlyEnabled() || shimmerStatus.btInSyncMode)
    {
      /* Skip to next stage */
      if (bt_setcommands_step == REBOOT + 1)
      {
        bt_setcommands_step = MODULE_WAKEUP;
      }
    }
    else
    {
      /* Temporary commands for RN4678 BLE must be set after a reboot - if one was needed. These temporary commands are not supported in V1.00.5 firmware */
      if (btFwVer == RN4678_V1_23_0)
      {
        if (bt_setcommands_step == RN4678_REENTER_CMD_MODE)
        {
          bt_setcommands_step++;
          if (!isRnCommandModeActive())
          {
            btCmdModeStart();
            return;
          }
        }

        if (bt_setcommands_step == MODULE_QUIET_2)
        {
          bt_setcommands_step++;
          cmdQuietModeEnter();
          return;
        }

        if (bt_setcommands_step == RN4678_SET_BLE_LOCAL_ADV_NAME)
        {
          bt_setcommands_step++;
          /* Append MAC ID to end of BLE advertising name */

          /* Note: We were using sprintf with "%02x" to make this a
           * one-liner but that requires full print support which
           * increases flash requirements by 6KB */
          bleCompleteLocalName[10U] = hex[((uint8_t) ('-' >> 4) & 0xF)];
          bleCompleteLocalName[11U] = hex[(uint8_t) ('-' & 0xF)];
          bleCompleteLocalName[12U]
              = hex[(uint8_t) ((ShimBt_macIdStrPtrGet()[8U] >> 4) & 0xF)];
          bleCompleteLocalName[13U] = hex[(uint8_t) (ShimBt_macIdStrPtrGet()[8U] & 0xF)];
          bleCompleteLocalName[14U]
              = hex[(uint8_t) ((ShimBt_macIdStrPtrGet()[9U] >> 4) & 0xF)];
          bleCompleteLocalName[15U] = hex[(uint8_t) (ShimBt_macIdStrPtrGet()[9U] & 0xF)];
          bleCompleteLocalName[16U]
              = hex[(uint8_t) ((ShimBt_macIdStrPtrGet()[10U] >> 4) & 0xF)];
          bleCompleteLocalName[17U]
              = hex[(uint8_t) (ShimBt_macIdStrPtrGet()[10U] & 0xF)];
          bleCompleteLocalName[18U]
              = hex[(uint8_t) ((ShimBt_macIdStrPtrGet()[11U] >> 4) & 0xF)];
          bleCompleteLocalName[19U]
              = hex[(uint8_t) (ShimBt_macIdStrPtrGet()[11U] & 0xF)];
          bleCompleteLocalName[20U] = 0;

          sprintf(commandbuf, "IA,09,%s\r", bleCompleteLocalName);
          writeCommandBufAndExpectAok();
          return;
        }

        /* TODO We haven't been able to get this working yet, RN4678 always shows as "Generic Computer" */
        //if (bt_setcommands_step == RN4678_SET_BLE_APPEARANCE)
        //{
        //    bt_setcommands_step++;
        //    sprintf(commandbuf, "IA,19,4005\r"); //0x0540 Generic Sensor
        //    writeCommandBufAndExpectAok();
        //    return;
        //}
      }
      else
      {
        if (bt_setcommands_step == REBOOT + 1)
        {
          bt_setcommands_step = MODULE_WAKEUP;
        }
      }
    }

    if (bt_setcommands_step == MODULE_WAKEUP)
    {
      bt_setcommands_step++;
      if (isRnCommandModeActive())
      {
        cmdQuietModeExit();
        return;
      }
    }

    if (bt_setcommands_step == CMD_MODE_STOP)
    {
      bt_setcommands_step++;
      //exit command mode
      if (isRnCommandModeActive())
      {
        btCmdModeStop();
        return;
      }
    }

    //arriving here = all done perfectly
    if (bt_setcommands_step == FINISH)
    {
      bt_setcommands_step = WAIT_FOR_BOOT;
      bt_setcommands_start = 0;
      setRnCommandModeActive(0);

      if (runSetCommands_cb)
      {
        runSetCommands_cb();
      }

      /* Charge up the DMA again to be able to read status strings from BT
       * module. If status strings are disabled, this is done when the
       * connection status pin interrupt is triggered. */
      setDmaWaitingForResponseIfStatusStrEnabled();
    }
  }
}

void runMasterCommands(void)
{
  if (!bt_runmastercommands_start)
  {
    BT_rst_MessageProgress();
    command_received = 1;
    bt_runmastercommands_start = 1;
    DMA2AndCtsDisable();
  }
  if (command_received)
  {
    command_received = 0;
    if (bt_runmastercommands_step == 0)
    {
      bt_runmastercommands_step++;
      if (!isRnCommandModeActive())
      {
        btCmdModeStart();
        return; //wait until response is received
      }
    }
    //Connect
    if (bt_runmastercommands_step == 1)
    {
      bt_runmastercommands_step++;
      if (deviceConn && (!shimmerStatus.btConnected))
      { //Connect
        sprintf(commandbuf, "C,%s\r", targetBt);
        if (isBtDeviceRn41orRN42())
        {
          writeCommand(commandbuf, "TRYING\r\n");
        }
        else if (isBtDeviceRn4678())
        {
          //lowercase
          writeCommand(commandbuf, "Trying\r\n");
        }
      }
      else if ((!deviceConn) && (shimmerStatus.btConnected))
      { //Disconnect
        sprintf(commandbuf, "K,\r");
        writeCommand(commandbuf, "KILL\r\n");
      }
      else
      { //exit command mode
        //not needed for connect and disconnect commands
        btCmdModeStop();
        deviceConn = 0;
      }
      return;
    }

    if (bt_runmastercommands_step == 2)
    {
      bt_runmastercommands_step = 0;
      bt_runmastercommands_start = 0;

      /* Charge up the DMA again to be able to read status strings from BT
       * module. If status strings are disabled, this is done when the
       * connection status pin interrupt is triggered. */
      setDmaWaitingForResponseIfStatusStrEnabled();
    }
  }
  return;
}

void cmdQuietModeEnter(void)
{
  /* From testing, RN42 FW v4.77 responds back with "Quiet\r\n". RN42 v6.15 and
   * RN4678 v1.00.5 and v1.23 respond back with "AOK\r\nCMD> " */
  if (btFwVer == RN41_V4_77 || btFwVer == RN42_V4_77)
  {
    //The Q command makes the module non-discoverable.
    writeCommand("Q\r", "Quiet\r\n");
  }
  else
  {
    /*The Q,1 command makes the module not non-discoverable and non-connectable
     * (both Classic BT and BLE - if supported).*/
    sprintf(commandbuf, "Q,1\r");
    writeCommandBufAndExpectAok();
  }
}

void cmdQuietModeExit(void)
{
  sprintf(commandbuf, "W\r");
  if (isBtDeviceRn41orRN42())
  {
    //Tested on RN42 v4.77 and v6.15
    writeCommand(commandbuf, "Wake\r\n");
  }
  else
  {
    writeCommandBufAndExpectAok();
  }
}

void sendBaudRateUpdateToBtModule(void)
{
  if (isBtDeviceRn41orRN42())
  {
    /* RN42:
     * Setting the baud rate on the RN42 immediately takes effect after a
     * successful baud rate is set - indicated by a received "AOK" message The
     * second parameter "N" is the parity type For the RN42, the baud rate
     * change is only temporary as the "new" baud rate settings aren't stored in
     * RN42 flash and the default 115200 baud is restored on each device reboot.
     */
    switch (ShimBt_getBtBaudRateToUse())
    {
      case BAUD_1200:
        sprintf(commandbuf, "U,1200,N\r");
        break;
      case BAUD_2400:
        sprintf(commandbuf, "U,2400,N\r");
        break;
      case BAUD_4800:
        sprintf(commandbuf, "U,4800,N\r");
        break;
      case BAUD_9600:
        sprintf(commandbuf, "U,9600,N\r");
        break;
      case BAUD_19200:
        sprintf(commandbuf, "U,19.2,N\r");
        break;
      case BAUD_38400:
        sprintf(commandbuf, "U,38.4,N\r");
        break;
      case BAUD_57600:
        sprintf(commandbuf, "U,57.6,N\r");
        break;
      case BAUD_230400:
        sprintf(commandbuf, "U,230K,N\r");
        break;
      case BAUD_460800:
        sprintf(commandbuf, "U,460K,N\r");
        break;
      case BAUD_921600:
        sprintf(commandbuf, "U,921K,N\r");
        break;
      default:
        sprintf(commandbuf, "U,115K,N\r");
        break;
    }
  }
  else if (isBtDeviceRn4678())
  {
    /* RN4678:
     * The RN4678 instead uses the SU command to set the baud rate.
     * NOTE: The "U" command used for RN42 means something very different to RN4678.
     *
     * In contrast with the RN42, the RN4678 does not have a temporary baud rate option
     * and will "remember" the last uart baud rate value in non-volatile EEPROM on the
     * RN4678 device. Because of this, changes in baud rate must be tracked locally on
     * the Shimmer when communicating with the RN4678 device.
     *
     * Once the SU command is used to set a new baud rate, the new baud rate does not
     * take effect until the device is reset/rebooted. This is (again) in contrast to
     * the RN42 and so the command flow must be different here.
     *
     * No support for 06=28800, 08=14400, 0C=3000000, 0D=4000000,
     * 0E=3250000, 0F=1843200, 10=307200
     */
    switch (ShimBt_getBtBaudRateToUse())
    {
      //NO RN4678 support for 1200 baud
      case BAUD_2400:
        sprintf(commandbuf, "SU,0B\r");
        break;
      case BAUD_4800:
        sprintf(commandbuf, "SU,0A\r");
        break;
      case BAUD_9600:
        sprintf(commandbuf, "SU,09\r");
        break;
      case BAUD_19200:
        sprintf(commandbuf, "SU,07\r");
        break;
      case BAUD_38400:
        sprintf(commandbuf, "SU,05\r");
        break;
      case BAUD_57600:
        sprintf(commandbuf, "SU,04\r");
        break;
      case BAUD_230400:
        sprintf(commandbuf, "SU,02\r");
        break;
      case BAUD_460800:
        sprintf(commandbuf, "SU,01\r");
        break;
      case BAUD_921600:
        sprintf(commandbuf, "SU,00\r");
        break;
      case BAUD_1000000:
        sprintf(commandbuf, "SU,11\r");
        break;
      default:
        sprintf(commandbuf, "SU,03\r"); //BAUD_115200
        break;
    }
  }

  if (slowRate)
  {
    //UCA1IE &= ~UCRXIE;                  //Disable USCI_A1 RX interrupt
    writeCommandNoRsp(commandbuf);
    DMA2AndCtsDisable();
    _delay_cycles(4800000);

    uint8_t expRespLen = 1U;
    if (isBtDeviceRn4678())
    {
      //Allow for read of "CMD> " string, 5 chars long
      expRespLen += RN4X_CMD_LEN;
    }
    setDmaWaitingForResponse(expRespLen);
  }
  else
  {
    writeCommandBufAndExpectAok();
  }
}

void writeCommandBufAndExpectAok(void)
{
  writeCommandBufAndExpectAokWithCmdLen(strlen(commandbuf));
}

void writeCommandBufAndExpectAokWithCmdLen(uint8_t cmdBufLen)
{
  if (isBtDeviceRn41orRN42())
  {
    writeCommandWithCmdLen(commandbuf, cmdBufLen, RN4X_AOK_RESPONSE);
  }
  else if (isBtDeviceRn4678())
  {
    writeCommandWithCmdLen(commandbuf, cmdBufLen, RN4678_AOK_CMD_RESPONSE);
  }
}

void btCmdModeStartAfterRn4xTempBaudChange(void)
{
  msp430_clock_disable();

  /* DMA was locked onto waiting for 1 byte (as set by default at the end of
   * Dma2ConversionDone) whereas we need it to be 5 bytes here. Simplest way
   * that makes it work is to disable the DMA before changing the number of
   * bytes. */
  DMA2_disable();

  btCmdModeStart();
}

void btCmdModeStart(void)
{
  BT_setWaitForStartCmd(1U);
  setDmaWaitingForResponse(RN4X_CMD_LEN);
  writeCommandNoRsp("$$$");
}

void btCmdModeStop(void)
{
  writeCommand("---\r", "END\r\n");
}

void BT_resetBaudRate(void)
{
  slowRate = 0;
  ShimBt_setBtBaudRateToUse(BAUD_115200);
}

void BT_setGoodCommand(void)
{
  command_received = 1;
  clearExpectedResponseBuf();
  if (bt_setcommands_start)
  {
    runSetCommands();
  }
  if (bt_runmastercommands_start)
  {
    runMasterCommands();
  }
}

uint8_t areBtSetupCommandsRunning(void)
{
  return (bt_setcommands_start || bt_runmastercommands_start);
}

uint8_t isBtModuleOverflowPinHigh(void)
{
  return (P1IN & BIT3);
}

HAL_StatusTypeDefShimmer BtTransmit(uint8_t *buf, uint8_t len)
{
  /* commenting out while loop as individual bytes are sent based on
   * interrupt firing so no need to wait here. */
  //ensure no tx interrupt is pending
  //while (UCA1IFG & UCTXIFG);
  UCA1TXBUF = *buf;
  return HAL_SHIM_OK;
}

void BT_init(void)
{
  txOverflow = 0;

  setRn4678ConnectionState(RN4678_DISCONNECTED);

  txie_reg = 0;
  command_received = 0;
  bt_setbaudrate_step = 0;
  bt_getmac_step = 0;
  bt_getmac_start = 0;
  bt_setcommands_start = 0;
  bt_setcommands_step = WAIT_FOR_BOOT;
  bt_runmastercommands_step = 0;
  bt_runmastercommands_start = 0;
  BT_setWaitForInitialBoot(0);
  BT_setWaitForReturnNewLine(0);
  BT_setWaitForVersion(0);
  BT_setWaitForMacAddress(0);
  BT_setWaitForStartCmd(0);
  BT_setUpdateBaudDuringBoot(0);
  getMacAddress = 0;
  BT_setRadioMode(SLAVE_MODE);
  BT_setDiscoverable(1U);
  encrypt = 0;
  resetDefaultsRequest = 0;
  setSvcClassRequest = 0;
  setSvcNameRequest = 0;
  setDevClassRequest = 0;
  BT_rn4xDisableRemoteConfig(0);
  getVersion = 0;
  memset(receiveBuffer, 0, 8);
  setRnCommandModeActive(0);
  //connect/disconnect commands
  deviceConn = 0;
  shimmerStatus.btConnected = 0;
  BT_useSpecificAdvertisingName(0U);

  clearExpectedResponseBuf();
  charsReceived = 0;

#if BT_FLUSH_TX_BUF_IF_RN4678_RTS_LOCK_DETECTED
  rn4678RtsLockDetected = 0U;
#endif

#if ADVERTISING_NAME_IS_OUTPUT
  BT_setAdvertisingName(ADVERTISING_NAME_OUTPUT);
#else
  BT_setAdvertisingName(ADVERTISING_NAME_SHIMMER3);
#endif

  BT_setPIN("1234");

  //to save power only leave inquiry on for approx 40msec (every 1.28 secs)
  BT_setInquiryScanWindow("0040");

  //to save power only leave paging on for approx 80msec. Default Roving Network
  //setting is 0x0100 (160ms) BT_setPagingTime("0080"); // 80ms
  BT_setPagingTime("0100"); //160ms

  if (shimmerStatus.sdSyncEnabled)
  {
    /* Using legacy "1234" pin code for SD Sync as it's all we can get working
     * so far to let the RN42, RN4678 and CYW20820 within the S3 and S3R models
     * to all connect together. */
    BT_setAuthentication(4U);
  }
  else
  {
    /* BLE isn't compatible with the standard "1234" passkey that Shimmer3 has
     * always used for Classic Bluetooth (authentication mode 4) so we're just
     * disabling the passkey here altogether. */
    BT_setAuthentication(2U);
  }

  setBleDeviceInformation(ShimBrd_getDaughtCardIdStrPtr(), FW_VERSION_MAJOR,
      FW_VERSION_MINOR, FW_VERSION_PATCH);

  //Enable fast mode with HW flow control enabled
  BT_setRn4678FastMode("9000");

  //Set the power level to maximum for all BT modules that Shimmer uses
  BT_setRn4678TxPower(RN4678_TX_POWER_PLUS_2_DBM);
  BT_setRn42TxPowerPreAug2012(RN42_TX_POWER_PRE_AUG_2012_PLUS_12_DBM);
  BT_setRn42TxPowerPostAug2012(RN42_TX_POWER_POST_AUG_2012_PLUS_16_DBM);

  /* Sets the connection parameters for BLE connection
   Field 1 = Minimum connection interval = 0x0001 = 1*1.25ms = 1.25ms
   Field 2 = Maximum connection interval = 0x001C = 28*1.25ms = 35ms (default setting)
   Field 3 = Slave latency = 0x0000 = 0 (default setting)
   Field 4 = Supervision timeout = 0x0200 = 512*10ms = 5.12s (default setting)
   */
  BT_setRn4678BleConnectionParameters("0001,001C,0000,0200");

  BT_setRn4678BleCompleteLocalName(BLE_ADVERTISING_NAME_SHIMMER3);
}

void btInit(void)
{
  starting = 1;

  //Turn on power (SW_BT P4.3 on SR30 and newer)
  setBtModulePower(1);

  msp430_clock_init();

  //10ms chosen through test. 100ms found not to work on RN42 v6.15
  msp430_register_timer_cb(start1, 10, 0);
}

void start1(void)
{
  initRN1();

  //powerup state is reset == low (true); mike conrad of roving networks sez:
  //wait about 1s to 2s after reset toggle
  /* Additionally, since we don't know which BT module is connected, this gives a chances for any REBOOT status sting to clear */
  msp430_register_timer_cb(start2, 2000, 0); //2 s
}

void start2(void)
{
  if (starting)
  {
    initRN2();
    msp430_register_timer_cb(start3, 5, 0); //5 ms
  }
}

void start3(void)
{
  if (starting)
  {
    initRN3();
    setupUART(ShimBt_getBtBaudRateToUse());

    msp430_register_timer_cb(runSetCommands, 15, 0); //15 ms

    starting = 0;
  }
}

void BT_disable(void)
{
  disableRN();
  //Turn off power (SW_BT P4.3 on SR30 and newer)
  setBtModulePower(0);
  starting = 0;
  DMA2AndCtsDisable();

  /* When RN42 is turned off it reverts to 115200 baud on next boot */
  if (isBtDeviceRn41orRN42())
  {
    BT_resetBaudRate();
  }
}

//write data to be transmitted to the Bluetooth module - specific to the RN42
uint8_t BT_write_rn42(uint8_t *buf, uint8_t len, btResponseType responseType)
{
  if (ShimBt_getSpaceInBtTxBuf() <= len)
  {
    return 1; //fail
  }

  ShimBt_pushBytesToBtTxBuf(buf, len);

  ShimBt_sendNextCharIfNotInProgress();
  return 0;
}

uint8_t BT_write_rn4678_460800(uint8_t *buf, uint8_t len, btResponseType responseType)
{
  /* If it's the RN4678 and 1Mbps isn't supported, only allow a fixed number of sensor data packets to be in the TX buffer */
  return BT_write_rn4678_with_buf(buf, len, responseType, rn4678ClassicBtSampleSetBufferSize);
}

uint8_t BT_write_rn4678_ble(uint8_t *buf, uint8_t len, btResponseType responseType)
{
  /* If it's BLE, try to fill as many bytes as possible into the available MTU size */
  return BT_write_rn4678_with_buf(buf, len, responseType, BLE_MTU_SIZE);
}

uint8_t BT_write_rn4678_with_buf(uint8_t *buf, uint8_t len, btResponseType responseType, uint8_t sampleSetBufferSize)
{
  /* Buffer before sending to the BT module */
  if (ShimBt_getSpaceInBtTxBuf() <= len
      || (responseType == SENSOR_DATA
          && (ShimBt_getUsedSpaceInBtTxBuf() >= sampleSetBufferSize
              || ShimBt_btTxInProgressGet() || isBtModuleOverflowPinHigh())))
  {
    return 1; //fail
  }

  ShimBt_pushBytesToBtTxBuf(buf, len);

#if BT_FLUSH_TX_BUF_IF_RN4678_RTS_LOCK_DETECTED
  if (responseType == SHIMMER_CMD && isBtModuleOverflowPinHigh())
  {
    rn4678RtsLockDetected = 1U;
  }
#endif

  /* Wait until the local buffer has reached a certain size before sending to
   * the Bluetooth module - grouping the bytes was found to help to reduce
   * packet loss */
  if (responseType != SENSOR_DATA || ((ShimBt_getUsedSpaceInBtTxBuf() + len) > sampleSetBufferSize))
  {
    ShimBt_sendNextCharIfNotInProgress();
  }
  return 0;
}

uint8_t BT_write_rn4678_1M(uint8_t *buf, uint8_t len, btResponseType responseType)
{
  if (ShimBt_getSpaceInBtTxBuf() <= len)
  {
    return 1; //fail
  }

  ShimBt_pushBytesToBtTxBuf(buf, len);

#if BT_FLUSH_TX_BUF_IF_RN4678_RTS_LOCK_DETECTED
  if (responseType == SHIMMER_CMD && isBtModuleOverflowPinHigh())
  {
    rn4678RtsLockDetected = 1U;
  }
#endif

  ShimBt_sendNextCharIfNotInProgress();

  return 0;
}

void BT_connect(uint8_t *addr)
{
  deviceConn = 1;
  strcpy(targetBt, (const char *) addr);
  runMasterCommands();
}

void BT_disconnect(void)
{
  //Delay: If any bytes are seen before or after $$$ in a 1
  //second window, command mode will not be entered and these
  //bytes will be passed on to other side

  deviceConn = 0;

  runMasterCommands();
}

void BT_setGetMacAddress(uint8_t val)
{
  getMacAddress = val;
  ShimBt_macIdVarsReset();
}

void BT_setGetVersion(uint8_t val)
{
  getVersion = val;
  if (getVersion)
  {
    setBtFwVersion(BT_FW_VER_UNKNOWN);
  }
}

void BT_setWaitForInitialBoot(uint8_t val)
{
  waitForInitialBoot = val;
}

void BT_setWaitForReturnNewLine(uint8_t val)
{
  waitForReturnNewLine = val;
  if (waitForReturnNewLine)
  {
    memset(btRxFullResponse, 0, strlen((char *) btRxFullResponse));
  }
}

void BT_setRadioMode(btOperatingMode mode)
{
  radioMode = mode;
}

void BT_setAutoMaster(char *master)
{
  snprintf(newAutoMaster, 13, "%s", master);
}

void BT_setDiscoverable(uint8_t disc)
{
  discoverable = disc;

  if (discoverable)
  {
    BT_setInquiryScanWindow("0040");
  }
  else
  {
    //device is discoverable with a non-zero inquiry scan window
    // NOTE: main.c doesn't seem to call this command
    /* TODO RN4678 responds back with ERR if trying to set 0000 */
    BT_setInquiryScanWindow("0000");
  }
}

void BT_setEncryption(uint8_t enc)
{
  encrypt = enc;
}

/*
 * MODES:
 * RN42 - see TABLE 2-3
 * 0 - Open mode ( if(authenticate) won't get here)
 * 1 - SSP keyboard I/O mode (default)
 * 2 - SSP "just works" mode
 * 4 - Pin code
 *
 * RN4678 - see TABLE 2-1
 * 1 - Secure Simple Pairing (SSP) Pin Code Confirm mode
 * 2 - SSP "just works" mode (default)
 * 3 - SSP Pin Code Input mode
 * 4 - Legacy Pin Code mode
 */
void BT_setAuthentication(uint8_t auth)
{
  authenticate = auth;
}

void BT_setAdvertisingName(char *name)
{
  snprintf(advertisingName, 17, "%s", name);
}

void BT_useSpecificAdvertisingName(uint8_t val)
{
  useSpecificAdvertisingName = val;
}

void BT_setPIN(char *PIN)
{
  snprintf(pinCode, 17, "%s", PIN);
}

void BT_setServiceClass(char *class)
{
  setSvcClassRequest = 1;
  snprintf(newSvcClass, 5, "%s", class);
}

void BT_setServiceName(char *name)
{
  setSvcNameRequest = 1;
  snprintf(newSvcName, 5, "%s", name);
}

void BT_setDeviceClass(char *class)
{
  setDevClassRequest = 1;
  snprintf(newDevClass, 5, "%s", class);
}

void BT_rn4xDisableRemoteConfig(uint8_t disableRemoteConfig)
{
  if (disableRemoteConfig)
  {
    BT_setRn4xRemoteConfigurationTimer("0");
  }
  else
  {
    BT_setRn4xRemoteConfigurationTimer("255");
  }
}

void BT_setUpdateBaudDuringBoot(uint8_t val)
{
  updateBaudDuringBoot = val;
}

//Sets the Paging Scan Window - amount of time device
//spends enabling page scan (connectability).
//Minimum = (hex word) "0012", corresponding to about 1% duty cycle.
//Maximum = (hex word) "1000"
void BT_setPagingTime(char *hexval_time)
{
  snprintf(pagingTime, sizeof(pagingTime), "%s", hexval_time);
}

//Sets the Inquiry Scan Window - amount of time device
//spends enabling inquiry scan (discoverability).
//Minimum = (hex word) "0012", corresponding to about 1% duty cycle.
//Maximum = (hex word) "1000"
void BT_setInquiryScanWindow(char *hexval_time)
{
  snprintf(inquiryScanWindow, sizeof(inquiryScanWindow), "%s", hexval_time);
}

void BT_setRn4678FastMode(char *hexval_time)
{
  snprintf(rn4678FastMode, sizeof(rn4678FastMode), "%s", hexval_time);
}

/* Sets the connection parameters for BLE connection
 Field 1 = Minimum connection interval
 Field 2 = Maximum connection interval
 Field 3 = Slave latency
 Field 4 = Supervision timeout
 */
void BT_setRn4678BleConnectionParameters(char *hexval_time)
{
  snprintf(rn4678BleConnectionParameters, sizeof(rn4678BleConnectionParameters),
      "%s", hexval_time);
}

void BT_setRn4678BleCompleteLocalName(char *hexval_name)
{
  string2hexString(hexval_name, bleCompleteLocalName);
}

const char *BT_getDesiredRnTxPowerForBtVerSetCmd(void)
{
  if (isBtDeviceRn4678())
  {
    return rn4678TxPower_str[rn4678TxPower];
  }
  /* Note: Leading zeros are not returned for 'GY' in the RN4X */
  else if (btFwVer == RN41_V4_77 || btFwVer == RN42_V4_77)
  {
    return rn42TxPowerPreAug2012_set_str[rn42TxPowerPreAug2012];
  }
  else
  {
    return rn42PostAug2012TxPower_set_str[rn42TxPowerPostAug2012];
  }
}

const char *BT_getDesiredRnTxPowerForBtVerGetCmd(void)
{
  if (isBtDeviceRn4678())
  {
    return rn4678TxPower_str[rn4678TxPower];
  }
  /* Note: Leading zeros are not returned for 'GY' in the RN4X */
  else if (btFwVer == RN41_V4_77 || btFwVer == RN42_V4_77)
  {
    return rn42TxPowerPreAug2012_get_str[rn42TxPowerPreAug2012];
  }
  else
  {
    return rn42PostAug2012TxPower_get_str[rn42TxPowerPostAug2012];
  }
}

//https://microchipsupport.force.com/s/article/Wireless-SY-0-4--command-clarifications-for-RN4678
void BT_setRn4678TxPower(rn4678TxPower_et newValue)
{
  rn4678TxPower = newValue;
}

void BT_setRn42TxPowerPreAug2012(rn42TxPowerPreAug2012_et newValue)
{
  rn42TxPowerPreAug2012 = newValue;
}

void BT_setRn42TxPowerPostAug2012(rn42TxPowerPostAug2012_et newValue)
{
  rn42TxPowerPostAug2012 = newValue;
}

/* Overrides weak function in LogAndStream driver.
 * Set RN4678 to use dual Bluetooth mode
 * 0 = Dual mode
 * 1 = Bluetooth Low Energy only
 * 2 = Bluetooth Classic only */
void BT_setBtMode(uint8_t btClassicEn, uint8_t bleEn)
{
  //Handle both enabled and also treat the same if both disabled
  if ((btClassicEn && bleEn) || (!btClassicEn && !bleEn))
  {
    sprintf(btMode, "0");
  }
  else if (bleEn)
  {
    sprintf(btMode, "1");
  }
  else if (btClassicEn)
  {
    sprintf(btMode, "2");
  }
}

void BT_setRn4xRemoteConfigurationTimer(char *hexval_time)
{
  snprintf(rn4xRemoteConfigurationTimer, sizeof(rn4xRemoteConfigurationTimer),
      "%s", hexval_time);
}

void BT_resetDefaults(void)
{
  resetDefaultsRequest = 1;
}

void BT_receiveFunction(uint8_t (*receiveFuncPtr)(uint8_t data))
{
  dataAvailableFuncPtr = receiveFuncPtr;
}

void BT_rtsInterrupt(uint8_t value)
{
  txOverflow = value;
#if BT_FLUSH_TX_BUF_IF_RN4678_RTS_LOCK_DETECTED
  if (!txOverflow)
  {
    rn4678RtsLockDetected = 0U;
  }
#endif

  if (txOverflow)
  { //in disabling sending
    txie_reg = UCA1IE & UCTXIE;
    UCA1IE &= ~UCTXIE;
    ShimBt_btTxInProgressSet(0); //false
  }
  else
  { //in resuming sending
    if (txie_reg)
    {
      UCA1IE |= UCTXIE;
    }
    ShimBt_sendNextCharIfNotInProgress();
  }
}

void BT_rst_MessageProgress(void)
{
  //messageInProgress = 0;
  clearExpectedResponseBuf();

  command_received = 0;
  bt_setbaudrate_step = 0;
  bt_setcommands_step = WAIT_FOR_BOOT;
  bt_setcommands_start = 0;
  bt_runmastercommands_step = 0;
  bt_runmastercommands_start = 0;
}

uint8_t *BT_getExpResp(void)
{
  return (uint8_t *) expectedCommandResponse;
}

uint8_t BT_getWaitForInitialBoot(void)
{
  return waitForInitialBoot;
}

void BT_setWaitForStartCmd(uint8_t val)
{
  waitForStartCmd = val;
}

uint8_t BT_getWaitForStartCmd(void)
{
  return waitForStartCmd;
}

void BT_setWaitForMacAddress(uint8_t val)
{
  waitForMacAddress = val;
}

uint8_t BT_getWaitForMacAddress(void)
{
  return waitForMacAddress;
}

void BT_setWaitForVersion(uint8_t val)
{
  waitForBtFwVersion = val;
}

uint8_t BT_getWaitForVersion(void)
{
  return waitForBtFwVersion;
}

uint8_t BT_getWaitForReturnNewLine(void)
{
  return waitForReturnNewLine;
}

void setDmaWaitingForResponseIfStatusStrEnabled(void)
{
  if (areBtStatusStringsEnabled())
  {
    setDmaWaitingForResponse(1U);
  }
}

void setDmaWaitingForResponseIfStatusStrDisabled(void)
{
  if (!areBtStatusStringsEnabled())
  {
    setDmaWaitingForResponse(1U);
  }
}

void setDmaWaitingForResponse(uint16_t numChars)
{
  DMA2SZ = numChars;
  DMA2_enable();
#if BT_CTS_CONTROL_ENABLED
  setIsBtClearToSend(1);
#endif
}

void setDmaWaitForReturnNewLine(void)
{
  BT_setWaitForReturnNewLine(1);
  setDmaWaitingForResponse(3U);
}

void DMA2AndCtsDisable(void)
{
#if BT_CTS_CONTROL_ENABLED
  setIsBtClearToSend(0);
#endif
  DMA2_disable();
}

//RADIO_CTS
void setIsBtClearToSend(uint8_t isBtClearToSend)
{
  /* Set the BT module's CTS high to pause data from the module */
  if (isBtClearToSend)
  {
    P2OUT &= ~BIT2;
  }
  else
  {
    P2OUT |= BIT2;
  }
}

//RADIO_RTS interrupt: RTS raises when BT has trans overflow
void setBtRtsInterruptState(uint8_t isEnabled)
{
  if (isEnabled)
  {
    P1IES &= ~BIT3; //can assume initially low as module is in reset so watch for low to high transition
    P1IFG &= ~BIT3; //clear flag
    P1IE |= BIT3;   //enable interrupt
  }
  else
  {
    P1IE &= ~BIT3; //disable RTS interrupt
  }
}

void setBtConnectionStatusInterruptIsEnabled(uint8_t isEnabled)
{
  if (isEnabled)
  {
    updateBtConnectionStatusInterruptDirection();
    P1IFG &= ~BIT0; //clear flag
    P1IE |= BIT0;   //enable interrupt
  }
  else
  {
    P1IE &= ~BIT0; //disable Connection interrupt
  }
}

void updateBtConnectionStatusInterruptDirection(void)
{
  if (P1IN & BIT0)
  {
    P1IES |= BIT0; //look for falling edge
  }
  else
  {
    P1IES &= ~BIT0; //look for rising edge
  }
}

void setBtModuleReset(uint8_t isResetHeld)
{
  /* Active low */
  if (isResetHeld)
  {
    P4OUT &= ~BIT4;
    setRnCommandModeActive(0);
  }
  else
  {
    P4OUT |= BIT4;
  }
}

void setBtModulePower(uint8_t isEnabled)
{
  if (isEnabled)
  {
    P4OUT |= BIT3;

    if (rn4678OpMode != RN4678_OP_MODE_NOT_USED)
    {
      setRn4678OperationalModePins(rn4678OpMode);
    }
  }
  else
  {
    P4OUT &= ~BIT3;

    if (rn4678OpMode != RN4678_OP_MODE_NOT_USED)
    {
      setRn4678OperationalModePins(RN4678_OP_MODE_DISABLE);
    }
  }
  shimmerStatus.btPowerOn = isEnabled;
}

uint8_t isBtDeviceUnknown(void)
{
  return (btFwVer == BT_FW_VER_UNKNOWN);
}

uint8_t isBtDeviceRn41orRN42(void)
{
  return (isBtDeviceRn41() || isBtDeviceRn42());
}

uint8_t isBtDeviceRn41(void)
{
  return (btFwVer == RN41_V4_77);
}

uint8_t isBtDeviceRn42(void)
{
  return (btFwVer == RN42_V4_77 || btFwVer == RN42_V6_15 || btFwVer == RN42_V6_30);
}

uint8_t isBtDeviceRn4678(void)
{
  return (btFwVer == RN4678_V1_00_5 || btFwVer == RN4678_V1_11_0 || btFwVer == RN4678_V1_13_5
      || btFwVer == RN4678_V1_22_0 || btFwVer == RN4678_V1_23_0);
}

uint8_t doesBtDeviceSupport1Mbps(void)
{
  /* Bug with RN4678 v1.13.5 in-which 1Mbps doesn't work so not adding it
   * here */
  return (btFwVer == RN4678_V1_22_0 || btFwVer == RN4678_V1_23_0);
}

void setBtFwVersion(enum BT_FIRMWARE_VERSION btFwVerNew)
{
  btFwVer = btFwVerNew;

  /* Write function needs to be updated depending BT FW version */
  updateBtWriteFunctionPtr();

  if (isBtDeviceRn41orRN42())
  {
    //Only RN4678 supports BLE
    BT_setBtMode(1, 0);
  }
}

enum BT_FIRMWARE_VERSION getBtFwVersion(void)
{
  return btFwVer;
}

void updateBtWriteFunctionPtr(void)
{
  if (isRn4678ConnectionBle())
  {
    ShimBt_writeToTxBufAndSend = &BT_write_rn4678_ble;
  }
  else if (doesBtDeviceSupport1Mbps())
  {
    ShimBt_writeToTxBufAndSend = &BT_write_rn4678_1M;
  }
  else if (isBtDeviceRn4678())
  {
    ShimBt_writeToTxBufAndSend = &BT_write_rn4678_460800;
  }
  else
  {
    ShimBt_writeToTxBufAndSend = &BT_write_rn42;
  }
}

void setBtRxFullResponsePtr(volatile char *ptr)
{
  btRxFullResponse = ptr;
}

uint8_t areBtStatusStringsEnabled(void)
{
  return btStatusStringsAreEnabled;
}

void setRnCommandModeActive(uint8_t state)
{
  command_mode_active = state;
}

uint8_t isRnCommandModeActive(void)
{
  return command_mode_active;
}

char *getTxCmdBufPtr(void)
{
  return &commandbuf[0];
}

void clearBtCmdBuf(void)
{
  memset(commandbuf, 0, sizeof(commandbuf));
}

void clearExpectedResponseBuf(void)
{
  //*expectedCommandResponse = '\0';   // NULL pointer
  memset(expectedCommandResponse, 0x00,
      sizeof(expectedCommandResponse) / sizeof(expectedCommandResponse[0]));
}

void setRn4678OperationalMode(rn4678OperationalMode mode)
{
  rn4678OpMode = mode;
}

void setRn4678OperationalModePins(rn4678OperationalMode mode)
{
  switch (mode)
  {
    case RN4678_OP_MODE_WRITE_FLASH:
      P4OUT &= ~BIT6; //P2_0
      P4OUT &= ~BIT7; //P2_4
      P8OUT |= BIT5;  //EAN
      break;
    case RN4678_OP_MODE_WRITE_EEPROM_AND_TEST:
      P4OUT &= ~BIT6; //P2_0
      P4OUT |= BIT7;  //P2_4
      P8OUT &= ~BIT5; //EAN
      break;
    case RN4678_OP_MODE_APPLICATION:
      P4OUT |= BIT6;  //P2_0
      P4OUT |= BIT7;  //P2_4
      P8OUT &= ~BIT5; //EAN
      break;
    case RN4678_OP_MODE_DISABLE:
      P4OUT &= ~BIT6; //P2_0
      P4OUT &= ~BIT7; //P2_4
      P8OUT &= ~BIT5; //EAN
      break;
    default:
      break;
  }
}

uint8_t isRn4678ConnectionEstablished(void)
{
  return (rn4678ConnectionState != RN4678_DISCONNECTED);
}

uint8_t isRn4678ConnectionBle(void)
{
  return (rn4678ConnectionState == RN4678_CONNECTED_BLE);
}

void setRn4678ConnectionState(rn4678ConnectionType state)
{
  rn4678ConnectionState = state;

  /* Write function needs to be updated depending on whether BLE is connected
   * or not in order to optimise how many bytes are transferred per BLE
   * packet */
  updateBtWriteFunctionPtr();
}

void calculateClassicBtTxSampleSetBufferSize(uint8_t len, uint16_t samplingRateTicks)
{
  uint8_t sampleSetsToBuffer = 1U;
  /* If sampling rate is <= 256 Hz (i.e., 32768Hz/256Hz = 128ticks), use double buffer*/
  if (samplingRateTicks >= 128U)
  {
    sampleSetsToBuffer = 2U;
  }
  /* Len + 1U for DATA_PACKET bytes header.
   * *sampleSetsToBuffer for X number of sample sets in buffer */
  /* Note: this doesn't include CRC bytes*/
  rn4678ClassicBtSampleSetBufferSize = ((len + 1U) * sampleSetsToBuffer);
}

uint8_t getDefaultBaudForBtVersion(void)
{
  if (shimmerStatus.btInSyncMode)
  {
    /* SDLog sync has significant difficulty using higher bauds for RN4678 BT
     * modules while trying to SD log and sync at the same time due to missing
     * status string bytes - especially when set to 1000000 baud. Sync seems
     * very stable at 115200 baud. */
    return BAUD_115200;
  }
  else
  {
    return (doesBtDeviceSupport1Mbps() ? BAUD_1000000 : BAUD_460800);
  }
}

void setBleDeviceInformation(char *daughtCardIdStrPtr,
    uint8_t fwVerMajorNew,
    uint8_t fwVerMinorNew,
    uint8_t fwVerRelNew)
{
  daughtCardIdStrPtrForBle = daughtCardIdStrPtr;
  sprintf(rn4678BleManufacturer, "Shimmer\0");
  /* Assumes major = 1 char and minor = 2 char*/
  sprintf(rn4678BleSwRevision, "%d.%d\0", fwVerMajorNew, fwVerMinorNew);
}

uint8_t parseRnGetResponse(char *cmdPtr, char *responsePtr)
{
  uint8_t responseParsed = 1;

  doesBtConfigNeedUpdating = 0;

  switch (*cmdPtr)
  {
    case 'A':
      checkAuth(responsePtr);
      break;
      //TODO
      //case 'B':
      //    /* Get MAC address */
      //    setMacId_cb(unwrappedResponse);
      //    responseParsed = 1;
      //    break
      //}
    case 'O':
      checkStatusStrPrefixAndSuffix(responsePtr);
      break;
    case 'M':
      checkOperatingMode(responsePtr);
      break;
    case 'I':
      checkInquiryScanWindow(responsePtr);
      break;
    case 'J':
      checkPagingTime(responsePtr);
      break;
    case 'G':
      checkBtmode(responsePtr);
      break;
    case 'Q':
      checkFastMode(responsePtr);
      break;
    case 'Y':
      checkTransmitPower(responsePtr);
      break;
    case 'T':
      if (isBtDeviceRn41orRN42())
      {
        checkRn4xRemoteConfigTimer(responsePtr);
      }
      else if (isBtDeviceRn4678())
      {
        checkBleConnectionParameters(responsePtr);
      }
      else
      {
        responseParsed = 0;
      }
      break;
    case 'D':
      if (*(cmdPtr + 1) == 'M')
      {
        checkModelString(responsePtr);
      }
      else if (*(cmdPtr + 1) == 'N')
      {
        checkManufacturerString(responsePtr);
      }
      else if (*(cmdPtr + 1) == 'R')
      {
        checkSoftwareRevision(responsePtr);
      }
      else
      {
        responseParsed = 0;
      }
      break;
    case 'N':
      checkAdvertisingName(responsePtr);
      break;
    case 'P':
      checkPin(responsePtr);
      break;
    default:
      responseParsed = 0;
      break;
  }

  if (doesBtConfigNeedUpdating)
  {
    setRebootRequired(1);
  }

  return responseParsed;
}

void setRebootRequired(uint8_t state)
{
  /* Noticed an issue with RN42 v4.77 where the authentication setting would
   * reset to 0 on every reboot even when we change it and this presents
   * issues when trying to connect to the sensors. We don't know if this is a
   * problem with other commands so the safest thing is to ignore doing a
   * reboot for RN42 v4.77 based sensors. */
  if (btFwVer == RN42_V4_77)
  {
    btRebootRequired = 0;
  }
  else
  {
    btRebootRequired = state;
  }
}

void checkAuth(char *rxBufPtr)
{
  char checkBuf[1] = { 0 };
  sprintf(checkBuf, "%d", authenticate);
  doesBtConfigNeedUpdating = rxBufPtr[0] == checkBuf[0] ? 0 : 1;
}

void checkStatusStrPrefixAndSuffix(char *rxBufPtr)
{
  char checkBuf[6] = { 0 };

  if (areBtStatusStringsEnabled())
  {
    if (isBtDeviceRn41orRN42())
    {
      sprintf(checkBuf, "%s", "%\r\n");
    }
    else
    {
      sprintf(checkBuf, "%s", "%,%\r\n");
    }
  }
  else
  {
    sprintf(checkBuf, "%s", "NULL\r\n");
  }

  doesBtConfigNeedUpdating = strstr(rxBufPtr, checkBuf) ? 0 : 1;
}

void checkOperatingMode(char *rxBufPtr)
{
  char checkBuf[5] = { 0 };
  if (isBtDeviceRn41orRN42())
  {
    if (radioMode == SLAVE_MODE)
    {
      sprintf(checkBuf, "Slav");
    }
    else if (radioMode == MASTER_MODE)
    {
      sprintf(checkBuf, "Mstr");
    }
    else if (radioMode == TRIGGER_MODE)
    {
      sprintf(checkBuf, "Trig");
    }
    else if (radioMode == AUTO_CONNECT_MASTER_MODE)
    {
      sprintf(checkBuf, "Auto");
    }
    else if (radioMode == AUTO_CONNECT_DTR_MODE)
    {
      sprintf(checkBuf, "DTR ");
    }
    else if (radioMode == AUTO_CONNECT_ANY_MODE)
    {
      sprintf(checkBuf, "Any ");
    }
    else if (radioMode == PAIRING_MODE)
    {
      sprintf(checkBuf, "Pair");
    }
    else
    {
      sprintf(checkBuf, "FFFF");
      return;
    }
    doesBtConfigNeedUpdating = strstr(rxBufPtr, checkBuf) ? 0 : 1;
  }
  else
  {
    sprintf(checkBuf, "%d", radioMode);
    doesBtConfigNeedUpdating = rxBufPtr[0] == checkBuf[0] ? 0 : 1;
  }
}

void checkPagingTime(char *rxBufPtr)
{
  doesBtConfigNeedUpdating = strstr(rxBufPtr, pagingTime) ? 0 : 1;
}

void checkBtmode(char *rxBufPtr)
{
  doesBtConfigNeedUpdating = rxBufPtr[0] == btMode[0] ? 0 : 1;
}

void checkFastMode(char *rxBufPtr)
{
  doesBtConfigNeedUpdating = strstr(rxBufPtr, rn4678FastMode) ? 0 : 1;
}

void checkTransmitPower(char *rxBufPtr)
{
  if (isBtDeviceRn4678())
  {
    doesBtConfigNeedUpdating
        = strstr(rxBufPtr, BT_getDesiredRnTxPowerForBtVerGetCmd()) ? 0 : 1;
  }
  else
  {
    doesBtConfigNeedUpdating
        = strcmp(rxBufPtr, BT_getDesiredRnTxPowerForBtVerGetCmd()) ? 1 : 0;
  }
}

void checkBleConnectionParameters(char *rxBufPtr)
{
  doesBtConfigNeedUpdating = strstr(rxBufPtr, rn4678BleConnectionParameters) ? 0 : 1;
}

void checkInquiryScanWindow(char *rxBufPtr)
{
  doesBtConfigNeedUpdating = strstr(rxBufPtr, inquiryScanWindow) ? 0 : 1;
}

void checkManufacturerString(char *rxBufPtr)
{
  doesBtConfigNeedUpdating = strstr(rxBufPtr, rn4678BleManufacturer) ? 0 : 1;
}

void checkModelString(char *rxBufPtr)
{
  doesBtConfigNeedUpdating = strstr(rxBufPtr, daughtCardIdStrPtrForBle) ? 0 : 1;
}

void checkSoftwareRevision(char *rxBufPtr)
{
  doesBtConfigNeedUpdating = strstr(rxBufPtr, rn4678BleSwRevision) ? 0 : 1;
}

void checkRn4xRemoteConfigTimer(char *rxBufPtr)
{
  doesBtConfigNeedUpdating = strstr(rxBufPtr, rn4xRemoteConfigurationTimer) ? 0 : 1;
}

void checkAdvertisingName(char *rxBufPtr)
{
  if (useSpecificAdvertisingName)
  {
    doesBtConfigNeedUpdating = strstr(rxBufPtr, advertisingName) ? 0 : 1;
  }
  else
  {
    doesBtConfigNeedUpdating = (strstr(rxBufPtr, advertisingName)
                                   || strstr(rxBufPtr, ADVERTISING_NAME_OUTPUT)
                                   || strstr(rxBufPtr, ADVERTISING_NAME_SHIMMER3)) ?
        0 :
        1;
  }
}

void checkPin(char *rxBufPtr)
{
  doesBtConfigNeedUpdating = strstr(rxBufPtr, pinCode) ? 0 : 1;
}

void string2hexString(char *input, char *output)
{
  uint8_t loop;
  uint8_t i;

  i = 0;
  loop = 0;

  while (input[loop] != '\0')
  {
    *(output + i++) = hex[(uint8_t) ((input[loop] >> 4) & 0xF)];
    *(output + i++) = hex[(uint8_t) (input[loop] & 0xF)];
    loop += 1;
  }
  //insert NULL at the end of the output string
  output[i++] = '\0';
}

#pragma vector = USCI_A1_VECTOR

__interrupt void USCI_A1_ISR(void)
{
  switch (__even_in_range(UCA1IV, 4))
  {
    case 0:
      break; //Vector 0 - no interrupt
    case 2:  //Vector 2 - RXIFG
      break;
    case 4: //Vector 4 - TXIFG
      ShimBt_TxCpltCallback();
      break;
    default:
      break;
  }
}
