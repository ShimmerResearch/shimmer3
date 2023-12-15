/*
 * shimmer_bt_comms.c
 *
 *  Created on: 22 Jun 2022
 *      Author: MarkNolan
 */

#include "shimmer_bt_comms.h"
#include <stdio.h>
#include <string.h>

#include "msp430.h"

#include "RN4X.h"
#include "../5xx_HAL/hal_board.h"
#include "../5xx_HAL/hal_RTC.h"
#include "../5xx_HAL/hal_CRC.h"
#include "../../shimmer_btsd.h"
#if BT_DMA_USED_FOR_RX
#include "../5xx_HAL/hal_DMA.h"
#endif
#include "sd_sync.h"

uint8_t unwrappedResponse[256] = {0};
char *commandBufPtr;
uint8_t *expectedResponsePtr;
#if BT_DMA_USED_FOR_RX
uint8_t waitingForArgs, waitingForArgsLength, argsSize;
volatile uint8_t btStatusStrIndex;

volatile char btStatusStr[BT_STAT_STR_LEN_LARGEST+1U]; /* +1 to always have a null char */
volatile char btRxBuffFullResponse[BT_VER_RESPONSE_LARGEST+1U]; /* +1 to always have a null char */
uint8_t btRxBuff[MAX_COMMAND_ARG_SIZE], *btRxExp;
#else
RingFifoRx_t *gBtRxFifoPtr;
/* isRn4678CmdDetectedOnBoot is a workaround to know if it's RN42 or RN4678 before version is parsed */
uint8_t isRn4678CmdDetectedOnBoot = 0;
#endif
volatile COMMS_CRC_MODE btCrcMode;

uint8_t *gActionPtr;
uint8_t *gArgsPtr;

volatile char btVerStrResponse[BT_VER_RESPONSE_LARGEST+1U]; /* +1 to always have a null char */

uint8_t (*newBtCmdToProcess_cb)(void);
void (*handleBtRfCommStateChange_cb)(uint8_t);
void (*setMacId_cb)(uint8_t *);

uint16_t numBytesInBtRxBufWhenLastProcessed = 0;
uint16_t indexOfFirstEol;
uint32_t firstProcessFailTicks = 0;

#if BT_DMA_USED_FOR_RX
/* Return of 1 brings MSP out of low-power mode */
uint8_t Dma2ConversionDone(void)
{
    uint8_t bt_waitForStartCmd, bt_waitForMacAddress, bt_waitForVersion, bt_waitForInitialBoot, bt_waitForReturnNewLine;
    uint8_t expectedlen = 0U;

    DMA2AndCtsDisable();
    bt_waitForStartCmd = BT_getWaitForStartCmd();
    bt_waitForMacAddress = BT_getWaitForMacAddress();
    bt_waitForVersion = BT_getWaitForVersion();
    bt_waitForInitialBoot = BT_getWaitForInitialBoot();
    bt_waitForReturnNewLine = BT_getWaitForReturnNewLine();

    if (!*btRxExp
            && (areBtStatusStringsEnabled()
                    || (isBtConnected() || bt_waitForStartCmd || bt_waitForMacAddress || bt_waitForVersion || bt_waitForInitialBoot || bt_waitForReturnNewLine)))
    {
        if (bt_waitForStartCmd)
        {
            /* RN42 responds with "CMD\r\n" and RN4678 with "CMD> " */
            uint8_t len = strlen((char*) btRxBuff);
            if (len == 5U
                    && btRxBuff[0] == 'C'
                    && btRxBuff[1] == 'M'
                    && btRxBuff[2] == 'D')
            {
                BT_setWaitForStartCmd(0);
                setCommandModeActive(1U);
                memset(btRxBuff, 0, len);
                BT_setGoodCommand();
            }
        }
        else if (bt_waitForReturnNewLine)
        {
            uint8_t btOffset = strlen(btRxBuffFullResponse);
            memcpy(btRxBuffFullResponse + btOffset, btRxBuff, strlen((char*) btRxBuff));
            memset(btRxBuff, 0, strlen((char*) btRxBuff));

            uint8_t responseLen = strlen(btRxBuffFullResponse);

            if(btRxBuffFullResponse[responseLen-1U]=='\r')
            {
                /* Wait for "\n" */
                setDmaWaitingForResponse(1U);
                return 0;
            }
            // Wait for RN4678_CMD
            else if(isBtDeviceRn4678()
                    && responseLen > RN4X_CMD_LEN
                    && btRxBuffFullResponse[responseLen-5U]=='C'
                    && btRxBuffFullResponse[responseLen-4U]=='M'
                    && btRxBuffFullResponse[responseLen-3U]=='D'
                    && btRxBuffFullResponse[responseLen-2U]=='>'
                    && btRxBuffFullResponse[responseLen-1U]==' ')
            {
                /* Return/New line received */
                BT_setWaitForReturnNewLine(0);
                BT_setGoodCommand();
            }
            else if(btRxBuffFullResponse[responseLen-2U]=='\r'
                    && btRxBuffFullResponse[responseLen-1U]=='\n')
            {
                if (isBtDeviceRn41orRN42())
                {
                    BT_setWaitForReturnNewLine(0);
                    BT_setGoodCommand();
                }
                else
                {
                    /* Wait for "CMD> " */
                    setDmaWaitingForResponse(RN4X_CMD_LEN);
                    return 0;
                }
            }
            else
            {
                /* Wait for "\r\n" */
                setDmaWaitingForResponse(2U);
                return 0;
            }
        }
        else if (bt_waitForInitialBoot)
        {
            if (isBtDeviceRn4678())
            {
                expectedlen = BT_STAT_STR_LEN_RN4678_REBOOT;
            }
            else
            {
                expectedlen = BT_STAT_STR_LEN_RN42_REBOOT;
            }
            memset(btRxBuff, 0, expectedlen);
            BT_setWaitForInitialBoot(0);
            BT_setGoodCommand();
        }
        else if (bt_waitForMacAddress)
        {
            setMacId_cb(btRxBuff);

            expectedlen = 14U;
            if (isBtDeviceRn4678())
            {
                expectedlen += RN4X_CMD_LEN; /* Allow for "CMD> " */
            }
            memset(btRxBuff, 0, expectedlen);
            BT_setWaitForMacAddress(0);
            BT_setGoodCommand();
        }
        else if (bt_waitForVersion)
        {
            uint8_t btVerRemainingChars = 0;
            uint8_t btOffset = strlen(btRxBuffFullResponse);
            memcpy(btRxBuffFullResponse + btOffset, btRxBuff, strlen((char*) btRxBuff));
            memset(btRxBuff, 0, strlen((char*) btRxBuff));

            uint8_t btVerLen = strlen(btRxBuffFullResponse);
            enum BT_FIRMWARE_VERSION btFwVerNew = BT_FW_VER_UNKNOWN;

            /* RN41 or RN42 */
            if (btRxBuffFullResponse[0U]=='V')
            {
                /* RN41_VERSION_RESPONSE_V4_77 or RN42_VERSION_RESPONSE_V4_77 */
                if (btRxBuffFullResponse[4U]=='4' && btRxBuffFullResponse[5U]=='.')
                {
                    if (btRxBuffFullResponse[9U]=='R' && btRxBuffFullResponse[10U]=='N')
                    {
                        btVerRemainingChars = RN42_VERSION_RESPONSE_LEN_V4_77;
                        btFwVerNew = RN42_V4_77;
                    }
                    else
                    {
                        btVerRemainingChars = RN41_VERSION_RESPONSE_LEN_V4_77;
                        btFwVerNew = RN41_V4_77;
                    }
                }
                /* RN42_VERSION_RESPONSE_V6_15 */
                else if (btRxBuffFullResponse[4U]=='6' && btRxBuffFullResponse[5U]=='.' && btRxBuffFullResponse[6U]=='1' && btRxBuffFullResponse[7U]=='5')
                {
                    btVerRemainingChars = RN42_VERSION_RESPONSE_LEN_V6_15;
                    btFwVerNew = RN42_V6_15;
                }
                /* V6.30 not supported */
                else if (btRxBuffFullResponse[4U]=='6' && btRxBuffFullResponse[5U]=='.' && btRxBuffFullResponse[6U]=='3' && btRxBuffFullResponse[7U]=='0')
                {
                    triggerShimmerErrorState();
                }
            }
            /* RN4678 */
            else if (btRxBuffFullResponse[0U]=='R')
            {
                /* RN4678_VERSION_RESPONSE_V1_00_5 */
                if (btRxBuffFullResponse[10U]=='0' && btRxBuffFullResponse[11U]=='0')
                {
                    btVerRemainingChars = RN4678_VERSION_LEN_V1_00_5;
                    btFwVerNew = RN4678_V1_00_5;
                }
                /* RN4678_VERSION_RESPONSE_V1_11_0 */
                else if (btRxBuffFullResponse[10U]=='1' && btRxBuffFullResponse[11U]=='1')
                {
                    btVerRemainingChars = RN4678_VERSION_LEN_V1_11_0;
                    btFwVerNew = RN4678_V1_11_0;
                }
                /* RN4678_VERSION_RESPONSE_V1_13_5 */
                else if (btRxBuffFullResponse[10U]=='1' && btRxBuffFullResponse[11U]=='3')
                {
                    btVerRemainingChars = RN4678_VERSION_LEN_V1_13_5;
                    btFwVerNew = RN4678_V1_13_5;
                }
                /* RN4678_VERSION_RESPONSE_V1_22_0 */
                else if (btRxBuffFullResponse[10U]=='2' && btRxBuffFullResponse[11U]=='2')
                {
                    btVerRemainingChars = RN4678_VERSION_LEN_V1_22_0;
                    btFwVerNew = RN4678_V1_22_0;
                }
                /* RN4678_VERSION_RESPONSE_V1_23_0 */
                else if (btRxBuffFullResponse[10U]=='2' && btRxBuffFullResponse[11U]=='3')
                {
                    btVerRemainingChars = RN4678_VERSION_LEN_V1_23_0;
                    btFwVerNew = RN4678_V1_23_0;
                }
            }
            else
            {
                /* Unkown BT module - bail */
                btVerRemainingChars = btVerLen;
            }

            btVerRemainingChars -= btVerLen;

            if(btVerRemainingChars)
            {
                setDmaWaitingForResponse(btVerRemainingChars);
                return 0;
            }
            else
            {
                setBtFwVersion(btFwVerNew);

                /* When storing the BT version, ignore from "\r" onwards */
                uint8_t btVerLen = strlen(btRxBuffFullResponse);
                uint8_t btVerIdx;
                for (btVerIdx = 0; btVerIdx < btVerLen; btVerIdx++)
                {
                    if (btRxBuffFullResponse[btVerIdx] == '\r')
                    {
                        btVerLen = btVerIdx;
                        break;
                    }
                }
                memcpy(btVerStrResponse, btRxBuffFullResponse, btVerLen);

                memset(btRxBuffFullResponse, 0, strlen((char*) btRxBuffFullResponse));
                BT_setWaitForVersion(0);
                BT_setGoodCommand();
            }
        }
#if USE_OLD_SD_SYNC_APPROACH
        else if (getRcommVar())
        {
            /* SD Sync Center - get's into this case when the center is waiting for a 0x01 or 0xFF from a node */
            // 1 byte of RC command
            setRcommResp(btRxBuff, 1U);
            setRcommVar(0);
            setDmaWaitingForResponse(1U);
            return TaskSet(TASK_RCCENTERR1);
        }
#endif
        else
        {
            if (waitingForArgs)
            {
                if ((!waitingForArgsLength)
                        && ((*(gActionPtr) == SET_EXG_REGS_COMMAND)
                                && (waitingForArgs == 3)))
                {
                    gArgsPtr[0] = btRxBuff[0];
                    gArgsPtr[1] = btRxBuff[1];
                    gArgsPtr[2] = btRxBuff[2];
                    waitingForArgsLength = gArgsPtr[2];
                    setDmaWaitingForResponse(waitingForArgsLength);
                    return 0;
                }
                else if ((!waitingForArgsLength)
                        && (((*(gActionPtr) == SET_INFOMEM_COMMAND)
                                && (waitingForArgs == 3))
                                || ((*(gActionPtr) == SET_CALIB_DUMP_COMMAND)
                                        && (waitingForArgs == 3))))
                {
                    gArgsPtr[0] = btRxBuff[0];
                    gArgsPtr[1] = btRxBuff[1];
                    gArgsPtr[2] = btRxBuff[2];
                    waitingForArgsLength = gArgsPtr[0];
                    setDmaWaitingForResponse(waitingForArgsLength);
                    return 0;
                }
                else if ((!waitingForArgsLength) && (
                //                ((*(gActionPtr) == SET_DAUGHTER_CARD_ID_COMMAND) && (waitingForArgs == 1)) ||
                        ((*(gActionPtr) == SET_DAUGHTER_CARD_MEM_COMMAND)
                                && (waitingForArgs == 1))
                                || ((*(gActionPtr) == SET_CENTER_COMMAND)
                                        && (waitingForArgs == 1))
                                || ((*(gActionPtr) == SET_CONFIGTIME_COMMAND)
                                        && (waitingForArgs == 1))
                                || ((*(gActionPtr) == SET_EXPID_COMMAND)
                                        && (waitingForArgs == 1))
                                || ((*(gActionPtr) == SET_SHIMMERNAME_COMMAND)
                                        && (waitingForArgs == 1))))
                {
                    gArgsPtr[0] = btRxBuff[0];
                    if (gArgsPtr[0])
                    {
                        waitingForArgsLength = gArgsPtr[0];
                        setDmaWaitingForResponse(waitingForArgsLength);
                        return 0;
                    }
                }
                else if (*(gActionPtr) == RN4678_STATUS_STRING_SEPARATOR)
                {
                    uint8_t numberOfCharRemaining = 0U;
                    uint8_t bringUcOutOfSleep = 0U;

                    memcpy(btStatusStr+btStatusStrIndex, btRxBuff, waitingForArgs);
                    memset(btRxBuff, 0, waitingForArgs);
                    btStatusStrIndex += waitingForArgs;

                    enum BT_FIRMWARE_VERSION btFwVer = getBtFwVersion();

                    uint8_t firstChar = btStatusStr[1U];
                    switch (firstChar)
                    {
                    case 'A':
                        /* "%AUTHENTICATED%" */
                        if (btStatusStr[14U]=='%')
                        {
                            /* TODO */
                        }
                        /* "%AUTHEN" - Read outstanding bytes */
                        else if (btStatusStr[6U]=='N')
                        {
                            numberOfCharRemaining = BT_STAT_STR_LEN_AUTHENTICATED - BT_STAT_STR_LEN_SMALLEST;
                        }
                        /* "%AUTH_FAIL%" */
                        else if (btStatusStr[10U]=='%')
                        {
                            /* TODO */
                        }
                        /* "%AUTH_FA" - Read outstanding bytes */
                        else if (btStatusStr[6U]=='F')
                        {
                            numberOfCharRemaining = BT_STAT_STR_LEN_AUTH_FAIL - BT_STAT_STR_LEN_SMALLEST;
                        }
                        break;
                    case 'B':
                        /* "%BONDED%" */
                        if (btStatusStr[7U]=='%')
                        {
                            /* TODO */
                        }
                        /* "%BONDED" - Read outstanding bytes */
                        else
                        {
                            numberOfCharRemaining = BT_STAT_STR_LEN_BONDED - BT_STAT_STR_LEN_SMALLEST;
                        }
                        break;
                    case 'C':
                        if (btStatusStr[5U]=='E')
                        {
                            /* "%CONNECT,001BDC06A3D5%" - RN4678 */
                            if (btStatusStr[21U]=='%')
                            {
                                setRn4678ConnectionState(RN4678_CONNECTED_CLASSIC);
                            }
                            /* "%CONNECT" for RN42 v4.77 or "%CONNECT,001BDC06A3D5," - RN42 v6.15 */
                            else if ((btFwVer == RN41_V4_77 && btStatusStr[7U]=='T')
                                    || (btFwVer == RN42_V4_77 && btStatusStr[7U]=='T')
                                    || (btFwVer == RN42_V6_15 && btStatusStr[21U]==','))
                            {
                                triggerBtRfCommStateChangeCallback(TRUE);
                                bringUcOutOfSleep = 1U;
                            }
                            else if (btStatusStr[BT_STAT_STR_LEN_SMALLEST]=='\0')
                            {
                                if (btFwVer == RN41_V4_77 || btFwVer == RN42_V4_77)
                                {
                                    numberOfCharRemaining = BT_STAT_STR_LEN_RN42_v477_CONNECT;
                                }
                                else if (btFwVer == RN42_V6_15)
                                {
                                    numberOfCharRemaining = BT_STAT_STR_LEN_RN42_v615_CONNECT;
                                }
                                else
                                {
                                    numberOfCharRemaining = BT_STAT_STR_LEN_RN4678_CONNECT;
                                }
                                numberOfCharRemaining -= BT_STAT_STR_LEN_SMALLEST;
                            }
                        }
                        else if (btStatusStr[5U]=='_')
                        {
                            /* "%CONN_PARAM,000C,0000,03C0%" - RN4678 */
                            if (btStatusStr[26U]=='%')
                            {
                                //TODO
                            }
                            else
                            {
                                numberOfCharRemaining = BT_STAT_STR_LEN_CONN_PARAM - BT_STAT_STR_LEN_SMALLEST;
                            }
                        }
                        break;
                    case 'D':
                        /* "%DISCONN%" -> RN4678 */
                        if (btStatusStr[8U]=='%')
                        {
                            /* This if is needed here for BLE connections as a
                             * disconnect over BLE does not trigger an
                             * RFCOMM_CLOSE status change as it does for classic
                             * Bluetooth connections. */
                            if (isBtConnected())
                            {
                                triggerBtRfCommStateChangeCallback(FALSE);
                                bringUcOutOfSleep = 1U;
                            }

                            setRn4678ConnectionState(RN4678_DISCONNECTED);
                        }
                        /* "%DISCONNECT" -> RN42 */
                        else if (btStatusStr[10U]=='T')
                        {
                            triggerBtRfCommStateChangeCallback(FALSE);
                            bringUcOutOfSleep = 1U;
                        }
                        /* "%DISCON" - Read outstanding bytes */
                        else if (btStatusStr[BT_STAT_STR_LEN_SMALLEST]=='\0')
                        {
                            if (isBtDeviceRn41orRN42())
                            {
                                numberOfCharRemaining = BT_STAT_STR_LEN_RN42_DISCONNECT;
                            }
                            else
                            {
                                numberOfCharRemaining = BT_STAT_STR_LEN_RN4678_DISCONN;
                            }
                            numberOfCharRemaining -= BT_STAT_STR_LEN_SMALLEST;
                        }
                        break;
                    case 'E':
                        if (btStatusStr[2U]=='N')
                        {
                            if (btStatusStr[5U]=='I')
                            {
                                /* "%END_INQ%" */
                                if (btStatusStr[8U]=='%')
                                {
                                    /* TODO */
                                }
                                /* "%END_IN" - Read outstanding bytes */
                                else if (btStatusStr[BT_STAT_STR_LEN_SMALLEST]=='\0')
                                {
                                    numberOfCharRemaining = BT_STAT_STR_LEN_END_INQ - BT_STAT_STR_LEN_SMALLEST;
                                }
                            }
                            else if (btStatusStr[5U]=='S')
                            {
                                /* "%END_SCN%" */
                                if (btStatusStr[8U]=='%')
                                {
                                    /* TODO */
                                }
                                /* "%END_SC" - Read outstanding bytes */
                                else if (btStatusStr[BT_STAT_STR_LEN_SMALLEST]=='\0')
                                {
                                    numberOfCharRemaining = BT_STAT_STR_LEN_END_SCN - BT_STAT_STR_LEN_SMALLEST;
                                }
                            }
                        }
                        else if (btStatusStr[2U]=='R')
                        {
                            if (btStatusStr[5U]=='C')
                            {
                                /* %ERR_CON is common to two status strings. If detected, read two more chars to determine which one it is */
                                /* "%ERR_CONN%" */
                                if (btStatusStr[9U]=='%')
                                {
                                    /* TODO */
                                }
                                /* "%ERR_CONN_PARAM%" */
                                else if (btStatusStr[15U]=='%')
                                {
                                    /* TODO */
                                }
                                /* "%ERR_CONN_" - Read outstanding bytes */
                                else if (btStatusStr[9U]=='_')
                                {
                                    numberOfCharRemaining = BT_STAT_STR_LEN_ERR_CONN_PARAM - BT_STAT_STR_LEN_ERR_CONN;
                                }
                                /* "%ERR_CON" - Read outstanding bytes */
                                else if (btStatusStr[BT_STAT_STR_LEN_SMALLEST]=='\0')
                                {
                                    numberOfCharRemaining = BT_STAT_STR_LEN_ERR_CONN - BT_STAT_STR_LEN_SMALLEST;
                                }
                            }
                            else if (btStatusStr[5U]=='L')
                            {
                                /* "%ERR_LSEC%" */
                                if (btStatusStr[9U]=='%')
                                {
                                    /* TODO */
                                }
                                /* "%ERR_LSE" - Read outstanding bytes */
                                else if (btStatusStr[BT_STAT_STR_LEN_SMALLEST]=='\0')
                                {
                                    numberOfCharRemaining = BT_STAT_STR_LEN_ERR_LSEC - BT_STAT_STR_LEN_SMALLEST;
                                }
                            }
                            else if (btStatusStr[5U]=='S')
                            {
                                /* "%ERR_SEC%" */
                                if (btStatusStr[8U]=='%')
                                {
                                    /* TODO */
                                }
                                /* "%ERR_SE" - Read outstanding bytes */
                                else if (btStatusStr[BT_STAT_STR_LEN_SMALLEST]=='\0')
                                {
                                    numberOfCharRemaining = BT_STAT_STR_LEN_ERR_SEC - BT_STAT_STR_LEN_SMALLEST;
                                }
                            }
                        }
                        break;
                    case 'F':
                        /* "%FACTORY_RESET%" */
                        if (btStatusStr[14U]=='%')
                        {
                            /* TODO */
                        }
                        /* "%FACTOR" - Read outstanding bytes */
                        else if (btStatusStr[BT_STAT_STR_LEN_SMALLEST]=='\0')
                        {
                            numberOfCharRemaining = BT_STAT_STR_LEN_FACTORY_RESET - BT_STAT_STR_LEN_SMALLEST;
                        }
                        break;
                    case 'L':
                        if (btStatusStr[2U]=='C')
                        {
                            /* "%LCONNECT,001BDC06A3D5,1%" - RN4678 BLE mode */
                            if (btStatusStr[24U]=='%')
                            {
                                setRn4678ConnectionState(RN4678_CONNECTED_BLE);

                                /* RN4678 seems to assume charactertic is advice once BLE connected */
                                triggerBtRfCommStateChangeCallback(TRUE);

                                bringUcOutOfSleep = 1U;
                            }
                            else if (btStatusStr[BT_STAT_STR_LEN_SMALLEST]=='\0')
                            {
                                numberOfCharRemaining = BT_STAT_STR_LEN_RN4678_LCONNECT - BT_STAT_STR_LEN_SMALLEST;
                            }
                            break;
                        }
                        else if (btStatusStr[2U]=='B')
                        {
                            /* "%LBONDED%" */
                            if (btStatusStr[8U]=='%')
                            {
                                /* TODO */
                            }
                            /* "%LBONDE" - Read outstanding bytes */
                            else if (btStatusStr[BT_STAT_STR_LEN_SMALLEST]=='\0')
                            {
                                numberOfCharRemaining = BT_STAT_STR_LEN_LBONDED - BT_STAT_STR_LEN_SMALLEST;
                            }
                        }
                        else if (btStatusStr[2U]=='S')
                        {
                            if (btStatusStr[6U]=='R')
                            {
                                /* "%LSECURED%" */
                                if (btStatusStr[9U]=='%')
                                {
                                    /* TODO */
                                }
                                /* "%LSECURE_FAIL%" */
                                else if (btStatusStr[13U]=='%')
                                {
                                    /* TODO */
                                }
                                /* "%LSECURE_F" */
                                else if (btStatusStr[9U]=='F')
                                {
                                    numberOfCharRemaining = BT_STAT_STR_LEN_LSECURE_FAIL - BT_STAT_STR_LEN_LSECURED;
                                }
                                /* "%LSECUR" - Read outstanding bytes */
                                else if (btStatusStr[BT_STAT_STR_LEN_SMALLEST]=='\0')
                                {
                                    numberOfCharRemaining = BT_STAT_STR_LEN_LSECURED - BT_STAT_STR_LEN_SMALLEST;
                                }
                            }
                            else if (btStatusStr[6U]=='A')
                            {
                                /* "%LSTREAM_OPEN%" */
                                if (btStatusStr[13U]=='%')
                                {
                                    /* TODO */
                                }
                                /* "%LSTREA" - Read outstanding bytes */
                                else if (btStatusStr[BT_STAT_STR_LEN_SMALLEST]=='\0')
                                {
                                    numberOfCharRemaining = BT_STAT_STR_LEN_LSTREAM_OPEN - BT_STAT_STR_LEN_SMALLEST;
                                }
                            }
                        }
                        break;
                    case 'M':
                        /* "%MLDP_MODE%" */
                        if (btStatusStr[10U]=='%')
                        {
                            /* TODO */
                        }
                        /* "%MLDP_M" - Read outstanding bytes */
                        else if (btStatusStr[BT_STAT_STR_LEN_SMALLEST]=='\0')
                        {
                            numberOfCharRemaining = BT_STAT_STR_LEN_MLDP_MODE - BT_STAT_STR_LEN_SMALLEST;
                        }
                        break;
                    case 'R':
                        if (btStatusStr[2U]=='E')
                        {
                            /* "%REBOOT%" -> RN4678 */
                            if (btStatusStr[7U]=='%')
                            {
                                /* TODO */
                                _NOP();
                            }
                            else
                            {
                                if (isBtDeviceRn41orRN42())
                                {
                                    numberOfCharRemaining = BT_STAT_STR_LEN_RN42_REBOOT;
                                }
                                else
                                {
                                    numberOfCharRemaining = BT_STAT_STR_LEN_RN4678_REBOOT;
                                }
                                numberOfCharRemaining -= BT_STAT_STR_LEN_SMALLEST;
                            }
                        }
                        else if (btStatusStr[2U]=='F')
                        {
                            if (btStatusStr[8U]=='C')
                            {
                                /* "%RFCOMM_CLOSE%" */
                                if (btStatusStr[13U]=='%')
                                {
                                    triggerBtRfCommStateChangeCallback(FALSE);
                                    bringUcOutOfSleep = 1U;
                                }
                                /* "%RFCOMM_CLOSE" - Read outstanding bytes */
                                else if (btStatusStr[13U]=='\0')
                                {
                                    numberOfCharRemaining = BT_STAT_STR_LEN_RFCOMM_CLOSE - BT_STAT_STR_LEN_RFCOMM_OPEN;
                                }
                            }
                            /* "%RFCOMM_OPEN%" */
                            else if (btStatusStr[12U]=='%')
                            {
                                triggerBtRfCommStateChangeCallback(TRUE);
                                bringUcOutOfSleep = 1U;
                            }
                            /* "%RFCOMM" - Read outstanding bytes */
                            else if (btStatusStr[BT_STAT_STR_LEN_SMALLEST]=='\0')
                            {
                                numberOfCharRemaining = BT_STAT_STR_LEN_RFCOMM_OPEN - BT_STAT_STR_LEN_SMALLEST;
                            }
                        }
                        break;
                    case 'S':
                        if (btStatusStr[3U]=='C')
                        {
                            /* "%SECURED%" */
                            if (btStatusStr[8U]=='%')
                            {
                                /* TODO */
                            }
                            /* "%SECURE_FAIL%" */
                            else if (btStatusStr[12U]=='%')
                            {
                                /* TODO */
                            }
                            /* "%SECURE_F" - Read outstanding bytes */
                            else if (btStatusStr[7U]=='_')
                            {
                                numberOfCharRemaining = BT_STAT_STR_LEN_SECURE_FAIL - BT_STAT_STR_LEN_SECURED;
                            }
                            /* "%SECURE" - Read outstanding bytes */
                            else if (btStatusStr[BT_STAT_STR_LEN_SMALLEST]=='\0')
                            {
                                numberOfCharRemaining = BT_STAT_STR_LEN_SECURED - BT_STAT_STR_LEN_SMALLEST;
                            }
                        }
                        else if (btStatusStr[3U]=='S')
                        {
                            /* "%SESSION_OPEN%" */
                            if (btStatusStr[13U]=='%')
                            {
                                /* TODO */
                            }
                            /* "%SESSION_CLOSE%" */
                            else if (btStatusStr[14U]=='%')
                            {
                                /* TODO */
                            }
                            /* "%SESSION_CLOSE" - Read outstanding bytes */
                            else if (btStatusStr[13U]=='E')
                            {
                                numberOfCharRemaining = BT_STAT_STR_LEN_SESSION_CLOSE - BT_STAT_STR_LEN_SESSION_OPEN;
                            }
                            /* "%SESSIO" - Read outstanding bytes */
                            else if (btStatusStr[BT_STAT_STR_LEN_SMALLEST]=='\0')
                            {
                                numberOfCharRemaining = BT_STAT_STR_LEN_SESSION_OPEN - BT_STAT_STR_LEN_SMALLEST;
                            }
                        }
                        break;
                    default:
                        break;
                    }

                    if (numberOfCharRemaining)
                    {
                        waitingForArgs = numberOfCharRemaining;
                    }
                    else
                    {
                        waitingForArgs = 0;
                        numberOfCharRemaining = 1U;
                    }
                    setDmaWaitingForResponse(numberOfCharRemaining);
                    return bringUcOutOfSleep;
                }
                else if (*(gActionPtr) == ACK_COMMAND_PROCESSED)
                {
#if USE_OLD_SD_SYNC_APPROACH
                    /* Store local time as early as possible after sync bytes have been received */
                    saveLocalTime();
#else
                    /* If waiting for command byte */
                    if(!waitingForArgsLength)
                    {
                        /* Save command byte */
                        gArgsPtr[0] = btRxBuff[0];

                        if (btRxBuff[0] == SD_SYNC_RESPONSE)
                        {
                            /* Wait for flag to be received */
                            waitingForArgsLength = 1U;
                            setDmaWaitingForResponse(waitingForArgsLength);
                            return 0;
                        }
                    }
#endif
                }

                if (waitingForArgsLength)
                {
                    memcpy(gArgsPtr + waitingForArgs, btRxBuff, waitingForArgsLength);
                }
                else
                {
                    memcpy(gArgsPtr, btRxBuff, waitingForArgs);
                }

                waitingForArgsLength = 0;
                waitingForArgs = 0;
                argsSize = 0;
                setDmaWaitingForResponse(1U);
                if(newBtCmdToProcess_cb)
                {
                    return newBtCmdToProcess_cb();
                }
                else
                {
                    return 1;
                }
            }
            else
            {
                uint8_t data = btRxBuff[0];
                uint8_t wakeupMcu = 0;

                switch (data)
                {
#if FW_IS_LOGANDSTREAM
                case INQUIRY_COMMAND:
                case DUMMY_COMMAND:
                case GET_SAMPLING_RATE_COMMAND:
                case TOGGLE_LED_COMMAND:
                case START_STREAMING_COMMAND:
                case GET_STATUS_COMMAND:
                case GET_VBATT_COMMAND:
                case GET_TRIAL_CONFIG_COMMAND:
                case START_SDBT_COMMAND:
                case GET_CONFIG_SETUP_BYTES_COMMAND:
                case STOP_STREAMING_COMMAND:
                case STOP_SDBT_COMMAND:
                case START_LOGGING_COMMAND:
                case STOP_LOGGING_COMMAND:
                case GET_A_ACCEL_CALIBRATION_COMMAND:
                case GET_MPU9150_GYRO_CALIBRATION_COMMAND:
                case GET_LSM303DLHC_MAG_CALIBRATION_COMMAND:
                case GET_LSM303DLHC_ACCEL_CALIBRATION_COMMAND:
                case GET_GSR_RANGE_COMMAND:
                case GET_ALL_CALIBRATION_COMMAND:
                case DEPRECATED_GET_DEVICE_VERSION_COMMAND:
                case GET_DEVICE_VERSION_COMMAND:
                case GET_FW_VERSION_COMMAND:
                case GET_CHARGE_STATUS_LED_COMMAND:
                case GET_BUFFER_SIZE_COMMAND:
                case GET_UNIQUE_SERIAL_COMMAND:
                case GET_LSM303DLHC_ACCEL_RANGE_COMMAND:
                case GET_LSM303DLHC_MAG_GAIN_COMMAND:
                case GET_LSM303DLHC_MAG_SAMPLING_RATE_COMMAND:
                case GET_LSM303DLHC_ACCEL_SAMPLING_RATE_COMMAND:
                case GET_LSM303DLHC_ACCEL_LPMODE_COMMAND:
                case GET_LSM303DLHC_ACCEL_HRMODE_COMMAND:
                case GET_MPU9150_GYRO_RANGE_COMMAND:
                case GET_BMP180_CALIBRATION_COEFFICIENTS_COMMAND:
                case GET_BMP280_CALIBRATION_COEFFICIENTS_COMMAND:
                case GET_MPU9150_SAMPLING_RATE_COMMAND:
                case GET_MPU9150_ACCEL_RANGE_COMMAND:
                case GET_BMPX80_PRES_OVERSAMPLING_RATIO_COMMAND:
                case GET_INTERNAL_EXP_POWER_ENABLE_COMMAND:
                case RESET_TO_DEFAULT_CONFIGURATION_COMMAND:
                case RESET_CALIBRATION_VALUE_COMMAND:
                case GET_MPU9150_MAG_SENS_ADJ_VALS_COMMAND:
                case GET_BT_COMMS_BAUD_RATE:
                case GET_CENTER_COMMAND:
                case GET_SHIMMERNAME_COMMAND:
                case GET_EXPID_COMMAND:
                case GET_MYID_COMMAND:
                case GET_NSHIMMER_COMMAND:
                case GET_CONFIGTIME_COMMAND:
                case GET_DIR_COMMAND:
                case GET_DERIVED_CHANNEL_BYTES:
                case GET_RWC_COMMAND:
                case UPD_SDLOG_CFG_COMMAND:
                case UPD_CALIB_DUMP_COMMAND:
                case GET_BT_VERSION_STR_COMMAND:
                    *(gActionPtr) = data;
                    if(newBtCmdToProcess_cb)
                    {
                        newBtCmdToProcess_cb();
                    }
                    setDmaWaitingForResponse(1U);
                    /* Wake-up MCU so that the get command can be processed */
                    wakeupMcu = 1U;
                    break;
                case SET_LSM303DLHC_ACCEL_RANGE_COMMAND:
                case SET_LSM303DLHC_ACCEL_SAMPLING_RATE_COMMAND:
                case SET_LSM303DLHC_MAG_GAIN_COMMAND:
                case SET_CHARGE_STATUS_LED_COMMAND:
                case SET_LSM303DLHC_MAG_SAMPLING_RATE_COMMAND:
                case SET_LSM303DLHC_ACCEL_LPMODE_COMMAND:
                case SET_LSM303DLHC_ACCEL_HRMODE_COMMAND:
                case SET_MPU9150_GYRO_RANGE_COMMAND:
                case SET_MPU9150_SAMPLING_RATE_COMMAND:
                case SET_MPU9150_ACCEL_RANGE_COMMAND:
                case SET_BMPX80_PRES_OVERSAMPLING_RATIO_COMMAND:
                case SET_INTERNAL_EXP_POWER_ENABLE_COMMAND:
                case SET_GSR_RANGE_COMMAND:
                case SET_BT_COMMS_BAUD_RATE:
                case SET_CENTER_COMMAND:
                case SET_SHIMMERNAME_COMMAND:
                case SET_EXPID_COMMAND:
                case SET_MYID_COMMAND:
                case SET_NSHIMMER_COMMAND:
                case SET_CONFIGTIME_COMMAND:
                case SET_CRC_COMMAND:
                case SET_INSTREAM_RESPONSE_ACK_PREFIX_STATE:
                case SET_DATA_RATE_TEST:
                    *(gActionPtr) = data;
                    waitingForArgs = 1U;
                    break;
                case SET_SAMPLING_RATE_COMMAND:
                case GET_DAUGHTER_CARD_ID_COMMAND:
                    //                case SET_DAUGHTER_CARD_ID_COMMAND:
                    *(gActionPtr) = data;
                    waitingForArgs = 2U;
                    break;
                case SET_SENSORS_COMMAND:
                case GET_EXG_REGS_COMMAND:
                case SET_EXG_REGS_COMMAND:
                case GET_DAUGHTER_CARD_MEM_COMMAND:
                case SET_DAUGHTER_CARD_MEM_COMMAND:
                case SET_TRIAL_CONFIG_COMMAND:
                case GET_INFOMEM_COMMAND:
                case SET_INFOMEM_COMMAND:
                case GET_CALIB_DUMP_COMMAND:
                case SET_CALIB_DUMP_COMMAND:
                    *(gActionPtr) = data;
                    waitingForArgs = 3U;
                    break;
                case SET_CONFIG_SETUP_BYTES_COMMAND:
                    *(gActionPtr) = data;
                    waitingForArgs = 4U;
                    break;
                case SET_RWC_COMMAND:
                case SET_DERIVED_CHANNEL_BYTES:
                    *(gActionPtr) = data;
                    waitingForArgs = 8U;
                    break;
                case SET_A_ACCEL_CALIBRATION_COMMAND:
                case SET_MPU9150_GYRO_CALIBRATION_COMMAND:
                case SET_LSM303DLHC_MAG_CALIBRATION_COMMAND:
                case SET_LSM303DLHC_ACCEL_CALIBRATION_COMMAND:
                    *(gActionPtr) = data;
                    waitingForArgs = 21U;
                    break;
#endif
                case RN4678_STATUS_STRING_SEPARATOR:
                    memset(btStatusStr, 0, sizeof(btStatusStr));
                    btStatusStr[0U] = RN4678_STATUS_STRING_SEPARATOR;
                    btStatusStrIndex = 1U;
                    *(gActionPtr) = data;
                    /* Minus 1 because we've already received 1 x RN4678_STATUS_STRING_SEPARATOR */
                    waitingForArgs = BT_STAT_STR_LEN_SMALLEST - 1U;
                    break;
#if USE_OLD_SD_SYNC_APPROACH
                case ACK_COMMAND_PROCESSED:
                    /* SD Sync Node - gets here when a node receives a sync packet from a center */
                    *(gActionPtr) = data;
                    waitingForArgs = SYNC_PACKET_PAYLOAD_SIZE;
                    break;
#else
                case ACK_COMMAND_PROCESSED:
                    /* Wait for command byte */
                    *(gActionPtr) = data;
                    waitingForArgs = 1U;
                    break;
                case SET_SD_SYNC_COMMAND:
                    /* Store local time as early as possible after sync bytes have been received */
                    saveLocalTime();

                    *(gActionPtr) = data;
                    waitingForArgs = SYNC_PACKET_PAYLOAD_SIZE + BT_SD_SYNC_CRC_MODE;
                    break;
#endif
                default:
                    setDmaWaitingForResponse(1U);
                    break;
                }

                if (waitingForArgs)
                {
                    setDmaWaitingForResponse(waitingForArgs);
                }

                return wakeupMcu;
            }
        }
    }
    else
    {
        uint8_t len = strlen((char*) btRxExp);
        if (!memcmp(btRxBuff, btRxExp, len))
        {
            memset(btRxBuff, 0, len);
            BT_setGoodCommand();
        }
        else
        {
            _NOP(); // bad command trap: reaching here = serious BT problem
        }
    }

    setDmaWaitingForResponseIfStatusStrEnabled();
    return 0;
}

void resetBtRxVariablesOnConnect(void)
{
    /* Reset to unsupported command */
    *(gActionPtr) = ACK_COMMAND_PROCESSED-1U;
    waitingForArgs = 0;
    waitingForArgsLength = 0;
}

void resetBtRxBuff(void)
{
    memset(btRxBuff, 0, sizeof(btRxBuff));
}

#else
void processBtUartBuf(void)
{
    uint8_t responseParsed;
    uint8_t firstByteInRxBuf;

    /* Continue parsing buffer until we've parsed everything that we can from it */
    do
    {
        responseParsed = 0;
        firstByteInRxBuf = getRxByteAtIndex(0);
        updateNumBytesInBtRxBufWhenLastProcessed();
        indexOfFirstEol = 0;

        if (numBytesInBtRxBufWhenLastProcessed)
        {
            /* Response from BT Module command mode */
            if(wasStartBtCmdModeSentAndReponseReceived())
            {
                processStartRnCmdResponse();
                responseParsed = 1U;
            }
            else if(isRnCommandModeActive() && (indexOfFirstEol = isNewLineDetectedInBtRxBuf())>0)
            {
                /* isRn4678CmdDetectedOnBoot and isBtDeviceUnknown() are needed
                 * here because the first command that is sent is to get the
                 * version number and so until the response is parsed, BtDevice
                 * has not been set yet */
                if(((isBtDeviceRn4678() || isRn4678CmdDetectedOnBoot) && isFullRN4678CmdResponseReceived())
                        || isBtDeviceRn41orRN42()
                        || isBtDeviceUnknown())
                {
                    responseParsed = processRnCmdResponse();
                }
            }
            /* Status responses begin with '%' */
            else if(firstByteInRxBuf==RN4678_STATUS_STRING_SEPARATOR
                    && numBytesInBtRxBufWhenLastProcessed>=BT_STAT_STR_LEN_SMALLEST)
            {
                responseParsed = processStatusString();
            }
            /* Shimmer command */
            else if(!areBtSetupCommandsRunning() && isShimmerBtCmd(firstByteInRxBuf))
            {
                responseParsed = processShimmerBtCmd();
            }

            /* Update reference time so that timeout can be detected */
            /* Don't enable timeout until BT is being initialised */
            if(isBtStarting() || responseParsed || getNumBytesInBtRxBuf()==0)
            {
                /* Reset time if successfully parsed or buffer is empty */
                firstProcessFailTicks = 0;
            }
            else if (firstProcessFailTicks == 0)
            {
                /* Set the time when the first fail to parse occurred */
                firstProcessFailTicks = RTC_get32();
            }
        }

    } while (responseParsed == 1U);
}

/* TODO Status string check here only supports RN4678 responses at the moment but this is fine because we have them turned off for the RN42 */
/* TODO If it's the end % missing, this code will currently remove it from the start of the next status string if there's one in the buffer */
void handleBtRxTimeout(void)
{
    uint8_t numberOfCharToRemove = 0U;

    if ((numberOfCharToRemove = isBtRxBufLike("%RFCOMM_CLOSE%", 1)) > 0)
    {
        triggerBtRfCommStateChangeCallback(0);
    }
    else if ((numberOfCharToRemove = isBtRxBufLike("%RFCOMM_OPEN%", 1)) > 0)
    {
        triggerBtRfCommStateChangeCallback(1U);
    }
    else if ((numberOfCharToRemove = isBtRxBufLike("%CONNECT,XXXXXXXXXXXX%", 1)) > 0)
    {
        setRn4678ConnectionState(RN4678_CONNECTED_CLASSIC);
    }
    else if ((numberOfCharToRemove = isBtRxBufLike("%DISCONN%", 1)) > 0)
    {
        setRn4678ConnectionState(RN4678_DISCONNECTED);
    }
    else
    {
        // Still unable to parse puffer, clear the first byte
        numberOfCharToRemove = 1U;
    }

    clearBytesFromBtRxBuf(numberOfCharToRemove);

    /* Need to update this variable to indicate to main to process new bytes when they come in */
    updateNumBytesInBtRxBufWhenLastProcessed();

    /* Reset process fail time */
    firstProcessFailTicks = 0;
}

uint8_t hasBtRxTimeoutOccurred(void)
{
    return (firstProcessFailTicks != 0
            && ((RTC_get32() - firstProcessFailTicks) > BT_RX_COMMS_TIMEOUT_TICKS));
}

void processStartRnCmdResponse(void)
{
//    clearBytesFromBtRxBuf(RN4X_CMD_LEN);

    uint8_t buf[RN4X_CMD_LEN], i;
    for (i = 0; i < RN4X_CMD_LEN; i++)
    {
        readByteFromBtRxBuf(&buf[i]);
    }

    /* RN42 returns "CMD\r\n", RN4678 returns "CMD> " */
    if (buf[3] == '>' && buf[4] == ' ')
    {
        isRn4678CmdDetectedOnBoot = 1U;
    }

    setCommandModeActive(1U);
    clearBtCmdTxRxBuffsAndProceed();
}

uint8_t wasStartBtCmdModeSentAndReponseReceived(void)
{
    return (commandBufPtr[0]=='$'
            && commandBufPtr[1U]=='$'
            && commandBufPtr[2U]=='$'
            && numBytesInBtRxBufWhenLastProcessed>=RN4X_CMD_LEN
            && getRxByteAtIndex(0)=='C'
            && getRxByteAtIndex(1U)=='M'
            && getRxByteAtIndex(2U)=='D');
}

//uint8_t waitingForRnBtCmdResponse(void)
//{
//    return expectedResponsePtr[0]!='\0';
//}
//
//uint8_t isExpectedRnBtCmdResponseInFifo(void)
//{
//    uint8_t i = strlen(expectedResponsePtr);
//    if(getNumBytesInBtRxBuf()>=i)
//    {
//        for(;i>0;i--)
//        {
//            if (expectedResponsePtr[i-1] != getRxByteAtIndex(i-1))
//            {
//                return 0;
//            }
//        }
//        return 1;
//    }
//    return 0;
//}

void clearBytesFromBtRxBuf(uint16_t numBytes)
{
    uint8_t buf;
    for(;numBytes>0;numBytes--)
    {
        readByteFromBtRxBuf(&buf);
    }
}

/*
 * Gets a byte from the BT RX buffer without removing it from it.
 *
 */
uint8_t getRxByteAtIndex(uint16_t index)
{
    return gBtRxFifoPtr->data[BT_RX_BUF_MASK & (gBtRxFifoPtr->rdIdx + index)];
}

uint8_t processStatusString(void)
{
    uint8_t numberOfCharToRemove = 0U;
    uint8_t triggerBtSetGoodCommand = 0U;

    enum BT_FIRMWARE_VERSION btFwVer = getBtFwVersion();

    uint8_t firstChar = getRxByteAtIndex(1U);
    switch (firstChar)
    {
    case 'A':
        /* "%AUTHENTICATED%" */
        if (numBytesInBtRxBufWhenLastProcessed >= BT_STAT_STR_LEN_AUTHENTICATED
                && getRxByteAtIndex(14U)=='%')
        {
            numberOfCharToRemove = BT_STAT_STR_LEN_AUTHENTICATED;
            /* TODO */
        }
        /* "%AUTH_FAIL%" */
        else if (numBytesInBtRxBufWhenLastProcessed >= BT_STAT_STR_LEN_AUTH_FAIL
                && getRxByteAtIndex(10U)=='%')
        {
            numberOfCharToRemove = BT_STAT_STR_LEN_AUTH_FAIL;
            /* TODO */
        }
        break;
    case 'B':
        /* "%BONDED%" */
        if (numBytesInBtRxBufWhenLastProcessed >= BT_STAT_STR_LEN_BONDED
                && getRxByteAtIndex(7U)=='%')
        {
            numberOfCharToRemove = BT_STAT_STR_LEN_BONDED;
            /* TODO */
        }
        break;
    case 'C':
        /* "%CONNECT,001BDC06A3D5%" - RN4678 */
        if (numBytesInBtRxBufWhenLastProcessed >= BT_STAT_STR_LEN_RN4678_CONNECT
                && getRxByteAtIndex(21U)=='%')
        {
            setRn4678ConnectionState(RN4678_CONNECTED_CLASSIC);
            numberOfCharToRemove = BT_STAT_STR_LEN_RN4678_CONNECT;
        }
        /* "%CONNECT" for RN42 v4.77 or "%CONNECT,001BDC06A3D5," - RN42 v6.15 */
        else if (( (btFwVer == RN41_V4_77 || btFwVer == RN42_V4_77)
                && numBytesInBtRxBufWhenLastProcessed >= BT_STAT_STR_LEN_RN42_v477_CONNECT
                && getRxByteAtIndex(7U)=='T')
            || (btFwVer == RN42_V6_15
                    && numBytesInBtRxBufWhenLastProcessed >= BT_STAT_STR_LEN_RN42_v615_CONNECT
                    && getRxByteAtIndex(21U)==','))
        {
            triggerBtRfCommStateChangeCallback(1U);

            if (btFwVer == RN41_V4_77 || btFwVer == RN42_V4_77)
            {
                numberOfCharToRemove = BT_STAT_STR_LEN_RN42_v477_CONNECT;
            }
            else if (btFwVer == RN42_V6_15)
            {
                numberOfCharToRemove = BT_STAT_STR_LEN_RN42_v615_CONNECT;
            }
            else
            {
                /* other RN42 FWs not supported */
            }
        }
        break;
    case 'D':
        /* "%DISCONN%" -> RN4678 */
        if (numBytesInBtRxBufWhenLastProcessed >= BT_STAT_STR_LEN_RN4678_DISCONN
                && getRxByteAtIndex(8U)=='%')
        {
            setRn4678ConnectionState(RN4678_DISCONNECTED);
            numberOfCharToRemove = BT_STAT_STR_LEN_RN4678_DISCONN;
        }
        /* "%DISCONNECT" -> RN42 */
        else if (numBytesInBtRxBufWhenLastProcessed >= BT_STAT_STR_LEN_RN42_DISCONNECT
                && getRxByteAtIndex(10U)=='T')
        {
            triggerBtRfCommStateChangeCallback(0);

            numberOfCharToRemove = BT_STAT_STR_LEN_RN42_DISCONNECT;
        }
        break;
    case 'E':
        if (getRxByteAtIndex(2U)=='N')
        {
            /* "%END_INQ%" */
            if (getRxByteAtIndex(5U)=='I'
                    && getRxByteAtIndex(8U)=='%'
                    && numBytesInBtRxBufWhenLastProcessed >= BT_STAT_STR_LEN_END_INQ)
            {
                numberOfCharToRemove = BT_STAT_STR_LEN_END_INQ;
                /* TODO */
            }
            /* "%END_SCN%" */
            else if (getRxByteAtIndex(5U)=='S'
                    && getRxByteAtIndex(8U)=='%'
                    && numBytesInBtRxBufWhenLastProcessed >= BT_STAT_STR_LEN_END_SCN)
            {
                numberOfCharToRemove = BT_STAT_STR_LEN_END_SCN;
                /* TODO */
            }
        }
        else if (getRxByteAtIndex(2U)=='R')
        {
            if (getRxByteAtIndex(5U)=='C')
            {
                /* %ERR_CON is common to two status strings. If detected, read two more chars to determine which one it is */
                /* "%ERR_CONN%" */
                if (numBytesInBtRxBufWhenLastProcessed >= BT_STAT_STR_LEN_ERR_CONN
                        && getRxByteAtIndex(9U)=='%')
                {
                    numberOfCharToRemove = BT_STAT_STR_LEN_ERR_CONN;
                    /* TODO */
                }
                /* "%ERR_CONN_PARAM%" */
                else if (numBytesInBtRxBufWhenLastProcessed >= BT_STAT_STR_LEN_ERR_CONN_PARAM
                        && getRxByteAtIndex(15U)=='%')
                {
                    numberOfCharToRemove = BT_STAT_STR_LEN_ERR_CONN_PARAM;
                    /* TODO */
                }
            }
            else if (getRxByteAtIndex(5U)=='L')
            {
                /* "%ERR_LSEC%" */
                if (numBytesInBtRxBufWhenLastProcessed >= BT_STAT_STR_LEN_ERR_LSEC
                        && getRxByteAtIndex(9U)=='%')
                {
                    numberOfCharToRemove = BT_STAT_STR_LEN_ERR_LSEC;
                    /* TODO */
                }
            }
            else if (getRxByteAtIndex(5U)=='S')
            {
                /* "%ERR_SEC%" */
                if (numBytesInBtRxBufWhenLastProcessed >= BT_STAT_STR_LEN_ERR_SEC
                        && getRxByteAtIndex(8U)=='%')
                {
                    numberOfCharToRemove = BT_STAT_STR_LEN_ERR_SEC;
                    /* TODO */
                }
            }
        }
        break;
    case 'F':
        /* "%FACTORY_RESET%" */
        if (numBytesInBtRxBufWhenLastProcessed >= BT_STAT_STR_LEN_FACTORY_RESET
                && getRxByteAtIndex(14U)=='%')
        {
            numberOfCharToRemove = BT_STAT_STR_LEN_FACTORY_RESET;
            /* TODO */
        }
        break;
    case 'L':
        if (getRxByteAtIndex(2U)=='B')
        {
            /* "%LBONDED%" */
            if (numBytesInBtRxBufWhenLastProcessed >= BT_STAT_STR_LEN_LBONDED
                    && getRxByteAtIndex(8U)=='%')
            {
                numberOfCharToRemove = BT_STAT_STR_LEN_LBONDED;
                /* TODO */
            }
        }
        else if (getRxByteAtIndex(2U)=='S')
        {
            if (getRxByteAtIndex(6U)=='R')
            {
                /* "%LSECURED%" */
                if (numBytesInBtRxBufWhenLastProcessed >= BT_STAT_STR_LEN_LSECURED
                        && getRxByteAtIndex(9U)=='%')
                {
                    numberOfCharToRemove = BT_STAT_STR_LEN_LSECURED;
                    /* TODO */
                }
                /* "%LSECURE_FAIL%" */
                else if (numBytesInBtRxBufWhenLastProcessed >= BT_STAT_STR_LEN_LSECURE_FAIL
                        && getRxByteAtIndex(13U)=='%')
                {
                    numberOfCharToRemove = BT_STAT_STR_LEN_LSECURE_FAIL;
                    /* TODO */
                }
            }
            else if (getRxByteAtIndex(6U)=='A')
            {
                /* "%LSTREAM_OPEN%" */
                if (numBytesInBtRxBufWhenLastProcessed >= BT_STAT_STR_LEN_LSTREAM_OPEN
                        && getRxByteAtIndex(13U)=='%')
                {
                    numberOfCharToRemove = BT_STAT_STR_LEN_LSTREAM_OPEN;
                    /* TODO */
                }
            }
        }
        break;
    case 'M':
        /* "%MLDP_MODE%" */
        if (numBytesInBtRxBufWhenLastProcessed >= BT_STAT_STR_LEN_MLDP_MODE
                && getRxByteAtIndex(10U)=='%')
        {
            numberOfCharToRemove = BT_STAT_STR_LEN_MLDP_MODE;
            /* TODO */
        }
        break;
    case 'R':
        if (getRxByteAtIndex(2U)=='E')
        {
            /* "%REBOOT%" -> RN4678 */
            if (numBytesInBtRxBufWhenLastProcessed >= BT_STAT_STR_LEN_RN4678_REBOOT
                    && getRxByteAtIndex(7U)=='%')
            {
                numberOfCharToRemove = BT_STAT_STR_LEN_RN4678_REBOOT;
            }
            else if (isBtDeviceRn41orRN42()
                    && numBytesInBtRxBufWhenLastProcessed >= BT_STAT_STR_LEN_RN42_REBOOT
                    && getRxByteAtIndex(6U)=='T')
            {
                numberOfCharToRemove = BT_STAT_STR_LEN_RN42_REBOOT;
            }

            if(BT_getWaitForInitialBoot() && numberOfCharToRemove)
            {
                BT_setWaitForInitialBoot(0);
                triggerBtSetGoodCommand = 1U;
            }
        }
        else if (getRxByteAtIndex(2U)=='F')
        {
            if (getRxByteAtIndex(8U)=='C')
            {
                /* "%RFCOMM_CLOSE%" */
                if (numBytesInBtRxBufWhenLastProcessed >= BT_STAT_STR_LEN_RFCOMM_CLOSE
                        && getRxByteAtIndex(13U)=='%')
                {
                    triggerBtRfCommStateChangeCallback(0);
                    numberOfCharToRemove = BT_STAT_STR_LEN_RFCOMM_CLOSE;
                }
            }
            /* "%RFCOMM_OPEN%" */
            else if (numBytesInBtRxBufWhenLastProcessed >= BT_STAT_STR_LEN_RFCOMM_OPEN
                    && getRxByteAtIndex(12U)=='%')
            {
                triggerBtRfCommStateChangeCallback(1U);
                numberOfCharToRemove = BT_STAT_STR_LEN_RFCOMM_OPEN;
            }
        }
        break;
    case 'S':
        if (getRxByteAtIndex(3U)=='C')
        {
            /* "%SECURED%" */
            if (numBytesInBtRxBufWhenLastProcessed >= BT_STAT_STR_LEN_SECURED
                    && getRxByteAtIndex(8U)=='%')
            {
                numberOfCharToRemove = BT_STAT_STR_LEN_SECURED;
                /* TODO */
            }
            /* "%SECURE_FAIL%" */
            else if (numBytesInBtRxBufWhenLastProcessed >= BT_STAT_STR_LEN_SECURE_FAIL
                    && getRxByteAtIndex(12U)=='%')
            {
                numberOfCharToRemove = BT_STAT_STR_LEN_SECURE_FAIL;
                /* TODO */
            }
        }
        else if (getRxByteAtIndex(3U)=='S')
        {
            /* "%SESSION_OPEN%" */
            if (numBytesInBtRxBufWhenLastProcessed >= BT_STAT_STR_LEN_SESSION_OPEN
                    && getRxByteAtIndex(13U)=='%')
            {
                numberOfCharToRemove = BT_STAT_STR_LEN_SESSION_OPEN;
                /* TODO */
            }
            /* "%SESSION_CLOSE%" */
            else if (numBytesInBtRxBufWhenLastProcessed >= BT_STAT_STR_LEN_SESSION_CLOSE
                    && getRxByteAtIndex(14U)=='%')
            {
                numberOfCharToRemove = BT_STAT_STR_LEN_SESSION_CLOSE;
                /* TODO */
            }
        }
        break;
    default:
        break;
    }

    clearBytesFromBtRxBuf(numberOfCharToRemove);
    if(triggerBtSetGoodCommand)
    {
        BT_setGoodCommand();
    }

    return numberOfCharToRemove>0;
}

uint8_t isBtRxBufLike(char statStrCheck[], uint8_t numCharTolerance)
{
    uint8_t count = 0, bufIndex = 0, i = 0;
    uint8_t statStrCheckLen = strlen(statStrCheck);

    /* If there's not enough bytes in the buf even with the tolerance, return straight away */
    if (getNumBytesInBtRxBuf() < (statStrCheckLen - numCharTolerance))
    {
        return 0;
    }

    for (i = 0; i < statStrCheckLen; i++)
    {
        if (getRxByteAtIndex(bufIndex) == statStrCheck[i]
                   || statStrCheck[i]=='X')
        {
            count++;
            bufIndex++;
        }
    }

    if(((statStrCheckLen - count) <= numCharTolerance))
    {
        return count;
    }
    else
    {
        return 0;
    }
}

uint8_t processRnCmdResponse(void)
{
    uint8_t responseParsed = 0;

    /* Unwrap response */
    uint16_t i;
    for (i = 0; i < (indexOfFirstEol+2); i++)
    {
        readByteFromBtRxBuf(&unwrappedResponse[i]);

        if(i>=2
                && unwrappedResponse[i-1]=='\r'
                && unwrappedResponse[i]=='\n')
        {
            break;
        }
    }

    uint16_t unwrappedResponseLen = strlen(unwrappedResponse);
    uint8_t cmdExpectedAfterEol = (isBtDeviceRn4678() || isRn4678CmdDetectedOnBoot);

    /* Get BT module hardware and firmware version */
    if(commandBufPtr[0]=='V')
    {
        enum BT_FIRMWARE_VERSION btFwVerNew = BT_FW_VER_UNKNOWN;

        /* RN41 or RN42 */
        if (unwrappedResponse[0U]=='V')
        {
            /* RN41_VERSION_RESPONSE_V4_77 or RN42_VERSION_RESPONSE_V4_77 */
            if (unwrappedResponse[4U]=='4' && unwrappedResponse[5U]=='.')
            {
                if (unwrappedResponse[9U]=='R' && unwrappedResponse[10U]=='N')
                {
                    btFwVerNew = RN42_V4_77;
                }
                else
                {
                    btFwVerNew = RN41_V4_77;
                }
            }
            /* RN42_VERSION_RESPONSE_V6_15 */
            else if (unwrappedResponse[4U]=='6' && unwrappedResponse[5U]=='.' && unwrappedResponse[6U]=='1' && unwrappedResponse[7U]=='5')
            {
                btFwVerNew = RN42_V6_15;
            }
            /* V6.30 not supported */
            else if (unwrappedResponse[4U]=='6' && unwrappedResponse[5U]=='.' && unwrappedResponse[6U]=='3' && unwrappedResponse[7U]=='0')
            {
                btFwVerNew = RN42_V6_30;
            }
        }
        /* RN4678 */
        else if (unwrappedResponse[0U]=='R')
        {
            /* RN4678_VERSION_RESPONSE_V1_00_5 */
            if (unwrappedResponse[10U]=='0' && unwrappedResponse[11U]=='0')
            {
                btFwVerNew = RN4678_V1_00_5;
            }
            /* RN4678_VERSION_RESPONSE_V1_11_0 */
            else if (unwrappedResponse[10U]=='1' && unwrappedResponse[11U]=='1')
            {
                btFwVerNew = RN4678_V1_11_0;
            }
            /* RN4678_VERSION_RESPONSE_V1_13_5 */
            else if (unwrappedResponse[10U]=='1' && unwrappedResponse[11U]=='3')
            {
                btFwVerNew = RN4678_V1_13_5;
            }
            /* RN4678_VERSION_RESPONSE_V1_22_0 */
            else if (unwrappedResponse[10U]=='2' && unwrappedResponse[11U]=='2')
            {
                btFwVerNew = RN4678_V1_22_0;
            }
            /* RN4678_VERSION_RESPONSE_V1_23_0 */
            else if (unwrappedResponse[10U]=='2' && unwrappedResponse[11U]=='3')
            {
                btFwVerNew = RN4678_V1_23_0;
            }
        }

        if (btFwVerNew == BT_FW_VER_UNKNOWN || btFwVerNew == RN42_V6_30)
        {
            triggerShimmerErrorState();
        }

        /* When storing the BT version, ignore from "\r" onwards */
        uint8_t btVerLen = strlen(unwrappedResponse);
        uint8_t btVerIdx;
        for (btVerIdx = 0; btVerIdx < btVerLen; btVerIdx++)
        {
            if (unwrappedResponse[btVerIdx] == '\r')
            {
                btVerLen = btVerIdx;
                break;
            }
        }
        memcpy(btVerStrResponse, unwrappedResponse, btVerLen);


        setBtFwVersion(btFwVerNew);

        responseParsed = 1;
    }
    /* Get commands */
    else if(commandBufPtr[0]=='G')
    {
        /* Get MAC address */
        if(commandBufPtr[1]=='B')
        {
            setMacId_cb(unwrappedResponse);
            responseParsed = 1;
        }
        else
        {
            responseParsed = parseRnGetResponse(commandBufPtr[1], &unwrappedResponse[0]);
        }
    }
//    else if(unwrappedResponse[0]=='C'
//            && unwrappedResponse[1]=='M'
//            && unwrappedResponse[2]=='D')
//    {
//        setCommandModeActive(1U);
//        responseParsed = 1;
//        cmdExpectedAfterEol = 0;
//    }
    else if(unwrappedResponse[0]=='A'
            && unwrappedResponse[1]=='O'
            && unwrappedResponse[2]=='K')
    {
        responseParsed = 1;
    }
    else if(unwrappedResponse[0]=='E'
            && unwrappedResponse[1]=='N'
            && unwrappedResponse[2]=='D')
    {
        setCommandModeActive(0U);
        responseParsed = 1;
        cmdExpectedAfterEol = 0;
    }
    else if(unwrappedResponse[0]=='E'
            && unwrappedResponse[1]=='R'
            && unwrappedResponse[2]=='R')
    {
        triggerShimmerErrorState();
    }
    else if(unwrappedResponse[0]=='?')
    {
        triggerShimmerErrorState();
    }

    else if(expectedResponsePtr[0]!='\0'
            && !memcmp(unwrappedResponse, expectedResponsePtr, strlen(expectedResponsePtr)))
    {
        responseParsed = 1;
    }

    memset(unwrappedResponse, 0, strlen(unwrappedResponse));

    if(responseParsed)
    {
        if(cmdExpectedAfterEol)
        {
            clearBytesFromBtRxBuf(RN4X_CMD_LEN);
        }

        clearBtCmdTxRxBuffsAndProceed();
    }

    return responseParsed;
}

uint8_t isFullRN4678CmdResponseReceived(void)
{
    int16_t numBytesAfterEol = numBytesInBtRxBufWhenLastProcessed-(indexOfFirstEol+2U);

    /* For the RN4678, all responses have "CMD> " as a suffix except for "Trying\r\n", "END\r\n".
     * There is an option to turn this off in the RN4678 but we are leaving it on at the moment */
    if((getRxByteAtIndex(0)=='T'
                    && getRxByteAtIndex(1U)=='r'
                    && getRxByteAtIndex(2U)=='y')
            || (getRxByteAtIndex(0)=='E'
                    && getRxByteAtIndex(1U)=='N'
                    && getRxByteAtIndex(2U)=='D'))
    {
        return 1U;
    }
    else
    {
        if(numBytesAfterEol!=0 && numBytesAfterEol>=RN4X_CMD_LEN)
        {
            return 1U;
        }
    }
    return 0;
}

void clearBtCmdTxRxBuffsAndProceed(void)
{
    clearBtCmdBuf();
    clearExpectedResponseBuf();
    BT_setGoodCommand();
}

uint8_t processShimmerBtCmd(void)
{
    uint8_t expectedLength = 0;
    uint8_t responseParsed = 0;

    uint8_t data = getRxByteAtIndex(0);
    switch (data)
    {
    /* 1 command byte, no argument bytes */
    case INQUIRY_COMMAND:
    case DUMMY_COMMAND:
    case GET_SAMPLING_RATE_COMMAND:
    case TOGGLE_LED_COMMAND:
    case START_STREAMING_COMMAND:
    case GET_STATUS_COMMAND:
    case GET_VBATT_COMMAND:
    case GET_TRIAL_CONFIG_COMMAND:
    case START_SDBT_COMMAND:
    case GET_CONFIG_SETUP_BYTES_COMMAND:
    case STOP_STREAMING_COMMAND:
    case STOP_SDBT_COMMAND:
    case START_LOGGING_COMMAND:
    case STOP_LOGGING_COMMAND:
    case GET_A_ACCEL_CALIBRATION_COMMAND:
    case GET_MPU9150_GYRO_CALIBRATION_COMMAND:
    case GET_LSM303DLHC_MAG_CALIBRATION_COMMAND:
    case GET_LSM303DLHC_ACCEL_CALIBRATION_COMMAND:
    case GET_GSR_RANGE_COMMAND:
    case GET_ALL_CALIBRATION_COMMAND:
    case DEPRECATED_GET_DEVICE_VERSION_COMMAND:
    case GET_DEVICE_VERSION_COMMAND:
    case GET_FW_VERSION_COMMAND:
    case GET_CHARGE_STATUS_LED_COMMAND:
    case GET_BUFFER_SIZE_COMMAND:
    case GET_UNIQUE_SERIAL_COMMAND:
    case GET_LSM303DLHC_ACCEL_RANGE_COMMAND:
    case GET_LSM303DLHC_MAG_GAIN_COMMAND:
    case GET_LSM303DLHC_MAG_SAMPLING_RATE_COMMAND:
    case GET_LSM303DLHC_ACCEL_SAMPLING_RATE_COMMAND:
    case GET_LSM303DLHC_ACCEL_LPMODE_COMMAND:
    case GET_LSM303DLHC_ACCEL_HRMODE_COMMAND:
    case GET_MPU9150_GYRO_RANGE_COMMAND:
    case GET_BMP180_CALIBRATION_COEFFICIENTS_COMMAND:
    case GET_BMP280_CALIBRATION_COEFFICIENTS_COMMAND:
    case GET_MPU9150_SAMPLING_RATE_COMMAND:
    case GET_MPU9150_ACCEL_RANGE_COMMAND:
    case GET_BMPX80_PRES_OVERSAMPLING_RATIO_COMMAND:
    case GET_INTERNAL_EXP_POWER_ENABLE_COMMAND:
    case RESET_TO_DEFAULT_CONFIGURATION_COMMAND:
    case RESET_CALIBRATION_VALUE_COMMAND:
    case GET_MPU9150_MAG_SENS_ADJ_VALS_COMMAND:
    case GET_BT_COMMS_BAUD_RATE:
    case GET_CENTER_COMMAND:
    case GET_SHIMMERNAME_COMMAND:
    case GET_EXPID_COMMAND:
    case GET_MYID_COMMAND:
    case GET_NSHIMMER_COMMAND:
    case GET_CONFIGTIME_COMMAND:
    case GET_DIR_COMMAND:
    case GET_DERIVED_CHANNEL_BYTES:
    case GET_RWC_COMMAND:
    case UPD_SDLOG_CFG_COMMAND:
    case UPD_CALIB_DUMP_COMMAND:
    case GET_BT_VERSION_STR_COMMAND:
        readActionAndArgBytes(0);
        responseParsed = 1U;
        break;

    /* 1 command byte, 1 argument byte */
    case SET_LSM303DLHC_ACCEL_RANGE_COMMAND:
    case SET_LSM303DLHC_ACCEL_SAMPLING_RATE_COMMAND:
    case SET_LSM303DLHC_MAG_GAIN_COMMAND:
    case SET_CHARGE_STATUS_LED_COMMAND:
    case SET_LSM303DLHC_MAG_SAMPLING_RATE_COMMAND:
    case SET_LSM303DLHC_ACCEL_LPMODE_COMMAND:
    case SET_LSM303DLHC_ACCEL_HRMODE_COMMAND:
    case SET_MPU9150_GYRO_RANGE_COMMAND:
    case SET_MPU9150_SAMPLING_RATE_COMMAND:
    case SET_MPU9150_ACCEL_RANGE_COMMAND:
    case SET_BMPX80_PRES_OVERSAMPLING_RATIO_COMMAND:
    case SET_INTERNAL_EXP_POWER_ENABLE_COMMAND:
    case SET_GSR_RANGE_COMMAND:
    case SET_BT_COMMS_BAUD_RATE:
    case SET_MYID_COMMAND:
    case SET_NSHIMMER_COMMAND:
    case SET_CRC_COMMAND:
    case SET_INSTREAM_RESPONSE_ACK_PREFIX_STATE
    case SET_DATA_RATE_TEST:
        if(numBytesInBtRxBufWhenLastProcessed>=(1U+1U))
        {
            readActionAndArgBytes(1U);
            responseParsed = 1U;
        }
        break;

    /* 1 command byte, 2 argument bytes */
    case SET_SAMPLING_RATE_COMMAND:
    case GET_DAUGHTER_CARD_ID_COMMAND:
        if(numBytesInBtRxBufWhenLastProcessed>=(1U+2U))
        {
            readActionAndArgBytes(2U);
            responseParsed = 1U;
        }
        break;

    /* 1 command byte, 3 argument bytes */
    case SET_SENSORS_COMMAND:
    case GET_EXG_REGS_COMMAND:
    case GET_DAUGHTER_CARD_MEM_COMMAND:
    case SET_TRIAL_CONFIG_COMMAND:
    case GET_INFOMEM_COMMAND:
    case GET_CALIB_DUMP_COMMAND:
        if(numBytesInBtRxBufWhenLastProcessed>=(1U+3U))
        {
            readActionAndArgBytes(3U);
            responseParsed = 1U;
        }
        break;

    /* 1 command byte, 4 argument bytes */
    case SET_CONFIG_SETUP_BYTES_COMMAND:
        if(numBytesInBtRxBufWhenLastProcessed>=(1U+4U))
        {
            readActionAndArgBytes(4U);
            responseParsed = 1U;
        }
        break;

    /* 1 command byte, 8 argument bytes */
    case SET_RWC_COMMAND:
    case SET_DERIVED_CHANNEL_BYTES:
        if(numBytesInBtRxBufWhenLastProcessed>=(1U+8U))
        {
            readActionAndArgBytes(8U);
            responseParsed = 1U;
        }
        break;

#if !USE_OLD_SD_SYNC_APPROACH
    /* 1 command byte, 9 to 11 argument bytes */
    case SET_SD_SYNC_COMMAND:
        if (numBytesInBtRxBufWhenLastProcessed >= (1U + SYNC_PACKET_PAYLOAD_SIZE + BT_SD_SYNC_CRC_MODE))
        {
            readActionAndArgBytes(SYNC_PACKET_PAYLOAD_SIZE + BT_SD_SYNC_CRC_MODE);
            responseParsed = 1U;
        }
        break;
#endif

    /* 1 command byte, 21 argument bytes */
    case SET_A_ACCEL_CALIBRATION_COMMAND:
    case SET_MPU9150_GYRO_CALIBRATION_COMMAND:
    case SET_LSM303DLHC_MAG_CALIBRATION_COMMAND:
    case SET_LSM303DLHC_ACCEL_CALIBRATION_COMMAND:
        if(numBytesInBtRxBufWhenLastProcessed>=(1U+21U))
        {
            readActionAndArgBytes(21U);
            responseParsed = 1U;
        }
        break;

/*********************************************/
    /* special cases */
    /* 1 command byte, minimum 1 argument byte - expected length is stored in arg[0] */
    case SET_CENTER_COMMAND:
    case SET_CONFIGTIME_COMMAND:
    case SET_EXPID_COMMAND:
    case SET_SHIMMERNAME_COMMAND:
        if(numBytesInBtRxBufWhenLastProcessed>=(1U+1U))
        {
            expectedLength = gBtRxFifoPtr->data[BT_RX_BUF_MASK & (gBtRxFifoPtr->rdIdx+1U)];
            if(numBytesInBtRxBufWhenLastProcessed>=(1U+1U+expectedLength))
            {
                readActionAndArgBytes(1U+expectedLength);
                responseParsed = 1U;
            }
        }
        break;

    /* 1 command byte, minimum 2 argument bytes - expected length is stored in arg[0] */
    case SET_DAUGHTER_CARD_ID_COMMAND:
        if(numBytesInBtRxBufWhenLastProcessed>=(1U+2U))
        {
            expectedLength = gBtRxFifoPtr->data[BT_RX_BUF_MASK & (gBtRxFifoPtr->rdIdx+1U)];
            if(numBytesInBtRxBufWhenLastProcessed>=(1U+2U+expectedLength))
            {
                readActionAndArgBytes(2U+expectedLength);
                responseParsed = 1U;
            }
        }
        break;

    /* 1 command byte, minimum 3 argument bytes - expected length is stored in arg[2] */
    case SET_EXG_REGS_COMMAND:
        if(numBytesInBtRxBufWhenLastProcessed>=(1U+3U))
        {
            expectedLength = gBtRxFifoPtr->data[BT_RX_BUF_MASK & (gBtRxFifoPtr->rdIdx+3U)];
            if(numBytesInBtRxBufWhenLastProcessed>=(1U+3U+expectedLength))
            {
                readActionAndArgBytes(3U+expectedLength);
                responseParsed = 1U;
            }
        }
        break;

    /* 1 command byte, minimum 3 argument bytes - expected length is stored in arg[0] */
    case SET_INFOMEM_COMMAND:
    case SET_CALIB_DUMP_COMMAND:
        if(numBytesInBtRxBufWhenLastProcessed>=(1U+3U))
        {
            expectedLength = gBtRxFifoPtr->data[BT_RX_BUF_MASK & (gBtRxFifoPtr->rdIdx+1U)];
            if(numBytesInBtRxBufWhenLastProcessed>=(1U+3U+expectedLength))
            {
                readActionAndArgBytes(3U+expectedLength);
                responseParsed = 1U;
            }
        }
        break;

#if !USE_OLD_SD_SYNC_APPROACH
    /* 1 command byte, minimum 1 argument byte representing command response */
    case ACK_COMMAND_PROCESSED:
        if(numBytesInBtRxBufWhenLastProcessed>=(1U+1U))
        {
            uint8_t cmdByte = gBtRxFifoPtr->data[BT_RX_BUF_MASK & (gBtRxFifoPtr->rdIdx+1U)];
            if(cmdByte==SD_SYNC_RESPONSE
                    && numBytesInBtRxBufWhenLastProcessed>=(1U+2U))
            {
                readActionAndArgBytes(2U);
                responseParsed = 1U;
            }
        }
        break;
#endif
/********************************************/

    default:
        break;
    }

    return responseParsed;
}

void readActionAndArgBytes(uint8_t numArgs)
{
    readByteFromBtRxBuf(gActionPtr);
    if(numArgs)
    {
        uint8_t i;
        for(i=0;i<numArgs;i++)
        {
            readByteFromBtRxBuf(gArgsPtr+i);
        }
    }

    if(newBtCmdToProcess_cb)
    {
        newBtCmdToProcess_cb();
    }
}

uint8_t isNewLineDetectedInBtRxBuf(void)
{
    uint16_t i = 0;
    if(numBytesInBtRxBufWhenLastProcessed>2U){
        /* numBytesInRxBuf is -1 here as we're looking to i+1 for \n and it was triggering too early as new bytes were coming in */
        for(i=0;i<(numBytesInBtRxBufWhenLastProcessed-1);i++)
        {
            if(getRxByteAtIndex(i)=='\r'
                && getRxByteAtIndex(i+1)=='\n')
            {
                return i;
            }
        }
    }
    return 0;
}

uint8_t isShimmerBtCmd(uint8_t data)
{
    switch (data)
    {
    case INQUIRY_COMMAND:
    case DUMMY_COMMAND:
    case GET_SAMPLING_RATE_COMMAND:
    case TOGGLE_LED_COMMAND:
    case START_STREAMING_COMMAND:
    case GET_STATUS_COMMAND:
    case GET_VBATT_COMMAND:
    case GET_TRIAL_CONFIG_COMMAND:
    case START_SDBT_COMMAND:
    case GET_CONFIG_SETUP_BYTES_COMMAND:
    case STOP_STREAMING_COMMAND:
    case STOP_SDBT_COMMAND:
    case START_LOGGING_COMMAND:
    case STOP_LOGGING_COMMAND:
    case GET_A_ACCEL_CALIBRATION_COMMAND:
    case GET_MPU9150_GYRO_CALIBRATION_COMMAND:
    case GET_LSM303DLHC_MAG_CALIBRATION_COMMAND:
    case GET_LSM303DLHC_ACCEL_CALIBRATION_COMMAND:
    case GET_GSR_RANGE_COMMAND:
    case GET_ALL_CALIBRATION_COMMAND:
    case DEPRECATED_GET_DEVICE_VERSION_COMMAND:
    case GET_DEVICE_VERSION_COMMAND:
    case GET_FW_VERSION_COMMAND:
    case GET_CHARGE_STATUS_LED_COMMAND:
    case GET_BUFFER_SIZE_COMMAND:
    case GET_UNIQUE_SERIAL_COMMAND:
    case GET_LSM303DLHC_ACCEL_RANGE_COMMAND:
    case GET_LSM303DLHC_MAG_GAIN_COMMAND:
    case GET_LSM303DLHC_MAG_SAMPLING_RATE_COMMAND:
    case GET_LSM303DLHC_ACCEL_SAMPLING_RATE_COMMAND:
    case GET_LSM303DLHC_ACCEL_LPMODE_COMMAND:
    case GET_LSM303DLHC_ACCEL_HRMODE_COMMAND:
    case GET_MPU9150_GYRO_RANGE_COMMAND:
    case GET_BMP180_CALIBRATION_COEFFICIENTS_COMMAND:
    case GET_BMP280_CALIBRATION_COEFFICIENTS_COMMAND:
    case GET_MPU9150_SAMPLING_RATE_COMMAND:
    case GET_MPU9150_ACCEL_RANGE_COMMAND:
    case GET_BMPX80_PRES_OVERSAMPLING_RATIO_COMMAND:
    case GET_INTERNAL_EXP_POWER_ENABLE_COMMAND:
    case RESET_TO_DEFAULT_CONFIGURATION_COMMAND:
    case RESET_CALIBRATION_VALUE_COMMAND:
    case GET_MPU9150_MAG_SENS_ADJ_VALS_COMMAND:
    case GET_BT_COMMS_BAUD_RATE:
    case GET_CENTER_COMMAND:
    case GET_SHIMMERNAME_COMMAND:
    case GET_EXPID_COMMAND:
    case GET_MYID_COMMAND:
    case GET_NSHIMMER_COMMAND:
    case GET_CONFIGTIME_COMMAND:
    case GET_DIR_COMMAND:
    case GET_DERIVED_CHANNEL_BYTES:
    case GET_RWC_COMMAND:
    case UPD_SDLOG_CFG_COMMAND:
    case UPD_CALIB_DUMP_COMMAND:
    case GET_BT_VERSION_STR_COMMAND:
    case SET_LSM303DLHC_ACCEL_RANGE_COMMAND:
    case SET_LSM303DLHC_ACCEL_SAMPLING_RATE_COMMAND:
    case SET_LSM303DLHC_MAG_GAIN_COMMAND:
    case SET_CHARGE_STATUS_LED_COMMAND:
    case SET_LSM303DLHC_MAG_SAMPLING_RATE_COMMAND:
    case SET_LSM303DLHC_ACCEL_LPMODE_COMMAND:
    case SET_LSM303DLHC_ACCEL_HRMODE_COMMAND:
    case SET_MPU9150_GYRO_RANGE_COMMAND:
    case SET_MPU9150_SAMPLING_RATE_COMMAND:
    case SET_MPU9150_ACCEL_RANGE_COMMAND:
    case SET_BMPX80_PRES_OVERSAMPLING_RATIO_COMMAND:
    case SET_INTERNAL_EXP_POWER_ENABLE_COMMAND:
    case SET_GSR_RANGE_COMMAND:
    case SET_BT_COMMS_BAUD_RATE:
    case SET_CENTER_COMMAND:
    case SET_SHIMMERNAME_COMMAND:
    case SET_EXPID_COMMAND:
    case SET_MYID_COMMAND:
    case SET_NSHIMMER_COMMAND:
    case SET_CONFIGTIME_COMMAND:
    case SET_CRC_COMMAND:
    case SET_INSTREAM_RESPONSE_ACK_PREFIX_STATE:
    case SET_DATA_RATE_TEST:
    case SET_SAMPLING_RATE_COMMAND:
    case GET_DAUGHTER_CARD_ID_COMMAND:
    case SET_SENSORS_COMMAND:
    case GET_EXG_REGS_COMMAND:
    case SET_EXG_REGS_COMMAND:
    case GET_DAUGHTER_CARD_MEM_COMMAND:
    case SET_DAUGHTER_CARD_MEM_COMMAND:
    case SET_TRIAL_CONFIG_COMMAND:
    case GET_INFOMEM_COMMAND:
    case SET_INFOMEM_COMMAND:
    case GET_CALIB_DUMP_COMMAND:
    case SET_CALIB_DUMP_COMMAND:
    case SET_CONFIG_SETUP_BYTES_COMMAND:
    case SET_RWC_COMMAND:
    case SET_DERIVED_CHANNEL_BYTES:
    case SET_A_ACCEL_CALIBRATION_COMMAND:
    case SET_MPU9150_GYRO_CALIBRATION_COMMAND:
    case SET_LSM303DLHC_MAG_CALIBRATION_COMMAND:
    case SET_LSM303DLHC_ACCEL_CALIBRATION_COMMAND:
#if !USE_OLD_SD_SYNC_APPROACH
    case SET_SD_SYNC_COMMAND:
#endif
    case ACK_COMMAND_PROCESSED:
        return 1;
    default:
        return 0;
    }
}

void updateNumBytesInBtRxBufWhenLastProcessed(void)
{
    numBytesInBtRxBufWhenLastProcessed = getNumBytesInBtRxBuf();
}

uint16_t getNumBytesInBtRxBufWhenLastProcessed(void)
{
    return numBytesInBtRxBufWhenLastProcessed;
}

uint8_t areUnprocessedBytesInBtRxBuff(void)
{
    uint16_t numBytes = getNumBytesInBtRxBuf();
    return numBytes != 0 && numBytes != getNumBytesInBtRxBufWhenLastProcessed();
}
#endif

void btCommsProtocolInit(uint8_t (*newBtCmdToProcessCb)(void),
                         void (*handleBtRfCommStateChangeCb)(uint8_t),
                         void (*setMacIdCb)(uint8_t *),
                         uint8_t * actionPtr,
                         uint8_t * argsPtr)
{
    setBtCrcMode(CRC_OFF);
    numBytesInBtRxBufWhenLastProcessed = 0;
    indexOfFirstEol = 0;
    firstProcessFailTicks = 0;
    memset(unwrappedResponse, 0, sizeof(unwrappedResponse));

    newBtCmdToProcess_cb = newBtCmdToProcessCb;
    handleBtRfCommStateChange_cb = handleBtRfCommStateChangeCb;
    setMacId_cb = setMacIdCb;

    gActionPtr = actionPtr;
    gArgsPtr = argsPtr;

    commandBufPtr = getTxCmdBufPtr();
    expectedResponsePtr = BT_getExpResp();

#if BT_DMA_USED_FOR_RX
    btRxExp = BT_getExpResp();

    waitingForArgs = 0;
    waitingForArgsLength = 0;
    argsSize = 0;

    memset(btStatusStr, 0, sizeof(btStatusStr));

    memset(btRxBuffFullResponse, 0x00, sizeof(btRxBuffFullResponse) / sizeof(btRxBuffFullResponse[0]));
    setBtRxFullResponsePtr(btRxBuffFullResponse);

    memset(btRxBuff, 0x00, sizeof(btRxBuff) / sizeof(btRxBuff[0]));
    DMA2_init((uint16_t*) &UCA1RXBUF, (uint16_t*) btRxBuff, sizeof(btRxBuff));
    DMA2_transferDoneFunction(&Dma2ConversionDone);
//    DMA2SZ = 1U;
//    DMA2_enable();

#else
    gBtRxFifoPtr = getRxFifoPtr();
    isRn4678CmdDetectedOnBoot = 0;
#endif

    memset(btVerStrResponse, 0x00, sizeof(btVerStrResponse) / sizeof(btVerStrResponse[0]));
}

void triggerBtRfCommStateChangeCallback(bool state)
{
    if (handleBtRfCommStateChange_cb)
    {
        handleBtRfCommStateChange_cb(state);
    }
}

void triggerShimmerErrorState(void)
{
    while (1)
    {
        Board_ledOff(LED_ALL);
        _delay_cycles(24000000);
        Board_ledOn(LED_YELLOW);
        _delay_cycles(12000000);
        Board_ledOn(LED_RED);
        _delay_cycles(12000000);
        Board_ledOn(LED_BLUE);
        _delay_cycles(12000000);
        Board_ledOn(LED_GREEN1);
        _delay_cycles(12000000);
    }
}

uint8_t getBtVerStrLen(void)
{
    return strlen(btVerStrResponse);
}

char * getBtVerStrPtr(void)
{
    return &btVerStrResponse[0];
}

void setBtCrcMode(COMMS_CRC_MODE btCrcModeNew)
{
    btCrcMode = btCrcModeNew;
}

COMMS_CRC_MODE getBtCrcMode(void)
{
    return btCrcMode;
}
