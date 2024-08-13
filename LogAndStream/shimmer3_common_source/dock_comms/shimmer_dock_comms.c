/*
 * shimmer_dock_comms.c
 *
 *  Created on: 13 Aug 2024
 *      Author: MarkNolan
 */

#include "shimmer_dock_comms.h"

#include <stdint.h>
#include <string.h>

#include "../../shimmer_btsd.h"
#include "../5xx_HAL/hal_CRC.h"
#include "../5xx_HAL/hal_InfoMem.h"
#include "../5xx_HAL/hal_UartA0.h"
#include "../5xx_HAL/hal_RTC.h"
#include "../shimmer_boards/shimmer_boards.h"
#include "../Bluetooth_SD/shimmer_bt_comms.h"
#include "../Bluetooth_SD/RN4X.h"
#include "../CAT24C16/cat24c16.h"

uint8_t uartSteps, uartArgSize, uartArg2Wait, uartCrc2Wait, uartAction;
uint8_t uartRxBuf[UART_DATA_LEN_MAX];
uint8_t uartSendRspMac, uartSendRspVer, uartSendRspBat,

uartSendRspRtcConfigTime, uartSendRspCurrentTime, uartSendRspGdi,
        uartSendRspGdm, uartSendRspGim, uartSendRspBtVer, uartSendRspAck,
        uartSendRspBadCmd, uartSendRspBadArg, uartSendRspBadCrc;
uint8_t uartRespBuf[UART_RSP_PACKET_SIZE];

uint8_t uartDcMemLength, uartInfoMemLength;
uint16_t uartDcMemOffset, uartInfoMemOffset;

/* Externs left in main.c */
extern uint8_t TaskSet(TASK_FLAGS task_id);
extern void RwcCheck();
extern void Infomem2Names();
extern void eepromRead(uint16_t dataAddr, uint16_t dataSize, uint8_t *dataBuf);
extern void eepromWrite(uint16_t dataAddr, uint16_t dataSize, uint8_t *dataBuf);

extern uint8_t initializing, sensing;
extern uint8_t storedConfig[NV_NUM_RWMEM_BYTES];
extern uint8_t sdHeadText[SDHEAD_LEN];
extern uint8_t daughtCardId[PAGE_SIZE];
extern uint8_t battVal[3];

void UartCommsInit(void)
{
    uartSendRspAck = 0;
    uartSendRspMac = 0;
    uartSendRspVer = 0;
    uartSendRspBat = 0;
    uartSendRspRtcConfigTime = 0;
    uartSendRspCurrentTime = 0;
    uartSendRspGdi = 0;
    uartSendRspGdm = 0;
    uartSendRspGim = 0;
    uartSendRspBtVer = 0;
    uartSendRspBadCmd = 0;
    uartSendRspBadArg = 0;
    uartSendRspBadCrc = 0;
    uartSteps = 0;
    uartArgSize = 0;
    uartArg2Wait = 0;
    uartCrc2Wait = 0;
}

uint8_t UartCallback(uint8_t data)
{
    if (initializing)
    {
        return 0;
    }

    if (uartSteps)
    { //wait for: cmd, len, data, crc -> process
        if (uartSteps == UART_STEP_WAIT4_CMD)
        {
            uartAction = data;
            uartArgSize = UART_RXBUF_CMD;
            uartRxBuf[uartArgSize++] = data;
            switch (uartAction)
            {
            case UART_SET:
            case UART_GET:
                uartSteps = UART_STEP_WAIT4_LEN;
                return 0;
            default:
                uartSteps = 0;
                uartSendRspBadCmd = 1;
                TaskSet(TASK_DOCK_RESPOND);
                return 1;
            }
        }
        else if (uartSteps == UART_STEP_WAIT4_LEN)
        {
            uartSteps = UART_STEP_WAIT4_DATA;
            uartArgSize = UART_RXBUF_LEN;
            uartRxBuf[uartArgSize++] = data;
            uartArg2Wait = data;
            return 0;
        }
        else if (uartSteps == UART_STEP_WAIT4_DATA)
        {
            uartRxBuf[uartArgSize++] = data;
            if (!--uartArg2Wait)
            {
                uartCrc2Wait = 2;
                uartSteps = UART_STEP_WAIT4_CRC;
            }
            return 0;
        }
        else if (uartSteps == UART_STEP_WAIT4_CRC)
        {
            uartRxBuf[uartArgSize++] = data;
            if (!--uartCrc2Wait)
            {
                uartSteps = 0;
                uartArgSize = 0;
                TaskSet(TASK_DOCK_PROCESS_CMD);
                return 1;
            }
            else
                return 0;
        }
        else
        {
            uartSteps = 0;
            return 0;
        }
    }
    else
    {
        if (data == '$')
        {
            //uint8_t uart_cmd_str[4];
            uartAction = 0;
            uartArgSize = UART_RXBUF_START;
            uartRxBuf[UART_RXBUF_START] = '$';
            uartSteps = UART_STEP_WAIT4_CMD;
            return 0;
        }
    }
    return 0;
}

void UartProcessCmd(void)
{
    if (uartAction)
    {
        if (UartCheckCrc(uartRxBuf[UART_RXBUF_LEN] + 3))
        {
            if (uartAction == UART_GET)
            {  // get
                if (uartRxBuf[UART_RXBUF_COMP] == UART_COMP_SHIMMER)
                { // get shimmer
                    switch (uartRxBuf[UART_RXBUF_PROP])
                    {
                    case UART_PROP_MAC:
                        if ((uartRxBuf[UART_RXBUF_LEN] == 2))
                            uartSendRspMac = 1;
                        else
                            uartSendRspBadArg = 1;
                        break;
                    case UART_PROP_VER:
                        if ((uartRxBuf[UART_RXBUF_LEN] == 2))
                            uartSendRspVer = 1;
                        else
                            uartSendRspBadArg = 1;
                        break;
                    case UART_PROP_RWC_CFG_TIME:
                        if ((uartRxBuf[UART_RXBUF_LEN] == 2))
                            uartSendRspRtcConfigTime = 1;
                        else
                            uartSendRspBadArg = 1;
                        break;
                    case UART_PROP_CURR_LOCAL_TIME:
                        if ((uartRxBuf[UART_RXBUF_LEN] == 2))
                            uartSendRspCurrentTime = 1;
                        else
                            uartSendRspBadArg = 1;
                        break;
                    case UART_PROP_INFOMEM:
                        uartInfoMemLength = uartRxBuf[UART_RXBUF_DATA];
                        uartInfoMemOffset = (uint16_t) uartRxBuf[UART_RXBUF_DATA
                                + 1]
                                + (((uint16_t) uartRxBuf[UART_RXBUF_DATA + 2])
                                        << 8);
                        //if((uartInfoMemLength<=128) && (uartInfoMemOffset<=0x19ff)  && (uartInfoMemOffset>=0x1800)
                        //      && (uartInfoMemLength+uartInfoMemOffset<=0x1a00)){
                        //   uartInfoMemOffset -= 0x1800;
                        if ((uartInfoMemLength <= 0x80)
                                && (uartInfoMemOffset <= 0x01ff)
                                && (uartInfoMemLength + uartInfoMemOffset
                                        <= 0x0200))
                        {
                            uartSendRspGim = 1;
                        }
                        else
                        {
                            uartSendRspBadArg = 1;
                        }
                        break;
                    default:
                        uartSendRspBadCmd = 1;
                        break;
                    }
                }
                else if (uartRxBuf[UART_RXBUF_COMP] == UART_COMP_BAT)
                { // get battery
                    switch (uartRxBuf[UART_RXBUF_PROP])
                    {
                    case UART_PROP_VALUE:
                        if ((uartRxBuf[UART_RXBUF_LEN] == 2))
                            uartSendRspBat = 1; // already in the callback function
                        else
                            uartSendRspBadArg = 1;
                        break;
                    default:
                        uartSendRspBadCmd = 1;
                        break;
                    }
                }
                else if (uartRxBuf[UART_RXBUF_COMP] == UART_COMP_DAUGHTER_CARD)
                { // get daughter card
                    switch (uartRxBuf[UART_RXBUF_PROP])
                    {
                    case UART_PROP_CARD_ID:
                        uartDcMemLength = uartRxBuf[UART_RXBUF_DATA];
                        uartDcMemOffset = (uint16_t) uartRxBuf[UART_RXBUF_DATA
                                + 1];
                        if ((uartDcMemLength <= 16) && (uartDcMemOffset <= 15)
                                && ((uint16_t) uartDcMemLength + uartDcMemOffset
                                        <= 16))
                        {
                            uartSendRspGdi = 1;
                        }
                        else
                            uartSendRspBadArg = 1;
                        break;
                    case UART_PROP_CARD_MEM:
                        uartDcMemLength = uartRxBuf[UART_RXBUF_DATA];
                        uartDcMemOffset = (uint16_t) uartRxBuf[UART_RXBUF_DATA
                                + 1]
                                + (((uint16_t) uartRxBuf[UART_RXBUF_DATA + 2])
                                        << 8);
                        if ((uartDcMemLength <= 128)
                                && (uartDcMemOffset <= 2031)
                                && ((uint16_t) uartDcMemLength + uartDcMemOffset
                                        <= 2032))
                        {
                            uartSendRspGdm = 1;
                        }
                        else
                            uartSendRspBadArg = 1;
                        break;
                    default:
                        uartSendRspBadCmd = 1;
                        break;
                    }
                }
                else if (uartRxBuf[UART_RXBUF_COMP] == UART_COMP_BT)
                {
                    switch (uartRxBuf[UART_RXBUF_PROP])
                    {
                    case UART_PROP_VER:
                        if ((uartRxBuf[UART_RXBUF_LEN] == 2))
                            uartSendRspBtVer = 1;
                        else
                            uartSendRspBadArg = 1;
                        break;
                    default:
                        uartSendRspBadCmd = 1;
                        break;
                    }
                }
                else
                {
                    uartSendRspBadCmd = 1;
                }
            }
            else if (uartAction == UART_SET)
            { // set
                if (uartRxBuf[UART_RXBUF_COMP] == UART_COMP_SHIMMER)
                { // set shimmer
                    switch (uartRxBuf[UART_RXBUF_PROP])
                    {
                    case UART_PROP_RWC_CFG_TIME:
                        if ((uartRxBuf[UART_RXBUF_LEN] == 10))
                        {
                            setRwcTime(uartRxBuf + UART_RXBUF_DATA);
                            RwcCheck();
                            storedConfig[NV_SD_TRIAL_CONFIG0] &=
                                    ~SDH_RTC_SET_BY_BT;
                            InfoMem_write((uint8_t*) NV_SD_TRIAL_CONFIG0,
                                          &storedConfig[NV_SD_TRIAL_CONFIG0],
                                          1);
                            sdHeadText[SDH_TRIAL_CONFIG0] =
                                    storedConfig[NV_SD_TRIAL_CONFIG0];
                            uartSendRspAck = 1;
                        }
                        else
                        {
                            uartSendRspBadArg = 1;
                        }
                        break;
                    case UART_PROP_INFOMEM:
                        uartInfoMemLength = uartRxBuf[UART_RXBUF_DATA];
                        uartInfoMemOffset = (uint16_t) uartRxBuf[UART_RXBUF_DATA
                                + 1]
                                + (((uint16_t) uartRxBuf[UART_RXBUF_DATA + 2])
                                        << 8);
                        //if((uartInfoMemLength<=128) && (uartInfoMemOffset<=0x19ff) && (uartInfoMemOffset>=0x1800)
                        //      && (uartInfoMemLength+uartInfoMemOffset<=0x1a00)) {
                        //   uartInfoMemOffset -= 0x1800;
                        if ((uartInfoMemLength <= 0x80)
                                && (uartInfoMemOffset <= 0x01ff)
                                && (uartInfoMemLength + uartInfoMemOffset
                                        <= 0x0200))
                        {

                            if (uartInfoMemOffset
                                    == (INFOMEM_SEG_C_ADDR - INFOMEM_OFFSET))
                            {
                                /* Read MAC address so it is not forgotten */
                                InfoMem_read((uint8_t*) NV_MAC_ADDRESS, getMacIdBytesPtr(),
                                             6);
                            }
                            if (uartInfoMemOffset
                                    == (INFOMEM_SEG_D_ADDR - INFOMEM_OFFSET))
                            {
                                /* Check if unit is SR47-4 or greater.
                                 * If so, amend configuration byte 2 of ADS chip 1 to have bit 3 set to 1.
                                 * This ensures clock lines on ADS chip are correct
                                 */
                                if ((daughtCardId[DAUGHT_CARD_ID] == EXP_BRD_EXG_UNIFIED)
                                        && (daughtCardId[DAUGHT_CARD_REV] >= 4))
                                {
                                    *(uartRxBuf + UART_RXBUF_DATA + 3
                                            + NV_EXG_ADS1292R_1_CONFIG2) |= 8;
                                }
                            }
#if !IS_SUPPORTED_TCXO
                            if (uartInfoMemOffset <= NV_SD_TRIAL_CONFIG1
                                    && NV_SD_TRIAL_CONFIG1 <= uartInfoMemOffset + uartInfoMemLength)
                            {
                                uint8_t tcxoInfomemOffset = NV_SD_TRIAL_CONFIG1 - uartInfoMemOffset;
                                uartRxBuf[3 + tcxoInfomemOffset] &= ~SDH_TCXO;
                            }
#endif
                            /* Write received UART bytes to infomem */
                            InfoMem_write((uint8_t*) uartInfoMemOffset,
                                          uartRxBuf + UART_RXBUF_DATA + 3,
                                          uartInfoMemLength);
                            if (uartInfoMemOffset
                                    == (INFOMEM_SEG_C_ADDR - INFOMEM_OFFSET))
                            {
                                /* Re-write MAC address to Infomem */
                                InfoMem_write((uint8_t*) NV_MAC_ADDRESS,
                                              getMacIdBytesPtr(), 6);
                            }
                            /* Reload latest infomem bytes to RAM */
                            InfoMem_read((uint8_t*) uartInfoMemOffset,
                                         storedConfig + uartInfoMemOffset,
                                         uartInfoMemLength);
                            Infomem2Names();
                            uartSendRspAck = 1;
                        }

                        else
                        {
                            uartSendRspBadArg = 1;
                        }
                        break;
                    default:
                        uartSendRspBadCmd = 1;
                        break;
                    }
                }
                else if (uartRxBuf[UART_RXBUF_COMP] == UART_COMP_DAUGHTER_CARD)
                { // set daughter card id
                    switch (uartRxBuf[UART_RXBUF_PROP])
                    {
                    case UART_PROP_CARD_ID:
//#if FACTORY_TEST
                        uartDcMemLength = uartRxBuf[UART_RXBUF_DATA];
                        uartDcMemOffset = uartRxBuf[UART_RXBUF_DATA + 1];
                        if ((uartDcMemLength <= 16) && (uartDcMemOffset < 16))
                        {
                            // Write (up to) 16 bytes to eeprom
                            eepromWrite(uartDcMemOffset,
                                            (uint16_t) uartDcMemLength,
                                            uartRxBuf + UART_RXBUF_DATA + 2U);
                            // Copy new bytes to active daughter card byte array
                            memcpy(daughtCardId + ((uint8_t) uartDcMemOffset),
                                   uartRxBuf + UART_RXBUF_DATA + 2,
                                   uartDcMemLength);
                            uartSendRspAck = 1;
                        }
                        else
                        {
                            uartSendRspBadArg = 1;
                        }
                        break;
//#else
//                        uartSendRspBadArg = 1;
//                        break;
//#endif
                    case UART_PROP_CARD_MEM:
                        uartDcMemLength = uartRxBuf[UART_RXBUF_DATA];
                        uartDcMemOffset = (uint16_t) uartRxBuf[UART_RXBUF_DATA
                                + 1]
                                + (((uint16_t) uartRxBuf[UART_RXBUF_DATA + 2])
                                        << 8);
                        if ((uartDcMemLength <= 128)
                                && (uartDcMemOffset <= 2031)
                                && ((uint16_t) uartDcMemLength + uartDcMemOffset
                                        <= 2032))
                        {
                            eepromWrite(uartDcMemOffset + 16U,
                                            (uint16_t) uartDcMemLength,
                                            uartRxBuf + UART_RXBUF_DATA + 3U);
                            uartSendRspAck = 1;
                        }
                        else
                        {
                            uartSendRspBadArg = 1;
                        }
                        break;
                    default:
                        uartSendRspBadCmd = 1;
                        break;
                    }
                }
                else
                {
                    uartSendRspBadCmd = 1;
                }
            }
        }
        else
        {
            uartSendRspBadCrc = 1;
        }
        TaskSet(TASK_DOCK_RESPOND);
    }
}

void UartSendRsp(void)
{
    uint8_t uart_resp_len = 0, cr = 0;
    uint16_t uartRespCrc;

    if (uartSendRspAck)
    {
        uartSendRspAck = 0;
        *(uartRespBuf + uart_resp_len++) = '$';
        *(uartRespBuf + uart_resp_len++) = UART_ACK_RESPONSE;
    }
    else if (uartSendRspBadCmd)
    {
        uartSendRspBadCmd = 0;
        *(uartRespBuf + uart_resp_len++) = '$';
        *(uartRespBuf + uart_resp_len++) = UART_BAD_CMD_RESPONSE;
    }
    else if (uartSendRspBadArg)
    {
        uartSendRspBadArg = 0;
        *(uartRespBuf + uart_resp_len++) = '$';
        *(uartRespBuf + uart_resp_len++) = UART_BAD_ARG_RESPONSE;
    }
    else if (uartSendRspBadCrc)
    {
        uartSendRspBadCrc = 0;
        *(uartRespBuf + uart_resp_len++) = '$';
        *(uartRespBuf + uart_resp_len++) = UART_BAD_CRC_RESPONSE;
    }
    else if (uartSendRspMac)
    {
        uartSendRspMac = 0;
        *(uartRespBuf + uart_resp_len++) = '$';
        *(uartRespBuf + uart_resp_len++) = UART_RESPONSE;
        *(uartRespBuf + uart_resp_len++) = 8;
        *(uartRespBuf + uart_resp_len++) = UART_COMP_SHIMMER;
        *(uartRespBuf + uart_resp_len++) = UART_PROP_MAC;
        memcpy(uartRespBuf + uart_resp_len, getMacIdBytesPtr(), 6);
        uart_resp_len += 6;
    }
    else if (uartSendRspVer)
    {
        uartSendRspVer = 0;
        *(uartRespBuf + uart_resp_len++) = '$';
        *(uartRespBuf + uart_resp_len++) = UART_RESPONSE;
        *(uartRespBuf + uart_resp_len++) = 9;
        *(uartRespBuf + uart_resp_len++) = UART_COMP_SHIMMER;
        *(uartRespBuf + uart_resp_len++) = UART_PROP_VER;
        *(uartRespBuf + uart_resp_len++) = DEVICE_VER;
        *(uartRespBuf + uart_resp_len++) = (FW_IDENTIFIER & 0xFF);
        *(uartRespBuf + uart_resp_len++) = ((FW_IDENTIFIER & 0xFF00) >> 8);
        *(uartRespBuf + uart_resp_len++) = (FW_VER_MAJOR & 0xFF);
        *(uartRespBuf + uart_resp_len++) = ((FW_VER_MAJOR & 0xFF00) >> 8);
        *(uartRespBuf + uart_resp_len++) = (FW_VER_MINOR);
        *(uartRespBuf + uart_resp_len++) = (FW_VER_REL
                + ((FACTORY_TEST) ? 200 : 0));
    }
    else if (uartSendRspBat)
    {
        uartSendRspBat = 0;
        *(uartRespBuf + uart_resp_len++) = '$';
        *(uartRespBuf + uart_resp_len++) = UART_RESPONSE;
        *(uartRespBuf + uart_resp_len++) = 5;
        *(uartRespBuf + uart_resp_len++) = UART_COMP_BAT;
        *(uartRespBuf + uart_resp_len++) = UART_PROP_VALUE;
        memcpy(uartRespBuf + uart_resp_len, battVal, 3);
        uart_resp_len += 3;
    }
    else if (uartSendRspRtcConfigTime)
    {
        uartSendRspRtcConfigTime = 0;
        *(uartRespBuf + uart_resp_len++) = '$';
        *(uartRespBuf + uart_resp_len++) = UART_RESPONSE;
        *(uartRespBuf + uart_resp_len++) = 10;
        *(uartRespBuf + uart_resp_len++) = UART_COMP_SHIMMER;
        *(uartRespBuf + uart_resp_len++) = UART_PROP_RWC_CFG_TIME;
        memcpy(uartRespBuf + uart_resp_len, (uint8_t*) getRwcConfigTimePtr(), 8);
        uart_resp_len += 8;
    }
    else if (uartSendRspCurrentTime)
    {
        uartSendRspCurrentTime = 0;
        uint64_t rwc_curr_time_64;
        *(uartRespBuf + uart_resp_len++) = '$';
        rwc_curr_time_64 = getRwcTime();
        *(uartRespBuf + uart_resp_len++) = UART_RESPONSE;
        *(uartRespBuf + uart_resp_len++) = 10;
        *(uartRespBuf + uart_resp_len++) = UART_COMP_SHIMMER;
        *(uartRespBuf + uart_resp_len++) = UART_PROP_CURR_LOCAL_TIME;
        memcpy(uartRespBuf + uart_resp_len, (uint8_t*) (&rwc_curr_time_64), 8);
        uart_resp_len += 8;
    }
    else if (uartSendRspGdi)
    {
        uartSendRspGdi = 0;
        *(uartRespBuf + uart_resp_len++) = '$';
        *(uartRespBuf + uart_resp_len++) = UART_RESPONSE;
        *(uartRespBuf + uart_resp_len++) = uartDcMemLength + 2;
        *(uartRespBuf + uart_resp_len++) = UART_COMP_DAUGHTER_CARD;
        *(uartRespBuf + uart_resp_len++) = UART_PROP_CARD_ID;
        if ((uartDcMemLength + uart_resp_len) < UART_RSP_PACKET_SIZE)
        {
            //   CAT24C16_init();
            //  CAT24C16_read(uartDcMemOffset, (uint16_t) uartDcMemLength,
            //               (uartRespBuf + uart_resp_len));
            //  CAT24C16_powerOff();

            memcpy(uartRespBuf + uart_resp_len, daughtCardId + uartDcMemOffset, uartDcMemLength);
            uart_resp_len += uartDcMemLength;
        }
    }
    else if (uartSendRspGdm)
    {
        uartSendRspGdm = 0;
        *(uartRespBuf + uart_resp_len++) = '$';
        *(uartRespBuf + uart_resp_len++) = UART_RESPONSE;
        *(uartRespBuf + uart_resp_len++) = uartDcMemLength + 2;
        *(uartRespBuf + uart_resp_len++) = UART_COMP_DAUGHTER_CARD;
        *(uartRespBuf + uart_resp_len++) = UART_PROP_CARD_MEM;
        if ((uartDcMemLength + uart_resp_len) < UART_RSP_PACKET_SIZE)
        {
            if (!sensing)
            {
                eepromRead(uartDcMemOffset + 16U,
                                (uint16_t) uartDcMemLength,
                                (uartRespBuf + uart_resp_len));
            }
            else
            {
                memset(uartRespBuf + uart_resp_len, 0xff, uartDcMemLength);
            }
            uart_resp_len += uartDcMemLength;
        }
    }
    else if (uartSendRspGim)
    {
        uartSendRspGim = 0;
        *(uartRespBuf + uart_resp_len++) = '$';
        *(uartRespBuf + uart_resp_len++) = UART_RESPONSE;
        *(uartRespBuf + uart_resp_len++) = uartInfoMemLength + 2;
        *(uartRespBuf + uart_resp_len++) = UART_COMP_SHIMMER;
        *(uartRespBuf + uart_resp_len++) = UART_PROP_INFOMEM;
        if ((uartInfoMemLength + uart_resp_len) < UART_RSP_PACKET_SIZE)
        {
            InfoMem_read((void*) uartInfoMemOffset, uartRespBuf + uart_resp_len,
                         uartInfoMemLength);
        }
        uart_resp_len += uartInfoMemLength;
    }
    else if (uartSendRspBtVer)
    {
        uartSendRspBtVer = 0;

        uint8_t btVerStrLen = getBtVerStrLen();

        *(uartRespBuf + uart_resp_len++) = '$';
        *(uartRespBuf + uart_resp_len++) = UART_RESPONSE;
        *(uartRespBuf + uart_resp_len++) = 2U + btVerStrLen;
        *(uartRespBuf + uart_resp_len++) = UART_COMP_BT;
        *(uartRespBuf + uart_resp_len++) = UART_PROP_VER;

        memcpy(uartRespBuf + uart_resp_len, getBtVerStrPtr(), btVerStrLen);
        uart_resp_len += btVerStrLen;
    }

    uartRespCrc = CRC_data(uartRespBuf, uart_resp_len);
    *(uartRespBuf + uart_resp_len++) = uartRespCrc & 0xff;
    *(uartRespBuf + uart_resp_len++) = (uartRespCrc & 0xff00) >> 8;
    if (cr)
    { // character return was in the old commands
        *(uartRespBuf + uart_resp_len++) = 0x0d;
        *(uartRespBuf + uart_resp_len++) = 0x0a;
    }

    UART_write(uartRespBuf, uart_resp_len);
}

uint8_t UartCheckCrc(uint8_t len)
{
    if (len > UART_DATA_LEN_MAX)
        return 0;
    uint16_t uart_rx_crc, uart_calc_crc;
    uart_calc_crc = CRC_data(uartRxBuf, len);
    uart_rx_crc = (uint16_t) uartRxBuf[len];
    uart_rx_crc += ((uint16_t) uartRxBuf[(uint8_t) (len + 1)]) << 8;
    return (uart_rx_crc == uart_calc_crc);
}

