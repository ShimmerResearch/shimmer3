/*
 * sd_sync.c
 *
 *  Code base created in 2014
 *      Author: Weibo Pan
 *  Moved to it's own driver: 11 Oct 2022
 *      Author: Mark Nolan
 */

#include "sd_sync.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "msp430.h"

#include "../5xx_HAL/hal_RTC.h"
#include "shimmer_bt_comms.h"
#include "RN4X.h"
#include "../../shimmer_btsd.h"
#include "../shimmer_externs.h"

uint8_t nodeName[MAX_NODES][MAX_CHARS], shortExpFlag;
uint8_t syncNodeCnt, syncNodeNum, syncThis, syncNodeSucc, nReboot,
        currNodeSucc, cReboot;
uint8_t syncSuccC, syncSuccN, syncCurrNode, syncCurrNodeDone,
        rcFirstOffsetRxed;
uint8_t rcNodeR10Cnt;
uint32_t firstOutlier, rcWindowC, rcNodeReboot;
uint32_t estLen, estLen3, syncCnt;
uint32_t nodeSucc, nodeSuccFull;
uint64_t myLocalTimeLong, myCenterTimeLong, myTimeDiffLong;
uint64_t myTimeDiffLongMin;
uint64_t myTimeDiffArr[SYNC_TRANS_IN_ONE_COMM];
uint8_t myTimeDiffFlagArr[SYNC_TRANS_IN_ONE_COMM];
uint16_t syncCurrNodeExpire, syncNodeWinExpire;
uint8_t centerName[MAX_CHARS], myTimeDiffLongFlag;
#if USE_OLD_SD_SYNC_APPROACH
uint8_t getRcomm;
#endif
uint8_t myTimeDiffLongFlagMin;
uint8_t syncResp[SYNC_PACKET_MAX_SIZE], btSdSyncIsRunning;
uint8_t myTimeDiff[SYNC_PACKET_PAYLOAD_SIZE];

//TODO figure out how best to do away with the need for externs
void (*btStartCb)(void);
void (*btStopCb)(uint8_t);
uint8_t (*taskSetCb)(TASK_FLAGS);

extern uint8_t onSingleTouch, stopLogging;
extern uint8_t sdHeadText[SDHEAD_LEN];
extern uint8_t all0xff[7U];

void sdSyncInit(void (*btStart_cb)(void),
                void (*btStop_cb)(uint8_t),
                uint8_t (*taskSet_cb)(TASK_FLAGS))
{
    btSdSyncIsRunning = 0;

    btStartCb = btStart_cb;
    btStopCb = btStop_cb;
    taskSetCb = taskSet_cb;

    firstOutlier = 1;

#if USE_OLD_SD_SYNC_APPROACH
    setRcommVar(0);
#endif

    resetSyncVariablesNode();
    resetSyncVariablesCenter();
    resetSyncNodeArray();
    resetSyncRcNodeR10Cnt();
    resetSyncVariablesBeforeParseConfig();
    resetSyncVariablesDuringSyncStart();
}

void resetMyTimeDiff(void)
{
    memset(myTimeDiff, 0xff, sizeof(myTimeDiff));
}

void resetMyTimeDiffArrays(void)
{
    memset(myTimeDiffFlagArr, 0xff, sizeof(myTimeDiffFlagArr));
    memset(myTimeDiffArr, 0, sizeof(myTimeDiffArr));
}

void resetMyTimeDiffLongMin(void)
{
    myTimeDiffLongMin = 0;
    myTimeDiffLongFlagMin = 0;
}

uint8_t * getMyTimeDiffPtr(void)
{
    return &myTimeDiff[0];
}

#if USE_OLD_SD_SYNC_APPROACH
void setRcommVar(uint8_t val)
{
    getRcomm = val;
}

uint8_t getRcommVar(void)
{
    return getRcomm;
}
#endif

void setSyncResp(uint8_t *args, uint8_t count)
{
    memcpy(syncResp, args, count);
}

uint8_t isBtSdSyncRunning(void)
{
    return btSdSyncIsRunning;
}

uint8_t getSyncNodeNum(void)
{
    return syncNodeNum;
}

uint8_t* getSyncNodeNamePtrForIndex(uint8_t index)
{
    return &nodeName[index][0];
}

uint8_t* getSyncCenterNamePtr(void)
{
    return &centerName[0];
}

uint8_t getRcFirstOffsetRxed(void)
{
    return rcFirstOffsetRxed;
}

uint8_t getSyncSuccC(void)
{
    return syncSuccC;
}

uint8_t getSyncSuccN(void)
{
    return syncSuccN;
}

uint8_t getSyncCnt(void)
{
    return syncCnt;
}

void saveLocalTime(void)
{
    myLocalTimeLong = getRwcTime();
}

void resetSyncRcNodeR10Cnt(void)
{
    rcNodeR10Cnt = 0;
}

void resetSyncVariablesBeforeParseConfig(void)
{
    syncNodeNum = 0;
    nodeSuccFull = 0;

    memset(centerName, 0, MAX_CHARS);
}

void resetSyncVariablesDuringSyncStart(void)
{
    resetSyncRcNodeR10Cnt();
    syncCnt = 0;
    syncThis = 0;
    syncSuccC = 0;
    syncSuccN = 0;

    resetMyTimeDiffLongMin();
    resetMyTimeDiffArrays();
    resetMyTimeDiff();

    /* Needs to be reset between sessions so that blue LED goes solid again */
    rcFirstOffsetRxed = 0;
}

void resetSyncVariablesCenter(void)
{
    cReboot = 0;
    syncNodeCnt = 0;
    nodeSucc = 0;
    syncCurrNode = 0;
    syncCurrNodeDone = 0;
    currNodeSucc = 0;
}

void resetSyncVariablesNode(void)
{
    syncNodeSucc = 0;
    nReboot = 0;
}

void resetSyncNodeArray(void)
{
    int node_i;
    for (node_i = 0; node_i < MAX_NODES; node_i++)
    {
        *nodeName[node_i] = '\0';
    }
}

uint16_t parseSyncEstExpLen(uint8_t estExpLenLsb, uint8_t estExpLenMsb)
{
    uint16_t temp16;
    uint32_t temp32;

    temp32 = (uint32_t) estExpLenLsb;
    temp32 += ((uint32_t) estExpLenMsb) << 8;
    temp16 = (uint16_t) (temp32 & 0xffff);

    setSyncEstExpLen(temp32);

    return temp16;
}

/* Usually a value of 5 for a 'Short' estimate trial duration (<1hr), 45 for a
 * 'Medium' estimated trial duration (<3hrs), or 200 for a 'Long' estimated
 * trial duration (>3hrs) */
void setSyncEstExpLen(uint32_t est_exp_len)
{
    if (est_exp_len < 10)
    {
        shortExpFlag = 1;
        est_exp_len = 0xffff;
    }
    else
    {
        shortExpFlag = 0;
        if (est_exp_len > 180)
        {
            est_exp_len = 180;
        }
    }
    estLen = est_exp_len * 60;
    estLen3 = estLen / 3;
}

void parseSyncNodeNamesFromConfig(uint8_t *storedConfigPtr)
{
    uint8_t i;
    uint8_t node_addr[6], byte_l, byte_h;

    while (memcmp(all0xff, storedConfigPtr + NV_NODE0 + syncNodeNum * 6, 6)
            && (syncNodeNum < MAX_NODES))
    {
        memcpy(node_addr, storedConfigPtr + NV_NODE0 + syncNodeNum * 6, 6);
        for (i = 0; i < 6; i++)
        {
            byte_h = (node_addr[i] >> 4) & 0x0f;
            byte_l = node_addr[i] & 0x0f;
            nodeName[syncNodeNum][i * 2] = byte_h
                    + (byte_h > 9 ? 'A' - 10 : '0');
            nodeName[syncNodeNum][i * 2 + 1] = byte_l
                    + (byte_l > 9 ? 'A' - 10 : '0');
        }
        *(nodeName[syncNodeNum] + 12) = 0;
        nodeSuccFull |= SyncNodeShift(syncNodeNum);
        syncNodeNum++;
    }
}

void parseSyncCenterNameFromConfig(uint8_t *storedConfigPtr)
{
    uint8_t i;
    uint8_t center_addr[6], byte_l, byte_h;

    memcpy(center_addr, storedConfigPtr + NV_CENTER, 6);
    for (i = 0; i < 6; i++)
    {
        byte_h = (center_addr[i] >> 4) & 0x0f;
        byte_l = center_addr[i] & 0x0f;
        centerName[i * 2] = byte_h + (byte_h > 9 ? 'A' - 10 : '0');
        centerName[i * 2 + 1] = byte_l
                + (byte_l > 9 ? 'A' - 10 : '0');
    }
    *(centerName + 12) = 0;
}

void parseSyncCenterNameFromCfgFile(uint8_t *storedConfigPtr, char *equals)
{
    uint8_t string_length = 0;
    uint8_t i, pchar[3], center_addr[6];

    string_length = strlen(equals);
    if (string_length > MAX_CHARS)
    {
        string_length = MAX_CHARS - 1;
    }
    else if (string_length >= 2)
    {
        string_length -= 2;
    }
    else
    {
        string_length = 0;
    }
    if (string_length == 12)
    {
        memcpy((char*) centerName, equals, string_length);
        *(centerName + string_length) = 0;
        pchar[2] = 0;
        for (i = 0; i < 6; i++)
        {
            pchar[0] = *(centerName + i * 2);
            pchar[1] = *(centerName + i * 2 + 1);
            center_addr[i] = strtoul((char*) pchar, 0, 16);
        }
        memcpy(storedConfigPtr + NV_CENTER, center_addr, 6);
    }
}

void parseSyncNodeNameFromCfgFile(uint8_t *storedConfigPtr, char *equals)
{
    uint8_t string_length = 0;
    uint8_t i, pchar[3], node_addr[6];

    string_length = strlen(equals);

    if (string_length > MAX_CHARS)
    {
        string_length = MAX_CHARS - 1;
    }
    else if (string_length >= 2)
    {
        string_length -= 2;
    }
    else
    {
        string_length = 0;
    }
    if ((string_length == 12) && (syncNodeNum < MAX_NODES))
    {
        memcpy((char*) nodeName[syncNodeNum], equals, string_length);
        *(nodeName[syncNodeNum] + string_length) = 0;
        pchar[2] = 0;
        for (i = 0; i < 6; i++)
        {
            pchar[0] = *(nodeName[syncNodeNum] + i * 2);
            pchar[1] = *(nodeName[syncNodeNum] + i * 2 + 1);
            node_addr[i] = strtoul((char*) pchar, 0, 16);
        }
        memcpy(storedConfigPtr + NV_NODE0 + syncNodeNum * 6, node_addr, 6);
        nodeSuccFull |= SyncNodeShift(syncNodeNum);
        syncNodeNum++;
    }
}

void checkSyncCenterName(void)
{
    if (strlen((char*) centerName) == 0) // if no center is appointed, let this guy be the center
    {
        strcpy((char*) centerName, "000000000000");
    }
}

void BtSdSyncStop(void)
{
    btSdSyncIsRunning = 0;
    rcFirstOffsetRxed = 0;
    CommTimerStop();
    btStopCb(1U);
}

void BtSdSyncStart(void)
{
    btSdSyncIsRunning = 1;

    if (SYNC_WINDOW_C < (estLen3 - SYNC_BOOT))
    {
        rcWindowC = SYNC_WINDOW_C;
    }
    else
    {
        rcWindowC = (estLen3 - SYNC_BOOT);
    }

    if (SYNC_NODE_REBOOT < (estLen3 / SYNC_WINDOW_N))
    {
        rcNodeReboot = SYNC_NODE_REBOOT;
    }
    else
    {
        rcNodeReboot = (estLen3 / SYNC_WINDOW_N);
    }

    if (sdHeadText[SDH_TRIAL_CONFIG0] & SDH_IAMMASTER)
    {
        firstOutlier = nodeSuccFull;
    }
    else
    {
        firstOutlier = 1;
    }

    resetSyncVariablesDuringSyncStart();

    CommTimerStart();
}

/* Sync Center only. Sends a single sync packet to a node (T10 = transmit 10
 * bytes) and sets getRcomm = 1 waiting for single byte response from a node.
 * Once received, TASK_RCCENTERR1 is set. The sync packet contains the 8-byte
 * real-world clock value in ticks, LSB order. */
void SyncCenterT10(void)
{
    uint8_t resPacket[SYNC_PACKET_MAX_SIZE];
    uint16_t packet_length = 0;

#if USE_OLD_SD_SYNC_APPROACH
    getRcomm = 1;

    *(resPacket + packet_length++) = ACK_COMMAND_PROCESSED;
#else
    *(resPacket + packet_length++) = SET_SD_SYNC_COMMAND;
#endif
    *(resPacket + packet_length++) = shimmerStatus.isSensing;
    myLocalTimeLong = getRwcTime();
    *(uint64_t*) (resPacket + packet_length) = myLocalTimeLong;
    packet_length += 8;

    if (BT_SD_SYNC_CRC_MODE)
    {
        calculateCrcAndInsert(BT_SD_SYNC_CRC_MODE, resPacket, packet_length);
        packet_length += BT_SD_SYNC_CRC_MODE;
    }

    BT_write(resPacket, packet_length, SHIMMER_CMD);
}

/* Sync Center only. Called by TASK_RCCENTERR1 when a center receives a byte
 * back from a node (R1 = received 1 byte) after it has sent a sync packet. If
 * ACK_COMMAND_PROCESSED is received, ?, Else the center sends the next sync
 * packet. */
void SyncCenterR1(void)
{
    if (syncResp[0] != SYNC_FINISHED)
    {
//        DMA2SZ = 1;
//        if (isBtDeviceRn4678())
//        {
//            // Clear out the BT UART receive buffer by reading its register
//            UCA1RXBUF;
//        }
//        DMA2_enable();

//#if BT_DMA_USED_FOR_RX
//        setDmaWaitingForResponse(1U);
//#endif
        /* If status strings are enabled the DMA will already be set to listen
         * for bytes so we only need to set if here if that is not the case */
//        if(!areBtStatusStringsEnabled())
//        {
//            setDmaWaitingForResponse(1U);
//        }
        SyncCenterT10();
        syncCurrNodeExpire = syncCurrNode + SYNC_EXTEND * SYNC_FACTOR;
    }
    else
    {
        currNodeSucc = 1;
        if (firstOutlier & SyncNodeShift(syncNodeCnt))
            firstOutlier &= ~SyncNodeShift(syncNodeCnt);
        else
            nodeSucc |= SyncNodeShift(syncNodeCnt);
    }
}

uint32_t SyncNodeShift(uint8_t shift_value)
{
    uint32_t sync_node_shift = 0x01;
    sync_node_shift <<= shift_value;
    return sync_node_shift;
}

/* Sync Node only. Called by TASK_RCNODER10 after a node has received a 10 byte
 * sync packet (R10 = received 10 bytes) from a center */
void SyncNodeR10(void)
{
    uint8_t nodeResponse;

    // only nodes do this
#if USE_OLD_SD_SYNC_APPROACH
    if (syncResp[SYNC_PACKET_IDX_ACK] == ACK_COMMAND_PROCESSED)
#else
    if(!BT_SD_SYNC_CRC_MODE || checkCrc(BT_SD_SYNC_CRC_MODE, &syncResp[0], SYNC_PACKET_SIZE_CMD+SYNC_PACKET_PAYLOAD_SIZE))
#endif
    { //if received the correct 6 bytes:
        uint8_t sd_tolog;
        sd_tolog = syncResp[SYNC_PACKET_FLG_IDX];
        myCenterTimeLong = *(uint64_t*) (syncResp + SYNC_PACKET_TIME_IDX); // get myCenterTimeLong

        if (myLocalTimeLong > myCenterTimeLong)
        {
            myTimeDiffLongFlag = 0;
            myTimeDiffLong = myLocalTimeLong - myCenterTimeLong;
        }
        else
        {
            myTimeDiffLongFlag = 1;
            myTimeDiffLong = myCenterTimeLong - myLocalTimeLong;
        }
        myTimeDiffArr[rcNodeR10Cnt] = myTimeDiffLong;
        myTimeDiffFlagArr[rcNodeR10Cnt] = myTimeDiffLongFlag;

        memset(syncResp, 0, SYNC_PACKET_MAX_SIZE);

        if (rcNodeR10Cnt++ < (SYNC_TRANS_IN_ONE_COMM - 1))
        {
            nodeResponse = SYNC_PACKET_RESEND;
        }
        else
        {
            if (onSingleTouch && !shimmerStatus.isSensing && sd_tolog)
            {
                taskSetCb(TASK_STARTSENSING);
            }
            syncNodeSucc = 1;
            if (!firstOutlier)
            {
                rcFirstOffsetRxed = 1;
                RcFindSmallest();
                myTimeDiff[0] = myTimeDiffLongFlagMin;
                memcpy(myTimeDiff + 1, (uint8_t*) &myTimeDiffLongMin, 8);
            }
            resetMyTimeDiffLongMin();
            resetSyncRcNodeR10Cnt();
            nodeResponse = SYNC_FINISHED;

        }
    }
#if !USE_OLD_SD_SYNC_APPROACH
    else
    {
        nodeResponse = SYNC_PACKET_RESEND;
    }
#endif

//#if BT_DMA_USED_FOR_RX
//    if(nodeResponse == SYNC_PACKET_RESEND)
//    {
//        /* If status strings are enabled the DMA will already be set to listen
//         * for bytes so we only need to set if here if that is not the case */
//        if(!areBtStatusStringsEnabled())
//        {
////            setDmaWaitingForResponse(10U);
//            setDmaWaitingForResponse(1U);
//        }
//    }
//#endif

#if !USE_OLD_SD_SYNC_APPROACH
    memset(syncResp, 0, sizeof(syncResp));
#endif
    SyncNodeT1(nodeResponse);
}

/* Sync Node only. For responding to Center node after having received a single
 * packet. 1 is passed in if ready to receive the next 10 byte sync packet.
 * ACK_COMMAND_PROCESSED is passed in if 50 packets have been received */
void SyncNodeT1(uint8_t val)
{
#if USE_OLD_SD_SYNC_APPROACH
    uint8_t tosend = val;
    BT_write(&tosend, 1U, SHIMMER_CMD);
#else
    uint8_t syncResponse[] = {ACK_COMMAND_PROCESSED, SD_SYNC_RESPONSE, val};
    BT_write(&syncResponse[0], 3U, SHIMMER_CMD);
#endif
    if (syncNodeWinExpire < (syncCnt + SYNC_EXTEND * SYNC_FACTOR))
    {
        syncNodeWinExpire++;
    }
}

/* Sync Node only. Called by a node after it has received all sync packets. */
uint8_t RcFindSmallest(void)
{
    uint8_t i, j, k, black_list[20], black_list_cnt = 0, far_cnt = 0, black = 0;
    uint64_t to_compare_val, diff_val;
    for (i = 0; i < SYNC_TRANS_IN_ONE_COMM; i++)
    {
        for (j = 1; j < 6; j++)
        {
            to_compare_val = myTimeDiffArr[(i + j) % SYNC_TRANS_IN_ONE_COMM];
            if (myTimeDiffArr[i] > to_compare_val)
                diff_val = myTimeDiffArr[i] - to_compare_val;
            else
                diff_val = to_compare_val - myTimeDiffArr[i];
            if (diff_val > 3277) //0.1*32768
                far_cnt++;
        }
        if ((far_cnt >= 4) && (black_list_cnt < 20))
        {
            black_list[black_list_cnt++] = i;
        }
        far_cnt = 0;
    }

    resetMyTimeDiffLongMin();
    for (i = 0; i < SYNC_TRANS_IN_ONE_COMM; i++)
    {
        for (k = 0; k < black_list_cnt; k++)
        {
            if (i == black_list[k])
            {
                black = 1;
                break;
            }
        }
        if (black)
        {
            black = 0;
            continue;
        }
        if ((!myTimeDiffLongMin) && (!myTimeDiffLongFlagMin))
        {
            myTimeDiffLongFlagMin = myTimeDiffFlagArr[i];
            myTimeDiffLongMin = myTimeDiffArr[i];
        }
        else
        {
            if ((!myTimeDiffFlagArr[i]) && (!myTimeDiffLongFlagMin))
            { // was pos, curr pos
                if (myTimeDiffArr[i] < myTimeDiffLongMin)
                {
                    myTimeDiffLongMin = myTimeDiffArr[i];
                }
            }
            else if ((!myTimeDiffFlagArr[i]) && myTimeDiffLongFlagMin)
            { // was neg, curr pos

            }
            else if (myTimeDiffFlagArr[i] && (!myTimeDiffLongFlagMin))
            { // was pos, curr neg
                myTimeDiffLongFlagMin = myTimeDiffFlagArr[i];
                myTimeDiffLongMin = myTimeDiffArr[i];
            }
            else if (myTimeDiffFlagArr[i] && myTimeDiffLongFlagMin)
            { // was neg, curr neg
                if (myTimeDiffArr[i] > myTimeDiffLongMin)
                {
                    myTimeDiffLongMin = myTimeDiffArr[i];
                }
            }
        }
    }

    resetMyTimeDiffArrays();
    return 0;
}

void handleSyncTimerTrigger(void)
{
    if (isBtSdSyncRunning())
    {
        if (syncCnt >= estLen3 * SYNC_FACTOR)
        { //there must be: estLen3>SYNC_WINDOW_C
            syncCnt = 0;
            syncSuccN = 0; //reset node success flag
            if (syncThis > 3)
            {
                // can stop syncing after certain #
            }
            else
                syncThis++;
        }
        else
        {
            syncCnt++;
        }

        if (sdHeadText[SDH_TRIAL_CONFIG0] & SDH_IAMMASTER)
        { //  i am Center
            handleSyncTimerTriggerCenter();
        }
        else
        {                              // i am Node
            handleSyncTimerTriggerNode();
        }
    }
    else                                   //idle: no_RC mode
    {
        if (shimmerStatus.isDocked)
        {
            if (shimmerStatus.isSensing)
            {
                /* Note SDLog calls TASK_STOPSENSING here whereas
                 * LogAndStream could be in the middle of streaming over
                 * Bluetooth */
                stopLogging = 1;
            }
            if (shimmerStatus.isBtPoweredOn)
            {
                btStopCb(0);
            }
        }
    }
}

void handleSyncTimerTriggerCenter(void)
{
    if (shimmerStatus.isSensing && (syncNodeNum > 0))
    {
        if (syncCnt == 1)
        {
            // start
            resetSyncVariablesCenter();
            btStartCb();
        }
        else if ((syncCnt > SYNC_BOOT * SYNC_FACTOR)
                && (syncCnt < SYNC_WINDOW_C * SYNC_FACTOR))
        {
            // try to connect to each node
            // loop 1:n, x times
            if (nodeSucc != nodeSuccFull)
            {
                if (syncNodeCnt < syncNodeNum)
                {
                    syncCurrNode++;
                    if (!cReboot)
                    {
                        if (syncCurrNode == 1)
                        {
                            while (nodeSucc
                                    & SyncNodeShift(syncNodeCnt))
                            {
                                if (++syncNodeCnt >= syncNodeNum)
                                    syncNodeCnt = 0;
                            }
                            BT_connect(nodeName[syncNodeCnt]);
                            currNodeSucc = 0;
                            syncCurrNodeDone = 0;
                            syncCurrNodeExpire = SYNC_T_EACHNODE_C
                                    * SYNC_FACTOR;
                        }
                        else if ((syncCurrNode == syncCurrNodeExpire)
                                || currNodeSucc)
                        {
                            if (currNodeSucc)
                            {
                                BT_disconnect();
                                currNodeSucc = 0;
                            }
                            btStopCb(0);
                            cReboot = 1;
                            if (shortExpFlag)
                                nodeSucc |= 1 << syncNodeCnt;

                            syncNodeCnt++;
                        }
                        else if (syncCurrNodeDone
                                && (syncCurrNode
                                        == syncCurrNodeDone
                                                + SYNC_CD
                                                        * SYNC_FACTOR))
                        {
                            syncCurrNode = 0;
                        }
                    }
                    else
                    {   // cReboot>0
                        if (cReboot == 1)
                        {
                            cReboot = 2;
                            btStartCb();
                        }
                        else if ((cReboot >= 2)
                                && (cReboot < 5 * SYNC_FACTOR))
                        {
                            if (shimmerStatus.isBtPoweredOn)
                            {
                                syncCurrNodeDone = syncCurrNode
                                        + SYNC_CD * SYNC_FACTOR - 1;
                                cReboot = 0;
                            }
                            else
                                cReboot++;
                        }
                        else
                        {
                            btStopCb(0);
                            cReboot = 1;
                        }
                    }
                }
                else
                {
                    syncNodeCnt = 0;
                }
            }
            else
            {
                btStopCb(0);
                syncSuccC = 1;
                if (shortExpFlag)
                    syncCnt = estLen3 * SYNC_FACTOR;
            }
        }
        else if (syncCnt == SYNC_WINDOW_C * SYNC_FACTOR)
        {
            // power off
            btStopCb(0);
            if (nodeSucc == nodeSuccFull)
                syncSuccC = 1;
            else
                syncSuccC = 0;
            if (shortExpFlag)
                syncCnt = estLen3 * SYNC_FACTOR;
        }
    }
}

void handleSyncTimerTriggerNode(void)
{
    if (!syncSuccN)
    {
        if (syncCnt == 1)
        {
            resetMyTimeDiffLongMin();
            resetSyncVariablesNode();
        }
        else if (((syncCnt < syncNodeWinExpire)
                && (syncCnt > SYNC_BOOT))
                && (syncCnt
                        != (SYNC_CD * (nReboot + 1)
                                + SYNC_WINDOW_N * nReboot)
                                * SYNC_FACTOR))
        {
            if (syncNodeSucc)
            {
                btStopCb(0);
                syncNodeSucc = 0;
                nReboot = 0;
                if (firstOutlier)
                {
                    syncCnt =
                            (SYNC_CD * nReboot
                                    + SYNC_WINDOW_N * nReboot)
                                    * SYNC_FACTOR;
                    firstOutlier = 0;
                }
                else
                {
                    syncSuccN = 1;
                    if (shortExpFlag)
                        syncCnt = estLen3 * SYNC_FACTOR;
                    else
                        syncCnt = (SYNC_CD * rcNodeReboot
                                + SYNC_WINDOW_N
                                        * (SYNC_NEXT2MATCH - 1))
                                * SYNC_FACTOR;
                }
            }
        }
        else if (syncCnt == syncNodeWinExpire)
        {
            btStopCb(0);
            if (firstOutlier)
            {
                nReboot = 1;
                syncCnt = (SYNC_CD * nReboot
                        + SYNC_WINDOW_N * nReboot) * SYNC_FACTOR;
            }
            else
            {
                if (nReboot < rcNodeReboot)
                    nReboot++;
                else
                    nReboot = 0;
            }
            syncSuccN = 0;
        }
        else if (syncCnt
                == (SYNC_CD * (nReboot + 1)
                        + SYNC_WINDOW_N * nReboot) * SYNC_FACTOR)
        {
            btStartCb();
            syncNodeWinExpire = (SYNC_CD * (nReboot + 1)
                    + SYNC_WINDOW_N * (nReboot + 1)) * SYNC_FACTOR;
        }
    }
}

//Timer2:
//ccr1: for blink timer
void CommTimerStart(void)
{
    TA0CTL = TASSEL_1 + MC_2 + TACLR;    //ACLK, continuous mode, clear TAR
    TA0CCTL1 = CCIE;
    TA0CCR1 = GetTA0() + 16384;
}

inline void CommTimerStop(void)
{
    TA0CTL = MC_0; // StopTb0()
    //rcommStatus=0;
    TA0CCTL1 &= ~CCIE;
}

inline uint16_t GetTA0(void)
{
    register uint16_t t0, t1;
    uint8_t ie;
    if (ie = (__get_SR_register() & GIE))   //interrupts enabled? // @suppress("Assignment in condition")
        __disable_interrupt();
    t1 = TA0R;
    do {t0=t1; t1=TA0R;} while(t0!=t1);
    if (ie)
        __enable_interrupt();
    return t1;
}

#pragma vector=TIMER0_A1_VECTOR
__interrupt void TIMER0_A1_ISR(void)
{
    switch (__even_in_range(TA0IV, 14))
    {
    case 0:
        break;                           // No interrupt
    case 2:                                  // TA0CCR1
        TA0CCR1 += SYNC_PERIOD;

        /* SDLog handles auto-stop in TIMER0_A1_VECTOR whereas LogAndStream handles it in TIMER0_B1_VECTOR */
//        if (sensing && maxLen)
//        {
//            if (maxLenCnt < maxLen * SYNC_FACTOR)
//                maxLenCnt++;
//            else
//            {
//                stopLogging = 1;
//                //stopSensing = 1;
//                maxLenCnt = 0;
//                return;
//            }
//        }

        handleSyncTimerTrigger();

        break;
    case 4:
        break;                           // TA0CCR2 not used
    case 6:
        break;                           // Reserved
    case 8:
        break;                           // Reserved
    case 10:
        break;                           // Reserved
    case 12:
        break;                           // Reserved
    case 14:
        break;                           // TAIFG overflow handler
    }
}
