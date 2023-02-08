/*
 * sd_sync.h
 *
 *  Created on: 11 Oct 2022
 *      Author: Mark Nolan
 */

#ifndef SHIMMER3_COMMON_SOURCE_BLUETOOTH_SD_SD_SYNC_H_
#define SHIMMER3_COMMON_SOURCE_BLUETOOTH_SD_SD_SYNC_H_

#include <stdint.h>
#include "msp430.h"
#include "../5xx_HAL/hal_CRC.h"
#include "../../shimmer_btsd.h"

#define USE_OLD_SD_SYNC_APPROACH        0
#define BT_SD_SYNC_CRC_MODE             CRC_1BYTES_ENABLED

// BT routine communication
// all node time must *2 in use
// all center time must *4 in use
//#define RC_AHD          3
//#define RC_WINDOW_N     13
//#define RC_WINDOW_C     27
//#define RC_INT_N        27
#define SYNC_INT_C        54//240
//#define RC_CLK_N        16384   //16384=2hz;//32768=1hz;8192=4hz
//#define RC_CLK_C        8192   //16384=2hz;//32768=1hz;8192=4hz
//#define RC_FACTOR_N     (32768/RC_CLK_N)   //16384=2hz;//32768=1hz;8192=4hz
//#define RC_FACTOR_C     (32768/RC_CLK_C)   //16384=2hz;//32768=1hz;8192=4hz

//Node to Center Sync Packet - ACK(1 byte):SD_SYNC_RESPONSE(1 byte):Flag(1 byte)
//Center to Node Sync Packet - SET_SD_SYNC_COMMAND(1 byte):Status(1 byte):Time(8 bytes, LSB order):CRC(0/1/2 bytes, LSB order)
#define SYNC_PACKET_MAX_SIZE     (SYNC_PACKET_SIZE_CMD+SYNC_PACKET_PAYLOAD_SIZE+CRC_2BYTES_ENABLED)
#define SYNC_PACKET_PAYLOAD_SIZE 9
#if USE_OLD_SD_SYNC_APPROACH
#define SYNC_PACKET_ACK_IDX      0
#else
#define SYNC_PACKET_CMD_IDX      0
#endif
#define SYNC_PACKET_SIZE_CMD     1
#define SYNC_PACKET_FLG_IDX      1
#define SYNC_PACKET_TIME_IDX     2
#define SYNC_PACKET_TIME_SIZE    8

// new sync
#define SYNC_PERIOD              32768
#define SYNC_FACTOR              1//32768/SYNC_PERIOD
#define SYNC_BOOT                3
#define SYNC_CD                  2
#define SYNC_EXTEND              4// should have SYNC_BOOT>SYNC_CD
//below were params in sdlog.cfg
#define SYNC_T_EACHNODE_C        12
#define SYNC_WINDOW_C            800
#define SYNC_WINDOW_N            50
#define SYNC_NODE_REBOOT         17
#define SYNC_TRANS_IN_ONE_COMM   50
#define SYNC_NEXT2MATCH          2

#define SYNC_PACKET_RESEND       0x01
#define SYNC_FINISHED            0xFF

void sdSyncInit(void (*btStart_cb)(void),
                void (*btStop_cb)(uint8_t),
                uint8_t (*taskSet_cb)(TASK_FLAGS));
void resetMyTimeDiff(void);
void resetMyTimeDiffArrays(void);
void resetMyTimeDiffLongMin(void);
uint8_t * getMyTimeDiffPtr(void);
#if USE_OLD_SD_SYNC_APPROACH
void setRcommVar(uint8_t val);
uint8_t getRcommVar(void);
#endif
void setSyncResp(uint8_t *args, uint8_t count);
uint8_t isBtSdSyncRunning(void);
uint8_t getSyncNodeNum(void);
uint8_t* getSyncNodeNamePtrForIndex(uint8_t index);
uint8_t* getSyncCenterNamePtr(void);
uint8_t getRcFirstOffsetRxed(void);
uint8_t getSyncSuccC(void);
uint8_t getSyncSuccN(void);
uint8_t getSyncCnt(void);
void saveLocalTime(void);
void resetSyncRcNodeR10Cnt(void);
void resetSyncVariablesBeforeParseConfig(void);
void resetSyncVariablesDuringSyncStart(void);
void resetSyncVariablesCenter(void);
void resetSyncVariablesNode(void);
void resetSyncNodeArray(void);
uint16_t parseSyncEstExpLen(uint8_t estExpLenLsb, uint8_t estExpLenMsb);
void setSyncEstExpLen(uint32_t est_exp_len);
void parseSyncNodeNamesFromConfig(uint8_t *storedConfigPtr);
void parseSyncCenterNameFromConfig(uint8_t *storedConfigPtr);
void parseSyncCenterNameFromCfgFile(uint8_t *storedConfigPtr, char *equals);
void parseSyncNodeNameFromCfgFile(uint8_t *storedConfigPtr, char *equals);
void checkSyncCenterName(void);

void BtSdSyncStop(void);
void BtSdSyncStart(void);

void SyncCenterT10(void);
void SyncCenterR1(void);
uint32_t SyncNodeShift(uint8_t shift_value);
void SyncNodeR10(void);
void SyncNodeT1(uint8_t val);
uint8_t RcFindSmallest(void);

void handleSyncTimerTrigger(void);
void handleSyncTimerTriggerCenter(void);
void handleSyncTimerTriggerNode(void);
void startBtForSync(void);

void CommTimerStart(void);
void CommTimerStop(void);
uint16_t GetTA0(void);

#endif /* SHIMMER3_COMMON_SOURCE_BLUETOOTH_SD_SD_SYNC_H_ */
