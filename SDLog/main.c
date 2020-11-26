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
 * @author Weibo Pan
 * @date June, 2015
 *
 * @modifed Mark Nolan
 * @date June, 2014
 *
 * @modifed Sam O'Mahony
 * @date January, 2018
 */

/***********************************************************************************
 Data Buffer Format:
      Packet Type |TimeStamp|Achan1|Achan2| ... |AchanX|Dchan1|Dchan2| ... |DchanX|
 Byte:     0      |   1-3   |Achan1|Achan2| ... |AchanX|Dchan1|Dchan2| ... |DchanX|

 Log file Format:
      SD Header | TimeStamp1  |Achan1 data1| ... |DchanX data1|  TimeStamp2 |Achan2 data2|...
 Byte:  0-255   |  256-258    |   259-260  | ... |            |             |            |...

 ***********************************************************************************/

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "msp430.h"

#include "5xx_HAL/hal_RTC.h"
#include "5xx_HAL/hal_TB0.h"
#include "5xx_HAL/hal_CRC.h"
#include "5xx_HAL/hal_UCA0.h"
#include "5xx_HAL/hal_UartA0.h"
#include "5xx_HAL/hal_ADC.h"
#include "5xx_HAL/hal_Board.h"
#include "5xx_HAL/hal_Button.h"
#include "5xx_HAL/hal_DMA.h"
#include "5xx_HAL/hal_I2C.h"
#include "5xx_HAL/hal_InfoMem.h"
#include "5xx_HAL/hal_pmm.h"
#include "5xx_HAL/hal_SDCard.h"
#include "5xx_HAL/hal_UCS.h"
#include "Bluetooth_SD/RN42.h"
#include "BMPX80/bmpX80.h"
#include "CAT24C16/cat24c16.h"
#include "EXG/exg.h"
#include "EXG/ads1292.h"
#include "FatFs/ff.h"
#include "GSR/gsr.h"
#include "LSM303DLHC/lsm303dlhc.h"
#include "LSM303AHTR/lsm303ahtr.h"
#include "MPU9150/mpu9150.h"
#include "msp430_clock/msp430_clock.h"
#include "shimmer_calibration/shimmer_calibration.h"
#include "MPU9150/mllite.h"
#include "shimmer_sd.h"

typedef uint8_t bool;
#define TRUE    (1)
#define FALSE   (0)

typedef uint8_t error_t;
#define SUCCESS (0)
#define FAIL    (1)

void Init(void);
void CommTimerStart(void);
void BlinkTimerStart(void);
void SampleTimerStart(void);
inline void SampleTimerStop(void);
inline uint8_t PreStartStreaming(void);
void StartStreaming(void);
inline void StopStreaming(void);
inline uint16_t GetTA0(void);
uint8_t Dma0ConversionDone(void);
uint8_t Dma2ConversionDone(void);
void ConfigureChannels(void);
void PrepareSDBuffHead(void);
error_t SetBasedir();
error_t MakeBasedir();
void Timestamp0ToFirstFile();
inline void GsrRange(void);
void ItoaWith0(uint64_t num, uint8_t* buf, uint8_t len);
void ItoaNo0(uint64_t num, uint8_t* buf, uint8_t max_len);
uint64_t Atol64(uint8_t* buf);
void DockSdPowerCycle();
void SdPowerOff(void);
void SdPowerOn(void);
uint8_t CheckSdInslot();
void RcCenterT10();
void RcNodeR10();
void RcNodeT1(uint8_t val);
void RcCenterR1(void);
uint8_t RcFindSmallest();
void BtStop();
void BtStart();
void BtStartDone();
void StreamData();
void RwcCheck();
void Write2SD();
void TB0Start();
void TB0Stop();
uint8_t GetSdCfgFlag();
void SetSdCfgFlag(uint8_t flag);
void ShimmerCalibSyncFromDumpRamAll(void);
void ShimmerCalibSyncFromDumpRamSingleSensor(uint8_t sensor);
void ShimmerCalibInitFromInfoAll(void);
void ShimmerCalibUpdateFromInfoAll(void);
void ShimmerCalibFromInfo(uint8_t sensor, uint8_t use_sys_time);
void UpdateSdConfig();
uint8_t ParseConfig();
void SetDefaultConfiguration(void);
void ChargeStatusTimerStart(void);
void ChargeStatusTimerStop(void);
void Config2SdHead(void);
void BattBlinkOn();
void SetBattDma();
void ClkAssignment();
uint16_t FreqProd(uint16_t num_in);
uint16_t FreqDiv(float num_in);
void SetBtBaudRate(uint8_t rate);
void ChangeBtBaudRate();
uint32_t SyncNodeShift(uint8_t shift_value);
void ReadBatt();
uint8_t UartCheckCrc(uint8_t len);
void UartProcessCmd();
void UartSendRsp();
uint8_t UartCallback(uint8_t data);
void UartCbufPush(uint8_t elem);
void UartCbufPickData(uint8_t * dst_buf, uint8_t offset, uint8_t len);
uint8_t MPL_calibrateCB(void);
void MPL_calibrateGo(void);
uint8_t TaskSet(TASK_FLAGS task_id);
void TaskClear(TASK_FLAGS task_id);
uint8_t TaskGet(TASK_FLAGS task_id);
uint16_t TaskCurrentGet();
void SetupDock();
void RcStop();
void RcStart();
void SetBattVal();
inline uint8_t Skip100ms();
void SamplingClkAssignment(void);

void i2cSlaveDiscover(void);
char i2cSlavePresent(char address);

uint8_t mplCalibrating, mplCalibratingMag, mpuIntTriggered, triggerSampling,
        mplCalibrateInit;
uint8_t currentBuffer, streamData, sensing, btIsConnected, streamDataInProc,
        vbattEn, i2cEn, boot, nbrAdcChans, nbrDigiChans, rwcErrorFlash,
        rwcErrorEn, rcFirstOffsetRxed, skip100ms, firstTsFlag;
uint8_t sdBuff[SDBUFF_SIZE], newDirFlag, dirLen, fileNeedNew, fileNextNum;
uint8_t myTimeDiff[9];
uint8_t dirName[64], expDirName[32], sdHeadText[SDHEAD_LEN], fileBad,
        fileBadCnt, btBad, fileName[64], expIdName[MAX_CHARS],
        shimmerName[MAX_CHARS], bmpX80Calib[BMP280_CALIB_DATA_SIZE], bmpInUse,
        lsmInUse, eepromIsPresent, configTimeText[MAX_CHARS],
        centerName[MAX_CHARS], mac[14], macAddr[6], macAddrSet, fwInfo[7],
        ackStr[4], storedConfig[NV_TOTAL_NUM_CONFIG_BYTES], configuring,
        txBuff0[DATA_PACKET_SIZE], txBuff1[DATA_PACKET_SIZE], btRxBuff[14],
        *btRxExp, daughtCardId[4], battStat, battRead, battWait, battVal[3],
        myTimeDiffLongFlag, myTimeDiffLongFlagMin,
        myTimeDiffFlagArr[SYNC_TRANS_IN_ONE_COMM];
uint8_t getRcomm, rcommTimeout, rcommResp[RCT_SIZE], rcommStatus,
        rcommWindowCenter, //rcommCurrentTry,
        btPowerOn, sampleTimerStatus, blinkStatus, rcommInterval, docked,
        setUndock, setUndockDone, setUndockStart, onUserButton, onSingleTouch,
        initializing, onDefault, preSampleBmpPress, bmpPressFreq, bmpPressCount,
        sampleBmpTemp, sampleBmpTempFreq, bmpVal[BMPX80_PACKET_SIZE],
        preSampleMpuMag, mpuMagFreq, mpuMagCount, mpu9150Initialised,
        gsrActiveResistor, exgLength, exgChip, exgStartAddr;
uint8_t realTimeClockText64[UINT64_LEN], realTimeDiffText64[UINT64_LEN],
        realTimeClockText40[14];
uint8_t nodeName[MAX_NODES][MAX_CHARS], shortExpFlag, uartInfoMemLength;
uint8_t syncNodeCnt, syncNodeNum, syncRetryCnt, syncThis, syncNodeSucc, nReboot,
        currNodeSucc, cReboot;
uint8_t syncSuccC, syncSuccN, syncCurrNode, syncCurrNodeDone, rcNodeR10Cnt; //, syncRstRcVars;
uint8_t uartRxBuf[UART_DATA_LEN_MAX], uartRespBuf[RESPONSE_PACKET_SIZE];
uint8_t uartSteps, uartArgSize, uartArg2Wait, uartCrc2Wait, uartOldAction,
        uartAction, uartSendRspOldMac, uartSendRspOldVer, uartSendRspOldBat,
        uartSendRspOldMem, uartSendRspOldRtc, uartSendRspOldRct,
        uartSendRspOldRdt, uartSendRspOldTim, uartSendRspSampleRate,
        uartSendRspMac, uartSendRspVer, uartSendRspBat, uartSendRspGdi,
        uartSendRspGdm, uartSendRspGim, uartSendRspRtcConfigTime,
        uartSendRspCurrentTime, uartSendRspLsm303dlhcAccelRange,
        uartSendRspLsm303dlhcAccelSamplingRate, uartSendRspGsrSamplingRate,
        uartSendRspLsm303dlhcAccelLPMode, uartSendRspLsm303dlhcAccelHRMode,
        uartSendRspLsm303dlhcAccelEn, uartSendRspGsrRange, uartSendRspGsrEn,
        uartSendRspBattSampleRate, uartSendRspLsm303dlhcAccelDataRate,
        uartSendRspBattEn, uartSendRspLsm303dlhcAccelDiv, uartSendRspGsrDiv,
        uartSendRspBattDiv, uartSendRspLsm303dlhcAccelCalib, uartSendRspOldAck,
        uartSendRspAck, uartSendRspBadCmd, uartSendRspBadArg, uartSendRspBadCrc;
uint8_t bmpTempCurrentVal[2], bmpPresCurrentVal[3];
uint8_t all0xff[7];

// variables used in i2c address scanning (used for BMP180/280 check)
uint8_t slave_addresses[128];
uint8_t * slave_address_pointer;

struct
{
    uint8_t idx;
    uint8_t entry[CBUF_SIZE];
} ucBuf;

uint16_t bmpTempInterval, bmpPresInterval;
uint16_t taskList, taskCurrent;
uint16_t dirCounter, blinkCnt20, blinkCnt50, sdBuffLen, blockLen, fileNum,
        rcommCnt, rcommIntervalCenter, rcommSpecialInt, uartDcMemLength,
        uartDcMemOffset, uartInfoMemOffset, lastGsrVal, syncCurrNodeExpire,
        syncNodeWinExpire;
uint16_t *adcStartPtr;

uint16_t clk_45, clk_55, clk_75, clk_85, clk_90, clk_105, clk_115, clk_120,
        clk_135, clk_145, clk_165, clk_195, clk_225, clk_255, clk_285, clk_375,
        clk_405, clk_1000, clk_2500, clk_90_45, clk_90_55, clk_90_75, clk_90_85,
        clk_135_90, clk_195_90, clk_255_90, clk_375_90;

uint64_t buttonPressTs64, buttonReleaseTs64, buttonLastReleaseTs64,
        buttonP2RTs64, battLastTs64;
uint64_t buttonTwoPressTd64;
uint32_t battInterval, firstOutlier, rcWindowC, rcNodeReboot;
uint32_t estLen, estLen3, maxLen, maxLenCnt, syncCnt;
uint32_t nodeSucc, nodeSuccFull;

uint64_t bmpTempStartTs, bmpPresStartTs;
uint64_t bmpTempSampleTs, bmpPresSampleTs;
uint64_t rwcTimeDiff64;
uint64_t fileLastHour, fileLastMin, rwcConfigTime64;
uint64_t myLocalTimeLong, myCenterTimeLong, myTimeDiffLong, myTimeDiffLongMin;
uint64_t myTimeDiffArr[SYNC_TRANS_IN_ONE_COMM];
uint64_t startSensingTs64, firstTs;

// variables used for delayed undock-start
bool battCritical;
uint8_t undockEvent, wr2sd, battCriticalCount;
uint64_t time_newUnDockEvent;

/* Variable for SR47-4 (and later) to indicate ADS clock lines are tied */
uint8_t adsClockTied;

// approx. 10% cutoff voltage - 3.65 Volts
#define BATT_CUTOFF_3_65VOLTS   (2500)
#define TIMEOUT_100_MS          (3277)

// make dir for SDLog files
FATFS fatfs;         // File object
DIRS dir;            //Directory object
FIL dataFil;
float clockFreq;

#define PRESS2UNDOCK  0
#define UNDOCKTEST    0
#define RTC_OFF       0
#define PRES_TS_EN    0

#define TS_BYTE3      1
#define SKIP100MS     1

void main(void)
{
    initializing = 1;

    // first/second MPL library callback fails if system_pre_init.c is used
    WDTCTL = WDTPW | WDTHOLD;
    Init();
    taskList = 0;

    if (!configuring && !wr2sd)
    {
        SetupDock();
    }
    ReadBatt();
    msp430_delay_ms(10);

    // Initialise Watchdog status timer
    ChargeStatusTimerStart();

    while (1)
    {
        taskCurrent = TaskCurrentGet();
        if (!taskCurrent)
        {
            __bis_SR_register(LPM3_bits + GIE);   //ACLK remains active
            taskCurrent = TaskCurrentGet();
        }
        TaskClear((TASK_FLAGS) taskCurrent);

        if (taskCurrent == TASK_MPL_CALIBRATE)
        {
            MPL_calibrateGo();
        }
        if (taskCurrent == TASK_SETUP_DOCK)
        {
            // wait 3 seconds first...
            if (!configuring && !wr2sd
                    && ((P2IN & BIT3)
                            || ((RTC_get64() - time_newUnDockEvent)
                                    > TIMEOUT_100_MS)))
            {
                if (docked != ((P2IN & BIT3) >> 3))
                {
                    docked = ((P2IN & BIT3) >> 3);
                    SetupDock();
                }

                if (docked != ((P2IN & BIT3) >> 3))
                {
                    TaskSet(TASK_SETUP_DOCK);
                }
                undockEvent = 0;
            }
            else
            {
                TaskSet(TASK_SETUP_DOCK);
            }
        }
        if (taskCurrent == TASK_UARTCMD)
        {
            UartProcessCmd();
        }
        if (taskCurrent == TASK_UARTRSP)
        {
            UartSendRsp();
        }
        if (taskCurrent == TASK_BATT_READ)
        { // use adc channel2 and mem4, read back battery status every certain period
            if (!sensing)
                ReadBatt();
        }
        if (taskCurrent == TASK_CFGCH)
        {
            ConfigureChannels();
        }
        if (taskCurrent == TASK_RCCENTERR1)
        {
            RcCenterR1();
        }
        if (taskCurrent == TASK_RCNODER10)
        {
            RcNodeR10();
        }
        if (taskCurrent == TASK_STOPSENSING)
        {
            if (sensing)
                StopStreaming();
        }
        if (taskCurrent == TASK_STARTSENSING)
        {
            configuring = 1;
            if ((!battCritical) && (fileBad == FR_OK))
            {
                PreStartStreaming();
                StartStreaming();
                if (sdHeadText[SDH_TRIAL_CONFIG0] & SDH_TIME_SYNC)
                {
                    RcStart();
                }
            }
            configuring = 0;
        }
        if (taskCurrent == TASK_SAMPLEMPU9150MAG)
        {
            if (!(storedConfig[NV_CONFIG_SETUP_BYTE4] & MPU9150_MPL_DMP))
                MPU9150_startMagMeasurement();
        }
        if (taskCurrent == TASK_SAMPLEBMPX80PRESS)
        {
            if (sampleBmpTemp == sampleBmpTempFreq)
            {
                bmpTempStartTs = RTC_get64();
                BMPX80_startTempMeasurement();
            }
            else
            {
                bmpPresStartTs = RTC_get64();
                BMPX80_startPressMeasurement(
                        (storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x30) >> 4);
            }
        }
        if (streamData)
        {
            if (streamDataInProc)
            {
                streamData = 0;
                StreamData();
                if (sdBuffLen > SDBUFF_SIZE - blockLen)
                    TaskSet(TASK_WR2SD);
            }
            streamDataInProc = 0;
            streamData = 0;
        }
        if (taskCurrent == TASK_WR2SD)
        {
            Write2SD();
        }
    }
}

void Init(void)
{
    // Needs to be here before LED functions are called:
    battCritical = FALSE;

    Board_init();
    Board_ledOn(LED_ALL);

    // Set Vcore to accommodate for max. allowed system speed
    SetVCore(3);

    // Start 32.768kHz XTAL as ACLK
    LFXT_Start(XT1DRIVE_0);

    // Start 24MHz XTAL as MCLK and SMCLK
    XT2_Start(XT2DRIVE_2); // XT2DRIVE_2 or XTDRIVE_3 for 24MHz (userguide section 5.4.7)
    UCSCTL4 |= SELS_5 + SELM_5;   // SMCLK=MCLK=XT2

    SFRIFG1 = 0;                  // clear interrupt flag register
    SFRIE1 |= OFIE;               // enable oscillator fault interrupt enable

    msp430_clock_init();
    memset(myTimeDiff, 0xff, 9);
    rwcTimeDiff64 = 0;
    rwcConfigTime64 = 0;
    memset(realTimeClockText64, '0', UINT64_LEN - 1);
    memset(realTimeDiffText64, '0', UINT64_LEN - 1);
    realTimeClockText64[UINT64_LEN - 1] = 0;
    realTimeDiffText64[UINT64_LEN - 1] = 0;
    realTimeClockText40[13] = 0;
    // flag variables initialisation

    firstTsFlag = 0;
    rcFirstOffsetRxed = 0;
    skip100ms = 0;
    boot = 1;
    rwcErrorEn = 0;
    rwcErrorFlash = 0;
    i2cEn = 0;
    firstOutlier = 1;
    streamDataInProc = 0;
    buttonLastReleaseTs64 = 0;
    macAddrSet = 0;
    syncThis = 0;
    syncCnt = 0;
    nReboot = 0;
    cReboot = 0;
    syncNodeSucc = 0;
    myTimeDiffLongMin = 0;
    myTimeDiffLongFlagMin = 0;
    currNodeSucc = 0;
    syncNodeNum = 0;
    nodeSuccFull = 0;
    syncSuccC = 0;
    syncSuccN = 0;
    syncCurrNode = 0;
    syncCurrNodeDone = 0;
    maxLenCnt = 0;
    configuring = 0;
    battLastTs64 = 0;
    battRead = 1;
    battWait = 0;
    battStat = 0;
    blinkCnt20 = 0;
    blinkCnt50 = 0;
    fileBad = 0;
    btBad = 0;
    rcommSpecialInt = 0;
    rcommWindowCenter = RC_WINDOW_C;
    rcommStatus = 0;
    onDefault = 0;
    onUserButton = 0;
    onSingleTouch = 0;
    setUndockDone = 0;
    setUndockStart = 0;
    setUndock = 0;
    getRcomm = 0;
    currentBuffer = 0;
    sampleBmpTemp = 0;
    preSampleMpuMag = 0;
    preSampleBmpPress = 0;
    streamData = 0;
    sensing = 0;
    btIsConnected = 0;
    nbrAdcChans = 0;
    nbrDigiChans = 0;
    newDirFlag = 1;
    memset(sdBuff, 0, SDBUFF_SIZE);
    memset(sdHeadText, 0, SDHEAD_LEN);
    memset(sdHeadText + SDH_LSM303DLHC_ACCEL_CALIBRATION, 0xff, 84);
    sdBuffLen = 0;
    clockFreq = (float) MSP430_CLOCK;
    ClkAssignment();
    sampleTimerStatus = 0;
    blinkStatus = 0;
    rcommTimeout = 0;
    bmpInUse = BMP180_IN_USE;
    lsmInUse = LSM303DLHC_IN_USE;
    eepromIsPresent = FALSE;
    btPowerOn = 0;
    lastGsrVal = 0;
    uartSendRspOldAck = 0;
    uartSendRspOldMac = 0;
    uartSendRspOldVer = 0;
    uartSendRspOldBat = 0;
    uartSendRspOldMem = 0;
    uartSendRspOldRtc = 0;
    uartSendRspOldRct = 0;
    uartSendRspOldRdt = 0;
    uartSendRspOldTim = 0;
    uartSendRspAck = 0;
    uartSendRspSampleRate = 0;
    uartSendRspMac = 0;
    uartSendRspVer = 0;
    uartSendRspBattEn = 0;
    uartSendRspBat = 0;
    uartSendRspRtcConfigTime = 0;
    uartSendRspCurrentTime = 0;
    uartSendRspGdi = 0;
    uartSendRspGdm = 0;
    uartSendRspGim = 0;
    uartSendRspLsm303dlhcAccelDiv = 0;
    uartSendRspLsm303dlhcAccelCalib = 0;
    uartSendRspGsrDiv = 0;
    uartSendRspBattDiv = 0;
    uartSendRspLsm303dlhcAccelEn = 0;
    uartSendRspLsm303dlhcAccelRange = 0;
    uartSendRspLsm303dlhcAccelSamplingRate = 0;
    uartSendRspLsm303dlhcAccelDataRate = 0;
    uartSendRspBattSampleRate = 0;
    uartSendRspLsm303dlhcAccelLPMode = 0;
    uartSendRspLsm303dlhcAccelHRMode = 0;
    uartSendRspGsrRange = 0;
    uartSendRspGsrEn = 0;
    uartSendRspGsrSamplingRate = 0;
    uartSendRspBadCmd = 0;
    uartSendRspBadArg = 0;
    uartSendRspBadCrc = 0;
    uartSteps = 0;
    uartArgSize = 0;
    uartArg2Wait = 0;
    uartCrc2Wait = 0;
    ucBuf.idx = 0;
    rcNodeR10Cnt = 0;

    memset(mac, 0, 14);
    memset(macAddr, 0, 6);

    /* variables used for delayed undock-start */
    wr2sd = 0;
    undockEvent = 0;
    time_newUnDockEvent = 0;
    battCriticalCount = 0;

    /* Variable for SR47-4 (and later) to indicate ADS clock lines are tied */
    adsClockTied = 0;

    mplCalibrating = 0;
    mplCalibratingMag = 0;
    mplCalibrateInit = 0;
#ifdef SAMP_AT_MPU_INT
    triggerSampling = 1;
#endif
    // DMP related - end

    UCA0_isrInit();
    UART_init(UartCallback);

#if UNDOCKTEST
    if(0)
    {
#else
    if (P2IN & BIT3)
    {
#endif
        P2IES |= BIT3;   //look for falling edge
        battInterval = BATT_INTERVAL_D;
        docked = 1;
        UART_activate();
        setUndockStart = 0;
        setUndockDone = 0;
        DockSdPowerCycle();
    }
    else
    {
        P2IES &= ~BIT3;   //look for rising edge
        battInterval = BATT_INTERVAL;
        docked = 0;
        UART_deactivate();
    }
    P2IFG &= ~BIT3;      //clear flag
    P2IE |= BIT3;        //enable interrupt

    // Globally enable interrupts
    _enable_interrupts();

    msp430_delay_ms(1900);

    BlinkTimerStart();

    memset(btRxBuff, 0, 14);
    DMA2_init((uint16_t *) &UCA1RXBUF, (uint16_t *) btRxBuff, 14);
    DMA2_transferDoneFunction(&Dma2ConversionDone);
    btRxExp = BT_getExpResp();
    // =========== below initialize bt for the first time and get its MAC address only ==========
    InfoMem_read((uint8_t *) NV_MAC_ADDRESS, macAddr, 6);
    memset(all0xff, 0xff, 7);

    if (!memcmp(all0xff, macAddr, 6))
    {
        //if(0){
        uint8_t j = 0;
        do
        {
            BT_init();
            BT_setGetMacAddress(1);
            BtStart();
            msp430_delay_ms(2200);

            uint8_t i = 20;
            while ((!macAddrSet) && i)
            {
                i--;
                msp430_delay_ms(100);
            }
            BtStop();
            if (!i)
            {
                btBad = 1;
                msp430_delay_ms(8000);
            }
            else
            {
                btBad = 0;
            }
            if (j >= 3)
            {
                // try 3 times max, if still bad, software POR reset
                PMMCTL0 = PMMPW + PMMSWPOR + (PMMCTL0 & 0x0003);
            }
        }
        while (btBad && j++ > 3);
        uint8_t i, pchar[3];
        pchar[2] = 0;
        for (i = 0; i < 6; i++)
        {
            pchar[0] = mac[i * 2];
            pchar[1] = mac[i * 2 + 1];
            macAddr[i] = strtoul((char*) pchar, 0, 16);
        }
        InfoMem_write((uint8_t*) NV_MAC_ADDRESS, macAddr, 6);
    }
    else
    {
        uint8_t i, byte_h, byte_l;
        for (i = 0; i < 6; i++)
        {
            byte_h = (macAddr[i] >> 4) & 0x0f;
            byte_l = macAddr[i] & 0x0f;
            mac[i * 2] = byte_h + (byte_h > 9 ? 'A' - 10 : '0');
            mac[i * 2 + 1] = byte_l + (byte_l > 9 ? 'A' - 10 : '0');
        }
    }
    // =========== above initialize bt for the first time and get its MAC address only ==========

    //EXP_RESET_N
    P3OUT &= ~BIT3;      //set low
    P3DIR |= BIT3;       //set as output

    i2cSlaveDiscover();

    bmpInUse = (i2cSlavePresent(BMP280_ADDR)) ? BMP280_IN_USE : BMP180_IN_USE;
    lsmInUse = (i2cSlavePresent(LSM303AHTR_ACCEL_ADDR)) ?
    LSM303AHTR_IN_USE : LSM303DLHC_IN_USE;

    eepromIsPresent = (i2cSlavePresent(CAT24C16_ADDR)) ? TRUE : FALSE;

    ShimmerCalib_init();
    ShimmerCalibInitFromInfoAll();

    memset(bmpX80Calib, 0, BMP280_CALIB_DATA_SIZE);

    BMPX80_setup(bmpInUse);

    BMPX80_getCalibCoeff(bmpX80Calib);
    P8OUT &= ~BIT4;

    memcpy(&sdHeadText[SDH_TEMP_PRES_CALIBRATION], bmpX80Calib,
    BMP180_CALIB_DATA_SIZE);
    if (bmpInUse == BMP280_IN_USE)
    {
        memcpy(&sdHeadText[BMP280_XTRA_CALIB_BYTES],
               &(bmpX80Calib[BMP180_CALIB_DATA_SIZE]),
               BMP280_CALIB_XTRA_BYTES);
    }

    // enable switch1 interrupt
    Button_init();
    Button_interruptEnable();

    CheckSdInslot();

    RTC_init(0);

    CommTimerStart();

    memset(daughtCardId, 0xaa, 3);
    CAT24C16_init();
    CAT24C16_read(0, 0x0003, daughtCardId);
    CAT24C16_powerOff();
    _delay_cycles(2400000);

    /*
     * ADS1292R chip issue workaround
     * Requires GPIO_INTERNAL1 to be an input (default is output)
     * Issues affects boards SR37, SR47 & SR59 at present.
     */
    if ((daughtCardId[DAUGHT_CARD_ID] == 37)
            || (daughtCardId[DAUGHT_CARD_ID] == 47)
            || (daughtCardId[DAUGHT_CARD_ID] == 59))
    {
        P2DIR &= ~BIT0;                  //EXG_DRDY as input
    }

    if ((lsmInUse == LSM303AHTR_IN_USE) && (bmpInUse == BMP280_IN_USE)
            && !eepromIsPresent)
    {
        daughtCardId[DAUGHT_CARD_ID] = 31;
        daughtCardId[DAUGHT_CARD_REV] = 6;
        daughtCardId[DAUGHT_CARD_SPECIAL_REV] = 0;
    }

    uint8_t srId = daughtCardId[DAUGHT_CARD_ID];
    if (eepromIsPresent
            && ((lsmInUse == LSM303AHTR_IN_USE) && (bmpInUse == BMP280_IN_USE))
            && !((srId == 59) || (srId == 31) || (srId == 47) || (srId == 48)
                    || (srId == 49)))
    {
        daughtCardId[DAUGHT_CARD_SPECIAL_REV] = 171;
    }

    initializing = 0;
    Board_ledOff(LED_ALL);
}

void SetupDock()
{
    configuring = 1;

    if (docked)
    {
        battCriticalCount = 0;
        onUserButton = 0;
        onSingleTouch = 0;
        onDefault = 0;
//        DockSdPowerCycle();
        if (sensing)
        {
            TaskSet(TASK_STOPSENSING);
        }
        else
        {
            UART_activate();
        }
        uartSteps = 0;
        battInterval = BATT_INTERVAL_D;
        RcStop();

        if (boot)
        {
            InfoMem_read((uint8_t *) 0, storedConfig, 6);
            if (memcmp(all0xff, storedConfig, 6))
            { /* not all 0xff: */
                InfoMem_read((uint8_t *) 0, storedConfig,
                NV_TOTAL_NUM_CONFIG_BYTES);
            }
            else
            { /* all 0xff: */
                SetDefaultConfiguration();
            }
        }
        else
        {
            DockSdPowerCycle();
        }
    }
    else
    {
        battInterval = BATT_INTERVAL;
        if (!sensing)
        {
            UART_deactivate();
        }
        P6OUT |= BIT0;             /* DETECT_N set to high */
        P4OUT &= ~BIT2;            /* SD power off */
        __delay_cycles(2880000);   /* wait 120ms */
        P4OUT |= BIT2;             /* SD power on */

        CheckSdInslot();

        if (GetSdCfgFlag())
        {
            InfoMem_read((uint8_t *) 0, storedConfig,
            NV_TOTAL_NUM_CONFIG_BYTES);
            SetSdCfgFlag(0);
            UpdateSdConfig();
        }
        else
        {
            if (!ParseConfig())
            {
            }
            else if (fileBad == FR_NO_FILE)
            {
                fileBad = 0;
                InfoMem_read((uint8_t *) 0, storedConfig, 6);
                if (memcmp(all0xff, storedConfig, 6))
                { /* not all 0xff: */
                    InfoMem_read((uint8_t *) 0, storedConfig,
                    NV_TOTAL_NUM_CONFIG_BYTES);
                }
                else
                { /* all 0xff: */
                    SetDefaultConfiguration();
                }
                UpdateSdConfig();
            }
            else
            {
            }
        }
        Config2SdHead();
        TaskSet(TASK_CFGCH);

        //CalibAll();
        if (ShimmerCalib_file2Ram())
        {
            /* fail, i.e. no such file. use current DumpRam to generate a file */
            ShimmerCalib_ram2File();
        }
        ShimmerCalibSyncFromDumpRamAll();

        if (sdHeadText[SDH_TRIAL_CONFIG1] & SDH_SINGLETOUCH) /* set trigger mode */
            onSingleTouch = 1;
        else if (sdHeadText[SDH_TRIAL_CONFIG0] & SDH_USER_BUTTON_ENABLE)
            onUserButton = 1;
        else
        {
            onDefault = 1;
            if (!fileBad)
            {
                TaskSet(TASK_STARTSENSING);
            }
        }
    }
    boot = 0;
    RwcCheck();

    configuring = 0;
}

inline uint8_t PreStartStreaming(void)
{
    streamDataInProc = 0;
    streamData = 0;
    P4OUT |= BIT2; //sd power
    fileBad = SetBasedir();
    fileBad = MakeBasedir();
    return 0;
}

void StartStreaming(void)
{

    if (!sensing)
    {
#if SKIP100MS
        skip100ms = 1;
#endif
        SamplingClkAssignment();

        ClkAssignment();
        TB0CTL = MC_0; // StopTb0()
        TB0Start();

        if (storedConfig[NV_SENSORS0] & SENSOR_A_ACCEL)
        {
            P8REN &= ~BIT6;      //disable pull down resistor
            P8SEL &= ~BIT6;      //disable pull down resistor
            P8DIR |= BIT6;       //set as output
            P8OUT |= BIT6;   //analog accel being used so take out of sleep mode
        }

        if (storedConfig[NV_CONFIG_SETUP_BYTE3] & EXP_POWER_ENABLE)
        { /*power of exp board*/
            P3SEL &= ~BIT3;
            P3DIR |= BIT3;
            P3OUT |= BIT3;
        }

        if (storedConfig[NV_SENSORS1] & SENSOR_STRAIN)
        {
            P2OUT |= BIT0;                            //GPIO_INTERNAL1 set high
        }

        if (storedConfig[NV_SENSORS0] & SENSOR_GSR)
        {
            GSR_init();
            if (((storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x0E) >> 1)
                    <= HW_RES_3M3)
            {
                GSR_setRange((storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x0E) >> 1);
                gsrActiveResistor = (storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x0E)
                        >> 1;
            }
            else
            {
                GSR_setRange(HW_RES_40K);
                gsrActiveResistor = HW_RES_40K;
            }
        }

        // DMP related - start
        // Enable DMP
        if (storedConfig[NV_CONFIG_SETUP_BYTE4] & MPU9150_MPL_DMP)
        {
            MPU_platformInit(&storedConfig[0]);
        }
        else if ((storedConfig[NV_SENSORS0] & SENSOR_MPU9150_GYRO)
                // DMP related - end
//--      if((storedConfig[NV_SENSORS0] & SENSOR_MPU9150_GYRO)
                || (storedConfig[NV_SENSORS2] & SENSOR_MPU9150_ACCEL)
                || (storedConfig[NV_SENSORS2] & SENSOR_MPU9150_MAG))
        {
            MPU9150_init();
            i2cEn = 1;
            MPU9150_wake(1);
            volatile uint8_t mpu_id = MPU9150_getId();
            volatile uint8_t mag_id = MPU9150_getMagId();
            if ((storedConfig[NV_SENSORS0] & SENSOR_MPU9150_GYRO)
                    || (storedConfig[NV_SENSORS2] & SENSOR_MPU9150_ACCEL))
            {
                MPU9150_setSamplingRate(storedConfig[NV_CONFIG_SETUP_BYTE1]);
                if (storedConfig[NV_SENSORS0] & SENSOR_MPU9150_GYRO)
                {
                    MPU9150_setGyroSensitivity(
                            storedConfig[NV_CONFIG_SETUP_BYTE2] & 0x03); //This needs to go after the wake?
                }
                if (storedConfig[NV_SENSORS2] & SENSOR_MPU9150_ACCEL)
                {
                    MPU9150_setAccelRange(
                            (storedConfig[NV_CONFIG_SETUP_BYTE3] & 0xC0) >> 6);
                }
            }
            else
            {
                //For some reason it seems necessary to power on the gyro/accel core before trying to access the mag
                //followed by one other I2C command (read or write)
                //No idea why
                //timing delays or other I2C commands to gyro/accel core do not seem to have the same effect?!?
                //Only relevant first time mag is accessed after powering up MPU9150
                MPU9150_wake(0);
            }
            if (storedConfig[NV_SENSORS2] & SENSOR_MPU9150_MAG)
            {
                if (*(uint16_t *) (storedConfig + NV_SAMPLING_RATE) >= clk_120)
                {
                    //max of approx. 3ms to sample everything + 9ms between starting mag to data ready
                    //so 12ms in total (394 ticks of 32768Hz clock = 12.024ms) (3070 ticks of 255765.625Hz clock = 12.024ms)
                    //so there is time to get the mag sampled before the readings need to start each sample period
                    preSampleMpuMag = 1;
                }
                else
                {
                    //sample, then check each sample period if ready to read. Start new sample immediately
                    MPU9150_startMagMeasurement();
                    preSampleMpuMag = 0;
                    mpuMagCount = mpuMagFreq = (clk_90
                            / *(uint16_t *) (storedConfig + NV_SAMPLING_RATE))
                            + 1;
                }
            }
        }

        if ((storedConfig[NV_SENSORS0] & SENSOR_LSM303DLHC_MAG)
                || (storedConfig[NV_SENSORS1] & SENSOR_LSM303DLHC_ACCEL))
        {
            if (!i2cEn)
            {
                //Initialise I2C
                if (lsmInUse == LSM303DLHC_IN_USE)
                {
                    LSM303DLHC_init();
                }
                else
                {
                    LSM303AHTR_init();
                }
                i2cEn = 1;
            }
            if (storedConfig[NV_SENSORS1] & SENSOR_LSM303DLHC_ACCEL)
            {
                if (lsmInUse == LSM303DLHC_IN_USE)
                {
                    LSM303DLHC_accelInit(
                            ((storedConfig[NV_CONFIG_SETUP_BYTE0] & 0xF0) >> 4), //sampling rate
                            ((storedConfig[NV_CONFIG_SETUP_BYTE0] & 0x0C) >> 2), //range
                            ((storedConfig[NV_CONFIG_SETUP_BYTE0] & 0x02) >> 1), //low power mode
                            (storedConfig[NV_CONFIG_SETUP_BYTE0] & 0x01)); //high resolution mode
                }
                else
                {
                    LSM303AHTR_accelInit(
                            ((storedConfig[NV_CONFIG_SETUP_BYTE0] & 0xF0) >> 4), //sampling rate
                            ((storedConfig[NV_CONFIG_SETUP_BYTE0] & 0x0C) >> 2), //range
                            ((storedConfig[NV_CONFIG_SETUP_BYTE0] & 0x02) >> 1), //low power mode
                            (storedConfig[NV_CONFIG_SETUP_BYTE0] & 0x01)); //high resolution mode
                }
            }
            if (storedConfig[NV_SENSORS0] & SENSOR_LSM303DLHC_MAG)
            {
                if (lsmInUse == LSM303DLHC_IN_USE)
                {
                    LSM303DLHC_magInit(
                            ((storedConfig[NV_CONFIG_SETUP_BYTE2] & 0x1C) >> 2), //sampling rate
                            ((storedConfig[NV_CONFIG_SETUP_BYTE2] & 0xE0) >> 5)); //gain
                }
                else
                {
                    LSM303AHTR_magInit(
                            ((storedConfig[NV_CONFIG_SETUP_BYTE2] & 0x1C) >> 2)); //sampling rate
                }
            }
        }

        if (storedConfig[NV_SENSORS2] & SENSOR_BMPX80_PRESSURE)
        {
            if (!i2cEn)
            {
                BMPX80_init();
            }
            //BMP180_getCalibCoeff(&sdHeadText[SDH_TEMP_PRES_CALIBRATION]);
            if ((((storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x30) >> 4) == 0)
                    && (*(uint16_t *) (storedConfig + NV_SAMPLING_RATE)
                            >= ((bmpInUse == BMP180_IN_USE) ? clk_75 : clk_85)))
            {
                //max of approx. 3ms to sample everything + 4.5ms between starting press to data ready
                //so7.5ms in total
                preSampleBmpPress = 1;
                bmpPressFreq = 1; //required for the calculation of sampleBmpTempFreq below
            }
            else if ((((storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x30) >> 4) == 1)
                    && (*(uint16_t *) (storedConfig + NV_SAMPLING_RATE)
                            >= ((bmpInUse == BMP180_IN_USE) ? clk_105 : clk_145)))
            {
                //max of approx. 3ms to sample everything + 7.5ms between starting press to data ready
                //so 10.5ms in total
                preSampleBmpPress = 1;
                bmpPressFreq = 1; //required for the calculation of sampleBmpTempFreq below
            }
            else if ((((storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x30) >> 4) == 2)
                    && (*(uint16_t *) (storedConfig + NV_SAMPLING_RATE)
                            >= ((bmpInUse == BMP180_IN_USE) ? clk_165 : clk_225)))
            {
                //max of approx. 3ms to sample everything + 13.5ms between starting press to data ready
                //so 16.5ms in total
                preSampleBmpPress = 1;
                bmpPressFreq = 1; //required for the calculation of sampleBmpTempFreq below
            }
            else if ((((storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x30) >> 4) == 3)
                    && (*(uint16_t *) (storedConfig + NV_SAMPLING_RATE)
                            >= ((bmpInUse == BMP180_IN_USE) ? clk_285 : clk_405)))
            {
                //max of approx. 3ms to sample everything + 25.5ms between starting press to data ready
                //so 28.5ms in total
                preSampleBmpPress = 1;
                bmpPressFreq = 1; //required for the calculation of sampleBmpTempFreq below
            }
            else
            {
                //sample, then check each sample period if ready to read. Start new sample immediately
                bmpTempStartTs = RTC_get64();
                BMPX80_startTempMeasurement();
                preSampleBmpPress = 0;
                if (((storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x30) >> 4) == 0)
                {
                    bmpPressCount = bmpPressFreq = ((
                            (bmpInUse == BMP180_IN_USE) ? clk_45 : clk_55)
                            / *(uint16_t *) (storedConfig + NV_SAMPLING_RATE))
                            + 1;      //
                }
                else if (((storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x30) >> 4)
                        == 1)
                {
                    bmpPressCount = bmpPressFreq = ((
                            (bmpInUse == BMP180_IN_USE) ? clk_75 : clk_115)
                            / *(uint16_t *) (storedConfig + NV_SAMPLING_RATE))
                            + 1;      //
                }
                else if (((storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x30) >> 4)
                        == 2)
                {
                    bmpPressCount = bmpPressFreq = ((
                            (bmpInUse == BMP180_IN_USE) ? clk_135 : clk_195)
                            / *(uint16_t *) (storedConfig + NV_SAMPLING_RATE))
                            + 1;      //
                }
                else
                {
                    bmpPressCount = bmpPressFreq = ((
                            (bmpInUse == BMP180_IN_USE) ? clk_255 : clk_375)
                            / *(uint16_t *) (storedConfig + NV_SAMPLING_RATE))
                            + 1;      //
                }
            }
            //only need to sample temp once a second at most
            if (*(uint16_t *) (storedConfig + NV_SAMPLING_RATE) >= clk_2500)
            {
                //less than 4Hz
                //so every second sample must be temp
                sampleBmpTemp = sampleBmpTempFreq = 1;
            }
            else
            {
                sampleBmpTemp = sampleBmpTempFreq = (uint8_t) ((FreqDiv(
                        *(uint16_t *) (storedConfig + NV_SAMPLING_RATE)) - 1)
                        / bmpPressFreq);
            }
            switch ((storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x30) >> 4)
            {
            case 0:
                bmpPresInterval =
                        ((bmpInUse == BMP180_IN_USE) ? clk_45 : clk_55);
                break;
            case 1:
                bmpPresInterval = (
                        (bmpInUse == BMP180_IN_USE) ? clk_75 : clk_115);
                break;
            case 2:
                bmpPresInterval = (
                        (bmpInUse == BMP180_IN_USE) ? clk_135 : clk_195);
                break;
            case 3:
                bmpPresInterval = (
                        (bmpInUse == BMP180_IN_USE) ? clk_255 : clk_375);
                break;
            default:
                bmpPresInterval = (
                        (bmpInUse == BMP180_IN_USE) ? clk_255 : clk_375);
                break;
            }
            bmpTempInterval = ((bmpInUse == BMP180_IN_USE) ? clk_45 : clk_55);
            memset(bmpVal, 0, BMPX80_PACKET_SIZE);
        }

        /* ExG */
        if ((storedConfig[NV_SENSORS0] & SENSOR_EXG1_24BIT)
                || (storedConfig[NV_SENSORS0] & SENSOR_EXG2_24BIT)
                || (storedConfig[NV_SENSORS2] & SENSOR_EXG1_16BIT)
                || (storedConfig[NV_SENSORS2] & SENSOR_EXG2_16BIT))
        {
            EXG_init();

            if (storedConfig[NV_SENSORS0] & SENSOR_EXG1_24BIT
                    || storedConfig[NV_SENSORS2] & SENSOR_EXG1_16BIT)
            {
                EXG_writeRegs(0, ADS1292R_CONFIG1, 10,
                              (storedConfig + NV_EXG_ADS1292R_1_CONFIG1));
            }

            /* This second long delay was added to satisfy program flow requirements
             * of the ADS1292R as per pg 63 of its datasheet. */
            __delay_cycles(24000000);

            if (storedConfig[NV_SENSORS0] & SENSOR_EXG2_24BIT
                    || storedConfig[NV_SENSORS2] & SENSOR_EXG2_16BIT)
            {
                EXG_writeRegs(1, ADS1292R_CONFIG1, 10,
                              (storedConfig + NV_EXG_ADS1292R_2_CONFIG1));
            }
            /*probably turning on internal reference, so wait for it to settle*/
            __delay_cycles(2400000);   /*100ms (assuming 24MHz clock)*/

            /*probably setting the PGA gain so cancel the channel offset*/
            if (((storedConfig[NV_SENSORS0] & SENSOR_EXG1_24BIT)
                    || (storedConfig[NV_SENSORS2] & SENSOR_EXG1_16BIT))
                    && (storedConfig[NV_EXG_ADS1292R_1_RESP2] & BIT7))
            {
                EXG_offsetCal(0);
            }
            if (((storedConfig[NV_SENSORS0] & SENSOR_EXG2_24BIT)
                    || (storedConfig[NV_SENSORS2] & SENSOR_EXG2_16BIT))
                    && (storedConfig[NV_EXG_ADS1292R_2_RESP2] & BIT7))
            {
                EXG_offsetCal(1);
            }

            if (((storedConfig[NV_SENSORS0] & SENSOR_EXG1_24BIT)
                    || (storedConfig[NV_SENSORS2] & SENSOR_EXG1_16BIT))
                    && ((storedConfig[NV_SENSORS0] & SENSOR_EXG2_24BIT)
                            || (storedConfig[NV_SENSORS2] & SENSOR_EXG2_16BIT)))
            {
                EXG_start(2);
            }
            else if ((storedConfig[NV_SENSORS0] & SENSOR_EXG1_24BIT)
                    || (storedConfig[NV_SENSORS2] & SENSOR_EXG1_16BIT))
            {
                EXG_start(0);
            }
            else
            {
                EXG_start(1);
            }

            if ((daughtCardId[DAUGHT_CARD_ID] == 47)
                    && (daughtCardId[DAUGHT_CARD_REV] >= 4))
            {
                /* Check if unit is SR47-4 or greater.
                 * If so, amend configuration byte 2 of ADS chip 1 to have bit 3 set to 1.
                 * This ensures clock lines on ADS chip are correct
                 */
                ADS1292_disableDrdyInterrupts(ADS1292_DRDY_INT_CHIP2);
                adsClockTied = 1;
            }
        }
    }

    SampleTimerStart();
    if (sdHeadText[SDH_TRIAL_CONFIG0] & SDH_TIME_SYNC)
        PrepareSDBuffHead();
    sensing = 1;
}

inline void StopStreaming(void)
{
    configuring = 1;
    sensing = 0;
    if (storedConfig[NV_CONFIG_SETUP_BYTE4] & MPU9150_MPL_DMP)
    {
        MPU_saveCalibration();
        MPL_saveCalibrationBytes();
    }

    /*shut everything down*/

    if (docked)
    {   /*if docked, cannot write to SD card any more*/
        DockSdPowerCycle();
    }
    else
    {
        Write2SD();
        fileBad = f_close(&dataFil);
        newDirFlag = 1;
        msp430_delay_ms(50);
        P4OUT &= ~BIT2;      /*sd access close*/
    }

    SampleTimerStop();
    ADC_disable();
    DMA0_disable();

    P8OUT &= ~BIT6;

    P3OUT &= ~BIT3;       /*set EXP_RESET_N low*/
    P2OUT &= ~BIT0;       /*set GPIO_INTERNAL1 low (strain)*/

    /*DMP related - start*/
    if (storedConfig[NV_CONFIG_SETUP_BYTE4] & MPU9150_MPL_DMP)
    {
        MPU9150_reset();
    }
     /*DMP related - end*/
    else if ((storedConfig[NV_SENSORS0] & SENSOR_MPU9150_GYRO)
            || (storedConfig[NV_SENSORS2] & SENSOR_MPU9150_ACCEL))
    {
        MPU9150_wake(0);
    }

    if ((storedConfig[NV_SENSORS0] & SENSOR_LSM303DLHC_MAG)
            || (storedConfig[NV_SENSORS1] & SENSOR_LSM303DLHC_ACCEL))
    {
        if (lsmInUse == LSM303DLHC_IN_USE)
        {
            LSM303DLHC_sleep();
        }
        else
        {
            LSM303AHTR_sleep();
        }
    }

    if (!docked)
    {
        if ((storedConfig[NV_SENSORS0] & SENSOR_EXG1_24BIT)
                || (storedConfig[NV_SENSORS2] & SENSOR_EXG1_16BIT))
        {
            EXG_stop(0);       /*probably not needed*/
        }
        if ((storedConfig[NV_SENSORS0] & SENSOR_EXG2_24BIT)
                || (storedConfig[NV_SENSORS2] & SENSOR_EXG2_16BIT))
        {
            EXG_stop(1);       /*probably not needed*/
        }
        if ((storedConfig[NV_SENSORS0] & SENSOR_EXG1_24BIT)
                || (storedConfig[NV_SENSORS0] & SENSOR_EXG2_24BIT)
                || (storedConfig[NV_SENSORS2] & SENSOR_EXG1_16BIT)
                || (storedConfig[NV_SENSORS2] & SENSOR_EXG2_16BIT))
        {
            EXG_powerOff();
        }
    }
    if (docked)
    {
        UART_activate();
    }

    msp430_delay_ms(10); /*give plenty of time for I2C operations to finish before disabling I2C*/
    I2C_Disable();
    i2cEn = 0;
    P8OUT &= ~BIT4;         /*set SW_I2C low to power off I2C chips*/
    streamData = 0;
    TaskClear(TASK_WR2SD);
    sdBuffLen = 0;
    TaskClear(TASK_SAMPLEMPU9150MAG);
    TaskClear(TASK_SAMPLEBMPX80PRESS);
    RcStop();
#if SKIP100MS
    skip100ms = 0;
#endif
    configuring = 0;
}

char i2cSlavePresent(char address)
{
    char isPresent = 0;

    /*I2C pull-ups must be enabled (on rev4 boards) before initialising I2C bus*/
    P8OUT |= BIT4;          /*enable I2C pull-ups by turning on SW_I2C*/
    P3OUT |= BIT3;
    __delay_cycles(48000);  /*2ms*/

    /*Source from SMCLK, which is running @ 24MHz. 4kHz desired BRCLK*/
    I2C_Master_Init(S_MCLK, 24000000, 400000);

    isPresent = TI_USCI_I2C_slave_present(address);

    P8OUT &= ~BIT4;         /*disable I2C pull-ups by turning off SW_I2C*/
    __delay_cycles(120000); /*5ms (assuming 24MHz MCLK) to ensure no writes pending*/
    P3OUT &= ~BIT3;

    return isPresent;
}

void i2cSlaveDiscover(void)
{
    char address;

    //I2C pull-ups must be enabled (on rev4 boards) before initialising I2C bus
    P8OUT |= BIT4;          //enable I2C pull-ups by turning on SW_I2C
    P3OUT |= BIT3;
    __delay_cycles(48000);  //2ms

    //Source from SMCLK, which is running @ 24MHz. 4kHz desired BRCLK
    I2C_Master_Init(S_MCLK, 24000000, 400000);

    slave_address_pointer = &slave_addresses[0];
    for (address = 1; address < 127; address++)
    {
        if (TI_USCI_I2C_slave_present(address))
        {
            *slave_address_pointer++ = address;
        }
    }
    *slave_address_pointer++ = 0xFF;
    *slave_address_pointer = 0xFE;

    P8OUT &= ~BIT4;         //disable I2C pull-ups by turning off SW_I2C
    __delay_cycles(120000); //5ms (assuming 24MHz MCLK) to ensure no writes pending
    P3OUT &= ~BIT3;
}

void ConfigureChannels(void)
{
    uint16_t mask = 0;

    nbrAdcChans = nbrDigiChans = 0;

    //Analog Accel
    if (storedConfig[NV_SENSORS0] & SENSOR_A_ACCEL)
    {
        mask += MASK_A_ACCEL;
        nbrAdcChans += 3;
    }
    //Battery Voltage
    vbattEn = (storedConfig[NV_SENSORS1] & SENSOR_VBATT) ? 1 : 0;
    mask += MASK_VBATT;
    nbrAdcChans++;
    //Strain gauge
    if (storedConfig[NV_SENSORS1] & SENSOR_STRAIN)
    {
        mask += MASK_STRAIN;
        nbrAdcChans += 2;
    }
    //External ADC channel A7
    if (storedConfig[NV_SENSORS0] & SENSOR_EXT_A7)
    {
        mask += MASK_EXT_A7;
        nbrAdcChans++;
    }
    //External ADC channel A6
    if (storedConfig[NV_SENSORS0] & SENSOR_EXT_A6)
    {
        mask += MASK_EXT_A6;
        nbrAdcChans++;
    }
    //External ADC channel A15
    if (storedConfig[NV_SENSORS1] & SENSOR_EXT_A15)
    {
        mask += MASK_EXT_A15;
        nbrAdcChans++;
    }
    //Internal ADC channel A12
    if (storedConfig[NV_SENSORS1] & SENSOR_INT_A12)
    {
        mask += MASK_INT_A12;
        nbrAdcChans++;
    }
    //Internal ADC channel A13
    if ((storedConfig[NV_SENSORS1] & SENSOR_INT_A13)
            && !(storedConfig[NV_SENSORS1] & SENSOR_STRAIN))
    {
        mask += MASK_INT_A13;
        nbrAdcChans++;
    }
    //Internal ADC channel A14
    if ((storedConfig[NV_SENSORS2] & SENSOR_INT_A14)
            && !(storedConfig[NV_SENSORS1] & SENSOR_STRAIN))
    {
        mask += MASK_INT_A14;
        nbrAdcChans++;
    }
    //Internal ADC channel A1
    if (storedConfig[NV_SENSORS0] & SENSOR_GSR)
    {
        mask += MASK_INT_A1;
        nbrAdcChans++;
    }
    if (storedConfig[NV_SENSORS1] & SENSOR_INT_A1)
    {
        mask += MASK_INT_A1;
        nbrAdcChans++;
    }
    //Digi Gyro
    if (storedConfig[NV_SENSORS0] & SENSOR_MPU9150_GYRO)
    {
        nbrDigiChans += 3;
    }
    //Digi Accel
    if (storedConfig[NV_SENSORS1] & SENSOR_LSM303DLHC_ACCEL)
    {
        nbrDigiChans += 3;
    }
    //Mag
    if (storedConfig[NV_SENSORS0] & SENSOR_LSM303DLHC_MAG)
    {
        nbrDigiChans += 3;
    }
    //Digi Accel - MPU9150
    if (storedConfig[NV_SENSORS2] & SENSOR_MPU9150_ACCEL)
    {
        nbrDigiChans += 3;
    }
    //Digi Mag - MPU9150
    if (storedConfig[NV_SENSORS2] & SENSOR_MPU9150_MAG)
    {
        nbrDigiChans += 3;
    }
#if TS_BYTE3
    blockLen = (((nbrAdcChans - (!vbattEn) + nbrDigiChans) * 2) + 3);
#else
    blockLen = (((nbrAdcChans - (!vbattEn) + nbrDigiChans)*2)+2);
#endif

    if (storedConfig[NV_SENSORS2] & SENSOR_BMPX80_PRESSURE)
    {
        nbrDigiChans += 2;         //PRES & TEMP, ON/OFF together
        blockLen += 5;
    }

    if (storedConfig[NV_SENSORS0] & SENSOR_EXG1_24BIT)
    {
        nbrDigiChans += 2;
        blockLen += 7;
    }
    if (storedConfig[NV_SENSORS2] & SENSOR_EXG1_16BIT)
    {
        nbrDigiChans += 2;
        blockLen += 5;
    }
    if (storedConfig[NV_SENSORS0] & SENSOR_EXG2_24BIT)
    {
        nbrDigiChans += 2;
        blockLen += 7;
    }
    if (storedConfig[NV_SENSORS2] & SENSOR_EXG2_16BIT)
    {
        nbrDigiChans += 2;
        blockLen += 5;
    }

    // DMP related - start
    if (storedConfig[NV_CONFIG_SETUP_BYTE4] & MPU9150_MPL_DMP)
    {
        if (storedConfig[NV_SENSORS3] & SENSOR_MPU9150_MPL_QUAT_6DOF)
        {
            nbrDigiChans += 4;
            blockLen += 16;
        }
        if (storedConfig[NV_SENSORS3] & SENSOR_MPU9150_MPL_QUAT_9DOF)
        {
            nbrDigiChans += 4;
            blockLen += 16;
        }
        //Euler 6DOF
        if (storedConfig[NV_SENSORS3] & SENSOR_MPU9150_MPL_EULER_6DOF)
        {
            nbrDigiChans += 3;
            blockLen += 12;
        }
        //--Euler 9DOF
        if (storedConfig[NV_SENSORS3] & SENSOR_MPU9150_MPL_EULER_9DOF)
        {
            nbrDigiChans += 3;
            blockLen += 12;
        }
        // Heading
        if (storedConfig[NV_SENSORS3] & SENSOR_MPU9150_MPL_HEADING)
        {
            nbrDigiChans += 1;
            blockLen += 4;
        }
        // MPU9150 temperature
        if (storedConfig[NV_SENSORS2] & SENSOR_MPU9150_TEMP)
        {
            nbrDigiChans += 1;
            blockLen += 4;
        }
        //Pedometer
        if (storedConfig[NV_SENSORS3] & SENSOR_MPU9150_MPL_PEDOMETER)
        {
            nbrDigiChans += 2;
            blockLen += 8;
        }
        //Tap_Dir_and_Cnt
        if (storedConfig[NV_SENSORS3] & SENSOR_MPU9150_MPL_TAP)
        {
            nbrDigiChans += 1;
            blockLen += 1;
        }
        //Mot_and_Orient
        if (storedConfig[NV_SENSORS3] & SENSOR_MPU9150_MPL_MOTION_ORIENT)
        {
            nbrDigiChans += 1;
            blockLen += 1;
        }
        //Digi Gyro - MPU9150
        if (storedConfig[NV_SENSORS4] & SENSOR_MPU9150_GYRO_CAL)
        {
            nbrDigiChans += 3;
            blockLen += 12;
        }
        //Digi Accel - MPU9150
        if (storedConfig[NV_SENSORS4] & SENSOR_MPU9150_ACCEL_CAL)
        {
            nbrDigiChans += 3;
            blockLen += 12;
        }
        //Digi Mag - MPU9150
        if (storedConfig[NV_SENSORS4] & SENSOR_MPU9150_MAG_CAL)
        {
            nbrDigiChans += 3;
            blockLen += 12;
        }
        //Raw 6DOF Quaternions
        if (storedConfig[NV_SENSORS4] & SENSOR_MPU9150_MPL_QUAT_6DOF_RAW)
        {
            nbrDigiChans += 4;
            blockLen += 16;
        }
    }
    // DMP related - end

    if (mask)
    {
        adcStartPtr = ADC_init(mask);
        DMA0_transferDoneFunction(&Dma0ConversionDone);
        if (adcStartPtr)
            DMA0_init(adcStartPtr, (uint16_t *) (txBuff0 + 4), nbrAdcChans);
    }
}

// Switch SW1, BT_RTS and BT connect/disconnect
#pragma vector=PORT1_VECTOR
__interrupt void Port1_ISR(void)
{
    // Context save interrupt flag before calling interrupt vector.
    // Reading interrupt vector generator will automatically clear IFG flag
    switch (__even_in_range(P1IV, P1IV_P1IFG7))
    {
    //BT Connect/Disconnect
    case P1IV_P1IFG0:
        if (P1IN & BIT0)
        {       //BT is connected
            P1IES |= BIT0;       //look for falling edge
            BT_connectionInterrupt(1);
            btIsConnected = 1;
            BT_rst_MessageProgress();
            if (sdHeadText[SDH_TRIAL_CONFIG0] & SDH_IAMMASTER)
            {
                // center sends (0xE0) and is waiting for 6 byte RC info from DMA2
                if (rcommStatus)
                {
                    DMA2SZ = 1;
                    DMA2_enable();
                    RcCenterT10();
                }
            }
            else
            {
                // node is waiting for 1 byte ROUTINE_COMMUNICATION(0xE0) from DMA2
                rcNodeR10Cnt = 0;
                DMA2SZ = 10;
                DMA2_enable();
            }
            //Board_ledToggle(LED_BLUE);
        }
        else
        {                //BT is disconnected
            P1IES &= ~BIT0;      //look for rising edge
            btIsConnected = 0;
            BT_connectionInterrupt(0);
        }
        break;

        //BT RTS
    case P1IV_P1IFG3:
        if (P1IN & BIT3)
        {
            P1IES |= BIT3;       //look for falling edge
            BT_rtsInterrupt(1);
        }
        else
        {
            P1IES &= ~BIT3;      //look for rising edge
            BT_rtsInterrupt(0);  // when 0, can call sendNextChar();
        }
        break;

        //BUTTON_SW1
    case P1IV_P1IFG6:
        // DMP related - start
        // added from LogAndStream FW
        if (!(P1IN & BIT6))
        {           //button pressed
            buttonPressTs64 = RTC_get64();
            P1IES &= ~BIT6;
            // call calibrate function 3s after button button pressed - a check is performed inside MPU_calibrate_cb to see if button is still pressed
            // only allow calibration when not docked and not sensing
            if (!sensing && !docked
                    && (storedConfig[NV_CONFIG_SETUP_BYTE4] & MPU9150_MPL_DMP)
                    && !(taskList & TASK_STARTSENSING) && !mplCalibrating
                    && !mplCalibrateInit)
            {
                mplCalibrateInit = 1;
                msp430_register_timer_cb(MPL_calibrateCB, 3300);         //, 0);
            }
            else
                mplCalibrateInit = 0;

            Board_ledOn(LED_GREEN0);

        }
        else
        {               //button released
            P1IES |= BIT6;
            buttonReleaseTs64 = RTC_get64();
            buttonP2RTs64 = buttonReleaseTs64 - buttonPressTs64;
            buttonTwoPressTd64 = buttonReleaseTs64 - buttonLastReleaseTs64;
            if ((buttonTwoPressTd64 > 16384) && !configuring)
            { // && (buttonP2RTs64>327)
                buttonLastReleaseTs64 = buttonReleaseTs64;
                mplCalibrateInit = 0;
#if !PRESS2UNDOCK
                if (!mplCalibrating)
                {
                    if (storedConfig[NV_SD_TRIAL_CONFIG0]
                            & SDH_USER_BUTTON_ENABLE)
                    {
                        if (sensing)
                        {
                            TaskSet(TASK_STOPSENSING);
                        }
                        else if (!docked)
                        {
                            if (TaskSet(TASK_STARTSENSING))
                                __bic_SR_register_on_exit(LPM3_bits);
                        }
                    }
                }
                else
                    _NOP();
#else
                test_time_1 = RTC_get64();
                if(docked)
                docked = 0;
                else
                docked = 1;
                if(TaskSet(TASK_SETUP_DOCK))
                __bic_SR_register_on_exit(LPM3_bits);
#endif
            }
            else
                mplCalibrateInit = 0;
        }
        _NOP();
        break;

        //ExG chip2 data ready
    case P1IV_P1IFG4:
        EXG_dataReadyChip2();
        break;

        // DMP related - start
        //MPU_INT
    case P1IV_P1IFG7:
        P1IFG &= ~BIT7;                          // P1.7 IFG cleared
        gyro_data_ready_cb();   // Tell the MPL that there is new data
        mpuIntTriggered = 1;  // Tell the Shimmer program that there is new data
//         P1OUT ^= BIT4;
#ifdef SAMP_AT_MPU_INT
                streamData = 1;         // taken from timer interrupt
                __bic_SR_register_on_exit(LPM3_bits);
#endif
//         __bic_SR_register_on_exit(LPM0_bits);
        break;
        // DMP related - end

        // Default case
    default:
        break;
    }
}

// SD_DETECT_N
#pragma vector=PORT2_VECTOR
__interrupt void Port2_ISR(void)
{
    // Context save interrupt flag before calling interrupt vector.
    // Reading interrupt vector generator will automatically clear IFG flag
    // buttonsPressed = PAIFG & BUTTON_ALL;

    switch (__even_in_range(P2IV, P2IV_P2IFG7))
    {
    //ExG chip1 data ready
    case P2IV_P2IFG0:
        EXG_dataReadyChip1();
        if (adsClockTied)
        {
            EXG_dataReadyChip2();
        }
        break;

        //EXP_DETECT_N
    case P2IV_P2IFG1:
        //TODO: Debounce this
        //see slaa513 for example using multiple time bases on a single timer module
        if (P2IN & BIT1)
        {       //card not inserted
            P2IES |= BIT1;       //look for falling edge
        }
        else
        {                //card inserted
            P2IES &= ~BIT1;      //look for rising edge
        }
        break;

        //dock_detect_N
    case P2IV_P2IFG3:
        if (!undockEvent)
        {
            if (!(P2IN & BIT3)) // undocked
            {
                undockEvent = 1;
                battCritical = FALSE;
                time_newUnDockEvent = RTC_get64();
            }
            //see slaa513 for example using multiple time bases on a single timer module
            if (TaskSet(TASK_SETUP_DOCK))
            {
                __bic_SR_register_on_exit(LPM3_bits);
            }
        }

        if (P2IN & BIT3)
        {
            P2IES |= BIT3;       //look for falling edge
        }
        else
        {
            P2IES &= ~BIT3;      //look for rising edge
        }
        break;
        // Default case
    default:
        break;
    }
}

// Timer2:
// ccr1: for blink timer
void CommTimerStart(void)
{
    TA0CTL = TASSEL_1 + MC_2 + TACLR;         //ACLK, continuous mode, clear TAR
    TA0CCTL1 = CCIE;
    TA0CCR1 = 16384;
}

inline uint16_t GetTA0(void)
{
    register uint16_t t0, t1;
    uint8_t ie;
    if (ie = (__get_SR_register() & 0x0008))       //interrupts enabled?
        __disable_interrupt();
    t1 = TA0R;
    do
    {
        t0 = t1;
        t1 = TA0R;
    }
    while (t0 != t1);
    if (ie)
        __enable_interrupt();
    return t1;
}

uint32_t SyncNodeShift(uint8_t shift_value)
{
    uint32_t sync_node_shift = 0x01;
    sync_node_shift <<= shift_value;
    return sync_node_shift;
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
        if (sensing && maxLen)
        {
            if (maxLenCnt < maxLen * SYNC_FACTOR)
                maxLenCnt++;
            else
            {
                TaskSet(TASK_STOPSENSING);
                //stopSensing = 1;
                maxLenCnt = 0;
                return;
            }
        }

        if (rcommStatus)
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
                if (sensing && (syncNodeNum > 0))
                {
                    if (syncCnt == 1)
                    {
                        // start
                        cReboot = 0;
                        BT_init();
                        BT_disableRemoteConfig(1);
                        BtStart();
                        syncNodeCnt = 0;
                        nodeSucc = 0;
                        syncCurrNode = 0;
                        syncCurrNodeDone = 0;
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
                                        BtStop();
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
                                        BT_init();
                                        BT_disableRemoteConfig(1);
                                        BtStart();
                                    }
                                    else if ((cReboot >= 2)
                                            && (cReboot < 5 * SYNC_FACTOR))
                                    {
                                        if (btPowerOn)
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
                                        BtStop();
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
                            BtStop();
                            syncSuccC = 1;
                            if (shortExpFlag)
                                syncCnt = estLen3 * SYNC_FACTOR;
                        }
                    }
                    else if (syncCnt == SYNC_WINDOW_C * SYNC_FACTOR)
                    {
                        // power off
                        BtStop();
                        if (nodeSucc == nodeSuccFull)
                            syncSuccC = 1;
                        else
                            syncSuccC = 0;
                        if (shortExpFlag)
                            syncCnt = estLen3 * SYNC_FACTOR;
                    }
                }
            }
            else
            {                              // i am Node
                if (!syncSuccN)
                {
                    if (syncCnt == 1)
                    {
                        myTimeDiffLongMin = 0;
                        myTimeDiffLongFlagMin = 0;
                        syncNodeSucc = 0;
                        nReboot = 0;
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
                            BtStop();
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
                        BtStop();
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
                        BT_init();
                        BT_disableRemoteConfig(1);
                        BtStart();
                        syncNodeWinExpire = (SYNC_CD * (nReboot + 1)
                                + SYNC_WINDOW_N * (nReboot + 1)) * SYNC_FACTOR;
                    }
                }
            }
        }
        else                                   //idle: no_RC mode
        {
            if (docked)
            {
                if (sensing)
                {
                    if (TaskSet(TASK_STOPSENSING))
                        __bic_SR_register_on_exit(LPM3_bits);
                }
                if (btPowerOn)
                    BtStop();
            }
        }

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

uint8_t RcFindSmallest()
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

    myTimeDiffLongMin = 0;
    myTimeDiffLongFlagMin = 0;
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

    memset(myTimeDiffFlagArr, 0xff, SYNC_TRANS_IN_ONE_COMM);
    memset(myTimeDiffArr, 0, SYNC_TRANS_IN_ONE_COMM);
    return 0;
}

void RcCenterT10()
{
    uint8_t resPacket[10];
    uint16_t packet_length = 0;

    getRcomm = 1;

    *(resPacket + packet_length++) = ACK_COMMAND_PROCESSED;
    *(resPacket + packet_length++) = sensing;
    myLocalTimeLong = rwcTimeDiff64 + RTC_get64();
    *(uint64_t*) (resPacket + packet_length) = myLocalTimeLong;
    packet_length += 8;

    BT_write(resPacket, packet_length);

}
void RcNodeR10()
{
    // only nodes do this
    if (rcommResp[RCT_ACK] == ACK_COMMAND_PROCESSED)
    { //if received the correct 6 bytes:
        uint8_t sd_tolog;
        sd_tolog = rcommResp[RCT_FLG];
        myCenterTimeLong = *(uint64_t*) (rcommResp + RCT_TIME); // get myCenterTimeLong

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

        memset(rcommResp, 0, RCT_SIZE);

        if (rcNodeR10Cnt++ < (SYNC_TRANS_IN_ONE_COMM - 1))
        {
            DMA2SZ = 10;
            DMA2_enable();
            RcNodeT1(1);
        }
        else
        {
            if (onSingleTouch)
            {
                if (!sensing)
                {
                    if (sd_tolog)
                    {
                        TaskSet(TASK_STARTSENSING);
                    }
                }
            }
            syncNodeSucc = 1;
            if (!firstOutlier)
            {
                rcFirstOffsetRxed = 1;
                RcFindSmallest();
                myTimeDiff[0] = myTimeDiffLongFlagMin;
                memcpy(myTimeDiff + 1, (uint8_t*) &myTimeDiffLongMin, 8);
            }
            myTimeDiffLongMin = 0;
            myTimeDiffLongFlagMin = 0;
            rcNodeR10Cnt = 0;
            RcNodeT1(0xff);
        }
    }
}

void RcNodeT1(uint8_t val)
{
    uint8_t tosend = val;
    BT_write(&tosend, 0x01);
    if (syncNodeWinExpire < (syncCnt + SYNC_EXTEND * SYNC_FACTOR))
        syncNodeWinExpire++;
}

void RcCenterR1(void)
{
    if (rcommResp[0] != 0xff)
    {
        DMA2SZ = 1;
        DMA2_enable();
        RcCenterT10();
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

/**
 *** Charge Status Timer for WatchDog
 **/
void ChargeStatusTimerStart(void)
{
    TB0CCR3 = GetTB0() + 3277;
    TB0CCTL3 = CCIE;
    WDTCTL = WDTPW + WDTSSEL__ACLK + WDTCNTCL + WDTIS_4;
}

void ChargeStatusTimerStop(void)
{
    TB0CCTL3 = 0;
    Board_ledOff(LED_ALL);
}

// Blink Timer
// USING TB0 with CCR1
void BlinkTimerStart(void)
{
    blinkStatus = 1;
    TB0Start();
    TB0CCTL3 = CCIE;

    TB0CCR3 = GetTB0() + clk_1000;
}

#pragma vector=TIMER0_B1_VECTOR
__interrupt void TIMER0_B1_ISR(void)
{
    switch (__even_in_range(TB0IV, 14))
    {
    case 0:
        break;                              // No interrupt
    case 2:                                    // TB0CCR1
        //MPU9150 mag
        TB0CCR1 += *(uint16_t *) (storedConfig + NV_SAMPLING_RATE);
        if (TaskSet(TASK_SAMPLEMPU9150MAG))
            __bic_SR_register_on_exit(LPM3_bits);
        break;
    case 4:                                    // TB0CCR2
        //Bmp180 press
        TB0CCR2 += *(uint16_t *) (storedConfig + NV_SAMPLING_RATE);
        if (TaskSet(TASK_SAMPLEBMPX80PRESS))
            __bic_SR_register_on_exit(LPM3_bits);
        break;
    case 6:                                    // TB0CCR3
        if (!(WDTCTL & WDTHOLD))
        {
            // Reset Watchdog timer
            WDTCTL = WDTPW + WDTSSEL__ACLK + WDTCNTCL + WDTIS_4;
        }
        // for LED blink usage details, please check shimmer user manual
        TB0CCR3 += clk_1000;
        if (blinkCnt50++ == 49)
            blinkCnt50 = 0;

        if (blinkCnt20++ == 19)
            blinkCnt20 = 0;

        uint64_t batt_td, batt_my_local_time_64;
        batt_my_local_time_64 = RTC_get64();
        batt_td = batt_my_local_time_64 - battLastTs64;

        if ((batt_td > battInterval) && (!sensing))
        {
            if (TaskSet(TASK_BATT_READ))
                __bic_SR_register_on_exit(LPM3_bits);
            battLastTs64 = batt_my_local_time_64;
        }

        if (blinkStatus)
        {
            // below are settings for green0, yellow and red leds, battery charge status
            if (P2IN & BIT3)
            {
                BattBlinkOn();
            }
            else
            {
                if (!blinkCnt50)
                    BattBlinkOn();
                else
                    Board_ledOff(LED_GREEN0 + LED_YELLOW + LED_RED);
            }

            // code for keeping LED_GREEN0 on when user button pressed
            if ((!(P1IN & BIT6)))
            {
                Board_ledOn(LED_GREEN0);
            }

            // DMP related - Change normal LED operation if MPL is calibrating
            if (!mplCalibrating)
            {
                // below are settings for green1, blue, yellow and red leds
                if (fileBad && !docked)
                {  // bad file = yellow/red alternating

                    if (fileBadCnt-- > 0)
                    {
                        if (fileBadCnt & 0x01)
                        {
                            Board_ledOn(LED_YELLOW);
                            Board_ledOff(LED_RED);
                        }
                        else
                        {
                            Board_ledOff(LED_YELLOW);
                            Board_ledOn(LED_RED);
                        }
                    }
                    else
                    {
                        fileBadCnt = 255;
                    }

                }
                if (rwcErrorFlash && (!sensing))
                {
                    if (!(P1OUT & BIT1))
                    {
                        Board_ledOn(LED_GREEN1);
                        Board_ledOff(LED_BLUE);
                    }
                    else
                    {
                        Board_ledOff(LED_GREEN1);
                        Board_ledOn(LED_BLUE);
                    }
                }
                else
                {
                    // good file - green1:
                    /*if(btIsConnected){
                     Board_ledOff(LED_GREEN1);
                     }
                     else*/
                    {
                        if (!sensing)
                        {                 //standby or configuring
                            if (initializing || configuring)
                            { //configuring
                                if (!(P1OUT & BIT1))
                                    Board_ledOn(LED_GREEN1);
                                else
                                    Board_ledOff(LED_GREEN1);
                            }
                            else
                            {                            //standby
                                if (!blinkCnt20)
                                    Board_ledOn(LED_GREEN1);
                                else
                                    Board_ledOff(LED_GREEN1);
                            }
                        }
                        else
                        {                              //sensing
                            if (blinkCnt20 < 10)
                                Board_ledOn(LED_GREEN1);
                            else
                                Board_ledOff(LED_GREEN1);
                        }
                    }
                    // good file - blue:

                    if (btPowerOn && btIsConnected)
                    {
                        Board_ledToggle(LED_BLUE);
                    }
                    else if ((!rcFirstOffsetRxed) && sensing
                            && (sdHeadText[SDH_TRIAL_CONFIG0] & SDH_TIME_SYNC)
                            && (!(sdHeadText[SDH_TRIAL_CONFIG0] & SDH_IAMMASTER)))
                    {
                        Board_ledOn(LED_BLUE);
                    }
                    else
                    {
                        if (!docked)
                        {
                            if (((blinkCnt20 == 12) || (blinkCnt20 == 14))
                                    && sensing
                                    && (sdHeadText[SDH_TRIAL_CONFIG0]
                                            & SDH_TIME_SYNC))
                            {
                                if (((!syncSuccC)
                                        && (sdHeadText[SDH_TRIAL_CONFIG0]
                                                & SDH_IAMMASTER))
                                        || ((!syncSuccN)
                                                && (!(sdHeadText[SDH_TRIAL_CONFIG0]
                                                        & SDH_IAMMASTER))))
                                {
                                    if (syncCnt > 3)
                                        Board_ledOn(LED_BLUE);
                                }
                                else
                                    Board_ledOff(LED_BLUE);
                            }
                            else
                                Board_ledOff(LED_BLUE);
                        }
                        else
                            Board_ledOff(LED_BLUE);
                    }
                }
            }
            else
            {
                if (mplCalibratingMag)
                {
                    Board_ledOn(LED_BLUE);
                }
                else
                {
                    Board_ledToggle(LED_BLUE);
                }
            }
        }
        break;
    case 8:
        break;                          // TB0CCR4
    case 10:
        break;                          // reserved
    case 12:
        break;                          // reserved
    case 14:
        break;                          // TBIFG overflow handler
    }
}

// BT start
void BtStartDone()
{
    btPowerOn = 1;
}

void BtStart()
{
    if (sdHeadText[SDH_TRIAL_CONFIG0] & SDH_IAMMASTER)
        BT_setRadioMode(MASTER_MODE); //master mode for nodes
    else
        BT_setRadioMode(SLAVE_MODE);  // slave mode for center
    if (!btPowerOn)
        BT_start();
    BT_startDone_cb(BtStartDone);
}

void BtStop()
{
    //rcNodeR10 = 0;
    TaskClear(TASK_RCNODER10);
    btBad = 0;                      //reset bt status, don't report bt problem
    getRcomm = 0;                   //don't try to get routine comm info
    DMA2_disable();                  //dma2 for bt disabled
    btIsConnected = 0;               //set connect status to false
    BT_connectionInterrupt(0);
    btPowerOn = 0;                   //set bt status to off
    P1IES &= ~BIT0;                  //look for rising edge
    BT_disable();                    //set bt disable, stop starting progress
    BT_rst_MessageProgress();        //reset message progress vars to 0
}

// Sample Timer
void SampleTimerStart(void)
{
    uint16_t val_tb0;
    val_tb0 = GetTB0();
    if (preSampleMpuMag || preSampleBmpPress)
    {
        if (preSampleBmpPress
                && (((storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x30) >> 4) == 0))
        {
            if (preSampleMpuMag)
            {
                TB0CCR0 = val_tb0
                        + *(uint16_t *) (storedConfig + NV_SAMPLING_RATE)
                        + clk_90;
                TB0CCR1 = val_tb0
                        + *(uint16_t *) (storedConfig + NV_SAMPLING_RATE);
                TB0CCR2 = val_tb0
                        + *(uint16_t *) (storedConfig + NV_SAMPLING_RATE)
                        + ((bmpInUse == BMP180_IN_USE) ? clk_90_45 : clk_90_55);
                TB0CCTL1 = CCIE;
            }
            else
            {
                TB0CCR0 = val_tb0
                        + *(uint16_t *) (storedConfig + NV_SAMPLING_RATE)
                        + ((bmpInUse == BMP180_IN_USE) ? clk_45 : clk_55);
                TB0CCR2 = val_tb0
                        + *(uint16_t *) (storedConfig + NV_SAMPLING_RATE);
                TB0CCTL1 = 0;
            }
            TB0CCTL2 = CCIE;
        }
        else if (preSampleBmpPress
                && (((storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x30) >> 4) == 1))
        {
            if (preSampleMpuMag)
            {
                TB0CCR0 = val_tb0
                        + *(uint16_t *) (storedConfig + NV_SAMPLING_RATE)
                        + clk_90;
                TB0CCR1 = val_tb0
                        + *(uint16_t *) (storedConfig + NV_SAMPLING_RATE);
                //2302-1919
                TB0CCR2 = val_tb0
                        + *(uint16_t *) (storedConfig + NV_SAMPLING_RATE)
                        + ((bmpInUse == BMP180_IN_USE) ? clk_90_75 : clk_90_85);
                TB0CCTL1 = CCIE;
            }
            else
            {
                TB0CCR0 = val_tb0
                        + *(uint16_t *) (storedConfig + NV_SAMPLING_RATE)
                        + ((bmpInUse == BMP180_IN_USE) ? clk_75 : clk_85);
                TB0CCR2 = val_tb0
                        + *(uint16_t *) (storedConfig + NV_SAMPLING_RATE);
                TB0CCTL1 = 0;
            }
            TB0CCTL2 = CCIE;
        }
        else if (preSampleBmpPress
                && (((storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x30) >> 4) == 2))
        {
            TB0CCR0 = val_tb0 + *(uint16_t *) (storedConfig + NV_SAMPLING_RATE)
                    + ((bmpInUse == BMP180_IN_USE) ? clk_135 : clk_195);
            TB0CCR2 = val_tb0 + *(uint16_t *) (storedConfig + NV_SAMPLING_RATE);
            TB0CCTL2 = CCIE;
            if (preSampleMpuMag)
            {
                TB0CCR1 =
                        val_tb0
                                + *(uint16_t *) (storedConfig + NV_SAMPLING_RATE)
                                + ((bmpInUse == BMP180_IN_USE) ?
                                        clk_135_90 : clk_195_90);
                TB0CCTL1 = CCIE;
            }
            else
                TB0CCTL1 = 0;
        }
        else if (preSampleBmpPress
                && (((storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x30) >> 4) == 3))
        {
            TB0CCR0 = val_tb0 + *(uint16_t *) (storedConfig + NV_SAMPLING_RATE)
                    + ((bmpInUse == BMP180_IN_USE) ? clk_255 : clk_375);
            TB0CCR2 = val_tb0 + *(uint16_t *) (storedConfig + NV_SAMPLING_RATE);
            TB0CCTL2 = CCIE;
            if (preSampleMpuMag)
            {
                TB0CCR1 =
                        val_tb0
                                + *(uint16_t *) (storedConfig + NV_SAMPLING_RATE)
                                + ((bmpInUse == BMP180_IN_USE) ?
                                        clk_255_90 : clk_375_90);
                TB0CCTL1 = CCIE;
            }
            else
                TB0CCTL1 = 0;
        }
        else
        {
            TB0CCR0 = val_tb0 + *(uint16_t *) (storedConfig + NV_SAMPLING_RATE)
                    + clk_90;
            TB0CCR1 = val_tb0 + *(uint16_t *) (storedConfig + NV_SAMPLING_RATE);
            TB0CCTL1 = CCIE;
            TB0CCTL2 = 0;
        }
    }
    else
    {
        TB0CCTL1 = 0;
        TB0CCTL2 = 0;
        TB0CCR0 = val_tb0 + *(uint16_t *) (storedConfig + NV_SAMPLING_RATE);
    }
    TB0CCTL0 = CCIE;
    sampleTimerStatus = 1;
    TB0Start();
}

inline void SampleTimerStop(void)
{
    sampleTimerStatus = 0;
    TB0Stop();
    TB0CCTL0 &= ~CCIE;
    TB0CCTL1 &= ~CCIE;
    TB0CCTL2 &= ~CCIE;
}

#pragma vector=TIMER0_B0_VECTOR
__interrupt void TIMER0_B0_ISR(void)
{
    TB0CCR0 += *(uint16_t *) (storedConfig + NV_SAMPLING_RATE);
    if (!streamDataInProc)
    {
        streamDataInProc = 1;
#if TS_BYTE3
        uint8_t rtc_temp[4];
        if (firstTsFlag == 1)
        {
            firstTs = RTC_get64();
            firstTsFlag = 2;
            *(uint32_t *) rtc_temp = (uint64_t) firstTs;
        }
        else
        {
            *(uint32_t *) rtc_temp = RTC_get32();
        }
        if (currentBuffer)
        {
            //*((uint16_t *)(txBuff1+2)) = timer_b0;   //the first two bytes are packet type bytes. reserved for BTstream
            txBuff1[1] = rtc_temp[0];
            txBuff1[2] = rtc_temp[1];
            txBuff1[3] = rtc_temp[2];
        }
        else
        {
            //*((uint16_t *)(txBuff0+2)) = timer_b0;
            txBuff0[1] = rtc_temp[0];
            txBuff0[2] = rtc_temp[1];
            txBuff0[3] = rtc_temp[2];
        }
#else
        if(currentBuffer)
        {
            *((uint16_t *)(txBuff1+2)) = GetTB0(); //the first two bytes are packet type bytes. reserved for BTstream
            if(clockFreq == (float) TCXO_CLOCK)
            {
                *((uint16_t *)(txBuff1+4)) = GetTA0();
            }
        }
        else
        {
            *((uint16_t *)(txBuff0+2)) = GetTB0();
            if(clockFreq == (float) TCXO_CLOCK)
            {
                *((uint16_t *)(txBuff0+4)) = GetTA0();
            }
        }
#endif
    }
    //start ADC conversion
    DMA0_enable();
    ADC_startConversion();
}

uint8_t Dma0ConversionDone(void)
{
    uint8_t adc_offset;

    adc_offset = 4;

    if (currentBuffer)
    {
        //Destination address for next transfer
        DMA0_repeatTransfer(adcStartPtr, (uint16_t *) (txBuff0 + adc_offset),
                            nbrAdcChans);
    }
    else
    {
        //Destination address for next transfer
        DMA0_repeatTransfer(adcStartPtr, (uint16_t *) (txBuff1 + adc_offset),
                            nbrAdcChans);
    }
    ADC_disable(); //can disable ADC until next time sampleTimer fires (to save power)?
    DMA0_disable();
    streamData = 1;
    return 1;
}

uint8_t Dma2ConversionDone(void)
{
    uint8_t bt_getmac;

    DMA2_disable();
    bt_getmac = BT_getGetMacAddress();

    if (!*btRxExp && (btIsConnected || bt_getmac))
    {
        if (getRcomm)
        {
            // 1 byte of RC command
            memcpy(rcommResp, btRxBuff, 1);
            memset(btRxBuff, 0, 14);
            //rcCenterR1 = 1;
            getRcomm = 0;
            return TaskSet(TASK_RCCENTERR1);        //1;
        }
        else if (bt_getmac)
        {
            // 14 bytes of BT mac address
            memcpy(mac, btRxBuff, 14);
            memset(btRxBuff, 0, 14);
            macAddrSet = 1;
            BT_setGetMacAddress(0);
            BT_setGoodCommand();
        }
        else if (btRxBuff[0] == ACK_COMMAND_PROCESSED)
        {
            // 10 bytes of RC info
            memcpy(rcommResp, btRxBuff, 10);
            memset(btRxBuff, 0, 14);
            //rcNodeR10 = 1;
            myLocalTimeLong = rwcTimeDiff64 + RTC_get64();
            return TaskSet(TASK_RCNODER10);
        }
    }
    else
    {
        if (!memcmp(btRxBuff, btRxExp, strlen((char*) btRxExp)))
        {
            memset(btRxBuff, 0, 14);
            BT_setGoodCommand();
        }
    }
    return 0;
}

uint16_t TaskCurrentGet()
{
    uint8_t i;
    uint16_t task;
    if (taskList)
    {
        for (i = 0; i < TASK_SIZE; i++)
        {
            task = 0x0001 << i;
            if (taskList & task)
                return task;
        }
    }
    return 0;
}
void TaskClear(TASK_FLAGS task_id)
{
    taskList &= ~task_id;
}
uint8_t TaskGet(TASK_FLAGS task_id)
{
    if (taskList & task_id)
        return 1;
    return 0;
}
uint8_t TaskSet(TASK_FLAGS task_id)
{
    uint8_t is_in_lpm = 0;
    if (!taskList)
        is_in_lpm = 1;
    taskList |= task_id;
    return is_in_lpm;
}

void PrepareSDBuffHead(void)
{
    memcpy(sdBuff, myTimeDiff, 9);
    sdBuffLen += 9;
    memset(myTimeDiff, 0xff, 9);
}

uint8_t Dma0BatteryRead(void)
{
    ADC_disable();
    DMA0_disable();
    return 1;
}

void ReadBatt(void)
{
    SetBattDma();
    __bis_SR_register(LPM3_bits + GIE); //ACLK remains active
    TaskSet(TASK_CFGCH);
}

void SetBattVal()
{
    uint16_t currentBattVal = *((uint16_t*) battVal);
    if (battStat & BATT_MID)
    {
        if (currentBattVal < 2568)
        {
            battStat = BATT_LOW;
        }
        else if (currentBattVal < 2767)
        {
            battStat = BATT_MID;
        }
        else
            battStat = BATT_HIGH;
    }
    else if (battStat & BATT_LOW)
    {
        if (currentBattVal < 2618)
        {
            battStat = BATT_LOW;
        }
        else if (currentBattVal < 2767)
        {
            battStat = BATT_MID;
        }
        else
            battStat = BATT_HIGH;
    }
    else
    {
        if (currentBattVal < 2568)
        {
            battStat = BATT_LOW;
        }
        else if (currentBattVal < 2717)
        {
            battStat = BATT_MID;
        }
        else
            battStat = BATT_HIGH;
    }
    battVal[2] = P2IN & 0xC0;

    // 10% Battery cutoff point - v0.17.3 onwards
    if ((storedConfig[NV_SD_TRIAL_CONFIG1] & SDH_BATT_CRITICAL_CUTOFF)
            && (currentBattVal < BATT_CUTOFF_3_65VOLTS))
    {
        if (battCriticalCount++ > 2)
        {
            battCritical = TRUE;
            if (sensing)
            {
                TaskSet(TASK_STOPSENSING);
            }
        }
    }
}

uint8_t UartCallback(uint8_t data)
{

    if (initializing)
    {
        return 0;
    }

    UartCbufPush(data);
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
                return TaskSet(TASK_UARTRSP);
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
                return TaskSet(TASK_UARTCMD);
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
            uint8_t uart_cmd_str[4];
            UartCbufPickData(uart_cmd_str, 0, 4);
            uartOldAction = 0;
            uartAction = 0;
            if (!memcmp(uart_cmd_str, "mac$", 4))
            {
                uartOldAction = UART_CMD_MAC;
                uartSendRspOldMac = 1;
                return TaskSet(TASK_UARTRSP);
            }
            else if (!memcmp(uart_cmd_str, "ver$", 4))
            {
                uartOldAction = UART_CMD_VER;
                uartSendRspOldVer = 1;
                return TaskSet(TASK_UARTRSP);
            }
            else if (!memcmp(uart_cmd_str, "bat$", 4))
            {
                uartOldAction = UART_CMD_BAT;
                uartSendRspOldBat = 1;
                return TaskSet(TASK_UARTRSP);
            }
            else if (!memcmp(uart_cmd_str, "mem$", 4))
            {
                uartOldAction = UART_CMD_MEM;
                uartSendRspOldMem = 1;
                return TaskSet(TASK_UARTRSP);
            }
            else if (!memcmp(uart_cmd_str, "rtc$", 4))
            {
                uartOldAction = UART_CMD_RTC;
                return TaskSet(TASK_UARTCMD);
            }
            else if (!memcmp(uart_cmd_str, "rct$", 4))
            {
                uartOldAction = UART_CMD_RCT;
                uartSendRspOldRct = 1;
                return TaskSet(TASK_UARTRSP);
            }
            else if (!memcmp(uart_cmd_str, "rdt$", 4))
            {
                uartOldAction = UART_CMD_RDT;
                uartSendRspOldRdt = 1;
                return TaskSet(TASK_UARTRSP);
            }

            else if (!memcmp(uart_cmd_str, "tim$", 4))
            {
                uartOldAction = UART_CMD_TIM;
                return TaskSet(TASK_UARTCMD);
            }
            else
            {
                uartArgSize = UART_RXBUF_START;
                uartRxBuf[UART_RXBUF_START] = '$';
                uartSteps = UART_STEP_WAIT4_CMD;
                return 0;
            }
        }
    }
    return 0;
}

void UartProcessCmd()
{
    uint16_t uart_rx_crc, uart_calc_crc;
    uint8_t uart_data_len;
    uint64_t rtc_ll = 0;
    if (uartOldAction)
    {
        switch (uartOldAction)
        {
        case UART_CMD_RTC:
            uart_data_len = 21;
            UartCbufPickData(uartRxBuf, 6, uart_data_len);
            UartCbufPickData((uint8_t*) (&uart_rx_crc), 4, 2);
            uart_calc_crc = CRC_data(uartRxBuf, uart_data_len);
            if (uart_rx_crc == uart_calc_crc)
            {
                memcpy(realTimeClockText64, uartRxBuf, UINT64_LEN - 1);
                realTimeClockText64[UINT64_LEN - 1] = 0;
                rwcConfigTime64 = Atol64(realTimeClockText64);
                rwcTimeDiff64 = rwcConfigTime64 - RTC_get64();
                ItoaWith0(rwcTimeDiff64, realTimeDiffText64, UINT64_LEN);
                uartSendRspOldAck = 1;
            }
            else
            {
                uartSendRspOldAck = 2;
            }
            TaskSet(TASK_UARTRSP);
            break;
        case UART_CMD_TIM:
            rtc_ll = RTC_get64();
            ItoaWith0(rtc_ll, realTimeClockText40, 14); //40 bits bin lead to at most 13 digits decimal
            uartSendRspOldTim = 1;
            TaskSet(TASK_UARTRSP);
            break;
        default:
            break;
        }
    }
    else if (uartAction)
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
                        if ((uartInfoMemLength <= 0x80)
                                && (uartInfoMemOffset <= 0x01ff)
                                && (uartInfoMemLength + uartInfoMemOffset
                                        <= 0x0200))
                        {
                            //uartInfoMemOffset -= 0x1800;
                            uartSendRspGim = 1;
                        }
                        else
                            uartSendRspBadArg = 1;
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
                        { //(uartDcMemOffset>=16) &&
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
                            memcpy((uint8_t*) (&rwcConfigTime64),
                                   uartRxBuf + UART_RXBUF_DATA, 8); // 64bits = 8bytes
                            rwcTimeDiff64 = rwcConfigTime64 - RTC_get64(); // this is the offset to be stored int the sd header
                            RwcCheck();
                            uartSendRspAck = 1;
                        }
                        else
                            uartSendRspBadArg = 1;
                        break;
                    case UART_PROP_INFOMEM:
                        uartInfoMemLength = uartRxBuf[UART_RXBUF_DATA];
                        uartInfoMemOffset = (uint16_t) uartRxBuf[UART_RXBUF_DATA
                                + 1]
                                + (((uint16_t) uartRxBuf[UART_RXBUF_DATA + 2])
                                        << 8);
                        if ((uartInfoMemLength <= 0x80)
                                && (uartInfoMemOffset <= 0x01ff)
                                && (uartInfoMemLength + uartInfoMemOffset
                                        <= 0x0200))
                        {
                            if (uartInfoMemOffset
                                    == (INFOMEM_SEG_C_ADDR - INFOMEM_OFFSET))
                            {
                                /* Read MAC address so it is not forgotten */
                                InfoMem_read((uint8_t *) NV_MAC_ADDRESS,
                                             macAddr, 6);
                            }
                            if (uartInfoMemOffset
                                    == (INFOMEM_SEG_D_ADDR - INFOMEM_OFFSET))
                            {
                                /* Check if unit is SR47-4 or greater.
                                 * If so, amend configuration byte 2 of ADS chip 1 to have bit 3 set to 1.
                                 * This ensures clock lines on ADS chip are correct
                                 */
                                if ((daughtCardId[DAUGHT_CARD_ID] == 47)
                                        && (daughtCardId[DAUGHT_CARD_REV] >= 4))
                                {
                                    *(uartRxBuf + UART_RXBUF_DATA + 3
                                            + NV_EXG_ADS1292R_1_CONFIG2) |= 8;
                                }
                            }
                            /* Write received UART bytes to infomem */
                            InfoMem_write((uint8_t*) uartInfoMemOffset,
                                          uartRxBuf + UART_RXBUF_DATA + 3,
                                          uartInfoMemLength);
                            if (uartInfoMemOffset
                                    == (INFOMEM_SEG_C_ADDR - INFOMEM_OFFSET))
                            {
                                /* Re-write MAC address to Infomem */
                                InfoMem_write((uint8_t*) NV_MAC_ADDRESS,
                                              macAddr, 6);
                            }
                            /* Reload latest infomem bytes to RAM */
                            InfoMem_read((uint8_t *) uartInfoMemOffset,
                                         storedConfig + uartInfoMemOffset,
                                         uartInfoMemLength);

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
                { // set daughter card
                    switch (uartRxBuf[UART_RXBUF_PROP])
                    {
                    case UART_PROP_CARD_MEM:
                        uartDcMemLength = uartRxBuf[UART_RXBUF_DATA];
                        uartDcMemOffset = (uint16_t) uartRxBuf[UART_RXBUF_DATA
                                + 1]
                                + (((uint16_t) uartRxBuf[UART_RXBUF_DATA + 2])
                                        << 8);
                        if ((uartDcMemLength <= 128)
                                && (uartDcMemOffset <= 2031)
                                && (uartDcMemOffset >= 16)
                                && ((uint16_t) uartDcMemLength + uartDcMemOffset
                                        <= 2032))
                        {
                            CAT24C16_init();
                            CAT24C16_write(uartDcMemOffset + 16,
                                           (uint16_t) uartDcMemLength,
                                           uartRxBuf + UART_RXBUF_DATA + 3);
                            CAT24C16_powerOff();
                            uartSendRspAck = 1;
                        }
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
        }
        else
            uartSendRspBadCrc = 1;
        TaskSet(TASK_UARTRSP);
    }
}

void UartSendRsp()
{
    uint8_t uart_resp_len = 0, cr = 0;
    uint16_t uartRespCrc;

    if (uartSendRspOldAck)
    {
        if (uartSendRspOldAck == 1)
            memcpy(uartRespBuf, "ack!", 4);
        else
            // if(uartSendRspOldRtc==2)
            memcpy(uartRespBuf, "nac!", 4);
        uart_resp_len += 4;
        uartSendRspOldAck = 0;
        cr = 1;
    }
    else if (uartSendRspAck)
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
    else if (uartSendRspOldMac)
    {
        uartSendRspOldMac = 0;
        memcpy(uartRespBuf, mac, 12);
        uart_resp_len += 12;
        cr = 1;
    }
    else if (uartSendRspOldVer)
    {
        uartSendRspOldVer = 0;
        *(uartRespBuf + uart_resp_len++) = DEVICE_VER;
        *(uartRespBuf + uart_resp_len++) = (FW_IDENTIFIER & 0xFF);
        *(uartRespBuf + uart_resp_len++) = ((FW_IDENTIFIER & 0xFF00) >> 8);
        *(uartRespBuf + uart_resp_len++) = (FW_VER_MAJOR & 0xFF);
        *(uartRespBuf + uart_resp_len++) = ((FW_VER_MAJOR & 0xFF00) >> 8);
        *(uartRespBuf + uart_resp_len++) = (FW_VER_MINOR);
        *(uartRespBuf + uart_resp_len++) = (FW_VER_REL);
        cr = 1;
    }
    else if (uartSendRspOldBat)
    {
        uartSendRspOldBat = 0;
        memcpy(uartRespBuf, battVal, 3);
        uart_resp_len += 3;
        cr = 1;
    }
    else if (uartSendRspOldMem)
    {
        uartSendRspOldMem = 0;
        memcpy(uartRespBuf, storedConfig, 128);
        uart_resp_len += 128;
        cr = 1;
    }
    else if (uartSendRspOldRct)
    {
        uartSendRspOldRct = 0;
        memcpy(uartRespBuf, realTimeClockText64, UINT64_LEN);
        uart_resp_len += UINT64_LEN;
        cr = 1;
    }
    else if (uartSendRspOldRdt)
    {
        uartSendRspOldRdt = 0;
        memcpy(uartRespBuf, realTimeDiffText64, UINT64_LEN);
        uart_resp_len += UINT64_LEN;
        cr = 1;
    }
    else if (uartSendRspOldTim)
    {
        uartSendRspOldTim = 0;
        memcpy(uartRespBuf, realTimeClockText40, 14);
        uart_resp_len += 14;
        cr = 1;
    }
    else if (uartSendRspMac)
    {
        uartSendRspMac = 0;
        *(uartRespBuf + uart_resp_len++) = '$';
        *(uartRespBuf + uart_resp_len++) = UART_RESPONSE;
        *(uartRespBuf + uart_resp_len++) = 8;
        *(uartRespBuf + uart_resp_len++) = UART_COMP_SHIMMER;
        *(uartRespBuf + uart_resp_len++) = UART_PROP_MAC;
        memcpy(uartRespBuf + uart_resp_len, macAddr, 6);
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
        *(uartRespBuf + uart_resp_len++) = (FW_VER_REL);
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
        memcpy(uartRespBuf + uart_resp_len, (uint8_t*) (&rwcConfigTime64), 8);
        uart_resp_len += 8;
    }
    else if (uartSendRspCurrentTime)
    {
        uartSendRspCurrentTime = 0;
        uint64_t rwc_curr_time_64;
        *(uartRespBuf + uart_resp_len++) = '$';
        rwc_curr_time_64 = rwcTimeDiff64 + RTC_get64();
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
          //  CAT24C16_init();
         //   CAT24C16_read(uartDcMemOffset, (uint16_t) uartDcMemLength,
          //                (uartRespBuf + uart_resp_len));
          //  CAT24C16_powerOff();
            memcpy(uartRespBuf + uart_resp_len, daughtCardId + uartDcMemOffset,
                              uartDcMemLength);
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
            CAT24C16_init();
            CAT24C16_read(uartDcMemOffset + 16, (uint16_t) uartDcMemLength,
                          (uartRespBuf + uart_resp_len));
            CAT24C16_powerOff();
        }
        uart_resp_len += uartDcMemLength;
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

// =================================== circular buffer start ==================================
void UartCbufPush(uint8_t elem)
{
    ucBuf.entry[ucBuf.idx++] = elem;
    if (ucBuf.idx >= CBUF_SIZE)
        ucBuf.idx = 0;
}

void UartCbufPickData(uint8_t * dst_buf, uint8_t offset, uint8_t len)
{
    if ((!len) || (len > CBUF_PARAM_LEN_MAX) || (offset > CBUF_PARAM_LEN_MAX)
            || ((len + offset) > CBUF_SIZE))
        return;
    if (ucBuf.idx >= offset + len)
    {
        memcpy(dst_buf, &ucBuf.entry[ucBuf.idx - len - offset], len);
    }
    else if (ucBuf.idx <= offset)
    {
        memcpy(dst_buf, &ucBuf.entry[CBUF_SIZE + ucBuf.idx - len - offset],
               len);
    }
    else
    { // len+offset > ucBuf.idx > offset
        memcpy(dst_buf, &ucBuf.entry[CBUF_SIZE + ucBuf.idx - len - offset],
               len + offset - ucBuf.idx);
        memcpy(dst_buf + len + offset - ucBuf.idx, &ucBuf.entry[0],
               ucBuf.idx - offset);
    }
}
// =================================== circular buffer end ==================================
uint8_t UartCheckCrc(uint8_t len)
{
    if (len > UART_DATA_LEN_MAX)
        return 0;
    uint16_t uart_rx_crc, uart_calc_crc;
    uart_calc_crc = CRC_data(uartRxBuf, len);
    uart_rx_crc = (uint16_t) uartRxBuf[len];
    uart_rx_crc += ((uint16_t) uartRxBuf[len + 1]) << 8;

    return (uart_rx_crc == uart_calc_crc);
}

uint8_t GetSdCfgFlag()
{
    uint8_t sd_config_delay_flag = 0;
    InfoMem_read((uint8_t*) NV_SD_CONFIG_DELAY_FLAG, &sd_config_delay_flag, 1);
    if (!(sd_config_delay_flag & BIT7))
    {
        if (sd_config_delay_flag & BIT0)
            return 1;
    }
    return 0;
}

void SetSdCfgFlag(uint8_t flag)
{
    uint8_t sd_config_delay_flag;
    if (flag)
    {
        InfoMem_read((uint8_t*) NV_SD_CONFIG_DELAY_FLAG, &sd_config_delay_flag,
                     1);
        if (!(sd_config_delay_flag & BIT7))
        {
            sd_config_delay_flag |= BIT0;
        }
        else
        {
            sd_config_delay_flag = BIT0;
        }
        InfoMem_write((uint8_t*) NV_SD_CONFIG_DELAY_FLAG, &sd_config_delay_flag,
                      1);
    }
    else
    {
        InfoMem_read((uint8_t*) NV_SD_CONFIG_DELAY_FLAG, &sd_config_delay_flag,
                     1);
        sd_config_delay_flag &= ~BIT0;
        InfoMem_write((uint8_t*) NV_SD_CONFIG_DELAY_FLAG, &sd_config_delay_flag,
                      1);
    }
}

void ShimmerCalibInitFromInfoAll(void)
{
    ShimmerCalibFromInfo(SC_SENSOR_ANALOG_ACCEL, 0);
    ShimmerCalibFromInfo(SC_SENSOR_MPU9150_GYRO, 0);
    ShimmerCalibFromInfo(SC_SENSOR_LSM303DLHC_ACCEL, 0);
    ShimmerCalibFromInfo(SC_SENSOR_LSM303DLHC_MAG, 0);
}
void ShimmerCalibUpdateFromInfoAll(void)
{
    ShimmerCalibFromInfo(SC_SENSOR_ANALOG_ACCEL, 1);
    ShimmerCalibFromInfo(SC_SENSOR_MPU9150_GYRO, 1);
    ShimmerCalibFromInfo(SC_SENSOR_LSM303DLHC_ACCEL, 1);
    ShimmerCalibFromInfo(SC_SENSOR_LSM303DLHC_MAG, 1);
}

void ShimmerCalibFromInfo(uint8_t sensor, uint8_t use_sys_time)
{
    uint8_t info_config, info_valid = 0;
    uint16_t offset;
    int byte_cnt = 0;
    sc_t sc1;

    sc1.id = sensor;
    if (use_sys_time)
    {
        memset(sc1.ts, 0, 8);
    }
    else
    {
        *(uint64_t*) (sc1.ts) = rwcTimeDiff64 + RTC_get64();
    }

    if (sc1.id == SC_SENSOR_ANALOG_ACCEL)
    {
        offset = NV_A_ACCEL_CALIBRATION;
        sc1.range = SC_SENSOR_RANGE_ANALOG_ACCEL;
        sc1.data_len = SC_DATA_LEN_ANALOG_ACCEL;
    }
    else if (sc1.id == SC_SENSOR_MPU9150_GYRO)
    {
        offset = NV_MPU9150_GYRO_CALIBRATION;
        sc1.data_len = SC_DATA_LEN_MPU9250_GYRO;
        InfoMem_read((uint8_t *) NV_CONFIG_SETUP_BYTE2, &info_config, 1);
        sc1.range = info_config & 0x03;
    }
    else if (sc1.id == SC_SENSOR_LSM303DLHC_ACCEL)
    {
        offset = NV_LSM303DLHC_ACCEL_CALIBRATION;
        sc1.data_len = SC_DATA_LEN_LSM303DLHC_ACCEL;
        InfoMem_read((uint8_t *) NV_CONFIG_SETUP_BYTE0, &info_config, 1);
        sc1.range = (info_config & 0x0c) >> 2;
    }
    else if (sc1.id == SC_SENSOR_LSM303DLHC_MAG)
    {
        offset = NV_LSM303DLHC_MAG_CALIBRATION;
        sc1.data_len = SC_DATA_LEN_LSM303DLHC_MAG;
        InfoMem_read((uint8_t *) NV_CONFIG_SETUP_BYTE2, &info_config, 1);
        if (lsmInUse == LSM303DLHC_IN_USE)
        {
            sc1.range = (info_config & 0xe0) >> 5;
        }
        else
        {
            sc1.range = 0;
        }
    }
    else
    {
        return;
    }

    InfoMem_read((uint8_t *) offset, storedConfig + offset, 21);
    for (byte_cnt = 0; byte_cnt < 21; byte_cnt++)
    {
        if (storedConfig[offset + byte_cnt] != 0xff)
        {
            info_valid = 1;
            break;
        }
    }

    if (info_valid)
    { // if not all 0xff in infomem
        memcpy(sc1.data.raw, storedConfig + offset, sc1.data_len);
        ShimmerCalib_singleSensorWrite(&sc1);
    }
}
void ShimmerCalibSyncFromDumpRamAll(void)
{
    ShimmerCalibSyncFromDumpRamSingleSensor(SC_SENSOR_ANALOG_ACCEL);
    ShimmerCalibSyncFromDumpRamSingleSensor(SC_SENSOR_MPU9150_GYRO);
    ShimmerCalibSyncFromDumpRamSingleSensor(SC_SENSOR_LSM303DLHC_MAG);
    ShimmerCalibSyncFromDumpRamSingleSensor(SC_SENSOR_LSM303DLHC_ACCEL);
}
void ShimmerCalibSyncFromDumpRamSingleSensor(uint8_t sensor)
{
    sc_t sc1;
    uint16_t scs_infomem_offset, scs_sdhead_offset, scs_sdhead_ts;
    sc1.id = sensor;
    sc1.data_len = ShimmerCalib_findLength(&sc1);
    switch (sensor)
    {
    case SC_SENSOR_ANALOG_ACCEL:
        scs_infomem_offset = NV_A_ACCEL_CALIBRATION;
        scs_sdhead_offset = SDH_A_ACCEL_CALIBRATION;
        scs_sdhead_ts = SDH_A_ACCEL_CALIB_TS;
        sc1.range = SC_SENSOR_RANGE_ANALOG_ACCEL;
        break;
    case SC_SENSOR_MPU9150_GYRO:
        scs_infomem_offset = NV_MPU9150_GYRO_CALIBRATION;
        scs_sdhead_offset = SDH_MPU9150_GYRO_CALIBRATION;
        scs_sdhead_ts = SDH_MPU9150_GYRO_CALIB_TS;
        sc1.range = storedConfig[NV_CONFIG_SETUP_BYTE2] & 0x03;
        break;
    case SC_SENSOR_LSM303DLHC_MAG:
        scs_infomem_offset = NV_LSM303DLHC_MAG_CALIBRATION;
        scs_sdhead_offset = SDH_LSM303DLHC_MAG_CALIBRATION;
        scs_sdhead_ts = SDH_LSM303DLHC_MAG_CALIB_TS;
        if (lsmInUse == LSM303DLHC_IN_USE)
        {
            sc1.range = (storedConfig[NV_CONFIG_SETUP_BYTE2] >> 5) & 0x07;
        }
        else
        {
            sc1.range = 0;
        }
        break;
    case SC_SENSOR_LSM303DLHC_ACCEL:
        scs_infomem_offset = NV_LSM303DLHC_ACCEL_CALIBRATION;
        scs_sdhead_offset = SDH_LSM303DLHC_ACCEL_CALIBRATION;
        scs_sdhead_ts = SDH_LSM303DLHC_ACCEL_CALIB_TS;
        sc1.range = (storedConfig[NV_CONFIG_SETUP_BYTE0] >> 2) & 0x03;
        break;
    default:
        scs_infomem_offset = NV_A_ACCEL_CALIBRATION;
        scs_sdhead_offset = SDH_A_ACCEL_CALIBRATION;
        scs_sdhead_ts = SDH_A_ACCEL_CALIB_TS;
        sc1.range = SC_SENSOR_RANGE_ANALOG_ACCEL;
        break;
    }
    ShimmerCalib_singleSensorRead(&sc1);
    memcpy(storedConfig + scs_infomem_offset, sc1.data.raw, sc1.data_len);
    InfoMem_write((uint8_t*) scs_infomem_offset, sc1.data.raw, sc1.data_len);
    memcpy(sdHeadText + scs_sdhead_offset, sc1.data.raw, sc1.data_len);
    memcpy(sdHeadText + scs_sdhead_ts, sc1.ts, 8);
}

void UpdateSdConfig()
{
    char buffer[66];
    FIL cfg_fil;
    if (!docked && CheckSdInslot())
    {
        uint8_t sd_power_state;
        if (!(P4OUT & BIT2))
        {
            SdPowerOn();
            sd_power_state = 0;
        }
        else
        {
            sd_power_state = 1;
        }

        float val_num;
        uint16_t val_int, val_f, temp16;
        uint32_t temp32;
        uint64_t temp64;
        uint8_t node_addr[6], center_addr[6], byte_l, byte_h;
        uint8_t i;
        char val_char[20];
        UINT bw;

        syncNodeNum = 0;
        nodeSuccFull = 0;
        memset(shimmerName, 0, MAX_CHARS);
        memset(expIdName, 0, MAX_CHARS);
        memset(centerName, 0, MAX_CHARS);
        memset(configTimeText, 0, MAX_CHARS);
        newDirFlag = 1;

        if (storedConfig[NV_SENSORS0] & SENSOR_GSR) // they are sharing adc1, ban intch1 when gsr is on
            storedConfig[NV_SENSORS1] &= ~SENSOR_INT_A1;
        if (storedConfig[NV_SENSORS0] & SENSOR_EXG1_24BIT)
            storedConfig[NV_SENSORS2] &= ~SENSOR_EXG1_16BIT;
        if (storedConfig[NV_SENSORS0] & SENSOR_EXG2_24BIT)
            storedConfig[NV_SENSORS2] &= ~SENSOR_EXG2_16BIT;
        if (storedConfig[NV_SENSORS0] & SENSOR_EXG1_24BIT
                || storedConfig[NV_SENSORS0] & SENSOR_EXG2_24BIT
                || storedConfig[NV_SENSORS2] & SENSOR_EXG1_16BIT
                || storedConfig[NV_SENSORS2] & SENSOR_EXG2_16BIT)
        {
            storedConfig[NV_CONFIG_SETUP_BYTE3] |= EXP_POWER_ENABLE;
            storedConfig[NV_SENSORS1] &= ~SENSOR_INT_A1;
            storedConfig[NV_SENSORS2] &= ~SENSOR_INT_A14;
        }
        if (storedConfig[NV_SENSORS1] & SENSOR_STRAIN)
        { // they are sharing adc13 and adc14
            storedConfig[NV_SENSORS1] &= ~SENSOR_INT_A13;
            storedConfig[NV_SENSORS2] &= ~SENSOR_INT_A14;
        }
        if (((storedConfig[NV_CONFIG_SETUP_BYTE3] >> 1) & 0x07) > 4)
        { // never larger than 4
            storedConfig[NV_CONFIG_SETUP_BYTE3] &= 0xf1;           //clearBIT3-1
            storedConfig[NV_CONFIG_SETUP_BYTE3] |= (4 & 0x07) << 1;   //BIT3-1
        }
        if (storedConfig[NV_SD_TRIAL_CONFIG1] & SDH_SINGLETOUCH)
        {
            storedConfig[NV_SD_TRIAL_CONFIG0] |= SDH_USER_BUTTON_ENABLE;
            storedConfig[NV_SD_TRIAL_CONFIG0] |= SDH_TIME_SYNC;
        }

        SamplingClkAssignment();
        ClkAssignment();
        TB0CTL = MC_0;
        TB0Start();

        char cfgname[] = "sdlog.cfg";

        if (memcmp(all0xff, storedConfig, 6))
        {
            fileBad = f_open(&cfg_fil, cfgname, FA_WRITE | FA_CREATE_ALWAYS);
            //sensor0
            sprintf(buffer, "accel=%d\r\n",
                    storedConfig[NV_SENSORS0] & SENSOR_A_ACCEL ? 1 : 0);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "gyro=%d\r\n",
                    storedConfig[NV_SENSORS0] & SENSOR_MPU9150_GYRO ? 1 : 0);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "mag=%d\r\n",
                    storedConfig[NV_SENSORS0] & SENSOR_LSM303DLHC_MAG ? 1 : 0);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "exg1_24bit=%d\r\n",
                    storedConfig[NV_SENSORS0] & SENSOR_EXG1_24BIT ? 1 : 0);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "exg2_24bit=%d\r\n",
                    storedConfig[NV_SENSORS0] & SENSOR_EXG2_24BIT ? 1 : 0);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "gsr=%d\r\n",
                    storedConfig[NV_SENSORS0] & SENSOR_GSR ? 1 : 0);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "extch7=%d\r\n",
                    storedConfig[NV_SENSORS0] & SENSOR_EXT_A7 ? 1 : 0);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "extch6=%d\r\n",
                    storedConfig[NV_SENSORS0] & SENSOR_EXT_A6 ? 1 : 0);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            //sensor1
            sprintf(buffer, "br_amp=%d\r\n",
                    storedConfig[NV_SENSORS1] & SENSOR_STRAIN ? 1 : 0);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "vbat=%d\r\n",
                    storedConfig[NV_SENSORS1] & SENSOR_VBATT ? 1 : 0);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer,
                    "accel_d=%d\r\n",
                    storedConfig[NV_SENSORS1] & SENSOR_LSM303DLHC_ACCEL ?
                            1 : 0);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "extch15=%d\r\n",
                    storedConfig[NV_SENSORS1] & SENSOR_EXT_A15 ? 1 : 0);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "intch1=%d\r\n",
                    storedConfig[NV_SENSORS1] & SENSOR_INT_A1 ? 1 : 0);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "intch12=%d\r\n",
                    storedConfig[NV_SENSORS1] & SENSOR_INT_A12 ? 1 : 0);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "intch13=%d\r\n",
                    storedConfig[NV_SENSORS1] & SENSOR_INT_A13 ? 1 : 0);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            //sensor2
            sprintf(buffer, "intch14=%d\r\n",
                    storedConfig[NV_SENSORS2] & SENSOR_INT_A14 ? 1 : 0);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "accel_mpu=%d\r\n",
                    storedConfig[NV_SENSORS2] & SENSOR_MPU9150_ACCEL ? 1 : 0);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "mag_mpu=%d\r\n",
                    storedConfig[NV_SENSORS2] & SENSOR_MPU9150_MAG ? 1 : 0);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "exg1_16bit=%d\r\n",
                    storedConfig[NV_SENSORS2] & SENSOR_EXG1_16BIT ? 1 : 0);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "exg2_16bit=%d\r\n",
                    storedConfig[NV_SENSORS2] & SENSOR_EXG2_16BIT ? 1 : 0);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, ((bmpInUse == BMP180_IN_USE) ? ("pres_bmp180=%d\r\n") : ("pres_bmp280=%d\r\n")),
                    storedConfig[NV_SENSORS2] & SENSOR_BMPX80_PRESSURE ? 1 : 0);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            // sample_rate
            val_num = clockFreq
                    / (*(uint16_t*) (storedConfig + NV_SAMPLING_RATE));
            val_int = (uint16_t) floor(val_num);
            val_f = (uint16_t) round((val_num - floor(val_num)) * 100);
            if (val_f == 100)
            {
                val_f = 0;
                val_int++;
            }
            if (val_f)
            {
                sprintf(val_char, "%d.%d", val_int, val_f);
            }
            else
            {
                sprintf(val_char, "%d", val_int);
            }
            sprintf(buffer, "sample_rate=%s\r\n", val_char);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            // setup config
            sprintf(buffer, "mg_internal_rate=%d\r\n",
                    (storedConfig[NV_CONFIG_SETUP_BYTE2] >> 2) & 0x07);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "mg_range=%d\r\n",
                    (storedConfig[NV_CONFIG_SETUP_BYTE2] >> 5) & 0x07);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "acc_internal_rate=%d\r\n",
                    (storedConfig[NV_CONFIG_SETUP_BYTE0] >> 4) & 0x0f);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "accel_mpu_range=%d\r\n",
                    (storedConfig[NV_CONFIG_SETUP_BYTE3] >> 6) & 0x03);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer,
                    ((bmpInUse == BMP180_IN_USE) ?
                            ("pres_bmp180_prec=%d\r\n") :
                            ("pres_bmp280_prec=%d\r\n")),
                    (storedConfig[NV_CONFIG_SETUP_BYTE3] >> 4) & 0x03);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "gsr_range=%d\r\n",
                    (storedConfig[NV_CONFIG_SETUP_BYTE3] >> 1) & 0x07);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer,
                    "exp_power=%d\r\n",
                    storedConfig[NV_CONFIG_SETUP_BYTE3] & EXP_POWER_ENABLE ?
                            1 : 0);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "gyro_range=%d\r\n",
                    (storedConfig[NV_CONFIG_SETUP_BYTE2]) & 0x03);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "gyro_samplingrate=%d\r\n",
                    storedConfig[NV_CONFIG_SETUP_BYTE1]);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "acc_range=%d\r\n",
                    (storedConfig[NV_CONFIG_SETUP_BYTE0] >> 2) & 0x03);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "acc_lpm=%d\r\n",
                    (storedConfig[NV_CONFIG_SETUP_BYTE0] >> 1) & 0x01);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "acc_hrm=%d\r\n",
                    (storedConfig[NV_CONFIG_SETUP_BYTE0]) & 0x01);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            // trial config
            sprintf(buffer,
                    "rtc_error_enable=%d\r\n",
                    storedConfig[NV_SD_TRIAL_CONFIG0] & SDH_RWCERROR_EN ?
                            1 : 0);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer,
                    "user_button_enable=%d\r\n",
                    storedConfig[NV_SD_TRIAL_CONFIG0] & SDH_USER_BUTTON_ENABLE ?
                            1 : 0);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "iammaster=%d\r\n",
                    storedConfig[NV_SD_TRIAL_CONFIG0] & SDH_IAMMASTER ? 1 : 0);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "sync=%d\r\n",
                    storedConfig[NV_SD_TRIAL_CONFIG0] & SDH_TIME_SYNC ? 1 : 0);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer,
                    "singletouch=%d\r\n",
                    storedConfig[NV_SD_TRIAL_CONFIG1] & SDH_SINGLETOUCH ?
                            1 : 0);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer,
                    "low_battery_autostop=%d\r\n",
                    storedConfig[NV_SD_TRIAL_CONFIG1] & SDH_BATT_CRITICAL_CUTOFF ?
                            1 : 0);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "tcxo=%d\r\n",
                    storedConfig[NV_SD_TRIAL_CONFIG1] & SDH_TCXO ? 1 : 0);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "interval=%d\r\n", storedConfig[NV_SD_BT_INTERVAL]);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            temp32 = (uint32_t) storedConfig[NV_EST_EXP_LEN_LSB];
            temp32 += ((uint32_t) storedConfig[NV_EST_EXP_LEN_MSB]) << 8;
            temp16 = (uint16_t) (temp32 & 0xffff);
            if (temp32 < 10)
            {
                shortExpFlag = 1;
                temp32 = 0xffff;
            }
            else
            {
                shortExpFlag = 0;
                if (temp32 > 180)
                {
                    temp32 = 180;
                }
            }
            estLen = temp32 * 60;
            estLen3 = estLen / 3;
            sprintf(buffer, "est_exp_len=%d\r\n", (uint16_t) temp16);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            temp32 = (uint32_t) storedConfig[NV_MAX_EXP_LEN_LSB];
            temp32 += ((uint32_t) storedConfig[NV_MAX_EXP_LEN_MSB]) << 8;
            maxLen = temp32 * 60;
            temp16 = (uint16_t) (temp32 & 0xffff);
            sprintf(buffer, "max_exp_len=%d\r\n", (uint32_t) temp32);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            while (memcmp(all0xff, storedConfig + NV_NODE0 + syncNodeNum * 6, 6)
                    && (syncNodeNum < MAX_NODES))
            {
                memcpy(node_addr, storedConfig + NV_NODE0 + syncNodeNum * 6, 6);
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
                sprintf(buffer, "node=%s\r\n", (char*) nodeName[syncNodeNum]);
                f_write(&cfg_fil, buffer, strlen(buffer), &bw);
                syncNodeNum++;
            }

            if (memcmp(all0xff, storedConfig + NV_CENTER, 6))
            {
                memcpy(center_addr, storedConfig + NV_CENTER, 6);
                for (i = 0; i < 6; i++)
                {
                    byte_h = (center_addr[i] >> 4) & 0x0f;
                    byte_l = center_addr[i] & 0x0f;
                    centerName[i * 2] = byte_h + (byte_h > 9 ? 'A' - 10 : '0');
                    centerName[i * 2 + 1] = byte_l
                            + (byte_l > 9 ? 'A' - 10 : '0');
                }
                *(centerName + 12) = 0;
                sprintf(buffer, "center=%s\r\n", (char*) centerName);
                f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            }
            sprintf(buffer, "myid=%d\r\n", storedConfig[NV_SD_MYTRIAL_ID]);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "Nshimmer=%d\r\n", storedConfig[NV_SD_NSHIMMER]);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            for (i = 0;
                    (i < MAX_CHARS - 1)
                            && isprint(storedConfig[NV_SD_SHIMMER_NAME + i]);
                    i++)
                ;
            if (i)
            {
                memcpy((char*) shimmerName,
                       (char*) (storedConfig + NV_SD_SHIMMER_NAME), i);
                shimmerName[i] = 0;
                sprintf(buffer, "shimmername=%s\r\n", shimmerName);
                f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            }
            for (i = 0;
                    (i < MAX_CHARS - 1)
                            && isprint(storedConfig[NV_SD_EXP_ID_NAME + i]);
                    i++)
                ;
            if (i)
            {
                memcpy((char*) expIdName,
                       (char*) (storedConfig + NV_SD_EXP_ID_NAME), i);
                expIdName[i] = 0;
                sprintf(buffer, "experimentid=%s\r\n", expIdName);
                f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            }
            temp32 = 0;
            for (i = 0; i < 4; i++)
            {
                temp32 <<= 8;
                temp32 += storedConfig[NV_SD_CONFIG_TIME + i];
            }
            ItoaWith0((uint64_t) temp32, configTimeText, UINT32_LEN);
            sprintf(buffer, "configtime=%s\r\n", configTimeText);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "baud_rate=%d\r\n",
                    storedConfig[NV_BT_COMMS_BAUD_RATE]);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
//         temp32 = storedConfig[NV_DERIVED_CHANNELS_0]
//                + (((uint32_t)storedConfig[NV_DERIVED_CHANNELS_1])<<8)
//                + (((uint32_t)storedConfig[NV_DERIVED_CHANNELS_2])<<16);
//         ItoaNo0((uint64_t)temp32, (uint8_t*)val_char, 9);
            temp64 = storedConfig[NV_DERIVED_CHANNELS_0]
                    + (((uint64_t) storedConfig[NV_DERIVED_CHANNELS_1]) << 8)
                    + (((uint64_t) storedConfig[NV_DERIVED_CHANNELS_2]) << 16)
                    + (((uint64_t) storedConfig[NV_DERIVED_CHANNELS_3]) << 24)
                    + (((uint64_t) storedConfig[NV_DERIVED_CHANNELS_4]) << 32)
                    + (((uint64_t) storedConfig[NV_DERIVED_CHANNELS_5]) << 40)
                    + (((uint64_t) storedConfig[NV_DERIVED_CHANNELS_6]) << 48)
                    + (((uint64_t) storedConfig[NV_DERIVED_CHANNELS_7]) << 56);
            ItoaNo0(temp64, (uint8_t*) val_char, 21);
            sprintf(buffer, "derived_channels=%s\r\n", val_char);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "EXG_ADS1292R_1_CONFIG1=%d\r\n",
                    storedConfig[NV_EXG_ADS1292R_1_CONFIG1]);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "EXG_ADS1292R_1_CONFIG2=%d\r\n",
                    storedConfig[NV_EXG_ADS1292R_1_CONFIG2]);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "EXG_ADS1292R_1_LOFF=%d\r\n",
                    storedConfig[NV_EXG_ADS1292R_1_LOFF]);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "EXG_ADS1292R_1_CH1SET=%d\r\n",
                    storedConfig[NV_EXG_ADS1292R_1_CH1SET]);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "EXG_ADS1292R_1_CH2SET=%d\r\n",
                    storedConfig[NV_EXG_ADS1292R_1_CH2SET]);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "EXG_ADS1292R_1_RLD_SENS=%d\r\n",
                    storedConfig[NV_EXG_ADS1292R_1_RLD_SENS]);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "EXG_ADS1292R_1_LOFF_SENS=%d\r\n",
                    storedConfig[NV_EXG_ADS1292R_1_LOFF_SENS]);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "EXG_ADS1292R_1_LOFF_STAT=%d\r\n",
                    storedConfig[NV_EXG_ADS1292R_1_LOFF_STAT]);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "EXG_ADS1292R_1_RESP1=%d\r\n",
                    storedConfig[NV_EXG_ADS1292R_1_RESP1]);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "EXG_ADS1292R_1_RESP2=%d\r\n",
                    storedConfig[NV_EXG_ADS1292R_1_RESP2]);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "EXG_ADS1292R_2_CONFIG1=%d\r\n",
                    storedConfig[NV_EXG_ADS1292R_2_CONFIG1]);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "EXG_ADS1292R_2_CONFIG2=%d\r\n",
                    storedConfig[NV_EXG_ADS1292R_2_CONFIG2]);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "EXG_ADS1292R_2_LOFF=%d\r\n",
                    storedConfig[NV_EXG_ADS1292R_2_LOFF]);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "EXG_ADS1292R_2_CH1SET=%d\r\n",
                    storedConfig[NV_EXG_ADS1292R_2_CH1SET]);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "EXG_ADS1292R_2_CH2SET=%d\r\n",
                    storedConfig[NV_EXG_ADS1292R_2_CH2SET]);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "EXG_ADS1292R_2_RLD_SENS=%d\r\n",
                    storedConfig[NV_EXG_ADS1292R_2_RLD_SENS]);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "EXG_ADS1292R_2_LOFF_SENS=%d\r\n",
                    storedConfig[NV_EXG_ADS1292R_2_LOFF_SENS]);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "EXG_ADS1292R_2_LOFF_STAT=%d\r\n",
                    storedConfig[NV_EXG_ADS1292R_2_LOFF_STAT]);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "EXG_ADS1292R_2_RESP1=%d\r\n",
                    storedConfig[NV_EXG_ADS1292R_2_RESP1]);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            sprintf(buffer, "EXG_ADS1292R_2_RESP2=%d\r\n",
                    storedConfig[NV_EXG_ADS1292R_2_RESP2]);
            f_write(&cfg_fil, buffer, strlen(buffer), &bw);
            if (memcmp(all0xff, storedConfig + NV_SENSORS3, 5))
            {
                // DMP related - start
                sprintf(buffer,
                        "DMP=%d\r\n",
                        storedConfig[NV_CONFIG_SETUP_BYTE4] & MPU9150_MPL_DMP ?
                                1 : 0);
                f_write(&cfg_fil, buffer, strlen(buffer), &bw);
                sprintf(buffer,
                        "MPL_use_LSM_mag=%d\r\n",
                        storedConfig[NV_CONFIG_SETUP_BYTE4]
                                & MPU9150_MPL_USE_LSM303_MAG ? 1 : 0);
                f_write(&cfg_fil, buffer, strlen(buffer), &bw);
                sprintf(buffer, "MPL_LPF=%d\r\n",
                        (storedConfig[NV_CONFIG_SETUP_BYTE4] >> 3) & 0x07);
                f_write(&cfg_fil, buffer, strlen(buffer), &bw);
                sprintf(buffer, "MPL_mot_cal=%d\r\n",
                        storedConfig[NV_CONFIG_SETUP_BYTE4] & 0x07);
                f_write(&cfg_fil, buffer, strlen(buffer), &bw);
                sprintf(buffer, "MPL_rate=%d\r\n",
                        (storedConfig[NV_CONFIG_SETUP_BYTE5] >> 5) & 0x07);
                f_write(&cfg_fil, buffer, strlen(buffer), &bw);
                sprintf(buffer, "MPU_mag_rate=%d\r\n",
                        (storedConfig[NV_CONFIG_SETUP_BYTE5] >> 2) & 0x07);
                f_write(&cfg_fil, buffer, strlen(buffer), &bw);
                sprintf(buffer, "MPL_mag_mix=%d\r\n",
                        storedConfig[NV_CONFIG_SETUP_BYTE5] & 0x03);
                f_write(&cfg_fil, buffer, strlen(buffer), &bw);

                sprintf(buffer,
                        "MPL_QUAT_6DOF=%d\r\n",
                        storedConfig[NV_SENSORS3] & SENSOR_MPU9150_MPL_QUAT_6DOF ?
                                1 : 0);
                f_write(&cfg_fil, buffer, strlen(buffer), &bw);
                sprintf(buffer,
                        "MPL_QUAT_9DOF=%d\r\n",
                        storedConfig[NV_SENSORS3] & SENSOR_MPU9150_MPL_QUAT_9DOF ?
                                1 : 0);
                f_write(&cfg_fil, buffer, strlen(buffer), &bw);
                sprintf(buffer,
                        "MPL_Euler_6DOF=%d\r\n",
                        storedConfig[NV_SENSORS3]
                                & SENSOR_MPU9150_MPL_EULER_6DOF ? 1 : 0);
                f_write(&cfg_fil, buffer, strlen(buffer), &bw);
                sprintf(buffer,
                        "MPL_Euler_9DOF=%d\r\n",
                        storedConfig[NV_SENSORS3]
                                & SENSOR_MPU9150_MPL_EULER_9DOF ? 1 : 0);
                f_write(&cfg_fil, buffer, strlen(buffer), &bw);
                sprintf(buffer,
                        "MPL_heading=%d\r\n",
                        storedConfig[NV_SENSORS3] & SENSOR_MPU9150_MPL_HEADING ?
                                1 : 0);
                f_write(&cfg_fil, buffer, strlen(buffer), &bw);
                sprintf(buffer,
                        "MPU_temp=%d\r\n",
                        storedConfig[NV_SENSORS2] & SENSOR_MPU9150_TEMP ?
                                1 : 0);
                f_write(&cfg_fil, buffer, strlen(buffer), &bw);
                sprintf(buffer,
                        "MPL_pedometer=%d\r\n",
                        storedConfig[NV_SENSORS3] & SENSOR_MPU9150_MPL_PEDOMETER ?
                                1 : 0);
                f_write(&cfg_fil, buffer, strlen(buffer), &bw);
                sprintf(buffer,
                        "MPL_tap=%d\r\n",
                        storedConfig[NV_SENSORS3] & SENSOR_MPU9150_MPL_TAP ?
                                1 : 0);
                f_write(&cfg_fil, buffer, strlen(buffer), &bw);
                sprintf(buffer,
                        "MPL_motion_orient=%d\r\n",
                        storedConfig[NV_SENSORS3]
                                & SENSOR_MPU9150_MPL_MOTION_ORIENT ? 1 : 0);
                f_write(&cfg_fil, buffer, strlen(buffer), &bw);

                sprintf(buffer,
                        "MPL_gyro_cal=%d\r\n",
                        storedConfig[NV_SENSORS4] & SENSOR_MPU9150_GYRO_CAL ?
                                1 : 0);
                f_write(&cfg_fil, buffer, strlen(buffer), &bw);
                sprintf(buffer,
                        "MPL_accel_cal=%d\r\n",
                        storedConfig[NV_SENSORS4] & SENSOR_MPU9150_ACCEL_CAL ?
                                1 : 0);
                f_write(&cfg_fil, buffer, strlen(buffer), &bw);
                sprintf(buffer,
                        "MPL_mag_cal=%d\r\n",
                        storedConfig[NV_SENSORS4] & SENSOR_MPU9150_MAG_CAL ?
                                1 : 0);
                f_write(&cfg_fil, buffer, strlen(buffer), &bw);
                sprintf(buffer,
                        "MPL_QUAT_6DOF_RAW=%d\r\n",
                        storedConfig[NV_SENSORS4]
                                & SENSOR_MPU9150_MPL_QUAT_6DOF_RAW ? 1 : 0);
                f_write(&cfg_fil, buffer, strlen(buffer), &bw);

                sprintf(buffer,
                        "MPL_sensor_fusion=%d\r\n",
                        storedConfig[NV_CONFIG_SETUP_BYTE6]
                                & MPU9150_MPL_SENS_FUSION ? 1 : 0);
                f_write(&cfg_fil, buffer, strlen(buffer), &bw);
                sprintf(buffer,
                        "MPL_gyro_cal_tc=%d\r\n",
                        storedConfig[NV_CONFIG_SETUP_BYTE6]
                                & MPU9150_MPL_GYRO_CAL_TC ? 1 : 0);
                f_write(&cfg_fil, buffer, strlen(buffer), &bw);
                sprintf(buffer,
                        "MPL_vect_comp_cal=%d\r\n",
                        storedConfig[NV_CONFIG_SETUP_BYTE6]
                                & MPU9150_MPL_VECT_COMP_CAL ? 1 : 0);
                f_write(&cfg_fil, buffer, strlen(buffer), &bw);
                sprintf(buffer,
                        "MPL_mag_dist_cal=%d\r\n",
                        storedConfig[NV_CONFIG_SETUP_BYTE6]
                                & MPU9150_MPL_MAG_DIST_CAL ? 1 : 0);
                f_write(&cfg_fil, buffer, strlen(buffer), &bw);
                sprintf(buffer,
                        "MPL=%d\r\n",
                        storedConfig[NV_CONFIG_SETUP_BYTE6] & MPU9150_MPL_ENABLE ?
                                1 : 0);
                f_write(&cfg_fil, buffer, strlen(buffer), &bw);
                // DMP related - end
            }
            else
            {
                memset(storedConfig + NV_SENSORS3, 0, 5);
            }

            fileBad = f_close(&cfg_fil);
            msp430_delay_ms(50);
        }
        else
            fileBad = 1;
        if (!sd_power_state)
            SdPowerOff();
    }
}

uint8_t ParseConfig(void)
{
    FIL cfg_fil;
    char buffer[66], *equals;
    uint8_t string_length = 0;
    float sample_rate = 51.2;
    uint16_t sample_period = 0;
    uint64_t derived_channels_val = 0;
    uint8_t accel_lpm = 0, accel_hrm = 0, broadcast_interval, accel_mpu_range =
            0, exp_power = 0;
    uint8_t accel_range = 0, accel_smplrate = 0, gyro_range = 0, mag_smplrate =
            0, mag_gain = 0, pres_bmpX80_prec = 0, gsr_range = 0;
    bool user_button_enable = FALSE;
#if !RTC_OFF
    bool rwc_error_enable = TRUE;
#endif
    uint8_t my_trial_id = 0, num_shimmers_in_trial = 0;
    uint32_t est_exp_len = 0;
    uint32_t max_exp_len = 0;
    uint32_t config_time = 0;
    bool iAmMaster = FALSE, time_sync = FALSE, singletouch = FALSE,
            tcxo = FALSE, low_battery_autostop = FALSE;
    uint8_t i, pchar[3], node_addr[6], center_addr[6];

    // DMP related - start
    uint8_t config_buffer;
    // DMP related - end
    int node_i;
    for (node_i = 0; node_i < MAX_NODES; node_i++)
    {
        *nodeName[node_i] = '\0';
    }
    syncNodeNum = 0;
    nodeSuccFull = 0;
    broadcast_interval = RC_INT_C;

    memset((uint8_t*) (storedConfig), 0, NV_A_ACCEL_CALIBRATION);   //0
    memset((uint8_t*) (storedConfig + NV_A_ACCEL_CALIBRATION), 0xff, 84);
    memset((uint8_t*) (storedConfig + NV_DERIVED_CHANNELS_3), 0, 5);   //0
    memset((uint8_t*) (storedConfig + NV_SENSORS3), 0, 5);   //0
    memset((uint8_t*) (storedConfig + NV_SD_SHIMMER_NAME), 0, 37);   //0
    InfoMem_read((uint8_t *) NV_MAC_ADDRESS, storedConfig + NV_MAC_ADDRESS, 7);
    memset((uint8_t*) (storedConfig + NV_SD_CONFIG_DELAY_FLAG + 1), 0xff, 25);
    memset((uint8_t*) (storedConfig + NV_NODE0), 0xff, 128);

    storedConfig[NV_SD_TRIAL_CONFIG0] &= ~SDH_SET_PMUX;    // PMUX reserved as 0
    storedConfig[NV_SD_TRIAL_CONFIG0] |= SDH_TIME_STAMP; // TIME_STAMP always = 1

    memset(shimmerName, 0, MAX_CHARS);
    memset(expIdName, 0, MAX_CHARS);
    memset(centerName, 0, MAX_CHARS);
    memset(configTimeText, 0, MAX_CHARS);
    clockFreq = (float) MSP430_CLOCK;

    newDirFlag = 1;

    CheckSdInslot();
    char cfgname[] = "sdlog.cfg";
    fileBad = f_open(&cfg_fil, cfgname, FA_READ | FA_OPEN_EXISTING);
    if (fileBad == FR_NO_FILE)
        return fileBad;

    while (f_gets(buffer, 64, &cfg_fil))
    {
        if (!(equals = strchr(buffer, '=')))
            continue;
        equals++;   // this is the value
        if (strstr(buffer, "accel="))  //a_accel on/off
            storedConfig[NV_SENSORS0] |= atoi(equals) * SENSOR_A_ACCEL;
        else if (strstr(buffer, "gyro="))
            storedConfig[NV_SENSORS0] |= atoi(equals) * SENSOR_MPU9150_GYRO;
        else if (strstr(buffer, "mag="))
            storedConfig[NV_SENSORS0] |= atoi(equals) * SENSOR_LSM303DLHC_MAG;
        else if (strstr(buffer, "exg1_24bit="))
            storedConfig[NV_SENSORS0] |= atoi(equals) * SENSOR_EXG1_24BIT;
        else if (strstr(buffer, "exg2_24bit="))
            storedConfig[NV_SENSORS0] |= atoi(equals) * SENSOR_EXG2_24BIT;
        else if (strstr(buffer, "gsr="))
            storedConfig[NV_SENSORS0] |= atoi(equals) * SENSOR_GSR;
        else if (strstr(buffer, "extch7="))
            storedConfig[NV_SENSORS0] |= atoi(equals) * SENSOR_EXT_A7;
        else if (strstr(buffer, "extch6="))
            storedConfig[NV_SENSORS0] |= atoi(equals) * SENSOR_EXT_A6;
        else if (strstr(buffer, "br_amp="))
            storedConfig[NV_SENSORS1] |= atoi(equals) * SENSOR_STRAIN;
        else if (strstr(buffer, "vbat="))
            storedConfig[NV_SENSORS1] |= atoi(equals) * SENSOR_VBATT;
        else if (strstr(buffer, "accel_d="))
            storedConfig[NV_SENSORS1] |= atoi(equals) * SENSOR_LSM303DLHC_ACCEL;
        else if (strstr(buffer, "extch15="))
            storedConfig[NV_SENSORS1] |= atoi(equals) * SENSOR_EXT_A15;
        else if (strstr(buffer, "intch1="))
            storedConfig[NV_SENSORS1] |= atoi(equals) * SENSOR_INT_A1;
        else if (strstr(buffer, "intch12="))
            storedConfig[NV_SENSORS1] |= atoi(equals) * SENSOR_INT_A12;
        else if (strstr(buffer, "intch13="))
            storedConfig[NV_SENSORS1] |= atoi(equals) * SENSOR_INT_A13;
        else if (strstr(buffer, "intch14="))
            storedConfig[NV_SENSORS2] |= atoi(equals) * SENSOR_INT_A14;
        else if (strstr(buffer, "accel_mpu="))
            storedConfig[NV_SENSORS2] |= atoi(equals) * SENSOR_MPU9150_ACCEL;
        else if (strstr(buffer, "mag_mpu="))
            storedConfig[NV_SENSORS2] |= atoi(equals) * SENSOR_MPU9150_MAG;
        else if (strstr(buffer, "exg1_16bit="))
            storedConfig[NV_SENSORS2] |= atoi(equals) * SENSOR_EXG1_16BIT;
        else if (strstr(buffer, "exg2_16bit="))
            storedConfig[NV_SENSORS2] |= atoi(equals) * SENSOR_EXG2_16BIT;
        else if (strstr(buffer, "pres_bmp180=") || strstr(buffer, "pres_bmp280="))
            storedConfig[NV_SENSORS2] |= atoi(equals) * SENSOR_BMPX80_PRESSURE;
        else if (strstr(buffer, "sample_rate="))
        {
            sample_rate = atof(equals);
            _NOP();
        }
        else if (strstr(buffer, "mg_internal_rate="))
        {
            mag_smplrate = atoi(equals);
            storedConfig[NV_CONFIG_SETUP_BYTE2] |= (mag_smplrate & 0x07) << 2; //BIT4-2
        }
        else if (strstr(buffer, "mg_range="))
        {
            mag_gain = atoi(equals);
            storedConfig[NV_CONFIG_SETUP_BYTE2] |= (mag_gain & 0x07) << 5; //BIT7-5
        }
        else if (strstr(buffer, "acc_internal_rate="))
        {
            accel_smplrate = atoi(equals);
            storedConfig[NV_CONFIG_SETUP_BYTE0] |= (accel_smplrate & 0x0f) << 4; //BIT7-4
        }
        else if (strstr(buffer, "accel_mpu_range="))
        {
            accel_mpu_range = atoi(equals);
            storedConfig[NV_CONFIG_SETUP_BYTE3] |= (accel_mpu_range & 0x03)
                    << 6;  //BIT7-6
        }
        else if (strstr(buffer, "acc_range="))
        {
            accel_range = atoi(equals);
            storedConfig[NV_CONFIG_SETUP_BYTE0] |= (accel_range & 0x03) << 2; //BIT3-2
        }
        else if (strstr(buffer, "acc_lpm="))
        {
            accel_lpm = atoi(equals);
            storedConfig[NV_CONFIG_SETUP_BYTE0] |= (accel_lpm & 0x01) << 1; //BIT1
            storedConfig[NV_SD_TRIAL_CONFIG1] |= (accel_lpm & 0x01)
                    * SDH_ACCEL_LPM;
        }
        else if (strstr(buffer, "acc_hrm="))
        {
            accel_hrm = atoi(equals);
            storedConfig[NV_CONFIG_SETUP_BYTE0] |= accel_hrm & 0x01;      //BIT0
            storedConfig[NV_SD_TRIAL_CONFIG1] |= (accel_hrm & 0x01)
                    * SDH_ACCEL_HRM;
        }
        else if (strstr(buffer, "gsr_range="))
        {
            gsr_range = atoi(equals);
            if (gsr_range <= 4)
                storedConfig[NV_CONFIG_SETUP_BYTE3] |= (gsr_range & 0x07) << 1; //BIT3-1
            else
                storedConfig[NV_CONFIG_SETUP_BYTE3] |= (4 & 0x07) << 1; //BIT3-1
        }
        else if (strstr(buffer, "gyro_samplingrate="))
            storedConfig[NV_CONFIG_SETUP_BYTE1] = atoi(equals);
        else if (strstr(buffer, "gyro_range="))
        {
            gyro_range = atoi(equals);
            storedConfig[NV_CONFIG_SETUP_BYTE2] |= (gyro_range) & 0x03; //BIT1-0
        }
        else if (strstr(buffer, "pres_bmp180_prec=") || strstr(buffer, "pres_bmp280_prec="))
        {
            pres_bmpX80_prec = atoi(equals);
            storedConfig[NV_CONFIG_SETUP_BYTE3] |= (pres_bmpX80_prec & 0x03)
                    << 4;          //BIT5-4
        }
#if !RTC_OFF
        else if (strstr(buffer, "rtc_error_enable="))
        {
            rwc_error_enable = (atoi(equals) == 0) ? FALSE : TRUE;
        }
#endif
        else if (strstr(buffer, "user_button_enable="))
        {
            user_button_enable = (atoi(equals) == 0) ? FALSE : TRUE;
            storedConfig[NV_SD_TRIAL_CONFIG0] |= user_button_enable
                    * SDH_USER_BUTTON_ENABLE;
        }
        else if (strstr(buffer, "iammaster="))
        {          //0=node
            iAmMaster = (atoi(equals) == 0) ? FALSE : TRUE;
            storedConfig[NV_SD_TRIAL_CONFIG0] |= iAmMaster * SDH_IAMMASTER;
        }
        else if (strstr(buffer, "sync="))
        {
            time_sync = (atoi(equals) == 0) ? FALSE : TRUE;
            storedConfig[NV_SD_TRIAL_CONFIG0] |= time_sync * SDH_TIME_SYNC;
        }
        else if (strstr(buffer, "interval="))
        {
            broadcast_interval = atoi(equals) > 255 ? 255 : atoi(equals);
        }
        else if (strstr(buffer, "singletouch="))
        {
            singletouch = (atoi(equals) == 0) ? FALSE : TRUE;
            storedConfig[NV_SD_TRIAL_CONFIG1] |= singletouch * SDH_SINGLETOUCH;
        }
        else if (strstr(buffer, "exp_power="))
        {
            exp_power = atoi(equals);
            storedConfig[NV_CONFIG_SETUP_BYTE3] |= (exp_power & 0x01)
                    * EXP_POWER_ENABLE;
        }
        else if (strstr(buffer, "low_battery_autostop="))
        {
            low_battery_autostop = (atoi(equals) == 0) ? FALSE : TRUE;
            storedConfig[NV_SD_TRIAL_CONFIG1] |= low_battery_autostop
                    * SDH_BATT_CRITICAL_CUTOFF;
        }
        else if (strstr(buffer, "tcxo="))
        {
            tcxo = (atoi(equals) == 0) ? FALSE : TRUE;
            storedConfig[NV_SD_TRIAL_CONFIG1] |= tcxo * SDH_TCXO;
        }
        else if (strstr(buffer, "center="))
        {
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
                memcpy(storedConfig + NV_CENTER, center_addr, 6);
            }
        }
        else if (strstr(buffer, "node"))
        {
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
                memcpy(storedConfig + NV_NODE0 + syncNodeNum * 6, node_addr, 6);
                nodeSuccFull |= SyncNodeShift(syncNodeNum);
                syncNodeNum++;
            }
        }
        else if (strstr(buffer, "est_exp_len="))
        {
            est_exp_len = atoi(equals);
            storedConfig[NV_EST_EXP_LEN_MSB] = (est_exp_len & 0xff00) >> 8;
            storedConfig[NV_EST_EXP_LEN_LSB] = est_exp_len & 0xff;
        }
        else if (strstr(buffer, "max_exp_len="))
        {
            max_exp_len = atoi(equals);
            storedConfig[NV_MAX_EXP_LEN_MSB] = (max_exp_len & 0xff00) >> 8;
            storedConfig[NV_MAX_EXP_LEN_LSB] = max_exp_len & 0xff;
        }
        else if (strstr(buffer, "myid="))
        {
            my_trial_id = atoi(equals);
            storedConfig[NV_SD_MYTRIAL_ID] = my_trial_id;
        }
        else if (strstr(buffer, "Nshimmer="))
        {
            num_shimmers_in_trial = atoi(equals);
            storedConfig[NV_SD_NSHIMMER] = num_shimmers_in_trial;
        }
        else if (strstr(buffer, "shimmername="))
        {
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
            if (string_length)
            {
                memcpy((char*) (storedConfig + NV_SD_SHIMMER_NAME), equals,
                       string_length);
                if (!memcmp((char*) (storedConfig + NV_SD_SHIMMER_NAME), "ID",
                            2))
                    memcpy((char*) (storedConfig + NV_SD_SHIMMER_NAME), "id",
                           2);
                memcpy((char*) shimmerName,
                       (char*) (storedConfig + NV_SD_SHIMMER_NAME),
                       MAX_CHARS - 1);
                shimmerName[string_length] = 0;
            }
            else
            {
                memset((char*) (storedConfig + NV_SD_SHIMMER_NAME), 0,
                MAX_CHARS - 1);
            }
        }
        else if (strstr(buffer, "experimentid="))
        {
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
            if (string_length)
            {
                memcpy((char*) (storedConfig + NV_SD_EXP_ID_NAME), equals,
                       string_length);
                memcpy((char*) expIdName,
                       (char*) (storedConfig + NV_SD_EXP_ID_NAME),
                       MAX_CHARS - 1);
                expIdName[string_length] = 0;
            }
            else
            {
                memset((char*) (storedConfig + NV_SD_EXP_ID_NAME), 0,
                MAX_CHARS - 1);
            }
        }
        else if (strstr(buffer, "configtime="))
        {
            config_time = atol(equals);
            storedConfig[NV_SD_CONFIG_TIME] = *((uint8_t*) &config_time + 3);
            storedConfig[NV_SD_CONFIG_TIME + 1] =
                    *((uint8_t*) &config_time + 2);
            storedConfig[NV_SD_CONFIG_TIME + 2] =
                    *((uint8_t*) &config_time + 1);
            storedConfig[NV_SD_CONFIG_TIME + 3] = *((uint8_t*) &config_time);
            string_length = strlen(equals);
            if (string_length > MAX_CHARS)
                string_length = MAX_CHARS - 1;
            else if (string_length >= 2)
                string_length -= 2;
            else
                string_length = 0;
            memcpy((char*) configTimeText, equals, string_length);
            configTimeText[MAX_CHARS - 1] = 0;
        }
        else if (strstr(buffer, "baud_rate="))
        {

        }
        else if (strstr(buffer, "derived_channels="))
        {
//         derived_channels_val = atol(equals);
//         storedConfig[NV_DERIVED_CHANNELS_0] = derived_channels_val & 0xff;
//         storedConfig[NV_DERIVED_CHANNELS_1] = (derived_channels_val >> 8) & 0xff;
//         storedConfig[NV_DERIVED_CHANNELS_2] = (derived_channels_val >> 16) & 0xff;
            derived_channels_val = atoll(equals);
            storedConfig[NV_DERIVED_CHANNELS_0] = derived_channels_val & 0xff;
            storedConfig[NV_DERIVED_CHANNELS_1] = (derived_channels_val >> 8)
                    & 0xff;
            storedConfig[NV_DERIVED_CHANNELS_2] = (derived_channels_val >> 16)
                    & 0xff;
            storedConfig[NV_DERIVED_CHANNELS_3] = (derived_channels_val >> 24)
                    & 0xff;
            storedConfig[NV_DERIVED_CHANNELS_4] = (derived_channels_val >> 32)
                    & 0xff;
            storedConfig[NV_DERIVED_CHANNELS_5] = (derived_channels_val >> 40)
                    & 0xff;
            storedConfig[NV_DERIVED_CHANNELS_6] = (derived_channels_val >> 48)
                    & 0xff;
            storedConfig[NV_DERIVED_CHANNELS_7] = (derived_channels_val >> 56)
                    & 0xff;
        }
        else if (strstr(buffer, "EXG_ADS1292R_1_CONFIG1="))
            storedConfig[NV_EXG_ADS1292R_1_CONFIG1] = atoi(equals);
        else if (strstr(buffer, "EXG_ADS1292R_1_CONFIG2="))
            storedConfig[NV_EXG_ADS1292R_1_CONFIG2] = atoi(equals);
        else if (strstr(buffer, "EXG_ADS1292R_1_LOFF="))
            storedConfig[NV_EXG_ADS1292R_1_LOFF] = atoi(equals);
        else if (strstr(buffer, "EXG_ADS1292R_1_CH1SET="))
            storedConfig[NV_EXG_ADS1292R_1_CH1SET] = atoi(equals);
        else if (strstr(buffer, "EXG_ADS1292R_1_CH2SET="))
            storedConfig[NV_EXG_ADS1292R_1_CH2SET] = atoi(equals);
        else if (strstr(buffer, "EXG_ADS1292R_1_RLD_SENS="))
            storedConfig[NV_EXG_ADS1292R_1_RLD_SENS] = atoi(equals);
        else if (strstr(buffer, "EXG_ADS1292R_1_LOFF_SENS="))
            storedConfig[NV_EXG_ADS1292R_1_LOFF_SENS] = atoi(equals);
        else if (strstr(buffer, "EXG_ADS1292R_1_LOFF_STAT="))
            storedConfig[NV_EXG_ADS1292R_1_LOFF_STAT] = atoi(equals);
        else if (strstr(buffer, "EXG_ADS1292R_1_RESP1="))
            storedConfig[NV_EXG_ADS1292R_1_RESP1] = atoi(equals);
        else if (strstr(buffer, "EXG_ADS1292R_1_RESP2="))
            storedConfig[NV_EXG_ADS1292R_1_RESP2] = atoi(equals);
        else if (strstr(buffer, "EXG_ADS1292R_2_CONFIG1="))
            storedConfig[NV_EXG_ADS1292R_2_CONFIG1] = atoi(equals);
        else if (strstr(buffer, "EXG_ADS1292R_2_CONFIG2="))
            storedConfig[NV_EXG_ADS1292R_2_CONFIG2] = atoi(equals);
        else if (strstr(buffer, "EXG_ADS1292R_2_LOFF="))
            storedConfig[NV_EXG_ADS1292R_2_LOFF] = atoi(equals);
        else if (strstr(buffer, "EXG_ADS1292R_2_CH1SET="))
            storedConfig[NV_EXG_ADS1292R_2_CH1SET] = atoi(equals);
        else if (strstr(buffer, "EXG_ADS1292R_2_CH2SET="))
            storedConfig[NV_EXG_ADS1292R_2_CH2SET] = atoi(equals);
        else if (strstr(buffer, "EXG_ADS1292R_2_RLD_SENS="))
            storedConfig[NV_EXG_ADS1292R_2_RLD_SENS] = atoi(equals);
        else if (strstr(buffer, "EXG_ADS1292R_2_LOFF_SENS="))
            storedConfig[NV_EXG_ADS1292R_2_LOFF_SENS] = atoi(equals);
        else if (strstr(buffer, "EXG_ADS1292R_2_LOFF_STAT="))
            storedConfig[NV_EXG_ADS1292R_2_LOFF_STAT] = atoi(equals);
        else if (strstr(buffer, "EXG_ADS1292R_2_RESP1="))
            storedConfig[NV_EXG_ADS1292R_2_RESP1] = atoi(equals);
        else if (strstr(buffer, "EXG_ADS1292R_2_RESP2="))
            storedConfig[NV_EXG_ADS1292R_2_RESP2] = atoi(equals);
        // DMP related - start
        else if (strstr(buffer, "DMP=")) //Digital Motion Processor Library - on/off
            storedConfig[NV_CONFIG_SETUP_BYTE4] |= atoi(equals)*MPU9150_MPL_DMP;
        else if (strstr(buffer, "MPL_use_LSM_mag=")) // Use the LSM303 Mag for the 9DOF calculations
            storedConfig[NV_CONFIG_SETUP_BYTE4] |= atoi(
                    equals)*MPU9150_MPL_USE_LSM303_MAG;
        //MPL Low Pass filter (options are: 0=256Hz(no LPF),1=188Hz,2=98Hz,3=42Hz,4=20Hz,5=10Hz,6=5Hz)
        else if (strstr(buffer, "MPL_LPF="))
        {
            config_buffer = atoi(equals);
            storedConfig[NV_CONFIG_SETUP_BYTE4] |= (config_buffer & 0x07) << 3; //BIT5-3
        }
        //MPL Gyro calibration (options are: No Calibration, Fast Calibration, 1s no motion, 2s no motion, 5s no motion, 10s no motion,   30s no motion, 60s no motion)
        else if (strstr(buffer, "MPL_mot_cal="))
        {
            config_buffer = atoi(equals);
            storedConfig[NV_CONFIG_SETUP_BYTE4] |= config_buffer & 0x07; //BIT2-0
        }

        else if (strstr(buffer, "MPL_rate="))
        {      //MPU9150 MPL Sampling Rate
            config_buffer = atoi(equals);
            storedConfig[NV_CONFIG_SETUP_BYTE5] |= (config_buffer & 0x07) << 5; //BIT7-5
        }
        else if (strstr(buffer, "MPU_mag_rate="))
        { //MPU9150 Magnetometer Sampling Rate
            config_buffer = atoi(equals);
            storedConfig[NV_CONFIG_SETUP_BYTE5] |= (config_buffer & 0x07) << 2; //BIT4-2
        }
        //Mag mix for Pansenti 9DOF calculation (options are: 0=GYRO_ONLY, 1=MAG_ONLY, 2=GYRO_AND_MAG, 3=GYRO_AND_SOME_MAG)
        else if (strstr(buffer, "MPL_mag_mix="))
        {
            config_buffer = atoi(equals);
            storedConfig[NV_CONFIG_SETUP_BYTE5] |= (config_buffer) & 0x03; //BIT1-0
        }

        else if (strstr(buffer, "MPL_QUAT_6DOF="))      //MPL QUAT 6DOF - on/off
            storedConfig[NV_SENSORS3] |= atoi(
                    equals)*SENSOR_MPU9150_MPL_QUAT_6DOF;
        else if (strstr(buffer, "MPL_QUAT_9DOF="))      //MPL QUAT 9DOF - on/off
            storedConfig[NV_SENSORS3] |= atoi(
                    equals)*SENSOR_MPU9150_MPL_QUAT_9DOF;
        else if (strstr(buffer, "MPL_Euler_6DOF="))    //MPL Euler 6DOF - on/off
            storedConfig[NV_SENSORS3] |= atoi(
                    equals)*SENSOR_MPU9150_MPL_EULER_6DOF;
        else if (strstr(buffer, "MPL_Euler_9DOF="))    //MPL Euler 9DOF - on/off
            storedConfig[NV_SENSORS3] |= atoi(
                    equals)*SENSOR_MPU9150_MPL_EULER_9DOF;
        else if (strstr(buffer, "MPL_heading="))          //MPL Heading - on/off
            storedConfig[NV_SENSORS3] |= atoi(
                    equals)*SENSOR_MPU9150_MPL_HEADING;
        else if (strstr(buffer, "MPU_temp="))  //MPU Temperature output - on/off
            storedConfig[NV_SENSORS2] |= atoi(equals) * SENSOR_MPU9150_TEMP;
        else if (strstr(buffer, "MPL_pedometer="))      //MPL Pedometer - on/off
            storedConfig[NV_SENSORS3] |= atoi(
                    equals)*SENSOR_MPU9150_MPL_PEDOMETER;
        else if (strstr(buffer, "MPL_tap=")) //MPL TAP detection output - on/off
            storedConfig[NV_SENSORS3] |= atoi(equals) * SENSOR_MPU9150_MPL_TAP;
        else if (strstr(buffer, "MPL_motion_orient=")) //MPL Motion&Orientation detection output - on/off
            storedConfig[NV_SENSORS3] |= atoi(
                    equals)*SENSOR_MPU9150_MPL_MOTION_ORIENT;

        else if (strstr(buffer, "MPL_gyro_cal=")) //MPL Gyro calibrated - on/off
            storedConfig[NV_SENSORS4] |= atoi(equals) * SENSOR_MPU9150_GYRO_CAL;
        else if (strstr(buffer, "MPL_accel_cal=")) //MPL Accel calibrated - on/off
            storedConfig[NV_SENSORS4] |=
                    atoi(equals) * SENSOR_MPU9150_ACCEL_CAL;
        else if (strstr(buffer, "MPL_mag_cal="))    //MPL Mag calibrated- on/off
            storedConfig[NV_SENSORS4] |= atoi(equals) * SENSOR_MPU9150_MAG_CAL;
        else if (strstr(buffer, "MPL_QUAT_6DOF_RAW=")) //MPL QUAT 6DOF RAW - on/off
            storedConfig[NV_SENSORS4] |= atoi(
                    equals)*SENSOR_MPU9150_MPL_QUAT_6DOF_RAW;

        else if (strstr(buffer, "MPL_sensor_fusion=")) //MPL 9x sensor fusion - on/off
            storedConfig[NV_CONFIG_SETUP_BYTE6] |= atoi(
                    equals)*MPU9150_MPL_SENS_FUSION;
        else if (strstr(buffer, "MPL_gyro_cal_tc=")) //MPL gyro calibrate on temp. change - on/off
            storedConfig[NV_CONFIG_SETUP_BYTE6] |= atoi(
                    equals)*MPU9150_MPL_GYRO_CAL_TC;
        else if (strstr(buffer, "MPL_vect_comp_cal=")) //MPL QUAT vector compass calibrate - on/off
            storedConfig[NV_CONFIG_SETUP_BYTE6] |= atoi(
                    equals)*MPU9150_MPL_VECT_COMP_CAL;
        else if (strstr(buffer, "MPL_mag_dist_cal=")) //MPL QUAT magnetic disturbance calibrate - on/off
            storedConfig[NV_CONFIG_SETUP_BYTE6] |= atoi(
                    equals)*MPU9150_MPL_MAG_DIST_CAL;
        else if (strstr(buffer, "MPL="))     //Motion Processor Library - on/off
            storedConfig[NV_CONFIG_SETUP_BYTE6] |= atoi(
                    equals)*MPU9150_MPL_ENABLE;
        // DMP related - end

    }
    fileBad = f_close(&cfg_fil);
    _delay_cycles(1200000);

    SamplingClkAssignment();
    ClkAssignment();
    TB0CTL = MC_0;
    TB0Start();

    sample_period = FreqDiv(sample_rate);

    storedConfig[NV_SAMPLING_RATE] = (uint8_t) (sample_period & 0xFF);
    storedConfig[NV_SAMPLING_RATE + 1] = (uint8_t) (sample_period >> 8);

    if (storedConfig[NV_SENSORS0] & SENSOR_GSR) // they are sharing adc1, ban intch1 when gsr is on
        storedConfig[NV_SENSORS1] &= ~SENSOR_INT_A1;

    if (storedConfig[NV_SENSORS0] & SENSOR_EXG1_24BIT)
        storedConfig[NV_SENSORS2] &= ~SENSOR_EXG1_16BIT;
    if (storedConfig[NV_SENSORS0] & SENSOR_EXG2_24BIT)
        storedConfig[NV_SENSORS2] &= ~SENSOR_EXG2_16BIT;
    if (storedConfig[NV_SENSORS0] & SENSOR_EXG1_24BIT
            || storedConfig[NV_SENSORS0] & SENSOR_EXG2_24BIT
            || storedConfig[NV_SENSORS2] & SENSOR_EXG1_16BIT
            || storedConfig[NV_SENSORS2] & SENSOR_EXG2_16BIT)
    {
        storedConfig[NV_CONFIG_SETUP_BYTE3] |= EXP_POWER_ENABLE;
        storedConfig[NV_SENSORS1] &= ~SENSOR_INT_A1;
        storedConfig[NV_SENSORS2] &= ~SENSOR_INT_A14;
    }

    if (storedConfig[NV_SENSORS1] & SENSOR_STRAIN)
    { // they are sharing adc13 and adc14
        storedConfig[NV_SENSORS1] &= ~SENSOR_INT_A13;
        storedConfig[NV_SENSORS2] &= ~SENSOR_INT_A14;
    }
    if (((storedConfig[NV_CONFIG_SETUP_BYTE3] >> 1) & 0x07) > 4) // never larger than 4
        storedConfig[NV_CONFIG_SETUP_BYTE3] |= (4 & 0x07) << 1;   //BIT3-1

#if !RTC_OFF
    storedConfig[NV_SD_TRIAL_CONFIG0] |= rwc_error_enable * SDH_RWCERROR_EN;
#endif

    // minimum sync broadcast interval is 54 seconds
    if (broadcast_interval < RC_INT_C)
    {
        broadcast_interval = RC_INT_C;
    }

    // the button always works for singletouch mode
    // sync always works for singletouch mode
    if (storedConfig[NV_SD_TRIAL_CONFIG1] & SDH_SINGLETOUCH)
    {
        storedConfig[NV_SD_TRIAL_CONFIG0] |= SDH_USER_BUTTON_ENABLE;
        storedConfig[NV_SD_TRIAL_CONFIG0] |= SDH_TIME_SYNC;
    }

    if (strlen((char*) centerName) == 0) // if no center is appointed, let this guy be the center
        strcpy((char*) centerName, "000000000000");

    if (est_exp_len < 10)
    {
        shortExpFlag = 1;
        est_exp_len = 0xffff;
    }
    else
    {
        shortExpFlag = 0;
        if (est_exp_len > 180)
            est_exp_len = 180;
    }
    estLen = est_exp_len * 60;
    estLen3 = estLen / 3;
    maxLen = max_exp_len * 60;

    InfoMem_write((uint8_t *) 0, storedConfig, NV_A_ACCEL_CALIBRATION);
    InfoMem_write((uint8_t *) NV_DERIVED_CHANNELS_3,
                  storedConfig + NV_DERIVED_CHANNELS_3, 5);
    InfoMem_write((uint8_t *) NV_SENSORS3, storedConfig + NV_SENSORS3, 5);
    InfoMem_write((uint8_t *) NV_SD_SHIMMER_NAME,
                  storedConfig + NV_SD_SHIMMER_NAME, 37);
    InfoMem_write((uint8_t *) (NV_MAC_ADDRESS + 7),
                  storedConfig + NV_MAC_ADDRESS + 7, 153); //25+128

    return fileBad;
}

void SetDefaultConfiguration(void)
{
    /* 51.2Hz */
    memset((uint8_t*) (storedConfig), 0, NV_A_ACCEL_CALIBRATION); //0
    memset((uint8_t*) (storedConfig + NV_A_ACCEL_CALIBRATION), 0xff, 84);
    memset((uint8_t*) (storedConfig + NV_DERIVED_CHANNELS_3), 0, 5); //0
    memset((uint8_t*) (storedConfig + NV_SENSORS3), 0, 5); //0
    memset((uint8_t*) (storedConfig + NV_SD_SHIMMER_NAME), 0, 37); //0
    InfoMem_read((uint8_t *) NV_MAC_ADDRESS, storedConfig + NV_MAC_ADDRESS, 7);
    memset((uint8_t*) (storedConfig + NV_SD_CONFIG_DELAY_FLAG + 1), 0xff, 25);
    memset((uint8_t*) (storedConfig + NV_NODE0), 0xff, 128);

    *(uint16_t *) (storedConfig + NV_SAMPLING_RATE) = 640; /* 32768 / 51.2 = 640 */
    storedConfig[NV_BUFFER_SIZE] = 1;
    /* core sensors enabled */
    storedConfig[NV_SENSORS0] = SENSOR_A_ACCEL + SENSOR_MPU9150_GYRO
            + SENSOR_LSM303DLHC_MAG;
    storedConfig[NV_SENSORS1] = SENSOR_VBATT;
    storedConfig[NV_SENSORS2] = 0;
    /* LSM303DLHC Accel 100Hz, +/-2G, Low Power and High Resolution modes off */
    storedConfig[NV_CONFIG_SETUP_BYTE0] = (LSM303DLHC_ACCEL_100HZ << 4)
            + (ACCEL_2G << 2);
    /* MPU9150 sampling rate of 8kHz/(155+1), i.e. 51.282Hz */
    storedConfig[NV_CONFIG_SETUP_BYTE1] = 0x9B;
    /* LSM303DLHC Mag 75Hz, +/-1.3 Gauss, MPU9150 Gyro +/-500 degrees per second */
    storedConfig[NV_CONFIG_SETUP_BYTE2] = (LSM303DLHC_MAG_1_3G << 5)
            + (LSM303DLHC_MAG_75HZ << 2) + MPU9150_GYRO_500DPS;
    /* MPU9150 Accel +/-2G, BMP pressure oversampling ratio 1, GSR auto range, EXP_RESET_N pin set low */
        /* todo: *** *** *** warning! *** *** ***  btStream for this here is not correct, this is mpu9150_accrange, not lsm303 acc range */
    storedConfig[NV_CONFIG_SETUP_BYTE3] = (ACCEL_2G << 6)
            + (GSR_AUTORANGE << 1);   /* HW_RES_40K */

    /* Set all ExG registers to their reset values */
        /*ADS CHIP 1*/
    storedConfig[NV_EXG_ADS1292R_1_CONFIG1] = 0x02;
    storedConfig[NV_EXG_ADS1292R_1_CONFIG2] = 0x80;
    if ((daughtCardId[DAUGHT_CARD_ID] == 47)
                && (daughtCardId[DAUGHT_CARD_REV] >= 4))
        {
            /* Check if unit is SR47-4 or greater.
             * If so, amend configuration byte 2 of ADS chip 1 to have bit 3 set to 1.
             * This ensures clock lines on ADS chip are correct
             */
            storedConfig[NV_EXG_ADS1292R_1_CONFIG2] |= 8;
        }
    storedConfig[NV_EXG_ADS1292R_1_LOFF] = 0x10;
    storedConfig[NV_EXG_ADS1292R_1_CH1SET] = 0x00;
    storedConfig[NV_EXG_ADS1292R_1_CH2SET] = 0x00;
    storedConfig[NV_EXG_ADS1292R_1_RLD_SENS] = 0x00;
    storedConfig[NV_EXG_ADS1292R_1_LOFF_SENS] = 0x00;
    storedConfig[NV_EXG_ADS1292R_1_LOFF_STAT] = 0x00;
    storedConfig[NV_EXG_ADS1292R_1_RESP1] = 0x00;
    storedConfig[NV_EXG_ADS1292R_1_RESP2] = 0x02;

    /*ADS CHIP 2*/
    storedConfig[NV_EXG_ADS1292R_2_CONFIG1] = 0x02;
    storedConfig[NV_EXG_ADS1292R_2_CONFIG2] = 0x80;
    storedConfig[NV_EXG_ADS1292R_2_LOFF] = 0x10;
    storedConfig[NV_EXG_ADS1292R_2_CH1SET] = 0x00;
    storedConfig[NV_EXG_ADS1292R_2_CH2SET] = 0x00;
    storedConfig[NV_EXG_ADS1292R_2_RLD_SENS] = 0x00;
    storedConfig[NV_EXG_ADS1292R_2_LOFF_SENS] = 0x00;
    storedConfig[NV_EXG_ADS1292R_2_LOFF_STAT] = 0x00;
    storedConfig[NV_EXG_ADS1292R_2_RESP1] = 0x00;
    storedConfig[NV_EXG_ADS1292R_2_RESP2] = 0x02;
    if (storedConfig[NV_BT_COMMS_BAUD_RATE] == 0xFF){
        storedConfig[NV_BT_COMMS_BAUD_RATE] = 9;
    }

    /* Write RAM contents to Infomem */
    InfoMem_write((void*) 0, storedConfig, NV_A_ACCEL_CALIBRATION);

    /* sd config */
        /* shimmername */
    strcpy((char*) (storedConfig + NV_SD_SHIMMER_NAME), "Shimmer_XXXX");

    memcpy((storedConfig + NV_SD_SHIMMER_NAME) + 8, mac + 8, 4);
    memcpy(shimmerName, (storedConfig + NV_SD_SHIMMER_NAME), 6);
    shimmerName[6] = 0;

    /* exp_id */
    strcpy((char*) (storedConfig + NV_SD_EXP_ID_NAME), "default_exp");
    strcpy((char*) expIdName, "default_exp");
    strcpy((char*) configTimeText, "0");
    memset(&storedConfig[NV_SD_CONFIG_TIME], 0x00, 4);
    storedConfig[NV_SD_MYTRIAL_ID] = 0x00;
    storedConfig[NV_SD_NSHIMMER] = 0x00;
    storedConfig[NV_SD_TRIAL_CONFIG0] =
    SDH_USER_BUTTON_ENABLE + SDH_RWCERROR_EN;
    storedConfig[NV_SD_TRIAL_CONFIG1] = 0;
    storedConfig[NV_SD_BT_INTERVAL] = 54;

    InfoMem_write((uint8_t *) 0, storedConfig, NV_A_ACCEL_CALIBRATION);
    InfoMem_write((uint8_t *) NV_SENSORS3, storedConfig + NV_SENSORS3, 5);
    InfoMem_write((uint8_t *) NV_SD_SHIMMER_NAME,
                  storedConfig + NV_SD_SHIMMER_NAME, 37);
    InfoMem_write((uint8_t *) (NV_MAC_ADDRESS + 7),
                  storedConfig + NV_MAC_ADDRESS + 7, 153);   //25+128

}
void Config2SdHead(void)
{
    memset(sdHeadText, 0, SDHEAD_LEN);
    sdHeadText[SDH_SAMPLE_RATE_0] = storedConfig[NV_SAMPLING_RATE];
    sdHeadText[SDH_SAMPLE_RATE_1] = storedConfig[NV_SAMPLING_RATE + 1];
    sdHeadText[SDH_SENSORS0] = storedConfig[NV_SENSORS0];
    sdHeadText[SDH_SENSORS1] = storedConfig[NV_SENSORS1];
    sdHeadText[SDH_SENSORS2] = storedConfig[NV_SENSORS2];
    sdHeadText[SDH_CONFIG_SETUP_BYTE0] = storedConfig[NV_CONFIG_SETUP_BYTE0];
    sdHeadText[SDH_CONFIG_SETUP_BYTE1] = storedConfig[NV_CONFIG_SETUP_BYTE1];
    sdHeadText[SDH_CONFIG_SETUP_BYTE2] = storedConfig[NV_CONFIG_SETUP_BYTE2];
    sdHeadText[SDH_CONFIG_SETUP_BYTE3] = storedConfig[NV_CONFIG_SETUP_BYTE3];
    // little endian in fw, but they want big endian in sw
    sdHeadText[SDH_TRIAL_CONFIG0] = storedConfig[NV_SD_TRIAL_CONFIG0];
    sdHeadText[SDH_TRIAL_CONFIG1] = storedConfig[NV_SD_TRIAL_CONFIG1];
    sdHeadText[SDH_BT_COMMS_BAUD_RATE] = storedConfig[NV_BT_COMMS_BAUD_RATE];
    sdHeadText[SDH_DERIVED_CHANNELS_0] = storedConfig[NV_DERIVED_CHANNELS_0];
    sdHeadText[SDH_DERIVED_CHANNELS_1] = storedConfig[NV_DERIVED_CHANNELS_1];
    sdHeadText[SDH_DERIVED_CHANNELS_2] = storedConfig[NV_DERIVED_CHANNELS_2];
    sdHeadText[SDH_DERIVED_CHANNELS_3] = storedConfig[NV_DERIVED_CHANNELS_3];
    sdHeadText[SDH_DERIVED_CHANNELS_4] = storedConfig[NV_DERIVED_CHANNELS_4];
    sdHeadText[SDH_DERIVED_CHANNELS_5] = storedConfig[NV_DERIVED_CHANNELS_5];
    sdHeadText[SDH_DERIVED_CHANNELS_6] = storedConfig[NV_DERIVED_CHANNELS_6];
    sdHeadText[SDH_DERIVED_CHANNELS_7] = storedConfig[NV_DERIVED_CHANNELS_7];

    // trivial
    sdHeadText[SDH_SHIMMERVERSION_BYTE_1] = DEVICE_VER >> 8;
    sdHeadText[SDH_SHIMMERVERSION_BYTE_1] = DEVICE_VER & 0xff;
    sdHeadText[SDH_FW_VERSION_TYPE_0] = FW_IDENTIFIER >> 8;
    sdHeadText[SDH_FW_VERSION_TYPE_1] = FW_IDENTIFIER & 0xff;
    sdHeadText[SDH_FW_VERSION_MAJOR_0] = FW_VER_MAJOR >> 8;
    sdHeadText[SDH_FW_VERSION_MAJOR_1] = FW_VER_MAJOR & 0xff;
    sdHeadText[SDH_FW_VERSION_MINOR] = FW_VER_MINOR;
    sdHeadText[SDH_FW_VERSION_INTERNAL] = FW_VER_REL;

    // exg
    sdHeadText[SDH_EXG_ADS1292R_1_CONFIG1] =
            storedConfig[NV_EXG_ADS1292R_1_CONFIG1];
    sdHeadText[SDH_EXG_ADS1292R_1_CONFIG2] =
            storedConfig[NV_EXG_ADS1292R_1_CONFIG2];
    sdHeadText[SDH_EXG_ADS1292R_1_LOFF] = storedConfig[NV_EXG_ADS1292R_1_LOFF];
    sdHeadText[SDH_EXG_ADS1292R_1_CH1SET] =
            storedConfig[NV_EXG_ADS1292R_1_CH1SET];
    sdHeadText[SDH_EXG_ADS1292R_1_CH2SET] =
            storedConfig[NV_EXG_ADS1292R_1_CH2SET];
    sdHeadText[SDH_EXG_ADS1292R_1_RLD_SENS] =
            storedConfig[NV_EXG_ADS1292R_1_RLD_SENS];
    sdHeadText[SDH_EXG_ADS1292R_1_LOFF_SENS] =
            storedConfig[NV_EXG_ADS1292R_1_LOFF_SENS];
    sdHeadText[SDH_EXG_ADS1292R_1_LOFF_STAT] =
            storedConfig[NV_EXG_ADS1292R_1_LOFF_STAT];
    sdHeadText[SDH_EXG_ADS1292R_1_RESP1] =
            storedConfig[NV_EXG_ADS1292R_1_RESP1];
    sdHeadText[SDH_EXG_ADS1292R_1_RESP2] =
            storedConfig[NV_EXG_ADS1292R_1_RESP2];
    sdHeadText[SDH_EXG_ADS1292R_2_CONFIG1] =
            storedConfig[NV_EXG_ADS1292R_2_CONFIG1];
    sdHeadText[SDH_EXG_ADS1292R_2_CONFIG2] =
            storedConfig[NV_EXG_ADS1292R_2_CONFIG2];
    sdHeadText[SDH_EXG_ADS1292R_2_LOFF] = storedConfig[NV_EXG_ADS1292R_2_LOFF];
    sdHeadText[SDH_EXG_ADS1292R_2_CH1SET] =
            storedConfig[NV_EXG_ADS1292R_2_CH1SET];
    sdHeadText[SDH_EXG_ADS1292R_2_CH2SET] =
            storedConfig[NV_EXG_ADS1292R_2_CH2SET];
    sdHeadText[SDH_EXG_ADS1292R_2_RLD_SENS] =
            storedConfig[NV_EXG_ADS1292R_2_RLD_SENS];
    sdHeadText[SDH_EXG_ADS1292R_2_LOFF_SENS] =
            storedConfig[NV_EXG_ADS1292R_2_LOFF_SENS];
    sdHeadText[SDH_EXG_ADS1292R_2_LOFF_STAT] =
            storedConfig[NV_EXG_ADS1292R_2_LOFF_STAT];
    sdHeadText[SDH_EXG_ADS1292R_2_RESP1] =
            storedConfig[NV_EXG_ADS1292R_2_RESP1];
    sdHeadText[SDH_EXG_ADS1292R_2_RESP2] =
            storedConfig[NV_EXG_ADS1292R_2_RESP2];
    // DMP related - start
    sdHeadText[SDH_SENSORS3] = storedConfig[NV_SENSORS3];
    sdHeadText[SDH_SENSORS4] = storedConfig[NV_SENSORS4];
    sdHeadText[SDH_CONFIG_SETUP_BYTE4] = storedConfig[NV_CONFIG_SETUP_BYTE4];
    sdHeadText[SDH_CONFIG_SETUP_BYTE5] = storedConfig[NV_CONFIG_SETUP_BYTE5];
    sdHeadText[SDH_CONFIG_SETUP_BYTE6] = storedConfig[NV_CONFIG_SETUP_BYTE6];
    // DMP related - end
    sdHeadText[SDH_MYTRIAL_ID] = storedConfig[NV_SD_MYTRIAL_ID];
    sdHeadText[SDH_NSHIMMER] = storedConfig[NV_SD_NSHIMMER];
    sdHeadText[SDH_EST_EXP_LEN_MSB] = storedConfig[NV_EST_EXP_LEN_MSB];
    sdHeadText[SDH_EST_EXP_LEN_LSB] = storedConfig[NV_EST_EXP_LEN_LSB];
    sdHeadText[SDH_MAX_EXP_LEN_MSB] = storedConfig[NV_MAX_EXP_LEN_MSB];
    sdHeadText[SDH_MAX_EXP_LEN_LSB] = storedConfig[NV_MAX_EXP_LEN_LSB];
    sdHeadText[SDH_RTC_DIFF_7] = *((uint8_t*) &rwcTimeDiff64);
    sdHeadText[SDH_RTC_DIFF_6] = *(((uint8_t*) &rwcTimeDiff64) + 1);
    sdHeadText[SDH_RTC_DIFF_5] = *(((uint8_t*) &rwcTimeDiff64) + 2);
    sdHeadText[SDH_RTC_DIFF_4] = *(((uint8_t*) &rwcTimeDiff64) + 3);
    sdHeadText[SDH_RTC_DIFF_3] = *(((uint8_t*) &rwcTimeDiff64) + 4);
    sdHeadText[SDH_RTC_DIFF_2] = *(((uint8_t*) &rwcTimeDiff64) + 5);
    sdHeadText[SDH_RTC_DIFF_1] = *(((uint8_t*) &rwcTimeDiff64) + 6);
    sdHeadText[SDH_RTC_DIFF_0] = *(((uint8_t*) &rwcTimeDiff64) + 7);

    memcpy(&sdHeadText[SDH_MAC_ADDR], &storedConfig[NV_MAC_ADDRESS], 6);
    memcpy(&sdHeadText[SDH_CONFIG_TIME_0], &storedConfig[NV_SD_CONFIG_TIME], 4);

    if (sdHeadText[SDH_SENSORS2] & SENSOR_BMPX80_PRESSURE)
    {
        BMPX80_init();

        memcpy(&sdHeadText[SDH_TEMP_PRES_CALIBRATION], bmpX80Calib,
        BMP180_CALIB_DATA_SIZE);
        if (bmpInUse == BMP280_IN_USE)
        {
            memcpy(&sdHeadText[BMP280_XTRA_CALIB_BYTES],
                   &(bmpX80Calib[BMP180_CALIB_DATA_SIZE]),
                   BMP280_CALIB_XTRA_BYTES);
        }

//		BMP180_getCalibCoeff(&sdHeadText[SDH_TEMP_PRES_CALIBRATION]);
//		P8OUT &= ~BIT4;         //set SW_I2C low to power off I2C chips
    }
//   memcpy(&sdHeadText[SDH_MPU9150_GYRO_CALIBRATION], &storedConfig[NV_MPU9150_GYRO_CALIBRATION], 21);
//   memcpy(&sdHeadText[SDH_LSM303DLHC_MAG_CALIBRATION], &storedConfig[NV_LSM303DLHC_MAG_CALIBRATION], 21);
//   memcpy(&sdHeadText[SDH_LSM303DLHC_ACCEL_CALIBRATION], &storedConfig[NV_LSM303DLHC_ACCEL_CALIBRATION], 21);
//   memcpy(&sdHeadText[SDH_A_ACCEL_CALIBRATION], &storedConfig[NV_A_ACCEL_CALIBRATION], 21);

    ShimmerCalibSyncFromDumpRamAll();
    memcpy(&sdHeadText[SDH_DAUGHTER_CARD_ID_BYTE0], daughtCardId, 3);
}

error_t SetBasedir()
{
    FILINFO fno;
    error_t res;
    uint16_t tmp_counter = 0;
    char lfn[_MAX_LFN + 1], *fname, *scout, *dash, dirnum[8];

    // first we'll make the shimmer mac address into a string
    if (strlen((char*) shimmerName) == 0)
    {
        // if name hasn't been assigned by user, use default (from Shimmer mac address)
        strcpy((char*) shimmerName, "Shimmer_XXXX");
        memcpy(shimmerName + 8, mac + 8, 4);
        memcpy(storedConfig + NV_SD_SHIMMER_NAME, shimmerName, 12);
    }

    if (strlen((char*) expIdName) == 0)
    { // if hasn't been assigned by user, use default
        strcpy((char*) expIdName, "default_exp");
        memcpy(storedConfig + NV_SD_EXP_ID_NAME, expIdName, 12);
    }

    if (strlen((char*) configTimeText) == 0) // if hasn't been assigned by user, use default
        strcpy((char*) configTimeText, "NoCfgTime");

    fno.lfname = lfn;
    fno.lfsize = sizeof(lfn);

    if ((res = f_opendir(&dir, "/data")))
    {
        if (res == FR_NO_PATH)               // we'll have to make /data first
            res = f_mkdir("/data");
        if (res)                             // in every case, we're toast
            return FAIL;

        // try one more time
        if ((res = f_opendir(&dir, "/data")))
            return FAIL;
    }

    strcpy((char*) expDirName, "data/");
    strcat((char*) expDirName, (char*) expIdName);
    strcat((char*) expDirName, "_");
    strcat((char*) expDirName, (char*) configTimeText);

    if ((res = f_opendir(&dir, (char*) expDirName)))
    {
        if (res == FR_NO_PATH) // we'll have to make the experiment folder first
            res = f_mkdir((char*) expDirName);
        if (res)                             // in every case, we're toast
            return FAIL;

        // try one more time
        if ((res = f_opendir(&dir, (char*) expDirName)))
            return FAIL;
    }

    dirCounter = 0;              // this might be the first log for this shimmer

    // file name format
    // shimmerName    as defined in sdlog.cfg
    // -              separator
    // 000
    // we want to create a new directory with a sequential run number each power-up/reset for each shimmer
    while (f_readdir(&dir, &fno) == FR_OK)
    {
        if (*fno.fname == 0)
            break;
        else if (fno.fattrib & AM_DIR)
        {
            fname = (*fno.lfname) ? fno.lfname : fno.fname;

            if (!strncmp(fname, (char*) shimmerName, strlen(fname) - 4))
            { // -4 because of the -000 etc.
                if ((scout = strchr(fname, '-')))
                { // if not, something is seriously wrong!
                    scout++;
                    while ((dash = strchr(scout, '-'))) // In case the shimmer name contains '-'
                        scout = dash + 1;
                    strcpy(dirnum, scout);
                    tmp_counter = atoi(dirnum);
                    if (tmp_counter >= dirCounter)
                    {
                        dirCounter = tmp_counter;
                        dirCounter++;   // start with next in numerical sequence
                    }
                }
                else
                    return FAIL;
            }
        }
    }

    // at this point, we have the id string and the counter, so we can make a directory name
    return SUCCESS;
}

error_t MakeBasedir()
{
    memset(dirName, 0, 64);
    char file_number_str[4];
    char dir_counter_text[4];
    UINT bw;
    uint32_t rwc_curr_time_l_32;

    ItoaWith0((uint64_t) dirCounter, (uint8_t*) dir_counter_text, 4);

    strcpy((char*) dirName, (char*) expDirName);
    strcat((char*) dirName, "/");
    strcat((char*) dirName, (char*) shimmerName);
    strcat((char*) dirName, "-");
    strcat((char*) dirName, dir_counter_text);

    if (fileBad = f_mkdir((char*) dirName))
        return FAIL;
    strcat((char*) dirName, "/");

    memset(fileName, 0, 64);
    fileNeedNew = 0;
    fileNextNum = 0;
    sdBuffLen = 0;
    ItoaWith0((uint64_t) fileNextNum++, (uint8_t*) file_number_str, 4);
    strcpy((char*) fileName, (char*) dirName);
    strcat((char*) fileName, file_number_str);
    fileBad = f_open(&dataFil, (char*) fileName, FA_WRITE | FA_CREATE_NEW);
    fileLastMin = fileLastHour = RTC_get64();
    firstTsFlag = 1;
//
//   rwc_curr_time_l_32 = fileLastHour & 0xffffffff;
//   sdHeadText[SDH_MY_LOCALTIME_5TH] = (fileLastHour >>32) & 0xff;
//   memcpy(&sdHeadText[SDH_MY_LOCALTIME_L], (uint8_t*)&rwc_curr_time_l_32, 4);
//   // Write header to file
//   fileBad = f_write(&dataFil, sdHeadText, SDHEAD_LEN, &bw);

    return SUCCESS;
}

void Timestamp0ToFirstFile()
{
    UINT bw;
    uint32_t my_local_time_long;
    my_local_time_long = firstTs & 0xffffffff;
    sdHeadText[SDH_MY_LOCALTIME_5TH] = (firstTs >> 32) & 0xff;
    ;
    memcpy(&sdHeadText[SDH_MY_LOCALTIME_L], (uint8_t*) &my_local_time_long, 4);
    fileBad = f_write(&dataFil, sdHeadText, SDHEAD_LEN, &bw);
}

// GSR
void GsrRange()
{
    // Fill the current active resistor into the upper two bits of the GSR value
    // if autorange is enabled, switch active resistor if required
    // If during resistor transition period use old ADC and resistor values
    // as determined by GSR_smoothTransition()

    uint8_t current_active_resistor = gsrActiveResistor;
    uint16_t ADC_val;

    // GSR channel will always be last ADC channel
    if (currentBuffer == 0)
    {
        ADC_val = *((uint16_t *) txBuff0 + (nbrAdcChans - 1 - 0) + 2); // - self - vbatt + dummy + ts
        if (((storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x0E) >> 1) == GSR_AUTORANGE)
        {
            if (GSR_smoothTransition(
                    &current_active_resistor,
                    *(uint16_t *) (storedConfig + NV_SAMPLING_RATE)))
            {
                ADC_val = lastGsrVal;
            }
            else
                gsrActiveResistor = GSR_controlRange(ADC_val,
                                                     gsrActiveResistor);
        }
        *((uint16_t *) txBuff0 + (nbrAdcChans - 1 - 0) + 2) = ADC_val
                | (current_active_resistor << 14);
    }
    else
    {
        ADC_val = *((uint16_t *) txBuff1 + (nbrAdcChans - 1 - 0) + 2);
        if (((storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x0E) >> 1) == GSR_AUTORANGE)
        {
            if (GSR_smoothTransition(
                    &current_active_resistor,
                    *(uint16_t *) (storedConfig + NV_SAMPLING_RATE)))
            {
                ADC_val = lastGsrVal;
            }
            else
                gsrActiveResistor = GSR_controlRange(ADC_val,
                                                     gsrActiveResistor);
        }
        *((uint16_t *) txBuff1 + (nbrAdcChans - 1 - 0) + 2) = ADC_val
                | (current_active_resistor << 14);
    }
    lastGsrVal = ADC_val;
}

void ItoaWith0(uint64_t num, uint8_t* buf, uint8_t len)
{ // len = actual len + 1 extra '\0' at the end
    memset(buf, 0, len--);
    while (len--)
    {
        buf[len] = '0' + num % 10;
        num /= 10;
    }
}
void ItoaNo0(uint64_t num, uint8_t* buf, uint8_t max_len)
{ // len = actual len + 1 extra '\0' at the end
    uint8_t idx, i_move;
    memset(buf, 0, max_len);
    if (!num)
        buf[0] = '0';
    for (idx = 0; (idx < max_len - 1) && (num > 0); idx++)
    {
        for (i_move = idx; i_move > 0; i_move--)
            buf[i_move] = buf[i_move - 1];
        buf[0] = '0' + num % 10;
        num /= 10;
    }
}

uint64_t Atol64(uint8_t* buf)
{
    uint8_t temp_str_1[6], temp_str_2[6], temp_str_3[6], temp_str_4[6];
    uint32_t temp_int_1, temp_int_2, temp_int_3, temp_int_4;
    uint64_t ret_val;

    memcpy(temp_str_1, buf, 5);
    memcpy(temp_str_2, buf + 5, 5);
    memcpy(temp_str_3, buf + 10, 5);
    memcpy(temp_str_4, buf + 15, 5);

    temp_str_1[5] = 0;
    temp_str_2[5] = 0;
    temp_str_3[5] = 0;
    temp_str_4[5] = 0;

    temp_int_1 = atol((char*) temp_str_1);
    temp_int_2 = atol((char*) temp_str_2);
    temp_int_3 = atol((char*) temp_str_3);
    temp_int_4 = atol((char*) temp_str_4);

    ret_val = temp_int_1;
    ret_val *= 100000;
    ret_val += temp_int_2;
    ret_val *= 100000;
    ret_val += temp_int_3;
    ret_val *= 100000;
    ret_val += temp_int_4;

    return ret_val;
}

void DockSdPowerCycle()
{
    __delay_cycles(2880000);   //wait 120ms (assuming 24MHz MCLK)
    P4OUT &= ~BIT2;            //SW_FLASH set low

    P5SEL &= ~(BIT4 + BIT5);
    P5OUT &= ~(BIT4 + BIT5);   //FLASH_SOMI and FLASH_SCLK set low
    P5DIR |= BIT4;             //FLASH_SOMI set as output
    P3SEL &= ~BIT7;
    P3OUT &= ~BIT7;            //FLASH_SIMO set low
    P4OUT &= ~BIT0;            //FLASH_CS_N set low
    P6OUT &= ~(BIT6 + BIT7);   //ADC6_FLASHDAT2 and ADC7_FLASHDAT1 set low
    P6DIR |= BIT6 + BIT7;      //ADC6_FLASHDAT2 and ADC7_FLASHDAT1 set as output

    //60ms as taken from TinyOS driver (SDP.nc powerCycle() function)
    __delay_cycles(2880000);   //wait 120ms (assuming 24MHz MCLK)

    P5DIR &= ~(BIT4 + BIT5);   //FLASH_SOMI and FLASH_SCLK set as input
    P3DIR &= ~BIT7;            //FLASH_SIMO set as input
    P4DIR &= ~BIT0;            //FLASH_CS_N set as input
    P6DIR &= ~(BIT6 + BIT7);   //ADC6_FLASHDAT2 and ADC7_FLASHDAT1 set as input

    P4OUT |= BIT2;             //SW_FLASH set high

    __delay_cycles(1200000);   //give SD card time to power back up
                               //50ms (assumint 24MHz MCLK)
    P6OUT &= ~BIT0;            //DETECT_N set low
}

void SdPowerOff(void)
{
    P4OUT &= ~BIT2;
}            //   SD power off
void SdPowerOn(void)
{
    P4OUT |= BIT2;
}            //   SD power on
uint8_t CheckSdInslot()
{
    //Check if card is inserted and enable interrupt for SD_DETECT_N
    if (!(P4IN & BIT1))
    {
        f_mount(0, &fatfs);
        set_sd_detect(1);
        return 1;
    }
    else
    {
        f_mount(0, NULL);
        set_sd_detect(0);
        return 0;
    }
}

void RwcCheck()
{
#if RTC_OFF
    rwcErrorEn = 0;
    rwcErrorFlash = 0;
#else
    rwcErrorEn = (storedConfig[NV_SD_TRIAL_CONFIG0] & SDH_RWCERROR_EN) ? 1 : 0;
    rwcErrorFlash = ((!rwcTimeDiff64) && rwcErrorEn) ? 1 : 0;
#endif
}

void StreamData()
{
    uint8_t adc_offset;
    uint8_t digi_offset;
    uint8_t DMP_read_fail;
    uint8_t tx_buff_temp[DATA_PACKET_SIZE];
    uint8_t *tx_buff_ptr;

    tx_buff_ptr = currentBuffer ? txBuff1 : txBuff0;

    adc_offset = 4;

    if (storedConfig[NV_SENSORS0] & SENSOR_GSR)
    {
        GsrRange();
    }

    // vbatt
    if (storedConfig[NV_SENSORS0] & SENSOR_A_ACCEL)
        *(uint16_t*) battVal = *((uint16_t *) (tx_buff_ptr + 6 + adc_offset)); // 2 dummy + ts + accel
    else
        *(uint16_t*) battVal = *((uint16_t *) (tx_buff_ptr + adc_offset)); // 2 dummy+ts
    if (vbattEn)
    {
    }
    else
    {      // shift data to cover vbatt bytes
        if (storedConfig[NV_SENSORS0] & SENSOR_A_ACCEL)
        {
            memcpy(tx_buff_temp, tx_buff_ptr + adc_offset + 8,
                   nbrAdcChans * 2 - 8);
            memcpy(tx_buff_ptr + adc_offset + 6, tx_buff_temp,
                   nbrAdcChans * 2 - 8);
        }
        else
        {
            memcpy(tx_buff_temp, tx_buff_ptr + adc_offset + 2,
                   nbrAdcChans * 2 - 2);
            memcpy(tx_buff_ptr + adc_offset, tx_buff_temp, nbrAdcChans * 2 - 2);
        }
    }

    digi_offset = (nbrAdcChans - (!vbattEn)) * 2 + adc_offset;

    // DMP related - start
    if (storedConfig[NV_CONFIG_SETUP_BYTE4] & MPU9150_MPL_DMP)
    {
        if (mpuIntTriggered)
        {
            mpuIntTriggered = 0;         // Clear the MPU data ready flag
            if (MPU_getDmpData())
            { // or if Shimmer sampling rate is higher > 100Hz
                DMP_read_fail = 1;
            }
            else
            {
                DMP_read_fail = 0;
            }
        }
        else
        {
            DMP_read_fail = 1;
        }
    }
    // DMP related - end

    if (currentBuffer)
    {

        // DMP related - start
        if ((storedConfig[NV_SENSORS0] & SENSOR_MPU9150_GYRO)
                && (storedConfig[NV_CONFIG_SETUP_BYTE4] & MPU9150_MPL_DMP))
        {
            if (DMP_read_fail)
                memcpy(txBuff1 + digi_offset, txBuff0 + digi_offset, 6);
            else
                MPL_getValue(PRINT_GYRO_RAW, txBuff1 + digi_offset);
            digi_offset += 6;
        }
        else if (storedConfig[NV_SENSORS0] & SENSOR_MPU9150_GYRO)
        {
            // DMP related - end
            MPU9150_getGyro(txBuff1 + digi_offset);
            digi_offset += 6;
        }
        if (storedConfig[NV_SENSORS1] & SENSOR_LSM303DLHC_ACCEL)
        {
            if (lsmInUse == LSM303DLHC_IN_USE)
            {
                LSM303DLHC_getAccel(txBuff1 + digi_offset);
            }
            else
            {
                LSM303AHTR_getAccel(txBuff1 + digi_offset);
            }
            digi_offset += 6;
        }
        if (storedConfig[NV_SENSORS0] & SENSOR_LSM303DLHC_MAG)
        {
            if (lsmInUse == LSM303DLHC_IN_USE)
            {
                LSM303DLHC_getMag(txBuff1 + digi_offset);
            }
            else
            {
                LSM303AHTR_getMag(txBuff1 + digi_offset);
            }
            digi_offset += 6;
        }
        // DMP related - start
        if ((storedConfig[NV_SENSORS2] & SENSOR_MPU9150_ACCEL)
                && (storedConfig[NV_CONFIG_SETUP_BYTE4] & MPU9150_MPL_DMP))
        {
            if (DMP_read_fail)
                memcpy(txBuff1 + digi_offset, txBuff0 + digi_offset, 6);
            else
                MPL_getValue(PRINT_ACCEL_RAW, txBuff1 + digi_offset);
            digi_offset += 6;
        }
        else if (storedConfig[NV_SENSORS2] & SENSOR_MPU9150_ACCEL)
        {
            // DMP related - end
            MPU9150_getAccel(txBuff1 + digi_offset);
            digi_offset += 6;
        }
        // DMP related - start
        if ((storedConfig[NV_SENSORS2] & SENSOR_MPU9150_MAG)
                && (storedConfig[NV_CONFIG_SETUP_BYTE4] & MPU9150_MPL_DMP))
        {
            if (DMP_read_fail)
                memcpy(txBuff1 + digi_offset, txBuff0 + digi_offset, 6);
            else
                MPL_getValue(PRINT_COMPASS_RAW, txBuff1 + digi_offset);
            digi_offset += 6;
        }
        else if (storedConfig[NV_SENSORS2] & SENSOR_MPU9150_MAG)
        {
            // DMP related - end
            if (preSampleMpuMag)
            {
                MPU9150_getMag(txBuff1 + digi_offset);
            }
            else if (!mpuMagCount--)
            {
                MPU9150_getMag(txBuff1 + digi_offset);
                mpuMagCount = mpuMagFreq;
                MPU9150_startMagMeasurement();
            }
            else
            {
                memcpy(txBuff1 + digi_offset, txBuff0 + digi_offset, 6);
            }
            digi_offset += 6;
        }
        if (storedConfig[NV_SENSORS2] & SENSOR_BMPX80_PRESSURE)
        {
            if (preSampleBmpPress)
            {
                if (sampleBmpTemp == sampleBmpTempFreq)
                {
                    sampleBmpTemp = 0;
#if PRES_TS_EN
                    bmpTempSampleTs = RTC_get64();
                    if((bmpTempSampleTs-bmpTempStartTs > bmpTempInterval) && (bmpTempStartTs > bmpPresStartTs))
                    {
                        BMPX80_getTemp(bmpTempCurrentVal);
                        if(abs(*(int16_t*)bmpTempCurrentVal - *(int16_t*)(bmpVal+2)) < 30)
                        { //wrong
                        }
                        else
                        memcpy(bmpVal, bmpTempCurrentVal, BMPX80_TEMP_BUFF_SIZE);
                    }
#else
                    BMPX80_getTemp(bmpVal);
#endif
                }
                else
                {
#if PRES_TS_EN
                    bmpPresSampleTs = RTC_get64();
                    if((bmpPresSampleTs-bmpPresStartTs > bmpPresInterval) && (bmpPresStartTs > bmpTempStartTs))
                    {
                        BMPX80_getPress(bmpPresCurrentVal);
                        if(abs(*(int16_t*)bmpPresCurrentVal - *(int16_t*)(bmpVal)) < 30)
                        { //wrong
                        }
                        else
                        memcpy(bmpVal+BMPX80_TEMP_BUFF_SIZE, bmpPresCurrentVal, BMPX80_PRESS_BUFF_SIZE);
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
                    if((bmpTempSampleTs-bmpTempStartTs > bmpTempInterval) && (bmpTempStartTs > bmpPresStartTs))
                    {
                        BMPX80_getTemp(bmpTempCurrentVal);
                        if(abs(*(int16_t*)bmpTempCurrentVal - *(int16_t*)(bmpVal+BMPX80_TEMP_BUFF_SIZE)) < 30)
                        { //wrong
                        }
                        else
                        memcpy(bmpVal, bmpTempCurrentVal, BMPX80_TEMP_BUFF_SIZE);
                    }
                    bmpPresStartTs = RTC_get64();
#else
                    BMPX80_getTemp(bmpVal);
#endif
                    BMPX80_startPressMeasurement(
                            (storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x30) >> 4);
                }
                else
                {
#if PRES_TS_EN
                    bmpPresSampleTs = RTC_get64();
                    if((bmpPresSampleTs-bmpPresStartTs > bmpPresInterval) && (bmpPresStartTs > bmpTempStartTs))
                    {
                        BMPX80_getPress(bmpPresCurrentVal);
                        if(abs(*(int16_t*)bmpPresCurrentVal - *(int16_t*)(bmpVal)) < 30)
                        { //wrong
                        }
                        else
                        memcpy(bmpVal+BMPX80_TEMP_BUFF_SIZE, bmpPresCurrentVal, BMPX80_PRESS_BUFF_SIZE);
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
                        BMPX80_startPressMeasurement(
                                (storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x30)
                                        >> 4);
                    }
                }
                bmpPressCount = bmpPressFreq;
            }
            memcpy(txBuff1 + digi_offset, bmpVal, BMPX80_PACKET_SIZE);
            digi_offset += BMPX80_PACKET_SIZE;
        }
        if (storedConfig[NV_SENSORS0] & SENSOR_EXG1_24BIT)
        {
            if (docked)
                memset(txBuff1 + digi_offset, 0, 7);
            else
                EXG_readData(0, 0, txBuff1 + digi_offset);
            digi_offset += 7;
        }
        else if (storedConfig[NV_SENSORS2] & SENSOR_EXG1_16BIT)
        {
            if (docked)
                memset(txBuff1 + digi_offset, 0, 5);
            else
                EXG_readData(0, 1, txBuff1 + digi_offset);
            digi_offset += 5;
        }
        if (storedConfig[NV_SENSORS0] & SENSOR_EXG2_24BIT)
        {
            if (docked)
                memset(txBuff1 + digi_offset, 0, 7);
            else
                EXG_readData(1, 0, txBuff1 + digi_offset);
            digi_offset += 7;
        }
        else if (storedConfig[NV_SENSORS2] & SENSOR_EXG2_16BIT)
        {
            if (docked)
                memset(txBuff1 + digi_offset, 0, 5);
            else
                EXG_readData(1, 1, txBuff1 + digi_offset);
            digi_offset += 5;
        }

        // DMP related - start
        if (storedConfig[NV_CONFIG_SETUP_BYTE4] & MPU9150_MPL_DMP)
        {
            if (storedConfig[NV_SENSORS3] & SENSOR_MPU9150_MPL_QUAT_6DOF)
            {
                if (DMP_read_fail)
                    memcpy(txBuff1 + digi_offset, txBuff0 + digi_offset, 16);
                else
                    MPL_getValue(PRINT_QUAT, txBuff1 + digi_offset);
                digi_offset += 16;
            }
            if (storedConfig[NV_SENSORS3] & SENSOR_MPU9150_MPL_QUAT_9DOF)
            {
                if (DMP_read_fail)
                    memcpy(txBuff1 + digi_offset, txBuff0 + digi_offset, 16);
                else
                    MPL_getValue(PRINT_QUAT_9DOF, txBuff1 + digi_offset);
                digi_offset += 16;
            }
            if (storedConfig[NV_SENSORS3] & SENSOR_MPU9150_MPL_EULER_6DOF)
            {
                if (DMP_read_fail)
                    memcpy(txBuff1 + digi_offset, txBuff0 + digi_offset, 12);
                else
                    MPL_getValue(PRINT_EULER, txBuff1 + digi_offset);
                digi_offset += 12;
            }
            if (storedConfig[NV_SENSORS3] & SENSOR_MPU9150_MPL_EULER_9DOF)
            {
                if (DMP_read_fail)
                    memcpy(txBuff1 + digi_offset, txBuff0 + digi_offset, 12);
                else
                    MPL_getValue(PRINT_EULER_9DOF, txBuff1 + digi_offset);
                digi_offset += 12;
            }
            if (storedConfig[NV_SENSORS3] & SENSOR_MPU9150_MPL_HEADING)
            {
                if (DMP_read_fail)
                    memcpy(txBuff1 + digi_offset, txBuff0 + digi_offset, 4);
                else
                    MPL_getValue(PRINT_HEADING, txBuff1 + digi_offset);
                digi_offset += 4;
            }
            if (storedConfig[NV_SENSORS2] & SENSOR_MPU9150_TEMP)
            {
                if (DMP_read_fail)
                    memcpy(txBuff1 + digi_offset, txBuff0 + digi_offset, 4);
                else
                    MPL_getValue(PRINT_TEMPERATURE, txBuff1 + digi_offset);
                digi_offset += 4;
            }
            if (storedConfig[NV_SENSORS3] & SENSOR_MPU9150_MPL_PEDOMETER)
            {
                if (DMP_read_fail)
                    memcpy(txBuff1 + digi_offset, txBuff0 + digi_offset, 8);
                else
                    MPL_getValue(PRINT_PEDOM, txBuff1 + digi_offset);
                digi_offset += 8;
            }
            if (storedConfig[NV_SENSORS3] & SENSOR_MPU9150_MPL_TAP)
            {
                if (DMP_read_fail)
                    memcpy(txBuff1 + digi_offset, txBuff0 + digi_offset, 1);
                else
                    MPL_getValue(PRINT_TAP_AND_DIR, txBuff1 + digi_offset);
                digi_offset += 1;
            }
            if (storedConfig[NV_SENSORS3] & SENSOR_MPU9150_MPL_MOTION_ORIENT)
            {
                if (DMP_read_fail)
                    memcpy(txBuff1 + digi_offset, txBuff0 + digi_offset, 1);
                else
                    MPL_getValue(PRINT_MOT_AND_ORIENT, txBuff1 + digi_offset);
                digi_offset += 1;
            }
            if (storedConfig[NV_SENSORS4] & SENSOR_MPU9150_GYRO_CAL)
            {
                if (DMP_read_fail)
                    memcpy(txBuff1 + digi_offset, txBuff0 + digi_offset, 12);
                else
                    MPL_getValue(PRINT_GYRO, txBuff1 + digi_offset);
                digi_offset += 12;
            }
            if (storedConfig[NV_SENSORS4] & SENSOR_MPU9150_ACCEL_CAL)
            {
                if (DMP_read_fail)
                    memcpy(txBuff1 + digi_offset, txBuff0 + digi_offset, 12);
                else
                    MPL_getValue(PRINT_ACCEL, txBuff1 + digi_offset);
                digi_offset += 12;
            }
            if (storedConfig[NV_SENSORS4] & SENSOR_MPU9150_MAG_CAL)
            {
                if (DMP_read_fail)
                    memcpy(txBuff1 + digi_offset, txBuff0 + digi_offset, 12);
                else
                    MPL_getValue(PRINT_COMPASS, txBuff1 + digi_offset);
                digi_offset += 12;
            }
            if (storedConfig[NV_SENSORS4] & SENSOR_MPU9150_MPL_QUAT_6DOF_RAW)
            {
                if (DMP_read_fail)
                    memcpy(txBuff1 + digi_offset, txBuff0 + digi_offset, 16);
                else
                    MPL_getValue(PRINT_QUAT_RAW, txBuff1 + digi_offset);
                digi_offset += 16;
            }
        }
        // DMP related - end

#if SKIP100MS
        if (Skip100ms())
        {
            firstTsFlag = 1; // has to re collect the init_ts
            return;
        }
#endif

#if TS_BYTE3
        memcpy(sdBuff + sdBuffLen, txBuff1 + 1, blockLen);
#else
        if(clockFreq == (float) MSP430_CLOCK)
        {
            memcpy(sdBuff+sdBuffLen, txBuff1+2, blockLen);
        }
        else
        {
            memcpy(sdBuff+sdBuffLen, txBuff1+4, blockLen);
        }
#endif
        sdBuffLen += blockLen;

        currentBuffer = 0;
    }
    else
    {

        // DMP related - start
        if ((storedConfig[NV_SENSORS0] & SENSOR_MPU9150_GYRO)
                && (storedConfig[NV_CONFIG_SETUP_BYTE4] & MPU9150_MPL_DMP))
        {
            if (DMP_read_fail)
                memcpy(txBuff0 + digi_offset, txBuff1 + digi_offset, 6);
            else
                MPL_getValue(PRINT_GYRO_RAW, txBuff0 + digi_offset);
            digi_offset += 6;
        }
        else if (storedConfig[NV_SENSORS0] & SENSOR_MPU9150_GYRO)
        {
            // DMP related - end
            MPU9150_getGyro(txBuff0 + digi_offset);
            digi_offset += 6;
        }
        if (storedConfig[NV_SENSORS1] & SENSOR_LSM303DLHC_ACCEL)
        {
            if (lsmInUse == LSM303DLHC_IN_USE)
            {
                LSM303DLHC_getAccel(txBuff0 + digi_offset);
            }
            else
            {
                LSM303AHTR_getAccel(txBuff0 + digi_offset);
            }
            digi_offset += 6;
        }
        if (storedConfig[NV_SENSORS0] & SENSOR_LSM303DLHC_MAG)
        {
            if (lsmInUse == LSM303DLHC_IN_USE)
            {
                LSM303DLHC_getMag(txBuff0 + digi_offset);
            }
            else
            {
                LSM303AHTR_getMag(txBuff0 + digi_offset);
            }
            digi_offset += 6;
        }
        // DMP related - start
        if ((storedConfig[NV_SENSORS2] & SENSOR_MPU9150_ACCEL)
                && (storedConfig[NV_CONFIG_SETUP_BYTE4] & MPU9150_MPL_DMP))
        {
            if (DMP_read_fail)
                memcpy(txBuff0 + digi_offset, txBuff1 + digi_offset, 6);
            else
                MPL_getValue(PRINT_ACCEL_RAW, txBuff0 + digi_offset);
            digi_offset += 6;
        }
        else if (storedConfig[NV_SENSORS2] & SENSOR_MPU9150_ACCEL)
        {
            // DMP related - end
            MPU9150_getAccel(txBuff0 + digi_offset);
            digi_offset += 6;
        }
        // DMP related - start
        if ((storedConfig[NV_SENSORS2] & SENSOR_MPU9150_MAG)
                && (storedConfig[NV_CONFIG_SETUP_BYTE4] & MPU9150_MPL_DMP))
        {
            if (DMP_read_fail)
                memcpy(txBuff0 + digi_offset, txBuff1 + digi_offset, 6);
            else
                MPL_getValue(PRINT_COMPASS_RAW, txBuff0 + digi_offset);
            digi_offset += 6;
        }
        else if (storedConfig[NV_SENSORS2] & SENSOR_MPU9150_MAG)
        {
            // DMP related - end
            if (preSampleMpuMag)
            {
                MPU9150_getMag(txBuff0 + digi_offset);
            }
            else if (!mpuMagCount--)
            {
                MPU9150_getMag(txBuff0 + digi_offset);
                mpuMagCount = mpuMagFreq;
                MPU9150_startMagMeasurement();
            }
            else
            {
                memcpy(txBuff0 + digi_offset, txBuff1 + digi_offset, 6);
            }
            digi_offset += 6;
        }
        if (storedConfig[NV_SENSORS2] & SENSOR_BMPX80_PRESSURE)
        {
            if (preSampleBmpPress)
            {
                if (sampleBmpTemp == sampleBmpTempFreq)
                {
                    sampleBmpTemp = 0;
#if PRES_TS_EN
                    bmpTempSampleTs = RTC_get64();
                    if((bmpTempSampleTs-bmpTempStartTs > bmpTempInterval) && (bmpTempStartTs > bmpPresStartTs))
                    {
                        BMPX80_getTemp(bmpTempCurrentVal);
                        if(abs(*(int16_t*)bmpTempCurrentVal - *(int16_t*)(bmpVal+BMPX80_TEMP_BUFF_SIZE)) < 30)
                        { //wrong
                        }
                        else
                        memcpy(bmpVal, bmpTempCurrentVal, BMPX80_TEMP_BUFF_SIZE);
                    }
#else
                    BMPX80_getTemp(bmpVal);
#endif
                }
                else
                {
#if PRES_TS_EN
                    bmpPresSampleTs = RTC_get64();
                    if((bmpPresSampleTs-bmpPresStartTs > bmpPresInterval) && (bmpPresStartTs > bmpTempStartTs))
                    {
                        BMPX80_getPress(bmpPresCurrentVal);
                        if(abs(*(int16_t*)bmpPresCurrentVal - *(int16_t*)(bmpVal)) < 30)
                        { //wrong
                        }
                        else
                        memcpy(bmpVal+BMPX80_TEMP_BUFF_SIZE, bmpPresCurrentVal, BMPX80_PRESS_BUFF_SIZE);
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
                    if((bmpTempSampleTs-bmpTempStartTs > bmpTempInterval) && (bmpTempStartTs > bmpPresStartTs))
                    {
                        BMPX80_getTemp(bmpTempCurrentVal);
                        if(abs(*(int16_t*)bmpTempCurrentVal - *(int16_t*)(bmpVal+BMPX80_TEMP_BUFF_SIZE)) < 30)
                        { //wrong
                        }
                        else
                        memcpy(bmpVal, bmpTempCurrentVal, BMPX80_TEMP_BUFF_SIZE);
                    }
                    bmpPresStartTs = RTC_get64();
#else

                    BMPX80_getTemp(bmpVal);

#endif
                    BMPX80_startPressMeasurement(
                            (storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x30) >> 4);
                }
                else
                {
#if PRES_TS_EN
                    bmpPresSampleTs = RTC_get64();
                    if((bmpPresSampleTs-bmpPresStartTs > bmpPresInterval) && (bmpPresStartTs > bmpTempStartTs))
                    {
                        BMPX80_getPress(bmpPresCurrentVal);
                        if(abs(*(int16_t*)bmpPresCurrentVal - *(int16_t*)(bmpVal)) < 30)
                        { //wrong
                        }
                        else
                        memcpy(bmpVal+BMPX80_TEMP_BUFF_SIZE, bmpPresCurrentVal, BMPX80_PRESS_BUFF_SIZE);
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
                        BMPX80_startPressMeasurement(
                                (storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x30)
                                        >> 4);
                    }
                }
                bmpPressCount = bmpPressFreq;
            }
            else
            {
            }
            memcpy(txBuff0 + digi_offset, bmpVal, BMPX80_PACKET_SIZE);
            digi_offset += 5;
        }
        if (storedConfig[NV_SENSORS0] & SENSOR_EXG1_24BIT)
        {
            if (docked)
                memset(txBuff0 + digi_offset, 0, 7);
            else
                EXG_readData(0, 0, txBuff0 + digi_offset);
            digi_offset += 7;
        }
        else if (storedConfig[NV_SENSORS2] & SENSOR_EXG1_16BIT)
        {
            if (docked)
                memset(txBuff0 + digi_offset, 0, 5);
            else
                EXG_readData(0, 1, txBuff0 + digi_offset);
            digi_offset += 5;
        }
        if (storedConfig[NV_SENSORS0] & SENSOR_EXG2_24BIT)
        {
            if (docked)
                memset(txBuff0 + digi_offset, 0, 7);
            else
                EXG_readData(1, 0, txBuff0 + digi_offset);
            digi_offset += 7;
        }
        else if (storedConfig[NV_SENSORS2] & SENSOR_EXG2_16BIT)
        {
            if (docked)
                memset(txBuff0 + digi_offset, 0, 5);
            else
                EXG_readData(1, 1, txBuff0 + digi_offset);
            digi_offset += 5;
        }

        // DMP related - start
        if (storedConfig[NV_CONFIG_SETUP_BYTE4] & MPU9150_MPL_DMP)
        {
            if (storedConfig[NV_SENSORS3] & SENSOR_MPU9150_MPL_QUAT_6DOF)
            {
                if (DMP_read_fail)
                    memcpy(txBuff0 + digi_offset, txBuff1 + digi_offset, 16);
                else
                    MPL_getValue(PRINT_QUAT, txBuff0 + digi_offset);
                digi_offset += 16;
            }
            if (storedConfig[NV_SENSORS3] & SENSOR_MPU9150_MPL_QUAT_9DOF)
            {
                if (DMP_read_fail)
                    memcpy(txBuff0 + digi_offset, txBuff1 + digi_offset, 16);
                else
                    MPL_getValue(PRINT_QUAT_9DOF, txBuff0 + digi_offset);
                digi_offset += 16;
            }
            if (storedConfig[NV_SENSORS3] & SENSOR_MPU9150_MPL_EULER_6DOF)
            {
                if (DMP_read_fail)
                    memcpy(txBuff0 + digi_offset, txBuff1 + digi_offset, 12);
                else
                    MPL_getValue(PRINT_EULER, txBuff0 + digi_offset);
                digi_offset += 12;
            }
            if (storedConfig[NV_SENSORS3] & SENSOR_MPU9150_MPL_EULER_9DOF)
            {
                if (DMP_read_fail)
                    memcpy(txBuff0 + digi_offset, txBuff1 + digi_offset, 12);
                else
                    MPL_getValue(PRINT_EULER_9DOF, txBuff0 + digi_offset);
                digi_offset += 12;
            }
            if (storedConfig[NV_SENSORS3] & SENSOR_MPU9150_MPL_HEADING)
            {
                if (DMP_read_fail)
                    memcpy(txBuff0 + digi_offset, txBuff1 + digi_offset, 4);
                else
                    MPL_getValue(PRINT_HEADING, txBuff0 + digi_offset);
                digi_offset += 4;
            }
            if (storedConfig[NV_SENSORS2] & SENSOR_MPU9150_TEMP)
            {
                if (DMP_read_fail)
                    memcpy(txBuff0 + digi_offset, txBuff1 + digi_offset, 4);
                else
                    MPL_getValue(PRINT_TEMPERATURE, txBuff0 + digi_offset);
                digi_offset += 4;
            }
            if (storedConfig[NV_SENSORS3] & SENSOR_MPU9150_MPL_PEDOMETER)
            {
                if (DMP_read_fail)
                    memcpy(txBuff0 + digi_offset, txBuff1 + digi_offset, 8);
                else
                    MPL_getValue(PRINT_PEDOM, txBuff0 + digi_offset);
                digi_offset += 8;
            }
            if (storedConfig[NV_SENSORS3] & SENSOR_MPU9150_MPL_TAP)
            {
                if (DMP_read_fail)
                    memcpy(txBuff0 + digi_offset, txBuff1 + digi_offset, 1);
                else
                    MPL_getValue(PRINT_TAP_AND_DIR, txBuff0 + digi_offset);
                digi_offset += 1;
            }
            if (storedConfig[NV_SENSORS3] & SENSOR_MPU9150_MPL_MOTION_ORIENT)
            {
                if (DMP_read_fail)
                    memcpy(txBuff0 + digi_offset, txBuff1 + digi_offset, 1);
                else
                    MPL_getValue(PRINT_MOT_AND_ORIENT, txBuff0 + digi_offset);
                digi_offset += 1;
            }
            if (storedConfig[NV_SENSORS4] & SENSOR_MPU9150_GYRO_CAL)
            {
                if (DMP_read_fail)
                    memcpy(txBuff0 + digi_offset, txBuff1 + digi_offset, 12);
                else
                    MPL_getValue(PRINT_GYRO, txBuff0 + digi_offset);
                digi_offset += 12;
            }
            if (storedConfig[NV_SENSORS4] & SENSOR_MPU9150_ACCEL_CAL)
            {
                if (DMP_read_fail)
                    memcpy(txBuff0 + digi_offset, txBuff1 + digi_offset, 12);
                else
                    MPL_getValue(PRINT_ACCEL, txBuff0 + digi_offset);
                digi_offset += 12;
            }
            if (storedConfig[NV_SENSORS4] & SENSOR_MPU9150_MAG_CAL)
            {
                if (DMP_read_fail)
                    memcpy(txBuff0 + digi_offset, txBuff1 + digi_offset, 12);
                else
                    MPL_getValue(PRINT_COMPASS, txBuff0 + digi_offset);
                digi_offset += 12;
            }
            if (storedConfig[NV_SENSORS4] & SENSOR_MPU9150_MPL_QUAT_6DOF_RAW)
            {
                if (DMP_read_fail)
                    memcpy(txBuff0 + digi_offset, txBuff1 + digi_offset, 16);
                else
                    MPL_getValue(PRINT_QUAT_RAW, txBuff0 + digi_offset);
                digi_offset += 16;
            }
        }
        // DMP related - end

#if SKIP100MS
        if (Skip100ms())
        {
            firstTsFlag = 1; // has to re collect the init_ts
            return;
        }
#endif

#if TS_BYTE3
        memcpy(sdBuff + sdBuffLen, txBuff0 + 1, blockLen);
#else
        if(clockFreq == (float) MSP430_CLOCK)
        {
            memcpy(sdBuff+sdBuffLen, txBuff0+2, blockLen);
        }
        else
        {
            memcpy(sdBuff+sdBuffLen, txBuff0+4, blockLen);
        }
#endif
        sdBuffLen += blockLen;
        currentBuffer = 1;
    }
    if (firstTsFlag == 2)
    {
        firstTsFlag = 3;
        Timestamp0ToFirstFile();
    }
}


/*
 * StreamData function modified for SDLog - tested working for ExG test signal
 * Removed references to PRES_TS_EN and DMP.
 *
void StreamData()
{
    uint8_t adc_offset;
    uint8_t* current_buffer_ptr;
    uint8_t *tx_buff_ptr;
    uint8_t tx_buff_temp[DATA_PACKET_SIZE];

    tx_buff_ptr = currentBuffer ? txBuff1 : txBuff0;

    adc_offset = 4;

//    uint8_t digi_offset = (nbrAdcChans * 2) + adc_offset;
    uint8_t digi_offset;

    if (storedConfig[NV_SENSORS0] & SENSOR_GSR)
    {
        GsrRange();
    }

    // vbatt
    if (storedConfig[NV_SENSORS0] & SENSOR_A_ACCEL)
        *(uint16_t*) battVal = *((uint16_t *) (tx_buff_ptr + 6 + adc_offset)); // 2 dummy + ts + accel
    else
        *(uint16_t*) battVal = *((uint16_t *) (tx_buff_ptr + adc_offset)); // 2 dummy+ts
    if (vbattEn)
    {
    }
    else
    {      // shift data to cover vbatt bytes
        if (storedConfig[NV_SENSORS0] & SENSOR_A_ACCEL)
        {
            memcpy(tx_buff_temp, tx_buff_ptr + adc_offset + 8,
                   nbrAdcChans * 2 - 8);
            memcpy(tx_buff_ptr + adc_offset + 6, tx_buff_temp,
                   nbrAdcChans * 2 - 8);
        }
        else
        {
            memcpy(tx_buff_temp, tx_buff_ptr + adc_offset + 2,
                   nbrAdcChans * 2 - 2);
            memcpy(tx_buff_ptr + adc_offset, tx_buff_temp, nbrAdcChans * 2 - 2);
        }
    }

    digi_offset = (nbrAdcChans - (!vbattEn)) * 2 + adc_offset;

    if (currentBuffer)
    {
        current_buffer_ptr = txBuff1;
    }
    else
    {
        current_buffer_ptr = txBuff0;
    }

    if (storedConfig[NV_SENSORS0] & SENSOR_MPU9150_GYRO)
    {
        MPU9150_getGyro(current_buffer_ptr + digi_offset);
        digi_offset += 6;
    }
    if (storedConfig[NV_SENSORS1] & SENSOR_LSM303DLHC_ACCEL)
    {
        if (lsmInUse == LSM303DLHC_IN_USE)
        {
            LSM303DLHC_getAccel(current_buffer_ptr + digi_offset);
        }
        else
        {
            LSM303AHTR_getAccel(current_buffer_ptr + digi_offset);
        }
        digi_offset += 6;
    }
    if (storedConfig[NV_SENSORS0] & SENSOR_LSM303DLHC_MAG)
    {
        if (lsmInUse == LSM303DLHC_IN_USE)
        {
            LSM303DLHC_getMag(current_buffer_ptr + digi_offset);
        }
        else
        {
            LSM303AHTR_getMag(current_buffer_ptr + digi_offset);
        }
        digi_offset += 6;
    }
    if (storedConfig[NV_SENSORS2] & SENSOR_MPU9150_ACCEL)
    {
        MPU9150_getAccel(current_buffer_ptr + digi_offset);
        digi_offset += 6;
    }
    if (storedConfig[NV_SENSORS2] & SENSOR_MPU9150_MAG)
    {
        if (preSampleMpuMag)
        {
            MPU9150_getMag(current_buffer_ptr + digi_offset);
        }
        else if (!mpuMagCount--)
        {
            MPU9150_getMag(current_buffer_ptr + digi_offset);
            mpuMagCount = mpuMagFreq;
            MPU9150_startMagMeasurement();
        }
        else
        {
            if (currentBuffer)
            {
                memcpy(txBuff1 + digi_offset, txBuff0 + digi_offset, 6);
            }
            else
            {
                memcpy(txBuff0 + digi_offset, txBuff1 + digi_offset, 6);
            }
        }
        digi_offset += 6;
    }
    if (storedConfig[NV_SENSORS2] & SENSOR_BMPX80_PRESSURE)
    {
        if (preSampleBmpPress)
        {
            if (sampleBmpTemp == sampleBmpTempFreq)
            {
                sampleBmpTemp = 0;
                BMPX80_getTemp(bmpVal);
            }
            else
            {
                BMPX80_getPress(bmpVal + BMPX80_TEMP_BUFF_SIZE);
                sampleBmpTemp++;
            }
        }
        else if (!bmpPressCount--)
        {
            if (sampleBmpTemp == sampleBmpTempFreq)
            {
                sampleBmpTemp = 0;
                BMPX80_getTemp(bmpVal);
                BMPX80_startPressMeasurement(
                        (storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x30) >> 4);
            }
            else
            {
                BMPX80_getPress(bmpVal + BMPX80_TEMP_BUFF_SIZE);

                if (++sampleBmpTemp == sampleBmpTempFreq)
                {
                    BMPX80_startTempMeasurement();
                }
                else
                {
                    BMPX80_startPressMeasurement(
                            (storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x30) >> 4);
                }
            }
            bmpPressCount = bmpPressFreq;
        }
        memcpy(current_buffer_ptr + digi_offset, bmpVal, BMPX80_PACKET_SIZE);
        digi_offset += BMPX80_PACKET_SIZE;
    }
    if (storedConfig[NV_SENSORS0] & SENSOR_EXG1_24BIT)
    {
        EXG_readData(0, 0, current_buffer_ptr + digi_offset);
        digi_offset += 7;
    }
    else if (storedConfig[NV_SENSORS2] & SENSOR_EXG1_16BIT)
    {
        EXG_readData(0, 1, current_buffer_ptr + digi_offset);
        digi_offset += 5;
    }
    if (storedConfig[NV_SENSORS0] & SENSOR_EXG2_24BIT)
    {
        EXG_readData(1, 0, current_buffer_ptr + digi_offset);
        if (!((*(current_buffer_ptr + digi_offset + 1) == 0x00)
                || (*(current_buffer_ptr + digi_offset + 1) == 0xff)))
            _NOP();
        digi_offset += 7;
    }
    else if (storedConfig[NV_SENSORS2] & SENSOR_EXG2_16BIT)
    {
        EXG_readData(1, 1, current_buffer_ptr + digi_offset);
        digi_offset += 5;
    }

    currentBuffer = !currentBuffer;

#if SKIP100MS
        if (Skip100ms())
        {
            firstTsFlag = 1; // has to re collect the init_ts
            return;
        }
#endif

#if TS_BYTE3

            memcpy(sdBuff + sdBuffLen, current_buffer_ptr + 1, digi_offset - 1);
            sdBuffLen += digi_offset - 1;
#else
            memcpy(sdBuff+sdBuffLen, current_buffer_ptr+2, digi_offset-2);
            sdBuffLen+=digi_offset-2;
#endif

            if (firstTsFlag == 2)
            {
                firstTsFlag = 3;
                Timestamp0ToFirstFile();
            }

}
*/

uint8_t Skip100ms()
{
    if (!skip100ms)
    {
        return 0;
    }
    if (skip100ms == 1)
    {
        startSensingTs64 = RTC_get64();
        skip100ms = 2;
    }
    else
    {
        uint64_t ts_64 = RTC_get64();
        if ((ts_64 - startSensingTs64) > 3277)
        { // 3277 - 100 ms, 1638 - 50 ms
            skip100ms = 0;
            return 0;
        }
    }
    return 1;
}

void RcStop()
{
    rcommStatus = 0;
    rcFirstOffsetRxed = 0;
    BtStop();
}

void RcStart()
{
    rcommStatus = 1;

    rcWindowC =
    SYNC_WINDOW_C < (estLen3 - SYNC_BOOT) ?
    SYNC_WINDOW_C :
                                            estLen3 - SYNC_BOOT;
    rcNodeReboot =
    SYNC_NODE_REBOOT < (estLen3 / SYNC_WINDOW_N) ?
    SYNC_NODE_REBOOT :
                                                   (estLen3 / SYNC_WINDOW_N);
    if (sdHeadText[SDH_TRIAL_CONFIG0] & SDH_IAMMASTER)
        firstOutlier = nodeSuccFull;
    else
        firstOutlier = 1;
    rcNodeR10Cnt = 0;
    maxLenCnt = 0;
    syncCnt = 0;
    syncThis = 0;
    syncSuccN = 0;

    myTimeDiffLongMin = 0;
    myTimeDiffLongFlagMin = 0;

    memset(myTimeDiffFlagArr, 0xff, SYNC_TRANS_IN_ONE_COMM);
    memset(myTimeDiffArr, 0, SYNC_TRANS_IN_ONE_COMM);
}

void Write2SD()
{
    wr2sd = 1;
    UINT bw;
    uint64_t file_td_h, file_td_m, local_time_64 = RTC_get64();
    //uint32_t rwc_curr_time_l_32;
    char file_number_str[4];

    if (sdBuffLen)
    {
        f_lseek(&dataFil, dataFil.fsize);      // seek to end of file, no spi op
        fileBad = f_write(&dataFil, sdBuff, sdBuffLen, &bw);    // Write to file

        file_td_h = local_time_64 - fileLastHour;
        file_td_m = local_time_64 - fileLastMin;

        //create a new file (from 000 up) every 1h
        if (file_td_h >= 117964800)
        {      // local_time_64 = 32768/s*3600s = 1h
            fileLastHour = fileLastMin = local_time_64;
            fileBad = f_close(&dataFil);
            //file number:from 000 up
            ItoaWith0((uint64_t) fileNextNum++, (uint8_t*) file_number_str, 4);
            strcpy((char*) fileName, (char*) dirName);
            strcat((char*) fileName, (char*) file_number_str);

            f_open(&dataFil, (char*) fileName, FA_WRITE | FA_CREATE_NEW);

            firstTsFlag = 1;
            streamDataInProc = 0;
            streamData = 0;
        }
        // sync data to SD card every 1 min
        else if (file_td_m >= 1966080)
        { //32768/s*60s = 1m
            fileLastMin = local_time_64;
            f_sync(&dataFil);
        }

        sdBuffLen = 0;
    }

    if (storedConfig[NV_SD_TRIAL_CONFIG0] & SDH_TIME_SYNC)
        PrepareSDBuffHead();

    wr2sd = 0;
}

void BattBlinkOn()
{
    SetBattVal();
    Board_ledOff(LED_GREEN0 + LED_YELLOW + LED_RED);
    switch (battStat)
    {
    case BATT_HIGH:
        Board_ledOn(LED_GREEN0);
        break;
    case BATT_MID:
        Board_ledOn(LED_YELLOW);
        break;
    case BATT_LOW:
        Board_ledOn(LED_RED);
        break;
    default:
        break;
    }
}


void SetBattDma()
{
    /* Ensure is off so all ADC12CTL0 and ADC12CTL1 fields can be modified */
    ADC12CTL0 &= ~ADC12ENC;

    /* ADC12SHT1_8 - SHT1 takes 256 ADC12CLK cycles
     * ADC12SHT0_8 - SHT0 takes 256 ADC12CLK cycles */
    ADC12CTL0 = ADC12SHT1_8 + ADC12SHT0_8;

    /* ADC12SHP 	 - Use sampling timer
     * ADC12CONSEQ_1 - ADC12OSC and single channel mode (SEQ_0) */
    ADC12CTL1 = ADC12SHP + ADC12CONSEQ_0;

    /* ADC12TCOFF - Turn off temperature sensor (saves power)
     * ADC12RES_2 - ADC12 resolution to 12bits
     * ADC12SR_L  - Sampling rate 50ksps */
    ADC12CTL2 = ADC12TCOFF + ADC12RES_2 + ADC12SR_L;

    if (storedConfig[NV_SENSORS1] & SENSOR_VBATT)
    {
        if (storedConfig[NV_SENSORS0] & SENSOR_A_ACCEL)
        {
            ADC12CTL1 += ADC12CSTARTADD_4;                /*Start with ADC12MEM4*/
            ADC12MCTL4 = ADC12INCH_2;
        }
        else
        {
            ADC12CTL1 += ADC12CSTARTADD_1;                /*Start with ADC12MEM1*/
            ADC12MCTL1 = ADC12INCH_2;
        }

    }
    else
    {
        ADC12CTL1 += ADC12CSTARTADD_0;                	  /*Start with ADC12MEM0*/
        ADC12MCTL0 = ADC12INCH_2;
    }

    DMACTL0 |= DMA0TSEL_24;            /*ADC12IFGx triggered*/
    DMACTL4 = DMARMWDIS;               /*Read-modify-write disable*/
    DMA0CTL &= ~DMAIFG;

    /* DMADT_0 		- Single transfer
     * DMADSTINCR_3 - Increment destination address
     * DMADSRINCR_3 - Increment source address
     * DMAIE 		- DMA interrupt enable
     * DMAEN        - DMA Enable */
    DMA0CTL = DMADT_1 + DMADSTINCR_3 + DMASRCINCR_3 + DMAIE; // + DMAEN;
    DMA0SZ = 1;                                              		 //DMA0 size

    if (storedConfig[NV_SENSORS1] & SENSOR_VBATT)
    {
        if (storedConfig[NV_SENSORS0] & SENSOR_A_ACCEL)
        {
            DMA0SA = (__SFR_FARPTR) (unsigned long) &ADC12MEM4;
        }
        else
        {
            DMA0SA = (__SFR_FARPTR) (unsigned long) &ADC12MEM1;
        }

    }
    else
    {
        DMA0SA = (__SFR_FARPTR) (unsigned long) &ADC12MEM0;
    }

	/*Writes a value to a 20-bit SFR register located at the given16/20-bit address*/
    DMA0DA = (__SFR_FARPTR) (unsigned long) battVal;

    DMA0_transferDoneFunction(&Dma0BatteryRead);

    ADC12IFG = 0;
    DMA0CTL |= DMAEN;
    ADC12CTL0 |= ADC12ON + ADC12ENC + ADC12SC;
}

void TB0Start()
{
    if (sampleTimerStatus + blinkStatus == 1)
    {
        if (clockFreq == (float) MSP430_CLOCK)
        {
            TB0CTL = TBSSEL_1 + MC_2; /*ACLK, continuous mode, clear TBR: + TBCLR*/
            TB0EX0 = 0;
        }
        else
        {
            TB0CTL = TBSSEL_0 + MC_2 + ID__8; /*use TBSSEL_0 for tcxo, ID__8:divider=8*/
            TB0EX0 = TBIDEX__8;               /*divider: 8*/
        }
    }
}

void TB0Stop()
{
    if (!sampleTimerStatus && !blinkStatus){
        TB0CTL = MC_0;
    }
}

void ClkAssignment()
{
    clk_45 = FreqProd(45);
    clk_55 = FreqProd(55);
    clk_75 = FreqProd(75);
    clk_85 = FreqProd(85);
    clk_90 = FreqProd(90);
    clk_115 = FreqProd(115);
    clk_135 = FreqProd(135);
    clk_145 = FreqProd(145);
    clk_120 = FreqProd(120);
    clk_105 = FreqProd(105);
    clk_165 = FreqProd(165);
    clk_195 = FreqProd(195);
    clk_225 = FreqProd(225);
    clk_255 = FreqProd(255);
    clk_285 = FreqProd(285);
    clk_375 = FreqProd(375);
    clk_405 = FreqProd(405);
    clk_1000 = FreqProd(1000);
    clk_2500 = FreqProd(2500);

    clk_90_45 = clk_90 - clk_45;
    clk_90_55 = clk_90 - clk_55;
    clk_90_75 = clk_90 - clk_75;
    clk_90_85 = clk_90 - clk_85;
    clk_135_90 = clk_135 - clk_90;
    clk_195_90 = clk_195 - clk_90;
    clk_255_90 = clk_255 - clk_90;
    clk_375_90 = clk_375 - clk_90;

}

uint16_t FreqProd(uint16_t num_in)
{ /*e.g. 7.5 ms: num_in=75, .25s:num_in=2500*/
    if (clockFreq == (float) MSP430_CLOCK)
    {
        return (uint16_t) ceil(
                ((float) MSP430_CLOCK) * ((float) (num_in / 10000.0)));
    }
    else
    {
        return (uint16_t) ceil(
                ((float) TCXO_CLOCK) * ((float) (num_in / 10000.0)));
    }
}

uint16_t FreqDiv(float num_in)
{
    if (clockFreq == (float) MSP430_CLOCK)
    {
        return (uint16_t) round(((float) MSP430_CLOCK) / num_in);
    }
    else
    {
        return (uint16_t) round(((float) TCXO_CLOCK) / num_in);
    }
}

/**
 *** Set the baud rate of the serial bus between the Bluetooth module and the MSP430
 *** 11 allowable options: 0=115.2K(default), 1=1200, 2=2400, 3=4800, 4=9600, 5=19.2K,
 *** 6=38.4K, 7=57.6K, 8=230.4K, 9=460.8K, 10=921.6K, else revert to default
 **/
void SetBtBaudRate(uint8_t rate)
{
    switch (rate)
    {
    case 1:
        BT_setTempBaudRate("1200");
        break;
    case 2:
        BT_setTempBaudRate("2400");
        break;
    case 3:
        BT_setTempBaudRate("4800");
        break;
    case 4:
        BT_setTempBaudRate("9600");
        break;
    case 5:
        BT_setTempBaudRate("19.2");
        break;
    case 6:
        BT_setTempBaudRate("38.4");
        break;
    case 7:
        BT_setTempBaudRate("57.6");
        break;
    case 8:
        BT_setTempBaudRate("230K");
        break;
    case 9:
        BT_setTempBaudRate("460K");
        break;
    case 10:
        BT_setTempBaudRate("921K");
        break;
    default:
        BT_setTempBaudRate("115K");
        break;
    }
}

// DMP related - start
uint8_t MPL_calibrateCB(void)
{
    return TaskSet(TASK_MPL_CALIBRATE);
}

void MPL_calibrateGo(void)
{
//   if((!(P1IN & BIT6)) && !sensing && !stopSensing && !startSensing){
    if ((!(P1IN & BIT6)) && !sensing && !(taskList & TASK_STOPSENSING)
            && !(taskList & TASK_STARTSENSING) && mplCalibrateInit)
    {
        uint8_t cnt, test_fail = 0;
        uint64_t button_ts_lcl, button_td_lcl;
        uint8_t sd_state;
        uint8_t stored_config_buffer[7];

        mplCalibrateInit = 0;

        // if button still pressed after timer callback timeout
        //msp430_get_clock_ms(&button_ts_lcl);
        button_ts_lcl = RTC_get64();
        button_td_lcl = button_ts_lcl - buttonPressTs64;

        if (button_td_lcl >= 90000)
        {

            mplCalibrating = 1;
            configuring = 1;       // led flag, in configuration period
            Board_ledOff(LED_GREEN1);

            // Wait until user releases button
            while (1)
            {
                msp430_delay_ms(100);
                if (P1IN & BIT6)
                    break;
            }

            msp430_delay_ms(5000);  // Delay to allow user to settle the Shimmer

            sd_state = P4OUT & BIT2;   // save previous SD power state
            P4OUT |= BIT2;            //SW_FLASH set high

            // Save original configuration and set defaults for calibration
            stored_config_buffer[0] = storedConfig[NV_SAMPLING_RATE];
            stored_config_buffer[1] = storedConfig[NV_SAMPLING_RATE + 1];
            stored_config_buffer[2] = storedConfig[NV_SENSORS3];
            stored_config_buffer[3] = storedConfig[NV_SENSORS4];
            stored_config_buffer[4] = storedConfig[NV_CONFIG_SETUP_BYTE4];
            stored_config_buffer[5] = storedConfig[NV_CONFIG_SETUP_BYTE5];
            stored_config_buffer[6] = storedConfig[NV_CONFIG_SETUP_BYTE6];

            storedConfig[NV_SAMPLING_RATE] = 0x80; //0x40;   // Sampling rate to 102.4Hz
            storedConfig[NV_SAMPLING_RATE + 1] = 0x02;            //0x01;
            storedConfig[NV_SENSORS3] = 0x80;   // MPL_Quat_6DOF = 1
            storedConfig[NV_SENSORS4] = 0xE0; // MPU9150_GYRO_CAL = MPU9150_ACCEL_CAL = MPU9150_MAG_CAL = 1
            // Note: for mag calibration to work, LPF must be enabled (set to next lowest value to half of Fs,
            // i.e., LPF = 42Hz for Fs = 100Hz)
            storedConfig[NV_CONFIG_SETUP_BYTE4] = 0x99; //  MPU9150_DMP = 1, MPU9150_LFP = 50Hz,
                                                        // MPU9150_MOT_CAL_CFG = Fast cal
            storedConfig[NV_CONFIG_SETUP_BYTE5] = 0x90; //  MPU9150_MPL_SAMPLING_RATE = 100Hz,
                                                        // MPU9150_MAG_SAMPLING_RATE = 100Hz
            storedConfig[NV_CONFIG_SETUP_BYTE6] = 0x78; //   MPL_sensor_fusion = 0, MPL_gyro_cal_tc = 1
                                                        // MPL_vect_comp_cal = MPL_mag_dist_cal = MPL_ENABLE = 1

            MPU_platformInit(&storedConfig[0]);
            msp430_delay_ms(1000);

            if (!(MPL_gyroCalibrate()))
            {
                // test success
                mplCalibratingMag = 1;
                if (!(MPL_magCalibrate()))
                {
                    // test success
                    P4OUT |= BIT2;            //SW_FLASH set high
                    if (MPU_saveCalibration())
                        test_fail = 1;
                    if (MPL_saveCalibrationBytes())
                        test_fail = 1;
                }
                else
                {
                    test_fail = 1;
                }
                mplCalibratingMag = 0;
            }
            else
            {
                test_fail = 1;
            }
            if (test_fail)
            {
                /*test fail*/
                Board_ledOn(LED_BLUE);
                Board_ledOff(LED_GREEN1);
                for (cnt = 0; cnt < 50; cnt++)
                {
                    Board_ledToggle(LED_BLUE);
                    Board_ledToggle(LED_GREEN1);
                    msp430_delay_ms(100);
                }
                test_fail = 0;
            }

            MPU9150_reset();
            /*Restore original configuration*/
            storedConfig[NV_SAMPLING_RATE] = stored_config_buffer[0];
            storedConfig[NV_SAMPLING_RATE + 1] = stored_config_buffer[1];
            storedConfig[NV_SENSORS3] = stored_config_buffer[2];
            storedConfig[NV_SENSORS4] = stored_config_buffer[3];
            storedConfig[NV_CONFIG_SETUP_BYTE4] = stored_config_buffer[4];
            storedConfig[NV_CONFIG_SETUP_BYTE5] = stored_config_buffer[5];
            storedConfig[NV_CONFIG_SETUP_BYTE6] = stored_config_buffer[6];

            Board_ledOff(LED_BLUE);
            Board_ledOff(LED_GREEN1);

            if (!sd_state)
            { /*restore SD state if it was previously off*/
                P4OUT &= ~BIT2; /*SW_FLASH set low*/
            }
            configuring = 0;       /*led flag, in configuration period*/
            mplCalibrating = 0;
        }
    }
    else
    {
        mplCalibrateInit = 0;
        mplCalibrating = 0;
    }
}

void SamplingClkAssignment(void)
{
    if (storedConfig[NV_SD_TRIAL_CONFIG1] & SDH_TCXO)
    {
        if (clockFreq != (float) TCXO_CLOCK)
        {
            clockFreq = (float) TCXO_CLOCK;
            P4OUT |= BIT6;
            _delay_cycles(2400000); /*100ms delay for tcxo*/
            P4SEL |= BIT7;
        }
    }
    else
    {
        if (clockFreq != (float) MSP430_CLOCK)
        {
            clockFreq = (float) MSP430_CLOCK;
            P4OUT &= ~BIT6;
            P4SEL &= ~BIT7;
        }
    }
}


 /*DMP related - end*/

// trap isr assignation - put all unused ISR vector here
//USCI_B0_VECTOR:i2c
//USCI_A0_VECTOR:dock/exp_uart
//USCI_A1_VECTOR:bt_uart
//RTC_VECTOR,
#pragma vector = WDT_VECTOR, SYSNMI_VECTOR, TIMER0_A0_VECTOR, \
    UNMI_VECTOR, USCI_B1_VECTOR,TIMER1_A1_VECTOR
__interrupt void TrapIsr(void)
{
     /*this is a trap ISR - check for the interrupt cause here by
     checking the interrupt flags, if necessary also clear the interrupt
     flag*/
}
