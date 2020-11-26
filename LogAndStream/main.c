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
 * @edited Sam O'Mahony
 * @date January, 2018
 */

/***********************************************************************************
 Data Buffer Format:
      Packet Type |TimeStamp|Achan1|Achan2| ... |AchanX|Dchan1|Dchan2| ... |DchanX|
 Byte:     0      |   1-3   |Achan1|Achan2| ... |AchanX|Dchan1|Dchan2| ... |DchanX|

 Log file Format:
       SD Header|TimeStamp1|Achan1 data1| ... |DchanX data1|TimeStamp2|Achan2 data2|...
 Byte:  0-255   |  256-258 |   259-260  | ... |            |          |            |...

 ***********************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include "msp430.h"
#include "shimmer3_common_source/shimmer_sd_include.h"
#include "shimmer_btsd.h"

void Init(void);
void CommTimerStart(void);
inline void CommTimerStop(void);
void BlinkTimerStart(void);
inline void BlinkTimerStop(void);
void SampleTimerStart(void);
inline void SampleTimerStop(void);
void StartLogging(void);
void StartStreaming(void);
inline void StopSensing(void);
inline uint16_t GetTA0(void);
uint8_t Dma0ConversionDone(void);
uint8_t Dma2ConversionDone(void);
void ProcessCommand(void);
void SendResponse(void);
void ConfigureChannels(void);
//void OpenFirstFile();
void Timestamp0ToFirstFile();
FRESULT WriteFile(uint8_t* text, WORD size);
void PrepareSDBuffHead(void);
void SetName();
void SetShimmerName();
void SetExpIdName();
void SetCfgTime();
error_t SetBasedir();
error_t MakeBasedir();
void FindError(uint8_t err, uint8_t *name);
inline void GsrRange(void);
void ItoaWith0(uint64_t num, uint8_t* buf, uint8_t len);
void ItoaNo0(uint64_t num, uint8_t* buf, uint8_t max_len);
uint64_t Atol64(uint8_t* buf);
void DockSdPowerCycle();
//void SetDocked(void);
//void SetUndocked(void);
void SetupDock(void);
uint8_t SendStatusByte();
void ReadSdConfiguration(void);
void PostSdCfg(void);
void SdPowerOff(void);
void SdPowerOn(void);
uint8_t CheckSdInslot();
//void SendRcommCommand();
void BtStop();
void BtStart();
void BtStartDone();
void StreamData();
void Write2SD();
void UpdateSdConfig();
void TB0Start();
void TB0Stop();
void ParseConfig();
void Config2SdHead(void);
void SetDefaultConfiguration(void);
void ChargeStatusTimerStart(void);
void ChargeStatusTimerStop(void);
void RstRcommVariables();
void ShimmerCalibSyncFromDumpRamAll(void);
void ShimmerCalibSyncFromDumpRamSingleSensor(uint8_t sensor);
void ShimmerCalibInitFromInfoAll(void);
void ShimmerCalibUpdateFromInfoAll(void);
void ShimmerCalibFromInfo(uint8_t sensor, uint8_t use_sys_time);
void CalibFromFileRead(uint8_t sensor);
void CalibDefault(uint8_t sensor);
void CalibAllAsDefault(void);
void CalibNewFile(uint8_t sensor, uint8_t range);
void CalibFromFile();
void CalibFromInfo(uint8_t sensor);
void CalibFromInfoAll();
void CalibAll();
uint8_t GetSdCfgFlag();
void SetSdCfgFlag(uint8_t flag);
uint8_t GetCalibFlag();
void SetCalibFlag(uint8_t flag);
uint8_t GetRamCalibFlag(void);
void SetRamCalibFlag(uint8_t flag);
void BattBlinkOn();
uint8_t Dma0BatteryRead(void);
void ReadBatt(void);
void ClkAssignment();
uint16_t FreqProd(uint16_t num_in);
uint16_t FreqDiv(float num_in);
void IniReadInfoMem(void);
void RwcCheck();
void Infomem2Names();
uint8_t CheckOnDefault();
void SetBtBaudRate(uint8_t rate);
void BtsdSelfcmd();
void SdInfoSync();
void ChangeBtBaudRateFunc();
uint8_t UartCheckCrc(uint8_t len);
void UartProcessCmd();
void UartSendRsp();
uint8_t UartCallback(uint8_t data);
inline uint8_t Skip65ms();
void SetStartSensing();
void ReadWriteSDTest(void);
void SamplingClkAssignment(void);

void i2cSlaveDiscover(void);
char i2cSlavePresent(char address);

uint8_t syncEnabled, btsdSelfCmd, lastLedGroup2, rwcErrorFlash, rwcErrorEn,
        sdErrorFlash;
uint32_t buttonPressTs, buttonReleaseTs, buttonLastReleaseTs;
uint64_t realTimeClock64, rwcTimeDiff64;
uint8_t realTimeClockText64[UINT64_LEN], realTimeDiffText64[UINT64_LEN],
        realTimeClockText40[14];
// 8 more bits for the rtc counter
uint64_t rwcConfigTime64, rwcCurrTime64;
uint32_t rwcCurrTime32h, rwcCurrTime32l;
uint32_t maxLen, maxLenCnt;

uint64_t buttonPressTs64, buttonReleaseTs64, buttonLastReleaseTs64;

uint8_t sdWrite, sdlogReady, btstreamReady, enableSdlog, enableBtstream;
uint8_t mac[14], macAddr[6], macAddrSet, fwInfo[7], ackStr[4],

storedConfig[NV_NUM_RWMEM_BYTES], channelContents[MAX_NUM_CHANNELS],
        configuring, nbrAdcChans, nbrDigiChans, infomemLength, calibRamLength;
uint16_t *adcStartPtr, infomemOffset, calibRamOffset;

uint8_t currentBuffer, processCommand, sendAck, inquiryResponse,
        samplingRateResponse, startSensing, stopSensing,
        aAccelCalibrationResponse, gyroCalibrationResponse,
        magCalibrationResponse, dAccelCalibrationResponse,
        allCalibrationResponse, configureChannels, streamData, sendResponse,
        sensing, btIsConnected, sdlogcfgUpdate, deviceVersionResponse,
        fwVersionResponse, bufferSizeResponse, uniqueSerialResponse,
        configSetupBytesResponse, lsm303dlhcAccelRangeResponse,
        lsm303dlhcMagGainResponse, lsm303dlhcMagSamplingRateResponse,
        lsm303dlhcAccelSamplingRateResponse, lsm303dlhcAccelLPModeResponse,
        lsm303dlhcAccelHRModeResponse, mpu9150GyroRangeResponse,
        mpu9150SamplingRateResponse, mpu9150AccelRangeResponse,
        sampleMpu9150Mag, mpu9150MagSensAdjValsResponse, sampleBmp180Press,
        bmp180CalibrationCoefficientsResponse,
        bmp280CalibrationCoefficientsResponse, bmpX80OversamplingRatioResponse,
        blinkLedResponse, gsrRangeResponse, internalExpPowerEnableResponse,
        exgRegsResponse, dcIdResponse, dcMemResponse, dockedResponse,
        trialConfigResponse, centerResponse, shimmerNameResponse, expIDResponse,
        nshimmerResponse, myIDResponse, configTimeResponse, dirResponse,
        btCommsBaudRateResponse, derivedChannelResponse, infomemResponse,
        rwcResponse, stopLogging, stopStreaming, isStreaming, isLogging,
        btVbattResponse, skip65ms, calibRamResponse;
uint8_t gAction;
uint8_t args[MAX_COMMAND_ARG_SIZE], waitingForArgs, waitingForArgsLength,
        argsSize;
uint8_t resPacket[RESPONSE_PACKET_SIZE + 2]; //+2 for crc checksum bytes;
uint8_t crcChecksum;

// file system vars
FATFS fatfs;         // File object
DIRS dir;               //Directory object
FIL fil;
/* make dir for SDLog files*/
uint8_t dirName[64], expDirName[32], sdHeadText[SDHEAD_LEN],
        bmpX80Calib[BMP280_CALIB_DATA_SIZE], bmpInUse, lsmInUse, eepromIsPresent,
        expIdName[MAX_CHARS], shimmerName[MAX_CHARS], daughtCardId[PAGE_SIZE],
        firstTsFlag, fileName[64],
        configTimeText[UINT32_LEN], //,exp_id_name[MAX_CHARS], shimmername[MAX_CHARS], centername[MAX_CHARS],
        txBuff0[DATA_PACKET_SIZE + 2], txBuff1[DATA_PACKET_SIZE + 2],
        btRxBuff[MAX_COMMAND_ARG_SIZE], *btRxExp, sdBuff[SDBUFF_SIZE], // todo: btRxBuff too long
        newDirFlag, dirLen, setConfig, calibRamFlag; //calibInFile, calibInInfo, calibInfoOverFile,
uint16_t sdBuffLen, blockLen, fileNum, dirCounter, blinkCnt10, blinkCnt20,
        blinkCnt50;
uint64_t firstTs, fileLastHour, fileLastMin;

volatile uint8_t fileBad, fileBadCnt, toggleLedRed;
static FRESULT ff_result;

/*battery evaluation vars*/
uint8_t battStat, battRead, battWait, battVal[3];
uint32_t battLastTs64, battInterval;
 /*routine comm*/
uint8_t myTimeDiff[5], getRcomm, rcommTimeout, rcommResp[RCT_SIZE],
        rcommCommand, rcommCurrentTry, rcommInterval, rcommSuccess,
        rcommWindowCenter, btPowerOn, sampleTimerStatus, blinkStatus;
uint8_t docked, setUndock, onUserButton, onSingleTouch, onDefault,

preSampleBmpPress, bmpPressFreq, bmpPressCount, sampleBmpTemp,

sampleBmpTempFreq, sampleBmp180Press, preSampleMpuMag, mpuMagFreq, mpuMagCount,
        sampleMpu9150Mag, waitpress, initializing;
uint16_t rcommCnt, rcommIntervalCenter, rcommSpecialInt;
float clockFreq;
uint16_t clk_10, clk_45, clk_50, clk_55, clk_75, clk_85, clk_90, clk_105,

clk_115, clk_120, clk_135, clk_145, clk_150, clk_165, clk_195, clk_225, clk_250,

clk_255, clk_285, clk_375, clk_405, clk_1000, clk_2500, clk_90_45, clk_90_55,
        clk_90_75, clk_90_85, clk_135_90, clk_195_90, clk_255_90, clk_375_90;

uint8_t bmpTempCurrentVal[BMPX80_TEMP_BUFF_SIZE],
        bmpPresCurrentVal[BMPX80_PRESS_BUFF_SIZE], bmpVal[BMPX80_PACKET_SIZE];

uint16_t bmpTempInterval, bmpPresInterval;
uint64_t bmpTempStartTs, bmpPresStartTs;
uint64_t bmpTempSampleTs, bmpPresSampleTs;
uint64_t startSensingTs64;

/*GSR*/
uint8_t gsrActiveResistor;
uint16_t lastGsrVal;
//ExG
uint8_t exgLength, exgChip, exgStartAddr;  /*, exgForcedOff;*/
/*Daughter card EEPROM*/
uint8_t dcMemLength;
uint16_t dcMemOffset;
/*BT baud rate*/
uint8_t changeBtBaudRate, sdInfoSyncDelayed;

uint8_t undockSimulate;

uint8_t uartDcMemLength, uartInfoMemLength;
uint16_t uartDcMemOffset, uartInfoMemOffset;

uint8_t uartSteps, uartArgSize, uartArg2Wait, uartCrc2Wait, uartAction;
uint8_t uartRxBuf[UART_DATA_LEN_MAX];
uint8_t uartProcessCmds, uartSendResponses;
uint8_t uartSendRspMac, uartSendRspVer, uartSendRspBat,

uartSendRspRtcConfigTime,
        uartSendRspCurrentTime, /* uartSendRspRct, uartSendRspRdt, */
        uartSendRspGdi, uartSendRspGdm, uartSendRspGim, uartSendRspAck,
        uartSendRspBadCmd, uartSendRspBadArg, uartSendRspBadCrc;
uint8_t uartRespBuf[UART_RSP_PACKET_SIZE];
uint8_t streamDataInProc;
char *dierecord;

/* variables used in i2c address scanning (used for BMP180/280 check) */
uint8_t slave_addresses[128];
uint8_t * slave_address_pointer;

bool SD_ERROR, SD_IN_SLOT;

/* variables used for delayed undock-start */
bool battCritical;
uint8_t undockEvent, wr2sd, battCriticalCount;
uint64_t time_newUnDockEvent;

/* Variable for SR47-4 (and later) to indicate ADS clock lines are tied */
uint8_t adsClockTied;

/* approx. 10% cutoff voltage - 3.65 Volts */
#define BATT_CUTOFF_3_65VOLTS   (2500)
#define TIMEOUT_100_MS          (3277)

/* should be 0 */
#define PRESS2UNDOCK   0
#define TESTDOCK       0
#define RTC_OFF        0
#define PRES_TS_EN     0
#define FACTORY_TEST   0

/* should be 1 */
#define TS_BYTE3       1
#define SKIP65MS       1

void main(void)
{
    initializing = 1;                      /* led flag, in initialisation period */
    Init();

    if (!configuring && !wr2sd)
    {
        SetupDock();
    }
    ReadBatt();

    /* Initialise Watchdog status timer */
    ChargeStatusTimerStart();

    while (1)
    {
        if (!processCommand)
            __bis_SR_register(LPM3_bits + GIE); /* ACLK remains active */

        if (setConfig)
        {
            setConfig = 0;
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
                    setConfig = 1;
                }
                undockEvent = 0;
            }
            else
            {
                setConfig = 1;
            }

            RwcCheck();
            sdErrorFlash =
                    (storedConfig[NV_SD_TRIAL_CONFIG0] & SDH_SDERROR_EN) ?
                            1 : 0;
        }

        ChangeBtBaudRateFunc();

        if (battRead)
        { /* use adc channel2 and mem4, read back battery status every certain period */
            battRead = 0;
            ReadBatt();
        }
        if (uartProcessCmds)
        {
            uartProcessCmds = 0;
            UartProcessCmd();
        }
        if (uartSendResponses)
        {
            uartSendResponses = 0;
            UartSendRsp();
        }
        if (processCommand)
        {
            processCommand = 0;
            ProcessCommand();
        }
        //debug
        if (configureChannels)
        {
            configureChannels = 0;
            ConfigureChannels();
        }
        if (sendResponse)
        {
            sendResponse = 0;
            SendResponse();
        }
        if (sampleMpu9150Mag)
        {
            sampleMpu9150Mag = 0;
            MPU9150_startMagMeasurement();
        }
        if (sampleBmp180Press)
        {
            sampleBmp180Press = 0;
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
                BMPX80_startPressMeasurement(
                        (storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x30) >> 4);
            }
        }
        if (streamData)
        {
            if (streamDataInProc)
            {
                streamData = 0;
                if ((!(enableSdlog && sdlogReady))
                        && (!(enableBtstream && btstreamReady)))
                {
                    btsdSelfCmd = 1;
                    stopSensing = 1;
                }
                if (newDirFlag && enableSdlog && sdlogReady)
                {
                    StartLogging();
                    //               Timestamp0ToFirstFile();
                }
                StreamData();
                // if sensor data buffer is large enough (about 1024 bytes),
                // write it to SDcard and clear the buffer
                if (enableSdlog && sdlogReady)
                {
                    if (sdBuffLen > SDBUFF_SIZE - blockLen)
                    {
                        sdWrite = 1;
                    }
                }
                if (btsdSelfCmd)
                {
                    BtsdSelfcmd();
                    btsdSelfCmd = 0;
                }
            }
            streamDataInProc = 0;
            streamData = 0;
        }
        if (sdlogcfgUpdate)
        {
            sdlogcfgUpdate = 0;
            if (!docked && !sensing && CheckSdInslot() && GetSdCfgFlag())
            {
                configuring = 1;
                IniReadInfoMem();
                UpdateSdConfig();
                SetSdCfgFlag(0);
                configuring = 0;
            }
        }
        if (startSensing)
        {
            startSensing = 0;
            configuring = 1;
            if (!sensing && !battCritical)
            {
                if (newDirFlag && enableSdlog && sdlogReady)
                {
                    StartLogging();
                    //isLogging = 1;
                }
                if ((enableSdlog && sdlogReady)
                        || (enableBtstream && btstreamReady))
                    StartStreaming();
                streamDataInProc = 0;
                //            if(enableSdlog && sdlogReady)
                //               Timestamp0ToFirstFile();
                if (docked)
                {
                    enableSdlog = 0;
                }
            }
            configuring = 0;
        }
        if (sdWrite)
        {
            sdWrite = 0;
            Write2SD();
        }
    }
}

void SetStartSensing()
{
    sdlogcfgUpdate = 1;
    startSensing = 1;
}

void Init(void)
{
    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
//     PM5CTL0 &= ~LOCKLPM5;

    // Needs to be here before LED functions are called:
    battCritical = FALSE;

    Board_init();
#if FACTORY_TEST
    Board_ledOn(LED_YELLOW);
    __delay_cycles(480000);
    Board_ledOff(LED_ALL);
    Board_ledOn(LED_GREEN0);
    __delay_cycles(480000);
    Board_ledOff(LED_ALL);
    Board_ledOn(LED_RED);
    __delay_cycles(480000);
    Board_ledOff(LED_ALL);
    Board_ledOn(LED_BLUE);
    __delay_cycles(480000);
    Board_ledOff(LED_ALL);
    Board_ledOn(LED_GREEN1);
#else
    Board_ledOn(LED_ALL);
#endif

    // Set Vcore to accommodate for max. allowed system speed
    SetVCore(PMMCOREV_3);

    // Start 32.768kHz XTAL as ACLK
    LFXT_Start(XT1DRIVE_0);

    // Start 24MHz XTAL as MCLK and SMCLK
    XT2_Start(XT2DRIVE_2); // XT2DRIVE_2 or XTDRIVE_3 for 24MHz (userguide section 5.4.7)
    UCSCTL4 |= SELS_5 + SELM_5;   // SMCLK=MCLK=XT2

    SFRIFG1 = 0;                  // clear interrupt flag register
    SFRIE1 |= OFIE;               // enable oscillator fault interrupt enable

    memset(myTimeDiff, 0xff, 5);
    realTimeClock64 = rwcTimeDiff64 = 0;
    memset(realTimeClockText64, '0', UINT64_LEN - 1);
    memset(realTimeDiffText64, '0', UINT64_LEN - 1);
    realTimeClockText64[UINT64_LEN - 1] = 0;
    realTimeDiffText64[UINT64_LEN - 1] = 0;
    realTimeClockText40[13] = 0;

    rwcConfigTime64 = 0;

    memset(txBuff0, 0, DATA_PACKET_SIZE);
    memset(txBuff1, 0, DATA_PACKET_SIZE);

    firstTsFlag = 0;
    sdlogcfgUpdate = 0;
    calibRamFlag = 0;
    skip65ms = 0;
    buttonLastReleaseTs64 = 0;
    rwcErrorEn = 0;
    rwcErrorFlash = 0;
    sdErrorFlash = 0;
    //calibInfoOverFile = 0;
    //calibInInfo = 0;
    //calibInFile = 0;
    crcChecksum = 0;
    undockSimulate = 1;
    sdInfoSyncDelayed = 0;
    lastLedGroup2 = 0;
    btsdSelfCmd = 0;
    toggleLedRed = 0;
    syncEnabled = 0;
    configuring = 0;
    battLastTs64 = 0;
    battRead = 1;
    battWait = 0;
    battStat = 0;
    blinkCnt10 = blinkCnt20 = blinkCnt50 = 0;
    fileBad = 0;
    fileBadCnt = 0;
    rcommSpecialInt = 0;
    rcommWindowCenter = RC_WINDOW_C;
    //rcommStatus = 0;
    onDefault = 0;
    onUserButton = 0;
    onSingleTouch = 0;
    setConfig = 0;
    setUndock = 0;
    getRcomm = 0;
    currentBuffer = 0;
    processCommand = 0;
    startSensing = 0;
    stopSensing = 0;
    stopLogging = 0;
    stopStreaming = 0;
    enableSdlog = 0;
    sdlogReady = 0;
    enableBtstream = 0;
    btstreamReady = 0;
    streamDataInProc = 0;

    // sd file system initiate.
    newDirFlag = 1;
    memset(sdBuff, 0, SDBUFF_SIZE);
    memset(storedConfig, 0xff, NV_NUM_RWMEM_BYTES);
    memset(sdHeadText, 0xff, SDHEAD_LEN);
    sdBuffLen = 0;

    clockFreq = (float) MSP430_CLOCK;
    ClkAssignment();

    sampleTimerStatus = 0;
    blinkStatus = 0;

    maxLenCnt = 0;
    sdWrite = 0;
    rcommTimeout = 0;
    btPowerOn = 0;
    //rcommCommand=ROUTINE_COMMUNICATION;
    lastGsrVal = 0;
    memset(mac, 0, sizeof(mac) / sizeof(mac[0]));

    sendAck = 0;
    inquiryResponse = 0;
    samplingRateResponse = 0;
    aAccelCalibrationResponse = 0;
    lsm303dlhcAccelRangeResponse = 0;
    lsm303dlhcMagGainResponse = 0;
    lsm303dlhcMagSamplingRateResponse = 0;
    dockedResponse = 0;
    trialConfigResponse = 0;
    centerResponse = 0;
    shimmerNameResponse = 0;
    expIDResponse = 0;
    configTimeResponse = 0;
    dirResponse = 0;
    nshimmerResponse = 0;
    myIDResponse = 0;
    lsm303dlhcAccelSamplingRateResponse = 0;
    lsm303dlhcAccelLPModeResponse = 0;
    lsm303dlhcAccelHRModeResponse = 0;
    mpu9150GyroRangeResponse = 0;
    bmp180CalibrationCoefficientsResponse = 0;
    bmp280CalibrationCoefficientsResponse = 0;
    bmpInUse = BMP180_IN_USE;
    lsmInUse = LSM303DLHC_IN_USE;
    eepromIsPresent = FALSE;
    mpu9150SamplingRateResponse = 0;
    mpu9150AccelRangeResponse = 0;
    bmpX80OversamplingRatioResponse = 0;
    internalExpPowerEnableResponse = 0;
    configSetupBytesResponse = 0;
    gyroCalibrationResponse = 0;
    magCalibrationResponse = 0;
    dAccelCalibrationResponse = 0;
    allCalibrationResponse = 0;
    deviceVersionResponse = 0;
    fwVersionResponse = 0;
    bufferSizeResponse = 0;
    uniqueSerialResponse = 0;
    mpu9150MagSensAdjValsResponse = 0;
    exgRegsResponse = 0;
    dcIdResponse = 0;
    dcMemResponse = 0;
    infomemResponse = 0;
    calibRamResponse = 0;
    rwcResponse = 0;
    btVbattResponse = 0;
    configureChannels = 0;
    sendResponse = 0;
    sampleMpu9150Mag = 0;
    sampleBmp180Press = 0;
    streamData = 0;
    sensing = 0;
    isStreaming = 0;
    isLogging = 0;
    btIsConnected = 0;
    waitingForArgs = 0;
    waitingForArgsLength = 0;
    argsSize = 0;
    nbrAdcChans = 0;
    nbrDigiChans = 0;
    preSampleMpuMag = 0;
    preSampleBmpPress = 0;
    sampleBmpTemp = 0;
    blinkLedResponse = 0;
    gsrRangeResponse = 0;
    lastGsrVal = 0;
    btCommsBaudRateResponse = 0;
    derivedChannelResponse = 0;
    changeBtBaudRate = 0xFF;   //indicates doesn't need changing
    uartSendRspAck = 0;
    uartSendRspMac = 0;
    uartSendRspVer = 0;
    uartSendRspBat = 0;
    uartSendRspRtcConfigTime = 0;
    uartSendRspCurrentTime = 0;
    uartSendRspGdi = 0;
    uartSendRspGdm = 0;
    uartSendRspGim = 0;
    uartSendRspBadCmd = 0;
    uartSendRspBadArg = 0;
    uartSendRspBadCrc = 0;
    uartSteps = 0;
    uartArgSize = 0;
    uartArg2Wait = 0;
    uartCrc2Wait = 0;
    uartProcessCmds = 0;
    uartSendResponses = 0;
    memset(mac, 0x00, sizeof(mac) / sizeof(mac[0]));
    memset(macAddr, 0x00, sizeof(macAddr) / sizeof(macAddr[0]));
    SD_ERROR = SD_IN_SLOT = FALSE;

    /* variables used for delayed undock-start */
    wr2sd = 0;
    undockEvent = 0;
    time_newUnDockEvent = 0;
    battCriticalCount = 0;

    /* Variable for SR47-4 (and later) to indicate ADS clock lines are tied */
    adsClockTied = 0;

    /* Globally enable interrupts */
    _enable_interrupts();

    UCA0_isrInit();
    UART_init(UartCallback);

    /* exp power */
    P3OUT &= ~BIT3;      //set low
    P3DIR |= BIT3;       //set as output
    P3SEL &= ~BIT3;

    SD_IN_SLOT = (bool) (!(P4IN & BIT1));

#if TESTDOCK
    if (0)
    {
#else
        if (P2IN & BIT3)
        {
#endif
        battInterval = BATT_INTERVAL_D;
        P2IES |= BIT3;       //look for falling edge
        docked = 1;
        UART_activate();
        sdlogReady = 0;
        if (CheckSdInslot())
        {
            DockSdPowerCycle();
        }
    }
    else
    {
        battInterval = BATT_INTERVAL;
        P2IES &= ~BIT3;      //look for rising edge
        docked = 0;
        UART_deactivate();
        P6OUT |= BIT0;       //   DETECT_N set to high
        if (CheckSdInslot())
        {
            sdlogReady = 1;
        }
        else
        {
            sdlogReady = 0;
        }
    }

    P2IFG &= ~BIT3;         //clear flag
    P2IE |= BIT3;           //enable interrupt

    *(storedConfig + NV_SD_SHIMMER_NAME) = '\0';
    *(storedConfig + NV_SD_EXP_ID_NAME) = '\0';
    *configTimeText = '\0';
    *fileName = '\0';

    P8OUT |= BIT4;             //set SW_I2C high to power on all I2C chips

    memset(btRxBuff, 0, 14);
    DMA2_init((uint16_t *) &UCA1RXBUF, (uint16_t *) btRxBuff, 14);
    DMA2_transferDoneFunction(&Dma2ConversionDone);
    btRxExp = BT_getExpResp();
    // =========== below initialize bt for the first time and get its MAC address ==========

    BT_init();
    BT_disableRemoteConfig(1);
    BT_setRadioMode(SLAVE_MODE);
    BT_setGetMacAddress(1);
#if FACTORY_TEST
    InfoMem_read((uint8_t *) 0, storedConfig, NV_NUM_RWMEM_BYTES);
    if (storedConfig[NV_BT_SET_PIN] == 0xAA)
    {
        BT_setAuthentication(4);
        BT_setName("Shimmer3");
        BT_setPIN("1234"); // requires BT restart to take effect
        storedConfig[NV_BT_SET_PIN] = 0xAB;
        InfoMem_write((uint8_t *) 0, storedConfig, NV_NUM_RWMEM_BYTES);
    }
#endif
    BtStart();
    uint8_t reset_cnt = 50;
    while (!btPowerOn)
    {
        _delay_cycles(2400000);
        if (!(reset_cnt--))
            PMMCTL0 = PMMPW + PMMSWPOR + (PMMCTL0 & 0x0003); // software POR reset
    }
    BT_setTempBaudRate("460K");
    _delay_cycles(2400000);
    // =========== above initialize bt for the first time and get its MAC address ==========

    //    RwcCheck();
    RTC_init(0);

    // enable switch1 interrupt
    Button_init();
    Button_interruptEnable();
    Button_waitpress();
    waitpress = 1;

    dierecord = (char *) 0x01A0A;

    Board_ledOff(LED_ALL);
    configuring = 1;
    BlinkTimerStart();

#if FACTORY_TEST
    /* i2c test */
    Board_ledOff(LED_ALL);
    Board_ledOn(LED_RED);
    i2cSlaveDiscover();
    Board_ledOff(LED_RED);
#else
    i2cSlaveDiscover();
#endif

    // Identify the presence of different sensors
    bmpInUse = (i2cSlavePresent(BMP280_ADDR)) ? BMP280_IN_USE : BMP180_IN_USE;
    lsmInUse = (i2cSlavePresent(LSM303AHTR_ACCEL_ADDR)) ?
            LSM303AHTR_IN_USE : LSM303DLHC_IN_USE;
    eepromIsPresent = (i2cSlavePresent(CAT24C16_ADDR)) ? TRUE : FALSE;

    ShimmerCalib_init();
    ShimmerCalibInitFromInfoAll();

    if (!docked && CheckSdInslot())
    {  //sd card ready to access
        if (!(P4OUT & BIT2))
        {
            // Hits here when undocked
            SdPowerOn();
        }
        if (GetSdCfgFlag())
        { // info > sdcard
            IniReadInfoMem();
            UpdateSdConfig();
            SetSdCfgFlag(0);
            if (ff_result != FR_OK)
            {
                sdlogReady = 0;
                SD_ERROR = TRUE;
            }
        }
        else
        {
            // Hits here when undocked
            ReadSdConfiguration();
        }

        if (ShimmerCalib_file2Ram())
        {
            //fail, i.e. no such file. use current DumpRam to generate a file
            ShimmerCalib_ram2File();
            IniReadInfoMem();
        }

        /* ====== SD READ/WRITE test ======*/
#if FACTORY_TEST
        ReadWriteSDTest();
#endif
        /* ====== SD READ/WRITE test ======*/

    }
    else
    {   // sd card not available
        IniReadInfoMem();
        //CalibFromInfoAll();
    }

    ShimmerCalibSyncFromDumpRamAll();

    RwcCheck();
    sdErrorFlash = (storedConfig[NV_SD_TRIAL_CONFIG0] & SDH_SDERROR_EN) ? 1 : 0;

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

    if (storedConfig[NV_BT_COMMS_BAUD_RATE] != 9)
    {
        SetBtBaudRate(storedConfig[NV_BT_COMMS_BAUD_RATE]);
        _delay_cycles(2400000);
    }

    CheckOnDefault();

    ConfigureChannels();
#if TS_BYTE3
    txBuff0[0] = txBuff1[0] = DATA_PACKET; //packet type
#else
            txBuff0[1] = txBuff1[1] = DATA_PACKET; //packet type
#endif

//    BlinkTimerStart();

    memset(daughtCardId, 0, PAGE_SIZE);

    CAT24C16_init();
    CAT24C16_read(0, PAGE_SIZE, daughtCardId);
    CAT24C16_powerOff();

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
    configuring = 0;

    Board_ledOff(LED_ALL);
}

void StartLogging(void)
{
    newDirFlag = 0;
    SdPowerOn();
    SetBasedir();
    MakeBasedir();
    Config2SdHead();
    fileNum = 0;
    sdBuffLen = 0;
    firstTsFlag = 1;
    //OpenFirstFile();
    ff_result = f_open(&fil, (char*) fileName, FA_WRITE | FA_CREATE_NEW);
    fileLastHour = fileLastMin = RTC_get64();
}

void StartStreaming(void)
{
    uint8_t i2c_en = 0;

    if (!sensing)
    {
#if SKIP65MS
        skip65ms = 1;
#endif
        if (docked)
        {
            UART_deactivate();
        }

        SamplingClkAssignment();

        ClkAssignment();
        TB0CTL = MC_0; // StopTb0()
        TB0Start();

        if (storedConfig[NV_CONFIG_SETUP_BYTE3] & EXP_POWER_ENABLE)
        { //EXT_RESET_N
            P3SEL &= ~BIT3;
            P3DIR |= BIT3;
            P3OUT |= BIT3;
        }
        if (storedConfig[NV_SENSORS0] & SENSOR_A_ACCEL)
        {
            P8REN &= ~BIT6;      //disable pull down resistor
            P8DIR |= BIT6;       //set as output
            P8SEL &= ~BIT6;  //analog accel being used so take out of sleep mode
            P8OUT |= BIT6;   //analog accel being used so take out of sleep mode
        }

        if (storedConfig[NV_SENSORS1] & SENSOR_STRAIN)
        {
            P2OUT |= BIT0;   //GPIO_INTERNAL1 set high
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

        if ((storedConfig[NV_SENSORS0] & SENSOR_MPU9150_GYRO)
                || (storedConfig[NV_SENSORS2] & SENSOR_MPU9150_ACCEL)
                || (storedConfig[NV_SENSORS2] & SENSOR_MPU9150_MAG))
        {
            MPU9150_init();
            i2c_en = 1;
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
            if (!i2c_en)
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
                i2c_en = 1;
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
            if (!i2c_en)
            {
                BMPX80_init();
            }
            if ((((storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x30) >> 4) == 0)
                    && (*(uint16_t *) (storedConfig + NV_SAMPLING_RATE)
                            >= ((bmpInUse == BMP180_IN_USE) ? clk_75 : clk_85)))
            {
                //max of approx. 3ms to sample everything + 4.5ms between starting press to data ready
                //so 7.5ms in total (246 ticks of 32768Hz clock = 7.507ms)(1919 ticks of 255765.625Hz clock = 7.5030ms)
                preSampleBmpPress = 1;
                bmpPressFreq = 1; //required for the calculation of sampleBmpTempFreq below
            }
            else if ((((storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x30) >> 4) == 1)
                    && (*(uint16_t *) (storedConfig + NV_SAMPLING_RATE)
                            >= ((bmpInUse == BMP180_IN_USE) ? clk_105 : clk_145)))
            {
                //max of approx. 3ms to sample everything + 7.5ms between starting press to data ready
                //so 10.5ms in total (345 ticks of 32768Hz clock = 10.529ms)(2686 ticks of 255765.625Hz clock = 10.5018ms)
                preSampleBmpPress = 1;
                bmpPressFreq = 1; //required for the calculation of sampleBmpTempFreq below
            }
            else if ((((storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x30) >> 4) == 2)
                    && (*(uint16_t *) (storedConfig + NV_SAMPLING_RATE)
                            >= ((bmpInUse == BMP180_IN_USE) ? clk_165 : clk_225)))
            {
                //max of approx. 3ms to sample everything + 13.5ms between starting press to data ready
                //so 16.5ms in total (541 ticks of 32768Hz clock = 16.510ms)(4221 ticks of 255765.625Hz clock = 16.5034ms)
                preSampleBmpPress = 1;
                bmpPressFreq = 1; //required for the calculation of sampleBmpTempFreq below
            }
            else if ((((storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x30) >> 4) == 3)
                    && (*(uint16_t *) (storedConfig + NV_SAMPLING_RATE)
                            >= ((bmpInUse == BMP180_IN_USE) ? clk_285 : clk_405)))
            {
                //max of approx. 3ms to sample everything + 25.5ms between starting press to data ready
                //so 28.5ms in total (934 ticks of 32768Hz clock = 28.503ms)(7290 ticks of 255765.625Hz clock = 28.5027ms)
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
                if (((storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x30) >> 4) == 0)
                {
                    bmpPressCount = bmpPressFreq = ((
                            (bmpInUse == BMP180_IN_USE) ? clk_45 : clk_55)
                            / *(uint16_t *) (storedConfig + NV_SAMPLING_RATE))
                            + 1;      //4.5
                }
                else if (((storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x30) >> 4)
                        == 1)
                {
                    bmpPressCount = bmpPressFreq = ((
                            (bmpInUse == BMP180_IN_USE) ? clk_75 : clk_115)
                            / *(uint16_t *) (storedConfig + NV_SAMPLING_RATE))
                            + 1;      //7.5
                }
                else if (((storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x30) >> 4)
                        == 2)
                {
                    bmpPressCount = bmpPressFreq = ((
                            (bmpInUse == BMP180_IN_USE) ? clk_135 : clk_195)
                            / *(uint16_t *) (storedConfig + NV_SAMPLING_RATE))
                            + 1;      //13.5
                }
                else
                {
                    bmpPressCount = bmpPressFreq = ((
                            (bmpInUse == BMP180_IN_USE) ? clk_255 : clk_375)
                            / *(uint16_t *) (storedConfig + NV_SAMPLING_RATE))
                            + 1;      //25.5
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
                sampleBmpTemp = sampleBmpTempFreq = (uint8_t) (FreqDiv(
                        *(uint16_t *) (storedConfig + NV_SAMPLING_RATE)) - 1)
                        / bmpPressFreq;
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
            /* probably turning on internal reference, so wait for it to settle */
            __delay_cycles(2400000);   /* 100ms (assuming 24MHz clock) */

            //probably setting the PGA gain so cancel the channel offset
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

        SampleTimerStart();
        if (storedConfig[NV_SD_TRIAL_CONFIG0] & SDH_TIME_SYNC)
            PrepareSDBuffHead();

        sensing = 1;
    }
}

inline void StopSensing(void)
{
    /*shut everything down*/
    configuring = 1;
    sensing = 0;
    isStreaming = 0;
    enableBtstream = 0;
    enableSdlog = 0;
    if (docked)
    {   /* if docked, cannot write to SD card any more*/
        DockSdPowerCycle();
    }
    else
    {
        newDirFlag = 1;
        if (isLogging)
        {
            Write2SD();
            f_close(&fil);
            _delay_cycles(1200000);
            SdPowerOff();
        }
    }
    isLogging = 0;

    SampleTimerStop();
    ADC_disable();
    DMA0_disable();

    P8OUT &= ~BIT6;
    //P8REN |= BIT6;      //enable pull down resistor
    //P8DIR &= ~BIT6;     //SW_ACCEL set as input

    P3OUT &= ~BIT3;     //set EXP_RESET_N low
    P2OUT &= ~BIT0;     //set GPIO_INTERNAL1 low (strain)

    if ((storedConfig[NV_SENSORS0] & SENSOR_MPU9150_GYRO)
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

    if ((storedConfig[NV_SENSORS0] & SENSOR_EXG1_24BIT)
            || (storedConfig[NV_SENSORS2] & SENSOR_EXG1_16BIT))
    {
        EXG_stop(0);     //probably not needed
    }
    if ((storedConfig[NV_SENSORS0] & SENSOR_EXG2_24BIT)
            || (storedConfig[NV_SENSORS2] & SENSOR_EXG2_16BIT))
    {
        EXG_stop(1);     //probably not needed
    }
    if (!docked
            && ((storedConfig[NV_SENSORS0] & SENSOR_EXG1_24BIT)
                    || (storedConfig[NV_SENSORS0] & SENSOR_EXG2_24BIT)
                    || (storedConfig[NV_SENSORS2] & SENSOR_EXG1_16BIT)
                    || (storedConfig[NV_SENSORS2] & SENSOR_EXG2_16BIT)))
    {
        EXG_powerOff();
    }
    if (docked)
        UART_activate();

    _delay_cycles(240000);
    I2C_Disable();
    P8OUT &= ~BIT4;         //set SW_I2C low to power off I2C chips
    streamData = 0;
    sdWrite = 0;
    sdBuffLen = 0;
    maxLenCnt = 0;
#if SKIP65MS
    skip65ms = 0;
#endif
    sampleBmp180Press = 0;
    sampleMpu9150Mag = 0;
    if (sdInfoSyncDelayed)
    {
        SdInfoSync();
    }
    _NOP();
    configuring = 0;
}

void BtsdSelfcmd()
{
    if (btIsConnected)
    {
        uint8_t selfcmd[4];         //i=0,
        selfcmd[0] = ACK_COMMAND_PROCESSED;
        selfcmd[1] = INSTREAM_CMD_RESPONSE;
        selfcmd[2] = STATUS_RESPONSE;
        selfcmd[3] = ((toggleLedRed & 0x01) << 7) + ((SD_ERROR & 0x01) << 6)
                + ((SD_IN_SLOT & 0x01) << 5) + ((isStreaming & 0x01) << 4)
                + ((isLogging & 0x01) << 3)
                + ((((rwcConfigTime64 > 0) ? 1 : 0) & 0x01) << 2)
                + ((sensing & 0x01) << 1) + (docked & 0x01);

        //while((!BT_write(selfcmd, 4)) && (i++<10)){
        //   _delay_cycles(240000);
        //}
        BT_append(selfcmd, 4);
    }
}

char i2cSlavePresent(char address)
{
    char isPresent = 0;

    //I2C pull-ups must be enabled (on rev4 boards) before initialising I2C bus
    P8OUT |= BIT4;          //enable I2C pull-ups by turning on SW_I2C
    P3OUT |= BIT3;
    __delay_cycles(48000);  //2ms

    //Source from SMCLK, which is running @ 24MHz. 4kHz desired BRCLK
    I2C_Master_Init(S_MCLK, 24000000, 400000);

    isPresent = TI_USCI_I2C_slave_present(address);

    P8OUT &= ~BIT4;         //disable I2C pull-ups by turning off SW_I2C
    __delay_cycles(120000); //5ms (assuming 24MHz MCLK) to ensure no writes pending
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
    uint8_t *channel_contents_ptr = channelContents;
    uint16_t mask = 0;

    nbrAdcChans = nbrDigiChans = 0;

    //Analog Accel
    if (storedConfig[NV_SENSORS0] & SENSOR_A_ACCEL)
    {
        *channel_contents_ptr++ = X_A_ACCEL;
        *channel_contents_ptr++ = Y_A_ACCEL;
        *channel_contents_ptr++ = Z_A_ACCEL;
        mask += MASK_A_ACCEL;
        nbrAdcChans += 3;
    }
    //Battery Voltage
    if (storedConfig[NV_SENSORS1] & SENSOR_VBATT)
    {
        *channel_contents_ptr++ = VBATT;
        mask += MASK_VBATT;
        nbrAdcChans++;
    }
    //External ADC channel A7
    if (storedConfig[NV_SENSORS0] & SENSOR_EXT_A7)
    {
        *channel_contents_ptr++ = EXTERNAL_ADC_7;
        mask += MASK_EXT_A7;
        nbrAdcChans++;
    }
    //External ADC channel A6
    if (storedConfig[NV_SENSORS0] & SENSOR_EXT_A6)
    {
        *channel_contents_ptr++ = EXTERNAL_ADC_6;
        mask += MASK_EXT_A6;
        nbrAdcChans++;
    }
    //External ADC channel A15
    if (storedConfig[NV_SENSORS1] & SENSOR_EXT_A15)
    {
        *channel_contents_ptr++ = EXTERNAL_ADC_15;
        mask += MASK_EXT_A15;
        nbrAdcChans++;
    }
    //Internal ADC channel A12
    if (storedConfig[NV_SENSORS1] & SENSOR_INT_A12)
    {
        *channel_contents_ptr++ = INTERNAL_ADC_12;
        mask += MASK_INT_A12;      //ppg
        nbrAdcChans++;
    }
    //Strain gauge
    if (storedConfig[NV_SENSORS1] & SENSOR_STRAIN)
    {
        *channel_contents_ptr++ = STRAIN_HIGH;
        *channel_contents_ptr++ = STRAIN_LOW;
        mask += MASK_STRAIN;
        nbrAdcChans += 2;
    }
    //Internal ADC channel A13
    if ((storedConfig[NV_SENSORS1] & SENSOR_INT_A13)
            && !(storedConfig[NV_SENSORS1] & SENSOR_STRAIN))
    {
        *channel_contents_ptr++ = INTERNAL_ADC_13;
        mask += MASK_INT_A13;
        nbrAdcChans++;
    }
    //Internal ADC channel A14
    if ((storedConfig[NV_SENSORS2] & SENSOR_INT_A14)
            && !(storedConfig[NV_SENSORS1] & SENSOR_STRAIN))
    {
        *channel_contents_ptr++ = INTERNAL_ADC_14;
        mask += MASK_INT_A14;
        nbrAdcChans++;
    }
    //Internal ADC channel A1
    if (storedConfig[NV_SENSORS0] & SENSOR_GSR)
    {
        //needs to be last analog channel
        *channel_contents_ptr++ = GSR_RAW;
        mask += MASK_INT_A1;
        nbrAdcChans++;
    }
    if (storedConfig[NV_SENSORS1] & SENSOR_INT_A1)
    {
        *channel_contents_ptr++ = INTERNAL_ADC_1;
        mask += MASK_INT_A1;
        nbrAdcChans++;
    }
    //Digi Gyro
    if (storedConfig[NV_SENSORS0] & SENSOR_MPU9150_GYRO)
    {
        *channel_contents_ptr++ = X_MPU9150_GYRO;
        *channel_contents_ptr++ = Y_MPU9150_GYRO;
        *channel_contents_ptr++ = Z_MPU9150_GYRO;
        nbrDigiChans += 3;
    }
    //Digi Accel
    if (storedConfig[NV_SENSORS1] & SENSOR_LSM303DLHC_ACCEL)
    {
        *channel_contents_ptr++ = X_LSM303DLHC_ACCEL;
        *channel_contents_ptr++ = Y_LSM303DLHC_ACCEL;
        *channel_contents_ptr++ = Z_LSM303DLHC_ACCEL;
        nbrDigiChans += 3;
    }
    //Mag
    if (storedConfig[NV_SENSORS0] & SENSOR_LSM303DLHC_MAG)
    {
        *channel_contents_ptr++ = X_LSM303DLHC_MAG;

        if (lsmInUse == LSM303DLHC_IN_USE)
        {
            *channel_contents_ptr++ = Z_LSM303DLHC_MAG;
            *channel_contents_ptr++ = Y_LSM303DLHC_MAG;
        }
        else
        {
            *channel_contents_ptr++ = Y_LSM303DLHC_MAG;
            *channel_contents_ptr++ = Z_LSM303DLHC_MAG;
        }
        nbrDigiChans += 3;
    }
    //Digi Accel
    if (storedConfig[NV_SENSORS2] & SENSOR_MPU9150_ACCEL)
    {
        *channel_contents_ptr++ = X_MPU9150_ACCEL;
        *channel_contents_ptr++ = Y_MPU9150_ACCEL;
        *channel_contents_ptr++ = Z_MPU9150_ACCEL;
        nbrDigiChans += 3;
    }
    //Digi Accel
    if (storedConfig[NV_SENSORS2] & SENSOR_MPU9150_MAG)
    {
        *channel_contents_ptr++ = X_MPU9150_MAG;
        *channel_contents_ptr++ = Y_MPU9150_MAG;
        *channel_contents_ptr++ = Z_MPU9150_MAG;
        nbrDigiChans += 3;
    }


#if TS_BYTE3
    uint8_t clk_offset = 3;
#else
    uint8_t clk_offset = 2;
#endif

    blockLen = (((nbrAdcChans + nbrDigiChans) * 2) + clk_offset);

    if (storedConfig[NV_SENSORS2] & SENSOR_BMPX80_PRESSURE)
    {
        *channel_contents_ptr++ = BMPX80_TEMP;
        *channel_contents_ptr++ = BMPX80_PRESSURE;
        nbrDigiChans += 2;   //PRES & TEMP, ON/OFF together
        blockLen += BMPX80_PACKET_SIZE;
    }

    if (storedConfig[NV_SENSORS0] & SENSOR_EXG1_24BIT)
    {
        *channel_contents_ptr++ = EXG_ADS1292R_1_STATUS;
        *channel_contents_ptr++ = EXG_ADS1292R_1_CH1_24BIT;
        *channel_contents_ptr++ = EXG_ADS1292R_1_CH2_24BIT;
        nbrDigiChans += 3;
        blockLen += 7;
    }
    else if (storedConfig[NV_SENSORS2] & SENSOR_EXG1_16BIT)
    {
        *channel_contents_ptr++ = EXG_ADS1292R_1_STATUS;
        *channel_contents_ptr++ = EXG_ADS1292R_1_CH1_16BIT;
        *channel_contents_ptr++ = EXG_ADS1292R_1_CH2_16BIT;
        nbrDigiChans += 3;
        blockLen += 5;
    }
    if (storedConfig[NV_SENSORS0] & SENSOR_EXG2_24BIT)
    {
        *channel_contents_ptr++ = EXG_ADS1292R_2_STATUS;
        *channel_contents_ptr++ = EXG_ADS1292R_2_CH1_24BIT;
        *channel_contents_ptr++ = EXG_ADS1292R_2_CH2_24BIT;
        nbrDigiChans += 3;
        blockLen += 7;
    }
    else if (storedConfig[NV_SENSORS2] & SENSOR_EXG2_16BIT)
    {
        *channel_contents_ptr++ = EXG_ADS1292R_2_STATUS;
        *channel_contents_ptr++ = EXG_ADS1292R_2_CH1_16BIT;
        *channel_contents_ptr++ = EXG_ADS1292R_2_CH2_16BIT;
        nbrDigiChans += 3;
        blockLen += 5;
    }

    if (mask)
    {
        adcStartPtr = ADC_init(mask);
        DMA0_transferDoneFunction(&Dma0ConversionDone);
        if (adcStartPtr)
        {
            DMA0_init(adcStartPtr, (uint16_t *) (txBuff0 + 4), nbrAdcChans);
        }
    }
}

uint8_t CheckOnDefault()
{
    onSingleTouch = 0;
    onUserButton = 0;
    onDefault = 0;
    if (storedConfig[NV_SD_TRIAL_CONFIG0] & SDH_SINGLETOUCH)
    {
        onSingleTouch = 1;
    }
    else if (storedConfig[NV_SD_TRIAL_CONFIG0] & SDH_USER_BUTTON_ENABLE)
    {
        onUserButton = 1;
    }
    else
    {
        onDefault = 1;
    }

    //    SD_ERROR = UpdateSdConfig() ? TRUE : FALSE;

    if (onDefault && sdlogReady && !sensing && (SD_ERROR == FALSE))
    {   //state == BTSD_IDLESD
        //startSensing = 1;
        SetStartSensing();
        //        enableSdlog = (SD_ERROR) ? 0 : 1;
        enableSdlog = 1;
        sensing = 1;
        BtsdSelfcmd();
        sensing = 0;
        return 1;
    }
    return 0;
}

// Switch SW1, BT_RTS and BT connect/disconnect
uint64_t button_two_release_td64, button_press_release_td64;
#pragma vector=PORT1_VECTOR
__interrupt void Port1_ISR(void)
{
    // Context save interrupt flag before calling interrupt vector.
    // Reading interrupt vector generator will automatically clear IFG flag
    //buttonsPressed = PAIFG & BUTTON_ALL;

    switch (__even_in_range(P1IV, P1IV_P1IFG7))
    {
    //BT Connect/Disconnect
    case P1IV_P1IFG0:   //BT Connect/Disconnect
        if (P1IN & BIT0)
        { //BT is connected
            P1IES |= BIT0; //look for falling edge
            BT_connectionInterrupt(1);
            btIsConnected = 1;
            BT_rst_MessageProgress();
            if (syncEnabled)
            {
                btstreamReady = 0;
                if (storedConfig[NV_SD_TRIAL_CONFIG0] & SDH_IAMMASTER)
                {
                    // center is waiting for 1 byte ROUTINE_COMMUNICATION(0xE0) from DMA2
                    DMA2_disable();
                    DMA2SZ = 1;
                    DMA2_enable();
                }
                /*else
                 // node sends (0xE0) and is waiting for 6 byte RC info from DMA2
                 if(rcommStatus){
                 //SendRcommCommand();
                 DMA2_disable();
                 DMA2SZ = 6;
                 DMA2_enable();
                 }*/
            }
            else
            {
                btstreamReady = 1;
                waitingForArgs = 0;
                waitingForArgsLength = 0;
                DMA2_disable();
                DMA2SZ = 1;
                DMA2_enable();
            }
        }
        else
        {             //BT is disconnected
            btstreamReady = 0;
            enableBtstream = 0;
            P1IES &= ~BIT0;   //look for rising edge
            btIsConnected = 0;
            BT_connectionInterrupt(0);
            BT_rst_MessageProgress();
            if ((changeBtBaudRate != 0xFF) && (!sensing))
            {
                __bic_SR_register_on_exit(LPM3_bits);
            }
        }
        sdlogcfgUpdate = 1;
        __bic_SR_register_on_exit(LPM3_bits);
        break;

        //BT RTS
    case P1IV_P1IFG3:
        if (P1IN & BIT3)
        {
            P1IES |= BIT3;   //look for falling edge
            BT_rtsInterrupt(1);
        }
        else
        {
            P1IES &= ~BIT3;   //look for rising edge
            BT_rtsInterrupt(0);   // when 0, can call sendNextChar();
        }
        break;

        //ExG chip2 data ready
    case P1IV_P1IFG4:
        EXG_dataReadyChip2();
        break;

        //BUTTON_SW1
    case P1IV_P1IFG6:
        if (!(P1IN & BIT6))
        {   //button pressed
            P1IES &= ~BIT6; //select rising edge trigger, wait for release
            buttonPressTs64 = RTC_get64();

            Board_ledOn(LED_GREEN0);

        }
        else
        { //button released
            P1IES |= BIT6; //select fall edge trigger, wait for press
            buttonReleaseTs64 = RTC_get64();
            button_press_release_td64 = buttonReleaseTs64 - buttonPressTs64;
            button_two_release_td64 = buttonReleaseTs64 - buttonLastReleaseTs64;
            if (button_press_release_td64 >= 163840)
            { // long button press: 5s
            }
            else if ((button_two_release_td64 > 16384) && !configuring
                    && !btIsConnected)
            { //  && (button_press_release_td64>327)
                buttonLastReleaseTs64 = buttonReleaseTs64;
#if PRESS2UNDOCK
                if(docked)
                {
                    docked = 0;
                }
                else
                {
                    docked = 1;
                }
                setConfig = 1;
                if(!sensing)
                __bic_SR_register_on_exit(LPM3_bits);
#else
                if (storedConfig[NV_SD_TRIAL_CONFIG0] & SDH_USER_BUTTON_ENABLE)
                {
                    // toggles sensing and refresh BT timers (for the centre)
                    if (sensing)
                    {
                        stopSensing = 1;
                        btsdSelfCmd = 1;
                    }
                    else
                    {
                        if (sdlogReady && (!SD_ERROR))
                        {
                            //startSensing = 1;
                            SetStartSensing();
                            enableSdlog = 1;
                        }

                        sensing = 1;
                        BtsdSelfcmd(); //only send cmd, not starting yet till START_BTSD_CMD received
                        sensing = 0;
                        __bic_SR_register_on_exit(LPM3_bits);
                    }
                }
#endif
            }
            else
                _NOP();
        }
        break;
    default:
        break;
    }
}

#pragma vector=PORT2_VECTOR
__interrupt void Port2_ISR(void)
{
    // Context save interrupt flag before calling interrupt vector.
    // Reading interrupt vector generator will automatically clear IFG flag
    //buttonsPressed = PAIFG & BUTTON_ALL;

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
        {
            //card not inserted
            P2IES |= BIT1;   //look for falling edge
        }
        else
        {
            //card inserted
            P2IES &= ~BIT1;   //look for rising edge
        }
        break;

        //dock_detect_N
    case P2IV_P2IFG3:
        setConfig = 1;
        if (!undockEvent)
        {
            if (!(P2IN & BIT3)) // undocked
            {
                undockEvent = 1;
                battCritical = FALSE;
                time_newUnDockEvent = RTC_get64();
            }
            //see slaa513 for example using multiple time bases on a single timer module
            if (!sensing)
                __bic_SR_register_on_exit(LPM3_bits);
        }
        if (P2IN & BIT3)
        {
            P2IES |= BIT3;       //look for falling edge
            sdlogReady = 0;
        }
        else
        {
            P2IES &= ~BIT3;      //look for rising edge
            if (CheckSdInslot() && !SD_ERROR)
            {
                sdlogReady = 1;
            }
            else
            {
                sdlogReady = 0;
            }
        }
        break;
        // Default case
    default:
        break;
    }
}

//Timer2:
//ccr1: for blink timer
void CommTimerStart(void)
{
    RstRcommVariables();
    TA0CTL = TASSEL_1 + MC_2 + TACLR;    //ACLK, continuous mode, clear TAR
    TA0CCTL1 = CCIE;
    TA0CCR1 = GetTA0() + 16384;
}

inline void CommTimerStop(void)
{
    TB0CTL = MC_0; // StopTb0()
    //rcommStatus=0;
    TA0CCTL1 &= ~CCIE;
}

inline uint16_t GetTA0(void)
{
    register uint16_t t0, t1;
    uint8_t ie;
    if (ie = (__get_SR_register() & 0x0008))   //interrupts enabled?
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

#pragma vector=TIMER0_A1_VECTOR
__interrupt void TIMER0_A1_ISR(void)
{
    switch (__even_in_range(TA0IV, 14))
    {
    case 0:
        break;                         // No interrupt
    case 2:
        break;                         // TA0CCR1
    case 4:
        break;                         // TA0CCR2 not used
    case 6:
        break;                         // Reserved
    case 8:
        break;                         // Reserved
    case 10:
        break;                         // Reserved
    case 12:
        break;                         // Reserved
    case 14:
        break;                         // TAIFG overflow handler
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

//void SendRcommCommand(){
//   memset(rcommResp,0,RCT_SIZE);
//   getRcomm=1;
//   BT_write(&rcommCommand, 0x01);
//}

// Blink Timer
//USING TB0 with CCR1
void BlinkTimerStart(void)
{
    blinkStatus = 1;
    TB0Start();
    TB0CCTL3 = CCIE;
    // clk_1000 = 100.0 ms = 0.1s
    TB0CCR3 = GetTB0() + clk_1000;
}
inline void BlinkTimerStop(void)
{
    blinkStatus = 0;
    TB0CCTL3 &= ~CCIE;
}

#pragma vector=TIMER0_B1_VECTOR
__interrupt void TIMER0_B1_ISR(void)
{
    switch (__even_in_range(TB0IV, 14))
    {
    case 0:
        break;                           // No interrupt
    case 2:                                 // TB0CCR1
        //MPU9150 mag
        TB0CCR1 = GetTB0() + *(uint16_t *) (storedConfig + NV_SAMPLING_RATE);
        sampleMpu9150Mag = 1;
        __bic_SR_register_on_exit(LPM3_bits);
        break;
    case 4:                                 // TB0CCR2
        //Bmp180 press
        TB0CCR2 = GetTB0() + *(uint16_t *) (storedConfig + NV_SAMPLING_RATE);
        sampleBmp180Press = 1;
        __bic_SR_register_on_exit(LPM3_bits);
        break;
    case 6:                                 // TB0CCR3
        if (!(WDTCTL & WDTHOLD))
        {
            // Reset Watchdog timer
            WDTCTL = WDTPW + WDTSSEL__ACLK + WDTCNTCL + WDTIS_4;
        }

        // for LED blink usage details, please check shimmer user manual
        // clk_1000 = 100.0 ms = 0.1s
        TB0CCR3 += clk_1000;
        if (blinkCnt50++ == 49)
            blinkCnt50 = 0;

        if (blinkCnt20++ == 19)
        {
            blinkCnt20 = 0;
        }

        if (blinkCnt20 % 10 == 0)
        {
            if (sensing && maxLen)
            {
                if (maxLenCnt < maxLen)
                    maxLenCnt++;
                else
                {
                    stopSensing = 1;
                    btsdSelfCmd = 1;
                    maxLenCnt = 0;
                }
            }
        }

        uint64_t batt_td, batt_my_local_time_64;
        batt_my_local_time_64 = RTC_get64();
        batt_td = batt_my_local_time_64 - battLastTs64;

        if (batt_td > battInterval)
        {              //10 mins = 19660800
            battRead = 1;
            battLastTs64 = batt_my_local_time_64;
        }

        if (!initializing)
            if (battRead)
                __bic_SR_register_on_exit(LPM3_bits);

        if (blinkStatus)
        {
            // below are settings for green0, yellow and red leds, battery charge status
            if (toggleLedRed)
            {
                Board_ledOff(LED_GREEN0 + LED_YELLOW);
                Board_ledOn(LED_RED);
                if (P2IN & BIT3)
                {
                    BattBlinkOn();
                }
                else
                {
                    if (!blinkCnt50)
                        BattBlinkOn();
                }
            }
            else
            {
                Board_ledOff(LED_GREEN0 + LED_YELLOW + LED_RED);
                if (P2IN & BIT3)
                {
                    BattBlinkOn();
                }
                else
                {
                    if (!blinkCnt50)
                    {
                        BattBlinkOn();
                    }
                }
            }

            // code for keeping LED_GREEN0 on when user button pressed
            if ((!(P1IN & BIT6)))
            {
                Board_ledOn(LED_GREEN0);
            }

            // below are settings for green1, blue, yellow and red leds

            if (!docked && (SD_ERROR || !SD_IN_SLOT) && sdErrorFlash)
            {   // bad file = yellow/red alternating
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
                    Board_ledOn(LED_YELLOW);
                    Board_ledOff(LED_RED);
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
                if (syncEnabled)
                { // sync not implemented yet
                    if (!sensing)
                    { //standby or configuring
                        if (initializing || configuring)
                        { //configuring
                            if (!(P1OUT & BIT1))
                                Board_ledOn(LED_GREEN1);
                            else
                                Board_ledOff(LED_GREEN1);
                        }
                        else
                        {                       //standby
                            if (!blinkCnt20)
                                Board_ledOn(LED_GREEN1);
                            else
                                Board_ledOff(LED_GREEN1);
                        }
                    }
                    else
                    {                           //sensing
                        if (!(blinkCnt20 % 10))
                        {
                            if (!(P1OUT & BIT1))
                                Board_ledOn(LED_GREEN1);
                            else
                                Board_ledOff(LED_GREEN1);
                        }
                    }
                    // good file - blue:
                    if (btPowerOn)
                    {
                        Board_ledOn(LED_BLUE);
                    }
                    else
                    {
                        Board_ledOff(LED_BLUE);
                    }
                }
                else
                {
                    if (!sensing)
                    {                     //standby or configuring
                        if (initializing || configuring)
                        {         //configuring
                            if (!(P1OUT & BIT1))
                                Board_ledOn(LED_GREEN1);
                            else
                                Board_ledOff(LED_GREEN1);
                        }
                        else if (btIsConnected && !configuring)
                        {
                            Board_ledOn(LED_BLUE);
                            Board_ledOff(LED_GREEN1);         // nothing to show
                        }
                        else
                        {                           //standby
                            if (!blinkCnt20)
                                Board_ledOn(LED_BLUE);
                            else
                                Board_ledOff(LED_BLUE);

                            Board_ledOff(LED_GREEN1);         // nothing to show
                        }
                    }
                    else
                    {                           //sensing
                        // sdlogReady, btstreamReady, enableSdlog, enableBtstream
                        // btstream only
                        if ((enableBtstream && btstreamReady)
                                && !(sdlogReady && enableSdlog))
                        {
                            if (!(blinkCnt20 % 10))
                            {
                                if (!(P1OUT & BIT2))
                                    Board_ledOn(LED_BLUE);
                                else
                                    Board_ledOff(LED_BLUE);
                            }
                            Board_ledOff(LED_GREEN1);         // nothing to show
                        }
                        // sdlog only
                        else if (!(enableBtstream && btstreamReady)
                                && (sdlogReady && enableSdlog))
                        {
                            if (!(blinkCnt20 % 10))
                            {
                                if (!(P1OUT & BIT1))
                                    Board_ledOn(LED_GREEN1);
                                else
                                    Board_ledOff(LED_GREEN1);
                            }
                            Board_ledOff(LED_BLUE);           // nothing to show
                        }
                        // btstream & sdlog
                        else if ((enableBtstream && btstreamReady)
                                && (sdlogReady && enableSdlog))
                        {
                            if (!(blinkCnt20 % 10))
                            {
                                if ((P1OUT & BIT2) || (P1OUT & BIT1))
                                    Board_ledOff(LED_BLUE + LED_GREEN1);
                                else
                                {
                                    if (lastLedGroup2)
                                    {
                                        Board_ledOn(LED_BLUE);
                                        lastLedGroup2 ^= 1;
                                    }
                                    else
                                    {
                                        Board_ledOn(LED_GREEN1);
                                        lastLedGroup2 ^= 1;
                                    }
                                }
                            }
                        }
                        else
                        {
                            Board_ledOff(LED_GREEN1 + LED_BLUE); // nothing to show
                        }
                    }
                }
            }
        }
        break;
    case 8:
        break;                       // TB0CCR4
    case 10:
        break;                       // reserved
    case 12:
        break;                       // reserved
    case 14:
        break;                       // TBIFG overflow handler
    }
}

// BT start Timer
void BtStartDone()
{
    btPowerOn = 1;
    configuring = 0;
}

void BtStart()
{
    if (!btPowerOn)
    {
        configuring = 1;
        BT_startAll();
    }
    BT_startDone_cb(BtStartDone);
}
void BtStop()
{
    getRcomm = 0;
    DMA2_disable();
    P1IES &= ~BIT0; //look for rising edge
    btIsConnected = 0;
    BT_connectionInterrupt(0);
    btPowerOn = 0;
    BT_disable();
    BT_rst_MessageProgress();
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
                        + clk_90;    //9ms
                TB0CCR1 = val_tb0
                        + *(uint16_t *) (storedConfig + NV_SAMPLING_RATE);
                TB0CCR2 = val_tb0
                        + *(uint16_t *) (storedConfig + NV_SAMPLING_RATE)
                        + ((bmpInUse == BMP180_IN_USE) ? clk_90_45 : clk_90_55); //9-4.5ms
                TB0CCTL1 = CCIE;
            }
            else
            {
                TB0CCR0 = val_tb0
                        + *(uint16_t *) (storedConfig + NV_SAMPLING_RATE)
                        + ((bmpInUse == BMP180_IN_USE) ? clk_45 : clk_55); //4.5ms
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
                        + clk_90;    //9ms
                TB0CCR1 = val_tb0
                        + *(uint16_t *) (storedConfig + NV_SAMPLING_RATE);
                TB0CCR2 = val_tb0
                        + *(uint16_t *) (storedConfig + NV_SAMPLING_RATE)
                        + ((bmpInUse == BMP180_IN_USE) ? clk_90_75 : clk_90_85); //9-7.5ms
                TB0CCTL1 = CCIE;
            }
            else
            {
                TB0CCR0 = val_tb0
                        + *(uint16_t *) (storedConfig + NV_SAMPLING_RATE)
                        + ((bmpInUse == BMP180_IN_USE) ? clk_75 : clk_85); //7.5ms
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
                    + ((bmpInUse == BMP180_IN_USE) ? clk_135 : clk_195); //13.5ms
            TB0CCR2 = val_tb0 + *(uint16_t *) (storedConfig + NV_SAMPLING_RATE);
            TB0CCTL2 = CCIE;
            if (preSampleMpuMag)
            {
                TB0CCR1 =
                        val_tb0
                                + *(uint16_t *) (storedConfig + NV_SAMPLING_RATE)
                                + ((bmpInUse == BMP180_IN_USE) ?
                                        clk_135_90 : clk_195_90);     //13.5-9ms
                TB0CCTL1 = CCIE;
            }
            else
                TB0CCTL1 = 0;
        }
        else if (preSampleBmpPress
                && (((storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x30) >> 4) == 3))
        {
            TB0CCR0 = val_tb0 + *(uint16_t *) (storedConfig + NV_SAMPLING_RATE)
                    + ((bmpInUse == BMP180_IN_USE) ? clk_255 : clk_375); //25.5ms
            TB0CCR2 = val_tb0 + *(uint16_t *) (storedConfig + NV_SAMPLING_RATE);
            TB0CCTL2 = CCIE;
            if (preSampleMpuMag)
            {
                TB0CCR1 =
                        val_tb0
                                + *(uint16_t *) (storedConfig + NV_SAMPLING_RATE)
                                + ((bmpInUse == BMP180_IN_USE) ?
                                        clk_255_90 : clk_375_90);      //25-9ms
                TB0CCTL1 = CCIE;
            }
            else
                TB0CCTL1 = 0;
        }
        else
        {
            TB0CCR0 = val_tb0 + *(uint16_t *) (storedConfig + NV_SAMPLING_RATE)
                    + clk_90;       //9ms
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

uint8_t timeStampLen;
#pragma vector=TIMER0_B0_VECTOR
__interrupt void TIMER0_B0_ISR(void)
{
    uint16_t timer_b0 = GetTB0();
    uint8_t rtc_temp[4];
    TB0CCR0 = timer_b0 + *(uint16_t *) (storedConfig + NV_SAMPLING_RATE);

    if (!streamDataInProc)
    {
        streamDataInProc = 1;
#if TS_BYTE3
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
            *((uint16_t *)(txBuff1+2)) = timer_b0; //the first two bytes are packet type bytes. reserved for BTstream
        }
        else
        {
            *((uint16_t *)(txBuff0+2)) = timer_b0;
        }
#endif
    }
    //start ADC conversion
    if (nbrAdcChans)
    {
        DMA0_enable();
        ADC_startConversion();
    }
    else
    {
        //no analog channels, so go straight to digital
        streamData = 1;
        __bic_SR_register_on_exit(LPM3_bits);
    }
}

uint8_t Dma0ConversionDone(void)
{
    uint8_t adc_offset;
    if (battWait)
    {
        battWait = 0;
        ConfigureChannels();

        // was: 0 - 2400 - 2550 - 4096
        // now: 0 - 2400 - 2600 - 4096
        if (battStat == BATT_MID)
        {
            if (*(uint16_t*) battVal < 2400)
            {
                battStat = BATT_LOW;
            }
            else if (*(uint16_t*) battVal < 2650)
            {
                battStat = BATT_MID;
            }
            else
                battStat = BATT_HIGH;
        }
        else if (battStat == BATT_LOW)
        {
            if (*(uint16_t*) battVal < 2450)
            {
                battStat = BATT_LOW;
            }
            else if (*(uint16_t*) battVal < 2600)
            {
                battStat = BATT_MID;
            }
            else
                battStat = BATT_HIGH;
        }
        else
        {
            if (*(uint16_t*) battVal < 2400)
            {
                battStat = BATT_LOW;
            }
            else if (*(uint16_t*) battVal < 2600)
            {
                battStat = BATT_MID;
            }
            else
                battStat = BATT_HIGH;
        }
        battVal[2] = P2IN & 0xc0;
    }
    else
    {
        adc_offset = 4;

        //Destination address for next transfer
        if (currentBuffer)
        {
            DMA0_repeatTransfer(adcStartPtr,
                                (uint16_t *) (txBuff0 + adc_offset),
                                nbrAdcChans);
        }
        else
        {
            DMA0_repeatTransfer(adcStartPtr,
                                (uint16_t *) (txBuff1 + adc_offset),
                                nbrAdcChans);
        }
        ADC_disable(); //can disable ADC until next time sampleTimer fires (to save power)?
        DMA0_disable();
        streamData = 1;
    }
    return 1;
}

uint8_t Dma2ConversionDone(void)
{
    uint8_t bt_getmac, bt_getVersion;

    DMA2_disable();
    bt_getmac = BT_getMacAddress();
    bt_getVersion = BT_getVersion();

    if (!*btRxExp && (btIsConnected || bt_getmac || bt_getVersion))
    {
        if (syncEnabled)
        {
            if (getRcomm)
            {
                // 6 bytes of RC info
                memcpy(rcommResp, btRxBuff, 6);
                memset(btRxBuff, 0, 14);
                getRcomm = 0;
                return 1;
            }/*else if(btRxBuff[0] == ROUTINE_COMMUNICATION){
             // 1 byte of RC command
             memset(btRxBuff,0,14);
             processCommand = 1;
             return 1;
             }*/
        }
        else if (bt_getmac)
        {
            // 14 bytes of BT mac address
            memcpy(mac, btRxBuff, 14);
            memset(btRxBuff, 0, 14);
            BT_setGetMacAddress(0);
            uint8_t i, pchar[3];
            pchar[2] = 0;
            for (i = 0; i < 6; i++)
            {
                pchar[0] = mac[i * 2];
                pchar[1] = mac[i * 2 + 1];
                macAddr[i] = strtoul((char*) pchar, 0, 16);
            }
            InfoMem_write((uint8_t*) NV_MAC_ADDRESS, macAddr, 6);
            memcpy(storedConfig + NV_MAC_ADDRESS, macAddr, 6);
            BT_setGoodCommand();
        }
        else if (bt_getVersion)
        {
            // 26 bytes of BT Version information
            BT_setGetVersion(0);
            char verString[4];
            sprintf(verString, "%c%c%c%c", btRxBuff[4], btRxBuff[5],
                    btRxBuff[6], btRxBuff[7]);

            if (strstr(verString, "6.30"))
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
            memset(btRxBuff, 0, 26);
            BT_setGoodCommand();
        }
        else
        {
            if (waitingForArgs)
            {
                if ((!waitingForArgsLength)
                        && ((gAction == SET_EXG_REGS_COMMAND)
                                && (waitingForArgs == 3)))
                {
                    args[0] = btRxBuff[0];
                    args[1] = btRxBuff[1];
                    args[2] = btRxBuff[2];
                    DMA2SZ = args[2];
                    DMA2_enable();
                    waitingForArgsLength = args[2];
                    return 0;
                }
                else if ((!waitingForArgsLength)
                        && (((gAction == SET_INFOMEM_COMMAND)
                                && (waitingForArgs == 3))
                                || ((gAction == SET_CALIB_DUMP_COMMAND)
                                        && (waitingForArgs == 3))))
                {
                    args[0] = btRxBuff[0];
                    args[1] = btRxBuff[1];
                    args[2] = btRxBuff[2];
                    DMA2SZ = args[0];
                    DMA2_enable();
                    waitingForArgsLength = args[0];
                    return 0;
                }
                else if ((!waitingForArgsLength) && (
                //                ((gAction == SET_DAUGHTER_CARD_ID_COMMAND) && (waitingForArgs == 1)) ||
                        ((gAction == SET_DAUGHTER_CARD_MEM_COMMAND)
                                && (waitingForArgs == 1))
                                || ((gAction == SET_CENTER_COMMAND)
                                        && (waitingForArgs == 1))
                                || ((gAction == SET_CONFIGTIME_COMMAND)
                                        && (waitingForArgs == 1))
                                || ((gAction == SET_EXPID_COMMAND)
                                        && (waitingForArgs == 1))
                                || ((gAction == SET_SHIMMERNAME_COMMAND)
                                        && (waitingForArgs == 1))))
                {
                    args[0] = btRxBuff[0];
                    if (args[0])
                    {
                        DMA2SZ = args[0];
                        DMA2_enable();
                        waitingForArgsLength = args[0];
                        return 0;
                    }
                }
                if (waitingForArgsLength)
                    memcpy(args + waitingForArgs, btRxBuff,
                           waitingForArgsLength);
                else
                    memcpy(args, btRxBuff, waitingForArgs);
                DMA2SZ = 1;
                DMA2_enable();
                waitingForArgsLength = 0;
                waitingForArgs = 0;
                processCommand = 1;
                argsSize = 0;
                return 1;
            }
            else
            {
                uint8_t data = btRxBuff[0];
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
                    gAction = data;
                    processCommand = 1;
                    DMA2SZ = 1;
                    DMA2_enable();
                    return 1;
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
                    waitingForArgs = 1;
                    DMA2SZ = 1;
                    DMA2_enable();
                    gAction = data;
                    return 0;
                case SET_SAMPLING_RATE_COMMAND:
                case GET_DAUGHTER_CARD_ID_COMMAND:
                    //                case SET_DAUGHTER_CARD_ID_COMMAND:
                    waitingForArgs = 2;
                    DMA2SZ = 2;
                    DMA2_enable();
                    gAction = data;
                    return 0;
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
                    waitingForArgs = 3;
                    DMA2SZ = 3;
                    DMA2_enable();
                    gAction = data;
                    return 0;
                case SET_CONFIG_SETUP_BYTES_COMMAND:
                    waitingForArgs = 4;
                    DMA2SZ = 4;
                    DMA2_enable();
                    gAction = data;
                    return 0;
                case SET_RWC_COMMAND:
                case SET_DERIVED_CHANNEL_BYTES:
                    waitingForArgs = 8;
                    DMA2SZ = 8;
                    DMA2_enable();
                    gAction = data;
                    return 0;
                case SET_A_ACCEL_CALIBRATION_COMMAND:
                case SET_MPU9150_GYRO_CALIBRATION_COMMAND:
                case SET_LSM303DLHC_MAG_CALIBRATION_COMMAND:
                case SET_LSM303DLHC_ACCEL_CALIBRATION_COMMAND:
                    waitingForArgs = 21;
                    DMA2SZ = 21;
                    DMA2_enable();
                    gAction = data;
                    return 0;
                default:
                    DMA2SZ = 1;
                    DMA2_enable();
                    return 0;
                }
            }
        }
    }
    else
    {
        if (!memcmp(btRxBuff, btRxExp, strlen((char*) btRxExp)))
        {
            memset(btRxBuff, 0, 14);
            BT_setGoodCommand();
        }
        else
        {
            _NOP();   // bad command trap: reaching here = serious BT problem
        }
    }

    return 0;
}

void ProcessCommand(void)
{
    uint32_t config_time;
    uint8_t my_config_time[4];
    uint8_t name_len;
    uint8_t update_sdconfig = 0, update_calib = 0, calib_sensor = 0,
            calib_range = 0, update_calib_dump_file = 0;
    //uint8_t update_sdconfig_manual = 0;
    sc_t sc1;
    switch (gAction)
    {
    case INQUIRY_COMMAND:
        inquiryResponse = 1;
        break;
    case DUMMY_COMMAND:
        break;
    case GET_SAMPLING_RATE_COMMAND:
        samplingRateResponse = 1;
        break;
    case TOGGLE_LED_COMMAND:
        toggleLedRed ^= 1;
        break;
    case START_STREAMING_COMMAND:
        enableBtstream = 1;
        if (!sensing)
        {
            configureChannels = 1;
            //startSensing = 1;
            SetStartSensing();
        }
        //enableSdlog = 0;
        //newDirFlag = 1;
        //CheckSdInslot();
        break;
    case START_SDBT_COMMAND:
        if (!sensing)
        {
            configureChannels = 1;
            //startSensing = 1;
            SetStartSensing();
        }
        enableBtstream = 1;
        if (sdlogReady && (!SD_ERROR))
        {
            enableSdlog = 1;
        }
        break;
    case START_LOGGING_COMMAND:
        if (sdlogReady && (!SD_ERROR))
        {
            enableSdlog = 1;
            if (!sensing)
            {
                //startSensing = 1;
                SetStartSensing();
                configureChannels = 1;
            }
        }
        break;
    case SET_CRC_COMMAND:
        crcChecksum = args[0];
        break;

    case STOP_STREAMING_COMMAND:
        if (isStreaming)
        {
            stopStreaming = 1;
        }
        break;
    case STOP_SDBT_COMMAND:
        if (sensing)
        {
            stopSensing = 1;
            //return;
        }
        break;
    case STOP_LOGGING_COMMAND:
        if (isLogging)
        {
            stopLogging = 1;
            //if(!isStreaming);
            //return;
        }
        break;
    case SET_SENSORS_COMMAND:
        storedConfig[NV_SENSORS0] = args[0];
        storedConfig[NV_SENSORS1] = args[1];
        storedConfig[NV_SENSORS2] = args[2];
        if (storedConfig[NV_SENSORS0] & SENSOR_GSR) // they are sharing adc1, so ban intch1 when gsr is on
            storedConfig[NV_SENSORS1] &= ~SENSOR_INT_A1;
        InfoMem_write((void*) NV_SENSORS0, &storedConfig[NV_SENSORS0], 3);
        sdHeadText[SDH_SENSORS0] = storedConfig[NV_SENSORS0];
        sdHeadText[SDH_SENSORS1] = storedConfig[NV_SENSORS1];
        sdHeadText[SDH_SENSORS2] = storedConfig[NV_SENSORS2];
        update_sdconfig = 1;
        configureChannels = 1;
        if (sensing)
        {
            stopSensing = 1;
            //startSensing = 1;
            SetStartSensing();
        }
        break;
    case GET_LSM303DLHC_ACCEL_RANGE_COMMAND:
        lsm303dlhcAccelRangeResponse = 1;
        break;
    case GET_LSM303DLHC_MAG_GAIN_COMMAND:
        lsm303dlhcMagGainResponse = 1;
        break;
    case GET_LSM303DLHC_MAG_SAMPLING_RATE_COMMAND:
        lsm303dlhcMagSamplingRateResponse = 1;
        break;
    case GET_STATUS_COMMAND:
        dockedResponse = 1;
        break;
    case GET_VBATT_COMMAND:
        btVbattResponse = 1;
        break;
    case GET_TRIAL_CONFIG_COMMAND:
        trialConfigResponse = 1;
        break;
    case SET_TRIAL_CONFIG_COMMAND:
        storedConfig[NV_SD_TRIAL_CONFIG0] = args[0];
        //        storedConfig[NV_SD_TRIAL_CONFIG1] = 0;
        storedConfig[NV_SD_TRIAL_CONFIG1] = args[1];
        storedConfig[NV_SD_BT_INTERVAL] = args[2];
        if (storedConfig[NV_SD_TRIAL_CONFIG1] & SDH_SINGLETOUCH)
        {
            storedConfig[NV_SD_TRIAL_CONFIG0] |= SDH_USER_BUTTON_ENABLE;
            storedConfig[NV_SD_TRIAL_CONFIG0] |= SDH_TIME_SYNC;
        }
        storedConfig[NV_SD_TRIAL_CONFIG1] &= ~SDH_SINGLETOUCH;   // disable sync
        storedConfig[NV_SD_TRIAL_CONFIG0] &= ~SDH_TIME_SYNC;  // todo: rmv this
        if (storedConfig[NV_SD_BT_INTERVAL] < RC_INT_C)
            storedConfig[NV_SD_BT_INTERVAL] = RC_INT_C;
        InfoMem_write((void*) NV_SAMPLING_RATE,
                      &storedConfig[NV_SD_TRIAL_CONFIG0], 3);
        sdHeadText[SDH_TRIAL_CONFIG0] = storedConfig[NV_SD_TRIAL_CONFIG0];
        //        sdHeadText[SDH_TRIAL_CONFIG1] = 0;  //storedConfig[NV_SD_TRIAL_CONFIG1];
        sdHeadText[SDH_TRIAL_CONFIG1] = storedConfig[NV_SD_TRIAL_CONFIG1];
        sdHeadText[SDH_BROADCAST_INTERVAL] = storedConfig[NV_SD_BT_INTERVAL];
        update_sdconfig = 1;
        break;
    case GET_CENTER_COMMAND:
        centerResponse = 1;
        break;
    case SET_CENTER_COMMAND:
        break;
    case GET_SHIMMERNAME_COMMAND:
        shimmerNameResponse = 1;
        break;
    case SET_SHIMMERNAME_COMMAND:
        name_len = args[0] < (MAX_CHARS - 1) ? args[0] : (MAX_CHARS - 1);
        memset((uint8_t*) (storedConfig + NV_SD_SHIMMER_NAME), 0,
        MAX_CHARS - 1);
        memcpy((uint8_t*) (storedConfig + NV_SD_SHIMMER_NAME), &args[1],
               name_len);
        InfoMem_write((uint8_t*) NV_SD_SHIMMER_NAME,
                      storedConfig + NV_SD_SHIMMER_NAME, MAX_CHARS - 1);
        SetShimmerName();
        update_sdconfig = 1;
        break;
    case GET_EXPID_COMMAND:
        expIDResponse = 1;
        break;
    case SET_EXPID_COMMAND:
        name_len = args[0] < (MAX_CHARS - 1) ? args[0] : (MAX_CHARS - 1);
        memset((uint8_t*) (storedConfig + NV_SD_EXP_ID_NAME), 0, MAX_CHARS - 1);
        memcpy((uint8_t*) (storedConfig + NV_SD_EXP_ID_NAME), &args[1],
               name_len);
        InfoMem_write((uint8_t*) NV_SD_EXP_ID_NAME,
                      storedConfig + NV_SD_EXP_ID_NAME, MAX_CHARS - 1);
        SetExpIdName();
        update_sdconfig = 1;
        break;
    case GET_CONFIGTIME_COMMAND:
        configTimeResponse = 1;
        break;
    case GET_DIR_COMMAND:
        dirResponse = 1;
        break;
    case SET_CONFIGTIME_COMMAND:
        name_len = args[0] < (MAX_CHARS - 1) ? args[0] : (MAX_CHARS - 1);
        memcpy(configTimeText, &args[1], name_len);
        configTimeText[args[0]] = '\0';
        SetName();
        config_time = atol((char*) configTimeText);
        my_config_time[3] = *((uint8_t*) &config_time);
        my_config_time[2] = *(((uint8_t*) &config_time) + 1);
        my_config_time[1] = *(((uint8_t*) &config_time) + 2);
        my_config_time[0] = *(((uint8_t*) &config_time) + 3);
        memcpy(&sdHeadText[SDH_CONFIG_TIME_0], my_config_time, 4);
        memcpy((uint8_t*) (storedConfig + NV_SD_CONFIG_TIME), my_config_time,
               4);
        InfoMem_write((uint8_t*) NV_SD_CONFIG_TIME,
                      storedConfig + NV_SD_CONFIG_TIME, 4);
        update_sdconfig = 1;
        break;
    case GET_NSHIMMER_COMMAND:
        nshimmerResponse = 1;
        break;
    case SET_NSHIMMER_COMMAND:
        storedConfig[NV_SD_NSHIMMER] = args[0];
        sdHeadText[SDH_NSHIMMER] = args[0];
        InfoMem_write((uint8_t*) NV_SD_NSHIMMER, storedConfig + NV_SD_NSHIMMER,
                      1);
        update_sdconfig = 1;
        break;
    case GET_MYID_COMMAND:
        myIDResponse = 1;
        break;
    case SET_MYID_COMMAND:
        storedConfig[NV_SD_MYTRIAL_ID] = args[0];
        sdHeadText[SDH_MYTRIAL_ID] = args[0];
        InfoMem_write((uint8_t*) NV_SD_MYTRIAL_ID,
                      storedConfig + NV_SD_MYTRIAL_ID, 1);
        update_sdconfig = 1;
        break;
    case SET_LSM303DLHC_ACCEL_RANGE_COMMAND:
        if (args[0] < 4){
            storedConfig[NV_CONFIG_SETUP_BYTE0] =
                    (storedConfig[NV_CONFIG_SETUP_BYTE0] & 0xF3)
                            + ((args[0] & 0x03) << 2);
        }
        else{
            storedConfig[NV_CONFIG_SETUP_BYTE0] &= 0xF3;
        }
        InfoMem_write((void*) NV_CONFIG_SETUP_BYTE0,
                      &storedConfig[NV_CONFIG_SETUP_BYTE0], 1);
        sdHeadText[SDH_CONFIG_SETUP_BYTE0] =
                storedConfig[NV_CONFIG_SETUP_BYTE0];
        update_sdconfig = 1;
        if (sensing)
        {
            stopSensing = 1;
            //startSensing = 1;
            SetStartSensing();
        }
        break;
    case GET_LSM303DLHC_ACCEL_SAMPLING_RATE_COMMAND:
        lsm303dlhcAccelSamplingRateResponse = 1;
        break;
    case GET_LSM303DLHC_ACCEL_LPMODE_COMMAND:
        lsm303dlhcAccelLPModeResponse = 1;
        break;
    case GET_LSM303DLHC_ACCEL_HRMODE_COMMAND:
        lsm303dlhcAccelHRModeResponse = 1;
        break;
    case GET_MPU9150_GYRO_RANGE_COMMAND:
        mpu9150GyroRangeResponse = 1;
        break;
    case GET_BMP180_CALIBRATION_COEFFICIENTS_COMMAND:
    {
        //      sc1.id = SC_SENSOR_BMPX80_PRESSURE;
        //      sc1.range = SC_SENSOR_RANGE_BMPX80;
        //      sc1.data_len = SC_DATA_LEN_BMPX80;
        //      memcpy(sc1.data, bmpX80Calib, 22);
        //      ShimmerCalib_singleSensorWrite(&sc1);
        //      update_calib_dump_file = 1;
        bmp180CalibrationCoefficientsResponse = 1;
        break;
    }
    case GET_BMP280_CALIBRATION_COEFFICIENTS_COMMAND:
        bmp280CalibrationCoefficientsResponse = 1;
        break;
    case GET_MPU9150_SAMPLING_RATE_COMMAND:
        mpu9150SamplingRateResponse = 1;
        break;
    case GET_MPU9150_ACCEL_RANGE_COMMAND:
        mpu9150AccelRangeResponse = 1;
        break;
    case GET_BMPX80_PRES_OVERSAMPLING_RATIO_COMMAND:
        bmpX80OversamplingRatioResponse = 1;
        break;
    case GET_INTERNAL_EXP_POWER_ENABLE_COMMAND:
        internalExpPowerEnableResponse = 1;
        break;
    case GET_MPU9150_MAG_SENS_ADJ_VALS_COMMAND:
        mpu9150MagSensAdjValsResponse = 1;
        break;
    case GET_EXG_REGS_COMMAND:
        if (args[0] < 2 && args[1] < 10 && args[2] < 11)
        {
            exgChip = args[0];
            exgStartAddr = args[1];
            exgLength = args[2];
        }
        else
            exgLength = 0;
        exgRegsResponse = 1;
        break;
    case SET_LSM303DLHC_ACCEL_SAMPLING_RATE_COMMAND:
        if (args[0] < 10)
            storedConfig[NV_CONFIG_SETUP_BYTE0] =
                    (storedConfig[NV_CONFIG_SETUP_BYTE0] & 0x0F)
                            + ((args[0] & 0x0F) << 4);
        else
            storedConfig[NV_CONFIG_SETUP_BYTE0] =
                    (storedConfig[NV_CONFIG_SETUP_BYTE0] & 0x0F)
                            + (LSM303DLHC_ACCEL_100HZ << 4);
        InfoMem_write((void*) NV_CONFIG_SETUP_BYTE0,
                      &storedConfig[NV_CONFIG_SETUP_BYTE0], 1);
        sdHeadText[SDH_CONFIG_SETUP_BYTE0] =
                storedConfig[NV_CONFIG_SETUP_BYTE0];
        update_sdconfig = 1;
        if (sensing)
        {
            stopSensing = 1;
            //startSensing = 1;
            SetStartSensing();
        }
        break;
    case SET_LSM303DLHC_MAG_GAIN_COMMAND:
        if (args[0] > 0 && args[0] < 8)
            storedConfig[NV_CONFIG_SETUP_BYTE2] =
                    (storedConfig[NV_CONFIG_SETUP_BYTE2] & 0x1F)
                            + ((args[0] & 0x07) << 5);
        else
            storedConfig[NV_CONFIG_SETUP_BYTE2] =
                    (storedConfig[NV_CONFIG_SETUP_BYTE2] & 0x1F)
                            + (LSM303DLHC_MAG_1_3G << 5);
        InfoMem_write((void*) NV_CONFIG_SETUP_BYTE2,
                      &storedConfig[NV_CONFIG_SETUP_BYTE2], 1);
        sdHeadText[SDH_CONFIG_SETUP_BYTE2] =
                storedConfig[NV_CONFIG_SETUP_BYTE2];
        update_sdconfig = 1;
        if (sensing)
        {
            stopSensing = 1;
            //startSensing = 1;
            SetStartSensing();
        }
        break;
    case SET_LSM303DLHC_MAG_SAMPLING_RATE_COMMAND:
        if (args[0] < 8)
            storedConfig[NV_CONFIG_SETUP_BYTE2] =
                    (storedConfig[NV_CONFIG_SETUP_BYTE2] & 0xE3)
                            + ((args[0] & 0x07) << 2);
        else
            storedConfig[NV_CONFIG_SETUP_BYTE2] =
                    (storedConfig[NV_CONFIG_SETUP_BYTE2] & 0xE3)
                            + (LSM303DLHC_MAG_75HZ << 2);
        InfoMem_write((void*) NV_CONFIG_SETUP_BYTE2,
                      &storedConfig[NV_CONFIG_SETUP_BYTE2], 1);
        sdHeadText[SDH_CONFIG_SETUP_BYTE2] =
                storedConfig[NV_CONFIG_SETUP_BYTE2];
        update_sdconfig = 1;
        if (sensing)
        {
            stopSensing = 1;
            //startSensing = 1;
            SetStartSensing();
        }
        break;
    case SET_LSM303DLHC_ACCEL_LPMODE_COMMAND:
        if (args[0] == 1)
        {
            storedConfig[NV_CONFIG_SETUP_BYTE0] =
                    (storedConfig[NV_CONFIG_SETUP_BYTE0] & 0xFD) + 0x02;
        }
        else
        {
            storedConfig[NV_CONFIG_SETUP_BYTE0] &= 0xFD;
        }
        InfoMem_write((void*) NV_CONFIG_SETUP_BYTE0,
                      &storedConfig[NV_CONFIG_SETUP_BYTE0], 1);
        sdHeadText[SDH_CONFIG_SETUP_BYTE0] =
                storedConfig[NV_CONFIG_SETUP_BYTE0];
        update_sdconfig = 1;
        if (sensing)
        {
            stopSensing = 1;
            //startSensing = 1;
            SetStartSensing();
        }
        break;
    case SET_LSM303DLHC_ACCEL_HRMODE_COMMAND:
        if (args[0] == 1)
        {
            storedConfig[NV_CONFIG_SETUP_BYTE0] =
                    (storedConfig[NV_CONFIG_SETUP_BYTE0] & 0xFE) + 0x01;
        }
        else
        {
            storedConfig[NV_CONFIG_SETUP_BYTE0] &= 0xFE;
        }
        InfoMem_write((void*) NV_CONFIG_SETUP_BYTE0,
                      &storedConfig[NV_CONFIG_SETUP_BYTE0], 1);
        sdHeadText[SDH_CONFIG_SETUP_BYTE0] =
                storedConfig[NV_CONFIG_SETUP_BYTE0];
        update_sdconfig = 1;
        if (sensing)
        {
            stopSensing = 1;
            //startSensing = 1;
            SetStartSensing();
        }
        break;
    case SET_MPU9150_GYRO_RANGE_COMMAND:
        if (args[0] < 4)
            storedConfig[NV_CONFIG_SETUP_BYTE2] =
                    (storedConfig[NV_CONFIG_SETUP_BYTE2] & 0xFC)
                            + (args[0] & 0x03);
        else
            storedConfig[NV_CONFIG_SETUP_BYTE2] =
                    (storedConfig[NV_CONFIG_SETUP_BYTE2] & 0xFC)
                            + MPU9150_GYRO_500DPS;
        InfoMem_write((void*) NV_CONFIG_SETUP_BYTE2,
                      &storedConfig[NV_CONFIG_SETUP_BYTE2], 1);
        sdHeadText[SDH_CONFIG_SETUP_BYTE2] =
                storedConfig[NV_CONFIG_SETUP_BYTE2];
        update_sdconfig = 1;
        if (sensing)
        {
            stopSensing = 1;
            //startSensing = 1;
            SetStartSensing();
        }
        break;
    case SET_MPU9150_SAMPLING_RATE_COMMAND:
        storedConfig[NV_CONFIG_SETUP_BYTE1] = args[0];
        InfoMem_write((void*) NV_CONFIG_SETUP_BYTE1,
                      &storedConfig[NV_CONFIG_SETUP_BYTE1], 1);
        sdHeadText[SDH_CONFIG_SETUP_BYTE1] =
                storedConfig[NV_CONFIG_SETUP_BYTE1];
        update_sdconfig = 1;
        if (sensing)
        {
            stopSensing = 1;
            //startSensing = 1;
            SetStartSensing();
        }
        break;
    case SET_MPU9150_ACCEL_RANGE_COMMAND:
        if (args[0] < 4)
            storedConfig[NV_CONFIG_SETUP_BYTE3] =
                    (storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x3F)
                            + ((args[0] & 0x03) << 6);
        else
            storedConfig[NV_CONFIG_SETUP_BYTE3] =
                    (storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x3F)
                            + (ACCEL_2G << 6);
        InfoMem_write((void*) NV_CONFIG_SETUP_BYTE3,
                      &storedConfig[NV_CONFIG_SETUP_BYTE3], 1);
        sdHeadText[SDH_CONFIG_SETUP_BYTE3] =
                storedConfig[NV_CONFIG_SETUP_BYTE3];
        update_sdconfig = 1;
        if (sensing)
        {
            stopSensing = 1;
            //startSensing = 1;
            SetStartSensing();
        }
        break;
    case SET_BMPX80_PRES_OVERSAMPLING_RATIO_COMMAND:
        if (args[0] < 4)
            storedConfig[NV_CONFIG_SETUP_BYTE3] =
                    (storedConfig[NV_CONFIG_SETUP_BYTE3] & 0xCF)
                            + ((args[0] & 0x03) << 4);
        else
            storedConfig[NV_CONFIG_SETUP_BYTE3] =
                    (storedConfig[NV_CONFIG_SETUP_BYTE3] & 0xCF)
                            + (BMPX80_OSS_1 << 4);
        InfoMem_write((void*) NV_CONFIG_SETUP_BYTE3,
                      &storedConfig[NV_CONFIG_SETUP_BYTE3], 1);
        sdHeadText[SDH_CONFIG_SETUP_BYTE3] =
                storedConfig[NV_CONFIG_SETUP_BYTE3];
        update_sdconfig = 1;
        break;
    case SET_INTERNAL_EXP_POWER_ENABLE_COMMAND:
        if (args[0] == 1)
            storedConfig[NV_CONFIG_SETUP_BYTE3] =
                    (storedConfig[NV_CONFIG_SETUP_BYTE3] & 0xFE)
                            + (args[0] & 0x01);
        else
            storedConfig[NV_CONFIG_SETUP_BYTE3] &= 0xFE;
        InfoMem_write((void*) NV_CONFIG_SETUP_BYTE3,
                      &storedConfig[NV_CONFIG_SETUP_BYTE3], 1);
        sdHeadText[SDH_CONFIG_SETUP_BYTE3] =
                storedConfig[NV_CONFIG_SETUP_BYTE3];
        update_sdconfig = 1;
        if (sensing)
        {
            stopSensing = 1;
            //startSensing = 1;
            SetStartSensing();
        }
        break;
    case GET_CONFIG_SETUP_BYTES_COMMAND:
        configSetupBytesResponse = 1;
        break;
    case SET_CONFIG_SETUP_BYTES_COMMAND:
        storedConfig[NV_CONFIG_SETUP_BYTE0] = args[0];
        storedConfig[NV_CONFIG_SETUP_BYTE1] = args[1];
        storedConfig[NV_CONFIG_SETUP_BYTE2] = args[2];
        storedConfig[NV_CONFIG_SETUP_BYTE3] = args[3];
        InfoMem_write((void*) NV_CONFIG_SETUP_BYTE0,
                      &storedConfig[NV_CONFIG_SETUP_BYTE0], 4);
        memcpy(&sdHeadText[SDH_CONFIG_SETUP_BYTE0],
               &storedConfig[NV_CONFIG_SETUP_BYTE0], 4);
        Config2SdHead();
        update_sdconfig = 1;
        if (sensing)
        {
            stopSensing = 1;
            //startSensing = 1;
            SetStartSensing();
        }
        break;
    case SET_SAMPLING_RATE_COMMAND:
        *(uint16_t *) (storedConfig + NV_SAMPLING_RATE) = *(uint16_t *) args;
        InfoMem_write((void*) NV_SAMPLING_RATE, &storedConfig[NV_SAMPLING_RATE],
                      2);
        sdHeadText[SDH_SAMPLE_RATE_0] = storedConfig[NV_SAMPLING_RATE];
        sdHeadText[SDH_SAMPLE_RATE_1] = storedConfig[NV_SAMPLING_RATE + 1];
        update_sdconfig = 1;
        if (sensing)
        {
            //restart sampling timer to use new sampling rate
            stopSensing = 1;
            //startSensing = 1;
            SetStartSensing();
        }
        break;
    case GET_CALIB_DUMP_COMMAND:
        // usage:
        // 0x98, offset, offset, length
        calibRamLength = args[0];
        calibRamOffset = args[1] + (args[2] << 8);
        calibRamResponse = 1;
        break;
    case SET_CALIB_DUMP_COMMAND:
        // usage:
        // 0x98, offset, offset, length, data[0:127]
        // max length of this command = 132
        calibRamLength = args[0];
        calibRamOffset = args[1] + (args[2] << 8);
        if (ShimmerCalib_ramWrite(args + 3, calibRamLength, calibRamOffset)
                == 1)
        {
            ShimmerCalibSyncFromDumpRamAll();
            update_calib_dump_file = 1;
        }
        break;
    case UPD_CALIB_DUMP_COMMAND:
        ShimmerCalibSyncFromDumpRamAll();
        update_calib_dump_file = 1;
        break;
    case UPD_SDLOG_CFG_COMMAND:
        sdlogcfgUpdate = 1;
        //update_sdconfig_manual = 1;
        break;
    case SET_A_ACCEL_CALIBRATION_COMMAND:
        memcpy(&storedConfig[NV_A_ACCEL_CALIBRATION], args, 21);
        InfoMem_write((void*) NV_A_ACCEL_CALIBRATION,
                      &storedConfig[NV_A_ACCEL_CALIBRATION], 21);
        memcpy(&sdHeadText[SDH_A_ACCEL_CALIBRATION],
               &storedConfig[NV_A_ACCEL_CALIBRATION], 21);
        sc1.id = SC_SENSOR_ANALOG_ACCEL;
        sc1.range = SC_SENSOR_RANGE_ANALOG_ACCEL;
        sc1.data_len = SC_DATA_LEN_ANALOG_ACCEL;
        memcpy(sc1.data.raw, args, sc1.data_len);
        ShimmerCalib_singleSensorWrite(&sc1);
        update_calib_dump_file = 1;
        //update_calib = 1;
        //calib_sensor = S_ACCEL_A;
        break;
    case GET_A_ACCEL_CALIBRATION_COMMAND:
        aAccelCalibrationResponse = 1;
        break;
    case SET_MPU9150_GYRO_CALIBRATION_COMMAND:
        memcpy(&storedConfig[NV_MPU9150_GYRO_CALIBRATION], args, 21);
        InfoMem_write((void*) NV_MPU9150_GYRO_CALIBRATION,
                      &storedConfig[NV_MPU9150_GYRO_CALIBRATION], 21);
        memcpy(&sdHeadText[SDH_MPU9150_GYRO_CALIBRATION],
               &storedConfig[NV_MPU9150_GYRO_CALIBRATION], 21);
        sc1.id = SC_SENSOR_MPU9150_GYRO;
        sc1.range = storedConfig[NV_CONFIG_SETUP_BYTE2] & 0x03;
        sc1.data_len = SC_DATA_LEN_MPU9250_GYRO;
        memcpy(sc1.data.raw, args, sc1.data_len);
        ShimmerCalib_singleSensorWrite(&sc1);
        update_calib_dump_file = 1;
        //      update_calib = 1;
        //      calib_sensor = S_GYRO;
        //      calib_range = storedConfig[NV_CONFIG_SETUP_BYTE2] & 0x03;
        break;
    case GET_MPU9150_GYRO_CALIBRATION_COMMAND:
        gyroCalibrationResponse = 1;
        break;
    case SET_LSM303DLHC_MAG_CALIBRATION_COMMAND:
        memcpy(&storedConfig[NV_LSM303DLHC_MAG_CALIBRATION], args, 21);
        InfoMem_write((void*) NV_LSM303DLHC_MAG_CALIBRATION,
                      &storedConfig[NV_LSM303DLHC_MAG_CALIBRATION], 21);
        memcpy(&sdHeadText[SDH_LSM303DLHC_MAG_CALIBRATION],
               &storedConfig[NV_LSM303DLHC_MAG_CALIBRATION], 21);
        sc1.id = SC_SENSOR_LSM303DLHC_MAG;
        sc1.range = (storedConfig[NV_CONFIG_SETUP_BYTE2] >> 5) & 0x07;
        sc1.data_len = SC_DATA_LEN_LSM303DLHC_MAG;
        memcpy(sc1.data.raw, args, sc1.data_len);
        ShimmerCalib_singleSensorWrite(&sc1);
        update_calib_dump_file = 1;
        //      update_calib = 1;
        //      calib_sensor = S_MAG;
        //      calib_range = (storedConfig[NV_CONFIG_SETUP_BYTE2]>>5) & 0x07;
        break;
    case GET_LSM303DLHC_MAG_CALIBRATION_COMMAND:
        magCalibrationResponse = 1;
        break;
    case SET_LSM303DLHC_ACCEL_CALIBRATION_COMMAND:
        memcpy(&storedConfig[NV_LSM303DLHC_ACCEL_CALIBRATION], args, 21);
        InfoMem_write((void*) NV_LSM303DLHC_ACCEL_CALIBRATION,
                      &storedConfig[NV_LSM303DLHC_ACCEL_CALIBRATION], 21);
        memcpy(&sdHeadText[SDH_LSM303DLHC_ACCEL_CALIBRATION],
               &storedConfig[NV_LSM303DLHC_ACCEL_CALIBRATION], 21);
        sc1.id = SC_SENSOR_LSM303DLHC_ACCEL;
        sc1.range = (storedConfig[NV_CONFIG_SETUP_BYTE0] >> 2) & 0x03;
        sc1.data_len = SC_DATA_LEN_LSM303DLHC_MAG;
        memcpy(sc1.data.raw, args, sc1.data_len);
        ShimmerCalib_singleSensorWrite(&sc1);
        update_calib_dump_file = 1;
        //      update_calib = 1;
        //      calib_sensor = S_ACCEL_D;
        //      calib_range = (storedConfig[NV_CONFIG_SETUP_BYTE0]>>2)&0x03;
        break;
    case SET_GSR_RANGE_COMMAND:
        if (args[0] <= 4)
            storedConfig[NV_CONFIG_SETUP_BYTE3] =
                    (storedConfig[NV_CONFIG_SETUP_BYTE3] & 0xF1)
                            + ((args[0] & 0x07) << 1);
        else
            storedConfig[NV_CONFIG_SETUP_BYTE3] =
                    (storedConfig[NV_CONFIG_SETUP_BYTE3] & 0xF1)
                            + (GSR_AUTORANGE << 1);
        InfoMem_write((void*) NV_CONFIG_SETUP_BYTE3,
                      &storedConfig[NV_CONFIG_SETUP_BYTE3], 1);
        sdHeadText[SDH_CONFIG_SETUP_BYTE3] =
                storedConfig[NV_CONFIG_SETUP_BYTE3];
        update_sdconfig = 1;
        if (sensing)
        {
            stopSensing = 1;
            //startSensing = 1;
            SetStartSensing();
        }
        break;
    case SET_EXG_REGS_COMMAND:
        if (args[0] < 2 && args[1] < 10 && args[2] < 11)
        {
            if (args[0])
            {
                memcpy((storedConfig + NV_EXG_ADS1292R_2_CONFIG1 + args[1]),
                       (args + 3), args[2]);
                InfoMem_write(
                        (void*) (NV_EXG_ADS1292R_2_CONFIG1 + args[1]),
                        (storedConfig + NV_EXG_ADS1292R_2_CONFIG1 + args[1]),
                        args[2]);
                memcpy(sdHeadText + SDH_EXG_ADS1292R_2_CONFIG1,
                       storedConfig + NV_EXG_ADS1292R_2_CONFIG1, args[2]);
            }
            else
            {
                memcpy((storedConfig + NV_EXG_ADS1292R_1_CONFIG1 + args[1]),
                       (args + 3), args[2]);

                if ((daughtCardId[DAUGHT_CARD_ID] == 47)
                        && (daughtCardId[DAUGHT_CARD_REV] >= 4))
                {
                    /* Check if unit is SR47-4 or greater.
                     * If so, amend configuration byte 2 of ADS chip 1 to have bit 3 set to 1.
                     * This ensures clock lines on ADS chip are correct
                     */
                    *(storedConfig + NV_EXG_ADS1292R_1_CONFIG2) |= 8;
                }

                InfoMem_write(
                        (void*) (NV_EXG_ADS1292R_1_CONFIG1 + args[1]),
                        (storedConfig + NV_EXG_ADS1292R_1_CONFIG1 + args[1]),
                        args[2]);
                memcpy(sdHeadText + SDH_EXG_ADS1292R_1_CONFIG1,
                       storedConfig + NV_EXG_ADS1292R_1_CONFIG1, args[2]);
            }
            update_sdconfig = 1;
        }
        break;
    case RESET_TO_DEFAULT_CONFIGURATION_COMMAND:
        SetDefaultConfiguration();
        Config2SdHead();
        update_sdconfig = 1;
        configureChannels = 1;
        if (sensing)
        {
            stopSensing = 1;
            //startSensing = 1;
            SetStartSensing();
        }
        break;
    case RESET_CALIBRATION_VALUE_COMMAND:
        //      memset(&storedConfig[NV_A_ACCEL_CALIBRATION], 0xFF, NV_NUM_CALIBRATION_BYTES);
        //      InfoMem_write((void*)NV_A_ACCEL_CALIBRATION, &storedConfig[NV_A_ACCEL_CALIBRATION], NV_NUM_CALIBRATION_BYTES);
        //      memcpy(&sdHeadText[SDH_LSM303DLHC_ACCEL_CALIBRATION], &storedConfig[NV_LSM303DLHC_ACCEL_CALIBRATION], 21);
        //      memcpy(&sdHeadText[SDH_MPU9150_GYRO_CALIBRATION], &storedConfig[NV_MPU9150_GYRO_CALIBRATION], 21);
        //      memcpy(&sdHeadText[SDH_LSM303DLHC_MAG_CALIBRATION], &storedConfig[NV_LSM303DLHC_MAG_CALIBRATION], 21);
        //      memcpy(&sdHeadText[SDH_A_ACCEL_CALIBRATION], &storedConfig[NV_A_ACCEL_CALIBRATION], 21);
        ShimmerCalib_init();
        ShimmerCalibSyncFromDumpRamAll();
        update_calib_dump_file = 1;
        break;
    case GET_LSM303DLHC_ACCEL_CALIBRATION_COMMAND:
        dAccelCalibrationResponse = 1;
        break;
    case GET_GSR_RANGE_COMMAND:
        gsrRangeResponse = 1;
        break;
    case GET_ALL_CALIBRATION_COMMAND:
        allCalibrationResponse = 1;
        break;
    case DEPRECATED_GET_DEVICE_VERSION_COMMAND:
    case GET_DEVICE_VERSION_COMMAND:
        deviceVersionResponse = 1;
        break;
    case GET_FW_VERSION_COMMAND:
        fwVersionResponse = 1;
        break;
    case GET_CHARGE_STATUS_LED_COMMAND:
        blinkLedResponse = 1;
        break;
    case GET_BUFFER_SIZE_COMMAND:
        bufferSizeResponse = 1;
        break;
    case GET_UNIQUE_SERIAL_COMMAND:
        uniqueSerialResponse = 1;
        break;
    case GET_DAUGHTER_CARD_ID_COMMAND:
        dcMemLength = args[0];
        dcMemOffset = args[1];
        if ((dcMemLength <= 16) && (dcMemOffset <= 15)
                && (dcMemLength + dcMemOffset <= 16))
            dcIdResponse = 1;
        break;

    case SET_DAUGHTER_CARD_ID_COMMAND:
#if FACTORY_TEST
        dcMemLength = args[0];
        dcMemOffset = args[1];
        if ((dcMemLength <= 16) && (dcMemOffset <= 15)
                && (dcMemLength + dcMemOffset <= 16))
        {
            CAT24C16_init();
            CAT24C16_write(dcMemOffset, dcMemLength, args + 2);
            CAT24C16_powerOff();
        }
        break;
#endif
    case GET_DAUGHTER_CARD_MEM_COMMAND:
        dcMemLength = args[0];
        dcMemOffset = args[1] + (args[2] << 8);
        if ((dcMemLength <= 128) && (dcMemOffset <= 2031)
                && (dcMemLength + dcMemOffset <= 2032))
            dcMemResponse = 1;
        break;
    case SET_DAUGHTER_CARD_MEM_COMMAND:
        dcMemLength = args[0];
        dcMemOffset = args[1] + (args[2] << 8);
        if ((dcMemLength <= 128) && (dcMemOffset <= 2031)
                && (dcMemLength + dcMemOffset <= 2032))
        {
            CAT24C16_init();
            CAT24C16_write(dcMemOffset + 16, dcMemLength, args + 3);
            CAT24C16_powerOff();
        }
        break;
    case GET_BT_COMMS_BAUD_RATE:
        btCommsBaudRateResponse = 1;
        break;
    case SET_BT_COMMS_BAUD_RATE:
        if (args[0] != storedConfig[NV_BT_COMMS_BAUD_RATE])
        {
            if (args[0] < 11)
            {
                changeBtBaudRate = args[0];
            }
            else
            {
                changeBtBaudRate = 9;
            }
        }
        break;
    case GET_DERIVED_CHANNEL_BYTES:
        derivedChannelResponse = 1;
        break;
    case SET_DERIVED_CHANNEL_BYTES:
        memcpy(&storedConfig[NV_DERIVED_CHANNELS_0], &args[0], 3);
        memcpy(&storedConfig[NV_DERIVED_CHANNELS_3], &args[3], 5);
        InfoMem_write((void*) NV_DERIVED_CHANNELS_0,
                      &storedConfig[NV_DERIVED_CHANNELS_0], 3);
        InfoMem_write((void*) NV_DERIVED_CHANNELS_3,
                      &storedConfig[NV_DERIVED_CHANNELS_3], 5);
        memcpy(&sdHeadText[SDH_DERIVED_CHANNELS_0],
               &storedConfig[NV_DERIVED_CHANNELS_0], 3);
        memcpy(&sdHeadText[SDH_DERIVED_CHANNELS_3],
               &storedConfig[NV_DERIVED_CHANNELS_3], 5);
        update_sdconfig = 1;
        break;
    case GET_INFOMEM_COMMAND:
        infomemLength = args[0];
        infomemOffset = args[1] + (args[2] << 8);
        if ((infomemLength <= 128)
                && (infomemOffset <= (NV_NUM_RWMEM_BYTES - 1))
                && (infomemLength + infomemOffset <= NV_NUM_RWMEM_BYTES))
            infomemResponse = 1;
        break;
    case SET_INFOMEM_COMMAND:
    {
        // Update SD_ERROR status bit after bt config
        sdErrorFlash =
                (storedConfig[NV_SD_TRIAL_CONFIG0] & SDH_SDERROR_EN) ? 1 : 0;
        infomemLength = args[0];
        infomemOffset = args[1] + (args[2] << 8);
        if ((infomemLength <= 128)
                && (infomemOffset <= (NV_NUM_RWMEM_BYTES - 1))
                && (infomemLength + infomemOffset <= NV_NUM_RWMEM_BYTES))
        {
            //memcpy(storedConfig+infomemOffset, args+3, infomemLength);
            //InfoMem_write((void*)(infomemOffset), (storedConfig+infomemOffset), infomemLength);

//            InfoMem_read((uint8_t *) NV_MAC_ADDRESS, macAddr, 6);
//            InfoMem_write((void*) infomemOffset, args + 3, infomemLength);
//            InfoMem_write((uint8_t*) NV_MAC_ADDRESS, macAddr, 6);
//            InfoMem_read((uint8_t *) infomemOffset,
//                         storedConfig + infomemOffset, infomemLength);

            if (infomemOffset == (INFOMEM_SEG_C_ADDR - INFOMEM_OFFSET))
            {
                /* Read MAC address so it is not forgotten */
                InfoMem_read((uint8_t *) NV_MAC_ADDRESS, macAddr, 6);
            }
            if (infomemOffset == (INFOMEM_SEG_D_ADDR - INFOMEM_OFFSET))
            {
                /* Check if unit is SR47-4 or greater.
                 * If so, amend configuration byte 2 of ADS chip 1 to have bit 3 set to 1.
                 * This ensures clock lines on ADS chip are correct
                 */
                if ((daughtCardId[DAUGHT_CARD_ID] == 47)
                        && (daughtCardId[DAUGHT_CARD_REV] >= 4))
                {
                    *(args + 3 + NV_EXG_ADS1292R_1_CONFIG2) |= 8;
                }
            }

            InfoMem_write((void*) infomemOffset, args + 3, infomemLength);

            if (infomemOffset == (INFOMEM_SEG_C_ADDR - INFOMEM_OFFSET))
            {
                /* Re-write MAC address to Infomem */
                InfoMem_write((uint8_t*) NV_MAC_ADDRESS, macAddr, 6);
            }

            InfoMem_read((uint8_t *) infomemOffset,
                         storedConfig + infomemOffset, infomemLength);

            Config2SdHead();
            Infomem2Names();
            configureChannels = 1;
            update_sdconfig = 1;
            if (((infomemOffset >= NV_A_ACCEL_CALIBRATION)
                    && (infomemOffset <= NV_CALIBRATION_END))
                    || (((infomemLength + infomemOffset)
                            >= NV_A_ACCEL_CALIBRATION)
                            && ((infomemLength + infomemOffset)
                                    <= NV_CALIBRATION_END))
                    || ((infomemOffset <= NV_A_ACCEL_CALIBRATION)
                            && ((infomemLength + infomemOffset)
                                    >= NV_CALIBRATION_END)))
            {
                //update_calib = 1;
                ShimmerCalibUpdateFromInfoAll();
                update_calib_dump_file = 1;
            }
        }
        else
            return;
        break;
    }
    case GET_RWC_COMMAND:
        rwcResponse = 1;
        break;
    case SET_RWC_COMMAND:
        memcpy((uint8_t*) (&rwcConfigTime64), args, 8);       // 64bits = 8bytes
        rwcTimeDiff64 = rwcConfigTime64 - RTC_get64(); // this is the offset to be stored int the sd header
        RwcCheck();
        storedConfig[NV_SD_TRIAL_CONFIG0] |= SDH_RTC_SET_BY_BT;
        InfoMem_write((uint8_t*) NV_SD_TRIAL_CONFIG0,
                      &storedConfig[NV_SD_TRIAL_CONFIG0], 1);
        sdHeadText[SDH_TRIAL_CONFIG0] = storedConfig[NV_SD_TRIAL_CONFIG0];
        sdHeadText[SDH_RTC_DIFF_7] = *((uint8_t*) &rwcTimeDiff64);
        sdHeadText[SDH_RTC_DIFF_6] = *(((uint8_t*) &rwcTimeDiff64) + 1);
        sdHeadText[SDH_RTC_DIFF_5] = *(((uint8_t*) &rwcTimeDiff64) + 2);
        sdHeadText[SDH_RTC_DIFF_4] = *(((uint8_t*) &rwcTimeDiff64) + 3);
        sdHeadText[SDH_RTC_DIFF_3] = *(((uint8_t*) &rwcTimeDiff64) + 4);
        sdHeadText[SDH_RTC_DIFF_2] = *(((uint8_t*) &rwcTimeDiff64) + 5);
        sdHeadText[SDH_RTC_DIFF_1] = *(((uint8_t*) &rwcTimeDiff64) + 6);
        sdHeadText[SDH_RTC_DIFF_0] = *(((uint8_t*) &rwcTimeDiff64) + 7);
        break;
    default:
        ;
    }
    sendAck = 1;
    sendResponse = 1;
    //   if(update_sdconfig_manual && CheckSdInslot()){
    //      if(!docked){
    //         UpdateSdConfig();
    //         SetSdCfgFlag(0);
    //      }else{
    //         SetSdCfgFlag(1);
    //         sdlogcfgUpdate = 0;
    //      }
    //   }
    if (update_sdconfig)
    {
        SetSdCfgFlag(1);
    }
    if (update_calib && CheckSdInslot() && !SD_ERROR)
    {
        if (!docked)
        {
            CalibNewFile(calib_sensor, calib_range);
        }
        else
        {
            SetCalibFlag(1);
        }
        //   CalibAll();
        //else
        //   SetCalibFlag(1);
    }
    if (update_calib_dump_file && CheckSdInslot() && !SD_ERROR)
    {
        if (!docked)
        {
            ShimmerCalib_ram2File();
        }
        else
        {
            SetRamCalibFlag(1);
        }
    }
}
void SendResponse(void)
{
    sc_t sc1;
    uint16_t packet_length = 0;

    if (btIsConnected)
    {
        if (sendAck)
        {
            *(resPacket + packet_length++) = ACK_COMMAND_PROCESSED;
            sendAck = 0;
        }
        if (inquiryResponse)
        {
            *(resPacket + packet_length++) = INQUIRY_RESPONSE;
            *(uint16_t *) (resPacket + packet_length) =
                    *(uint16_t *) (storedConfig + NV_SAMPLING_RATE); //ADC sampling rate
            packet_length += 2;
            memcpy((resPacket + packet_length),
                   (storedConfig + NV_CONFIG_SETUP_BYTE0), 4); //4 config bytes
            packet_length += 4;
            *(resPacket + packet_length++) = nbrAdcChans + nbrDigiChans; //number of data channels
            *(resPacket + packet_length++) = storedConfig[NV_BUFFER_SIZE]; //buffer size
            memcpy((resPacket + packet_length), channelContents,
                   (nbrAdcChans + nbrDigiChans));
            packet_length += nbrAdcChans + nbrDigiChans;
            inquiryResponse = 0;
        }
        else if (samplingRateResponse)
        {
            *(resPacket + packet_length++) = SAMPLING_RATE_RESPONSE;
            *(uint16_t *) (resPacket + packet_length) =
                    *(uint16_t *) (storedConfig + NV_SAMPLING_RATE); //ADC sampling rate
            packet_length += 2;
            samplingRateResponse = 0;
        }
        else if (lsm303dlhcAccelRangeResponse)
        {
            *(resPacket + packet_length++) = LSM303DLHC_ACCEL_RANGE_RESPONSE;
            *(resPacket + packet_length++) =
                    (storedConfig[NV_CONFIG_SETUP_BYTE0] & 0x0C) >> 2;
            lsm303dlhcAccelRangeResponse = 0;
        }
        else if (lsm303dlhcMagGainResponse)
        {
            *(resPacket + packet_length++) = LSM303DLHC_MAG_GAIN_RESPONSE;
            *(resPacket + packet_length++) =
                    (storedConfig[NV_CONFIG_SETUP_BYTE2] & 0xE0) >> 5;
            lsm303dlhcMagGainResponse = 0;
        }
        else if (lsm303dlhcMagSamplingRateResponse)
        {
            *(resPacket + packet_length++) =
            LSM303DLHC_MAG_SAMPLING_RATE_RESPONSE;
            *(resPacket + packet_length++) =
                    (storedConfig[NV_CONFIG_SETUP_BYTE2] & 0x1C) >> 2;
            lsm303dlhcMagSamplingRateResponse = 0;
        }
        else if (dockedResponse)
        {
            *(resPacket + packet_length++) = INSTREAM_CMD_RESPONSE;
            *(resPacket + packet_length++) = STATUS_RESPONSE;
            *(resPacket + packet_length++) = ((toggleLedRed & 0x01) << 7)
                    + ((SD_ERROR & 0x01) << 6) + ((SD_IN_SLOT & 0x01) << 5)
                    + ((isStreaming & 0x01) << 4) + ((isLogging & 0x01) << 3)
                    + ((((rwcConfigTime64 > 0) ? 1 : 0) & 0x01) << 2)
                    + ((sensing & 0x01) << 1) + (docked & 0x01);

            dockedResponse = 0;
        }
        else if (btVbattResponse)
        {
            ReadBatt();
            *(resPacket + packet_length++) = INSTREAM_CMD_RESPONSE;
            *(resPacket + packet_length++) = VBATT_RESPONSE;
            memcpy((resPacket + packet_length), battVal, 3);
            packet_length += 3;
            btVbattResponse = 0;
        }
        else if (trialConfigResponse)
        {
            *(resPacket + packet_length++) = TRIAL_CONFIG_RESPONSE;
            memcpy((resPacket + packet_length),
                   (storedConfig + NV_SD_TRIAL_CONFIG0), 3); //2 trial config bytes + 1 interval byte
            packet_length += 3;
            trialConfigResponse = 0;
        }
        else if (centerResponse)
        {
            centerResponse = 0;
        }
        else if (shimmerNameResponse)
        {
            SetShimmerName();
            uint8_t shimmer_name_len = strlen((char*) shimmerName);
            *(resPacket + packet_length++) = SHIMMERNAME_RESPONSE;
            *(resPacket + packet_length++) = shimmer_name_len;
            memcpy((resPacket + packet_length), shimmerName, shimmer_name_len);
            packet_length += shimmer_name_len;
            shimmerNameResponse = 0;
        }
        else if (expIDResponse)
        {
            SetExpIdName();
            uint8_t exp_id_name_len = strlen((char*) expIdName);
            *(resPacket + packet_length++) = EXPID_RESPONSE;
            *(resPacket + packet_length++) = exp_id_name_len;
            memcpy((resPacket + packet_length), expIdName, exp_id_name_len);
            packet_length += exp_id_name_len;
            expIDResponse = 0;
        }
        else if (configTimeResponse)
        {
            SetCfgTime();
            uint8_t cfgtime_name_len = strlen((char*) configTimeText);
            *(resPacket + packet_length++) = CONFIGTIME_RESPONSE;
            *(resPacket + packet_length++) = cfgtime_name_len;
            memcpy((resPacket + packet_length), configTimeText,
                   cfgtime_name_len);
            packet_length += cfgtime_name_len;
            configTimeResponse = 0;
        }
        else if (dirResponse)
        {
            uint8_t dir_len = strlen((char*) fileName) - 3;
            *(resPacket + packet_length++) = INSTREAM_CMD_RESPONSE;
            *(resPacket + packet_length++) = DIR_RESPONSE;
            *(resPacket + packet_length++) = dir_len;
            memcpy((resPacket + packet_length), fileName, dir_len);
            packet_length += dir_len;
            dirResponse = 0;
        }
        else if (nshimmerResponse)
        {
            *(resPacket + packet_length++) = NSHIMMER_RESPONSE;
            *(resPacket + packet_length++) = storedConfig[NV_SD_NSHIMMER];
            nshimmerResponse = 0;
        }
        else if (myIDResponse)
        {
            *(resPacket + packet_length++) = MYID_RESPONSE;
            *(resPacket + packet_length++) = storedConfig[NV_SD_MYTRIAL_ID];
            myIDResponse = 0;
        }
        else if (lsm303dlhcAccelSamplingRateResponse)
        {
            *(resPacket + packet_length++) =
            LSM303DLHC_ACCEL_SAMPLING_RATE_RESPONSE;
            *(resPacket + packet_length++) =
                    (storedConfig[NV_CONFIG_SETUP_BYTE0] & 0xF0) >> 4;
            lsm303dlhcAccelSamplingRateResponse = 0;
        }
        else if (lsm303dlhcAccelLPModeResponse)
        {
            *(resPacket + packet_length++) = LSM303DLHC_ACCEL_LPMODE_RESPONSE;
            *(resPacket + packet_length++) =
                    (storedConfig[NV_CONFIG_SETUP_BYTE0] & 0x02) >> 1;
            lsm303dlhcAccelLPModeResponse = 0;
        }
        else if (lsm303dlhcAccelHRModeResponse)
        {
            *(resPacket + packet_length++) = LSM303DLHC_ACCEL_HRMODE_RESPONSE;
            *(resPacket + packet_length++) = storedConfig[NV_CONFIG_SETUP_BYTE0]
                    & 0x01;
            lsm303dlhcAccelHRModeResponse = 0;
        }
        else if (mpu9150GyroRangeResponse)
        {
            *(resPacket + packet_length++) = MPU9150_GYRO_RANGE_RESPONSE;
            *(resPacket + packet_length++) = storedConfig[NV_CONFIG_SETUP_BYTE2]
                    & 0x03;
            mpu9150GyroRangeResponse = 0;
        }
        else if (bmp180CalibrationCoefficientsResponse)
        {
            *(resPacket + packet_length++) =
            BMP180_CALIBRATION_COEFFICIENTS_RESPONSE;

            if (bmpInUse == BMP280_IN_USE)
            {
                // Dummy bytes sent if incorrect calibration bytes requested.
                memset(bmpX80Calib, 0x01, BMP180_CALIB_DATA_SIZE);
            }
            memcpy(resPacket + packet_length, bmpX80Calib,
            BMP180_CALIB_DATA_SIZE);
            packet_length += BMP180_CALIB_DATA_SIZE;

            bmp180CalibrationCoefficientsResponse = 0;
        }
        else if (bmp280CalibrationCoefficientsResponse)
        {
            if (bmpInUse == BMP280_IN_USE)
            {
                *(resPacket + packet_length++) =
                BMP280_CALIBRATION_COEFFICIENTS_RESPONSE;
                memcpy(resPacket + packet_length, bmpX80Calib,
                BMP280_CALIB_DATA_SIZE);
                packet_length += BMP280_CALIB_DATA_SIZE;
            }
            bmp280CalibrationCoefficientsResponse = 0;
        }
        else if (mpu9150SamplingRateResponse)
        {
            *(resPacket + packet_length++) = MPU9150_SAMPLING_RATE_RESPONSE;
            *(resPacket + packet_length++) =
                    storedConfig[NV_CONFIG_SETUP_BYTE1];
            mpu9150SamplingRateResponse = 0;
        }
        else if (mpu9150AccelRangeResponse)
        {
            *(resPacket + packet_length++) = MPU9150_ACCEL_RANGE_RESPONSE;
            *(resPacket + packet_length++) =
                    (storedConfig[NV_CONFIG_SETUP_BYTE3] & 0xC0) >> 6;
            mpu9150AccelRangeResponse = 0;
        }
        else if (bmpX80OversamplingRatioResponse)
        {
            *(resPacket + packet_length++) =
            BMPX80_PRES_OVERSAMPLING_RATIO_RESPONSE;
            *(resPacket + packet_length++) =
                    (storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x30) >> 4;
            bmpX80OversamplingRatioResponse = 0;
        }
        else if (internalExpPowerEnableResponse)
        {
            *(resPacket + packet_length++) = INTERNAL_EXP_POWER_ENABLE_RESPONSE;
            *(resPacket + packet_length++) = storedConfig[NV_CONFIG_SETUP_BYTE3]
                    & 0x01;
            internalExpPowerEnableResponse = 0;
        }
        else if (configSetupBytesResponse)
        {
            *(resPacket + packet_length++) = CONFIG_SETUP_BYTES_RESPONSE;
            memcpy((resPacket + packet_length),
                   &storedConfig[NV_CONFIG_SETUP_BYTE0], 4);
            packet_length += 4;
            configSetupBytesResponse = 0;
        }
        else if (calibRamResponse)
        {
            *(resPacket + packet_length++) = RSP_CALIB_DUMP_COMMAND;
            *(resPacket + packet_length++) = calibRamLength;
            *(resPacket + packet_length++) = calibRamOffset & 0xff;
            *(resPacket + packet_length++) = (calibRamOffset >> 8) & 0xff;
            ShimmerCalib_ramRead(resPacket + packet_length, calibRamLength,
                                 calibRamOffset);
            packet_length += calibRamLength;
            calibRamResponse = 0;
        }
        else if (aAccelCalibrationResponse)
        {
            *(resPacket + packet_length++) = A_ACCEL_CALIBRATION_RESPONSE;
            sc1.id = SC_SENSOR_ANALOG_ACCEL;
            sc1.range = SC_SENSOR_RANGE_ANALOG_ACCEL;
            sc1.data_len = SC_DATA_LEN_ANALOG_ACCEL;
            ShimmerCalib_singleSensorRead(&sc1);
            memcpy((resPacket + packet_length), sc1.data.raw, sc1.data_len);
            packet_length += sc1.data_len;
            aAccelCalibrationResponse = 0;
        }
        else if (gyroCalibrationResponse)
        {
            *(resPacket + packet_length++) = MPU9150_GYRO_CALIBRATION_RESPONSE;
            sc1.id = SC_SENSOR_MPU9150_GYRO;
            sc1.range = storedConfig[NV_CONFIG_SETUP_BYTE2] & 0x03;
            sc1.data_len = SC_DATA_LEN_MPU9250_GYRO;
            ShimmerCalib_singleSensorRead(&sc1);
            memcpy((resPacket + packet_length), sc1.data.raw, sc1.data_len);
            packet_length += sc1.data_len;
            gyroCalibrationResponse = 0;
        }
        else if (magCalibrationResponse)
        {
            *(resPacket + packet_length++) =
            LSM303DLHC_MAG_CALIBRATION_RESPONSE;
            sc1.id = SC_SENSOR_LSM303DLHC_MAG;
            sc1.range = (storedConfig[NV_CONFIG_SETUP_BYTE2] >> 5) & 0x07;
            sc1.data_len = SC_DATA_LEN_LSM303DLHC_MAG;
            ShimmerCalib_singleSensorRead(&sc1);
            memcpy((resPacket + packet_length), sc1.data.raw, sc1.data_len);
            packet_length += sc1.data_len;
            magCalibrationResponse = 0;
        }
        else if (dAccelCalibrationResponse)
        {
            *(resPacket + packet_length++) =
            LSM303DLHC_ACCEL_CALIBRATION_RESPONSE;
            sc1.id = SC_SENSOR_LSM303DLHC_ACCEL;
            sc1.range = (storedConfig[NV_CONFIG_SETUP_BYTE0] >> 2) & 0x03;
            sc1.data_len = SC_DATA_LEN_LSM303DLHC_ACCEL;
            ShimmerCalib_singleSensorRead(&sc1);
            memcpy((resPacket + packet_length), sc1.data.raw, sc1.data_len);
            packet_length += sc1.data_len;
            dAccelCalibrationResponse = 0;
        }
        else if (gsrRangeResponse)
        {
            *(resPacket + packet_length++) = GSR_RANGE_RESPONSE;
            *(resPacket + packet_length++) =
                    (storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x0E) >> 1;
            gsrRangeResponse = 0;
        }
        else if (allCalibrationResponse)
        {
            *(resPacket + packet_length++) = ALL_CALIBRATION_RESPONSE;
            //         memcpy((resPacket+packet_length), &storedConfig[NV_A_ACCEL_CALIBRATION], NV_NUM_CALIBRATION_BYTES);
            //         packet_length += NV_NUM_CALIBRATION_BYTES;
            uint8_t i;
            for (i = 0; i < 4; i++)
            {
                if (i == 0)
                {
                    sc1.id = SC_SENSOR_ANALOG_ACCEL;
                    sc1.range = 0;
                    sc1.data_len = SC_DATA_LEN_ANALOG_ACCEL;
                }
                else if (i == 1)
                {
                    sc1.id = SC_SENSOR_MPU9150_GYRO;
                    sc1.range = storedConfig[NV_CONFIG_SETUP_BYTE2] & 0x03;
                    sc1.data_len = SC_DATA_LEN_MPU9250_GYRO;
                }
                else if (i == 2)
                {
                    sc1.id = SC_SENSOR_LSM303DLHC_MAG;
                    sc1.range = (storedConfig[NV_CONFIG_SETUP_BYTE2] >> 5)
                            & 0x07;
                    sc1.data_len = SC_DATA_LEN_LSM303DLHC_MAG;
                }
                else if (i == 3)
                {
                    sc1.id = SC_SENSOR_LSM303DLHC_ACCEL;
                    sc1.range = (storedConfig[NV_CONFIG_SETUP_BYTE0] >> 2)
                            & 0x03;
                    sc1.data_len = SC_DATA_LEN_LSM303DLHC_ACCEL;
                }
                ShimmerCalib_singleSensorRead(&sc1);
                memcpy((resPacket + packet_length), sc1.data.raw, sc1.data_len);
                packet_length += sc1.data_len;
            }

            allCalibrationResponse = 0;
        }
        else if (deviceVersionResponse)
        {
            *(resPacket + packet_length++) = DEVICE_VERSION_RESPONSE;
            *(resPacket + packet_length++) = DEVICE_VER;
            deviceVersionResponse = 0;
        }
        else if (mpu9150MagSensAdjValsResponse)
        {
            MPU9150_init();
            MPU9150_wake(1);
            MPU9150_wake(0);
            *(resPacket + packet_length++) = MPU9150_MAG_SENS_ADJ_VALS_RESPONSE;
            MPU9150_getMagSensitivityAdj(resPacket + packet_length);
            packet_length += 3;
            mpu9150MagSensAdjValsResponse = 0;
        }
        else if (fwVersionResponse)
        {
            *(resPacket + packet_length++) = FW_VERSION_RESPONSE;
            *(resPacket + packet_length++) = FW_IDENTIFIER & 0xFF;
            *(resPacket + packet_length++) = (FW_IDENTIFIER & 0xFF00) >> 8;
            *(resPacket + packet_length++) = FW_VER_MAJOR & 0xFF;
            *(resPacket + packet_length++) = (FW_VER_MAJOR & 0xFF00) >> 8;
            *(resPacket + packet_length++) = FW_VER_MINOR;
            *(resPacket + packet_length++) = FW_VER_REL + ((FACTORY_TEST) ? 200 : 0);
            fwVersionResponse = 0;
        }
        else if (blinkLedResponse)
        {
            *(resPacket + packet_length++) = CHARGE_STATUS_LED_RESPONSE;
            *(resPacket + packet_length++) = battStat;
            blinkLedResponse = 0;
        }
        else if (bufferSizeResponse)
        {
            *(resPacket + packet_length++) = BUFFER_SIZE_RESPONSE;
            *(resPacket + packet_length++) = storedConfig[NV_BUFFER_SIZE];
            bufferSizeResponse = 0;
        }
        else if (uniqueSerialResponse)
        {
            *(resPacket + packet_length++) = UNIQUE_SERIAL_RESPONSE;
            memcpy((resPacket + packet_length), dierecord, 8);
            packet_length += 8;
            uniqueSerialResponse = 0;
        }
        else if (exgRegsResponse)
        {
            *(resPacket + packet_length++) = EXG_REGS_RESPONSE;
            *(resPacket + packet_length++) = exgLength;
            if (exgLength)
            {
                if (exgChip)
                    memcpy((resPacket + packet_length),
                           (storedConfig + NV_EXG_ADS1292R_2_CONFIG1
                                   + exgStartAddr),
                           exgLength);
                else
                    memcpy((resPacket + packet_length),
                           (storedConfig + NV_EXG_ADS1292R_1_CONFIG1
                                   + exgStartAddr),
                           exgLength);
                packet_length += exgLength;
            }
            exgRegsResponse = 0;
        }
        else if (dcIdResponse)
        {
            *(resPacket + packet_length++) = DAUGHTER_CARD_ID_RESPONSE;
            *(resPacket + packet_length++) = dcMemLength;
            //CAT24C16_init();
            //CAT24C16_read(dcMemOffset, dcMemLength, (resPacket+packet_length));
            //CAT24C16_powerOff();
            memcpy(resPacket + packet_length, daughtCardId + dcMemOffset,
                   dcMemLength);
            packet_length += dcMemLength;
            dcIdResponse = 0;
        }
        else if (dcMemResponse)
        {
            *(resPacket + packet_length++) = DAUGHTER_CARD_MEM_RESPONSE;
            *(resPacket + packet_length++) = dcMemLength;
            if (!sensing)
            {
                CAT24C16_init();
                CAT24C16_read(dcMemOffset + 16, dcMemLength,
                              (resPacket + packet_length));
                CAT24C16_powerOff();
            }
            else
            {
                memset(resPacket + packet_length, 0xff, dcMemLength);
            }
            packet_length += dcMemLength;
            dcMemResponse = 0;
        }
        else if (btCommsBaudRateResponse)
        {
            *(resPacket + packet_length++) = BT_COMMS_BAUD_RATE_RESPONSE;
            *(resPacket + packet_length++) =
                    storedConfig[NV_BT_COMMS_BAUD_RATE];
            btCommsBaudRateResponse = 0;
        }
        else if (infomemResponse)
        {
            *(resPacket + packet_length++) = INFOMEM_RESPONSE;
            *(resPacket + packet_length++) = infomemLength;
            memcpy((resPacket + packet_length), &storedConfig[infomemOffset],
                   infomemLength);
            packet_length += infomemLength;
            infomemResponse = 0;
        }
        else if (derivedChannelResponse)
        {
            *(resPacket + packet_length++) = DERIVED_CHANNEL_BYTES_RESPONSE;
            *(resPacket + packet_length++) =
                    storedConfig[NV_DERIVED_CHANNELS_0];
            *(resPacket + packet_length++) =
                    storedConfig[NV_DERIVED_CHANNELS_1];
            *(resPacket + packet_length++) =
                    storedConfig[NV_DERIVED_CHANNELS_2];
            *(resPacket + packet_length++) =
                    storedConfig[NV_DERIVED_CHANNELS_3];
            *(resPacket + packet_length++) =
                    storedConfig[NV_DERIVED_CHANNELS_4];
            *(resPacket + packet_length++) =
                    storedConfig[NV_DERIVED_CHANNELS_5];
            *(resPacket + packet_length++) =
                    storedConfig[NV_DERIVED_CHANNELS_6];
            *(resPacket + packet_length++) =
                    storedConfig[NV_DERIVED_CHANNELS_7];
            derivedChannelResponse = 0;
        }
        else if (rwcResponse)
        {
            uint64_t rwc_curr_time_64;
            rwc_curr_time_64 = rwcTimeDiff64 + RTC_get64();
            *(resPacket + packet_length++) = RWC_RESPONSE;
            memcpy(resPacket + packet_length, (uint8_t*) (&rwc_curr_time_64),
                   8);
            packet_length += 8;
            rwcResponse = 0;
        }
    }

    if (crcChecksum)
    {
        uint16_t crc_value;
        crc_value = CRC_data(resPacket, (uint8_t) packet_length);
        *(resPacket + packet_length++) = crc_value & 0xFF;
        *(resPacket + packet_length++) = (crc_value & 0xFF00) >> 8;
    }
    BT_write(resPacket, packet_length);
}

//void OpenFirstFile(){
//}

void Timestamp0ToFirstFile()
{
    UINT bw;
    uint32_t my_local_time_long;
//   uint64_t rwc_curr_time_64;
//   rwc_curr_time_64 = RTC_get64();
    my_local_time_long = firstTs & 0xffffffff;
    sdHeadText[SDH_MY_LOCALTIME_5TH] = (firstTs >> 32) & 0xff;
    ;
    memcpy(&sdHeadText[SDH_MY_LOCALTIME], (uint8_t*) &my_local_time_long, 4);
//   fileLastHour = rwc_curr_time_64;
//   fileLastMin = rwc_curr_time_64;
// Write header to file
    ff_result = f_write(&fil, sdHeadText, SDHEAD_LEN, &bw);
}

FRESULT WriteFile(uint8_t* text, WORD size)
{
// Result code
    FRESULT rc;
    UINT bw;
    uint32_t file_td_h, file_td_m;   //, my_local_time_long32
    uint64_t local_time_40;
    wr2sd = 1;

    local_time_40 = RTC_get64();
    f_lseek(&fil, fil.fsize);                  // seek to end of file, no spi op
    rc = f_write(&fil, text, size, &bw);         // Write to file

    file_td_h = local_time_40 - fileLastHour;
    file_td_m = local_time_40 - fileLastMin;

//create a new file (from 000 up) every 1h
    if (file_td_h >= 117964800)
    {         // 117964800 = 32768/s*3600s = 1h
        fileLastHour = fileLastMin = local_time_40;
        rc = f_close(&fil);
        //file number:from 000 up
        fileNum++;

        //avoid using printf()
        fileName[dirLen + 3] = (char) (((int) '0') + fileNum % 10);
        fileName[dirLen + 2] = (char) (((int) '0') + (fileNum / 10) % 10);
        fileName[dirLen + 1] = (char) (((int) '0') + (fileNum / 100) % 10);

        f_open(&fil, (char*) fileName, FA_WRITE | FA_CREATE_NEW);

        //      my_local_time_long32 = local_time_40 & 0xffffffff;
        //      sdHeadText[SDH_MY_LOCALTIME_5TH] = (local_time_40>>32) & 0xff;
        //      memcpy(&sdHeadText[SDH_MY_LOCALTIME], (uint8_t*)&my_local_time_long32, 4);
        //      fileLastMin = local_time_40;
        //      f_write(&fil, sdHeadText, SDHEAD_LEN, &bw);        // Write head to file
        firstTsFlag = 1;
        streamDataInProc = 0;
        streamData = 0;
    }
// sync data to SD card every 1 min
    else if (file_td_m >= 1966080)
    { //32768/s*60s = 1m
        fileLastMin = local_time_40;
        f_sync(&fil);
    }

    ff_result = rc;

    wr2sd = 0;
    return rc;
}
void PrepareSDBuffHead(void)
{
    memcpy(sdBuff + sdBuffLen, myTimeDiff, 5);
    sdBuffLen += 5;
    memset(myTimeDiff, 0xff, 5);
}

void ParseConfig(void)
{
    char buffer[66], *equals;
    uint8_t string_length = 0;
    float sample_rate = 51.2;
    uint16_t sample_period = 0;
    uint64_t derived_channels_val = 0;
    uint8_t accel_lpm = 0, accel_hrm = 0, broadcast_interval, accel_mpu_range =
            0, exp_power = 0;
    uint8_t accel_range = 0, accel_smplrate = 0, gyro_range = 0, mag_smplrate =
            0, mag_gain = 0, pres_bmpX80_prec = 0, gsr_range = 0;
    uint8_t my_trial_id = 0, num_shimmers_in_trial = 0, my_config_time[4],
            config_baudrate = 0;
    uint32_t config_time = 0;
    bool iAmMaster = FALSE, time_sync = FALSE, // singletouch = FALSE,
            tcxo = FALSE, user_button_enable = FALSE, low_battery_autostop = FALSE;

#if !RTC_OFF
    bool rwc_error_enable = TRUE;
#endif
    uint32_t max_exp_len = 0;

    bool sd_error_enable = TRUE;

    CheckSdInslot();
    char cfgname[] = "sdlog.cfg";
    ff_result = f_open(&fil, cfgname, FA_READ | FA_OPEN_EXISTING);
    if (ff_result == FR_NO_FILE)
    {
        IniReadInfoMem();
        UpdateSdConfig();
        //        fileBad = 0;
    }
    else if (ff_result != FR_OK)
    {
        SD_ERROR = TRUE;
        //        fileBad = (initializing) ? 0 : 1;
        return;
    }
    else
    {
        uint8_t stored_config_temp[NV_NUM_RWMEM_BYTES];

        broadcast_interval = RC_INT_C;

        memset((uint8_t*) (stored_config_temp), 0, NV_A_ACCEL_CALIBRATION); //0
        memset((uint8_t*) (stored_config_temp + NV_A_ACCEL_CALIBRATION), 0xff,
               128 - NV_A_ACCEL_CALIBRATION);
        memset((uint8_t*) (stored_config_temp + NV_DERIVED_CHANNELS_3), 0, 5); //0
        memset((uint8_t*) (stored_config_temp + NV_SENSORS3), 0, 5); //0
        memset((uint8_t*) (stored_config_temp + NV_MPL_ACCEL_CALIBRATION), 0xff,
               82);
        memset((uint8_t*) (stored_config_temp + NV_SD_MYTRIAL_ID), 0, 9); //0
        InfoMem_read((uint8_t *) NV_MAC_ADDRESS,
                     stored_config_temp + NV_MAC_ADDRESS, 7);
        memset((uint8_t*) (stored_config_temp + NV_BT_SET_PIN + 1), 0xff, 25);
        memset((uint8_t*) (stored_config_temp + NV_NODE0), 0xff, 128);

        stored_config_temp[NV_SD_TRIAL_CONFIG0] &= ~SDH_SET_PMUX; // PMUX reserved as 0
        stored_config_temp[NV_SD_TRIAL_CONFIG0] |= SDH_TIME_STAMP; // TIME_STAMP always = 1
        stored_config_temp[NV_CONFIG_SETUP_BYTE3] |= (GSR_AUTORANGE & 0x07)
                << 1; //BIT3-1
        stored_config_temp[NV_BUFFER_SIZE] = 1;
        stored_config_temp[NV_BT_COMMS_BAUD_RATE] = 0x09;
        changeBtBaudRate = 0xff; // set this flag to no_event as every sdlog.cfg read is followed by a baudrate setting

        memset(shimmerName, 0, MAX_CHARS);
        memset(expIdName, 0, MAX_CHARS);
        memset(configTimeText, 0, MAX_CHARS);
        *(stored_config_temp + NV_SD_SHIMMER_NAME) = '\0';
        *(stored_config_temp + NV_SD_EXP_ID_NAME) = '\0';
        *configTimeText = '\0';
        memset((uint8_t*) (stored_config_temp + NV_SD_CONFIG_TIME), 0, 4);
        float old_clock_freq = clockFreq;

        while (f_gets(buffer, 64, &fil))
        {
            if (!(equals = strchr(buffer, '=')))
                continue;
            equals++;     // this is the value
            if (strstr(buffer, "accel="))
                stored_config_temp[NV_SENSORS0] |=
                        atoi(equals) * SENSOR_A_ACCEL;
            else if (strstr(buffer, "gyro="))
                stored_config_temp[NV_SENSORS0] |= atoi(
                        equals)*SENSOR_MPU9150_GYRO;
            else if (strstr(buffer, "mag="))
                stored_config_temp[NV_SENSORS0] |= atoi(
                        equals)*SENSOR_LSM303DLHC_MAG;
            else if (strstr(buffer, "exg1_24bit="))
                stored_config_temp[NV_SENSORS0] |= atoi(
                        equals) * SENSOR_EXG1_24BIT;
            else if (strstr(buffer, "exg2_24bit="))
                stored_config_temp[NV_SENSORS0] |= atoi(
                        equals) * SENSOR_EXG2_24BIT;
            else if (strstr(buffer, "gsr="))
                stored_config_temp[NV_SENSORS0] |= atoi(equals) * SENSOR_GSR;
            else if (strstr(buffer, "extch7="))
                stored_config_temp[NV_SENSORS0] |= atoi(equals) * SENSOR_EXT_A7;
            else if (strstr(buffer, "extch6="))
                stored_config_temp[NV_SENSORS0] |= atoi(equals) * SENSOR_EXT_A6;
            else if (strstr(buffer, "str=") || strstr(buffer, "br_amp="))
                stored_config_temp[NV_SENSORS1] |= atoi(equals) * SENSOR_STRAIN;
            else if (strstr(buffer, "vbat="))
                stored_config_temp[NV_SENSORS1] |= atoi(equals) * SENSOR_VBATT;
            else if (strstr(buffer, "accel_d="))
                stored_config_temp[NV_SENSORS1] |= atoi(
                        equals)*SENSOR_LSM303DLHC_ACCEL;
            else if (strstr(buffer, "extch15="))
                stored_config_temp[NV_SENSORS1] |=
                        atoi(equals) * SENSOR_EXT_A15;
            else if (strstr(buffer, "intch1="))
                stored_config_temp[NV_SENSORS1] |= atoi(equals) * SENSOR_INT_A1;
            else if (strstr(buffer, "intch12="))
                stored_config_temp[NV_SENSORS1] |=
                        atoi(equals) * SENSOR_INT_A12;
            else if (strstr(buffer, "intch13="))
                stored_config_temp[NV_SENSORS1] |=
                        atoi(equals) * SENSOR_INT_A13;
            else if (strstr(buffer, "intch14="))
                stored_config_temp[NV_SENSORS2] |=
                        atoi(equals) * SENSOR_INT_A14;
            else if (strstr(buffer, "accel_mpu="))
                stored_config_temp[NV_SENSORS2] |= atoi(
                        equals)*SENSOR_MPU9150_ACCEL;
            else if (strstr(buffer, "mag_mpu="))
                stored_config_temp[NV_SENSORS2] |= atoi(
                        equals) * SENSOR_MPU9150_MAG;
            else if (strstr(buffer, "exg1_16bit="))
                stored_config_temp[NV_SENSORS2] |= atoi(
                        equals) * SENSOR_EXG1_16BIT;
            else if (strstr(buffer, "exg2_16bit="))
                stored_config_temp[NV_SENSORS2] |= atoi(
                        equals) * SENSOR_EXG2_16BIT;
            else if (strstr(buffer, "pres_bmp180=") || strstr(buffer, "pres_bmp280="))
                stored_config_temp[NV_SENSORS2] |= atoi(
                        equals)*SENSOR_BMPX80_PRESSURE;
            else if (strstr(buffer, "sample_rate="))
            {
                sample_rate = atof(equals);
            }
            else if (strstr(buffer, "mg_internal_rate="))
            {
                mag_smplrate = atoi(equals);
                stored_config_temp[NV_CONFIG_SETUP_BYTE2] |= (mag_smplrate
                        & 0x07) << 2;     //BIT4-2
            }
            else if (strstr(buffer, "mg_range="))
            {
                mag_gain = atoi(equals);
                stored_config_temp[NV_CONFIG_SETUP_BYTE2] |= (mag_gain & 0x07)
                        << 5; //BIT7-5
            }
            else if (strstr(buffer, "acc_internal_rate="))
            {
                accel_smplrate = atoi(equals);
                stored_config_temp[NV_CONFIG_SETUP_BYTE0] |= (accel_smplrate
                        & 0x0f) << 4;   //BIT7-4
            }
            else if (strstr(buffer, "accel_mpu_range="))
            {
                accel_mpu_range = atoi(equals);
                stored_config_temp[NV_CONFIG_SETUP_BYTE3] |= (accel_mpu_range
                        & 0x03) << 6;  //BIT7-6
            }
            else if (strstr(buffer, "acc_range="))
            {
                accel_range = atoi(equals);
                stored_config_temp[NV_CONFIG_SETUP_BYTE0] |=
                        (accel_range & 0x03) << 2;      //BIT3-2
            }
            else if (strstr(buffer, "acc_lpm="))
            {
                accel_lpm = atoi(equals);
                stored_config_temp[NV_CONFIG_SETUP_BYTE0] |= (accel_lpm & 0x01)
                        << 1;       //BIT1
            }
            else if (strstr(buffer, "acc_hrm="))
            {
                accel_hrm = atoi(equals);
                stored_config_temp[NV_CONFIG_SETUP_BYTE0] |= accel_hrm & 0x01; //BIT0
            }
            else if (strstr(buffer, "gsr_range="))
            {        // or "gsr_range="?
                gsr_range = atoi(equals);
                if (gsr_range > 4)
                    gsr_range = 4;

                stored_config_temp[NV_CONFIG_SETUP_BYTE3] &= 0xF1;
                stored_config_temp[NV_CONFIG_SETUP_BYTE3] |= ((gsr_range << 1)
                        & GSR_RANGE);
            }
            else if (strstr(buffer, "gyro_samplingrate="))
                stored_config_temp[NV_CONFIG_SETUP_BYTE1] = atoi(equals);
            else if (strstr(buffer, "gyro_range="))
            {
                gyro_range = atoi(equals);
                stored_config_temp[NV_CONFIG_SETUP_BYTE2] |= (gyro_range)
                        & 0x03; //BIT1-0
            }
            else if (strstr(buffer, "pres_bmp180_prec=") || strstr(buffer, "pres_bmp280_prec="))
            {
                pres_bmpX80_prec = atoi(equals);
                stored_config_temp[NV_CONFIG_SETUP_BYTE3] |= (pres_bmpX80_prec
                        & 0x03) << 4;   //BIT5-4
            }
#if !RTC_OFF
            else if (strstr(buffer, "rtc_error_enable="))
            {
                rwc_error_enable = (atoi(equals) == 0) ? FALSE : TRUE;
            }
#endif
            else if (strstr(buffer, "sd_error_enable="))
            {
                sd_error_enable = (atoi(equals) == 0) ? FALSE : TRUE;
                stored_config_temp[NV_SD_TRIAL_CONFIG0] |= sd_error_enable
                        * SDH_SDERROR_EN;
            }
            else if (strstr(buffer, "user_button_enable="))
            {
                user_button_enable = (atoi(equals) == 0) ? FALSE : TRUE;
                stored_config_temp[NV_SD_TRIAL_CONFIG0] |= user_button_enable
                        * SDH_USER_BUTTON_ENABLE;
            }
            else if (strstr(buffer, "iammaster="))
            {   //0=slave=node
                iAmMaster = (atoi(equals) == 0) ? FALSE : TRUE;
                stored_config_temp[NV_SD_TRIAL_CONFIG0] |= iAmMaster
                        * SDH_IAMMASTER;
            }
            else if (strstr(buffer, "sync="))
            {
                time_sync = (atoi(equals) == 0) ? FALSE : TRUE;
                stored_config_temp[NV_SD_TRIAL_CONFIG0] |= time_sync
                        * SDH_TIME_SYNC;
            }
            else if (strstr(buffer, "low_battery_autostop="))
            {
                low_battery_autostop = (atoi(equals) == 0) ? FALSE : TRUE;
                stored_config_temp[NV_SD_TRIAL_CONFIG1] |= low_battery_autostop * SDH_BATT_CRITICAL_CUTOFF;
            }
            else if (strstr(buffer, "tcxo="))
            {
                tcxo = (atoi(equals) == 0) ? FALSE : TRUE;
                stored_config_temp[NV_SD_TRIAL_CONFIG1] |= tcxo * SDH_TCXO;
            }
            else if (strstr(buffer, "interval="))
            {
                broadcast_interval = atoi(equals) > 255 ? 255 : atoi(equals);
            }
            else if (strstr(buffer, "exp_power="))
            {
                exp_power = atoi(equals);
                stored_config_temp[NV_CONFIG_SETUP_BYTE3] |=
                        exp_power ? EXP_POWER_ENABLE : 0;
            }
            else if (strstr(buffer, "center="))
            {
            }
            else if (strstr(buffer, "max_exp_len="))
            {
                max_exp_len = atoi(equals);
                stored_config_temp[NV_MAX_EXP_LEN_MSB] = (max_exp_len & 0xff00)
                        >> 8;
                stored_config_temp[NV_MAX_EXP_LEN_LSB] = max_exp_len & 0xff;
            }
            else if (strstr(buffer, "myid="))
            {
                my_trial_id = atoi(equals);
                stored_config_temp[NV_SD_MYTRIAL_ID] = my_trial_id;
            }
            else if (strstr(buffer, "Nshimmer="))
            {
                num_shimmers_in_trial = atoi(equals);
                stored_config_temp[NV_SD_NSHIMMER] = num_shimmers_in_trial;
            }
            else if (strstr(buffer, "shimmername="))
            {
                string_length = strlen(equals);
                if (string_length > MAX_CHARS)
                    string_length = MAX_CHARS - 1;
                else if (string_length >= 2)
                    string_length -= 2;
                else
                    string_length = 0;
                memcpy((char*) (stored_config_temp + NV_SD_SHIMMER_NAME),
                       equals, string_length);
                if (!memcmp((char*) (stored_config_temp + NV_SD_SHIMMER_NAME),
                            "ID", 2))
                    memcpy((char*) (stored_config_temp + NV_SD_SHIMMER_NAME),
                           "id", 2);
                memcpy((char*) shimmerName,
                       (char*) (stored_config_temp + NV_SD_SHIMMER_NAME),
                       MAX_CHARS - 1);
                shimmerName[string_length] = 0;
            }
            else if (strstr(buffer, "experimentid="))
            {
                string_length = strlen(equals);
                if (string_length > MAX_CHARS)
                    string_length = MAX_CHARS - 1;
                else if (string_length >= 2)
                    string_length -= 2;
                else
                    string_length = 0;
                memcpy((char*) (stored_config_temp + NV_SD_EXP_ID_NAME), equals,
                       string_length);
                memcpy((char*) expIdName,
                       (char*) (stored_config_temp + NV_SD_EXP_ID_NAME),
                       MAX_CHARS - 1);
                expIdName[string_length] = 0;
            }
            else if (strstr(buffer, "configtime="))
            {
                config_time = atol(equals);
                string_length =
                MAX_CHARS < strlen(equals) ?
                MAX_CHARS :
                                             strlen(equals) - 1;
                memcpy((char*) configTimeText, equals, string_length - 1);
                my_config_time[3] = *((uint8_t*) &config_time);
                my_config_time[2] = *(((uint8_t*) &config_time) + 1);
                my_config_time[1] = *(((uint8_t*) &config_time) + 2);
                my_config_time[0] = *(((uint8_t*) &config_time) + 3);
                memcpy((uint8_t*) (stored_config_temp + NV_SD_CONFIG_TIME),
                       my_config_time, 4);
                *(configTimeText + string_length - 1) = 0;
            }
            else if (strstr(buffer, "EXG_ADS1292R_1_CONFIG1="))
                stored_config_temp[NV_EXG_ADS1292R_1_CONFIG1] = atoi(equals);
            else if (strstr(buffer, "EXG_ADS1292R_1_CONFIG2="))
                stored_config_temp[NV_EXG_ADS1292R_1_CONFIG2] = atoi(equals);
            else if (strstr(buffer, "EXG_ADS1292R_1_LOFF="))
                stored_config_temp[NV_EXG_ADS1292R_1_LOFF] = atoi(equals);
            else if (strstr(buffer, "EXG_ADS1292R_1_CH1SET="))
                stored_config_temp[NV_EXG_ADS1292R_1_CH1SET] = atoi(equals);
            else if (strstr(buffer, "EXG_ADS1292R_1_CH2SET="))
                stored_config_temp[NV_EXG_ADS1292R_1_CH2SET] = atoi(equals);
            else if (strstr(buffer, "EXG_ADS1292R_1_RLD_SENS="))
                stored_config_temp[NV_EXG_ADS1292R_1_RLD_SENS] = atoi(equals);
            else if (strstr(buffer, "EXG_ADS1292R_1_LOFF_SENS="))
                stored_config_temp[NV_EXG_ADS1292R_1_LOFF_SENS] = atoi(equals);
            else if (strstr(buffer, "EXG_ADS1292R_1_LOFF_STAT="))
                stored_config_temp[NV_EXG_ADS1292R_1_LOFF_STAT] = atoi(equals);
            else if (strstr(buffer, "EXG_ADS1292R_1_RESP1="))
                stored_config_temp[NV_EXG_ADS1292R_1_RESP1] = atoi(equals);
            else if (strstr(buffer, "EXG_ADS1292R_1_RESP2="))
                stored_config_temp[NV_EXG_ADS1292R_1_RESP2] = atoi(equals);
            else if (strstr(buffer, "EXG_ADS1292R_2_CONFIG1="))
                stored_config_temp[NV_EXG_ADS1292R_2_CONFIG1] = atoi(equals);
            else if (strstr(buffer, "EXG_ADS1292R_2_CONFIG2="))
                stored_config_temp[NV_EXG_ADS1292R_2_CONFIG2] = atoi(equals);
            else if (strstr(buffer, "EXG_ADS1292R_2_LOFF="))
                stored_config_temp[NV_EXG_ADS1292R_2_LOFF] = atoi(equals);
            else if (strstr(buffer, "EXG_ADS1292R_2_CH1SET="))
                stored_config_temp[NV_EXG_ADS1292R_2_CH1SET] = atoi(equals);
            else if (strstr(buffer, "EXG_ADS1292R_2_CH2SET="))
                stored_config_temp[NV_EXG_ADS1292R_2_CH2SET] = atoi(equals);
            else if (strstr(buffer, "EXG_ADS1292R_2_RLD_SENS="))
                stored_config_temp[NV_EXG_ADS1292R_2_RLD_SENS] = atoi(equals);
            else if (strstr(buffer, "EXG_ADS1292R_2_LOFF_SENS="))
                stored_config_temp[NV_EXG_ADS1292R_2_LOFF_SENS] = atoi(equals);
            else if (strstr(buffer, "EXG_ADS1292R_2_LOFF_STAT="))
                stored_config_temp[NV_EXG_ADS1292R_2_LOFF_STAT] = atoi(equals);
            else if (strstr(buffer, "EXG_ADS1292R_2_RESP1="))
                stored_config_temp[NV_EXG_ADS1292R_2_RESP1] = atoi(equals);
            else if (strstr(buffer, "EXG_ADS1292R_2_RESP2="))
                stored_config_temp[NV_EXG_ADS1292R_2_RESP2] = atoi(equals);
            else if (strstr(buffer, "baud_rate="))
            {
                config_baudrate = atoi(equals);
                if (config_baudrate != storedConfig[NV_BT_COMMS_BAUD_RATE])
                {
                    if (config_baudrate < 11)
                    {
                        changeBtBaudRate = config_baudrate;
                    }
                    else
                    {
                        changeBtBaudRate = 9;
                    }
                }
                stored_config_temp[NV_BT_COMMS_BAUD_RATE] =
                        storedConfig[NV_BT_COMMS_BAUD_RATE];
            }
            else if (strstr(buffer, "derived_channels="))
            {
                //            derived_channels_val = atol(equals);
                //            stored_config_temp[NV_DERIVED_CHANNELS_0] = derived_channels_val & 0xff;
                //            stored_config_temp[NV_DERIVED_CHANNELS_1] = (derived_channels_val >> 8) & 0xff;
                //            stored_config_temp[NV_DERIVED_CHANNELS_2] = (derived_channels_val >> 16) & 0xff;
                derived_channels_val = atoll(equals);
                stored_config_temp[NV_DERIVED_CHANNELS_0] = derived_channels_val
                        & 0xff;
                stored_config_temp[NV_DERIVED_CHANNELS_1] =
                        (derived_channels_val >> 8) & 0xff;
                stored_config_temp[NV_DERIVED_CHANNELS_2] =
                        (derived_channels_val >> 16) & 0xff;
                stored_config_temp[NV_DERIVED_CHANNELS_3] =
                        (derived_channels_val >> 24) & 0xff;
                stored_config_temp[NV_DERIVED_CHANNELS_4] =
                        (derived_channels_val >> 32) & 0xff;
                stored_config_temp[NV_DERIVED_CHANNELS_5] =
                        (derived_channels_val >> 40) & 0xff;
                stored_config_temp[NV_DERIVED_CHANNELS_6] =
                        (derived_channels_val >> 48) & 0xff;
                stored_config_temp[NV_DERIVED_CHANNELS_7] =
                        (derived_channels_val >> 56) & 0xff;
            }
        }
        ff_result = f_close(&fil);
        _delay_cycles(1200000);

        maxLen = max_exp_len * 60;
        SamplingClkAssignment();

        if (clockFreq != old_clock_freq)
        {
            ClkAssignment();
            TB0CTL = MC_0; // StopTb0()
            TB0Start();
        }

        sample_period = FreqDiv(sample_rate);

        // little endian:
        stored_config_temp[NV_SAMPLING_RATE] = (uint8_t) (sample_period & 0xFF);
        stored_config_temp[NV_SAMPLING_RATE + 1] =
                (uint8_t) (sample_period >> 8);

        if (stored_config_temp[NV_SENSORS0] & SENSOR_GSR)
        { // they are sharing adc1, so ban intch1 when gsr is on
            stored_config_temp[NV_SENSORS1] &= ~SENSOR_INT_A1;
        }
        if (stored_config_temp[NV_SENSORS1] & SENSOR_STRAIN)
        { // they are sharing adc13 and adc14
            stored_config_temp[NV_SENSORS1] &= ~SENSOR_INT_A13;
            stored_config_temp[NV_SENSORS2] &= ~SENSOR_INT_A14;
        }
        if (stored_config_temp[NV_SENSORS0] & SENSOR_EXG1_24BIT)
        {
            stored_config_temp[NV_SENSORS2] &= ~SENSOR_EXG1_16BIT;
        }
        if (stored_config_temp[NV_SENSORS0] & SENSOR_EXG2_24BIT)
        {
            stored_config_temp[NV_SENSORS2] &= ~SENSOR_EXG2_16BIT;
        }
        if (stored_config_temp[NV_SENSORS0] & SENSOR_EXG1_24BIT
                || stored_config_temp[NV_SENSORS0] & SENSOR_EXG2_24BIT
                || stored_config_temp[NV_SENSORS2] & SENSOR_EXG1_16BIT
                || stored_config_temp[NV_SENSORS2] & SENSOR_EXG2_16BIT)
        {
            stored_config_temp[NV_SENSORS1] &= ~SENSOR_INT_A1;
            stored_config_temp[NV_SENSORS2] &= ~SENSOR_INT_A14;
        }

        if (((stored_config_temp[NV_CONFIG_SETUP_BYTE3] >> 1) & 0x07) > 4)
        { // never larger than 4
            stored_config_temp[NV_CONFIG_SETUP_BYTE3] &= 0xf1;
            stored_config_temp[NV_CONFIG_SETUP_BYTE3] |= (GSR_AUTORANGE & 0x07)
                    << 1;      //BIT3-1
        }
#if !RTC_OFF
        stored_config_temp[NV_SD_TRIAL_CONFIG0] |= rwc_error_enable
                * SDH_RWCERROR_EN;
#endif
        stored_config_temp[NV_SD_TRIAL_CONFIG0] |= sd_error_enable
                * SDH_SDERROR_EN;

        // minimum sync broadcast interval is 54 seconds
        if (broadcast_interval < RC_INT_C)
        {
            broadcast_interval = RC_INT_C;
        }
        stored_config_temp[NV_SD_BT_INTERVAL] = broadcast_interval;

        // the button always works for singletouch mode
        // sync always works for singletouch mode
        if (stored_config_temp[NV_SD_TRIAL_CONFIG1] & SDH_SINGLETOUCH)
        {
            stored_config_temp[NV_SD_TRIAL_CONFIG0] |= SDH_USER_BUTTON_ENABLE;
            stored_config_temp[NV_SD_TRIAL_CONFIG0] |= SDH_TIME_SYNC;
        }
        stored_config_temp[NV_SD_TRIAL_CONFIG1] &= ~SDH_SINGLETOUCH;
        stored_config_temp[NV_SD_TRIAL_CONFIG0] &= ~SDH_TIME_SYNC;
        //        stored_config_temp[NV_SD_TRIAL_CONFIG1] = 0;

        memcpy(storedConfig, stored_config_temp, NV_NUM_SETTINGS_BYTES);
        memcpy((uint8_t*) (storedConfig + NV_DERIVED_CHANNELS_3),
               (uint8_t*) (stored_config_temp + NV_DERIVED_CHANNELS_3), 5);
        memcpy((uint8_t*) (storedConfig + NV_SENSORS3),
               (uint8_t*) (stored_config_temp + NV_SENSORS3), 5);
        memcpy((uint8_t*) (storedConfig + NV_SD_SHIMMER_NAME),
               (uint8_t*) (stored_config_temp + NV_SD_SHIMMER_NAME), 37);

        Config2SdHead();
        SetName();

        InfoMem_write((uint8_t*) 0, storedConfig, NV_NUM_SETTINGS_BYTES);
        InfoMem_write((uint8_t*) NV_SD_SHIMMER_NAME,
                      storedConfig + NV_SD_SHIMMER_NAME, 37);
    }

}

void Config2SdHead(void)
{
    memset(sdHeadText, 0xff, SDHEAD_LEN);

    sdHeadText[SDH_SAMPLE_RATE_0] = storedConfig[NV_SAMPLING_RATE];
    sdHeadText[SDH_SAMPLE_RATE_1] = storedConfig[NV_SAMPLING_RATE + 1];
    sdHeadText[SDH_BUFFER_SIZE] = storedConfig[NV_BUFFER_SIZE];
    sdHeadText[SDH_SENSORS0] = storedConfig[NV_SENSORS0];
    sdHeadText[SDH_SENSORS1] = storedConfig[NV_SENSORS1];
    sdHeadText[SDH_SENSORS2] = storedConfig[NV_SENSORS2];
    sdHeadText[SDH_CONFIG_SETUP_BYTE0] = storedConfig[NV_CONFIG_SETUP_BYTE0];
    sdHeadText[SDH_CONFIG_SETUP_BYTE1] = storedConfig[NV_CONFIG_SETUP_BYTE1];
    sdHeadText[SDH_CONFIG_SETUP_BYTE2] = storedConfig[NV_CONFIG_SETUP_BYTE2];
    sdHeadText[SDH_CONFIG_SETUP_BYTE3] = storedConfig[NV_CONFIG_SETUP_BYTE3];

    /* DMP related - start */
    sdHeadText[SDH_SENSORS3] = storedConfig[NV_SENSORS3];
    sdHeadText[SDH_SENSORS4] = storedConfig[NV_SENSORS4];
    sdHeadText[SDH_CONFIG_SETUP_BYTE4] = storedConfig[NV_CONFIG_SETUP_BYTE4];
    sdHeadText[SDH_CONFIG_SETUP_BYTE5] = storedConfig[NV_CONFIG_SETUP_BYTE5];
    sdHeadText[SDH_CONFIG_SETUP_BYTE6] = storedConfig[NV_CONFIG_SETUP_BYTE6];
    /* DMP related - end */

    /* little endian in fw, but they want big endian in sw
     * trivial */
    sdHeadText[SDH_SHIMMERVERSION_BYTE_0] = DEVICE_VER >> 8;
    sdHeadText[SDH_SHIMMERVERSION_BYTE_1] = DEVICE_VER & 0xff;
    sdHeadText[SDH_FW_VERSION_TYPE_0] = FW_IDENTIFIER >> 8;
    sdHeadText[SDH_FW_VERSION_TYPE_1] = FW_IDENTIFIER & 0xff;
    sdHeadText[SDH_FW_VERSION_MAJOR_0] = FW_VER_MAJOR >> 8;
    sdHeadText[SDH_FW_VERSION_MAJOR_1] = FW_VER_MAJOR & 0xff;
    sdHeadText[SDH_FW_VERSION_MINOR] = FW_VER_MINOR;
    sdHeadText[SDH_FW_VERSION_INTERNAL] = FW_VER_REL
            + ((FACTORY_TEST) ? 200 : 0);

    /* exg */
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

    sdHeadText[SDH_BT_COMMS_BAUD_RATE] = storedConfig[NV_BT_COMMS_BAUD_RATE];
    sdHeadText[SDH_DERIVED_CHANNELS_0] = storedConfig[NV_DERIVED_CHANNELS_0];
    sdHeadText[SDH_DERIVED_CHANNELS_1] = storedConfig[NV_DERIVED_CHANNELS_1];
    sdHeadText[SDH_DERIVED_CHANNELS_2] = storedConfig[NV_DERIVED_CHANNELS_2];
    sdHeadText[SDH_DERIVED_CHANNELS_3] = storedConfig[NV_DERIVED_CHANNELS_3];
    sdHeadText[SDH_DERIVED_CHANNELS_4] = storedConfig[NV_DERIVED_CHANNELS_4];
    sdHeadText[SDH_DERIVED_CHANNELS_5] = storedConfig[NV_DERIVED_CHANNELS_5];
    sdHeadText[SDH_DERIVED_CHANNELS_6] = storedConfig[NV_DERIVED_CHANNELS_6];
    sdHeadText[SDH_DERIVED_CHANNELS_7] = storedConfig[NV_DERIVED_CHANNELS_7];

    /* sd config */
    sdHeadText[SDH_MYTRIAL_ID] = storedConfig[NV_SD_MYTRIAL_ID];
    sdHeadText[SDH_NSHIMMER] = storedConfig[NV_SD_NSHIMMER];
    sdHeadText[SDH_EST_EXP_LEN_MSB] = storedConfig[NV_EST_EXP_LEN_MSB];
    sdHeadText[SDH_EST_EXP_LEN_LSB] = storedConfig[NV_EST_EXP_LEN_LSB];
    sdHeadText[SDH_MAX_EXP_LEN_MSB] = storedConfig[NV_MAX_EXP_LEN_MSB];
    sdHeadText[SDH_MAX_EXP_LEN_LSB] = storedConfig[NV_MAX_EXP_LEN_LSB];
    sdHeadText[SDH_TRIAL_CONFIG0] = storedConfig[NV_SD_TRIAL_CONFIG0];
    sdHeadText[SDH_TRIAL_CONFIG1] = storedConfig[NV_SD_TRIAL_CONFIG1];
    sdHeadText[SDH_BROADCAST_INTERVAL] = storedConfig[NV_SD_BT_INTERVAL];

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
    memcpy(&sdHeadText[SDH_TEMP_PRES_CALIBRATION], bmpX80Calib,
    BMP180_CALIB_DATA_SIZE);
    if (bmpInUse == BMP280_IN_USE)
    {
        memcpy(&sdHeadText[BMP280_XTRA_CALIB_BYTES],
               &(bmpX80Calib[BMP180_CALIB_DATA_SIZE]),
               BMP280_CALIB_XTRA_BYTES);
    }

    /* memcpy(&sdHeadText[SDH_MPU9150_GYRO_CALIBRATION], &storedConfig[NV_MPU9150_GYRO_CALIBRATION], 21);
     * memcpy(&sdHeadText[SDH_LSM303DLHC_MAG_CALIBRATION], &storedConfig[NV_LSM303DLHC_MAG_CALIBRATION], 21);
     * memcpy(&sdHeadText[SDH_LSM303DLHC_ACCEL_CALIBRATION], &storedConfig[NV_LSM303DLHC_ACCEL_CALIBRATION], 21);
     * memcpy(&sdHeadText[SDH_A_ACCEL_CALIBRATION], &storedConfig[NV_A_ACCEL_CALIBRATION], 21);
     */
    ShimmerCalibSyncFromDumpRamAll();
    memcpy(&sdHeadText[SDH_DAUGHTER_CARD_ID_BYTE0], daughtCardId, 3);
}

void SetDefaultConfiguration(void)
{
    /* 51.2Hz */
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
    storedConfig[NV_CONFIG_SETUP_BYTE3] = (ACCEL_2G << 6) + (BMPX80_OSS_1 << 4)
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
    if (storedConfig[NV_BT_COMMS_BAUD_RATE] == 0xFF)
    {
        storedConfig[NV_BT_COMMS_BAUD_RATE] = 9;
    }

    /* Write RAM contents to Infomem */
    InfoMem_write((void*) 0, storedConfig, NV_NUM_SETTINGS_BYTES);

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
    SDH_USER_BUTTON_ENABLE + SDH_RWCERROR_EN + SDH_SDERROR_EN;
    storedConfig[NV_SD_TRIAL_CONFIG1] = 0;

    storedConfig[NV_SD_BT_INTERVAL] = 54;

    InfoMem_write((uint8_t*) NV_SD_SHIMMER_NAME,
                  &storedConfig[NV_SD_SHIMMER_NAME],
                  NV_NUM_SD_BYTES);
}

void SetShimmerName()
{
    uint8_t i;
    for (i = 0;
            (i < MAX_CHARS - 1) && isprint(storedConfig[NV_SD_SHIMMER_NAME + i]);
            i++)
        ;
    if (i)
    {
        memcpy((char*) shimmerName, (char*) (storedConfig + NV_SD_SHIMMER_NAME),
               i);
        shimmerName[i] = 0;
    }
    else
    {
        strcpy((char*) (storedConfig + NV_SD_SHIMMER_NAME), "Shimmer_XXXX");
        memcpy((storedConfig + NV_SD_SHIMMER_NAME) + 8, mac + 8, 4);
        strcpy((char*) shimmerName,
               (char*) (storedConfig + NV_SD_SHIMMER_NAME));
    }
}
void SetExpIdName()
{
    uint8_t i;   //, len_temp;
    for (i = 0;
            (i < MAX_CHARS - 1) && isprint(storedConfig[NV_SD_EXP_ID_NAME + i]);
            i++)
        ;
    if (i)
    {
        memcpy((char*) expIdName, (char*) (storedConfig + NV_SD_EXP_ID_NAME),
               i);
        expIdName[i] = 0;
    }
    else
    {
        strcpy((char*) (storedConfig + NV_SD_EXP_ID_NAME), "default_exp");
        strcpy((char*) expIdName, (char*) (storedConfig + NV_SD_EXP_ID_NAME));
    }
}
void SetCfgTime()
{
    uint32_t cfg_time_temp = 0;
    uint8_t i;
    for (i = 0; i < 4; i++)
    {
        cfg_time_temp <<= 8;
        cfg_time_temp += storedConfig[NV_SD_CONFIG_TIME + i];
    }
    if (cfg_time_temp)
    {
        ItoaNo0((uint64_t) cfg_time_temp, configTimeText, UINT32_LEN);
    }
    else
    {
        strcpy((char*) configTimeText, "0");
    }
}

void Infomem2Names()
{
    SetShimmerName();
    SetExpIdName();
    SetCfgTime();
}

void SetName()
{
    if (strlen((char*) configTimeText) == 0)
    {
        strcpy((char*) configTimeText, "0");
    }

    if (strlen((char*) fileName) == 0)
        strcpy((char*) fileName, "no_file   ");
}

error_t SetBasedir()
{
    FILINFO fno;
    volatile error_t res;
    uint16_t tmp_counter = 0;
    char lfn[_MAX_LFN + 1], *fname, *scout, *dash, dirnum[8];

    Infomem2Names();
//SetName();

    fno.lfname = lfn;
    fno.lfsize = sizeof(lfn);

    if ((res = f_opendir(&dir, "/data")))
    {
        if (res == FR_NO_PATH)   // we'll have to make /data first
            res = f_mkdir("/data");
        if (res)  // in every case, we're toast
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
        if (res)         // in every case, we're toast
            return FAIL;

        // try one more time
        if ((res = f_opendir(&dir, (char*) expDirName)))
            return FAIL;
    }

    dirCounter = 0;   // this might be the first log for this shimmer

// file name format
// shimmername    as defined in sdlog.cfg
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

    char dir_counter_text[4];
    ItoaWith0((uint64_t) dirCounter, (uint8_t*) dir_counter_text, 4);

    strcpy((char*) dirName, (char*) expDirName);
    strcat((char*) dirName, "/");
    strcat((char*) dirName, (char*) shimmerName);
    strcat((char*) dirName, "-");
    strcat((char*) dirName, dir_counter_text);

    if (ff_result = f_mkdir((char*) dirName))
    {
        FindError(ff_result, dirName);
        return FAIL;
    }

    memset(fileName, 0, 64);
    strcpy((char*) fileName, (char*) dirName);
    dirLen = strlen((char*) dirName);
    strcat((char*) fileName, "/000");

    return SUCCESS;
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
        sc1.range = (info_config & 0xe0) >> 5;
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
    for (byte_cnt = 21; byte_cnt > 0; byte_cnt--)
    {
        if (storedConfig[offset + byte_cnt] != 0xff)
        {
            info_valid = 1;
            break;
        }
    }

    if (info_valid)
    {
        /* if not all 0xff in infomem */
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
        if(lsmInUse == LSM303DLHC_IN_USE){
            sc1.range = (storedConfig[NV_CONFIG_SETUP_BYTE2] >> 5) & 0x07;
        } else {
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

void CalibFromFile()
{
    streamData = 0;                   // this will skip one sample
    if (storedConfig[NV_SENSORS1] & SENSOR_LSM303DLHC_ACCEL)
    {
        CalibFromFileRead(S_ACCEL_D);
        memcpy(&sdHeadText[SDH_LSM303DLHC_ACCEL_CALIBRATION],
               &storedConfig[NV_LSM303DLHC_ACCEL_CALIBRATION], 21);
        InfoMem_write((uint8_t*) NV_LSM303DLHC_ACCEL_CALIBRATION,
                      &storedConfig[NV_LSM303DLHC_ACCEL_CALIBRATION], 21);
    }
    if (storedConfig[NV_SENSORS0] & SENSOR_MPU9150_GYRO)
    {
        CalibFromFileRead(S_GYRO);
        memcpy(&sdHeadText[SDH_MPU9150_GYRO_CALIBRATION],
               &storedConfig[NV_MPU9150_GYRO_CALIBRATION], 21);
        InfoMem_write((uint8_t*) NV_MPU9150_GYRO_CALIBRATION,
                      &storedConfig[NV_MPU9150_GYRO_CALIBRATION], 21);
    }
    if (storedConfig[NV_SENSORS0] & SENSOR_LSM303DLHC_MAG)
    {
        CalibFromFileRead(S_MAG);
        memcpy(&sdHeadText[SDH_LSM303DLHC_MAG_CALIBRATION],
               &storedConfig[NV_LSM303DLHC_MAG_CALIBRATION], 21);
        InfoMem_write((uint8_t*) NV_LSM303DLHC_MAG_CALIBRATION,
                      &storedConfig[NV_LSM303DLHC_MAG_CALIBRATION], 21);
    }
    if (storedConfig[NV_SENSORS0] & SENSOR_A_ACCEL)
    {
        CalibFromFileRead(S_ACCEL_A);
        memcpy(&sdHeadText[SDH_A_ACCEL_CALIBRATION],
               &storedConfig[NV_A_ACCEL_CALIBRATION], 21);
        InfoMem_write((uint8_t*) NV_A_ACCEL_CALIBRATION,
                      &storedConfig[NV_A_ACCEL_CALIBRATION], 21);
    }
    if (storedConfig[NV_SENSORS2] & SENSOR_BMPX80_PRESSURE)
    {
        //      BMP180_init();
        //      BMP180_getCalibCoeff(&sdHeadText[SDH_TEMP_PRES_CALIBRATION]);
        //      P8OUT &= ~BIT4;         //set SW_I2C low to power off I2C chips
        memcpy(&sdHeadText[SDH_TEMP_PRES_CALIBRATION], bmpX80Calib,
        BMP180_CALIB_DATA_SIZE);
        if (bmpInUse == BMP280_IN_USE)
        {
            memcpy(&sdHeadText[BMP280_XTRA_CALIB_BYTES],
                   &(bmpX80Calib[BMP180_CALIB_DATA_SIZE]),
                   BMP280_CALIB_XTRA_BYTES);
        }
    }
}

void CalibFromFileRead(uint8_t sensor)
{
    char buffer[66], *equals, keyword[20], cal_file[48], cal_indiv_file[32],
            cal_newformat_fullname[48];
    uint8_t i = 0, num_2byte_params = 6, num_1byte_params = 9;
    uint16_t address;
    bool sensor_found = FALSE;
    float value;
    int8_t rounded_value, oldformat_calib_file = 0;
    volatile error_t res;

    uint8_t mpu9150_gyro_range = (sdHeadText[SDH_CONFIG_SETUP_BYTE2] & 0x03);
    uint8_t lsm303dlhc_mag_range = ((sdHeadText[SDH_CONFIG_SETUP_BYTE2] >> 5)
            & 0x07);
    uint8_t lsm303_accel_range = ((sdHeadText[SDH_CONFIG_SETUP_BYTE0] >> 2)
            & 0x03);

    DIRS gdc;
    FIL gfc;

    if (sensor == S_GYRO)
    {
        strcpy(cal_indiv_file, "/calib_gyro_");          //calib_gyro_250dps.ini
        if (mpu9150_gyro_range == MPU9150_GYRO_250DPS)
        {
            strcpy(keyword, "Gyro 250dps");
            strcat(cal_indiv_file, "250");
        }
        else if (mpu9150_gyro_range == MPU9150_GYRO_500DPS)
        {
            strcpy(keyword, "Gyro 500dps");
            strcat(cal_indiv_file, "500");
        }
        else if (mpu9150_gyro_range == MPU9150_GYRO_1000DPS)
        {
            strcpy(keyword, "Gyro 1000dps");
            strcat(cal_indiv_file, "1000");
        }
        else if (mpu9150_gyro_range == MPU9150_GYRO_2000DPS)
        {
            strcpy(keyword, "Gyro 2000dps");
            strcat(cal_indiv_file, "2000");
        }
        strcat(cal_indiv_file, "dps.ini");
        address = NV_MPU9150_GYRO_CALIBRATION;
    }
    else if (sensor == S_MAG)
    {
        strcpy(cal_indiv_file, "/calib_mag_");              //calib_mag_13ga.ini
        if (lsm303dlhc_mag_range == LSM303_MAG_13GA)
        {
            strcpy(keyword, "Mag 1.3Ga");
            strcat(cal_indiv_file, "13");
        }
        else if (lsm303dlhc_mag_range == LSM303_MAG_19GA)
        {
            strcpy(keyword, "Mag 1.9Ga");
            strcat(cal_indiv_file, "19");
        }
        else if (lsm303dlhc_mag_range == LSM303_MAG_25GA)
        {
            strcpy(keyword, "Mag 2.5Ga");
            strcat(cal_indiv_file, "25");
        }
        else if (lsm303dlhc_mag_range == LSM303_MAG_40GA)
        {
            strcpy(keyword, "Mag 4.0Ga");
            strcat(cal_indiv_file, "40");
        }
        else if (lsm303dlhc_mag_range == LSM303_MAG_47GA)
        {
            strcpy(keyword, "Mag 4.7Ga");
            strcat(cal_indiv_file, "47");
        }
        else if (lsm303dlhc_mag_range == LSM303_MAG_56GA)
        {
            strcpy(keyword, "Mag 5.6Ga");
            strcat(cal_indiv_file, "56");
        }
        else if (lsm303dlhc_mag_range == LSM303_MAG_81GA)
        {
            strcpy(keyword, "Mag 8.1Ga");
            strcat(cal_indiv_file, "81");
        }
        strcat(cal_indiv_file, "ga.ini");
        address = NV_LSM303DLHC_MAG_CALIBRATION;
    }
    else if (sensor == S_ACCEL_D)
    {
        strcpy(cal_indiv_file, "/calib_accel_wr_");       //calib_accel_d_2g.ini
        if (lsm303_accel_range == RANGE_2G)
        {
            strcpy(keyword, "Accel 2.0g");
            strcat(cal_indiv_file, "2");
        }
        else if (lsm303_accel_range == RANGE_4G)
        {
            strcpy(keyword, "Accel 4.0g");
            strcat(cal_indiv_file, "4");
        }
        else if (lsm303_accel_range == RANGE_8G)
        {
            strcpy(keyword, "Accel 8.0g");
            strcat(cal_indiv_file, "8");
        }
        else
        { //(sdHeadText[SDH_ACCEL_RANGE] == RANGE_16G)
            strcpy(keyword, "Accel 16.0g");
            strcat(cal_indiv_file, "16");
        }
        strcat(cal_indiv_file, "g.ini");
        address = NV_LSM303DLHC_ACCEL_CALIBRATION;
    }
    else if (sensor == S_ACCEL_A)
    {
        strcpy(keyword, "Accel_A");
        strcpy(cal_indiv_file, "/calib_accel_ln_2g.ini");
        address = NV_A_ACCEL_CALIBRATION;
    }
    else
    {
        return;
    }

    strcpy(cal_file, "/Calibration"); // "/Calibration/calibParams.ini"
    if (f_opendir(&gdc, "/Calibration"))
    {
        if (f_opendir(&gdc, "/calibration"))
        {
            CalibDefault(sensor);
            return;
        }
        else
            strcpy(cal_file, "/calibration");
    }
    strcpy(cal_newformat_fullname, cal_file);
    strcat(cal_file, "/calibParams.ini");
    strcat(cal_newformat_fullname, cal_indiv_file);

    res = f_open(&gfc, cal_newformat_fullname, (FA_OPEN_EXISTING | FA_READ));
    if (res == FR_NO_FILE)
    {
        if (f_open(&gfc, cal_file, (FA_OPEN_EXISTING | FA_READ)))
        { // no calibration file, use default
            CalibDefault(sensor);
            return;
        }
        oldformat_calib_file = 1;
    }

// look for sensor in calibration file.
    if (oldformat_calib_file)
    {
        while (f_gets(buffer, 64, &gfc))
        {
            if (!strstr(buffer, keyword))
                continue;
            else
            { // found the right sensor
                sensor_found = TRUE;
                break;
            }
        }
    }
    else
        sensor_found = TRUE;

    if (sensor_found)
    {
        for (i = 0; i < num_2byte_params; i++)
        {
            f_gets(buffer, 64, &gfc);
            if (!(equals = strchr(buffer, '=')))
            {
                sensor_found = FALSE; // there's an error, use the default
                break;
            }
            equals++;
            value = atof(equals);
            if ((sensor == S_GYRO) & (i >= 3))
                value *= 100;
            storedConfig[address + 2 * i] = ((int16_t) value & 0xFF00) >> 8;
            storedConfig[address + 2 * i + 1] = ((int16_t) value & 0xFF);
        }
        for (i = 0; i < num_1byte_params; i++)
        {
            f_gets(buffer, 64, &gfc);
            if (!(equals = strchr(buffer, '=')))
            {
                sensor_found = FALSE; // there's an error, use the default
                break;
            }
            equals++;
            value = atof(equals) * 100;
            rounded_value = (int8_t) value;
            storedConfig[address + 2 * num_2byte_params + i] = (rounded_value);
        }
    }

    f_close(&gfc);

    if (!sensor_found)
        CalibDefault(sensor);
}

uint8_t CalibCheckInfoValid(uint8_t sensor)
{
    uint16_t address;
    uint8_t ram_range, info_range = 0xff, info_config;
    int byte_cnt = 0;

    if (sensor == S_ACCEL_A)
    {
        address = NV_A_ACCEL_CALIBRATION;
    }
    else if (sensor == S_GYRO)
    {
        address = NV_MPU9150_GYRO_CALIBRATION;
        InfoMem_read((uint8_t *) NV_CONFIG_SETUP_BYTE2, &info_config, 1);
        info_range = info_config & 0x03;
        ram_range = storedConfig[NV_CONFIG_SETUP_BYTE2] & 0x03;
        // must have info_range == ram_range or there is a mismatch between RAM and infomem
        if (info_range != ram_range)
            return 0;
    }
    else if (sensor == S_MAG)
    {
        address = NV_LSM303DLHC_MAG_CALIBRATION;
        InfoMem_read((uint8_t *) NV_CONFIG_SETUP_BYTE2, &info_config, 1);
        info_range = (info_config & 0xe0) >> 5;
        ram_range = (storedConfig[NV_CONFIG_SETUP_BYTE2] & 0xe0) >> 5;
        if (info_range != ram_range)
            return 0;
    }
    else if (sensor == S_ACCEL_D)
    {
        address = NV_LSM303DLHC_ACCEL_CALIBRATION;
        InfoMem_read((uint8_t *) NV_CONFIG_SETUP_BYTE0, &info_config, 1);
        info_range = (info_config & 0x0c) >> 2;
        ram_range = (storedConfig[NV_CONFIG_SETUP_BYTE0] & 0x0c) >> 2;
        if (info_range != ram_range)
            return 0;
    }

    InfoMem_read((uint8_t *) address, storedConfig + address, 21);
    byte_cnt = 0;
    while (byte_cnt < 21)
    {
        if (storedConfig[address + byte_cnt] != 0xff)
        {
            return 1;
        }
        byte_cnt++;
    }
    return 0;
}

void CalibAll()
{
    uint8_t cal_aaccel_done = 0, cal_gyro_done = 0, cal_mag_done = 0,
            cal_daccel_done = 0;
//uint8_t ram_range, info_range = 0xff, info_config;//, info_valid = 0
//int byte_cnt = 0;

    uint8_t mpu9150_gyro_range = (sdHeadText[SDH_CONFIG_SETUP_BYTE2] & 0x03);
    uint8_t lsm303dlhc_mag_range = ((sdHeadText[SDH_CONFIG_SETUP_BYTE2] >> 5)
            & 0x07);
    uint8_t lsm303_accel_range = ((sdHeadText[SDH_CONFIG_SETUP_BYTE0] >> 2)
            & 0x03);

    if (GetCalibFlag())
    { // info > sdcard
        if (CalibCheckInfoValid(S_ACCEL_A))
        {
            CalibNewFile(S_ACCEL_A, 0);
            cal_aaccel_done = 1;
        }
        if (CalibCheckInfoValid(S_GYRO))
        {
            CalibNewFile(S_GYRO, mpu9150_gyro_range);
            cal_aaccel_done = 1;
        }
        if (CalibCheckInfoValid(S_MAG))
        {
            CalibNewFile(S_MAG, lsm303dlhc_mag_range);
            cal_aaccel_done = 1;
        }
        if (CalibCheckInfoValid(S_ACCEL_D))
        {
            CalibNewFile(S_ACCEL_D, lsm303_accel_range);
            cal_aaccel_done = 1;
        }
        SetCalibFlag(0);
    }

    if (!cal_aaccel_done)
    {
        CalibFromFileRead(S_ACCEL_A);
        InfoMem_write((uint8_t*) NV_A_ACCEL_CALIBRATION,
                      &storedConfig[NV_A_ACCEL_CALIBRATION], 21);
    }
    memcpy(&sdHeadText[SDH_A_ACCEL_CALIBRATION],
           &storedConfig[NV_A_ACCEL_CALIBRATION], 21);
    if (!cal_gyro_done)
    {
        CalibFromFileRead(S_GYRO);
        InfoMem_write((uint8_t*) NV_MPU9150_GYRO_CALIBRATION,
                      &storedConfig[NV_MPU9150_GYRO_CALIBRATION], 21);
    }
    memcpy(&sdHeadText[SDH_MPU9150_GYRO_CALIBRATION],
           &storedConfig[NV_MPU9150_GYRO_CALIBRATION], 21);
    if (!cal_mag_done)
    {
        CalibFromFileRead(S_MAG);
        InfoMem_write((uint8_t*) NV_LSM303DLHC_MAG_CALIBRATION,
                      &storedConfig[NV_LSM303DLHC_MAG_CALIBRATION], 21);
    }
    memcpy(&sdHeadText[SDH_LSM303DLHC_MAG_CALIBRATION],
           &storedConfig[NV_LSM303DLHC_MAG_CALIBRATION], 21);
    if (!cal_daccel_done)
    {
        CalibFromFileRead(S_ACCEL_D);
        InfoMem_write((uint8_t*) NV_LSM303DLHC_ACCEL_CALIBRATION,
                      &storedConfig[NV_LSM303DLHC_ACCEL_CALIBRATION], 21);
    }
    memcpy(&sdHeadText[SDH_LSM303DLHC_ACCEL_CALIBRATION],
           &storedConfig[NV_LSM303DLHC_ACCEL_CALIBRATION], 21);

    if (storedConfig[NV_SENSORS2] & SENSOR_BMPX80_PRESSURE)
    {
        //      BMP180_init();
        //      BMP180_getCalibCoeff(&sdHeadText[SDH_TEMP_PRES_CALIBRATION]);
        //      P8OUT &= ~BIT4;         //set SW_I2C low to power off I2C chips
        memcpy(&sdHeadText[SDH_TEMP_PRES_CALIBRATION], bmpX80Calib,
        BMP180_CALIB_DATA_SIZE);
        if (bmpInUse == BMP280_IN_USE)
        {
            memcpy(&sdHeadText[BMP280_XTRA_CALIB_BYTES],
                   &(bmpX80Calib[BMP180_CALIB_DATA_SIZE]),
                   BMP280_CALIB_XTRA_BYTES);
        }
    }
}
void CalibFromInfoAll()
{
    CalibFromInfo(S_ACCEL_D);
    CalibFromInfo(S_GYRO);
    CalibFromInfo(S_MAG);
    CalibFromInfo(S_ACCEL_A);

    memcpy(&sdHeadText[SDH_A_ACCEL_CALIBRATION],
           &storedConfig[NV_A_ACCEL_CALIBRATION], 21);
    memcpy(&sdHeadText[SDH_MPU9150_GYRO_CALIBRATION],
           &storedConfig[NV_MPU9150_GYRO_CALIBRATION], 21);
    memcpy(&sdHeadText[SDH_LSM303DLHC_MAG_CALIBRATION],
           &storedConfig[NV_LSM303DLHC_MAG_CALIBRATION], 21);
    memcpy(&sdHeadText[SDH_LSM303DLHC_ACCEL_CALIBRATION],
           &storedConfig[NV_LSM303DLHC_ACCEL_CALIBRATION], 21);

    InfoMem_write((uint8_t*) NV_A_ACCEL_CALIBRATION,
                  &storedConfig[NV_A_ACCEL_CALIBRATION], 21);
    InfoMem_write((uint8_t*) NV_MPU9150_GYRO_CALIBRATION,
                  &storedConfig[NV_MPU9150_GYRO_CALIBRATION], 21);
    InfoMem_write((uint8_t*) NV_LSM303DLHC_MAG_CALIBRATION,
                  &storedConfig[NV_LSM303DLHC_MAG_CALIBRATION], 21);
    InfoMem_write((uint8_t*) NV_LSM303DLHC_ACCEL_CALIBRATION,
                  &storedConfig[NV_LSM303DLHC_ACCEL_CALIBRATION], 21);

    if (storedConfig[NV_SENSORS2] & SENSOR_BMPX80_PRESSURE)
    {
        //      BMP180_init();
        //      BMP180_getCalibCoeff(&sdHeadText[SDH_TEMP_PRES_CALIBRATION]);
        //      P8OUT &= ~BIT4;         //set SW_I2C low to power off I2C chips
        memcpy(&sdHeadText[SDH_TEMP_PRES_CALIBRATION], bmpX80Calib,
        BMP180_CALIB_DATA_SIZE);
        if (bmpInUse == BMP280_IN_USE)
        {
            memcpy(&sdHeadText[BMP280_XTRA_CALIB_BYTES],
                   &(bmpX80Calib[BMP180_CALIB_DATA_SIZE]),
                   BMP280_CALIB_XTRA_BYTES);
        }
    }
}

void CalibFromInfo(uint8_t sensor)
{
    uint8_t ram_range, info_range = 0xff, info_config, info_valid = 0;
    int byte_cnt = 0;

    if (sensor == S_ACCEL_A)
    {
        InfoMem_read((uint8_t *) NV_A_ACCEL_CALIBRATION,
                     storedConfig + NV_A_ACCEL_CALIBRATION, 21);
        byte_cnt = 0;
        while (byte_cnt < 21)
        {
            if (storedConfig[NV_A_ACCEL_CALIBRATION + byte_cnt] != 0xff)
            {
                info_valid = 1;
                break;
            }
            byte_cnt++;
        }
    }
    else if (sensor == S_GYRO)
    {
        InfoMem_read((uint8_t *) NV_CONFIG_SETUP_BYTE2, &info_config, 1);
        info_range = info_config & 0x03;
        ram_range = storedConfig[NV_CONFIG_SETUP_BYTE2] & 0x03;
        // must have info_range == ram_range or there is a mismatch between RAM and infomem
        if (info_range == ram_range)
        {
            InfoMem_read((uint8_t *) NV_MPU9150_GYRO_CALIBRATION,
                         storedConfig + NV_MPU9150_GYRO_CALIBRATION, 21);
            byte_cnt = 0;
            while (byte_cnt < 21)
            {
                if (storedConfig[NV_MPU9150_GYRO_CALIBRATION + byte_cnt]
                        != 0xff)
                {
                    info_valid = 1;
                    break;
                }
                byte_cnt++;
            }
        }
    }
    else if (sensor == S_MAG)
    {
        InfoMem_read((uint8_t *) NV_CONFIG_SETUP_BYTE2, &info_config, 1);
        info_range = (info_config & 0xe0) >> 5;
        ram_range = (storedConfig[NV_CONFIG_SETUP_BYTE2] & 0xe0) >> 5;
        if (info_range == ram_range)
        {
            InfoMem_read((uint8_t *) NV_LSM303DLHC_MAG_CALIBRATION,
                         storedConfig + NV_LSM303DLHC_MAG_CALIBRATION, 21);
            byte_cnt = 0;
            while (byte_cnt < 21)
            {
                if (storedConfig[NV_LSM303DLHC_MAG_CALIBRATION + byte_cnt]
                        != 0xff)
                {
                    info_valid = 1;
                    break;
                }
                byte_cnt++;
            }
        }
    }
    else if (sensor == S_ACCEL_D)
    {
        InfoMem_read((uint8_t *) NV_CONFIG_SETUP_BYTE0, &info_config, 1);
        info_range = (info_config & 0x0c) >> 2;
        ram_range = (storedConfig[NV_CONFIG_SETUP_BYTE0] & 0x0c) >> 2;
        if (info_range == ram_range)
        {
            InfoMem_read((uint8_t *) NV_LSM303DLHC_ACCEL_CALIBRATION,
                         storedConfig + NV_LSM303DLHC_ACCEL_CALIBRATION, 21);
            byte_cnt = 0;
            while (byte_cnt < 21)
            {
                if (storedConfig[NV_LSM303DLHC_ACCEL_CALIBRATION + byte_cnt]
                        != 0xff)
                {
                    info_valid = 1;
                    break;
                }
                byte_cnt++;
            }
        }
    }
    else
        return;

    if (!info_valid) // if range not match, or all 0xff in infomem
        CalibDefault(sensor);

    if (!docked && CheckSdInslot() && !SD_ERROR)
        CalibNewFile(sensor, info_range);
}


void CalibAllAsDefault(void)
{
//    sc_t sc1;

    CalibDefault(S_ACCEL_A);
    CalibDefault(S_ACCEL_D);
    CalibDefault(S_GYRO);
    CalibDefault(S_MAG);

//    memcpy(&sc1.data.dd, &storedConfig[NV_A_ACCEL_CALIBRATION],
//    SC_DATA_LEN_ANALOG_ACCEL);
//    ShimmerCalib_singleSensorWrite(&sc1);
//    ShimmerCalib_singleSensorRead(&sc1);
//
//    memcpy(&sc1.data.dd, &storedConfig[NV_LSM303DLHC_ACCEL_CALIBRATION],
//    SC_DATA_LEN_LSM303DLHC_ACCEL);
//    ShimmerCalib_singleSensorWrite(&sc1);
//
//    memcpy(&sc1.data.dd, &storedConfig[NV_MPU9150_GYRO_CALIBRATION],
//    SC_DATA_LEN_MPU9250_GYRO);
//    ShimmerCalib_singleSensorWrite(&sc1);
//
//    memcpy(&sc1.data.dd, &storedConfig[NV_LSM303DLHC_MAG_CALIBRATION],
//    SC_DATA_LEN_LSM303DLHC_MAG);
//    ShimmerCalib_singleSensorWrite(&sc1);
}

void CalibDefault(uint8_t sensor)
{
    int16_t bias, sensitivity, sensitivity_x, sensitivity_y, sensitivity_z;
    uint8_t bias_byte_one, bias_byte_two, sens_byte_one, sens_byte_two,
            number_axes = 1;
    int8_t align_xx, align_xy, align_xz, align_yx, align_yy, align_yz, align_zx,
            align_zy, align_zz, i = 0;
    uint16_t address;
    bool align = FALSE;

    uint8_t mpu9150_gyro_range = storedConfig[NV_CONFIG_SETUP_BYTE2] & 0x03;
    uint8_t lsm303dlhc_mag_range = (storedConfig[NV_CONFIG_SETUP_BYTE2] >> 5)
            & 0x07;
    uint8_t lsm303_accel_range = (storedConfig[NV_CONFIG_SETUP_BYTE0] >> 2)
            & 0x03;

    if (sensor == S_ACCEL_D)
    {
        number_axes = 3;
        bias = 0;
        align = TRUE;
        address = NV_LSM303DLHC_ACCEL_CALIBRATION;
        if (lsm303_accel_range == RANGE_2G)
        {
            sensitivity = (lsmInUse == LSM303DLHC_IN_USE) ? 1631 : 1671;
        }
        else if (lsm303_accel_range == RANGE_4G)
        {
            sensitivity = (lsmInUse == LSM303DLHC_IN_USE) ? 815 : 836;
        }
        else if (lsm303_accel_range == RANGE_8G)
        {
            sensitivity = (lsmInUse == LSM303DLHC_IN_USE) ? 408 : 418;
        }
        else
        {
            sensitivity = (lsmInUse == LSM303DLHC_IN_USE) ? 135 : 209;
        }
        align_xx = (lsmInUse == LSM303DLHC_IN_USE) ? (-100) : 0;
        align_xy = (lsmInUse == LSM303DLHC_IN_USE) ? 0 : -100;
        align_xz = 0;

        align_yx = (lsmInUse == LSM303DLHC_IN_USE) ? 0 : 100;
        align_yy = (lsmInUse == LSM303DLHC_IN_USE) ? 100 : 0;
        align_yz = 0;

        align_zx = 0;
        align_zy = 0;
        align_zz = -100;
    }
    else if (sensor == S_GYRO)
    {
        number_axes = 3;
        bias = 0;
        if (mpu9150_gyro_range == MPU9150_GYRO_250DPS)
        {
            sensitivity = 13100;
        }
        else if (mpu9150_gyro_range == MPU9150_GYRO_500DPS)
        {
            sensitivity = 6550;
        }
        else if (mpu9150_gyro_range == MPU9150_GYRO_1000DPS)
        {
            sensitivity = 3280;
        }
        else
        {
            //(sdHeadText[SDH_GYRO_RANGE] == MPU9150_GYRO_2000DPS)
            sensitivity = 1640;
        }
        align = TRUE;
        align_xx = 0;
        align_xy = -100;
        align_xz = 0;
        align_yx = -100;
        align_yy = 0;
        align_yz = 0;
        align_zx = 0;
        align_zy = 0;
        align_zz = -100;
        address = NV_MPU9150_GYRO_CALIBRATION;
    }
    else if (sensor == S_MAG)
    {
        number_axes = 3;
        bias = 0;
        if (lsmInUse == LSM303DLHC_IN_USE)
        {
            if (lsm303dlhc_mag_range == LSM303_MAG_13GA)
            {
                sensitivity_x = 1100;
                sensitivity_y = 1100;
                sensitivity_z = 980;
            }
            else if (lsm303dlhc_mag_range == LSM303_MAG_19GA)
            {
                sensitivity_x = 855;
                sensitivity_y = 855;
                sensitivity_z = 760;
            }
            else if (lsm303dlhc_mag_range == LSM303_MAG_25GA)
            {
                sensitivity_x = 670;
                sensitivity_y = 670;
                sensitivity_z = 600;
            }
            else if (lsm303dlhc_mag_range == LSM303_MAG_40GA)
            {
                sensitivity_x = 450;
                sensitivity_y = 450;
                sensitivity_z = 400;
            }
            else if (lsm303dlhc_mag_range == LSM303_MAG_47GA)
            {
                sensitivity_x = 400;
                sensitivity_y = 400;
                sensitivity_z = 355;
            }
            else if (lsm303dlhc_mag_range == LSM303_MAG_56GA)
            {
                sensitivity_x = 330;
                sensitivity_y = 330;
                sensitivity_z = 295;
            }
            else
            { //(sdHeadText[SDH_MAG_RANGE] == LSM303_MAG_81GA)
                sensitivity_x = 230;
                sensitivity_y = 230;
                sensitivity_z = 205;
            }
        }
        else
        {
            sensitivity_x = 667;
            sensitivity_y = 667;
            sensitivity_z = 667;
        }

        align = TRUE;
        align_xx = (lsmInUse == LSM303DLHC_IN_USE) ? (-100) : 0;
        align_xy = (lsmInUse == LSM303DLHC_IN_USE) ? 0 : -100;
        align_xz = 0;

        align_yx = (lsmInUse == LSM303DLHC_IN_USE) ? 0 : 100;
        align_yy = (lsmInUse == LSM303DLHC_IN_USE) ? 100 : 0;
        align_yz = 0;

        align_zx = 0;
        align_zy = 0;
        align_zz = -100;

        address = NV_LSM303DLHC_MAG_CALIBRATION;
    }
    else if (sensor == S_ACCEL_A)
    {
        number_axes = 3;
        bias = 2047;
        sensitivity = 83;
        if ((lsmInUse == LSM303DLHC_IN_USE) && (bmpInUse == BMP280_IN_USE))
        {
            // Assuming here that new bmp and lsm infer new low-noise accel used
            bias = 2253;
            sensitivity = 92;
        }
        align = TRUE;
        align_xx = 0;
        align_xy = -100;
        align_xz = 0;
        align_yx = -100;
        align_yy = 0;
        align_yz = 0;
        align_zx = 0;
        align_zy = 0;
        align_zz = -100;
        address = NV_A_ACCEL_CALIBRATION;
    }
    else
    {
        return;
    }
    bias_byte_one = (uint8_t)(bias >> 8);
    bias_byte_two = (uint8_t)(bias);
    sens_byte_one = (uint8_t)(sensitivity >> 8);
    sens_byte_two = (uint8_t)(sensitivity);
    // offset
    for (i = 0; i < number_axes; i++)
    {
        storedConfig[address + 2 * i] = bias_byte_one;
        storedConfig[address + 2 * i + 1] = bias_byte_two;
    }
    // sensitivity
    if (sensor == S_MAG)
    {
        sens_byte_one = (uint8_t)(sensitivity_x >> 8);
        sens_byte_two = (uint8_t)(sensitivity_x);
        storedConfig[address + 2 * (number_axes)] = sens_byte_one;
        storedConfig[address + 2 * (number_axes) + 1] = sens_byte_two;
        sens_byte_one = (uint8_t)(sensitivity_y >> 8);
        sens_byte_two = (uint8_t)(sensitivity_y);
        storedConfig[address + 2 * (number_axes + 1)] = sens_byte_one;
        storedConfig[address + 2 * (number_axes + 1) + 1] = sens_byte_two;
        sens_byte_one = (uint8_t)(sensitivity_z >> 8);
        sens_byte_two = (uint8_t)(sensitivity_z);
        storedConfig[address + 2 * (number_axes + 2)] = sens_byte_one;
        storedConfig[address + 2 * (number_axes + 2) + 1] = sens_byte_two;
    }
    else
    {
        for (i = 0; i < number_axes; i++)
        {
            storedConfig[address + 2 * (number_axes + i)] = sens_byte_one;
            storedConfig[address + 2 * (number_axes + i) + 1] = sens_byte_two;
        }
    }
    // alignment
    if (align)
    {
        storedConfig[address + 12] = align_xx;
        storedConfig[address + 13] = align_xy;
        storedConfig[address + 14] = align_xz;
        storedConfig[address + 15] = align_yx;
        storedConfig[address + 16] = align_yy;
        storedConfig[address + 17] = align_yz;
        storedConfig[address + 18] = align_zx;
        storedConfig[address + 19] = align_zy;
        storedConfig[address + 20] = align_zz;
    }
}

void CalibNewFile(uint8_t sensor, uint8_t range)
{
    if (!docked && CheckSdInslot() && !SD_ERROR)
    {
        uint8_t sd_power_state;
        if (!(P4OUT & BIT2))
        {
            SdPowerOn();
            sd_power_state = 0;
        }
        else
            sd_power_state = 1;

        // set file name
        volatile error_t res;
        char buffer[66];
        char cal_file_name[66], sensor_str[9], range_str[9];
        uint16_t address;
        DIRS gdc;
        FIL gfc;
        UINT bw;
        strcpy((char*) cal_file_name, "/Calibration");
        if (res = f_opendir(&gdc, "/Calibration"))
        {
            if (res = f_opendir(&gdc, "/calibration"))
                if (res == FR_NO_PATH)   // we'll have to make /data first
                    res = f_mkdir("/Calibration");
                else
                    strcpy((char*) cal_file_name, "/calibration");
        }

        if (sensor == S_ACCEL_D)
        {
            strcat((char*) cal_file_name, "/calib_accel_wr_"); //calib_accel_d_2g.ini
            address = NV_LSM303DLHC_ACCEL_CALIBRATION;
            strcpy((char*) sensor_str, "Accel ");
            if (range == RANGE_2G)
            {
                strcat((char*) cal_file_name, "2");
                strcpy((char*) range_str, "2.0g");
            }
            else if (range == RANGE_4G)
            {
                strcat((char*) cal_file_name, "4");
                strcpy((char*) range_str, "4.0g");
            }
            else if (range == RANGE_8G)
            {
                strcat((char*) cal_file_name, "8");
                strcpy((char*) range_str, "8.0g");
            }
            else if (range == RANGE_16G)
            {
                strcat((char*) cal_file_name, "16");
                strcpy((char*) range_str, "16.0g");
            }
            else
                return;
            strcat((char*) cal_file_name, "g.ini");
        }
        else if (sensor == S_GYRO)
        {
            strcat((char*) cal_file_name, "/calib_gyro_"); //calib_gyro_250dps.ini
            address = NV_MPU9150_GYRO_CALIBRATION;
            strcpy((char*) sensor_str, "Gyro ");
            if (range == MPU9150_GYRO_250DPS)
            {
                strcat((char*) cal_file_name, "250");
                strcpy((char*) range_str, "250dps");
            }
            else if (range == MPU9150_GYRO_500DPS)
            {
                strcat((char*) cal_file_name, "500");
                strcpy((char*) range_str, "500dps");
            }
            else if (range == MPU9150_GYRO_1000DPS)
            {
                strcat((char*) cal_file_name, "1000");
                strcpy((char*) range_str, "1000dps");
            }
            else if (range == MPU9150_GYRO_2000DPS)
            {
                strcat((char*) cal_file_name, "2000");
                strcpy((char*) range_str, "2000dps");
            }
            else
                return;
            strcat((char*) cal_file_name, "dps.ini");
        }
        else if (sensor == S_MAG)
        {
            strcat((char*) cal_file_name, "/calib_mag_");   //calib_mag_13ga.ini
            address = NV_LSM303DLHC_MAG_CALIBRATION;
            strcpy((char*) sensor_str, "Mag ");
            if (range == LSM303_MAG_13GA)
            {
                strcat((char*) cal_file_name, "13");
                strcpy((char*) range_str, "1.3Ga");
            }
            else if (range == LSM303_MAG_19GA)
            {
                strcat((char*) cal_file_name, "19");
                strcpy((char*) range_str, "1.9Ga");
            }
            else if (range == LSM303_MAG_25GA)
            {
                strcat((char*) cal_file_name, "25");
                strcpy((char*) range_str, "2.5Ga");
            }
            else if (range == LSM303_MAG_40GA)
            {
                strcat((char*) cal_file_name, "40");
                strcpy((char*) range_str, "4.0Ga");
            }
            else if (range == LSM303_MAG_47GA)
            {
                strcat((char*) cal_file_name, "47");
                strcpy((char*) range_str, "4.7Ga");
            }
            else if (range == LSM303_MAG_56GA)
            {
                strcat((char*) cal_file_name, "56");
                strcpy((char*) range_str, "5.6Ga");
            }
            else if (range == LSM303_MAG_81GA)
            {
                strcat((char*) cal_file_name, "81");
                strcpy((char*) range_str, "8.1Ga");
            }
            else
                return;
            strcat((char*) cal_file_name, "ga.ini");
        }
        else if (sensor == S_ACCEL_A)
        {
            strcat((char*) cal_file_name, "/calib_accel_ln_2g.ini");
            address = NV_A_ACCEL_CALIBRATION;
            strcpy((char*) sensor_str, "Accel_A ");
            strcpy((char*) range_str, "2.0g");
        }
        else
        {
            return;
        }

        if (ff_result = f_open(&gfc, (char*) cal_file_name,
                               (FA_WRITE | FA_CREATE_ALWAYS))) // no calibration file, use default
            return;
        else
        {
            uint8_t i;
            char minus_sign[2];
            int16_t cal_val_16, val_int, val_f;
            uint8_t val_f_str[5];
            float cal_val_f;
            char cal_param_str[4];
            for (i = 0; i < 6; i++)
            {
                switch (i)
                {
                case 0:
                    strcpy((char*) cal_param_str, "b0");
                    break;
                case 1:
                    strcpy((char*) cal_param_str, "b1");
                    break;
                case 2:
                    strcpy((char*) cal_param_str, "b2");
                    break;
                case 3:
                    strcpy((char*) cal_param_str, "k0");
                    break;
                case 4:
                    strcpy((char*) cal_param_str, "k1");
                    break;
                case 5:
                    strcpy((char*) cal_param_str, "k2");
                    break;
                default:
                    return;
                }
                cal_val_16 =
                        (int16_t) (((uint16_t) storedConfig[address + 2 * i])
                                << 8)
                                + (uint16_t) storedConfig[address + 2 * i + 1];
                cal_val_f = (float) cal_val_16;
                minus_sign[0] = '\0';
                if ((sensor == S_GYRO) & (i >= 3))
                    cal_val_f /= 100;
                if (cal_val_f >= 0)
                {
                    val_int = (uint16_t) floor(cal_val_f); // the compiler can't handle sprintf %f here
                }
                else
                { //cal_val_f<0
                    cal_val_f *= -1;
                    strcpy(minus_sign, "-"); // in case -0 = 0
                    val_int = (uint16_t) floor(cal_val_f); // the compiler can't handle sprintf %f here
                }
                val_f = (uint16_t) round((cal_val_f - (float) val_int) * 10000);
                ItoaWith0((uint64_t) val_f, val_f_str, 5);
                sprintf(buffer, "%s = %s%d.%s\r\n", cal_param_str, minus_sign,
                        val_int, val_f_str);
                f_write(&gfc, buffer, strlen(buffer), &bw);
            }
            for (i = 6; i < 15; i++)
            {
                switch (i)
                {
                case 6:
                    strcpy((char*) cal_param_str, "r00");
                    break;
                case 7:
                    strcpy((char*) cal_param_str, "r01");
                    break;
                case 8:
                    strcpy((char*) cal_param_str, "r02");
                    break;
                case 9:
                    strcpy((char*) cal_param_str, "r10");
                    break;
                case 10:
                    strcpy((char*) cal_param_str, "r11");
                    break;
                case 11:
                    strcpy((char*) cal_param_str, "r12");
                    break;
                case 12:
                    strcpy((char*) cal_param_str, "r20");
                    break;
                case 13:
                    strcpy((char*) cal_param_str, "r21");
                    break;
                case 14:
                    strcpy((char*) cal_param_str, "r22");
                    break;
                default:
                    return;
                }
                cal_val_f = (float) ((int8_t) storedConfig[address + 6 + i]);
                cal_val_f /= 100;
                minus_sign[0] = '\0';
                if (cal_val_f >= 0)
                {
                    val_int = (uint16_t) floor(cal_val_f); // the compiler can't handle sprintf %f here
                }
                else
                { //cal_val_f<0
                    cal_val_f *= -1;
                    strcpy(minus_sign, "-");
                    val_int = (uint16_t) floor(cal_val_f); // the compiler can't handle sprintf %f here
                }
                val_f = (uint16_t) round((cal_val_f - (float) val_int) * 100);
                ItoaWith0((uint64_t) val_f, val_f_str, 3);
                sprintf(buffer, "%s = %s%d.%s\r\n", cal_param_str, minus_sign,
                        val_int, val_f_str);
                f_write(&gfc, buffer, strlen(buffer), &bw);
            }
            ff_result = f_close(&gfc);
            _delay_cycles(1200000);
        }
        if (!sd_power_state)
            SdPowerOff();
    }
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
        ADC_val = *((uint16_t *) txBuff0 + (nbrAdcChans - 1) + 2);
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
        *((uint16_t *) txBuff0 + (nbrAdcChans - 1) + 2) = ADC_val
                | (current_active_resistor << 14);
    }
    else
    {
        ADC_val = *((uint16_t *) txBuff1 + (nbrAdcChans - 1) + 2);
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
        *((uint16_t *) txBuff1 + (nbrAdcChans - 1) + 2) = ADC_val
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
    _delay_cycles(2880000);
    SdPowerOff();              //SW_FLASH set low, SdPowerOff();

    P5SEL &= ~(BIT4 + BIT5);
    P5OUT &= ~(BIT4 + BIT5);   //FLASH_SOMI and FLASH_SCLK set low
    P5DIR |= BIT4;             //FLASH_SOMI set as output
    P3SEL &= ~BIT7;
    P3OUT &= ~BIT7;            //FLASH_SIMO set low
    P4OUT &= ~BIT0;            //FLASH_CS_N set low
    P6OUT &= ~(BIT6 + BIT7);   //ADC6_FLASHDAT2 and ADC7_FLASHDAT1 set low
    P6DIR |= BIT6 + BIT7;      //ADC6_FLASHDAT2 and ADC7_FLASHDAT1 set as output

//60ms as taken from TinyOS driver (SDP.nc powerCycle() function)
    _delay_cycles(2880000);

    P5DIR &= ~(BIT4 + BIT5);   //FLASH_SOMI and FLASH_SCLK set as input
    P3DIR &= ~BIT7;            //FLASH_SIMO set as input
    P4DIR &= ~BIT0;            //FLASH_CS_N set as input
    P6DIR &= ~(BIT6 + BIT7);   //ADC6_FLASHDAT2 and ADC7_FLASHDAT1 set as input

    SdPowerOn();               //SW_FLASH set high, SdPowerOn();
    _delay_cycles(1200000);    //give SD card time to power back up
    P6OUT &= ~BIT0;            //DETECT_N set low
}

void SetupDock()
{
    configuring = 1;
    if (docked)
    {
        battInterval = BATT_INTERVAL_D;
        battCriticalCount = 0;
        enableSdlog = 0;
        sdlogReady = 0;
        newDirFlag = 1;
        if (CheckSdInslot())
        {
            DockSdPowerCycle();
        }
        if (!sensing)
        {
            UART_activate();
        }
        BtsdSelfcmd();
    }
    else
    {
        battInterval = BATT_INTERVAL;
        if (!sensing)
        {
            UART_deactivate();
        }
        P6OUT |= BIT0;
        //SendStatusByte();
        BtsdSelfcmd();
        SdPowerOff();
        if (CheckSdInslot() && !sensing && !SD_ERROR)
        {
            _delay_cycles(2880000);
            SdPowerOn();
            SdInfoSync();
        }
        else
        {
            sdInfoSyncDelayed = 1;
        }
    }
    configuring = 0;
}

void SdInfoSync()
{
    sdInfoSyncDelayed = 0;
    if (GetSdCfgFlag())
    { // info > sdcard
        IniReadInfoMem();
        UpdateSdConfig();
        SetSdCfgFlag(0);
    }
    else
    {
        ReadSdConfiguration();
    }

    if (GetRamCalibFlag())
    {
        ShimmerCalib_ram2File();
        SetRamCalibFlag(0);
    }
    else
    {
        if (ShimmerCalib_file2Ram())
        {
            ShimmerCalib_ram2File();
        }
        else
        {
            // only need to do this when file2Ram succeeds
            ShimmerCalibSyncFromDumpRamAll();
        }

    }

    ConfigureChannels();
    ChangeBtBaudRateFunc();
    CheckOnDefault();
}

void ChangeBtBaudRateFunc()
{
    if (!btIsConnected && (changeBtBaudRate != 0xFF) && !sensing)
    {
        storedConfig[NV_BT_COMMS_BAUD_RATE] = changeBtBaudRate;
        sdHeadText[SDH_BT_COMMS_BAUD_RATE] = changeBtBaudRate;
        InfoMem_write((void*) NV_BT_COMMS_BAUD_RATE,
                      &storedConfig[NV_BT_COMMS_BAUD_RATE], 1);
        SetBtBaudRate(storedConfig[NV_BT_COMMS_BAUD_RATE]);
        _delay_cycles(24000000);
        UpdateSdConfig();
        changeBtBaudRate = 0xFF;
    }
}

uint8_t SendStatusByte()
{
    if (btIsConnected && (!syncEnabled))
    {
        dockedResponse = 1;
        sendAck = 1;
        sendResponse = 1;
        return 1;
    }
    return 0;
}

void PostSdCfg(void)
{
    if (!sensing)
        ConfigureChannels();
    CalibAll();
    CheckOnDefault();
}
void ReadSdConfiguration(void)
{
    streamData = 0; // this will skip one sample
    newDirFlag = 1;
    SdPowerOn();
    ParseConfig();
}
void SdPowerOff(void)
{
    P4OUT &= ~BIT2;
} //   SD power off
void SdPowerOn(void)
{
    P4OUT |= BIT2;
} //   SD power on

uint8_t CheckSdInslot()
{
//Check if card is inserted and enable interrupt for SD_DETECT_N
    if (!(P4IN & BIT1))
    {
        ff_result = f_mount(0, &fatfs);
        set_sd_detect(1);
        return 1;
    }
    else
    {
        ff_result = f_mount(0, NULL);
        set_sd_detect(0);
        return 0;
    }
}

void IniReadInfoMem()
{
    InfoMem_read((uint8_t *) 0, storedConfig, NV_NUM_RWMEM_BYTES);
    if (storedConfig[NV_SENSORS1] == 0xFF)
    {
        //if config was never written to Infomem, write default
        //assuming some other app didn't make use of InfoMem, or else InfoMem was erased
        SetDefaultConfiguration();
        //return 1;
    }
    Config2SdHead();
    Infomem2Names();
//return 0;
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
    uint16_t crc_value;
    uint8_t adc_offset;
    uint8_t* current_buffer_ptr;
    adc_offset = 4;

    uint8_t digi_offset = (nbrAdcChans * 2) + adc_offset;
    if (storedConfig[NV_SENSORS0] & SENSOR_GSR)
    {
        GsrRange();
    }

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

    isStreaming = enableBtstream && btstreamReady;
    isLogging = enableSdlog && sdlogReady;

//uint8_t self_stop_sensing = 0;
    if (stopLogging)
    {
        stopLogging = 0;
        if (isLogging)
        {
            if (!isStreaming)
            {
                stopSensing = 1;
            }
            else
            {
                isLogging = 0;
                newDirFlag = 1;
                f_close(&fil);
                _delay_cycles(1200000);
                SdPowerOff();
                enableSdlog = 0;
            }
        }
    }

    if (stopStreaming)
    {
        stopStreaming = 0;
        if (isStreaming)
        {
            enableBtstream = 0;
            isStreaming = 0;
            if (!isLogging)
            {
                stopSensing = 1;
            }
        }
    }

    if (stopSensing)
    {
        stopSensing = 0;
        StopSensing();
    }
    else
    { //here does the log and stream job
#if SKIP65MS
        if (Skip65ms())
        {
            firstTsFlag = 1; // has to re collect the init_ts
            return;
        }
#endif
        if (enableSdlog && sdlogReady)
        {
#if TS_BYTE3
            if (firstTsFlag == 2)
            {
                firstTsFlag = 3;
                Timestamp0ToFirstFile();
            }

            memcpy(sdBuff + sdBuffLen, current_buffer_ptr + 1, digi_offset - 1);
            sdBuffLen += digi_offset - 1;
#else
            memcpy(sdBuff+sdBuffLen, current_buffer_ptr+2, digi_offset-2);
            sdBuffLen+=digi_offset-2;
#endif
        }

        if (enableBtstream && btstreamReady && btIsConnected && !syncEnabled)
        {
            if (crcChecksum)
            {
                crc_value = CRC_data(current_buffer_ptr + 1, digi_offset - 1);
                *(current_buffer_ptr + digi_offset++) = crc_value & 0xFF;
                *(current_buffer_ptr + digi_offset++) = (crc_value & 0xFF00)
                        >> 8;
            }
#if TS_BYTE3
            BT_write(current_buffer_ptr, digi_offset);
#else
            BT_write(current_buffer_ptr+1, digi_offset-1);
#endif
        }
    }
}

uint8_t Skip65ms()
{
    if (!skip65ms)
    {
        return 0;
    }
    if (skip65ms == 1)
    {
        startSensingTs64 = RTC_get64();
        skip65ms = 2;
    }
    else
    {
        uint64_t ts_64 = RTC_get64();
        if ((ts_64 - startSensingTs64) > 2130)
        { // 2130 - 65 ms, 1638 - 50 ms
            skip65ms = 0;
            return 0;
        }
    }
    return 1;
}

void RstRcommVariables()
{
    rcommCnt = 0;
    rcommWindowCenter = RC_WINDOW_C;
    rcommCurrentTry = 0;
    rcommInterval = RC_INT_N;
    rcommIntervalCenter = storedConfig[NV_SD_BT_INTERVAL];
    rcommSuccess = 0;
}
void Write2SD()
{
    if (sdBuffLen)
        WriteFile(sdBuff, sdBuffLen);
    sdBuffLen = 0;
    if (storedConfig[NV_SD_TRIAL_CONFIG0] & SDH_TIME_SYNC)
        PrepareSDBuffHead();
}

uint8_t GetRamCalibFlag()
{
    return calibRamFlag;
}
void SetRamCalibFlag(uint8_t flag)
{
// flag == 1: Ram>File, ShimmerCalib_ram2File()
//         0: File>Ram, ShimmerCalib_file2Ram()
    calibRamFlag = flag;
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
uint8_t GetCalibFlag()
{
    uint8_t sd_config_delay_flag = 0;
    InfoMem_read((uint8_t*) NV_SD_CONFIG_DELAY_FLAG, &sd_config_delay_flag, 1);
    if (!(sd_config_delay_flag & BIT7))
    {
        if (sd_config_delay_flag & BIT1)
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
void SetCalibFlag(uint8_t flag)
{
    uint8_t sd_config_delay_flag;
    if (flag)
    {
        InfoMem_read((uint8_t*) NV_SD_CONFIG_DELAY_FLAG, &sd_config_delay_flag,
                     1);
        if (!(sd_config_delay_flag & BIT7))
        {
            sd_config_delay_flag |= BIT1;
        }
        else
        {
            sd_config_delay_flag = BIT1;
        }
        InfoMem_write((uint8_t*) NV_SD_CONFIG_DELAY_FLAG, &sd_config_delay_flag,
                      1);
    }
    else
    {
        InfoMem_read((uint8_t*) NV_SD_CONFIG_DELAY_FLAG, &sd_config_delay_flag,
                     1);
        sd_config_delay_flag &= ~BIT1;
        InfoMem_write((uint8_t*) NV_SD_CONFIG_DELAY_FLAG, &sd_config_delay_flag,
                      1);
    }
}

void UpdateSdConfig()
{
    if (!docked && CheckSdInslot() && !SD_ERROR)
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

        char buffer[66], val_char[21];
        //uint8_t i;
        float val_num;
        uint16_t val_int, val_f;
        uint32_t temp32;
        uint64_t temp64;
        UINT bw;

        char cfgname[] = "sdlog.cfg";

        ff_result = f_open(&fil, cfgname, FA_WRITE | FA_CREATE_ALWAYS);

        memset(shimmerName, 0, MAX_CHARS);
        memset(expIdName, 0, MAX_CHARS);
        memset(configTimeText, 0, MAX_CHARS);

        SamplingClkAssignment();
        ClkAssignment();
        TB0CTL = MC_0;
        TB0Start();

        //sensor0
        sprintf(buffer, "accel=%d\r\n",
                storedConfig[NV_SENSORS0] & SENSOR_A_ACCEL ? 1 : 0);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "gyro=%d\r\n",
                storedConfig[NV_SENSORS0] & SENSOR_MPU9150_GYRO ? 1 : 0);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "mag=%d\r\n",
                storedConfig[NV_SENSORS0] & SENSOR_LSM303DLHC_MAG ? 1 : 0);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "exg1_24bit=%d\r\n",
                storedConfig[NV_SENSORS0] & SENSOR_EXG1_24BIT ? 1 : 0);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "exg2_24bit=%d\r\n",
                storedConfig[NV_SENSORS0] & SENSOR_EXG2_24BIT ? 1 : 0);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "gsr=%d\r\n",
                storedConfig[NV_SENSORS0] & SENSOR_GSR ? 1 : 0);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "extch7=%d\r\n",
                storedConfig[NV_SENSORS0] & SENSOR_EXT_A7 ? 1 : 0);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "extch6=%d\r\n",
                storedConfig[NV_SENSORS0] & SENSOR_EXT_A6 ? 1 : 0);
        f_write(&fil, buffer, strlen(buffer), &bw);
        //sensor1
        sprintf(buffer, "br_amp=%d\r\n",
                storedConfig[NV_SENSORS1] & SENSOR_STRAIN ? 1 : 0);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "vbat=%d\r\n",
                storedConfig[NV_SENSORS1] & SENSOR_VBATT ? 1 : 0);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "accel_d=%d\r\n",
                storedConfig[NV_SENSORS1] & SENSOR_LSM303DLHC_ACCEL ? 1 : 0);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "extch15=%d\r\n",
                storedConfig[NV_SENSORS1] & SENSOR_EXT_A15 ? 1 : 0);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "intch1=%d\r\n",
                storedConfig[NV_SENSORS1] & SENSOR_INT_A1 ? 1 : 0);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "intch12=%d\r\n",
                storedConfig[NV_SENSORS1] & SENSOR_INT_A12 ? 1 : 0);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "intch13=%d\r\n",
                storedConfig[NV_SENSORS1] & SENSOR_INT_A13 ? 1 : 0);
        f_write(&fil, buffer, strlen(buffer), &bw);
        //sensor2
        sprintf(buffer, "intch14=%d\r\n",
                storedConfig[NV_SENSORS2] & SENSOR_INT_A14 ? 1 : 0);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "accel_mpu=%d\r\n",
                storedConfig[NV_SENSORS2] & SENSOR_MPU9150_ACCEL ? 1 : 0);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "mag_mpu=%d\r\n",
                storedConfig[NV_SENSORS2] & SENSOR_MPU9150_MAG ? 1 : 0);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "exg1_16bit=%d\r\n",
                storedConfig[NV_SENSORS2] & SENSOR_EXG1_16BIT ? 1 : 0);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "exg2_16bit=%d\r\n",
                storedConfig[NV_SENSORS2] & SENSOR_EXG2_16BIT ? 1 : 0);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer,
                ((bmpInUse == BMP180_IN_USE) ?
                        ("pres_bmp180=%d\r\n") : ("pres_bmp280=%d\r\n")),
                storedConfig[NV_SENSORS2] & SENSOR_BMPX80_PRESSURE ? 1 : 0);
        f_write(&fil, buffer, strlen(buffer), &bw);
        // sample_rate
        val_num = clockFreq / (*(uint16_t*) (storedConfig + NV_SAMPLING_RATE));
        val_int = (uint16_t) floor(val_num); // the compiler can't handle sprintf %f here
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
        f_write(&fil, buffer, strlen(buffer), &bw);
        // setup config
        sprintf(buffer, "mg_internal_rate=%d\r\n",
                (storedConfig[NV_CONFIG_SETUP_BYTE2] >> 2) & 0x07);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "mg_range=%d\r\n",
                (storedConfig[NV_CONFIG_SETUP_BYTE2] >> 5) & 0x07);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "acc_internal_rate=%d\r\n",
                (storedConfig[NV_CONFIG_SETUP_BYTE0] >> 4) & 0x0f);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "accel_mpu_range=%d\r\n",
                (storedConfig[NV_CONFIG_SETUP_BYTE3] >> 6) & 0x03);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer,
                ((bmpInUse == BMP180_IN_USE) ?
                        ("pres_bmp180_prec=%d\r\n") :
                        ("pres_bmp280_prec=%d\r\n")),
                (storedConfig[NV_CONFIG_SETUP_BYTE3] >> 4) & 0x03);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "gsr_range=%d\r\n",
                (storedConfig[NV_CONFIG_SETUP_BYTE3] >> 1) & 0x07);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "exp_power=%d\r\n",
                storedConfig[NV_CONFIG_SETUP_BYTE3] & EXP_POWER_ENABLE ? 1 : 0);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "gyro_range=%d\r\n",
                (storedConfig[NV_CONFIG_SETUP_BYTE2]) & 0x03);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "gyro_samplingrate=%d\r\n",
                storedConfig[NV_CONFIG_SETUP_BYTE1]);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "acc_range=%d\r\n",
                (storedConfig[NV_CONFIG_SETUP_BYTE0] >> 2) & 0x03);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "acc_lpm=%d\r\n",
                (storedConfig[NV_CONFIG_SETUP_BYTE0] >> 1) & 0x01);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "acc_hrm=%d\r\n",
                (storedConfig[NV_CONFIG_SETUP_BYTE0]) & 0x01);
        f_write(&fil, buffer, strlen(buffer), &bw);
        // trial config
        sprintf(buffer,
                "user_button_enable=%d\r\n",
                storedConfig[NV_SD_TRIAL_CONFIG0] & SDH_USER_BUTTON_ENABLE ?
                        1 : 0);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "rtc_error_enable=%d\r\n",
                storedConfig[NV_SD_TRIAL_CONFIG0] & SDH_RWCERROR_EN ? 1 : 0);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "sd_error_enable=%d\r\n",
                storedConfig[NV_SD_TRIAL_CONFIG0] & SDH_SDERROR_EN ? 1 : 0);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "iammaster=%d\r\n",
                storedConfig[NV_SD_TRIAL_CONFIG0] & SDH_IAMMASTER ? 1 : 0);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "sync=%d\r\n",
                storedConfig[NV_SD_TRIAL_CONFIG0] & SDH_TIME_SYNC ? 1 : 0);
        f_write(&fil, buffer, strlen(buffer), &bw);
        //sprintf(buffer, "singletouch=%d\r\n", storedConfig[NV_SD_TRIAL_CONFIG1]&SDH_SINGLETOUCH?1:0);
        //f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer,
                "low_battery_autostop=%d\r\n",
                storedConfig[NV_SD_TRIAL_CONFIG1] & SDH_BATT_CRITICAL_CUTOFF ?
                        1 : 0);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "tcxo=%d\r\n",
                storedConfig[NV_SD_TRIAL_CONFIG1] & SDH_TCXO ? 1 : 0);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "interval=%d\r\n", storedConfig[NV_SD_BT_INTERVAL]);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "myid=%d\r\n", storedConfig[NV_SD_MYTRIAL_ID]);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "Nshimmer=%d\r\n", storedConfig[NV_SD_NSHIMMER]);
        f_write(&fil, buffer, strlen(buffer), &bw);
        //SetName();
        /*for(i=0; (i<MAX_CHARS-1)&&isprint(storedConfig[NV_SD_SHIMMER_NAME+i]) ; i++);
         if(i){
         memcpy((char*)shimmerName, (char*)(storedConfig+NV_SD_SHIMMER_NAME), i);
         shimmerName[i] = 0;
         }
         for(i=0; (i<MAX_CHARS-1)&&isprint(storedConfig[NV_SD_EXP_ID_NAME+i]) ; i++);
         if(i){
         memcpy((char*)expIdName, (char*)(storedConfig+NV_SD_EXP_ID_NAME), i);
         expIdName[i] = 0;
         }
         uint32_t cfg_time_temp = 0;
         for(i=0;i<4;i++){
         cfg_time_temp<<=8;
         cfg_time_temp +=storedConfig[NV_SD_CONFIG_TIME+i];
         }
         ItoaWith0((uint64_t)cfg_time_temp, configTimeText, UINT32_LEN);*/
        Infomem2Names();
        sprintf(buffer, "shimmername=%s\r\n", shimmerName);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "experimentid=%s\r\n", expIdName);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "configtime=%s\r\n", configTimeText);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "baud_rate=%d\r\n",
                storedConfig[NV_BT_COMMS_BAUD_RATE]);
        f_write(&fil, buffer, strlen(buffer), &bw);
        //      temp32 = storedConfig[NV_DERIVED_CHANNELS_0]
        //             + (((uint32_t)storedConfig[NV_DERIVED_CHANNELS_1])<<8)
        //             + (((uint32_t)storedConfig[NV_DERIVED_CHANNELS_2])<<16);
        //      ItoaNo0((uint64_t)temp32, (uint8_t*)val_char, 9);
        temp64 = storedConfig[NV_DERIVED_CHANNELS_0]
                + (((uint64_t) storedConfig[NV_DERIVED_CHANNELS_1]) << 8)
                + (((uint64_t) storedConfig[NV_DERIVED_CHANNELS_2]) << 16)
                + (((uint64_t) storedConfig[NV_DERIVED_CHANNELS_3]) << 24)
                + (((uint64_t) storedConfig[NV_DERIVED_CHANNELS_4]) << 32)
                + (((uint64_t) storedConfig[NV_DERIVED_CHANNELS_5]) << 40)
                + (((uint64_t) storedConfig[NV_DERIVED_CHANNELS_6]) << 48)
                + (((uint64_t) storedConfig[NV_DERIVED_CHANNELS_7]) << 56);
        ItoaNo0(temp64, (uint8_t*) val_char, 21);
        sprintf(buffer, "derived_channels=%s\r\n", val_char); // todo: got value 0?
        f_write(&fil, buffer, strlen(buffer), &bw);
        temp32 = (uint32_t) storedConfig[NV_MAX_EXP_LEN_LSB];
        temp32 += ((uint32_t) storedConfig[NV_MAX_EXP_LEN_MSB]) << 8;
        maxLen = temp32 * 60;
        sprintf(buffer, "max_exp_len=%d\r\n", temp32);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "EXG_ADS1292R_1_CONFIG1=%d\r\n",
                storedConfig[NV_EXG_ADS1292R_1_CONFIG1]);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "EXG_ADS1292R_1_CONFIG2=%d\r\n",
                storedConfig[NV_EXG_ADS1292R_1_CONFIG2]);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "EXG_ADS1292R_1_LOFF=%d\r\n",
                storedConfig[NV_EXG_ADS1292R_1_LOFF]);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "EXG_ADS1292R_1_CH1SET=%d\r\n",
                storedConfig[NV_EXG_ADS1292R_1_CH1SET]);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "EXG_ADS1292R_1_CH2SET=%d\r\n",
                storedConfig[NV_EXG_ADS1292R_1_CH2SET]);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "EXG_ADS1292R_1_RLD_SENS=%d\r\n",
                storedConfig[NV_EXG_ADS1292R_1_RLD_SENS]);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "EXG_ADS1292R_1_LOFF_SENS=%d\r\n",
                storedConfig[NV_EXG_ADS1292R_1_LOFF_SENS]);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "EXG_ADS1292R_1_LOFF_STAT=%d\r\n",
                storedConfig[NV_EXG_ADS1292R_1_LOFF_STAT]);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "EXG_ADS1292R_1_RESP1=%d\r\n",
                storedConfig[NV_EXG_ADS1292R_1_RESP1]);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "EXG_ADS1292R_1_RESP2=%d\r\n",
                storedConfig[NV_EXG_ADS1292R_1_RESP2]);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "EXG_ADS1292R_2_CONFIG1=%d\r\n",
                storedConfig[NV_EXG_ADS1292R_2_CONFIG1]);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "EXG_ADS1292R_2_CONFIG2=%d\r\n",
                storedConfig[NV_EXG_ADS1292R_2_CONFIG2]);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "EXG_ADS1292R_2_LOFF=%d\r\n",
                storedConfig[NV_EXG_ADS1292R_2_LOFF]);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "EXG_ADS1292R_2_CH1SET=%d\r\n",
                storedConfig[NV_EXG_ADS1292R_2_CH1SET]);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "EXG_ADS1292R_2_CH2SET=%d\r\n",
                storedConfig[NV_EXG_ADS1292R_2_CH2SET]);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "EXG_ADS1292R_2_RLD_SENS=%d\r\n",
                storedConfig[NV_EXG_ADS1292R_2_RLD_SENS]);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "EXG_ADS1292R_2_LOFF_SENS=%d\r\n",
                storedConfig[NV_EXG_ADS1292R_2_LOFF_SENS]);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "EXG_ADS1292R_2_LOFF_STAT=%d\r\n",
                storedConfig[NV_EXG_ADS1292R_2_LOFF_STAT]);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "EXG_ADS1292R_2_RESP1=%d\r\n",
                storedConfig[NV_EXG_ADS1292R_2_RESP1]);
        f_write(&fil, buffer, strlen(buffer), &bw);
        sprintf(buffer, "EXG_ADS1292R_2_RESP2=%d\r\n",
                storedConfig[NV_EXG_ADS1292R_2_RESP2]);
        f_write(&fil, buffer, strlen(buffer), &bw);

        ff_result = f_close(&fil);
        _delay_cycles(2400000);
        if (!sd_power_state)
        {
            SdPowerOff();
        }
    }
}

void FindError(uint8_t err, uint8_t *name)
{
    switch (err)
    {
    case 0:
        strcpy((char*) name, "OK");
        break;
    case 1:
        strcpy((char*) name, "DISK_ERR");
        break;
    case 2:
        strcpy((char*) name, "INT_ERR");
        break;
    case 3:
        strcpy((char*) name, "NOT_READY");
        break;
    case 4:
        strcpy((char*) name, "NO_FILE");
        break;
    case 5:
        strcpy((char*) name, "NO_PATH");
        break;
    case 6:
        strcpy((char*) name, "INVALID_NAME");
        break;
    case 7:
        strcpy((char*) name, "DENIED");
        break;
    case 8:
        strcpy((char*) name, "EXIST");
        break;
    case 9:
        strcpy((char*) name, "INVALID_OBJ");
        break;
    case 10:
        strcpy((char*) name, "WRITE_PROTEC");
        break;
    case 11:
        strcpy((char*) name, "INVALID_DRIV");
        break;
    case 12:
        strcpy((char*) name, "NOT_ENABLED");
        break;
    case 13:
        strcpy((char*) name, "NO_FILESYSTE");
        break;
    case 14:
        strcpy((char*) name, "MKFS_ABORTED");
        break;
    case 15:
        strcpy((char*) name, "TIMEOUT");
        break;
    case 16:
        strcpy((char*) name, "LOCKED");
        break;
    case 17:
        strcpy((char*) name, "NOT_ENOUGH_C");
        break;
    case 18:
        strcpy((char*) name, "TOO_MANY_OPE");
        break;
    default:
        strcpy((char*) name, "NO_REASON");
        break;
    } //FRESULT;
}

void BattBlinkOn()
{
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
     * ADC12CONSEQ_0 - ADC12OSC and single channel mode (SEQ_0) */
    ADC12CTL1 = ADC12SHP + ADC12CONSEQ_0;

    /* ADC12TCOFF - Turn off temperature sensor (saves power)
     * ADC12RES_2 - ADC12 resolution to 12bits
     * ADC12SR_L  - Sampling rate 50ksps */
    ADC12CTL2 = ADC12TCOFF + ADC12RES_2 + ADC12SR_L;

    if (storedConfig[NV_SENSORS1] & SENSOR_VBATT)
    {
        if (storedConfig[NV_SENSORS0] & SENSOR_A_ACCEL)
        {
            ADC12CTL1 += ADC12CSTARTADD_4;                //Start with ADC12MEM4
            ADC12MCTL4 = ADC12INCH_2;
        }
        else
        {
            ADC12CTL1 += ADC12CSTARTADD_1;                //Start with ADC12MEM1
            ADC12MCTL1 = ADC12INCH_2;
        }
    }
    else
    {
        ADC12CTL1 += ADC12CSTARTADD_0;                	  //Start with ADC12MEM0
        ADC12MCTL0 = ADC12INCH_2;
    }

    DMACTL0 |= DMA0TSEL_24;            //ADC12IFGx triggered
    DMACTL4 = DMARMWDIS;               //Read-modify-write disable
    DMA0CTL &= ~DMAIFG;

    /* DMADT_2 		- Burst block transfer
     * DMADSTINCR_3 - Increment destination address
     * DMADSRINCR_3 - Increment source address
     * DMAIE 		- DMA interrupt enable
     * DMAEN        - DMA Enable */
//    DMA0CTL = DMADT_2 + DMADSTINCR_3 + DMASRCINCR_3 + DMAIE
//            + ((isLogging || isStreaming) ? DMAEN : 0);
    DMA0CTL = DMADT_2 + DMADSTINCR_3 + DMASRCINCR_3 + DMAIE
            + (((isLogging || isStreaming)
                    && storedConfig[NV_SENSORS0] & SENSOR_GSR) ? DMAEN : 0);
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

//	Writes a value to a 20-bit SFR register located at the given16/20-bit address
    DMA0DA = (__SFR_FARPTR) (unsigned long) battVal;

    DMA0_transferDoneFunction(&Dma0BatteryRead);

    ADC12IFG = 0;
    DMA0CTL |= DMAEN;
    ADC12CTL0 |= ADC12ON + ADC12ENC + ADC12SC;
}

void TB0Start()
{
    if (sampleTimerStatus + blinkStatus >= 1)
    {
        if (clockFreq == (float) MSP430_CLOCK)
        {
            TB0CTL = TBSSEL_1 + MC_2;
            TB0EX0 = 0;
        }
        else
        {
            TB0CTL = TBSSEL_0 + MC_2 + ID__8; // use TBSSEL_0 for tcxo, ID__8:divider=8//+ TBCLR
            TB0EX0 = TBIDEX__8;   // divider: 8 - Note TCXO_CLOCK will change...
        }
    }
}

void TB0Stop()
{
    if (!sampleTimerStatus && !blinkStatus)
    {
        TB0CTL = MC_0; // StopTb0()
    }
}

void ClkAssignment()
{
    clk_10 = FreqProd(10);
    clk_50 = FreqProd(50);
    clk_150 = FreqProd(150);
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
    clk_250 = FreqProd(250);
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
{ // e.g. 7.5 ms: num_in=75, .25s:num_in=2500
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
//    return (uint16_t) ceil(clockFreq / num_in);
//    return (uint16_t) round(((float)clockFreq) / num_in);

    if (clockFreq == (float) MSP430_CLOCK)
        {
            return (uint16_t) round(((float)MSP430_CLOCK) / num_in);
        }
        else
        {
            return (uint16_t) round(((float)TCXO_CLOCK) / num_in);
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

uint8_t Dma0BatteryRead(void)
{
    ADC_disable();
    DMA0_disable();
//== new uart starts ==
//uartSendRspBat = 1;// don't do this for sdlog/L&S
//uartSendResponses = 1;
//== new uart ends ==
    return 1;
}

void ReadBatt(void)
{
    SetBattDma();
// Produces spikes in PPG data when only GSR enabled & aaccel, vbatt are off.
    __bis_SR_register(LPM3_bits + GIE); //ACLK remains active
    configureChannels = 1;

    uint16_t currentBattVal = *((uint16_t*) battVal);
// was: 0 - 2400 - 2600 - 4096
// now: 0 - 2568 - 2717 - 4096
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

    // 10% Battery cutoff point - v0.9.6 onwards
    if ((storedConfig[NV_SD_TRIAL_CONFIG1] & SDH_BATT_CRITICAL_CUTOFF)
            && (currentBattVal < BATT_CUTOFF_3_65VOLTS))
    {
        if (battCriticalCount++ > 2)
        {
            battCritical = TRUE;
            if (sensing)
            {
                stopSensing = 1;
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
                uartSendResponses = 1;
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
                uartProcessCmds = 1;
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

void UartProcessCmd()
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

                            if (uartInfoMemOffset == (INFOMEM_SEG_C_ADDR - INFOMEM_OFFSET))
                            {
                                /* Read MAC address so it is not forgotten */
                                InfoMem_read((uint8_t *) NV_MAC_ADDRESS,
                                             macAddr, 6);
                            }
                            if (uartInfoMemOffset == (INFOMEM_SEG_D_ADDR - INFOMEM_OFFSET))
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
                            if (uartInfoMemOffset == (INFOMEM_SEG_C_ADDR - INFOMEM_OFFSET))
                            {
                                /* Re-write MAC address to Infomem */
                                InfoMem_write((uint8_t*) NV_MAC_ADDRESS,
                                              macAddr, 6);
                            }
                            /* Reload latest infomem bytes to RAM */
                            InfoMem_read((uint8_t *) uartInfoMemOffset,
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
                            CAT24C16_init();
                            CAT24C16_write(uartDcMemOffset,
                                           (uint16_t) uartDcMemLength,
                                           uartRxBuf + UART_RXBUF_DATA + 2);
                            CAT24C16_powerOff();
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
                            CAT24C16_init();
                            CAT24C16_write(uartDcMemOffset + 16,
                                           (uint16_t) uartDcMemLength,
                                           uartRxBuf + UART_RXBUF_DATA + 3);
                            CAT24C16_powerOff();
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
            uartSendRspBadCrc = 1;
        uartSendResponses = 1;
    }
}

void UartSendRsp()
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
        *(uartRespBuf + uart_resp_len++) = (FW_VER_REL + ((FACTORY_TEST) ? 200 : 0));
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
         //   CAT24C16_init();
          //  CAT24C16_read(uartDcMemOffset, (uint16_t) uartDcMemLength,
          //               (uartRespBuf + uart_resp_len));
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
            if (!sensing)
            {
                CAT24C16_init();
                CAT24C16_read(uartDcMemOffset + 16, (uint16_t) uartDcMemLength,
                              (uartRespBuf + uart_resp_len));
                CAT24C16_powerOff();
            }
            else
            {
                memset(resPacket + uart_resp_len, 0xff, uartDcMemLength);
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

void ReadWriteSDTest(void)
{
    if (!SD_ERROR)
    {
        UINT bw;
        char testFileName[] = "testFile.txt";
        uint8_t dummyData[100];
        if (!(P4OUT & BIT2))
        {
            SdPowerOn();
        }

        ff_result = f_open(&fil, (char*) testFileName,
        FA_WRITE | FA_CREATE_ALWAYS);
        memset(dummyData, 0xff, sizeof(dummyData) / sizeof(dummyData[0]));
        ff_result = f_write(&fil, dummyData,
                            sizeof(dummyData) / sizeof(dummyData[0]), &bw);
        __delay_cycles(1200000);
        ff_result = f_close(&fil);
        __delay_cycles(1200000);

        ff_result = f_open(&fil, (char*) testFileName,
        FA_READ | FA_OPEN_EXISTING);
        __delay_cycles(1200000);
        f_unlink(testFileName);
        if (ff_result != FR_OK)
        {
            sdlogReady = 0;
            SD_ERROR = TRUE;
        }
        else
        {
            ff_result = f_close(&fil);
            __delay_cycles(1200000);
        }
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
            _delay_cycles(2400000); // 100ms delay for tcxo
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
#pragma vector = WDT_VECTOR, SYSNMI_VECTOR, TIMER0_A0_VECTOR, \
        UNMI_VECTOR, USCI_B1_VECTOR, TIMER1_A1_VECTOR
__interrupt void TrapIsr(void)
{
    /*
     * this is a trap ISR - check for the interrupt cause here by
     * checking the interrupt flags, if necessary also clear the interrupt
     * flag
     */
}
