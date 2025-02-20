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
#include "shimmer3_common_source/5xx_HAL/hal_FactoryTest.h"
#include "shimmer_btsd.h"
#include "log_and_stream_globals.h"
#include "battery.h"

void Init(void);
void resetBtResponseBools(void);
void handleIfDockedStateOnBoot(void);
void detectI2cSlaves(void);
void loadSensorConfigurationAndCalibration(void);
void checkBtModeConfig(void);
void saveBmpCalibrationToSdHeader(void);
void ProcessHwRevision(void);
void InitialiseBt(void);
void InitialiseBtAfterBoot(void);
void BlinkTimerStart(void);
void BlinkTimerStop(void);
void setBootStage(boot_stage_t bootStageNew);
void SampleTimerStart(void);
inline void SampleTimerStop(void);
void StartLogging(void);
void StartStreaming(void);
inline void StopSensing(void);
uint8_t Dma0ConversionDone(void);
void setMacId(uint8_t *buf);
void ProcessCommand(void);
void SendResponse(void);
void ConfigureChannels(void);
void Timestamp0ToFirstFile();
FRESULT WriteFile(uint8_t *text, WORD size);
uint32_t TaskCurrentGet(void);
void TaskClear(TASK_FLAGS task_id);
uint8_t TaskGet(TASK_FLAGS task_id);
uint8_t TaskSet(TASK_FLAGS task_id);
void PrepareSDBuffHead(void);
void SetName();
void SetShimmerName();
void SetExpIdName();
void SetCfgTime();
error_t SetBasedir();
error_t MakeBasedir();
void FindError(uint8_t err, uint8_t *name);
inline void GsrRange(void);
void ItoaWith0(uint64_t num, uint8_t *buf, uint8_t len);
void ItoaNo0(uint64_t num, uint8_t *buf, uint8_t max_len);
uint64_t Atol64(uint8_t *buf);
void DockSdPowerCycle();
void SetupDock(void);
uint8_t SendStatusByte();
void ReadSdConfiguration(void);
void PostSdCfg(void);
void SdPowerOff(void);
void SdPowerOn(void);
uint8_t CheckSdInslot();
void BtStop(uint8_t isCalledFromMain);
void BtStart(void);
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
void CalibSaveFromInfoMemToCalibDump(uint8_t id);
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
void HandleBtRfCommStateChange(bool isOpen);
#if BT_ENABLE_BAUD_RATE_CHANGE
void BtBaudRateChangeDone(void);
void SetBtBaudRate(uint8_t rate);
#endif
void BtsdSelfcmd();
void SdInfoSync();
#if BT_ENABLE_BAUD_RATE_CHANGE
void ChangeBtBaudRateFunc();
#endif
inline uint8_t Skip65ms();
void SetStartSensing();
void setStopSensing(uint8_t state);
void saveBatteryVoltageAndUpdateStatus(void);
void ReadWriteSDTest(void);
void SamplingClkAssignment(uint8_t *storedConfigPtr);
uint16_t getBmpX80SamplingTimeInTicks(void);
uint16_t getBmpX80SamplingTimeDiffFrom9msInTicks(void);
void updateBtDetailsInEeprom(void);
void eepromRead(uint16_t dataAddr, uint16_t dataSize, uint8_t *dataBuf);
void eepromWrite(uint16_t dataAddr, uint16_t dataSize, uint8_t *dataBuf);
void eepromReadWrite(uint16_t dataAddr, uint16_t dataSize, uint8_t *dataBuf,
                     enum EEPROM_RW eepromRW);

uint8_t setTaskNewBtCmdToProcess(void);

uint8_t btsdSelfCmd, lastLedGroup2, rwcErrorFlash, rwcErrorEn,
        sdErrorFlash;
uint32_t buttonPressTs, buttonReleaseTs, buttonLastReleaseTs;

uint32_t maxLen, maxLenCnt;

uint64_t buttonPressTs64, buttonReleaseTs64, buttonLastReleaseTs64;

uint8_t fwInfo[7], ackStr[4],
        storedConfig[NV_NUM_RWMEM_BYTES], channelContents[MAX_NUM_CHANNELS],
        nbrAdcChans, nbrDigiChans, infomemLength, calibRamLength;
uint16_t *adcStartPtr, infomemOffset, calibRamOffset;

uint8_t currentBuffer, sendAck, inquiryResponse,
        samplingRateResponse, stopSensing,
        lnAccelCalibrationResponse, gyroCalibrationResponse,
        magCalibrationResponse, wrAccelCalibrationResponse,
        allCalibrationResponse,
        deviceVersionResponse,
        fwVersionResponse, bufferSizeResponse, uniqueSerialResponse,
        configSetupBytesResponse, wrAccelRangeResponse,
        magGainResponse, magSamplingRateResponse,
        wrAccelSamplingRateResponse, wrAccelLpModeResponse,
        wrAccelHrModeResponse, gyroRangeResponse,
        gyroSamplingRateResponse, altAccelRangeResponse,
        mpu9150MagSensAdjValsResponse,
        bmp180CalibrationCoefficientsResponse,
        bmp280CalibrationCoefficientsResponse, pressureOversamplingRatioResponse,
        blinkLedResponse, gsrRangeResponse, internalExpPowerEnableResponse,
        exgRegsResponse, dcIdResponse, dcMemResponse, dockedResponse,
        trialConfigResponse, centerResponse, shimmerNameResponse, expIDResponse,
        nshimmerResponse, myIDResponse, configTimeResponse, dirResponse,
        btCommsBaudRateResponse, derivedChannelResponse, infomemResponse,
        rwcResponse, stopLogging, stopStreaming,
        btVbattResponse, skip65ms, calibRamResponse, btVerResponse,
        useAckPrefixForInstreamResponses, btDataRateTestResponse,
        bmpGenericCalibrationCoefficientsResponse;
volatile uint8_t gAction;
uint8_t args[MAX_COMMAND_ARG_SIZE];
uint8_t resPacket[RESPONSE_PACKET_SIZE + 2]; //+2 for crc checksum bytes;
uint8_t watchDogWasOnDuringBtStart;

// file system vars
FATFS fatfs;         // File object
DIRS dir;               //Directory object
FIL dataFile;
/* make dir for SDLog files*/
uint8_t dirName[64], expDirName[32], sdHeadText[SDHEAD_LEN],
        expIdName[MAX_CHARS],
        shimmerName[MAX_CHARS], daughtCardId[CAT24C16_PAGE_SIZE], firstTsFlag,
        fileName[64],
        configTimeText[UINT32_LEN], //,exp_id_name[MAX_CHARS], shimmername[MAX_CHARS], centername[MAX_CHARS],
        txBuff0[DATA_PACKET_SIZE + 2], txBuff1[DATA_PACKET_SIZE + 2],
        sdBuff[SDBUFF_SIZE], // todo: btRxBuff too long
        newDirFlag, dirLen, calibRamFlag;
uint16_t sdBuffLen, blockLen, fileNum, dirCounter, blinkCnt10, blinkCnt20,
        blinkCnt50;
uint64_t firstTs, fileLastHour, fileLastMin;
volatile uint8_t currentSampleTsTicks[4];

volatile uint8_t fileBad, fileBadCnt;
static FRESULT ff_result;

/*battery evaluation vars*/
uint8_t battVal[3];
uint8_t battWait;
uint32_t battLastTs64;
uint8_t setUndock, onUserButton, onSingleTouch, onDefault,
        preSampleBmpPress, bmpPressFreq, bmpPressCount, sampleBmpTemp,
        sampleBmpTempFreq, preSampleMpuMag, mpuMagFreq, mpuMagCount,
        waitpress, initializing, sampleTimerStatus, blinkStatus;
float clockFreq;
uint16_t clk_30, clk_45, clk_55, clk_64, clk_75, clk_133, clk_90, clk_120,
        clk_135, clk_225, clk_255, clk_432, clk_1000, clk_2500,
        clk_90_45, clk_90_64, clk_90_75,
        clk_133_90, clk_135_90, clk_225_90, clk_255_90, clk_432_90;

uint8_t bmpTempCurrentVal[BMPX80_TEMP_BUFF_SIZE],
        bmpPresCurrentVal[BMPX80_PRESS_BUFF_SIZE], bmpVal[BMPX80_PACKET_SIZE];
uint8_t all0xff[7U], all0x00[7U];

uint16_t bmpTempInterval, bmpPresInterval;
uint64_t bmpTempStartTs, bmpPresStartTs;
uint64_t bmpTempSampleTs, bmpPresSampleTs;
uint64_t startSensingTs64;

volatile uint32_t taskList;
uint32_t taskCurrent;

/*GSR*/
uint8_t gsrActiveResistor;
uint16_t lastGsrVal;
//ExG
uint8_t exgLength, exgChip, exgStartAddr; /*, exgForcedOff;*/
/*Daughter card EEPROM*/
uint8_t dcMemLength;
uint16_t dcMemOffset;
/*BT baud rate*/
uint8_t changeBtBaudRate, sdInfoSyncDelayed;

uint8_t undockSimulate;

uint8_t streamDataInProc;
char *dierecord;

/* variables used for delayed undock-start */
uint8_t undockEvent, wr2sd;
uint64_t time_newUnDockEvent;

/* Variable for SR47-4 (and later) to indicate ADS clock lines are tied */
uint8_t adsClockTied;

/*variables for ICM20948 Accel/Gyro*/
uint8_t icm20948AccelGyroBuf[12]= {0};
bool isIcm20948AccelEn = FALSE;
bool isIcm20948GyroEn = FALSE;

/* approx. 10% cutoff voltage - 3.65 Volts */
#define BATT_CUTOFF_3_65VOLTS   (2500)
#define TIMEOUT_100_MS          (3277)

/* should be 0 */
#define PRESS2UNDOCK        0
#define TESTDOCK            0
#define RTC_OFF             0
#define PRES_TS_EN          0
#define IS_SUPPORTED_TCXO   0
/* should be 1 */
#define TS_BYTE3            1
#define SKIP65MS            1

// bluetooth variables
uint8_t rnx_radio_eeprom[CAT24C16_PAGE_SIZE];

#if !BT_DMA_USED_FOR_RX
bool areNewBytesInBtRxBuf = FALSE;
#endif

boot_stage_t bootStage;

void main(void)
{
    shimmerStatus.initialising = 1; /* led flag, in initialisation period */
    Init();

    if (!shimmerStatus.configuring && !wr2sd)
    {
        SetupDock();
    }
    ReadBatt();

    /* Initialise Watchdog status timer */
    ChargeStatusTimerStart();

    while (1)
    {
#if BT_DMA_USED_FOR_RX
        if (!taskList)
            __bis_SR_register(LPM3_bits + GIE); /* ACLK remains active */
#else
        /* Check BT RX bytes to see if there's something new that we need to try and parse */
        areNewBytesInBtRxBuf = areUnprocessedBytesInBtRxBuff();

        if (!taskList && !(areNewBytesInBtRxBuf || hasBtRxTimeoutOccurred()))
        {
            __bis_SR_register(LPM3_bits + GIE);   //ACLK remains active

            /* need to check again after the MSP has woken up */
            areNewBytesInBtRxBuf = areUnprocessedBytesInBtRxBuff();
        }

        if(areNewBytesInBtRxBuf)
        {
//            setIsBtClearToSend(0);
            processBtUartBuf();
//            setIsBtClearToSend(1);
        }
        else if(hasBtRxTimeoutOccurred())
        {
            handleBtRxTimeout();
        }
#endif

        if (taskList & TASK_SETUP_DOCK)
        {
            TaskClear(TASK_SETUP_DOCK);
            if (!shimmerStatus.configuring && !wr2sd
                    && ((P2IN & BIT3)
                            || ((RTC_get64() - time_newUnDockEvent)
                                    > TIMEOUT_100_MS)))
            {
                if (shimmerStatus.docked != ((P2IN & BIT3) >> 3))
                {
                    shimmerStatus.docked = ((P2IN & BIT3) >> 3);
                    SetupDock();
                }

                if (shimmerStatus.docked != ((P2IN & BIT3) >> 3))
                {
                    TaskSet(TASK_SETUP_DOCK);
                }
                undockEvent = 0;
            }
            else
            {
                TaskSet(TASK_SETUP_DOCK);
            }

            RwcCheck();
            sdErrorFlash =
                    (storedConfig[NV_SD_TRIAL_CONFIG0] & SDH_SDERROR_EN) ?
                            1 : 0;
        }

        //TODO fit into task list?
#if BT_ENABLE_BAUD_RATE_CHANGE
        ChangeBtBaudRateFunc();
#endif

        if (taskList & TASK_BATT_READ)
        {
            TaskClear(TASK_BATT_READ);
            /* use adc channel2 and mem4, read back battery status every certain period */
            if (!shimmerStatus.sensing)
            {
                ReadBatt();
            }
        }

        if (taskList & TASK_DOCK_PROCESS_CMD)
        {
            TaskClear(TASK_DOCK_PROCESS_CMD);
            DockUart_processCmd();
        }
        if (taskList & TASK_DOCK_RESPOND)
        {
            TaskClear(TASK_DOCK_RESPOND);
            DockUart_sendRsp();
        }

        if (taskList & TASK_BT_PROCESS_CMD)
        {
            TaskClear(TASK_BT_PROCESS_CMD);
            ProcessCommand();
        }

        if (taskList & TASK_CFGCH)
        {
            TaskClear(TASK_CFGCH);
            ConfigureChannels();
        }

        if (taskList & TASK_BT_RESPOND)
        {
            TaskClear(TASK_BT_RESPOND);
            SendResponse();
        }

        /* SD Sync - Center */
        if (taskList & TASK_RCCENTERR1)
        {
            TaskClear(TASK_RCCENTERR1);
            SyncCenterR1();
        }
        /* SD Sync - Node */
        if (taskList & TASK_RCNODER10)
        {
            TaskClear(TASK_RCNODER10);
            SyncNodeR10();
        }

        if (taskList & TASK_SAMPLE_MPU9150_MAG)
        {
            TaskClear(TASK_SAMPLE_MPU9150_MAG);
            MPU9150_startMagMeasurement();
        }

        if (taskList & TASK_SAMPLE_BMPX80_PRESS)
        {
            TaskClear(TASK_SAMPLE_BMPX80_PRESS);
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

        if (taskList & TASK_STREAMDATA)
        {
            TaskClear(TASK_STREAMDATA);

            if (streamDataInProc)
            {
                if ((!(shimmerStatus.sdlogCmd && shimmerStatus.sdlogReady))
                        && (!(shimmerStatus.btstreamCmd && shimmerStatus.btstreamReady)))
                {
                    btsdSelfCmd = 1;
                    setStopSensing(1U);
                }
                if (newDirFlag && shimmerStatus.sdlogCmd && shimmerStatus.sdlogReady)
                {
                    StartLogging();
                    //               Timestamp0ToFirstFile();
                }
                StreamData();
                // if sensor data buffer is large enough (about 1024 bytes),
                // write it to SDcard and clear the buffer
                if (shimmerStatus.sdlogCmd && shimmerStatus.sdlogReady)
                {
                    if (sdBuffLen > SDBUFF_SIZE - blockLen)
                    {
                        TaskSet(TASK_WR2SD);
                    }
                }
                if (btsdSelfCmd)
                {
                    BtsdSelfcmd();
                    btsdSelfCmd = 0;
                }
            }
            streamDataInProc = 0;
        }
		
        if(getBtClearTxBufFlag())
        {
            setBtClearTxBufFlag(0);
            clearBtTxBuf(1U);
        }

        if (taskList & TASK_SDLOG_CFG_UPDATE)
        {
            TaskClear(TASK_SDLOG_CFG_UPDATE);
            if (!shimmerStatus.docked && !shimmerStatus.sensing
                    && CheckSdInslot() && GetSdCfgFlag())
            {
                shimmerStatus.configuring = 1;
                IniReadInfoMem();
                UpdateSdConfig();
                SetSdCfgFlag(0);
                shimmerStatus.configuring = 0;
            }
        }

        if (taskList & TASK_STARTSENSING)
        {
            TaskClear(TASK_STARTSENSING);
            shimmerStatus.configuring = 1;
            if (!shimmerStatus.sensing && !batteryStatus.battCritical)
            {
                if (newDirFlag && shimmerStatus.sdlogCmd && shimmerStatus.sdlogReady)
                {
                    StartLogging();
                    //shimmerStatus.isLogging = 1;
                }
                if ((shimmerStatus.sdlogCmd && shimmerStatus.sdlogReady)
                        || (shimmerStatus.btstreamCmd && shimmerStatus.btstreamReady))
                {
                    StartStreaming();
                    if (shimmerStatus.sdSyncEnabled)
                    {
                        maxLenCnt = 0;
                        BtSdSyncStart();
                    }
                }
                streamDataInProc = 0;
                //            if(enableSdlog && shimmerStatus.sdlogReady)
                //               Timestamp0ToFirstFile();
                if (shimmerStatus.docked)
                {
                    shimmerStatus.sdlogCmd = 0;
                }
            }
            shimmerStatus.configuring = 0;
        }

        if (taskList & TASK_WR2SD)
        {
            TaskClear(TASK_WR2SD);
            Write2SD();
        }

        if (taskList & TASK_FACTORY_TEST)
        {
            TaskClear(TASK_FACTORY_TEST);
            run_factory_test();
        }
    }
}

void SetStartSensing()
{
    TaskSet(TASK_SDLOG_CFG_UPDATE);
    TaskSet(TASK_STARTSENSING);
}

void setStopSensing(uint8_t state)
{
    stopSensing = state;
}

void Init(void)
{
    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
//     PM5CTL0 &= ~LOCKLPM5;

    // Need to reset battery critical alarm before LED functions are called:
    batteryInit();

    Board_init();

    setBootStage(BOOT_STAGE_START);

    // Set Vcore to accommodate for max. allowed system speed
    SetVCore(PMMCOREV_3);

    // Start 32.768kHz XTAL as ACLK
    LFXT_Start(XT1DRIVE_0);

    // Start 24MHz XTAL as MCLK and SMCLK
    XT2_Start(XT2DRIVE_2); // XT2DRIVE_2 or XTDRIVE_3 for 24MHz (userguide section 5.4.7)
    UCSCTL4 |= SELS_5 + SELM_5;   // SMCLK=MCLK=XT2

    SFRIFG1 = 0;                  // clear interrupt flag register
    SFRIE1 |= OFIE;               // enable oscillator fault interrupt enable

    taskList = 0;
    TaskSet(TASK_BATT_READ);

    memset(txBuff0, 0, DATA_PACKET_SIZE);
    memset(txBuff1, 0, DATA_PACKET_SIZE);

    firstTsFlag = 0;
    calibRamFlag = 0;
    skip65ms = 0;
    buttonLastReleaseTs64 = 0;
    rwcErrorEn = 0;
    rwcErrorFlash = 0;
    sdErrorFlash = 0;

    undockSimulate = 1;
    sdInfoSyncDelayed = 0;
    lastLedGroup2 = 0;
    btsdSelfCmd = 0;
    battLastTs64 = 0;
    battWait = 0;
    blinkCnt10 = blinkCnt20 = blinkCnt50 = 0;
    fileBad = 0;
    fileBadCnt = 0;
    onDefault = 0;
    onUserButton = 0;
    onSingleTouch = 0;
    setUndock = 0;
    currentBuffer = 0;
    stopSensing = 0;
    setStopSensing(0);
    stopLogging = 0;
    stopStreaming = 0;
    streamDataInProc = 0;

    memset((uint8_t *) &shimmerStatus, 0, sizeof(STATTypeDef));

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
    lastGsrVal = 0;
    setGsrRangePinsAreReversed(0);

    resetBtResponseBools();

    nbrAdcChans = 0;
    nbrDigiChans = 0;
    preSampleMpuMag = 0;
    preSampleBmpPress = 0;
    sampleBmpTemp = 0;

    setBmpInUse(BMP180_IN_USE);
    setWrAccelAndMagInUse(WR_ACCEL_AND_MAG_NONE_IN_USE);
    setGyroInUse(GYRO_NONE_IN_USE);
    setEepromIsPresent(0);

    shimmerStatus.sdBadFile = shimmerStatus.sdInserted = FALSE;

    /* variables used for delayed undock-start */
    wr2sd = 0;
    undockEvent = 0;
    time_newUnDockEvent = 0;

    /* Variable for SR47-4 (and later) to indicate ADS clock lines are tied */
    adsClockTied = 0;

    *(storedConfig + NV_SD_SHIMMER_NAME) = '\0';
    *(storedConfig + NV_SD_EXP_ID_NAME) = '\0';
    *configTimeText = '\0';
    *fileName = '\0';

    memset(all0xff, 0xff, sizeof(all0xff) / sizeof(all0xff[0]));
    memset(all0x00, 0x00, sizeof(all0x00) / sizeof(all0x00[0]));

    /* Globally enable interrupts */
    _enable_interrupts();

    BlinkTimerStart();

    DockUart_resetVariables();
    UCA0_isrInit();
    UART_init(DockUart_rxCallback);

    /* exp power */
    P3OUT &= ~BIT3;      //set low
    P3DIR |= BIT3;       //set as output
    P3SEL &= ~BIT3;

    shimmerStatus.sdInserted = (bool) (!(P4IN & BIT1));

    handleIfDockedStateOnBoot();

    P2IFG &= ~BIT3;         //clear flag
    P2IE |= BIT3;           //enable interrupt

    //    RwcCheck();
    RTC_init(0);

    // enable switch1 interrupt
    Button_init();
    Button_interruptEnable();
    waitpress = 1;

    dierecord = (char*) 0x01A0A;

    setBootStage(BOOT_STAGE_I2C);
    detectI2cSlaves();
    loadBmpCalibration();
    saveBmpCalibrationToSdHeader();

    memset(daughtCardId, 0, CAT24C16_PAGE_SIZE);
    eepromRead(DAUGHT_CARD_ID, CAT24C16_PAGE_SIZE, daughtCardId);

    ProcessHwRevision();
    Board_init_for_revision(isAds1292Present(daughtCardId[DAUGHT_CARD_ID]),
                            isRn4678PresentAndCmdModeSupport(daughtCardId[DAUGHT_CARD_ID],
                                                             daughtCardId[DAUGHT_CARD_REV],
                                                             daughtCardId[DAUGHT_CARD_SPECIAL_REV]));

    /* Used to flash green LED on boot but no longer serves that purpose */
    shimmerStatus.configuring = 1;

    CheckOnDefault();

    ConfigureChannels();
#if TS_BYTE3
    txBuff0[0] = txBuff1[0] = DATA_PACKET; //packet type
#else
    txBuff0[1] = txBuff1[1] = DATA_PACKET; //packet type
#endif

    setBootStage(BOOT_STAGE_BLUETOOTH);
    InitialiseBt();

    setBootStage(BOOT_STAGE_CONFIGURATION);
    /* Calibration needs to be loaded after the chips have been detected in
     * order to know which default calib to set for attached chips.
     * It also needs to be loaded after the BT is initialised so that the
     * MAC ID can be used for default Shimmer name and calibration file names.*/
    loadSensorConfigurationAndCalibration();

    UART_setState(shimmerStatus.docked);

    RwcCheck();
    sdErrorFlash = (storedConfig[NV_SD_TRIAL_CONFIG0] & SDH_SDERROR_EN) ? 1 : 0;

    shimmerStatus.initialising = 0;
    shimmerStatus.configuring = 0;

    checkBtModeConfig();

    setBootStage(BOOT_STAGE_END);
}

void resetBtResponseBools(void)
{
    sendAck = 0;
    inquiryResponse = 0;
    samplingRateResponse = 0;
    lnAccelCalibrationResponse = 0;
    wrAccelRangeResponse = 0;
    magGainResponse = 0;
    magSamplingRateResponse = 0;
    dockedResponse = 0;
    trialConfigResponse = 0;
    centerResponse = 0;
    shimmerNameResponse = 0;
    expIDResponse = 0;
    configTimeResponse = 0;
    dirResponse = 0;
    nshimmerResponse = 0;
    myIDResponse = 0;
    wrAccelSamplingRateResponse = 0;
    wrAccelLpModeResponse = 0;
    wrAccelHrModeResponse = 0;
    gyroRangeResponse = 0;
    bmp180CalibrationCoefficientsResponse = 0;
    bmp280CalibrationCoefficientsResponse = 0;
    gyroSamplingRateResponse = 0;
    altAccelRangeResponse = 0;
    pressureOversamplingRatioResponse = 0;
    internalExpPowerEnableResponse = 0;
    configSetupBytesResponse = 0;
    gyroCalibrationResponse = 0;
    magCalibrationResponse = 0;
    wrAccelCalibrationResponse = 0;
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
    btVerResponse = 0;
    rwcResponse = 0;
    btVbattResponse = 0;
    blinkLedResponse = 0;
    gsrRangeResponse = 0;
    btCommsBaudRateResponse = 0;
    derivedChannelResponse = 0;
    btDataRateTestResponse = 0;
    bmpGenericCalibrationCoefficientsResponse = 0;

    useAckPrefixForInstreamResponses = 1U;

    changeBtBaudRate = BAUD_NO_CHANGE_NEEDED;   //indicates doesn't need changing

#if defined(SHIMMER4_SDK)
    i2cvBattBtRsp = 0;
#endif
}

void handleIfDockedStateOnBoot(void)
{
#if TESTDOCK
    if (0)
    {
#else
    if (P2IN & BIT3)
    {
#endif
        setBatteryInterval(BATT_INTERVAL_DOCKED);
        P2IES |= BIT3;       //look for falling edge
        shimmerStatus.docked = 1;
        shimmerStatus.sdlogReady = 0;
        if (CheckSdInslot())
        {
            DockSdPowerCycle();
        }
    }
    else
    {
        setBatteryInterval(BATT_INTERVAL_UNDOCKED);
        P2IES &= ~BIT3;      //look for rising edge
        shimmerStatus.docked = 0;
        P6OUT |= BIT0;       //   DETECT_N set to high
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

void detectI2cSlaves(void)
{
    i2cSlaveDiscover();

    // Identify the presence of different sensors
    setBmpInUse((i2cSlavePresent(BMP280_ADDR)) ? BMP280_IN_USE : BMP180_IN_USE);

    if(i2cSlavePresent(LSM303AHTR_ACCEL_ADDR))
    {
        setWrAccelAndMagInUse(WR_ACCEL_AND_MAG_LSM303AHTR_IN_USE);
    }
    else if(i2cSlavePresent(LSM303DHLC_ACCEL_ADDR))
    {
        setWrAccelAndMagInUse(WR_ACCEL_AND_MAG_LSM303DLHC_IN_USE);
    }
    else
    {
        setWrAccelAndMagInUse(WR_ACCEL_AND_MAG_NONE_IN_USE);
    }

    setEepromIsPresent(i2cSlavePresent(CAT24C16_ADDR));

    if (i2cSlavePresent(ICM20948_ADDR))
    {
        setGyroInUse(GYRO_ICM20948_IN_USE);
    }
    else if (i2cSlavePresent(MPU9150_ADDR))
    {
        setGyroInUse(GYRO_MPU9X50_IN_USE);
    }
    else
    {
        setGyroInUse(GYRO_NONE_IN_USE);
    }
}

void loadSensorConfigurationAndCalibration(void)
{
    ShimmerCalib_init();
    ShimmerCalibInitFromInfoAll();

    if (!shimmerStatus.docked && CheckSdInslot())
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
                shimmerStatus.sdlogReady = 0;
                shimmerStatus.sdBadFile = TRUE;
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
    }
    else
    {   // sd card not available
        IniReadInfoMem();
        //CalibFromInfoAll();
    }

    ShimmerCalibSyncFromDumpRamAll();
}

void checkBtModeConfig(void)
{
    if (!shimmerStatus.btConnected)
    {
        shimmerStatus.btSupportEnabled =
                (storedConfig[NV_SD_TRIAL_CONFIG0] & SDH_BLUETOOTH_DISABLE) ? 0 : 1;
        // Don't allow sync to be enabled if BT is disabled.
        if (shimmerStatus.btSupportEnabled)
        {
            shimmerStatus.sdSyncEnabled =
                    (storedConfig[NV_SD_TRIAL_CONFIG0] & SDH_TIME_SYNC) ? 1 : 0;
        }
        else
        {
            shimmerStatus.sdSyncEnabled = 0;
        }

        if (shimmerStatus.btSupportEnabled && !shimmerStatus.btPowerOn)
        {
//            BtStart();
            InitialiseBtAfterBoot();
        }
        if (!shimmerStatus.btSupportEnabled && shimmerStatus.btPowerOn)
        {
            BtStop(0);
        }
    }
}

void saveBmpCalibrationToSdHeader(void)
{
    uint8_t *bmpX80CalibPtr = get_bmp_calib_data_bytes();
    memcpy(&sdHeadText[SDH_TEMP_PRES_CALIBRATION], bmpX80CalibPtr,
    BMP180_CALIB_DATA_SIZE);
    if (isBmp280InUse())
    {
        memcpy(&sdHeadText[BMP280_XTRA_CALIB_BYTES],
               bmpX80CalibPtr + BMP180_CALIB_DATA_SIZE,
               BMP280_CALIB_XTRA_BYTES);
    }
}

void ProcessHwRevision(void)
{
    uint8_t srId = daughtCardId[DAUGHT_CARD_ID];
    uint8_t srRevMajor = daughtCardId[DAUGHT_CARD_REV];
    uint8_t srRevMinor = daughtCardId[DAUGHT_CARD_SPECIAL_REV];

    setDaugherCardIdPage(daughtCardId);
    parseDaughterCardId(srId);

    if (isEepromIsPresent())
    {
        // Some board batches don't have the LSM303AHTR placed, in these
        // cases, the ICM-20948's channels are used instead
        if (isSubstitutionNeededForWrAccel(srId, srRevMajor, srRevMinor))
        {
            setWrAccelAndMagInUse(WR_ACCEL_AND_MAG_ICM20948_IN_USE);
        }
        else
        {
            // If the hardware is any board other then the above, overwrite the
            // special rev with 171 (used as a failsafe) to let Consensys know
            // what sensors are on-board
            if (are2ndGenSensorsPresentAndUnknownBoard(srId))
            {
                daughtCardId[DAUGHT_CARD_SPECIAL_REV] = 171;
            }
        }

        if(areGsrControlsPinsReversed(srId, srRevMajor, srRevMinor))
        {
            setGsrRangePinsAreReversed(1);
        }
    }
    else
    {
        // The EEPROM is not present on the SR31-6-0 so the firmware needs mock
        // the version number so that Consensys knows which sensors are on-board
        if (are2ndGenImuSensorsPresent())
        {
            daughtCardId[DAUGHT_CARD_ID] = SHIMMER3_IMU;
            daughtCardId[DAUGHT_CARD_REV] = 6;
            daughtCardId[DAUGHT_CARD_SPECIAL_REV] = 0;
        }
    }
}

void InitialiseBt(void)
{
    // This is the start of all BT initialisation
    /* The RN4678 operational mode need to be set before BT_init() is called so
     * that the pins are set correctly prior to communication with the module */
    if (isRn4678PresentAndCmdModeSupport(daughtCardId[DAUGHT_CARD_ID],
                                   daughtCardId[DAUGHT_CARD_REV],
                                   daughtCardId[DAUGHT_CARD_SPECIAL_REV]))
    {
        setRn4678OperationalMode(RN4678_OP_MODE_APPLICATION);
    }
    else
    {
        setRn4678OperationalMode(RN4678_OP_MODE_NOT_USED);
    }

    btCommsProtocolInit(setTaskNewBtCmdToProcess, HandleBtRfCommStateChange, setMacId, &gAction, &args[0]);
    sdSyncInit(InitialiseBtAfterBoot, BtStop, TaskSet);
    BT_init();
    BT_rn4xDisableRemoteConfig(1);
    BT_setRadioMode(SLAVE_MODE);  // slave mode for center&node
    BT_setGetMacAddress(1);
    BT_setGetVersion(1);

#if BT_ENABLE_BLE_FOR_LOGANDSTREAM_AND_RN4678
    /* BLE isn't compatible with the standard "1234" passkey that Shimmer3 has
     * always used for Classic Bluetooth so we're just disabling the passkey
     * altogether here */
    BT_setAuthentication(2U);
    setBleDeviceInformation(getDaughtCardIdStrPtr(), FW_VER_MAJOR, FW_VER_MINOR, FW_VER_REL);
#endif

#if !BT_ENABLE_BAUD_RATE_CHANGE
    BT_setUpdateBaudDuringBoot(1);
#endif

//    BT_useSpecificAdvertisingName(1U);
    /* This used to be used to trigger reset of the Bluetooth advertising name
     * and pin code but is no longer needed due to BT driver updates. Leaving
     * the remaining code in here just to reset the infomem bit but it isn't
     * necessary any more. */
    InfoMem_read((uint8_t *) 0, storedConfig, NV_NUM_RWMEM_BYTES);
    if (storedConfig[NV_BT_SET_PIN] == 0xAA)
    {
        storedConfig[NV_BT_SET_PIN] = 0xAB;
        InfoMem_write((uint8_t *) 0, storedConfig, NV_NUM_RWMEM_BYTES);
    }

    /* Read previous baud rate from the EEPROM if it is present */
    uint8_t initialBaudRate = BAUD_115200;
    memset(rnx_radio_eeprom, 0xFF, sizeof(rnx_radio_eeprom) / sizeof(rnx_radio_eeprom[0]));
    if (isEepromIsPresent())
    {
        // Read Bluetooth configuration parameters from EEPROM
        /* Variable to help initialise BT radio RN42/4678 type information to EEPROM */
        eepromRead(RNX_TYPE_EEPROM_ADDRESS, CAT24C16_PAGE_SIZE, rnx_radio_eeprom);

        if (rnx_radio_eeprom[RN4678_BAUD_RATE_IDX] <= BAUD_1000000)
        {
            initialBaudRate = rnx_radio_eeprom[RN4678_BAUD_RATE_IDX];
        }
    }

#if BT_DMA_USED_FOR_RX
    uint8_t reset_cnt = 50U; // 50 * 100ms = 5s per baud rate attempt
#else
    uint16_t reset_cnt = 500U; // 500 * 10ms = 5s per baud rate attempt
#endif
    uint8_t failCount = 0U;
    uint8_t baudIndex = 0;
    uint8_t baudsTried[BAUD_1000000+1U] = {0};

    /* Try the inital baud rate first */
    baudsTried[initialBaudRate] = 1U;
    setBtBaudRateToUse(initialBaudRate);
    BtStart();

    /* Try the baud that's stored in the EEPROM firstly, if that fails try
     * 115200, 1000000 or 460800 and then all other bauds. If they all fail, soft-reset */
    while (!shimmerStatus.btPowerOn)
    {
#if BT_DMA_USED_FOR_RX
        _delay_cycles(2400000); // 100ms
#else
        _delay_cycles(240000); // 10ms

        areNewBytesInBtRxBuf = areUnprocessedBytesInBtRxBuff();
        if(areNewBytesInBtRxBuf)
        {
//            setIsBtClearToSend(0);
            processBtUartBuf();
//            setIsBtClearToSend(1);
        }
#endif

        if (!(reset_cnt--))
        {
            failCount++;

            if(failCount==sizeof(baudsTried))
            {
//                // software POR reset
//                PMMCTL0 = PMMPW + PMMSWPOR + (PMMCTL0 & 0x0003);

                setBootStage(BOOT_STAGE_BLUETOOTH_FAILURE);
                while(1)
                {
                    __bis_SR_register(LPM3_bits + GIE); /* ACLK remains active */
                }
            }

            /* Baud rate is likely 115200, 1000000 or 460800 so try them first */
            if(baudsTried[BAUD_115200]!=1U)
            {
                initialBaudRate = BAUD_115200;
            }
            else if(baudsTried[BAUD_460800]!=1U)
            {
                initialBaudRate = BAUD_460800;
            }
            else if(baudsTried[BAUD_1000000]!=1U)
            {
                initialBaudRate = BAUD_1000000;
            }
            else
            {
                for (baudIndex = 0; baudIndex < sizeof(baudsTried); baudIndex++)
                {
                    if(baudsTried[baudIndex]!=1U)
                    {
                        initialBaudRate = baudIndex;
                        break;
                    }
                }
            }

            BT_rst_MessageProgress();
            clearBtTxBuf(1U);

            baudsTried[initialBaudRate] = 1U;
            setBtBaudRateToUse(initialBaudRate);
            BtStart();

            reset_cnt = 50U;
        }
    }

    if (isEepromIsPresent()
            && (rnx_radio_eeprom[RNX_RADIO_TYPE_IDX] != getBtHwVersion()
            || rnx_radio_eeprom[RN4678_BAUD_RATE_IDX] == 0xFF
            || (isBtDeviceRn4678() && rnx_radio_eeprom[RN4678_BAUD_RATE_IDX] != getCurrentBtBaudRate())
            || (isBtDeviceRn41orRN42() && rnx_radio_eeprom[RN4678_BAUD_RATE_IDX] != BAUD_115200)))
    {
        updateBtDetailsInEeprom();
    }

    if (storedConfig[NV_BT_COMMS_BAUD_RATE] != getCurrentBtBaudRate())
    {
#if BT_ENABLE_BAUD_RATE_CHANGE
        SetBtBaudRate(storedConfig[NV_BT_COMMS_BAUD_RATE]);
#else
        storedConfig[NV_BT_COMMS_BAUD_RATE] = getCurrentBtBaudRate();
        InfoMem_write((void*) NV_BT_COMMS_BAUD_RATE,
                      &storedConfig[NV_BT_COMMS_BAUD_RATE], 1);

        TaskSet(TASK_SDLOG_CFG_UPDATE);
#endif
    }
}

void InitialiseBtAfterBoot(void)
{
    BT_init();
    BT_rn4xDisableRemoteConfig(1);
    BT_setUpdateBaudDuringBoot(1);
    BtStart();
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
    ff_result = f_open(&dataFile, (char*) fileName, FA_WRITE | FA_CREATE_NEW);
    fileLastHour = fileLastMin = RTC_get64();
}

void StartStreaming(void)
{
    uint8_t i2cEn = 0;
    uint8_t ICMsampleRateDiv = 0;

    if (!shimmerStatus.sensing)
    {
#if SKIP65MS
        skip65ms = 1;
#endif
        if (shimmerStatus.docked)
        {
            UART_deactivate();
        }

        SamplingClkAssignment(&storedConfig[0]);

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
            P8DIR |= BIT6;      //set as output
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

        if ((storedConfig[NV_SENSORS0] & SENSOR_MPU9X50_ICM20948_GYRO)
                || (storedConfig[NV_SENSORS2] & SENSOR_MPU9X50_ICM20948_ACCEL)
                || (storedConfig[NV_SENSORS2] & SENSOR_MPU9X50_ICM20948_MAG)
                || (isWrAccelInUseIcm20948()
                        && ((storedConfig[NV_SENSORS1] & SENSOR_LSM303XXXX_ACCEL)
                        || (storedConfig[NV_SENSORS0] & SENSOR_LSM303XXXX_MAG))))
        {
            if (isGyroInUseIcm20948())
            {
                ICM20948_init();
                ICM20948_wake(1);
                volatile uint8_t icm_id = ICM20948_getId();    // should be 0xEA
                volatile uint8_t mag_id = ICM20948_getMagId(); // should be 0x09
            }
            else if (isGyroInUseMpu9x50())
            {
                MPU9150_init();
                MPU9150_wake(1);
                volatile uint8_t mpu_id = MPU9150_getId();
            }
            i2cEn = 1;
            if ((storedConfig[NV_SENSORS0] & SENSOR_MPU9X50_ICM20948_GYRO)
                    || (storedConfig[NV_SENSORS2] & SENSOR_MPU9X50_ICM20948_ACCEL)
                    || (isWrAccelInUseIcm20948()
                            && (storedConfig[NV_SENSORS1] & SENSOR_LSM303XXXX_ACCEL)))
            {
                if (isGyroInUseIcm20948())
                {
                    ICMsampleRateDiv = ICM20948_convertSampleRateDivFromMPU9X50(
                            storedConfig[NV_CONFIG_SETUP_BYTE1],
                            (storedConfig[NV_CONFIG_SETUP_BYTE4] & 0x38));

                    ICM20948_setGyroSamplingRate(ICMsampleRateDiv);
                }
                else if (isGyroInUseMpu9x50())
                {
                    MPU9150_setSamplingRate(
                            storedConfig[NV_CONFIG_SETUP_BYTE1]);
                }
                if (storedConfig[NV_SENSORS0] & SENSOR_MPU9X50_ICM20948_GYRO)
                {
                    if (isGyroInUseIcm20948())
                    {
                        ICM20948_setGyroSensitivity(
                                storedConfig[NV_CONFIG_SETUP_BYTE2] & 0x03);
                    }
                    else if (isGyroInUseMpu9x50())
                    {
                        MPU9150_setGyroSensitivity(
                                storedConfig[NV_CONFIG_SETUP_BYTE2] & 0x03); //This needs to go after the wake?
                    }
                }
                if ((storedConfig[NV_SENSORS2] & SENSOR_MPU9X50_ICM20948_ACCEL)
                        || (isWrAccelInUseIcm20948() && (storedConfig[NV_SENSORS1] & SENSOR_LSM303XXXX_ACCEL)))
                {
                    uint8_t wrAccelRange = (storedConfig[NV_CONFIG_SETUP_BYTE3] & 0xC0) >> 6;
                    if (isGyroInUseIcm20948())
                    {
                        if (isWrAccelInUseIcm20948())
                        {
                            // Use setting design for WR accel - re-map to suit the ICM-20948
                            wrAccelRange = (storedConfig[NV_CONFIG_SETUP_BYTE0] & 0x0C) >> 2;
                            switch (wrAccelRange)
                            {
                            case 2:
                                wrAccelRange = 1; // +-4g
                                break;
                            case 3:
                                wrAccelRange = 2; // +-8g
                                break;
                            case 1:
                                wrAccelRange = 3; // +-16g
                                break;
                            default:
                                break;
                            }
                        }
                        ICM20948_setAccelRange(wrAccelRange);
                    }
                    else if (isGyroInUseMpu9x50())
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
                if (isGyroInUseIcm20948())
                {
                    ICM20948_wake(0);
                }
                else if (isGyroInUseMpu9x50())
                {
                    MPU9150_wake(0);
                }
            }
            if ((storedConfig[NV_SENSORS2] & SENSOR_MPU9X50_ICM20948_MAG)
                || (isWrAccelInUseIcm20948() && (storedConfig[NV_SENSORS0] & SENSOR_LSM303XXXX_MAG)))
            {
                uint16_t samplingRateTicks = *(uint16_t*) (storedConfig + NV_SAMPLING_RATE);
                if (isGyroInUseIcm20948())
                {
                    ICM20948_setMagSamplingRateFromShimmerRate(samplingRateTicks);
                }
                else if (isGyroInUseMpu9x50())
                {
                    if (samplingRateTicks >= clk_120)
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
                        mpuMagCount = mpuMagFreq = (clk_90
                                / *(uint16_t*) (storedConfig + NV_SAMPLING_RATE))
                                + 1;
                    }
                }
            }
        }

        if (!isWrAccelInUseIcm20948()
                && ((storedConfig[NV_SENSORS0] & SENSOR_LSM303XXXX_MAG)
                || (storedConfig[NV_SENSORS1] & SENSOR_LSM303XXXX_ACCEL)))
        {
            if (!i2cEn)
            {
                //Initialise I2C
                if (isWrAccelInUseLsm303dlhc())
                {
                    LSM303DLHC_init();
                }
                else
                {
                    LSM303AHTR_init();
                }
                i2cEn = 1;
            }
            if (storedConfig[NV_SENSORS1] & SENSOR_LSM303XXXX_ACCEL)
            {
                if (isWrAccelInUseLsm303dlhc())
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
            if (storedConfig[NV_SENSORS0] & SENSOR_LSM303XXXX_MAG)
            {
                if (isWrAccelInUseLsm303dlhc())
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
            uint16_t bmpX80SamplingTimeTicks = getBmpX80SamplingTimeInTicks();

            if (!i2cEn)
            {
                BMPX80_init();
            }
            if ((*(uint16_t*) (storedConfig + NV_SAMPLING_RATE) >= (clk_30 + bmpX80SamplingTimeTicks)))
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
                bmpPressCount = bmpPressFreq = (bmpX80SamplingTimeTicks
                        / *(uint16_t*) (storedConfig + NV_SAMPLING_RATE))
                        + 1;
            }
            //only need to sample temp once a second at most
            if (*(uint16_t*) (storedConfig + NV_SAMPLING_RATE) >= clk_2500)
            {
                //less than 4Hz
                //so every second sample must be temp
                sampleBmpTemp = sampleBmpTempFreq = 1;
            }
            else
            {
                sampleBmpTemp = sampleBmpTempFreq = (uint8_t) ((FreqDiv(
                        *(uint16_t*) (storedConfig + NV_SAMPLING_RATE)) - 1)
                        / bmpPressFreq);
            }
#if PRES_TS_EN
            bmpPresInterval = bmpX80SamplingTimeTicks;
            // TODO Set as 5.5ms here for BMP280 but datasheet recommends to set same as pressure channel
            bmpTempInterval = (isBmp180InUse() ? clk_45 : clk_55);
#endif
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
            __delay_cycles(2400000); /* 100ms (assuming 24MHz clock) */

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

            if ((daughtCardId[DAUGHT_CARD_ID] == EXP_BRD_EXG_UNIFIED)
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
        if (shimmerStatus.sdSyncEnabled)
        {
            PrepareSDBuffHead();
        }
        shimmerStatus.sensing = 1;
    }
}

inline void StopSensing(void)
{
    /*shut everything down*/
    shimmerStatus.configuring = 1;
    shimmerStatus.sensing = 0;
    shimmerStatus.btStreaming = 0;
    shimmerStatus.btstreamCmd = 0;
    shimmerStatus.sdlogCmd = 0;
    if (shimmerStatus.docked)
    { /* if docked, cannot write to SD card any more*/
        DockSdPowerCycle();
    }
    else
    {
        newDirFlag = 1;
        if (shimmerStatus.sdLogging)
        {
            Write2SD();
            f_close(&dataFile);
            _delay_cycles(1200000);
            SdPowerOff();
        }
    }
    shimmerStatus.sdLogging = 0;

    SampleTimerStop();
    ADC_disable();
    DMA0_disable();

    P8OUT &= ~BIT6;
    //P8REN |= BIT6;      //enable pull down resistor
    //P8DIR &= ~BIT6;     //SW_ACCEL set as input

    P3OUT &= ~BIT3;        //set EXP_RESET_N low
    P2OUT &= ~BIT0;        //set GPIO_INTERNAL1 low (strain)

    if ((storedConfig[NV_SENSORS0] & SENSOR_MPU9X50_ICM20948_GYRO)
            || (storedConfig[NV_SENSORS2] & SENSOR_MPU9X50_ICM20948_ACCEL)
            || (storedConfig[NV_SENSORS2] & SENSOR_MPU9X50_ICM20948_MAG)
            || (isWrAccelInUseIcm20948()
                    && ((storedConfig[NV_SENSORS1] & SENSOR_LSM303XXXX_ACCEL)
                        || (storedConfig[NV_SENSORS0] & SENSOR_LSM303XXXX_MAG))))
    {
        if (isGyroInUseIcm20948())
        {
            ICM20948_setMagMode(AK09916_PWR_DOWN);
            ICM20948_wake(0);
        }
        else if (isGyroInUseMpu9x50())
        {
            MPU9150_wake(0);
        }
    }

    if (!isWrAccelInUseIcm20948()
            && ((storedConfig[NV_SENSORS0] & SENSOR_LSM303XXXX_MAG)
            || (storedConfig[NV_SENSORS1] & SENSOR_LSM303XXXX_ACCEL)))
    {
        if (isWrAccelInUseLsm303dlhc())
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
    if (!shimmerStatus.docked
            && ((storedConfig[NV_SENSORS0] & SENSOR_EXG1_24BIT)
                    || (storedConfig[NV_SENSORS0] & SENSOR_EXG2_24BIT)
                    || (storedConfig[NV_SENSORS2] & SENSOR_EXG1_16BIT)
                    || (storedConfig[NV_SENSORS2] & SENSOR_EXG2_16BIT)))
    {
        EXG_powerOff();
    }
    if (shimmerStatus.docked)
    {
        UART_activate();
    }

    _delay_cycles(240000);
    I2C_Disable();
    P8OUT &= ~BIT4;         //set SW_I2C low to power off I2C chips
    TaskClear(TASK_STREAMDATA);
    TaskClear(TASK_WR2SD);
    sdBuffLen = 0;
    maxLenCnt = 0;
#if SKIP65MS
    skip65ms = 0;
#endif
    TaskClear(TASK_SAMPLE_BMPX80_PRESS);
    TaskClear(TASK_SAMPLE_MPU9150_MAG);
    if (sdInfoSyncDelayed)
    {
        SdInfoSync();
    }
    _NOP();
    shimmerStatus.configuring = 0;
}

void BtsdSelfcmd()
{
    if (shimmerStatus.btConnected)
    {
        uint8_t i = 0;
        uint8_t selfcmd[6]; /* max is 6 bytes */

        if (useAckPrefixForInstreamResponses)
        {
            selfcmd[i++] = ACK_COMMAND_PROCESSED;
        }
        selfcmd[i++] = INSTREAM_CMD_RESPONSE;
        selfcmd[i++] = STATUS_RESPONSE;
        selfcmd[i++] = ((shimmerStatus.toggleLedRedCmd & 0x01) << 7)
                + ((shimmerStatus.sdBadFile & 0x01) << 6)
                + ((shimmerStatus.sdInserted & 0x01) << 5)
                + ((shimmerStatus.btStreaming & 0x01) << 4)
                + ((shimmerStatus.sdLogging & 0x01) << 3)
                + (isRwcTimeSet() << 2)
                + ((shimmerStatus.sensing & 0x01) << 1)
                + (shimmerStatus.docked & 0x01);

        uint8_t crcMode = getBtCrcMode();
        if (crcMode != CRC_OFF)
        {
            calculateCrcAndInsert(crcMode, &selfcmd[0], i);
            i += crcMode;  // Ordinal of enum is how many bytes are used
        }

        BT_write(selfcmd, i, SHIMMER_CMD);
    }
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
    if (storedConfig[NV_SENSORS0] & SENSOR_MPU9X50_ICM20948_GYRO)
    {
        *channel_contents_ptr++ = X_MPU9150_GYRO;
        *channel_contents_ptr++ = Y_MPU9150_GYRO;
        *channel_contents_ptr++ = Z_MPU9150_GYRO;
        nbrDigiChans += 3;
    }
    //Digi Accel
    if (storedConfig[NV_SENSORS1] & SENSOR_LSM303XXXX_ACCEL)
    {
        *channel_contents_ptr++ = X_LSM303DLHC_ACCEL;
        *channel_contents_ptr++ = Y_LSM303DLHC_ACCEL;
        *channel_contents_ptr++ = Z_LSM303DLHC_ACCEL;
        nbrDigiChans += 3;
    }
    //Mag
    if (storedConfig[NV_SENSORS0] & SENSOR_LSM303XXXX_MAG)
    {
        *channel_contents_ptr++ = X_LSM303DLHC_MAG;

        if (isWrAccelInUseLsm303dlhc())
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
    if (storedConfig[NV_SENSORS2] & SENSOR_MPU9X50_ICM20948_ACCEL)
    {
        *channel_contents_ptr++ = X_MPU9150_ACCEL;
        *channel_contents_ptr++ = Y_MPU9150_ACCEL;
        *channel_contents_ptr++ = Z_MPU9150_ACCEL;
        nbrDigiChans += 3;
    }
    //Digi Accel
    if (storedConfig[NV_SENSORS2] & SENSOR_MPU9X50_ICM20948_MAG)
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
            DMA0_init(adcStartPtr, (uint16_t*) (txBuff0 + 4), nbrAdcChans);
        }
    }

    isIcm20948AccelEn = FALSE;
    isIcm20948GyroEn = FALSE;
    if(isGyroInUseIcm20948() && storedConfig[NV_SENSORS0] & SENSOR_MPU9X50_ICM20948_GYRO)
    {
        isIcm20948GyroEn = TRUE;
    }
    if((storedConfig[NV_SENSORS2] & SENSOR_MPU9X50_ICM20948_ACCEL)
            || (isWrAccelInUseIcm20948() && (storedConfig[NV_SENSORS1] & SENSOR_LSM303XXXX_ACCEL)))
    {
        isIcm20948AccelEn = TRUE;
    }

    calculateClassicBtTxSampleSetBufferSize(blockLen, (*(uint16_t*) (storedConfig + NV_SAMPLING_RATE)));
}

uint8_t CheckOnDefault()
{
    onSingleTouch = 0;
    onUserButton = 0;
    onDefault = 0;
    if (IS_SUPPORTED_SINGLE_TOUCH && storedConfig[NV_SD_TRIAL_CONFIG0] & SDH_SINGLETOUCH)
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

    //    shimmerStatus.badFile = UpdateSdConfig() ? TRUE : FALSE;

    if (onDefault && shimmerStatus.sdlogReady && !shimmerStatus.sensing && (shimmerStatus.sdBadFile == FALSE))
    {   //state == BTSD_IDLESD
        //startSensing = 1;
        SetStartSensing();
        //        enableSdlog = (shimmerStatus.badFile) ? 0 : 1;
        shimmerStatus.sdlogCmd = 1;
        shimmerStatus.sensing = 1;
        BtsdSelfcmd();
        shimmerStatus.sensing = 0;
        return 1;
    }
    return 0;
}

void HandleBtRfCommStateChange(bool isOpen)
{
    shimmerStatus.btConnected = isOpen;
    BT_rst_MessageProgress();

    updateBtConnectionStatusInterruptDirection();

    if (isOpen)
    { //BT is connected
#if BT_DMA_USED_FOR_RX
        resetBtRxVariablesOnConnect();
#endif

        if (shimmerStatus.sdSyncEnabled)
        {
            shimmerStatus.btstreamReady = 0;
        }
        else
        {
            shimmerStatus.btstreamReady = 1;

#if BT_DMA_USED_FOR_RX
            setDmaWaitingForResponseIfStatusStrDisabled();
#endif
        }

        if (sdHeadText[SDH_TRIAL_CONFIG0] & SDH_IAMMASTER)
        {
            // center sends sync packet and is waiting for response
            if (isBtSdSyncRunning())
            {
#if BT_DMA_USED_FOR_RX
                /* Only need to charge up the DMA if status strings aren't enabled. Otherwise this is handled within the setup/DMA code. */
                setDmaWaitingForResponseIfStatusStrDisabled();
#endif
                SyncCenterT10();
            }
        }
        else
        {
            resetSyncRcNodeR10Cnt();
            // node is waiting for 1 byte ROUTINE_COMMUNICATION(0xE0)
#if BT_DMA_USED_FOR_RX
            /* Only need to charge up the DMA if status strings aren't enabled. Otherwise this is handled within the setup/DMA code. */
            setDmaWaitingForResponseIfStatusStrDisabled();
#endif
        }
    }
    else
    { //BT is disconnected
        shimmerStatus.btstreamReady = 0;
        shimmerStatus.btstreamCmd = 0;

        setBtDataRateTestState(0);

        clearBtTxBuf(0);

        setBtCrcMode(CRC_OFF);
        /* Revert to default state if changed */
        useAckPrefixForInstreamResponses = 1U;

        checkBtModeConfig();
    }

    if (!shimmerStatus.sensing)
    {
        TaskSet(TASK_SDLOG_CFG_UPDATE);
    }
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
    case P1IV_P1IFG0:
        // PIO2_CONNECT interrupt (Pin 19 on RN42)
        // High: when connected
        // Low: otherwise

        // RADIO_STATUS interrupt (P1_5 on RN4678)
        // High: Powered On and not connected
        // Low: Connected to peer device

        if (((P1IN & BIT0) & isBtDeviceRn41orRN42())
                || ((!(P1IN & BIT0)) & isBtDeviceRn4678()))
        {   //BT is connected
            /* BLE relies on this pin to know when the UART service is open/not */
            if(!areBtStatusStringsEnabled() || isRn4678ConnectionBle())
            {
                HandleBtRfCommStateChange(TRUE);
            }
        }
        else
        {   //BT is disconnected
            if(!areBtStatusStringsEnabled() || shimmerStatus.btConnected)
            {
                /* Fail-safe incase the FW has missed the BT DISCONNECT/RFCOM_CLOSE status strings */
                HandleBtRfCommStateChange(FALSE);
            }
        }
        __bic_SR_register_on_exit(LPM3_bits);
        break;

        //BT RTS
    case P1IV_P1IFG3:
        if (isBtModuleOverflowPinHigh())
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
            P1IES &= ~BIT6;   //select rising edge trigger, wait for release
            shimmerStatus.buttonPressed = 1;
            buttonPressTs64 = RTC_get64();

            Board_ledOn(LED_GREEN0);
        }
        else
        { //button released
            P1IES |= BIT6; //select fall edge trigger, wait for press
            shimmerStatus.buttonPressed = 0;
            buttonReleaseTs64 = RTC_get64();
            button_press_release_td64 = buttonReleaseTs64 - buttonPressTs64;
            button_two_release_td64 = buttonReleaseTs64 - buttonLastReleaseTs64;
            if (button_press_release_td64 >= 163840)
            { // long button press: 5s
            }
            else if ((button_two_release_td64 > 16384) && !shimmerStatus.configuring
                    && !shimmerStatus.btConnected)
            { //  && (button_press_release_td64>327)
                buttonLastReleaseTs64 = buttonReleaseTs64;
#if PRESS2UNDOCK
                    if(shimmerStatus.docked)
                    {
                        shimmerStatus.docked = 0;
                    }
                    else
                    {
                        shimmerStatus.docked = 1;
                    }
                    TaskSet(TASK_SETUP_DOCK);
                    if(!shimmerStatus.sensing)
                    __bic_SR_register_on_exit(LPM3_bits);
#else
                if (storedConfig[NV_SD_TRIAL_CONFIG0] & SDH_USER_BUTTON_ENABLE)
                {
                    // toggles sensing and refresh BT timers (for the centre)
                    if (shimmerStatus.sensing)
                    {
                        setStopSensing(1U);
                        btsdSelfCmd = 1;
                    }
                    else
                    {
                        if (shimmerStatus.sdlogReady && (!shimmerStatus.sdBadFile))
                        {
                            //startSensing = 1;
                            SetStartSensing();
                            shimmerStatus.sdlogCmd = 1;
                        }

                        shimmerStatus.sensing = 1;
                        BtsdSelfcmd(); //only send cmd, not starting yet till START_BTSD_CMD received
                        shimmerStatus.sensing = 0;
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
            P2IES |= BIT1;            //look for falling edge
        }
        else
        {
            //card inserted
            P2IES &= ~BIT1;            //look for rising edge
        }
        break;

        //dock_detect_N
    case P2IV_P2IFG3:
        TaskSet(TASK_SETUP_DOCK);
        /* Reset to battery "charging"/"checking" LED indication on docking change */
        resetBatteryChargingStatus();
        if (!undockEvent)
        {
            if (!(P2IN & BIT3)) // undocked
            {
                undockEvent = 1;
                setBattCritical(0);
                time_newUnDockEvent = RTC_get64();
            }
            //see slaa513 for example using multiple time bases on a single timer module
            if (!shimmerStatus.sensing)
                __bic_SR_register_on_exit(LPM3_bits);
        }
        if (P2IN & BIT3)
        {
            P2IES |= BIT3;       //look for falling edge
            shimmerStatus.sdlogReady = 0;
        }
        else
        {
            P2IES &= ~BIT3;      //look for rising edge
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
        // Default case
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

void BlinkTimerStop(void)
{
    blinkStatus = 0;
    TB0CCTL3 &= ~CCIE;
}

void setBootStage(boot_stage_t bootStageNew)
{
    bootStage = bootStageNew;

    switch (bootStage)
    {
    case BOOT_STAGE_START:
        Board_ledOn(LED_ALL);
        break;
    case BOOT_STAGE_I2C:
        Board_ledOff(LED_ALL);
        break;
    case BOOT_STAGE_BLUETOOTH:
        Board_ledOn(LED_ALL);
        break;
    case BOOT_STAGE_BLUETOOTH_FAILURE:
        Board_ledOff(LED_ALL);
        break;
    case BOOT_STAGE_CONFIGURATION:
        Board_ledOn(LED_ALL);
        break;
    case BOOT_STAGE_END:
        Board_ledOff(LED_ALL);
        break;
    default:
        break;
    }
    return;
}

#pragma vector=TIMER0_B1_VECTOR
__interrupt void TIMER0_B1_ISR(void)
{
    switch (__even_in_range(TB0IV, 14))
    {
    case 0:
        break;                           // No interrupt
    case 2:                           // TB0CCR1
        //MPU9150 mag
        TB0CCR1 = GetTB0() + *(uint16_t*) (storedConfig + NV_SAMPLING_RATE);
        if (TaskSet(TASK_SAMPLE_MPU9150_MAG))
            __bic_SR_register_on_exit(LPM3_bits);
        break;
    case 4:            // TB0CCR2
        //Bmp180 press
        TB0CCR2 = GetTB0() + *(uint16_t*) (storedConfig + NV_SAMPLING_RATE);
        if (TaskSet(TASK_SAMPLE_BMPX80_PRESS))
            __bic_SR_register_on_exit(LPM3_bits);
        break;
    case 6:            // TB0CCR3
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

        if (bootStage != BOOT_STAGE_END)
        {
            switch (bootStage)
            {
            case BOOT_STAGE_I2C:
                Board_ledToggle(LED_RED);
                break;
            case BOOT_STAGE_BLUETOOTH_FAILURE:
                Board_ledToggle(LED_YELLOW);
                break;
            default:
                break;
            }
            return;
        }

        /* SDLog handles auto-stop in TIMER0_A1_VECTOR whereas LogAndStream handles it in TIMER0_B1_VECTOR */
        if (blinkCnt20 % 10 == 0)
        {
            if (shimmerStatus.sensing && maxLen)
            {
                if (maxLenCnt < maxLen)
                    maxLenCnt++;
                else
                {
                    setStopSensing(1U);
                    btsdSelfCmd = 1;
                    maxLenCnt = 0;
                }
            }
        }

        uint64_t batt_td, batt_my_local_time_64;
        batt_my_local_time_64 = RTC_get64();
        batt_td = batt_my_local_time_64 - battLastTs64;

        if ((batt_td > getBatteryIntervalTicks()))
        {              //10 mins = 19660800
            if (!shimmerStatus.sensing && TaskSet(TASK_BATT_READ))
                __bic_SR_register_on_exit(LPM3_bits);
            battLastTs64 = batt_my_local_time_64;
        }

        if (!shimmerStatus.initialising)
            if (TaskGet(TASK_BATT_READ))
                __bic_SR_register_on_exit(LPM3_bits);

        if (blinkStatus && !shimmerStatus.initialising)
        {
            // below are settings for green0, yellow and red leds, battery charge status
            if (shimmerStatus.toggleLedRedCmd)
            {
                Board_ledOff(LED_GREEN0 + LED_YELLOW);
                Board_ledOn(LED_RED);
            }
            else if (P2IN & BIT3) // Docked
            {
                Board_ledOff(LED_GREEN0 + LED_YELLOW + LED_RED);
                if (batteryStatus.battStatLedCharging != LED_ALL_OFF)
                {
                    if (batteryStatus.battStatLedFlash)
                    {
                        Board_ledToggle(batteryStatus.battStatLedCharging);
                    }
                    else
                    {
                        Board_ledOn(batteryStatus.battStatLedCharging);
                    }
                }
            }
            else
            {
                Board_ledOff(LED_GREEN0 + LED_YELLOW + LED_RED);
                if (!blinkCnt50)
                {
                    BattBlinkOn();
                }
            }

            // code for keeping LED_GREEN0 on when user button pressed
            if ((!(P1IN & BIT6)))
            {
                Board_ledOn(LED_GREEN0);
            }

            // below are settings for green1, blue, yellow and red leds

            if (!shimmerStatus.docked && (shimmerStatus.sdBadFile || !shimmerStatus.sdInserted) && sdErrorFlash)
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
            if (rwcErrorFlash && (!shimmerStatus.sensing))
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
                if (shimmerStatus.btSupportEnabled
                         && !shimmerStatus.sdSyncEnabled)
                {
                    if (!shimmerStatus.sensing)
                    { //standby or configuring
                        if (shimmerStatus.configuring)
                        { //configuring
                            if (!(P1OUT & BIT1))
                                Board_ledOn(LED_GREEN1);
                            else
                                Board_ledOff(LED_GREEN1);
                        }
                        else if (shimmerStatus.btConnected && !shimmerStatus.configuring)
                        {
                            Board_ledOn(LED_BLUE);
                            Board_ledOff(LED_GREEN1);     // nothing to show
                        }
                        else if (isRn4678ConnectionEstablished())
                        {
                            /* BT connection established but RFComm not open */
                            if (P1OUT & BIT2)
                                Board_ledOff(LED_BLUE);
                            else
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
                        // shimmerStatus.sdlogReady, shimmerStatus.btstreamReady, enableSdlog, enableBtstream
                        // btstream only
                        if ((shimmerStatus.btstreamCmd && shimmerStatus.btstreamReady)
                                && !(shimmerStatus.sdlogReady && shimmerStatus.sdlogCmd))
                        {
                            if (!(blinkCnt20 % 10))
                            {
                                if (!(P1OUT & BIT2))
                                    Board_ledOn(LED_BLUE);
                                else
                                    Board_ledOff(LED_BLUE);
                            }
                            Board_ledOff(LED_GREEN1);     // nothing to show
                        }
                        // sdlog only
                        else if (!(shimmerStatus.btstreamCmd && shimmerStatus.btstreamReady)
                                && (shimmerStatus.sdlogReady && shimmerStatus.sdlogCmd))
                        {
                            if (!(blinkCnt20 % 10))
                            {
                                if (!(P1OUT & BIT1))
                                    Board_ledOn(LED_GREEN1);
                                else
                                    Board_ledOff(LED_GREEN1);
                            }
                            Board_ledOff(LED_BLUE);       // nothing to show
                        }
                        // btstream & sdlog
                        else if ((shimmerStatus.btstreamCmd && shimmerStatus.btstreamReady)
                                && (shimmerStatus.sdlogReady && shimmerStatus.sdlogCmd))
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
                else
                {
                    // good file - green1:
                    if (!shimmerStatus.sensing)
                    {   //standby or configuring
                        if (shimmerStatus.configuring)
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

                    // good file - blue:
                    /* Toggle blue LED while a connection is established */
                    if (shimmerStatus.btPowerOn && shimmerStatus.btConnected)
                    {
                        Board_ledToggle(LED_BLUE);
                    }
                    /* Leave blue LED on solid if it's a node and a sync hasn't occurred yet (first 'outlier' not included) */
                    else if (!getRcFirstOffsetRxed()
                            && shimmerStatus.sensing
                            && shimmerStatus.sdSyncEnabled
                            && !(sdHeadText[SDH_TRIAL_CONFIG0] & SDH_IAMMASTER))
                    {
                        Board_ledOn(LED_BLUE);
                    }
                    else
                    {
                        /* Flash twice if sync is not successfull */
                        if (((blinkCnt20 == 12) || (blinkCnt20 == 14))
                                && !shimmerStatus.docked
                                && shimmerStatus.sensing
                                && shimmerStatus.sdSyncEnabled
                                && ((!getSyncSuccC() && (sdHeadText[SDH_TRIAL_CONFIG0] & SDH_IAMMASTER))
                                        || (!getSyncSuccN() && !(sdHeadText[SDH_TRIAL_CONFIG0] & SDH_IAMMASTER))))
                        {
                            if (getSyncCnt() > 3)
                            {
                                Board_ledOn(LED_BLUE);
                            }
                            //TODO should there be an else here?
                        }
                        else
                        {
                            Board_ledOff(LED_BLUE);
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
#if BT_DMA_USED_FOR_RX
        resetBtRxBuff();
#else
        clearBtRxBuf();
#endif
        BT_start();
    }
}

void BtStop(uint8_t isCalledFromMain)
{
    clearBtTxBuf(isCalledFromMain);

    TaskClear(TASK_RCNODER10);
#if USE_OLD_SD_SYNC_APPROACH
    setRcommVar(0);                   //don't try to get routine comm info
#endif

#if BT_DMA_USED_FOR_RX
    DMA2_disable();                  //dma2 for bt disabled
#endif

    updateBtConnectionStatusInterruptDirection();
    shimmerStatus.btConnected = 0;
    shimmerStatus.btPowerOn = 0;
    BT_disable();
    BT_rst_MessageProgress();

    if (watchDogWasOnDuringBtStart)
    {
        // Reset Watchdog timer
        WDTCTL = WDTPW + WDTSSEL__ACLK + WDTCNTCL + WDTIS_4;
    }
}

// Sample Timer
void SampleTimerStart(void)
{
    uint16_t val_tb0 = GetTB0();
    uint16_t baseClockOffset = val_tb0 + *(uint16_t *) (storedConfig + NV_SAMPLING_RATE);
    uint8_t bmpX80Precision = (storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x30) >> 4;

    if (preSampleMpuMag || preSampleBmpPress)
    {
        if (preSampleBmpPress)
        {
            uint16_t bmpX80SamplingTimeInTicks = getBmpX80SamplingTimeInTicks();
            uint16_t bmpX80SamplingTimeDiffFrom9msInTicks = getBmpX80SamplingTimeDiffFrom9msInTicks();

            // When max BMPX80 sampling time is less than 9ms
            if (bmpX80Precision == 0 || (bmpX80Precision == 1 && isBmp180InUse()))
            {
                if (preSampleMpuMag)
                {
                    TB0CCR0 = baseClockOffset + clk_90;    //9ms
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
            // When max BMPX80 sampling time is greater than 9ms
            else if ((bmpX80Precision == 1 && !isBmp180InUse()) || bmpX80Precision == 2 || bmpX80Precision == 3)
            {
                TB0CCR0 = baseClockOffset + bmpX80SamplingTimeInTicks;
                TB0CCR2 = baseClockOffset;
                TB0CCTL2 = CCIE;

                if (preSampleMpuMag)
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
            TB0CCR0 = baseClockOffset + clk_90;       //9ms
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
    TB0CCR0 = timer_b0 + *(uint16_t*) (storedConfig + NV_SAMPLING_RATE);

    if (!streamDataInProc)
    {
        streamDataInProc = 1;
#if TS_BYTE3
        if (firstTsFlag == 1)
        {
            firstTs = RTC_get64();
            firstTsFlag = 2;
            *(uint32_t*) currentSampleTsTicks = (uint64_t) firstTs;
        }
        else
        {
            *(uint32_t*) currentSampleTsTicks = RTC_get32();
        }

        uint8_t *current_buffer_ptr = currentBuffer ? txBuff1 : txBuff0;
        // The first byte is packet type byte when Bluetooth streaming
        current_buffer_ptr[1] = currentSampleTsTicks[0];
        current_buffer_ptr[2] = currentSampleTsTicks[1];
        current_buffer_ptr[3] = currentSampleTsTicks[2];
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
        TaskSet(TASK_STREAMDATA);
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
        saveBatteryVoltageAndUpdateStatus();
    }
    else
    {
        adc_offset = 4;

        //Destination address for next transfer
        if (currentBuffer)
        {
            DMA0_repeatTransfer(adcStartPtr, (uint16_t*) (txBuff0 + adc_offset),
                                nbrAdcChans);
        }
        else
        {
            DMA0_repeatTransfer(adcStartPtr, (uint16_t*) (txBuff1 + adc_offset),
                                nbrAdcChans);
        }
        ADC_disable(); //can disable ADC until next time sampleTimer fires (to save power)?
        DMA0_disable();
        TaskSet(TASK_STREAMDATA);
    }
    return 1;
}

void setMacId(uint8_t *buf)
{
    bt_setMacId(buf);
    InfoMem_write((uint8_t*) NV_MAC_ADDRESS, getMacIdBytesPtr(), 6);
    memcpy(storedConfig + NV_MAC_ADDRESS, getMacIdBytesPtr(), 6);
}

void ProcessCommand(void)
{
    uint32_t config_time;
    uint8_t my_config_time[4];
    uint8_t name_len;
    uint8_t update_sdconfig = 0, update_calib = 0, calib_sensor = 0,
            calib_range = 0, update_calib_dump_file = 0;
    //uint8_t update_sdconfig_manual = 0;
//    sc_t sc1;
    uint8_t fullSyncResp[SYNC_PACKET_MAX_SIZE] = {0};

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
        shimmerStatus.toggleLedRedCmd ^= 1;
        break;
    case START_STREAMING_COMMAND:
        shimmerStatus.btstreamCmd = 1;
        if (!shimmerStatus.sensing)
        {
            TaskSet(TASK_CFGCH);
            //startSensing = 1;
            SetStartSensing();
        }
        //enableSdlog = 0;
        //newDirFlag = 1;
        //CheckSdInslot();
        break;
    case START_SDBT_COMMAND:
        if (!shimmerStatus.sensing)
        {
            TaskSet(TASK_CFGCH);
            //startSensing = 1;
            SetStartSensing();
        }
        shimmerStatus.btstreamCmd = 1;
        if (shimmerStatus.sdlogReady && (!shimmerStatus.sdBadFile))
        {
            shimmerStatus.sdlogCmd = 1;
        }
        break;
    case START_LOGGING_COMMAND:
        if (shimmerStatus.sdlogReady && (!shimmerStatus.sdBadFile))
        {
            shimmerStatus.sdlogCmd = 1;
            if (!shimmerStatus.sensing)
            {
                //startSensing = 1;
                SetStartSensing();
                TaskSet(TASK_CFGCH);
            }
        }
        break;
    case SET_CRC_COMMAND:
        setBtCrcMode(args[0]);
        break;

    case SET_INSTREAM_RESPONSE_ACK_PREFIX_STATE:
        useAckPrefixForInstreamResponses = args[0];
        break;

    case STOP_STREAMING_COMMAND:
        if (shimmerStatus.btStreaming)
        {
            stopStreaming = 1;
        }
        break;
    case STOP_SDBT_COMMAND:
        if (shimmerStatus.sensing)
        {
            setStopSensing(1U);
            //return;
        }
        break;
    case STOP_LOGGING_COMMAND:
        if (shimmerStatus.sdLogging)
        {
            stopLogging = 1;
            //if(!shimmerStatus.isStreaming);
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
        TaskSet(TASK_CFGCH);
        if (shimmerStatus.sensing)
        {
            setStopSensing(1U);
            //startSensing = 1;
            SetStartSensing();
        }
        break;
    case GET_WR_ACCEL_RANGE_COMMAND:
        wrAccelRangeResponse = 1;
        break;
    case GET_MAG_GAIN_COMMAND:
        magGainResponse = 1;
        break;
    case GET_MAG_SAMPLING_RATE_COMMAND:
        magSamplingRateResponse = 1;
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
#if !IS_SUPPORTED_TCXO
        storedConfig[NV_SD_TRIAL_CONFIG1] &= ~SDH_TCXO; /* Disable TCXO */
#endif
        storedConfig[NV_SD_BT_INTERVAL] = args[2];
        if (IS_SUPPORTED_SINGLE_TOUCH && storedConfig[NV_SD_TRIAL_CONFIG1] & SDH_SINGLETOUCH)
        {
            storedConfig[NV_SD_TRIAL_CONFIG0] |= SDH_USER_BUTTON_ENABLE;
            storedConfig[NV_SD_TRIAL_CONFIG0] |= SDH_TIME_SYNC;
        }
        if (storedConfig[NV_SD_BT_INTERVAL] < SYNC_INT_C)
            storedConfig[NV_SD_BT_INTERVAL] = SYNC_INT_C;
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
    case SET_WR_ACCEL_RANGE_COMMAND:
        if (args[0] < 4)
        {
            storedConfig[NV_CONFIG_SETUP_BYTE0] =
                    (storedConfig[NV_CONFIG_SETUP_BYTE0] & 0xF3)
                            + ((args[0] & 0x03) << 2);
        }
        else
        {
            storedConfig[NV_CONFIG_SETUP_BYTE0] &= 0xF3;
        }
        InfoMem_write((void*) NV_CONFIG_SETUP_BYTE0,
                      &storedConfig[NV_CONFIG_SETUP_BYTE0], 1);
        sdHeadText[SDH_CONFIG_SETUP_BYTE0] =
                storedConfig[NV_CONFIG_SETUP_BYTE0];
        update_sdconfig = 1;
        if (shimmerStatus.sensing)
        {
            setStopSensing(1U);
            //startSensing = 1;
            SetStartSensing();
        }
        break;
    case GET_WR_ACCEL_SAMPLING_RATE_COMMAND:
        wrAccelSamplingRateResponse = 1;
        break;
    case GET_WR_ACCEL_LPMODE_COMMAND:
        wrAccelLpModeResponse = 1;
        break;
    case GET_WR_ACCEL_HRMODE_COMMAND:
        wrAccelHrModeResponse = 1;
        break;
    case GET_GYRO_RANGE_COMMAND:
        gyroRangeResponse = 1;
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
    case GET_PRESSURE_CALIBRATION_COEFFICIENTS_COMMAND:
        bmpGenericCalibrationCoefficientsResponse = 1;
        break;
    case GET_GYRO_SAMPLING_RATE_COMMAND:
        gyroSamplingRateResponse = 1;
        break;
    case GET_ALT_ACCEL_RANGE_COMMAND:
        altAccelRangeResponse = 1;
        break;
    case GET_PRESSURE_OVERSAMPLING_RATIO_COMMAND:
        pressureOversamplingRatioResponse = 1;
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
    case GET_BT_VERSION_STR_COMMAND:
        btVerResponse = 1;
        break;
    case SET_DATA_RATE_TEST:
        /* Stop test before ACK is sent */
        if (args[0] == 0)
        {
            setBtDataRateTestState(0);
            clearBtTxBuf(1);
        }
        btDataRateTestResponse = 1;
        break;
    case SET_FACTORY_TEST:
        if (args[0] < FACTORY_TEST_COUNT)
        {
            setup_factory_test(PRINT_TO_BT_UART, (factory_test_t) args[0]);
            TaskSet(TASK_FACTORY_TEST);
        }
        break;

    case SET_WR_ACCEL_SAMPLING_RATE_COMMAND:
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
        if (shimmerStatus.sensing)
        {
            setStopSensing(1U);
            //startSensing = 1;
            SetStartSensing();
        }
        break;
    case SET_MAG_GAIN_COMMAND:
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
        if (shimmerStatus.sensing)
        {
            setStopSensing(1U);
            //startSensing = 1;
            SetStartSensing();
        }
        break;
    case SET_MAG_SAMPLING_RATE_COMMAND:
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
        if (shimmerStatus.sensing)
        {
            setStopSensing(1U);
            //startSensing = 1;
            SetStartSensing();
        }
        break;
    case SET_WR_ACCEL_LPMODE_COMMAND:
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
        if (shimmerStatus.sensing)
        {
            setStopSensing(1U);
            //startSensing = 1;
            SetStartSensing();
        }
        break;
    case SET_WR_ACCEL_HRMODE_COMMAND:
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
        if (shimmerStatus.sensing)
        {
            setStopSensing(1U);
            //startSensing = 1;
            SetStartSensing();
        }
        break;
    case SET_GYRO_RANGE_COMMAND:
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
        if (shimmerStatus.sensing)
        {
            setStopSensing(1U);
            //startSensing = 1;
            SetStartSensing();
        }
        break;
    case SET_GYRO_SAMPLING_RATE_COMMAND:
        storedConfig[NV_CONFIG_SETUP_BYTE1] = args[0];
        InfoMem_write((void*) NV_CONFIG_SETUP_BYTE1,
                      &storedConfig[NV_CONFIG_SETUP_BYTE1], 1);
        sdHeadText[SDH_CONFIG_SETUP_BYTE1] =
                storedConfig[NV_CONFIG_SETUP_BYTE1];
        update_sdconfig = 1;
        if (shimmerStatus.sensing)
        {
            setStopSensing(1U);
            //startSensing = 1;
            SetStartSensing();
        }
        break;
    case SET_ALT_ACCEL_RANGE_COMMAND:
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
        if (shimmerStatus.sensing)
        {
            setStopSensing(1U);
            //startSensing = 1;
            SetStartSensing();
        }
        break;
    case SET_PRESSURE_OVERSAMPLING_RATIO_COMMAND:
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
        if (shimmerStatus.sensing)
        {
            setStopSensing(1U);
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
        if (shimmerStatus.sensing)
        {
            setStopSensing(1U);
            //startSensing = 1;
            SetStartSensing();
        }
        break;
    case SET_SAMPLING_RATE_COMMAND:
        *(uint16_t*) (storedConfig + NV_SAMPLING_RATE) = *(uint16_t*) args;
        InfoMem_write((void*) NV_SAMPLING_RATE, &storedConfig[NV_SAMPLING_RATE],
                      2);
        sdHeadText[SDH_SAMPLE_RATE_0] = storedConfig[NV_SAMPLING_RATE];
        sdHeadText[SDH_SAMPLE_RATE_1] = storedConfig[NV_SAMPLING_RATE + 1];
        update_sdconfig = 1;
        if (shimmerStatus.sensing)
        {
            //restart sampling timer to use new sampling rate
            setStopSensing(1U);
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
        TaskSet(TASK_SDLOG_CFG_UPDATE);
        //update_sdconfig_manual = 1;
        break;
    case SET_LN_ACCEL_CALIBRATION_COMMAND:
        memcpy(&storedConfig[NV_A_ACCEL_CALIBRATION], args, 21);
        InfoMem_write((void*) NV_A_ACCEL_CALIBRATION,
                      &storedConfig[NV_A_ACCEL_CALIBRATION], 21);
        memcpy(&sdHeadText[SDH_A_ACCEL_CALIBRATION],
               &storedConfig[NV_A_ACCEL_CALIBRATION], 21);

        CalibSaveFromInfoMemToCalibDump(SC_SENSOR_ANALOG_ACCEL);

        update_calib_dump_file = 1;
        //update_calib = 1;
        //calib_sensor = S_ACCEL_A;
        break;
    case GET_LN_ACCEL_CALIBRATION_COMMAND:
        lnAccelCalibrationResponse = 1;
        break;
    case SET_GYRO_CALIBRATION_COMMAND:
        memcpy(&storedConfig[NV_MPU9150_GYRO_CALIBRATION], args, 21);
        InfoMem_write((void*) NV_MPU9150_GYRO_CALIBRATION,
                      &storedConfig[NV_MPU9150_GYRO_CALIBRATION], 21);
        memcpy(&sdHeadText[SDH_MPU9150_GYRO_CALIBRATION],
               &storedConfig[NV_MPU9150_GYRO_CALIBRATION], 21);

        CalibSaveFromInfoMemToCalibDump(SC_SENSOR_MPU9150_GYRO);

        update_calib_dump_file = 1;
        //      update_calib = 1;
        //      calib_sensor = S_GYRO;
        //      calib_range = storedConfig[NV_CONFIG_SETUP_BYTE2] & 0x03;
        break;
    case GET_GYRO_CALIBRATION_COMMAND:
        gyroCalibrationResponse = 1;
        break;
    case SET_MAG_CALIBRATION_COMMAND:
        memcpy(&storedConfig[NV_LSM303DLHC_MAG_CALIBRATION], args, 21);
        InfoMem_write((void*) NV_LSM303DLHC_MAG_CALIBRATION,
                      &storedConfig[NV_LSM303DLHC_MAG_CALIBRATION], 21);
        memcpy(&sdHeadText[SDH_LSM303DLHC_MAG_CALIBRATION],
               &storedConfig[NV_LSM303DLHC_MAG_CALIBRATION], 21);

        CalibSaveFromInfoMemToCalibDump(SC_SENSOR_LSM303DLHC_MAG);

        update_calib_dump_file = 1;
        //      update_calib = 1;
        //      calib_sensor = S_MAG;
        //      calib_range = (storedConfig[NV_CONFIG_SETUP_BYTE2]>>5) & 0x07;
        break;
    case GET_MAG_CALIBRATION_COMMAND:
        magCalibrationResponse = 1;
        break;
    case SET_WR_ACCEL_CALIBRATION_COMMAND:
        memcpy(&storedConfig[NV_LSM303DLHC_ACCEL_CALIBRATION], args, 21);
        InfoMem_write((void*) NV_LSM303DLHC_ACCEL_CALIBRATION,
                      &storedConfig[NV_LSM303DLHC_ACCEL_CALIBRATION], 21);
        memcpy(&sdHeadText[SDH_LSM303DLHC_ACCEL_CALIBRATION],
               &storedConfig[NV_LSM303DLHC_ACCEL_CALIBRATION], 21);

        CalibSaveFromInfoMemToCalibDump(SC_SENSOR_LSM303DLHC_ACCEL);

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
        if (shimmerStatus.sensing)
        {
            setStopSensing(1U);
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

                if ((daughtCardId[DAUGHT_CARD_ID] == EXP_BRD_EXG_UNIFIED)
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
        TaskSet(TASK_CFGCH);
        if (shimmerStatus.sensing)
        {
            setStopSensing(1U);
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
    case GET_WR_ACCEL_CALIBRATION_COMMAND:
        wrAccelCalibrationResponse = 1;
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
        dcMemLength = args[0];
        dcMemOffset = args[1];
        if ((dcMemLength <= 16) && (dcMemOffset <= 15)
                && (dcMemLength + dcMemOffset <= 16))
        {
            eepromReadWrite(dcMemOffset, dcMemLength, args + 2U, EEPROM_WRITE);
        }
        break;
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
            eepromWrite(dcMemOffset + 16U, (uint16_t) dcMemLength, args + 3U);
        }
        break;
    case GET_BT_COMMS_BAUD_RATE:
        btCommsBaudRateResponse = 1;
        break;
    case SET_BT_COMMS_BAUD_RATE:
#if BT_ENABLE_BAUD_RATE_CHANGE
        if (args[0] != storedConfig[NV_BT_COMMS_BAUD_RATE])
        {
            if (args[0] <= BAUD_1000000)
            {
                changeBtBaudRate = args[0];
            }
            else
            {
                changeBtBaudRate = DEFAULT_BT_BAUD_RATE;
            }
        }
#endif
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
        // Update shimmerStatus.badFile status bit after bt config
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
                InfoMem_read((uint8_t*) NV_MAC_ADDRESS, getMacIdBytesPtr(), 6);
            }
            if (infomemOffset == (INFOMEM_SEG_D_ADDR - INFOMEM_OFFSET))
            {
                /* Check if unit is SR47-4 or greater.
                 * If so, amend configuration byte 2 of ADS chip 1 to have bit 3 set to 1.
                 * This ensures clock lines on ADS chip are correct
                 */
                if ((daughtCardId[DAUGHT_CARD_ID] == EXP_BRD_EXG_UNIFIED)
                        && (daughtCardId[DAUGHT_CARD_REV] >= 4))
                {
                    *(args + 3 + NV_EXG_ADS1292R_1_CONFIG2) |= 8;
                }
            }
#if !IS_SUPPORTED_TCXO
            if (infomemOffset <= NV_SD_TRIAL_CONFIG1 && NV_SD_TRIAL_CONFIG1 <= infomemOffset + infomemLength)
            {
                uint8_t tcxoInfomemOffset = NV_SD_TRIAL_CONFIG1 - infomemOffset;
                args[3 + tcxoInfomemOffset] &= ~SDH_TCXO;
            }
#endif

            InfoMem_write((void*) infomemOffset, args + 3, infomemLength);

            if (infomemOffset == (INFOMEM_SEG_C_ADDR - INFOMEM_OFFSET))
            {
                /* Re-write MAC address to Infomem */
                InfoMem_write((uint8_t*) NV_MAC_ADDRESS, getMacIdBytesPtr(), 6);
            }

            InfoMem_read((uint8_t*) infomemOffset, storedConfig + infomemOffset,
                         infomemLength);


            if (infomemOffset == (INFOMEM_SEG_D_ADDR - INFOMEM_OFFSET))
            {
                CalibSaveFromInfoMemToCalibDump(0xFF);
            }

            Config2SdHead();
            Infomem2Names();
            TaskSet(TASK_CFGCH);
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
        setRwcTime(&args[0]);
        RwcCheck();
        storedConfig[NV_SD_TRIAL_CONFIG0] |= SDH_RTC_SET_BY_BT;
        InfoMem_write((uint8_t*) NV_SD_TRIAL_CONFIG0,
                      &storedConfig[NV_SD_TRIAL_CONFIG0], 1);
        sdHeadText[SDH_TRIAL_CONFIG0] = storedConfig[NV_SD_TRIAL_CONFIG0];
        uint64_t * rwcTimeDiffPtr = getRwcTimeDiffPtr();
        sdHeadText[SDH_RTC_DIFF_7] = *((uint8_t*) rwcTimeDiffPtr);
        sdHeadText[SDH_RTC_DIFF_6] = *(((uint8_t*) rwcTimeDiffPtr) + 1);
        sdHeadText[SDH_RTC_DIFF_5] = *(((uint8_t*) rwcTimeDiffPtr) + 2);
        sdHeadText[SDH_RTC_DIFF_4] = *(((uint8_t*) rwcTimeDiffPtr) + 3);
        sdHeadText[SDH_RTC_DIFF_3] = *(((uint8_t*) rwcTimeDiffPtr) + 4);
        sdHeadText[SDH_RTC_DIFF_2] = *(((uint8_t*) rwcTimeDiffPtr) + 5);
        sdHeadText[SDH_RTC_DIFF_1] = *(((uint8_t*) rwcTimeDiffPtr) + 6);
        sdHeadText[SDH_RTC_DIFF_0] = *(((uint8_t*) rwcTimeDiffPtr) + 7);
        break;
#if USE_OLD_SD_SYNC_APPROACH
    case ACK_COMMAND_PROCESSED:
#else
    case SET_SD_SYNC_COMMAND:
#endif
        /* Reassemble full packet so that original RcNodeR10() will work without modificiation */
        fullSyncResp[0] = gAction;
        memcpy(&fullSyncResp[1], &args[0], SYNC_PACKET_MAX_SIZE-SYNC_PACKET_SIZE_CMD);
        setSyncResp(&fullSyncResp[0], SYNC_PACKET_MAX_SIZE);
        TaskSet(TASK_RCNODER10);
#if !USE_OLD_SD_SYNC_APPROACH
    case ACK_COMMAND_PROCESSED:
        /* Slave response received by Master */
        if (args[0] == SD_SYNC_RESPONSE)
        {
            /* SD Sync Center - get's into this case when the center is waiting for a 0x01 or 0xFF from a node */
            setSyncResp(&args[1], 1U);
            TaskSet(TASK_RCCENTERR1);
        }
        break;
#endif
    default:
        ;
    }

	/* Send ACK back for all commands except when FW has received an ACK */
    if (gAction != ACK_COMMAND_PROCESSED
#if USE_OLD_SD_SYNC_APPROACH
            )
#else
            /* ACK is sent back as part of SD_SYNC_RESPONSE so no need to send it here */
            && gAction != SET_SD_SYNC_COMMAND)
#endif
    {
        sendAck = 1;
        TaskSet(TASK_BT_RESPOND);
    }

    //   if(update_sdconfig_manual && CheckSdInslot()){
    //      if(!shimmerStatus.isDocked){
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
    //TODO update_calib is not used any more
    if (update_calib && CheckSdInslot() && !shimmerStatus.sdBadFile)
    {
        if (!shimmerStatus.docked)
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
    if (update_calib_dump_file && CheckSdInslot() && !shimmerStatus.sdBadFile)
    {
        if (!shimmerStatus.docked)
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

    if (shimmerStatus.btConnected)
    {
        if (sendAck)
        {
            *(resPacket + packet_length++) = ACK_COMMAND_PROCESSED;
            sendAck = 0;
        }
        if (inquiryResponse)
        {
            *(resPacket + packet_length++) = INQUIRY_RESPONSE;
            *(uint16_t*) (resPacket + packet_length) =
                    *(uint16_t*) (storedConfig + NV_SAMPLING_RATE); //ADC sampling rate
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
            *(uint16_t*) (resPacket + packet_length) =
                    *(uint16_t*) (storedConfig + NV_SAMPLING_RATE); //ADC sampling rate
            packet_length += 2;
            samplingRateResponse = 0;
        }
        else if (wrAccelRangeResponse)
        {
            *(resPacket + packet_length++) = WR_ACCEL_RANGE_RESPONSE;
            *(resPacket + packet_length++) =
                    (storedConfig[NV_CONFIG_SETUP_BYTE0] & 0x0C) >> 2;
            wrAccelRangeResponse = 0;
        }
        else if (magGainResponse)
        {
            *(resPacket + packet_length++) = MAG_GAIN_RESPONSE;
            *(resPacket + packet_length++) =
                    (storedConfig[NV_CONFIG_SETUP_BYTE2] & 0xE0) >> 5;
            magGainResponse = 0;
        }
        else if (magSamplingRateResponse)
        {
            *(resPacket + packet_length++) =
            MAG_SAMPLING_RATE_RESPONSE;
            *(resPacket + packet_length++) =
                    (storedConfig[NV_CONFIG_SETUP_BYTE2] & 0x1C) >> 2;
            magSamplingRateResponse = 0;
        }
        else if (dockedResponse)
        {
            *(resPacket + packet_length++) = INSTREAM_CMD_RESPONSE;
            *(resPacket + packet_length++) = STATUS_RESPONSE;
            *(resPacket + packet_length++) = ((shimmerStatus.toggleLedRedCmd & 0x01) << 7)
                    + ((shimmerStatus.sdBadFile & 0x01) << 6)
                    + ((shimmerStatus.sdInserted & 0x01) << 5)
                    + ((shimmerStatus.btStreaming & 0x01) << 4)
                    + ((shimmerStatus.sdLogging & 0x01) << 3)
                    + (isRwcTimeSet() << 2)
                    + ((shimmerStatus.sensing & 0x01) << 1)
                    + (shimmerStatus.docked & 0x01);

            dockedResponse = 0;
        }
        else if (btVbattResponse)
        {
            ReadBatt();
            *(resPacket + packet_length++) = INSTREAM_CMD_RESPONSE;
            *(resPacket + packet_length++) = VBATT_RESPONSE;
            memcpy((resPacket + packet_length), batteryStatus.battStatusRaw.rawBytes[0], 3);
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
        else if (wrAccelSamplingRateResponse)
        {
            *(resPacket + packet_length++) =
            WR_ACCEL_SAMPLING_RATE_RESPONSE;
            *(resPacket + packet_length++) =
                    (storedConfig[NV_CONFIG_SETUP_BYTE0] & 0xF0) >> 4;
            wrAccelSamplingRateResponse = 0;
        }
        else if (wrAccelLpModeResponse)
        {
            *(resPacket + packet_length++) = WR_ACCEL_LPMODE_RESPONSE;
            *(resPacket + packet_length++) =
                    (storedConfig[NV_CONFIG_SETUP_BYTE0] & 0x02) >> 1;
            wrAccelLpModeResponse = 0;
        }
        else if (wrAccelHrModeResponse)
        {
            *(resPacket + packet_length++) = WR_ACCEL_HRMODE_RESPONSE;
            *(resPacket + packet_length++) = storedConfig[NV_CONFIG_SETUP_BYTE0]
                    & 0x01;
            wrAccelHrModeResponse = 0;
        }
        else if (gyroRangeResponse)
        {
            *(resPacket + packet_length++) = GYRO_RANGE_RESPONSE;
            *(resPacket + packet_length++) = storedConfig[NV_CONFIG_SETUP_BYTE2]
                    & 0x03;
            gyroRangeResponse = 0;
        }
        else if (bmp180CalibrationCoefficientsResponse)
        {
            *(resPacket + packet_length++) =
            BMP180_CALIBRATION_COEFFICIENTS_RESPONSE;

            uint8_t *bmpX80CalibPtr = get_bmp_calib_data_bytes();
            if (isBmp280InUse())
            {
                // Dummy bytes sent if incorrect calibration bytes requested.
                memset(bmpX80CalibPtr, 0x01, BMP180_CALIB_DATA_SIZE);
            }
            memcpy(resPacket + packet_length, bmpX80CalibPtr,
                   BMP180_CALIB_DATA_SIZE);
            packet_length += BMP180_CALIB_DATA_SIZE;

            bmp180CalibrationCoefficientsResponse = 0;
        }
        else if (bmp280CalibrationCoefficientsResponse)
        {
            if (isBmp280InUse())
            {
                *(resPacket + packet_length++) =
                BMP280_CALIBRATION_COEFFICIENTS_RESPONSE;
                memcpy(resPacket + packet_length, get_bmp_calib_data_bytes(),
                BMP280_CALIB_DATA_SIZE);
                packet_length += BMP280_CALIB_DATA_SIZE;
            }
            bmp280CalibrationCoefficientsResponse = 0;
        }
        else if (bmpGenericCalibrationCoefficientsResponse)
        {
            uint8_t bmpCalibByteLen = get_bmp_calib_data_bytes_len();
            *(resPacket + packet_length++) = PRESSURE_CALIBRATION_COEFFICIENTS_RESPONSE;
            *(resPacket + packet_length++) = 1U + bmpCalibByteLen;
            if (isBmp180InUse())
            {
                *(resPacket + packet_length++) = PRESSURE_SENSOR_BMP180;
            }
            else if (isBmp280InUse())
            {
                *(resPacket + packet_length++) = PRESSURE_SENSOR_BMP280;
            }
            else
            {
                *(resPacket + packet_length++) = PRESSURE_SENSOR_BMP390;
            }
            memcpy(resPacket + packet_length, get_bmp_calib_data_bytes(), bmpCalibByteLen);
            packet_length += bmpCalibByteLen;
            bmpGenericCalibrationCoefficientsResponse = 0;
        }
        else if (gyroSamplingRateResponse)
        {
            *(resPacket + packet_length++) = GYRO_SAMPLING_RATE_RESPONSE;
            *(resPacket + packet_length++) =
                    storedConfig[NV_CONFIG_SETUP_BYTE1];
            gyroSamplingRateResponse = 0;
        }
        else if (altAccelRangeResponse)
        {
            *(resPacket + packet_length++) = ALT_ACCEL_RANGE_RESPONSE;
            *(resPacket + packet_length++) =
                    (storedConfig[NV_CONFIG_SETUP_BYTE3] & 0xC0) >> 6;
            altAccelRangeResponse = 0;
        }
        else if (pressureOversamplingRatioResponse)
        {
            *(resPacket + packet_length++) =
            PRESSURE_OVERSAMPLING_RATIO_RESPONSE;
            *(resPacket + packet_length++) =
                    (storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x30) >> 4;
            pressureOversamplingRatioResponse = 0;
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
        else if (lnAccelCalibrationResponse)
        {
            *(resPacket + packet_length++) = LN_ACCEL_CALIBRATION_RESPONSE;
            sc1.id = SC_SENSOR_ANALOG_ACCEL;
            sc1.range = SC_SENSOR_RANGE_ANALOG_ACCEL;
            sc1.data_len = SC_DATA_LEN_ANALOG_ACCEL;
            ShimmerCalib_singleSensorRead(&sc1);
            memcpy((resPacket + packet_length), sc1.data.raw, sc1.data_len);
            packet_length += sc1.data_len;
            lnAccelCalibrationResponse = 0;
        }
        else if (gyroCalibrationResponse)
        {
            *(resPacket + packet_length++) = GYRO_CALIBRATION_RESPONSE;
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
            MAG_CALIBRATION_RESPONSE;
            sc1.id = SC_SENSOR_LSM303DLHC_MAG;
            sc1.range = (storedConfig[NV_CONFIG_SETUP_BYTE2] >> 5) & 0x07;
            sc1.data_len = SC_DATA_LEN_LSM303DLHC_MAG;
            ShimmerCalib_singleSensorRead(&sc1);
            memcpy((resPacket + packet_length), sc1.data.raw, sc1.data_len);
            packet_length += sc1.data_len;
            magCalibrationResponse = 0;
        }
        else if (wrAccelCalibrationResponse)
        {
            *(resPacket + packet_length++) =
            WR_ACCEL_CALIBRATION_RESPONSE;
            sc1.id = SC_SENSOR_LSM303DLHC_ACCEL;
            sc1.range = (storedConfig[NV_CONFIG_SETUP_BYTE0] >> 2) & 0x03;
            sc1.data_len = SC_DATA_LEN_LSM303DLHC_ACCEL;
            ShimmerCalib_singleSensorRead(&sc1);
            memcpy((resPacket + packet_length), sc1.data.raw, sc1.data_len);
            packet_length += sc1.data_len;
            wrAccelCalibrationResponse = 0;
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
            // Mag sensitivity adj feature is not present in ICM-20948
            if(isGyroInUseMpu9x50())
            {
                MPU9150_init();
                MPU9150_wake(1);
                MPU9150_wake(0);
                *(resPacket + packet_length++) = MPU9150_MAG_SENS_ADJ_VALS_RESPONSE;
                MPU9150_getMagSensitivityAdj(resPacket + packet_length);
                packet_length += 3;
            }
            else
            {
                *(resPacket + packet_length++) = ACK_COMMAND_PROCESSED;
            }
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
            *(resPacket + packet_length++) = FW_VER_REL;
            fwVersionResponse = 0;
        }
        else if (blinkLedResponse)
        {
            *(resPacket + packet_length++) = CHARGE_STATUS_LED_RESPONSE;
            *(resPacket + packet_length++) = batteryStatus.battStat;
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
            if (!shimmerStatus.sensing)
            {
                eepromRead(dcMemOffset + 16U, dcMemLength, resPacket + packet_length);
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
            rwc_curr_time_64 = getRwcTime();
            *(resPacket + packet_length++) = RWC_RESPONSE;
            memcpy(resPacket + packet_length, (uint8_t*) (&rwc_curr_time_64),
                   8);
            packet_length += 8;
            rwcResponse = 0;
        }
        else if (btVerResponse)
        {
            uint8_t btVerStrLen = getBtVerStrLen();

            *(resPacket + packet_length++) = BT_VERSION_STR_RESPONSE;
            *(resPacket + packet_length++) = btVerStrLen;
            memcpy((resPacket + packet_length), getBtVerStrPtr(), btVerStrLen);
            packet_length += btVerStrLen;

            btVerResponse = 0;
        }
        else if (btDataRateTestResponse)
        {
            /* Start test after ACK is sent - this will be handled by the
             * interrupt after ACK byte is transmitted */
            if (args[0] != 0)
            {
                setBtDataRateTestState(1);
            }
            btDataRateTestResponse = 0;
        }
    }

    uint8_t crcMode = getBtCrcMode();
    if (crcMode != CRC_OFF)
    {
        calculateCrcAndInsert(crcMode, resPacket, packet_length);
        packet_length += crcMode;
    }

    BT_write(resPacket, packet_length, SHIMMER_CMD);
}

void Timestamp0ToFirstFile()
{
    UINT bw;
    uint32_t my_local_time_long;
//   uint64_t rwc_curr_time_64;
//   rwc_curr_time_64 = RTC_get64();
    my_local_time_long = firstTs & 0xffffffff;
    sdHeadText[SDH_MY_LOCALTIME_5TH] = (firstTs >> 32) & 0xff;

    memcpy(&sdHeadText[SDH_MY_LOCALTIME], (uint8_t*) &my_local_time_long, 4);
//   fileLastHour = rwc_curr_time_64;
//   fileLastMin = rwc_curr_time_64;
// Write header to file
    ff_result = f_write(&dataFile, sdHeadText, SDHEAD_LEN, &bw);
}

FRESULT WriteFile(uint8_t *text, WORD size)
{
// Result code
    FRESULT rc;
    UINT bw;
    uint32_t file_td_h, file_td_m;                //, my_local_time_long32
    uint64_t local_time_40;
    wr2sd = 1;

    local_time_40 = RTC_get64();
    f_lseek(&dataFile, dataFile.fsize);                // seek to end of file, no spi op
    rc = f_write(&dataFile, text, size, &bw);                // Write to file

    file_td_h = local_time_40 - fileLastHour;
    file_td_m = local_time_40 - fileLastMin;

//create a new file (from 000 up) every 1h
    if (file_td_h >= 117964800)
    {         // 117964800 = 32768/s*3600s = 1h
        fileLastHour = fileLastMin = local_time_40;
        rc = f_close(&dataFile);
        //file number:from 000 up
        fileNum++;

        //avoid using printf()
        fileName[dirLen + 3] = (char) (((int) '0') + fileNum % 10);
        fileName[dirLen + 2] = (char) (((int) '0') + (fileNum / 10) % 10);
        fileName[dirLen + 1] = (char) (((int) '0') + (fileNum / 100) % 10);

        f_open(&dataFile, (char*) fileName, FA_WRITE | FA_CREATE_NEW);

        //      my_local_time_long32 = local_time_40 & 0xffffffff;
        //      sdHeadText[SDH_MY_LOCALTIME_5TH] = (local_time_40>>32) & 0xff;
        //      memcpy(&sdHeadText[SDH_MY_LOCALTIME], (uint8_t*)&my_local_time_long32, 4);
        //      fileLastMin = local_time_40;
        //      f_write(&fil, sdHeadText, SDHEAD_LEN, &bw);        // Write head to file
        firstTsFlag = 1;
        streamDataInProc = 0;
        TaskClear(TASK_STREAMDATA);
    }
// sync data to SD card every 1 min
    else if (file_td_m >= 1966080)
    { //32768/s*60s = 1m
        fileLastMin = local_time_40;
        f_sync(&dataFile);
    }

    ff_result = rc;

    wr2sd = 0;
    return rc;
}

uint32_t TaskCurrentGet(void)
{
    uint8_t i;
    uint32_t task = 0;
    if (taskList)
    {
        for (i = 0; i < TASK_SIZE; i++)
        {
            task = 0x00000001 << i;
            if (taskList & task)
            {
                break;
            }
        }
    }
    return task;
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
    memcpy(sdBuff + sdBuffLen, getMyTimeDiffPtr(), SYNC_PACKET_PAYLOAD_SIZE);
    sdBuffLen += SYNC_PACKET_PAYLOAD_SIZE;
    resetMyTimeDiff();
}

void ParseConfig(void)
{
    FIL cfgFile;
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
#if IS_SUPPORTED_TCXO
    tcxo = FALSE,
#endif
    user_button_enable = FALSE, low_battery_autostop = FALSE;

#if !RTC_OFF
    bool rwc_error_enable = TRUE;
#endif
    uint32_t est_exp_len = 0;
    uint32_t max_exp_len = 0;

    bool sd_error_enable = TRUE;
    bool triggerSdCardUpdate = FALSE;

    CheckSdInslot();
    char cfgname[] = "sdlog.cfg";
    ff_result = f_open(&cfgFile, cfgname, FA_READ | FA_OPEN_EXISTING);
    if (ff_result == FR_NO_FILE)
    {
        IniReadInfoMem();
        UpdateSdConfig();
        //        fileBad = 0;
    }
    else if (ff_result != FR_OK)
    {
        shimmerStatus.sdBadFile = TRUE;
        //        fileBad = (initializing) ? 0 : 1;
        return;
    }
    else
    {
        uint8_t stored_config_temp[NV_NUM_RWMEM_BYTES] = {0};

        resetSyncVariablesBeforeParseConfig();
        resetSyncNodeArray();

        broadcast_interval = SYNC_INT_C;

        memset((uint8_t*) (stored_config_temp), 0, NV_A_ACCEL_CALIBRATION); //0
        memset((uint8_t*) (stored_config_temp + NV_A_ACCEL_CALIBRATION), 0xff, 84);
        memset((uint8_t*) (stored_config_temp + NV_DERIVED_CHANNELS_3), 0, 5); //0
        memset((uint8_t*) (stored_config_temp + NV_SENSORS3), 0, 5); //0
        memset((uint8_t*) (stored_config_temp + NV_MPL_ACCEL_CALIBRATION), 0xff, 82);
        memset((uint8_t*) (stored_config_temp + NV_SD_MYTRIAL_ID), 0, 9); //0
        InfoMem_read((uint8_t*) NV_MAC_ADDRESS, stored_config_temp + NV_MAC_ADDRESS, 7);
        memset((uint8_t*) (stored_config_temp + NV_BT_SET_PIN + 1), 0xff, 24);
        memset((uint8_t*) (stored_config_temp + NV_CENTER), 0xff, 128);

        stored_config_temp[NV_SD_TRIAL_CONFIG0] &= ~SDH_SET_PMUX; // PMUX reserved as 0
        stored_config_temp[NV_SD_TRIAL_CONFIG0] &= ~SDH_BLUETOOTH_DISABLE; // Enable Bluetooth
//        stored_config_temp[NV_SD_TRIAL_CONFIG0] |= SDH_TIME_STAMP; // TIME_STAMP always = 1
        stored_config_temp[NV_CONFIG_SETUP_BYTE3] |= (GSR_AUTORANGE & 0x07) << 1; //BIT3-1
        stored_config_temp[NV_BUFFER_SIZE] = 1;
        stored_config_temp[NV_BT_COMMS_BAUD_RATE] = getDefaultBaudForBtVersion();
        changeBtBaudRate = BAUD_NO_CHANGE_NEEDED; // set this flag to no_event as every sdlog.cfg read is followed by a baudrate setting

        memset(shimmerName, 0, MAX_CHARS);
        memset(expIdName, 0, MAX_CHARS);
        memset(configTimeText, 0, MAX_CHARS);

        *(stored_config_temp + NV_SD_SHIMMER_NAME) = '\0';
        *(stored_config_temp + NV_SD_EXP_ID_NAME) = '\0';
        *configTimeText = '\0';
        memset((uint8_t*) (stored_config_temp + NV_SD_CONFIG_TIME), 0, 4);
        float old_clock_freq = clockFreq;

        while (f_gets(buffer, 64, &cfgFile))
        {
            if (!(equals = strchr(buffer, '=')))
                continue;
            equals++;     // this is the value
            if (strstr(buffer, "accel="))
                stored_config_temp[NV_SENSORS0] |= atoi(equals) * SENSOR_A_ACCEL;
            else if (strstr(buffer, "gyro="))
                stored_config_temp[NV_SENSORS0] |= atoi(equals)*SENSOR_MPU9X50_ICM20948_GYRO;
            else if (strstr(buffer, "mag="))
                stored_config_temp[NV_SENSORS0] |= atoi(equals)*SENSOR_LSM303XXXX_MAG;
            else if (strstr(buffer, "exg1_24bit="))
                stored_config_temp[NV_SENSORS0] |= atoi(equals) * SENSOR_EXG1_24BIT;
            else if (strstr(buffer, "exg2_24bit="))
                stored_config_temp[NV_SENSORS0] |= atoi(equals) * SENSOR_EXG2_24BIT;
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
                stored_config_temp[NV_SENSORS1] |= atoi(equals)*SENSOR_LSM303XXXX_ACCEL;
            else if (strstr(buffer, "extch15="))
                stored_config_temp[NV_SENSORS1] |= atoi(equals) * SENSOR_EXT_A15;
            else if (strstr(buffer, "intch1="))
                stored_config_temp[NV_SENSORS1] |= atoi(equals) * SENSOR_INT_A1;
            else if (strstr(buffer, "intch12="))
                stored_config_temp[NV_SENSORS1] |= atoi(equals) * SENSOR_INT_A12;
            else if (strstr(buffer, "intch13="))
                stored_config_temp[NV_SENSORS1] |= atoi(equals) * SENSOR_INT_A13;
            else if (strstr(buffer, "intch14="))
                stored_config_temp[NV_SENSORS2] |= atoi(equals) * SENSOR_INT_A14;
            else if (strstr(buffer, "accel_mpu="))
                stored_config_temp[NV_SENSORS2] |= atoi(equals)*SENSOR_MPU9X50_ICM20948_ACCEL;
            else if (strstr(buffer, "mag_mpu="))
                stored_config_temp[NV_SENSORS2] |= atoi(equals) * SENSOR_MPU9X50_ICM20948_MAG;
            else if (strstr(buffer, "exg1_16bit="))
                stored_config_temp[NV_SENSORS2] |= atoi(equals) * SENSOR_EXG1_16BIT;
            else if (strstr(buffer, "exg2_16bit="))
                stored_config_temp[NV_SENSORS2] |= atoi(equals) * SENSOR_EXG2_16BIT;
            else if (strstr(buffer, "pres_bmp180=")
                    || strstr(buffer, "pres_bmp280="))
                stored_config_temp[NV_SENSORS2] |= atoi(equals)*SENSOR_BMPX80_PRESSURE;
            else if (strstr(buffer, "sample_rate="))
            {
                sample_rate = atof(equals);
            }
            else if (strstr(buffer, "mg_internal_rate="))
            {
                mag_smplrate = atoi(equals);
                stored_config_temp[NV_CONFIG_SETUP_BYTE2] |= (mag_smplrate & 0x07) << 2;     //BIT4-2
            }
            else if (strstr(buffer, "mg_range="))
            {
                mag_gain = atoi(equals);
                stored_config_temp[NV_CONFIG_SETUP_BYTE2] |= (mag_gain & 0x07) << 5; //BIT7-5
            }
            else if (strstr(buffer, "acc_internal_rate="))
            {
                accel_smplrate = atoi(equals);
                stored_config_temp[NV_CONFIG_SETUP_BYTE0] |= (accel_smplrate & 0x0f) << 4;   //BIT7-4
            }
            else if (strstr(buffer, "accel_mpu_range="))
            {
                accel_mpu_range = atoi(equals);
                stored_config_temp[NV_CONFIG_SETUP_BYTE3] |= (accel_mpu_range & 0x03) << 6;  //BIT7-6
            }
            else if (strstr(buffer, "acc_range="))
            {
                accel_range = atoi(equals);
                stored_config_temp[NV_CONFIG_SETUP_BYTE0] |= (accel_range & 0x03) << 2;      //BIT3-2
            }
            else if (strstr(buffer, "acc_lpm="))
            {
                accel_lpm = atoi(equals);
                stored_config_temp[NV_CONFIG_SETUP_BYTE0] |= (accel_lpm & 0x01) << 1;       //BIT1
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
                stored_config_temp[NV_CONFIG_SETUP_BYTE3] |= ((gsr_range << 1) & GSR_RANGE);
            }
            else if (strstr(buffer, "gyro_samplingrate="))
                stored_config_temp[NV_CONFIG_SETUP_BYTE1] = atoi(equals);
            else if (strstr(buffer, "gyro_range="))
            {
                gyro_range = atoi(equals);
                stored_config_temp[NV_CONFIG_SETUP_BYTE2] |= (gyro_range) & 0x03; //BIT1-0
            }
            else if (strstr(buffer, "pres_bmp180_prec=")
                    || strstr(buffer, "pres_bmp280_prec="))
            {
                pres_bmpX80_prec = atoi(equals);
                stored_config_temp[NV_CONFIG_SETUP_BYTE3] |= (pres_bmpX80_prec & 0x03) << 4;   //BIT5-4
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
                stored_config_temp[NV_SD_TRIAL_CONFIG0] |= sd_error_enable * SDH_SDERROR_EN;
            }
            else if (strstr(buffer, "user_button_enable="))
            {
                user_button_enable = (atoi(equals) == 0) ? FALSE : TRUE;
                stored_config_temp[NV_SD_TRIAL_CONFIG0] |= user_button_enable * SDH_USER_BUTTON_ENABLE;
            }
            else if (strstr(buffer, "iammaster="))
            {   //0=slave=node
                iAmMaster = (atoi(equals) == 0) ? FALSE : TRUE;
                stored_config_temp[NV_SD_TRIAL_CONFIG0] |= iAmMaster * SDH_IAMMASTER;
            }
            else if (strstr(buffer, "sync="))
            {
                time_sync = (atoi(equals) == 0) ? FALSE : TRUE;
                stored_config_temp[NV_SD_TRIAL_CONFIG0] |= time_sync * SDH_TIME_SYNC;
            }
            else if (strstr(buffer, "bluetoothDisabled="))
            {
                stored_config_temp[NV_SD_TRIAL_CONFIG0] |= (atoi(equals) == 0) ? 0 : SDH_BLUETOOTH_DISABLE;
            }
            else if (strstr(buffer, "low_battery_autostop="))
            {
                low_battery_autostop = (atoi(equals) == 0) ? FALSE : TRUE;
                stored_config_temp[NV_SD_TRIAL_CONFIG1] |= low_battery_autostop * SDH_BATT_CRITICAL_CUTOFF;
            }
#if IS_SUPPORTED_TCXO
            else if (strstr(buffer, "tcxo="))
            {
                tcxo = (atoi(equals) == 0) ? FALSE : TRUE;
                stored_config_temp[NV_SD_TRIAL_CONFIG1] |= tcxo * SDH_TCXO;
            }
#endif
            else if (strstr(buffer, "interval="))
            {
                broadcast_interval = atoi(equals) > 255 ? 255 : atoi(equals);
            }
            else if (strstr(buffer, "exp_power="))
            {
                exp_power = atoi(equals);
                stored_config_temp[NV_CONFIG_SETUP_BYTE3] |= exp_power ? EXP_POWER_ENABLE : 0;
            }

            else if (strstr(buffer, "center="))
            {
                parseSyncCenterNameFromCfgFile(&stored_config_temp[0], equals);
            }
            else if (strstr(buffer, "node"))
            {
                parseSyncNodeNameFromCfgFile(&stored_config_temp[0], equals);
            }
            else if (strstr(buffer, "est_exp_len="))
            {
                est_exp_len = atoi(equals);
                stored_config_temp[NV_EST_EXP_LEN_MSB] = (est_exp_len & 0xff00) >> 8;
                stored_config_temp[NV_EST_EXP_LEN_LSB] = est_exp_len & 0xff;
            }

            else if (strstr(buffer, "max_exp_len="))
            {
                max_exp_len = atoi(equals);
                stored_config_temp[NV_MAX_EXP_LEN_MSB] = (max_exp_len & 0xff00) >> 8;
                stored_config_temp[NV_MAX_EXP_LEN_LSB] = max_exp_len & 0xff;
            }

            else if (strstr(buffer, "singletouch="))
            {
                if (IS_SUPPORTED_SINGLE_TOUCH)
                {
                    stored_config_temp[NV_SD_TRIAL_CONFIG1] &= ~SDH_SINGLETOUCH;
                }
                else
                {
                    stored_config_temp[NV_SD_TRIAL_CONFIG1] |= (atoi(equals) == 0) ? 0 : SDH_SINGLETOUCH;
                }
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
                memcpy((char*) (stored_config_temp + NV_SD_SHIMMER_NAME), equals, string_length);
                if (!memcmp((char*) (stored_config_temp + NV_SD_SHIMMER_NAME), "ID", 2))
                    memcpy((char*) (stored_config_temp + NV_SD_SHIMMER_NAME), "id", 2);
                memcpy((char*) shimmerName, (char*) (stored_config_temp + NV_SD_SHIMMER_NAME), MAX_CHARS - 1);
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
                memcpy((char*) (stored_config_temp + NV_SD_EXP_ID_NAME), equals, string_length);
                memcpy((char*) expIdName, (char*) (stored_config_temp + NV_SD_EXP_ID_NAME), MAX_CHARS - 1);
                expIdName[string_length] = 0;
            }
            else if (strstr(buffer, "configtime="))
            {
                config_time = atol(equals);
                string_length = MAX_CHARS < strlen(equals) ? MAX_CHARS : strlen(equals) - 1;
                memcpy((char*) configTimeText, equals, string_length - 1);
                my_config_time[3] = *((uint8_t*) &config_time);
                my_config_time[2] = *(((uint8_t*) &config_time) + 1);
                my_config_time[1] = *(((uint8_t*) &config_time) + 2);
                my_config_time[0] = *(((uint8_t*) &config_time) + 3);
                memcpy((uint8_t*) (stored_config_temp + NV_SD_CONFIG_TIME), my_config_time, 4);
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
                if (config_baudrate != getCurrentBtBaudRate())
                {
#if BT_ENABLE_BAUD_RATE_CHANGE
                    if (config_baudrate <= BAUD_1000000)
                    {
                        changeBtBaudRate = config_baudrate;
                    }
#else
                    triggerSdCardUpdate = TRUE;
#endif
                }
                stored_config_temp[NV_BT_COMMS_BAUD_RATE] = getCurrentBtBaudRate();
            }
            else if (strstr(buffer, "derived_channels="))
            {
                //            derived_channels_val = atol(equals);
                //            stored_config_temp[NV_DERIVED_CHANNELS_0] = derived_channels_val & 0xff;
                //            stored_config_temp[NV_DERIVED_CHANNELS_1] = (derived_channels_val >> 8) & 0xff;
                //            stored_config_temp[NV_DERIVED_CHANNELS_2] = (derived_channels_val >> 16) & 0xff;
                derived_channels_val = atoll(equals);
                stored_config_temp[NV_DERIVED_CHANNELS_0] = derived_channels_val & 0xff;
                stored_config_temp[NV_DERIVED_CHANNELS_1] = (derived_channels_val >> 8) & 0xff;
                stored_config_temp[NV_DERIVED_CHANNELS_2] = (derived_channels_val >> 16) & 0xff;
                stored_config_temp[NV_DERIVED_CHANNELS_3] = (derived_channels_val >> 24) & 0xff;
                stored_config_temp[NV_DERIVED_CHANNELS_4] = (derived_channels_val >> 32) & 0xff;
                stored_config_temp[NV_DERIVED_CHANNELS_5] = (derived_channels_val >> 40) & 0xff;
                stored_config_temp[NV_DERIVED_CHANNELS_6] = (derived_channels_val >> 48) & 0xff;
                stored_config_temp[NV_DERIVED_CHANNELS_7] = (derived_channels_val >> 56) & 0xff;
            }
        }
        ff_result = f_close(&cfgFile);
        _delay_cycles(1200000);

        maxLen = max_exp_len * 60;
        SamplingClkAssignment(&stored_config_temp[0]);

        if (clockFreq != old_clock_freq)
        {
            ClkAssignment();
            TB0CTL = MC_0; // StopTb0()
            TB0Start();
        }

        sample_period = FreqDiv(sample_rate);

        // little endian:
        stored_config_temp[NV_SAMPLING_RATE] = (uint8_t) (sample_period & 0xFF);
        stored_config_temp[NV_SAMPLING_RATE + 1] = (uint8_t) (sample_period >> 8);

        if (stored_config_temp[NV_SENSORS0] & SENSOR_GSR)
        { // they are sharing adc1, so ban intch1 when gsr is on
            stored_config_temp[NV_SENSORS1] &= ~SENSOR_INT_A1;
            triggerSdCardUpdate = TRUE;
        }
        if (stored_config_temp[NV_SENSORS1] & SENSOR_STRAIN)
        { // they are sharing adc13 and adc14
            stored_config_temp[NV_SENSORS1] &= ~SENSOR_INT_A13;
            stored_config_temp[NV_SENSORS2] &= ~SENSOR_INT_A14;
            triggerSdCardUpdate = TRUE;
        }
        if (stored_config_temp[NV_SENSORS0] & SENSOR_EXG1_24BIT)
        {
            stored_config_temp[NV_SENSORS2] &= ~SENSOR_EXG1_16BIT;
            triggerSdCardUpdate = TRUE;
        }
        if (stored_config_temp[NV_SENSORS0] & SENSOR_EXG2_24BIT)
        {
            stored_config_temp[NV_SENSORS2] &= ~SENSOR_EXG2_16BIT;
            triggerSdCardUpdate = TRUE;
        }
        if (stored_config_temp[NV_SENSORS0] & SENSOR_EXG1_24BIT
                || stored_config_temp[NV_SENSORS0] & SENSOR_EXG2_24BIT
                || stored_config_temp[NV_SENSORS2] & SENSOR_EXG1_16BIT
                || stored_config_temp[NV_SENSORS2] & SENSOR_EXG2_16BIT)
        {
            stored_config_temp[NV_SENSORS1] &= ~SENSOR_INT_A1;
            stored_config_temp[NV_SENSORS2] &= ~SENSOR_INT_A14;
            triggerSdCardUpdate = TRUE;
        }

        if (((stored_config_temp[NV_CONFIG_SETUP_BYTE3] >> 1) & 0x07) > 4)
        { // never larger than 4
            stored_config_temp[NV_CONFIG_SETUP_BYTE3] &= 0xf1;
            stored_config_temp[NV_CONFIG_SETUP_BYTE3] |= (GSR_AUTORANGE & 0x07) << 1; //BIT3-1
            triggerSdCardUpdate = TRUE;
        }
#if !RTC_OFF
        stored_config_temp[NV_SD_TRIAL_CONFIG0] |= rwc_error_enable * SDH_RWCERROR_EN;
#endif
        stored_config_temp[NV_SD_TRIAL_CONFIG0] |= sd_error_enable * SDH_SDERROR_EN;

        // minimum sync broadcast interval is 54 seconds
        if (broadcast_interval < SYNC_INT_C)
        {
            broadcast_interval = SYNC_INT_C;
            triggerSdCardUpdate = TRUE;
        }
        stored_config_temp[NV_SD_BT_INTERVAL] = broadcast_interval;

        // the button always works for singletouch mode
        // sync always works for singletouch mode
        if (IS_SUPPORTED_SINGLE_TOUCH && stored_config_temp[NV_SD_TRIAL_CONFIG1] & SDH_SINGLETOUCH)
        {
            stored_config_temp[NV_SD_TRIAL_CONFIG0] |= SDH_USER_BUTTON_ENABLE;
            stored_config_temp[NV_SD_TRIAL_CONFIG0] |= SDH_TIME_SYNC;
            triggerSdCardUpdate = TRUE;
        }

        checkSyncCenterName();
        setSyncEstExpLen(est_exp_len);

        /* Calibration bytes are not copied over from the infomem */

        /* Infomem D - Bytes 0-33 - General settings */
        memcpy(storedConfig, stored_config_temp, NV_NUM_SETTINGS_BYTES);
        /* Infomem D - Bytes 118-122 - Derived channel settings */
        memcpy((uint8_t*) (storedConfig + NV_DERIVED_CHANNELS_3), (uint8_t*) (stored_config_temp + NV_DERIVED_CHANNELS_3), 5);
        /* Infomem C - Bytes 128-132 - MPL related settings - no longer used/supported */
        memcpy((uint8_t*) (storedConfig + NV_SENSORS3), (uint8_t*) (stored_config_temp + NV_SENSORS3), 5);
        /* Infomem C - Bytes 187-223 - Shimmer name, exp ID, config time, trial ID, num Shimmers, trial config, BT interval, est exp len, max exp len */
        memcpy((uint8_t*) (storedConfig + NV_SD_SHIMMER_NAME), (uint8_t*) (stored_config_temp + NV_SD_SHIMMER_NAME), 37);
        /* Infomem B - Bytes 256-381 - Center and Node MAC addresses */
        memcpy((uint8_t*) (storedConfig + NV_CENTER), (uint8_t*) (stored_config_temp + NV_CENTER), NV_NUM_BYTES_SYNC_CENTER_NODE_ADDRS);
//        memcpy((uint8_t*) (storedConfig + NV_MAC_ADDRESS + 7), (uint8_t*) (stored_config_temp + NV_MAC_ADDRESS + 7), 153); //25+128

        Config2SdHead();
        SetName();

        InfoMem_write((uint8_t*) 0, storedConfig, NV_NUM_SETTINGS_BYTES);
        InfoMem_write((uint8_t*) NV_DERIVED_CHANNELS_3, storedConfig + NV_DERIVED_CHANNELS_3, 5);
        InfoMem_write((uint8_t*) NV_SENSORS3, storedConfig + NV_SENSORS3, 5);
        InfoMem_write((uint8_t*) NV_SD_SHIMMER_NAME, storedConfig + NV_SD_SHIMMER_NAME, 37);
        InfoMem_write((uint8_t*) NV_CENTER, storedConfig + NV_CENTER, NV_NUM_BYTES_SYNC_CENTER_NODE_ADDRS);
//        InfoMem_write((uint8_t*) (NV_MAC_ADDRESS + 7), storedConfig + NV_MAC_ADDRESS + 7, 153); //25+128

        /* If the configuration needed to be corrected, update the config file */
        if (triggerSdCardUpdate)
        {
            UpdateSdConfig();
        }
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
    sdHeadText[SDH_FW_VERSION_INTERNAL] = FW_VER_REL;

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

    uint64_t * rwcTimeDiffPtr = getRwcTimeDiffPtr();
    sdHeadText[SDH_RTC_DIFF_7] = *((uint8_t*) rwcTimeDiffPtr);
    sdHeadText[SDH_RTC_DIFF_6] = *(((uint8_t*) rwcTimeDiffPtr) + 1);
    sdHeadText[SDH_RTC_DIFF_5] = *(((uint8_t*) rwcTimeDiffPtr) + 2);
    sdHeadText[SDH_RTC_DIFF_4] = *(((uint8_t*) rwcTimeDiffPtr) + 3);
    sdHeadText[SDH_RTC_DIFF_3] = *(((uint8_t*) rwcTimeDiffPtr) + 4);
    sdHeadText[SDH_RTC_DIFF_2] = *(((uint8_t*) rwcTimeDiffPtr) + 5);
    sdHeadText[SDH_RTC_DIFF_1] = *(((uint8_t*) rwcTimeDiffPtr) + 6);
    sdHeadText[SDH_RTC_DIFF_0] = *(((uint8_t*) rwcTimeDiffPtr) + 7);

    memcpy(&sdHeadText[SDH_MAC_ADDR], &storedConfig[NV_MAC_ADDRESS], 6);
    memcpy(&sdHeadText[SDH_CONFIG_TIME_0], &storedConfig[NV_SD_CONFIG_TIME], 4);
    saveBmpCalibrationToSdHeader();

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
    memset((uint8_t*) (storedConfig), 0, NV_A_ACCEL_CALIBRATION); //0
    memset((uint8_t*) (storedConfig + NV_A_ACCEL_CALIBRATION), 0xff, 84);
    memset((uint8_t*) (storedConfig + NV_DERIVED_CHANNELS_3), 0, 5); //0
    memset((uint8_t*) (storedConfig + NV_SENSORS3), 0, 5); //0
    memset((uint8_t*) (storedConfig + NV_SD_SHIMMER_NAME), 0, 37); //0
    InfoMem_read((uint8_t*) NV_MAC_ADDRESS, storedConfig + NV_MAC_ADDRESS, 7);
    memset((uint8_t*) (storedConfig + NV_SD_CONFIG_DELAY_FLAG + 1), 0xff, 25);
    memset((uint8_t*) (storedConfig + NV_NODE0), 0xff, 128);

    /* 51.2Hz */
    *(uint16_t*) (storedConfig + NV_SAMPLING_RATE) = 640; /* 32768 / 51.2 = 640 */
    storedConfig[NV_BUFFER_SIZE] = 1;
    /* core sensors enabled */
    storedConfig[NV_SENSORS0] = SENSOR_A_ACCEL + SENSOR_MPU9X50_ICM20948_GYRO + SENSOR_LSM303XXXX_MAG;
    storedConfig[NV_SENSORS1] = SENSOR_VBATT;
    storedConfig[NV_SENSORS2] = 0;
    /* LSM303DLHC Accel 100Hz, +/-2G, Low Power and High Resolution modes off */
    storedConfig[NV_CONFIG_SETUP_BYTE0] = (LSM303DLHC_ACCEL_100HZ << 4) + (ACCEL_2G << 2);
    /* MPU9150 sampling rate of 8kHz/(155+1), i.e. 51.282Hz */
    storedConfig[NV_CONFIG_SETUP_BYTE1] = 0x9B;
    /* LSM303DLHC Mag 75Hz, +/-1.3 Gauss, MPU9150 Gyro +/-500 degrees per second */
    storedConfig[NV_CONFIG_SETUP_BYTE2] = (LSM303DLHC_MAG_1_3G << 5) + (LSM303DLHC_MAG_75HZ << 2) + MPU9150_GYRO_500DPS;
    /* MPU9150 Accel +/-2G, BMP pressure oversampling ratio 1, GSR auto range, EXP_RESET_N pin set low */
    /* todo: *** *** *** warning! *** *** ***  btStream for this here is not correct, this is mpu9150_accrange, not lsm303 acc range */
    storedConfig[NV_CONFIG_SETUP_BYTE3] = (ACCEL_2G << 6) + (BMPX80_OSS_1 << 4) + (GSR_AUTORANGE << 1); /* HW_RES_40K */

    /* Set all ExG registers to their reset values */
    /*ADS CHIP 1*/
    storedConfig[NV_EXG_ADS1292R_1_CONFIG1] = 0x02;
    storedConfig[NV_EXG_ADS1292R_1_CONFIG2] = 0x80;
    if ((daughtCardId[DAUGHT_CARD_ID] == EXP_BRD_EXG_UNIFIED)
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

    /*BT Baud Rate*/
    if (storedConfig[NV_BT_COMMS_BAUD_RATE] == 0xFF)
    {
        storedConfig[NV_BT_COMMS_BAUD_RATE] = getDefaultBaudForBtVersion();
    }

    /* sd config */
    /* shimmername */
    strcpy((char*) (storedConfig + NV_SD_SHIMMER_NAME), "Shimmer_XXXX");
    memcpy((storedConfig + NV_SD_SHIMMER_NAME) + 8, getMacIdStrPtr() + 8, 4);
    memcpy(shimmerName, (storedConfig + NV_SD_SHIMMER_NAME), 6);
    shimmerName[6] = 0;

    /* exp_id */
    strcpy((char*) (storedConfig + NV_SD_EXP_ID_NAME), "default_exp");
    strcpy((char*) expIdName, "default_exp");
    strcpy((char*) configTimeText, "0");
    memset(&storedConfig[NV_SD_CONFIG_TIME], 0x00, 4);
    storedConfig[NV_SD_MYTRIAL_ID] = 0x00;
    storedConfig[NV_SD_NSHIMMER] = 0x00;
    storedConfig[NV_SD_TRIAL_CONFIG0] = SDH_USER_BUTTON_ENABLE + SDH_RWCERROR_EN + SDH_SDERROR_EN;
    storedConfig[NV_SD_TRIAL_CONFIG1] = 0;
    storedConfig[NV_SD_BT_INTERVAL] = 54;

    /* Write RAM contents to Infomem */
    InfoMem_write((uint8_t*) 0, storedConfig, NV_NUM_SETTINGS_BYTES);
    InfoMem_write((uint8_t*) NV_SENSORS3, storedConfig + NV_SENSORS3, 5);
    InfoMem_write((uint8_t*) NV_SD_SHIMMER_NAME,
                  &storedConfig[NV_SD_SHIMMER_NAME],
                  NV_NUM_SD_BYTES);
    InfoMem_write((uint8_t*) (NV_MAC_ADDRESS + 7),
                  storedConfig + NV_MAC_ADDRESS + 7, 153);   //25+128
}

void SetShimmerName()
{
    uint8_t i;
    for (i = 0; (i < MAX_CHARS - 1) && isprint(storedConfig[NV_SD_SHIMMER_NAME + i]); i++);
    if (i)
    {
        memcpy((char*) shimmerName, (char*) (storedConfig + NV_SD_SHIMMER_NAME), i);
        shimmerName[i] = 0;
    }
    else
    {
        strcpy((char*) (storedConfig + NV_SD_SHIMMER_NAME), "Shimmer_XXXX");
        memcpy((storedConfig + NV_SD_SHIMMER_NAME) + 8, getMacIdStrPtr() + 8, 4);
        strcpy((char*) shimmerName, (char*) (storedConfig + NV_SD_SHIMMER_NAME));
    }
}

void SetExpIdName()
{
    uint8_t i;   //, len_temp;
    for (i = 0; (i < MAX_CHARS - 1) && isprint(storedConfig[NV_SD_EXP_ID_NAME + i]); i++);
    if (i)
    {
        memcpy((char*) expIdName, (char*) (storedConfig + NV_SD_EXP_ID_NAME), i);
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
                        dirCounter++; // start with next in numerical sequence
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
        *(uint64_t*) (sc1.ts) = getRwcTime();
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
        InfoMem_read((uint8_t*) NV_CONFIG_SETUP_BYTE2, &info_config, 1);
        sc1.range = info_config & 0x03;
    }
    else if (sc1.id == SC_SENSOR_LSM303DLHC_ACCEL)
    {
        offset = NV_LSM303DLHC_ACCEL_CALIBRATION;
        sc1.data_len = SC_DATA_LEN_LSM303DLHC_ACCEL;
        InfoMem_read((uint8_t*) NV_CONFIG_SETUP_BYTE0, &info_config, 1);
        sc1.range = (info_config & 0x0c) >> 2;
    }
    else if (sc1.id == SC_SENSOR_LSM303DLHC_MAG)
    {
        offset = NV_LSM303DLHC_MAG_CALIBRATION;
        sc1.data_len = SC_DATA_LEN_LSM303DLHC_MAG;
        InfoMem_read((uint8_t*) NV_CONFIG_SETUP_BYTE2, &info_config, 1);
        sc1.range = (info_config & 0xe0) >> 5;
        if (isWrAccelInUseLsm303dlhc())
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

    InfoMem_read((uint8_t*) offset, storedConfig + offset, 21);
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
        if (isWrAccelInUseLsm303dlhc())
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

void CalibFromFile()
{
    TaskClear(TASK_STREAMDATA);                   // this will skip one sample
    if (storedConfig[NV_SENSORS1] & SENSOR_LSM303XXXX_ACCEL)
    {
        CalibFromFileRead(S_ACCEL_D);
        memcpy(&sdHeadText[SDH_LSM303DLHC_ACCEL_CALIBRATION],
               &storedConfig[NV_LSM303DLHC_ACCEL_CALIBRATION], 21);
        InfoMem_write((uint8_t*) NV_LSM303DLHC_ACCEL_CALIBRATION,
                      &storedConfig[NV_LSM303DLHC_ACCEL_CALIBRATION], 21);
    }
    if (storedConfig[NV_SENSORS0] & SENSOR_MPU9X50_ICM20948_GYRO)
    {
        CalibFromFileRead(S_GYRO);
        memcpy(&sdHeadText[SDH_MPU9150_GYRO_CALIBRATION],
               &storedConfig[NV_MPU9150_GYRO_CALIBRATION], 21);
        InfoMem_write((uint8_t*) NV_MPU9150_GYRO_CALIBRATION,
                      &storedConfig[NV_MPU9150_GYRO_CALIBRATION], 21);
    }
    if (storedConfig[NV_SENSORS0] & SENSOR_LSM303XXXX_MAG)
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
        saveBmpCalibrationToSdHeader();
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
        strcpy(cal_indiv_file, "/calib_gyro_");     //calib_gyro_250dps.ini
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
        strcpy(cal_indiv_file, "/calib_mag_");         //calib_mag_13ga.ini
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
        strcpy(cal_indiv_file, "/calib_accel_wr_");  //calib_accel_d_2g.ini
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
        InfoMem_read((uint8_t*) NV_CONFIG_SETUP_BYTE2, &info_config, 1);
        info_range = info_config & 0x03;
        ram_range = storedConfig[NV_CONFIG_SETUP_BYTE2] & 0x03;
        // must have info_range == ram_range or there is a mismatch between RAM and infomem
        if (info_range != ram_range)
            return 0;
    }
    else if (sensor == S_MAG)
    {
        address = NV_LSM303DLHC_MAG_CALIBRATION;
        InfoMem_read((uint8_t*) NV_CONFIG_SETUP_BYTE2, &info_config, 1);
        info_range = (info_config & 0xe0) >> 5;
        ram_range = (storedConfig[NV_CONFIG_SETUP_BYTE2] & 0xe0) >> 5;
        if (info_range != ram_range)
            return 0;
    }
    else if (sensor == S_ACCEL_D)
    {
        address = NV_LSM303DLHC_ACCEL_CALIBRATION;
        InfoMem_read((uint8_t*) NV_CONFIG_SETUP_BYTE0, &info_config, 1);
        info_range = (info_config & 0x0c) >> 2;
        ram_range = (storedConfig[NV_CONFIG_SETUP_BYTE0] & 0x0c) >> 2;
        if (info_range != ram_range)
            return 0;
    }

    InfoMem_read((uint8_t*) address, storedConfig + address, 21);
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
        saveBmpCalibrationToSdHeader();
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
        saveBmpCalibrationToSdHeader();
    }
}

void CalibFromInfo(uint8_t sensor)
{
    uint8_t ram_range, info_range = 0xff, info_config, info_valid = 0;
    int byte_cnt = 0;

    if (sensor == S_ACCEL_A)
    {
        InfoMem_read((uint8_t*) NV_A_ACCEL_CALIBRATION,
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
        InfoMem_read((uint8_t*) NV_CONFIG_SETUP_BYTE2, &info_config, 1);
        info_range = info_config & 0x03;
        ram_range = storedConfig[NV_CONFIG_SETUP_BYTE2] & 0x03;
        // must have info_range == ram_range or there is a mismatch between RAM and infomem
        if (info_range == ram_range)
        {
            InfoMem_read((uint8_t*) NV_MPU9150_GYRO_CALIBRATION,
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
        InfoMem_read((uint8_t*) NV_CONFIG_SETUP_BYTE2, &info_config, 1);
        info_range = (info_config & 0xe0) >> 5;
        ram_range = (storedConfig[NV_CONFIG_SETUP_BYTE2] & 0xe0) >> 5;
        if (info_range == ram_range)
        {
            InfoMem_read((uint8_t*) NV_LSM303DLHC_MAG_CALIBRATION,
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
        InfoMem_read((uint8_t*) NV_CONFIG_SETUP_BYTE0, &info_config, 1);
        info_range = (info_config & 0x0c) >> 2;
        ram_range = (storedConfig[NV_CONFIG_SETUP_BYTE0] & 0x0c) >> 2;
        if (info_range == ram_range)
        {
            InfoMem_read((uint8_t*) NV_LSM303DLHC_ACCEL_CALIBRATION,
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

    if (!shimmerStatus.docked && CheckSdInslot() && !shimmerStatus.sdBadFile)
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
            sensitivity = isWrAccelInUseLsm303dlhc() ? 1631 : 1671;
        }
        else if (lsm303_accel_range == RANGE_4G)
        {
            sensitivity = isWrAccelInUseLsm303dlhc() ? 815 : 836;
        }
        else if (lsm303_accel_range == RANGE_8G)
        {
            sensitivity = isWrAccelInUseLsm303dlhc() ? 408 : 418;
        }
        else
        {
            sensitivity = isWrAccelInUseLsm303dlhc() ? 135 : 209;
        }
        align_xx = isWrAccelInUseLsm303dlhc() ? (-100) : 0;
        align_xy = isWrAccelInUseLsm303dlhc() ? 0 : -100;
        align_xz = 0;

        align_yx = isWrAccelInUseLsm303dlhc() ? 0 : 100;
        align_yy = isWrAccelInUseLsm303dlhc() ? 100 : 0;
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
        if (isWrAccelInUseLsm303dlhc())
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
        align_xx = isWrAccelInUseLsm303dlhc() ? (-100) : 0;
        align_xy = isWrAccelInUseLsm303dlhc() ? 0 : -100;
        align_xz = 0;

        align_yx = isWrAccelInUseLsm303dlhc() ? 0 : 100;
        align_yy = isWrAccelInUseLsm303dlhc() ? 100 : 0;
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
        if ((isWrAccelInUseLsm303ahtr() || isWrAccelInUseIcm20948()) && isBmp280InUse())
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
    bias_byte_one = (uint8_t) (bias >> 8);
    bias_byte_two = (uint8_t) (bias);
    sens_byte_one = (uint8_t) (sensitivity >> 8);
    sens_byte_two = (uint8_t) (sensitivity);
    // offset
    for (i = 0; i < number_axes; i++)
    {
        storedConfig[address + 2 * i] = bias_byte_one;
        storedConfig[address + 2 * i + 1] = bias_byte_two;
    }
    // sensitivity
    if (sensor == S_MAG)
    {
        sens_byte_one = (uint8_t) (sensitivity_x >> 8);
        sens_byte_two = (uint8_t) (sensitivity_x);
        storedConfig[address + 2 * (number_axes)] = sens_byte_one;
        storedConfig[address + 2 * (number_axes) + 1] = sens_byte_two;
        sens_byte_one = (uint8_t) (sensitivity_y >> 8);
        sens_byte_two = (uint8_t) (sensitivity_y);
        storedConfig[address + 2 * (number_axes + 1)] = sens_byte_one;
        storedConfig[address + 2 * (number_axes + 1) + 1] = sens_byte_two;
        sens_byte_one = (uint8_t) (sensitivity_z >> 8);
        sens_byte_two = (uint8_t) (sensitivity_z);
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
    if (!shimmerStatus.docked && CheckSdInslot() && !shimmerStatus.sdBadFile)
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
            strcat((char*) cal_file_name, "/calib_mag_"); //calib_mag_13ga.ini
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

void CalibSaveFromInfoMemToCalibDump(uint8_t id)
{
    if (id==0xFF || id==SC_SENSOR_ANALOG_ACCEL)
    {
        ShimmerCalib_singleSensorWriteFromInfoMem(SC_SENSOR_ANALOG_ACCEL,
                                                  SC_SENSOR_RANGE_ANALOG_ACCEL,
                                                  SC_DATA_LEN_ANALOG_ACCEL,
                                                  &storedConfig[NV_A_ACCEL_CALIBRATION]);
    }
    if (id==0xFF || id==SC_SENSOR_MPU9150_GYRO)
    {
        ShimmerCalib_singleSensorWriteFromInfoMem(SC_SENSOR_MPU9150_GYRO,
                                                  storedConfig[NV_CONFIG_SETUP_BYTE2] & 0x03,
                                                  SC_DATA_LEN_MPU9250_GYRO,
                                                  &storedConfig[NV_MPU9150_GYRO_CALIBRATION]);
    }
    if (id==0xFF || id==SC_SENSOR_LSM303DLHC_MAG)
    {
        ShimmerCalib_singleSensorWriteFromInfoMem(SC_SENSOR_LSM303DLHC_MAG,
                                                  (storedConfig[NV_CONFIG_SETUP_BYTE2] >> 5) & 0x07,
                                                  SC_DATA_LEN_LSM303DLHC_MAG,
                                                  &storedConfig[NV_LSM303DLHC_MAG_CALIBRATION]);
    }
    if (id==0xFF || id==SC_SENSOR_LSM303DLHC_ACCEL)
    {
        ShimmerCalib_singleSensorWriteFromInfoMem(SC_SENSOR_LSM303DLHC_ACCEL,
                                                  (storedConfig[NV_CONFIG_SETUP_BYTE0] >> 2) & 0x03,
                                                  SC_DATA_LEN_LSM303DLHC_ACCEL,
                                                  &storedConfig[NV_LSM303DLHC_ACCEL_CALIBRATION]);
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
        ADC_val = *((uint16_t*) txBuff0 + (nbrAdcChans - 1) + 2);
        if (((storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x0E) >> 1) == GSR_AUTORANGE)
        {
            if (GSR_smoothTransition(
                    &current_active_resistor,
                    *(uint16_t*) (storedConfig + NV_SAMPLING_RATE)))
            {
                ADC_val = lastGsrVal;
            }
            else
                gsrActiveResistor = GSR_controlRange(ADC_val,
                                                     gsrActiveResistor);
        }
        *((uint16_t*) txBuff0 + (nbrAdcChans - 1) + 2) = ADC_val
                | (current_active_resistor << 14);
    }
    else
    {
        ADC_val = *((uint16_t*) txBuff1 + (nbrAdcChans - 1) + 2);
        if (((storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x0E) >> 1) == GSR_AUTORANGE)
        {
            if (GSR_smoothTransition(
                    &current_active_resistor,
                    *(uint16_t*) (storedConfig + NV_SAMPLING_RATE)))
            {
                ADC_val = lastGsrVal;
            }
            else
                gsrActiveResistor = GSR_controlRange(ADC_val,
                                                     gsrActiveResistor);
        }
        *((uint16_t*) txBuff1 + (nbrAdcChans - 1) + 2) = ADC_val
                | (current_active_resistor << 14);
    }

    lastGsrVal = ADC_val;
}
void ItoaWith0(uint64_t num, uint8_t *buf, uint8_t len)
{ // len = actual len + 1 extra '\0' at the end
    memset(buf, 0, len--);
    while (len--)
    {
        buf[len] = '0' + num % 10;
        num /= 10;
    }
}
void ItoaNo0(uint64_t num, uint8_t *buf, uint8_t max_len)
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

uint64_t Atol64(uint8_t *buf)
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
    P5OUT &= ~(BIT4 + BIT5);              //FLASH_SOMI and FLASH_SCLK set low
    P5DIR |= BIT4;              //FLASH_SOMI set as output
    P3SEL &= ~BIT7;
    P3OUT &= ~BIT7;              //FLASH_SIMO set low
    P4OUT &= ~BIT0;              //FLASH_CS_N set low
    P6OUT &= ~(BIT6 + BIT7);         //ADC6_FLASHDAT2 and ADC7_FLASHDAT1 set low
    P6DIR |= BIT6 + BIT7;      //ADC6_FLASHDAT2 and ADC7_FLASHDAT1 set as output

//60ms as taken from TinyOS driver (SDP.nc powerCycle() function)
    _delay_cycles(2880000);

    P5DIR &= ~(BIT4 + BIT5);            //FLASH_SOMI and FLASH_SCLK set as input
    P3DIR &= ~BIT7;              //FLASH_SIMO set as input
    P4DIR &= ~BIT0;              //FLASH_CS_N set as input
    P6DIR &= ~(BIT6 + BIT7);    //ADC6_FLASHDAT2 and ADC7_FLASHDAT1 set as input

    SdPowerOn();              //SW_FLASH set high, SdPowerOn();
    _delay_cycles(1200000);              //give SD card time to power back up
    P6OUT &= ~BIT0;              //DETECT_N set low
}

void SetupDock()
{
    shimmerStatus.configuring = 1;
    if (shimmerStatus.docked)
    {
        setBatteryInterval(BATT_INTERVAL_DOCKED);
        resetBatteryCriticalCount();

        shimmerStatus.sdlogCmd = 0;
        shimmerStatus.sdlogReady = 0;
        newDirFlag = 1;
        if (CheckSdInslot())
        {
            DockSdPowerCycle();
        }
        if (!shimmerStatus.sensing)
        {
            UART_activate();
        }
        BtsdSelfcmd();
    }
    else
    {
        setBatteryInterval(BATT_INTERVAL_UNDOCKED);
        if (!shimmerStatus.sensing)
        {
            UART_deactivate();
        }
        P6OUT |= BIT0;
        //SendStatusByte();
        BtsdSelfcmd();
        SdPowerOff();
        if (CheckSdInslot() && !shimmerStatus.sensing && !shimmerStatus.sdBadFile)
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
    shimmerStatus.configuring = 0;
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
#if BT_ENABLE_BAUD_RATE_CHANGE
    ChangeBtBaudRateFunc();
#endif
    CheckOnDefault();
    checkBtModeConfig();
}

#if BT_ENABLE_BAUD_RATE_CHANGE
/*TODO this function is updating the different stored configurations before knowing whether the job is successfully completed */
/*TODO what happens if UpdateSdConfig() is called but the Shimmer is docked - therefore having no access to the SD card */
void ChangeBtBaudRateFunc()
{
    if (!shimmerStatus.btConnected && (changeBtBaudRate != BAUD_NO_CHANGE_NEEDED) && !shimmerStatus.sensing)
    {
        storedConfig[NV_BT_COMMS_BAUD_RATE] = changeBtBaudRate;
        sdHeadText[SDH_BT_COMMS_BAUD_RATE] = changeBtBaudRate;
        InfoMem_write((void*) NV_BT_COMMS_BAUD_RATE,
                      &storedConfig[NV_BT_COMMS_BAUD_RATE], 1);
        if (storedConfig[NV_BT_COMMS_BAUD_RATE] != getCurrentBtBaudRate())
        {
            SetBtBaudRate(storedConfig[NV_BT_COMMS_BAUD_RATE]);
        }
        UpdateSdConfig();
        changeBtBaudRate = BAUD_NO_CHANGE_NEEDED;
    }
}
#endif

uint8_t SendStatusByte()
{
    if (shimmerStatus.btConnected
            && !shimmerStatus.sdSyncEnabled)
    {
        dockedResponse = 1;
        sendAck = 1;
        TaskSet(TASK_BT_RESPOND);
        return 1;
    }
    return 0;
}

void PostSdCfg(void)
{
    if (!shimmerStatus.sensing)
        ConfigureChannels();
    CalibAll();
    CheckOnDefault();
}
void ReadSdConfiguration(void)
{
    TaskClear(TASK_STREAMDATA); // this will skip one sample
    newDirFlag = 1;
    SdPowerOn();
    ParseConfig();
}
void SdPowerOff(void)
{
    P4OUT &= ~BIT2;
    shimmerStatus.sdPowerOn = 0;
} //   SD power off
void SdPowerOn(void)
{
    P4OUT |= BIT2;
    shimmerStatus.sdPowerOn = 1;
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
    InfoMem_read((uint8_t*) 0, storedConfig, NV_NUM_RWMEM_BYTES);
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
    rwcErrorFlash = ((!getRwcTimeDiff()) && rwcErrorEn) ? 1 : 0;
#endif
}

void StreamData()
{
    uint8_t adc_offset;
    uint8_t *current_buffer_ptr, *other_buffer_ptr;

    current_buffer_ptr = currentBuffer ? txBuff1 : txBuff0;
    other_buffer_ptr = currentBuffer ? txBuff0 : txBuff1;

    adc_offset = 4;

    uint8_t digi_offset = (nbrAdcChans * 2) + adc_offset;
    if (storedConfig[NV_SENSORS0] & SENSOR_GSR)
    {
        GsrRange();
    }

    // Pre-read the 9-axis chip in-case it is needed for substition on the LSM303 channels
    if (isIcm20948AccelEn && isIcm20948GyroEn)
    {
        ICM20948_getAccelAndGyro(&icm20948AccelGyroBuf[0]);
    }
    else if(isIcm20948GyroEn)
    {
        ICM20948_getGyro(&icm20948AccelGyroBuf[6U]);
    }
    else if(isIcm20948AccelEn)
    {
        ICM20948_getAccel(&icm20948AccelGyroBuf[0]);
    }

    if (storedConfig[NV_SENSORS0] & SENSOR_MPU9X50_ICM20948_GYRO)
    {
        if (isGyroInUseIcm20948())
        {
            current_buffer_ptr[digi_offset + 0U] = icm20948AccelGyroBuf[0U + 6U];
            current_buffer_ptr[digi_offset + 1U] = icm20948AccelGyroBuf[1U + 6U];
            current_buffer_ptr[digi_offset + 2U] = icm20948AccelGyroBuf[2U + 6U];
            current_buffer_ptr[digi_offset + 3U] = icm20948AccelGyroBuf[3U + 6U];
            current_buffer_ptr[digi_offset + 4U] = icm20948AccelGyroBuf[4U + 6U];
            current_buffer_ptr[digi_offset + 5U] = icm20948AccelGyroBuf[5U + 6U];
        }
        else if (isGyroInUseMpu9x50())
        {
            MPU9150_getGyro(current_buffer_ptr + digi_offset);
        }
        digi_offset += 6;
    }

    uint8_t icm20948MagBuf[ICM_MAG_RD_SIZE] = {0};
    uint8_t icm20948MagRdy = 0;
    if ((storedConfig[NV_SENSORS2] & SENSOR_MPU9X50_ICM20948_MAG)
            || (isWrAccelInUseIcm20948() && (storedConfig[NV_SENSORS0] & SENSOR_LSM303XXXX_MAG)))
    {
        if (ICM20948_isMagSampleSkipEnabled())
        {
            /* This system tries to avoid lock-up scenario in the ICM20948 Mag
             * (AK09916) in-which we see a 0.1 ms worth of repeated data samples
             * if the chip was being read from too often. */
            if(icm20948MagRdy = ICM20948_hasTimeoutPeriodPassed(*(uint32_t*)currentSampleTsTicks))
            {
                icm20948MagRdy = ICM20948_getMagAndStatus(*(uint32_t*)currentSampleTsTicks, &icm20948MagBuf[0]);
            }
        }
        else
        {
            /* Original approach in-which the status 1 register is read first
             * before reading the remaining bytes. This approach was found to
             * work fine <512Hz but after that it would cause packet loss due
             * to the length of time the I2C operations take to finish. */
            if(icm20948MagRdy = ICM20948_isMagDataRdy())
            {
                ICM20948_getMag(&icm20948MagBuf[1]);
            }
        }
    }

    if (storedConfig[NV_SENSORS1] & SENSOR_LSM303XXXX_ACCEL)
    {
        if(isWrAccelInUseIcm20948())
        {
            // Swap byte order to immitate LSM303 chip
            current_buffer_ptr[digi_offset + 0U] = icm20948AccelGyroBuf[1U];
            current_buffer_ptr[digi_offset + 1U] = icm20948AccelGyroBuf[0U];

            //Invert sign of uncalibrated Y-axis to match LSM303 chip placement
            int16_t signInvertBuffer = - ((int16_t)((icm20948AccelGyroBuf[3U] << 8) | icm20948AccelGyroBuf[2U]));
            current_buffer_ptr[digi_offset + 2U] = (signInvertBuffer >> 8) & 0xFF;
            current_buffer_ptr[digi_offset + 3U] = signInvertBuffer & 0xFF;

            current_buffer_ptr[digi_offset + 4U] = icm20948AccelGyroBuf[5U];
            current_buffer_ptr[digi_offset + 5U] = icm20948AccelGyroBuf[4U];
        }
        else
        {
            if (isWrAccelInUseLsm303dlhc())
            {
                LSM303DLHC_getAccel(current_buffer_ptr + digi_offset);
            }
            else
            {
                LSM303AHTR_getAccel(current_buffer_ptr + digi_offset);
            }
        }
        digi_offset += 6;
    }

    if (storedConfig[NV_SENSORS0] & SENSOR_LSM303XXXX_MAG)
    {
        if(isWrAccelInUseIcm20948())
        {
            if(icm20948MagRdy)
            {
                current_buffer_ptr[digi_offset + 0U] = icm20948MagBuf[ICM_MAG_IDX_XOUT_L];
                current_buffer_ptr[digi_offset + 1U] = icm20948MagBuf[ICM_MAG_IDX_XOUT_H];

                current_buffer_ptr[digi_offset + 2U] = icm20948MagBuf[ICM_MAG_IDX_YOUT_L];
                current_buffer_ptr[digi_offset + 3U] = icm20948MagBuf[ICM_MAG_IDX_YOUT_H];

                //Invert sign of uncalibrated Z-axis to match LSM303 chip placement
                int16_t signInvertBuffer = - ((int16_t)((icm20948MagBuf[ICM_MAG_IDX_ZOUT_H] << 8) | icm20948MagBuf[ICM_MAG_IDX_ZOUT_L]));
                current_buffer_ptr[digi_offset + 4U] = signInvertBuffer & 0xFF;
                current_buffer_ptr[digi_offset + 5U] = (signInvertBuffer >> 8) & 0xFF;
            }
            else
            {
                //Mag not ready, repeat last sample
                memcpy(current_buffer_ptr + digi_offset, other_buffer_ptr + digi_offset, 6);
            }
        }
        else
        {
            if (isWrAccelInUseLsm303dlhc())
            {
                LSM303DLHC_getMag(current_buffer_ptr + digi_offset);
            }
            else
            {
                LSM303AHTR_getMag(current_buffer_ptr + digi_offset);
            }
        }
        digi_offset += 6;
    }
    if (storedConfig[NV_SENSORS2] & SENSOR_MPU9X50_ICM20948_ACCEL)
    {
        if (isGyroInUseIcm20948())
        {
            current_buffer_ptr[digi_offset + 0U] = icm20948AccelGyroBuf[0U];
            current_buffer_ptr[digi_offset + 1U] = icm20948AccelGyroBuf[1U];
            current_buffer_ptr[digi_offset + 2U] = icm20948AccelGyroBuf[2U];
            current_buffer_ptr[digi_offset + 3U] = icm20948AccelGyroBuf[3U];
            current_buffer_ptr[digi_offset + 4U] = icm20948AccelGyroBuf[4U];
            current_buffer_ptr[digi_offset + 5U] = icm20948AccelGyroBuf[5U];
        }
        else if (isGyroInUseMpu9x50())
        {
            MPU9150_getAccel(current_buffer_ptr + digi_offset);
        }
        digi_offset += 6;
    }
    if (storedConfig[NV_SENSORS2] & SENSOR_MPU9X50_ICM20948_MAG)
    {
        if (isGyroInUseIcm20948())
        {
            if(icm20948MagRdy)
            {
                current_buffer_ptr[digi_offset + 0U] = icm20948MagBuf[ICM_MAG_IDX_XOUT_L];
                current_buffer_ptr[digi_offset + 1U] = icm20948MagBuf[ICM_MAG_IDX_XOUT_H];
                current_buffer_ptr[digi_offset + 2U] = icm20948MagBuf[ICM_MAG_IDX_YOUT_L];
                current_buffer_ptr[digi_offset + 3U] = icm20948MagBuf[ICM_MAG_IDX_YOUT_H];
                current_buffer_ptr[digi_offset + 4U] = icm20948MagBuf[ICM_MAG_IDX_ZOUT_L];
                current_buffer_ptr[digi_offset + 5U] = icm20948MagBuf[ICM_MAG_IDX_ZOUT_H];
            }
            else
            {
                //Mag not ready, repeat last sample
                memcpy(current_buffer_ptr + digi_offset, other_buffer_ptr + digi_offset, 6);
            }
        }
        else if (isGyroInUseMpu9x50())
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
                //Mag not ready, repeat last sample
                memcpy(current_buffer_ptr + digi_offset, other_buffer_ptr + digi_offset, 6);
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

    shimmerStatus.btStreaming = shimmerStatus.btstreamCmd && shimmerStatus.btstreamReady;
    shimmerStatus.sdLogging = shimmerStatus.sdlogCmd && shimmerStatus.sdlogReady;

//uint8_t self_stop_sensing = 0;
    if (stopLogging)
    {
        stopLogging = 0;
        if (shimmerStatus.sdLogging)
        {
            if (!shimmerStatus.btStreaming)
            {
                setStopSensing(1U);
            }
            else
            {
                shimmerStatus.sdLogging = 0;
                newDirFlag = 1;
                f_close(&dataFile);
                _delay_cycles(1200000);
                SdPowerOff();
                shimmerStatus.sdlogCmd = 0;
            }
        }
    }

    if (stopStreaming)
    {
        stopStreaming = 0;
        if (shimmerStatus.btStreaming)
        {
            shimmerStatus.btstreamCmd = 0;
            shimmerStatus.btStreaming = 0;
            if (!shimmerStatus.sdLogging)
            {
                setStopSensing(1U);
            }
        }
    }

    if (stopSensing)
    {
        setStopSensing(0);
        StopSensing();

        if (shimmerStatus.sdSyncEnabled)
        {
            BtSdSyncStop();
        }
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
        if (shimmerStatus.sdlogCmd && shimmerStatus.sdlogReady)
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

        if (shimmerStatus.btstreamCmd && shimmerStatus.btstreamReady
                && shimmerStatus.btConnected
                && !shimmerStatus.sdSyncEnabled)
        {
            uint8_t crcMode = getBtCrcMode();
            if (crcMode != CRC_OFF)
            {
                calculateCrcAndInsert(crcMode, current_buffer_ptr, digi_offset);
                digi_offset += crcMode;
            }
#if TS_BYTE3
            BT_write(current_buffer_ptr, digi_offset, SENSOR_DATA);
#else
            BT_write(current_buffer_ptr+1, digi_offset-1, SENSOR_DATA);
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

void Write2SD()
{
    if (sdBuffLen)
    {
        WriteFile(sdBuff, sdBuffLen);
        sdBuffLen = 0;
    }

    if (shimmerStatus.sdSyncEnabled)
    {
        PrepareSDBuffHead();
    }
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
    FIL cfgFile;
    if (!shimmerStatus.docked && CheckSdInslot() && !shimmerStatus.sdBadFile)
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
        float val_num;
        uint16_t val_int, val_f;
        uint32_t temp32;
        uint64_t temp64;

        uint8_t i;
        uint16_t temp16;
        resetSyncVariablesBeforeParseConfig();
        UINT bw;

        memset(shimmerName, 0, MAX_CHARS);
        memset(expIdName, 0, MAX_CHARS);
        memset(configTimeText, 0, MAX_CHARS);

        SamplingClkAssignment(&storedConfig[0]);
        ClkAssignment();
        TB0CTL = MC_0;
        TB0Start();

        char cfgname[] = "sdlog.cfg";

        if (memcmp(all0xff, storedConfig, 6))
        {
            ff_result = f_open(&cfgFile, cfgname, FA_WRITE | FA_CREATE_ALWAYS);

            //sensor0
            sprintf(buffer, "accel=%d\r\n",
                    storedConfig[NV_SENSORS0] & SENSOR_A_ACCEL ? 1 : 0);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "gyro=%d\r\n",
                    storedConfig[NV_SENSORS0] & SENSOR_MPU9X50_ICM20948_GYRO ? 1 : 0);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "mag=%d\r\n",
                    storedConfig[NV_SENSORS0] & SENSOR_LSM303XXXX_MAG ? 1 : 0);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "exg1_24bit=%d\r\n",
                    storedConfig[NV_SENSORS0] & SENSOR_EXG1_24BIT ? 1 : 0);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "exg2_24bit=%d\r\n",
                    storedConfig[NV_SENSORS0] & SENSOR_EXG2_24BIT ? 1 : 0);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "gsr=%d\r\n",
                    storedConfig[NV_SENSORS0] & SENSOR_GSR ? 1 : 0);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "extch7=%d\r\n",
                    storedConfig[NV_SENSORS0] & SENSOR_EXT_A7 ? 1 : 0);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "extch6=%d\r\n",
                    storedConfig[NV_SENSORS0] & SENSOR_EXT_A6 ? 1 : 0);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            //sensor1
            sprintf(buffer, "br_amp=%d\r\n",
                    storedConfig[NV_SENSORS1] & SENSOR_STRAIN ? 1 : 0);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "vbat=%d\r\n",
                    storedConfig[NV_SENSORS1] & SENSOR_VBATT ? 1 : 0);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "accel_d=%d\r\n",
                    storedConfig[NV_SENSORS1] & SENSOR_LSM303XXXX_ACCEL ? 1 : 0);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "extch15=%d\r\n",
                    storedConfig[NV_SENSORS1] & SENSOR_EXT_A15 ? 1 : 0);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "intch1=%d\r\n",
                    storedConfig[NV_SENSORS1] & SENSOR_INT_A1 ? 1 : 0);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "intch12=%d\r\n",
                    storedConfig[NV_SENSORS1] & SENSOR_INT_A12 ? 1 : 0);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "intch13=%d\r\n",
                    storedConfig[NV_SENSORS1] & SENSOR_INT_A13 ? 1 : 0);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            //sensor2
            sprintf(buffer, "intch14=%d\r\n",
                    storedConfig[NV_SENSORS2] & SENSOR_INT_A14 ? 1 : 0);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "accel_mpu=%d\r\n",
                    storedConfig[NV_SENSORS2] & SENSOR_MPU9X50_ICM20948_ACCEL ? 1 : 0);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "mag_mpu=%d\r\n",
                    storedConfig[NV_SENSORS2] & SENSOR_MPU9X50_ICM20948_MAG ? 1 : 0);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "exg1_16bit=%d\r\n",
                    storedConfig[NV_SENSORS2] & SENSOR_EXG1_16BIT ? 1 : 0);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "exg2_16bit=%d\r\n",
                    storedConfig[NV_SENSORS2] & SENSOR_EXG2_16BIT ? 1 : 0);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer,
                    (isBmp180InUse() ?
                            ("pres_bmp180=%d\r\n") : ("pres_bmp280=%d\r\n")),
                    storedConfig[NV_SENSORS2] & SENSOR_BMPX80_PRESSURE ? 1 : 0);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
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
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            // setup config
            sprintf(buffer, "mg_internal_rate=%d\r\n",
                    (storedConfig[NV_CONFIG_SETUP_BYTE2] >> 2) & 0x07);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "mg_range=%d\r\n",
                    (storedConfig[NV_CONFIG_SETUP_BYTE2] >> 5) & 0x07);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "acc_internal_rate=%d\r\n",
                    (storedConfig[NV_CONFIG_SETUP_BYTE0] >> 4) & 0x0f);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "accel_mpu_range=%d\r\n",
                    (storedConfig[NV_CONFIG_SETUP_BYTE3] >> 6) & 0x03);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer,
                    (isBmp180InUse() ?
                            ("pres_bmp180_prec=%d\r\n") :
                            ("pres_bmp280_prec=%d\r\n")),
                    (storedConfig[NV_CONFIG_SETUP_BYTE3] >> 4) & 0x03);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "gsr_range=%d\r\n",
                    (storedConfig[NV_CONFIG_SETUP_BYTE3] >> 1) & 0x07);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "exp_power=%d\r\n",
                    storedConfig[NV_CONFIG_SETUP_BYTE3] & EXP_POWER_ENABLE ? 1 : 0);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "gyro_range=%d\r\n",
                    (storedConfig[NV_CONFIG_SETUP_BYTE2]) & 0x03);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "gyro_samplingrate=%d\r\n",
                    storedConfig[NV_CONFIG_SETUP_BYTE1]);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "acc_range=%d\r\n",
                    (storedConfig[NV_CONFIG_SETUP_BYTE0] >> 2) & 0x03);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "acc_lpm=%d\r\n",
                    (storedConfig[NV_CONFIG_SETUP_BYTE0] >> 1) & 0x01);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "acc_hrm=%d\r\n",
                    (storedConfig[NV_CONFIG_SETUP_BYTE0]) & 0x01);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            // trial config
            sprintf(buffer,
                    "user_button_enable=%d\r\n",
                    storedConfig[NV_SD_TRIAL_CONFIG0] & SDH_USER_BUTTON_ENABLE ?
                            1 : 0);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "rtc_error_enable=%d\r\n",
                    storedConfig[NV_SD_TRIAL_CONFIG0] & SDH_RWCERROR_EN ? 1 : 0);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "sd_error_enable=%d\r\n",
                    storedConfig[NV_SD_TRIAL_CONFIG0] & SDH_SDERROR_EN ? 1 : 0);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "iammaster=%d\r\n",
                    storedConfig[NV_SD_TRIAL_CONFIG0] & SDH_IAMMASTER ? 1 : 0);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "sync=%d\r\n",
                    storedConfig[NV_SD_TRIAL_CONFIG0] & SDH_TIME_SYNC ? 1 : 0);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "bluetoothDisabled=%d\r\n",
                    storedConfig[NV_SD_TRIAL_CONFIG0] & SDH_BLUETOOTH_DISABLE ? 1 : 0);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer,
                    "low_battery_autostop=%d\r\n",
                    storedConfig[NV_SD_TRIAL_CONFIG1] & SDH_BATT_CRITICAL_CUTOFF ?
                            1 : 0);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
#if IS_SUPPORTED_TCXO
            sprintf(buffer, "tcxo=%d\r\n",
                    storedConfig[NV_SD_TRIAL_CONFIG1] & SDH_TCXO ? 1 : 0);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
#endif
            sprintf(buffer, "interval=%d\r\n", storedConfig[NV_SD_BT_INTERVAL]);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);

            temp32 = (uint32_t) storedConfig[NV_MAX_EXP_LEN_LSB];
            temp32 += ((uint32_t) storedConfig[NV_MAX_EXP_LEN_MSB]) << 8;
            maxLen = temp32 * 60;
            sprintf(buffer, "max_exp_len=%d\r\n", (uint32_t) temp32);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);

            temp16 = parseSyncEstExpLen(storedConfig[NV_EST_EXP_LEN_LSB],
                                    storedConfig[NV_EST_EXP_LEN_MSB]);
            sprintf(buffer, "est_exp_len=%d\r\n", (uint16_t) temp16);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);

            parseSyncNodeNamesFromConfig(&storedConfig[0]);
            for (i = 0; i < getSyncNodeNum(); i++)
            {
                sprintf(buffer, "node=%s\r\n", (char*) getSyncNodeNamePtrForIndex(i));
                f_write(&cfgFile, buffer, strlen(buffer), &bw);
            }

            if (memcmp(all0xff, storedConfig + NV_CENTER, 6))
            {
                parseSyncCenterNameFromConfig(&storedConfig[0]);
                sprintf(buffer, "center=%s\r\n", (char*) getSyncCenterNamePtr());
                f_write(&cfgFile, buffer, strlen(buffer), &bw);
            }

            sprintf(buffer, "singletouch=%d\r\n", (IS_SUPPORTED_SINGLE_TOUCH && storedConfig[NV_SD_TRIAL_CONFIG1] & SDH_SINGLETOUCH) ? 1 : 0);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);

            sprintf(buffer, "myid=%d\r\n", storedConfig[NV_SD_MYTRIAL_ID]);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "Nshimmer=%d\r\n", storedConfig[NV_SD_NSHIMMER]);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);

            Infomem2Names();
            sprintf(buffer, "shimmername=%s\r\n", shimmerName);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "experimentid=%s\r\n", expIdName);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "configtime=%s\r\n", configTimeText);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);

            sprintf(buffer, "baud_rate=%d\r\n",
                    storedConfig[NV_BT_COMMS_BAUD_RATE]);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
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
            f_write(&cfgFile, buffer, strlen(buffer), &bw);

            sprintf(buffer, "EXG_ADS1292R_1_CONFIG1=%d\r\n",
                    storedConfig[NV_EXG_ADS1292R_1_CONFIG1]);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "EXG_ADS1292R_1_CONFIG2=%d\r\n",
                    storedConfig[NV_EXG_ADS1292R_1_CONFIG2]);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "EXG_ADS1292R_1_LOFF=%d\r\n",
                    storedConfig[NV_EXG_ADS1292R_1_LOFF]);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "EXG_ADS1292R_1_CH1SET=%d\r\n",
                    storedConfig[NV_EXG_ADS1292R_1_CH1SET]);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "EXG_ADS1292R_1_CH2SET=%d\r\n",
                    storedConfig[NV_EXG_ADS1292R_1_CH2SET]);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "EXG_ADS1292R_1_RLD_SENS=%d\r\n",
                    storedConfig[NV_EXG_ADS1292R_1_RLD_SENS]);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "EXG_ADS1292R_1_LOFF_SENS=%d\r\n",
                    storedConfig[NV_EXG_ADS1292R_1_LOFF_SENS]);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "EXG_ADS1292R_1_LOFF_STAT=%d\r\n",
                    storedConfig[NV_EXG_ADS1292R_1_LOFF_STAT]);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "EXG_ADS1292R_1_RESP1=%d\r\n",
                    storedConfig[NV_EXG_ADS1292R_1_RESP1]);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "EXG_ADS1292R_1_RESP2=%d\r\n",
                    storedConfig[NV_EXG_ADS1292R_1_RESP2]);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "EXG_ADS1292R_2_CONFIG1=%d\r\n",
                    storedConfig[NV_EXG_ADS1292R_2_CONFIG1]);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "EXG_ADS1292R_2_CONFIG2=%d\r\n",
                    storedConfig[NV_EXG_ADS1292R_2_CONFIG2]);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "EXG_ADS1292R_2_LOFF=%d\r\n",
                    storedConfig[NV_EXG_ADS1292R_2_LOFF]);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "EXG_ADS1292R_2_CH1SET=%d\r\n",
                    storedConfig[NV_EXG_ADS1292R_2_CH1SET]);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "EXG_ADS1292R_2_CH2SET=%d\r\n",
                    storedConfig[NV_EXG_ADS1292R_2_CH2SET]);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "EXG_ADS1292R_2_RLD_SENS=%d\r\n",
                    storedConfig[NV_EXG_ADS1292R_2_RLD_SENS]);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "EXG_ADS1292R_2_LOFF_SENS=%d\r\n",
                    storedConfig[NV_EXG_ADS1292R_2_LOFF_SENS]);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "EXG_ADS1292R_2_LOFF_STAT=%d\r\n",
                    storedConfig[NV_EXG_ADS1292R_2_LOFF_STAT]);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "EXG_ADS1292R_2_RESP1=%d\r\n",
                    storedConfig[NV_EXG_ADS1292R_2_RESP1]);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);
            sprintf(buffer, "EXG_ADS1292R_2_RESP2=%d\r\n",
                    storedConfig[NV_EXG_ADS1292R_2_RESP2]);
            f_write(&cfgFile, buffer, strlen(buffer), &bw);

            ff_result = f_close(&cfgFile);

            _delay_cycles(2400000); // 100ms @ 24MHz
        }
        else
        {
            ff_result = FR_DISK_ERR;
        }
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
    switch (batteryStatus.battStat)
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

    /* ADC12SHP - Use sampling timer
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
            ADC12CTL1 += ADC12CSTARTADD_4;            //Start with ADC12MEM4
            ADC12MCTL4 = ADC12INCH_2;
        }
        else
        {
            ADC12CTL1 += ADC12CSTARTADD_1;            //Start with ADC12MEM1
            ADC12MCTL1 = ADC12INCH_2;
        }
    }
    else
    {
        ADC12CTL1 += ADC12CSTARTADD_0;                //Start with ADC12MEM0
        ADC12MCTL0 = ADC12INCH_2;
    }

    DMACTL0 |= DMA0TSEL_24;            //ADC12IFGx triggered
    DMACTL4 = DMARMWDIS;            //Read-modify-write disable
    DMA0CTL &= ~DMAIFG;

    /* DMADT_2      - Burst block transfer
     * DMADSTINCR_3 - Increment destination address
     * DMADSRINCR_3 - Increment source address
     * DMAIE        - DMA interrupt enable
     * DMAEN        - DMA Enable */
//    DMA0CTL = DMADT_2 + DMADSTINCR_3 + DMASRCINCR_3 + DMAIE
//            + ((shimmerStatus.isLogging || shimmerStatus.isStreaming) ? DMAEN : 0);
    DMA0CTL = DMADT_2 + DMADSTINCR_3 + DMASRCINCR_3 + DMAIE
            + (((shimmerStatus.sdLogging || shimmerStatus.btStreaming)
                    && storedConfig[NV_SENSORS0] & SENSOR_GSR) ? DMAEN : 0);
    DMA0SZ = 1;            //DMA0 size

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

    // Writes a value to a 20-bit SFR register located at the given16/20-bit address
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
#if IS_SUPPORTED_TCXO
        else
        {
            TB0CTL = TBSSEL_0 + MC_2 + ID__8; // use TBSSEL_0 for tcxo, ID__8:divider=8//+ TBCLR
            TB0EX0 = TBIDEX__8; // divider: 8 - Note TCXO_CLOCK will change...
        }
#endif
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

uint16_t FreqProd(uint16_t num_in)
{ // e.g. 7.5 ms: num_in=75, .25s:num_in=2500
#if IS_SUPPORTED_TCXO
    if (clockFreq == (float) MSP430_CLOCK)
    {
#endif
        return (uint16_t) ceil(
                ((float) MSP430_CLOCK) * ((float) (num_in / 10000.0)));
#if IS_SUPPORTED_TCXO
    }
    else
    {
        return (uint16_t) ceil(
                ((float) TCXO_CLOCK) * ((float) (num_in / 10000.0)));
    }
#endif
}
uint16_t FreqDiv(float num_in)
{
//    return (uint16_t) ceil(clockFreq / num_in);
//    return (uint16_t) round(((float)clockFreq) / num_in);

#if IS_SUPPORTED_TCXO
    if (clockFreq == (float) MSP430_CLOCK)
    {
#endif
        return (uint16_t) round(((float) MSP430_CLOCK) / num_in);
#if IS_SUPPORTED_TCXO
    }
    else
    {
        return (uint16_t) round(((float) TCXO_CLOCK) / num_in);
    }
#endif
}

#if BT_ENABLE_BAUD_RATE_CHANGE
void BtBaudRateChangeDone(void)
{
    // if RN4678, commit this new baud rate to EEPROM
    if (isBtDeviceRn4678())
    {
        updateBtDetailsInEeprom();
    }
}

/**
 *** Set the baud rate of the serial bus between the Bluetooth module and the MSP430
 *** 11 allowable options: 0=115.2K(default), 1=1200, 2=2400, 3=4800, 4=9600, 5=19.2K,
 *** 6=38.4K, 7=57.6K, 8=230.4K, 9=460.8K, 10=921.6K, else revert to default
 **/
void SetBtBaudRate(uint8_t rate)
{
    BT_baudRateChange_cb(BtBaudRateChangeDone);
    BT_changeBaudRateInBtModule(rate);
    _delay_cycles(2400000);
}
#endif


void eepromRead(uint16_t dataAddr, uint16_t dataSize, uint8_t *dataBuf)
{
    eepromReadWrite(dataAddr, dataSize, dataBuf, EEPROM_READ);
}

void eepromWrite(uint16_t dataAddr, uint16_t dataSize, uint8_t *dataBuf)
{
    eepromReadWrite(dataAddr, dataSize, dataBuf, EEPROM_WRITE);
}

void eepromReadWrite(uint16_t dataAddr, uint16_t dataSize, uint8_t *dataBuf,
                     enum EEPROM_RW eepromRW)
{
    bool timer_was_stopped = FALSE;

    // Spool up EEPROM and required timing peripherals
    if (TB0CTL == MC_0)            // Timer is stopped
    {
        BlinkTimerStart();
        timer_was_stopped = TRUE;
    }

    CAT24C16_init();

    // EEPROM needs to be updated with latest bt baud rate, configure here
    if (eepromRW == EEPROM_READ)
    {
        CAT24C16_read(dataAddr, dataSize, dataBuf);
    }
    else
    {
        CAT24C16_write(dataAddr, dataSize, dataBuf);
    }

    // Wind down EEPROM and required timing peripherals
    CAT24C16_powerOff();
    if (timer_was_stopped == TRUE)
    {
        BlinkTimerStop();
    }
}

uint8_t Dma0BatteryRead(void)
{
    ADC_disable();
    DMA0_disable();
//== new uart starts ==
//uartSendRspBat = 1;// don't do this for sdlog/L&S
//TaskSet(TASK_UARTRSP);
//== new uart ends ==
    return 1;
}

void ReadBatt(void)
{
    SetBattDma();
// Produces spikes in PPG data when only GSR enabled & aaccel, vbatt are off.
    __bis_SR_register(LPM3_bits + GIE);            //ACLK remains active
    TaskSet(TASK_CFGCH);

    saveBatteryVoltageAndUpdateStatus();

    // 10% Battery cutoff point - v0.9.6 onwards
    if ((storedConfig[NV_SD_TRIAL_CONFIG1] & SDH_BATT_CRITICAL_CUTOFF)
            && (batteryStatus.battStatusRaw.adcBattVal < BATT_CUTOFF_3_65VOLTS))
    {
        incrementBatteryCriticalCount();
        if (checkIfBatteryCritical() && shimmerStatus.sensing)
        {
            setStopSensing(1U);
        }
    }
}

void saveBatteryVoltageAndUpdateStatus(void)
{
    uint16_t currentBattVal = *((uint16_t*) battVal);

    //Multiplied by 2 due to voltage divider
    uint16_t battValMV = (((uint32_t)currentBattVal * 3000) >> 12) * 2;

    updateBatteryStatus(currentBattVal, battValMV, LM3658SD_STAT1 ? 1 : 0,
                        LM3658SD_STAT2 ? 1 : 0);
}

void ReadWriteSDTest(void)
{
    if (!shimmerStatus.sdBadFile)
    {
        UINT bw;
        char testFileName[] = "testFile.txt";
        uint8_t dummyData[100];
        if (!(P4OUT & BIT2))
        {
            SdPowerOn();
        }

        ff_result = f_open(&dataFile, (char*) testFileName,
        FA_WRITE | FA_CREATE_ALWAYS);
        memset(dummyData, 0xff, sizeof(dummyData) / sizeof(dummyData[0]));
        ff_result = f_write(&dataFile, dummyData,
                            sizeof(dummyData) / sizeof(dummyData[0]), &bw);
        __delay_cycles(1200000);
        ff_result = f_close(&dataFile);
        __delay_cycles(1200000);

        ff_result = f_open(&dataFile, (char*) testFileName,
        FA_READ | FA_OPEN_EXISTING);
        __delay_cycles(1200000);
        f_unlink(testFileName);
        if (ff_result != FR_OK)
        {
            shimmerStatus.sdlogReady = 0;
            shimmerStatus.sdBadFile = TRUE;
        }
        else
        {
            ff_result = f_close(&dataFile);
            __delay_cycles(1200000);
        }
    }
}

void SamplingClkAssignment(uint8_t *storedConfigPtr)
{
#if IS_SUPPORTED_TCXO
    if (storedConfigPtr[NV_SD_TRIAL_CONFIG1] & SDH_TCXO)
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
#endif
        if (clockFreq != (float) MSP430_CLOCK)
        {
            clockFreq = (float) MSP430_CLOCK;
            P4OUT &= ~BIT6;
            P4SEL &= ~BIT7;
        }
#if IS_SUPPORTED_TCXO
    }
#endif
}

uint16_t getBmpX80SamplingTimeInTicks(void)
{
    uint8_t bmpX80Precision = (storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x30) >> 4;
    uint16_t bmpX80SamplingTimeTicks = 0;

    // Setting == 0: BMP180 Ultra low power = 4.5ms (max),   BMP280 Ultra low power res = 6.4ms (max)
    // Setting == 1: BMP180 Standard = 7.5ms (max),          BMP280 Standard res = 13.3ms (max)
    // Setting == 2: BMP180 High res = 13.5ms (max),         BMP280 high res = 22.5ms (max)
    // Setting == 3: BMP180 Ultra high res = 25.5ms (max),   BMP280 Ultra high res = 43.2ms (max)

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
    uint8_t bmpX80Precision = (storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x30) >> 4;
    uint16_t bmpX80SamplingTimeTicks = 0;

    switch (bmpX80Precision)
    {
    case 0:
        bmpX80SamplingTimeTicks = (isBmp180InUse() ? clk_90_45 : clk_90_64);
        break;
    case 1:
        // Note here that the BMP180 takes <9ms to sample for this setting but the BMP280 takes >9ms
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

uint8_t setTaskNewBtCmdToProcess(void)
{
    return TaskSet(TASK_BT_PROCESS_CMD);
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
