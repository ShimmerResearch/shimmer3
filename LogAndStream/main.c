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

void Init(void);
void handleIfDockedStateOnBoot(void);
void sleepWhenNoTask(void);
void checkSetupDock(void);
void BMPX80_startMeasurement(void);
void checkStreamData(void);
void checkStartSensing(void);
void setStopSensingFlag(uint8_t state);
void setStopLoggingFlag(uint8_t state);
void setStopStreamingFlag(uint8_t state);
void detectI2cSlaves(void);
void loadSensorConfigurationAndCalibration(void);
void ProcessHwRevision(void);
void InitialiseBt(void);
void InitialiseBtAfterBoot(void);
void BlinkTimerStart(void);
void BlinkTimerStop(void);
void setBootStage(boot_stage_t bootStageNew);
void SampleTimerStart(void);
inline void SampleTimerStop(void);
void StartStreaming(void);
inline void StopSensing(void);
uint8_t Dma0ConversionDone(void);
void setMacId(uint8_t *buf);
void ConfigureChannels(void);
char *HAL_GetUID(void);
void Timestamp0ToFirstFile();
FRESULT WriteFile(uint8_t *text, WORD size);
void PrepareSDBuffHead(void);
inline void GsrRange(void);
void DockSdPowerCycle();
void SetupDock(void);
void ReadSdConfiguration(void);
void SdPowerOff(void);
void SdPowerOn(void);
void Board_setSdPower(uint8_t state);
uint8_t CheckSdInslot(void);
void BtStop(uint8_t isCalledFromMain);
void BtStart(void);
void BtStartDone();
void StreamData();
void Write2SD();
void TB0Start();
void TB0Stop();
void ChargeStatusTimerStart(void);
void ChargeStatusTimerStop(void);
void BattBlinkOn();
uint8_t Dma0BatteryRead(void);
void manageReadBatt(uint8_t isCalledFromMain);
void ClkAssignment();
uint16_t FreqProd(uint16_t samplingFreq);
float samplingClockFreqGet(void);
void RwcCheck(void);
uint8_t CheckOnDefault();
void SdInfoSync();
inline uint8_t Skip65ms();
void saveBatteryVoltageAndUpdateStatus(void);
void ReadWriteSDTest(void);
void setSamplingClkSource(float samplingClock);
uint16_t getBmpX80SamplingTimeInTicks(void);
uint16_t getBmpX80SamplingTimeDiffFrom9msInTicks(void);
void updateBtDetailsInEeprom(void);
void eepromRead(uint16_t dataAddr, uint16_t dataSize, uint8_t *dataBuf);
void eepromWrite(uint16_t dataAddr, uint16_t dataSize, uint8_t *dataBuf);
void eepromReadWrite(uint16_t dataAddr, uint16_t dataSize, uint8_t *dataBuf,
                     enum EEPROM_RW eepromRW);

uint8_t btsdSelfCmd, lastLedGroup2, rwcErrorFlash, rwcErrorEn,
        sdErrorFlash;
uint32_t buttonPressTs, buttonReleaseTs, buttonLastReleaseTs;

uint64_t buttonPressTs64, buttonReleaseTs64, buttonLastReleaseTs64;

uint8_t fwInfo[7], ackStr[4],
        channelContents[MAX_NUM_CHANNELS],
        infomemLength, calibRamLength;
uint16_t *adcStartPtr, infomemOffset, calibRamOffset;

uint8_t currentBuffer;
uint8_t stopSensing, stopLogging, stopStreaming;
uint8_t skip65ms;

uint8_t watchDogWasOnDuringBtStart;

// file system vars
FATFS fatfs;         // File object
DIRS dir;               //Directory object
FIL dataFile;
/* make dir for SDLog files*/
uint8_t firstTsFlag,
        txBuff0[DATA_PACKET_SIZE + 2], txBuff1[DATA_PACKET_SIZE + 2],
        sdBuff[SD_WRITE_BUF_SIZE], // todo: btRxBuff too long
        dirLen;
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

/*GSR*/
uint8_t gsrActiveResistor;
uint16_t lastGsrVal;
//ExG
uint8_t exgLength, exgChip, exgStartAddr; /*, exgForcedOff;*/
/*Daughter card EEPROM*/
uint8_t dcMemLength;
uint16_t dcMemOffset;

uint8_t undockSimulate;

uint8_t streamDataInProc;
char *dierecord;

/* variables used for delayed undock-start */
uint8_t undockEvent;
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

boot_stage_t bootStage;

void main(void)
{
    shimmerStatus.initialising = 1; /* led flag, in initialisation period */
    Init();

    if (!shimmerStatus.configuring && !sensing.inSdWr)
    {
        SetupDock();
    }
    manageReadBatt(1);

    /* Initialise Watchdog status timer */
    ChargeStatusTimerStart();

    while (1)
    {
        ShimTask_manage();

        if(getBtClearTxBufFlag())
        {
            setBtClearTxBufFlag(0);
            clearBtTxBuf(1U);
        }
    }
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
    setHwId(DEVICE_VER);

    // Set Vcore to accommodate for max. allowed system speed
    SetVCore(PMMCOREV_3);

    // Start 32.768kHz XTAL as ACLK
    LFXT_Start(XT1DRIVE_0);

    // Start 24MHz XTAL as MCLK and SMCLK
    XT2_Start(XT2DRIVE_2); // XT2DRIVE_2 or XTDRIVE_3 for 24MHz (userguide section 5.4.7)
    UCSCTL4 |= SELS_5 + SELM_5;   // SMCLK=MCLK=XT2

    SFRIFG1 = 0;                  // clear interrupt flag register
    SFRIE1 |= OFIE;               // enable oscillator fault interrupt enable

    ShimTask_init();
    ShimTask_set(TASK_BATT_READ);

    memset(txBuff0, 0, DATA_PACKET_SIZE);
    memset(txBuff1, 0, DATA_PACKET_SIZE);

    firstTsFlag = 0;
    skip65ms = 0;
    buttonLastReleaseTs64 = 0;
    rwcErrorEn = 0;
    rwcErrorFlash = 0;
    sdErrorFlash = 0;

    undockSimulate = 1;
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
    setStopSensingFlag(0);
    stopLogging = 0;
    stopStreaming = 0;
    streamDataInProc = 0;

    memset((uint8_t *) &shimmerStatus, 0, sizeof(STATTypeDef));

    // sd file system initiate.
    memset(sdBuff, 0, SD_WRITE_BUF_SIZE);
    ShimSdHead_reset();
    sdBuffLen = 0;

    setSamplingClkSource((float) MSP430_CLOCK);

    sampleTimerStatus = 0;
    blinkStatus = 0;

    ShimConfig_experimentLengthCntReset();
    lastGsrVal = 0;
    setGsrRangePinsAreReversed(0);

    ShimConfig_reset();
    SD_init();

    S4Sens_init();
    preSampleMpuMag = 0;
    preSampleBmpPress = 0;
    sampleBmpTemp = 0;

    setBmpInUse(BMP180_IN_USE);
    setWrAccelAndMagInUse(WR_ACCEL_AND_MAG_NONE_IN_USE);
    setGyroInUse(GYRO_NONE_IN_USE);
    setEepromIsPresent(0);

    /* variables used for delayed undock-start */
    undockEvent = 0;
    time_newUnDockEvent = 0;

    /* Variable for SR47-4 (and later) to indicate ADS clock lines are tied */
    adsClockTied = 0;

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

    resetDaughterCardId();
    eepromRead(DAUGHT_CARD_ID, CAT24C16_PAGE_SIZE, getDaughtCardIdPtr());

    ProcessHwRevision();
    shimmer_expansion_brd *expBrd = getDaughtCardId();
    Board_init_for_revision(isAds1292Present(), isRn4678PresentAndCmdModeSupport());

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
    sdErrorFlash = ShimConfig_getStoredConfig()->sdErrorEnable;

    shimmerStatus.initialising = 0;
    shimmerStatus.configuring = 0;

    setBootStage(BOOT_STAGE_END);
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

void sleepWhenNoTask(void)
{
  __bis_SR_register(LPM3_bits + GIE); /* ACLK remains active */
}

void checkSetupDock(void)
{
    if (!shimmerStatus.configuring && !sensing.inSdWr
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
            ShimTask_set(TASK_SETUP_DOCK);
        }
        undockEvent = 0;
    }
    else
    {
        ShimTask_set(TASK_SETUP_DOCK);
    }

    RwcCheck();
    sdErrorFlash = ShimConfig_getStoredConfig()->sdErrorEnable;
}

void BMPX80_startMeasurement(void)
{
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
        BMPX80_startPressMeasurement(ShimConfig_configBytePressureOversamplingRatioGet());
    }
}

void checkStreamData(void)
{
    if (streamDataInProc)
    {
        if ((!(shimmerStatus.sdlogCmd && shimmerStatus.sdlogReady))
                && (!(shimmerStatus.btstreamCmd && shimmerStatus.btstreamReady)))
        {
            btsdSelfCmd = 1;
            setStopSensingFlag(1U);
        }
        if (!sensing.isFileCreated && shimmerStatus.sdlogCmd && shimmerStatus.sdlogReady)
        {
            SD_fileInit();
            //               Timestamp0ToFirstFile();
        }
        StreamData();
        // if sensor data buffer is large enough (about 1024 bytes),
        // write it to SDcard and clear the buffer
        if (shimmerStatus.sdlogCmd && shimmerStatus.sdlogReady)
        {
            if (sdBuffLen > SD_WRITE_BUF_SIZE - blockLen)
            {
                ShimTask_set(TASK_SDWRITE);
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

void checkStartSensing(void)
{
    shimmerStatus.configuring = 1;
    if (!shimmerStatus.sensing && !batteryStatus.battCritical)
    {
        if (!sensing.isFileCreated && shimmerStatus.sdlogCmd && shimmerStatus.sdlogReady)
        {
            SD_fileInit();
            //shimmerStatus.isLogging = 1;
        }
        if ((shimmerStatus.sdlogCmd && shimmerStatus.sdlogReady)
                || (shimmerStatus.btstreamCmd && shimmerStatus.btstreamReady))
        {
            StartStreaming();
            if (shimmerStatus.sdSyncEnabled)
            {
                ShimConfig_experimentLengthCntReset();
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

void setStopSensingFlag(uint8_t state)
{
    stopSensing = state;
}

void setStopLoggingFlag(uint8_t state)
{
    stopLogging = state;
}

void setStopStreamingFlag(uint8_t state)
{
    stopStreaming = state;
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
            ShimConfig_readRam();
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
            ShimConfig_readRam();
        }
    }
    else
    {   // sd card not available
        ShimConfig_readRam();
    }

    ShimmerCalibSyncFromDumpRamAll();
}

void ProcessHwRevision(void)
{
    shimmer_expansion_brd *expBrd = getDaughtCardId();

    parseDaughterCardId();

    if (isEepromIsPresent())
    {
        // Some board batches don't have the LSM303AHTR placed, in these
        // cases, the ICM-20948's channels are used instead
        if (isSubstitutionNeededForWrAccel())
        {
            setWrAccelAndMagInUse(WR_ACCEL_AND_MAG_ICM20948_IN_USE);
        }
        else
        {
            // If the hardware is any board other then the above, overwrite the
            // special rev with 171 (used as a failsafe) to let Consensys know
            // what sensors are on-board
            if (are2ndGenSensorsPresentAndUnknownBoard())
            {
                expBrd->exp_brd_minor = 171;
            }
        }

        if(areGsrControlsPinsReversed())
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
            expBrd->exp_brd_id = SHIMMER3_IMU;
            expBrd->exp_brd_major = 6;
            expBrd->exp_brd_minor = 0;
        }
    }
}

void InitialiseBt(void)
{
    // This is the start of all BT initialisation
    /* The RN4678 operational mode need to be set before BT_init() is called so
     * that the pins are set correctly prior to communication with the module */
    if (isRn4678PresentAndCmdModeSupport())
    {
        setRn4678OperationalMode(RN4678_OP_MODE_APPLICATION);
    }
    else
    {
        setRn4678OperationalMode(RN4678_OP_MODE_NOT_USED);
    }

    btCommsProtocolInit(ShimTask_setNewBtCmdToProcess, setMacId, getBtActionPtr(), getBtArgsPtr());
    sdSyncInit(InitialiseBtAfterBoot, BtStop, ShimTask_set);
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

    BT_setUpdateBaudDuringBoot(1);
//    BT_useSpecificAdvertisingName(1U);

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

    uint8_t reset_cnt = 50U; // 50 * 100ms = 5s per baud rate attempt
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
        _delay_cycles(2400000); // 100ms

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

void StartStreaming(void)
{
    uint8_t i2cEn = 0;
    uint8_t ICMsampleRateDiv = 0;
    gConfigBytes *storedConfigPtr = ShimConfig_getStoredConfig();

    if (!shimmerStatus.sensing)
    {
#if SKIP65MS
        skip65ms = 1;
#endif
        if (shimmerStatus.docked)
        {
            UART_deactivate();
        }

        TB0CTL = MC_0; // StopTb0()
        TB0Start();

        if (storedConfigPtr->expansionBoardPower)
        { //EXT_RESET_N
            P3SEL &= ~BIT3;
            P3DIR |= BIT3;
            P3OUT |= BIT3;
        }
        if (storedConfigPtr->chEnLnAccel)
        {
            P8REN &= ~BIT6;      //disable pull down resistor
            P8DIR |= BIT6;      //set as output
            P8SEL &= ~BIT6;  //analog accel being used so take out of sleep mode
            P8OUT |= BIT6;   //analog accel being used so take out of sleep mode
        }

        if (storedConfigPtr->chEnBridgeAmp)
        {
            P2OUT |= BIT0;   //GPIO_INTERNAL1 set high
        }

        if (storedConfigPtr->chEnGsr)
        {
            GSR_init();
            if (storedConfigPtr->gsrRange <= HW_RES_3M3)
            {
                GSR_setRange(storedConfigPtr->gsrRange);
                gsrActiveResistor = storedConfigPtr->gsrRange;
            }
            else
            {
                GSR_setRange(HW_RES_40K);
                gsrActiveResistor = HW_RES_40K;
            }
        }

        if (storedConfigPtr->chEnGyro || storedConfigPtr->chEnAltAccel
                || storedConfigPtr->chEnAltMag
                || (isWrAccelInUseIcm20948()
                        && (storedConfigPtr->chEnWrAccel
                                || storedConfigPtr->chEnMag)))
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
            if (storedConfigPtr->chEnGyro || storedConfigPtr->chEnAltAccel
                    || (isWrAccelInUseIcm20948() && storedConfigPtr->chEnWrAccel))
            {
                if (isGyroInUseIcm20948())
                {
                    ICMsampleRateDiv = ICM20948_convertSampleRateDivFromMPU9X50(
                            storedConfigPtr->gyroRate, 0);

                    ICM20948_setGyroSamplingRate(ICMsampleRateDiv);
                }
                else if (isGyroInUseMpu9x50())
                {
                    MPU9150_setSamplingRate(storedConfigPtr->gyroRate);
                }
                if (storedConfigPtr->chEnGyro)
                {
                    if (isGyroInUseIcm20948())
                    {
                        ICM20948_setGyroSensitivity(ShimConfig_gyroRangeGet());
                    }
                    else if (isGyroInUseMpu9x50())
                    {
                        MPU9150_setGyroSensitivity(
                                ShimConfig_gyroRangeGet()); //This needs to go after the wake?
                    }
                }
                if (storedConfigPtr->chEnAltAccel
                        || (isWrAccelInUseIcm20948() && storedConfigPtr->chEnWrAccel))
                {
                    uint8_t wrAccelRange = storedConfigPtr->altAccelRange;
                    if (isGyroInUseIcm20948())
                    {
                        if (isWrAccelInUseIcm20948())
                        {
                            // Use setting design for WR accel - re-map to suit the ICM-20948
                            wrAccelRange = storedConfigPtr->wrAccelRange;
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
            if (storedConfigPtr->chEnAltMag
                || (isWrAccelInUseIcm20948() && storedConfigPtr->chEnMag))
            {
                if (isGyroInUseIcm20948())
                {
                    ICM20948_setMagSamplingRateFromShimmerRate(storedConfigPtr->samplingRateTicks);
                }
                else if (isGyroInUseMpu9x50())
                {
                    if (storedConfigPtr->samplingRateTicks >= clk_120)
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
                                / storedConfigPtr->samplingRateTicks)
                                + 1;
                    }
                }
            }
        }

        if (!isWrAccelInUseIcm20948()
                && (storedConfigPtr->chEnMag
                || storedConfigPtr->chEnWrAccel))
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
            if (storedConfigPtr->chEnWrAccel)
            {
                if (isWrAccelInUseLsm303dlhc())
                {
                    LSM303DLHC_accelInit(
                            storedConfigPtr->wrAccelRate, //sampling rate
                            storedConfigPtr->wrAccelRange, //range
                            storedConfigPtr->wrAccelLpModeLsb, //low power mode
                            storedConfigPtr->wrAccelHrMode); //high resolution mode
                }
                else
                {
                    LSM303AHTR_accelInit(
                            storedConfigPtr->wrAccelRate, //sampling rate
                            storedConfigPtr->wrAccelRange, //range
                            storedConfigPtr->wrAccelLpModeLsb, //low power mode
                            storedConfigPtr->wrAccelHrMode); //high resolution mode
                }
            }
            if (storedConfigPtr->chEnMag)
            {
                if (isWrAccelInUseLsm303dlhc())
                {
                    LSM303DLHC_magInit(
                            storedConfigPtr->magRateLsb, //sampling rate
                            storedConfigPtr->magRange); //gain
                }
                else
                {
                    LSM303AHTR_magInit(
                            storedConfigPtr->magRateLsb); //sampling rate
                }
            }
        }

        if (storedConfigPtr->chEnPressureAndTemperature)
        {
            uint16_t bmpX80SamplingTimeTicks = getBmpX80SamplingTimeInTicks();

            if (!i2cEn)
            {
                BMPX80_init();
            }
            if (storedConfigPtr->samplingRateTicks >= (clk_30 + bmpX80SamplingTimeTicks))
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
                        / storedConfigPtr->samplingRateTicks)
                        + 1;
            }
            //only need to sample temp once a second at most
            if (storedConfigPtr->samplingRateTicks >= clk_2500)
            {
                //less than 4Hz
                //so every second sample must be temp
                sampleBmpTemp = sampleBmpTempFreq = 1;
            }
            else
            {
                sampleBmpTemp = sampleBmpTempFreq = (uint8_t) ((FreqDiv(
                        storedConfigPtr->samplingRateTicks) - 1)
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
        if (storedConfigPtr->chEnExg1_24Bit
                || storedConfigPtr->chEnExg2_24Bit
                || storedConfigPtr->chEnExg1_16Bit
                || storedConfigPtr->chEnExg2_16Bit)
        {
            EXG_init();

            if (storedConfigPtr->chEnExg1_24Bit
                    || storedConfigPtr->chEnExg1_16Bit)
            {
                EXG_writeRegs(0, ADS1292R_CONFIG1, 10,
                              &storedConfigPtr->exgADS1292rRegsCh1.rawBytes[0]);
            }

            /* This second long delay was added to satisfy program flow requirements
             * of the ADS1292R as per pg 63 of its datasheet. */
            __delay_cycles(24000000);

            if (storedConfigPtr->chEnExg2_24Bit
                    || storedConfigPtr->chEnExg2_16Bit)
            {
                EXG_writeRegs(1, ADS1292R_CONFIG1, 10,
                              &storedConfigPtr->exgADS1292rRegsCh2.rawBytes[0]);
            }
            /* probably turning on internal reference, so wait for it to settle */
            __delay_cycles(2400000); /* 100ms (assuming 24MHz clock) */

            //probably setting the PGA gain so cancel the channel offset
            if ((storedConfigPtr->chEnExg1_24Bit
                    || storedConfigPtr->chEnExg1_16Bit)
                    && (storedConfigPtr->exgADS1292rRegsCh1.resp2 & BIT7))
            {
                EXG_offsetCal(0);
            }
            if ((storedConfigPtr->chEnExg2_24Bit
                    || storedConfigPtr->chEnExg2_16Bit)
                    && (storedConfigPtr->exgADS1292rRegsCh2.resp2 & BIT7))
            {
                EXG_offsetCal(1);
            }

            if ((storedConfigPtr->chEnExg1_24Bit
                    || storedConfigPtr->chEnExg1_16Bit)
                    && (storedConfigPtr->chEnExg2_24Bit
                            || storedConfigPtr->chEnExg2_16Bit))
            {
                EXG_start(2);
            }
            else if (storedConfigPtr->chEnExg1_24Bit
                    || storedConfigPtr->chEnExg1_16Bit)
            {
                EXG_start(0);
            }
            else
            {
                EXG_start(1);
            }

            if (areADS1292RClockLinesTied())
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
    gConfigBytes *storedConfigPtr = ShimConfig_getStoredConfig();

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
        sensing.isFileCreated = 0;
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

    if (storedConfigPtr->chEnGyro
            || storedConfigPtr->chEnAltAccel
            || storedConfigPtr->chEnAltMag
            || (isWrAccelInUseIcm20948()
                    && (storedConfigPtr->chEnWrAccel
                        || storedConfigPtr->chEnMag)))
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
            && (storedConfigPtr->chEnMag
            || storedConfigPtr->chEnWrAccel))
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

    if (storedConfigPtr->chEnExg1_24Bit
            || storedConfigPtr->chEnExg1_16Bit)
    {
        EXG_stop(0);     //probably not needed
    }
    if (storedConfigPtr->chEnExg2_24Bit
            || storedConfigPtr->chEnExg2_16Bit)
    {
        EXG_stop(1);     //probably not needed
    }
    if (!shimmerStatus.docked
            && (storedConfigPtr->chEnExg1_24Bit
                    || storedConfigPtr->chEnExg2_24Bit
                    || storedConfigPtr->chEnExg1_16Bit
                    || storedConfigPtr->chEnExg2_16Bit))
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
    ShimTask_clear(TASK_STREAMDATA);
    ShimTask_clear(TASK_SDWRITE);
    sdBuffLen = 0;
    ShimConfig_experimentLengthCntReset();
#if SKIP65MS
    skip65ms = 0;
#endif
    ShimTask_clear(TASK_SAMPLE_BMPX80_PRESS);
    ShimTask_clear(TASK_SAMPLE_MPU9150_MAG);
    if (isSdInfoSyncDelayed())
    {
        SdInfoSync();
    }
    _NOP();
    shimmerStatus.configuring = 0;
}

void ConfigureChannels(void)
{
    uint8_t *channel_contents_ptr = channelContents;
    uint16_t mask = 0;
    gConfigBytes *storedConfigPtr = ShimConfig_getStoredConfig();

    sensing.nbrAdcChans = sensing.nbrAdcChans = 0;

    //Analog Accel
    if (storedConfigPtr->chEnLnAccel)
    {
        *channel_contents_ptr++ = X_A_ACCEL;
        *channel_contents_ptr++ = Y_A_ACCEL;
        *channel_contents_ptr++ = Z_A_ACCEL;
        mask += MASK_A_ACCEL;
        sensing.nbrAdcChans += 3;
    }
    //Battery Voltage
    if (storedConfigPtr->chEnVBattery)
    {
        *channel_contents_ptr++ = VBATT;
        mask += MASK_VBATT;
        sensing.nbrAdcChans++;
    }
    //External ADC channel A7
    if (storedConfigPtr->chEnExtADC7)
    {
        *channel_contents_ptr++ = EXTERNAL_ADC_7;
        mask += MASK_EXT_A7;
        sensing.nbrAdcChans++;
    }
    //External ADC channel A6
    if (storedConfigPtr->chEnExtADC6)
    {
        *channel_contents_ptr++ = EXTERNAL_ADC_6;
        mask += MASK_EXT_A6;
        sensing.nbrAdcChans++;
    }
    //External ADC channel A15
    if (storedConfigPtr->chEnExtADC15)
    {
        *channel_contents_ptr++ = EXTERNAL_ADC_15;
        mask += MASK_EXT_A15;
        sensing.nbrAdcChans++;
    }
    //Internal ADC channel A12
    if (storedConfigPtr->chEnIntADC12)
    {
        *channel_contents_ptr++ = INTERNAL_ADC_12;
        mask += MASK_INT_A12;      //ppg
        sensing.nbrAdcChans++;
    }
    //Strain gauge
    if (storedConfigPtr->chEnBridgeAmp)
    {
        *channel_contents_ptr++ = STRAIN_HIGH;
        *channel_contents_ptr++ = STRAIN_LOW;
        mask += MASK_STRAIN;
        sensing.nbrAdcChans += 2;
    }
    //Internal ADC channel A13
    if (storedConfigPtr->chEnIntADC13 && !storedConfigPtr->chEnBridgeAmp)
    {
        *channel_contents_ptr++ = INTERNAL_ADC_13;
        mask += MASK_INT_A13;
        sensing.nbrAdcChans++;
    }
    //Internal ADC channel A14
    if (storedConfigPtr->chEnIntADC14 && !storedConfigPtr->chEnBridgeAmp)
    {
        *channel_contents_ptr++ = INTERNAL_ADC_14;
        mask += MASK_INT_A14;
        sensing.nbrAdcChans++;
    }
    //Internal ADC channel A1
    if (storedConfigPtr->chEnGsr)
    {
        //needs to be last analog channel
        *channel_contents_ptr++ = GSR_RAW;
        mask += MASK_INT_A1;
        sensing.nbrAdcChans++;
    }
    if (storedConfigPtr->chEnIntADC1)
    {
        *channel_contents_ptr++ = INTERNAL_ADC_1;
        mask += MASK_INT_A1;
        sensing.nbrAdcChans++;
    }
    //Digi Gyro
    if (storedConfigPtr->chEnGyro)
    {
        *channel_contents_ptr++ = X_MPU9150_GYRO;
        *channel_contents_ptr++ = Y_MPU9150_GYRO;
        *channel_contents_ptr++ = Z_MPU9150_GYRO;
        sensing.nbrAdcChans += 3;
    }
    //Digi Accel
    if (storedConfigPtr->chEnWrAccel)
    {
        *channel_contents_ptr++ = X_LSM303DLHC_ACCEL;
        *channel_contents_ptr++ = Y_LSM303DLHC_ACCEL;
        *channel_contents_ptr++ = Z_LSM303DLHC_ACCEL;
        sensing.nbrAdcChans += 3;
    }
    //Mag
    if (storedConfigPtr->chEnMag)
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
        sensing.nbrAdcChans += 3;
    }
    //Digi Accel
    if (storedConfigPtr->chEnAltAccel)
    {
        *channel_contents_ptr++ = X_MPU9150_ACCEL;
        *channel_contents_ptr++ = Y_MPU9150_ACCEL;
        *channel_contents_ptr++ = Z_MPU9150_ACCEL;
        sensing.nbrAdcChans += 3;
    }
    //Digi Accel
    if (storedConfigPtr->chEnAltMag)
    {
        *channel_contents_ptr++ = X_MPU9150_MAG;
        *channel_contents_ptr++ = Y_MPU9150_MAG;
        *channel_contents_ptr++ = Z_MPU9150_MAG;
        sensing.nbrAdcChans += 3;
    }

#if TS_BYTE3
    uint8_t clk_offset = 3;
#else
        uint8_t clk_offset = 2;
#endif

    blockLen = (((sensing.nbrAdcChans + sensing.nbrAdcChans) * 2) + clk_offset);

    if (storedConfigPtr->chEnPressureAndTemperature)
    {
        *channel_contents_ptr++ = BMPX80_TEMP;
        *channel_contents_ptr++ = BMPX80_PRESSURE;
        sensing.nbrAdcChans += 2;   //PRES & TEMP, ON/OFF together
        blockLen += BMPX80_PACKET_SIZE;
    }

    if (storedConfigPtr->chEnExg1_24Bit)
    {
        *channel_contents_ptr++ = EXG_ADS1292R_1_STATUS;
        *channel_contents_ptr++ = EXG_ADS1292R_1_CH1_24BIT;
        *channel_contents_ptr++ = EXG_ADS1292R_1_CH2_24BIT;
        sensing.nbrAdcChans += 3;
        blockLen += 7;
    }
    else if (storedConfigPtr->chEnExg1_16Bit)
    {
        *channel_contents_ptr++ = EXG_ADS1292R_1_STATUS;
        *channel_contents_ptr++ = EXG_ADS1292R_1_CH1_16BIT;
        *channel_contents_ptr++ = EXG_ADS1292R_1_CH2_16BIT;
        sensing.nbrAdcChans += 3;
        blockLen += 5;
    }
    if (storedConfigPtr->chEnExg2_24Bit)
    {
        *channel_contents_ptr++ = EXG_ADS1292R_2_STATUS;
        *channel_contents_ptr++ = EXG_ADS1292R_2_CH1_24BIT;
        *channel_contents_ptr++ = EXG_ADS1292R_2_CH2_24BIT;
        sensing.nbrAdcChans += 3;
        blockLen += 7;
    }
    else if (storedConfigPtr->chEnExg2_16Bit)
    {
        *channel_contents_ptr++ = EXG_ADS1292R_2_STATUS;
        *channel_contents_ptr++ = EXG_ADS1292R_2_CH1_16BIT;
        *channel_contents_ptr++ = EXG_ADS1292R_2_CH2_16BIT;
        sensing.nbrAdcChans += 3;
        blockLen += 5;
    }

    if (mask)
    {
        adcStartPtr = ADC_init(mask);
        DMA0_transferDoneFunction(&Dma0ConversionDone);
        if (adcStartPtr)
        {
            DMA0_init(adcStartPtr, (uint16_t*) (txBuff0 + 4), sensing.nbrAdcChans);
        }
    }

    isIcm20948AccelEn = FALSE;
    isIcm20948GyroEn = FALSE;
    if(isGyroInUseIcm20948() && storedConfigPtr->chEnGyro)
    {
        isIcm20948GyroEn = TRUE;
    }
    if(storedConfigPtr->chEnAltAccel
            || (isWrAccelInUseIcm20948() && storedConfigPtr->chEnWrAccel))
    {
        isIcm20948AccelEn = TRUE;
    }

    calculateClassicBtTxSampleSetBufferSize(blockLen, storedConfigPtr->samplingRateTicks);
}

uint8_t CheckOnDefault()
{
    onSingleTouch = 0;
    onUserButton = 0;
    onDefault = 0;
    if (IS_SUPPORTED_SINGLE_TOUCH && ShimConfig_getStoredConfig()->singleTouchStart)
    {
        onSingleTouch = 1;
    }
    else if (ShimConfig_getStoredConfig()->userButtonEnable)
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
        ShimTask_setStartSensing();
        //        enableSdlog = (shimmerStatus.badFile) ? 0 : 1;
        shimmerStatus.sdlogCmd = 1;
        shimmerStatus.sensing = 1;
        BtsdSelfcmd();
        shimmerStatus.sensing = 0;
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
                    ShimTask_set(TASK_SETUP_DOCK);
                    if(!shimmerStatus.sensing)
                    __bic_SR_register_on_exit(LPM3_bits);
#else
                if (ShimConfig_getStoredConfig()->userButtonEnable)
                {
                    // toggles sensing and refresh BT timers (for the centre)
                    if (shimmerStatus.sensing)
                    {
                        setStopSensingFlag(1U);
                        btsdSelfCmd = 1;
                    }
                    else
                    {
                        if (shimmerStatus.sdlogReady && (!shimmerStatus.sdBadFile))
                        {
                            //startSensing = 1;
                            ShimTask_setStartSensing();
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
        ShimTask_set(TASK_SETUP_DOCK);
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
        TB0CCR1 = GetTB0() + ShimConfig_getStoredConfig()->samplingRateTicks;
        if (ShimTask_set(TASK_SAMPLE_MPU9150_MAG))
            __bic_SR_register_on_exit(LPM3_bits);
        break;
    case 4:            // TB0CCR2
        //Bmp180 press
        TB0CCR2 = GetTB0() + ShimConfig_getStoredConfig()->samplingRateTicks;
        if (ShimTask_set(TASK_SAMPLE_BMPX80_PRESS))
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
            if (ShimConfig_checkAutostopCondition())
            {
                setStopSensingFlag(1U);
                btsdSelfCmd = 1;
            }
        }

        uint64_t batt_td, batt_my_local_time_64;
        batt_my_local_time_64 = RTC_get64();
        batt_td = batt_my_local_time_64 - battLastTs64;

        if ((batt_td > getBatteryIntervalTicks()))
        {              //10 mins = 19660800
            if (!shimmerStatus.sensing && ShimTask_set(TASK_BATT_READ))
                __bic_SR_register_on_exit(LPM3_bits);
            battLastTs64 = batt_my_local_time_64;
        }

        if (!shimmerStatus.initialising)
            if (ShimTask_getList() & TASK_BATT_READ)
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
                            && !(S4Ram_sdHeadTextGetByte(SDH_TRIAL_CONFIG0) & SDH_IAMMASTER))
                    {
                        Board_ledOn(LED_BLUE);
                    }
                    else
                    {
                        /* Flash if BT is on */
                        if (shimmerStatus.btPowerOn
                                && (blinkCnt20 == 12 || blinkCnt20 == 14))
                        {
                            Board_ledOn(LED_BLUE);
                        }
                        else
                        {
                            Board_ledOff(LED_BLUE);
                        }

//                        /* Flash twice if sync is not successfull */
//                        if (((blinkCnt20 == 12) || (blinkCnt20 == 14))
//                                && !shimmerStatus.docked
//                                && shimmerStatus.sensing
//                                && shimmerStatus.sdSyncEnabled
//                                && ((!getSyncSuccC() && (sdHeadText[SDH_TRIAL_CONFIG0] & SDH_IAMMASTER))
//                                        || (!getSyncSuccN() && !(sdHeadText[SDH_TRIAL_CONFIG0] & SDH_IAMMASTER))))
//                        {
//                            if (getSyncCnt() > 3)
//                            {
//                                Board_ledOn(LED_BLUE);
//                            }
//                            //TODO should there be an else here?
//                        }
//                        else
//                        {
//                            Board_ledOff(LED_BLUE);
//                        }
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
        resetBtRxBuff();

        shimmerStatus.btInSyncMode = shimmerStatus.sdSyncEnabled;

        BT_start();
    }
}

void BtStop(uint8_t isCalledFromMain)
{
    clearBtTxBuf(isCalledFromMain);

    ShimTask_clear(TASK_RCNODER10);

    DMA2_disable();                  //dma2 for bt disabled

    updateBtConnectionStatusInterruptDirection();
    shimmerStatus.btConnected = 0;
    shimmerStatus.btPowerOn = 0;
    shimmerStatus.btInSyncMode = 0;
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
    uint16_t baseClockOffset = val_tb0 + ShimConfig_getStoredConfig()->samplingRateTicks;
    uint8_t bmpX80Precision = ShimConfig_getStoredConfig()->pressureOversamplingRatioLsb;

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
    TB0CCR0 = timer_b0 + ShimConfig_getStoredConfig()->samplingRateTicks;

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
    if (sensing.nbrAdcChans)
    {
        DMA0_enable();
        ADC_startConversion();
    }
    else
    {
        //no analog channels, so go straight to digital
        ShimTask_set(TASK_STREAMDATA);
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
                                sensing.nbrAdcChans);
        }
        else
        {
            DMA0_repeatTransfer(adcStartPtr, (uint16_t*) (txBuff1 + adc_offset),
                                sensing.nbrAdcChans);
        }
        ADC_disable(); //can disable ADC until next time sampleTimer fires (to save power)?
        DMA0_disable();
        ShimTask_set(TASK_STREAMDATA);
    }
    return 1;
}

void setMacId(uint8_t *buf)
{
    bt_setMacId(buf);
    InfoMem_write(NV_MAC_ADDRESS, getMacIdBytesPtr(), 6);
    memcpy(&ShimConfig_getStoredConfig()->rawBytes[NV_MAC_ADDRESS], getMacIdBytesPtr(), 6);
}

//void ProcessCommand(void)
//{
//    uint32_t config_time;
//    uint8_t my_config_time[4];
//    uint8_t name_len;
//    uint8_t update_sdconfig = 0, calib_sensor = 0,
//            calib_range = 0, update_calib_dump_file = 0;
//    //uint8_t update_sdconfig_manual = 0;
////    sc_t sc1;
//    uint8_t fullSyncResp[SYNC_PACKET_MAX_SIZE] = {0};
//
//    uint8_t *sdHeadTextPtr = S4Ram_getSdHeadText();
//
//    switch (gAction)
//    {
//    case INQUIRY_COMMAND:
//        inquiryResponse = 1;
//        break;
//    case DUMMY_COMMAND:
//        break;
//    case GET_SAMPLING_RATE_COMMAND:
//        samplingRateResponse = 1;
//        break;
//    case TOGGLE_LED_COMMAND:
//        shimmerStatus.toggleLedRedCmd ^= 1;
//        break;
//    case START_STREAMING_COMMAND:
//        shimmerStatus.btstreamCmd = 1;
//        if (!shimmerStatus.sensing)
//        {
//            ShimTask_set(TASK_CFGCH);
//            //startSensing = 1;
//            SetStartSensing();
//        }
//        //enableSdlog = 0;
//        //newDirFlag = 1;
//        //CheckSdInslot();
//        break;
//    case START_SDBT_COMMAND:
//        if (!shimmerStatus.sensing)
//        {
//            ShimTask_set(TASK_CFGCH);
//            //startSensing = 1;
//            SetStartSensing();
//        }
//        shimmerStatus.btstreamCmd = 1;
//        if (shimmerStatus.sdlogReady && (!shimmerStatus.sdBadFile))
//        {
//            shimmerStatus.sdlogCmd = 1;
//        }
//        break;
//    case START_LOGGING_COMMAND:
//        if (shimmerStatus.sdlogReady && (!shimmerStatus.sdBadFile))
//        {
//            shimmerStatus.sdlogCmd = 1;
//            if (!shimmerStatus.sensing)
//            {
//                //startSensing = 1;
//                SetStartSensing();
//                ShimTask_set(TASK_CFGCH);
//            }
//        }
//        break;
//    case SET_CRC_COMMAND:
//        setBtCrcMode(args[0]);
//        break;
//
//    case SET_INSTREAM_RESPONSE_ACK_PREFIX_STATE:
//        setUseAckPrefixForInstreamResponses(args[0]);
//        break;
//
//    case STOP_STREAMING_COMMAND:
//        if (shimmerStatus.btStreaming)
//        {
//            stopStreaming = 1;
//        }
//        break;
//    case STOP_SDBT_COMMAND:
//        if (shimmerStatus.sensing)
//        {
//            setStopSensingFlag(1U);
//            //return;
//        }
//        break;
//    case STOP_LOGGING_COMMAND:
//        if (shimmerStatus.sdLogging)
//        {
//            stopLogging = 1;
//            //if(!shimmerStatus.isStreaming);
//            //return;
//        }
//        break;
//    case SET_SENSORS_COMMAND:
//        storedConfig[NV_SENSORS0] = args[0];
//        storedConfig[NV_SENSORS1] = args[1];
//        storedConfig[NV_SENSORS2] = args[2];
//        if (storedConfigPtr->chEnGsr) // they are sharing adc1, so ban intch1 when gsr is on
//            storedConfig[NV_SENSORS1] &= ~SENSOR_INT_A1;
//        InfoMem_write((void*) NV_SENSORS0, &storedConfig[NV_SENSORS0], 3);
//        sdHeadTextPtr[SDH_SENSORS0] = storedConfig[NV_SENSORS0];
//        sdHeadTextPtr[SDH_SENSORS1] = storedConfig[NV_SENSORS1];
//        sdHeadTextPtr[SDH_SENSORS2] = storedConfig[NV_SENSORS2];
//        update_sdconfig = 1;
//        ShimTask_set(TASK_CFGCH);
//        if (shimmerStatus.sensing)
//        {
//            setStopSensingFlag(1U);
//            //startSensing = 1;
//            SetStartSensing();
//        }
//        break;
//    case GET_WR_ACCEL_RANGE_COMMAND:
//        wrAccelRangeResponse = 1;
//        break;
//    case GET_MAG_GAIN_COMMAND:
//        magGainResponse = 1;
//        break;
//    case GET_MAG_SAMPLING_RATE_COMMAND:
//        magSamplingRateResponse = 1;
//        break;
//    case GET_STATUS_COMMAND:
//        dockedResponse = 1;
//        break;
//    case GET_VBATT_COMMAND:
//        btVbattResponse = 1;
//        break;
//    case GET_TRIAL_CONFIG_COMMAND:
//        trialConfigResponse = 1;
//        break;
//    case SET_TRIAL_CONFIG_COMMAND:
//        storedConfig[NV_SD_TRIAL_CONFIG0] = args[0];
//        //        storedConfig[NV_SD_TRIAL_CONFIG1] = 0;
//        storedConfig[NV_SD_TRIAL_CONFIG1] = args[1];
//#if !IS_SUPPORTED_TCXO
//        storedConfig[NV_SD_TRIAL_CONFIG1] &= ~SDH_TCXO; /* Disable TCXO */
//#endif
//        storedConfig[NV_SD_BT_INTERVAL] = args[2];
//        if (IS_SUPPORTED_SINGLE_TOUCH && storedConfig[NV_SD_TRIAL_CONFIG1] & SDH_SINGLETOUCH)
//        {
//            storedConfig[NV_SD_TRIAL_CONFIG0] |= SDH_USER_BUTTON_ENABLE;
//            storedConfig[NV_SD_TRIAL_CONFIG0] |= SDH_TIME_SYNC;
//        }
//        if (storedConfig[NV_SD_BT_INTERVAL] < SYNC_INT_C)
//            storedConfig[NV_SD_BT_INTERVAL] = SYNC_INT_C;
//        InfoMem_write((void*) NV_SAMPLING_RATE,
//                      &storedConfig[NV_SD_TRIAL_CONFIG0], 3);
//        sdHeadTextPtr[SDH_TRIAL_CONFIG0] = storedConfig[NV_SD_TRIAL_CONFIG0];
//        //        sdHeadText[SDH_TRIAL_CONFIG1] = 0;  //storedConfig[NV_SD_TRIAL_CONFIG1];
//        sdHeadTextPtr[SDH_TRIAL_CONFIG1] = storedConfig[NV_SD_TRIAL_CONFIG1];
//        sdHeadTextPtr[SDH_BROADCAST_INTERVAL] = storedConfig[NV_SD_BT_INTERVAL];
//        update_sdconfig = 1;
//        break;
//    case GET_CENTER_COMMAND:
//        centerResponse = 1;
//        break;
//    case SET_CENTER_COMMAND:
//        break;
//    case GET_SHIMMERNAME_COMMAND:
//        shimmerNameResponse = 1;
//        break;
//    case SET_SHIMMERNAME_COMMAND:
//        name_len = args[0] < (MAX_CHARS - 1) ? args[0] : (MAX_CHARS - 1);
//        memset((uint8_t*) (storedConfig + NV_SD_SHIMMER_NAME), 0,
//        MAX_CHARS - 1);
//        memcpy((uint8_t*) (storedConfig + NV_SD_SHIMMER_NAME), &args[1],
//               name_len);
//        InfoMem_write(NV_SD_SHIMMER_NAME,
//                      storedConfig + NV_SD_SHIMMER_NAME, MAX_CHARS - 1);
//        SD_setShimmerName();
//        update_sdconfig = 1;
//        break;
//    case GET_EXPID_COMMAND:
//        expIDResponse = 1;
//        break;
//    case SET_EXPID_COMMAND:
//        name_len = args[0] < (MAX_CHARS - 1) ? args[0] : (MAX_CHARS - 1);
//        memset((uint8_t*) (storedConfig + NV_SD_EXP_ID_NAME), 0, MAX_CHARS - 1);
//        memcpy((uint8_t*) (storedConfig + NV_SD_EXP_ID_NAME), &args[1],
//               name_len);
//        InfoMem_write(NV_SD_EXP_ID_NAME,
//                      storedConfig + NV_SD_EXP_ID_NAME, MAX_CHARS - 1);
//        SD_setExpIdName();
//        update_sdconfig = 1;
//        break;
//    case GET_CONFIGTIME_COMMAND:
//        configTimeResponse = 1;
//        break;
//    case GET_DIR_COMMAND:
//        dirResponse = 1;
//        break;
//    case SET_CONFIGTIME_COMMAND:
//        name_len = args[0] < (MAX_CHARS - 1) ? args[0] : (MAX_CHARS - 1);
//        memcpy(configTimeText, &args[1], name_len);
//        configTimeText[args[0]] = '\0';
//        SetName();
//        config_time = atol((char*) configTimeText);
//        my_config_time[3] = *((uint8_t*) &config_time);
//        my_config_time[2] = *(((uint8_t*) &config_time) + 1);
//        my_config_time[1] = *(((uint8_t*) &config_time) + 2);
//        my_config_time[0] = *(((uint8_t*) &config_time) + 3);
//        memcpy(sdHeadTextPtr + SDH_CONFIG_TIME_0, my_config_time, 4);
//        memcpy((uint8_t*) (storedConfig + NV_SD_CONFIG_TIME), my_config_time,
//               4);
//        InfoMem_write(NV_SD_CONFIG_TIME,
//                      storedConfig + NV_SD_CONFIG_TIME, 4);
//        update_sdconfig = 1;
//        break;
//    case GET_NSHIMMER_COMMAND:
//        nshimmerResponse = 1;
//        break;
//    case SET_NSHIMMER_COMMAND:
//        storedConfig[NV_SD_NSHIMMER] = args[0];
//        sdHeadTextPtr[SDH_NSHIMMER] = args[0];
//        InfoMem_write(NV_SD_NSHIMMER, storedConfig + NV_SD_NSHIMMER,
//                      1);
//        update_sdconfig = 1;
//        break;
//    case GET_MYID_COMMAND:
//        myIDResponse = 1;
//        break;
//    case SET_MYID_COMMAND:
//        storedConfig[NV_SD_MYTRIAL_ID] = args[0];
//        sdHeadTextPtr[SDH_MYTRIAL_ID] = args[0];
//        InfoMem_write(NV_SD_MYTRIAL_ID,
//                      storedConfig + NV_SD_MYTRIAL_ID, 1);
//        update_sdconfig = 1;
//        break;
//    case SET_WR_ACCEL_RANGE_COMMAND:
//        if (args[0] < 4)
//        {
//            storedConfig[NV_CONFIG_SETUP_BYTE0] =
//                    (storedConfig[NV_CONFIG_SETUP_BYTE0] & 0xF3)
//                            + ((args[0] & 0x03) << 2);
//        }
//        else
//        {
//            storedConfig[NV_CONFIG_SETUP_BYTE0] &= 0xF3;
//        }
//        InfoMem_write((void*) NV_CONFIG_SETUP_BYTE0,
//                      &storedConfig[NV_CONFIG_SETUP_BYTE0], 1);
//        sdHeadTextPtr[SDH_CONFIG_SETUP_BYTE0] =
//                storedConfig[NV_CONFIG_SETUP_BYTE0];
//        update_sdconfig = 1;
//        if (shimmerStatus.sensing)
//        {
//            setStopSensingFlag(1U);
//            //startSensing = 1;
//            SetStartSensing();
//        }
//        break;
//    case GET_WR_ACCEL_SAMPLING_RATE_COMMAND:
//        wrAccelSamplingRateResponse = 1;
//        break;
//    case GET_WR_ACCEL_LPMODE_COMMAND:
//        wrAccelLpModeResponse = 1;
//        break;
//    case GET_WR_ACCEL_HRMODE_COMMAND:
//        wrAccelHrModeResponse = 1;
//        break;
//    case GET_GYRO_RANGE_COMMAND:
//        gyroRangeResponse = 1;
//        break;
//    case GET_BMP180_CALIBRATION_COEFFICIENTS_COMMAND:
//    {
//        //      sc1.id = SC_SENSOR_BMPX80_PRESSURE;
//        //      sc1.range = SC_SENSOR_RANGE_BMPX80;
//        //      sc1.data_len = SC_DATA_LEN_BMPX80;
//        //      memcpy(sc1.data, bmpX80Calib, 22);
//        //      ShimmerCalib_singleSensorWrite(&sc1);
//        //      update_calib_dump_file = 1;
//        bmp180CalibrationCoefficientsResponse = 1;
//        break;
//    }
//    case GET_BMP280_CALIBRATION_COEFFICIENTS_COMMAND:
//        bmp280CalibrationCoefficientsResponse = 1;
//        break;
//    case GET_PRESSURE_CALIBRATION_COEFFICIENTS_COMMAND:
//        bmpGenericCalibrationCoefficientsResponse = 1;
//        break;
//    case GET_GYRO_SAMPLING_RATE_COMMAND:
//        gyroSamplingRateResponse = 1;
//        break;
//    case GET_ALT_ACCEL_RANGE_COMMAND:
//        altAccelRangeResponse = 1;
//        break;
//    case GET_PRESSURE_OVERSAMPLING_RATIO_COMMAND:
//        pressureOversamplingRatioResponse = 1;
//        break;
//    case GET_INTERNAL_EXP_POWER_ENABLE_COMMAND:
//        internalExpPowerEnableResponse = 1;
//        break;
//    case GET_MPU9150_MAG_SENS_ADJ_VALS_COMMAND:
//        mpu9150MagSensAdjValsResponse = 1;
//        break;
//    case GET_EXG_REGS_COMMAND:
//        if (args[0] < 2 && args[1] < 10 && args[2] < 11)
//        {
//            exgChip = args[0];
//            exgStartAddr = args[1];
//            exgLength = args[2];
//        }
//        else
//            exgLength = 0;
//        exgRegsResponse = 1;
//        break;
//    case GET_BT_VERSION_STR_COMMAND:
//        btVerResponse = 1;
//        break;
//    case SET_DATA_RATE_TEST:
//        /* Stop test before ACK is sent */
//        if (args[0] == 0)
//        {
//            setBtDataRateTestState(0);
//            clearBtTxBuf(1);
//        }
//        btDataRateTestResponse = 1;
//        break;
//    case SET_FACTORY_TEST:
//        if (args[0] < FACTORY_TEST_COUNT)
//        {
//            setup_factory_test(PRINT_TO_BT_UART, (factory_test_t) args[0]);
//            ShimTask_set(TASK_FACTORY_TEST);
//        }
//        break;
//
//    case SET_WR_ACCEL_SAMPLING_RATE_COMMAND:
//        if (args[0] < 10)
//            storedConfig[NV_CONFIG_SETUP_BYTE0] =
//                    (storedConfig[NV_CONFIG_SETUP_BYTE0] & 0x0F)
//                            + ((args[0] & 0x0F) << 4);
//        else
//            storedConfig[NV_CONFIG_SETUP_BYTE0] =
//                    (storedConfig[NV_CONFIG_SETUP_BYTE0] & 0x0F)
//                            + (LSM303DLHC_ACCEL_100HZ << 4);
//        InfoMem_write((void*) NV_CONFIG_SETUP_BYTE0,
//                      &storedConfig[NV_CONFIG_SETUP_BYTE0], 1);
//        sdHeadTextPtr[SDH_CONFIG_SETUP_BYTE0] =
//                storedConfig[NV_CONFIG_SETUP_BYTE0];
//        update_sdconfig = 1;
//        if (shimmerStatus.sensing)
//        {
//            setStopSensingFlag(1U);
//            //startSensing = 1;
//            SetStartSensing();
//        }
//        break;
//    case SET_MAG_GAIN_COMMAND:
//        if (args[0] > 0 && args[0] < 8)
//            storedConfig[NV_CONFIG_SETUP_BYTE2] =
//                    (storedConfig[NV_CONFIG_SETUP_BYTE2] & 0x1F)
//                            + ((args[0] & 0x07) << 5);
//        else
//            storedConfig[NV_CONFIG_SETUP_BYTE2] =
//                    (storedConfig[NV_CONFIG_SETUP_BYTE2] & 0x1F)
//                            + (LSM303DLHC_MAG_1_3G << 5);
//        InfoMem_write((void*) NV_CONFIG_SETUP_BYTE2,
//                      &storedConfig[NV_CONFIG_SETUP_BYTE2], 1);
//        sdHeadTextPtr[SDH_CONFIG_SETUP_BYTE2] =
//                storedConfig[NV_CONFIG_SETUP_BYTE2];
//        update_sdconfig = 1;
//        if (shimmerStatus.sensing)
//        {
//            setStopSensingFlag(1U);
//            //startSensing = 1;
//            SetStartSensing();
//        }
//        break;
//    case SET_MAG_SAMPLING_RATE_COMMAND:
//        if (args[0] < 8)
//            storedConfig[NV_CONFIG_SETUP_BYTE2] =
//                    (storedConfig[NV_CONFIG_SETUP_BYTE2] & 0xE3)
//                            + ((args[0] & 0x07) << 2);
//        else
//            storedConfig[NV_CONFIG_SETUP_BYTE2] =
//                    (storedConfig[NV_CONFIG_SETUP_BYTE2] & 0xE3)
//                            + (LSM303DLHC_MAG_75HZ << 2);
//        InfoMem_write((void*) NV_CONFIG_SETUP_BYTE2,
//                      &storedConfig[NV_CONFIG_SETUP_BYTE2], 1);
//        sdHeadTextPtr[SDH_CONFIG_SETUP_BYTE2] =
//                storedConfig[NV_CONFIG_SETUP_BYTE2];
//        update_sdconfig = 1;
//        if (shimmerStatus.sensing)
//        {
//            setStopSensingFlag(1U);
//            //startSensing = 1;
//            SetStartSensing();
//        }
//        break;
//    case SET_WR_ACCEL_LPMODE_COMMAND:
//        if (args[0] == 1)
//        {
//            storedConfig[NV_CONFIG_SETUP_BYTE0] =
//                    (storedConfig[NV_CONFIG_SETUP_BYTE0] & 0xFD) + 0x02;
//        }
//        else
//        {
//            storedConfig[NV_CONFIG_SETUP_BYTE0] &= 0xFD;
//        }
//        InfoMem_write((void*) NV_CONFIG_SETUP_BYTE0,
//                      &storedConfig[NV_CONFIG_SETUP_BYTE0], 1);
//        sdHeadTextPtr[SDH_CONFIG_SETUP_BYTE0] =
//                storedConfig[NV_CONFIG_SETUP_BYTE0];
//        update_sdconfig = 1;
//        if (shimmerStatus.sensing)
//        {
//            setStopSensingFlag(1U);
//            //startSensing = 1;
//            SetStartSensing();
//        }
//        break;
//    case SET_WR_ACCEL_HRMODE_COMMAND:
//        if (args[0] == 1)
//        {
//            storedConfig[NV_CONFIG_SETUP_BYTE0] =
//                    (storedConfig[NV_CONFIG_SETUP_BYTE0] & 0xFE) + 0x01;
//        }
//        else
//        {
//            storedConfig[NV_CONFIG_SETUP_BYTE0] &= 0xFE;
//        }
//        InfoMem_write((void*) NV_CONFIG_SETUP_BYTE0,
//                      &storedConfig[NV_CONFIG_SETUP_BYTE0], 1);
//        sdHeadTextPtr[SDH_CONFIG_SETUP_BYTE0] =
//                storedConfig[NV_CONFIG_SETUP_BYTE0];
//        update_sdconfig = 1;
//        if (shimmerStatus.sensing)
//        {
//            setStopSensingFlag(1U);
//            //startSensing = 1;
//            SetStartSensing();
//        }
//        break;
//    case SET_GYRO_RANGE_COMMAND:
//        if (args[0] < 4)
//            storedConfig[NV_CONFIG_SETUP_BYTE2] =
//                    (storedConfig[NV_CONFIG_SETUP_BYTE2] & 0xFC)
//                            + (args[0] & 0x03);
//        else
//            storedConfig[NV_CONFIG_SETUP_BYTE2] =
//                    (storedConfig[NV_CONFIG_SETUP_BYTE2] & 0xFC)
//                            + MPU9X50_GYRO_500DPS;
//        InfoMem_write((void*) NV_CONFIG_SETUP_BYTE2,
//                      &storedConfig[NV_CONFIG_SETUP_BYTE2], 1);
//        sdHeadTextPtr[SDH_CONFIG_SETUP_BYTE2] =
//                storedConfig[NV_CONFIG_SETUP_BYTE2];
//        update_sdconfig = 1;
//        if (shimmerStatus.sensing)
//        {
//            setStopSensingFlag(1U);
//            //startSensing = 1;
//            SetStartSensing();
//        }
//        break;
//    case SET_GYRO_SAMPLING_RATE_COMMAND:
//        storedConfig[NV_CONFIG_SETUP_BYTE1] = args[0];
//        InfoMem_write((void*) NV_CONFIG_SETUP_BYTE1,
//                      &storedConfig[NV_CONFIG_SETUP_BYTE1], 1);
//        sdHeadTextPtr[SDH_CONFIG_SETUP_BYTE1] =
//                storedConfig[NV_CONFIG_SETUP_BYTE1];
//        update_sdconfig = 1;
//        if (shimmerStatus.sensing)
//        {
//            setStopSensingFlag(1U);
//            //startSensing = 1;
//            SetStartSensing();
//        }
//        break;
//    case SET_ALT_ACCEL_RANGE_COMMAND:
//        if (args[0] < 4)
//            storedConfig[NV_CONFIG_SETUP_BYTE3] =
//                    (storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x3F)
//                            + ((args[0] & 0x03) << 6);
//        else
//            storedConfig[NV_CONFIG_SETUP_BYTE3] =
//                    (storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x3F)
//                            + (ACCEL_2G << 6);
//        InfoMem_write((void*) NV_CONFIG_SETUP_BYTE3,
//                      &storedConfig[NV_CONFIG_SETUP_BYTE3], 1);
//        sdHeadTextPtr[SDH_CONFIG_SETUP_BYTE3] =
//                storedConfig[NV_CONFIG_SETUP_BYTE3];
//        update_sdconfig = 1;
//        if (shimmerStatus.sensing)
//        {
//            setStopSensingFlag(1U);
//            //startSensing = 1;
//            SetStartSensing();
//        }
//        break;
//    case SET_PRESSURE_OVERSAMPLING_RATIO_COMMAND:
//        if (args[0] < 4)
//            storedConfig[NV_CONFIG_SETUP_BYTE3] =
//                    (storedConfig[NV_CONFIG_SETUP_BYTE3] & 0xCF)
//                            + ((args[0] & 0x03) << 4);
//        else
//            storedConfig[NV_CONFIG_SETUP_BYTE3] =
//                    (storedConfig[NV_CONFIG_SETUP_BYTE3] & 0xCF)
//                            + (BMPX80_OSS_1 << 4);
//        InfoMem_write((void*) NV_CONFIG_SETUP_BYTE3,
//                      &storedConfig[NV_CONFIG_SETUP_BYTE3], 1);
//        sdHeadTextPtr[SDH_CONFIG_SETUP_BYTE3] =
//                storedConfig[NV_CONFIG_SETUP_BYTE3];
//        update_sdconfig = 1;
//        break;
//    case SET_INTERNAL_EXP_POWER_ENABLE_COMMAND:
//        if (args[0] == 1)
//            storedConfig[NV_CONFIG_SETUP_BYTE3] =
//                    (storedConfig[NV_CONFIG_SETUP_BYTE3] & 0xFE)
//                            + (args[0] & 0x01);
//        else
//            storedConfig[NV_CONFIG_SETUP_BYTE3] &= 0xFE;
//        InfoMem_write((void*) NV_CONFIG_SETUP_BYTE3,
//                      &storedConfig[NV_CONFIG_SETUP_BYTE3], 1);
//        sdHeadTextPtr[SDH_CONFIG_SETUP_BYTE3] =
//                storedConfig[NV_CONFIG_SETUP_BYTE3];
//        update_sdconfig = 1;
//        if (shimmerStatus.sensing)
//        {
//            setStopSensingFlag(1U);
//            //startSensing = 1;
//            SetStartSensing();
//        }
//        break;
//    case GET_CONFIG_SETUP_BYTES_COMMAND:
//        configSetupBytesResponse = 1;
//        break;
//    case SET_CONFIG_SETUP_BYTES_COMMAND:
//        storedConfig[NV_CONFIG_SETUP_BYTE0] = args[0];
//        storedConfig[NV_CONFIG_SETUP_BYTE1] = args[1];
//        storedConfig[NV_CONFIG_SETUP_BYTE2] = args[2];
//        storedConfig[NV_CONFIG_SETUP_BYTE3] = args[3];
//        InfoMem_write((void*) NV_CONFIG_SETUP_BYTE0,
//                      &storedConfig[NV_CONFIG_SETUP_BYTE0], 4);
//        memcpy(sdHeadTextPtr + SDH_CONFIG_SETUP_BYTE0,
//               &storedConfig[NV_CONFIG_SETUP_BYTE0], 4);
//        Config2SdHead();
//        update_sdconfig = 1;
//        if (shimmerStatus.sensing)
//        {
//            setStopSensingFlag(1U);
//            //startSensing = 1;
//            SetStartSensing();
//        }
//        break;
//    case SET_SAMPLING_RATE_COMMAND:
//        storedConfigPtr->samplingRateTicks = *(uint16_t*) args;
//        InfoMem_write((void*) NV_SAMPLING_RATE, &storedConfig[NV_SAMPLING_RATE],
//                      2);
//        sdHeadTextPtr[SDH_SAMPLE_RATE_0] = storedConfig[NV_SAMPLING_RATE];
//        sdHeadTextPtr[SDH_SAMPLE_RATE_1] = storedConfig[NV_SAMPLING_RATE + 1];
//        update_sdconfig = 1;
//        if (shimmerStatus.sensing)
//        {
//            //restart sampling timer to use new sampling rate
//            setStopSensingFlag(1U);
//            //startSensing = 1;
//            SetStartSensing();
//        }
//        break;
//    case GET_CALIB_DUMP_COMMAND:
//        // usage:
//        // 0x98, offset, offset, length
//        calibRamLength = args[0];
//        calibRamOffset = args[1] + (args[2] << 8);
//        calibRamResponse = 1;
//        break;
//    case SET_CALIB_DUMP_COMMAND:
//        // usage:
//        // 0x98, offset, offset, length, data[0:127]
//        // max length of this command = 132
//        calibRamLength = args[0];
//        calibRamOffset = args[1] + (args[2] << 8);
//        if (ShimmerCalib_ramWrite(args + 3, calibRamLength, calibRamOffset)
//                == 1)
//        {
//            ShimmerCalibSyncFromDumpRamAll();
//            update_calib_dump_file = 1;
//        }
//        break;
//    case UPD_CALIB_DUMP_COMMAND:
//        ShimmerCalibSyncFromDumpRamAll();
//        update_calib_dump_file = 1;
//        break;
//    case UPD_SDLOG_CFG_COMMAND:
//        ShimTask_set(TASK_SDLOG_CFG_UPDATE);
//        //update_sdconfig_manual = 1;
//        break;
//    case SET_LN_ACCEL_CALIBRATION_COMMAND:
//        memcpy(&storedConfigPtr->lnAccelCalib.rawBytes[0], args, 21);
//        InfoMem_write((void*) NV_LN_ACCEL_CALIBRATION,
//                      &storedConfigPtr->lnAccelCalib.rawBytes[0], 21);
//        memcpy(sdHeadTextPtr + SDH_LN_ACCEL_CALIBRATION,
//               &storedConfigPtr->lnAccelCalib.rawBytes[0], 21);
//
//        CalibSaveFromInfoMemToCalibDump(SC_SENSOR_ANALOG_ACCEL);
//
//        update_calib_dump_file = 1;
//        //update_calib = 1;
//        //calib_sensor = S_ACCEL_A;
//        break;
//    case GET_LN_ACCEL_CALIBRATION_COMMAND:
//        lnAccelCalibrationResponse = 1;
//        break;
//    case SET_GYRO_CALIBRATION_COMMAND:
//        memcpy(&storedConfigPtr->gyroCalib.rawBytes[0], args, 21);
//        InfoMem_write((void*) NV_GYRO_CALIBRATION,
//                      &storedConfigPtr->gyroCalib.rawBytes[0], 21);
//        memcpy(sdHeadTextPtr + SDH_GYRO_CALIBRATION,
//               &storedConfigPtr->gyroCalib.rawBytes[0], 21);
//
//        CalibSaveFromInfoMemToCalibDump(SC_SENSOR_MPU9X50_ICM20948_GYRO);
//
//        update_calib_dump_file = 1;
//        //      update_calib = 1;
//        //      calib_sensor = S_GYRO;
//        //      calib_range = get_config_byte_gyro_range();
//        break;
//    case GET_GYRO_CALIBRATION_COMMAND:
//        gyroCalibrationResponse = 1;
//        break;
//    case SET_MAG_CALIBRATION_COMMAND:
//        memcpy(&storedConfigPtr->magCalib.rawBytes[0], args, 21);
//        InfoMem_write((void*) NV_MAG_CALIBRATION,
//                      &storedConfigPtr->magCalib.rawBytes[0], 21);
//        memcpy(sdHeadTextPtr + SDH_MAG_CALIBRATION,
//               &storedConfigPtr->magCalib.rawBytes[0], 21);
//
//        CalibSaveFromInfoMemToCalibDump(SC_SENSOR_LSM303_MAG);
//
//        update_calib_dump_file = 1;
//        //      update_calib = 1;
//        //      calib_sensor = S_MAG;
//        //      calib_range = (storedConfig[NV_CONFIG_SETUP_BYTE2]>>5) & 0x07;
//        break;
//    case GET_MAG_CALIBRATION_COMMAND:
//        magCalibrationResponse = 1;
//        break;
//    case SET_WR_ACCEL_CALIBRATION_COMMAND:
//        memcpy(&storedConfigPtr->wrAccelCalib.rawBytes[0], args, 21);
//        InfoMem_write((void*) NV_WR_ACCEL_CALIBRATION,
//                      &storedConfigPtr->wrAccelCalib.rawBytes[0], 21);
//        memcpy(sdHeadTextPtr + SDH_WR_ACCEL_CALIBRATION,
//               &storedConfigPtr->wrAccelCalib.rawBytes[0], 21);
//
//        CalibSaveFromInfoMemToCalibDump(SC_SENSOR_LSM303_ACCEL);
//
//        update_calib_dump_file = 1;
//        //      update_calib = 1;
//        //      calib_sensor = S_ACCEL_D;
//        //      calib_range = (storedConfig[NV_CONFIG_SETUP_BYTE0]>>2)&0x03;
//        break;
//    case SET_GSR_RANGE_COMMAND:
//        if (args[0] <= 4)
//            storedConfig[NV_CONFIG_SETUP_BYTE3] =
//                    (storedConfig[NV_CONFIG_SETUP_BYTE3] & 0xF1)
//                            + ((args[0] & 0x07) << 1);
//        else
//            storedConfig[NV_CONFIG_SETUP_BYTE3] =
//                    (storedConfig[NV_CONFIG_SETUP_BYTE3] & 0xF1)
//                            + (GSR_AUTORANGE << 1);
//        InfoMem_write((void*) NV_CONFIG_SETUP_BYTE3,
//                      &storedConfig[NV_CONFIG_SETUP_BYTE3], 1);
//        sdHeadTextPtr[SDH_CONFIG_SETUP_BYTE3] =
//                storedConfig[NV_CONFIG_SETUP_BYTE3];
//        update_sdconfig = 1;
//        if (shimmerStatus.sensing)
//        {
//            setStopSensingFlag(1U);
//            //startSensing = 1;
//            SetStartSensing();
//        }
//        break;
//    case SET_EXG_REGS_COMMAND:
//        if (args[0] < 2 && args[1] < 10 && args[2] < 11)
//        {
//            if (args[0])
//            {
//                memcpy((storedConfig + NV_EXG_ADS1292R_2_CONFIG1 + args[1]),
//                       (args + 3), args[2]);
//                InfoMem_write(
//                        (void*) (NV_EXG_ADS1292R_2_CONFIG1 + args[1]),
//                        (storedConfig + NV_EXG_ADS1292R_2_CONFIG1 + args[1]),
//                        args[2]);
//                memcpy(sdHeadTextPtr + SDH_EXG_ADS1292R_2_CONFIG1,
//                       storedConfig + NV_EXG_ADS1292R_2_CONFIG1, args[2]);
//            }
//            else
//            {
//                memcpy((storedConfig + NV_EXG_ADS1292R_1_CONFIG1 + args[1]),
//                       (args + 3), args[2]);
//
//                if (areADS1292RClockLinesTied())
//                {
//                    /* Check if unit is SR47-4 or greater.
//                     * If so, amend configuration byte 2 of ADS chip 1 to have bit 3 set to 1.
//                     * This ensures clock lines on ADS chip are correct
//                     */
//                    *(storedConfig + NV_EXG_ADS1292R_1_CONFIG2) |= 8;
//                }
//
//                InfoMem_write(
//                        (void*) (NV_EXG_ADS1292R_1_CONFIG1 + args[1]),
//                        (storedConfig + NV_EXG_ADS1292R_1_CONFIG1 + args[1]),
//                        args[2]);
//                memcpy(sdHeadTextPtr + SDH_EXG_ADS1292R_1_CONFIG1,
//                       storedConfig + NV_EXG_ADS1292R_1_CONFIG1, args[2]);
//            }
//            update_sdconfig = 1;
//        }
//        break;
//    case RESET_TO_DEFAULT_CONFIGURATION_COMMAND:
//        ShimConfig_setDefaultConfig();
//        Config2SdHead();
//        update_sdconfig = 1;
//        ShimTask_set(TASK_CFGCH);
//        if (shimmerStatus.sensing)
//        {
//            setStopSensingFlag(1U);
//            //startSensing = 1;
//            SetStartSensing();
//        }
//        break;
//    case RESET_CALIBRATION_VALUE_COMMAND:
//        //      memset(&storedConfig[NV_A_ACCEL_CALIBRATION], 0xFF, NV_NUM_CALIBRATION_BYTES);
//        //      InfoMem_write((void*)NV_A_ACCEL_CALIBRATION, &storedConfig[NV_A_ACCEL_CALIBRATION], NV_NUM_CALIBRATION_BYTES);
//        //      memcpy(&sdHeadText[SDH_LSM303DLHC_ACCEL_CALIBRATION], &storedConfig[NV_LSM303DLHC_ACCEL_CALIBRATION], 21);
//        //      memcpy(&sdHeadText[SDH_MPU9150_GYRO_CALIBRATION], &storedConfig[NV_MPU9150_GYRO_CALIBRATION], 21);
//        //      memcpy(&sdHeadText[SDH_LSM303DLHC_MAG_CALIBRATION], &storedConfig[NV_LSM303DLHC_MAG_CALIBRATION], 21);
//        //      memcpy(&sdHeadText[SDH_A_ACCEL_CALIBRATION], &storedConfig[NV_A_ACCEL_CALIBRATION], 21);
//        ShimmerCalib_init();
//        ShimmerCalibSyncFromDumpRamAll();
//        update_calib_dump_file = 1;
//        break;
//    case GET_WR_ACCEL_CALIBRATION_COMMAND:
//        wrAccelCalibrationResponse = 1;
//        break;
//    case GET_GSR_RANGE_COMMAND:
//        gsrRangeResponse = 1;
//        break;
//    case GET_ALL_CALIBRATION_COMMAND:
//        allCalibrationResponse = 1;
//        break;
//    case DEPRECATED_GET_DEVICE_VERSION_COMMAND:
//    case GET_DEVICE_VERSION_COMMAND:
//        deviceVersionResponse = 1;
//        break;
//    case GET_FW_VERSION_COMMAND:
//        fwVersionResponse = 1;
//        break;
//    case GET_CHARGE_STATUS_LED_COMMAND:
//        blinkLedResponse = 1;
//        break;
//    case GET_BUFFER_SIZE_COMMAND:
//        bufferSizeResponse = 1;
//        break;
//    case GET_UNIQUE_SERIAL_COMMAND:
//        uniqueSerialResponse = 1;
//        break;
//    case GET_DAUGHTER_CARD_ID_COMMAND:
//        dcMemLength = args[0];
//        dcMemOffset = args[1];
//        if ((dcMemLength <= 16) && (dcMemOffset <= 15)
//                && (dcMemLength + dcMemOffset <= 16))
//            dcIdResponse = 1;
//        break;
//    case SET_DAUGHTER_CARD_ID_COMMAND:
//        dcMemLength = args[0];
//        dcMemOffset = args[1];
//        if ((dcMemLength <= 16) && (dcMemOffset <= 15)
//                && (dcMemLength + dcMemOffset <= 16))
//        {
//            eepromReadWrite(dcMemOffset, dcMemLength, args + 2U, EEPROM_WRITE);
//        }
//        break;
//    case GET_DAUGHTER_CARD_MEM_COMMAND:
//        dcMemLength = args[0];
//        dcMemOffset = args[1] + (args[2] << 8);
//        if ((dcMemLength <= 128) && (dcMemOffset <= 2031)
//                && (dcMemLength + dcMemOffset <= 2032))
//            dcMemResponse = 1;
//        break;
//    case SET_DAUGHTER_CARD_MEM_COMMAND:
//        dcMemLength = args[0];
//        dcMemOffset = args[1] + (args[2] << 8);
//        if ((dcMemLength <= 128) && (dcMemOffset <= 2031)
//                && (dcMemLength + dcMemOffset <= 2032))
//        {
//            eepromWrite(dcMemOffset + 16U, (uint16_t) dcMemLength, args + 3U);
//        }
//        break;
//    case GET_BT_COMMS_BAUD_RATE:
//        btCommsBaudRateResponse = 1;
//        break;
//    case SET_BT_COMMS_BAUD_RATE:
//#if BT_ENABLE_BAUD_RATE_CHANGE
//        if (args[0] != storedConfig[NV_BT_COMMS_BAUD_RATE])
//        {
//            if (args[0] <= BAUD_1000000)
//            {
//                changeBtBaudRate = args[0];
//            }
//            else
//            {
//                changeBtBaudRate = DEFAULT_BT_BAUD_RATE;
//            }
//        }
//#endif
//        break;
//    case GET_DERIVED_CHANNEL_BYTES:
//        derivedChannelResponse = 1;
//        break;
//    case SET_DERIVED_CHANNEL_BYTES:
//        memcpy(&storedConfig[NV_DERIVED_CHANNELS_0], &args[0], 3);
//        memcpy(&storedConfig[NV_DERIVED_CHANNELS_3], &args[3], 5);
//        InfoMem_write((void*) NV_DERIVED_CHANNELS_0,
//                      &storedConfig[NV_DERIVED_CHANNELS_0], 3);
//        InfoMem_write((void*) NV_DERIVED_CHANNELS_3,
//                      &storedConfig[NV_DERIVED_CHANNELS_3], 5);
//        memcpy(sdHeadTextPtr + SDH_DERIVED_CHANNELS_0,
//               &storedConfig[NV_DERIVED_CHANNELS_0], 3);
//        memcpy(sdHeadTextPtr + SDH_DERIVED_CHANNELS_3,
//               &storedConfig[NV_DERIVED_CHANNELS_3], 5);
//        update_sdconfig = 1;
//        break;
//    case GET_INFOMEM_COMMAND:
//        infomemLength = args[0];
//        infomemOffset = args[1] + (args[2] << 8);
//        if ((infomemLength <= 128)
//                && (infomemOffset <= (NV_NUM_RWMEM_BYTES - 1))
//                && (infomemLength + infomemOffset <= NV_NUM_RWMEM_BYTES))
//            infomemResponse = 1;
//        break;
//    case SET_INFOMEM_COMMAND:
//    {
//        // Update shimmerStatus.badFile status bit after bt config
//        sdErrorFlash =
//                (storedConfig[NV_SD_TRIAL_CONFIG0] & SDH_SDERROR_EN) ? 1 : 0;
//        infomemLength = args[0];
//        infomemOffset = args[1] + (args[2] << 8);
//        if ((infomemLength <= 128)
//                && (infomemOffset <= (NV_NUM_RWMEM_BYTES - 1))
//                && (infomemLength + infomemOffset <= NV_NUM_RWMEM_BYTES))
//        {
//            //memcpy(storedConfig+infomemOffset, args+3, infomemLength);
//            //InfoMem_write((void*)(infomemOffset), (storedConfig+infomemOffset), infomemLength);
//
////            InfoMem_read((uint8_t *) NV_MAC_ADDRESS, macAddr, 6);
////            InfoMem_write((void*) infomemOffset, args + 3, infomemLength);
////            InfoMem_write(NV_MAC_ADDRESS, macAddr, 6);
////            InfoMem_read((uint8_t *) infomemOffset,
////                         storedConfig + infomemOffset, infomemLength);
//
//            if (infomemOffset == (INFOMEM_SEG_C_ADDR - INFOMEM_OFFSET))
//            {
//                /* Read MAC address so it is not forgotten */
//                InfoMem_read(NV_MAC_ADDRESS, getMacIdBytesPtr(), 6);
//            }
//            if (infomemOffset == (INFOMEM_SEG_D_ADDR - INFOMEM_OFFSET))
//            {
//                /* Check if unit is SR47-4 or greater.
//                 * If so, amend configuration byte 2 of ADS chip 1 to have bit 3 set to 1.
//                 * This ensures clock lines on ADS chip are correct
//                 */
//                if (areADS1292RClockLinesTied())
//                {
//                    *(args + 3 + NV_EXG_ADS1292R_1_CONFIG2) |= 8;
//                }
//            }
//#if !IS_SUPPORTED_TCXO
//            if (infomemOffset <= NV_SD_TRIAL_CONFIG1 && NV_SD_TRIAL_CONFIG1 <= infomemOffset + infomemLength)
//            {
//                uint8_t tcxoInfomemOffset = NV_SD_TRIAL_CONFIG1 - infomemOffset;
//                args[3 + tcxoInfomemOffset] &= ~SDH_TCXO;
//            }
//#endif
//
//            InfoMem_write((void*) infomemOffset, args + 3, infomemLength);
//
//            if (infomemOffset == (INFOMEM_SEG_C_ADDR - INFOMEM_OFFSET))
//            {
//                /* Re-write MAC address to Infomem */
//                InfoMem_write(NV_MAC_ADDRESS, getMacIdBytesPtr(), 6);
//            }
//
//            InfoMem_read(infomemOffset, storedConfig + infomemOffset,
//                         infomemLength);
//
//
//            if (infomemOffset == (INFOMEM_SEG_D_ADDR - INFOMEM_OFFSET))
//            {
//                CalibSaveFromInfoMemToCalibDump(0xFF);
//            }
//
//            Config2SdHead();
//            SD_infomem2Names();
//            ShimTask_set(TASK_CFGCH);
//            update_sdconfig = 1;
//            if (((infomemOffset >= NV_LN_ACCEL_CALIBRATION)
//                    && (infomemOffset <= NV_CALIBRATION_END))
//                    || (((infomemLength + infomemOffset)
//                            >= NV_LN_ACCEL_CALIBRATION)
//                            && ((infomemLength + infomemOffset)
//                                    <= NV_CALIBRATION_END))
//                    || ((infomemOffset <= NV_LN_ACCEL_CALIBRATION)
//                            && ((infomemLength + infomemOffset)
//                                    >= NV_CALIBRATION_END)))
//            {
//                //update_calib = 1;
//                ShimmerCalibUpdateFromInfoAll();
//                update_calib_dump_file = 1;
//            }
//        }
//        else
//            return;
//        break;
//    }
//    case GET_RWC_COMMAND:
//        rwcResponse = 1;
//        break;
//    case SET_RWC_COMMAND:
//        setRwcTime(&args[0]);
//        RwcCheck();
//        storedConfig[NV_SD_TRIAL_CONFIG0] |= SDH_RTC_SET_BY_BT;
//        InfoMem_write(NV_SD_TRIAL_CONFIG0,
//                      &storedConfig[NV_SD_TRIAL_CONFIG0], 1);
//        sdHeadTextPtr[SDH_TRIAL_CONFIG0] = storedConfig[NV_SD_TRIAL_CONFIG0];
//        uint64_t * rwcTimeDiffPtr = getRwcTimeDiffPtr();
//        sdHeadTextPtr[SDH_RTC_DIFF_7] = *((uint8_t*) rwcTimeDiffPtr);
//        sdHeadTextPtr[SDH_RTC_DIFF_6] = *(((uint8_t*) rwcTimeDiffPtr) + 1);
//        sdHeadTextPtr[SDH_RTC_DIFF_5] = *(((uint8_t*) rwcTimeDiffPtr) + 2);
//        sdHeadTextPtr[SDH_RTC_DIFF_4] = *(((uint8_t*) rwcTimeDiffPtr) + 3);
//        sdHeadTextPtr[SDH_RTC_DIFF_3] = *(((uint8_t*) rwcTimeDiffPtr) + 4);
//        sdHeadTextPtr[SDH_RTC_DIFF_2] = *(((uint8_t*) rwcTimeDiffPtr) + 5);
//        sdHeadTextPtr[SDH_RTC_DIFF_1] = *(((uint8_t*) rwcTimeDiffPtr) + 6);
//        sdHeadTextPtr[SDH_RTC_DIFF_0] = *(((uint8_t*) rwcTimeDiffPtr) + 7);
//        break;
//    case SET_SD_SYNC_COMMAND:
//        if (isBtModuleRunningInSyncMode() && isBtSdSyncRunning())
//        {
//            /* Reassemble full packet so that original RcNodeR10() will work without modificiation */
//            fullSyncResp[0] = gAction;
//            memcpy(&fullSyncResp[1], &args[0], SYNC_PACKET_MAX_SIZE-SYNC_PACKET_SIZE_CMD);
//            setSyncResp(&fullSyncResp[0], SYNC_PACKET_MAX_SIZE);
//            ShimTask_set(TASK_RCNODER10);
//        }
//        else
//        {
//            sendNack = 1;
//        }
//        break;
//    case ACK_COMMAND_PROCESSED:
//        if (isBtModuleRunningInSyncMode() && isBtSdSyncRunning())
//        {
//            /* Slave response received by Master */
//            if (args[0] == SD_SYNC_RESPONSE)
//            {
//                /* SD Sync Center - get's into this case when the center is waiting for a 0x01 or 0xFF from a node */
//                setSyncResp(&args[1], 1U);
//                ShimTask_set(TASK_RCCENTERR1);
//            }
//        }
//        else
//        {
//            sendNack = 1;
//        }
//        break;
//    default:
//        ;
//    }
//
//    /* Send Response back for all commands except when FW has received an ACK */
//    /* ACK is sent back as part of SD_SYNC_RESPONSE so no need to send it here */
//    if (!(gAction == ACK_COMMAND_PROCESSED || gAction == SET_SD_SYNC_COMMAND)
//            || sendNack)
//    {
//        if (sendNack == 0)
//        {
//            /* Send ACK back for all commands except when FW is sending a NACK */
//            sendAck = 1;
//        }
//        ShimTask_set(TASK_BT_RESPOND);
//    }
//
//    //   if(update_sdconfig_manual && CheckSdInslot()){
//    //      if(!shimmerStatus.isDocked){
//    //         UpdateSdConfig();
//    //         SetSdCfgFlag(0);
//    //      }else{
//    //         SetSdCfgFlag(1);
//    //         sdlogcfgUpdate = 0;
//    //      }
//    //   }
//    if (update_sdconfig)
//    {
//        SetSdCfgFlag(1);
//    }
//    if (update_calib_dump_file && CheckSdInslot() && !shimmerStatus.sdBadFile)
//    {
//        if (!shimmerStatus.docked)
//        {
//            ShimmerCalib_ram2File();
//        }
//        else
//        {
//            SetRamCalibFlag(1);
//        }
//    }
//}
//
//void SendResponse(void)
//{
//    sc_t sc1;
//    uint16_t packet_length = 0;
//
//    if (shimmerStatus.btConnected)
//    {
//        if (sendAck)
//        {
//            *(resPacket + packet_length++) = ACK_COMMAND_PROCESSED;
//            sendAck = 0;
//        }
//        if (sendNack)
//        {
//            *(resPacket + packet_length++) = NACK_COMMAND_PROCESSED;
//            sendNack = 0;
//        }
//        if (inquiryResponse)
//        {
//            *(resPacket + packet_length++) = INQUIRY_RESPONSE;
//            *(uint16_t*) (resPacket + packet_length) =
//                    storedConfigPtr->samplingRateTicks; //ADC sampling rate
//            packet_length += 2;
//            memcpy((resPacket + packet_length),
//                   (storedConfig + NV_CONFIG_SETUP_BYTE0), 4); //4 config bytes
//            packet_length += 4;
//            *(resPacket + packet_length++) = sensing.nbrAdcChans + sensing.nbrAdcChans; //number of data channels
//            *(resPacket + packet_length++) = storedConfig[NV_BUFFER_SIZE]; //buffer size
//            memcpy((resPacket + packet_length), channelContents,
//                   (sensing.nbrAdcChans + sensing.nbrAdcChans));
//            packet_length += sensing.nbrAdcChans + sensing.nbrAdcChans;
//            inquiryResponse = 0;
//        }
//        else if (samplingRateResponse)
//        {
//            *(resPacket + packet_length++) = SAMPLING_RATE_RESPONSE;
//            *(uint16_t*) (resPacket + packet_length) =
//                    storedConfigPtr->samplingRateTicks; //ADC sampling rate
//            packet_length += 2;
//            samplingRateResponse = 0;
//        }
//        else if (wrAccelRangeResponse)
//        {
//            *(resPacket + packet_length++) = WR_ACCEL_RANGE_RESPONSE;
//            *(resPacket + packet_length++) =
//                    (storedConfig[NV_CONFIG_SETUP_BYTE0] & 0x0C) >> 2;
//            wrAccelRangeResponse = 0;
//        }
//        else if (magGainResponse)
//        {
//            *(resPacket + packet_length++) = MAG_GAIN_RESPONSE;
//            *(resPacket + packet_length++) =
//                    (storedConfig[NV_CONFIG_SETUP_BYTE2] & 0xE0) >> 5;
//            magGainResponse = 0;
//        }
//        else if (magSamplingRateResponse)
//        {
//            *(resPacket + packet_length++) =
//            MAG_SAMPLING_RATE_RESPONSE;
//            *(resPacket + packet_length++) =
//                    (storedConfig[NV_CONFIG_SETUP_BYTE2] & 0x1C) >> 2;
//            magSamplingRateResponse = 0;
//        }
//        else if (dockedResponse)
//        {
//            *(resPacket + packet_length++) = INSTREAM_CMD_RESPONSE;
//            *(resPacket + packet_length++) = STATUS_RESPONSE;
//            *(resPacket + packet_length++) = ((shimmerStatus.toggleLedRedCmd & 0x01) << 7)
//                    + ((shimmerStatus.sdBadFile & 0x01) << 6)
//                    + ((shimmerStatus.sdInserted & 0x01) << 5)
//                    + ((shimmerStatus.btStreaming & 0x01) << 4)
//                    + ((shimmerStatus.sdLogging & 0x01) << 3)
//                    + (isRwcTimeSet() << 2)
//                    + ((shimmerStatus.sensing & 0x01) << 1)
//                    + (shimmerStatus.docked & 0x01);
//
//            dockedResponse = 0;
//        }
//        else if (btVbattResponse)
//        {
//            manageReadBatt();
//            *(resPacket + packet_length++) = INSTREAM_CMD_RESPONSE;
//            *(resPacket + packet_length++) = VBATT_RESPONSE;
//            memcpy((resPacket + packet_length), &batteryStatus.battStatusRaw.rawBytes[0], 3);
//            packet_length += 3;
//            btVbattResponse = 0;
//        }
//        else if (trialConfigResponse)
//        {
//            *(resPacket + packet_length++) = TRIAL_CONFIG_RESPONSE;
//            memcpy((resPacket + packet_length),
//                   (storedConfig + NV_SD_TRIAL_CONFIG0), 3); //2 trial config bytes + 1 interval byte
//            packet_length += 3;
//            trialConfigResponse = 0;
//        }
//        else if (centerResponse)
//        {
//            centerResponse = 0;
//        }
//        else if (shimmerNameResponse)
//        {
//            SD_setShimmerName();
//            uint8_t shimmer_name_len = strlen(ShimConfig_getStoredConfig()->shimmerName);
//            *(resPacket + packet_length++) = SHIMMERNAME_RESPONSE;
//            *(resPacket + packet_length++) = shimmer_name_len;
//            memcpy((resPacket + packet_length), ShimConfig_getStoredConfig()->shimmerName, shimmer_name_len);
//            packet_length += shimmer_name_len;
//            shimmerNameResponse = 0;
//        }
//        else if (expIDResponse)
//        {
//            SD_setExpIdName();
//            uint8_t exp_id_name_len = strlen(ShimConfig_getStoredConfig()->expIdName);
//            *(resPacket + packet_length++) = EXPID_RESPONSE;
//            *(resPacket + packet_length++) = exp_id_name_len;
//            memcpy((resPacket + packet_length), ShimConfig_getStoredConfig()->expIdName, exp_id_name_len);
//            packet_length += exp_id_name_len;
//            expIDResponse = 0;
//        }
//        else if (configTimeResponse)
//        {
//            SD_setCfgTime();
//            uint8_t cfgtime_name_len = strlen((char*) configTimeText);
//            *(resPacket + packet_length++) = CONFIGTIME_RESPONSE;
//            *(resPacket + packet_length++) = cfgtime_name_len;
//            memcpy((resPacket + packet_length), configTimeText,
//                   cfgtime_name_len);
//            packet_length += cfgtime_name_len;
//            configTimeResponse = 0;
//        }
//        else if (dirResponse)
//        {
//            uint8_t dir_len = strlen((char*) fileName) - 3;
//            *(resPacket + packet_length++) = INSTREAM_CMD_RESPONSE;
//            *(resPacket + packet_length++) = DIR_RESPONSE;
//            *(resPacket + packet_length++) = dir_len;
//            memcpy((resPacket + packet_length), fileName, dir_len);
//            packet_length += dir_len;
//            dirResponse = 0;
//        }
//        else if (nshimmerResponse)
//        {
//            *(resPacket + packet_length++) = NSHIMMER_RESPONSE;
//            *(resPacket + packet_length++) = storedConfig[NV_SD_NSHIMMER];
//            nshimmerResponse = 0;
//        }
//        else if (myIDResponse)
//        {
//            *(resPacket + packet_length++) = MYID_RESPONSE;
//            *(resPacket + packet_length++) = storedConfig[NV_SD_MYTRIAL_ID];
//            myIDResponse = 0;
//        }
//        else if (wrAccelSamplingRateResponse)
//        {
//            *(resPacket + packet_length++) =
//            WR_ACCEL_SAMPLING_RATE_RESPONSE;
//            *(resPacket + packet_length++) =
//                    (storedConfig[NV_CONFIG_SETUP_BYTE0] & 0xF0) >> 4;
//            wrAccelSamplingRateResponse = 0;
//        }
//        else if (wrAccelLpModeResponse)
//        {
//            *(resPacket + packet_length++) = WR_ACCEL_LPMODE_RESPONSE;
//            *(resPacket + packet_length++) =
//                    (storedConfig[NV_CONFIG_SETUP_BYTE0] & 0x02) >> 1;
//            wrAccelLpModeResponse = 0;
//        }
//        else if (wrAccelHrModeResponse)
//        {
//            *(resPacket + packet_length++) = WR_ACCEL_HRMODE_RESPONSE;
//            *(resPacket + packet_length++) = storedConfig[NV_CONFIG_SETUP_BYTE0]
//                    & 0x01;
//            wrAccelHrModeResponse = 0;
//        }
//        else if (gyroRangeResponse)
//        {
//            *(resPacket + packet_length++) = GYRO_RANGE_RESPONSE;
//            *(resPacket + packet_length++) = storedConfig[NV_CONFIG_SETUP_BYTE2]
//                    & 0x03;
//            gyroRangeResponse = 0;
//        }
//        else if (bmp180CalibrationCoefficientsResponse)
//        {
//            *(resPacket + packet_length++) =
//            BMP180_CALIBRATION_COEFFICIENTS_RESPONSE;
//
//            uint8_t *bmpX80CalibPtr = get_bmp_calib_data_bytes();
//            if (isBmp280InUse())
//            {
//                // Dummy bytes sent if incorrect calibration bytes requested.
//                memset(bmpX80CalibPtr, 0x01, BMP180_CALIB_DATA_SIZE);
//            }
//            memcpy(resPacket + packet_length, bmpX80CalibPtr,
//                   BMP180_CALIB_DATA_SIZE);
//            packet_length += BMP180_CALIB_DATA_SIZE;
//
//            bmp180CalibrationCoefficientsResponse = 0;
//        }
//        else if (bmp280CalibrationCoefficientsResponse)
//        {
//            if (isBmp280InUse())
//            {
//                *(resPacket + packet_length++) =
//                BMP280_CALIBRATION_COEFFICIENTS_RESPONSE;
//                memcpy(resPacket + packet_length, get_bmp_calib_data_bytes(),
//                BMP280_CALIB_DATA_SIZE);
//                packet_length += BMP280_CALIB_DATA_SIZE;
//            }
//            bmp280CalibrationCoefficientsResponse = 0;
//        }
//        else if (bmpGenericCalibrationCoefficientsResponse)
//        {
//            uint8_t bmpCalibByteLen = get_bmp_calib_data_bytes_len();
//            *(resPacket + packet_length++) = PRESSURE_CALIBRATION_COEFFICIENTS_RESPONSE;
//            *(resPacket + packet_length++) = 1U + bmpCalibByteLen;
//            if (isBmp180InUse())
//            {
//                *(resPacket + packet_length++) = PRESSURE_SENSOR_BMP180;
//            }
//            else if (isBmp280InUse())
//            {
//                *(resPacket + packet_length++) = PRESSURE_SENSOR_BMP280;
//            }
//            else
//            {
//                *(resPacket + packet_length++) = PRESSURE_SENSOR_BMP390;
//            }
//            memcpy(resPacket + packet_length, get_bmp_calib_data_bytes(), bmpCalibByteLen);
//            packet_length += bmpCalibByteLen;
//            bmpGenericCalibrationCoefficientsResponse = 0;
//        }
//        else if (gyroSamplingRateResponse)
//        {
//            *(resPacket + packet_length++) = GYRO_SAMPLING_RATE_RESPONSE;
//            *(resPacket + packet_length++) =
//                    storedConfig[NV_CONFIG_SETUP_BYTE1];
//            gyroSamplingRateResponse = 0;
//        }
//        else if (altAccelRangeResponse)
//        {
//            *(resPacket + packet_length++) = ALT_ACCEL_RANGE_RESPONSE;
//            *(resPacket + packet_length++) =
//                    (storedConfig[NV_CONFIG_SETUP_BYTE3] & 0xC0) >> 6;
//            altAccelRangeResponse = 0;
//        }
//        else if (pressureOversamplingRatioResponse)
//        {
//            *(resPacket + packet_length++) =
//            PRESSURE_OVERSAMPLING_RATIO_RESPONSE;
//            *(resPacket + packet_length++) =
//                    (storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x30) >> 4;
//            pressureOversamplingRatioResponse = 0;
//        }
//        else if (internalExpPowerEnableResponse)
//        {
//            *(resPacket + packet_length++) = INTERNAL_EXP_POWER_ENABLE_RESPONSE;
//            *(resPacket + packet_length++) = storedConfig[NV_CONFIG_SETUP_BYTE3]
//                    & 0x01;
//            internalExpPowerEnableResponse = 0;
//        }
//        else if (configSetupBytesResponse)
//        {
//            *(resPacket + packet_length++) = CONFIG_SETUP_BYTES_RESPONSE;
//            memcpy((resPacket + packet_length),
//                   &storedConfig[NV_CONFIG_SETUP_BYTE0], 4);
//            packet_length += 4;
//            configSetupBytesResponse = 0;
//        }
//        else if (calibRamResponse)
//        {
//            *(resPacket + packet_length++) = RSP_CALIB_DUMP_COMMAND;
//            *(resPacket + packet_length++) = calibRamLength;
//            *(resPacket + packet_length++) = calibRamOffset & 0xff;
//            *(resPacket + packet_length++) = (calibRamOffset >> 8) & 0xff;
//            ShimmerCalib_ramRead(resPacket + packet_length, calibRamLength,
//                                 calibRamOffset);
//            packet_length += calibRamLength;
//            calibRamResponse = 0;
//        }
//        else if (lnAccelCalibrationResponse)
//        {
//            *(resPacket + packet_length++) = LN_ACCEL_CALIBRATION_RESPONSE;
//            sc1.id = SC_SENSOR_ANALOG_ACCEL;
//            sc1.range = SC_SENSOR_RANGE_ANALOG_ACCEL;
//            sc1.data_len = SC_DATA_LEN_ANALOG_ACCEL;
//            ShimmerCalib_singleSensorRead(&sc1);
//            memcpy((resPacket + packet_length), sc1.data.raw, sc1.data_len);
//            packet_length += sc1.data_len;
//            lnAccelCalibrationResponse = 0;
//        }
//        else if (gyroCalibrationResponse)
//        {
//            *(resPacket + packet_length++) = GYRO_CALIBRATION_RESPONSE;
//            sc1.id = SC_SENSOR_MPU9X50_ICM20948_GYRO;
//            sc1.range = get_config_byte_gyro_range();
//            sc1.data_len = SC_DATA_LEN_MPU9X50_ICM20948_GYRO;
//            ShimmerCalib_singleSensorRead(&sc1);
//            memcpy((resPacket + packet_length), sc1.data.raw, sc1.data_len);
//            packet_length += sc1.data_len;
//            gyroCalibrationResponse = 0;
//        }
//        else if (magCalibrationResponse)
//        {
//            *(resPacket + packet_length++) =
//            MAG_CALIBRATION_RESPONSE;
//            sc1.id = SC_SENSOR_LSM303_MAG;
//            sc1.range = (storedConfig[NV_CONFIG_SETUP_BYTE2] >> 5) & 0x07;
//            sc1.data_len = SC_DATA_LEN_LSM303_MAG;
//            ShimmerCalib_singleSensorRead(&sc1);
//            memcpy((resPacket + packet_length), sc1.data.raw, sc1.data_len);
//            packet_length += sc1.data_len;
//            magCalibrationResponse = 0;
//        }
//        else if (wrAccelCalibrationResponse)
//        {
//            *(resPacket + packet_length++) =
//            WR_ACCEL_CALIBRATION_RESPONSE;
//            sc1.id = SC_SENSOR_LSM303_ACCEL;
//            sc1.range = (storedConfig[NV_CONFIG_SETUP_BYTE0] >> 2) & 0x03;
//            sc1.data_len = SC_DATA_LEN_LSM303_ACCEL;
//            ShimmerCalib_singleSensorRead(&sc1);
//            memcpy((resPacket + packet_length), sc1.data.raw, sc1.data_len);
//            packet_length += sc1.data_len;
//            wrAccelCalibrationResponse = 0;
//        }
//        else if (gsrRangeResponse)
//        {
//            *(resPacket + packet_length++) = GSR_RANGE_RESPONSE;
//            *(resPacket + packet_length++) =
//                    (storedConfig[NV_CONFIG_SETUP_BYTE3] & 0x0E) >> 1;
//            gsrRangeResponse = 0;
//        }
//        else if (allCalibrationResponse)
//        {
//            *(resPacket + packet_length++) = ALL_CALIBRATION_RESPONSE;
//            //         memcpy((resPacket+packet_length), &storedConfig[NV_A_ACCEL_CALIBRATION], NV_NUM_CALIBRATION_BYTES);
//            //         packet_length += NV_NUM_CALIBRATION_BYTES;
//            uint8_t i;
//            for (i = 0; i < 4; i++)
//            {
//                if (i == 0)
//                {
//                    sc1.id = SC_SENSOR_ANALOG_ACCEL;
//                    sc1.range = 0;
//                    sc1.data_len = SC_DATA_LEN_ANALOG_ACCEL;
//                }
//                else if (i == 1)
//                {
//                    sc1.id = SC_SENSOR_MPU9X50_ICM20948_GYRO;
//                    sc1.range = get_config_byte_gyro_range();
//                    sc1.data_len = SC_DATA_LEN_MPU9X50_ICM20948_GYRO;
//                }
//                else if (i == 2)
//                {
//                    sc1.id = SC_SENSOR_LSM303_MAG;
//                    sc1.range = (storedConfig[NV_CONFIG_SETUP_BYTE2] >> 5)
//                            & 0x07;
//                    sc1.data_len = SC_DATA_LEN_LSM303_MAG;
//                }
//                else if (i == 3)
//                {
//                    sc1.id = SC_SENSOR_LSM303_ACCEL;
//                    sc1.range = (storedConfig[NV_CONFIG_SETUP_BYTE0] >> 2)
//                            & 0x03;
//                    sc1.data_len = SC_DATA_LEN_LSM303_ACCEL;
//                }
//                ShimmerCalib_singleSensorRead(&sc1);
//                memcpy((resPacket + packet_length), sc1.data.raw, sc1.data_len);
//                packet_length += sc1.data_len;
//            }
//
//            allCalibrationResponse = 0;
//        }
//        else if (deviceVersionResponse)
//        {
//            *(resPacket + packet_length++) = DEVICE_VERSION_RESPONSE;
//            *(resPacket + packet_length++) = DEVICE_VER;
//            deviceVersionResponse = 0;
//        }
//        else if (mpu9150MagSensAdjValsResponse)
//        {
//            // Mag sensitivity adj feature is not present in ICM-20948
//            if(isGyroInUseMpu9x50())
//            {
//                MPU9150_init();
//                MPU9150_wake(1);
//                MPU9150_wake(0);
//                *(resPacket + packet_length++) = MPU9150_MAG_SENS_ADJ_VALS_RESPONSE;
//                MPU9150_getMagSensitivityAdj(resPacket + packet_length);
//                packet_length += 3;
//            }
//            else
//            {
//                *(resPacket + packet_length++) = ACK_COMMAND_PROCESSED;
//            }
//            mpu9150MagSensAdjValsResponse = 0;
//        }
//        else if (fwVersionResponse)
//        {
//            *(resPacket + packet_length++) = FW_VERSION_RESPONSE;
//            *(resPacket + packet_length++) = FW_IDENTIFIER & 0xFF;
//            *(resPacket + packet_length++) = (FW_IDENTIFIER & 0xFF00) >> 8;
//            *(resPacket + packet_length++) = FW_VER_MAJOR & 0xFF;
//            *(resPacket + packet_length++) = (FW_VER_MAJOR & 0xFF00) >> 8;
//            *(resPacket + packet_length++) = FW_VER_MINOR;
//            *(resPacket + packet_length++) = FW_VER_REL;
//            fwVersionResponse = 0;
//        }
//        else if (blinkLedResponse)
//        {
//            *(resPacket + packet_length++) = CHARGE_STATUS_LED_RESPONSE;
//            *(resPacket + packet_length++) = batteryStatus.battStat;
//            blinkLedResponse = 0;
//        }
//        else if (bufferSizeResponse)
//        {
//            *(resPacket + packet_length++) = BUFFER_SIZE_RESPONSE;
//            *(resPacket + packet_length++) = storedConfig[NV_BUFFER_SIZE];
//            bufferSizeResponse = 0;
//        }
//        else if (uniqueSerialResponse)
//        {
//            *(resPacket + packet_length++) = UNIQUE_SERIAL_RESPONSE;
//            memcpy((resPacket + packet_length), dierecord, 8);
//            packet_length += 8;
//            uniqueSerialResponse = 0;
//        }
//        else if (exgRegsResponse)
//        {
//            *(resPacket + packet_length++) = EXG_REGS_RESPONSE;
//            *(resPacket + packet_length++) = exgLength;
//            if (exgLength)
//            {
//                if (exgChip)
//                    memcpy((resPacket + packet_length),
//                           (storedConfig + NV_EXG_ADS1292R_2_CONFIG1
//                                   + exgStartAddr),
//                           exgLength);
//                else
//                    memcpy((resPacket + packet_length),
//                           (storedConfig + NV_EXG_ADS1292R_1_CONFIG1
//                                   + exgStartAddr),
//                           exgLength);
//                packet_length += exgLength;
//            }
//            exgRegsResponse = 0;
//        }
//        else if (dcIdResponse)
//        {
//            *(resPacket + packet_length++) = DAUGHTER_CARD_ID_RESPONSE;
//            *(resPacket + packet_length++) = dcMemLength;
//            eepromRead(dcMemOffset, dcMemLength, resPacket + packet_length);
//            packet_length += dcMemLength;
//            dcIdResponse = 0;
//        }
//        else if (dcMemResponse)
//        {
//            *(resPacket + packet_length++) = DAUGHTER_CARD_MEM_RESPONSE;
//            *(resPacket + packet_length++) = dcMemLength;
//            if (!shimmerStatus.sensing)
//            {
//                eepromRead(dcMemOffset + 16U, dcMemLength, resPacket + packet_length);
//            }
//            else
//            {
//                memset(resPacket + packet_length, 0xff, dcMemLength);
//            }
//            packet_length += dcMemLength;
//            dcMemResponse = 0;
//        }
//        else if (btCommsBaudRateResponse)
//        {
//            *(resPacket + packet_length++) = BT_COMMS_BAUD_RATE_RESPONSE;
//            *(resPacket + packet_length++) =
//                    storedConfig[NV_BT_COMMS_BAUD_RATE];
//            btCommsBaudRateResponse = 0;
//        }
//        else if (infomemResponse)
//        {
//            *(resPacket + packet_length++) = INFOMEM_RESPONSE;
//            *(resPacket + packet_length++) = infomemLength;
//            memcpy((resPacket + packet_length), &storedConfig[infomemOffset],
//                   infomemLength);
//            packet_length += infomemLength;
//            infomemResponse = 0;
//        }
//        else if (derivedChannelResponse)
//        {
//            *(resPacket + packet_length++) = DERIVED_CHANNEL_BYTES_RESPONSE;
//            *(resPacket + packet_length++) =
//                    storedConfig[NV_DERIVED_CHANNELS_0];
//            *(resPacket + packet_length++) =
//                    storedConfig[NV_DERIVED_CHANNELS_1];
//            *(resPacket + packet_length++) =
//                    storedConfig[NV_DERIVED_CHANNELS_2];
//            *(resPacket + packet_length++) =
//                    storedConfig[NV_DERIVED_CHANNELS_3];
//            *(resPacket + packet_length++) =
//                    storedConfig[NV_DERIVED_CHANNELS_4];
//            *(resPacket + packet_length++) =
//                    storedConfig[NV_DERIVED_CHANNELS_5];
//            *(resPacket + packet_length++) =
//                    storedConfig[NV_DERIVED_CHANNELS_6];
//            *(resPacket + packet_length++) =
//                    storedConfig[NV_DERIVED_CHANNELS_7];
//            derivedChannelResponse = 0;
//        }
//        else if (rwcResponse)
//        {
//            uint64_t rwc_curr_time_64;
//            rwc_curr_time_64 = getRwcTime();
//            *(resPacket + packet_length++) = RWC_RESPONSE;
//            memcpy(resPacket + packet_length, (uint8_t*) (&rwc_curr_time_64),
//                   8);
//            packet_length += 8;
//            rwcResponse = 0;
//        }
//        else if (btVerResponse)
//        {
//            uint8_t btVerStrLen = getBtVerStrLen();
//
//            *(resPacket + packet_length++) = BT_VERSION_STR_RESPONSE;
//            *(resPacket + packet_length++) = btVerStrLen;
//            memcpy((resPacket + packet_length), getBtVerStrPtr(), btVerStrLen);
//            packet_length += btVerStrLen;
//
//            btVerResponse = 0;
//        }
//        else if (btDataRateTestResponse)
//        {
//            /* Start test after ACK is sent - this will be handled by the
//             * interrupt after ACK byte is transmitted */
//            if (args[0] != 0)
//            {
//                setBtDataRateTestState(1);
//            }
//            btDataRateTestResponse = 0;
//        }
//    }
//
//    uint8_t crcMode = getBtCrcMode();
//    if (crcMode != CRC_OFF)
//    {
//        calculateCrcAndInsert(crcMode, resPacket, packet_length);
//        packet_length += crcMode;
//    }
//
//    BT_write(resPacket, packet_length, SHIMMER_CMD);
//}

char *HAL_GetUID(void)
{
    return dierecord;
}

void Timestamp0ToFirstFile()
{
    UINT bw;
    uint32_t my_local_time_long;
//   uint64_t rwc_curr_time_64;
//   rwc_curr_time_64 = RTC_get64();
    my_local_time_long = firstTs & 0xffffffff;

    uint8_t *sdHeadTextPtr = S4Ram_getSdHeadText();

    sdHeadTextPtr[SDH_MY_LOCALTIME_5TH] = (firstTs >> 32) & 0xff;

    memcpy(sdHeadTextPtr + SDH_MY_LOCALTIME, (uint8_t*) &my_local_time_long, 4);
//   fileLastHour = rwc_curr_time_64;
//   fileLastMin = rwc_curr_time_64;
// Write header to file
    ff_result = f_write(&dataFile, sdHeadTextPtr, SD_HEAD_SIZE, &bw);
}

FRESULT WriteFile(uint8_t *text, WORD size)
{
// Result code
    FRESULT rc;
    UINT bw;
    uint32_t file_td_h, file_td_m;                //, my_local_time_long32
    uint64_t local_time_40;

    sensing.inSdWr = 1;
    sensing.isSdOperating = 1;

    local_time_40 = RTC_get64();
    f_lseek(&dataFile, dataFile.fsize);                // seek to end of file, no spi op
    rc = f_write(&dataFile, text, size, &bw);                // Write to file

    file_td_h = local_time_40 - fileLastHour;
    file_td_m = local_time_40 - fileLastMin;

    //create a new file (from 000 up) every 1h
    if (file_td_h >= BIN_FILE_SPLIT_TIME_TICKS)
    {
        fileLastHour = fileLastMin = local_time_40;
        rc = f_close(&dataFile);
        //file number:from 000 up
        fileNum++;

        //avoid using printf()
        getFileNamePtr()[dirLen + 3] = (char) (((int) '0') + fileNum % 10);
        getFileNamePtr()[dirLen + 2] = (char) (((int) '0') + (fileNum / 10) % 10);
        getFileNamePtr()[dirLen + 1] = (char) (((int) '0') + (fileNum / 100) % 10);

        f_open(&dataFile, (char*) getFileNamePtr(), FA_WRITE | FA_CREATE_NEW);

        //      my_local_time_long32 = local_time_40 & 0xffffffff;
        //      sdHeadText[SDH_MY_LOCALTIME_5TH] = (local_time_40>>32) & 0xff;
        //      memcpy(&sdHeadText[SDH_MY_LOCALTIME], (uint8_t*)&my_local_time_long32, 4);
        //      fileLastMin = local_time_40;
        //      f_write(&fil, sdHeadText, SDHEAD_LEN, &bw);        // Write head to file
        firstTsFlag = 1;
        streamDataInProc = 0;
        ShimTask_clear(TASK_STREAMDATA);
    }
    // sync data to SD card every 1 min
    else if (file_td_m >= BIN_FILE_SYNC_TIME_TICKS)
    {
        fileLastMin = local_time_40;
        f_sync(&dataFile);
    }
    sensing.isSdOperating = 0;

    ff_result = rc;

    sensing.inSdWr = 0;
    sensing.inSdWrCnt = 0;

    return rc;
}

void PrepareSDBuffHead(void)
{
    memcpy(sdBuff + sdBuffLen, getMyTimeDiffPtr(), SYNC_PACKET_PAYLOAD_SIZE);
    sdBuffLen += SYNC_PACKET_PAYLOAD_SIZE;
    resetMyTimeDiff();
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
    gConfigBytes *storedConfigPtr = ShimConfig_getStoredConfig();

// GSR channel will always be last ADC channel
    if (currentBuffer == 0)
    {
        ADC_val = *((uint16_t*) txBuff0 + (sensing.nbrAdcChans - 1) + 2);
        if (storedConfigPtr->gsrRange == GSR_AUTORANGE)
        {
            if (GSR_smoothTransition(
                    &current_active_resistor,
                    storedConfigPtr->samplingRateTicks))
            {
                ADC_val = lastGsrVal;
            }
            else
                gsrActiveResistor = GSR_controlRange(ADC_val,
                                                     gsrActiveResistor);
        }
        *((uint16_t*) txBuff0 + (sensing.nbrAdcChans - 1) + 2) = ADC_val
                | (current_active_resistor << 14);
    }
    else
    {
        ADC_val = *((uint16_t*) txBuff1 + (sensing.nbrAdcChans - 1) + 2);
        if (storedConfigPtr->gsrRange == GSR_AUTORANGE)
        {
            if (GSR_smoothTransition(
                    &current_active_resistor,
                    storedConfigPtr->samplingRateTicks))
            {
                ADC_val = lastGsrVal;
            }
            else
                gsrActiveResistor = GSR_controlRange(ADC_val,
                                                     gsrActiveResistor);
        }
        *((uint16_t*) txBuff1 + (sensing.nbrAdcChans - 1) + 2) = ADC_val
                | (current_active_resistor << 14);
    }

    lastGsrVal = ADC_val;
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
        sensing.isFileCreated = 0;
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
            setSdInfoSyncDelayed(1);
        }
    }
    shimmerStatus.configuring = 0;
}

void SdInfoSync()
{
    setSdInfoSyncDelayed(0);
    if (GetSdCfgFlag())
    { // info > sdcard
        ShimConfig_readRam();
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
    CheckOnDefault();
}

void ReadSdConfiguration(void)
{
    ShimTask_clear(TASK_STREAMDATA); // this will skip one sample
    sensing.isFileCreated = 0;
    SdPowerOn();
    ParseConfig();

    /* Check BT module configuration after sensor configuration read from SD
     * card to see if it is in the correct state (i.e., BT on vs. BT off vs. SD
     * Sync) */
    checkBtModeConfig();
}

void SdPowerOff(void)
{
    Board_setSdPower(0);
} //   SD power off

void SdPowerOn(void)
{
    Board_setSdPower(1);
} //   SD power on

void Board_setSdPower(uint8_t state)
{
    if (state)
    {
        P4OUT |= BIT2;
    }
    else
    {
        P4OUT &= ~BIT2;
    }
    shimmerStatus.sdPowerOn = state;
}

uint8_t CheckSdInslot(void)
{
    // Check if card is inserted and enable interrupt for SD_DETECT_N
    shimmerStatus.sdInserted = (P4IN & BIT1) ? 0 : 1;
    ff_result = SD_mount(shimmerStatus.sdInserted);
    return shimmerStatus.sdInserted;
}

void RwcCheck(void)
{
#if RTC_OFF
    rwcErrorEn = 0;
    rwcErrorFlash = 0;
#else
    rwcErrorEn = ShimConfig_getStoredConfig()->rtcErrorEnable;
    rwcErrorFlash = ((!getRwcTimeDiff()) && rwcErrorEn) ? 1 : 0;
#endif
}

void StreamData()
{
    gConfigBytes *storedConfigPtr = ShimConfig_getStoredConfig();
    uint8_t adc_offset;
    uint8_t *current_buffer_ptr, *other_buffer_ptr;

    current_buffer_ptr = currentBuffer ? txBuff1 : txBuff0;
    other_buffer_ptr = currentBuffer ? txBuff0 : txBuff1;

    adc_offset = 4;

    uint8_t digi_offset = (sensing.nbrAdcChans * 2) + adc_offset;
    if (storedConfigPtr->chEnGsr)
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

    if (storedConfigPtr->chEnGyro)
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
    if (storedConfigPtr->chEnAltMag
            || (isWrAccelInUseIcm20948() && storedConfigPtr->chEnMag))
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

    if (storedConfigPtr->chEnWrAccel)
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

    if (storedConfigPtr->chEnMag)
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
    if (storedConfigPtr->chEnAltAccel)
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
    if (storedConfigPtr->chEnAltMag)
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
    if (storedConfigPtr->chEnPressureAndTemperature)
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
                        ShimConfig_configBytePressureOversamplingRatioGet());
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
                            ShimConfig_configBytePressureOversamplingRatioGet());
                }
            }
            bmpPressCount = bmpPressFreq;
        }
        memcpy(current_buffer_ptr + digi_offset, bmpVal, BMPX80_PACKET_SIZE);
        digi_offset += BMPX80_PACKET_SIZE;
    }
    if (storedConfigPtr->chEnExg1_24Bit)
    {
        EXG_readData(0, 0, current_buffer_ptr + digi_offset);
        digi_offset += 7;
    }
    else if (storedConfigPtr->chEnExg1_16Bit)
    {
        EXG_readData(0, 1, current_buffer_ptr + digi_offset);
        digi_offset += 5;
    }
    if (storedConfigPtr->chEnExg2_24Bit)
    {
        EXG_readData(1, 0, current_buffer_ptr + digi_offset);
        if (!((*(current_buffer_ptr + digi_offset + 1) == 0x00)
                || (*(current_buffer_ptr + digi_offset + 1) == 0xff)))
            _NOP();
        digi_offset += 7;
    }
    else if (storedConfigPtr->chEnExg2_16Bit)
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
                setStopSensingFlag(1U);
            }
            else
            {
                shimmerStatus.sdLogging = 0;
                sensing.isFileCreated = 0;
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
                setStopSensingFlag(1U);
            }
        }
    }

    if (stopSensing)
    {
        setStopSensingFlag(0);
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
    gConfigBytes *storedConfigPtr = ShimConfig_getStoredConfig();

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

    if (storedConfigPtr->chEnVBattery)
    {
        if (storedConfigPtr->chEnLnAccel)
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
                    && storedConfigPtr->chEnGsr) ? DMAEN : 0);
    DMA0SZ = 1;            //DMA0 size

    if (storedConfigPtr->chEnVBattery)
    {
        if (storedConfigPtr->chEnLnAccel)
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

uint16_t FreqProd(uint16_t samplingFreq)
{ // e.g. 7.5 ms: num_in=75, .25s:num_in=2500
    return (uint16_t) ceil(
            samplingClockFreqGet() * ((float) (samplingFreq / 10000.0)));
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

void manageReadBatt(uint8_t isBlockingRead)
{
    SetBattDma();
// Produces spikes in PPG data when only GSR enabled & aaccel, vbatt are off.
    __bis_SR_register(LPM3_bits + GIE);            //ACLK remains active
    ShimTask_set(TASK_CFGCH);

    saveBatteryVoltageAndUpdateStatus();

    // 10% Battery cutoff point - v0.9.6 onwards
    if (ShimConfig_getStoredConfig()->lowBatteryAutoStop
            && (batteryStatus.battStatusRaw.adcBattVal < BATT_CUTOFF_3_65VOLTS))
    {
        incrementBatteryCriticalCount();
        if (checkIfBatteryCritical() && shimmerStatus.sensing)
        {
            setStopSensingFlag(1U);
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

void StartLogging(void)
{
    SD_fileInit();
    sdBuffLen = 0;
    firstTsFlag = 1;
    fileLastHour = fileLastMin = RTC_get64();
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

void setSamplingClkSource(float samplingClock)
{
    clockFreq = (float) samplingClock;
    if (clockFreq == (float) TCXO_CLOCK)
    {
        P4OUT |= BIT6;
        _delay_cycles(2400000); // 100ms delay for tcxo
        P4SEL |= BIT7;
    }
    else if (clockFreq == (float) MSP430_CLOCK)
    {
        P4OUT &= ~BIT6;
        P4SEL &= ~BIT7;
    }
    ClkAssignment();
}

uint16_t getBmpX80SamplingTimeInTicks(void)
{
    uint8_t bmpX80Precision = ShimConfig_configBytePressureOversamplingRatioGet();
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
    uint8_t bmpX80Precision = ShimConfig_configBytePressureOversamplingRatioGet();
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
