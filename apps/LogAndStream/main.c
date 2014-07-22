/*
 * Copyright (c) 2014, Shimmer Research, Ltd.
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
 * @date July, 2014
 */


/***********************************************************************************
 Data Buffer Format:
      Packet Type |TimeStamp|Achan1|Achan2| ... |AchanX|Dchan1|Dchan2| ... |DchanX|
 Byte:    0-1     |   2-3   |Achan1|Achan2| ... |AchanX|Dchan1|Dchan2| ... |DchanX|

 Log file Format:
      SD Header   |TimeStamp1|Achan1 data1| ... |DchanX data1|TimeStamp2|Achan2 data2|...
   Byte:  0-255   |  256-257 |   258-259  | ... |            |          |            |...

***********************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "msp430.h"
#include "hal_UCA0.h"
#include "hal_uart0.h"
#include "hal_ADC.h"
#include "hal_Board.h"
#include "hal_Button.h"
#include "hal_DMA.h"
#include "hal_I2C.h"
#include "hal_InfoMem.h"
#include "hal_pmm.h"
#include "hal_SDCard.h"
#include "hal_UCS.h"
#include "RN42.h"
#include "bmp180.h"
#include "cat24c16.h"
#include "exg.h"
#include "ads1292.h"
#include "ff.h"
#include "gsr.h"
#include "lsm303dlhc.h"
#include "mpu9150.h"
#include "msp430_clock.h"
#include "shimmer_btsd.h"

void Init(void);
void CommTimerStart(void);
inline void CommTimerStop(void);
void BlinkTimerStart(void);
inline void BlinkTimerStop(void);
void SampleTimerStart(void);
inline void SampleTimerStop(void);
void StartStreaming(void);
inline void StopStreaming(void);
inline uint16_t GetTB0(void);
inline uint16_t GetTA0(void);
inline uint16_t GetRTC_short(void);
uint8_t Dma0ConversionDone(void);
uint8_t Dma2ConversionDone(void);
void ProcessCommand(void);
void SendResponse(void);
void ConfigureChannels(void);
void OpenFirstFile();
void Timestamp0ToFirstFile();
FRESULT WriteFile(uint8_t* text, WORD size);
void PrepareSDBuffHead(void);
void SetName();
error_t SetBasedir();
error_t MakeBasedir();
inline void GsrRange(void);
void Itoa(uint32_t num, uint8_t* buf, uint8_t len);
void DockSdPowerCycle();
void SetDocked(void);
void SetUndocked(void);
void ReadSdConfiguration(void);
void SetConfigVal(void);
void SdPowerOff(void);
void SdPowerOn(void);
uint8_t CheckSdInslot();
void ParseRcommResponse();
void SendRcommCommand();
void BtStop();
void BtStart();
void BtStartDone();
void StreamData();
void Write2SD();
void UpdateSdConfig();
void TB0Start();
void TB0Stop();
void SetupRTC(void);
uint32_t GetRTC();
void ParseConfig();
void Config2SdHead(void);
void SetDefaultConfiguration(void);
void RstRcommVariables();
void Calibrate();
void ReadCalibration(uint8_t sensor);
void DefaultCalibration(uint8_t sensor);
void BattBlinkOn();
void SetBattDma();
void ClkAssignment();
uint16_t FreqProd(uint16_t num_in);
uint16_t FreqDiv(float num_in);
void IniReadInfoMem();
uint8_t CheckOnDefault();

//data segment initialisation is disabled in system_pre_init.c

//BTSD_STATE state;
uint8_t sync_enabled, btsd_selfcmd, last_led;
//button vars
uint32_t button_press_ts, button_release_ts, button_last_release_ts;

uint8_t sd_write, sdlogging, btstreaming, enable_sdlog, enable_btstream;
uint8_t exp_cmd[4], mac[14],storedConfig[NV_TOTAL_NUM_CONFIG_BYTES], channelContents[MAX_NUM_CHANNELS], configuring, nbrAdcChans, nbrDigiChans;
uint16_t *adcStartPtr;


uint8_t currentBuffer, processCommand, sendAck, inquiryResponse, samplingRateResponse, startSensing, stopSensing,
      aAccelCalibrationResponse, gyroCalibrationResponse, magCalibrationResponse, dAccelCalibrationResponse,
      allCalibrationResponse, configureChannels, streamData, sendResponse, sensing, btIsConnected,
      deviceVersionResponse, fwVersionResponse, bufferSizeResponse, uniqueSerialResponse, configSetupBytesResponse,
      lsm303dlhcAccelRangeResponse, lsm303dlhcMagGainResponse, lsm303dlhcMagSamplingRateResponse,
      lsm303dlhcAccelSamplingRateResponse, lsm303dlhcAccelLPModeResponse, lsm303dlhcAccelHRModeResponse,
      mpu9150GyroRangeResponse, mpu9150SamplingRateResponse, mpu9150AccelRangeResponse, sampleMpu9150Mag,
      mpu9150MagSensAdjValsResponse, sampleBmp180Press, bmp180CalibrationCoefficientsResponse,
      bmp180OversamplingRatioResponse, blinkLedResponse, gsrRangeResponse, internalExpPowerEnableResponse,
      exgRegsResponse, dcIdResponse, dcMemResponse, dockedResponse, trialConfigResponse, centerResponse,
      shimmerNameResponse, expIDResponse, nshimmerResponse, myIDResponse, configTimeResponse, dirResponse;
uint8_t gAction;
uint8_t args[MAX_COMMAND_ARG_SIZE], waitingForArgs, waitingForArgsLength, argsSize;
uint8_t resPacket[RESPONSE_PACKET_SIZE];

// file system vars
FATFS fatfs;            //File object
DIRS dir;               //Directory object
FIL fil;
// make dir for SDLog files
uint8_t dirname[64], exp_dir_name[32],sdhead_text[SDHEAD_LEN],file_bad, toggle_redled, //file_new,
      filename[64],  exp_id_name[MAX_CHARS], shimmername[MAX_CHARS],config_time_text[MAX_CHARS], centername[MAX_CHARS],
      txBuff0[DATA_PACKET_SIZE], txBuff1[DATA_PACKET_SIZE], btrx_buff[14], *btrx_exp, sdbuff[SDBUFF_SIZE],
      dir_new, len_dir, set_config, isInMainLoop;
uint16_t sdbuff_len, block_len, file_num, dir_counter, blink_cnt_2, blink_cnt_5;
uint32_t last_hour, last_min;

// battery evaluation vars
uint8_t batt_stat, get_vbatt, batt_read, batt_wait,  batt_val[2];
uint32_t batt_last_time;
// routine comm
uint8_t my_time_diff[5], get_rcomm, rcomm_timeout, rcomm_resp[RCT_SIZE], rcomm_command, rcomm_status,rcomm_current_try,
       rcomm_interval, rcomm_success, rcomm_window_c, bt_status, sample_timer_status, blink_status;
uint16_t rcomm_cnt, rcomm_interval_c, rcomm_special_int;

uint8_t docked, dockDetected, set_undock, set_undock_done, set_undock_start, on_user_button, on_single_touch,
      on_default, preSampleBmpPress, bmpPressFreq, bmpPressCount, sampleBmpTemp, sampleBmpTempFreq, sampleBmp180Press,
      preSampleMpuMag, mpuMagFreq, mpuMagCount, sampleMpu9150Mag, gsr_active_resistor, waitpress, initializing;
uint8_t inSetDockDelay, inSetUndockDelay;
uint16_t last_GSR_val;
float clock_freq;
uint16_t clk_10,clk_50,clk_150,clk_45,clk_90,clk_135,clk_75,clk_90_45,clk_90_75,clk_135_90,
      clk_250,clk_250_90, clk_255,clk_105, clk_120,clk_2500,clk_1000,clk_165,clk_285;
//GSR
uint8_t gsr_active_resistor;
uint16_t last_GSR_val;
//ExG
uint8_t exgLength, exgChip, exgStartAddr;
//Daughter card EEPROM
uint8_t dcMemLength;
uint16_t dcMemOffset;


char *dierecord;

void main(void) {
   initializing = 1;                      // led flag, in initialisation period
   Init();
   dierecord = (char *)0x01A0A;

   if(!docked && CheckSdInslot()){
      ReadSdConfiguration();
   }else{
      IniReadInfoMem();
   }

   SetupRTC();
   ConfigureChannels();
   txBuff0[1] = txBuff1[1] = DATA_PACKET; //packet type
   initializing = 0;

   while(1) {
      isInMainLoop = 0;
      __bis_SR_register(LPM3_bits + GIE); //ACLK remains active
      isInMainLoop = 1;

      if(set_config){
         set_config = 0;
         ReadSdConfiguration();
      }
      if(batt_read){ //use adc channel2 and mem4, read back battery status every certain period
         batt_read = 0;
         batt_wait = 1;
         SetBattDma();
      }
      if(processCommand) {
         processCommand = 0;
         ProcessCommand();
      }
      //debug
      if(configureChannels) {
         configureChannels = 0;
         ConfigureChannels();
      }
      if(sendResponse) {
         sendResponse = 0;
         SendResponse();
      }
      if (sampleMpu9150Mag) {
         sampleMpu9150Mag = 0;
         MPU9150_startMagMeasurement();
      }
      if(sampleBmp180Press) {
         sampleBmp180Press = 0;
         if(sampleBmpTemp == sampleBmpTempFreq)
            BMP180_startTempMeasurement();
         else
            BMP180_startPressMeasurement((storedConfig[NV_CONFIG_SETUP_BYTE3]&0x30)>>4);
      }
      if(streamData){
         streamData=0;
         if((!(enable_sdlog && sdlogging))&&(!btstreaming)){
            btsd_selfcmd = 1;
            stopSensing = 1;
         }
         StreamData();
         // if sensor date buffer is large enough (about 1024 bytes),
         // write it to SDcard and clear the buffer
         if(enable_sdlog && sdlogging){
            if (sdbuff_len > SDBUFF_SIZE-block_len){
               sd_write = 1;
            }
            else{
               sd_write = 0;
            }
         }
      }
      if(startSensing) {
         startSensing = 0;
         if(!sensing) {
            if(btsd_selfcmd){
               enable_sdlog = 1;
               if(btIsConnected){
                  uint8_t selfcmd[3];
                  selfcmd[0] = INSTREAM_CMD_RESPONSE;
                  selfcmd[1] = STATUS_RESPONSE;
                  //STATUS: 0|0|0|0|0|SELFCMD|SENSING|DOCKED
                  selfcmd[2] = (docked & 0x01) + ((1  & 0x01)<<1) + ((1  & 0x01)<<2);
                  BT_write(selfcmd, 3);
               }
               btsd_selfcmd = 0;
            }
            if(dir_new && enable_sdlog && sdlogging){
               SdPowerOn();
               SetBasedir();
               MakeBasedir();
               file_num=0;
               sdbuff_len = 0;
               OpenFirstFile();
               dir_new = 0;
            }
            StartStreaming();
            if(enable_sdlog && sdlogging)
               Timestamp0ToFirstFile();
            if(docked)
               enable_sdlog = 0;
         }
      }
      if(sd_write){
         sd_write=0;
         Write2SD();
      }
   }
}

void Init(void) {
   // Stop WDTp
   //WDTCTL = WDTPW + WDTHOLD; // already handled in system_pre_init.c
   Board_init();

   // Set Vcore to accommodate for max. allowed system speed
   SetVCore(3);

   // Start 32.768kHz XTAL as ACLK
   LFXT_Start(XT1DRIVE_0);

   // Start 24MHz XTAL as MCLK and SMCLK
   XT2_Start(XT2DRIVE_2);        // XT2DRIVE_2 or XTDRIVE_3 for 24MHz (userguide section 5.4.7)
   UCSCTL4 |= SELS_5 + SELM_5;   // SMCLK=MCLK=XT2

   SFRIFG1 = 0;                  // clear interrupt flag register
   SFRIE1 |= OFIE;               // enable oscillator fault interrupt enable

   msp430_clock_init();
   memset(my_time_diff,0xff,5);
   //initialise the sensor data buff.
   memset(txBuff0,0,DATA_PACKET_SIZE);
   memset(txBuff1,0,DATA_PACKET_SIZE);
   // flag variables initialisation
   inSetDockDelay = 0;
   inSetUndockDelay = 0;
   last_led = 0;
   btsd_selfcmd = 0;
   toggle_redled = 0;
   sync_enabled = 0;
   configuring = 0;
   batt_last_time = 0;
   batt_read = 1;
   batt_wait = 0;
   batt_stat = 0;
   blink_cnt_2 = blink_cnt_5 = 0;
   file_bad = 0;
   rcomm_special_int = 0;
   rcomm_window_c = RC_WINDOW_C;
   rcomm_status = 0;
   on_default = 0;
   on_user_button = 0;
   on_single_touch = 0;
   set_config = 0;
   isInMainLoop = 0;
   set_undock = 0;
   get_rcomm = 0;
   currentBuffer = 0;
   processCommand = 0;
   startSensing = 0;
   stopSensing = 0;
   sampleBmp180Press = 0;
   sampleMpu9150Mag = 0;
   sampleBmpTemp = 0;
   preSampleMpuMag = 0;
   preSampleBmpPress = 0;
   streamData = 0;
   sensing = 0;
   btIsConnected = 0;
   nbrAdcChans = 0;
   nbrDigiChans = 0;
   enable_sdlog = 0;
   sdlogging = 0;
   enable_btstream = 0;
   btstreaming = 0;

   // sd file system initiate.
   dir_new = 1;
   memset(sdbuff,0,SDBUFF_SIZE);
   memset(storedConfig,0,NV_TOTAL_NUM_CONFIG_BYTES);
   memset(sdhead_text,0,SDHEAD_LEN);
   sdbuff_len = 0;

   clock_freq = MSP430_CLOCK;
   ClkAssignment();

   sample_timer_status = 0;
   blink_status = 0;

   sd_write=0;
   rcomm_timeout = 0;
   bt_status = 0;
   rcomm_command=ROUTINE_COMMUNICATION;
   last_GSR_val = 0;
   memset(mac,0,14);

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
   mpu9150SamplingRateResponse = 0;
   mpu9150AccelRangeResponse = 0;
   bmp180OversamplingRatioResponse = 0;
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
   configureChannels = 0;
   sendResponse = 0;
   sampleMpu9150Mag = 0;
   sampleBmp180Press = 0;
   streamData = 0;
   sensing = 0;
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
   last_GSR_val = 0;

   // Globally enable interrupts
   _enable_interrupts();

   BlinkTimerStart();

   if(P2IN & BIT3) {
      P2IES |= BIT3;       //look for falling edge
      dockDetected = 1;
      docked = 1;
      UART_activate();
      UART_init();
      sdlogging = 0;
      if(CheckSdInslot()){
         DockSdPowerCycle();
      }
   } else {
      P2IES &= ~BIT3;      //look for rising edge
      dockDetected = 0;
      docked = 0;
      ADS1292_activate();
      P6OUT |= BIT0;       //   DETECT_N set to high
      if(CheckSdInslot()){
         sdlogging = 1;
      }else {
         sdlogging = 0;
      }
   }
   P2IFG &= ~BIT3;         //clear flag
   P2IE |= BIT3;           //enable interrupt

   *shimmername = '\0';
   *exp_id_name = '\0';
   *centername = '\0';
   *config_time_text = '\0';

   memset(btrx_buff,0,14);
   DMA2_init((uint16_t *)&UCA1RXBUF, (uint16_t *)btrx_buff, 14);
   DMA2_transferDoneFunction(&Dma2ConversionDone);
   btrx_exp = BT_getExpResp();
   // =========== below initialize bt for the first time and get its MAC address ==========

   BT_init();
   //BT_setPIN("1234");
   BT_setRadioMode(SLAVE_MODE);
   BT_setGetMacAddress(1);
   BtStart();
   msp430_delay_ms(2200);
   uint8_t reset_cnt = 50;
   while(!bt_status){
      msp430_delay_ms(100);
      if(!(reset_cnt--))
         PMMCTL0 = PMMPW + PMMSWPOR + (PMMCTL0 & 0x0003);  // software POR reset
   }
   // =========== above initialize bt for the first time and get its MAC address ==========

   // enable switch1 interrupt
   Button_init();
   Button_interruptEnable();
   Button_waitpress();
   waitpress = 1;

   UCA0_isrInit();
   ADS1292_reg2Uca0();
   memcpy((char*)exp_cmd,"mac$",4);
   UART_setStr(exp_cmd, 4, mac, 12);
   UART_reg2Uca0();
}

void StartStreaming(void) {
   uint8_t i2c_en = 0;

   if(!sensing) {
      sensing = 1;
      if(storedConfig[NV_SENSORS0] & SENSOR_A_ACCEL) {
         P8OUT |= BIT6;    //analog accel being used so take out of sleep mode
      }

      if(storedConfig[NV_SENSORS1] & SENSOR_STRAIN){
          P2OUT |= BIT0;   //GPIO_INTERNAL1 set high
      }

      if(storedConfig[NV_SENSORS0] & SENSOR_GSR) {
         GSR_init();
         if(((storedConfig[NV_CONFIG_SETUP_BYTE3]&0x0E)>>1) <= HW_RES_3M3) {
            GSR_setRange((storedConfig[NV_CONFIG_SETUP_BYTE3]&0x0E)>>1);
            gsr_active_resistor = (storedConfig[NV_CONFIG_SETUP_BYTE3]&0x0E)>>1;
         } else {
            GSR_setRange(HW_RES_40K);
            gsr_active_resistor = HW_RES_40K;
         }
      }

      if((storedConfig[NV_SENSORS0] & SENSOR_MPU9150_GYRO)
            || (storedConfig[NV_SENSORS2] & SENSOR_MPU9150_ACCEL)
            || (storedConfig[NV_SENSORS2] & SENSOR_MPU9150_MAG)) {
         MPU9150_init();
         i2c_en = 1;
         MPU9150_wake(1);
         volatile uint8_t mpu_id = MPU9150_getId();
         volatile uint8_t mag_id = MPU9150_getMagId();
         if((storedConfig[NV_SENSORS0] & SENSOR_MPU9150_GYRO) || (storedConfig[NV_SENSORS2] & SENSOR_MPU9150_ACCEL)) {
            MPU9150_setSamplingRate(storedConfig[NV_CONFIG_SETUP_BYTE1]);
            if(storedConfig[NV_SENSORS0] & SENSOR_MPU9150_GYRO) {
               MPU9150_setGyroSensitivity(storedConfig[NV_CONFIG_SETUP_BYTE2]&0x03);   //This needs to go after the wake?
            }
            if(storedConfig[NV_SENSORS2] & SENSOR_MPU9150_ACCEL) {
               MPU9150_setAccelRange((storedConfig[NV_CONFIG_SETUP_BYTE3]&0xC0)>>6);
            }
         } else {
            //For some reason it seems necessary to power on the gyro/accel core before trying to access the mag
            //followed by one other I2C command (read or write)
            //No idea why
            //timing delays or other I2C commands to gyro/accel core do not seem to have the same effect?!?
            //Only relevant first time mag is accessed after powering up MPU9150
            MPU9150_wake(0);
         }
         if(storedConfig[NV_SENSORS2] & SENSOR_MPU9150_MAG) {
            if(*(uint16_t *)(storedConfig+NV_SAMPLING_RATE) >= clk_120) {
               //max of approx. 3ms to sample everything + 9ms between starting mag to data ready
               //so 12ms in total (394 ticks of 32768Hz clock = 12.024ms) (3070 ticks of 255765.625Hz clock = 12.024ms)
               //so there is time to get the mag sampled before the readings need to start each sample period
               preSampleMpuMag = 1;
            } else {
               //sample, then check each sample period if ready to read. Start new sample immediately
               MPU9150_startMagMeasurement();
               preSampleMpuMag = 0;
               mpuMagCount = mpuMagFreq = (clk_90 / *(uint16_t *)(storedConfig+NV_SAMPLING_RATE)) + 1;
            }
         }
      }

      if((storedConfig[NV_SENSORS0] & SENSOR_LSM303DLHC_MAG) || (storedConfig[NV_SENSORS1] & SENSOR_LSM303DLHC_ACCEL)) {
         if(!i2c_en) {
            //Initialise I2C
            LSM303DLHC_init();
            i2c_en = 1;
         }
         if(storedConfig[NV_SENSORS1] & SENSOR_LSM303DLHC_ACCEL) {
            LSM303DLHC_accelInit(((storedConfig[NV_CONFIG_SETUP_BYTE0]&0xF0)>>4),   //sampling rate
                            ((storedConfig[NV_CONFIG_SETUP_BYTE0]&0x0C)>>2),   //range
                            ((storedConfig[NV_CONFIG_SETUP_BYTE0]&0x02)>>1),    //low power mode
                            (storedConfig[NV_CONFIG_SETUP_BYTE0]&0x01));      //high resolution mode
         }
         if(storedConfig[NV_SENSORS0] & SENSOR_LSM303DLHC_MAG)
            LSM303DLHC_magInit(((storedConfig[NV_CONFIG_SETUP_BYTE2]&0x1C)>>2),    //sampling rate
                           ((storedConfig[NV_CONFIG_SETUP_BYTE2]&0xE0)>>5));   //gain
      }

      if(storedConfig[NV_SENSORS2] & SENSOR_BMP180_PRESSURE) {
         if(!i2c_en) {
            BMP180_init();
         }
         if((((storedConfig[NV_CONFIG_SETUP_BYTE3]&0x30)>>4)==0) && (*(uint16_t *)(storedConfig+NV_SAMPLING_RATE) >= clk_75)) {
            //max of approx. 3ms to sample everything + 4.5ms between starting press to data ready
            //so 7.5ms in total (246 ticks of 32768Hz clock = 7.507ms)(1919 ticks of 255765.625Hz clock = 7.5030ms)
            preSampleBmpPress = 1;
            bmpPressFreq = 1;      //required for the calculation of sampleBmpTempFreq below
         } else if((((storedConfig[NV_CONFIG_SETUP_BYTE3]&0x30)>>4)==1) && (*(uint16_t *)(storedConfig+NV_SAMPLING_RATE) >= clk_105)) {
            //max of approx. 3ms to sample everything + 7.5ms between starting press to data ready
            //so 10.5ms in total (345 ticks of 32768Hz clock = 10.529ms)(2686 ticks of 255765.625Hz clock = 10.5018ms)
            preSampleBmpPress = 1;
            bmpPressFreq = 1;      //required for the calculation of sampleBmpTempFreq below
         } else if((((storedConfig[NV_CONFIG_SETUP_BYTE3]&0x30)>>4)==2) && (*(uint16_t *)(storedConfig+NV_SAMPLING_RATE) >= clk_165)) {
            //max of approx. 3ms to sample everything + 13.5ms between starting press to data ready
            //so 16.5ms in total (541 ticks of 32768Hz clock = 16.510ms)(4221 ticks of 255765.625Hz clock = 16.5034ms)
            preSampleBmpPress = 1;
            bmpPressFreq = 1;      //required for the calculation of sampleBmpTempFreq below
         } else if((((storedConfig[NV_CONFIG_SETUP_BYTE3]&0x30)>>4)==3) && (*(uint16_t *)(storedConfig+NV_SAMPLING_RATE) >= clk_285)) {
            //max of approx. 3ms to sample everything + 25.5ms between starting press to data ready
            //so 28.5ms in total (934 ticks of 32768Hz clock = 28.503ms)(7290 ticks of 255765.625Hz clock = 28.5027ms)
            preSampleBmpPress = 1;
            bmpPressFreq = 1;      //required for the calculation of sampleBmpTempFreq below
         } else {
            //sample, then check each sample period if ready to read. Start new sample immediately
            BMP180_startTempMeasurement();
            preSampleBmpPress = 0;
            if(((storedConfig[NV_CONFIG_SETUP_BYTE3]&0x30)>>4)==0) {
               bmpPressCount = bmpPressFreq = (clk_45 / *(uint16_t *)(storedConfig+NV_SAMPLING_RATE)) + 1;//4.5
            } else if(((storedConfig[NV_CONFIG_SETUP_BYTE3]&0x30)>>4)==1) {
               bmpPressCount = bmpPressFreq = (clk_75 / *(uint16_t *)(storedConfig+NV_SAMPLING_RATE)) + 1;//7.5
            } else if(((storedConfig[NV_CONFIG_SETUP_BYTE3]&0x30)>>4)==2) {
               bmpPressCount = bmpPressFreq = (clk_135 / *(uint16_t *)(storedConfig+NV_SAMPLING_RATE)) + 1;//13.5
            } else {
               bmpPressCount = bmpPressFreq = (clk_255 / *(uint16_t *)(storedConfig+NV_SAMPLING_RATE)) + 1;//25.5
            }
         }
         //only need to sample temp once a second at most
         if(*(uint16_t *)(storedConfig+NV_SAMPLING_RATE) >= clk_2500) {
            //less than 4Hz
            //so every second sample must be temp
            sampleBmpTemp = sampleBmpTempFreq = 1;
         } else {
            sampleBmpTemp = sampleBmpTempFreq = (uint8_t)(FreqDiv( *(uint16_t *)(storedConfig+NV_SAMPLING_RATE)) - 1) / bmpPressFreq;
         }
      }

      //ExG
      if((storedConfig[NV_SENSORS0] & SENSOR_EXG1_24BIT) || (storedConfig[NV_SENSORS0] & SENSOR_EXG2_24BIT) ||
            (storedConfig[NV_SENSORS2] & SENSOR_EXG1_16BIT) || (storedConfig[NV_SENSORS2] & SENSOR_EXG2_16BIT)) {
         EXG_init();

         if(storedConfig[NV_SENSORS0] & SENSOR_EXG1_24BIT || storedConfig[NV_SENSORS2] & SENSOR_EXG1_16BIT)
            EXG_writeRegs(0, ADS1292R_CONFIG1, 10, (storedConfig+NV_EXG_ADS1292R_1_CONFIG1));
         if(storedConfig[NV_SENSORS0] & SENSOR_EXG2_24BIT || storedConfig[NV_SENSORS2] & SENSOR_EXG2_16BIT)
            EXG_writeRegs(1, ADS1292R_CONFIG1, 10, (storedConfig+NV_EXG_ADS1292R_2_CONFIG1));
         //probably turning on internal reference, so wait for it to settle
         __delay_cycles(2400000);   //100ms (assuming 24MHz clock)

         //probably setting the PGA gain so cancel the channel offset
         if(((storedConfig[NV_SENSORS0] & SENSOR_EXG1_24BIT) || (storedConfig[NV_SENSORS2] & SENSOR_EXG1_16BIT)) &&
               (storedConfig[NV_EXG_ADS1292R_1_RESP2] & BIT7)) {
            EXG_offsetCal(0);
         }
         if(((storedConfig[NV_SENSORS0] & SENSOR_EXG2_24BIT) || (storedConfig[NV_SENSORS2] & SENSOR_EXG2_16BIT)) &&
               (storedConfig[NV_EXG_ADS1292R_2_RESP2] & BIT7)) {
            EXG_offsetCal(1);
         }

         if(((storedConfig[NV_SENSORS0] & SENSOR_EXG1_24BIT) || (storedConfig[NV_SENSORS2] & SENSOR_EXG1_16BIT)) &&
               ((storedConfig[NV_SENSORS0] & SENSOR_EXG2_24BIT) || (storedConfig[NV_SENSORS2] & SENSOR_EXG2_16BIT)))
            EXG_start(2);
         else if ((storedConfig[NV_SENSORS0] & SENSOR_EXG1_24BIT) || (storedConfig[NV_SENSORS2] & SENSOR_EXG1_16BIT))
            EXG_start(0);
         else
            EXG_start(1);
      }

      SampleTimerStart();
      if(storedConfig[NV_TRIAL_CONFIG0]&SDH_TIME_SYNC)
         PrepareSDBuffHead();
   }
}
inline void StopStreaming(void) {
   //shut every thing down
   sensing = 0;
   enable_btstream = 0;
   enable_sdlog = 0;

   if(docked)           // if docked, cannot write to SD card any more
      DockSdPowerCycle();
   else{
      dir_new = 1;
      f_close(&fil);
      msp430_delay_ms(50);
      SdPowerOff();
   }

   SampleTimerStop();
   ADC_disable();
   DMA0_disable();

   P8OUT &= ~BIT6;
   P8REN |= BIT6;      //enable pull down resistor
   P8DIR &= ~BIT6;     //SW_ACCEL set as input

   P3OUT &= ~BIT3;     //set EXP_RESET_N low

   if((storedConfig[NV_SENSORS0] & SENSOR_MPU9150_GYRO) || (storedConfig[NV_SENSORS2] & SENSOR_MPU9150_ACCEL)) {
      MPU9150_wake(0);
   }
   if((storedConfig[NV_SENSORS0] & SENSOR_LSM303DLHC_MAG) || (storedConfig[NV_SENSORS1] & SENSOR_LSM303DLHC_ACCEL)) {
      LSM303DLHC_sleep();
   }
   if((storedConfig[NV_SENSORS0] & SENSOR_EXG1_24BIT) || (storedConfig[NV_SENSORS2] & SENSOR_EXG1_16BIT)) {
      EXG_stop(0);     //probably not needed
   }
   if((storedConfig[NV_SENSORS0] & SENSOR_EXG2_24BIT) || (storedConfig[NV_SENSORS2] & SENSOR_EXG2_16BIT)) {
      EXG_stop(1);     //probably not needed
   }
   if((storedConfig[NV_SENSORS0] & SENSOR_EXG1_24BIT) || (storedConfig[NV_SENSORS0] & SENSOR_EXG2_24BIT) ||
        (storedConfig[NV_SENSORS2] & SENSOR_EXG1_16BIT) || (storedConfig[NV_SENSORS2] & SENSOR_EXG2_16BIT)) {
      EXG_powerOff();
   }

   msp430_delay_ms(10); //give plenty of time for I2C operations to finish before disabling I2C
   I2C_Disable();
   I2C_PowerOff();
   streamData = 0;
   sd_write = 0;
   sdbuff_len = 0;
   sampleBmp180Press = 0;
   sampleMpu9150Mag = 0;
}


void ConfigureChannels(void) {
   uint8_t *channel_contents_ptr = channelContents;
   uint16_t mask=0;

   nbrAdcChans = nbrDigiChans = 0;

   if(storedConfig[NV_CONFIG_SETUP_BYTE3] & EXP_POWER_ENABLE){//power of exp board
      P3OUT |= BIT3;
   }else{
      P3OUT &= ~BIT3;
   }
   //Analog Accel
   if(storedConfig[NV_SENSORS0] & SENSOR_A_ACCEL) {
      *channel_contents_ptr++ = X_A_ACCEL;
      *channel_contents_ptr++ = Y_A_ACCEL;
      *channel_contents_ptr++ = Z_A_ACCEL;
      mask += MASK_A_ACCEL;
      nbrAdcChans += 3;
   }
   //Battery Voltage
   if(storedConfig[NV_SENSORS1] & SENSOR_VBATT) {
      *channel_contents_ptr++ = VBATT;
      mask += MASK_VBATT;
      nbrAdcChans++;
   }
   //External ADC channel A7
   if(storedConfig[NV_SENSORS0] & SENSOR_EXT_A7) {
      *channel_contents_ptr++ = EXTERNAL_ADC_7;
      mask += MASK_EXT_A7;
      nbrAdcChans++;
   }
   //External ADC channel A6
   if(storedConfig[NV_SENSORS0] & SENSOR_EXT_A6) {
      *channel_contents_ptr++ = EXTERNAL_ADC_6;
      mask += MASK_EXT_A6;
      nbrAdcChans++;
   }
   //External ADC channel A15
   if(storedConfig[NV_SENSORS1] & SENSOR_EXT_A15) {
      *channel_contents_ptr++ = EXTERNAL_ADC_15;
      mask += MASK_EXT_A15;
      nbrAdcChans++;
   }
   //Internal ADC channel A12
   if(storedConfig[NV_SENSORS1] & SENSOR_INT_A12) {
      *channel_contents_ptr++ = INTERNAL_ADC_12;
      mask += MASK_INT_A12;
      nbrAdcChans++;
   }
   //Strain gauge
   if(storedConfig[NV_SENSORS1] & SENSOR_STRAIN) {
      *channel_contents_ptr++ = STRAIN_HIGH;
      *channel_contents_ptr++ = STRAIN_LOW;
      mask += MASK_STRAIN;
      nbrAdcChans += 2;
   }
   //Internal ADC channel A13
   if((storedConfig[NV_SENSORS1] & SENSOR_INT_A13) && !(storedConfig[NV_SENSORS1] & SENSOR_STRAIN)) {
      *channel_contents_ptr++ = INTERNAL_ADC_13;
      mask += MASK_INT_A13;
      nbrAdcChans++;
   }
   //Internal ADC channel A14
   if((storedConfig[NV_SENSORS2] & SENSOR_INT_A14) && !(storedConfig[NV_SENSORS1] & SENSOR_STRAIN)) {
      *channel_contents_ptr++ = INTERNAL_ADC_14;
      mask += MASK_INT_A14;
      nbrAdcChans++;
   }
   //Internal ADC channel A1
   if (storedConfig[NV_SENSORS0] & SENSOR_GSR) {
      //needs to be last analog channel
      *channel_contents_ptr++ = GSR_RAW;
      mask += MASK_INT_A1;
      nbrAdcChans++;
   }
   if(storedConfig[NV_SENSORS1] & SENSOR_INT_A1) {
      *channel_contents_ptr++ = INTERNAL_ADC_1;
      mask += MASK_INT_A1;
      nbrAdcChans++;
   }
   //Digi Gyro
   if(storedConfig[NV_SENSORS0] & SENSOR_MPU9150_GYRO) {
      *channel_contents_ptr++ = X_MPU9150_GYRO;
      *channel_contents_ptr++ = Y_MPU9150_GYRO;
      *channel_contents_ptr++ = Z_MPU9150_GYRO;
      nbrDigiChans += 3;
   }
   //Digi Accel
   if(storedConfig[NV_SENSORS1] & SENSOR_LSM303DLHC_ACCEL) {
      *channel_contents_ptr++ = X_LSM303DLHC_ACCEL;
      *channel_contents_ptr++ = Y_LSM303DLHC_ACCEL;
      *channel_contents_ptr++ = Z_LSM303DLHC_ACCEL;
      nbrDigiChans += 3;
   }
   //Mag
   if(storedConfig[NV_SENSORS0] & SENSOR_LSM303DLHC_MAG) {
      *channel_contents_ptr++ = X_LSM303DLHC_MAG;
      *channel_contents_ptr++ = Z_LSM303DLHC_MAG;
      *channel_contents_ptr++ = Y_LSM303DLHC_MAG;
      nbrDigiChans += 3;
   }
   //Digi Accel
   if(storedConfig[NV_SENSORS2] & SENSOR_MPU9150_ACCEL) {
      *channel_contents_ptr++ = X_MPU9150_ACCEL;
      *channel_contents_ptr++ = Y_MPU9150_ACCEL;
      *channel_contents_ptr++ = Z_MPU9150_ACCEL;
      nbrDigiChans += 3;
   }
   //Digi Accel
   if(storedConfig[NV_SENSORS2] & SENSOR_MPU9150_MAG) {
      *channel_contents_ptr++ = X_MPU9150_MAG;
      *channel_contents_ptr++ = Y_MPU9150_MAG;
      *channel_contents_ptr++ = Z_MPU9150_MAG;
      nbrDigiChans += 3;
   }

   uint8_t clk_offset;
   if(clock_freq == TCXO_CLOCK)
      clk_offset = 2;
   else
      clk_offset = 0;

   if(storedConfig[NV_SENSORS2] & SENSOR_BMP180_PRESSURE) {
      *channel_contents_ptr++ = BMP180_TEMP;
      *channel_contents_ptr++ = BMP180_PRESSURE;
      nbrDigiChans+=2;//PRES & TEMP, ON/OFF together
      block_len = (((nbrAdcChans+nbrDigiChans)*2)+3+clk_offset);
   }else {
      block_len = (((nbrAdcChans+nbrDigiChans)*2)+2+clk_offset);
   }

   if(storedConfig[NV_SENSORS0] & SENSOR_EXG1_24BIT) {
      *channel_contents_ptr++ = EXG_ADS1292R_1_STATUS;
      *channel_contents_ptr++ = EXG_ADS1292R_1_CH1_24BIT;
      *channel_contents_ptr++ = EXG_ADS1292R_1_CH2_24BIT;
      nbrDigiChans += 3;
      block_len += 7;
   } else if(storedConfig[NV_SENSORS2] & SENSOR_EXG1_16BIT) {
      *channel_contents_ptr++ = EXG_ADS1292R_1_STATUS;
      *channel_contents_ptr++ = EXG_ADS1292R_1_CH1_16BIT;
      *channel_contents_ptr++ = EXG_ADS1292R_1_CH2_16BIT;
      nbrDigiChans += 3;
      block_len += 5;
   }
   if(storedConfig[NV_SENSORS0] & SENSOR_EXG2_24BIT) {
      *channel_contents_ptr++ = EXG_ADS1292R_2_STATUS;
      *channel_contents_ptr++ = EXG_ADS1292R_2_CH1_24BIT;
      *channel_contents_ptr++ = EXG_ADS1292R_2_CH2_24BIT;
      nbrDigiChans += 3;
      block_len += 7;
   } else if(storedConfig[NV_SENSORS2] & SENSOR_EXG2_16BIT) {
      *channel_contents_ptr++ = EXG_ADS1292R_2_STATUS;
      *channel_contents_ptr++ = EXG_ADS1292R_2_CH1_16BIT;
      *channel_contents_ptr++ = EXG_ADS1292R_2_CH2_16BIT;
      nbrDigiChans += 3;
      block_len += 5;
   }

   if(mask) {
      adcStartPtr = ADC_init(mask);
      DMA0_transferDoneFunction(&Dma0ConversionDone);
      if(adcStartPtr)
         DMA0_init(adcStartPtr, (uint16_t *)(txBuff0+4), nbrAdcChans);
   }
}


uint8_t CheckOnDefault(){
   if(on_default && !docked && !sensing){ //state == BTSD_IDLESD
      startSensing = 1;
      btsd_selfcmd = 1;
      return 1;
   }
   return 0;
}

//Switch SW1, BT_RTS and BT connect/disconnect
#pragma vector=PORT1_VECTOR
__interrupt void Port1_ISR(void)
{
   //Context save interrupt flag before calling interrupt vector.
   //Reading interrupt vector generator will automatically clear IFG flag
   //buttonsPressed = PAIFG & BUTTON_ALL;

   switch (__even_in_range(P1IV, P1IV_P1IFG7))
   {
      //BT Connect/Disconnect
      case  P1IV_P1IFG0:   //BT Connect/Disconnect
         if(P1IN & BIT0) { //BT is connected
            P1IES |= BIT0; //look for falling edge
            BT_connectionInterrupt(1);
            btIsConnected = 1;
            BT_rst_MessageProgress();
            if(sync_enabled){
               btstreaming = 0;
               if(storedConfig[NV_TRIAL_CONFIG0]&SDH_IAMMASTER){
                  //center is waiting for 1 byte ROUTINE_COMMUNICATION(0xE0) from DMA2
                  DMA2_disable();
                  DMA2SZ = 1;
                  DMA2_enable();
               }
               else
                  //node sends (0xE0) and is waiting for 6 byte RC info from DMA2
                  if(rcomm_status){
                     SendRcommCommand();
                     DMA2_disable();
                     DMA2SZ = 6;
                     DMA2_enable();
                  }
            }
            else{
               btstreaming = 1;
               waitingForArgs = 0;
               waitingForArgsLength = 0;
               DMA2_disable();
               DMA2SZ = 1;
               DMA2_enable();
            }
         } else {             //BT is disconnected
            btstreaming = 0;
            enable_btstream = 0;
            P1IES &= ~BIT0;   //look for rising edge
            btIsConnected = 0;
            BT_connectionInterrupt(0);
            BT_rst_MessageProgress();
         }
         break;

      //BT RTS
      case  P1IV_P1IFG3:
         if(P1IN & BIT3) {
            P1IES |= BIT3;       //look for falling edge
            BT_rtsInterrupt(1);
         } else {
            P1IES &= ~BIT3;      //look for rising edge
            BT_rtsInterrupt(0);  //when 0, can call sendNextChar();
         }
         break;

      //ExG chip2 data ready
       case  P1IV_P1IFG4:
          EXG_dataReadyChip2();
          break;

        //BUTTON_SW1
        case  P1IV_P1IFG6:
           if((!(P1IN & BIT6)) && waitpress){   //button pressed
               _NOP();
              Button_waitrelease();
              waitpress = 0;
              msp430_get_clock_ms(&button_press_ts);
               _NOP();
           }
           else{                                //button released
               _NOP();
              Button_waitpress();
              waitpress = 1;
              msp430_get_clock_ms(&button_release_ts);
              uint32_t button_press_release_td = (uint32_t)fmod(4294967296+(uint64_t)button_release_ts - (uint64_t)button_press_ts,4294967296);
              uint32_t button_two_press_td = (uint32_t)fmod(4294967296+(uint64_t)button_release_ts - (uint64_t)button_last_release_ts,4294967296);
              button_last_release_ts = button_release_ts;
              button_press_ts = button_release_ts;
              //this part is not implemented yet
              //so please do not press the button for more than 30s
              if(button_press_release_td>=30000){
                 if(sync_enabled){
                    sync_enabled = 0;
                    CommTimerStop();
                    if(bt_status){
                       BtStop();
                    }
                  BT_init();
                  BT_disableRemoteConfig(1);
                  BT_setRadioMode(SLAVE_MODE);
                  msp430_register_timer_cb(BtStart, 1000, 0);
                 }
                 else{
                    sync_enabled = 1;
                    enable_btstream = 0;
                    msp430_register_timer_cb(CommTimerStart, 1000, 0);
                    if(storedConfig[NV_TRIAL_CONFIG0]&SDH_TIME_SYNC){
                       rcomm_status=1;
                       RstRcommVariables();
                    }
                    BtStop();
                 }
              }else if(button_press_release_td>=10 && button_two_press_td>200){
                  if(storedConfig[NV_TRIAL_CONFIG0] & SDH_USER_BUTTON_ENABLE){
                     // toggles sensing and refresh BT timers (for the centre)
                     if(sensing){
                        stopSensing = 1;
                        btsd_selfcmd = 1;
                        if(storedConfig[NV_TRIAL_CONFIG0]&SDH_IAMMASTER){
                           rcomm_window_c = storedConfig[NV_BT_INTERVAL]+RC_WINDOW_C*2;
                           if(storedConfig[NV_BT_INTERVAL]>RC_WINDOW_C*3)
                              rcomm_interval_c = storedConfig[NV_BT_INTERVAL]*2;
                           else
                              rcomm_interval_c = storedConfig[NV_BT_INTERVAL]+RC_WINDOW_C*3;
                           rcomm_cnt=0;
                           rcomm_special_int = 1;
                        }
                     }
                     else{
                        startSensing = 1;
                        btsd_selfcmd = 1;
                        if(storedConfig[NV_TRIAL_CONFIG0]&SDH_IAMMASTER){
                           rcomm_window_c = storedConfig[NV_BT_INTERVAL]+RC_WINDOW_C*2;
                           if(storedConfig[NV_BT_INTERVAL]>RC_WINDOW_C*3)
                              rcomm_interval_c = storedConfig[NV_BT_INTERVAL]*2;
                           else
                              rcomm_interval_c = storedConfig[NV_BT_INTERVAL]+RC_WINDOW_C*3;
                           rcomm_cnt=0;
                           rcomm_special_int = 1;
                        }
                        __bic_SR_register_on_exit(LPM3_bits);
                     }
                  }
              }
           }
           _NOP();
           break;
        default:
           break;
    }
}


#pragma vector=PORT2_VECTOR
__interrupt void Port2_ISR(void)
{
   switch (__even_in_range(P2IV, P2IV_P2IFG7))
   {
       //ExG chip1 data ready
      case  P2IV_P2IFG0:
         EXG_dataReadyChip1();
         break;

      //EXP_DETECT_N
      case  P2IV_P2IFG1:
         if(P2IN & BIT1) {
            //card not inserted
            P2IES |= BIT1;    //look for falling edge
         } else {
            //card inserted
            P2IES &= ~BIT1;   //look for rising edge
         }
         break;

      //dock_detect_N
      case  P2IV_P2IFG3:
         if(P2IN & BIT3) {
            dockDetected = 1;
            P2IES |= BIT3;   //look for falling edge
            if(inSetDockDelay){

            }
            else if(inSetUndockDelay){

            }
            else{
               inSetDockDelay = 1;
               msp430_register_timer_cb(SetDocked, 100, 0);
            }
         } else {
            dockDetected = 0;
            P2IES &= ~BIT3;   //look for rising edge
            if(inSetUndockDelay){

            }
            else if(inSetDockDelay){

            }
            else{
               inSetUndockDelay = 1;
               msp430_register_timer_cb(SetUndocked, 100, 0);
            }
         }
         if(btIsConnected && (!sync_enabled)){
            dockedResponse = 1;
            sendAck = 1;
            sendResponse = 1;
            if(!isInMainLoop)
               __bic_SR_register_on_exit(LPM3_bits);
         }
         break;
      // Default case
      default:
         break;
   }
}




//Timer2:
//ccr1: for blink timer
void CommTimerStart(void) {
   RstRcommVariables();
   TA0CTL = TASSEL_1 + MC_2 + TACLR;    //ACLK, continuous mode, clear TAR
   TA0CCTL1 = CCIE;
   TA0CCR1 = GetTA0() + 16384;
}

inline void CommTimerStop(void) {
   TA0CTL = MC_0;
   rcomm_status=0;
   TA0CCTL1 &= ~CCIE;
}

inline uint16_t GetTA0(void) {
   register uint16_t t0, t1;
   uint8_t ie;
   if(ie=(__get_SR_register()&0x0008))   //interrupts enabled?
      __disable_interrupt();
   t1 =TA0R;
   do {t0=t1; t1=TA0R;} while(t0!=t1);
   if(ie)
      __enable_interrupt();
   return t1;
}


#pragma vector=TIMER0_A1_VECTOR
__interrupt void TIMER0_A1_ISR(void){
   switch(__even_in_range(TA0IV,14))
   {
   case  0: break;                         //No interrupt
   case  2:                                //TA0CCR1
      if(rcomm_status){
         if(!(storedConfig[NV_TRIAL_CONFIG0]&SDH_IAMMASTER)){
            TA0CCR1 = GetTA0() + RC_CLK_N;
            if(rcomm_cnt==1){
               rcomm_current_try = 0;
               if(!bt_status){
                  BT_init();
                  BT_setRadioMode(MASTER_MODE);
                  BtStart();
               }
            }else if((rcomm_cnt>(RC_AHD*RC_FACTOR_N+1)) && (rcomm_cnt < (RC_WINDOW_N*RC_FACTOR_N + RC_AHD*RC_FACTOR_N + 1))){//
               if(bt_status){
                  if(rcomm_success){
                     if(!btIsConnected){
                        BtStop();
                     }
                  }else
                     if(!(rcomm_current_try%9)){
                        BT_connect(centername);
                     }
                  rcomm_current_try++;
               }
            }else if(rcomm_cnt == RC_WINDOW_N*RC_FACTOR_N + RC_AHD*RC_FACTOR_N + 1){//
               if(!rcomm_success){  //if no success in this communication, try earlier next time.
                  rcomm_interval=RC_INT_N;
               }
               else{
                  if(sensing)
                     rcomm_interval = storedConfig[NV_BT_INTERVAL];
                  else
                     rcomm_interval=RC_INT_N;
               }
               rcomm_success = 0;
               BtStop();
            }else if(rcomm_cnt >= rcomm_interval*RC_FACTOR_N){
               rcomm_cnt = 0;
            }
         }else{   // i am center/slave
                  // center:
                  // 1) start+initialise BT
                  // 2) wait rcomm_window_c secs and shut down BT
                  // if connected by a node, disconnect within 0.5s and wait for another
            TA0CCR1  = GetTA0() + RC_CLK_C;
            //==================== below is bt timeout function ====================
            if(btIsConnected){
               rcomm_timeout ++;
            }
            else
               rcomm_timeout = 0;
            if(rcomm_timeout ==2){
               if(btIsConnected){
                  BT_disconnect();
               }
               rcomm_timeout = 0;
            }
            //==================== above is bt timeout function ====================

            if(rcomm_cnt==1){// bt start
               if(!bt_status){
                  BT_init();
                  BT_disableRemoteConfig(1);
                  BT_setRadioMode(SLAVE_MODE);
                  BtStart();
               }
            }
            else if(rcomm_cnt==rcomm_window_c*RC_FACTOR_C){ //bt stop
               if(bt_status){
                  if(!rcomm_special_int)
                     rcomm_interval_c = storedConfig[NV_BT_INTERVAL];
               }
               else{
                  rcomm_interval_c = rcomm_window_c+RC_WINDOW_C;
               }
               BtStop();
            }
            else if(rcomm_cnt>=rcomm_interval_c*RC_FACTOR_C){  //reaching counter_max, and return to normal iteration
               rcomm_window_c = RC_WINDOW_C;
               rcomm_interval_c = storedConfig[NV_BT_INTERVAL];
               rcomm_cnt=0;
               rcomm_special_int = 0;
            }
         }
         if(on_user_button && !sensing){
            rcomm_cnt = 0;
            if(bt_status){
               rcomm_success = 0;
               BtStop();
            }
         }else
            rcomm_cnt++;

         if(docked){
            //when docked, shut every thing and return to no_RC mode
            BtStop();
            if(!(set_undock + btIsConnected + bt_status +
               stopSensing + startSensing + streamData + sd_write +
               processCommand)) {
               on_user_button = 0;
               on_single_touch = 0;
               on_default = 0;
               rcomm_status = 0;
               rcomm_cnt = 0;
               stopSensing = 1;
               btsd_selfcmd = 1;
               __bic_SR_register_on_exit(LPM3_bits);
            }
         }
      }
      else
      {//ideal: no_RC mode
         TA0CCR1  = GetTA0() +  RC_CLK_C;
         if(docked){
            // when docked, stay quiet and shut sensors if they are on
            if(sensing){
               stopSensing = 1;
               btsd_selfcmd = 1;
               __bic_SR_register_on_exit(LPM3_bits);
            }
         }
      }
      if(CheckOnDefault())
         __bic_SR_register_on_exit(LPM3_bits);
      break;
   case  4: break;                         // TA0CCR2 not used
   case  6: break;                         // Reserved
   case  8: break;                         // Reserved
   case 10: break;                         // Reserved
   case 12: break;                         // Reserved
   case 14: break;                         // TAIFG overflow handler
   }
}


void SendRcommCommand(){
   memset(rcomm_resp,0,RCT_SIZE);
   get_rcomm=1;
   BT_write(&rcomm_command, 0x01);
}

void ParseRcommResponse(){
   // only nodes do this
   if(rcomm_resp[RCT_ACK] == ACK_COMMAND_PROCESSED){     //if received the correct 6 bytes:
      uint32_t my_local_time_long, my_center_time_long, my_time_diff_long;
      uint8_t sd_tolog;
      my_local_time_long = GetRTC();                     // get my_local_time_long
      rcomm_success = 1;

      sd_tolog = rcomm_resp[RCT_FLG];
      my_center_time_long = *(uint32_t*)(rcomm_resp+RCT_TIME); // get my_center_time_long
      //check sensing state flag, and follow
      if(on_single_touch){
         if(sensing){
            if(!sd_tolog){
               stopSensing = 1;
               btsd_selfcmd = 1;
            }
         }
         else{
            if(sd_tolog){
               startSensing = 1;
               btsd_selfcmd = 1;
            }
         }
      }

      //calc time difference and save into my_time_diff[5]
      //which will be written into every SD card buffer
      if(my_local_time_long>my_center_time_long){
         my_time_diff[0] = 0;
         my_time_diff_long = (uint32_t)fmod(4294967296+(uint64_t)my_local_time_long - (uint64_t)my_center_time_long,4294967296);
      }else{
         my_time_diff[0] = 1;
         my_time_diff_long = (uint32_t)fmod(4294967296+(uint64_t)my_center_time_long - (uint64_t)my_local_time_long,4294967296);
      }
      memcpy(my_time_diff+1,(uint8_t*)&my_time_diff_long,4);

   }
}

// Blink Timer
//USING TB0 with CCR1
void BlinkTimerStart(void) {
   blink_status=1;
   TB0Start();
   TB0CCTL3 = CCIE;
   TB0CCR3 = GetTB0() + clk_1000;
}
inline void BlinkTimerStop(void) {
   blink_status=0;
   TB0CCTL3 &= ~CCIE;
}

#pragma vector=TIMER0_B1_VECTOR
__interrupt void TIMER0_B1_ISR(void)
{
   switch(__even_in_range(TB0IV,14))
   {
   case  0: break;                           // No interrupt
    case  2:                                 // TB0CCR1
       //MPU9150 mag
       TB0CCR1 = GetTB0() + *(uint16_t *)(storedConfig+NV_SAMPLING_RATE);
       sampleMpu9150Mag = 1;
       __bic_SR_register_on_exit(LPM3_bits);
       break;
    case  4:                                 // TB0CCR2
       //Bmp180 press
       TB0CCR2 = GetTB0() + *(uint16_t *)(storedConfig+NV_SAMPLING_RATE);
       sampleBmp180Press = 1;
       __bic_SR_register_on_exit(LPM3_bits);
       break;
    case  6:                                 // TB0CCR3
       //for LED blink usage details, please check shimmer user manual
      TB0CCR3 = GetTB0() + clk_1000;
      if(blink_cnt_5++==49)
         blink_cnt_5 = 0;

      if(blink_cnt_2++==19)
         blink_cnt_2 = 0;

      uint32_t my_local_time_long, batt_td;
      my_local_time_long = GetRTC();
      batt_td = (uint32_t)fmod(4294967296+(uint64_t)my_local_time_long - (uint64_t)batt_last_time,4294967296);
      if(batt_td > BATT_ITNERVAL){  //10 mins = 19660800
         batt_read=1;
         batt_last_time = my_local_time_long;
      }

      if(!initializing)
         if(batt_read)
            __bic_SR_register_on_exit(LPM3_bits);

       if(blink_status){
          //below are settings for green0, yellow and red leds, battery charge status
          if(toggle_redled){
            Board_ledOff(LED_GREEN0+LED_YELLOW);
            Board_ledOn(LED_RED);
            if(docked){
               BattBlinkOn();
            }else{
               if(!blink_cnt_5)
                  BattBlinkOn();
            }
          }
          else{
            Board_ledOff(LED_GREEN0+LED_YELLOW+LED_RED);
            if(docked){
               BattBlinkOn();
            }else{
               if(!blink_cnt_5)
                  BattBlinkOn();
            }
          }

         //below are settings for green1 and blue leds
         if(file_bad && !docked){   //bad file = green1/blue alternating
            if(!(P1OUT & BIT1)){
               Board_ledOn(LED_GREEN1);
               Board_ledOff(LED_BLUE);
            }
            else{
               Board_ledOff(LED_GREEN1);
               Board_ledOn(LED_BLUE);
            }
         }
         else{
            //good file - green1:
            if(sync_enabled){ //sync not implemented yet
               if(!sensing){  //standby or configuring
                  if(initializing || configuring){ //configuring
                     if(!(P1OUT & BIT1))
                        Board_ledOn(LED_GREEN1);
                     else
                        Board_ledOff(LED_GREEN1);
                  }
                  else{                            //standby
                     if(!blink_cnt_2)
                        Board_ledOn(LED_GREEN1);
                     else
                        Board_ledOff(LED_GREEN1);
                  }
               }else{   //sensing
                  if(!(blink_cnt_2%10)){
                     if(!(P1OUT & BIT1))
                        Board_ledOn(LED_GREEN1);
                     else
                        Board_ledOff(LED_GREEN1);
                  }
               }
               //good file - blue:
               if(bt_status){
                  Board_ledOn(LED_BLUE);
               }
               else{
                  Board_ledOff(LED_BLUE);
               }
            }
            else{
               if(!sensing){  //standby or configuring
                  if(initializing || configuring){ //configuring
                     if(!(P1OUT & BIT2))
                        Board_ledOn(LED_BLUE);
                     else
                        Board_ledOff(LED_BLUE);
                  } else if(btIsConnected){
                     Board_ledOn(LED_BLUE);
                  } else{                          //standby
                     if(!blink_cnt_2)
                        Board_ledOn(LED_BLUE);
                     else
                        Board_ledOff(LED_BLUE);
                  }
                  Board_ledOff(LED_GREEN1);  //nothing to show
               }else {  //sensing
                        //sdlogging, btstreaming, enable_sdlog, enable_btstream
                        //btstream only
                  if((enable_btstream && btstreaming) && !(sdlogging && enable_sdlog)){
                     if(!(blink_cnt_2%10)){
                        if(!(P1OUT & BIT2))
                           Board_ledOn(LED_BLUE);
                        else
                           Board_ledOff(LED_BLUE);
                     }
                     Board_ledOff(LED_GREEN1);  //nothing to show
                  }
                  //sdlog only
                  else if(!(enable_btstream && btstreaming) && (sdlogging && enable_sdlog)){
                     if(!(blink_cnt_2%10)){
                        if(!(P1OUT & BIT1))
                           Board_ledOn(LED_GREEN1);
                        else
                           Board_ledOff(LED_GREEN1);
                     }
                     Board_ledOff(LED_BLUE); //nothing to show
                  }
                  else if((enable_btstream && btstreaming) && (sdlogging && enable_sdlog)){
                     if(!(blink_cnt_2%10)){
                        if((P1OUT & BIT2) || (P1OUT & BIT1))
                           Board_ledOff(LED_BLUE+LED_GREEN1);
                        else{
                           if(last_led){
                              Board_ledOn(LED_BLUE);
                              last_led^=1;
                           }
                           else{
                              Board_ledOn(LED_GREEN1);
                              last_led^=1;
                           }
                        }
                     }
                  }else {
                     Board_ledOff(LED_GREEN1 + LED_BLUE);   //nothing to show
                  }
               }
            }
         }
       }
       break;
    case  8: break;                       // TB0CCR4
    case 10: break;                       // reserved
    case 12: break;                       // reserved
    case 14: break;                       // TBIFG overflow handler
   }
}


//BT start Timer
void BtStartDone(){
   bt_status = 1;
   configuring = 0;
}

void BtStart(){
   if(!bt_status){
      configuring = 1;
      BT_start();
   }
   BT_startDone_cb(BtStartDone);
}
void BtStop(){
   get_rcomm = 0;
   DMA2_disable();
   P1IES &= ~BIT0; //look for rising edge
   btIsConnected = 0;
   BT_connectionInterrupt(0);
   bt_status = 0;
   BT_disable();
   BT_rst_MessageProgress();
}

// Sample Timer
void SampleTimerStart(void) {
   uint16_t val_tb0;
   val_tb0 = GetTB0();
   if(preSampleMpuMag || preSampleBmpPress) {
      if(preSampleBmpPress && (((storedConfig[NV_CONFIG_SETUP_BYTE3]&0x30)>>4)==0)) {
         if(preSampleMpuMag) {
            TB0CCR0 = val_tb0 + *(uint16_t *)(storedConfig+NV_SAMPLING_RATE) + clk_90;    //9ms
            TB0CCR1 = val_tb0 + *(uint16_t *)(storedConfig+NV_SAMPLING_RATE);
            TB0CCR2 = val_tb0 + *(uint16_t *)(storedConfig+NV_SAMPLING_RATE) + clk_90_45; //9-4.5ms
            TB0CCTL1 = CCIE;
         } else {
            TB0CCR0 = val_tb0 + *(uint16_t *)(storedConfig+NV_SAMPLING_RATE) + clk_45;    //4.5ms
            TB0CCR2 = val_tb0 + *(uint16_t *)(storedConfig+NV_SAMPLING_RATE);
            TB0CCTL1 = 0;
         }
         TB0CCTL2 = CCIE;
      } else if(preSampleBmpPress && (((storedConfig[NV_CONFIG_SETUP_BYTE3]&0x30)>>4)==1)) {
         if(preSampleMpuMag) {
            TB0CCR0 = val_tb0 + *(uint16_t *)(storedConfig+NV_SAMPLING_RATE) + clk_90;    //9ms
            TB0CCR1 = val_tb0 + *(uint16_t *)(storedConfig+NV_SAMPLING_RATE);
            TB0CCR2 = val_tb0 + *(uint16_t *)(storedConfig+NV_SAMPLING_RATE) + clk_90_75; //9-7.5ms
            TB0CCTL1 = CCIE;
         } else {
            TB0CCR0 = val_tb0 + *(uint16_t *)(storedConfig+NV_SAMPLING_RATE) + clk_75;    //7.5ms
            TB0CCR2 = val_tb0 + *(uint16_t *)(storedConfig+NV_SAMPLING_RATE);
            TB0CCTL1 = 0;
         }
         TB0CCTL2 = CCIE;
      } else if(preSampleBmpPress && (((storedConfig[NV_CONFIG_SETUP_BYTE3]&0x30)>>4)==2)) {
         TB0CCR0 = val_tb0 + *(uint16_t *)(storedConfig+NV_SAMPLING_RATE) + clk_135;      //13.5ms
         TB0CCR2 = val_tb0 + *(uint16_t *)(storedConfig+NV_SAMPLING_RATE);
         TB0CCTL2 = CCIE;
         if(preSampleMpuMag) {
            TB0CCR1 = val_tb0 + *(uint16_t *)(storedConfig+NV_SAMPLING_RATE) + clk_135_90;//13.5-9ms
            TB0CCTL1 = CCIE;
         } else TB0CCTL1 = 0;
      } else if(preSampleBmpPress && (((storedConfig[NV_CONFIG_SETUP_BYTE3]&0x30)>>4)==3)) {
         TB0CCR0 = val_tb0 + *(uint16_t *)(storedConfig+NV_SAMPLING_RATE) + clk_250;      //25ms
         TB0CCR2 = val_tb0 + *(uint16_t *)(storedConfig+NV_SAMPLING_RATE);
         TB0CCTL2 = CCIE;
         if(preSampleMpuMag) {
            TB0CCR1 = val_tb0 + *(uint16_t *)(storedConfig+NV_SAMPLING_RATE) + clk_250_90;//25-9ms
            TB0CCTL1 = CCIE;
         } else TB0CCTL1 = 0;
      } else {
         TB0CCR0 = val_tb0 + *(uint16_t *)(storedConfig+NV_SAMPLING_RATE) + clk_90;       //9ms
         TB0CCR1 = val_tb0 + *(uint16_t *)(storedConfig+NV_SAMPLING_RATE);
         TB0CCTL1 = CCIE;
         TB0CCTL2 = 0;
      }
   }
   else {
      TB0CCTL1 = 0;
      TB0CCTL2 = 0;
      TB0CCR0 = val_tb0 + *(uint16_t *)(storedConfig+NV_SAMPLING_RATE);
   }
   TB0CCTL0 = CCIE;
   sample_timer_status = 1;
   TB0Start();
}

inline void SampleTimerStop(void) {
   sample_timer_status = 0;
   TB0Stop();
   TB0CCTL0 &= ~CCIE;
   TB0CCTL1 &= ~CCIE;
   TB0CCTL2 &= ~CCIE;
}

//read real time clock counter while timer is running
inline uint16_t GetRTC_short(void) {
   register uint16_t t0, t1;
   uint8_t ie;
   if(ie=(__get_SR_register()&0x0008))   //interrupts enabled?
      __disable_interrupt();
   t1 = RTCTIM0;
   do {t0=t1; t1=RTCTIM0;} while(t0!=t1);
   if(ie)
      __enable_interrupt();
   return t1;
}

//read TB0 counter while timer is running
inline uint16_t GetTB0(void) {
   register uint16_t t0, t1;
   uint8_t ie;
   if(ie=(__get_SR_register()&0x0008))   //interrupts enabled?
      __disable_interrupt();
   t1 =TB0R;
   do {t0=t1; t1=TB0R;} while(t0!=t1);
   if(ie)
      __enable_interrupt();
   return t1;
}

#pragma vector=TIMER0_B0_VECTOR
__interrupt void TIMER0_B0_ISR(void)
{
   TB0CCR0 = GetTB0() + *(uint16_t *)(storedConfig+NV_SAMPLING_RATE);
   if(currentBuffer){
      *((uint16_t *)(txBuff1+2)) = GetTB0();   //the first two bytes are packet type bytes. reserved for BTstream
      if(clock_freq == TCXO_CLOCK)
         *((uint16_t *)(txBuff1+4)) = GetTA0();
   }
   else {
      *((uint16_t *)(txBuff0+2)) = GetTB0();
      if(clock_freq == TCXO_CLOCK)
         *((uint16_t *)(txBuff0+4)) = GetTA0();
   }
   //start ADC conversion

   if(nbrAdcChans) {
      DMA0_enable();
      ADC_startConversion();
   } else {
      //no analog channels, so go straight to digital
      streamData = 1;
      __bic_SR_register_on_exit(LPM3_bits);
   }
}

uint8_t Dma0ConversionDone(void) {
   uint8_t adc_offset;
   if(batt_wait){
      batt_wait=0;
      ConfigureChannels();

      if(batt_stat == BATT_MID){
         if(*(uint16_t*)batt_val<2400){
            batt_stat = BATT_LOW;
         }else if(*(uint16_t*)batt_val<2600){
            batt_stat = BATT_MID;
         }else
            batt_stat = BATT_HIGH;
      }else if(batt_stat == BATT_LOW){
         if(*(uint16_t*)batt_val<2450){
            batt_stat = BATT_LOW;
         }else if(*(uint16_t*)batt_val<2550){
            batt_stat = BATT_MID;
         }else
            batt_stat = BATT_HIGH;
      }else{
         if(*(uint16_t*)batt_val<2400){
            batt_stat = BATT_LOW;
         }else if(*(uint16_t*)batt_val<2550){
            batt_stat = BATT_MID;
         }else
            batt_stat = BATT_HIGH;
      }
   }else{
      if(clock_freq == TCXO_CLOCK)
         adc_offset = 6;
      else
         adc_offset = 4;
      //Destination address for next transfer
      if(currentBuffer){
         DMA0_repeatTransfer(adcStartPtr, (uint16_t *)(txBuff0+adc_offset), nbrAdcChans);
      } else {
         DMA0_repeatTransfer(adcStartPtr, (uint16_t *)(txBuff1+adc_offset), nbrAdcChans);
      }
      ADC_disable();        //can disable ADC until next time sampleTimer fires (to save power)?
      DMA0_disable();
      streamData = 1;
   }
   return 1;
}

uint8_t Dma2ConversionDone(void) {
   uint8_t bt_getmac;

   DMA2_disable();
   bt_getmac = BT_getGetMacAddress();

   if(!*btrx_exp && (btIsConnected || bt_getmac)) {
      if(sync_enabled){
         if(get_rcomm){
            //6 bytes of RC info
            memcpy(rcomm_resp, btrx_buff, 6);
            memset(btrx_buff,0,14);
            ParseRcommResponse();
            get_rcomm = 0;
            return 1;
         }else if(btrx_buff[0] == ROUTINE_COMMUNICATION){
            //1 byte of RC command
            memset(btrx_buff,0,14);
            processCommand = 1;
            return 1;
         }
      }else if(bt_getmac){
         //14 bytes of BT mac address
         memcpy(mac, btrx_buff, 14);
         memset(btrx_buff,0,14);
         BT_setGetMacAddress(0);
         BT_setGoodCommand();
      }else {
         if(waitingForArgs) {
            if((!waitingForArgsLength) &&
               ((gAction == SET_EXG_REGS_COMMAND) && (waitingForArgs == 3)) ){
               args[0]=btrx_buff[0];
               args[1]=btrx_buff[1];
               args[2]=btrx_buff[2];
               DMA2SZ = args[2];
               DMA2_enable();
               waitingForArgsLength = args[2];
               return 0;
            }
            else if((!waitingForArgsLength) &&
               (((gAction == SET_DAUGHTER_CARD_ID_COMMAND) && (waitingForArgs == 1)) ||
               ((gAction == SET_DAUGHTER_CARD_MEM_COMMAND) && (waitingForArgs == 1))||
               ((gAction == SET_CENTER_COMMAND) && (waitingForArgs == 1))||
               ((gAction == SET_CONFIGTIME_COMMAND) && (waitingForArgs == 1))||
               ((gAction == SET_EXPID_COMMAND) && (waitingForArgs == 1))||
               ((gAction == SET_SHIMMERNAME_COMMAND) && (waitingForArgs == 1)))) {
               args[0]=btrx_buff[0];
               if(args[0]){
                  DMA2SZ = args[0];
                  DMA2_enable();
                  waitingForArgsLength = args[0];
                  return 0;
               }
            }
            if(waitingForArgsLength)
               memcpy(args+waitingForArgs, btrx_buff, waitingForArgsLength);
            else
               memcpy(args, btrx_buff, waitingForArgs);
            DMA2SZ = 1;
            DMA2_enable();
            waitingForArgsLength = 0;
            waitingForArgs = 0;
            processCommand = 1;
            argsSize = 0;
            return 1;
         } else {
            uint8_t data = btrx_buff[0];
            switch(data) {
            case INQUIRY_COMMAND:
            case GET_SAMPLING_RATE_COMMAND:
            case TOGGLE_LED_COMMAND:
            case START_STREAMING_COMMAND:
            case GET_STATUS_COMMAND:
            case GET_TRIAL_CONFIG_COMMAND:
            case START_SDBT_COMMAND:
            case GET_CONFIG_SETUP_BYTES_COMMAND:
            case STOP_STREAMING_COMMAND:
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
            case GET_MPU9150_SAMPLING_RATE_COMMAND:
            case GET_MPU9150_ACCEL_RANGE_COMMAND:
            case GET_BMP180_PRES_OVERSAMPLING_RATIO_COMMAND:
            case GET_INTERNAL_EXP_POWER_ENABLE_COMMAND:
            case RESET_TO_DEFAULT_CONFIGURATION_COMMAND:
            case RESET_CALIBRATION_VALUE_COMMAND:
            case GET_MPU9150_MAG_SENS_ADJ_VALS_COMMAND:
            case GET_CENTER_COMMAND:
            case GET_SHIMMERNAME_COMMAND:
            case GET_EXPID_COMMAND:
            case GET_MYID_COMMAND:
            case GET_NSHIMMER_COMMAND:
            case GET_CONFIGTIME_COMMAND:
            case GET_DIR_COMMAND:
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
            case SET_BMP180_PRES_OVERSAMPLING_RATIO_COMMAND:
            case SET_INTERNAL_EXP_POWER_ENABLE_COMMAND:
            case SET_GSR_RANGE_COMMAND:
            case SET_CENTER_COMMAND:
            case SET_SHIMMERNAME_COMMAND:
            case SET_EXPID_COMMAND:
            case SET_MYID_COMMAND:
            case SET_NSHIMMER_COMMAND:
            case SET_CONFIGTIME_COMMAND:
               waitingForArgs = 1;
               DMA2SZ = 1;
               DMA2_enable();
               gAction = data;
               return 0;
            case SET_SAMPLING_RATE_COMMAND:
            case GET_DAUGHTER_CARD_ID_COMMAND:
            case SET_DAUGHTER_CARD_ID_COMMAND:
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
               return 0;
            }
         }
      }
   } else {
      if(!memcmp(btrx_buff, btrx_exp, strlen((char*)btrx_exp))) {
         memset(btrx_buff,0,14);
         BT_setGoodCommand();
      }else{
         _NOP();  //bad command trap: reaching here = serious BT problem
      }
   }

   return 0;
}


void ProcessCommand(void) {
   uint32_t config_time;
   uint8_t my_config_time[4], update_sdconfig = 0;
   if(sync_enabled){
      if(btIsConnected) {
         uint8_t resPacket[6];
         uint16_t packet_length = 0;
         uint32_t my_local_time_long;

         *(resPacket + packet_length++) = ACK_COMMAND_PROCESSED;

         *(resPacket + packet_length++) = sensing;
         my_local_time_long = GetRTC();
         *(uint32_t*)(resPacket + packet_length) = my_local_time_long;
         packet_length+=4;

         BT_write(resPacket, packet_length);

         _NOP();
      }
   } else {
      switch(gAction) {
      case INQUIRY_COMMAND:
         inquiryResponse = 1;
         break;
      case GET_SAMPLING_RATE_COMMAND:
         samplingRateResponse = 1;
         break;
      case TOGGLE_LED_COMMAND:
        toggle_redled ^= 1;
         //Board_ledToggle(LED_RED);
         break;
      case START_STREAMING_COMMAND:
        enable_btstream = 1;
        enable_sdlog = 0;
        dir_new = 1;
        CheckSdInslot();
         startSensing = 1;
         break;
      case START_SDBT_COMMAND:
        enable_btstream = 1;
        startSensing = 1;
        enable_sdlog = 1;
        break;
      case STOP_STREAMING_COMMAND:
         if(sensing) {
            stopSensing = 1;
            return;
         }
         break;
      case SET_SENSORS_COMMAND:
         storedConfig[NV_SENSORS0] = args[0];
         storedConfig[NV_SENSORS1] = args[1];
         storedConfig[NV_SENSORS2] = args[2];
          if(storedConfig[NV_SENSORS0] & SENSOR_GSR)  //they are sharing adc1, so ban intch1 when gsr is on
            storedConfig[NV_SENSORS1] &= ~SENSOR_INT_A1;
         InfoMem_write((void*)NV_SENSORS0, &storedConfig[NV_SENSORS0], 3);
         sdhead_text[SDH_SENSORS0] = storedConfig[NV_SENSORS0];
         sdhead_text[SDH_SENSORS1] = storedConfig[NV_SENSORS1];
         sdhead_text[SDH_SENSORS2] = storedConfig[NV_SENSORS2];
         update_sdconfig = 1;
         configureChannels = 1;
         if(sensing) {
            stopSensing = 1;     //stops streaming before configuring channels
            startSensing = 1;    //starts streaming after configuring channels
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
      case GET_TRIAL_CONFIG_COMMAND:
        trialConfigResponse = 1;
         break;
      case SET_TRIAL_CONFIG_COMMAND:
         storedConfig[NV_TRIAL_CONFIG0] = args[0];
         storedConfig[NV_TRIAL_CONFIG1] = args[1];
         storedConfig[NV_BT_INTERVAL] = args[2];
         if(storedConfig[NV_TRIAL_CONFIG1]&SDH_SINGLETOUCH){
            storedConfig[NV_TRIAL_CONFIG0] |= SDH_USER_BUTTON_ENABLE;
            storedConfig[NV_TRIAL_CONFIG0] |= SDH_TIME_SYNC;
         }
         if(storedConfig[NV_BT_INTERVAL]<RC_INT_C)
            storedConfig[NV_BT_INTERVAL] = RC_INT_C;
         InfoMem_write((void*)NV_SAMPLING_RATE, &storedConfig[NV_TRIAL_CONFIG0], 3);
         sdhead_text[SDH_TRIAL_CONFIG0] = storedConfig[NV_TRIAL_CONFIG0];
         sdhead_text[SDH_TRIAL_CONFIG1] = storedConfig[NV_TRIAL_CONFIG1];
         sdhead_text[SDH_BROADCAST_INTERVAL] = storedConfig[NV_BT_INTERVAL];
         update_sdconfig = 1;
         break;
      case GET_CENTER_COMMAND:
         centerResponse = 1;
         break;
      case SET_CENTER_COMMAND:
         memcpy(centername, &args[1], args[0]);
         centername[args[0]]='\0';
         SetName();
         update_sdconfig = 1;
         break;
      case GET_SHIMMERNAME_COMMAND:
         shimmerNameResponse = 1;
         break;
      case SET_SHIMMERNAME_COMMAND:
         memcpy(shimmername, &args[1], args[0]);
         shimmername[args[0]]='\0';
         SetName();
         update_sdconfig = 1;
         break;
      case GET_EXPID_COMMAND:
         expIDResponse = 1;
         break;
      case SET_EXPID_COMMAND:
         memcpy(exp_id_name, &args[1], args[0]);
         exp_id_name[args[0]]='\0';
         SetName();
         update_sdconfig = 1;
         break;
      case GET_CONFIGTIME_COMMAND:
          configTimeResponse = 1;
         break;
      case GET_DIR_COMMAND:
         dirResponse = 1;
         break;
      case SET_CONFIGTIME_COMMAND:
         memcpy(config_time_text, &args[1], args[0]);
         config_time_text[args[0]]='\0';
         SetName();
         config_time = atol((char*)config_time_text);
         my_config_time[3] = *((uint8_t*)&config_time);
         my_config_time[2] = *(((uint8_t*)&config_time)+1);
         my_config_time[1] = *(((uint8_t*)&config_time)+2);
         my_config_time[0] = *(((uint8_t*)&config_time)+3);
         memcpy(&sdhead_text[SDH_CONFIG_TIME_0], my_config_time, 4);
         update_sdconfig = 1;
         break;
      case GET_NSHIMMER_COMMAND:
         nshimmerResponse = 1;
         break;
      case SET_NSHIMMER_COMMAND:
         sdhead_text[SDH_NSHIMMER] = args[0];
         update_sdconfig = 1;
         break;
      case GET_MYID_COMMAND:
         myIDResponse = 1;
         break;
      case SET_MYID_COMMAND:
         sdhead_text[SDH_MYTRIAL_ID] = args[0];
         update_sdconfig = 1;
         break;
      case SET_LSM303DLHC_ACCEL_RANGE_COMMAND:
         if(args[0] < 4)
            storedConfig[NV_CONFIG_SETUP_BYTE0] = (storedConfig[NV_CONFIG_SETUP_BYTE0]&0xF3) + ((args[0]&0x03)<<2);
         else
            storedConfig[NV_CONFIG_SETUP_BYTE0] &= 0xF3;
         InfoMem_write((void*)NV_CONFIG_SETUP_BYTE0, &storedConfig[NV_CONFIG_SETUP_BYTE0], 1);
         sdhead_text[SDH_CONFIG_SETUP_BYTE0] = storedConfig[NV_CONFIG_SETUP_BYTE0];
        update_sdconfig = 1;
         if(sensing) {
            stopSensing = 1;     //stops streaming before configuring channels
            startSensing = 1;    //starts streaming after configuring channels
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
         bmp180CalibrationCoefficientsResponse = 1;
         break;
      case GET_MPU9150_SAMPLING_RATE_COMMAND:
         mpu9150SamplingRateResponse = 1;
         break;
      case GET_MPU9150_ACCEL_RANGE_COMMAND:
         mpu9150AccelRangeResponse = 1;
         break;
      case GET_BMP180_PRES_OVERSAMPLING_RATIO_COMMAND:
         bmp180OversamplingRatioResponse = 1;
         break;
      case GET_INTERNAL_EXP_POWER_ENABLE_COMMAND:
         internalExpPowerEnableResponse = 1;
         break;
      case GET_MPU9150_MAG_SENS_ADJ_VALS_COMMAND:
         mpu9150MagSensAdjValsResponse = 1;
         break;
      case GET_EXG_REGS_COMMAND:
         if(args[0]<2 && args[1]<10 && args[2]<11) {
            exgChip = args[0];
            exgStartAddr = args[1];
            exgLength = args[2];
         } else
            exgLength = 0;
         exgRegsResponse = 1;
         break;
      case SET_LSM303DLHC_ACCEL_SAMPLING_RATE_COMMAND:
         if(args[0] < 10)
            storedConfig[NV_CONFIG_SETUP_BYTE0] = (storedConfig[NV_CONFIG_SETUP_BYTE0]&0x0F) + ((args[0]&0x0F)<<4);
         else
            storedConfig[NV_CONFIG_SETUP_BYTE0] = (storedConfig[NV_CONFIG_SETUP_BYTE0]&0x0F) + (LSM303DLHC_ACCEL_100HZ<<4);
         InfoMem_write((void*)NV_CONFIG_SETUP_BYTE0, &storedConfig[NV_CONFIG_SETUP_BYTE0], 1);
         sdhead_text[SDH_CONFIG_SETUP_BYTE0] = storedConfig[NV_CONFIG_SETUP_BYTE0];
          update_sdconfig = 1;
         if(sensing) {
            stopSensing = 1;     //stops streaming before configuring channels
            startSensing = 1;    //starts streaming after configuring channels
         }
         break;
      case SET_LSM303DLHC_MAG_GAIN_COMMAND:
         if(args[0]>0 && args[0]<8)
            storedConfig[NV_CONFIG_SETUP_BYTE2] = (storedConfig[NV_CONFIG_SETUP_BYTE2]&0x1F) + ((args[0]&0x07)<<5);
         else
            storedConfig[NV_CONFIG_SETUP_BYTE2] = (storedConfig[NV_CONFIG_SETUP_BYTE2]&0x1F) + (LSM303DLHC_MAG_1_3G<<5);
         InfoMem_write((void*)NV_CONFIG_SETUP_BYTE2, &storedConfig[NV_CONFIG_SETUP_BYTE2], 1);
         sdhead_text[SDH_CONFIG_SETUP_BYTE2] = storedConfig[NV_CONFIG_SETUP_BYTE2];
         update_sdconfig = 1;
         if(sensing) {
            stopSensing = 1;     //stops streaming before configuring channels
            startSensing = 1;    //starts streaming after configuring channels
         }
         break;
      case SET_LSM303DLHC_MAG_SAMPLING_RATE_COMMAND:
         if(args[0] < 8)
            storedConfig[NV_CONFIG_SETUP_BYTE2] = (storedConfig[NV_CONFIG_SETUP_BYTE2]&0xE3) + ((args[0]&0x07)<<2);
         else
            storedConfig[NV_CONFIG_SETUP_BYTE2] = (storedConfig[NV_CONFIG_SETUP_BYTE2]&0xE3) + (LSM303DLHC_MAG_75HZ<<2);
         InfoMem_write((void*)NV_CONFIG_SETUP_BYTE2, &storedConfig[NV_CONFIG_SETUP_BYTE2], 1);
         sdhead_text[SDH_CONFIG_SETUP_BYTE2] = storedConfig[NV_CONFIG_SETUP_BYTE2];
         update_sdconfig = 1;
         if(sensing) {
            stopSensing = 1;     //stops streaming before configuring channels
            startSensing = 1;    //starts streaming after configuring channels
         }
         break;
      case SET_LSM303DLHC_ACCEL_LPMODE_COMMAND:
         if(args[0] == 1){
            storedConfig[NV_CONFIG_SETUP_BYTE0] = (storedConfig[NV_CONFIG_SETUP_BYTE0]&0xFD) + 0x02;
            storedConfig[NV_TRIAL_CONFIG1] |= SDH_ACCEL_LPM;
         }else{
            storedConfig[NV_CONFIG_SETUP_BYTE0] &= 0xFD;
            storedConfig[NV_TRIAL_CONFIG1] &= ~SDH_ACCEL_LPM;
         }
         InfoMem_write((void*)NV_CONFIG_SETUP_BYTE0, &storedConfig[NV_CONFIG_SETUP_BYTE0], 1);
         sdhead_text[SDH_TRIAL_CONFIG1] = storedConfig[NV_TRIAL_CONFIG1];
         update_sdconfig = 1;
         if(sensing) {
            stopSensing = 1;     //stops streaming before configuring channels
            startSensing = 1;    //starts streaming after configuring channels
         }
         break;
      case SET_LSM303DLHC_ACCEL_HRMODE_COMMAND:
         if(args[0] == 1){
            storedConfig[NV_CONFIG_SETUP_BYTE0] = (storedConfig[NV_CONFIG_SETUP_BYTE0]&0xFE) + 0x01;
            storedConfig[NV_TRIAL_CONFIG1] |= SDH_ACCEL_HRM;
         }else{
            storedConfig[NV_CONFIG_SETUP_BYTE0] &= 0xFE;
            storedConfig[NV_TRIAL_CONFIG1] &= ~SDH_ACCEL_HRM;
         }
         InfoMem_write((void*)NV_CONFIG_SETUP_BYTE0, &storedConfig[NV_CONFIG_SETUP_BYTE0], 1);
         sdhead_text[SDH_TRIAL_CONFIG1] = storedConfig[NV_TRIAL_CONFIG1];
         update_sdconfig = 1;
         if(sensing) {
            stopSensing = 1;     //stops streaming before configuring channels
            startSensing = 1;    //starts streaming after configuring channels
         }
         break;
      case SET_MPU9150_GYRO_RANGE_COMMAND:
         if(args[0] < 4)
            storedConfig[NV_CONFIG_SETUP_BYTE2] = (storedConfig[NV_CONFIG_SETUP_BYTE2]&0xFC) + (args[0]&0x03);
         else
            storedConfig[NV_CONFIG_SETUP_BYTE2] = (storedConfig[NV_CONFIG_SETUP_BYTE2]&0xFC) + MPU9150_GYRO_500DPS;
         InfoMem_write((void*)NV_CONFIG_SETUP_BYTE2, &storedConfig[NV_CONFIG_SETUP_BYTE2], 1);
         sdhead_text[SDH_CONFIG_SETUP_BYTE2] = storedConfig[NV_CONFIG_SETUP_BYTE2];
         update_sdconfig = 1;
         if(sensing) {
            stopSensing = 1;     //stops streaming before configuring channels
            startSensing = 1;    //starts streaming after configuring channels
         }
         break;
      case SET_MPU9150_SAMPLING_RATE_COMMAND:
         storedConfig[NV_CONFIG_SETUP_BYTE1] = args[0];
         InfoMem_write((void*)NV_CONFIG_SETUP_BYTE1, &storedConfig[NV_CONFIG_SETUP_BYTE1], 1);
         sdhead_text[SDH_CONFIG_SETUP_BYTE1] = storedConfig[NV_CONFIG_SETUP_BYTE1];
         update_sdconfig = 1;
         if(sensing) {
            stopSensing = 1;     //stops streaming before configuring channels
            startSensing = 1;    //starts streaming after configuring channels
         }
         break;
      case SET_MPU9150_ACCEL_RANGE_COMMAND:
         if(args[0] < 4)
            storedConfig[NV_CONFIG_SETUP_BYTE3] = (storedConfig[NV_CONFIG_SETUP_BYTE3]&0x3F) + ((args[0]&0x03)<<6);
         else
            storedConfig[NV_CONFIG_SETUP_BYTE3] = (storedConfig[NV_CONFIG_SETUP_BYTE3]&0x3F) + (ACCEL_2G<<6);
         InfoMem_write((void*)NV_CONFIG_SETUP_BYTE3, &storedConfig[NV_CONFIG_SETUP_BYTE3], 1);
         update_sdconfig = 1;
         if(sensing) {
            stopSensing = 1;     //stops streaming before configuring channels
            startSensing = 1;    //starts streaming after configuring channels
         }
         break;
      case SET_BMP180_PRES_OVERSAMPLING_RATIO_COMMAND:
         if(args[0] < 4)
            storedConfig[NV_CONFIG_SETUP_BYTE3] = (storedConfig[NV_CONFIG_SETUP_BYTE3]&0xCF) + ((args[0]&0x03)<<4);
         else
            storedConfig[NV_CONFIG_SETUP_BYTE3] = (storedConfig[NV_CONFIG_SETUP_BYTE3]&0xCF) + (BMP180_OSS_1<<4);
         InfoMem_write((void*)NV_CONFIG_SETUP_BYTE3, &storedConfig[NV_CONFIG_SETUP_BYTE3], 1);
         sdhead_text[SDH_CONFIG_SETUP_BYTE3] = storedConfig[NV_CONFIG_SETUP_BYTE3];
         update_sdconfig = 1;
         break;
      case SET_INTERNAL_EXP_POWER_ENABLE_COMMAND:
         if(args[0] == 1)
            storedConfig[NV_CONFIG_SETUP_BYTE3] = (storedConfig[NV_CONFIG_SETUP_BYTE3]&0xFE) + (args[0]&0x01);
         else
            storedConfig[NV_CONFIG_SETUP_BYTE3] &= 0xFE;
         InfoMem_write((void*)NV_CONFIG_SETUP_BYTE3, &storedConfig[NV_CONFIG_SETUP_BYTE3], 1);
         sdhead_text[SDH_TRIAL_CONFIG1] = sdhead_text[SDH_TRIAL_CONFIG1]&~EXP_POWER_ENABLE + (storedConfig[NV_CONFIG_SETUP_BYTE3]&0x01)*EXP_POWER_ENABLE;
         sdhead_text[SDH_TRIAL_CONFIG1] = sdhead_text[SDH_TRIAL_CONFIG1];
         update_sdconfig = 1;
         if(sensing) {
            stopSensing = 1;     //stops streaming before configuring channels
            startSensing = 1;    //starts streaming after configuring channels
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
         InfoMem_write((void*)NV_CONFIG_SETUP_BYTE0, &storedConfig[NV_CONFIG_SETUP_BYTE0], 4);
         Config2SdHead();
         update_sdconfig = 1;
         if(sensing) {
            stopSensing = 1;     //stops streaming before configuring channels
            startSensing = 1;    //starts streaming after configuring channels
         }
         break;
      case SET_SAMPLING_RATE_COMMAND:
         *(uint16_t *)(storedConfig+NV_SAMPLING_RATE) = *(uint16_t *)args;
         InfoMem_write((void*)NV_SAMPLING_RATE, &storedConfig[NV_SAMPLING_RATE], 2);
         sdhead_text[SDH_SAMPLE_RATE_0] = storedConfig[NV_SAMPLING_RATE+1];
         sdhead_text[SDH_SAMPLE_RATE_1] = storedConfig[NV_SAMPLING_RATE];
         update_sdconfig = 1;
         if(sensing) {
            //restart sampling timer to use new sampling rate
            stopSensing = 1;
            startSensing = 1;
         }
         break;
      case SET_A_ACCEL_CALIBRATION_COMMAND:
         memcpy(&storedConfig[NV_A_ACCEL_CALIBRATION], args, 21);
         InfoMem_write((void*)NV_A_ACCEL_CALIBRATION, &storedConfig[NV_A_ACCEL_CALIBRATION], 21);
         memcpy(&sdhead_text[SDH_A_ACCEL_CALIBRATION], &storedConfig[NV_A_ACCEL_CALIBRATION], 21);
         break;
      case GET_A_ACCEL_CALIBRATION_COMMAND:
         aAccelCalibrationResponse = 1;
         break;
      case SET_MPU9150_GYRO_CALIBRATION_COMMAND:
         memcpy(&storedConfig[NV_MPU9150_GYRO_CALIBRATION], args, 21);
         InfoMem_write((void*)NV_MPU9150_GYRO_CALIBRATION, &storedConfig[NV_MPU9150_GYRO_CALIBRATION], 21);
         memcpy(&sdhead_text[SDH_MPU9150_GYRO_CALIBRATION], &storedConfig[NV_MPU9150_GYRO_CALIBRATION], 21);
         break;
      case GET_MPU9150_GYRO_CALIBRATION_COMMAND:
         gyroCalibrationResponse = 1;
         break;
      case SET_LSM303DLHC_MAG_CALIBRATION_COMMAND:
         memcpy(&storedConfig[NV_LSM303DLHC_MAG_CALIBRATION], args, 21);
         InfoMem_write((void*)NV_LSM303DLHC_MAG_CALIBRATION, &storedConfig[NV_LSM303DLHC_MAG_CALIBRATION], 21);
         memcpy(&sdhead_text[SDH_LSM303DLHC_MAG_CALIBRATION], &storedConfig[NV_LSM303DLHC_MAG_CALIBRATION], 21);
         break;
      case GET_LSM303DLHC_MAG_CALIBRATION_COMMAND:
         magCalibrationResponse = 1;
         break;
      case SET_LSM303DLHC_ACCEL_CALIBRATION_COMMAND:
         memcpy(&storedConfig[NV_LSM303DLHC_ACCEL_CALIBRATION], args, 21);
         InfoMem_write((void*)NV_LSM303DLHC_ACCEL_CALIBRATION, &storedConfig[NV_LSM303DLHC_ACCEL_CALIBRATION], 21);
         memcpy(&sdhead_text[SDH_LSM303DLHC_ACCEL_CALIBRATION], &storedConfig[NV_LSM303DLHC_ACCEL_CALIBRATION], 21);
         break;
      case SET_GSR_RANGE_COMMAND:
         if(args[0] <= 4)
            storedConfig[NV_CONFIG_SETUP_BYTE3] = (storedConfig[NV_CONFIG_SETUP_BYTE3]&0xF1) + ((args[0]&0x07)<<1);
         else
            storedConfig[NV_CONFIG_SETUP_BYTE3] = (storedConfig[NV_CONFIG_SETUP_BYTE3]&0xF1) + (HW_RES_40K<<1);
         InfoMem_write((void*)NV_CONFIG_SETUP_BYTE3, &storedConfig[NV_CONFIG_SETUP_BYTE3], 1);
         sdhead_text[SDH_CONFIG_SETUP_BYTE3] = storedConfig[NV_CONFIG_SETUP_BYTE3];
         update_sdconfig = 1;
         if(sensing) {
            stopSensing = 1;
            startSensing = 1;
         }
         break;
      case SET_EXG_REGS_COMMAND:
         if (args[0]<2 && args[1]<10 && args[2]<11) {
            if(args[0]) {
               memcpy((storedConfig+NV_EXG_ADS1292R_2_CONFIG1+args[1]), (args+3), args[2]);
               InfoMem_write((void*)(NV_EXG_ADS1292R_2_CONFIG1+args[1]), (storedConfig+NV_EXG_ADS1292R_2_CONFIG1+args[1]), args[2]);
               memcpy(sdhead_text+SDH_EXG_ADS1292R_1_CONFIG1, storedConfig+NV_EXG_ADS1292R_2_CONFIG1,args[2]);
            } else {
               memcpy((storedConfig+NV_EXG_ADS1292R_1_CONFIG1+args[1]), (args+3), args[2]);
               InfoMem_write((void*)(NV_EXG_ADS1292R_1_CONFIG1+args[1]), (storedConfig+NV_EXG_ADS1292R_1_CONFIG1+args[1]), args[2]);
               memcpy(sdhead_text+SDH_EXG_ADS1292R_2_CONFIG1, storedConfig+NV_EXG_ADS1292R_1_CONFIG1,args[2]);
            }
         update_sdconfig = 1;
         }
         break;
      case RESET_TO_DEFAULT_CONFIGURATION_COMMAND:
         SetDefaultConfiguration();
         update_sdconfig = 1;
         configureChannels = 1;
         if(sensing) {
            stopSensing = 1;
            startSensing = 1;
         }
         break;
      case RESET_CALIBRATION_VALUE_COMMAND:
         memset(&storedConfig[NV_A_ACCEL_CALIBRATION], 0xFF, NV_NUM_CALIBRATION_BYTES);
         InfoMem_write((void*)NV_A_ACCEL_CALIBRATION, &storedConfig[NV_A_ACCEL_CALIBRATION], NV_NUM_CALIBRATION_BYTES);
         memcpy(&sdhead_text[SDH_LSM303DLHC_ACCEL_CALIBRATION], &storedConfig[NV_LSM303DLHC_ACCEL_CALIBRATION], 21);
         memcpy(&sdhead_text[SDH_MPU9150_GYRO_CALIBRATION], &storedConfig[NV_MPU9150_GYRO_CALIBRATION], 21);
         memcpy(&sdhead_text[SDH_LSM303DLHC_MAG_CALIBRATION], &storedConfig[NV_LSM303DLHC_MAG_CALIBRATION], 21);
         memcpy(&sdhead_text[SDH_A_ACCEL_CALIBRATION], &storedConfig[NV_A_ACCEL_CALIBRATION], 21);
         update_sdconfig = 1;
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
         if((dcMemLength<=16) && (dcMemOffset<=15) && (dcMemLength+dcMemOffset<=16))
            dcIdResponse = 1;
         break;
      case SET_DAUGHTER_CARD_ID_COMMAND:
         dcMemLength = args[0];
         dcMemOffset = args[1];
         if((dcMemLength<=16) && (dcMemOffset<=15) && (dcMemLength+dcMemOffset<=16)) {
            CAT24C16_init();
            CAT24C16_write(dcMemOffset, dcMemLength, args+2);
            CAT24C16_powerOff();
         }
         break;
      case GET_DAUGHTER_CARD_MEM_COMMAND:
         dcMemLength = args[0];
         dcMemOffset = args[1] + (args[2] << 8);
         if((dcMemLength<=128) && (dcMemOffset<=2031) && (dcMemLength+dcMemOffset<=2032))
            dcMemResponse = 1;
         break;
      case SET_DAUGHTER_CARD_MEM_COMMAND:
         dcMemLength = args[0];
         dcMemOffset = args[1] + (args[2] << 8);
         if((dcMemLength<=128) && (dcMemOffset<=2031) && (dcMemLength+dcMemOffset<=2032)) {
            CAT24C16_init();
            CAT24C16_write(dcMemOffset+16, dcMemLength, args+3);
            CAT24C16_powerOff();
         }
         break;
      default:;
      }
      sendAck = 1;
      sendResponse = 1;
      if(update_sdconfig)
         UpdateSdConfig();
   }
}
void SendResponse(void) {
   uint16_t packet_length = 0;

   if(btIsConnected) {
      if(sendAck) {
         *(resPacket + packet_length++) = ACK_COMMAND_PROCESSED;
         sendAck = 0;
      }
      if(inquiryResponse) {
         *(resPacket + packet_length++) = INQUIRY_RESPONSE;
         *(uint16_t *)(resPacket + packet_length) = *(uint16_t *)(storedConfig+NV_SAMPLING_RATE);  //ADC sampling rate
         packet_length+=2;
         memcpy((resPacket + packet_length), (storedConfig+NV_CONFIG_SETUP_BYTE0), 4);             //4 config bytes
         packet_length+=4;
         *(resPacket + packet_length++) = nbrAdcChans + nbrDigiChans;                              //number of data channels
         *(resPacket + packet_length++) = storedConfig[NV_BUFFER_SIZE];                            //buffer size
         memcpy((resPacket + packet_length), channelContents, (nbrAdcChans+nbrDigiChans));
         packet_length += nbrAdcChans + nbrDigiChans;
         inquiryResponse = 0;
      } else if(samplingRateResponse) {
         *(resPacket + packet_length++) = SAMPLING_RATE_RESPONSE;
         *(uint16_t *)(resPacket + packet_length) = *(uint16_t *)(storedConfig+NV_SAMPLING_RATE);  //ADC sampling rate
         packet_length+=2;
         samplingRateResponse = 0;
      } else if(lsm303dlhcAccelRangeResponse) {
         *(resPacket + packet_length++) = LSM303DLHC_ACCEL_RANGE_RESPONSE;
         *(resPacket + packet_length++) = (storedConfig[NV_CONFIG_SETUP_BYTE0] & 0x0C) >> 2;
         lsm303dlhcAccelRangeResponse = 0;
      } else if(lsm303dlhcMagGainResponse) {
         *(resPacket + packet_length++) = LSM303DLHC_MAG_GAIN_RESPONSE;
         *(resPacket + packet_length++) = (storedConfig[NV_CONFIG_SETUP_BYTE2] & 0xE0) >> 5;
         lsm303dlhcMagGainResponse = 0;
      } else if(lsm303dlhcMagSamplingRateResponse) {
         *(resPacket + packet_length++) = LSM303DLHC_MAG_SAMPLING_RATE_RESPONSE;
         *(resPacket + packet_length++) = (storedConfig[NV_CONFIG_SETUP_BYTE2] & 0x1C) >> 2;
         lsm303dlhcMagSamplingRateResponse = 0;
      } else if(dockedResponse) {
         *(resPacket + packet_length++) = INSTREAM_CMD_RESPONSE;
          *(resPacket + packet_length++) = STATUS_RESPONSE;
          *(resPacket + packet_length++) = (docked & 0x01) + ((sensing  & 0x01)<<1);   //2 valid bits: sensing+docked
          dockedResponse = 0;
      } else if(trialConfigResponse) {
          *(resPacket + packet_length++) = TRIAL_CONFIG_RESPONSE;
          memcpy((resPacket + packet_length), (storedConfig+NV_TRIAL_CONFIG0), 3);     //2 trial config bytes + 1 interval byte
          packet_length+=3;
          trialConfigResponse = 0;
      } else if(centerResponse) {
          *(resPacket + packet_length++) = CENTER_RESPONSE;
          *(resPacket + packet_length++) = strlen((char*)centername);
          memcpy((resPacket + packet_length), centername, strlen((char*)centername));
          packet_length+=strlen((char*)centername);
          centerResponse = 0;
      } else if(shimmerNameResponse) {
         *(resPacket + packet_length++) = SHIMMERNAME_RESPONSE;
         *(resPacket + packet_length++) = strlen((char*)shimmername);
         memcpy((resPacket + packet_length), shimmername, strlen((char*)shimmername));
         packet_length+=strlen((char*)shimmername);
         shimmerNameResponse = 0;
      } else if(expIDResponse) {
         *(resPacket + packet_length++) = EXPID_RESPONSE;
         *(resPacket + packet_length++) = strlen((char*)exp_id_name);
         memcpy((resPacket + packet_length), exp_id_name, strlen((char*)exp_id_name));
         packet_length+=strlen((char*)exp_id_name);
         expIDResponse = 0;
      } else if(configTimeResponse) {
         *(resPacket + packet_length++) = CONFIGTIME_RESPONSE;
         *(resPacket + packet_length++) = strlen((char*)config_time_text);
         memcpy((resPacket + packet_length), config_time_text, strlen((char*)config_time_text));
         packet_length+=strlen((char*)config_time_text);
         configTimeResponse = 0;
      } else if(dirResponse) {
         uint8_t dir_len = strlen((char*)filename)-3;
         *(resPacket + packet_length++) = INSTREAM_CMD_RESPONSE;
         *(resPacket + packet_length++) = DIR_RESPONSE;
         *(resPacket + packet_length++) = dir_len;
         memcpy((resPacket + packet_length), filename, dir_len);
         packet_length+=dir_len;
         dirResponse = 0;
      } else if(nshimmerResponse) {
         *(resPacket + packet_length++) = NSHIMMER_RESPONSE;
         *(resPacket + packet_length++) = sdhead_text[SDH_NSHIMMER];
         nshimmerResponse = 0;
      } else if(myIDResponse) {
          *(resPacket + packet_length++) = MYID_RESPONSE;
          *(resPacket + packet_length++) = sdhead_text[SDH_MYTRIAL_ID];
          myIDResponse = 0;
      } else if(lsm303dlhcAccelSamplingRateResponse) {
          *(resPacket + packet_length++) = LSM303DLHC_ACCEL_SAMPLING_RATE_RESPONSE;
          *(resPacket + packet_length++) = (storedConfig[NV_CONFIG_SETUP_BYTE0] & 0xF0) >> 4;
          lsm303dlhcAccelSamplingRateResponse = 0;
      } else if(lsm303dlhcAccelLPModeResponse) {
         *(resPacket + packet_length++) = LSM303DLHC_ACCEL_LPMODE_RESPONSE;
         *(resPacket + packet_length++) = (storedConfig[NV_CONFIG_SETUP_BYTE0] & 0x02) >> 1;
         lsm303dlhcAccelLPModeResponse = 0;
      } else if(lsm303dlhcAccelHRModeResponse) {
         *(resPacket + packet_length++) = LSM303DLHC_ACCEL_HRMODE_RESPONSE;
         *(resPacket + packet_length++) = storedConfig[NV_CONFIG_SETUP_BYTE0] & 0x01;
         lsm303dlhcAccelHRModeResponse = 0;
      } else if(mpu9150GyroRangeResponse) {
         *(resPacket + packet_length++) = MPU9150_GYRO_RANGE_RESPONSE;
         *(resPacket + packet_length++) = storedConfig[NV_CONFIG_SETUP_BYTE2] & 0x03;
         mpu9150GyroRangeResponse = 0;
      } else if(bmp180CalibrationCoefficientsResponse) {
         *(resPacket + packet_length++) = BMP180_CALIBRATION_COEFFICIENTS_RESPONSE;
         BMP180_init();
         BMP180_getCalibCoeff(resPacket+packet_length);
         packet_length += 22;
         bmp180CalibrationCoefficientsResponse = 0;
      } else if(mpu9150SamplingRateResponse) {
         *(resPacket + packet_length++) = MPU9150_SAMPLING_RATE_RESPONSE;
         *(resPacket + packet_length++) = storedConfig[NV_CONFIG_SETUP_BYTE1];
         mpu9150SamplingRateResponse = 0;
      } else if(mpu9150AccelRangeResponse) {
         *(resPacket + packet_length++) = MPU9150_ACCEL_RANGE_RESPONSE;
         *(resPacket + packet_length++) = (storedConfig[NV_CONFIG_SETUP_BYTE3]&0xC0) >> 6;
         mpu9150AccelRangeResponse = 0;
      } else if(bmp180OversamplingRatioResponse) {
         *(resPacket + packet_length++) = BMP180_PRES_OVERSAMPLING_RATIO_RESPONSE;
         *(resPacket + packet_length++) = (storedConfig[NV_CONFIG_SETUP_BYTE3]&0x30) >> 4;
         bmp180OversamplingRatioResponse = 0;
      } else if(internalExpPowerEnableResponse) {
         *(resPacket + packet_length++) = INTERNAL_EXP_POWER_ENABLE_RESPONSE;
         *(resPacket + packet_length++) = storedConfig[NV_CONFIG_SETUP_BYTE3]&0x01;
         internalExpPowerEnableResponse = 0;
      } else if(configSetupBytesResponse) {
         *(resPacket + packet_length++) = CONFIG_SETUP_BYTES_RESPONSE;
         memcpy((resPacket+packet_length), &storedConfig[NV_CONFIG_SETUP_BYTE0], 4);
         packet_length += 4;
         configSetupBytesResponse = 0;
      } else if(aAccelCalibrationResponse) {
         *(resPacket + packet_length++) = A_ACCEL_CALIBRATION_RESPONSE;
         memcpy((resPacket+packet_length), &storedConfig[NV_A_ACCEL_CALIBRATION], 21);
         packet_length += 21;
         aAccelCalibrationResponse = 0;
      } else if(gyroCalibrationResponse) {
         *(resPacket + packet_length++) = MPU9150_GYRO_CALIBRATION_RESPONSE;
         memcpy((resPacket+packet_length), &storedConfig[NV_MPU9150_GYRO_CALIBRATION], 21);
         packet_length += 21;
         gyroCalibrationResponse = 0;
      } else if(magCalibrationResponse) {
         *(resPacket + packet_length++) = LSM303DLHC_MAG_CALIBRATION_RESPONSE;
         memcpy((resPacket+packet_length), &storedConfig[NV_LSM303DLHC_MAG_CALIBRATION], 21);
         packet_length += 21;
         magCalibrationResponse = 0;
      } else if(dAccelCalibrationResponse) {
         *(resPacket + packet_length++) = LSM303DLHC_ACCEL_CALIBRATION_RESPONSE;
         memcpy((resPacket+packet_length), &storedConfig[NV_LSM303DLHC_ACCEL_CALIBRATION], 21);
         packet_length += 21;
         dAccelCalibrationResponse = 0;
      } else if(gsrRangeResponse) {
         *(resPacket + packet_length++) = GSR_RANGE_RESPONSE;
         *(resPacket + packet_length++) = (storedConfig[NV_CONFIG_SETUP_BYTE3]&0x0E) >> 1;
         gsrRangeResponse = 0;
      } else if(allCalibrationResponse) {
         *(resPacket + packet_length++) = ALL_CALIBRATION_RESPONSE;
         memcpy((resPacket+packet_length), &storedConfig[NV_A_ACCEL_CALIBRATION], NV_NUM_CALIBRATION_BYTES);
         packet_length += NV_NUM_CALIBRATION_BYTES;
         allCalibrationResponse = 0;
      } else if(deviceVersionResponse) {
         *(resPacket + packet_length++) = DEVICE_VERSION_RESPONSE;
         *(resPacket + packet_length++) = DEVICE_VER;
         deviceVersionResponse = 0;
      } else if(mpu9150MagSensAdjValsResponse) {
         //if(!mpu9150Initialised) {
         MPU9150_init();
         MPU9150_wake(1);
         MPU9150_wake(0);
         //}
         *(resPacket + packet_length++) = MPU9150_MAG_SENS_ADJ_VALS_RESPONSE;
         MPU9150_getMagSensitivityAdj(resPacket+packet_length);
         packet_length += 3;
         mpu9150MagSensAdjValsResponse = 0;
      } else if (fwVersionResponse) {
         *(resPacket + packet_length++) = FW_VERSION_RESPONSE;
         *(resPacket + packet_length++) = FW_IDENTIFIER & 0xFF;
         *(resPacket + packet_length++) = (FW_IDENTIFIER & 0xFF00) >> 8;
         *(resPacket + packet_length++) = FW_VER_MAJOR & 0xFF;
         *(resPacket + packet_length++) = (FW_VER_MAJOR & 0xFF00) >> 8;
         *(resPacket + packet_length++) = FW_VER_MINOR;
         *(resPacket + packet_length++) = FW_VER_INTERNAL;
         fwVersionResponse = 0;
      } else if (blinkLedResponse) {
         *(resPacket + packet_length++) = CHARGE_STATUS_LED_RESPONSE;
         *(resPacket + packet_length++) = batt_stat;
         blinkLedResponse = 0;
      } else if (bufferSizeResponse) {
         *(resPacket + packet_length++) = BUFFER_SIZE_RESPONSE;
         *(resPacket + packet_length++) = storedConfig[NV_BUFFER_SIZE];
         bufferSizeResponse = 0;
      } else if (uniqueSerialResponse) {
         *(resPacket + packet_length++) = UNIQUE_SERIAL_RESPONSE;
         memcpy((resPacket+packet_length), dierecord, 8);
         packet_length += 8;
         uniqueSerialResponse = 0;
      } else if (exgRegsResponse) {
         *(resPacket + packet_length++) = EXG_REGS_RESPONSE;
         *(resPacket + packet_length++) = exgLength;
         if(exgLength) {
            if(exgChip)
               memcpy((resPacket+packet_length), (storedConfig+NV_EXG_ADS1292R_2_CONFIG1+exgStartAddr), exgLength);
            else
               memcpy((resPacket+packet_length), (storedConfig+NV_EXG_ADS1292R_1_CONFIG1+exgStartAddr), exgLength);
            packet_length += exgLength;
         }
         exgRegsResponse = 0;
      } else if (dcIdResponse) {
         *(resPacket + packet_length++) = DAUGHTER_CARD_ID_RESPONSE;
         *(resPacket + packet_length++) = dcMemLength;
         CAT24C16_init();
         CAT24C16_read(dcMemOffset, dcMemLength, (resPacket+packet_length));
         CAT24C16_powerOff();
         packet_length += dcMemLength;
         dcIdResponse = 0;
      } else if (dcMemResponse) {
         *(resPacket + packet_length++) = DAUGHTER_CARD_MEM_RESPONSE;
         *(resPacket + packet_length++) = dcMemLength;
         CAT24C16_init();
         CAT24C16_read(dcMemOffset+16, dcMemLength, (resPacket+packet_length));
         CAT24C16_powerOff();
         packet_length += dcMemLength;
         dcMemResponse = 0;
      }
   }

   BT_write(resPacket, packet_length);
}

void OpenFirstFile(){
   file_bad = f_open(&fil, (char*)filename, FA_WRITE | FA_CREATE_NEW);
}

void Timestamp0ToFirstFile(){
   uint32_t my_local_time_long;
   UINT bw;

   my_local_time_long = GetRTC();
   memcpy(&sdhead_text[SDH_MY_LOCALTIME], (uint8_t*)&my_local_time_long, 4);
   last_hour = my_local_time_long;
   last_min = my_local_time_long;
   // Write header to file
   file_bad = f_write(&fil, sdhead_text, SDHEAD_LEN, &bw);
}

FRESULT WriteFile(uint8_t* text, WORD size)
{
    // Result code
   FRESULT rc;
   UINT bw;
   uint32_t my_local_time_long, file_td_h, file_td_m;

   my_local_time_long = GetRTC();               //calculate time difference for file use.

   f_lseek (&fil, fil.fsize);                   //seek to end of file, no spi op

   // Write body to file
   rc = f_write(&fil, text, size, &bw);         //Write to file

   file_td_h = (uint32_t)fmod(4294967296+(uint64_t)my_local_time_long - (uint64_t)last_hour,4294967296);
   file_td_m = (uint32_t)fmod(4294967296+(uint64_t)my_local_time_long - (uint64_t)last_min,4294967296);

   //create a new file (from 000 up) every 1h
   if(file_td_h >= 117964800){   //117964800 = 32768/s*3600s = 1h
      last_hour = my_local_time_long;
      rc = f_close(&fil);
      //file number:from 000 up
      file_num++;

      //avoid using printf()
      filename[len_dir+3]=(char)(((int)'0')+file_num%10);
      filename[len_dir+2]=(char)(((int)'0')+(file_num/10)%10);
      filename[len_dir+1]=(char)(((int)'0')+(file_num/100)%10);

      f_open(&fil, (char*)filename, FA_WRITE | FA_CREATE_NEW);

      memcpy(&sdhead_text[SDH_MY_LOCALTIME], (uint8_t*)&my_local_time_long, 4);
      last_min = my_local_time_long;

      f_write(&fil, sdhead_text, SDHEAD_LEN, &bw);        // Write head to file
   }
   // sync data to SD card every 1 min
   else if(file_td_m >= 1966080){ //32768/s*60s = 1m

      last_min = my_local_time_long;
      f_sync(&fil);
   }

   file_bad = rc;

   return rc;
}

void PrepareSDBuffHead(void){
   memcpy(sdbuff+sdbuff_len,my_time_diff,5);
   sdbuff_len+=5;
   memset(my_time_diff,0xff,5);
}

// Initializes the RTC
void SetupRTC(void)
{
    RTCCTL01 = RTCHOLD + RTCTEV_1;  //hold
    RTCTIM0 = 0;
    RTCTIM1 = 0;
    RTCCTL01 &= ~RTCHOLD;           //start
}

uint32_t GetRTC(void)
{
   uint32_t my_local_time_long;
   my_local_time_long = ((uint32_t)RTCTIM1<<16)+RTCTIM0;
   return my_local_time_long;
}

void ParseConfig(void) {
   memset(storedConfig,0,NV_TOTAL_NUM_CONFIG_BYTES);
   memset(sdhead_text,0,SDHEAD_LEN);
   char buffer[66], * equals;
   uint8_t string_length = 0;
   float sample_rate=51.2;
   uint16_t sample_period = 0;
   uint8_t accel_lpm=0, accel_hrm=0, broadcast_interval, accel_mpu_range=0, exp_power=0;
   uint8_t accel_range=0, accel_smplrate=0, gyro_range=0, mag_smplrate=0, mag_gain=0, pres_bmp180_prec=0, gsr_range=0;
   uint8_t my_trial_id=0, num_shimmers_in_trial=0, my_config_time[4];
   uint32_t config_time = 0;
   bool iAmMaster = FALSE, time_sync = FALSE, singletouch = FALSE,
      tcxo=FALSE, user_button_enable = FALSE;

   broadcast_interval = RC_INT_C;

   storedConfig[NV_TRIAL_CONFIG0] &= ~SDH_SET_PMUX;         //PMUX reserved as 0
   storedConfig[NV_TRIAL_CONFIG0] |= SDH_TIME_STAMP;        //TIME_STAMP always = 1
   storedConfig[NV_CONFIG_SETUP_BYTE3] |= (4 & 0x07) <<1;   //BIT3-1
   storedConfig[NV_BUFFER_SIZE] = 1;

   *shimmername='\0';
   *exp_id_name='\0';
   *centername='\0';
   *config_time_text = '\0';
   float old_clock_freq = clock_freq;
   CheckSdInslot();

   char cfgname[]="sdlog.cfg";
   file_bad = f_open(&fil, cfgname, FA_READ | FA_OPEN_EXISTING);
   if(file_bad == FR_NO_FILE)   {
      IniReadInfoMem();
      UpdateSdConfig();
      file_bad = 0;
   }else{
      while(f_gets(buffer, 64, &fil)){
         if(!(equals = strchr(buffer, '=')))
            continue;
         equals++;     // this is the value
         if(strstr(buffer, "accel="))
            storedConfig[NV_SENSORS0] |= atoi(equals)*SENSOR_A_ACCEL;
         else if(strstr(buffer, "gyro="))
            storedConfig[NV_SENSORS0] |= atoi(equals)*SENSOR_MPU9150_GYRO;
         else if(strstr(buffer, "mag="))
            storedConfig[NV_SENSORS0] |= atoi(equals)*SENSOR_LSM303DLHC_MAG;
         else if(strstr(buffer, "exg1_24bit="))
            storedConfig[NV_SENSORS0] |= atoi(equals)*SENSOR_EXG1_24BIT;
         else if(strstr(buffer, "exg2_24bit="))
            storedConfig[NV_SENSORS0] |= atoi(equals)*SENSOR_EXG2_24BIT;
         else if(strstr(buffer, "gsr="))
            storedConfig[NV_SENSORS0] |= atoi(equals)*SENSOR_GSR;
         else if(strstr(buffer, "extch7="))
            storedConfig[NV_SENSORS0] |= atoi(equals)*SENSOR_EXT_A7;
         else if(strstr(buffer, "extch6="))
            storedConfig[NV_SENSORS0] |= atoi(equals)*SENSOR_EXT_A6;
         else if(strstr(buffer, "str="))
            storedConfig[NV_SENSORS1] |= atoi(equals)*SENSOR_STRAIN;
         else if(strstr(buffer, "vbat="))
            storedConfig[NV_SENSORS1] |= atoi(equals)*SENSOR_VBATT;
         else if(strstr(buffer, "accel_d="))
            storedConfig[NV_SENSORS1] |= atoi(equals)*SENSOR_LSM303DLHC_ACCEL;
         else if(strstr(buffer, "extch15="))
            storedConfig[NV_SENSORS1] |= atoi(equals)*SENSOR_EXT_A15;
         else if(strstr(buffer, "intch1="))
            storedConfig[NV_SENSORS1] |= atoi(equals)*SENSOR_INT_A1;
         else if(strstr(buffer, "intch12="))
            storedConfig[NV_SENSORS1] |= atoi(equals)*SENSOR_INT_A12;
         else if(strstr(buffer, "intch13="))
            storedConfig[NV_SENSORS1] |= atoi(equals)*SENSOR_INT_A13;
         else if(strstr(buffer, "intch14="))
            storedConfig[NV_SENSORS2] |= atoi(equals)*SENSOR_INT_A14;
         else if(strstr(buffer, "accel_mpu="))
            storedConfig[NV_SENSORS2] |= atoi(equals)*SENSOR_MPU9150_ACCEL;
         else if(strstr(buffer, "mag_mpu="))
            storedConfig[NV_SENSORS2] |= atoi(equals)*SENSOR_MPU9150_MAG;
         else if(strstr(buffer, "exg1_16bit="))
            storedConfig[NV_SENSORS2] |= atoi(equals)*SENSOR_EXG1_16BIT;
         else if(strstr(buffer, "exg2_16bit="))
            storedConfig[NV_SENSORS2] |= atoi(equals)*SENSOR_EXG2_16BIT;
         else if(strstr(buffer, "pres_bmp180="))
            storedConfig[NV_SENSORS2] |= atoi(equals)*SENSOR_BMP180_PRESSURE;
         else if(strstr(buffer, "sample_rate=")){
            sample_rate = atof(equals);
         }
         else if(strstr(buffer, "mg_internal_rate=")){
            mag_smplrate = atoi(equals);
            storedConfig[NV_CONFIG_SETUP_BYTE2]|= (mag_smplrate&0x07)<<2;     //BIT4-2
         }
         else if(strstr(buffer, "mg_range=")){
            mag_gain = atoi(equals);
            storedConfig[NV_CONFIG_SETUP_BYTE2]|= (mag_gain&0x07)<<5;         //BIT7-5
         }
         else if(strstr(buffer, "acc_internal_rate=")){
            accel_smplrate = atoi(equals);
            storedConfig[NV_CONFIG_SETUP_BYTE0]|= (accel_smplrate&0x0f)<<4;   //BIT7-4
         }
         else if(strstr(buffer, "accel_mpu_range=")){
            accel_mpu_range = atoi(equals);
            storedConfig[NV_CONFIG_SETUP_BYTE3]|= (accel_mpu_range&0x03)<<6;  //BIT7-6
         }
         else if(strstr(buffer, "acc_range=")){
            accel_range = atoi(equals);
            storedConfig[NV_CONFIG_SETUP_BYTE0]|= (accel_range&0x03)<<2;      //BIT3-2
         }
         else if(strstr(buffer, "acc_lpm=")){
            accel_lpm = atoi(equals);
            storedConfig[NV_TRIAL_CONFIG1] |= accel_lpm * SDH_ACCEL_LPM;
            storedConfig[NV_CONFIG_SETUP_BYTE0] |= (accel_lpm&0x01)<<1;       //BIT1
         }
         else if(strstr(buffer, "acc_hrm=")){
            accel_hrm = atoi(equals);
            storedConfig[NV_TRIAL_CONFIG1] |= accel_hrm * SDH_ACCEL_HRM;
            storedConfig[NV_CONFIG_SETUP_BYTE0]|= accel_hrm&0x01;             //BIT0
         }
         else if(strstr(buffer, "gs_range=")){// or "gsr_range="?
            gsr_range = atoi(equals);
            if(gsr_range>4)
               gsr_range = 4;
            storedConfig[NV_CONFIG_SETUP_BYTE3] |= (gsr_range& 0x07) <<1;     //BIT3-1
         }
         else if(strstr(buffer, "gyro_samplingrate="))
            storedConfig[NV_CONFIG_SETUP_BYTE1] = atoi(equals);
         else if(strstr(buffer, "gyro_range=")){
            gyro_range = atoi(equals);
            storedConfig[NV_CONFIG_SETUP_BYTE2]|= (gyro_range)&0x03;          //BIT1-0
         }
         else if(strstr(buffer, "pres_bmp180_prec=")){
            pres_bmp180_prec = atoi(equals);
            storedConfig[NV_CONFIG_SETUP_BYTE3]|= (pres_bmp180_prec& 0x03) <<4;   //BIT5-4
         }
         else if(strstr(buffer, "user_button_enable=")){
            user_button_enable = (atoi(equals)==0)?FALSE:TRUE;
            storedConfig[NV_TRIAL_CONFIG0] |= user_button_enable*SDH_USER_BUTTON_ENABLE;
         }
         else if(strstr(buffer, "iammaster=")){//0=slave=node
            iAmMaster = (atoi(equals)==0)?FALSE:TRUE;
            storedConfig[NV_TRIAL_CONFIG0] |= iAmMaster*SDH_IAMMASTER;
         }
         else if(strstr(buffer, "sync=")){
            time_sync = (atoi(equals)==0)?FALSE:TRUE;
            storedConfig[NV_TRIAL_CONFIG0] |= time_sync*SDH_TIME_SYNC;
         }
         else if(strstr(buffer, "interval=")){
            broadcast_interval = atoi(equals)>255?255:atoi(equals);
         }
         else if(strstr(buffer, "singletouch=")){
            singletouch = (atoi(equals)==0)?FALSE:TRUE;
            storedConfig[NV_TRIAL_CONFIG1] |= singletouch * SDH_SINGLETOUCH;
         }
         else if(strstr(buffer, "exp_power=")){
            exp_power = atoi(equals);
            storedConfig[NV_TRIAL_CONFIG1] |= exp_power * SDH_EXP_POWER;
            storedConfig[NV_CONFIG_SETUP_BYTE3] |= exp_power& 0x01;
         }
         else if(strstr(buffer, "tcxo=")){
            tcxo = 0;
            storedConfig[NV_TRIAL_CONFIG1] |= tcxo * SDH_TCXO;
         }
         else if(strstr(buffer, "center=")){
            string_length = MAX_CHARS<strlen(equals)?MAX_CHARS:strlen(equals)-1;
            memcpy((char*)centername, equals, string_length-1);
            *(centername+string_length-1)=0;
         }
         else if(strstr(buffer, "myid=")){
            my_trial_id = atoi(equals);
            sdhead_text[SDH_MYTRIAL_ID] = my_trial_id;
         }
         else if(strstr(buffer, "Nshimmer=")){
            num_shimmers_in_trial = atoi(equals);
            sdhead_text[SDH_NSHIMMER] = num_shimmers_in_trial;
         }
         else if(strstr(buffer, "shimmername=")){
            string_length = MAX_CHARS<strlen(equals)?MAX_CHARS:strlen(equals)-1;
            memcpy((char*)shimmername, equals, string_length-1);
            *(shimmername+string_length-1)=0;
         }
         else if(strstr(buffer, "experimentid=")){
            string_length = MAX_CHARS<strlen(equals)?MAX_CHARS:strlen(equals)-1;
            memcpy((char*)exp_id_name, equals, string_length-1);
            *(exp_id_name+string_length-1)=0;
         }
         else if(strstr(buffer, "configtime=")){
            config_time = atol(equals);
            my_config_time[3] = *((uint8_t*)&config_time);
            my_config_time[2] = *(((uint8_t*)&config_time)+1);
            my_config_time[1] = *(((uint8_t*)&config_time)+2);
            my_config_time[0] = *(((uint8_t*)&config_time)+3);
            string_length = MAX_CHARS<strlen(equals)?MAX_CHARS:strlen(equals)-1;
            memcpy((char*)config_time_text, equals, string_length-1);
            *(config_time_text+string_length-1)=0;
         }
         else if(strstr(buffer, "EXG_ADS1292R_1_CONFIG1="))
            storedConfig[NV_EXG_ADS1292R_1_CONFIG1] = atoi(equals);
         else if(strstr(buffer, "EXG_ADS1292R_1_CONFIG2="))
            storedConfig[NV_EXG_ADS1292R_1_CONFIG2] = atoi(equals);
         else if(strstr(buffer, "EXG_ADS1292R_1_LOFF="))
            storedConfig[NV_EXG_ADS1292R_1_LOFF] = atoi(equals);
         else if(strstr(buffer, "EXG_ADS1292R_1_CH1SET="))
            storedConfig[NV_EXG_ADS1292R_1_CH1SET] = atoi(equals);
         else if(strstr(buffer, "EXG_ADS1292R_1_CH2SET="))
            storedConfig[NV_EXG_ADS1292R_1_CH2SET] = atoi(equals);
         else if(strstr(buffer, "EXG_ADS1292R_1_RLD_SENS="))
            storedConfig[NV_EXG_ADS1292R_1_RLD_SENS] = atoi(equals);
         else if(strstr(buffer, "EXG_ADS1292R_1_LOFF_SENS="))
            storedConfig[NV_EXG_ADS1292R_1_LOFF_SENS] = atoi(equals);
         else if(strstr(buffer, "EXG_ADS1292R_1_LOFF_STAT="))
            storedConfig[NV_EXG_ADS1292R_1_LOFF_STAT] = atoi(equals);
         else if(strstr(buffer, "EXG_ADS1292R_1_RESP1="))
            storedConfig[NV_EXG_ADS1292R_1_RESP1] = atoi(equals);
         else if(strstr(buffer, "EXG_ADS1292R_1_RESP2="))
            storedConfig[NV_EXG_ADS1292R_1_RESP2] = atoi(equals);
         else if(strstr(buffer, "EXG_ADS1292R_2_CONFIG1="))
            storedConfig[NV_EXG_ADS1292R_2_CONFIG1] = atoi(equals);
         else if(strstr(buffer, "EXG_ADS1292R_2_CONFIG2="))
            storedConfig[NV_EXG_ADS1292R_2_CONFIG2] = atoi(equals);
         else if(strstr(buffer, "EXG_ADS1292R_2_LOFF="))
            storedConfig[NV_EXG_ADS1292R_2_LOFF] = atoi(equals);
         else if(strstr(buffer, "EXG_ADS1292R_2_CH1SET="))
            storedConfig[NV_EXG_ADS1292R_2_CH1SET] = atoi(equals);
         else if(strstr(buffer, "EXG_ADS1292R_2_CH2SET="))
            storedConfig[NV_EXG_ADS1292R_2_CH2SET] = atoi(equals);
         else if(strstr(buffer, "EXG_ADS1292R_2_RLD_SENS="))
            storedConfig[NV_EXG_ADS1292R_2_RLD_SENS] = atoi(equals);
         else if(strstr(buffer, "EXG_ADS1292R_2_LOFF_SENS="))
            storedConfig[NV_EXG_ADS1292R_2_LOFF_SENS] = atoi(equals);
         else if(strstr(buffer, "EXG_ADS1292R_2_LOFF_STAT="))
            storedConfig[NV_EXG_ADS1292R_2_LOFF_STAT] = atoi(equals);
         else if(strstr(buffer, "EXG_ADS1292R_2_RESP1="))
            storedConfig[NV_EXG_ADS1292R_2_RESP1] = atoi(equals);
         else if(strstr(buffer, "EXG_ADS1292R_2_RESP2="))
            storedConfig[NV_EXG_ADS1292R_2_RESP2] = atoi(equals);
      }
      file_bad = f_close(&fil);
   }

   sample_period = FreqDiv(sample_rate);
   // little endian:
   storedConfig[NV_SAMPLING_RATE] = (uint8_t)(sample_period & 0xFF);
   storedConfig[NV_SAMPLING_RATE + 1] = (uint8_t)(sample_period >> 8 );

   if(storedConfig[NV_SENSORS0] & SENSOR_GSR)      //they are sharing adc1, so ban intch1 when gsr is on
      storedConfig[NV_SENSORS1] &= ~SENSOR_INT_A1;
   if(storedConfig[NV_SENSORS1] & SENSOR_STRAIN){  //they are sharing adc13 and adc14
      storedConfig[NV_SENSORS1] &= ~SENSOR_INT_A13;
      storedConfig[NV_SENSORS2] &= ~SENSOR_INT_A14;
   }
   if(storedConfig[NV_SENSORS0]&SENSOR_EXG1_24BIT){
      storedConfig[NV_SENSORS2] &= ~SENSOR_EXG1_16BIT;
   }
   if(storedConfig[NV_SENSORS0]&SENSOR_EXG2_24BIT){
      storedConfig[NV_SENSORS2] &= ~SENSOR_EXG2_16BIT;
   }
   if(storedConfig[NV_SENSORS0] & SENSOR_EXG1_24BIT || storedConfig[NV_SENSORS0] & SENSOR_EXG2_24BIT ||
      storedConfig[NV_SENSORS2] & SENSOR_EXG1_16BIT || storedConfig[NV_SENSORS2] & SENSOR_EXG2_16BIT){
      storedConfig[NV_CONFIG_SETUP_BYTE3] |= EXP_POWER_ENABLE;
      storedConfig[NV_SENSORS1] &= ~SENSOR_INT_A1;
      storedConfig[NV_SENSORS2] &= ~SENSOR_INT_A14;
   }

   if(((storedConfig[NV_CONFIG_SETUP_BYTE3]>>1)&0x07)>4)       //never larger than 4
      storedConfig[NV_CONFIG_SETUP_BYTE3] |= (4 & 0x07) <<1;   //BIT3-1

   if(clock_freq == TCXO_CLOCK){
      P4OUT |= BIT6;
      P4SEL |= BIT7;
   }
   else{
      P4OUT &= ~BIT6;
       P4SEL &= ~BIT7;
   }

   if(tcxo)
      clock_freq = TCXO_CLOCK;
   else
      clock_freq = MSP430_CLOCK;
   if(clock_freq != old_clock_freq){
      ClkAssignment();
      TB0CTL = MC_0;
      TB0Start();
   }

   //minimum sync broadcast interval is 54 seconds
   if(broadcast_interval < RC_INT_C){
      broadcast_interval = RC_INT_C;
   }

   //the button always works for singletouch mode
   //sync always works for singletouch mode
   if(storedConfig[NV_TRIAL_CONFIG1]&SDH_SINGLETOUCH){
      storedConfig[NV_TRIAL_CONFIG0] |= SDH_USER_BUTTON_ENABLE;
      storedConfig[NV_TRIAL_CONFIG0] |= SDH_TIME_SYNC;
   }

   SetName();

   storedConfig[NV_BT_INTERVAL]= broadcast_interval;

   Config2SdHead();
   memcpy(&sdhead_text[SDH_CONFIG_TIME_0], my_config_time, 4);

   InfoMem_write((void*)0, storedConfig, NV_NUM_SETTINGS_BYTES);

   on_single_touch = 0;
   on_user_button = 0;
   on_default = 0;
   if(storedConfig[NV_TRIAL_CONFIG0]&SDH_SINGLETOUCH)
      on_single_touch = 1;
   else if(storedConfig[NV_TRIAL_CONFIG0]&SDH_USER_BUTTON_ENABLE)
      on_user_button = 1;
   else{
      on_default = 1;
   }
}

void Config2SdHead(void){
   sdhead_text[SDH_SAMPLE_RATE_0] = storedConfig[NV_SAMPLING_RATE];
   sdhead_text[SDH_SAMPLE_RATE_1] = storedConfig[NV_SAMPLING_RATE+1];
   sdhead_text[SDH_SENSORS0] = storedConfig[NV_SENSORS0];
   sdhead_text[SDH_SENSORS1] = storedConfig[NV_SENSORS1];
   sdhead_text[SDH_SENSORS2] = storedConfig[NV_SENSORS2];
   sdhead_text[SDH_CONFIG_SETUP_BYTE0] = storedConfig[NV_CONFIG_SETUP_BYTE0];
   sdhead_text[SDH_CONFIG_SETUP_BYTE1] = storedConfig[NV_CONFIG_SETUP_BYTE1];
   sdhead_text[SDH_CONFIG_SETUP_BYTE2] = storedConfig[NV_CONFIG_SETUP_BYTE2];
   sdhead_text[SDH_CONFIG_SETUP_BYTE3] = storedConfig[NV_CONFIG_SETUP_BYTE3];
   sdhead_text[SDH_TRIAL_CONFIG0] = storedConfig[NV_TRIAL_CONFIG0];
   sdhead_text[SDH_TRIAL_CONFIG1] = storedConfig[NV_TRIAL_CONFIG1];
   sdhead_text[SDH_BROADCAST_INTERVAL] = storedConfig[NV_BT_INTERVAL];
   //little endian in fw, but they want big endian in sw
   //trivial
   sdhead_text[SDH_SHIMMERVERSION_BYTE_1] = DEVICE_VER;
   sdhead_text[SDH_FW_VERSION_TYPE_1] = FW_IDENTIFIER;
   sdhead_text[SDH_FW_VERSION_MAJOR_1] = FW_VER_MAJOR;
   sdhead_text[SDH_FW_VERSION_MINOR] = FW_VER_MINOR;
   sdhead_text[SDH_FW_VERSION_INTERNAL] = FW_VER_INTERNAL;

   //exg
   sdhead_text[SDH_EXG_ADS1292R_1_CONFIG1] = storedConfig[NV_EXG_ADS1292R_1_CONFIG1];
   sdhead_text[SDH_EXG_ADS1292R_1_CONFIG2] = storedConfig[NV_EXG_ADS1292R_1_CONFIG2];
   sdhead_text[SDH_EXG_ADS1292R_1_LOFF] = storedConfig[NV_EXG_ADS1292R_1_LOFF];
   sdhead_text[SDH_EXG_ADS1292R_1_CH1SET] = storedConfig[NV_EXG_ADS1292R_1_CH1SET];
   sdhead_text[SDH_EXG_ADS1292R_1_CH2SET] = storedConfig[NV_EXG_ADS1292R_1_CH2SET];
   sdhead_text[SDH_EXG_ADS1292R_1_RLD_SENS] = storedConfig[NV_EXG_ADS1292R_1_RLD_SENS];
   sdhead_text[SDH_EXG_ADS1292R_1_LOFF_SENS] = storedConfig[NV_EXG_ADS1292R_1_LOFF_SENS];
   sdhead_text[SDH_EXG_ADS1292R_1_LOFF_STAT] = storedConfig[NV_EXG_ADS1292R_1_LOFF_STAT];
   sdhead_text[SDH_EXG_ADS1292R_1_RESP1] = storedConfig[NV_EXG_ADS1292R_1_RESP1];
   sdhead_text[SDH_EXG_ADS1292R_1_RESP2] = storedConfig[NV_EXG_ADS1292R_1_RESP2];
   sdhead_text[SDH_EXG_ADS1292R_2_CONFIG1] = storedConfig[NV_EXG_ADS1292R_2_CONFIG1];
   sdhead_text[SDH_EXG_ADS1292R_2_CONFIG2] = storedConfig[NV_EXG_ADS1292R_2_CONFIG2];
   sdhead_text[SDH_EXG_ADS1292R_2_LOFF] = storedConfig[NV_EXG_ADS1292R_2_LOFF];
   sdhead_text[SDH_EXG_ADS1292R_2_CH1SET] = storedConfig[NV_EXG_ADS1292R_2_CH1SET];
   sdhead_text[SDH_EXG_ADS1292R_2_CH2SET] = storedConfig[NV_EXG_ADS1292R_2_CH2SET];
   sdhead_text[SDH_EXG_ADS1292R_2_RLD_SENS] = storedConfig[NV_EXG_ADS1292R_2_RLD_SENS];
   sdhead_text[SDH_EXG_ADS1292R_2_LOFF_SENS] = storedConfig[NV_EXG_ADS1292R_2_LOFF_SENS];
   sdhead_text[SDH_EXG_ADS1292R_2_LOFF_STAT] = storedConfig[NV_EXG_ADS1292R_2_LOFF_STAT];
   sdhead_text[SDH_EXG_ADS1292R_2_RESP1] = storedConfig[NV_EXG_ADS1292R_2_RESP1];
   sdhead_text[SDH_EXG_ADS1292R_2_RESP2] = storedConfig[NV_EXG_ADS1292R_2_RESP2];

}

void SetDefaultConfiguration(void) {
   //51.2Hz
   *(uint16_t *)(storedConfig+NV_SAMPLING_RATE) = 640;
   storedConfig[NV_BUFFER_SIZE] = 1;
   //core sensors enabled
   storedConfig[NV_SENSORS0] = SENSOR_A_ACCEL + SENSOR_MPU9150_GYRO + SENSOR_LSM303DLHC_MAG;
   storedConfig[NV_SENSORS1] = SENSOR_VBATT;
   storedConfig[NV_SENSORS2] = 0;
   //LSM303DLHC Accel 100Hz, +/-2G, Low Power and High Resolution modes off
   storedConfig[NV_CONFIG_SETUP_BYTE0] = (LSM303DLHC_ACCEL_100HZ<<4) + (ACCEL_2G<<2);
   //MPU9150 sampling rate of 8kHz/(155+1), i.e. 51.282Hz
   storedConfig[NV_CONFIG_SETUP_BYTE1] = 0x9B;
   //LSM303DLHC Mag 75Hz, +/-1.3 Gauss, MPU9150 Gyro +/-500 degrees per second
   storedConfig[NV_CONFIG_SETUP_BYTE2] = (LSM303DLHC_MAG_1_3G<<5) + (LSM303DLHC_MAG_75HZ<<2) + MPU9150_GYRO_500DPS;
   //MPU9150 Accel +/-2G, BMP pressure oversampling ratio 1, GSR range resistor 40k, EXP_RESET_N pin set low
   storedConfig[NV_CONFIG_SETUP_BYTE3] = (ACCEL_2G<<6) + (BMP180_OSS_1<<4) + (HW_RES_40K<<1);
   storedConfig[NV_TRIAL_CONFIG0] = SDH_USER_BUTTON_ENABLE;
   storedConfig[NV_TRIAL_CONFIG1] = 0;
   storedConfig[NV_BT_INTERVAL] = 54;
   //set all ExG registers to their reset values
   storedConfig[NV_EXG_ADS1292R_1_CONFIG1] = 0x02;
   storedConfig[NV_EXG_ADS1292R_1_CONFIG2] = 0x80;
   storedConfig[NV_EXG_ADS1292R_1_LOFF] = 0x10;
   storedConfig[NV_EXG_ADS1292R_1_CH1SET] = 0x00;
   storedConfig[NV_EXG_ADS1292R_1_CH2SET] = 0x00;
   storedConfig[NV_EXG_ADS1292R_1_RLD_SENS] = 0x00;
   storedConfig[NV_EXG_ADS1292R_1_LOFF_SENS] = 0x00;
   storedConfig[NV_EXG_ADS1292R_1_LOFF_STAT] = 0x00;
   storedConfig[NV_EXG_ADS1292R_1_RESP1] = 0x00;
   storedConfig[NV_EXG_ADS1292R_1_RESP2] = 0x02;
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

   InfoMem_write((void*)0, storedConfig, NV_NUM_SETTINGS_BYTES);
}

void SetName(){
   // first we'll make the shimmer mac address into a string
   if(strlen((char*)shimmername)==0){ // if name hasn't been assigned by user, use default (from Shimmer mac address)
      strcpy((char*)shimmername, "IDXXXX");
      memcpy(shimmername+2, mac+8, 4);
   }

   if(strlen((char*)exp_id_name)==0)
      strcpy((char*)exp_id_name, "default_exp");

   if(strlen((char*)centername)==0)
      strcpy((char*)centername,"00066643b48e");

   if(strlen((char*)config_time_text)==0)
      strcpy((char*)config_time_text,"112014");
}

error_t SetBasedir() {
   FILINFO fno;
   volatile error_t res;
   uint16_t tmp_counter = 0;
   char lfn[_MAX_LFN + 1], * fname, * scout, * dash, dirnum[8];

   SetName();

   fno.lfname = lfn;
   fno.lfsize = sizeof(lfn);

   if((res = f_opendir(&dir, "/data"))){
      if(res == FR_NO_PATH)   //we'll have to make /data first
         res = f_mkdir("/data");
      if(res)                 //in every case, we're toast
         return FAIL;

      //try one more time
      if((res = f_opendir(&dir, "/data")))
         return FAIL;
   }

   strcpy((char*)exp_dir_name, "data/");
   strcat((char*)exp_dir_name, (char*)exp_id_name);
   strcat((char*)exp_dir_name, "_");
   strcat((char*)exp_dir_name, (char*)config_time_text);

   if((res = f_opendir(&dir, (char*)exp_dir_name))){
      if(res == FR_NO_PATH)      //we'll have to make the experiment folder first
         res = f_mkdir((char*)exp_dir_name);
      if(res)                    //in every case, we're toast
         return FAIL;

      //try one more time
      if((res = f_opendir(&dir, (char*)exp_dir_name)))
         return FAIL;
   }

   dir_counter = 0;   //this might be the first log for this shimmer

   //file name format
   //shimmername    as defined in sdlog.cfg
   //-              separator
   //000
   //we want to create a new directory with a sequential run number each power-up/reset for each shimmer
   while(f_readdir(&dir, &fno) == FR_OK){
      if(*fno.fname == 0)
         break;
      else if(fno.fattrib & AM_DIR){
         fname = (*fno.lfname) ? fno.lfname : fno.fname;

         if(!strncmp(fname, (char*)shimmername, strlen(fname)-4)){ //-4 because of the -000 etc.
            if((scout = strchr(fname, '-'))){      //if not, something is seriously wrong!
               scout++;
               while((dash = strchr(scout, '-')))  //In case the shimmer name contains '-'
                  scout = dash + 1;
               strcpy(dirnum, scout);
               tmp_counter = atoi(dirnum);
               if(tmp_counter >= dir_counter){
                  dir_counter = tmp_counter;
                  dir_counter++;                   //start with next in numerical sequence
               }
            }
            else
               return FAIL;
         }
      }
   }

   //at this point, we have the id string and the counter, so we can make a directory name
   return SUCCESS;
}

error_t MakeBasedir() {
   memset(dirname,0,64);

   char dir_counter_text[4];
   Itoa((uint32_t)dir_counter, (uint8_t*)dir_counter_text,4);

   strcpy((char*)dirname, (char*)exp_dir_name);
   strcat((char*)dirname, "/");
   strcat((char*)dirname, (char*)shimmername);
   strcat((char*)dirname, "-");
   strcat((char*)dirname, dir_counter_text);

   if(file_bad = f_mkdir((char*)dirname))
      return FAIL;

   memset(filename,0,64);
   strcpy((char*)filename,(char*)dirname);
   len_dir = strlen((char*)dirname);
   strcat((char*)filename,"/000");

   return SUCCESS;
}

void Calibrate(){
   if(storedConfig[NV_SENSORS1] & SENSOR_LSM303DLHC_ACCEL)
      ReadCalibration(S_ACCEL);
   if(storedConfig[NV_SENSORS0] & SENSOR_MPU9150_GYRO)
      ReadCalibration(S_GYRO);
   if(storedConfig[NV_SENSORS0] & SENSOR_LSM303DLHC_MAG)
      ReadCalibration(S_MAG);
   if(storedConfig[NV_SENSORS0] & SENSOR_A_ACCEL)
      ReadCalibration(S_ACCEL_A);
   if(storedConfig[NV_SENSORS2] & SENSOR_BMP180_PRESSURE){
      BMP180_init();
      BMP180_getCalibCoeff(&sdhead_text[SDH_TEMP_PRES_CALIBRATION]);
      I2C_PowerOff();
   }
   memcpy(&sdhead_text[SDH_LSM303DLHC_ACCEL_CALIBRATION], &storedConfig[NV_LSM303DLHC_ACCEL_CALIBRATION], 21);
   memcpy(&sdhead_text[SDH_MPU9150_GYRO_CALIBRATION], &storedConfig[NV_MPU9150_GYRO_CALIBRATION], 21);
   memcpy(&sdhead_text[SDH_LSM303DLHC_MAG_CALIBRATION], &storedConfig[NV_LSM303DLHC_MAG_CALIBRATION], 21);
   memcpy(&sdhead_text[SDH_A_ACCEL_CALIBRATION], &storedConfig[NV_A_ACCEL_CALIBRATION], 21);
   InfoMem_write((void*)NV_A_ACCEL_CALIBRATION, &storedConfig[NV_A_ACCEL_CALIBRATION], NV_NUM_CALIBRATION_BYTES);
}


void ReadCalibration(uint8_t sensor){
   char buffer[66], * equals, keyword[20];
   uint8_t i = 0, num_2byte_params = 6, num_1byte_params = 9, cal_file[32];
   uint16_t address;
   bool sensor_found = FALSE;
   float value;
   int8_t rounded_value;

   uint8_t mpu9150_gyro_range = (sdhead_text[SDH_CONFIG_SETUP_BYTE2] & 0x03);
   uint8_t lsm303dlhc_mag_range = ((sdhead_text[SDH_CONFIG_SETUP_BYTE2]>>5)&0x07);
   uint8_t lsm303dlhc_accel_range = ((sdhead_text[SDH_CONFIG_SETUP_BYTE0]>>2)&0x03);

   DIRS gdc;
   FIL gfc;

   if(sensor == S_GYRO){
      if(mpu9150_gyro_range == MPU9150_GYRO_250DPS)
         strcpy(keyword, "Gyro 250dps");
      else if(mpu9150_gyro_range == MPU9150_GYRO_500DPS)
         strcpy(keyword, "Gyro 500dps");
      else if(mpu9150_gyro_range == MPU9150_GYRO_1000DPS)
             strcpy(keyword, "Gyro 1000dps");
      else if(mpu9150_gyro_range == MPU9150_GYRO_2000DPS)
         strcpy(keyword, "Gyro 2000dps");
      address = NV_MPU9150_GYRO_CALIBRATION;
   }
   else if(sensor == S_MAG){
      if(lsm303dlhc_mag_range == LSM303_MAG_13GA)
         strcpy(keyword, "Mag 1.3Ga");
      else if(lsm303dlhc_mag_range == LSM303_MAG_19GA)
         strcpy(keyword, "Mag 1.9Ga");
      else if(lsm303dlhc_mag_range == LSM303_MAG_25GA)
         strcpy(keyword, "Mag 2.5Ga");
      else if(lsm303dlhc_mag_range == LSM303_MAG_40GA)
         strcpy(keyword, "Mag 4.0Ga");
      else if(lsm303dlhc_mag_range == LSM303_MAG_47GA)
         strcpy(keyword, "Mag 4.7Ga");
      else if(lsm303dlhc_mag_range == LSM303_MAG_56GA)
         strcpy(keyword, "Mag 5.6Ga");
      else if(lsm303dlhc_mag_range == LSM303_MAG_81GA)
         strcpy(keyword, "Mag 8.1Ga");
      address = NV_LSM303DLHC_MAG_CALIBRATION;
   }
   else if(sensor == S_ACCEL){
      if(lsm303dlhc_accel_range == RANGE_2G)
         strcpy(keyword, "Accel 2.0g");
      else if(lsm303dlhc_accel_range == RANGE_4G)
         strcpy(keyword, "Accel 4.0g");
      else if(lsm303dlhc_accel_range == RANGE_8G)
         strcpy(keyword, "Accel 8.0g");
      else //(sdhead_text[SDH_ACCEL_RANGE] == RANGE_16G)
         strcpy(keyword, "Accel 16.0g");
      address = NV_LSM303DLHC_ACCEL_CALIBRATION;
   }
   else if(sensor == S_ACCEL_A){
      strcpy(keyword, "Accel_A");
      address = NV_A_ACCEL_CALIBRATION;
   }
   else {
      return;
   }
   strcpy((char*)cal_file, "/Calibration/calibParams.ini");
   if(f_opendir(&gdc, "/Calibration")){
      if(f_opendir(&gdc, "/calibration"))
         DefaultCalibration(sensor);
      else
         strcpy((char*)cal_file, "/calibration/calibParams.ini");
   }
   else if(f_open(&gfc, (char*)cal_file, (FA_OPEN_EXISTING | FA_READ))) //no calibration file, use default
      DefaultCalibration(sensor);
   else{ //look for sensor in calibration file.
      while(f_gets(buffer, 64, &gfc)){
         if(!strstr(buffer, keyword))
            continue;
         else{ //found the right sensor
            sensor_found = TRUE;
            for(i = 0; i < num_2byte_params; i++){
               f_gets(buffer, 64, &gfc);
               if(!(equals = strchr(buffer, '='))){
                  sensor_found = FALSE; //there's an error, use the default
                  break;
               }
               equals ++;
               value = atof(equals);
               if((sensor == S_GYRO) & (i >= 3))
                  value *= 100;
               storedConfig[address + 2*i] = ((int16_t)(value + 0.5) & 0xFF00) >> 8;
               storedConfig[address + 2*i + 1] = ((int16_t)(value + 0.5) & 0xFF);
            }
            for(i = 0; i < num_1byte_params; i++){
               f_gets(buffer, 64, &gfc);
               if(!(equals = strchr(buffer, '='))){
                  sensor_found = FALSE; //there's an error, use the default
                  break;
               }
               equals ++;
               value = atof(equals)*100;
               rounded_value = (int8_t)(value>=0?value+0.5:value-0.5);
               storedConfig[address + 2*num_2byte_params + i] = (rounded_value);
            }
            f_close(&gfc);
            break;
         }
      }
   }
   if(!sensor_found)
      DefaultCalibration(sensor);
}

void DefaultCalibration(uint8_t sensor){
   int16_t bias, sensitivity;
   uint8_t bias_byte_one, bias_byte_two, sens_byte_one, sens_byte_two, number_axes = 1;
   int8_t align_xx, align_xy, align_xz, align_yx, align_yy, align_yz, align_zx, align_zy, align_zz, i = 0;
   uint16_t address;
   bool align = FALSE;

   uint8_t mpu9150_gyro_range = (sdhead_text[SDH_CONFIG_SETUP_BYTE2] & 0x03);
   uint8_t lsm303dlhc_mag_range = ((sdhead_text[SDH_CONFIG_SETUP_BYTE2]>>5)&0x07);
   uint8_t lsm303dlhc_accel_range = ((sdhead_text[SDH_CONFIG_SETUP_BYTE0]>>2)&0x03);

   if(sensor==S_ACCEL){
      number_axes = 3;
      bias = 0;
      align = TRUE;
      address = NV_LSM303DLHC_ACCEL_CALIBRATION;
      if(lsm303dlhc_accel_range == RANGE_2G)
         sensitivity = 1631;
      else if(lsm303dlhc_accel_range == RANGE_4G)
         sensitivity = 815;
      else if (lsm303dlhc_accel_range == RANGE_8G)
         sensitivity = 408;
      else //(sdhead_text[SDH_ACCEL_RANGE] == RANGE_16G)
         sensitivity = 135;
      align_xx = -100;
      align_xy = 0;
      align_xz = 0;
      align_yx = 0;
      align_yy = 100;
      align_yz = 0;
      align_zx = 0;
      align_zy = 0;
      align_zz = -100;
   }
   else if(sensor==S_GYRO){
      number_axes = 3;
      bias = 0;
      if(mpu9150_gyro_range == MPU9150_GYRO_250DPS)
         sensitivity = 13100;
      else if(mpu9150_gyro_range == MPU9150_GYRO_500DPS)
         sensitivity = 6550;
      else if (mpu9150_gyro_range == MPU9150_GYRO_1000DPS)
         sensitivity = 3280;
      else //(sdhead_text[SDH_GYRO_RANGE] == MPU9150_GYRO_2000DPS)
         sensitivity = 1640;
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
   else if(sensor==S_MAG){
      number_axes = 3;
      bias = 0;
      if(lsm303dlhc_mag_range == LSM303_MAG_13GA)
         sensitivity = 1100;
      else if(lsm303dlhc_mag_range == LSM303_MAG_19GA)
         sensitivity = 855;
      else if (lsm303dlhc_mag_range == LSM303_MAG_25GA)
         sensitivity = 670;
      else if (lsm303dlhc_mag_range == LSM303_MAG_40GA)
         sensitivity = 450;
      else if (lsm303dlhc_mag_range == LSM303_MAG_47GA)
         sensitivity = 355;
      else if (lsm303dlhc_mag_range == LSM303_MAG_56GA)
         sensitivity = 330;
      else //(sdhead_text[SDH_MAG_RANGE] == LSM303_MAG_81GA)
         sensitivity = 230;
      align = TRUE;
      align_xx = 100;
      align_xy = 0;
      align_xz = 0;
      align_yx = 0;
      align_yy = -100;
      align_yz = 0;
      align_zx = 0;
      align_zy = 0;
      align_zz = 100;
      address = NV_LSM303DLHC_MAG_CALIBRATION;
   }
   else if(sensor==S_ACCEL_A){
      number_axes = 3;
      bias = 2047;
      sensitivity = 83;
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
   else{
      return;
   }
   bias_byte_one = (uint8_t)(bias >> 8);
   bias_byte_two = (uint8_t)(bias);
   sens_byte_one = (uint8_t)(sensitivity >> 8);
   sens_byte_two = (uint8_t)(sensitivity);
   //offset
   for(i = 0; i < number_axes; i++){
      storedConfig[address + 2*i] = bias_byte_one;
      storedConfig[address + 2*i + 1] = bias_byte_two;
   }
   //sensitivity
   for(i = 0; i < number_axes; i++){
      storedConfig[address + 2*(number_axes + i)] = sens_byte_one;
      storedConfig[address + 2*(number_axes + i) + 1] = sens_byte_two;
   }
   //alignment
   if(align){
      storedConfig[address+ 12] = align_xx;
      storedConfig[address+ 13] = align_xy;
      storedConfig[address+ 14] = align_xz;
      storedConfig[address+ 15] = align_yx;
      storedConfig[address+ 16] = align_yy;
      storedConfig[address+ 17] = align_yz;
      storedConfig[address+ 18] = align_zx;
      storedConfig[address+ 19] = align_zy;
      storedConfig[address+ 20] = align_zz;
   }
}

// GSR
void GsrRange() {
   //Fill the current active resistor into the upper two bits of the GSR value
   //if autorange is enabled, switch active resistor if required
   //If during resistor transition period use old ADC and resistor values
   //as determined by GSR_smoothTransition()

   uint8_t current_active_resistor = gsr_active_resistor;
   uint16_t ADC_val;

   //GSR channel will always be last ADC channel
   if(currentBuffer == 0) {
      ADC_val = *((uint16_t *)txBuff0 + (nbrAdcChans - 1) + 2);
      if(((storedConfig[NV_CONFIG_SETUP_BYTE3]&0x0E)>>1) == GSR_AUTORANGE) {
         gsr_active_resistor = GSR_controlRange(ADC_val, gsr_active_resistor);
         if(GSR_smoothTransition(&current_active_resistor, *(uint16_t *)(storedConfig+NV_SAMPLING_RATE))) {
            ADC_val = last_GSR_val;
         }
      }
      *((uint16_t *)txBuff0 + (nbrAdcChans - 1) + 2) =  ADC_val | (current_active_resistor << 14);
   }
   else {
      ADC_val = *((uint16_t *)txBuff1 + (nbrAdcChans - 1) + 2);
      if(((storedConfig[NV_CONFIG_SETUP_BYTE3]&0x0E)>>1) == GSR_AUTORANGE) {
         gsr_active_resistor = GSR_controlRange(ADC_val, gsr_active_resistor);
         if(GSR_smoothTransition(&current_active_resistor, *(uint16_t *)(storedConfig+NV_SAMPLING_RATE))) {
            ADC_val = last_GSR_val;
         }
      }
      *((uint16_t *)txBuff1 + (nbrAdcChans - 1) + 2) =  ADC_val | (current_active_resistor << 14);
   }

   last_GSR_val = ADC_val;
}

void Itoa(uint32_t num, uint8_t* buf, uint8_t len){
   memset(buf,0,len--);
   while(len--){
      buf[len]='0'+num%10;
      num/=10;
   }
}

void DockSdPowerCycle() {
   msp430_delay_ms(60);
   P4OUT &= ~BIT2;            //SW_FLASH set low, SdPowerOff();

   P5SEL &= ~(BIT4 + BIT5);
   P5OUT &= ~(BIT4 + BIT5);   //FLASH_SOMI and FLASH_SCLK set low
   P5DIR |= BIT4;             //FLASH_SOMI set as output
   P3SEL &= ~BIT7;
   P3OUT &= ~BIT7;            //FLASH_SIMO set low
   P4OUT &= ~BIT0;            //FLASH_CS_N set low
   P6OUT &= ~(BIT6 + BIT7);   //ADC6_FLASHDAT2 and ADC7_FLASHDAT1 set low
   P6DIR |= BIT6 + BIT7;      //ADC6_FLASHDAT2 and ADC7_FLASHDAT1 set as output

   //60ms as taken from TinyOS driver (SDP.nc powerCycle() function)
   msp430_delay_ms(60);

   P5DIR &= ~(BIT4 + BIT5);   //FLASH_SOMI and FLASH_SCLK set as input
   P3DIR &= ~BIT7;            //FLASH_SIMO set as input
   P4DIR &= ~BIT0;            //FLASH_CS_N set as input
   P6DIR &= ~(BIT6 + BIT7);   //ADC6_FLASHDAT2 and ADC7_FLASHDAT1 set as input

   P4OUT |= BIT2;             //SW_FLASH set high, SdPowerOn();

   msp430_delay_ms(50);       //give SD card time to power back up
   P6OUT &= ~BIT0;            //DETECT_N set low
}

void SetDocked(void){
   inSetDockDelay = 0;
   if(dockDetected){
      docked = 1;
      enable_sdlog = 0;
      sdlogging = 0;
      UART_activate();
      UART_init();
      dir_new = 1;
      if(CheckSdInslot()){
         DockSdPowerCycle();
      }
   }
}

void SetUndocked(void){
   inSetUndockDelay = 0;
   if(!dockDetected){
      docked = 0;
      ADS1292_activate();
      P6OUT |= BIT0;
      if(CheckSdInslot()){
         SdPowerOff();
         if(!sensing)
            msp430_register_timer_cb(SetConfigVal, 60, 1);
         sdlogging = 1;
      }else {
         sdlogging = 0;
      }
   }
}

void SetConfigVal(void){   set_config = 1;}

void ReadSdConfiguration(void){
   if(sensing)
      streamData = 0;// this will skip one sample
   dir_new = 1;
   SdPowerOn();
   ParseConfig();
   ConfigureChannels();
   Calibrate();
   CheckOnDefault();
   SdPowerOff();
}

void SdPowerOff(void){   P4OUT &= ~BIT2;}//   SD power off

void SdPowerOn(void){      P4OUT |= BIT2;}//   SD power on

uint8_t CheckSdInslot(){
   //Check if card is inserted and enable interrupt for SD_DETECT_N
   if(!(P4IN & BIT1)) {
      file_bad = f_mount(0, &fatfs);
      set_sd_detect(1);
      return 1;
   }else{
      file_bad = f_mount(0,NULL);
      set_sd_detect(0);
      return 0;
   }
}

void IniReadInfoMem(){
   InfoMem_read((uint8_t *)0, storedConfig, NV_TOTAL_NUM_CONFIG_BYTES);
   if((storedConfig[NV_SENSORS0] == 0xFF) || (storedConfig[NV_BUFFER_SIZE] != 1)) {
      //if config was never written to Infomem, write default
      //assuming some other app didn't make use of InfoMem, or else InfoMem was erased
      SetDefaultConfiguration();
   }
   SetName();
   memset(sdhead_text,0,SDHEAD_LEN);
   Config2SdHead();
}

void StreamData(){
   uint8_t adc_offset;
   if(clock_freq == TCXO_CLOCK)
      adc_offset = 6;
   else
      adc_offset = 4;

   uint8_t digi_offset = (nbrAdcChans*2) + adc_offset;
   if(storedConfig[NV_SENSORS0] & SENSOR_GSR) {
      GsrRange();
   }
   if(currentBuffer){
      if(storedConfig[NV_SENSORS0] & SENSOR_MPU9150_GYRO) {
         MPU9150_getGyro(txBuff1+digi_offset);
         digi_offset+=6;
      }
      if(storedConfig[NV_SENSORS1] & SENSOR_LSM303DLHC_ACCEL) {
         LSM303DLHC_getAccel(txBuff1+digi_offset);
         digi_offset+=6;
      }
      if(storedConfig[NV_SENSORS0] & SENSOR_LSM303DLHC_MAG){
         LSM303DLHC_getMag(txBuff1+digi_offset);
         digi_offset+=6;
      }
      if(storedConfig[NV_SENSORS2] & SENSOR_MPU9150_ACCEL) {
         MPU9150_getAccel(txBuff1+digi_offset);
         digi_offset+=6;
      }
      if(storedConfig[NV_SENSORS2] & SENSOR_MPU9150_MAG) {
         if(preSampleMpuMag) {
            MPU9150_getMag(txBuff1+digi_offset);
         } else if(!mpuMagCount--) {
            MPU9150_getMag(txBuff1+digi_offset);
            mpuMagCount = mpuMagFreq;
            MPU9150_startMagMeasurement();
         } else {
            memcpy(txBuff1+digi_offset, txBuff0+digi_offset, 6);
         }
         digi_offset+=6;
      }
      if(storedConfig[NV_SENSORS2] & SENSOR_BMP180_PRESSURE) {
         if(preSampleBmpPress) {
            if(sampleBmpTemp == sampleBmpTempFreq) {
               sampleBmpTemp = 0;
               BMP180_getTemp(txBuff1+digi_offset);
               memcpy(txBuff1+digi_offset+2, txBuff0+digi_offset+2, 3);
            } else {
               BMP180_getPress(txBuff1+digi_offset+2);
               memcpy(txBuff1+digi_offset, txBuff0+digi_offset, 2);
               sampleBmpTemp++;
            }
         } else if(!bmpPressCount--) {
            if(sampleBmpTemp == sampleBmpTempFreq) {
               sampleBmpTemp = 0;
               BMP180_getTemp(txBuff1+digi_offset);
               BMP180_startPressMeasurement((storedConfig[NV_CONFIG_SETUP_BYTE3]&0x30)>>4);
               memcpy(txBuff1+digi_offset+2, txBuff0+digi_offset+2, 3);
            } else {
               BMP180_getPress(txBuff1+digi_offset+2);
               memcpy(txBuff1+digi_offset, txBuff0+digi_offset, 2);
               if(++sampleBmpTemp == sampleBmpTempFreq)
                  BMP180_startTempMeasurement();
               else
                  BMP180_startPressMeasurement((storedConfig[NV_CONFIG_SETUP_BYTE3]&0x30)>>4);
            }
            bmpPressCount = bmpPressFreq;
         } else {
            memcpy(txBuff1+digi_offset, txBuff0+digi_offset, 5);
         }
         digi_offset+=5;
      }
      if(storedConfig[NV_SENSORS0] & SENSOR_EXG1_24BIT) {
         if(docked)
            memset(txBuff1+digi_offset,0,7);
         else
            EXG_readData(0, 0, txBuff1+digi_offset);
         digi_offset+=7;
      } else if(storedConfig[NV_SENSORS2] & SENSOR_EXG1_16BIT) {
         if(docked)
            memset(txBuff1+digi_offset,0,5);
         else
            EXG_readData(0, 1, txBuff1+digi_offset);
         digi_offset+=5;
      }
      if(storedConfig[NV_SENSORS0] & SENSOR_EXG2_24BIT) {
         if(docked)
            memset(txBuff1+digi_offset,0,7);
         else
            EXG_readData(1, 0, txBuff1+digi_offset);
         digi_offset+=7;
      } else if(storedConfig[NV_SENSORS2] & SENSOR_EXG2_16BIT) {
         if(docked)
            memset(txBuff1+digi_offset,0,5);
         else
            EXG_readData(1, 1, txBuff1+digi_offset);
         digi_offset+=5;
      }

      if(stopSensing) {
         stopSensing = 0;
         *(txBuff1+digi_offset++) = ACK_COMMAND_PROCESSED;
         if(enable_btstream && btstreaming && btIsConnected && !sync_enabled)//&& !BT0SD1()
            BT_write((txBuff1+1), (digi_offset-1));
         StopStreaming();

         if(btsd_selfcmd){
            // todo: bt
            if(btIsConnected){
               uint8_t selfcmd[3];
               selfcmd[0] = INSTREAM_CMD_RESPONSE;
               selfcmd[1] = STATUS_RESPONSE;
               selfcmd[2] = (docked & 0x01) + ((sensing  & 0x01)<<1) + ((btsd_selfcmd  & 0x01)<<2);
               BT_write(selfcmd, 3);
            }
            btsd_selfcmd = 0;
         }
      }
      else{
         if(enable_sdlog && sdlogging){
            memcpy(sdbuff+sdbuff_len, txBuff1+2, block_len);
            sdbuff_len+=block_len;
         }

         if(enable_btstream && btstreaming && btIsConnected && !sync_enabled)//&& !BT0SD1()
            BT_write((txBuff1+1), (digi_offset-1));
      }

      currentBuffer = 0;
   } else {
      if(storedConfig[NV_SENSORS0] & SENSOR_MPU9150_GYRO) {
         MPU9150_getGyro(txBuff0+digi_offset);
         digi_offset+=6;
      }
      if(storedConfig[NV_SENSORS1] & SENSOR_LSM303DLHC_ACCEL) {
         LSM303DLHC_getAccel(txBuff0+digi_offset);
         digi_offset+=6;
      }
      if(storedConfig[NV_SENSORS0] & SENSOR_LSM303DLHC_MAG){
         LSM303DLHC_getMag(txBuff0+digi_offset);
         digi_offset+=6;
      }
      if(storedConfig[NV_SENSORS2] & SENSOR_MPU9150_ACCEL) {
         MPU9150_getAccel(txBuff0+digi_offset);
         digi_offset+=6;
      }
      if(storedConfig[NV_SENSORS2] & SENSOR_MPU9150_MAG) {
         if(preSampleMpuMag) {
            MPU9150_getMag(txBuff0+digi_offset);
         } else if(!mpuMagCount--) {
            MPU9150_getMag(txBuff0+digi_offset);
            mpuMagCount = mpuMagFreq;
            MPU9150_startMagMeasurement();
         } else {
            memcpy(txBuff0+digi_offset, txBuff1+digi_offset, 6);
         }
         digi_offset+=6;
      }
      if(storedConfig[NV_SENSORS2] & SENSOR_BMP180_PRESSURE) {
         if(preSampleBmpPress) {
            if(sampleBmpTemp == sampleBmpTempFreq) {
               sampleBmpTemp = 0;
               BMP180_getTemp(txBuff0+digi_offset);
               memcpy(txBuff0+digi_offset+2, txBuff1+digi_offset+2, 3);
            } else {
               BMP180_getPress(txBuff0+digi_offset+2);
               memcpy(txBuff0+digi_offset, txBuff1+digi_offset, 2);
               sampleBmpTemp++;
            }
         } else if(!bmpPressCount--) {
            if(sampleBmpTemp == sampleBmpTempFreq) {
               sampleBmpTemp = 0;
               BMP180_getTemp(txBuff0+digi_offset);
               BMP180_startPressMeasurement((storedConfig[NV_CONFIG_SETUP_BYTE3]&0x30)>>4);
               memcpy(txBuff0+digi_offset+2, txBuff1+digi_offset+2, 3);
            } else {
               BMP180_getPress(txBuff0+digi_offset+2);
               memcpy(txBuff0+digi_offset, txBuff1+digi_offset, 2);
               if(++sampleBmpTemp == sampleBmpTempFreq)
                  BMP180_startTempMeasurement();
               else
                  BMP180_startPressMeasurement((storedConfig[NV_CONFIG_SETUP_BYTE3]&0x30)>>4);
            }
            bmpPressCount = bmpPressFreq;
         } else {
            memcpy(txBuff0+digi_offset, txBuff1+digi_offset, 5);
         }
         digi_offset+=5;
      }
      if(storedConfig[NV_SENSORS0] & SENSOR_EXG1_24BIT) {
         if(docked)
            memset(txBuff0+digi_offset,0,7);
         else
            EXG_readData(0, 0, txBuff0+digi_offset);
         digi_offset+=7;
      } else if(storedConfig[NV_SENSORS2] & SENSOR_EXG1_16BIT) {
         if(docked)
            memset(txBuff0+digi_offset,0,5);
         else
            EXG_readData(0, 1, txBuff0+digi_offset);
         digi_offset+=5;
      }
      if(storedConfig[NV_SENSORS0] & SENSOR_EXG2_24BIT) {
         if(docked)
            memset(txBuff0+digi_offset,0,7);
         else
            EXG_readData(1, 0, txBuff0+digi_offset);
         digi_offset+=7;
      } else if(storedConfig[NV_SENSORS2] & SENSOR_EXG2_16BIT) {
         if(docked)
            memset(txBuff0+digi_offset,0,5);
         else
            EXG_readData(1, 1, txBuff0+digi_offset);
         digi_offset+=5;
      }

      if(stopSensing) {
         stopSensing = 0;
         *(txBuff0+digi_offset++) = ACK_COMMAND_PROCESSED;
         if(enable_btstream && btstreaming && btIsConnected && !sync_enabled)//!BT0SD1()
            BT_write((txBuff0+1), (digi_offset-1));
         StopStreaming();

         if(btsd_selfcmd){
            if(btIsConnected){
               uint8_t selfcmd[3];
               selfcmd[0] = INSTREAM_CMD_RESPONSE;
               selfcmd[1] = STATUS_RESPONSE;
               selfcmd[2] = (docked & 0x01) + ((sensing  & 0x01)<<1) + ((btsd_selfcmd  & 0x01)<<2);
               BT_write(selfcmd, 3);
            }
            btsd_selfcmd = 0;
         }
      }
      else{
         if(enable_sdlog && sdlogging){
            memcpy(sdbuff+sdbuff_len, txBuff0+2, digi_offset-2);
            sdbuff_len+=digi_offset-2;
         }

         if(enable_btstream && btstreaming && btIsConnected && !sync_enabled)//!BT0SD1()
            BT_write((txBuff0+1), (digi_offset-1));
      }

      currentBuffer = 1;
   }
}

void RstRcommVariables(){
   rcomm_cnt = 0;
   rcomm_window_c = RC_WINDOW_C;
   rcomm_current_try = 0;
   rcomm_interval = RC_INT_N;
   rcomm_interval_c = storedConfig[NV_BT_INTERVAL];
   rcomm_success = 0;
}

void Write2SD(){
   WriteFile(sdbuff, sdbuff_len);
   sdbuff_len = 0;
   if(storedConfig[NV_TRIAL_CONFIG0]&SDH_TIME_SYNC)
      PrepareSDBuffHead();
}

void UpdateSdConfig(){
   if(!docked){
   uint8_t sd_power_state;
   if(!(P4OUT & BIT2)){
      SdPowerOn();
      sd_power_state = 0;
   }
   else
      sd_power_state = 1;
   float val_num;
   uint16_t val_int, val_f;
   char buffer[66], val_char[20];
   UINT bw;

   char cfgname[]="sdlog.cfg";

   file_bad = f_open(&fil, cfgname, FA_WRITE | FA_CREATE_ALWAYS);
   //sensor0
   sprintf(buffer, "accel=%d\r\n", storedConfig[NV_SENSORS0] & SENSOR_A_ACCEL?1:0);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "gyro=%d\r\n", storedConfig[NV_SENSORS0] & SENSOR_MPU9150_GYRO?1:0);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "mag=%d\r\n", storedConfig[NV_SENSORS0] & SENSOR_LSM303DLHC_MAG?1:0);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "exg1_24bit=%d\r\n", storedConfig[NV_SENSORS0] & SENSOR_EXG1_24BIT?1:0);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "exg2_24bit=%d\r\n", storedConfig[NV_SENSORS0] & SENSOR_EXG2_24BIT?1:0);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "gsr=%d\r\n", storedConfig[NV_SENSORS0] & SENSOR_GSR?1:0);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "extch7=%d\r\n", storedConfig[NV_SENSORS0] & SENSOR_EXT_A7?1:0);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "extch6=%d\r\n", storedConfig[NV_SENSORS0] & SENSOR_EXT_A6?1:0);
   f_write(&fil, buffer, strlen(buffer), &bw);
   //sensor1
   sprintf(buffer, "vbat=%d\r\n", storedConfig[NV_SENSORS1] & SENSOR_VBATT?1:0);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "accel_d=%d\r\n", storedConfig[NV_SENSORS1] & SENSOR_LSM303DLHC_ACCEL?1:0);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "extch15=%d\r\n", storedConfig[NV_SENSORS1] & SENSOR_EXT_A15?1:0);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "intch1=%d\r\n", storedConfig[NV_SENSORS1] & SENSOR_INT_A1?1:0);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "intch12=%d\r\n", storedConfig[NV_SENSORS1] & SENSOR_INT_A12?1:0);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "intch13=%d\r\n", storedConfig[NV_SENSORS1] & SENSOR_INT_A13?1:0);
   f_write(&fil, buffer, strlen(buffer), &bw);
   //sensor2
   sprintf(buffer, "intch14=%d\r\n", storedConfig[NV_SENSORS2] & SENSOR_INT_A14?1:0);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "accel_mpu=%d\r\n", storedConfig[NV_SENSORS2] & SENSOR_MPU9150_ACCEL?1:0);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "mag_mpu=%d\r\n", storedConfig[NV_SENSORS2] & SENSOR_MPU9150_MAG?1:0);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "exg1_16bit=%d\r\n", storedConfig[NV_SENSORS2] & SENSOR_EXG1_16BIT?1:0);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "exg2_16bit=%d\r\n", storedConfig[NV_SENSORS2] & SENSOR_EXG2_16BIT?1:0);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "pres_bmp180=%d\r\n", storedConfig[NV_SENSORS2] & SENSOR_BMP180_PRESSURE?1:0);
   f_write(&fil, buffer, strlen(buffer), &bw);
   //sample_rate
   val_num =clock_freq/(*(uint16_t*)(storedConfig+NV_SAMPLING_RATE));
   val_int = (uint16_t)floor(val_num);
   val_f = (uint16_t)floor((val_num-floor(val_num))*100);
   if(val_f)
      sprintf(val_char, "%d.%d", val_int, val_f);
   else
      sprintf(val_char, "%d", val_int);
   sprintf(buffer, "sample_rate=%s\r\n", val_char);
   f_write(&fil, buffer, strlen(buffer), &bw);
   //setup config
   sprintf(buffer, "mg_internal_rate=%d\r\n", (storedConfig[NV_CONFIG_SETUP_BYTE2]>>2)&0x07);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "mg_range=%d\r\n", (storedConfig[NV_CONFIG_SETUP_BYTE2]>>5)&0x07);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "acc_internal_rate=%d\r\n", (storedConfig[NV_CONFIG_SETUP_BYTE0]>>4)&0x0f);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "accel_mpu_range=%d\r\n", (storedConfig[NV_CONFIG_SETUP_BYTE3]>>6)&0x03);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "pres_bmp180_prec=%d\r\n", (storedConfig[NV_CONFIG_SETUP_BYTE3]>>4)&0x03);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "gs_range=%d\r\n", (storedConfig[NV_CONFIG_SETUP_BYTE3]>>1)&0x07);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "exp_power=%d\r\n", storedConfig[NV_CONFIG_SETUP_BYTE3] & EXP_POWER_ENABLE?1:0);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "gyro_range=%d\r\n", (storedConfig[NV_CONFIG_SETUP_BYTE2])&0x03);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "gyro_samplingrate=%d\r\n", storedConfig[NV_CONFIG_SETUP_BYTE1]);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "acc_range=%d\r\n", (storedConfig[NV_CONFIG_SETUP_BYTE0]>>2)&0x03);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "acc_lpm=%d\r\n", (storedConfig[NV_CONFIG_SETUP_BYTE0]>>1)&0x01);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "acc_hrm=%d\r\n", (storedConfig[NV_CONFIG_SETUP_BYTE0])&0x01);
   f_write(&fil, buffer, strlen(buffer), &bw);
   //trial config
   sprintf(buffer, "user_button_enable=%d\r\n", storedConfig[NV_TRIAL_CONFIG0]&SDH_USER_BUTTON_ENABLE?1:0);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "iammaster=%d\r\n", storedConfig[NV_TRIAL_CONFIG0]&SDH_IAMMASTER?1:0);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "sync=%d\r\n", storedConfig[NV_TRIAL_CONFIG0]&SDH_TIME_SYNC?1:0);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "singletouch=%d\r\n", storedConfig[NV_TRIAL_CONFIG1]&SDH_SINGLETOUCH?1:0);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "tcxo=%d\r\n", storedConfig[NV_TRIAL_CONFIG1]&SDH_TCXO?1:0);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "interval=%d\r\n", storedConfig[NV_BT_INTERVAL]);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "center=%s\r\n", centername);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "myid=%d\r\n", sdhead_text[SDH_MYTRIAL_ID]);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "Nshimmer=%d\r\n", sdhead_text[SDH_NSHIMMER]);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "shimmername=%s\r\n", shimmername);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "experimentid=%s\r\n", exp_id_name);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "configtime=%s\r\n", config_time_text);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "EXG_ADS1292R_1_CONFIG1=%d\r\n", storedConfig[NV_EXG_ADS1292R_1_CONFIG1]);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "EXG_ADS1292R_1_CONFIG2=%d\r\n", storedConfig[NV_EXG_ADS1292R_1_CONFIG2]);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "EXG_ADS1292R_1_LOFF=%d\r\n", storedConfig[NV_EXG_ADS1292R_1_LOFF]);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "EXG_ADS1292R_1_CH1SET=%d\r\n", storedConfig[NV_EXG_ADS1292R_1_CH1SET]);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "EXG_ADS1292R_1_CH2SET=%d\r\n", storedConfig[NV_EXG_ADS1292R_1_CH2SET]);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "EXG_ADS1292R_1_RLD_SENS=%d\r\n", storedConfig[NV_EXG_ADS1292R_1_RLD_SENS]);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "EXG_ADS1292R_1_LOFF_SENS=%d\r\n", storedConfig[NV_EXG_ADS1292R_1_LOFF_SENS]);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "EXG_ADS1292R_1_LOFF_STAT=%d\r\n", storedConfig[NV_EXG_ADS1292R_1_LOFF_STAT]);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "EXG_ADS1292R_1_RESP1=%d\r\n", storedConfig[NV_EXG_ADS1292R_1_RESP1]);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "EXG_ADS1292R_1_RESP2=%d\r\n", storedConfig[NV_EXG_ADS1292R_1_RESP2]);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "EXG_ADS1292R_2_CONFIG1=%d\r\n", storedConfig[NV_EXG_ADS1292R_2_CONFIG1]);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "EXG_ADS1292R_2_CONFIG2=%d\r\n", storedConfig[NV_EXG_ADS1292R_2_CONFIG2]);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "EXG_ADS1292R_2_LOFF=%d\r\n", storedConfig[NV_EXG_ADS1292R_2_LOFF]);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "EXG_ADS1292R_2_CH1SET=%d\r\n", storedConfig[NV_EXG_ADS1292R_2_CH1SET]);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "EXG_ADS1292R_2_CH2SET=%d\r\n", storedConfig[NV_EXG_ADS1292R_2_CH2SET]);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "EXG_ADS1292R_2_RLD_SENS=%d\r\n", storedConfig[NV_EXG_ADS1292R_2_RLD_SENS]);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "EXG_ADS1292R_2_LOFF_SENS=%d\r\n", storedConfig[NV_EXG_ADS1292R_2_LOFF_SENS]);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "EXG_ADS1292R_2_LOFF_STAT=%d\r\n", storedConfig[NV_EXG_ADS1292R_2_LOFF_STAT]);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "EXG_ADS1292R_2_RESP1=%d\r\n", storedConfig[NV_EXG_ADS1292R_2_RESP1]);
   f_write(&fil, buffer, strlen(buffer), &bw);
   sprintf(buffer, "EXG_ADS1292R_2_RESP2=%d\r\n", storedConfig[NV_EXG_ADS1292R_2_RESP2]);
   f_write(&fil, buffer, strlen(buffer), &bw);

   file_bad = f_close(&fil);
   if(!sd_power_state)
      msp430_register_timer_cb(SdPowerOff,50,0);
   }
}

void BattBlinkOn(){
   switch(batt_stat){
   case BATT_HIGH:
      Board_ledOn(LED_GREEN0);
      break;
   case BATT_MID:
      Board_ledOn(LED_YELLOW);
      break;
   case BATT_LOW:
      Board_ledOn(LED_RED);
      break;
   default: break;
   }
}

void SetBattDma(){
   ADC12CTL0 &= ~ADC12ENC;                                  //ensure is off so all ADC12CTL0 and ADC12CTL1 fields can be modified
   ADC12CTL0 = ADC12SHT1_8 + ADC12SHT0_8 + ADC12MSC;        //SHT1 and SHT0 of 256 ADC12CLK cycles, enable multiple sample and conversion
   ADC12CTL1 = ADC12SHP + ADC12CONSEQ_1;                    //use sampling timer, ADC12OSC and sequence-of-channels mode
   ADC12CTL2 = ADC12TCOFF + ADC12RES_2 + ADC12SR_L;         //turn off temperature sensor (saves power), ADC12 resolution to 12bits,
   ADC12CTL1 += ADC12CSTARTADD_4;                           //Start with ADC12MEM4
   ADC12MCTL4 = ADC12INCH_2 + ADC12EOS;

   DMACTL0 |= DMA0TSEL_24;                                  //ADC12IFGx triggered
   DMACTL4 = DMARMWDIS;                                     //Read-modify-write disable
   DMA0CTL &= ~DMAIFG;
   DMA0CTL = DMADT_1+DMADSTINCR_3+DMASRCINCR_3+DMAIE;       //block transfer, inc dst, inc src, Int enable
   DMA0SZ = 1;                                              //DMA0 size

   //Writes a value to a 20-bit SFR register located at the given16/20-bit address
   DMA0SA = (__SFR_FARPTR) (unsigned long) &ADC12MEM4;
   DMA0DA = (__SFR_FARPTR) (unsigned long) batt_val;
   DMA0_transferDoneFunction(&Dma0ConversionDone);

   ADC12IFG = 0;
   DMA0CTL |= DMAEN;
   ADC12CTL0 |= ADC12ON + ADC12ENC + ADC12SC;
}

void TB0Start(){
   if(sample_timer_status + blink_status >= 1){
      if(clock_freq == MSP430_CLOCK){
         TB0CTL = TBSSEL_1 + MC_2;
         TB0EX0 = 0;
      }else{
         TB0CTL = TBSSEL_0 + MC_2 + ID__8;// use TBSSEL_0 for tcxo, ID__8:divider=8//+ TBCLR
         TB0EX0 = TBIDEX__8;              // divider: 8
      }
   }
}

void TB0Stop(){
   if(!sample_timer_status && !blink_status)
      TB0CTL = MC_0;
}

void ClkAssignment(){
   clk_10 = FreqProd(10);
   clk_50 = FreqProd(50);
   clk_150 = FreqProd(150);
   clk_45 = FreqProd(45);
   clk_75 = FreqProd(75);
   clk_90 = FreqProd(90);
   clk_135 = FreqProd(135);
   clk_90_45 = clk_90-clk_45;
   clk_90_75 = clk_90-clk_75;
   clk_135_90 = clk_135-clk_90;
   clk_120 = FreqProd(120);
   clk_105 = FreqProd(105);
   clk_165 = FreqProd(165);
   clk_250 = FreqProd(250);
   clk_250_90 = clk_250-clk_90;
   clk_255 = FreqProd(255);
   clk_285 = FreqProd(285);
   clk_1000 = FreqProd(1000);
   clk_2500 = FreqProd(2500);
}

uint16_t FreqProd(uint16_t num_in){// e.g. 7.5 ms: num_in=75, .25s:num_in=2500
   return (uint16_t)ceil(clock_freq*(float)num_in/10000);
}

uint16_t FreqDiv(float num_in){
   return (uint16_t)ceil(clock_freq/num_in);
}

// trap isr assignation - put all unused ISR vector here
// no COMP_B_VECTOR,TIMER2_A0_VECTOR , TIMER2_A1_VECTOR,
//USCI_B0_VECTOR:i2c
//USCI_A0_VECTOR:dock/exp_uart
//USCI_A1_VECTOR:bt_uart
//TIMER1_A0_VECTOR: msp430_clock.h
#pragma vector = WDT_VECTOR, RTC_VECTOR, SYSNMI_VECTOR, TIMER0_A0_VECTOR, \
                 UNMI_VECTOR, USCI_B1_VECTOR, TIMER1_A1_VECTOR
__interrupt void TrapIsr(void){
   // this is a trap ISR - check for the interrupt cause here by
   // checking the interrupt flags, if necessary also clear the interrupt
   // flag
}
