/*
 * Copyright (c) 2013, Shimmer Research, Ltd.
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
 * @date June, 2014
 */

/***********************************************************************************
 Data Buffer Format:
      Packet Type |TimeStamp|Achan1|Achan2| ... |AchanX|Dchan1|Dchan2| ... |DchanX|
 Byte:    0-1     |   2-3   |Achan1|Achan2| ... |AchanX|Dchan1|Dchan2| ... |DchanX|

 Log file Format:
      SD Header   |TimeStamp1   |Achan1 data1| ... |DchanX data1|TimeStamp2   |Achan2 data2|...
   Byte:  0-255   |  256-257    |   258-259  | ... |            |             |            |...

***********************************************************************************/

/***********************************************************************************
   Synchronisation: BlueTooth Routine Communication (BT Rcomm)

   Concept:    In a BT Rcomm system, there must be only 1 shimmer configured as the
               center, all the other shimmers must be configured as nodes.
               Center:  In every 'interval' seconds, turns on its bluetooth and awaits
                        the connection attempts from the nodes.
               Nodes:   Attemp to connect to the center, get the center's current
                        timestamp (4 bytes), compare the time difference with the
                        nodes' own current timestamp and record it in the logging data.
                        If the attempt succeeds, retry in 'interval' seconds.
                        If the attempt fails, retry in a shorter period.
   Key word:   "sync=1" to enable Synchronisation
               "singletouch=1" also enables Synchronisation
               "interval=" sets the interval time between two sync attempts
               "center=" sets the BT mac address of the center shimmer
   Mode:       Center: passively (in BT:SLAVE_MODE) waits for connects from the nodes
               Nodes: actively (in BT:MASTER_MODE) connect to the center
   Advantage:  The center doesn't care how many nodes there are and who they are.
               All nodes only have to configure the same center name.
               Easy to add/remove/change nodes.
***********************************************************************************/


#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "msp430.h"
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
#include "bmp180/bmp180.h"
#include "CAT24C16/cat24c16.h"
#include "EXG/exg.h"
#include "EXG/ads1292.h"
#include "FatFs/ff.h"
#include "GSR/gsr.h"
#include "LSM303DLHC/lsm303dlhc.h"
#include "MPU9150/mpu9150.h"
#include "msp430_clock/msp430_clock.h"
#include "shimmer_sd.h"

typedef uint8_t bool;
#define TRUE 1
#define FALSE 0
typedef uint8_t error_t;
#define SUCCESS 0
#define FAIL 1

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
uint8_t BtDataAvailable(uint8_t data);
uint8_t Dma0ConversionDone(void);
uint8_t Dma2ConversionDone(void);
void ProcessCommand(void);
void ConfigureChannels(void);
void OpenFirstFile();
void Timestamp0ToFirstFile();
FRESULT WriteFile(uint8_t* text, WORD size);
FRESULT SetDirectory();
void PrepareSDBuffHead(void);
error_t SetBasedir();
error_t MakeBasedir();
inline void GsrRange(void);
void Itoa(uint32_t num, uint8_t* buf, uint8_t len);
void DockSdPowerCycle();
void CheckSdInslot();
void ParseRcommResponse();
void SendRcommCommand();
void BtStop();
void BtStart();
void BtStartDone();
void StreamData();
void Write2SD();
void TB0Start();
void TB0Stop();
void SetupRTC(void);
uint32_t GetRTC();
void ParseConfig();
void Config2SdHead(void);
void RstRcommVariables();
void Calibrate();
void ReadCalibration(uint8_t sensor);
void DefaultCalibration(uint8_t sensor);
void BattBlinkOn();
void SetBattDma();
void ClkAssignment();
uint16_t FreqProd(uint16_t num_in);
uint16_t FreqDiv(float num_in);


//data segment initialisation is disabled in system_pre_init.c
uint8_t currentBuffer, processCommand, startSensing, stopSensing, streamData, sensing, btIsConnected, sd_write;

uint8_t exp_cmd[4], mac[14],storedConfig[NV_TOTAL_NUM_CONFIG_BYTES], configuring;//
uint8_t nbrAdcChans, nbrDigiChans;
uint16_t *adcStartPtr;

//1byte packet type, 2byte timestamp, 3x2byte analog accel, 3x2byte gyro, 2byte batt, 3x2byte accel
// + 3x2byte mag + 1 byte at start
//as can only read/write 16-bit values at even addresses
uint8_t txBuff0[DATA_PACKET_SIZE], txBuff1[DATA_PACKET_SIZE], btrx_buff[14], *btrx_exp;

// file system vars
FATFS fatfs;         // File object
DIRS dir;            //Directory object
FIL fil;
uint8_t sdbuff[SDBUFF_SIZE], dir_new, len_dir;
uint16_t sdbuff_len, block_len, file_num;
// make dir for SDLog files
uint8_t dirname[64], exp_dir_name[32],sdhead_text[SDHEAD_LEN],file_bad,bt_bad,
      filename[64],  exp_id_name[MAX_CHARS], shimmername[MAX_CHARS],config_time_text[MAX_CHARS], centername[MAX_CHARS];
uint16_t dir_counter, blink_cnt_2, blink_cnt_5;
uint32_t last_hour, last_min;
uint8_t my_time_diff[5];

// battery evaluation vars
uint8_t batt_stat, get_vbatt, batt_read, batt_wait,  batt_val[2];
uint32_t batt_last_time;

uint8_t get_rcomm, rcomm_timeout, rcomm_resp[RCT_SIZE], rcomm_command, rcomm_status, rcomm_current_try, rcomm_window_c,
      parse_rcomm_resp, bt_status, sample_timer_status, blink_status, rcomm_interval, rcomm_success;
uint16_t rcomm_cnt, rcomm_interval_c, rcomm_special_int;

uint8_t docked,set_undock, set_undock_done, set_undock_start, on_user_button, on_single_touch, initializing,on_default;
uint8_t preSampleBmpPress, bmpPressFreq, bmpPressCount, sampleBmpTemp, sampleBmpTempFreq, sampleBmp180Press;
uint8_t preSampleMpuMag, mpuMagFreq, mpuMagCount, mpu9150Initialised, sampleMpu9150Mag, gsr_active_resistor;
uint16_t last_GSR_val;
float clock_freq;
uint16_t clk_10,clk_50,clk_150,clk_45,clk_90,clk_135,clk_75,clk_90_45,clk_90_75,clk_135_90,
      clk_250,clk_250_90, clk_255,clk_105, clk_120,clk_2500,clk_1000,clk_165,clk_285;

//ExG
uint8_t exgLength, exgChip, exgStartAddr;
//Daughter card EEPROM
uint8_t dcMemLength;
uint16_t dcMemOffset;


void main(void) {
   initializing = 1;                // led flag, in initialisation period
   Init();

   SetupRTC();
   CommTimerStart();

   initializing = 0;
   while(1) {
      __bis_SR_register(LPM3_bits + GIE);   //ACLK remains active

      if(set_undock){
         configuring = 1;           // led flag, in configuration period

         P6OUT |= BIT0;             //   DETECT_N set to high

         P4OUT &= ~BIT2;            //   SD power off
         msp430_delay_ms(120);      //wait 60ms (assuming 24MHz MCLK)
         P4OUT |= BIT2;             //   SD power on

         set_undock = 0;
         set_undock_start =1;

         on_user_button = 0;
         on_single_touch = 0;
         on_default = 0;

         ParseConfig();
         ConfigureChannels();
         Calibrate();

         if(sdhead_text[SDH_TRIAL_CONFIG1]&SDH_SINGLETOUCH)    // set trigger mode
            on_single_touch = 1;
         else if(sdhead_text[SDH_TRIAL_CONFIG0]&SDH_USER_BUTTON_ENABLE)
            on_user_button = 1;
         else{
            on_default = 1;
            if(!file_bad)
               startSensing = 1;
         }

         set_undock_done =1;
         configuring = 0;
      }

      if(file_bad){
         while(file_bad);
      }
      if(batt_read){// use adc channel2 and mem4, read back battery status every certain period
         batt_read = 0;
         batt_wait = 1;
         SetBattDma();
      }
      if(processCommand) {
         processCommand = 0;
         ProcessCommand();
      }
      if(parse_rcomm_resp){
         parse_rcomm_resp = 0;
         ParseRcommResponse();

      }
      if(stopSensing) {
         stopSensing = 0;
         if(sensing)
            StopStreaming();
      }
      if(startSensing) {
         startSensing = 0;

         memset(txBuff0,0,DATA_PACKET_SIZE); // initialise the sensor data buff.
         memset(txBuff1,0,DATA_PACKET_SIZE);
         if(dir_new){
            P4OUT |= BIT2; //sd power
            file_bad = SetBasedir();
            file_bad = MakeBasedir();
            file_num=0;
            sdbuff_len = 0;
             OpenFirstFile();
             dir_new = 0;
         }
         StartStreaming();
         Timestamp0ToFirstFile();
         _NOP();
      }
      if (sampleMpu9150Mag) {
         _NOP();
         sampleMpu9150Mag = 0;
         MPU9150_startMagMeasurement();
         _NOP();
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
         StreamData();
         // if sensor date buffer is large enough, write it to SDcard and clear the buffer
         if (sdbuff_len > SDBUFF_SIZE-block_len){
            sd_write = 1;
         }
         else{
            sd_write = 0;
         }
         _NOP();
      }
      if(sd_write){
         sd_write=0;


         Write2SD();
      }
   _NOP();
   }
}


void Init(void) {
   //WDTCTL = WDTPW + WDTHOLD; // Stop WDTp already handled in system_pre_init.c
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
   // flag variables initialisation
   configuring = 0;
   batt_last_time = 0;
   batt_read = 1;
   batt_wait = 0;
   batt_stat = 0;
   blink_cnt_2 = 0;
   blink_cnt_5 = 0;
   file_bad = 0;
   bt_bad = 0;
   rcomm_special_int = 0;
   rcomm_window_c = RC_WINDOW_C;
   rcomm_status = 0;
   on_default = 0;
   on_user_button = 0;
   on_single_touch = 0;
   set_undock_done = 0;
   set_undock_start = 0;
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
   dir_new = 1;
   memset(sdbuff,0,SDBUFF_SIZE);
   memset(storedConfig,0,NV_TOTAL_NUM_CONFIG_BYTES);
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

   //Check dock status and enable interrupt for EXP_DETECT_N
   if(P2IN & BIT3) {
      P2IES |= BIT3;   //look for falling edge
      docked = 1;
      UART_setExgOrUart(1);
      UART_init();
      set_undock_start=0;
      set_undock_done=0;
      DockSdPowerCycle();
   } else {
      P2IES &= ~BIT3;   //look for rising edge
      docked = 0;
      UART_setExgOrUart(0);
   }
   P2IFG &= ~BIT3;      //clear flag
   P2IE |= BIT3;        //enable interrupt

   // Globally enable interrupts
   _enable_interrupts();
   BlinkTimerStart();

   memset(btrx_buff,0,14);
   DMA2_init((uint16_t *)&UCA1RXBUF, (uint16_t *)btrx_buff, 14);
   DMA2_transferDoneFunction(&Dma2ConversionDone);
   btrx_exp = BT_getExpResp();
   // =========== below initialize bt for the first time and get its MAC address only ==========
   InfoMem_read((uint8_t *)0x1f0, mac, 14);

   if(mac[0] == 0xFF || (mac[12]!=0x0d)) {
      uint8_t j = 0;
      do{
         BT_init();
         BT_setGetMacAddress(1);
         BtStart();
         msp430_delay_ms(2200);

         uint8_t i = 20 ;
         while((mac[11]==0xff) && i){
            i--;
            msp430_delay_ms(100);
         }
         BtStop();
         if(!i){
            bt_bad = 1;
            msp430_delay_ms(8000);
         }
         else{
            bt_bad = 0;
            ;
         }
         if(j >= 3){
            // try 3 times max, if still bad, software POR reset
            PMMCTL0 = PMMPW + PMMSWPOR + (PMMCTL0 & 0x0003);
         }
      }while(bt_bad && j++>3);

      InfoMem_write((void*)0x1f0, mac, 14);
   }
   // =========== above initialize bt for the first time and get its MAC address only ==========
   memcpy((char*)exp_cmd,"mac$",4);
   UART_setStr(exp_cmd, 4, mac, 12);

   // enable switch1 interrupt
   Button_init();
   Button_interruptEnable();

   //EXP_RESET_N
   P3OUT &= ~BIT3;      //set low
   P3DIR |= BIT3;       //set as output

   CheckSdInslot();
}


void StartStreaming(void) {
   uint8_t i2c_en = 0;

   if(!sensing) {
      sensing = 1;
      if(storedConfig[NV_SENSORS0] & SENSOR_A_ACCEL) {
          P8REN &= ~BIT6;      //disable pull down resistor
          P8DIR |= BIT6;       //set as output
         P8OUT |= BIT6;      //analog accel being used so take out of sleep mode
      }

      if(storedConfig[NV_CONFIG_SETUP_BYTE3] & EXP_POWER_ENABLE){//power of exp board
         P3OUT |= BIT3;
      }

      if(storedConfig[NV_SENSORS1] & SENSOR_STRAIN){
         P2OUT |= BIT0;                            //GPIO_INTERNAL1 set high
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
               MPU9150_setGyroSensitivity(storedConfig[NV_CONFIG_SETUP_BYTE2]&0x03); //This needs to go after the wake?
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

      if((storedConfig[NV_SENSORS0] & SENSOR_LSM303DLHC_MAG) || (storedConfig[NV_SENSORS1] & SENSOR_LSM303DLHC_ACCEL)){
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
            //i2c_en = 1;
         }
         if((((storedConfig[NV_CONFIG_SETUP_BYTE3]&0x30)>>4)==0)
               && (*(uint16_t *)(storedConfig+NV_SAMPLING_RATE) >= clk_75)) {
            //max of approx. 3ms to sample everything + 4.5ms between starting press to data ready
            //so 7.5ms in total (246 ticks of 32768Hz clock = 7.507ms)(1919 ticks of 255765.625Hz clock = 7.5030ms)
            preSampleBmpPress = 1;
            bmpPressFreq = 1;      //required for the calculation of sampleBmpTempFreq below
         } else if((((storedConfig[NV_CONFIG_SETUP_BYTE3]&0x30)>>4)==1)
               && (*(uint16_t *)(storedConfig+NV_SAMPLING_RATE) >= clk_105)) {
            //max of approx. 3ms to sample everything + 7.5ms between starting press to data ready
            //so 10.5ms in total (345 ticks of 32768Hz clock = 10.529ms)(2686 ticks of 255765.625Hz clock = 10.5018ms)
            preSampleBmpPress = 1;
            bmpPressFreq = 1;      //required for the calculation of sampleBmpTempFreq below
         } else if((((storedConfig[NV_CONFIG_SETUP_BYTE3]&0x30)>>4)==2)
               && (*(uint16_t *)(storedConfig+NV_SAMPLING_RATE) >= clk_165)) {
            //max of approx. 3ms to sample everything + 13.5ms between starting press to data ready
            //so 16.5ms in total (541 ticks of 32768Hz clock = 16.510ms)(4221 ticks of 255765.625Hz clock = 16.5034ms)
            preSampleBmpPress = 1;
            bmpPressFreq = 1;      //required for the calculation of sampleBmpTempFreq below
         } else if((((storedConfig[NV_CONFIG_SETUP_BYTE3]&0x30)>>4)==3)
               && (*(uint16_t *)(storedConfig+NV_SAMPLING_RATE) >= clk_285)) {
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
            sampleBmpTemp = sampleBmpTempFreq =
                  (uint8_t)(FreqDiv( *(uint16_t *)(storedConfig+NV_SAMPLING_RATE)) - 1) / bmpPressFreq;
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
   }

   SampleTimerStart();
   if(sdhead_text[SDH_TRIAL_CONFIG0]&SDH_TIME_SYNC)
      PrepareSDBuffHead();
}


inline void StopStreaming(void) {
   //shut every thing down
   sensing = 0;

   if(docked)   // if docked, cannot write to SD card any more
      DockSdPowerCycle();
   else{
      dir_new = 1;
      file_bad = f_close(&fil);
      msp430_delay_ms(50);
   }
   P4OUT&= ~BIT2;      //sd access close

   SampleTimerStop();
   ADC_disable();
   DMA0_disable();

   P8OUT &= ~BIT6;
   P8REN |= BIT6;        //enable pull down resistor
   P8DIR &= ~BIT6;       //SW_ACCEL set as input

   P3OUT &= ~BIT3;       //set EXP_RESET_N low

   P2OUT &= ~BIT0;       //set GPIO_INTERNAL1 low

   if((storedConfig[NV_SENSORS0] & SENSOR_MPU9150_GYRO) || (storedConfig[NV_SENSORS2] & SENSOR_MPU9150_ACCEL)) {
      MPU9150_wake(0);
   }
   if((storedConfig[NV_SENSORS0] & SENSOR_LSM303DLHC_MAG) || (storedConfig[NV_SENSORS1] & SENSOR_LSM303DLHC_ACCEL)) {
      LSM303DLHC_sleep();
   }

   if((storedConfig[NV_SENSORS0] & SENSOR_EXG1_24BIT) || (storedConfig[NV_SENSORS2] & SENSOR_EXG1_16BIT)) {
      EXG_stop(0);       //probably not needed
   }
   if((storedConfig[NV_SENSORS0] & SENSOR_EXG2_24BIT) || (storedConfig[NV_SENSORS2] & SENSOR_EXG2_16BIT)) {
      EXG_stop(1);       //probably not needed
   }
   if((storedConfig[NV_SENSORS0] & SENSOR_EXG1_24BIT) || (storedConfig[NV_SENSORS0] & SENSOR_EXG2_24BIT) ||
        (storedConfig[NV_SENSORS2] & SENSOR_EXG1_16BIT) || (storedConfig[NV_SENSORS2] & SENSOR_EXG2_16BIT)) {
      EXG_powerOff();
   }
   msp430_delay_ms(10);  //give plenty of time for I2C operations to finish before disabling I2C
   I2C_Disable();
   I2C_PowerOff();       //set SW_I2C low to power off I2C chips
   streamData = 0;
   sd_write = 0;
   sdbuff_len = 0;
   sampleBmp180Press = 0;
   sampleMpu9150Mag = 0;
}


void ConfigureChannels(void) {
   uint16_t mask=0;

   nbrAdcChans = nbrDigiChans = 0;

   //Analog Accel
   if(storedConfig[NV_SENSORS0] & SENSOR_A_ACCEL) {
      mask += MASK_A_ACCEL;
      nbrAdcChans += 3;
   }
   //Battery Voltage
   if(storedConfig[NV_SENSORS1] & SENSOR_VBATT) {
      mask += MASK_VBATT;
      nbrAdcChans++;
   }
   //Strain gauge
   if(storedConfig[NV_SENSORS1] & SENSOR_STRAIN) {
      mask += MASK_STRAIN;
      nbrAdcChans += 2;
   }
   //External ADC channel A7
   if(storedConfig[NV_SENSORS0] & SENSOR_EXT_A7) {
      mask += MASK_EXT_A7;
      nbrAdcChans++;
   }
   //External ADC channel A6
   if(storedConfig[NV_SENSORS0] & SENSOR_EXT_A6) {
      mask += MASK_EXT_A6;
      nbrAdcChans++;
   }
   //External ADC channel A15
   if(storedConfig[NV_SENSORS1] & SENSOR_EXT_A15) {
      mask += MASK_EXT_A15;
      nbrAdcChans++;
   }
   //Internal ADC channel A12
   if(storedConfig[NV_SENSORS1] & SENSOR_INT_A12) {
      mask += MASK_INT_A12;
      nbrAdcChans++;
   }
   //Internal ADC channel A13
   if((storedConfig[NV_SENSORS1] & SENSOR_INT_A13) && !(storedConfig[NV_SENSORS1] & SENSOR_STRAIN)){
      mask += MASK_INT_A13;
      nbrAdcChans++;
   }
   //Internal ADC channel A14
   if((storedConfig[NV_SENSORS2] & SENSOR_INT_A14) && !(storedConfig[NV_SENSORS1] & SENSOR_STRAIN)){
      mask += MASK_INT_A14;
      nbrAdcChans++;
   }
   //Internal ADC channel A1
   if (storedConfig[NV_SENSORS0] & SENSOR_GSR) {
      mask += MASK_INT_A1;
      nbrAdcChans++;
   }
   if(storedConfig[NV_SENSORS1] & SENSOR_INT_A1) {
      mask += MASK_INT_A1;
      nbrAdcChans++;
   }
   //Digi Gyro
   if(storedConfig[NV_SENSORS0] & SENSOR_MPU9150_GYRO) {
      nbrDigiChans += 3;
   }
   //Digi Accel
   if(storedConfig[NV_SENSORS1] & SENSOR_LSM303DLHC_ACCEL) {
      nbrDigiChans += 3;
   }
   //Mag
   if(storedConfig[NV_SENSORS0] & SENSOR_LSM303DLHC_MAG) {
      nbrDigiChans += 3;
   }
   //Digi Accel
   if(storedConfig[NV_SENSORS2] & SENSOR_MPU9150_ACCEL) {
      nbrDigiChans += 3;
   }
   //Digi Accel
   if(storedConfig[NV_SENSORS2] & SENSOR_MPU9150_MAG) {
      nbrDigiChans += 3;
   }

   block_len = (((nbrAdcChans+nbrDigiChans)*2)+2);

   if(storedConfig[NV_SENSORS2] & SENSOR_BMP180_PRESSURE) {
      nbrDigiChans += 2;//PRES & TEMP, ON/OFF together
      block_len += 5;
   }

   if(storedConfig[NV_SENSORS0] & SENSOR_EXG1_24BIT){
      nbrDigiChans += 2;
      block_len += 7;
   }
   if(storedConfig[NV_SENSORS2] & SENSOR_EXG1_16BIT){
      nbrDigiChans += 2;
      block_len += 5;
   }
   if(storedConfig[NV_SENSORS0] & SENSOR_EXG2_24BIT){
      nbrDigiChans += 2;
      block_len += 7;
   }
   if(storedConfig[NV_SENSORS2] & SENSOR_EXG2_16BIT){
      nbrDigiChans += 2;
      block_len += 5;
   }

   if(mask) {
      adcStartPtr = ADC_init(mask);
      DMA0_transferDoneFunction(&Dma0ConversionDone);
      if(adcStartPtr)
         DMA0_init(adcStartPtr, (uint16_t *)(txBuff0+4), nbrAdcChans);
   }
}


void ProcessCommand(void) {

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
}


// Switch SW1, BT_RTS and BT connect/disconnect
#pragma vector=PORT1_VECTOR
__interrupt void Port1_ISR(void)
{
   // Context save interrupt flag before calling interrupt vector.
   // Reading interrupt vector generator will automatically clear IFG flag
   switch (__even_in_range(P1IV, P1IV_P1IFG7)){
      //BT Connect/Disconnect
      case  P1IV_P1IFG0:
         if(P1IN & BIT0) {       //BT is connected
            P1IES |= BIT0;       //look for falling edge
            BT_connectionInterrupt(1);
            btIsConnected = 1;
            BT_rst_MessageProgress();
            if(sdhead_text[SDH_TRIAL_CONFIG0]&SDH_IAMMASTER){
               // center is waiting for 1 byte ROUTINE_COMMUNICATION(0xE0) from DMA2
               DMA2SZ = 1;
               DMA2_enable();
            }
            else
               // node sends (0xE0) and is waiting for 6 byte RC info from DMA2
               if(rcomm_status){
                  SendRcommCommand();
                  DMA2SZ = 6;
                  DMA2_enable();
               }

         } else {                //BT is disconnected
            P1IES &= ~BIT0;      //look for rising edge
            btIsConnected = 0;
            BT_connectionInterrupt(0);
         }
         break;

      //BT RTS
      case  P1IV_P1IFG3:
         if(P1IN & BIT3) {
            P1IES |= BIT3;       //look for falling edge
            BT_rtsInterrupt(1);
         } else {
            P1IES &= ~BIT3;      //look for rising edge
            BT_rtsInterrupt(0);  // when 0, can call sendNextChar();
         }
         break;

     //BUTTON_SW1
     case  P1IV_P1IFG6:
        Button_debounce();

        //pressing button toggles sensing
        if(sdhead_text[SDH_TRIAL_CONFIG0] & SDH_USER_BUTTON_ENABLE){
            // toggles sensing and refresh BT timers (for the centre)
            if(sensing){
               stopSensing = 1;
               if(sdhead_text[SDH_TRIAL_CONFIG0]&SDH_IAMMASTER){
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
               if(sdhead_text[SDH_TRIAL_CONFIG0]&SDH_IAMMASTER){
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
         break;

      //ExG chip2 data ready
      case  P1IV_P1IFG4:
         EXG_dataReadyChip2();
         break;
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
      case  P2IV_P2IFG0:
         EXG_dataReadyChip1();
         break;

      //EXP_DETECT_N
      case  P2IV_P2IFG1:
         //TODO: Debounce this
         //see slaa513 for example using multiple time bases on a single timer module
         if(P2IN & BIT1) {       //card not inserted
            P2IES |= BIT1;       //look for falling edge
         } else {                //card inserted
            P2IES &= ~BIT1;      //look for rising edge
         }
         break;

      //dock_detect_N
      case  P2IV_P2IFG3:
         //TODO: Debounce this
         //see slaa513 for example using multiple time bases on a single timer module
         if(P2IN & BIT3) {
            docked = 1;
            UART_setExgOrUart(1);
            UART_init();
            set_undock_start=0;
            set_undock_done=0;
            DockSdPowerCycle();
            P2IES |= BIT3;       //look for falling edge
         } else {
            docked = 0;
            UART_setExgOrUart(0);
            P2IES &= ~BIT3;      //look for rising edge
            CheckSdInslot();
         }
         break;
      // Default case
      default:
         break;
   }
}


// Timer2:
// ccr1: for blink timer
void CommTimerStart(void) {
   TA0CTL = TASSEL_1 + MC_2 + TACLR;         //ACLK, continuous mode, clear TAR
   TA0CCTL1 = CCIE;
   TA0CCR1 = 16384;
   RstRcommVariables();
}


inline void CommTimerStop(void) {
   TA0CTL = MC_0;
   rcomm_status=0;
   TA0CCTL1 &= ~CCIE;
}


inline uint16_t GetTA0(void) {
   register uint16_t t0, t1;
   uint8_t ie;
   if(ie=(__get_SR_register()&0x0008))       //interrupts enabled?
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
   case  0: break;                           // No interrupt
   case  2:                                  // TA0CCR1
      if(rcomm_status){
         if(!(sdhead_text[SDH_TRIAL_CONFIG0]&SDH_IAMMASTER)){
            // nodes:
            // for every rcomm_interval secs do:
            // 1) start+initialise BT RC_AHD secs before the ist RC command is sent
            // 2) send Routine Communication command (0xE0) every 4.5 secs
            // 3) if success, shut BT at once; if not, shut BT in RC_WINDOW_N secs.
            TA0CCR1 += RC_CLK_N;
            if(rcomm_cnt==1){                // give it 1 sec, rcomm_interval-RC_AHD
               rcomm_current_try = 0;
               BT_init();
               BtStart();
            }else if((rcomm_cnt>(RC_AHD*RC_FACTOR_N+1))
                  && (rcomm_cnt < (RC_WINDOW_N*RC_FACTOR_N + RC_AHD*RC_FACTOR_N + 1))){
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
            }else if(rcomm_cnt == RC_WINDOW_N*RC_FACTOR_N + RC_AHD*RC_FACTOR_N + 1){
               if(!rcomm_success){           // if no success in this communication, try earlier next time.
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
         }else{                              // i am center/slave
            // center:
            // for every rcomm_interval_c secs do:
            // 1) start+initialise BT
            // 2) wait rcomm_window_c secs and shut down BT
            // if connected by a node, disconnect within 0.5s and wait for another
            TA0CCR1 += RC_CLK_C;
            //==================== below is bt timeout function ====================
            if(btIsConnected){
               rcomm_timeout ++;
            }
            else
               rcomm_timeout = 0;
            if(rcomm_timeout ==2){           //rcomm_timeout
               if(btIsConnected){
                  BT_disconnect();
               }
               rcomm_timeout = 0;
            }
            //==================== above is bt timeout function ====================

            if(rcomm_cnt==1){                // bt start
               if(!bt_status){
                  BT_init();
                  BT_disableRemoteConfig(1);
                  BtStart();
               }
            }
            else if(rcomm_cnt==rcomm_window_c*RC_FACTOR_C){    // bt stop
               if(bt_status){
                  if(!rcomm_special_int)
                     rcomm_interval_c = storedConfig[NV_BT_INTERVAL];
               }
               else{
                  rcomm_interval_c = rcomm_window_c+RC_WINDOW_C;
               }
               BtStop();
            }
            else if(rcomm_cnt>=rcomm_interval_c*RC_FACTOR_C){  // reaching counter_max, and return to normal iteration
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
            // when docked, shut every thing and return to no_RC mode
            BtStop();
            if(!(set_undock + btIsConnected + bt_status + processCommand +
               parse_rcomm_resp + stopSensing + startSensing + streamData + sd_write
               )){
               on_user_button = 0;
               on_single_touch = 0;
               on_default = 0;
               rcomm_status = 0;
               rcomm_cnt = 0;
               stopSensing = 1;
               __bic_SR_register_on_exit(LPM3_bits);
            }
         }
      }
      else                                   //idle: no_RC mode
      {
         TA0CCR1 += RC_CLK_C;
         if(!docked){
            // when undocked, read config file and enter RC mode
            if(!set_undock_done){
               if(!set_undock_start){
                  set_undock = 1;
                  __bic_SR_register_on_exit(LPM3_bits);
               }
            }
            else{
               if(sdhead_text[SDH_TRIAL_CONFIG0]&SDH_TIME_SYNC){
                  if(on_single_touch || (on_user_button &&sensing)||on_default){
                     rcomm_status=1;
                     RstRcommVariables();
                  }
               }
            }
         }else{
            // when docked, stay quiet and shut sensors if they are on
            if(sensing){
               stopSensing = 1;
               __bic_SR_register_on_exit(LPM3_bits);
            }
         }
      }
      break;
   case  4: break;                           // TA0CCR2 not used
   case  6: break;                           // Reserved
   case  8: break;                           // Reserved
   case 10: break;                           // Reserved
   case 12: break;                           // Reserved
   case 14: break;                           // TAIFG overflow handler
   }
}


void SendRcommCommand(){
   memset(rcomm_resp,0,RCT_SIZE);
   get_rcomm=1;
   BT_write(&rcomm_command, 0x01);
}


void ParseRcommResponse(){
   // only nodes do this
   if(rcomm_resp[RCT_ACK] == ACK_COMMAND_PROCESSED){           //if received the correct 6 bytes:
      uint32_t my_local_time_long, my_center_time_long, my_time_diff_long;
      uint8_t sd_tolog;
      my_local_time_long = GetRTC();                           // get my_local_time_long
      rcomm_success = 1;

      sd_tolog = rcomm_resp[RCT_FLG];
      my_center_time_long = *(uint32_t*)(rcomm_resp+RCT_TIME); // get my_center_time_long
      //check sensing status flag, and follow
      if(on_single_touch){
         if(sensing){
            if(!sd_tolog)
               stopSensing = 1;
         }
         else{
            if(sd_tolog)
               startSensing = 1;
         }
      }

      //calc time difference and save into my_time_diff[5]
      //which will be written into every SD card buffer
      if(my_local_time_long>my_center_time_long){
         my_time_diff[0] = 0;
         my_time_diff_long =
               (uint32_t)fmod(4294967296+(uint64_t)my_local_time_long - (uint64_t)my_center_time_long,4294967296);
      }else{
         my_time_diff[0] = 1;
         my_time_diff_long =
               (uint32_t)fmod(4294967296+(uint64_t)my_center_time_long - (uint64_t)my_local_time_long,4294967296);
      }
      memcpy(my_time_diff+1,(uint8_t*)&my_time_diff_long,4);

   }
}


// Blink Timer
// USING TB0 with CCR1
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
   case  0: break;                              // No interrupt
    case  2:                                    // TB0CCR1
       //MPU9150 mag
       TB0CCR1 += *(uint16_t *)(storedConfig+NV_SAMPLING_RATE);
       sampleMpu9150Mag = 1;
       __bic_SR_register_on_exit(LPM3_bits);
       break;
    case  4:                                    // TB0CCR2
       //Bmp180 press
       TB0CCR2 += *(uint16_t *)(storedConfig+NV_SAMPLING_RATE);
       sampleBmp180Press = 1;
       __bic_SR_register_on_exit(LPM3_bits);
       break;
    case  6:                                    // TB0CCR3
       // for LED blink usage details, please check shimmer user manual
      TB0CCR3 += clk_1000;
      if(blink_cnt_5++==49)
         blink_cnt_5 = 0;

      if(blink_cnt_2++==19)
         blink_cnt_2 = 0;

      uint32_t my_local_time_long, batt_td;
      my_local_time_long = GetRTC();
      batt_td = (uint32_t)fmod(4294967296+(uint64_t)my_local_time_long - (uint64_t)batt_last_time,4294967296);
      if(batt_td > BATT_INTERVAL){              //10 mins = 19660800
         batt_read=1;
         batt_last_time = my_local_time_long;
      }

      if(!initializing)
         if(batt_read)
            __bic_SR_register_on_exit(LPM3_bits);

       if(blink_status){
          // below are settings for green0, yellow and red leds, battery charge status
          if(docked){
             BattBlinkOn();
          }else{
             if(!blink_cnt_5)
                BattBlinkOn();
             else
               Board_ledOff(LED_GREEN0+LED_YELLOW+LED_RED);
          }

         // below are settings for green1 and blue leds
         if(file_bad && !docked){               // bad file = green1/blue alternating
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
            // good file - green1:
            if(!sensing){                       //standby or configuring
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
            }else{                              //sensing
               if(!(blink_cnt_2%10)){
                  if(!(P1OUT & BIT1))
                     Board_ledOn(LED_GREEN1);
                  else
                     Board_ledOff(LED_GREEN1);
               }
            }
            // good file - blue:
            if(bt_status){
               Board_ledOn(LED_BLUE);
            }
            else{
               Board_ledOff(LED_BLUE);
            }
         }
       }
       break;
    case  8: break;                          // TB0CCR4
    case 10: break;                          // reserved
    case 12: break;                          // reserved
    case 14: break;                          // TBIFG overflow handler
   }
}


// BT start
void BtStartDone(){
   bt_status = 1;
}


void BtStart(){
   if(!(sdhead_text[SDH_TRIAL_CONFIG0]&SDH_IAMMASTER))
      BT_setRadioMode(MASTER_MODE); //master mode for nodes

   else
      BT_setRadioMode(SLAVE_MODE);  // slave mode for center
   BT_start();
   BT_startDone_cb(BtStartDone);
}


void BtStop(){
   bt_bad = 0;                      //reset bt status, don't report bt problem
   get_rcomm = 0;                   //don't try to get routine comm info
   DMA2_disable();                  //dma2 for bt disabled
   btIsConnected = 0;               //set connect status to false
   BT_connectionInterrupt(0);
   bt_status = 0;                   //set bt status to off
   P1IES &= ~BIT2;                  //look for rising edge
   BT_disable();                    //set bt disable, stop starting progress
   BT_rst_MessageProgress();        //reset message progress vars to 0
}


// Sample Timer
void SampleTimerStart(void) {
   uint16_t val_tb0;
   val_tb0 = GetTB0();
   if(preSampleMpuMag || preSampleBmpPress) {
      if(preSampleBmpPress && (((storedConfig[NV_CONFIG_SETUP_BYTE3]&0x30)>>4)==0)) {
         if(preSampleMpuMag) {
            TB0CCR0 = val_tb0 + *(uint16_t *)(storedConfig+NV_SAMPLING_RATE) + clk_90;
            TB0CCR1 = val_tb0 + *(uint16_t *)(storedConfig+NV_SAMPLING_RATE);
            TB0CCR2 = val_tb0 + *(uint16_t *)(storedConfig+NV_SAMPLING_RATE) + clk_90_45;
            TB0CCTL1 = CCIE;
         } else {
            TB0CCR0 = val_tb0 + *(uint16_t *)(storedConfig+NV_SAMPLING_RATE) + clk_45;
            TB0CCR2 = val_tb0 + *(uint16_t *)(storedConfig+NV_SAMPLING_RATE);
            TB0CCTL1 = 0;
         }
         TB0CCTL2 = CCIE;
      } else if(preSampleBmpPress && (((storedConfig[NV_CONFIG_SETUP_BYTE3]&0x30)>>4)==1)) {
         if(preSampleMpuMag) {
            TB0CCR0 = val_tb0 + *(uint16_t *)(storedConfig+NV_SAMPLING_RATE) + clk_90;
            TB0CCR1 = val_tb0 + *(uint16_t *)(storedConfig+NV_SAMPLING_RATE);
            //2302-1919
            TB0CCR2 = val_tb0 + *(uint16_t *)(storedConfig+NV_SAMPLING_RATE) + clk_90_75;
            TB0CCTL1 = CCIE;
         } else {
            TB0CCR0 = val_tb0 + *(uint16_t *)(storedConfig+NV_SAMPLING_RATE) + clk_75;
            TB0CCR2 = val_tb0 + *(uint16_t *)(storedConfig+NV_SAMPLING_RATE);
            TB0CCTL1 = 0;
         }
         TB0CCTL2 = CCIE;
      } else if(preSampleBmpPress && (((storedConfig[NV_CONFIG_SETUP_BYTE3]&0x30)>>4)==2)) {
         TB0CCR0 = val_tb0 + *(uint16_t *)(storedConfig+NV_SAMPLING_RATE) + clk_135;
         TB0CCR2 = val_tb0 + *(uint16_t *)(storedConfig+NV_SAMPLING_RATE);
         TB0CCTL2 = CCIE;
         if(preSampleMpuMag) {
            TB0CCR1 = val_tb0 + *(uint16_t *)(storedConfig+NV_SAMPLING_RATE) + clk_135_90;
            TB0CCTL1 = CCIE;
         } else TB0CCTL1 = 0;
      } else if(preSampleBmpPress && (((storedConfig[NV_CONFIG_SETUP_BYTE3]&0x30)>>4)==3)) {
         TB0CCR0 = val_tb0 + *(uint16_t *)(storedConfig+NV_SAMPLING_RATE) + clk_250;
         TB0CCR2 = val_tb0 + *(uint16_t *)(storedConfig+NV_SAMPLING_RATE);
         TB0CCTL2 = CCIE;
         if(preSampleMpuMag) {
            TB0CCR1 = val_tb0 + *(uint16_t *)(storedConfig+NV_SAMPLING_RATE) + clk_250_90;
            TB0CCTL1 = CCIE;
         } else TB0CCTL1 = 0;
      } else {
         TB0CCR0 = val_tb0 + *(uint16_t *)(storedConfig+NV_SAMPLING_RATE) + clk_90;
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
   TB0CCR0 += *(uint16_t *)(storedConfig+NV_SAMPLING_RATE);
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

      if(batt_stat & BATT_MID){
         if(*(uint16_t*)batt_val<2400){
            batt_stat = BATT_LOW;
         }else if(*(uint16_t*)batt_val<2600){
            batt_stat = BATT_MID;
         }else
            batt_stat = BATT_HIGH;
      }else if(batt_stat & BATT_LOW){
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
      if(currentBuffer){
         //Destination address for next transfer
         DMA0_repeatTransfer(adcStartPtr, (uint16_t *)(txBuff0+adc_offset), nbrAdcChans);
      } else {
         //Destination address for next transfer
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
      if(get_rcomm){
         // 6 bytes of RC info
         memcpy(rcomm_resp, btrx_buff, 6);
         memset(btrx_buff,0,14);
         parse_rcomm_resp = 1;
         get_rcomm = 0;
         return 1;
      }else if(bt_getmac){
         // 14 bytes of BT mac address
         memcpy(mac, btrx_buff, 14);
         memset(btrx_buff,0,14);
         BT_setGetMacAddress(0);
         BT_setGoodCommand();
      }else if(btrx_buff[0] == ROUTINE_COMMUNICATION){
         // 1 byte of RC command
         memset(btrx_buff,0,14);
         processCommand = 1;
         return 1;
      }
   } else {
      if(!memcmp(btrx_buff, btrx_exp, strlen((char*)btrx_exp))) {
         memset(btrx_buff,0,14);
         BT_setGoodCommand();
      }else{
         // bad command trap: reaching here = serious BT problem
         // restart BT helps resolving the problem
         _NOP();
      }
   }
   return 0;
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

   my_local_time_long = GetRTC();                           // calculate time difference for file use.
   f_lseek (&fil, fil.fsize);                               // seek to end of file, no spi op
   rc = f_write(&fil, text, size, &bw);                     // Write to file

   file_td_h = (uint32_t)fmod(4294967296+(uint64_t)my_local_time_long - (uint64_t)last_hour,4294967296);
   file_td_m = (uint32_t)fmod(4294967296+(uint64_t)my_local_time_long - (uint64_t)last_min,4294967296);

   //create a new file (from 000 up) every 1h
   if(file_td_h >= 117964800){                              // 117964800 = 32768/s*3600s = 1h
      last_hour = my_local_time_long;
      rc = f_close(&fil);

      file_num++;                                           //file number:from 000 up

      filename[len_dir+3]=(char)(((int)'0')+file_num%10);
      filename[len_dir+2]=(char)(((int)'0')+(file_num/10)%10);
      filename[len_dir+1]=(char)(((int)'0')+(file_num/100)%10);

      f_open(&fil, (char*)filename, FA_WRITE | FA_CREATE_NEW);

      memcpy(&sdhead_text[SDH_MY_LOCALTIME], (uint8_t*)&my_local_time_long, 4);
      last_min = my_local_time_long;

      f_write(&fil, sdhead_text, SDHEAD_LEN, &bw);          // Write head to file
   }
   // sync data to SD card every 1 min
   else if(file_td_m >= 1966080){                           //32768/s*60s = 1m
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


void SetupRTC(void)
{
    RTCCTL01 = RTCHOLD + RTCTEV_1;     //hold
    RTCTIM0 = 0;
    RTCTIM1 = 0;
    RTCCTL01 &= ~RTCHOLD;              //start
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
   bool user_button_enable = FALSE;
   uint8_t my_trial_id=0, num_shimmers_in_trial=0, my_config_time[4];
   uint32_t config_time = 0, my_local_time_long;
   bool iAmMaster = FALSE, time_sync = FALSE, singletouch = FALSE, tcxo=FALSE;

   broadcast_interval = RC_INT_C;

   storedConfig[NV_CONFIG_SETUP_BYTE3] |= (4 & 0x07) <<1;      //BIT3-1
   storedConfig[NV_TRIAL_CONFIG0] &= ~SDH_SET_PMUX;            // PMUX reserved as 0
   storedConfig[NV_TRIAL_CONFIG0] |= SDH_TIME_STAMP;           // TIME_STAMP always = 1

   *shimmername='\0';
   *exp_id_name='\0';
   *centername='\0';
   *config_time_text='\0';
   clock_freq = MSP430_CLOCK;

   dir_new = 1;

   CheckSdInslot();

   char cfgname[]="sdlog.cfg";
   file_bad = f_open(&fil, cfgname, FA_READ | FA_OPEN_EXISTING);

   while(f_gets(buffer, 64, &fil)){
      if(!(equals = strchr(buffer, '=')))
         continue;
      equals++;   // this is the value
      if(strstr(buffer, "accel="))  //a_accel on/off
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
         storedConfig[NV_CONFIG_SETUP_BYTE0] |= (accel_lpm&0x01)<<1;       //BIT1
         storedConfig[NV_TRIAL_CONFIG1] |= (accel_lpm&0x01)*SDH_ACCEL_LPM;
      }
      else if(strstr(buffer, "acc_hrm=")){
         accel_hrm = atoi(equals);
         storedConfig[NV_CONFIG_SETUP_BYTE0]|= accel_hrm&0x01;             //BIT0
         storedConfig[NV_TRIAL_CONFIG1] |= (accel_hrm&0x01)*SDH_ACCEL_HRM;
      }
      else if(strstr(buffer, "gs_range=")){
         gsr_range = atoi(equals);
         if(gsr_range<=4)
            storedConfig[NV_CONFIG_SETUP_BYTE3] |= (gsr_range& 0x07) <<1;  //BIT3-1
         else
            storedConfig[NV_CONFIG_SETUP_BYTE3] |= (4 & 0x07) <<1;         //BIT3-1
      }
      else if(strstr(buffer, "gyro_samplingrate="))
         storedConfig[NV_CONFIG_SETUP_BYTE1] = atoi(equals);
      else if(strstr(buffer, "gyro_range=")){
         gyro_range = atoi(equals);
         storedConfig[NV_CONFIG_SETUP_BYTE2]|= (gyro_range)&0x03;          //BIT1-0
      }
      else if(strstr(buffer, "pres_bmp180_prec=")){
         pres_bmp180_prec = atoi(equals);
         storedConfig[NV_CONFIG_SETUP_BYTE3]|= (pres_bmp180_prec& 0x03) <<4;//BIT5-4
      }
      else if(strstr(buffer, "user_button_enable=")){
         user_button_enable = (atoi(equals)==0)?FALSE:TRUE;
         storedConfig[NV_TRIAL_CONFIG0] |= user_button_enable*SDH_USER_BUTTON_ENABLE;
      }
      else if(strstr(buffer, "iammaster=")){//0=node
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
         storedConfig[NV_CONFIG_SETUP_BYTE3] |= (exp_power& 0x01)*EXP_POWER_ENABLE;
      }
      else if(strstr(buffer, "tcxo=")){
         tcxo = (atoi(equals)==0)?FALSE:TRUE;
         storedConfig[NV_TRIAL_CONFIG1] |= tcxo*SDH_TCXO;
         if(tcxo)
            clock_freq = TCXO_CLOCK;
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

   sample_period = FreqDiv(sample_rate);
   // little endian:
   storedConfig[NV_SAMPLING_RATE] = (uint8_t)(sample_period & 0xFF);
   storedConfig[NV_SAMPLING_RATE + 1] = (uint8_t)(sample_period >> 8 );

   if(storedConfig[NV_SENSORS0] & SENSOR_GSR)                  // they are sharing adc1, ban intch1 when gsr is on
      storedConfig[NV_SENSORS1] &= ~SENSOR_INT_A1;

   if(storedConfig[NV_SENSORS0] & SENSOR_EXG1_24BIT)
      storedConfig[NV_SENSORS2] &= ~SENSOR_EXG1_16BIT;
   if(storedConfig[NV_SENSORS0] & SENSOR_EXG2_24BIT)
      storedConfig[NV_SENSORS2] &= ~SENSOR_EXG2_16BIT;
   if(storedConfig[NV_SENSORS0] & SENSOR_EXG1_24BIT || storedConfig[NV_SENSORS0] & SENSOR_EXG2_24BIT ||
      storedConfig[NV_SENSORS2] & SENSOR_EXG1_16BIT || storedConfig[NV_SENSORS2] & SENSOR_EXG2_16BIT){
      storedConfig[NV_CONFIG_SETUP_BYTE3] |= EXP_POWER_ENABLE;
      storedConfig[NV_SENSORS1] &= ~SENSOR_INT_A1;
      storedConfig[NV_SENSORS2] &= ~SENSOR_INT_A14;
   }

   if(storedConfig[NV_SENSORS1] & SENSOR_STRAIN){              // they are sharing adc13 and adc14
      storedConfig[NV_SENSORS1] &= ~SENSOR_INT_A13;
      storedConfig[NV_SENSORS2] &= ~SENSOR_INT_A14;
   }

   if(((storedConfig[NV_CONFIG_SETUP_BYTE3]>>1)&0x07)>4)       // never larger than 4
      storedConfig[NV_CONFIG_SETUP_BYTE3] |= (4 & 0x07) <<1;   //BIT3-1

   if(clock_freq == TCXO_CLOCK){
      P4OUT |= BIT6;
      P4SEL |= BIT7;
   }
   else{
      P4OUT &= ~BIT6;
       P4SEL &= ~BIT7;
   }
   ClkAssignment();
   TB0CTL = MC_0;
   TB0Start();

   // minimum sync broadcast interval is 54 seconds
   if(broadcast_interval < RC_INT_C){
      broadcast_interval = RC_INT_C;
   }

   // the button always works for singletouch mode
   // sync always works for singletouch mode
   if(storedConfig[NV_TRIAL_CONFIG1]&SDH_SINGLETOUCH){
      storedConfig[NV_TRIAL_CONFIG0] |= SDH_USER_BUTTON_ENABLE;
      storedConfig[NV_TRIAL_CONFIG0] |= SDH_TIME_SYNC;
   }

   if(strlen((char*)centername)==0) // if no center is appointed, let this guy be the center
      strcpy((char*)centername,"00066643b48e");

   storedConfig[NV_BT_INTERVAL]= broadcast_interval;

   Config2SdHead();
   memcpy(&sdhead_text[SDH_CONFIG_TIME_0], my_config_time, 4);
   my_local_time_long = GetRTC();
   memcpy(&sdhead_text[SDH_MY_LOCALTIME], (uint8_t*)&my_local_time_long, 4);
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
   // little endian in fw, but they want big endian in sw
   sdhead_text[SDH_TRIAL_CONFIG0] = storedConfig[NV_TRIAL_CONFIG0];
   sdhead_text[SDH_TRIAL_CONFIG1] = storedConfig[NV_TRIAL_CONFIG1];
   sdhead_text[SDH_BROADCAST_INTERVAL] = storedConfig[NV_BT_INTERVAL];

   // trivial
   sdhead_text[SDH_SHIMMERVERSION_BYTE_1] = DEVICE_VER;
   sdhead_text[SDH_FW_VERSION_TYPE_1] = FW_IDENTIFIER;
   sdhead_text[SDH_FW_VERSION_MAJOR_1] = FW_VER_MAJOR;
   sdhead_text[SDH_FW_VERSION_MINOR] = FW_VER_MINOR;
   sdhead_text[SDH_FW_VERSION_INTERNAL] = FW_VER_INTERNAL;

   // exg
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


error_t SetBasedir() {
   FILINFO fno;
   error_t res;
   uint16_t tmp_counter = 0;
   char lfn[_MAX_LFN + 1], * fname, * scout, * dash, dirnum[8];

   // first we'll make the shimmer mac address into a string
   if(strlen((char*)shimmername)==0){
      // if name hasn't been assigned by user, use default (from Shimmer mac address)
      strcpy((char*)shimmername, "IDXXXX");
      memcpy(shimmername+2, mac+8, 4);
   }

   if(strlen((char*)exp_id_name)==0)      // if hasn't been assigned by user, use default
      strcpy((char*)exp_id_name, "default_exp");

   if(strlen((char*)config_time_text)==0) // if hasn't been assigned by user, use default
      strcpy((char*)config_time_text, "NoCfgTime");

   fno.lfname = lfn;
   fno.lfsize = sizeof(lfn);

   if((res = f_opendir(&dir, "/data"))){
      if(res == FR_NO_PATH)               // we'll have to make /data first
         res = f_mkdir("/data");
      if(res)                             // in every case, we're toast
         return FAIL;

      // try one more time
      if((res = f_opendir(&dir, "/data")))
         return FAIL;
   }

   strcpy((char*)exp_dir_name, "data/");
   strcat((char*)exp_dir_name, (char*)exp_id_name);
   strcat((char*)exp_dir_name, "_");
   strcat((char*)exp_dir_name, (char*)config_time_text);

   if((res = f_opendir(&dir, (char*)exp_dir_name))){
      if(res == FR_NO_PATH)               // we'll have to make the experiment folder first
         res = f_mkdir((char*)exp_dir_name);
      if(res)                             // in every case, we're toast
         return FAIL;

      // try one more time
      if((res = f_opendir(&dir, (char*)exp_dir_name)))
         return FAIL;
   }

   dir_counter = 0;                       // this might be the first log for this shimmer


   // file name format
   // shimmername    as defined in sdlog.cfg
   // -              separator
   // 000
   // we want to create a new directory with a sequential run number each power-up/reset for each shimmer
   while(f_readdir(&dir, &fno) == FR_OK){
      if(*fno.fname == 0)
      break;
      else if(fno.fattrib & AM_DIR){
         fname = (*fno.lfname) ? fno.lfname : fno.fname;

         if(!strncmp(fname, (char*)shimmername, strlen(fname)-4)){ // -4 because of the -000 etc.
            if((scout = strchr(fname, '-'))){               // if not, something is seriously wrong!
               scout++;
               while((dash = strchr(scout, '-')))           // In case the shimmer name contains '-'
                  scout = dash + 1;
               strcpy(dirnum, scout);
               tmp_counter = atoi(dirnum);
               if(tmp_counter >= dir_counter){
                  dir_counter = tmp_counter;
                  dir_counter++;                            // start with next in numerical sequence
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
   memcpy(&sdhead_text[SDH_LSM303DLHC_MAG_CALIBRATION], &storedConfig[NV_LSM303DLHC_MAG_CALIBRATION], 21);
   memcpy(&sdhead_text[SDH_MPU9150_GYRO_CALIBRATION], &storedConfig[NV_MPU9150_GYRO_CALIBRATION], 21);
   memcpy(&sdhead_text[SDH_A_ACCEL_CALIBRATION], &storedConfig[NV_A_ACCEL_CALIBRATION], 21);
}


void ReadCalibration(uint8_t sensor){
   char buffer[66], * equals, keyword[20];
   uint8_t i = 0, num_2byte_params = 6, num_1byte_params = 9, cal_file[32];
   uint16_t address;
   bool sensor_found = FALSE;
   float value;
   int8_t rounded_value;
   //FRESULT res;

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
   else if(f_open(&gfc, (char*)cal_file, (FA_OPEN_EXISTING | FA_READ)))
      DefaultCalibration(sensor);                           // no calibration file, use default
   else{ // look for sensor in calibration file.
      while(f_gets(buffer, 64, &gfc)){
         if(!strstr(buffer, keyword))
            continue;
         else{ // found the right sensor
            sensor_found = TRUE;
            for(i = 0; i < num_2byte_params; i++){
               f_gets(buffer, 64, &gfc);
               if(!(equals = strchr(buffer, '='))){
                  sensor_found = FALSE;                     // there's an error, use the default
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
                  sensor_found = FALSE;                     // there's an error, use the default
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

   // offset
   for(i = 0; i < number_axes; i++){
      storedConfig[address + 2*i] = bias_byte_one;
      storedConfig[address + 2*i + 1] = bias_byte_two;
   }
   // sensitivity
   for(i = 0; i < number_axes; i++){
      storedConfig[address + 2*(number_axes + i)] = sens_byte_one;
      storedConfig[address + 2*(number_axes + i) + 1] = sens_byte_two;
   }
   // alignment
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
   // Fill the current active resistor into the upper two bits of the GSR value
   // if autorange is enabled, switch active resistor if required
   // If during resistor transition period use old ADC and resistor values
   // as determined by GSR_smoothTransition()

   uint8_t current_active_resistor = gsr_active_resistor;
   uint16_t ADC_val;

   // GSR channel will always be last ADC channel
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


void CheckSdInslot(){
   //Check if card is inserted and enable interrupt for SD_DETECT_N
   if(!(P4IN & BIT1)) {
      f_mount(0, &fatfs);
      set_sd_detect(1);
   }else{
      f_mount(0,NULL);
      set_sd_detect(0);
   }
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
           EXG_readData(0, 0, txBuff1+digi_offset);
           digi_offset+=7;
        } else if(storedConfig[NV_SENSORS2] & SENSOR_EXG1_16BIT) {
           EXG_readData(0, 1, txBuff1+digi_offset);
           digi_offset+=5;
        }
        if(storedConfig[NV_SENSORS0] & SENSOR_EXG2_24BIT) {
           EXG_readData(1, 0, txBuff1+digi_offset);
           digi_offset+=7;
        } else if(storedConfig[NV_SENSORS2] & SENSOR_EXG2_16BIT) {
           EXG_readData(1, 1, txBuff1+digi_offset);
           digi_offset+=5;
        }
        if(clock_freq == MSP430_CLOCK)
           memcpy(sdbuff+sdbuff_len, txBuff1+2, block_len);
        else
           memcpy(sdbuff+sdbuff_len, txBuff1+4, block_len);

      sdbuff_len+=block_len;

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
           EXG_readData(0, 0, txBuff0+digi_offset);
           digi_offset+=7;
        } else if(storedConfig[NV_SENSORS2] & SENSOR_EXG1_16BIT) {
           EXG_readData(0, 1, txBuff0+digi_offset);
           digi_offset+=5;
        }
        if(storedConfig[NV_SENSORS0] & SENSOR_EXG2_24BIT) {
           EXG_readData(1, 0, txBuff0+digi_offset);
           digi_offset+=7;
        } else if(storedConfig[NV_SENSORS2] & SENSOR_EXG2_16BIT) {
           EXG_readData(1, 1, txBuff0+digi_offset);
           digi_offset+=5;
        }
        if(clock_freq == MSP430_CLOCK)
           memcpy(sdbuff+sdbuff_len, txBuff0+2, block_len);
        else
           memcpy(sdbuff+sdbuff_len, txBuff0+4, block_len);

      sdbuff_len+=block_len;

      currentBuffer = 1;
   }
   _NOP();
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
   if(sdhead_text[SDH_TRIAL_CONFIG0]&SDH_TIME_SYNC)
      PrepareSDBuffHead();

}


void BattBlinkOn(){
   Board_ledOff(LED_GREEN0+LED_YELLOW+LED_RED);
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
   ADC12CTL0 &= ~ADC12ENC;                                  //ensure is off so all ADC12CTL0
                                                            //and ADC12CTL1 fields can be modified
   ADC12CTL0 = ADC12SHT1_8 + ADC12SHT0_8 + ADC12MSC;        //SHT1 and SHT0 of 256 ADC12CLK cycles,
                                                            //enable multiple sample and conversion
   ADC12CTL1 = ADC12SHP + ADC12CONSEQ_1;                    //use sampling timer, ADC12OSC
                                                            //and sequence-of-channels mode
   ADC12CTL2 = ADC12TCOFF + ADC12RES_2 + ADC12SR_L;         //turn off temperature sensor (saves power),
                                                            //ADC12 resolution to 12bits,
   ADC12CTL1 += ADC12CSTARTADD_4;                           //Start with ADC12MEM4
   ADC12MCTL4 = ADC12INCH_2 + ADC12EOS;

   DMACTL0 |= DMA0TSEL_24;                                  //ADC12IFGx triggered
   DMACTL4 = DMARMWDIS;                                     //Read-modify-write disable
   DMA0CTL &= ~DMAIFG;
   DMA0CTL = DMADT_1+DMADSTINCR_3+DMASRCINCR_3+DMAIE;       //block transfer, inc dst, inc src, Int enable
   DMA0SZ = 1;                                              //DMA0 size

   //Writes a value to a 20-bit SFR register located at the given16/20-bit address
   //__data16_write_addr((unsigned long) &DMA0SA,(unsigned long)&ADC12MEM4);              //Source block address
   //__data16_write_addr((unsigned short) &DMA0DA,(unsigned long)batt_val);               //Destination single address
   DMA0SA = (__SFR_FARPTR) (unsigned long) &ADC12MEM4;
   DMA0DA = (__SFR_FARPTR) (unsigned long) batt_val;
   DMA0_transferDoneFunction(&Dma0ConversionDone);

   ADC12IFG = 0;
   DMA0CTL |= DMAEN;
   ADC12CTL0 |= ADC12ON + ADC12ENC + ADC12SC;
}


void TB0Start(){
   if(sample_timer_status + blink_status == 1){
      if(clock_freq == MSP430_CLOCK){
         TB0CTL = TBSSEL_1 + MC_2;                          //ACLK, continuous mode, clear TBR: + TBCLR
         TB0EX0 = 0;
      }else{
         TB0CTL = TBSSEL_0 + MC_2 + ID__8;                  // use TBSSEL_0 for tcxo, ID__8:divider=8
         TB0EX0 = TBIDEX__8;                                // divider: 5
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
//USCI_B0_VECTOR:i2c
//USCI_A0_VECTOR:dock/exp_uart
//USCI_A1_VECTOR:bt_uart
#pragma vector = WDT_VECTOR, RTC_VECTOR, SYSNMI_VECTOR, TIMER0_A0_VECTOR, \
    UNMI_VECTOR, USCI_B1_VECTOR,TIMER1_A1_VECTOR
__interrupt void TrapIsr(void){
  // this is a trap ISR - check for the interrupt cause here by
  // checking the interrupt flags, if necessary also clear the interrupt
  // flag
}
