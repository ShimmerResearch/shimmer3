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
 * @author Mike Healy
 * @date December, 2013
 */

/***********************************************************************************

   Data Packet Format:
          Packet Type | TimeStamp | chan1 | chan2 | ... | chanX
   Byte:       0      |    1-2    |  3-4  |  5-6  | ... | chanX

   Inquiry Response Packet Format:
          Packet Type | ADC Sampling rate | Config Bytes | Num Chans | Buf size | Chan1 | Chan2 | ... | ChanX
   Byte:       0      |        1-2        |      3-6     |     7     |     8    |   9   |   10  | ... |   x


   ExG registers GET Packet Format:
          Packet Type | Chip | Start Address | Length
   Byte:       0      |   1  |       2       |    3

   ExG registers Response Packet Format:
          Packet Type | Length | Data
   Byte:       0      |    1   | 2-(1+Length)

   ExG registers SET Packet Format:
          Packet Type | Chip | Start Address | Length | Data
   Byte:       0      |   1  |       2       |    3   | 4 - (3+Length)

   Max "Length" value is 10


   Daughter card  Memory Read Packet Format:
          Packet Type | Length | Offset
   Byte:       0      |    1   |  2-3

   Daughter card Memory Read Response Packet Format:
          Packet Type | Length | Data
   Byte:       0      |    1   | 2-(1+Length)

   Daughter card Memory Write Packet Format:
          Packet Type | Length | Offset | Data
   Byte:       0      |    1   |   2-3  | 4-(3+Length)

   Max "Length" value of a single read/write is 128 bytes for daughter card memory
   Total available daughter card memory is 2032bytes

   The packet format for the daughter card ID commands is similar, only difference being
   that the Offset value is only 1 byte in length
   Max "Length" value of a single read/write is 16 bytes for daughter card ID
   Total available daughter card ID space is 16 bytes


***********************************************************************************/

//TODO: figure out why first sample after starting streaming is sometimes incorrect when using DMA with the ADC

#include <stdint.h>
#include "msp430.h"
#include "shimmer.h"
#include "hal_pmm.h"
#include "hal_UCS.h"
#include "hal_Board.h"
#include "hal_Button.h"
#include "hal_I2C.h"
#include "hal_ADC.h"
#include "hal_DMA.h"
#include "hal_InfoMem.h"
#include "RN42.h"
#include "lsm303dlhc.h"
#include "string.h"
#include "bmp180.h"
#include "mpu9150.h"
#include "gsr.h"
#include "exg.h"
#include "cat24c16.h"

void Init(void);
void BlinkTimerStart(void);
inline void BlinkTimerStop(void);
void SampleTimerStart(void);
inline void SampleTimerStop(void);
uint8_t BtDataAvailable(uint8_t data);
uint8_t Dma0ConversionDone(void);
void ProcessCommand(void);
void SendResponse(void);
void ConfigureChannels();
inline void StartStreaming(void);
inline void StopStreaming(void);
void SetDefaultConfiguration(void);
inline void StartTB0(void);
inline void ResetTB0(void);
inline void StopTB0(void);
inline uint16_t GetTB0(void);
void ChargeStatusTimerStart(void);
void ChargeStatusTimerStop(void);
inline void SetLed(void);
inline void ClearLed(void);
inline uint8_t IsLedSet(void);
inline void MonitorChargeStatus(void);
inline void MonitorChargeStatusStop(void);
inline void SetChargeStatusLeds(void);
inline void GsrRange(void);
void SetBtBaudRate(uint8_t rate);

//data segment initialisation is disabled in system_pre_init.c
uint8_t currentBuffer, processCommand, sendAck, inquiryResponse, samplingRateResponse, startSensing, stopSensing,
      aAccelCalibrationResponse, gyroCalibrationResponse, magCalibrationResponse, dAccelCalibrationResponse,
      allCalibrationResponse, configureChannels, streamData, sendResponse, sensing, btIsConnected,
      deviceVersionResponse, fwVersionResponse, bufferSizeResponse, uniqueSerialResponse, configSetupBytesResponse,
      lsm303dlhcAccelRangeResponse, lsm303dlhcMagGainResponse, lsm303dlhcMagSamplingRateResponse,
      lsm303dlhcAccelSamplingRateResponse, lsm303dlhcAccelLPModeResponse, lsm303dlhcAccelHRModeResponse,
      mpu9150GyroRangeResponse, mpu9150SamplingRateResponse, mpu9150AccelRangeResponse, sampleMpu9150Mag,
      mpu9150MagSensAdjValsResponse, sampleBmp180Press, bmp180CalibrationCoefficientsResponse,
      bmp180OversamplingRatioResponse, blinkLedResponse, gsrRangeResponse, internalExpPowerEnableResponse,
      exgRegsResponse, dcIdResponse, dcMemResponse, btCommsBaudRateResponse;
uint8_t gAction;
uint8_t args[MAX_COMMAND_ARG_SIZE], waitingForArgs, argsSize;
uint8_t resPacket[RESPONSE_PACKET_SIZE];
uint8_t storedConfig[NV_TOTAL_NUM_CONFIG_BYTES], channelContents[MAX_NUM_CHANNELS];
uint8_t nbrAdcChans, nbrDigiChans;
uint16_t *adcStartPtr;
uint8_t preSampleMpuMag, mpuMagFreq, mpuMagCount, mpu9150Initialised;
uint8_t preSampleBmpPress, bmpPressFreq, bmpPressCount, sampleBmpTemp, sampleBmpTempFreq;
//+ 1 byte (at start) as can only read/write 16-bit values at even addresses
//+ 1 byte to allow ACK at end of last data packet when stop streaming command is received
uint8_t txBuff0[DATA_PACKET_SIZE+2], txBuff1[DATA_PACKET_SIZE+2];
uint8_t docked, selectedLed;
//GSR
uint8_t gsr_active_resistor;
uint16_t last_GSR_val;
//ExG
uint8_t exgLength, exgChip, exgStartAddr;
//Daughter card EEPROM
uint8_t dcMemLength;
uint16_t dcMemOffset;
//BT baud rate
uint8_t changeBtBaudRate;

char *dierecord;

void main(void) {
   uint8_t digi_offset;

   Init();

   dierecord = (char *)0x01A0A;

   InfoMem_read((uint8_t *)0, storedConfig, NV_TOTAL_NUM_CONFIG_BYTES);
   if((storedConfig[NV_SENSORS0] == 0xFF) || (storedConfig[NV_BUFFER_SIZE] != 1)) {
      //if config was never written to Infomem, write default
      //assuming some other app didn't make use of InfoMem, or else InfoMem was erased
      SetDefaultConfiguration();
   }

   if(storedConfig[NV_BT_COMMS_BAUD_RATE]) {

      SetBtBaudRate(storedConfig[NV_BT_COMMS_BAUD_RATE]);
   }

   ConfigureChannels();

   txBuff0[1] = txBuff1[1] = DATA_PACKET; //packet type

   BlinkTimerStart();   //blink the blue LED
                        //also starts the charge status timer if not docked

   /*
   //Test PWM on Proto3 Deluxe board
   P1DIR |= BIT4;
   P1SEL |= BIT4;

   TA0CCR0 = 8192-1;    //1s period
   TA0CCR3 = 2048;      //25% duty cycle
   TA0CCTL3 = OUTMOD_7; //reset/set
   TA0CTL = TASSEL_1 + ID_2 +  MC_1 + TACLR; //ACLK, divide by 4, up mode, clear TAR

   P3OUT |= BIT3;       //enable PV_SW on Proto3 Deluxe board
    */

   if(docked)
      MonitorChargeStatus();

   while(1) {
      __bis_SR_register(LPM3_bits + GIE); //ACLK remains active
      if(!btIsConnected && (changeBtBaudRate!=0xFF)) {
         storedConfig[NV_BT_COMMS_BAUD_RATE] = changeBtBaudRate;
         InfoMem_write((void*)NV_BT_COMMS_BAUD_RATE, &storedConfig[NV_BT_COMMS_BAUD_RATE], 1);
         SetBtBaudRate(storedConfig[NV_BT_COMMS_BAUD_RATE]);
         changeBtBaudRate = 0xFF;
      }

      //order of these if statements is important
      //as ProcessCommand() relies on this ordering
      if(processCommand) {
         processCommand = 0;
         ProcessCommand();
      }
      if(configureChannels) {
         configureChannels = 0;
         ConfigureChannels();
      }
      if(sendResponse) {
         sendResponse = 0;
         SendResponse();
      }
      if(startSensing) {
         if(!stopSensing) {
            startSensing = 0;
            StartStreaming();
         }
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
      if(streamData) {
         streamData = 0;
         if(storedConfig[NV_SENSORS0] & SENSOR_GSR) {
            GsrRange();
         }
         digi_offset = (nbrAdcChans*2) + 4;
         if(currentBuffer){
            if(storedConfig[NV_SENSORS0] & SENSOR_MPU9150_GYRO) {
               MPU9150_getGyro(txBuff1+digi_offset);
               digi_offset+=6;
            }
            if(storedConfig[NV_SENSORS1] & SENSOR_LSM303DLHC_ACCEL) {
               LSM303DLHC_getAccel(txBuff1+digi_offset);
               digi_offset+=6;
            }
            if(storedConfig[NV_SENSORS0] & SENSOR_LSM303DLHC_MAG) {
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
            if(stopSensing) {
               stopSensing = 0;
               StopStreaming();
               *(txBuff1+digi_offset++) = ACK_COMMAND_PROCESSED;
            }
            if(btIsConnected)
               BT_write((txBuff1+1), (digi_offset-1));
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
            if(storedConfig[NV_SENSORS0] & SENSOR_LSM303DLHC_MAG) {
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
            if(stopSensing) {
               stopSensing = 0;
               StopStreaming();
               *(txBuff0+digi_offset++) = ACK_COMMAND_PROCESSED;
            }
            if(btIsConnected)
               BT_write((txBuff0+1), (digi_offset-1));
            currentBuffer = 1;
         }
         if(startSensing) {
            //if restarting sensing after previously stopping
            startSensing = 0;
            StartStreaming();
         }
      }
   }
}


void Init(void) {
   // Stop WDT
   //WDTCTL = WDTPW + WDTHOLD; // already handled in system_pre_init.c

   Board_init();
   Board_ledOn(LED_ALL);

   // Set Vcore to accommodate for max. allowed system speed
   SetVCore(3);

   // Start 32.768kHz XTAL as ACLK
   LFXT_Start(XT1DRIVE_0);

   // Start 24MHz XTAL as MCLK and SMCLK
   XT2_Start(XT2DRIVE_2);        // XT2DRIVE_2 or XTDRIVE_3 for 24MHz (userguide section 5.4.7)
   UCSCTL4 |= SELS_5 + SELM_5;   // SMCLK=MCLK=XT2

   SFRIFG1 = 0;                  // clear interrupt flag register
   SFRIE1 |= OFIE;               // enable oscillator fault interrupt enable

   currentBuffer = 0;
   processCommand = 0;
   sendAck = 0;
   inquiryResponse = 0;
   samplingRateResponse = 0;
   aAccelCalibrationResponse = 0;
   lsm303dlhcAccelRangeResponse = 0;
   lsm303dlhcMagGainResponse = 0;
   lsm303dlhcMagSamplingRateResponse = 0;
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
   startSensing = 0;
   stopSensing = 0;
   configureChannels = 0;
   sendResponse = 0;
   sampleMpu9150Mag = 0;
   sampleBmp180Press = 0;
   streamData = 0;
   sensing = 0;
   btIsConnected = 0;
   waitingForArgs = 0;
   argsSize = 0;
   nbrAdcChans = 0;
   nbrDigiChans = 0;
   preSampleMpuMag = 0;
   mpu9150Initialised = 0;
   preSampleBmpPress = 0;
   sampleBmpTemp = 0;
   docked = 0;
   selectedLed = 0;
   blinkLedResponse = 0;
   gsrRangeResponse = 0;
   last_GSR_val = 0;
   btCommsBaudRateResponse = 0;
   changeBtBaudRate = 0xFF;   //indicates doesn't need changing

   //Globally enable interrupts
   _enable_interrupts();

   if(P2IN & BIT3) {
      //high (docked)
      P2IES |= BIT3;    //look for falling edge
      docked = 1;
   } else {
      //low (undocked)
      P2IES &= ~BIT3;   //look for rising edge
   }
   P2IFG &= ~BIT3;      //clear flag
   P2IE |= BIT3;        //enable interrupt

   BT_init();
   BT_disableRemoteConfig(1);
   BT_setRadioMode(SLAVE_MODE);
   BT_configure();
   BT_receiveFunction(&BtDataAvailable);

   // enable switch1 interrupt
   Button_init();
   Button_interruptEnable();

   //EXP_RESET_N
   P3OUT &= ~BIT3;      //set low
   P3DIR |= BIT3;       //set as output

   StartTB0();

   Board_ledOff(LED_ALL);
}


inline void StartStreaming(void) {
   uint8_t offset;
   if(!sensing) {
      sensing = 1;

      if(storedConfig[NV_SENSORS0] & SENSOR_A_ACCEL) {
         P8REN &= ~BIT6;      //disable pull down resistor
         P8DIR |= BIT6;       //set as output
         P8OUT |= BIT6;       //analog accel being used so take out of sleep mode
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

      if(storedConfig[NV_CONFIG_SETUP_BYTE3]&0x01) //EXT_RESET_N
         P3OUT |= BIT3;                            //set high

      if(storedConfig[NV_SENSORS1] & SENSOR_BRIDGE_AMP)
         P2OUT |= BIT0;                            //GPIO_INTERNAL1 set high

      if((storedConfig[NV_SENSORS0] & SENSOR_MPU9150_GYRO)
            || (storedConfig[NV_SENSORS2] & SENSOR_MPU9150_ACCEL)
            || (storedConfig[NV_SENSORS2] & SENSOR_MPU9150_MAG)) {
         MPU9150_init();
         MPU9150_wake(1);
         mpu9150Initialised = 1;
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
            if(*(uint16_t *)(storedConfig+NV_SAMPLING_RATE) >= 394) {
               //max of approx. 3ms to sample everything + 9ms between starting mag to data ready
               //so 12ms in total (394 ticks of 32768Hz clock = 12.024ms)
               //so there is time to get the mag sampled before the readings need to start each sample period
               preSampleMpuMag = 1;
            } else {
               //set the mag values as invalid for first few samples
               offset = (nbrAdcChans*2) + 4;
               if(storedConfig[NV_SENSORS0] & SENSOR_MPU9150_GYRO) offset += 6;
               if(storedConfig[NV_SENSORS1] & SENSOR_LSM303DLHC_ACCEL) offset += 6;
               if(storedConfig[NV_SENSORS0] & SENSOR_LSM303DLHC_MAG) offset += 6;
               if(storedConfig[NV_SENSORS2] & SENSOR_MPU9150_ACCEL) offset += 6;
               *(txBuff0+offset) = 0xFF;
               *(txBuff1+offset++) = 0xFF;
               *(txBuff0+offset) = 0x7F;
               *(txBuff1+offset++) = 0x7F;
               *(txBuff0+offset) = 0xFF;
               *(txBuff1+offset++) = 0xFF;
               *(txBuff0+offset) = 0x7F;
               *(txBuff1+offset++) = 0x7F;
               *(txBuff0+offset) = 0xFF;
               *(txBuff1+offset++) = 0xFF;
               *(txBuff0+offset) = 0x7F;
               *(txBuff1+offset) = 0x7F;

               //sample, then check each sample period if ready to read. Start new sample immediately
               MPU9150_startMagMeasurement();
               preSampleMpuMag = 0;
               mpuMagCount = mpuMagFreq = (295 / *(uint16_t *)(storedConfig+NV_SAMPLING_RATE)) + 1;
            }
         }
      }

      if((storedConfig[NV_SENSORS0] & SENSOR_LSM303DLHC_MAG) || (storedConfig[NV_SENSORS1] & SENSOR_LSM303DLHC_ACCEL)) {
         if(!((storedConfig[NV_SENSORS0] & SENSOR_MPU9150_GYRO) || (storedConfig[NV_SENSORS2] & SENSOR_MPU9150_ACCEL)
               || (storedConfig[NV_SENSORS2] & SENSOR_MPU9150_MAG))) {
            //Initialise I2C
            LSM303DLHC_init();
         }
         if(storedConfig[NV_SENSORS1] & SENSOR_LSM303DLHC_ACCEL) {
            LSM303DLHC_accelInit(((storedConfig[NV_CONFIG_SETUP_BYTE0]&0xF0)>>4),   //sampling rate
                            ((storedConfig[NV_CONFIG_SETUP_BYTE0]&0x0C)>>2),        //range
                            ((storedConfig[NV_CONFIG_SETUP_BYTE0]&0x02)>>1),        //low power mode
                            (storedConfig[NV_CONFIG_SETUP_BYTE0]&0x01));            //high resolution mode
         }
         if(storedConfig[NV_SENSORS0] & SENSOR_LSM303DLHC_MAG)
            LSM303DLHC_magInit(((storedConfig[NV_CONFIG_SETUP_BYTE2]&0x1C)>>2),     //sampling rate
                           ((storedConfig[NV_CONFIG_SETUP_BYTE2]&0xE0)>>5));        //gain
      }

      if(storedConfig[NV_SENSORS2] & SENSOR_BMP180_PRESSURE) {
         if(!((storedConfig[NV_SENSORS0] & SENSOR_MPU9150_GYRO) || (storedConfig[NV_SENSORS2] & SENSOR_MPU9150_ACCEL)
               || (storedConfig[NV_SENSORS2] & SENSOR_MPU9150_MAG) || (storedConfig[NV_SENSORS0] & SENSOR_LSM303DLHC_MAG)
               || (storedConfig[NV_SENSORS1] & SENSOR_LSM303DLHC_ACCEL))) {
            BMP180_init();
         }
         if((((storedConfig[NV_CONFIG_SETUP_BYTE3]&0x30)>>4)==0) && (*(uint16_t *)(storedConfig+NV_SAMPLING_RATE) >= 246)) {
            //max of approx. 3ms to sample everything + 4.5ms between starting press to data ready
            //so 7.5ms in total (246 ticks of 32768Hz clock = 7.507ms)
            preSampleBmpPress = 1;
            bmpPressFreq = 1;    //required for the calculation of sampleBmpTempFreq below
         } else if((((storedConfig[NV_CONFIG_SETUP_BYTE3]&0x30)>>4)==1) && (*(uint16_t *)(storedConfig+NV_SAMPLING_RATE) >= 345)) {
            //max of approx. 3ms to sample everything + 7.5ms between starting press to data ready
            //so 10.5ms in total (345 ticks of 32768Hz clock = 10.529ms)
            preSampleBmpPress = 1;
            bmpPressFreq = 1;    //required for the calculation of sampleBmpTempFreq below
         } else if((((storedConfig[NV_CONFIG_SETUP_BYTE3]&0x30)>>4)==2) && (*(uint16_t *)(storedConfig+NV_SAMPLING_RATE) >= 541)) {
            //max of approx. 3ms to sample everything + 13.5ms between starting press to data ready
            //so 16.5ms in total (541 ticks of 32768Hz clock = 16.510ms)
            preSampleBmpPress = 1;
            bmpPressFreq = 1;    //required for the calculation of sampleBmpTempFreq below
         } else if((((storedConfig[NV_CONFIG_SETUP_BYTE3]&0x30)>>4)==3) && (*(uint16_t *)(storedConfig+NV_SAMPLING_RATE) >= 934)) {
            //max of approx. 3ms to sample everything + 25.5ms between starting press to data ready
            //so 28.5ms in total (934 ticks of 32768Hz clock = 28.503ms)
            preSampleBmpPress = 1;
            bmpPressFreq = 1;    //required for the calculation of sampleBmpTempFreq below
         } else {
            //set the pressure values to 0 for first few samples
            offset = (nbrAdcChans*2) + 4;
            if(storedConfig[NV_SENSORS0] & SENSOR_MPU9150_GYRO) offset += 6;
            if(storedConfig[NV_SENSORS1] & SENSOR_LSM303DLHC_ACCEL) offset += 6;
            if(storedConfig[NV_SENSORS0] & SENSOR_LSM303DLHC_MAG) offset += 6;
            if(storedConfig[NV_SENSORS2] & SENSOR_MPU9150_ACCEL) offset += 6;
            if(storedConfig[NV_SENSORS2] & SENSOR_MPU9150_MAG) offset += 6;
            *(txBuff0+offset) = 0x00;
            *(txBuff1+offset++) = 0x00;
            *(txBuff0+offset) = 0x00;
            *(txBuff1+offset++) = 0x00;
            *(txBuff0+offset) = 0x00;
            *(txBuff1+offset++) = 0x00;
            *(txBuff0+offset) = 0x00;
            *(txBuff1+offset++) = 0x00;
            *(txBuff0+offset) = 0x00;
            *(txBuff1+offset) = 0x00;

            //sample, then check each sample period if ready to read. Start new sample immediately
            BMP180_startTempMeasurement();
            preSampleBmpPress = 0;
            if(((storedConfig[NV_CONFIG_SETUP_BYTE3]&0x30)>>4)==0) {
               bmpPressCount = bmpPressFreq = (148 / *(uint16_t *)(storedConfig+NV_SAMPLING_RATE)) + 1;
            } else if(((storedConfig[NV_CONFIG_SETUP_BYTE3]&0x30)>>4)==1) {
               bmpPressCount = bmpPressFreq = (246 / *(uint16_t *)(storedConfig+NV_SAMPLING_RATE)) + 1;
            } else if(((storedConfig[NV_CONFIG_SETUP_BYTE3]&0x30)>>4)==2) {
               bmpPressCount = bmpPressFreq = (443 / *(uint16_t *)(storedConfig+NV_SAMPLING_RATE)) + 1;
            } else {
               bmpPressCount = bmpPressFreq = (836 / *(uint16_t *)(storedConfig+NV_SAMPLING_RATE)) + 1;
            }
         }
         //only need to sample temp once a second at most
         if(*(uint16_t *)(storedConfig+NV_SAMPLING_RATE) >= 10923) {
            //less than 3Hz
            //so every second sample must be temp
            sampleBmpTemp = sampleBmpTempFreq = 1;
         } else {
            sampleBmpTemp = sampleBmpTempFreq = ((32768 / *(uint16_t *)(storedConfig+NV_SAMPLING_RATE)) - 1) / bmpPressFreq;
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
      BlinkTimerStart();      //Needs to go after SampleTimerStart(), as TBR is reset in SampleTimerStart()
   }
}

inline void StopStreaming(void) {
   if(sensing) {
      SampleTimerStop();
      ADC_disable();
      DMA0_disable();
      P8OUT &= ~BIT6;
      P8REN |= BIT6;          //enable pull down resistor
      P8DIR &= ~BIT6;         //SW_ACCEL set as input

      P3OUT &= ~BIT3;         //set EXP_RESET_N low

      P2OUT &= ~BIT0;         //set GPIO_INTERNAL1 low

      if((storedConfig[NV_SENSORS0] & SENSOR_MPU9150_GYRO) || (storedConfig[NV_SENSORS2] & SENSOR_MPU9150_ACCEL)) {
         MPU9150_wake(0);
      }
      if((storedConfig[NV_SENSORS0] & SENSOR_LSM303DLHC_MAG) || (storedConfig[NV_SENSORS1] & SENSOR_LSM303DLHC_ACCEL)) {
         LSM303DLHC_sleep();
      }
      if((storedConfig[NV_SENSORS0] & SENSOR_EXG1_24BIT) || (storedConfig[NV_SENSORS2] & SENSOR_EXG1_16BIT)) {
         EXG_stop(0);         //probably not needed
      }
      if((storedConfig[NV_SENSORS0] & SENSOR_EXG2_24BIT) || (storedConfig[NV_SENSORS2] & SENSOR_EXG2_16BIT)) {
         EXG_stop(1);         //probably not needed
      }
      if((storedConfig[NV_SENSORS0] & SENSOR_EXG1_24BIT) || (storedConfig[NV_SENSORS0] & SENSOR_EXG2_24BIT) ||
            (storedConfig[NV_SENSORS2] & SENSOR_EXG1_16BIT) || (storedConfig[NV_SENSORS2] & SENSOR_EXG2_16BIT)) {
         EXG_powerOff();
      }

      __delay_cycles(240000); //10ms (assuming 24MHz clock)
                              //give plenty of time for I2C operations to finish before disabling I2C
      I2C_Disable();
      P8OUT &= ~BIT4;         //set SW_I2C low to power off I2C chips
      if(btIsConnected)
         BlinkTimerStop();
      preSampleMpuMag = 0;
      preSampleBmpPress = 0;
      sensing = 0;
   }
}


void ConfigureChannels(void) {
   uint8_t *channel_contents_ptr = channelContents;
   uint16_t mask=0;

   nbrAdcChans = nbrDigiChans = 0;

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
   //Bridge amplifier
   if(storedConfig[NV_SENSORS1] & SENSOR_BRIDGE_AMP) {
      *channel_contents_ptr++ = BRIDGE_AMP_HIGH;
      *channel_contents_ptr++ = BRIDGE_AMP_LOW;
      mask += MASK_BRIDGE_AMP;
      nbrAdcChans += 2;
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
   //Internal ADC channel A13
   if((storedConfig[NV_SENSORS1] & SENSOR_INT_A13) && !(storedConfig[NV_SENSORS1] & SENSOR_BRIDGE_AMP)) {
      *channel_contents_ptr++ = INTERNAL_ADC_13;
      mask += MASK_INT_A13;
      nbrAdcChans++;
   }
   //Internal ADC channel A14
   if((storedConfig[NV_SENSORS2] & SENSOR_INT_A14) && !(storedConfig[NV_SENSORS1] & SENSOR_BRIDGE_AMP)) {
      *channel_contents_ptr++ = INTERNAL_ADC_14;
      mask += MASK_INT_A14;
      nbrAdcChans++;
   }
   if (storedConfig[NV_SENSORS0] & SENSOR_GSR) {
      //needs to be last analog channel
      *channel_contents_ptr++ = GSR_RAW;
      mask += MASK_INT_A1;
      nbrAdcChans++;
   }
   //Internal ADC channel A1
   else if(storedConfig[NV_SENSORS1] & SENSOR_INT_A1) {
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
   if(storedConfig[NV_SENSORS2] & SENSOR_MPU9150_MAG) {
      *channel_contents_ptr++ = X_MPU9150_MAG;
      *channel_contents_ptr++ = Y_MPU9150_MAG;
      *channel_contents_ptr++ = Z_MPU9150_MAG;
      nbrDigiChans += 3;
   }
   if(storedConfig[NV_SENSORS2] & SENSOR_BMP180_PRESSURE) {
      *channel_contents_ptr++ = BMP180_TEMP;
      *channel_contents_ptr++ = BMP180_PRESSURE;
      nbrDigiChans += 2;
   }
   if(!(storedConfig[NV_SENSORS1] & SENSOR_INT_A1) &&
		   !(storedConfig[NV_SENSORS2] & SENSOR_INT_A14) &&
		   !(storedConfig[NV_SENSORS1] & SENSOR_BRIDGE_AMP)) {
	   if(storedConfig[NV_SENSORS0] & SENSOR_EXG1_24BIT) {
		  *channel_contents_ptr++ = EXG_ADS1292R_1_STATUS;
		  *channel_contents_ptr++ = EXG_ADS1292R_1_CH1_24BIT;
		  *channel_contents_ptr++ = EXG_ADS1292R_1_CH2_24BIT;
		  nbrDigiChans += 3;
	   } else if(storedConfig[NV_SENSORS2] & SENSOR_EXG1_16BIT) {
		  *channel_contents_ptr++ = EXG_ADS1292R_1_STATUS;
		  *channel_contents_ptr++ = EXG_ADS1292R_1_CH1_16BIT;
		  *channel_contents_ptr++ = EXG_ADS1292R_1_CH2_16BIT;
		  nbrDigiChans += 3;
	   }
	   if(storedConfig[NV_SENSORS0] & SENSOR_EXG2_24BIT) {
		  *channel_contents_ptr++ = EXG_ADS1292R_2_STATUS;
		  *channel_contents_ptr++ = EXG_ADS1292R_2_CH1_24BIT;
		  *channel_contents_ptr++ = EXG_ADS1292R_2_CH2_24BIT;
		  nbrDigiChans += 3;
	   } else if(storedConfig[NV_SENSORS2] & SENSOR_EXG2_16BIT) {
		  *channel_contents_ptr++ = EXG_ADS1292R_2_STATUS;
		  *channel_contents_ptr++ = EXG_ADS1292R_2_CH1_16BIT;
		  *channel_contents_ptr++ = EXG_ADS1292R_2_CH2_16BIT;
		  nbrDigiChans += 3;
	   }
   }

   if(mask) {
      adcStartPtr = ADC_init(mask);
      DMA0_transferDoneFunction(&Dma0ConversionDone);
      if(adcStartPtr)
         DMA0_init(adcStartPtr, (uint16_t *)(txBuff0+4), nbrAdcChans);
   }

   //TODO: check if was sensing, and if so restart
}

uint8_t BtDataAvailable(uint8_t data) {
   if(waitingForArgs) {
      args[argsSize++] = data;
      if(((gAction == SET_EXG_REGS_COMMAND) && (argsSize == 3)) ||
            ((gAction == SET_DAUGHTER_CARD_MEM_COMMAND) && (argsSize == 1))) {
         waitingForArgs += data;
      }
      if(!--waitingForArgs) {
         processCommand = 1;
         argsSize = 0;
         return 1;
      }
      else
         return 0;
   } else {
      switch(data) {
      case INQUIRY_COMMAND:
      case GET_SAMPLING_RATE_COMMAND:
      case TOGGLE_LED_COMMAND:
      case START_STREAMING_COMMAND:
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
      case GET_BT_COMMS_BAUD_RATE:
         gAction = data;
         processCommand = 1;
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
      case SET_BT_COMMS_BAUD_RATE:
         waitingForArgs = 1;
         gAction = data;
         return 0;
      case SET_SAMPLING_RATE_COMMAND:
      case GET_DAUGHTER_CARD_ID_COMMAND:
         waitingForArgs = 2;
         gAction = data;
         return 0;
      case SET_SENSORS_COMMAND:
      case GET_EXG_REGS_COMMAND:
      case SET_EXG_REGS_COMMAND:
      case GET_DAUGHTER_CARD_MEM_COMMAND:
      case SET_DAUGHTER_CARD_MEM_COMMAND:
         waitingForArgs = 3;
         gAction = data;
         return 0;
      case SET_CONFIG_SETUP_BYTES_COMMAND:
         waitingForArgs = 4;
         gAction = data;
         return 0;
      case SET_A_ACCEL_CALIBRATION_COMMAND:
      case SET_MPU9150_GYRO_CALIBRATION_COMMAND:
      case SET_LSM303DLHC_MAG_CALIBRATION_COMMAND:
      case SET_LSM303DLHC_ACCEL_CALIBRATION_COMMAND:
         waitingForArgs = 21;
         gAction = data;
         return 0;
      default:
         return 0;
      }
   }
}


void ProcessCommand(void) {
   switch(gAction) {
   case INQUIRY_COMMAND:
      inquiryResponse = 1;
      break;
   case GET_SAMPLING_RATE_COMMAND:
      samplingRateResponse = 1;
      break;
   case TOGGLE_LED_COMMAND:
      Board_ledToggle(LED_RED);
      break;
   case START_STREAMING_COMMAND:
      startSensing = 1;
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
      InfoMem_write((void*)NV_SENSORS0, &storedConfig[NV_SENSORS0], 3);
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
   case SET_LSM303DLHC_ACCEL_RANGE_COMMAND:
      if(args[0] < 4)
         storedConfig[NV_CONFIG_SETUP_BYTE0] = (storedConfig[NV_CONFIG_SETUP_BYTE0]&0xF3) + ((args[0]&0x03)<<2);
      else
         storedConfig[NV_CONFIG_SETUP_BYTE0] &= 0xF3;
      InfoMem_write((void*)NV_CONFIG_SETUP_BYTE0, &storedConfig[NV_CONFIG_SETUP_BYTE0], 1);
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
      if(sensing) {
         stopSensing = 1;     //stops streaming before configuring channels
         startSensing = 1;    //starts streaming after configuring channels
      }
      break;
   case SET_CHARGE_STATUS_LED_COMMAND:
      if(args[0] > 2)
         args[0] = 0;
      _disable_interrupts();
      if(!docked && IsLedSet()) {
         ClearLed();
         selectedLed = args[0];
         SetLed();
      } else
         selectedLed = args[0];
      _enable_interrupts();
      break;
   case SET_LSM303DLHC_MAG_SAMPLING_RATE_COMMAND:
      if(args[0] < 8)
         storedConfig[NV_CONFIG_SETUP_BYTE2] = (storedConfig[NV_CONFIG_SETUP_BYTE2]&0xE3) + ((args[0]&0x07)<<2);
      else
         storedConfig[NV_CONFIG_SETUP_BYTE2] = (storedConfig[NV_CONFIG_SETUP_BYTE2]&0xE3) + (LSM303DLHC_MAG_75HZ<<2);
      InfoMem_write((void*)NV_CONFIG_SETUP_BYTE2, &storedConfig[NV_CONFIG_SETUP_BYTE2], 1);
      if(sensing) {
         stopSensing = 1;     //stops streaming before configuring channels
         startSensing = 1;    //starts streaming after configuring channels
      }
      break;
   case SET_LSM303DLHC_ACCEL_LPMODE_COMMAND:
      if(args[0] == 1)
         storedConfig[NV_CONFIG_SETUP_BYTE0] = (storedConfig[NV_CONFIG_SETUP_BYTE0]&0xFD) + 0x02;
      else
         storedConfig[NV_CONFIG_SETUP_BYTE0] &= 0xFD;
      InfoMem_write((void*)NV_CONFIG_SETUP_BYTE0, &storedConfig[NV_CONFIG_SETUP_BYTE0], 1);
      if(sensing) {
         stopSensing = 1;     //stops streaming before configuring channels
         startSensing = 1;    //starts streaming after configuring channels
      }
      break;
   case SET_LSM303DLHC_ACCEL_HRMODE_COMMAND:
      if(args[0] == 1)
         storedConfig[NV_CONFIG_SETUP_BYTE0] = (storedConfig[NV_CONFIG_SETUP_BYTE0]&0xFE) + 0x01;
      else
         storedConfig[NV_CONFIG_SETUP_BYTE0] &= 0xFE;
      InfoMem_write((void*)NV_CONFIG_SETUP_BYTE0, &storedConfig[NV_CONFIG_SETUP_BYTE0], 1);
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
      if(sensing) {
         stopSensing = 1;     //stops streaming before configuring channels
         startSensing = 1;    //starts streaming after configuring channels
      }
      break;
   case SET_MPU9150_SAMPLING_RATE_COMMAND:
      storedConfig[NV_CONFIG_SETUP_BYTE1] = args[0];
      InfoMem_write((void*)NV_CONFIG_SETUP_BYTE1, &storedConfig[NV_CONFIG_SETUP_BYTE1], 1);
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
      break;
   case SET_INTERNAL_EXP_POWER_ENABLE_COMMAND:
      if(args[0] == 1)
         storedConfig[NV_CONFIG_SETUP_BYTE3] = (storedConfig[NV_CONFIG_SETUP_BYTE3]&0xFE) + (args[0]&0x01);
      else
         storedConfig[NV_CONFIG_SETUP_BYTE3] &= 0xFE;
      InfoMem_write((void*)NV_CONFIG_SETUP_BYTE3, &storedConfig[NV_CONFIG_SETUP_BYTE3], 1);
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
      if(sensing) {
         stopSensing = 1;     //stops streaming before configuring channels
         startSensing = 1;    //starts streaming after configuring channels
      }
      break;
   case SET_SAMPLING_RATE_COMMAND:
      *(uint16_t *)(storedConfig+NV_SAMPLING_RATE) = *(uint16_t *)args;
      InfoMem_write((void*)NV_SAMPLING_RATE, &storedConfig[NV_SAMPLING_RATE], 2);
      if(sensing) {
         //restart sampling timer to use new sampling rate
         stopSensing = 1;
         startSensing = 1;
      }
      break;
   case SET_A_ACCEL_CALIBRATION_COMMAND:
      memcpy(&storedConfig[NV_A_ACCEL_CALIBRATION], args, 21);
      InfoMem_write((void*)NV_A_ACCEL_CALIBRATION, &storedConfig[NV_A_ACCEL_CALIBRATION], 21);
      break;
   case GET_A_ACCEL_CALIBRATION_COMMAND:
      aAccelCalibrationResponse = 1;
      break;
   case SET_MPU9150_GYRO_CALIBRATION_COMMAND:
      memcpy(&storedConfig[NV_MPU9150_GYRO_CALIBRATION], args, 21);
      InfoMem_write((void*)NV_MPU9150_GYRO_CALIBRATION, &storedConfig[NV_MPU9150_GYRO_CALIBRATION], 21);
      break;
   case GET_MPU9150_GYRO_CALIBRATION_COMMAND:
      gyroCalibrationResponse = 1;
      break;
   case SET_LSM303DLHC_MAG_CALIBRATION_COMMAND:
      memcpy(&storedConfig[NV_LSM303DLHC_MAG_CALIBRATION], args, 21);
      InfoMem_write((void*)NV_LSM303DLHC_MAG_CALIBRATION, &storedConfig[NV_LSM303DLHC_MAG_CALIBRATION], 21);
      break;
   case GET_LSM303DLHC_MAG_CALIBRATION_COMMAND:
      magCalibrationResponse = 1;
      break;
   case SET_LSM303DLHC_ACCEL_CALIBRATION_COMMAND:
      memcpy(&storedConfig[NV_LSM303DLHC_ACCEL_CALIBRATION], args, 21);
      InfoMem_write((void*)NV_LSM303DLHC_ACCEL_CALIBRATION, &storedConfig[NV_LSM303DLHC_ACCEL_CALIBRATION], 21);
      break;
   case SET_GSR_RANGE_COMMAND:
      if(args[0] <= 4)
         storedConfig[NV_CONFIG_SETUP_BYTE3] = (storedConfig[NV_CONFIG_SETUP_BYTE3]&0xF1) + ((args[0]&0x07)<<1);
      else
         storedConfig[NV_CONFIG_SETUP_BYTE3] = (storedConfig[NV_CONFIG_SETUP_BYTE3]&0xF1) + (HW_RES_40K<<1);
      InfoMem_write((void*)NV_CONFIG_SETUP_BYTE3, &storedConfig[NV_CONFIG_SETUP_BYTE3], 1);
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
         } else {
            memcpy((storedConfig+NV_EXG_ADS1292R_1_CONFIG1+args[1]), (args+3), args[2]);
            InfoMem_write((void*)(NV_EXG_ADS1292R_1_CONFIG1+args[1]), (storedConfig+NV_EXG_ADS1292R_1_CONFIG1+args[1]), args[2]);
         }
      }
      break;
   case RESET_TO_DEFAULT_CONFIGURATION_COMMAND:
      SetDefaultConfiguration();
      configureChannels = 1;
      if(sensing) {
         stopSensing = 1;
         startSensing = 1;
      }
      break;
   case RESET_CALIBRATION_VALUE_COMMAND:
      memset(&storedConfig[NV_A_ACCEL_CALIBRATION], 0xFF, NV_NUM_CALIBRATION_BYTES);
      InfoMem_write((void*)NV_A_ACCEL_CALIBRATION, &storedConfig[NV_A_ACCEL_CALIBRATION], NV_NUM_CALIBRATION_BYTES);
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
   case GET_BT_COMMS_BAUD_RATE:
      btCommsBaudRateResponse = 1;
      break;
   case SET_BT_COMMS_BAUD_RATE:
      if(args[0] != storedConfig[NV_BT_COMMS_BAUD_RATE]) {
         if(args[0]>0 && args[0]<11) {
            changeBtBaudRate = args[0];
         } else {
            changeBtBaudRate = 0;
         }
      }
      break;
   default:;
   }
   sendAck = 1;
   sendResponse = 1;
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
         *(resPacket + packet_length++) = FW_VER_REL;
         fwVersionResponse = 0;
      } else if (blinkLedResponse) {
         *(resPacket + packet_length++) = CHARGE_STATUS_LED_RESPONSE;
         *(resPacket + packet_length++) = selectedLed;
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
      } else if (btCommsBaudRateResponse) {
         *(resPacket + packet_length++) = BT_COMMS_BAUD_RATE_RESPONSE;
         *(resPacket + packet_length++) = storedConfig[NV_BT_COMMS_BAUD_RATE];
         btCommsBaudRateResponse = 0;
      }
   }

   BT_write(resPacket, packet_length);
}


void SetDefaultConfiguration(void) {
   *(uint16_t *)(storedConfig+NV_SAMPLING_RATE) = 640;                                       //51.2Hz
   storedConfig[NV_BUFFER_SIZE] = 1;
   storedConfig[NV_SENSORS0] = SENSOR_A_ACCEL + SENSOR_MPU9150_GYRO + SENSOR_LSM303DLHC_MAG; //core sensors enabled
   storedConfig[NV_SENSORS1] = SENSOR_VBATT;
   storedConfig[NV_SENSORS2] = 0;
   storedConfig[NV_CONFIG_SETUP_BYTE0] = (LSM303DLHC_ACCEL_100HZ<<4) + (ACCEL_2G<<2);        //LSM303DLHC Accel 100Hz, +/-2G,
                                                                                             //Low Power and High Resolution modes off
   storedConfig[NV_CONFIG_SETUP_BYTE1] = 0x9B;                                               //MPU9150 sampling rate of 8kHz/(155+1)
                                                                                             //i.e. 51.282Hz
   storedConfig[NV_CONFIG_SETUP_BYTE2] = (LSM303DLHC_MAG_1_3G<<5) + (LSM303DLHC_MAG_75HZ<<2) //LSM303DLHC Mag 75Hz, +/-1.3 Gauss
                                    + MPU9150_GYRO_500DPS;                                   //MPU9150 Gyro +/-500 degrees per second
   storedConfig[NV_CONFIG_SETUP_BYTE3] = (ACCEL_2G<<6) + (BMP180_OSS_1<<4)                   //MPU9150 Accel +/-2G, BMP pressure oversampling ratio 1
                                    + (HW_RES_40K<<1);                                       //GSR range resistor 40k
                                                                                             //EXP_RESET_N pin set low
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

   //set BT baud rate to 115200
   storedConfig[NV_BT_COMMS_BAUD_RATE] = 0;                                                  // 115200 baud

   InfoMem_write((void*)0, storedConfig, NV_NUM_SETTINGS_BYTES);
}


/**
*** Switch SW1, BT_RTS and BT connect/disconnect
**/
#pragma vector=PORT1_VECTOR
__interrupt void Port1_ISR(void)
{
   // Context save interrupt flag before calling interrupt vector.
   // Reading interrupt vector generator will automatically clear IFG flag

   switch (__even_in_range(P1IV, P1IV_P1IFG7)) {
   //BT Connect/Disconnect
   case  P1IV_P1IFG0:   //BT Connect/Disconnect
      if(P1IN & BIT0) {
         //BT is connected
         P1IES |= BIT0; //look for falling edge
         BT_connectionInterrupt(1);
         btIsConnected = 1;
         BlinkTimerStop();
         waitingForArgs = 0;
         Board_ledOn(LED_BLUE);
      } else {
         //BT is not connected
         P1IES &= ~BIT0; //look for rising edge
         BT_connectionInterrupt(0);
         btIsConnected = 0;
         if(sensing) {
            stopSensing = 1;
         }
         Board_ledOff(LED_BLUE);
         BlinkTimerStart();
         if(changeBtBaudRate != 0xFF) {
            __bic_SR_register_on_exit(LPM3_bits);
         }
      }
      break;

   //BT RTS
   case  P1IV_P1IFG3:
      if(P1IN & BIT3) {
         P1IES |= BIT3;    //look for falling edge
         BT_rtsInterrupt(1);
      } else {
         P1IES &= ~BIT3;   //look for rising edge
         BT_rtsInterrupt(0);
      }
      break;

   //ExG chip2 data ready
   case  P1IV_P1IFG4:
      EXG_dataReadyChip2();
      break;

   //BUTTON_SW1
   case  P1IV_P1IFG6:
      //disable switch interrupts
      Button_debounce();
      break;

   // Default case
   default:
      break;
   }
}

inline void SetLed(void) {
   switch(selectedLed) {
   case 0:
      Board_ledOn(LED_GREEN0);
      break;
   case 1:
      Board_ledOn(LED_YELLOW0);
      break;
   default:
      Board_ledOn(LED_RED);
   }
}

inline void ClearLed(void) {
   switch(selectedLed) {
   case 0:
      Board_ledOff(LED_GREEN0);
      break;
   case 1:
      Board_ledOff(LED_YELLOW0);
      break;
   default:
      Board_ledOff(LED_RED);
   }
}

inline uint8_t IsLedSet(void) {
   switch(selectedLed) {
   case 0:
      if(P7OUT & BIT3) return 1;
      else return 0;
   case 1:
      if(P8OUT & BIT0) return 1;
      else return 0;
   default:
      if(P7OUT & BIT2) return 1;
      else return 0;
   }
}

/**
*** TB0 is used for both the blink timer and sample timers
**/
inline void StartTB0(void) {
   TB0CTL = TBSSEL_1 + MC_2 + TBCLR;   //ACLK, continuous mode, clear TBR
}

inline void ResetTB0(void) {
   TB0CTL += TBCLR;
}

inline void StopTB0(void) {
   TB0CTL = MC0;
}

//read TB0 counter while timer is running
inline uint16_t GetTB0(void) {
   register uint16_t t0, t1;
   uint8_t ie;
   if(ie=(__get_SR_register()&0x0008)) //interrupts enabled?
      __disable_interrupt();
   t1 = TB0R;
   do {t0=t1; t1=TB0R;} while(t0!=t1);
   if(ie)
      __enable_interrupt();
   return t1;
}


/**
*** Charge Status Timer
**/
void ChargeStatusTimerStart(void) {
   TB0CCR3 = GetTB0() + 100;
   TB0CCTL3 = CCIE;
   SetLed();
}

void ChargeStatusTimerStop(void) {
   TB0CCTL3 = 0;
   ClearLed();
}

/**
*** Monitor charge status
**/
inline void MonitorChargeStatus(void) {
   if(P2IN & BIT6) {    //CHG_STAT1
      P2IES |= BIT6;    //look for falling edge
   } else {
      P2IES &= ~BIT6;   //look for rising edge
   }
   if(P2IN & BIT7) {    //CHG_STAT2
      P2IES |= BIT7;    //look for falling edge
   } else {
      P2IES &= ~BIT7;   //look for rising edge
   }
   P2IFG &= ~(BIT6 + BIT7);

   SetChargeStatusLeds();

   P2IE |= BIT6 + BIT7; //enable interrupts
}

inline void MonitorChargeStatusStop(void) {
   P2IE &= ~(BIT6 + BIT7);
   Board_ledOff(LED_RED + LED_YELLOW0 + LED_GREEN0);
}

inline void SetChargeStatusLeds(void) {
   Board_ledOff(LED_RED + LED_YELLOW0 + LED_GREEN0);

   if((P2IN&BIT6) && !(P2IN&BIT7)) {
      //charge completed
      Board_ledOn(LED_GREEN0);
   } else if(!(P2IN&BIT6) && (P2IN&BIT7)) {
      //charging
      Board_ledOn(LED_YELLOW0);
   }
}
/**
*** Blink Timer
**/
// USING TB0 with CCR4
void BlinkTimerStart(void) {
   if(sensing) {
      TB0CCR4 = GetTB0() + 32768;   //1Hz
      Board_ledOff(LED_BLUE);
   } else {
      TB0CCR4 = GetTB0() + 100;
      Board_ledOn(LED_BLUE);
      if(!docked) {
         //to keep both LEDs in sync
         ChargeStatusTimerStop();
         ChargeStatusTimerStart();
      }
   }
   TB0CCTL4 = CCIE;
}

inline void BlinkTimerStop() {
   TB0CCTL4 = 0;
   Board_ledOn(LED_BLUE);
}

/**
*** Sample Timer
**/
void SampleTimerStart(void) {
   StopTB0();
   if(preSampleMpuMag || preSampleBmpPress) {
      if(preSampleBmpPress && (((storedConfig[NV_CONFIG_SETUP_BYTE3]&0x30)>>4)==0)) {
         if(preSampleMpuMag) {
            TB0CCR0 = *(uint16_t *)(storedConfig+NV_SAMPLING_RATE) + 295;  //295 ticks of 32768Hz clock is 9.0027ms
            TB0CCR1 = *(uint16_t *)(storedConfig+NV_SAMPLING_RATE);
            TB0CCR2 = *(uint16_t *)(storedConfig+NV_SAMPLING_RATE) + 147;  //295-148
            TB0CCTL1 = CCIE;
         } else {
            TB0CCR0 = *(uint16_t *)(storedConfig+NV_SAMPLING_RATE) + 148;  //148 ticks of 32768Hz clock is 4.5166ms
            TB0CCR2 = *(uint16_t *)(storedConfig+NV_SAMPLING_RATE);
            TB0CCTL1 = 0;
         }
         TB0CCTL2 = CCIE;
      } else if(preSampleBmpPress && (((storedConfig[NV_CONFIG_SETUP_BYTE3]&0x30)>>4)==1)) {
         if(preSampleMpuMag) {
            TB0CCR0 = *(uint16_t *)(storedConfig+NV_SAMPLING_RATE) + 295;  //295 ticks of 32768Hz clock is 9.0027ms
            TB0CCR1 = *(uint16_t *)(storedConfig+NV_SAMPLING_RATE);
            TB0CCR2 = *(uint16_t *)(storedConfig+NV_SAMPLING_RATE) + 49;   //295-246
            TB0CCTL1 = CCIE;
         } else {
            TB0CCR0 = *(uint16_t *)(storedConfig+NV_SAMPLING_RATE) + 246;  //246 ticks of 32768Hz clock is 7.5073ms
            TB0CCR2 = *(uint16_t *)(storedConfig+NV_SAMPLING_RATE);
            TB0CCTL1 = 0;
         }
         TB0CCTL2 = CCIE;
      } else if(preSampleBmpPress && (((storedConfig[NV_CONFIG_SETUP_BYTE3]&0x30)>>4)==2)) {
         TB0CCR0 = *(uint16_t *)(storedConfig+NV_SAMPLING_RATE) + 443;     //443 ticks of 32768Hz clock is 13.519ms
         TB0CCR2 = *(uint16_t *)(storedConfig+NV_SAMPLING_RATE);
         TB0CCTL2 = CCIE;
         if(preSampleMpuMag) {
            TB0CCR1 = *(uint16_t *)(storedConfig+NV_SAMPLING_RATE) + 148;  //443-295
            TB0CCTL1 = CCIE;
         } else TB0CCTL1 = 0;
      } else if(preSampleBmpPress && (((storedConfig[NV_CONFIG_SETUP_BYTE3]&0x30)>>4)==3)) {
         TB0CCR0 = *(uint16_t *)(storedConfig+NV_SAMPLING_RATE) + 836;     //836 ticks of 32768Hz clock is 25.513ms
         TB0CCR2 = *(uint16_t *)(storedConfig+NV_SAMPLING_RATE);
         TB0CCTL2 = CCIE;
         if(preSampleMpuMag) {
            TB0CCR1 = *(uint16_t *)(storedConfig+NV_SAMPLING_RATE) + 541;  //836-295
            TB0CCTL1 = CCIE;
         } else TB0CCTL1 = 0;
      } else {
         TB0CCR0 = *(uint16_t *)(storedConfig+NV_SAMPLING_RATE) + 295;     //295 ticks of 32768Hz clock is 9.0027ms
         TB0CCR1 = *(uint16_t *)(storedConfig+NV_SAMPLING_RATE);
         TB0CCTL1 = CCIE;
         TB0CCTL2 = 0;
      }
   }
   else {
      TB0CCTL1 = 0;
      TB0CCTL2 = 0;
      TB0CCR0 = *(uint16_t *)(storedConfig+NV_SAMPLING_RATE);
   }
   TB0CCTL0 = CCIE;
   StartTB0();
}

inline void SampleTimerStop(void) {
   TB0CCTL1 = 0;
   TB0CCTL2 = 0;
   TB0CCTL0 = 0;
}

#pragma vector=TIMER0_B0_VECTOR
__interrupt void TIMER0_B0_ISR(void)
{
   //main sampling rate control
   TB0CCR0 += *(uint16_t *)(storedConfig+NV_SAMPLING_RATE);
   if(currentBuffer){
      *((uint16_t *)(txBuff1+2)) = GetTB0();
   } else {
      *((uint16_t *)(txBuff0+2)) = GetTB0();
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

#pragma vector=TIMER0_B1_VECTOR
__interrupt void TIMER0_B1_ISR(void) {
   switch(__even_in_range(TB0IV,14)) {
   case  0: break;                        // No interrupt
   case  2:                               // TB0CCR1
      //MPU9150 mag
      TB0CCR1 += *(uint16_t *)(storedConfig+NV_SAMPLING_RATE);
      sampleMpu9150Mag = 1;
      __bic_SR_register_on_exit(LPM3_bits);
      break;
   case  4:                               // TB0CCR2
      //Bmp180 press
      TB0CCR2 += *(uint16_t *)(storedConfig+NV_SAMPLING_RATE);
      sampleBmp180Press = 1;
      __bic_SR_register_on_exit(LPM3_bits);
      break;
   case  6:                               // TB0CCR3
      //Charge status LED
      if(IsLedSet()) {                    //check current status of LED
         TB0CCR3 += 65435;
         ClearLed();
      } else {
         //LED is off
         TB0CCR3 += 100;
          SetLed();
      }
      break;
   case  8:                               // TB0CCR4
      //Control blue LED
      if(sensing) {
         TB0CCR4 += 32768;
          Board_ledToggle(LED_BLUE);
      } else {
         if(P1OUT & BIT2) {               //check current status of blue LED
            TB0CCR4 += 65435;
            Board_ledOff(LED_BLUE);
          } else {
            //LED is off
            TB0CCR4 += 100;
            Board_ledOn(LED_BLUE);
          }
      }
      break;
   case 10: break;                       // reserved
   case 12: break;                       // reserved
   case 14: break;                       // TBIFG overflow handler
   }
}


/**
*** DMA interrupt vector
**/
uint8_t Dma0ConversionDone(void) {
   if(currentBuffer){
      DMA0_repeatTransfer(adcStartPtr, (uint16_t *)(txBuff0+4), nbrAdcChans); //Destination address for next transfer
   } else {
      DMA0_repeatTransfer(adcStartPtr, (uint16_t *)(txBuff1+4), nbrAdcChans); //Destination address for next transfer
   }
   ADC_disable();    //can disable ADC until next time sampleTimer fires (to save power)?
   DMA0_disable();
   streamData = 1;
   return 1;
}


#pragma vector=PORT2_VECTOR
__interrupt void Port2_ISR(void) {

   switch (__even_in_range(P2IV, P2IV_P2IFG7)) {
   //ExG chip1 data ready
   case  P2IV_P2IFG0:
      EXG_dataReadyChip1();
      break;

   //DOCK
   case  P2IV_P2IFG3:
      if(P2IN & BIT3) {
         P2IES |= BIT3;    //look for falling edge
         docked = 1;
         ChargeStatusTimerStop();
         MonitorChargeStatus();
         selectedLed = 0;  //reset to green for when removed from dock
      } else {
         P2IES &= ~BIT3;   //look for rising edge
         docked = 0;
         MonitorChargeStatusStop();
         if(btIsConnected) {
            ChargeStatusTimerStart();
         } else {
            //keep both blinking LEDs in sync
            ClearLed();
            BlinkTimerStart();
         }
      }
      break;

   //CHG_STAT1
   case P2IV_P2IFG6:
      SetChargeStatusLeds();
      if(P2IN & BIT6) {
         P2IES |= BIT6;    //look for falling edge
      } else {
         P2IES &= ~BIT6;   //look for rising edge
      }
      break;

   //CHG_STAT2
   case P2IV_P2IFG7:
      SetChargeStatusLeds();
      if(P2IN & BIT7) {
         P2IES |= BIT7;    //look for falling edge
      } else {
         P2IES &= ~BIT7;   //look for rising edge
      }
      break;

   // Default case
   default: break;
   }
}


/**
*** GSR
**/
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


/**
*** Set the baud rate of the serial bus between the Bluetooth module and the MSP430
*** 11 allowable options: 0=115.2K(default), 1=1200, 2=2400, 3=4800, 4=9600, 5=19.2K,
*** 6=38.4K, 7=57.6K, 8=230.4K, 9=460.8K, 10=921.6K, else revert to default
**/
void SetBtBaudRate(uint8_t rate) {
   switch(rate) {
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


/**
***
 */
// trap isr assignation - put all unused ISR vector here
#pragma vector = RTC_VECTOR, USCI_B1_VECTOR,\
   TIMER0_A0_VECTOR, TIMER0_A1_VECTOR, TIMER1_A1_VECTOR, \
   UNMI_VECTOR, WDT_VECTOR, SYSNMI_VECTOR
__interrupt void TrapIsr(void) {
  // this is a trap ISR - check for the interrupt cause here by
  // checking the interrupt flags, if necessary also clear the interrupt
  // flag
}
