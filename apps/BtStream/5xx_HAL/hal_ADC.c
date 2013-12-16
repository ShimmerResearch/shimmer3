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

#include "msp430.h"
#include "hal_ADC.h"
#include "../shimmer.h"

uint8_t (*reportConversionDoneFuncPtr)(void) = 0;

//Use MODOSC as ADC clock source as it is available during LPM3
//According to datasheet (SLAS655C) MODOSC varies from 4.2MHz to 5.4Mhz with 4.8MHz as a typical value
//
//tsample is calculated by the formula in section 28.2.5.3 of the userguide (slau208k.pdf)
//i.e. tsample > (Rout + 1.8kOhms) * 9.01091 * 25pF + 800ns
//Max Rout for Kionix KXRB5 is 40kOhms
//so tsample must be greater than 10.22us (= 55.19 ticks @5.4MHz)
//
//To keep things simple set ADC12SHT1 and ADC12SHT0 to 256 ADC12CLK cycles
//Also found, experimentally, that if SHT is less than 256 for VBatt then the reading has not settled
//i.e. looks like crosstalk from the previously sampled channel
uint16_t* ADC_init(uint16_t mask) {
   if(!mask)
      return 0;

   volatile unsigned char *memCtrlReg = &ADC12MCTL0;

   ADC12CTL0 &= ~ADC12ENC;                            //ensure is off so all ADC12CTL0 and ADC12CTL1 fields can be modified
   ADC12CTL0 = ADC12SHT1_8 + ADC12SHT0_8 + ADC12MSC;  //SHT1 and SHT0 of 256 ADC12CLK cycles, enable multiple sample and conversion
   ADC12CTL1 = ADC12SHP + ADC12CONSEQ_1;              //use sampling timer, ADC12OSC and sequence-of-channels mode
   ADC12CTL2 = ADC12TCOFF + ADC12RES_2 + ADC12SR_L;   //turn off temperature sensor (saves power), ADC12 resolution to 12bits,
                                                      //sampling rate to 50ksps (saves power)

   ADC12CTL1 += ADC12CSTARTADD_1;                     //Start with ADC12MEM1

   if(mask&MASK_A_ACCEL) {
      *(++memCtrlReg) = ADC12INCH_3;
      *(++memCtrlReg) = ADC12INCH_4;
      *(++memCtrlReg) = ADC12INCH_5;
   }
   if(mask&MASK_VBATT) {
      *(++memCtrlReg) = ADC12INCH_2;
   }
   if(mask&MASK_EXT_A7) {
      *(++memCtrlReg) = ADC12INCH_7;
   }
   if(mask&MASK_EXT_A6) {
      *(++memCtrlReg) = ADC12INCH_6;
   }
   if(mask&MASK_EXT_A15) {
      *(++memCtrlReg) = ADC12INCH_15;
   }
   if(mask&MASK_INT_A12) {
      *(++memCtrlReg) = ADC12INCH_12;
   }
   if(mask&MASK_INT_A13) {
      *(++memCtrlReg) = ADC12INCH_13;
   }
   if(mask&MASK_INT_A14) {
      *(++memCtrlReg) = ADC12INCH_14;
   }
   if(mask&MASK_INT_A1) {
      //needs to be last (for gsr)
      *(++memCtrlReg) = ADC12INCH_1;
   }

   *memCtrlReg += ADC12EOS;

   return (uint16_t *)&ADC12MEM1;
}

void ADC_startConversion(void) {
   ADC12CTL0 |= ADC12ON + ADC12ENC + ADC12SC;         //ADC on, enable conversion, start conversion
}

//see section 28.2.7.6 of userguide (slau208k.pdf) for information on stopping conversion
//stops conversion mode immediately (if in progress), conversion data are unreliable and disable ADC (to save power)
void ADC_disable(void) {
   uint16_t ctl1 = ADC12CTL1;
   ADC12CTL1 &= ~(ADC12CONSEQ0 + ADC12CONSEQ1);
   ADC12CTL0 &= ~(ADC12SC + ADC12ENC);
   ADC12CTL0 &= ~ADC12ON;
   ADC12CTL1 = ctl1 & 0xFFFE;                         //don't try to write ADC12BUSY flag
}

void ADC_setInterrupts(uint16_t mask) {
   ADC12IV = 0;
   ADC12IFG = 0;
   ADC12IE = mask;
}

void ADC_conversionDoneFunction(uint8_t (*conversionDoneFuncPtr)(void)) {
   reportConversionDoneFuncPtr = conversionDoneFuncPtr;
}

#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void) {
   switch (__even_in_range(ADC12IV, ADC12IV_ADC12IFG15)) {
   //Only one interrupt should be activated at a time
   case  ADC12IV_NONE:           //No interrupt
      break;
   case  ADC12IV_ADC12OVIFG:     //ADC overflow
      break;
   case  ADC12IV_ADC12TOVIFG:    //ADC timing overflow
      break;
   case  ADC12IV_ADC12IFG0:      //ADC12IFG0
      break;
   case  ADC12IV_ADC12IFG1:      //ADC12IFG1
   case  ADC12IV_ADC12IFG2:      //ADC12IFG2
   case  ADC12IV_ADC12IFG3:      //ADC12IFG3
   case  ADC12IV_ADC12IFG4:      //ADC12IFG4
   case  ADC12IV_ADC12IFG5:      //ADC12IFG5
   case  ADC12IV_ADC12IFG6:      //ADC12IFG6
   case  ADC12IV_ADC12IFG7:      //ADC12IFG7
   case  ADC12IV_ADC12IFG8:      //ADC12IFG8
   case  ADC12IV_ADC12IFG9:      //ADC12IFG9
   case  ADC12IV_ADC12IFG10:     //ADC12IFG10
   case  ADC12IV_ADC12IFG11:     //ADC12IFG11
      if(reportConversionDoneFuncPtr) {   // ensure this has been set
         if((*reportConversionDoneFuncPtr)())
            __bic_SR_register_on_exit(LPM3_bits);
      }
      break;
   case  ADC12IV_ADC12IFG12:     //ADC12IFG12
      break;
   case  ADC12IV_ADC12IFG13:     //ADC12IFG13
      break;
   case  ADC12IV_ADC12IFG14:     //ADC12IFG14
      break;
   case  ADC12IV_ADC12IFG15:     //ADC12IFG15
      break;
   default:
      break;
   }
}
