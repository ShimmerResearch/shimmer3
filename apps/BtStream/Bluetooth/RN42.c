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
#include "RN42.h"
#include <string.h>
#include <ctype.h>
#include <stdio.h>

//copied as directly as possible from TinyOS RovingNetworksP.nc
//http://code.google.com/p/tinyos-main/source/browse/trunk/tos/platforms/shimmer/chips/bluetooth/RovingNetworksP.nc

uint8_t messageInProgress, transmissionOverflow, messageLength;
uint8_t messageBuffer[256]; //max possible size
uint8_t receiveBuffer[5];
char expectedCommandResponse[6], newName[17], newPIN[17], newSvcClass[5], newDevClass[5], newSvcName[17], newRawBaudrate[5],
     newBaudrate[5], newInquiryTime[5], newPagingTime[5], newFriendlyName[16];
char commandbuf[32];

uint8_t radioMode, charsSent, charsReceived;

uint8_t discoverable, authenticate, encrypt, setNameRequest, setPINRequest, resetDefaultsRequest, setSvcClassRequest,
        setDevClassRequest, setSvcNameRequest, setRawBaudrate, setBaudrate, disableRemoteConfig, newMode,
        setCustomInquiryTime, setCustomPagingTime, newAuthMode, setFriendlyNameRequest;

//master mode stuff
uint8_t btConnected, deviceConn;
char targetBt[16];

uint8_t (*dataAvailableFuncPtr)(uint8_t data) = 0;

uint8_t txIe;

uint8_t slowRate;

void initRN() {
   // powerup state is reset == low (true); mike conrad of roving networks sez:
   // wait about 1s to 2s after reset toggle
   P4OUT |= BIT4;             //Shimmer3
   __delay_cycles(48000000);  //wait 2s (assuming 24MHz MCLK)

   //UART_RTS interrupt: RTS raises when BT has trans overflow
   P1IES &= ~BIT3;            //can assume initially low as module is in reset so watch for low to high transition
   P1IFG &= ~BIT3;            //clear flag
   P1IE |= BIT3;              //enable interrupt
   //PIO2_CONNECT interrupt
   P1IES &= ~BIT0;            //can assume initially low as module is in reset so watch for low to high transition
   P1IFG &= ~BIT0;            //clear flag
   P1IE |= BIT0;              //enable interrupt

   P2OUT &= ~BIT2;            //toggling cts wakes it up
   P2OUT |= BIT2;
   __delay_cycles(120000);    //wait 5ms (assuming 24MHz MCLK)
   P2OUT &= ~BIT2;
}


void setupUART(char * baudRate) {
   UCA1CTL1 |= UCSWRST;                   //**Put state machine in reset**
   UCA1CTL1 |= UCSSEL_2;                  //SMCLK

   if((strlen(baudRate)==4) && (strncmp(baudRate, "115K", 4))) {
      if(!strncmp(baudRate, "1200", 4)) {
         //1200
         UCA1BR0 = 226;                         //24MHz 38400
         UCA1BR1 = 4;                           //24MHz 38400
         UCA1MCTL = UCBRS_0 + UCBRF_0 + UCOS16; //Modln UCBRSx=0, UCBRFx=0, over sampling
      } else if (!strncmp(baudRate, "2400", 4)) {
         //2400
         UCA1BR0 = 113;                         //24MHz 2400
         UCA1BR1 = 2;                           //24MHz 2400
         UCA1MCTL = UCBRS_0 + UCBRF_0 + UCOS16; //Modln UCBRSx=0, UCBRFx=0, over sampling
      } else if (!strncmp(baudRate, "4800", 4)) {
         //4800
         UCA1BR0 = 56;                          //24MHz 4800
         UCA1BR1 = 1;                           //24MHz 4800
         UCA1MCTL = UCBRS_0 + UCBRF_8 + UCOS16; //Modln UCBRSx=0, UCBRFx=8, over sampling
      } else if (!strncmp(baudRate, "9600", 4)) {
         //9600
         UCA1BR0 = 156;                         //24MHz 9600
         UCA1BR1 = 0;                           //24MHz 9600
         UCA1MCTL = UCBRS_0 + UCBRF_4 + UCOS16; //Modln UCBRSx=0, UCBRFx=4, over sampling
      } else if (!strncmp(baudRate, "19.2", 4)) {
         //19200
         UCA1BR0 = 78;                          //24MHz 19200
         UCA1BR1 = 0;                           //24MHz 19200
         UCA1MCTL = UCBRS_0 + UCBRF_2 + UCOS16; //Modln UCBRSx=0, UCBRFx=2, over sampling
      } else if (!strncmp(baudRate, "38.4", 4)) {
         //38400
         UCA1BR0 = 39;                          //24MHz 38400
         UCA1BR1 = 0;                           //24MHz 38400
         UCA1MCTL = UCBRS_0 + UCBRF_1 + UCOS16; //Modln UCBRSx=0, UCBRFx=1, over sampling
      } else if (!strncmp(baudRate, "57.6", 4)) {
         //57600
         UCA1BR0 = 26;                          //24MHz 57600
         UCA1BR1 = 0;                           //24MHz 57600
         UCA1MCTL = UCBRS_0 + UCBRF_1 + UCOS16; //Modln UCBRSx=0, UCBRFx=1, over sampling
      } else if (!strncmp(baudRate, "230K", 4)) {
         //230400
         UCA1BR0 = 6;                           //24MHz 230400
         UCA1BR1 = 0;                           //24MHz 230400
         UCA1MCTL = UCBRS_0 + UCBRF_8 + UCOS16; //Modln UCBRSx=0, UCBRFx=8, over sampling
      } else if (!strncmp(baudRate, "460K", 4)) {
         //460800
         UCA1BR0 = 3;                           //24MHz 460800
         UCA1BR1 = 0;                           //24MHz 460800
         UCA1MCTL = UCBRS_0 + UCBRF_4 + UCOS16; //Modln UCBRSx=0, UCBRFx=4, over sampling
      } else if (!strncmp(baudRate, "921K", 4)) {
         //921600
         UCA1BR0 = 26;                          //24MHz 921600
         UCA1BR1 = 0;                           //24MHz 921600
         UCA1MCTL = UCBRS_0 + UCBRF_0;          //Modln UCBRSx=0, UCBRFx=0, no over sampling
                                                //(cannot use over sampling as max error would be > 50%)
      }
   } else {
      //115200
      //default
      UCA1BR0 = 13;                          //24MHz 115200
      UCA1BR1 = 0;                           //24MHz 115200
      UCA1MCTL = UCBRS_0 + UCBRF_0 + UCOS16; //Modln UCBRSx=0, UCBRFx=0, over sampling
   }

   UCA1CTL1 &= ~UCSWRST;                  //**Initialize USCI state machine**
   UCA1IFG = 0;                           // reset interrupts
   UCA1IE |= UCTXIE + UCRXIE;             // Enable USCI_A1 TX and RX interrupt
}

void disableUART() {
   UCA1CTL1 |= UCSWRST;                   //Put state machine in reset
   UCA1IE &= ~(UCTXIE + UCRXIE);          //Disable USCI_A1 TX and RX interrupt
}


void disableRN() {
   //hold in reset
   P4OUT &= ~BIT4;                        //Shimmer3 board
   disableUART();
   P1IE &= ~BIT3;                         //disable RTS interrupt
   P1IE &= ~BIT0;                         //disable Connection interrupt
}


//write data to be transmitted to the Bluetooth module
//returns 0 if fails, else 1
//will only fail if a previous BT_write is still in progress
uint8_t writeCommand(char *cmd, char *response) {
   if(messageInProgress)
      return 0;   //fail

   strcpy(expectedCommandResponse, response);
   charsReceived = 0;
   if(BT_write((uint8_t *)cmd, strlen(cmd)))
      return 1;   //success
   else
      return 0;   //fail
}


// Connect and Disconnect commands are exceptional commands in that
// they automatically return to data mode once they are issued
uint8_t writeCommandNoRsp(char * cmd) {
   if(BT_write((uint8_t *)cmd, strlen(cmd)))
      return 1;   //success
   else
      return 0;   //fail
}


// this one is awkward. we need to send one command at a time and wait until the
// response is received until sending the next command
// There are a couple of possible approaches
// - use a stepped switch statement and call this function from the receive ISR
//   once the response is received. But results in overly long ISR, from sending
//   the next command from within the ISR and then "hoping" that the ISR exits
//   before the response comes back (not a problem, but bad design)
// - block on a variable between each step and reset variable in ISR
//   poor design, but this function will only need to be run rarely
// - put MSP430 in low power mode while waiting for response from RN42
//   best option powerwise, but need to ensure no other interrupt will start
//   processor executing while waiting for response. Also need to take out of low
//   power mode to send commands and return to LPM afterwards
uint8_t interrupts0, interrupts1, interrupts2, interrupts4,
      interrupts5, interrupts6, interrupts7, interrupts8;
uint16_t interrupts3w;

void saveInterruptSettings() {
   interrupts0 = 0;
   interrupts1 = 0;
   interrupts2 = 0;
   interrupts4 = 0;
   interrupts5 = 0;
   interrupts6 = 0;
   interrupts7 = 0;
   interrupts8 = 0;
   interrupts3w = 0;

   //==================================================================================================================================
   //Need to check all interrupt sources here (including global IE)
   //save status and disable
   //to ensure only receiving response from RN42 will cause execution to continue
   //==================================================================================================================================

   if(__get_SR_register()&0x0008) interrupts0 |= BIT0; //global interrupts
   //TB0
   if(TB0CTL&TBIE) { //Timer_B interrupt enable
      interrupts0 |= BIT3;
      TB0CTL &= ~TBIE;
   }
   if(TB0CCTL0&CCIE) { //TB0 Capture/Compare 0 interrupt enable
      interrupts0 |= BIT4;
      TB0CCTL0 &= ~CCIE;
   }
   if(TB0CCTL1&CCIE) { //TB0 Capture/Compare 1 interrupt enable
      interrupts0 |= BIT5;
      TB0CCTL1 &= ~CCIE;
   }
   if(TB0CCTL2&CCIE) { //TB0 Capture/Compare 2 interrupt enable
      interrupts0 |= BIT6;
      TB0CCTL2 &= ~CCIE;
   }
   if(TB0CCTL3&CCIE) { //TB0 Capture/Compare 3 interrupt enable
      interrupts0 |= BIT7;
      TB0CCTL3 &= ~CCIE;
   }
   if(TB0CCTL4&CCIE) { //TB0 Capture/Compare 4 interrupt enable
      interrupts1 |= BIT0;
      TB0CCTL4 &= ~CCIE;
   }
   if(TB0CCTL5&CCIE) { //TB0 Capture/Compare 5 interrupt enable
      interrupts1 |= BIT1;
      TB0CCTL5 &= ~CCIE;
   }
   if(TB0CCTL6&CCIE) { //TB0 Capture/Compare 6 interrupt enable
      interrupts1 |= BIT2;
      TB0CCTL6 &= ~CCIE;
   }
   //Watchdog Timer_A interval Timer mode
   if((WDTCTL&WDTTMSEL) && ~(WDTCTL&WDTHOLD)) { //Watchdog timer running and in interval timer mode
      interrupts1 |= BIT3;
      WDTCTL = (WDTCTL & 0x00FF) + WDTPW + WDTHOLD; //Stop Watchdog
   }
   //USCI_A0
   if(UCA0CTL1&UCRXEIE) { //USCI_A0 Receive erroneous-character interrupt enable
      interrupts1 |= BIT4;
      UCA0CTL1 &= ~UCRXEIE;
   }
   if(UCA0CTL1&UCBRKIE) { //USCI_A0 Receive break character interrupt enable
      interrupts1 |= BIT5;
      UCA0CTL1 &= ~UCBRKIE;
   }
   if(UCA0IE&UCTXIE) { //USCI_A0 Transmit interrupt enable
      interrupts1 |= BIT6;
      UCA0IE &= ~UCTXIE;
   }
   if(UCA0IE&UCRXIE) { //USCI_A0 Receive interrupt enable
      interrupts1 |= BIT7;
      UCA0IE &= ~UCRXIE;
   }
   //USCI_B0
   if(UCB0IE&UCNACKIE) { //USCI_B0 Not-acknowledge interrupt enable
      interrupts2 |= BIT0;
      UCB0IE &= ~UCNACKIE;
   }
   if(UCB0IE&UCALIE) { //USCI_B0 Arbitration lost interrupt enable
      interrupts2 |= BIT1;
      UCB0IE &= ~UCALIE;
   }
   if(UCB0IE&UCSTPIE) { //USCI_B0 STOP condition interrupt enable
      interrupts2 |= BIT2;
      UCB0IE &= ~UCSTPIE;
   }
   if(UCB0IE&UCSTTIE) { //USCI_B0 START condition interrupt enable
      interrupts2 |= BIT3;
      UCB0IE &= ~UCSTTIE;
   }
   if(UCB0IE&UCTXIE) { //USCI_B0 Transmit interrupt enable
      interrupts2 |= BIT4;
      UCB0IE &= ~UCTXIE;
   }
   if(UCB0IE&UCRXIE) { //USCI_B0 Receive interrupt enable
      interrupts2 |= BIT5;
      UCB0IE &= ~UCRXIE;
   }
   //ADC12_A
   if(ADC12CTL0&ADC12OVIE) { //ADC12MEMx overflow-interrupt enable
      interrupts2 |= BIT6;
      ADC12CTL0 &= ~ADC12OVIE;
   }
   if(ADC12CTL0&ADC12TOVIE) { //ADC12_A conversion-time-overflow interrupt enable
      interrupts2 |= BIT7;
      ADC12CTL0 &= ~ADC12TOVIE;
   }
   if(ADC12IE) { //ADC12_A interrupt enable register
      interrupts3w = ADC12IE;
      ADC12IE = 0;
   }
   //TA0
   if(TA0CTL&TAIE) { //Timer_A0 interrupt enable
      interrupts4 |= BIT0;
      TA0CTL &= ~TAIE;
   }
   if(TA0CCTL0&CCIE) { //TA0 Capture/Compare 0 interrupt enable
      interrupts4 |= BIT1;
      TA0CCTL0 &= ~CCIE;
   }
   if(TA0CCTL1&CCIE) { //TA0 Capture/Compare 1 interrupt enable
      interrupts4 |= BIT2;
      TA0CCTL1 &= ~CCIE;
   }
   if(TA0CCTL2&CCIE) { //TA0 Capture/Compare 2 interrupt enable
      interrupts4 |= BIT3;
      TA0CCTL2 &= ~CCIE;
   }
   if(TA0CCTL3&CCIE) { //TA0 Capture/Compare 3 interrupt enable
      interrupts4 |= BIT4;
      TA0CCTL3 &= ~CCIE;
   }
   if(TA0CCTL4&CCIE) { //TA0 Capture/Compare 4 interrupt enable
      interrupts4 |= BIT5;
      TA0CCTL4 &= ~CCIE;
   }
   //DMA
   if(DMA0CTL&DMAIE) { //DMA 0 interrupt enable
      interrupts5 |= BIT2;
      DMA0CTL &= ~DMAIE;
   }
   if(DMA1CTL&DMAIE) { //DMA 1 interrupt enable
      interrupts5 |= BIT3;
      DMA1CTL &= ~DMAIE;
   }
   if(DMA2CTL&DMAIE) { //DMA 2 interrupt enable
      interrupts5 |= BIT4;
      DMA2CTL &= ~DMAIE;
   }
   //TA1
   if(TA1CTL&TAIE) { //Timer_A1 interrupt enable
      interrupts5 |= BIT5;
      TA1CTL &= ~TAIE;
   }
   if(TA1CCTL0&CCIE) { //TA1 Capture/Compare 0 interrupt enable
      interrupts5 |= BIT6;
      TA1CCTL0 &= ~CCIE;
   }
   if(TA1CCTL1&CCIE) { //TA1 Capture/Compare 1 interrupt enable
      interrupts5 |= BIT7;
      TA1CCTL1 &= ~CCIE;
   }
   if(TA1CCTL2&CCIE) { //TA1 Capture/Compare 2 interrupt enable
      interrupts6 |= BIT0;
      TA1CCTL2 &= ~CCIE;
   }
   //I/O Port P1
   if(P1IE&0xF9) { //Port 1 interrupt enable
      interrupts7 = P1IE;
      P1IE &= 0x06; //preserve BT_PIO and BT_RTS setting
   }
   //USCI_B1
   if(UCB1IE&UCNACKIE) { //USCI_B1 Not-acknowledge interrupt enable
      interrupts6 |= BIT1;
      UCB1IE &= ~UCNACKIE;
   }
   if(UCB1IE&UCALIE) { //USCI_B1 Arbitration lost interrupt enable
      interrupts6 |= BIT2;
      UCB1IE &= ~UCALIE;
   }
   if(UCB1IE&UCSTPIE) { //USCI_B1 STOP condition interrupt enable
      interrupts6 |= BIT3;
      UCB1IE &= ~UCSTPIE;
   }
   if(UCB1IE&UCSTTIE) { //USCI_B1 START condition interrupt enable
      interrupts6 |= BIT4;
      UCB1IE &= ~UCSTTIE;
   }
   if(UCB1IE&UCTXIE) { //USCI_B1 Transmit interrupt enable
      interrupts6 |= BIT5;
      UCB1IE &= ~UCTXIE;
   }
   if(UCB1IE&UCRXIE) { //USCI_B1 Receive interrupt enable
      interrupts6 |= BIT6;
      UCB1IE &= ~UCRXIE;
   }
   //I/O Port P2
   if(P2IE) { //Port 2 interrupt enable
      interrupts8 = P2IE;
      P2IE = 0;
   }
   //RTC_A
   if(RTCCTL0&RTCTEVIE) { //RTC time event interrupt enable
      interrupts0 |= BIT1;
      RTCCTL0 &= ~RTCTEVIE;
   }
   if(RTCCTL0&RTCAIE) { //RTC alarm interrupt enable
      interrupts0 |= BIT2;
      RTCCTL0 &= ~RTCAIE;
   }
   if(RTCCTL0&RTCRDYIE) { //RTC read ready interrupt enable
      interrupts4 |= BIT6;
      RTCCTL0 &= ~RTCRDYIE;
   }
   if(RTCPS0CTL&RT0PSIE) { //RTC prescale timer 0 interrupt enable
      interrupts4 |= BIT7;
      RTCPS0CTL &= ~RT0PSIE;
   }
   if(RTCPS1CTL&RT1PSIE) { //RTC prescale timer 1 interrupt enable
      interrupts5 |= BIT0;
      RTCPS1CTL &= ~RT1PSIE;
   }
}

void restoreInterruptSettings() {
   //==================================================================================================================================
   //restore interrupt settings
   //==================================================================================================================================
   if(!(interrupts0 & BIT0)) _disable_interrupts(); //global interrupts
   //TB0
   if(interrupts0&BIT3) TB0CTL |= TBIE;      //Timer_B interrupt enable
   if(interrupts0&BIT4) TB0CCTL0 |= CCIE;    //TB0 Capture/Compare 0 interrupt enable
   if(interrupts0&BIT5) TB0CCTL1 |= CCIE;    //TB0 Capture/Compare 1 interrupt enable
   if(interrupts0&BIT6) TB0CCTL2 |= CCIE;    //TB0 Capture/Compare 2 interrupt enable
   if(interrupts0&BIT7) TB0CCTL3 |= CCIE;    //TB0 Capture/Compare 3 interrupt enable
   if(interrupts1&BIT0) TB0CCTL4 |= CCIE;    //TB0 Capture/Compare 4 interrupt enable
   if(interrupts1&BIT1) TB0CCTL5 |= CCIE;    //TB0 Capture/Compare 5 interrupt enable
   if(interrupts1&BIT2) TB0CCTL6 |= CCIE;    //TB0 Capture/Compare 6 interrupt enable
   //Watchdog Timer_A interval Timer mode
   if(interrupts1&BIT3) WDTCTL = (WDTCTL & 0x007F) + WDTPW; //Restart Watchdog (no need to clear counter as is in interval timer mode)
   //USCI_A0
   if(interrupts1&BIT4) UCA0CTL1 |= UCRXEIE; //USCI_A0 Receive erroneous-character interrupt enable
   if(interrupts1&BIT5) UCA0CTL1 |= UCBRKIE; //USCI_A0 Receive break character interrupt enable
   if(interrupts1&BIT6) UCA0IE |= UCTXIE;    //USCI_A0 Transmit interrupt enable
   if(interrupts1&BIT7) UCA0IE |= UCRXIE;    //USCI_A0 Receive interrupt enable
   //USCI_B0
   if(interrupts2&BIT0) UCB0IE |= UCNACKIE;  //USCI_B0 Not-acknowledge interrupt enable
   if(interrupts2&BIT1) UCB0IE |= UCALIE;    //USCI_B0 Arbitration lost interrupt enable
   if(interrupts2&BIT2) UCB0IE |= UCSTPIE;   //USCI_B0 STOP condition interrupt enable
   if(interrupts2&BIT3) UCB0IE |= UCSTTIE;   //USCI_B0 START condition interrupt enable
   if(interrupts2&BIT4) UCB0IE |= UCTXIE;    //USCI_B0 Transmit interrupt enable
   if(interrupts2&BIT5) UCB0IE |= UCRXIE;    //USCI_B0 Receive interrupt enable
   //ADC12_A
   if(interrupts2&BIT6) ADC12CTL0 |= ADC12OVIE;    //ADC12MEMx overflow-interrupt enable
   if(interrupts2&BIT7) ADC12CTL0 |= ADC12TOVIE;   //ADC12_A conversion-time-overflow interrupt enable
   if(interrupts3w) ADC12IE = interrupts3w;        //ADC12_A interrupt enable register
   //TA0
   if(interrupts4&BIT0) TA0CTL   |= TAIE;    //Timer_A interrupt enable
   if(interrupts4&BIT1) TA0CCTL0 |= CCIE;    //TA0 Capture/Compare 0 interrupt enable
   if(interrupts4&BIT2) TA0CCTL1 |= CCIE;    //TA0 Capture/Compare 1 interrupt enable
   if(interrupts4&BIT3) TA0CCTL2 |= CCIE;    //TA0 Capture/Compare 2 interrupt enable
   if(interrupts4&BIT4) TA0CCTL3 |= CCIE;    //TA0 Capture/Compare 3 interrupt enable
   if(interrupts4&BIT5) TA0CCTL4 |= CCIE;    //TA0 Capture/Compare 4 interrupt enable
   //DMA
   if(interrupts5&BIT2) DMA0CTL |= DMAIE;    //DMA 0 interrupt enable
   if(interrupts5&BIT3) DMA1CTL |= DMAIE;    //DMA 0 interrupt enable
   if(interrupts5&BIT4) DMA2CTL |= DMAIE;    //DMA 0 interrupt enable
   //TA1
   if(interrupts5&BIT5) TA1CTL   |= TAIE;    //Timer_A1 interrupt enable
   if(interrupts5&BIT6) TA1CCTL0 |= CCIE;    //TA1 Capture/Compare 0 interrupt enable
   if(interrupts5&BIT7) TA1CCTL1 |= CCIE;    //TA1 Capture/Compare 1 interrupt enable
   if(interrupts6&BIT0) TA1CCTL2 |= CCIE;    //TA1 Capture/Compare 2 interrupt enable
   //I/O Port P1
   if(interrupts7&0xF9) P1IE = interrupts7;  //Port 1 interrupt enable
   //USCI_B1
   if(interrupts6&BIT1) UCB1IE |= UCNACKIE;  //USCI_B1 Not-acknowledge interrupt enable
   if(interrupts6&BIT2) UCB1IE |= UCALIE;    //USCI_B1 Arbitration lost interrupt enable
   if(interrupts6&BIT3) UCB1IE |= UCSTPIE;   //USCI_B1 STOP condition interrupt enable
   if(interrupts6&BIT4) UCB1IE |= UCSTTIE;   //USCI_B1 START condition interrupt enable
   if(interrupts6&BIT5) UCB1IE |= UCTXIE;    //USCI_B1 Transmit interrupt enable
   if(interrupts6&BIT6) UCB1IE |= UCRXIE;    //USCI_B1 Receive interrupt enable
   //I/O Port P2
   if(interrupts8) P2IE = interrupts8;       //Port 2 interrupt enable
   //RTC_A
   if(interrupts0&BIT1) RTCCTL0   |= RTCTEVIE;  //RTC time event interrupt enable
   if(interrupts0&BIT2) RTCCTL0   |= RTCAIE;    //RTC alarm interrupt enable
   if(interrupts4&BIT6) RTCCTL0   |= RTCRDYIE;  //RTC read ready interrupt enable
   if(interrupts4&BIT7) RTCPS0CTL |= RT0PSIE;   //RTC prescale timer 0 interrupt enable
   if(interrupts5&BIT0) RTCPS1CTL |= RT1PSIE;   //RTC prescale timer 1 interrupt enable
}

void runSetCommands() {
   saveInterruptSettings();

   writeCommand("$$$", "CMD\r\n");
   __bis_SR_register(LPM3_bits + GIE);    //wait until response is received

   //reset factory defaults
   if(resetDefaultsRequest) {
      writeCommand("SF,1\r", "AOK\r\n");
      __bis_SR_register(LPM3_bits + GIE); //wait until response is received
   }

   //default is slave (== 0), otherwise set mode
   if(newMode) {
      sprintf(commandbuf, "SM,%d\r", radioMode);
      writeCommand(commandbuf, "AOK\r\n");
      __bis_SR_register(LPM3_bits + GIE); //wait until response is received
   }

   //device is discoverable with a non-zero inquiry scan window
   //default "time" is 0x0200 (units unspecified)
   if(!discoverable) {
      writeCommand("SI,0000\r", "AOK\r\n");
      __bis_SR_register(LPM3_bits + GIE); //wait until response is received
   }

   // device default is off
   if(authenticate) {
      sprintf(commandbuf, "SA,%d\r", newAuthMode);
      writeCommand(commandbuf, "AOK\r\n");
      __bis_SR_register(LPM3_bits + GIE); //wait until response is received
   }

   // device default is off
   if(encrypt) {
      writeCommand("SE,1\r", "AOK\r\n");
      __bis_SR_register(LPM3_bits + GIE); //wait until response is received
   }

   // default is none
   if(setNameRequest) {
      sprintf(commandbuf, "SN,%s\r", newName);
      writeCommand(commandbuf, "AOK\r\n");
      __bis_SR_register(LPM3_bits + GIE); //wait until response is received
   }

   // default is none
   if(setFriendlyNameRequest) {
      sprintf(commandbuf, "S-,%s\r", newFriendlyName);
      writeCommand(commandbuf, "AOK\r\n");
      __bis_SR_register(LPM3_bits + GIE); //wait until response is received
   }


   // default is none
   if(setPINRequest) {
      sprintf(commandbuf, "SP,%s\r", newPIN);
      writeCommand(commandbuf, "AOK\r\n");
      __bis_SR_register(LPM3_bits + GIE); //wait until response is received
   }

   if(setSvcClassRequest) {
      sprintf(commandbuf, "SC,%s\r", newSvcClass);
      writeCommand(commandbuf, "AOK\r\n");
      __bis_SR_register(LPM3_bits + GIE); //wait until response is received
   }

   if(setDevClassRequest) {
      sprintf(commandbuf, "SD,%s\r", newDevClass);
      writeCommand(commandbuf, "AOK\r\n");
      __bis_SR_register(LPM3_bits + GIE); //wait until response is received
   }

   if(setSvcNameRequest) {
      sprintf(commandbuf, "SS,%s\r", newSvcName);
      writeCommand(commandbuf, "AOK\r\n");
      __bis_SR_register(LPM3_bits + GIE); //wait until response is received
   }

   if(setRawBaudrate) {
      //set the baudrate to suit the MSP430 running at 8Mhz
      sprintf(commandbuf, "SZ,%s\r", newRawBaudrate);
      writeCommand(commandbuf, "AOK\r\n");
      __bis_SR_register(LPM3_bits + GIE); //wait until response is received
   }

   if(disableRemoteConfig) {
      //disable remote configuration to enhance throughput
      writeCommand("ST,0\r", "AOK\r\n");
   } else {
      //disable remote configuration to enhance throughput
      writeCommand("ST,60\r", "AOK\r\n");
   }
   __bis_SR_register(LPM3_bits + GIE);    //wait until response is received

   if(setCustomInquiryTime) {
      sprintf(commandbuf, "SI,%s\r", newInquiryTime);
      writeCommand(commandbuf, "AOK\r\n");
   } else {
      //to save power only leave inquiry on for approx 40msec (every 1.28 secs)
      writeCommand("SI,0040\r", "AOK\r\n");
   }
   __bis_SR_register(LPM3_bits + GIE);    //wait until response is received

   if(setCustomPagingTime) {
      sprintf(commandbuf, "SJ,%s\r", newPagingTime);
      writeCommand(commandbuf, "AOK\r\n");
   } else {
      //to save power only leave paging on for approx 80msec (every 1.28 secs)
      writeCommand("SJ,0080\r", "AOK\r\n");
   }
   __bis_SR_register(LPM3_bits + GIE);    //wait until response is received

   if(setBaudrate) {
      //set the baudrate to suit the MSP430
      sprintf(commandbuf, "SU,%s\r", newBaudrate);
      writeCommand(commandbuf, "AOK\r\n");
      __bis_SR_register(LPM3_bits + GIE); //wait until response is received
   }

   //exit command mode
   writeCommand("---\r", "END\r\n");
   __bis_SR_register(LPM3_bits + GIE);    //wait until response is received

   restoreInterruptSettings();
}


// IMPORTANT: Connect and Disconnect commands are exceptional commands
// in that they automatically return to data mode once they are issued
// so no response and no "---" needed to return to data mode
void runMasterCommands() {
   saveInterruptSettings();

   writeCommand("$$$", "CMD\r\n");
   __bis_SR_register(LPM3_bits + GIE); //wait until response is received

   //Connect
   if(deviceConn && (!btConnected)){  //Connect
      sprintf(commandbuf, "C,%s\r", targetBt);
      writeCommandNoRsp(commandbuf);
   } else if((!deviceConn) && (btConnected)) { //Disconnect
      writeCommandNoRsp("K,\r");
   } else { //exit command mode
      //not needed for connect and disconnect commands */
      writeCommand("---\r", "END\r\n");
      __bis_SR_register(LPM3_bits + GIE); //wait until response is received
      deviceConn = 0;
   }

   restoreInterruptSettings();
}


void sendNextChar() {
   if(charsSent < messageLength) {
      while (UCA1IFG & UCTXIFG);    //ensure no tx interrupt is pending
      UCA1TXBUF = *(messageBuffer + charsSent++);
   } else {
      messageInProgress = 0;        //false
   }
}


void BT_init() {
   messageInProgress = transmissionOverflow = 0; //false

   //Turn on power (SW_BT P4.3 on Shimmer3)
   P4OUT |= BIT3;

   newMode = 0;
   radioMode = SLAVE_MODE;
   discoverable = 1;
   authenticate = 0;
   encrypt = 0;
   resetDefaultsRequest = 0;
   setNameRequest = 0;
   setFriendlyNameRequest = 0;
   setPINRequest = 0;
   setSvcClassRequest = 0;
   setSvcNameRequest = 0;
   setDevClassRequest = 0;
   setRawBaudrate = 0;
   disableRemoteConfig = 0;
   setCustomInquiryTime = 0;
   setCustomPagingTime = 0;
   setBaudrate = 0;
   newAuthMode = 0;
   txIe = 0;
   slowRate = 0;

   //connect/disconnect commands
   deviceConn = btConnected = 0;

   *expectedCommandResponse = '\0';   //NULL pointer
   charsReceived = 0;
   transmissionOverflow = messageInProgress = 0;

   initRN();

   setupUART("115K");
}


void BT_configure() {
   __delay_cycles(360000); //wait for 15ms (assuming 24MHz MCLK) why???
   runSetCommands();
}

void BT_disable() {
   disableRN();
   //Turn off power (SW_BT P4.3 on Shimmer3)
   P4OUT &= ~BIT3;
}


//write data to be transmitted to the Bluetooth module
//returns 0 if fails, else 1
//will only fail if a previous BT_write is still in progress
uint8_t BT_write(uint8_t *buf, uint8_t len) {
   if(messageInProgress)
      return 0;   //fail

   charsSent = 0;
   memcpy(messageBuffer, buf, len);
   messageLength = len;

   if(!transmissionOverflow) {
      messageInProgress = 1;
      sendNextChar();
      return 1;   //success
   } else {
      return 0;   //fail
   }
}


uint8_t BT_connect(uint8_t *addr) {
   deviceConn = 1;   //True
   strcpy(targetBt, (const char *)addr);
   runMasterCommands();
   return 1;         //success
}


uint8_t BT_disconnect() {
    //Delay: If any bytes are seen before or after $$$ in a 1
    //second window, command mode will not be entered and these
    //bytes will be passed on to other side
    __delay_cycles(288000000);   //wait 8s (assuming 24MHz MCLK)

    deviceConn = 0;
    runMasterCommands();
    return 1;
}


void BT_setRadioMode(uint8_t mode) {
   newMode = 1;
   radioMode = mode;
}


void BT_setDiscoverable(uint8_t disc) {
   discoverable = disc;
}


void BT_setEncryption(uint8_t enc) {
   encrypt = enc;
}


void BT_setAuthentication(uint8_t mode) {
   authenticate = 1;
   newAuthMode = mode;
}


void BT_setName(char * name) {
   setNameRequest = 1;
   snprintf(newName, 17, "%s", name);
}


void BT_setFriendlyName(char * name) {
   setFriendlyNameRequest = 1;
   snprintf(newFriendlyName, 16, "%s", name);
}


void BT_setPIN(char * PIN) {
   setPINRequest = 1;
   snprintf(newPIN, 17, "%s", PIN);
}


void BT_setServiceClass(char * class) {
   setSvcClassRequest = 1;
   snprintf(newSvcClass, 5, "%s", class);
}


void BT_setServiceName(char * name) {
   setSvcNameRequest = 1;
   snprintf(newSvcName, 5, "%s", name);
}


void BT_setDeviceClass(char * class){
   setDevClassRequest = 1;
   snprintf(newDevClass, 5, "%s", class);
}


void BT_disableRemoteConfig(uint8_t disableConfig) {
   disableRemoteConfig = disableConfig;
}


//this one makes sense only to roving networks
//the supplied "rate_factor" is the baudrate * 0.004096
//this factor must be an integer value...
void BT_setRawBaudrate(char * rate_factor) {
   setRawBaudrate = 1;
   snprintf(newRawBaudrate, 5, "%s", rate_factor);
}


//to set the baudrate of the BT to MSP serial interface
//as per RovingNetworks command spec EG "SU,96" or "SU,230"
//SU,<rate> - Baudrate, {1200, 2400, 4800, 9600, 19.2,
//38.4, 57.6, 115K, 230K, 460K, 921K }
void BT_setBaudrate(char * new_baud) {
   setBaudrate = 1;
   snprintf(newBaudrate, 5, "%s", new_baud);
}


//Sets the Paging Scan Window - amount of time device
//spends enabling page scan (connectability).
//Minimum = (hex word) "0012", corresponding to about 1% duty cycle.
//Maximum = (hex word) "1000"
void BT_setPagingTime(char * hexval_time) {
   setCustomPagingTime = 1;
   snprintf(newPagingTime, 5, "%s", hexval_time);
}


//Sets the Inquiry Scan Window - amount of time device
//spends enabling inquiry scan (discoverability).
//Minimum = (hex word) "0012", corresponding to about 1% duty cycle.
//Maximum = (hex word) "1000"
void BT_setInquiryTime(char * hexval_time) {
   setCustomInquiryTime = 1;
   snprintf(newInquiryTime, 5, "%s", hexval_time);
}


void BT_resetDefaults() {
   resetDefaultsRequest = 1;
}


void BT_setTempBaudRate(char * baudRate) {
   if((strlen(baudRate)==4) && (!strncmp(baudRate, "1200", 4) || !strncmp(baudRate, "2400", 4) ||
         !strncmp(baudRate, "4800", 4) || !strncmp(baudRate, "9600", 4) || !strncmp(baudRate, "19.2", 4) ||
         !strncmp(baudRate, "38.4", 4) || !strncmp(baudRate, "57.6", 4) || !strncmp(baudRate, "115K", 4) ||
         !strncmp(baudRate, "230K", 4) || !strncmp(baudRate, "460K", 4) || !strncmp(baudRate, "921K", 4))) {

      saveInterruptSettings();

      writeCommand("$$$", "CMD\r\n");
      __bis_SR_register(LPM3_bits + GIE); //wait until response is received

      //Connect
      sprintf(commandbuf, "U,%s,N\r", baudRate);

      //if existing rate is 1200 or 2400 baud then something is going wrong with reading "AOK\r\n" response
      //from the RN42
      //So instead of reading response and then continuing, wait 200ms and then continue
      //("U,XXXX\r" and "AOK\r\n" is 12 characters = 120 bit @ 1200baud = 100ms
      //Double this for some processing margin)
      //TODO
      if(slowRate) {
         UCA1IE &= ~UCRXIE;                  //Disable USCI_A1 RX interrupt
         writeCommandNoRsp(commandbuf);
         __delay_cycles(4800000);            //wait 200ms (assuming 24MHz MCLK)
         UCA1IFG &= ~(UCSTTIFG + UCRXIFG);   //clear pending receive interrupts?
         UCA1IE |= UCRXIE;                   //Enable USCI_A1 RX interrupt
      } else {
         writeCommand(commandbuf, "AOK\r\n");
         __bis_SR_register(LPM3_bits + GIE); //wait until response is received
      }

      restoreInterruptSettings();

      //change MSP430 UART to use new baud rate
      setupUART(baudRate);
      if(!strncmp(baudRate, "1200", 4) || !strncmp(baudRate, "2400", 4)) {
         slowRate = 1;
      } else {
         slowRate = 0;
      }
   }
}


void BT_receiveFunction(uint8_t (*receiveFuncPtr)(uint8_t data)) {
   dataAvailableFuncPtr = receiveFuncPtr;
}


void BT_connectionInterrupt(uint8_t value) {
   btConnected = value;
}


void BT_rtsInterrupt(uint8_t value) {
   transmissionOverflow = value;
   if(transmissionOverflow) {
      //disable sending
      txIe = UCA1IE & UCTXIE;
      UCA1IE &= ~UCTXIE;
   } else {
      //re-enable sending if appropriate
      if(txIe)
         UCA1IE |= UCTXIE;
   }
}


#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void) {
   switch(__even_in_range(UCA1IV,4)) {
   case 0:break;                          //Vector 0 - no interrupt
   case 2:                                //Vector 2 - RXIFG
      if(!*expectedCommandResponse && btConnected) {
         if(dataAvailableFuncPtr) {       // ensure this has been set
            if((*dataAvailableFuncPtr)(UCA1RXBUF))
               __bic_SR_register_on_exit(LPM3_bits);
         }
      } else {
         receiveBuffer[charsReceived++] = UCA1RXBUF;
         if(charsReceived == strlen(expectedCommandResponse)) {
            if(!memcmp(receiveBuffer, expectedCommandResponse, charsReceived)) {
               //continue
               *expectedCommandResponse = '\0';
               __bic_SR_register_on_exit(LPM3_bits);
            }
         }
      }
      break;
   case 4:                                //Vector 4 - TXIFG
      if(!transmissionOverflow) {
         sendNextChar();
      }
      break;
   default: break;
   }
}
