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
 * @date March, 2015
 *
 */
#include "hal_uart0.h"
#include "hal_UCA0.h"
#include "msp430.h"
#include "hal_Board.h"
#include "hal_CRC.h"
#include <string.h>

uint8_t uart_messageBuffer[137];
uint8_t uart_messageInProgress, uart_charsSent, uart_messageLength, uart_isr_number_uart;
uint8_t (*uartCallbackFunc)(uint8_t data);

void uartRxEnable(){   UARTIE |= UCRXIE;}
void uartTxEnable(){   UARTIE |= UCTXIE;}
void uartRxDisable(){   UARTIE &= ~UCRXIE;}
void uartTxDisable(){   UARTIE &= ~UCTXIE;}

void UART_init(uint8_t (*uart_cb)(uint8_t data)){
   uartCallbackFunc = uart_cb;
   //uart_num_registered_cmds=0;
}

void UART_config(){
   UARTSEL |= UARTTXD+UARTRXD;

   UARTCTL1 |= UCSWRST;                      // **Put state machine in reset**
   UARTCTL0 = 0;
   UARTCTL1 |= UCSSEL_2;                     // SMCLK

   //UARTBR0 = 0x03;                           // 24MHz 460800
   //UARTBR1 = 0;
   //UARTMCTL =  UCBRS_0 + UCBRF_4 + UCOS16;   // Modln UCBRSx=0, UCBRFx=0, over sampling
   //UARTBR0 = 0x06;                           // 24MHz 230400
   //UARTBR1 = 0;
   //UARTMCTL =  UCBRS_0 + UCBRF_8 + UCOS16;   // Modln UCBRSx=0, UCBRFx=0, over sampling
   UARTBR0 = 0x0d;                           // 24MHz 115200
   UARTBR1 = 0;
   UARTMCTL =  UCBRS_0 + UCBRF_0 + UCOS16;   // Modln UCBRSx=0, UCBRFx=0, over sampling
   //UARTBR0 = 156;                            // 24MHz 9600
   //UARTBR1 = 0;
   //UARTMCTL =  UCBRS_0 + UCBRF_4 + UCOS16;   // Modln UCBRSx=0, UCBRFx=0, over sampling

   UARTCTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**

   uart_messageInProgress = 0;
   UARTIFG=0;
   uartRxEnable();
   uartTxEnable();
}

void uartSendNextChar() {
   if(uart_charsSent < uart_messageLength){
      while (UARTIFG & UCTXIFG); //ensure no tx interrupt is pending
      UARTTXBUF = *(uart_messageBuffer + uart_charsSent++);
   }
   else
      uart_messageInProgress = 0; //false
}

void UART_write(uint8_t *buf, uint8_t len) {
   if(uart_messageInProgress)
      return;   //fail

   uart_charsSent = 0;
   memcpy(uart_messageBuffer, buf, len);
   uart_messageLength = len;

   uart_messageInProgress = 1;
   uartSendNextChar();
}

uint8_t uartUca0TxIsr(){
   uartSendNextChar();
   return 0;
}

uint8_t uartUca0RxIsr(){
   uint8_t rx_char = UARTRXBUF;
   if(uartCallbackFunc)
      return uartCallbackFunc(rx_char);
   return 0;
}

//void UART_reg2Uca0(){
//   uart_isr_number_uart = UCA0_isrRegister(uartUca0RxIsr, uartUca0TxIsr);
//}

void UART_deactivate(){
   //P6SEL |= BIT1;
   P6DIR &= ~BIT1;

   //P7SEL |= BIT6;
   P7DIR &= ~BIT6;

   //P3SEL &= ~BIT3;
   //P3DIR |= BIT3;
   //P3OUT &= ~BIT3;
}

void UART_activate(){
   P6SEL &= ~BIT1;
   P6DIR |= BIT1;
   P6OUT |= BIT1;

   P7SEL &= ~BIT6;
   P7DIR |= BIT6;
   P7OUT |= BIT6;

   //P3SEL &= ~BIT3;
   //P3DIR |= BIT3;
   //P3OUT |= BIT3;

   UCA0_isrActivate(UCA0_isrRegister(uartUca0RxIsr, uartUca0TxIsr));

   UART_config();
}
//============================== above is the UART part ===============================



