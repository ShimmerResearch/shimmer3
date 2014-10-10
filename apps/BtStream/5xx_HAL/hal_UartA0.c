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
 * @author Weibo Pan
 * @date June, 2014
 */

#include "hal_UartA0.h"
#include "hal_UCA0.h"
#include "msp430.h"
#include "hal_Board.h"
#include "hal_CRC.h"
#include <string.h>


#define MAX_CMD         7
// a command must be 3 chars + $, e.g. mac$, ver$
#define CMD_LEN         4

// =================================== circular buffer start ==================================
#define CBUF_SIZE    36
#define PARAM_LEN_MAX   CBUF_SIZE-4

struct
{
   uint8_t idx;
   uint8_t entry[ CBUF_SIZE ];
}ucBuf;


void CBUF_init(){
   ucBuf.idx = 0;
}

void CBUF_push(uint8_t elem){
   ucBuf.entry[ucBuf.idx++] = elem;
   if(ucBuf.idx >= CBUF_SIZE)
      ucBuf.idx = 0;
}

void CBUF_formCmd(uint8_t * dst_buf){
   if(ucBuf.idx >= CMD_LEN){
      memcpy(dst_buf, &ucBuf.entry[ucBuf.idx-CMD_LEN], CMD_LEN);
   }
   else{
      memcpy(dst_buf, &ucBuf.entry[CBUF_SIZE + ucBuf.idx-CMD_LEN], CMD_LEN-ucBuf.idx);
      memcpy(dst_buf+CMD_LEN-ucBuf.idx, &ucBuf.entry[0], ucBuf.idx);
   }
}

void CBUF_formParam(uint8_t * dst_buf, uint8_t len){
   if(len > PARAM_LEN_MAX)
      return;

   // must exclude the last 6 chars as they are cmd$ + crc 2bytes
   if(ucBuf.idx >= len+6){
      memcpy(dst_buf, &ucBuf.entry[ucBuf.idx-len-6], len);
   }else if(ucBuf.idx <= 6){
      memcpy(dst_buf, &ucBuf.entry[CBUF_SIZE + ucBuf.idx-6-len], len);
   }
   else{// len+6 > ucBuf.idx > 6
      memcpy(dst_buf, &ucBuf.entry[CBUF_SIZE + ucBuf.idx-6-len], len+6-ucBuf.idx);
      memcpy(dst_buf+len+6-ucBuf.idx, &ucBuf.entry[0], ucBuf.idx - 6);
   }
}
uint16_t CBUF_formParamCrc(){
   uint16_t crc_val;

   if(ucBuf.idx >= 6){
      crc_val = (uint16_t)ucBuf.entry[ucBuf.idx-6];
      crc_val += (uint16_t)ucBuf.entry[ucBuf.idx-5]<<8;
   }else if(ucBuf.idx <= 4){
      crc_val = (uint16_t)ucBuf.entry[CBUF_SIZE + ucBuf.idx-6];
      crc_val += (uint16_t)ucBuf.entry[CBUF_SIZE + ucBuf.idx-5]<<8;
   }
   else{// ucBuf.idx == 5
      crc_val = (uint16_t)ucBuf.entry[CBUF_SIZE-1];
      crc_val += (uint16_t)ucBuf.entry[1]<<8;
   }
   return crc_val;
}

// =================================== circular buffer end ====================================


struct uart0_cmd {
   uint8_t cmd[CMD_LEN];
   uint8_t *rspBuf;
   uint8_t rspLen;
   uint8_t *paramBuf;
   uint8_t paramLen;
   void (*cmd_cb)(uint8_t val);
};

static struct uart0_cmd uart0_command[MAX_CMD];

uint8_t uart_messageInProgress, uart_charsSent, uart_charsReceived, uart_rx_buf[4];
uint8_t uart_num_registered_cmds, uart_rsp2send;

// circular buffer
uint8_t uart_cbuf[CMD_LEN];
uint16_t uart_crcVal;


void uartRxEnable(){
   UARTIE |= UCRXIE;                         // Enable USCI_A0 RX interrupt
}

void uartRxDisable(){
   UARTIE &= ~UCRXIE;                        // Enable USCI_A0 RX interrupt
}

void uartTxEnable(){
   UARTIE |= UCTXIE;                         // Enable USCI_A0 RX interrupt
}

void uartTxDisable(){
   UARTIE &= ~UCTXIE;                        // Enable USCI_A0 RX interrupt
}

void UART_init(){
   uart_num_registered_cmds=0;
   CBUF_init();
}

void uartConfig(){
   UARTSEL |= UARTTXD+UARTRXD;

   UARTCTL1 |= UCSWRST;                      // **Put state machine in reset**
   UARTCTL0 = 0x00;
   UARTCTL1 |= UCSSEL_2;                     // SMCLK

   UARTBR0 = 0x9c;                           // 24MHz 9600
   UARTBR1 = 0;
   UARTMCTL =  UCBRS_0 + UCBRF_4 + UCOS16;   // Modln UCBRSx=0, UCBRFx=0, over sampling

   UARTCTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**

   uart_messageInProgress = 0;
   uart_charsReceived = 0;
   UARTIFG=0;
   uartRxEnable();
   uartTxEnable();

}

// e.g.: UART_regCmd("ver$", fwInfo, 6, 0, 0, 0);
// e.g.: UART_regCmd("rtc$", ackStr, 4, realTime, 21, SetRealTime);
void UART_regCmd(uint8_t *cmd_buff, uint8_t *rsp_buf, uint8_t rsp_len,
      uint8_t* param_buf, uint8_t param_len, void (*uart_cb)(uint8_t val)){
   if(uart_num_registered_cmds < MAX_CMD){
      memcpy(uart0_command[uart_num_registered_cmds].cmd, cmd_buff, 4);
      uart0_command[uart_num_registered_cmds].rspBuf = rsp_buf;
      uart0_command[uart_num_registered_cmds].rspLen = rsp_len;
      uart0_command[uart_num_registered_cmds].paramBuf = param_buf;
      uart0_command[uart_num_registered_cmds].paramLen = param_len;
      uart0_command[uart_num_registered_cmds].cmd_cb = uart_cb;
   }
   uart_num_registered_cmds++;
}

void uartSendNextChar() {
   if(uart_charsSent < uart0_command[uart_rsp2send].rspLen){
      while (UARTIFG & UCTXIFG); //ensure no tx interrupt is pending
      UARTTXBUF = *(uart0_command[uart_rsp2send].rspBuf + uart_charsSent++);
   }
   // send a 0x0d as the Carriage Return
   else if(uart_charsSent == uart0_command[uart_rsp2send].rspLen){
      while (UARTIFG & UCTXIFG);
      UARTTXBUF = uart_crcVal & 0xff;
      uart_charsSent++;
   }
   else if(uart_charsSent == (uart0_command[uart_rsp2send].rspLen+1)){
      while (UARTIFG & UCTXIFG);
      UARTTXBUF = (uart_crcVal & 0xff00)>>8;
      uart_charsSent++;
   }
   // send a 0x0d as the Carriage Return
   else if(uart_charsSent == (uart0_command[uart_rsp2send].rspLen+2)){
      while (UARTIFG & UCTXIFG);
      UARTTXBUF = 0x0d;
      uart_charsSent++;
   }
   else if(uart_charsSent == (uart0_command[uart_rsp2send].rspLen+3)){
      while (UARTIFG & UCTXIFG);
      UARTTXBUF = 0x0a;
      uart_charsSent++;
   }
   else {
      uart_messageInProgress = 0; //false
   }
}

void uartSend(){
   if(uart_messageInProgress)
      return;

   uart_messageInProgress = 1;
   uart_charsSent = 0;

   uartSendNextChar();
}

uint8_t uartUca0TxIsr(){
   uartSendNextChar();
   return 0;
}

uint8_t uartUca0RxIsr(){
   uint8_t rx_char = UARTRXBUF;

   CBUF_push(rx_char);

   if(rx_char == '$'){
      uint8_t rx_cmd[CMD_LEN];
      uint8_t i_cmd;
      uint8_t cb_val = 0;

      CBUF_formCmd(rx_cmd);
      for(i_cmd = 0; i_cmd < MAX_CMD; i_cmd++){
         if(!memcmp(uart0_command[i_cmd].cmd, rx_cmd, CMD_LEN)){
            if(uart0_command[i_cmd].paramLen){
               uint16_t rx_crc, calc_crc;
               CBUF_formParam(uart0_command[i_cmd].paramBuf, uart0_command[i_cmd].paramLen);
               rx_crc = CBUF_formParamCrc();
               calc_crc = CRC_data(uart0_command[i_cmd].paramBuf, uart0_command[i_cmd].paramLen);
               if(rx_crc != calc_crc){
                  cb_val = 1;
               }
            }
            if(uart0_command[i_cmd].cmd_cb)
               uart0_command[i_cmd].cmd_cb(cb_val);
            uart_crcVal = CRC_data(uart0_command[i_cmd].rspBuf, uart0_command[i_cmd].rspLen);
            uart_rsp2send = i_cmd;
            uartSend();
            return 0;
         }
      }
   }
   return 0;
}


void UART_deactivate(){
   UARTCTL1 |= UCSWRST;                      // **Put state machine in reset**

   UARTSEL &= ~(UARTTXD+UARTRXD);

   P6SEL |= BIT1;
   P6DIR &= ~BIT1;

   P7SEL |= BIT6;
   P7DIR &= ~BIT6;

   P3SEL &= ~BIT3;
   P3DIR |= BIT3;
   P3OUT &= ~BIT3;
}

void UART_activate(){
   P6SEL &= ~BIT1;
   P6DIR |= BIT1;
   P6OUT |= BIT1;

   P7SEL &= ~BIT6;
   P7DIR |= BIT6;
   P7OUT |= BIT6;

   P3SEL &= ~BIT3;
   P3DIR |= BIT3;
   P3OUT |= BIT3;

   UCA0_isrActivate(UCA0_isrRegister(uartUca0RxIsr, uartUca0TxIsr));

   uartConfig();
}
