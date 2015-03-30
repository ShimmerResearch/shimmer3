/*
 * hal_uart0.c
 *
 *  Created on: 19 Jun 2014
 *      Author: WeiboP
 */
#include "hal_uart0.h"
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

//#define CBUF_init( cbuf )       cbuf.idx = 0
//#define CBUF_push( cbuf, elem ) (cbuf.entry)[ cbuf.idx++ & (( cbuf##_SIZE ) - 1 )] = (elem)

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
      //memcpy((uint8_t*)(&crc_val), &ucBuf.entry[ucBuf.idx-6], 2);
      crc_val = (uint16_t)ucBuf.entry[ucBuf.idx-6];
      crc_val += (uint16_t)ucBuf.entry[ucBuf.idx-5]<<8;
   }else if(ucBuf.idx <= 4){
      //memcpy((uint8_t*)(&crc_val), &ucBuf.entry[CBUF_SIZE + ucBuf.idx-6], 2);
      crc_val = (uint16_t)ucBuf.entry[CBUF_SIZE + ucBuf.idx-6];
      crc_val += (uint16_t)ucBuf.entry[CBUF_SIZE + ucBuf.idx-5]<<8;
   }
   else{// ucBuf.idx == 5
      //*(uint8_t*)(&crc_val) = ucBuf.entry[CBUF_SIZE-1];
      //*((uint8_t*)(&crc_val)+1) = ucBuf.entry[1];
      crc_val = (uint16_t)ucBuf.entry[CBUF_SIZE-1];
      crc_val += (uint16_t)ucBuf.entry[1]<<8;
   }
   return crc_val;
}

/*void CBUF_formParam(uint8_t * dst_buf, uint8_t len){
   if(len > PARAM_LEN_MAX)
      return;

   // must exclude the last 4 chars as they are cmd$
   if(ucBuf.idx >= len+4){
      memcpy(dst_buf, &ucBuf.entry[ucBuf.idx-len-4], len);
   }else if(ucBuf.idx <= 4){
      memcpy(dst_buf, &ucBuf.entry[CBUF_SIZE + ucBuf.idx-4-len], len);
   }
   else{// len+4 > ucBuf.idx > 4
      memcpy(dst_buf, &ucBuf.entry[CBUF_SIZE + ucBuf.idx-4-len], len+4-ucBuf.idx);
      memcpy(dst_buf+len+4-ucBuf.idx, &ucBuf.entry[0], ucBuf.idx - 4);
   }
}*/

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
uint8_t uart_isr_number_uart, uart_num_registered_cmds, uart_rsp2send;

// circular buffer
uint8_t uart_cbuf[CMD_LEN];
uint16_t uart_crcVal;

//void UART_setExgOrUart(uint8_t val){
//   exgOrUart = val;
//}

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
void UART_config(){
   UARTSEL |= UARTTXD+UARTRXD;

   UARTCTL1 |= UCSWRST;                      // **Put state machine in reset**
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

/*uint8_t uartUca0RxIsr(){
   uint8_t i_cmd;

   memcpy((uint8_t*)uart_cbuf, (uint8_t*)(uart_cbuf+1), CMD_LEN-1);
   uart_cbuf[CMD_LEN-1]=UARTRXBUF;

   //UARTTXBUF = UARTRXBUF;
   for(i_cmd = 0; i_cmd < MAX_CMD; i_cmd++){
      if(!memcmp(uart0_command[i_cmd].cmd, uart_cbuf, CMD_LEN)){
         uart_rsp2send = i_cmd;
         uartSend();
         return 0;
      }
   }
   return 0;
}*/

   //Board_ledToggle(LED_GREEN0);

void UART_reg2Uca0(){
   uart_isr_number_uart = UCA0_isrRegister(uartUca0RxIsr, uartUca0TxIsr);
}

void UART_deactivate(){
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

   UCA0_isrActivate(uart_isr_number_uart);

   UART_config();
}
//============================== above is the UART part ===============================



