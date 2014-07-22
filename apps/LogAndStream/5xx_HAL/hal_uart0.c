/*
 * hal_uart0.c
 *
 *  Created on: 19 Jun 2014
 *      Author: WeiboP
 */
#include "hal_uart0.h"
#include "hal_UCA0.h"
#include "msp430.h"
#include <string.h>

//============================== below is the UART part ===============================
uint8_t *uart_str_buf, uart_str_len, *uart_exp_buf, uart_exp_len;
uint8_t uart_messageInProgress, uart_charssent, uart_charsreceived, rx_buf[12];
uint8_t uart_isr_number;

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

   UARTSEL |= UARTTXD+UARTRXD;

   UARTCTL1 |= UCSWRST;                      // **Put state machine in reset**
   UARTCTL1 |= UCSSEL_2;                     // SMCLK

   UARTBR0 = 0x9c;                           // 24MHz 9600
   UARTBR1 = 0;
   UARTMCTL =  UCBRS_0 + UCBRF_4 + UCOS16;   // Modln UCBRSx=0, UCBRFx=0, over sampling

   UARTCTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**

   uart_messageInProgress = 0;
   uart_charsreceived = 0;
   UARTIFG=0;
   uartRxEnable();
   uartTxEnable();

}


void UART_setStr(uint8_t *exp_buff, uint8_t exp_length, uint8_t *buf, uint8_t buf_length){
   uart_str_buf = buf;
   uart_str_len = buf_length;
   uart_exp_buf = exp_buff;
   uart_exp_len = exp_length;
   uart_charsreceived = 0;
}

void uartSendNextChar() {
   if(uart_charssent < uart_str_len){
      while (UARTIFG & UCTXIFG); //ensure no tx interrupt is pending
      UARTTXBUF = *(uart_str_buf + uart_charssent++);
   } else {
      uart_messageInProgress = 0; //false
   }
}
void uartSend(){
   if(uart_messageInProgress)
      return;

   uart_messageInProgress = 1;
   uart_charssent = 0;

   uartSendNextChar();
}


void uartUca0TxIsr(){
   uartSendNextChar();
}
void uartUca0RxIsr(){
   rx_buf[uart_charsreceived++]=UARTRXBUF;
   if(uart_charsreceived==uart_exp_len){
      if(!memcmp(uart_exp_buf, rx_buf, uart_charsreceived)){
         uart_charsreceived=0;
         uartSend();
      }
   }
   else if(UARTRXBUF=='$'){
      uart_charsreceived=0;
   }
}

void UART_reg2Uca0(){
   uart_isr_number = UCA0_isrRegister(uartUca0RxIsr, 0, uartUca0TxIsr, 0);
}

void UART_activate(){
   UCA0_isrActivate(uart_isr_number);
}
//============================== above is the UART part ===============================



