/*
 * hal_uart0.h
 *
 *  Created on: 19 Jun 2014
 *      Author: WeiboP
 */

#ifndef HAL_UART0_H_
#define HAL_UART0_H_

#include <stdint.h>
//============================== below is the UART part ===============================
#define UARTSEL         P3SEL
#define UARTTXD         BIT4
#define UARTRXD         BIT5

#define UARTCTL0        UCA0CTL0
#define UARTCTL1        UCA0CTL1
#define UARTBR0         UCA0BR0
#define UARTBR1         UCA0BR1
#define UARTMCTL        UCA0MCTL
#define UARTIFG         UCA0IFG
#define UARTIE          UCA0IE
#define UARTTXBUF       UCA0TXBUF
#define UARTRXBUF       UCA0RXBUF
#define UART_VECTOR     USCI_A0_VECTOR
#define UCIV            UCA0IV


// registers the uart to usci_a0
// must run only once before using the uart for the first time
extern void UART_reg2Uca0();

//initializes the uart_num_registered_cmds value
extern void UART_init();

// configures the pin settings
// run this every time before using UART
extern void UART_config();

// register commands
// usage: on receiving 'cmd_buff' through uart0_rx,
// return 'response_buf' through uart0_tx
// cmd_buff must be a string of exactly 4 bytes, the 4th byte must be '$'
//extern void UART_regCmd(uint8_t *cmd_buff, uint8_t *response_buf, uint8_t response_length);
extern void UART_regCmd(uint8_t *cmd_buff, uint8_t *rsp_buf, uint8_t rsp_len,
      uint8_t* param_buf, uint8_t param_len, void (*uart_cb)(uint8_t val));
// to switch between uart_isr and other isrs
// the last activated one works
extern void UART_activate();

// reset p6.1 and p7.6 back to sel+input
extern void UART_deactivate();

//============================== above is the UART part ===============================



#endif /* HAL_UART0_H_ */
