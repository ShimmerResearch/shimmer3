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

// initializes the pin settings
// run this every time before using UART
extern void UART_init();

// set expected values
// usage: on receiving 'exp_buff' through rx,
// return 'buf' through tx
extern void UART_setStr(uint8_t *exp_buff, uint8_t exp_length, uint8_t *buf, uint8_t buf_length);

// to switch between uart_isr and other isrs
// the last activated one works
extern void UART_activate();
//============================== above is the UART part ===============================



#endif /* HAL_UART0_H_ */
