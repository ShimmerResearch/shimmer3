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

#ifndef HAL_UARTA0_H_
#define HAL_UARTA0_H_

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

#define DOCK_TX_BUF_SIZE                  256U              /* serial buffer in bytes (power 2)  */
#define DOCK_TX_BUF_MASK                  (DOCK_TX_BUF_SIZE-1UL)

typedef struct{
    uint8_t data[DOCK_TX_BUF_SIZE];
    // tail points to the buffer index for the oldest byte that was added to it
    uint16_t rdIdx;
    // head points to the index of the next empty byte in the buffer
    uint16_t wrIdx;
} RingFifoDockTx_t;

// registers the uart to usci_a0
// must run only once before using the uart for the first time

//extern void UART_reg2Uca0();

void DockUart_write(uint8_t *buf, uint8_t len);
void DockUart_writeBlocking(uint8_t *buf, uint8_t len);
void DockUart_writeText(char *str);

//initializes the uart_num_registered_cmds value
void UART_init(uint8_t (*uart_cb)(uint8_t data));

// configures the pin settings
// run this every time before using UART
void UART_config();

// register commands
// usage: on receiving 'cmd_buff' through uart0_rx,
// return 'response_buf' through uart0_tx
// cmd_buff must be a string of exactly 4 bytes, the 4th byte must be '$'
//extern void UART_regCmd(uint8_t *cmd_buff, uint8_t *response_buf, uint8_t response_length);
//extern void UART_regCmd(uint8_t *cmd_buff, uint8_t *rsp_buf, uint8_t rsp_len,
//      uint8_t param_flag, void (*uart_cb)(uint8_t crc_succ));
// to switch between uart_isr and other isrs
// the last activated one works
void DockUart_enable(void);

void UART_setState(uint8_t state);

// reset p6.1 and p7.6 back to sel+input
void DockUart_disable(void);

void pushBytesToDockTxBuf(uint8_t *buf, uint8_t len);
uint16_t getUsedSpaceInDockTxBuf(void);
uint16_t getSpaceInDockTxBuf(void);

//============================== above is the UART part ===============================



#endif /* HAL_UARTA0_H_ */
