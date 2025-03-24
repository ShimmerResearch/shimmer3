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

/* Buffer read / write macros                                                 */
#define RINGFIFO_RESET(ringFifo)                {ringFifo.rdIdx = ringFifo.wrIdx = 0;}
#define RINGFIFO_WR(ringFifo, dataIn, mask)     {ringFifo.data[mask & ringFifo.wrIdx++] = (dataIn);}
#define RINGFIFO_RD(ringFifo, dataOut, mask)    {ringFifo.rdIdx++; dataOut = ringFifo.data[mask & (ringFifo.rdIdx-1)];}
#define RINGFIFO_EMPTY(ringFifo)                (ringFifo.rdIdx == ringFifo.wrIdx)
#define RINGFIFO_FULL(ringFifo, mask)           ((mask & ringFifo.rdIdx) == (mask & (ringFifo.wrIdx+1)))
#define RINGFIFO_COUNT(ringFifo, mask)          (mask & (ringFifo.wrIdx - ringFifo.rdIdx))

volatile RingFifoDockTx_t gDockTxFifo;

uint8_t uart_messageInProgress;
uint8_t (*uartCallbackFunc)(uint8_t data);

void uartRxEnable(void)
{
    UARTIE |= UCRXIE;
}

void uartTxEnable(void)
{
    UARTIE |= UCTXIE;
}

void uartRxDisable(void)
{
    UARTIE &= ~UCRXIE;
}

void uartTxDisable(void)
{
    UARTIE &= ~UCTXIE;
}

void UART_init(uint8_t (*uart_cb)(uint8_t data))
{
    uartCallbackFunc = uart_cb;
    //uart_num_registered_cmds=0;

    RINGFIFO_RESET(gDockTxFifo);
    memset(gDockTxFifo.data, 0x00,
           sizeof(gDockTxFifo.data) / sizeof(gDockTxFifo.data[0]));
}

void UART_config()
{
    UARTSEL |= UARTTXD + UARTRXD;

    UARTCTL1 |= UCSWRST;                      // **Put state machine in reset**
    UARTCTL0 = 0;
    UARTCTL1 |= UCSSEL_2;                     // SMCLK

    /* 1000000 baud would be a better setting compared with 115200 (even clock
     * divider) but Consensys is currently set to use 115200 baud */
//   UARTBR0 = 24;                          //24MHz 1000000
//   UARTMCTL = 0;                           //24MHz 1000000
//   UCA1MCTL = UCBRS_0 + UCBRF_0; //Modln UCBRSx=0, UCBRFx=0, no over sampling
    //UARTBR0 = 0x03;                           // 24MHz 460800
    //UARTBR1 = 0;
    //UARTMCTL =  UCBRS_0 + UCBRF_4 + UCOS16;   // Modln UCBRSx=0, UCBRFx=0, over sampling
    //UARTBR0 = 0x06;                           // 24MHz 230400
    //UARTBR1 = 0;
    //UARTMCTL =  UCBRS_0 + UCBRF_8 + UCOS16;   // Modln UCBRSx=0, UCBRFx=0, over sampling
    UARTBR0 = 0x0d;                           // 24MHz 115200
    UARTBR1 = 0;
    UARTMCTL = UCBRS_0 + UCBRF_0 + UCOS16; // Modln UCBRSx=0, UCBRFx=0, over sampling
    //UARTBR0 = 156;                            // 24MHz 9600
    //UARTBR1 = 0;
    //UARTMCTL =  UCBRS_0 + UCBRF_4 + UCOS16;   // Modln UCBRSx=0, UCBRFx=0, over sampling

    UARTCTL1 &= ~UCSWRST;                   // **Initialize USCI state machine**

    uart_messageInProgress = 0;
    UARTIFG = 0;
    uartRxEnable();
    uartTxEnable();
}

void uartSendNextChar(void)
{
    if (!RINGFIFO_EMPTY(gDockTxFifo))
    {
        while (UARTIFG & UCTXIFG)
            ; //ensure no tx interrupt is pending
        uart_messageInProgress = 1;
        RINGFIFO_RD(gDockTxFifo, UARTTXBUF, DOCK_TX_BUF_MASK);
    }
    else
    {
        uart_messageInProgress = 0; //false
    }
}

void DockUart_write(uint8_t *buf, uint8_t len)
{
    if (getSpaceInDockTxBuf() <= len)
    {
        return; //fail
    }

    pushBytesToDockTxBuf(buf, len);

    if (!uart_messageInProgress)
    {
        uartSendNextChar();
    }
}

void DockUart_writeBlocking(uint8_t *buf, uint8_t len)
{
    DockUart_write(buf, len);
    while(getUsedSpaceInDockTxBuf());
}

void DockUart_writeText(char *str)
{
    DockUart_writeBlocking((uint8_t *)str, strlen(str));
}

uint8_t uartUca0TxIsr()
{
    uartSendNextChar();
    return 0;
}

uint8_t uartUca0RxIsr()
{
    uint8_t rx_char = UARTRXBUF;
    if (uartCallbackFunc)
        return uartCallbackFunc(rx_char);
    return 0;
}

void UART_setState(uint8_t state)
{
    if (state)
    {
        DockUart_enable();
    }
    else
    {
        DockUart_disable();
    }
}

void DockUart_disable(void)
{
    UARTCTL1 |= UCSWRST;                      // **Put state machine in reset**

    UARTSEL &= ~(UARTTXD + UARTRXD);

    P6SEL |= BIT1;
    P6DIR &= ~BIT1;

    P7SEL |= BIT6;
    P7DIR &= ~BIT6;

    Board_setExpansionBrdPower(0);
}

void DockUart_enable(void)
{
    P6SEL &= ~BIT1;
    P6DIR |= BIT1;
    P6OUT |= BIT1;

    P7SEL &= ~BIT6;
    P7DIR |= BIT6;
    P7OUT |= BIT6;

    Board_setExpansionBrdPower(1);

    UCA0_isrActivate(UCA0_isrRegister(uartUca0RxIsr, uartUca0TxIsr));

    UART_config();
}

void pushBytesToDockTxBuf(uint8_t *buf, uint8_t len)
{
    /* if enough space at after head, copy it in */
    uint16_t spaceAfterHead = DOCK_TX_BUF_SIZE
            - (gDockTxFifo.wrIdx & DOCK_TX_BUF_MASK);
    if (spaceAfterHead > len)
    {
        memcpy(&gDockTxFifo.data[(gDockTxFifo.wrIdx & DOCK_TX_BUF_MASK)], buf,
               len);
        gDockTxFifo.wrIdx += len;
    }
    else
    {
        /* Fill from head to end of buf */
        memcpy(&gDockTxFifo.data[(gDockTxFifo.wrIdx & DOCK_TX_BUF_MASK)], buf,
               spaceAfterHead);
        gDockTxFifo.wrIdx += spaceAfterHead;

        /* Fill from start of buf. We already checked above whether there is
         * enough space in the buf (getSpaceInDockTxBuf()) so we don't need to
         * worry about the tail position. */
        uint16_t remaining = len - spaceAfterHead;
        memcpy(&gDockTxFifo.data[(gDockTxFifo.wrIdx & DOCK_TX_BUF_MASK)],
               buf + spaceAfterHead, remaining);
        gDockTxFifo.wrIdx += remaining;
    }
}

uint16_t getUsedSpaceInDockTxBuf(void)
{
    return RINGFIFO_COUNT(gDockTxFifo, DOCK_TX_BUF_MASK);
}

uint16_t getSpaceInDockTxBuf(void)
{
    // Minus 1 as we always need to leave 1 empty byte in the rolling buffer
    return DOCK_TX_BUF_SIZE - 1 - getUsedSpaceInDockTxBuf();
}
