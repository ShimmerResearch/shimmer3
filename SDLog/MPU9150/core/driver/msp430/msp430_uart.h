/*
 $License:
    Copyright (C) 2011 InvenSense Corporation, All Rights Reserved.
 $
 */
/******************************************************************************
 * $Id: msp430_uart.h $
 *****************************************************************************/

/**
 *  @defgroup MSP430-SL
 *  @brief  MSP430 System Layer APIs.
 *          To interface with any platform, eMPL needs access to various
 *          system layer functions.
 *
 *  @{
 *      @file       msp430_uart.c
 *      @brief      UART driver needed by eMPL to output log messages and
 *                  other data.
 *      @details    The main processor should be connected to the MSP430's
 *                  TX/RX pins. Alternatively, these pins can be connected
 *                  to USB-UART interfacing hardware to allow the MSP430 to
 *                  communicate with a PC.
 */

#ifndef _MSP430_UART_H_
#define _MSP430_UART_H_
//--int msp430_uart_enable(void);
//--int msp430_uart_disable(void);
//--int msp430_uart_init(void);
int msp430_uart_tx(char length, char *data);
int msp430_uart_register_rx_cb(int (*func)(char));
void msp430_get_uart_rx_data(unsigned char *uart_data);

#endif  /* _MSP430_UART_H_ */
