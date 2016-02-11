/*
 $License:
    Copyright (C) 2011 InvenSense Corporation, All Rights Reserved.
 $
 */
/******************************************************************************
 * $Id: msp430_uart.c $
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
 *      @details    eMPL outputs data to another UART device connected to the
 *                  MSP430's TX/RX pins. Alternatively, these pins can be
 *                  connected to USB-UART interfacing hardware to allow the
 *                  MSP430 to communicate with a PC. This driver works for the
 *                  following MSP430s:
 *
  *                  MSP430F5528 and UART A0
 *          // Modified by MN
 *                  MSP430F5437A
 *
 *                  Dear Reader,
 *
 *                  Similar to the I2C driver used in this project, this UART
 *                  driver is specifically written for eMPL. Therefore, only
 *                  the bare minimum functionality is provided, such that the
 *                  MSP430 can send information from eMPL to the serial port,
 *                  as well as receive commands as needed from the user.
 *                  Other features such as error detection and support for
 *                  multiprocessor communication won't be present here.
 *
 *                  Finally, this driver is not multi-thread safe. It doesn't
 *                  cache data to save memory.
 *
 */
#include <stdio.h>
#include "msp430.h"
#include "msp430_uart.h"

#define CTL1        UCA1CTL1
#define BR0         UCA1BR0
#define BR1         UCA1BR1
#define MCTL        UCA1MCTL
#define IE          UCA1IE
#define TXBUF       UCA1TXBUF
#define RXBUF       UCA1RXBUF
#define IFG         UCA1IFG
#define UART_VEC    USCI_A1_VECTOR
#define IV          UCA1IV


static unsigned char rx_data;

extern volatile unsigned char rx_new;

typedef struct {
    /* RX callback. */
    int (*rx_cb)(char);
    char *data;
    char length;
    volatile unsigned char is_busy_writing_data_right_now;
} msp430_uart_info;

static msp430_uart_info uart;

/**
 *  @brief      Set up the UART port.
 *  @return     0 if successful.
 */
//int msp430_uart_init(void)
//{
//
//#if defined __MSP430F5438A__ || defined __MSP430F5437A__
//   //Set P3.3 (TX) and P3.4 (RX) to UART mode.
//	//UART RX and TX
//	P3DIR &= ~BIT7;
//    P3DIR |= BIT6;
//	P5SEL |= BIT6+BIT7;        //P5.6,P5.7 = USCI_A1 TXD/RXD
//#elif defined __MSP430F5528__ || defined __MSP430F5529__
//    // Set P3.3 (TX) and P3.4 (RX) to UART mode.
//    P3DIR &= ~(1<<4);
//    P3DIR |= (1<<3);
//    P3SEL |= 0x18;
//#endif
//
//    // Enable reset.
//    CTL1 |= UCSWRST;
//
//    // Use auxiliary clock.
//    CTL1 |= UCSSEL_2;
//
//    // Set baud rate to 115200 (see MSP430x5xx datasheet).
//    BR0 = 0x06;
//    BR1 = 0x00;
//    MCTL |= (UCBRF_8 | UCBRS_0 | UCOS16);
//
//    // Clear reset bit.
//    CTL1 &= ~UCSWRST;
//
//    // Enable interrupts. Moved to enable
//    //  IE |= UCRXIE | UCTXIE;
//    //
//
//    // Initialize struct.
//    uart.data = NULL;
//    uart.length = 0;
//    uart.rx_cb = NULL;
//
//    return 0;
//}

/* Disable UART */
//int msp430_uart_disable(void){
//	IE &=~ (UCRXIE | UCTXIE);
//	return 0;
//}
//
/*Enable UART */
//int msp430_uart_enable(void){
//	IE |= (UCRXIE | UCTXIE);
//	return 0;
//}


/**
 *  @brief      Send a string via UART.
 *
 *  NOTE: This is a blocking function and will not relinquish control of the
 *  processor until all the bytes have been sent. Deal with it. =]
 *
 *  @param[in]  length  Number of bytes.
 *  @param[in]  data    Data to be written.
 */
int msp430_uart_tx(char length, char *data)
{
//    if (!length)
//        return 0;
//
//    /* Populate struct. */
//    uart.length = length - 1;
//    uart.data = data + 1;
//    uart.is_busy_writing_data_right_now = 1;
//
//    /* Place first byte explicitly. */
//    TXBUF = data[0];
//
//    while (uart.is_busy_writing_data_right_now);
    return 0;
}

/**
 *  @brief      Register a function to be called whenever there is a byte in
 *              the RX buffer.
 *
 *  This driver only supports one callback. If multiple processes need to be
 *  executed, let the callback function handle them.
 *
 *  The registered function must take in a single parameter: the byte received.
 *  I'm sure that was an obvious one.
 *
 *  @param[in]  func    Function to be registered.
 */
int msp430_uart_register_rx_cb(int (*func)(char))
{
//    uart.rx_cb = func;
    return 0;
}

/**
 *  @brief  Interrupt handler.
 *  When an RX interrupt is detected, a callback function (if one is
 *  registered) is executed with the RX byte as a parameter.
 */
//#pragma vector=UART_VEC
//__interrupt void UART_ISR(void)
//{
//    switch(__even_in_range(IV,4)) {
//        case 2:     /* RX interrupt. */
//            /* Check if previous data processed*/
//        	if(rx_new==0){
//        		rx_new=1;
//        		rx_data = RXBUF;
//        		/*
//        		if (uart.rx_cb){
//        			uart.rx_cb(RXBUF);
//        			rx_new=1;
//        		}
//        		*/
//        	}
//            break;
//        case 4:     /* TX interrupt. */
//            /* Place next byte on buffer. */
//            if (uart.length) {
//                TXBUF = uart.data[0];
//                uart.data++;
//                uart.length--;
//            } else {
//                IFG &= UCTXIFG;
//                uart.is_busy_writing_data_right_now = 0;
//            }
//            break;
//        case 0:     /* No interrupt. */
//        default:
//            break;
//    }
//}


/*  Expect only one UART data at a time or the last one */
void msp430_get_uart_rx_data(unsigned char *uart_data){
//	*uart_data = rx_data;
}



