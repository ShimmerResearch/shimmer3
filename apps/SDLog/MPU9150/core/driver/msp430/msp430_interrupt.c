/*
 $License:
    Copyright (C) 2011 InvenSense Corporation, All Rights Reserved.
 $
 */
/******************************************************************************
 * $Id: msp430_interrupt.c $
 *****************************************************************************/
/**
 *  @defgroup MSP430 System Layer
 *  @brief  MSP430 System Layer APIs.
 *          To interface with any platform, eMPL needs access to various
 *          system layer functions.
 *
 *  @{
 *      @file       msp430_interrupt.c
 *      @brief      Supports common interrupt vectors using callbacks.
 *      @details    The following MSP430s are supported:
 *
 *                  MSP430F5528
 *                  MSP430F5529
 */

#include <stdio.h>
#include "msp430.h"
#include "msp430_interrupt.h"
//#include "descriptors.h"
//#include "USB_API/USB_Common/usb.h"
//--#if !defined __MSP430F5528__ && !defined __MSP430F5529__
//--#error  The interrupt driver does not support this MSP430.
//--#endif

/* MN
 * 	On MotionFit board
 *  	P20 is INT_MPU9150
 *  	P21 is INT_CMPS (i.e. Mag)
 *
 *	On Shimmer3
 *		P17 is MPU_INT
 *		P25 is MAG_INT1
 *	and (but I'll ignore for the moment)
 *		P24 is MAG_DRDY
 *
 *		P10* is BT Connect/Disconnect
 *		P13* is BT RTS
 *		P16* is BUTTON_SW1
 *		P23* is DOCK
 *		P26* is CHG_STAT1
 *		P27* is CHG_STAT2
 *
 *		* denotes in-use in BtStream
 */

#if defined __MSP430F5438A__ || defined __MSP430F5437A__
struct msp430_int_s {
    /* As more shared interrupt vectors are supported, add them here. */
//    void (*P10_cb)(void);
//    unsigned char p10_exit;
//    void (*P13_cb)(void);
//    unsigned char p13_exit;
//    void (*P16_cb)(void);
//    unsigned char p16_exit;
    void (*P17_cb)(void);
    unsigned char p17_exit;
//    void (*P23_cb)(void);
//    unsigned char p23_exit;
    void (*P25_cb)(void);
    unsigned char p25_exit;
//    void (*P26_cb)(void);
//    unsigned char p26_exit;
//    void (*P27_cb)(void);
//    unsigned char p27_exit;
    unsigned char active_low;
};
static struct msp430_int_s msp_int = {
//    .P10_cb = NULL,
//    .p10_exit = INT_EXIT_NONE,
//    .P13_cb = NULL,
//    .p13_exit = INT_EXIT_NONE,
//    .P16_cb = NULL,
//    .p16_exit = INT_EXIT_NONE,
    .P17_cb = NULL,
    .p17_exit = INT_EXIT_NONE,
//    .P23_cb = NULL,
//    .p23_exit = INT_EXIT_NONE,
    .P25_cb = NULL,
    .p25_exit = INT_EXIT_NONE
//    .P26_cb = NULL,
//    .p26_exit = INT_EXIT_NONE,
//    .P27_cb = NULL,
//    .p27_exit = INT_EXIT_NONE
};


int msp430_int_init(unsigned char active_low)
{
	// From P20 & 21 on MotionFIT to P17 & P25 on Shimmer3
	P1REN |= BIT7;
	P2REN |= BIT5;
    if (active_low) {
        /* Pull-up resistor set opposite to interrupt level. */
        P1OUT |= BIT7;
        P1IES |= BIT7;
        P2OUT |= BIT5;
        P2IES |= BIT5;
    } else {
        P1OUT &= ~BIT7;
        P1IES &= ~BIT7;
        P2OUT &= ~BIT5;
        P2IES &= ~BIT5;
    }
    P1IFG &= ~BIT7;
    P2IFG &= ~BIT5;

    msp_int.P17_cb = NULL;
    msp_int.P25_cb = NULL;
    msp_int.active_low = active_low;
    msp430_int_enable();
    return 0;
}

int msp430_int_enable(void)
{
    P1DIR &= ~BIT7;
    P2DIR &= ~BIT5;
    if (msp_int.active_low) {
        P1OUT |= BIT7;
    	P2OUT |= BIT5;
    } else {
        P1OUT &= ~BIT7;
    	P2OUT &= ~BIT5;
    }
    P1IFG &= ~BIT7;
    P1IE |= BIT7;
    P2IFG &= ~BIT5;
    P2IE |= BIT5;
    return 0;
}

int msp430_int_disable(void)
{
    P1DIR |= BIT7;
    P1OUT |= BIT7;
    P1IFG &= ~BIT7;
    P1IE |= BIT7;

    P2DIR |= BIT5;
    P2OUT |= BIT5;
    P2IFG &= ~BIT5;
    P2IE |= BIT5;
    return 0;
}

int msp430_reg_int_cb(void (*cb)(void), unsigned char port, unsigned char pin,
    unsigned char lp_exit)
{
    if (!port)
        return -1;
    if (pin > 7)
        return -1;
    switch (port) {
    case 1:
        switch (pin) {
        case 7:
            msp_int.P17_cb = cb;
            msp_int.p17_exit = lp_exit;
            break;
        default:
            return -1;
        }
    case 2:
        switch (pin) {
        case 5:
            msp_int.P25_cb = cb;
            msp_int.p25_exit = lp_exit;
            break;
        default:
            return -1;
        }
    default:
        return -1;
    }
}

#elif defined __MSP430F5528__ || defined __MSP430F5529__
struct msp430_int_s {
    /* As more shared interrupt vectors are supported, add them here. */
    void (*P20_cb)(void);
    unsigned char p20_exit;
    void (*P21_cb)(void);
    unsigned char p21_exit;
    unsigned char active_low;
};
static struct msp430_int_s msp_int = {
    .P20_cb = NULL,
    .p20_exit = INT_EXIT_NONE,
    .P21_cb = NULL,
    .p21_exit = INT_EXIT_NONE
};

//--#pragma vector = UNMI_VECTOR
//--__interrupt void UNMI_ISR(void)
//--{
//--    switch (__even_in_range(SYSUNIV, SYSUNIV_BUSIFG ))
//--    {
//--        case SYSUNIV_NONE:
//--            __no_operation();
//--            break;
//--        case SYSUNIV_NMIIFG:
//--            __no_operation();
//--            break;
//--        case SYSUNIV_OFIFG:
//--            UCSCTL7 &= ~(DCOFFG + XT1LFOFFG + XT2OFFG);
//--            SFRIFG1 &= ~OFIFG;
//--            break;
//--        case SYSUNIV_ACCVIFG:
//--            __no_operation();
//--            break;
//--        case SYSUNIV_BUSIFG:
//--            SYSBERRIV = 0;
//--#ifdef CONFIG_INTERFACE_USB
//--            USB_disable();
//--#endif
//--    }
//--}

//--
//#pragma vector=PORT2_VECTOR
//__interrupt void P2_ISR(void)
//{
//    unsigned char lp_exit;
//    switch(__even_in_range(P2IV,16)) {
//    case 2:
//        if (msp_int.P20_cb)
//            msp_int.P20_cb();
//        lp_exit = msp_int.p20_exit;
//        P2IFG &= ~0x01;
//        break;
//    case 4:
//        if (msp_int.P21_cb)
//            msp_int.P21_cb();
//        lp_exit = msp_int.p21_exit;
//        P2IFG &= ~0x02;
//        break;
//    default:
//        break;
//    }
//    switch (lp_exit) {
//    case INT_EXIT_NONE:
//        break;
//    case INT_EXIT_LPM0:
//        __bic_SR_register_on_exit(LPM0_bits);
//        break;
//    case INT_EXIT_LPM1:
//        __bic_SR_register_on_exit(LPM1_bits);
//        break;
//    case INT_EXIT_LPM2:
//        __bic_SR_register_on_exit(LPM2_bits);
//        break;
//    case INT_EXIT_LPM3:
//        __bic_SR_register_on_exit(LPM3_bits);
//        break;
//    case INT_EXIT_LPM4:
//        __bic_SR_register_on_exit(LPM4_bits);
//        break;
//    default:
//        break;
//    }
//}

int msp430_int_init(unsigned char active_low)
{
    P2REN |= 0x03;
    if (active_low) {
        /* Pull-up resistor set opposite to interrupt level. */
        P2OUT |= 0x03;
        P2IES |= 0x03;
    } else {
        P2OUT &= ~0x03;
        P2IES &= ~0x03;
    }
    P2IFG &= ~0x03;
    msp_int.P20_cb = NULL;
    msp_int.P21_cb = NULL;
    msp_int.active_low = active_low;
    msp430_int_enable();
    return 0;
}

int msp430_int_enable(void)
{
    P2DIR &= ~0x03;
    if (msp_int.active_low)
        P2OUT |= 0x03;
    else
        P2OUT &= ~0x03;
    P2IFG &= ~0x03;
    P2IE |= 0x03;
    return 0;
}

int msp430_int_disable(void)
{
    P2DIR |= 0x03;
    P2OUT |= 0x03;
    P2IFG &= ~0x03;
    P2IE |= 0x03;
    return 0;
}

int msp430_reg_int_cb(void (*cb)(void), unsigned char port, unsigned char pin,
    unsigned char lp_exit)
{
    if (!port)
        return -1;
    if (pin > 7)
        return -1;
    switch (port) {
    case 2:
        switch (pin) {
        case 0:
            msp_int.P20_cb = cb;
            msp_int.p20_exit = lp_exit;
            break;
        case 1:
            msp_int.P21_cb = cb;
            msp_int.p21_exit = lp_exit;
            break;
        default:
            return -1;
        }
    default:
        return -1;
    }
}
#endif
