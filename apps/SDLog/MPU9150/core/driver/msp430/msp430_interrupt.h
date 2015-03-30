/*
 $License:
    Copyright (C) 2011 InvenSense Corporation, All Rights Reserved.
 $
 */
/******************************************************************************
 * $Id: msp430_interrupt.h $
 *****************************************************************************/
/**
 *  @defgroup MSP430_System_Layer MSP430 System Layer
 *  @brief  MSP430 System Layer APIs.
 *          To interface with any platform, eMPL needs access to various
 *          system layer functions.
 *
 *  @{
 *      @file       msp430_interrupt.h
 *      @brief      Supports common interrupt vectors using callbacks.
 *      @details    The following MSP430s are supported:
 *
 *                  MSP430F5528
 *                  MSP430F5529
 */
#ifndef _MSP430_INT_H_
#define _MSP430_INT_H_

#define INT_EXIT_NONE   (0)
#define INT_EXIT_LPM0   (1)
#define INT_EXIT_LPM1   (2)
#define INT_EXIT_LPM2   (3)
#define INT_EXIT_LPM3   (4)
#define INT_EXIT_LPM4   (5)

/**
 *  @brief      Set up shared interrupt vectors.
 *  This function will automatically call @e msp430_int_enable before
 *  returning.
 *  @param[in]  active_low  1 if interrupts are active low.
 *  @return     0 if successful.
 */
int msp430_int_init(unsigned char active_low);

/**
 *  @brief  Enable interrupts.
 *  @return 0 if successful.
 */
int msp430_int_enable(void);

/**
 *  @brief  Disable interrupts.
 *  @return 0 if successful.
 */
int msp430_int_disable(void);

/**
 *  @brief      Register callback for a particular interrupt pin.
 *  This function will override any function already registered.
 *  @param[in]  cb      Function executed for this interrupt.
 *  @param[in]  port    Port number.
 *  @param[in]  pin     Pin number.
 *  @param[in]  lp_exit Low-power mode exited after this interrupt.
 *  @return     0 if successful.
 */
int msp430_reg_int_cb(void (*cb)(void), unsigned char port, unsigned char pin,
    unsigned char lp_exit);

#endif  /* _MSP430_INT_H_ */

/**
 * @}
 */
