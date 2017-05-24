/*
 * Adapted from Texas Instruments supplied example code
 */

/*******************************************************************************
 *
 * Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

#include "msp430.h"
#include "HAL_Button.h"


/***************************************************************************//**
 * @brief  Initialise port for button SW1 as active low inputs
 * @param  none
 * @return none
 ******************************************************************************/
void Button_init() {
   P1OUT |= BIT6;  //button is active low
   P1REN |= BIT6;  //pullup resistor
   P1SEL &= ~BIT6;
}

/***************************************************************************//**
 * @brief  Enable button interrupts for selected buttons
 * @param  none
 * @return none
 ******************************************************************************/
void Button_interruptEnable() {
   P1IES |= BIT6;    //select fall edge trigger
   P1IFG &= ~BIT6;   //clear flags
   P1IE |= BIT6;     //enable interrupts
}

/***************************************************************************//**
 * @brief  Disable button interrupts for selected buttons
 * @param  none
 * @return none
 ******************************************************************************/
void Button_interruptDisable() {
   P1IE &= ~BIT6;
}


void Button_debounce() {
   //disable switch interrupts
   Button_interruptDisable();
   //start timer to reenable switch after 250ms
   TA1CCR0 = 8192;                     //250ms
   TA1CCTL0 = CCIE;
   TA1CTL = TASSEL_1 + MC_1 + TACLR;   //ACLK, continuous mode, clear TBR
}


#pragma vector=TIMER1_A0_VECTOR
__interrupt void TIMER1_A0_ISR(void) {
   TA1CTL = MC_0;    //disable timerA1
   Button_interruptEnable();
}
