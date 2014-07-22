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

//*******************************************************************************
//  Provides Functions to Initialize the UCS/FLL and clock sources
//    File: hal_ucs.c
//
//    Texas Instruments
//
//    Version 1.2
//    11/24/09
//
//    V1.0  Initial Version
//    V1.1  Added timeout function
//    V1.1  Added parameter for XTDrive
//*******************************************************************************

#include "msp430.h"
#include "hal_UCS.h"

//====================================================================
/**
 * Startup routine for 32kHz Crystal on LFXT1
 *
*/
void LFXT_Start(unsigned int xtdrive) {
   UCSCTL6_L |= XT1DRIVE1_L+XT1DRIVE0_L; // Highest drive setting for XT1 startup

   while (SFRIFG1 & OFIFG) {                             // check OFIFG fault flag
      UCSCTL7 &= ~(DCOFFG+XT1LFOFFG+XT1HFOFFG+XT2OFFG);  // Clear OSC fault Flags fault flags
      SFRIFG1 &= ~OFIFG;                                 // Clear OFIFG fault flag
   }
   UCSCTL6 = (UCSCTL6 & ~(XT1DRIVE_3)) |(xtdrive);       // set Drive mode
}

//====================================================================
/**
 * Startup routine for 32kHz Cristal on LFXT1 with timeout counter
 *
*/
unsigned int LFXT_Start_Timeout(unsigned int xtdrive, unsigned int timeout) {
   UCSCTL6_L |= XT1DRIVE1_L+XT1DRIVE0_L; // Highest drive setting for XT1 startup

   while ((SFRIFG1 & OFIFG) && timeout--){               // check OFIFG fault flag
      UCSCTL7 &= ~(DCOFFG+XT1LFOFFG+XT1HFOFFG+XT2OFFG);  // Clear OSC fault Flags fault flags
      SFRIFG1 &= ~OFIFG;                                 // Clear OFIFG fault flag
   }
   UCSCTL6 = (UCSCTL6 & ~(XT1DRIVE_3)) |(xtdrive);       // set Drive mode
   if (timeout)
      return (UCS_STATUS_OK);
   else
      return (UCS_STATUS_ERROR);
}


//====================================================================
/**
 * Startup routine for  XT1
 *
*/
void XT1_Start(unsigned int xtdrive) {
   UCSCTL6 &= ~(XT1OFF & XT1DRIVE_3);  // enable XT1
   UCSCTL6 |= (XTS & xtdrive);         // enable XT1 and set XT1Drive

   while (SFRIFG1 & OFIFG) {                             // check OFIFG fault flag
      UCSCTL7 &= ~(DCOFFG+XT1LFOFFG+XT1HFOFFG+XT2OFFG);  // Clear OSC fault Flags
      SFRIFG1 &= ~OFIFG;                                 // Clear OFIFG fault flag
   }
}

//====================================================================
/**
 * Startup routine for XT1 with timeout counter
 *
*/
unsigned int XT1_Start_Timeout(unsigned int xtdrive, unsigned int timeout) {
   UCSCTL6 &= ~(XT1OFF & XT1DRIVE_3);  // enable XT1
   UCSCTL6 |= (XTS & xtdrive);         // enable XT1 and set XT1Drive

   while ((SFRIFG1 & OFIFG) && timeout--) {              // check OFIFG fault flag
      UCSCTL7 &= ~(DCOFFG+XT1LFOFFG+XT1HFOFFG+XT2OFFG);  // Clear OSC fault Flags
      SFRIFG1 &= ~OFIFG;                                 // Clear OFIFG fault flag
   }
   if (timeout)
      return (UCS_STATUS_OK);
   else
      return (UCS_STATUS_ERROR);
}

//====================================================================
/**
 * Use  XT1 in Bypasss mode
 *
*/
void XT1_Bypass(void) {
   UCSCTL6 = XT1BYPASS;

   while (SFRIFG1 & OFIFG) {                             // check OFIFG fault flag
      UCSCTL7 &= ~(DCOFFG+XT1LFOFFG+XT1HFOFFG+XT2OFFG);  // Clear OSC fault Flags
      SFRIFG1 &= ~OFIFG;                                 // Clear OFIFG fault flag
   }
}

//====================================================================
/**
 * Startup routine for  XT2
 *
*/
void XT2_Start(unsigned int xtdrive) {
  //UCSCTL6 &= ~(XT2OFF & XT1DRIVE_3);  // enable XT2
  UCSCTL6 &= ~(XT2OFF);    // enable XT2
  UCSCTL6 |= (xtdrive);    // Set XT2Drive

   while (SFRIFG1 & OFIFG) {                             // check OFIFG fault flag
      UCSCTL7 &= ~(DCOFFG+XT1LFOFFG+XT1HFOFFG+XT2OFFG);  // Clear OSC fault Flags
      SFRIFG1 &= ~OFIFG;                                 // Clear OFIFG fault flag
   }
}

//====================================================================
/**
 * Startup routine for XT2 with timeout counter
 *
*/
unsigned int XT2_Start_Timeout(unsigned int xtdrive, unsigned int timeout) {
   UCSCTL6 &= ~(XT2OFF & XT1DRIVE_3);   // enable XT2
   UCSCTL6 |= (xtdrive);                // Set XT2Drive

   while ((SFRIFG1 & OFIFG) && timeout--){               // check OFIFG fault flag
      UCSCTL7 &= ~(DCOFFG+XT1LFOFFG+XT1HFOFFG+XT2OFFG);  // Clear OSC fault Flags
      SFRIFG1 &= ~OFIFG;                                 // Clear OFIFG fault flag
   }
   if (timeout)
      return (UCS_STATUS_OK);
   else
      return (UCS_STATUS_ERROR);
}

//====================================================================
/**
 * Use XT2 in Bypasss mode
 *
*/
void XT2_Bypass(void) {
#ifdef XT2BYPASS
   UCSCTL6 |= XT2BYPASS;

   while (SFRIFG1 & OFIFG) {                             // check OFIFG fault flag
      UCSCTL7 &= ~(DCOFFG+XT1LFOFFG+XT1HFOFFG+XT2OFFG);  // Clear OSC fault Flags
      SFRIFG1 &= ~OFIFG;                                 // Clear OFIFG fault flag
   }
#endif
}

//====================================================================
/**
  * Initializes FLL of the UCS
  *
  * \param fsystem  required system frequency (MCLK) in kHz
  * \param ratio    ratio between fsystem and FLLREFCLK
  */
void Init_FLL(unsigned int fsystem, const unsigned int ratio) {
   unsigned int d, dco_div_bits;
   unsigned int mode = 0;
   //  /\  Prevent variables from being "optimized".

   // save actual state of FLL loop control
   unsigned short globalInterruptState = __get_SR_register() & SCG0;
   __bic_SR_register(SCG0);      // Disable FLL loop control

   d = ratio;
   dco_div_bits = FLLD__2;       // Have at least a divider of 2
   if (fsystem > 16000){
      d >>= 1 ;
      mode = 1;
   } else
      fsystem <<= 1;             // fsystem = fsystem * 2

   while (d > 512) {
      dco_div_bits = dco_div_bits + FLLD0; // set next higher div level
      d >>= 1;
   }

   UCSCTL0 = 0x000;              // Set DCO to lowest Tap

   UCSCTL2 &= ~(0x3FF);          // Reset FN bits
   UCSCTL2 = dco_div_bits | (d - 1);

   if (fsystem <= 630)           //           fsystem < 0.63MHz
      UCSCTL1= DCORSEL_0 ;
   else if (fsystem <  1250)     // 0.63MHz < fsystem < 1.25MHz
      UCSCTL1= DCORSEL_1 ;
   else if (fsystem <  2500)     // 1.25MHz < fsystem <  2.5MHz
      UCSCTL1= DCORSEL_2 ;
   else if (fsystem <  5000)     // 2.5MHz  < fsystem <    5MHz
      UCSCTL1= DCORSEL_3 ;
   else if (fsystem <  10000)    // 5MHz    < fsystem <   10MHz
      UCSCTL1= DCORSEL_4 ;
   else if (fsystem <  20000)    // 10MHz   < fsystem <   20MHz
      UCSCTL1= DCORSEL_5 ;
   else if (fsystem <  40000)    // 20MHz   < fsystem <   40MHz
      UCSCTL1= DCORSEL_6 ;
   else
      UCSCTL1= DCORSEL_7 ;

   while (UCSCTL7 & DCOFFG) {    // check for DCOFFG fault flag
      UCSCTL7 &= ~(DCOFFG);      // Clear OSC flaut Flags
   }

   if (mode == 1)                                           // fsystem > 16000
      SELECT_MCLK_SMCLK(SELM__DCOCLK + SELS__DCOCLK);       // select DCOCLK
   else
      SELECT_MCLK_SMCLK(SELM__DCOCLKDIV + SELS__DCOCLKDIV); // select DCODIVCLK

   __bis_SR_register(globalInterruptState);                 // restore previous state

} // End of fll_init()

//====================================================================
/**
  * Initializes FLL of the UCS and wait till settled
  *
  * \param fsystem  required system frequency (MCLK) in kHz
  * \param ratio    ratio between fsystem and FLLREFCLK
  */
void Init_FLL_Settle(unsigned int fsystem, const unsigned int ratio) {
   volatile unsigned int x = ratio * 10;     // we have 32 steps in the DCO / loop takes at least three cycles
                                             // (int)(32/3) = 10
   Init_FLL(fsystem, ratio);
   while (x--);
}
