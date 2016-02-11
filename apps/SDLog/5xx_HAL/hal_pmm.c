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

//****************************************************************************//
// Function Library for setting the PMM
//    File: hal_pmm.c
//
//    Texas Instruments
//
//    Version 1.2
//    11/24/09
//
//    V1.0  Initial Version
//    V1.1  Adjustment to UG
//    V1.2  Added return values
//****************************************************************************////====================================================================

#include "msp430.h"
#include "hal_pmm.h"

#define _HAL_PMM_DISABLE_SVML_
#define _HAL_PMM_DISABLE_SVSL_
#define _HAL_PMM_DISABLE_FULL_PERFORMANCE_


//****************************************************************************//
#ifdef _HAL_PMM_DISABLE_SVML_
#define _HAL_PMM_SVMLE SVMLE
#else
#define _HAL_PMM_SVMLE 0
#endif
#ifdef _HAL_PMM_DISABLE_SVSL_
#define _HAL_PMM_SVSLE SVSLE
#else
#define _HAL_PMM_SVSLE 0
#endif
#ifdef _HAL_PMM_DISABLE_FULL_PERFORMANCE_
#define _HAL_PMM_SVSFP SVSLFP
#else
#define _HAL_PMM_SVSFP 0
#endif

//****************************************************************************//
// Set VCore
//****************************************************************************//
unsigned int SetVCore (unsigned char level) {
   unsigned int actlevel;
   unsigned int status = 0;
  
   level &= PMMCOREV_3;                   // Set Mask for Max. level
   actlevel = (PMMCTL0 & PMMCOREV_3);     // Get actual VCore

   while (((level != actlevel) && (status == 0)) || (level < actlevel)) {  // step by step increase or decrease
      if (level > actlevel)
         status = SetVCoreUp(++actlevel);
      else
         status = SetVCoreDown(--actlevel);
   }
   return status;
}

//****************************************************************************//
// Set VCore Up
//****************************************************************************//
unsigned int SetVCoreUp (unsigned char level) {
   unsigned int PMMRIE_backup, SVSMHCTL_backup;

   // Open PMM registers for write access
   PMMCTL0_H = 0xA5;

   // Disable dedicated Interrupts to prevent that needed flags will be cleared
   PMMRIE_backup = PMMRIE;
   PMMRIE &= ~(SVSMHDLYIE | SVSMLDLYIE | SVMLVLRIE | SVMHVLRIE | SVMHVLRPE);
   // Set SVM highside to new level and check if a VCore increase is possible
   SVSMHCTL_backup = SVSMHCTL;
   PMMIFG &= ~(SVMHIFG | SVSMHDLYIFG);
   SVSMHCTL = SVMHE | SVMHFP | (SVSMHRRL0 * level);
   // Wait until SVM highside is settled
   while ((PMMIFG & SVSMHDLYIFG) == 0);
   // Disable full-performance mode to save energy
   SVSMHCTL &= ~_HAL_PMM_SVSFP ;
   // Check if a VCore increase is possible
   if ((PMMIFG & SVMHIFG) == SVMHIFG) {      //-> Vcc is to low for a Vcore increase
      // recover the previous settings
      PMMIFG &= ~SVSMHDLYIFG;
      SVSMHCTL = SVSMHCTL_backup;
      // Wait until SVM highside is settled
      while ((PMMIFG & SVSMHDLYIFG) == 0);
      // Clear all Flags
      PMMIFG &= ~(SVMHVLRIFG | SVMHIFG | SVSMHDLYIFG | SVMLVLRIFG | SVMLIFG | SVSMLDLYIFG);
      // backup PMM-Interrupt-Register
      PMMRIE = PMMRIE_backup;
   
      // Lock PMM registers for write access
      PMMCTL0_H = 0x00;
      return PMM_STATUS_ERROR;               // return: voltage not set
   }
   // Set also SVS highside to new level     //-> Vcc is high enough for a Vcore increase
   SVSMHCTL |= SVSHE | (SVSHRVL0 * level);
   // Set SVM low side to new level
   SVSMLCTL = SVMLE | SVMLFP | (SVSMLRRL0 * level);
   // Wait until SVM low side is settled
   while ((PMMIFG & SVSMLDLYIFG) == 0);
   // Clear already set flags
   PMMIFG &= ~(SVMLVLRIFG | SVMLIFG);
   // Set VCore to new level
   PMMCTL0_L = PMMCOREV0 * level;
   // Wait until new level reached
   if (PMMIFG & SVMLIFG)
      while ((PMMIFG & SVMLVLRIFG) == 0);
   // Set also SVS/SVM low side to new level
   PMMIFG &= ~SVSMLDLYIFG;
   SVSMLCTL |= SVSLE | (SVSLRVL0 * level);
   // wait for lowside delay flags
   while ((PMMIFG & SVSMLDLYIFG) == 0);

   // Disable SVS/SVM Low
   // Disable full-performance mode to save energy
   SVSMLCTL &= ~(_HAL_PMM_DISABLE_SVSL_+_HAL_PMM_DISABLE_SVML_+_HAL_PMM_SVSFP );

   // Clear all Flags
   PMMIFG &= ~(SVMHVLRIFG | SVMHIFG | SVSMHDLYIFG | SVMLVLRIFG | SVMLIFG | SVSMLDLYIFG);
   // backup PMM-Interrupt-Register
   PMMRIE = PMMRIE_backup;

   // Lock PMM registers for write access
   PMMCTL0_H = 0x00;
   return PMM_STATUS_OK;                     // return: OK
}

//****************************************************************************//
// Set VCore down (Independent from the enabled Interrupts in PMMRIE)
//****************************************************************************//
unsigned int SetVCoreDown (unsigned char level) {
   unsigned int PMMRIE_backup;

   // Open PMM registers for write access
   PMMCTL0_H = 0xA5;

   // Disable dedicated Interrupts to prevent that needed flags will be cleared
   PMMRIE_backup = PMMRIE;
   PMMRIE &= ~(SVSMHDLYIE | SVSMLDLYIE | SVMLVLRIE | SVMHVLRIE | SVMHVLRPE);

   // Set SVM high side and SVM low side to new level
   PMMIFG &= ~(SVMHIFG | SVSMHDLYIFG | SVMLIFG | SVSMLDLYIFG);
   SVSMHCTL = SVMHE | SVMHFP | (SVSMHRRL0 * level);
   SVSMLCTL = SVMLE | SVMLFP | (SVSMLRRL0 * level);
   // Wait until SVM high side and SVM low side is settled
   while ((PMMIFG & SVSMHDLYIFG) == 0 || (PMMIFG & SVSMLDLYIFG) == 0);

   // Set VCore to new level
   PMMCTL0_L = PMMCOREV0 * level;

   // Set also SVS highside and SVS low side to new level
   PMMIFG &= ~(SVSHIFG | SVSMHDLYIFG | SVSLIFG | SVSMLDLYIFG);
   SVSMHCTL |= SVSHE | SVSHFP | (SVSHRVL0 * level);
   SVSMLCTL |= SVSLE | SVSLFP | (SVSLRVL0 * level);
   // Wait until SVS high side and SVS low side is settled
   while ((PMMIFG & SVSMHDLYIFG) == 0 || (PMMIFG & SVSMLDLYIFG) == 0);
   // Disable full-performance mode to save energy
   SVSMHCTL &= ~_HAL_PMM_SVSFP;
   // Disable SVS/SVM Low
   // Disable full-performance mode to save energy
   SVSMLCTL &= ~(_HAL_PMM_DISABLE_SVSL_+_HAL_PMM_DISABLE_SVML_+_HAL_PMM_SVSFP );
   
   // Clear all Flags
   PMMIFG &= ~(SVMHVLRIFG | SVMHIFG | SVSMHDLYIFG | SVMLVLRIFG | SVMLIFG | SVSMLDLYIFG);
   // backup PMM-Interrupt-Register
   PMMRIE = PMMRIE_backup;
   // Lock PMM registers for write access
   PMMCTL0_H = 0x00;

   if ((PMMIFG & SVMHIFG) == SVMHIFG)
      return PMM_STATUS_ERROR;               // Highside is still to low for the adjusted VCore Level
   else 
      return PMM_STATUS_OK;                  // Return: OK
}

