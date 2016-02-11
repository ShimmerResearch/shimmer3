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


#ifndef HAL_UCS_H
#define HAL_UCS_H

//************************************************************************
// Check and define required Defines
//************************************************************************

#ifndef XT1LFOFFG               // Defines if not available in header file
#define XT1LFOFFG 0
#endif
#ifndef XT1HFOFFG               // Defines if not available in header file
#define XT1HFOFFG 0
#endif
#ifndef XT2OFFG                 // Defines if not available in header file
#define XT2OFFG 0
#endif

//************************************************************************
// Defines
//************************************************************************

#define UCS_STATUS_OK     0
#define UCS_STATUS_ERROR  1


#define SELECT_FLLREF(source) UCSCTL3 = (UCSCTL3 & ~(SELREF_7)) | (source) /* Select source for FLLREF  e.g. SELECT_FLLREF(SELREF__XT1CLK) */
#define SELECT_ACLK(source)   UCSCTL4 = (UCSCTL4 & ~(SELA_7))   | (source) /* Select source for ACLK    e.g. SELECT_ACLK(SELA__XT1CLK) */
#define SELECT_MCLK(source)   UCSCTL4 = (UCSCTL4 & ~(SELM_7))   | (source) /* Select source for MCLK    e.g. SELECT_MCLK(SELM__XT2CLK) */
#define SELECT_SMCLK(source)  UCSCTL4 = (UCSCTL4 & ~(SELS_7))   | (source) /* Select source for SMCLK   e.g. SELECT_SMCLK(SELS__XT2CLK) */
#define SELECT_MCLK_SMCLK(sources) UCSCTL4 = (UCSCTL4 & ~(SELM_7 + SELS_7)) | (sources)
    /* Select source for MCLK and SMCLK e.g. SELECT_MCLK_SMCLK(SELM__DCOCLK + SELS__DCOCLK) */


#define XT1_TO_MCLK   SELECT_MCLK(SELM__XT1CLK)   /* Select XT1 for MCLK */
#define XT2_TO_MCLK   SELECT_MCLK(SELM__XT2CLK)   /* Select XT2 for MCLK */
#define XT1_TO_SMCLK  SELECT_SMCLK(SELS__XT1CLK)  /* Select XT1 for SMCLK */
#define XT2_TO_SMCLK  SELECT_SMCLK(SELS__XT2CLK)  /* Select XT2 for SMCLK */

#define ACLK_DIV(x)   UCSCTL5 = (UCSCTL5 & ~(DIVA_7)) | (DIVA__##x)     /* set ACLK/x */
#define MCLK_DIV(x)   UCSCTL5 = (UCSCTL5 & ~(DIVM_7)) | (DIVM__##x)     /* set MCLK/x */
#define SMCLK_DIV(x)  UCSCTL5 = (UCSCTL5 & ~(DIVS_7)) | (DIVS__##x)     /* set SMCLK/x */


//====================================================================
/**
 * Startup routine for 32kHz Cristal on LFXT1
 *
 * \param xtdrive: Bits defining the LFXT drive mode after startup
 *
*/
void LFXT_Start(unsigned int xtdrive);

//====================================================================
/**
 * Startup routine for 32kHz Cristal on LFXT1 with timeout counter
 *
 * \param xtdrive: Bits defining the LFXT drive mode after startup
 * \param timeout: value for the timeout counter
 *
*/
unsigned int LFXT_Start_Timeout(unsigned int xtdrive, unsigned int timeout);

//====================================================================
/**
 * Startup routine for XT1
 *
 * \param xtdrive: Bits defining the XT drive mode
 *
*/
void XT1_Start(unsigned int xtdrive);

//====================================================================
/**
 * Startup routine for XT1 with timeout counter
 *
 * \param xtdrive: Bits defining the XT drive mode
 * \param timeout: value for the timeout counter
 *
*/
unsigned int XT1_Start_Timeout(unsigned int xtdrive, unsigned int timeout);

//====================================================================
/**
 * Use XT1 in Bypasss mode
 *
*/
void XT1_Bypass(void);

//====================================================================
/**
 * Startup routine for XT2
 *
 * \param xtdrive: Bits defining the XT drive mode
 *
*/
void XT2_Start(unsigned int xtdrive);

//====================================================================
/**
 * Startup routine for XT2 with timeout counter
 *
 * \param xtdrive: Bits defining the XT drive mode
 * \param timeout: value for the timeout counter
 *
*/
unsigned int XT2_Start_Timeout(unsigned int xtdrive, unsigned int timeout);

//====================================================================
/**
 * Use XT2 in Bypasss mode for MCLK
 *
*/
void XT2_Bypass(void);

//====================================================================
/**
  * Initializes FLL of the UCS
  *
  * \param fsystem  required system frequency (MCLK) in kHz
  * \param ratio    ratio between fsystem and FLLREFCLK
  */
void Init_FLL(unsigned int fsystem, const unsigned int ratio);

//====================================================================
/**
  * Initializes FLL of the UCS and wait till settled
  *
  * \param fsystem  required system frequency (MCLK) in kHz
  * \param ratio    ratio between fsystem and FLLREFCLK
  */
void Init_FLL_Settle(unsigned int fsystem, const unsigned int ratio);

#endif /* HAL_UCS_H */
