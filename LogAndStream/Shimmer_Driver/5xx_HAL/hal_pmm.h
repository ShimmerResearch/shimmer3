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
//    File: hal_pmm.h
//
//    Texas Instruments
//
//    Version 1.2
//    10/17/09
//
//    V1.0  Initial Version
//    V1.1  Adjustment to UG
//    V1.2  Added return values
//****************************************************************************////====================================================================


#ifndef HAL_PMM_H
#define HAL_PMM_H

#define PMM_STATUS_OK     0
#define PMM_STATUS_ERROR  1

//====================================================================
/**
  * Set the VCore to a new level if it is possible and return a
  * error - value.
  *
  * \param      level       PMM level ID
  * \return int       1: error / 0: done
  */
unsigned int SetVCore (unsigned char level);

//====================================================================
/**
  * Set the VCore to a higher level, if it is possible.
  * Return a 1 if voltage at highside (Vcc) is to low
  * for the selected Level (level).
  *
  * \param      level       PMM level ID
  * \return int       1: error / 0: done
  */
unsigned int SetVCoreUp (unsigned char level);

//====================================================================
/**
  * Set the VCore to a lower level.
  * Return a 1 if voltage at highside (Vcc) is still to low
  * for the selected Level (level).
  *
  * \param      level       PMM level ID
  * \return int       1: done with error / 0: done without error
  */
unsigned int SetVCoreDown (unsigned char level);

#endif /* HAL_PMM_H */
