/*
 * Adapted from Texas Instruments supplied example code
 */
/*******************************************************************************
 *
 *  Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef HAL_BOARD_H
#define HAL_BOARD_H

#include <stdint.h>

#define LED_RED 	0x01
#define LED_GREEN0	0x02
#define LED_GREEN1	0x04
#define LED_YELLOW	0x08
#define LED_BLUE	0x10
#define LED_ALL		0xFF

enum SR_BOARD_CODES
{
    EXP_BRD_BR_AMP              = 8,
    EXP_BRD_GSR                 = 14,
    SHIMMER3_IMU                = 31,
    EXP_BRD_PROTO3_MINI         = 36,
    EXP_BRD_EXG                 = 37,
    EXP_BRD_PROTO3_DELUXE       = 38,
    EXP_BRD_ADXL377_ACCEL_200G  = 44,
    EXP_BRD_EXG_UNIFIED         = 47,
    EXP_BRD_GSR_UNIFIED         = 48,
    EXP_BRD_BR_AMP_UNIFIED      = 49,
    EXP_BRD_H3LIS331DL_ACCEL_HIGH_G     = 55,
    SHIMMER_ECG_MD              = 59
};

extern void Board_init(void);
void Board_init_for_revision(uint8_t srId, uint8_t srRev, uint8_t srRevSpecial);
extern void Board_ledOn(uint8_t ledMask);
extern void Board_ledOff(uint8_t ledMask);
extern void Board_ledToggle(uint8_t ledMask);

#endif /* HAL_BOARD_H */
