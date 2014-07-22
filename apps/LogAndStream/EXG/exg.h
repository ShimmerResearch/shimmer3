/*
 * Copyright (c) 2013, Shimmer Research, Ltd.
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *    * Neither the name of Shimmer Research, Ltd. nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *    * You may not use or distribute this Software or any derivative works
 *      in any form for commercial purposes with the exception of commercial
 *      purposes when used in conjunction with Shimmer products purchased
 *      from Shimmer or their designated agent or with permission from
 *      Shimmer.
 *      Examples of commercial purposes would be running business
 *      operations, licensing, leasing, or selling the Software, or
 *      distributing the Software for use with commercial products.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @author Mike Healy
 * @date December, 2013
 */

#ifndef EXG_H
#define EXG_H

#include <stdint.h>

//ADS1292R registers
//There are two of these chips on the ExG board
#define ADS1292R_DEVID     0x00
#define ADS1292R_CONFIG1   0x01
#define ADS1292R_CONFIG2   0x02
#define ADS1292R_LOFF      0x03
#define ADS1292R_CH1SET    0x04
#define ADS1292R_CH2SET    0x05
#define ADS1292R_RLD_SENS  0x06
#define ADS1292R_LOFF_SENS 0x07
#define ADS1292R_LOFF_STAT 0x08
#define ADS1292R_RESP1     0x09
#define ADS1292R_RESP2     0x0A

//initialize both ADS1292R chips on ExG board
//i.e. power them on and enable internal reference
//leave both in SDATAC mode
void EXG_init(void);

//put ADS1292R chip in RDATAC mode and start sampling
//also enable data ready interrupts for selected chip
//P2.0 for chip 1 and P1.4 for chip 2
//chip = which of the ADS1292R chips to issue the command to
// 0 for chip 1 (i.e. LL-RA and LA-RA)
// 1 for chip 2 (i.e. LA/RA RESP DEMOD and V1-WCT)
// 2 for both chips (desirable to start both as close as to
// each other as possible)
void EXG_start(uint8_t chip);

//stop selected ADC1292R chips sampling and put in SDATAC mode
//also disable data ready interrupt
//chip = which of the ADS1292R chips to issue the command to
// 0 for chip 1 (i.e. LL-RA and LA-RA)
// 1 for chip 2 (i.e. LA/RA RESP DEMOD and V1-WCT)
void EXG_stop(uint8_t chip);

//power off both ExG chips
void EXG_powerOff(void);

//reset all ADS1292 registers to default values
//chip = which of the ADS1292R chips to issue the command to
// 0 for chip 1 (i.e. LL-RA and LA-RA)
// 1 for chip 2 (i.e. LA/RA RESP DEMOD and V1-WCT)
void EXG_resetRegs(uint8_t chip);

//Used to cancel channel offset on the ADS1292R chips
//Must be sent every time there is a change in the PGA gain settings
//chip = which of the ADS1292R chips to issue the command to
// 0 for chip 1 (i.e. LL-RA and LA-RA)
// 1 for chip 2 (i.e. LA/RA RESP DEMOD and V1-WCT)
void EXG_offsetCal(uint8_t chip);

//Read ADS1292 register
//chip = which of the two ADS1292R chips to read from
// 0 for chip 1 (i.e. LL-RA and LA-RA)
// 1 for chip 2 (i.e. LA/RA RESP DEMOD and V1-WCT)
//startaddress = address of first register to read from
//size = number of bytes to read
//rdata = location to store read bytes
void EXG_readRegs(uint8_t chip, uint8_t startaddress, uint8_t size, uint8_t *rdata);

//Write ADS1292 register
//chip = which of the two ADS1292R chips to write to
// 0 for chip 1 (i.e. LL-RA and LA-RA)
// 1 for chip 2 (i.e. LA/RA RESP DEMOD and V1-WCT)
//startaddress = address of first register to write to
//size = number of bytes to write
//wdata = location of bytes to write
void EXG_writeRegs(uint8_t chip, uint8_t startaddress, uint8_t size, uint8_t *wdata);

//read most recently sampled data into buf
//chip = which of the two ADS1292R chips to read
// 0 for chip 1 (i.e. LL-RA and LA-RA)
// 1 for chip 2 (i.e. LA/RA RESP DEMOD and V1-WCT)
//size = 24-bit or 16-bit data
// 0 for 24bit
// 1 for 16bit (drops 7 least significant bits and most significant bit)
//Format: x00 + 5 LOFF_STAT bits + 24-bits/16-bits × 2 channels per chip
//so returns 5 or 7 bytes in total, depending on size setting
//The data format for each channel data is twos complement, MSB first.
//If data is valid MSB of status byte for each chip is 1, else 0
void EXG_readData(uint8_t chip, uint8_t size, uint8_t *buf);


//Tell the driver that the data is ready to be read from chipX
void EXG_dataReadyChip1();
void EXG_dataReadyChip2();

#endif //EXG_H
