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

#include "msp430.h"
#include "hal_DMA.h"

uint8_t (*reportTransferDoneFuncPtr)(void) = 0;

void DMA0_init(uint16_t *srcAddr, uint16_t *destAddr, uint16_t size) {
   // Setup DMA0
   DMA0CTL = DMADT_1+DMADSTINCR_3+DMASRCINCR_3+DMAIE;    //block transfer, inc dst, inc src, Int enable
   DMACTL0 = DMA0TSEL_24;                                //ADC12IFGx triggered
   DMACTL4 = DMARMWDIS;                                  //Read-modify-write disable

   DMA0SZ = size;                                        //DMA0 size

   //Writes a value to a 20-bit SFR register located at the given 16-bit address
   __data16_write_addr((unsigned short) &DMA0SA,(unsigned long)srcAddr);   //Source block address
   __data16_write_addr((unsigned short) &DMA0DA,(unsigned long)destAddr);  //Destination single address
}

void DMA0_repeatTransfer(uint16_t *srcAddr, uint16_t *destAddr, uint16_t size) {
   DMA0SZ = size;
   __data16_write_addr((unsigned short) &DMA0SA,(unsigned long)srcAddr);
   __data16_write_addr((unsigned short) &DMA0DA,(unsigned long) destAddr);
}

void DMA0_enable() {
   DMA0CTL &= ~DMAIFG;
   DMA0CTL |= DMAEN;
}

void DMA0_disable() {
   DMA0CTL &= ~DMAEN;
}


void DMA0_transferDoneFunction(uint8_t (*conversionDoneFuncPtr)(void)) {
   reportTransferDoneFuncPtr = conversionDoneFuncPtr;
}

#pragma vector=DMA_VECTOR
__interrupt void DMA_ISR(void) {
   switch(__even_in_range(DMAIV,16)) {
   case 0: break;
   case 2:                                // DMA0IFG = DMA Channel 0
      if(reportTransferDoneFuncPtr) {     // ensure this has been set
         if((*reportTransferDoneFuncPtr)())
            __bic_SR_register_on_exit(LPM3_bits);
      }
      break;
   case 4: break;                         // DMA1IFG = DMA Channel 1
   case 6: break;                         // DMA2IFG = DMA Channel 2
   case 8: break;                         // DMA3IFG = DMA Channel 3
   case 10: break;                        // DMA4IFG = DMA Channel 4
   case 12: break;                        // DMA5IFG = DMA Channel 5
   case 14: break;                        // DMA6IFG = DMA Channel 6
   case 16: break;                        // DMA7IFG = DMA Channel 7
   default: break;
   }
}
