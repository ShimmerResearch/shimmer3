/*
 * Copyright (c) 2015, Shimmer Research, Ltd.
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
 * @author Weibo Pan
 * @date March, 2015
 *
 */
#include "hal_UCA0.h"
#include "msp430.h"

#define MAX_UCA0_ISR 2

struct uca0_isr_t {
   uint8_t (*rxIsr)(void);
   uint8_t (*txIsr)(void);
};

uint8_t uca0NumIsr, uca0ActivatedIsr;
static struct uca0_isr_t uca0Isr[MAX_UCA0_ISR];

void UCA0_isrInit(){
   uca0NumIsr = 0;
   uca0ActivatedIsr = 0;
   int i;
   for(i=0;i<MAX_UCA0_ISR;i++){
      uca0Isr[i].rxIsr = '\0';
      uca0Isr[i].txIsr = '\0';
   }
}

uint8_t UCA0_isrRegister(uint8_t (*rx_isr)(void), uint8_t (*tx_isr)(void)){
   uint8_t i;

   //first check if already registered
   //if either are, ensure both are
   //if one is, replace second one for same value
   //return value
   //this has performance hit, but uca0NumIsr will always be small
   //(normally only 1 or 2) and this function is unlikely to be called
   //in any timing critical place
   for(i=0; i<uca0NumIsr; i++) {
      if(uca0Isr[i].rxIsr == rx_isr) {
         uca0Isr[i].txIsr = tx_isr;
         return i;
      } else if (uca0Isr[i].txIsr == tx_isr) {
         uca0Isr[i].rxIsr = rx_isr;
         return i;
      }
   }

   //only get here if not already registered
   uca0Isr[uca0NumIsr].rxIsr = rx_isr;
   uca0Isr[uca0NumIsr].txIsr = tx_isr;
   return uca0NumIsr++;
}

void UCA0_isrActivate(uint8_t isr){
   uca0ActivatedIsr = isr;
}

#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{
   switch(__even_in_range(UCA0IV,4)) {
   case 0:break;                       //Vector 0 - no interrupt

   case 2:                             //Vector 2 - RXIFG
      if(uca0Isr[uca0ActivatedIsr].rxIsr){
         if(uca0Isr[uca0ActivatedIsr].rxIsr())
            __bic_SR_register_on_exit(LPM3_bits);
      }
      break;
   case 4:
      if(uca0Isr[uca0ActivatedIsr].txIsr){
         if(uca0Isr[uca0ActivatedIsr].txIsr())
            __bic_SR_register_on_exit(LPM3_bits);
      }
      break;                       //Vector 4 - TXIFG

   default: break;
   }
}


