/*
 * hal_UCA0.c
 *
 *  Created on: 19 Jun 2014
 *      Author: WeiboP
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
      break;                           //Vector 4 - TXIFG

   default: break;
   }
}


