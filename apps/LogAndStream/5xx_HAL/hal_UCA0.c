/*
 * hal_UCA0.c
 *
 *  Created on: 19 Jun 2014
 *      Author: WeiboP
 */
#include "hal_UCA0.h"
#include "msp430.h"

#define MAX_ISR 2

struct uca0_isr_t {
   void (*rxIsr)(void);
   void (*txIsr)(void);
   uint8_t rxExitLpm;
   uint8_t txExitLpm;
};

uint8_t numIsr, activatedIsr;;
static struct uca0_isr_t uca0Isr[MAX_ISR];

void UCA0_isrInit(){
   numIsr = 0;
   activatedIsr = 0;
}

uint8_t UCA0_isrRegister(void (*rx_isr)(void), uint8_t rx_exit_lpm, void (*tx_isr)(void), uint8_t tx_exit_lpm){
   uca0Isr[numIsr].rxIsr = rx_isr;
   uca0Isr[numIsr].rxExitLpm = rx_exit_lpm;
   uca0Isr[numIsr].txIsr = tx_isr;
   uca0Isr[numIsr].txExitLpm = tx_exit_lpm;
   return numIsr++;
}

void UCA0_isrActivate(uint8_t isr){
   activatedIsr = isr;
}

#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{
   switch(__even_in_range(UCA0IV,4)) {
   case 0:break;                       //Vector 0 - no interrupt

   case 2:                             //Vector 2 - RXIFG
      if(uca0Isr[activatedIsr].rxIsr){
         uca0Isr[activatedIsr].rxIsr();
         if(uca0Isr[activatedIsr].rxExitLpm)
            __bic_SR_register_on_exit(LPM3_bits);
      }
      break;
   case 4:
      if(uca0Isr[activatedIsr].txIsr){
         uca0Isr[activatedIsr].txIsr();
         if(uca0Isr[activatedIsr].txExitLpm)
            __bic_SR_register_on_exit(LPM3_bits);
      }
      break;                       //Vector 4 - TXIFG

   default: break;
   }
}


