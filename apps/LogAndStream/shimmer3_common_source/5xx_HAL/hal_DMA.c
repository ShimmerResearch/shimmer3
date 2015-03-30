#include "msp430.h"
#include "hal_DMA.h"

uint8_t (*reportTransferDoneFuncPtr)(void) = 0;
uint8_t (*reportTransferDoneFuncPtr_1)(void) = 0;
uint8_t (*reportTransferDoneFuncPtr_2)(void) = 0;

void DMA0_init(uint16_t *srcAddr, uint16_t *destAddr, uint16_t size) {
   // Setup DMA0
   DMACTL0 |= DMA0TSEL_24;                                   //ADC12IFGx triggered
   DMACTL4 = DMARMWDIS;                                     //Read-modify-write disable
   DMA0CTL &= ~DMAIFG;
   DMA0CTL = DMADT_1+DMADSTINCR_3+DMASRCINCR_3+DMAEN+DMAIE;    //block transfer, inc dst, inc src, Int enable
   DMA0SZ = size;                                            //DMA0 size

   //Writes a value to a 20-bit SFR register located at the given 16-bit address
   //__data16_write_addr((unsigned short) &DMA0SA,(unsigned long)srcAddr);   //Source block address
   //__data16_write_addr((unsigned short) &DMA0DA,(unsigned long)destAddr);   //Destination single address
   DMA0SA = (__SFR_FARPTR) (unsigned long) srcAddr;
   DMA0DA = (__SFR_FARPTR) (unsigned long) destAddr;
}

void DMA0_repeatTransfer(uint16_t *srcAddr, uint16_t *destAddr, uint16_t size) {
   DMA0SZ = size;
   //__data16_write_addr((unsigned short) &DMA0SA,(unsigned long)srcAddr);
   //__data16_write_addr((unsigned short) &DMA0DA,(unsigned long) destAddr);
   DMA0SA = (__SFR_FARPTR) (unsigned long) srcAddr;
   DMA0DA = (__SFR_FARPTR) (unsigned long) destAddr;
}

void DMA0_enable() {
   ADC12IFG = 0;
   DMA0CTL |= DMAEN;
}

void DMA0_disable() {
   DMA0CTL &= ~DMAEN;
   //DMA0CTL &= ~DMAIE;
}


void DMA0_transferDoneFunction(uint8_t (*conversionDoneFuncPtr)(void)) {
   reportTransferDoneFuncPtr = conversionDoneFuncPtr;
}
//DMASRCINCR_3+DMASBDB+DMALEVEL;


void DMA1_init(uint8_t *srcAddr, uint8_t *destAddr, uint16_t size) {
   // Setup DMA0
   DMACTL0 |= DMA1TSEL_23;                                   //ADC12IFGx triggered
   DMACTL4 = DMARMWDIS;                                     //Read-modify-write disable
   DMA1CTL &= ~DMAIFG;
   DMA1CTL = DMADT_0+DMADSTINCR_0+DMASRCINCR_3+DMAIE+DMASBDB+DMALEVEL;    //block transfer, fixed dst, inc src, Int enable//
   DMA1SZ = size;                                            //DMA0 size

   //Writes a value to a 20-bit SFR register located at the given 16-bit address
   //__data16_write_addr((unsigned short) &DMA0SA,(unsigned long)srcAddr);   //Source block address
   //__data16_write_addr((unsigned short) &DMA0DA,(unsigned long)destAddr);   //Destination single address
   DMA1SA = (__SFR_FARPTR) (unsigned long) srcAddr;
   DMA1DA = (__SFR_FARPTR) (unsigned long) destAddr;
}

void DMA1_repeatTransfer(uint8_t *srcAddr, uint8_t *destAddr, uint16_t size) {
   DMA1SZ = size;
   //__data16_write_addr((unsigned short) &DMA0SA,(unsigned long)srcAddr);
   //__data16_write_addr((unsigned short) &DMA0DA,(unsigned long) destAddr);
   DMA1SA = (__SFR_FARPTR) (unsigned long) srcAddr;
   DMA1DA = (__SFR_FARPTR) (unsigned long) destAddr;
}

void DMA1_enable() {
   //ADC12IFG = 0;
   DMA1CTL |= DMAEN;
   DMA1CTL |= DMAIE;
}

void DMA1_disable() {
   DMA1CTL &= ~DMAEN;
   DMA1CTL &= ~DMAIE;
   //DMA0CTL &= ~DMAIE;
}

void DMA1_transferDoneFunction(uint8_t (*conversionDoneFuncPtr)(void)) {
   reportTransferDoneFuncPtr_1 = conversionDoneFuncPtr;
}

void DMA2_init(uint16_t *srcAddr, uint16_t *destAddr, uint16_t size) {
   // Setup DMA1
   DMACTL1 |= DMA2TSEL_20;                                   //uca1rx triggered
   DMACTL4 = DMARMWDIS;                                     //Read-modify-write disable
   DMA2CTL &= ~DMAIFG;
   DMA2CTL = DMADSTINCR_3+DMASBDB+DMAIE+DMALEVEL;    //block transfer, inc dst, inc src, Int enable
   //__data16_write_addr((unsigned short) &DMA2SA,(unsigned long) srcAddr);
   //__data16_write_addr((unsigned short) &DMA2DA,(unsigned long) destAddr);
   DMA2SA = (__SFR_FARPTR) (unsigned long) srcAddr;
   DMA2DA = (__SFR_FARPTR) (unsigned long) destAddr;
}

void DMA2_repeatTransfer(uint16_t *srcAddr, uint16_t *destAddr, uint16_t size) {
   DMA2SZ = size;
   //__data16_write_addr((unsigned short) &DMA2SA,(unsigned long)srcAddr);
   //__data16_write_addr((unsigned short) &DMA2DA,(unsigned long) destAddr);
   DMA2SA = (__SFR_FARPTR) (unsigned long) srcAddr;
   DMA2DA = (__SFR_FARPTR) (unsigned long) destAddr;
}

void DMA2_enable() {
   //UCA1IFG = 0;
   DMA2CTL |= DMAEN;
   DMA2CTL |= DMAIE;
}

void DMA2_disable() {
   DMA2CTL &= ~DMAEN;
   DMA2CTL &= ~DMAIE;
}


void DMA2_transferDoneFunction(uint8_t (*conversionDoneFuncPtr)(void)) {
   reportTransferDoneFuncPtr_2 = conversionDoneFuncPtr;
}
#pragma vector=DMA_VECTOR
__interrupt void DMA_ISR(void)
{
   switch(__even_in_range(DMAIV,16))
   {
	  case 0: break;
      case 2:                                 // DMA0IFG = DMA Channel 0
         if(reportTransferDoneFuncPtr) {   // ensure this has been set
            if((*reportTransferDoneFuncPtr)())
               __bic_SR_register_on_exit(LPM3_bits);
         }
         break;
      case 4:                           // DMA1IFG = DMA Channel 1
      // if(reportTransferDoneFuncPtr_1) {   // ensure this has been set
      // if((*reportTransferDoneFuncPtr_1)())
         __bic_SR_register_on_exit(LPM3_bits);
      //}
      break;
      case 6:
         if(reportTransferDoneFuncPtr_2) {   // ensure this has been set
            if((*reportTransferDoneFuncPtr_2)())
               __bic_SR_register_on_exit(LPM3_bits);
         }
         break;                          // DMA2IFG = DMA Channel 2
      case 8: break;                          // DMA3IFG = DMA Channel 3
      case 10: break;                         // DMA4IFG = DMA Channel 4
      case 12: break;                         // DMA5IFG = DMA Channel 5
      case 14: break;                         // DMA6IFG = DMA Channel 6
      case 16: break;                         // DMA7IFG = DMA Channel 7
      default: break;
   }
}
