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

#include <stdint.h>
#include "msp430.h"
#include "hal_InfoMem.h"

#define INFOMEM_OFFSET 0x1800
#define INFOMEM_SIZE 512
#define INFOMEM_SEG_SIZE 128
#define INFOMEM_SEG_A_ADDR 0x1980
#define INFOMEM_SEG_B_ADDR 0x1900
#define INFOMEM_SEG_C_ADDR 0x1880
#define INFOMEM_SEG_D_ADDR 0x1800


//returns 1 if successful, 0 if failure
uint8_t InfoMem_write(uint8_t *addr, uint8_t *buf, uint16_t size) {
   uint8_t ie, i, tempsize=0, segoffset, tempbuf[INFOMEM_SEG_SIZE];
   uint16_t written = 0;

   uint8_t *infomemaddr = addr + INFOMEM_OFFSET;
   if((infomemaddr+size) > (uint8_t *)(INFOMEM_OFFSET+INFOMEM_SIZE))
      return 0;

   //Disable interrupts while erasing
   //why?
   //see comment in MSP430F55xx_flashwrite_01.c (example code form TI) which states:
   //5xx Workaround: Disable global interrupt while erasing
   if(ie=(__get_SR_register()&0x0008))    //interrupts enabled?
      __disable_interrupt();

   FCTL3 = FWKEY;                         //Clear Lock bit
   if((uint16_t)infomemaddr < INFOMEM_SEG_C_ADDR) {
      //need to modify segment D
      memcpy(tempbuf, (uint8_t *)INFOMEM_SEG_D_ADDR, 128);  //temporarily copy contents of seg D
      if((tempsize=(INFOMEM_SEG_C_ADDR - (uint16_t)infomemaddr)) > size) tempsize = size;
      memcpy(tempbuf+(uint16_t)addr, buf, tempsize);        //modify with values to be written
      FCTL1 = FWKEY+ERASE;                                  //Set Erase bit
      *(uint8_t *)INFOMEM_SEG_D_ADDR = 0;                   //Dummy write to erase seg D
      FCTL1 = FWKEY+BLKWRT;                                 //Enable long-word write (4x faster than byte or word mode)

      for(i=0; i<INFOMEM_SEG_SIZE; i+=4)                    //write values back to seg D
         *((uint32_t *)(INFOMEM_SEG_D_ADDR+i)) = *((uint32_t *)(tempbuf+i));
   }

   if(((uint16_t)infomemaddr >= INFOMEM_SEG_C_ADDR) && ((uint16_t)infomemaddr < INFOMEM_SEG_B_ADDR) ||
         (tempsize && (tempsize < size))) {
      //need to modify segment C
      memcpy(tempbuf, (uint8_t *)INFOMEM_SEG_C_ADDR, 128);  //temporarily copy contents of seg C
      if(written=tempsize) {                                //the write started in previous seg
         segoffset = 0;
         tempsize = ((size-written)>128)?128:(size-written);
      } else {
         segoffset = (uint16_t)addr - 128;
         if((tempsize=(INFOMEM_SEG_B_ADDR - (uint16_t)infomemaddr)) > size) tempsize = size;
      }
      memcpy(tempbuf+segoffset, buf+written, tempsize);     //modify with values to be written
      FCTL1 = FWKEY+ERASE;                                  //Set Erase bit
      *(uint8_t *)INFOMEM_SEG_C_ADDR = 0;                   //Dummy write to erase seg C
      FCTL1 = FWKEY+BLKWRT;                                 //Enable long-word write (4x faster than byte or word mode)

      for(i=0; i<INFOMEM_SEG_SIZE; i+=4)                    //write values back to seg C
         *((uint32_t *)(INFOMEM_SEG_C_ADDR+i)) = *((uint32_t *)(tempbuf+i));
      written+=tempsize;
   }

   if(((uint16_t)infomemaddr >= INFOMEM_SEG_B_ADDR) && ((uint16_t)infomemaddr < INFOMEM_SEG_A_ADDR) ||
         (written && (written < size))) {
      //need to modify segment B
      memcpy(tempbuf, (uint8_t *)INFOMEM_SEG_B_ADDR, 128);  //temporarily copy contents of seg B
      if(written) {                                         //the write started in a previous seg
         segoffset = 0;
         tempsize = ((size-written)>128)?128:(size-written);
      } else {
         segoffset = (uint16_t)addr - 256;
         if((tempsize=(INFOMEM_SEG_A_ADDR - (uint16_t)infomemaddr)) > size) tempsize = size;
      }
      memcpy(tempbuf+segoffset, buf+written, tempsize);     //modify with values to be written
      FCTL1 = FWKEY+ERASE;                                  //Set Erase bit
      *(uint8_t *)INFOMEM_SEG_B_ADDR = 0;                   //Dummy write to erase seg B
      FCTL1 = FWKEY+BLKWRT;                                 //Enable long-word write (4x faster than byte or word mode)

      for(i=0; i<INFOMEM_SEG_SIZE; i+=4)                    //write values back to seg B
         *((uint32_t *)(INFOMEM_SEG_B_ADDR+i)) = *((uint32_t *)(tempbuf+i));
      written+=tempsize;
   }

   if(((uint16_t)infomemaddr >= INFOMEM_SEG_A_ADDR) || (written && (written < size))) {
      //need to modify segment A
      memcpy(tempbuf, (uint8_t *)INFOMEM_SEG_A_ADDR, 128);  //temporarily copy contents of seg B
      if(written) {                                         //the write started in a previous seg
         segoffset = 0;
         tempsize = size-written;                           //already checked that can't exceed block size
      } else {
         segoffset = (uint16_t)addr - 384;
         tempsize = size;
      }
      memcpy(tempbuf+segoffset, buf+written, tempsize);     //modify with values to be written
      if(FCTL3&LOCKA)
         FCTL3 = FWKEY+LOCKA;                               //clear the lock on seg A
      FCTL1 = FWKEY+ERASE;                                  //Set Erase bit
      *(uint8_t *)INFOMEM_SEG_A_ADDR = 0;                   //Dummy write to erase seg A
      FCTL1 = FWKEY+BLKWRT;                                 //Enable long-word write (4x faster than byte or word mode)

      for(i=0; i<INFOMEM_SEG_SIZE; i+=4)                    //write values back to seg A
         *((uint32_t *)(INFOMEM_SEG_A_ADDR+i)) = *((uint32_t *)(tempbuf+i));
   }

   FCTL1 = FWKEY;                                           //Clear WRT bit
   if(FCTL3&LOCKA)
      FCTL3 = FWKEY+LOCK;                                   //Set LOCK bit
   else
      FCTL3 = FWKEY+LOCK+LOCKA;                             //Set LOCK and LOCKA bits

   if(ie)
      __enable_interrupt();

   return 1;
}


//returns 1 if successful, 0 if failure
uint8_t InfoMem_read(uint8_t *addr, uint8_t *buf, uint16_t size) {
   addr += INFOMEM_OFFSET;
   if((addr+size) > (uint8_t *)(INFOMEM_OFFSET+INFOMEM_SIZE))
      return 0;

   memcpy(buf, addr, size);
   return 1;
}

void InfoMem_erase(uint8_t segments) {
   FCTL3 = FWKEY;                         //Clear Lock bit

   if(segments&INFOMEM_SEG_D) {
      FCTL1 = FWKEY+ERASE;                //Set Erase bit
      *(uint8_t *)INFOMEM_SEG_D_ADDR = 0; //Dummy write to erase Flash seg D
   }
   if(segments&INFOMEM_SEG_C) {
      FCTL1 = FWKEY+ERASE;
      *(uint8_t *)INFOMEM_SEG_C_ADDR = 0; //Dummy write to erase Flash seg C
   }
   if(segments&INFOMEM_SEG_B) {
      FCTL1 = FWKEY+ERASE;
      *(uint8_t *)INFOMEM_SEG_B_ADDR = 0; //Dummy write to erase Flash seg B
   }
   if(segments&INFOMEM_SEG_A) {
      if(FCTL3&LOCKA)
         FCTL3 = FWKEY+LOCKA;             //clear the lock on seg A
      FCTL1 = FWKEY+ERASE;
      *(uint8_t *)INFOMEM_SEG_A_ADDR = 0; //Dummy write to erase Flash seg A
   }

   FCTL1 = FWKEY;                         //Clear Erase bit
   if(FCTL3&LOCKA)
      FCTL3 = FWKEY+LOCK;                 //Set LOCK bit
   else
      FCTL3 = FWKEY+LOCK+LOCKA;           //Set LOCK and LOCKA bits
}
