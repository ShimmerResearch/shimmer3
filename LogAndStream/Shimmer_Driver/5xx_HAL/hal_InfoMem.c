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
 *
 * @modified Sam O'Mahony
 * @date January, 2018
 */

#include "hal_InfoMem.h"
#include "msp430.h"
#include <stdint.h>

//returns 1 if successful, 0 if failure
uint8_t InfoMem_write(uint16_t addr, uint8_t *buf, uint16_t size)
{
  uint8_t ie, i, tempsize = 0, segoffset, tempbuf[INFOMEM_SEG_SIZE];
  uint16_t written = 0;

  uint32_t infomemaddr = addr + INFOMEM_OFFSET_MSP430;
  if ((infomemaddr + size) > (INFOMEM_OFFSET_MSP430 + INFOMEM_SIZE))
  {
    return 0;
  }

  //Disable interrupts while erasing
  //why?
  //see comment in MSP430F55xx_flashwrite_01.c (example code form TI) which
  //states: 5xx Workaround: Disable global interrupt while erasing
  if (ie = (__get_SR_register() & GIE)) //interrupts enabled?
  {
    __disable_interrupt();
  }

  FCTL3 = FWKEY; //Clear Lock bit
  if (infomemaddr < INFOMEM_SEG_C_ADDR_MSP430)
  {
    //need to modify segment D
    memcpy(tempbuf, (uint8_t *) INFOMEM_SEG_D_ADDR_MSP430, INFOMEM_SEG_SIZE); //temporarily copy contents of seg D
    if ((tempsize = (INFOMEM_SEG_C_ADDR_MSP430 - infomemaddr)) > size)
    {
      tempsize = size;
    }
    memcpy(tempbuf + addr, buf, tempsize); //modify with values to be written
    FCTL1 = FWKEY + ERASE;                 //Set Erase bit
    *(uint8_t *) INFOMEM_SEG_D_ADDR_MSP430 = 0; //Dummy write to erase seg D
    FCTL1 = FWKEY + BLKWRT; //Enable long-word write (4x faster than byte or word mode)

    for (i = 0; i < INFOMEM_SEG_SIZE; i += 4) //write values back to seg D
    {
      *((uint32_t *) (INFOMEM_SEG_D_ADDR_MSP430 + i)) = *((uint32_t *) (tempbuf + i));
    }
  }

  if (((infomemaddr >= INFOMEM_SEG_C_ADDR_MSP430) && (infomemaddr < INFOMEM_SEG_B_ADDR_MSP430))
      || (tempsize && (tempsize < size)))
  {
    //need to modify segment C
    memcpy(tempbuf, (uint8_t *) INFOMEM_SEG_C_ADDR_MSP430, INFOMEM_SEG_SIZE); //temporarily copy contents of seg C
    if (written = tempsize)
    { //the write started in previous seg
      segoffset = 0;
      tempsize = ((size - written) > INFOMEM_SEG_SIZE) ? INFOMEM_SEG_SIZE :
                                                         (size - written);
    }
    else
    {
      segoffset = addr - INFOMEM_SEG_SIZE;
      if ((tempsize = (INFOMEM_SEG_B_ADDR_MSP430 - infomemaddr)) > size)
      {
        tempsize = size;
      }
    }
    memcpy(tempbuf + segoffset, buf + written, tempsize); //modify with values to be written
    FCTL1 = FWKEY + ERASE;                                //Set Erase bit
    *(uint8_t *) INFOMEM_SEG_C_ADDR_MSP430 = 0; //Dummy write to erase seg C
    FCTL1 = FWKEY + BLKWRT; //Enable long-word write (4x faster than byte or word mode)

    for (i = 0; i < INFOMEM_SEG_SIZE; i += 4) //write values back to seg C
    {
      *((uint32_t *) (INFOMEM_SEG_C_ADDR_MSP430 + i)) = *((uint32_t *) (tempbuf + i));
    }
    written += tempsize;
  }

  if (((infomemaddr >= INFOMEM_SEG_B_ADDR_MSP430) && (infomemaddr < INFOMEM_SEG_A_ADDR_MSP430))
      || (written && (written < size)))
  {
    //need to modify segment B
    memcpy(tempbuf, (uint8_t *) INFOMEM_SEG_B_ADDR_MSP430, INFOMEM_SEG_SIZE); //temporarily copy contents of seg B
    if (written)
    { //the write started in a previous seg
      segoffset = 0;
      tempsize = ((size - written) > INFOMEM_SEG_SIZE) ? INFOMEM_SEG_SIZE :
                                                         (size - written);
    }
    else
    {
      segoffset = addr - 256;
      if ((tempsize = (INFOMEM_SEG_A_ADDR_MSP430 - infomemaddr)) > size)
      {
        tempsize = size;
      }
    }
    memcpy(tempbuf + segoffset, buf + written, tempsize); //modify with values to be written
    FCTL1 = FWKEY + ERASE;                                //Set Erase bit
    *(uint8_t *) INFOMEM_SEG_B_ADDR_MSP430 = 0; //Dummy write to erase seg B
    FCTL1 = FWKEY + BLKWRT; //Enable long-word write (4x faster than byte or word mode)

    for (i = 0; i < INFOMEM_SEG_SIZE; i += 4) //write values back to seg B
    {
      *((uint32_t *) (INFOMEM_SEG_B_ADDR_MSP430 + i)) = *((uint32_t *) (tempbuf + i));
    }
    written += tempsize;
  }

  if ((infomemaddr >= INFOMEM_SEG_A_ADDR_MSP430) || (written && (written < size)))
  {
    //need to modify segment A
    memcpy(tempbuf, (uint8_t *) INFOMEM_SEG_A_ADDR_MSP430, 128); //temporarily copy contents of seg B
    if (written)
    { //the write started in a previous seg
      segoffset = 0;
      tempsize = size - written; //already checked that can't exceed block size
    }
    else
    {
      segoffset = addr - 384;
      tempsize = size;
    }
    memcpy(tempbuf + segoffset, buf + written, tempsize); //modify with values to be written
    if (FCTL3 & LOCKA)
    {
      FCTL3 = FWKEY + LOCKA; //clear the lock on seg A
    }
    FCTL1 = FWKEY + ERASE;                      //Set Erase bit
    *(uint8_t *) INFOMEM_SEG_A_ADDR_MSP430 = 0; //Dummy write to erase seg A
    FCTL1 = FWKEY + BLKWRT; //Enable long-word write (4x faster than byte or word mode)

    for (i = 0; i < INFOMEM_SEG_SIZE; i += 4) //write values back to seg A
    {
      *((uint32_t *) (INFOMEM_SEG_A_ADDR_MSP430 + i)) = *((uint32_t *) (tempbuf + i));
    }
  }

  FCTL1 = FWKEY; //Clear WRT bit
  if (FCTL3 & LOCKA)
  {
    FCTL3 = FWKEY + LOCK; //Set LOCK bit
  }
  else
  {
    FCTL3 = FWKEY + LOCK + LOCKA; //Set LOCK and LOCKA bits
  }

  if (ie)
  {
    __enable_interrupt();
  }

  return 1;
}

//returns 1 if successful, 0 if failure
uint8_t InfoMem_read(uint16_t addr, uint8_t *buf, uint16_t size)
{
  uint32_t infomemaddr = addr + INFOMEM_OFFSET_MSP430;
  if ((infomemaddr + size) > (INFOMEM_OFFSET_MSP430 + INFOMEM_SIZE))
  {
    return 0;
  }

  memcpy(buf, (uint8_t *) infomemaddr, size);
  return 1;
}

void InfoMem_erase(uint8_t segments)
{
  FCTL3 = FWKEY; //Clear Lock bit

  if (segments & INFOMEM_SEG_D)
  {
    FCTL1 = FWKEY + ERASE;                      //Set Erase bit
    *(uint8_t *) INFOMEM_SEG_D_ADDR_MSP430 = 0; //Dummy write to erase Flash seg D
  }
  if (segments & INFOMEM_SEG_C)
  {
    FCTL1 = FWKEY + ERASE;
    *(uint8_t *) INFOMEM_SEG_C_ADDR_MSP430 = 0; //Dummy write to erase Flash seg C
  }
  if (segments & INFOMEM_SEG_B)
  {
    FCTL1 = FWKEY + ERASE;
    *(uint8_t *) INFOMEM_SEG_B_ADDR_MSP430 = 0; //Dummy write to erase Flash seg B
  }
  if (segments & INFOMEM_SEG_A)
  {
    if (FCTL3 & LOCKA)
    {
      FCTL3 = FWKEY + LOCKA; //clear the lock on seg A
    }
    FCTL1 = FWKEY + ERASE;
    *(uint8_t *) INFOMEM_SEG_A_ADDR_MSP430 = 0; //Dummy write to erase Flash seg A
  }

  FCTL1 = FWKEY; //Clear Erase bit
  if (FCTL3 & LOCKA)
  {
    FCTL3 = FWKEY + LOCK; //Set LOCK bit
  }
  else
  {
    FCTL3 = FWKEY + LOCK + LOCKA; //Set LOCK and LOCKA bits
  }
}
