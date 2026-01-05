/*
 * hal_CRC.c
 *
 *  Created on: 23 Sep 2014
 *      Author: WeiboP
 */
#include "hal_CRC.h"
#include "msp430.h"

#include "CRC/shimmer_crc.h"

uint32_t platform_crcData(uint8_t *buf, uint8_t len)
{
  uint8_t i;
  uint16_t crc_val = 0;

  CRCINIRES = CRC_INIT;
  if (len & 0x01)
  {
    for (i = 0; i < (uint8_t) ((len - 1) >> 1); i++)
    {
      CRCDIRB = *(((uint16_t *) buf) + i);
    }
    CRCDIRB = (uint16_t) (buf[(uint8_t) (len - 1)]);
  }
  else
  {
    for (i = 0; i < (uint8_t) (len >> 1); i++)
    {
      CRCDIRB = *(((uint16_t *) buf) + i);
    }
  }
  crc_val = CRCINIRES;
  // The cast to uint32_t is necessary for cross-platform compatibility
  return (uint32_t) crc_val;
}
