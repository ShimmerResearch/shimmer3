/*
 * hal_CRC.c
 *
 *  Created on: 23 Sep 2014
 *      Author: WeiboP
 */
#include "hal_CRC.h"
#include "msp430.h"

uint16_t CRC_data(uint8_t *buf, uint8_t len)
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
  return crc_val;
}

void calculateCrcAndInsert(uint8_t crcMode, uint8_t *aryPtr, uint8_t len)
{
  uint16_t crc_value;
  if (crcMode != CRC_OFF)
  {
    crc_value = CRC_data(aryPtr, len);
    *(aryPtr + len++) = crc_value & 0xFF;

    if (crcMode == CRC_2BYTES_ENABLED)
    {
      *(aryPtr + len++) = (crc_value & 0xFF00) >> 8;
    }
  }
}

uint8_t checkCrc(uint8_t crcMode, uint8_t *aryPtr, uint8_t payloadLen)
{
  uint16_t crc_value_calc;
  if (crcMode != CRC_OFF)
  {
    crc_value_calc = CRC_data(aryPtr, payloadLen);

    if ((crc_value_calc & 0xFF) != *(aryPtr + payloadLen))
    {
      return 0;
    }

    if (crcMode == CRC_2BYTES_ENABLED
        && (((crc_value_calc >> 8) & 0xFF) != *(aryPtr + payloadLen + 1)))
    {
      return 0;
    }
  }

  return 1;
}
