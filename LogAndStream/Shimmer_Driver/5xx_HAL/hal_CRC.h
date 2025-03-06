/*
 * hal_CRC.h
 *
 *  Created on: 23 Sep 2014
 *      Author: WeiboP
 */

#ifndef HAL_CRC_H_
#define HAL_CRC_H_

#include <stdint.h>

#define CRC_INIT     0xB0CA

typedef enum
{
    CRC_OFF              = 0,
    CRC_1BYTES_ENABLED   = 1,
    CRC_2BYTES_ENABLED   = 2,
} COMMS_CRC_MODE;

uint16_t CRC_data(uint8_t *buf, uint8_t len);
void calculateCrcAndInsert(uint8_t crcMode, uint8_t *aryPtr, uint8_t len);
uint8_t checkCrc(uint8_t crcMode, uint8_t *aryPtr, uint8_t payloadLen);

#endif /* HAL_CRC_H_ */
