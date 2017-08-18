/*
 * hal_CRC.h
 *
 *  Created on: 23 Sep 2014
 *      Author: WeiboP
 */

#ifndef HAL_CRC_H_
#define HAL_CRC_H_

#include <stdint.h>
#include "msp430.h"

#define CRC_INIT     0xB0CA

uint16_t CRC_data(uint8_t *buf, uint8_t len);



#endif /* HAL_CRC_H_ */
