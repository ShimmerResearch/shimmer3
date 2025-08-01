/*
 * hal_cat24c16.h
 *
 *  Created on: 31 Jul 2025
 *      Author: MarkNolan
 */

#ifndef SHIMMER_DRIVER_CAT24C16_HAL_CAT24C16_H_
#define SHIMMER_DRIVER_CAT24C16_HAL_CAT24C16_H_

#include <stdint.h>

enum EEPROM_RW
{
  EEPROM_READ = 0,
  EEPROM_WRITE = 1,
};

void eepromRead(uint16_t dataAddr, uint16_t dataSize, uint8_t *dataBuf);
void eepromWrite(uint16_t dataAddr, uint16_t dataSize, uint8_t *dataBuf);
void eepromReadWrite(uint16_t dataAddr, uint16_t dataSize, uint8_t *dataBuf, enum EEPROM_RW eepromRW);

#endif /* SHIMMER_DRIVER_CAT24C16_HAL_CAT24C16_H_ */
