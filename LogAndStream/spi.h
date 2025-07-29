/*
 * spi.h
 *
 *  Created on: 24 Mar 2025
 *      Author: MarkNolan
 */

#ifndef SPI_H_
#define SPI_H_

#include <stdint.h>

void SPI_varsInit(void);
void SPI_startSensing(void);
void SPI_stopSensing(void);
void SPI_configureChannels(void);
void SPI_pollSensors(void);

void adsClockTiedSet(uint8_t state);
uint8_t adsClockTiedGet(void);

#endif /* SPI_H_ */
