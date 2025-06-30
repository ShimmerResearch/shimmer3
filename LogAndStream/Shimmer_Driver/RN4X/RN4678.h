/*
 * RN4678.h
 *
 *  Created on: 30 Jun 2025
 *      Author: MarkNolan
 */

#ifndef SHIMMER_DRIVER_RN4X_RN4678_H_
#define SHIMMER_DRIVER_RN4X_RN4678_H_

#include <stdint.h>

void RN4678_resetStatusString(void);
void RN4678_startOfNewStatusString(void);
uint8_t Rn4678_parseStatusString(uint8_t *waitingForArgs, uint8_t *btRxBuffPtr);

#endif /* SHIMMER_DRIVER_RN4X_RN4678_H_ */
