/*
 * adc.h
 *
 *  Created on: 24 Mar 2025
 *      Author: MarkNolan
 */

#ifndef ADC_H_
#define ADC_H_

#include <stdint.h>

void ADC_varsInit(void);
void ADC_startSensing(void);
void ADC_stopSensing(void);
void ADC_configureChannels(void);
void ADC_gatherDataStart(void);

inline void GsrRange(void);

uint8_t Dma0ConversionDone(void);
uint8_t Dma0BatteryRead(void);
void SetBattDma(void);

void manageReadBatt(uint8_t isCalledFromMain);
void saveBatteryVoltageAndUpdateStatus(void);

#endif /* ADC_H_ */
