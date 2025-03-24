/*
 * i2c.h
 *
 *  Created on: 24 Mar 2025
 *      Author: MarkNolan
 */

#ifndef I2C_H_
#define I2C_H_

#include <stdint.h>

void I2C_varsInit(void);
void I2C_start(uint8_t controlExpBrd);
void I2C_stop(uint8_t controlExpBrd);
void I2C_startSensing(void);
void I2C_stopSensing(void);
void I2C_configureChannels(void);
void I2C_pollSensors(void);

void detectI2cSlaves(void);

void BMPX80_startMeasurement(void);
uint16_t getBmpX80SamplingTimeInTicks(void);
uint16_t getBmpX80SamplingTimeDiffFrom9msInTicks(void);

uint8_t isPreSampleMpuMagEn(void);
uint8_t isPreSampleMpuPressEn(void);

#endif /* I2C_H_ */
