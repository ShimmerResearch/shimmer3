/*
 * hal_FactoryTest.h
 *
 *  Created on: 14 Aug 2024
 *      Author: MarkNolan
 */

#ifndef SHIMMER3_COMMON_SOURCE_5XX_HAL_HAL_FACTORYTEST_H_
#define SHIMMER3_COMMON_SOURCE_5XX_HAL_HAL_FACTORYTEST_H_

#include <stdint.h>

#include "Test/shimmer_test.h"

#define DELAY_BETWEEN_LED_CHANGES_TICKS 48000000UL

void hal_run_factory_test(factory_test_t factoryTestToRun, char *bufPtr);
void print_shimmer_model(void);
void led_test(void);
void sd_card_test(void);
void bt_module_test(void);
void I2C_test(void);
void SPI_test(void);

#endif /* SHIMMER3_COMMON_SOURCE_5XX_HAL_HAL_FACTORYTEST_H_ */
