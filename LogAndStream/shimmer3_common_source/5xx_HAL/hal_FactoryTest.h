/*
 * hal_FactoryTest.h
 *
 *  Created on: 14 Aug 2024
 *      Author: MarkNolan
 */

#ifndef SHIMMER3_COMMON_SOURCE_5XX_HAL_HAL_FACTORYTEST_H_
#define SHIMMER3_COMMON_SOURCE_5XX_HAL_HAL_FACTORYTEST_H_

#include <stdint.h>

#define DELAY_BETWEEN_LED_CHANGES_TICKS 48000000UL

typedef enum
{
  PRINT_TO_DEBUGGER = 0,
  PRINT_TO_DOCK_UART,
  PRINT_TO_BT_UART
} factory_test_target_t;

typedef enum
{
  FACTORY_TEST_ALL = 0,
//  FACTORY_TEST_LED_START,
//  FACTORY_TEST_STOP,
  FACTORY_TEST_COUNT
} factory_test_t;

void run_factory_test(void);
void led_test(void);
void sd_card_test(void);
void setup_factory_test(factory_test_target_t target, factory_test_t testToRun);
void send_test_report(char *str);

#endif /* SHIMMER3_COMMON_SOURCE_5XX_HAL_HAL_FACTORYTEST_H_ */
