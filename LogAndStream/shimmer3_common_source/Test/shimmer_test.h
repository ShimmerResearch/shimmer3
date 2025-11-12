/*
 * shimmer_test.h
 *
 *  Created on: Aug 14, 2025
 *      Author: MarkNolan
 */

#ifndef TEST_SHIMMER_TEST_H_
#define TEST_SHIMMER_TEST_H_

#include <stdint.h>

typedef enum
{
  PRINT_TO_DEBUGGER = 0,
  PRINT_TO_DOCK_UART,
  PRINT_TO_BT_UART
} factory_test_target_t;

typedef enum
{
  FACTORY_TEST_MAIN = 0,
  FACTORY_TEST_LEDS,
  FACTORY_TEST_ICS,
  FACTORY_TEST_LED_STATES,
  FACTORY_TEST_COUNT
} factory_test_t;

#define MAX_TEST_REPORT_LENGTH          128
#define SELF_TEST_STR_PASS              "PASS"
#define SELF_TEST_STR_FAIL              "FAIL"
#define SELF_TEST_STR_EMPTY             ""
#define SELF_TEST_STR_CHIP_DETECTION    " - Chip not detected"
#define SELF_TEST_STR_SIGNAL_ISSUE      " - Signal issue"
#define SELF_TEST_STR_TEMPERATURE_ISSUE " - Temperature issue"
#define SELF_TEST_STR_DRDY_ISSUE        " - DRDY/INT issue"
#define SELF_TEST_STR_UNKNOWN           " - Unknown"

#ifndef __weak
#define __weak __attribute__((weak))
#endif /* __weak */

void ShimFactoryTest_setup(factory_test_target_t target, factory_test_t testToRun);

uint32_t ShimFactoryTest_run(void);

void ShimFactoryTest_sendReport(const char *str);
__weak void ShimFactoryTest_sendReportImpl(const char *str,
    factory_test_target_t factoryTestTarget);
factory_test_t ShimFactoryTest_getTestToRun(void);
factory_test_target_t ShimFactoryTest_getTarget(void);

#endif /* TEST_SHIMMER_TEST_H_ */
