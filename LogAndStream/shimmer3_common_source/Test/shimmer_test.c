/*
 * shimmer_test.c
 *
 *  Created on: Aug 14, 2025
 *      Author: MarkNolan
 */

#include "shimmer_test.h"

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>

//#include "log_and_stream_includes.h"
#include "../5xx_HAL/hal_FactoryTest.h"

factory_test_target_t factoryTestTarget = PRINT_TO_DEBUGGER;
factory_test_t factoryTestToRun;

char outputBuffer[100];

uint32_t testResult;

uint32_t ShimFactoryTest_run(void)
{
  ShimFactoryTest_sendReport("//**************************** TEST START "
                             "************************************//\r\n");

  testResult = 0;

  if (factoryTestToRun == FACTORY_TEST_LED_STATES)
  {
//    ShimLeds_testOperationalStates();
  }
  else
  {
    hal_run_factory_test(factoryTestToRun, &outputBuffer[0]);
  }

  if (factoryTestToRun == FACTORY_TEST_MAIN || factoryTestToRun == FACTORY_TEST_ICS)
  {
    if (testResult)
    {
      sprintf(outputBuffer, "\r\nOverall Result = FAIL (0x%08" PRIX32 ")\r\n",
          testResult);
    }
    else
    {
      sprintf(outputBuffer, "\r\nOverall Result = PASS\r\n");
    }
    ShimFactoryTest_sendReport(outputBuffer);
  }

  ShimFactoryTest_sendReport("//***************************** TEST END "
                             "*************************************//\r\n");

  return testResult;
}

void ShimFactoryTest_setup(factory_test_target_t target, factory_test_t testToRun)
{
  factoryTestTarget = target;
  factoryTestToRun = testToRun;
}

void ShimFactoryTest_sendReport(const char *str)
{
  if (str == NULL || strlen(str) == 0)
  {
    return;
  }

  //If the string is too long, truncate it
  if (strlen(str) > MAX_TEST_REPORT_LENGTH)
  {
    char truncatedStr[MAX_TEST_REPORT_LENGTH + 1];
    strncpy(truncatedStr, str, MAX_TEST_REPORT_LENGTH);
    truncatedStr[MAX_TEST_REPORT_LENGTH] = '\0';
    ShimFactoryTest_sendReportImpl(truncatedStr, factoryTestTarget);
  }
  else
  {
    ShimFactoryTest_sendReportImpl(str, factoryTestTarget);
  }
}

__weak void ShimFactoryTest_sendReportImpl(const char *str, factory_test_target_t factoryTestTarget)
{
  //This function can be overridden by the main application to send a test
  //report. The default implementation does nothing.
  (void) str; //Suppress unused parameter warning
}

factory_test_t ShimFactoryTest_getTestToRun(void)
{
  return factoryTestToRun;
}

factory_test_target_t ShimFactoryTest_getTarget(void)
{
  return factoryTestTarget;
}
