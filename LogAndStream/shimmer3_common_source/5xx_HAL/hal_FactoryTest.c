/*
 * hal_FactoryTest.c
 *
 *  Created on: 14 Aug 2024
 *      Author: MarkNolan
 */

#include "hal_FactoryTest.h"

#include <stdio.h>

#include "../../shimmer_btsd.h"

#include "msp430.h"
#include "../shimmer_sd_include.h"

factory_test_target_t factoryTestTarget;
factory_test_t factoryTestToRun;

extern void BlinkTimerStart(void);
extern void BlinkTimerStop(void);
extern void ReadWriteSDTest(void);
extern bool SD_ERROR;

void run_factory_test(void)
{
    send_test_report("//**************************** TEST START ************************************//\r\n");

    led_test();
    send_test_report("\r\n");

    sd_card_test();
    send_test_report("\r\n");

//    if (stat.isBtPoweredOn)
//    {
//      sprintf(buffer, "BT Module: %.*s\r\n", getBtVerStrLen(), getBtVerStrPtr());
      send_test_report("BT Module:\r\n - ");
      send_test_report(getBtVerStrPtr());
      send_test_report("\r\n");
//    }

    send_test_report("//***************************** TEST END *************************************//\r\n");
}

void led_test(void)
{
    // Stop watch dog timer while running LED test
    WDTCTL = WDTPW | WDTHOLD;

    BlinkTimerStop();

    send_test_report("LED test:\r\n");

    Board_ledOff(LED_ALL);

    send_test_report(" - Lower Green on\r\n");
    Board_ledOn(LED_GREEN0);
    __delay_cycles(DELAY_BETWEEN_LED_CHANGES_TICKS);
    Board_ledOff(LED_ALL);

    send_test_report(" - Lower Yellow on\r\n");
    Board_ledOn(LED_YELLOW);
    __delay_cycles(DELAY_BETWEEN_LED_CHANGES_TICKS);
    Board_ledOff(LED_ALL);

    send_test_report(" - Lower Red on\r\n");
    Board_ledOn(LED_RED);
    __delay_cycles(DELAY_BETWEEN_LED_CHANGES_TICKS);
    Board_ledOff(LED_ALL);

    send_test_report(" - Upper Green on\r\n");
    Board_ledOn(LED_GREEN1);
    __delay_cycles(DELAY_BETWEEN_LED_CHANGES_TICKS);
    Board_ledOff(LED_ALL);

    send_test_report(" - Upper Blue on\r\n");
    Board_ledOn(LED_BLUE);
    __delay_cycles(DELAY_BETWEEN_LED_CHANGES_TICKS);
    Board_ledOff(LED_ALL);

    send_test_report(" - All off\r\n");
    __delay_cycles(DELAY_BETWEEN_LED_CHANGES_TICKS);

    send_test_report(" - All on\r\n");
    Board_ledOn(LED_ALL);
    __delay_cycles(DELAY_BETWEEN_LED_CHANGES_TICKS);

    BlinkTimerStart();

    // Reset Watchdog timer
    WDTCTL = WDTPW + WDTSSEL__ACLK + WDTCNTCL + WDTIS_4;
}

void sd_card_test(void)
{
    char buffer[100];

    if (P4IN & BIT1)
    {
        send_test_report("FAIL: SD Card not detected\r\n");
    }
    else
    {
        send_test_report("PASS: SD card detected\r\n");

        if (P2IN & BIT3)
        {
            send_test_report("FAIL: Shimmer is docked so SD card test can not be performed\r\n");
        }
        else
        {
            ReadWriteSDTest();
            sprintf(buffer, "%s: SD card read/write test\r\n", SD_ERROR ? "FAIL" : "PASS");
            send_test_report(buffer);
        }
    }
}

void setup_factory_test(factory_test_target_t target, factory_test_t testToRun)
{
  factoryTestTarget = target;
  factoryTestToRun = testToRun;
}

void send_test_report(char *str)
{
  switch(factoryTestTarget)
  {
//  case PRINT_TO_DEBUGGER:
//    printf(str);
//    break;
  case PRINT_TO_DOCK_UART:
    DockUart_writeBlocking((uint8_t *)str, strlen(str));
    break;
  case PRINT_TO_BT_UART:
    BT_write((uint8_t *)str, strlen(str), SHIMMER_CMD);
    while(getUsedSpaceInBtTxBuf()>0);
    break;
  default:
    break;
  }
}
