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
#include "../shimmer_externs.h"

factory_test_target_t factoryTestTarget;
factory_test_t factoryTestToRun;

char buffer[100];

extern void BlinkTimerStart(void);
extern void BlinkTimerStop(void);
extern void ReadWriteSDTest(void);

void run_factory_test(void)
{
    send_test_report("//**************************** TEST START "
                     "************************************//\r\n");

    if (factoryTestToRun == FACTORY_TEST_MAIN
            || factoryTestToRun == FACTORY_TEST_ICS)
    {
        print_shimmer_model();
        send_test_report("\r\n");
    }

    if (factoryTestToRun == FACTORY_TEST_MAIN
            || factoryTestToRun == FACTORY_TEST_LEDS)
    {
        led_test();

        if (factoryTestToRun == FACTORY_TEST_MAIN)
        {
            send_test_report("\r\n");
        }
    }

    if (factoryTestToRun == FACTORY_TEST_MAIN
            || factoryTestToRun == FACTORY_TEST_ICS)
    {
        sd_card_test();
        send_test_report("\r\n");

        bt_module_test();
        send_test_report("\r\n");

        I2C_test();
    }

    send_test_report("//***************************** TEST END "
                     "*************************************//\r\n");
}

void print_shimmer_model(void)
{
    send_test_report("Shimmer model:\r\n");
    if (isDaughterCardIdSet())
    {
        sprintf(buffer, " - PASS: %s", getDaughtCardIdStrPtr());
        send_test_report(buffer);
        shimmer_expansion_brd *daughterCardId = getDaughtCardId();
        sprintf(buffer, " (SR%d-%d-%d)\r\n", daughterCardId->exp_brd_id,
                daughterCardId->exp_brd_rev,
                daughterCardId->exp_brd_special_rev);
        send_test_report(buffer);
    }
    else
    {
        send_test_report(" - FAIL: not set\r\n");
    }
}

void led_test(void)
{
    // Stop watch dog timer while running LED test
    WDTCTL = WDTPW | WDTHOLD;

    BlinkTimerStop();

    send_test_report("LED test:\r\n");

    Board_ledOff(LED_ALL);
    send_test_report(" - All LEDs off\r\n");
    __delay_cycles(DELAY_BETWEEN_LED_CHANGES_TICKS);

    send_test_report(" - Lower Green LED on\r\n");
    Board_ledOn(LED_GREEN0);
    __delay_cycles(DELAY_BETWEEN_LED_CHANGES_TICKS);
    Board_ledOff(LED_ALL);

    send_test_report(" - Lower Yellow LED on\r\n");
    Board_ledOn(LED_YELLOW);
    __delay_cycles(DELAY_BETWEEN_LED_CHANGES_TICKS);
    Board_ledOff(LED_ALL);

    send_test_report(" - Lower Red LED on\r\n");
    Board_ledOn(LED_RED);
    __delay_cycles(DELAY_BETWEEN_LED_CHANGES_TICKS);
    Board_ledOff(LED_ALL);

    send_test_report(" - Upper Green LED on\r\n");
    Board_ledOn(LED_GREEN1);
    __delay_cycles(DELAY_BETWEEN_LED_CHANGES_TICKS);
    Board_ledOff(LED_ALL);

    send_test_report(" - Upper Blue LED on\r\n");
    Board_ledOn(LED_BLUE);
    __delay_cycles(DELAY_BETWEEN_LED_CHANGES_TICKS);
    Board_ledOff(LED_ALL);

    send_test_report(" - All LEDs off\r\n");
    __delay_cycles(DELAY_BETWEEN_LED_CHANGES_TICKS);

    send_test_report(" - All LEDs on\r\n");
    Board_ledOn(LED_ALL);
    __delay_cycles(DELAY_BETWEEN_LED_CHANGES_TICKS);

    BlinkTimerStart();

    // Reset Watchdog timer
    WDTCTL = WDTPW + WDTSSEL__ACLK + WDTCNTCL + WDTIS_4;
}

void sd_card_test(void)
{
    send_test_report("SD Card:\r\n");
    if (P4IN & BIT1)
    {
        send_test_report(" - FAIL: SD Card not detected\r\n");
    }
    else
    {
        send_test_report(" - PASS: SD card detected\r\n");

        if (P2IN & BIT3)
        {
            send_test_report(
                    " - FAIL: Shimmer is docked so SD card test can not be performed\r\n");
        }
        else
        {
            ReadWriteSDTest();
            sprintf(buffer, " - %s: SD card read/write test\r\n",
                    shimmerStatus.sdBadFile ? "FAIL" : "PASS");
            send_test_report(buffer);
        }
    }
}

void bt_module_test(void)
{
//    if (stat.isBtPoweredOn)
//    {
        send_test_report("BT Module:\r\n");

        send_test_report(" - MAC ID: ");
        BT_getMacAddressAscii(buffer);
        sprintf(&buffer[12], "\r\n");
        send_test_report(buffer);

        sprintf(buffer, " - %s\r\n", getBtVerStrPtr());
        send_test_report(buffer);

        if(strstr(buffer, "RN4678") != NULL && strstr(buffer, "V1.23") == NULL)
        {
            send_test_report(" - FAIL: incorrect BT firmware version\r\n");
        }
        else
        {
            send_test_report(" - PASS\r\n");
        }
//    }
//    else
//    {
//        send_test_report(" - FAIL\r\n");
//    }
//    return stat.isBtPoweredOn;
}

void I2C_test(void)
{
    send_test_report("I2C:\r\n");

    uint8_t eeprom_result = CAT24C16_test();
    sprintf(buffer, " - %s: CAT24C16\r\n", eeprom_result ? "FAIL" : "PASS");
    send_test_report(buffer);

    i2cSlaveDiscover();

    //There is no self test feature implemented yet in S3 but we can at least indicate what chips are detected.
    if(i2cSlavePresent(LSM303DHLC_ACCEL_ADDR))
    {
        send_test_report(" - LSM303DHLC detected (self-test not implemented yet)\r\n");
    }
    if(i2cSlavePresent(LSM303AHTR_ACCEL_ADDR))
    {
        send_test_report(" - LSM303AH detected (self-test not implemented yet)\r\n");
    }
    if(i2cSlavePresent(ICM20948_ADDR))
    {
        send_test_report(" - ICM20948 detected (self-test not implemented yet)\r\n");
    }
    if(i2cSlavePresent(MPU9150_ADDR))
    {
        send_test_report(" - MPU9x50 detected (self-test not implemented yet)\r\n");
    }
    if(i2cSlavePresent(BMP180_ADDR))
    {
        send_test_report(" - BMP180 detected (self-test not implemented yet)\r\n");
    }
    if(i2cSlavePresent(BMP280_ADDR))
    {
        send_test_report(" - BMP280 detected (self-test not implemented yet)\r\n");
    }
}

void setup_factory_test(factory_test_target_t target, factory_test_t testToRun)
{
    factoryTestTarget = target;
    factoryTestToRun = testToRun;
}

void send_test_report(char *str)
{
    switch (factoryTestTarget)
    {
//  case PRINT_TO_DEBUGGER:
//    printf(str);
//    break;
    case PRINT_TO_DOCK_UART:
        DockUart_writeBlocking((uint8_t*) str, strlen(str));
        break;
    case PRINT_TO_BT_UART:
        BT_write((uint8_t*) str, strlen(str), SHIMMER_CMD);
        while (getUsedSpaceInBtTxBuf() > 0)
            ;
        break;
    default:
        break;
    }
}
