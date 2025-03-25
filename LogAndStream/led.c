/*
 * led.c
 *
 *  Created on: 25 Mar 2025
 *      Author: MarkNolan
 */

#include "led.h"

#include "Shimmer_Driver/shimmer_driver_include.h"

volatile uint8_t fileBadCnt;
uint16_t blinkCnt20, blinkCnt50;
uint8_t lastLedGroup2, rwcErrorFlash;

void LED_varsInit(void)
{
    fileBadCnt = 0;
    blinkCnt20 = blinkCnt50 = 0;
    lastLedGroup2 = 0;
    rwcErrorFlash = 0;
}

void LED_incrementCounters(void)
{
    // Note: each count is 0.1s
    if (blinkCnt50++ == 49)
        blinkCnt50 = 0;

    if (blinkCnt20++ == 19)
    {
        blinkCnt20 = 0;
    }
}

void LED_controlDuringBoot(boot_stage_t bootStageCurrent)
{
    switch (bootStageCurrent)
    {
    case BOOT_STAGE_I2C:
        Board_ledToggle(LED_LWR_RED);
        break;
    case BOOT_STAGE_BLUETOOTH_FAILURE:
        Board_ledToggle(LED_LWR_YELLOW);
        break;
    default:
        break;
    }
}

void LED_control(void)
{
    // below are settings for green0, yellow and red leds, battery charge status
    if (shimmerStatus.toggleLedRedCmd)
    {
        Board_ledOff(LED_LWR_GREEN + LED_LWR_YELLOW);
        Board_ledOn(LED_LWR_RED);
    }
    else if (BOARD_IS_DOCKED) // Docked
    {
        Board_ledOff(LED_LWR_GREEN + LED_LWR_YELLOW + LED_LWR_RED);
        if (batteryStatus.battStatLedCharging != LED_ALL_OFF)
        {
            if (batteryStatus.battStatLedFlash)
            {
                Board_ledToggle(batteryStatus.battStatLedCharging);
            }
            else
            {
                Board_ledOn(batteryStatus.battStatLedCharging);
            }
        }
    }
    else
    {
        Board_ledOff(LED_LWR_GREEN + LED_LWR_YELLOW + LED_LWR_RED);
        if (!blinkCnt50)
        {
            LED_battBlinkOn();
        }
    }

    // code for keeping LED_GREEN0 on when user button pressed
    if ((!BOARD_IS_BTN_RELEASED))
    {
        Board_ledOn(LED_LWR_GREEN);
    }

    // below are settings for green1, blue, yellow and red leds

    if (!shimmerStatus.docked
            && (shimmerStatus.sdBadFile || !shimmerStatus.sdInserted)
            && ShimConfig_getStoredConfig()->sdErrorEnable)
    {   // bad file = yellow/red alternating
        if (fileBadCnt-- > 0)
        {
            if (fileBadCnt & 0x01)
            {
                Board_ledOn(LED_LWR_YELLOW);
                Board_ledOff(LED_LWR_RED);
            }
            else
            {
                Board_ledOff(LED_LWR_YELLOW);
                Board_ledOn(LED_LWR_RED);
            }
        }
        else
        {
            fileBadCnt = 255;
            Board_ledOn(LED_LWR_YELLOW);
            Board_ledOff(LED_LWR_RED);
        }
    }
    if (rwcErrorFlash && (!shimmerStatus.sensing))
    {
        if (BOARD_IS_LED_GREEN1_ON)
        {
            Board_ledOff(LED_UPR_GREEN);
            Board_ledOn(LED_UPR_BLUE);
        }
        else
        {
            Board_ledOn(LED_UPR_GREEN);
            Board_ledOff(LED_UPR_BLUE);
        }
    }
    else
    {
        if (shimmerStatus.btSupportEnabled
                 && !shimmerStatus.sdSyncEnabled)
        {
            if (!shimmerStatus.sensing)
            { //standby or configuring
                if (shimmerStatus.configuring)
                { //configuring
                    Board_ledToggle(LED_UPR_GREEN);
                }
                else if (shimmerStatus.btConnected && !shimmerStatus.configuring)
                {
                    Board_ledOn(LED_UPR_BLUE);
                    Board_ledOff(LED_UPR_GREEN);     // nothing to show
                }
                else if (isRn4678ConnectionEstablished())
                {
                    /* BT connection established but RFComm not open */
                    Board_ledToggle(LED_UPR_BLUE);
                    Board_ledOff(LED_UPR_GREEN);         // nothing to show
                }
                else
                {                           //standby
                    if (LED_isBlinkCntTime2s())
                    {
                        Board_ledOn(LED_UPR_BLUE);
                    }
                    else
                    {
                        Board_ledOff(LED_UPR_BLUE);
                    }
                    Board_ledOff(LED_UPR_GREEN);         // nothing to show
                }
            }
            else
            {                           //sensing
                // shimmerStatus.sdlogReady, shimmerStatus.btstreamReady, enableSdlog, enableBtstream
                // btstream only
                if ((shimmerStatus.btStreaming && shimmerStatus.btstreamReady)
                        && !(shimmerStatus.sdlogReady && shimmerStatus.sdLogging))
                {
                    if (LED_isBlinkCntTime1s())
                    {
                        Board_ledToggle(LED_UPR_BLUE);
                    }
                    Board_ledOff(LED_UPR_GREEN);     // nothing to show
                }
                // sdlog only
                else if (!(shimmerStatus.btStreaming && shimmerStatus.btstreamReady)
                        && (shimmerStatus.sdlogReady && shimmerStatus.sdLogging))
                {
                    if (LED_isBlinkCntTime1s())
                    {
                        Board_ledToggle(LED_UPR_GREEN);
                    }
                    Board_ledOff(LED_UPR_BLUE);       // nothing to show
                }
                // btstream & sdlog
                else if ((shimmerStatus.btStreaming && shimmerStatus.btstreamReady)
                        && (shimmerStatus.sdlogReady && shimmerStatus.sdLogging))
                {
                    if (LED_isBlinkCntTime1s())
                    {
                        if (BOARD_IS_LED_BLUE_ON || BOARD_IS_LED_GREEN1_ON)
                        {
                            Board_ledOff(LED_UPR_BLUE + LED_UPR_GREEN);
                        }
                        else
                        {
                            if (lastLedGroup2)
                            {
                                Board_ledOn(LED_UPR_BLUE);
                                lastLedGroup2 ^= 1;
                            }
                            else
                            {
                                Board_ledOn(LED_UPR_GREEN);
                                lastLedGroup2 ^= 1;
                            }
                        }
                    }
                }
                else
                {
                    Board_ledOff(LED_UPR_GREEN + LED_UPR_BLUE); // nothing to show
                }
            }
        }
        else
        {
            // good file - green1:
            if (!shimmerStatus.sensing)
            {   //standby or configuring
                if (shimmerStatus.configuring)
                { //configuring
                    Board_ledToggle(LED_UPR_GREEN);
                }
                else
                {                            //standby
                    if (LED_isBlinkCntTime2s())
                    {
                        Board_ledOn(LED_UPR_GREEN);
                    }
                    else
                    {
                        Board_ledOff(LED_UPR_GREEN);
                    }
                }
            }
            else
            {                              //sensing
                if (blinkCnt20 < 10)
                {
                    Board_ledOn(LED_UPR_GREEN);
                }
                else
                {
                    Board_ledOff(LED_UPR_GREEN);
                }
            }

            // good file - blue:
            /* Toggle blue LED while a connection is established */
            if (shimmerStatus.btPowerOn && shimmerStatus.btConnected)
            {
                Board_ledToggle(LED_UPR_BLUE);
            }
            /* Leave blue LED on solid if it's a node and a sync hasn't occurred yet (first 'outlier' not included) */
            else if (!ShimSdSync_rcFirstOffsetRxedGet()
                    && shimmerStatus.sensing
                    && shimmerStatus.sdSyncEnabled
                    && !(ShimSdHead_sdHeadTextGetByte(SDH_TRIAL_CONFIG0) & SDH_IAMMASTER))
            {
                Board_ledOn(LED_UPR_BLUE);
            }
            else
            {
                /* Flash if BT is on */
                if (shimmerStatus.btPowerOn
                        && (blinkCnt20 == 12 || blinkCnt20 == 14))
                {
                    Board_ledOn(LED_UPR_BLUE);
                }
                else
                {
                    Board_ledOff(LED_UPR_BLUE);
                }

                //TODO original SD sync blink code is below, trying to simplify things above. Decide whether to keep above and then remove below.

//                        /* Flash twice if sync is not successfull */
//                        if (((blinkCnt20 == 12) || (blinkCnt20 == 14))
//                                && !shimmerStatus.docked
//                                && shimmerStatus.sensing
//                                && shimmerStatus.sdSyncEnabled
//                                && ((!getSyncSuccC() && (sdHeadText[SDH_TRIAL_CONFIG0] & SDH_IAMMASTER))
//                                        || (!getSyncSuccN() && !(sdHeadText[SDH_TRIAL_CONFIG0] & SDH_IAMMASTER))))
//                        {
//                            if (getSyncCnt() > 3)
//                            {
//                                Board_ledOn(LED_BLUE);
//                            }
//                            //TODO should there be an else here?
//                        }
//                        else
//                        {
//                            Board_ledOff(LED_BLUE);
//                        }
            }
        }
    }
}

void LED_battBlinkOn()
{
    switch (batteryStatus.battStat)
    {
    case BATT_HIGH:
        Board_ledOn(LED_LWR_GREEN);
        break;
    case BATT_MID:
        Board_ledOn(LED_LWR_YELLOW);
        break;
    case BATT_LOW:
        Board_ledOn(LED_LWR_RED);
        break;
    default:
        break;
    }
}

uint8_t LED_isBlinkCntTime1s(void)
{
    return (blinkCnt20 % 10 == 0);
}

uint8_t LED_isBlinkCntTime2s(void)
{
    return (blinkCnt20 == 0);
}

void RwcCheck(void)
{
#if TEST_RTC_ERR_FLASH_OFF
    rwcErrorFlash = 0;
#else
    rwcErrorFlash = ((!getRwcTimeDiff()) && ShimConfig_getStoredConfig()->rtcErrorEnable) ? 1 : 0;
#endif
}
