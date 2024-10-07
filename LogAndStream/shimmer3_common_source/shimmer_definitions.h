/*
 * shimmer_definitions.h
 *
 *  Created on: 7 Oct 2024
 *      Author: MarkNolan
 */

#ifndef SHIMMER3_COMMON_SOURCE_SHIMMER_DEFINITIONS_H_
#define SHIMMER3_COMMON_SOURCE_SHIMMER_DEFINITIONS_H_

#define IS_SUPPORTED_SINGLE_TOUCH   0

typedef volatile struct STATTypeDef_t
{ //STATUS
    uint8_t isInitialising:1;
    uint8_t isSensing:1;
    uint8_t isDocked:1;
    uint8_t isLogging:1;
    uint8_t isStreaming:1;
    uint8_t isConfiguring:1;
#if defined(SHIMMER3R)
    uint8_t isButtonPressed:1;
#endif
    uint8_t isBtConnected:1;
    uint8_t isBtPoweredOn:1;
    uint8_t isBtSupportEnabled:1;
    uint8_t isSdInserted:1;
#if defined(SHIMMER3R)
    uint8_t isLedRed:1;
    uint8_t isDockUartStep;
#endif
    uint8_t battStat:1;
#if defined(SHIMMER3R)
    uint32_t battStatLed;
#endif
    uint8_t battVal[3];
#if defined(SHIMMER4_SDK)
    uint8_t battDigital[10];
#endif
    uint8_t sdlogReady:1;
    uint8_t btstreamReady:1;
    uint8_t badFile:1;
    uint8_t sdlogCmd:2;
    uint8_t btstreamCmd:2;
    uint8_t toggleLedRedCmd:1;
#if defined(SHIMMER3R)
    uint32_t testResult;
    uint8_t pinPvI2c;
    uint8_t pinPvSd;
    uint8_t pinPvExt;
    uint8_t periStat;
#endif
    uint8_t isSyncEnabled:1;
} STATTypeDef;

typedef enum
{
    BOOT_STAGE_START,
    BOOT_STAGE_I2C,
    BOOT_STAGE_BLUETOOTH,
    BOOT_STAGE_BLUETOOTH_FAILURE,
    BOOT_STAGE_CONFIGURATION,
    BOOT_STAGE_END
} boot_stage_t;

#endif /* SHIMMER3_COMMON_SOURCE_SHIMMER_DEFINITIONS_H_ */
