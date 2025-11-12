/*
 * shimmer_eeprom.h
 *
 *  Created on: 31 Jul 2025
 *      Author: MarkNolan
 */

#ifndef LOG_AND_STREAM_COMMON_COMMS_SHIMMER_EEPROM_H_
#define LOG_AND_STREAM_COMMON_COMMS_SHIMMER_EEPROM_H_

#include <stdint.h>

#include "../CAT24C16/CAT24C16.h"

//Bluetooth information is stored in the last EEPROM page
#define EEPROM_ADDRESS_HW_DETAILS 0
#define EEPROM_ADDRESS_BLUETOOTH_DETAILS \
  (CAT24C16_TOTAL_SIZE - CAT24C16_PAGE_SIZE)
#define EEPROM_ADDRESS_BLUETOOTH_DETAILS_MINUS_OFFSET \
  (EEPROM_ADDRESS_BLUETOOTH_DETAILS - CAT24C16_PAGE_SIZE)

#define EEPROM_AVAILABLE_SIZE (CAT24C16_TOTAL_SIZE - CAT24C16_PAGE_SIZE)

//Indices of important daughter card information
enum EEPROM_HARDWARE_REVISON
{
  DAUGHT_CARD_ID = 0,
  DAUGHT_CARD_REV = 1,
  DAUGHT_CARD_SPECIAL_REV = 2,
};

enum RADIO_HARDWARE_VERSION
{
  RN42 = 0U,
  RN4678 = 1U,
  RN41 = 2U,
  CYW20820 = 3U,
  BT_HW_VER_UNKNOWN = 0xFF,
};

enum EEPROM_BLUETOOTH_SETTINGS
{
  RADIO_TYPE_IDX = 0,
  BAUD_RATE_IDX = 1,
  RADIO_SETTINGS_IDX = 2,
};

typedef struct
{
  uint8_t exp_brd_id;
  uint8_t exp_brd_major;
  uint8_t exp_brd_minor;
} shimmer_expansion_brd;

typedef union
{
  uint8_t raw[CAT24C16_PAGE_SIZE];

  struct __attribute__((packed))
  {
    shimmer_expansion_brd expansion_brd;
    uint8_t padding[13];
  };
} daughter_card_id_page;

typedef union
{
  uint8_t rawBytes[CAT24C16_PAGE_SIZE];

  struct __attribute__((packed))
  {
    uint8_t radioHwVer;
    uint8_t baudRate;

    uint8_t bleEnabled       : 1;
    uint8_t btClassicEnabled : 1;
    uint8_t unusedIdx3Bit2   : 1;
    uint8_t unusedIdx3Bit3   : 1;
    uint8_t unusedIdx3Bit4   : 1;
    uint8_t unusedIdx3Bit5   : 1;
    uint8_t unusedIdx3Bit6   : 1;
    uint8_t unusedIdx3Bit7   : 1;

#if defined(SHIMMER3)
    uint16_t btCntDisconnectWhileStreaming;
    uint16_t btCntUnsolicitedReboot;
    uint16_t btCntRtsLockup;
    uint16_t btCntDataRateTestBlockage;
    uint8_t padding[5];
#else
    uint8_t padding[13];
#endif
  };
} gEepromBtSettings;

void ShimEeprom_init(void);
void ShimEeprom_setIsPresent(uint8_t eeprom_is_preset);
uint8_t ShimEeprom_isPresent(void);
void ShimEeprom_readAll(void);
void ShimEeprom_readHwDetails(void);
void ShimEeprom_readRadioDetails(void);
void ShimEeprom_writeRadioDetails(void);
void ShimEeprom_updateRadioDetails(void);
uint8_t ShimEeprom_areRadioDetailsIncorrect(void);
#if defined(SHIMMER3)
uint8_t ShimEeprom_checkBtErrorCounts(void);
void ShimEeprom_resetBtErrorCounts(void);
#endif
gEepromBtSettings *ShimEeprom_getRadioDetails(void);
uint8_t ShimEeprom_isBleEnabled(void);
uint8_t ShimEeprom_isBtClassicEnabled(void);
enum RADIO_HARDWARE_VERSION ShimEeprom_getRadioHwVersion(void);
uint8_t ShimEeprom_writeDaughterCardMem(uint16_t memOffset, uint8_t memLength, uint8_t *buf);

#endif /* LOG_AND_STREAM_COMMON_COMMS_SHIMMER_EEPROM_H_ */
