/*
 * shimmer_eeprom.c
 *
 *  Created on: 31 Jul 2025
 *      Author: MarkNolan
 */

#include "shimmer_eeprom.h"
#include <stdint.h>

#include "../Bluetooth_SD/RN4X.h"
#include "../Boards/shimmer_boards.h"

uint8_t eepromIsPresent = 0;
gEepromBtSettings eepromBtSettings;

void ShimEeprom_init(void)
{
  ShimEeprom_setIsPresent(0);
  memset((uint8_t *) &eepromBtSettings, 0xFF, sizeof(eepromBtSettings));
}

void ShimEeprom_setIsPresent(uint8_t eeprom_is_preset)
{
  eepromIsPresent = eeprom_is_preset;
}

uint8_t ShimEeprom_isPresent(void)
{
  return eepromIsPresent;
}

void ShimEeprom_readAll(void)
{
  /* Read Daughter card ID */
  ShimEeprom_readHwDetails();
  //Read Bluetooth configuration parameters from EEPROM
  ShimEeprom_readRadioDetails();
}

void ShimEeprom_readHwDetails(void)
{
  eepromRead(EEPROM_ADDRESS_HW_DETAILS, CAT24C16_PAGE_SIZE, ShimBrd_getDaughtCardIdPtr());
}

void ShimEeprom_readRadioDetails(void)
{
  eepromRead(EEPROM_ADDRESS_BLUETOOTH_DETAILS,
      sizeof(eepromBtSettings.rawBytes), &eepromBtSettings.rawBytes[0]);
}

void ShimEeprom_writeRadioDetails(void)
{
  eepromWrite(EEPROM_ADDRESS_BLUETOOTH_DETAILS,
      sizeof(eepromBtSettings.rawBytes), &eepromBtSettings.rawBytes[0]);
}

void ShimEeprom_updateRadioDetails(void)
{
  eepromBtSettings.radioHwVer = (uint8_t) ShimEeprom_getRadioHwVersion();
#if defined(SHIMMER3) || defined(SHIMMER4_SDK)
  if (isBtDeviceRn41orRN42())
  {
    eepromBtSettings.baudRate = BAUD_115200;
    eepromBtSettings.bleEnabled = 0; //BLE not supprted in RN42
  }
  else
  {
    eepromBtSettings.baudRate = ShimBt_getBtBaudRateToUse();
  }
#else
  eepromBtSettings.baudRate = ShimBt_getBtBaudRateToUse();
#endif
  //leave eepromBtSettings.bleDisabled as is
}

uint8_t ShimEeprom_areRadioDetailsIncorrect(void)
{
  return (eepromBtSettings.radioHwVer != ShimEeprom_getRadioHwVersion()
      || eepromBtSettings.baudRate == 0xFF
#if defined(SHIMMER3) || defined(SHIMMER4_SDK)
      || (isBtDeviceRn41orRN42() && eepromBtSettings.baudRate != BAUD_115200)
      || (isBtDeviceRn4678() && eepromBtSettings.baudRate != ShimBt_getBtBaudRateToUse())
      || (!ShimBrd_doesDeviceSupportBle() && eepromBtSettings.bleEnabled != 0)
#else
      || eepromBtSettings.baudRate != ShimBt_getBtBaudRateToUse()
#endif
  );
}

#if defined(SHIMMER3)
uint8_t ShimEeprom_checkBtErrorCounts(void)
{
  if (eepromBtSettings.btCntDisconnectWhileStreaming == 0xFFFF
      || eepromBtSettings.btCntUnsolicitedReboot == 0xFFFF
      || eepromBtSettings.btCntRtsLockup == 0xFFFF
      || eepromBtSettings.btCntDataRateTestBlockage == 0xFFFF)
  {
    ShimEeprom_resetBtErrorCounts();
    return 1;
  }
  return 0;
}

void ShimEeprom_resetBtErrorCounts(void)
{
  eepromBtSettings.btCntDisconnectWhileStreaming = 0;
  eepromBtSettings.btCntUnsolicitedReboot = 0;
  eepromBtSettings.btCntRtsLockup = 0;
  eepromBtSettings.btCntDataRateTestBlockage = 0;
}
#endif

gEepromBtSettings *ShimEeprom_getRadioDetails(void)
{
  return &eepromBtSettings;
}

uint8_t ShimEeprom_isBleEnabled(void)
{
  return eepromBtSettings.bleEnabled;
}

uint8_t ShimEeprom_isBtClassicEnabled(void)
{
  return eepromBtSettings.btClassicEnabled;
}

enum RADIO_HARDWARE_VERSION ShimEeprom_getRadioHwVersion(void)
{
#if defined(SHIMMER4_SDK)
  return RN42;
#elif defined(SHIMMER3)
  if (isBtDeviceRn42())
  {
    return RN42;
  }
  else if (isBtDeviceRn4678())
  {
    return RN4678;
  }
  else if (isBtDeviceRn41())
  {
    return RN41;
  }
  return BT_HW_VER_UNKNOWN;
#else
  return CYW20820;
#endif
}

/* This function skips the first page of the EEPROM as this is reserved
 * for HW information and therefore an offset of 0 is actually the start
 * of the second page in the EEPROM. */
uint8_t ShimEeprom_writeDaughterCardMem(uint16_t memOffset, uint8_t memLength, uint8_t *buf)
{
  uint16_t writeStart = memOffset;
  uint16_t writeEnd = memOffset + memLength - 1;
  uint16_t targetAddr = EEPROM_ADDRESS_BLUETOOTH_DETAILS_MINUS_OFFSET + RADIO_SETTINGS_IDX;

  if ((memLength <= 128) && (writeEnd < EEPROM_AVAILABLE_SIZE))
  {
    eepromWrite(memOffset + CAT24C16_PAGE_SIZE, (uint16_t) memLength, buf);

    /* Handle if the BLE/BT state is being changed */
    if (writeStart <= targetAddr && writeEnd >= targetAddr)
    {
      ShimEeprom_readRadioDetails();
    }

    return 1;
  }
  return 0;
}
