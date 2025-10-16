/*
 * hal_cat24c16.c
 *
 *  Created on: 31 Jul 2025
 *      Author: MarkNolan
 */

#include "hal_CAT24C16.h"

#include <stdint.h>

#include "CAT24C16.h"
#include "hal_Board.h"
#include "log_and_stream_globals.h"
#include "msp430.h"

extern void BlinkTimerStart(void);
extern void BlinkTimerStop(void);

void eepromRead(uint16_t dataAddr, uint16_t dataSize, uint8_t *dataBuf)
{
  eepromReadWrite(dataAddr, dataSize, dataBuf, EEPROM_READ);
}

void eepromWrite(uint16_t dataAddr, uint16_t dataSize, uint8_t *dataBuf)
{
  eepromReadWrite(dataAddr, dataSize, dataBuf, EEPROM_WRITE);
}

void eepromReadWrite(uint16_t dataAddr, uint16_t dataSize, uint8_t *dataBuf, enum EEPROM_RW eepromRW)
{
  uint8_t timer_was_stopped = 0;
  uint8_t wasI2cRunning = shimmerStatus.pinPvI2c;
  uint8_t wasExpBoardRunning = shimmerStatus.pinPvExt;

  //Spool up EEPROM and required timing peripherals
  if (TB0CTL == MC_0) //Timer is stopped
  {
    BlinkTimerStart();
    timer_was_stopped = 1;
  }

  CAT24C16_init();

  //EEPROM needs to be updated with latest bt baud rate, configure here
  if (eepromRW == EEPROM_READ)
  {
    CAT24C16_read(dataAddr, dataSize, dataBuf);
  }
  else
  {
    CAT24C16_write(dataAddr, dataSize, dataBuf);
  }

  //Wind down EEPROM and required timing peripherals
  if (!wasI2cRunning)
  {
    CAT24C16_powerOff(); //turn OFF if it was not already ON
  }
  else if (!wasExpBoardRunning)
  {
    Board_setExpansionBrdPower(0); //turn OFF if it was not already ON
  }
  if (timer_was_stopped)
  {
    BlinkTimerStop();
  }
}
