/*
 * cat24c16.c
 *
 *  Created on: 21 Nov 2013
 *      Author: WeiboP
 */

#include "cat24c16.h"

#include "../5xx_HAL/hal_Board.h"
#include "../5xx_HAL/hal_I2C.h"
#include "../5xx_HAL/hal_RTC.h"
#include "msp430.h"
#include "string.h"

#define min(a, b) ((a < b) ? a : b)

extern void I2C_start(uint8_t controlExpBrd);
extern void I2C_stop(uint8_t controlExpBrd);

void CAT24C16_init(void)
{
  CAT24C16_powerOn(); //I2C pull-ups must be enabled (on rev4 boards) before initialising I2C bus
  I2C_Master_Init(S_MCLK, 24000000,
      400000); //Source from SMCLK, which is running @ 24MHz. 400kHz desired BRCLK
}

void CAT24C16_powerOn(void)
{
  I2C_start(1);
}

void CAT24C16_powerOff(void)
{
  I2C_stop(1);
}

int32_t CAT24C16_read(uint16_t address, uint16_t length, uint8_t *outBuffer)
{
  if ((!length) || (length > EEPROM_MAX_SIZE_IN_BYTES)
      || (address + length > EEPROM_MAX_SIZE_IN_BYTES))
  {
    return CAT24C16_OUT_OF_BOUNDS_ERROR;
  }

  I2C_Set_Slave_Address(CAT24C16_ADDR | (address >> 8));
  *outBuffer = (uint8_t) (address & 0xFF);

  if (!I2C_Read_Packet_From_Sensor(outBuffer, length))
  {
    //read failed, set buffer to all 0xFF
    while (length--)
    {
      *(outBuffer++) = 0xFF;
    }
    return 1;
  }
  return 0;
}

//testing features:
//1 bytes writing: 0.087 ms
//16 bytes writing: 0.5 ms (within one page)
//16 bytes writing: 5.7 ms (cross pages)
//waiting time between two page writes: 5 ms (as suggested by the CAT24 user
//manual) 1024 bytes writing: 351 ms
int32_t CAT24C16_write(uint16_t address, uint16_t length, uint8_t *data)
{
  if ((!length) || (length > EEPROM_MAX_SIZE_IN_BYTES)
      || (address + length > EEPROM_MAX_SIZE_IN_BYTES))
  {
    return CAT24C16_OUT_OF_BOUNDS_ERROR;
  }

  int32_t result = 0;
  uint8_t final_addr;
  uint8_t buf[17], margin, this_write, inc_addr; //17 = PAGE_SIZE+1
  uint16_t mem_ptr, buf_offset;

  final_addr = CAT24C16_ADDR | (address >> 8);
  I2C_Set_Slave_Address(final_addr);

  inc_addr = 0;
  buf_offset = 0;
  mem_ptr = address;
  this_write = 0;

  while (mem_ptr < address + length)
  {
    //reassign I2C slave address
    if (inc_addr)
    {
      inc_addr = 0;
      I2C_Set_Slave_Address(++final_addr);
    }

    __delay_cycles(120000); //5ms
    //bounds check
    margin = CAT24C16_PAGE_SIZE - (mem_ptr % CAT24C16_PAGE_SIZE);
    this_write = min(margin, address + length - mem_ptr);

    //set byte address
    buf[0] = mem_ptr & 0xff;
    //data copy
    memcpy(buf + 1, data + buf_offset, this_write);

    result = I2C_Write_Packet_To_Sensor(buf, this_write + 1) ? 0 : 1;
    if (result)
    {
      break;
    }

    mem_ptr += this_write;
    buf_offset += this_write;

    //if reaches edge of flash mem, reassign the slave address
    if (!(mem_ptr % CAT24C16_BLOCK_SIZE))
    {
      inc_addr = 1;
    }
  }
  return result;
}

//returns 0 if successful, 1 if failure
uint8_t CAT24C16_test(void)
{
  int32_t result = 0;
  uint16_t i, j = 0;
  uint8_t test_eeprom_backup[CAT24C16_TEST_SIZE];
  uint8_t test_eeprom_wr[CAT24C16_TEST_SIZE];
  uint8_t test_eeprom_rd[CAT24C16_TEST_SIZE];

  result = CAT24C16_read(CAT24C16_TEST_OFFSET, CAT24C16_TEST_SIZE, test_eeprom_backup);
  if (result)
  {
    return EEPROM_TEST_FAIL_INITIAL_BACKUP;
  }

  while (j++ < 3)
  {
    //SRAND/RAND not working on MSP430 for some reason
    //srand((unsigned) RTC_get32());
    //for (i = 0; i < CAT24C16_TEST_SIZE; i++)
    //{
    //    test_eeprom_wr[i] = rand();
    //}

    //Increment value in
    for (i = 0; i < CAT24C16_TEST_SIZE; i++)
    {
      test_eeprom_wr[i] = j + i;
    }

    memset(test_eeprom_rd, 0, CAT24C16_TEST_SIZE);
    result = CAT24C16_write(CAT24C16_TEST_OFFSET, CAT24C16_TEST_SIZE, test_eeprom_wr);
    __delay_cycles(120000); //5ms
    if (result)
    {
      return EEPROM_TEST_FAIL_WRITING_BUF;
    }
    result = CAT24C16_read(CAT24C16_TEST_OFFSET, CAT24C16_TEST_SIZE, test_eeprom_rd);
    if (result)
    {
      return EEPROM_TEST_FAIL_READING_BUF;
    }

    if (memcmp(test_eeprom_wr, test_eeprom_rd, CAT24C16_TEST_SIZE))
    {
      return EEPROM_TEST_FAIL_BUF_COMPARISON;
    }
  }

  result = CAT24C16_write(CAT24C16_TEST_OFFSET, CAT24C16_TEST_SIZE, test_eeprom_backup);
  __delay_cycles(120000); //5ms

  if (result)
  {
    return EEPROM_TEST_FAIL_RESTORING_BACKUP;
  }

  return EEPROM_TEST_PASS;
}
