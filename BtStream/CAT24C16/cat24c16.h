#ifndef CAT24C16_H
#define CAT24C16_H

#include <stdint.h>

#define CAT24C16_ADDR	0x50 //7 bit address I2C address
							 //lower 3 bits are highest order bits of memory address
#define PAGE_SIZE		16
#define BLOCK_SIZE		256
#define READ_SIZE		128


//initialise the I2C for use with the CAT24C16
//and also power on
void CAT24C16_init(void);

//power on the CAT24C16 chip
void CAT24C16_powerOn(void);

//power off the CAT24C16 chip
void CAT24C16_powerOff(void);

//Read from the CAT24C16 EEPROM
//address = starting address to read from
//length = number of bytes to read
//outBuffer = location to put read bytes
//Note the CAT24C16 had 2048 bytes of storage
//So address + length must be <= 2048
void CAT24C16_read(uint16_t address, uint16_t length, uint8_t *outBuffer);

//Write to the CAT24C16 EEPROM
//address = starting address to write to
//length = number of bytes to write
//data = bytes to write
//Note the CAT24C16 had 2048 bytes of storage
//So address + length must be <= 2048
//It takes 5ms per write cycle
//Each write cycle can write up to 16bytes of data
//but only within a 16-byte page (of which there are
//128 in the CAT24C16)
void CAT24C16_write(uint16_t address, uint16_t length, uint8_t *data);

#endif //CAT24C16_H
