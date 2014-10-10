/*
 * cat24c16.c
 *
 *  Created on: 21 Nov 2013
 *      Author: WeiboP
 */

#include "cat24c16.h"
#include "hal_I2C.h"
#include "msp430.h"
#include "string.h"

#define min(a,b) ((a<b)?a:b)

void CAT24C16_init(void){
   CAT24C16_powerOn();                       //I2C pull-ups must be enabled (on rev4 boards) before initialising I2C bus
	I2C_Master_Init(S_MCLK,24000000,400000);	//Source from SMCLK, which is running @ 24MHz. 4kHz desired BRCLK
}


void CAT24C16_powerOn(void) {
   P8OUT |= BIT4;          //enable I2C pull-ups by turning on SW_I2C
	P3OUT |= BIT3;
	__delay_cycles(48000);  //2ms
}


void CAT24C16_powerOff(void) {
   P8OUT &= ~BIT4;         //disable I2C pull-ups by turning off SW_I2C
   __delay_cycles(120000); //5ms (assuming 24MHz MCLK) to ensure no writes pending
	P3OUT &= ~BIT3;
}


void CAT24C16_read(uint16_t address, uint16_t length, uint8_t *outBuffer) {
	if((!length) || (length>2048) || (address+length>2048))
		return;

	I2C_Set_Slave_Address(CAT24C16_ADDR|(address>>8));
	*outBuffer = (uint8_t)(address & 0xFF);

	if(!I2C_Read_Packet_From_Sensor(outBuffer, length)) {
	   //read failed, set buffer to all 0xFF
	   while(length--) *(outBuffer++) = 0xFF;
	}
}


//testing features:
//1 bytes writing: 0.087 ms
//16 bytes writing: 0.5 ms (within one page)
//16 bytes writing: 5.7 ms (cross pages)
//waiting time between two page writes: 5 ms (as suggested by the CAT24 user manual)
//1024 bytes writing: 351 ms
void CAT24C16_write(uint16_t address, uint16_t length, uint8_t *data){
	if((!length) || (length>2048) || (address+length>2048))
		return;

	uint8_t final_addr;
	uint8_t buf[17], margin, this_write, inc_addr;//17 = PAGE_SIZE+1
	uint16_t mem_ptr, buf_offset;

	final_addr = CAT24C16_ADDR|(address>>8);
    I2C_Set_Slave_Address(final_addr);

    inc_addr=0;
    buf_offset=0;
    mem_ptr = address;
    this_write=0;

    while(mem_ptr < address + length){
    	//reassign I2C slave address
    	if(inc_addr){
    		inc_addr=0;
    	    I2C_Set_Slave_Address(++final_addr);
    	}

		__delay_cycles(120000);//5ms
    	//bounds check
    	margin = PAGE_SIZE - (mem_ptr % PAGE_SIZE);
        this_write = min(margin, address + length - mem_ptr);

    	//set byte address
        buf[0] = mem_ptr & 0xff;
    	// data copy
    	memcpy(buf+1, data+buf_offset, this_write);

    	I2C_Write_Packet_To_Sensor(buf,this_write+1);

    	mem_ptr += this_write;
    	buf_offset += this_write;

    	//if reaches edge of flash mem, reassign the slave address
    	if(!(mem_ptr % BLOCK_SIZE))
    		inc_addr=1;
    }
}


