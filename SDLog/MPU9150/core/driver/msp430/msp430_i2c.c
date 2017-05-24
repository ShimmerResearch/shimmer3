/*
 * msp430_i2c.c
 *
 *  Created on: 1 Jul 2014
 *      Author: mnolan
 *
 *      Created to act as an adaptor between the Invensense library and the Shimmer I2C driver
 */

#include <stdint.h>
#include "msp430.h"
#include "msp430_i2c.h"
#include "../../../../5xx_HAL/hal_I2C.h"


//void msp430_i2c_initial(void){
//	_NOP(); // not needed in Shimmer I2C driver but used in Invensense library
//}

int msp430_i2c_enable(uint8_t selectClockSource,
	      uint32_t clockSourceFrequency, uint32_t desiredI2CClock) {
	I2C_Master_Init(selectClockSource, clockSourceFrequency, desiredI2CClock);
	return 0;
}


int msp430_i2c_read(unsigned char slave_addr,
                    unsigned char reg_addr,
                    unsigned char length,
                    unsigned char *data) {

	I2C_Set_Slave_Address(slave_addr);
	*data = reg_addr;
	I2C_Read_Packet_From_Sensor(data,length);
	return 0;
}

int msp430_i2c_write(unsigned char slave_addr,
                     unsigned char reg_addr,
                     unsigned char length,
                     unsigned char const *data) {
	uint8_t local_buffer[17], i;

	I2C_Set_Slave_Address(slave_addr);
	local_buffer[0] = reg_addr;
	for(i=0;i<length;i++) {
		local_buffer[i+1] = *(data+i);
	}
	I2C_Write_Packet_To_Sensor(local_buffer,length+1);
	return 0;
}


