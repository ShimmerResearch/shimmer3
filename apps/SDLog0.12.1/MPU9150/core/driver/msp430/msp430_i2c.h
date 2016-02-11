/*
 * msp430_i2c.h
 *
 *  Created on: 1 Jul 2014
 *      Author: mnolan
 */

#ifndef MSP430_I2C_H_
#define MSP430_I2C_H_

#include <stdint.h>


//void msp430_i2c_initial(void);

int msp430_i2c_enable(uint8_t selectClockSource,
	      uint32_t clockSourceFrequency, uint32_t desiredI2CClock);

/**
 *  @brief      Write to a device register.
 *
 *  @param[in]  slave_addr  Slave address of device.
 *  @param[in]  reg_addr	Slave register to be written to.
 *  @param[in]  length      Number of bytes to write.
 *  @param[out] data        Data to be written to register.
 *
 *  @return     0 if successful.
 */
int msp430_i2c_write(unsigned char slave_addr,
                     unsigned char reg_addr,
                     unsigned char length,
                     unsigned char const *data);
/**
 *  @brief      Read from a device.
 *
 *  @param[in]  slave_addr  Slave address of device.
 *  @param[in]  reg_addr	Slave register to be read from.
 *  @param[in]  length      Number of bytes to read.
 *  @param[out] data        Data from register.
 *
 *  @return     0 if successful.
 */
int msp430_i2c_read(unsigned char slave_addr,
                    unsigned char reg_addr,
                    unsigned char length,
                    unsigned char *data);


#endif /* MSP430_I2C_H_ */
