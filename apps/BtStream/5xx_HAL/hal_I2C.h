/*
 * Adapted from Texas Instruments supplied example code
 */

/*******************************************************************************
 *
 * Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

#ifndef HAL_I2C_H
#define HAL_I2C_H

#include <stdint.h>

// I2C Settings for master initialization
//****************************************************************************//
#define  I2C_SOURCE_CLOCK        0x00  //0x00 SMCLK or 0x01 ACLK
// Comment out the source clock which will not be used
#define  I2C_SOURCE_CLOCK_FREQ   SMCLK_FREQ
//#define     SPI_SOURCE_CLOCK_FREQ  ACLK_FREQ
#define  I2C_CLOCK_FREQ          400000
//****************************************************************************//

#define SDA_BIT         BIT1
#define SCL_BIT         BIT2

#define UCB0RXIE        BIT0
#define UCB0TXIE        BIT1
#define UCB0RXIFG       BIT0
#define UCB0TXIFG       BIT1

#define S_MCLK          0x00
//! Return value when function is OK
#define RET_OK          1
// Return value when there's an error
#define RET_ERR         0

// Global Functions
extern void I2C_Master_Init(uint8_t selectClockSource,
               uint32_t clockSourceFrequency,
               uint32_t desiredI2CClock);

extern void I2C_Set_Slave_Address(uint8_t slaveAddress);
extern void I2C_Enable(void);
extern void I2C_Disable(void);
extern uint16_t I2C_Bus_Busy(void);
extern uint16_t I2C_Busy(void);
extern void I2C_Interrupt_Enable(uint8_t interruptFlags);
extern void I2C_Interrupt_Disable(uint8_t interruptFlags);
extern void I2C_Interrupt_Clear(uint8_t interruptFlags);
extern unsigned char I2C_Interrupt_Status(uint8_t mask);
extern void I2C_Write_Packet_To_Sensor(uint8_t *writeData,
               uint8_t dataLength);

extern void I2C_Read_Packet_From_Sensor(uint8_t *readData,
               uint8_t dataLength);

#endif /*HAL_I2C_H*/
