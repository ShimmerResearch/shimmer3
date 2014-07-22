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

#include <stdint.h>
#include "msp430.h"
#include "hal_I2C.h"


// Local Variables
// @brief tx_packet_length is how many bytes of data will be written
uint8_t tx_packet_length;     
// @brief *tx_packet_data will point to the data to be written
uint8_t *tx_packet_data;      

// @brief rx_packet_length is how many bytes of data will be read
uint16_t rx_packet_length;
// @brief *rx_packet_data will point to the data to be read
uint8_t *rx_packet_data;      

// @brief receive flag is used for the ISR to know the MSP430 will be reading from the CP
uint8_t receive_flag;      
// @brief transmit flag is used for the ISR to know the MSP430 will be writing to the CP
uint8_t transmit_flag;    

// @brief Used to keep track how many bytes have been received
uint16_t rx_byte_ctr;
// @brief Used to keep track how many bytes have been transmitted
uint8_t tx_byte_ctr;      


/**
* @brief <b>Function Name</b>:     : I2C_Master_Init                                                
* @brief  <b>Description</b>: Initializes the I2C Master Block. Upon succesful
* initialization of the I2C master block, this function will have set the bus
* speed for the master, and will have enabled the SPI Master block.
* @param Input Parameters:
* <BR> uint8_t <b>selectClockSource</b> selects Clock source SMCLK (0x00) 
* or ACLK (0x1)        
* <BR> uint32_t <b>clockSourceFrequency</b> is the frequency of the 
* selected clock source
* <BR> uint32_t <b>desiredI2CClock</b> is the desired clock rate for SPI 
* communication                                 
* @return Return Values: None                                              
**/
void I2C_Master_Init(uint8_t selectClockSource,
      uint32_t clockSourceFrequency, uint32_t desiredI2CClock) {
   // Disable the USCI Module
   I2C_Disable();                      
  
   // Clock Source
   if (S_MCLK == selectClockSource) {
      UCB0CTL1 |= UCSSEL_2; //Select SMCLK
   } else {
      // Select ACLK
      UCB0CTL1 |= UCSSEL_1;
   }
  
   // Set the Baud rate
   UCB0BR0 = (unsigned int) (clockSourceFrequency/desiredI2CClock);
   UCB0BR1 = (unsigned int) ( (clockSourceFrequency/desiredI2CClock) >> 8 );
    
   /*!
    * Configure as I2C master mode.
    * UCMST = Master mode
    * UCMODE_3 = I2C mode
    * UCSYNC = Synchronous mode
    */
   UCB0CTL0 |= UCMST + UCMODE_3 + UCSYNC;
  
   I2C_Enable();
}


/**
* @brief <b>Function Name</b>:     : I2C_Set_Slave_Address                                               
* @brief  <b>Description</b>: This function will set the address that the I2C 
* Master will place on the bus when initiating a transaction.
* @param Input Parameters:
* <BR> uint8_t <b>slaveAddress</b> is the address of the slave                                                   
* @return Return Values: None                                                    
**/ 
void I2C_Set_Slave_Address(uint8_t slaveAddress) {
   // Set the address of the slave with which the master will communicate.
   UCB0I2CSA = slaveAddress;
}


/**
* @brief <b>Function Name</b>:     : I2C_Enable                                                 
* @brief  <b>Description</b>: Enable the I2C block
* @param Input Parameters: None                                                
* @return Return Values: None                                                    
**/ 
void I2C_Enable(void) {
   // Reset the UCSWRST bit to enable the USCI Module
   UCB0CTL1 &= ~(UCSWRST);
}


/**
* @brief <b>Function Name</b>:     : I2C_Disable                                                
* @brief  <b>Description</b>: Disable the I2C block
* @param Input Parameters: None                                                
* @return Return Values: None                                                      
**/ 
void I2C_Disable(void) {
   // Set the UCSWRST bit to disable the USCI Module
   UCB0CTL1 |= (UCSWRST);
}

void I2C_PowerOff(void){
	P8OUT &= ~BIT4;			//set SW_I2C low to power off I2C chips
}

/**
* @brief <b>Function Name</b>:     : I2C_Set_Mode
* @brief  <b>Description</b>: When the receive parameter is set to 0x01, the
* address will indicate that the I2C module is in receive mode; otherwise, the
* I2C module is in transmit mode
* @param Input Parameters:
* <BR> uint16_t <b>receive</b> can be set to a 0x01 to be in receive mode,
*  or 0x00 to be in transmit mode
* @return Return Values: None
**/
void I2C_Set_Mode(uint16_t receive) {
   // Disable the USCI module
   UCB0CTL1 |= UCSWRST;

   if (receive) {
      // Configure in receive mode
      UCB0CTL1 &= ~(UCTR);
   } else {
      // Configure in transmit mode
      UCB0CTL1 |= UCTR;
   }
}


/**
* @brief <b>Function Name</b>:     : I2C_Bus_Busy                                                
* @brief  <b>Description</b>: This function returns an indication of whether or 
* not the I2C bus is busy. This function can be used in a multi-master 
* enviroment to determine if another master is currently using the bus. This 
* function checks the status of the bus via UCBBUSY bit in the UCB0STAT register.
* @param Input Parameters: None                                          
* @return Return Values:
* <BR>Returns 0x01 if the I2C master is busy; otherwise, returns 0x00.                                                    
**/ 
uint16_t I2C_Bus_Busy(void) {
   // Return the bus busy status.
   if(UCB0STAT & UCBBUSY) {
      return(RET_OK);
   } else {
      return(RET_ERR);
   }
}


/**
* @brief <b>Function Name</b>:     : I2C_Busy                                                
* @brief  <b>Description</b>: This function returns an indication of whether or 
* not the I2C module is busy transmitting or receiving data. This function 
* checks if the transmit or receive flag is set.
* @param Input Parameters: None                                             
* @return Return Values:
* <BR>Returns 0x01 if the I2C master is busy; otherwise, returns 0x00.                                                    
**/ 
uint16_t I2C_Busy(void) {
   // Return the busy status.
   if( (UCB0IFG & UCB0TXIFG) || (UCB0IFG & UCB0RXIFG)) {
      return((uint16_t)RET_OK);
   } else {
      return((uint16_t)RET_ERR);
   }
}


/**
* @brief <b>Function Name</b>:     : I2C_Interrupt_Enable
* @brief  <b>Description</b>: Enables Individual I2C interrupts
* @param Input Parameters:
* <BR> uint8_t <b>interruptFlags</b> is the logical OR of any of the
* following:
*  <BR> I2C_INT_STOP - STOP condition interrupt
*  <BR> I2C_INT_START - START condition interrupt
*  <BR> I2C_INT_DATA_RX -Receive interrupt
*  <BR> I2C_INT_DATA_TX - Transmit interrupt
*  <BR> I2C_INT_NACK - Not-acknowledge interrupt
*  <BR> I2C_INT_CALIFG - Arbitration lost interrupt
* @return Return Values: None
**/
void I2C_Interrupt_Enable(uint8_t interruptFlags) {
   // Enable the interrupt masked bit
   UCB0IE |= interruptFlags;
}


/**
* @brief <b>Function Name</b>:     : I2C_Interrupt_Disable
* @brief  <b>Description</b>: Disables Individual I2C interrupts
* @param Input Parameters:
* <BR> uint8_t <b>interruptFlags</b> is the logical OR of any of the
* following:
*  <BR> I2C_INT_STOP - STOP condition interrupt
*  <BR> I2C_INT_START - START condition interrupt
*  <BR> I2C_INT_DATA_RX -Receive interrupt
*  <BR> I2C_INT_DATA_TX - Transmit interrupt
*  <BR> I2C_INT_NACK - Not-acknowledge interrupt
*  <BR> I2C_INT_CALIFG - Arbitration lost interrupt
* @return Return Values: None
**/
void I2C_Interrupt_Disable(uint8_t interruptFlags) {
   // Disable the interrupt masked bit
   UCB0IE &= ~(interruptFlags);
}


/**
* @brief <b>Function Name</b>:     : I2C_Interrupt_Clear
* @brief  <b>Description</b>: Clears Individual I2C interrupts flags
* @param Input Parameters:
* <BR> uint8_t <b>interruptFlags</b> is the logical OR of any of the
* following:
*  <BR> I2C_INT_STOP - STOP condition interrupt
*  <BR> I2C_INT_START - START condition interrupt
*  <BR> I2C_INT_DATA_RX -Receive interrupt
*  <BR> I2C_INT_DATA_TX - Transmit interrupt
*  <BR> I2C_INT_NACK - Not-acknowledge interrupt
*  <BR> I2C_INT_CALIFG - Arbitration lost interrupt
* @return Return Values: None
**/
void I2C_Interrupt_Clear(uint8_t interruptFlags) {
   // Clear the I2C interrupt source.
   UCB0IFG &= ~(interruptFlags);
}


/**
* @brief <b>Function Name</b>:     : I2C_Interrupt_Status
* @brief  <b>Description</b>: Returns the interrupt status for the I2C module
* based on which flag is passed.
* @param Input Parameters:
* <BR> uint8_t <b>mask</b> is the masked interrupt flag status to be
* returned.
*  <BR> I2C_INT_STOP - STOP condition interrupt
*  <BR> I2C_INT_START - START condition interrupt
*  <BR> I2C_INT_DATA_RX -Receive interrupt
*  <BR> I2C_INT_DATA_TX - Transmit interrupt
*  <BR> I2C_INT_NACK - Not-acknowledge interrupt
*  <BR> I2C_INT_CALIFG - Arbitration lost interrupt
* @return Return Values:
* <BR>The current interrupt status, returned as 0x01 if active, otherwise 0x00.
**/
uint8_t I2C_Interrupt_Status(uint8_t mask) {
   // Return the interrupt status of the request masked bit.
   return (UCB0IFG & mask);
}


/**
* @brief <b>Function Name</b>:     : I2C_Write_Packet_To_Sensor                                            
* @brief  <b>Description</b>: Sends data packet to sensor
* @param Input Parameters:
* <BR> uint8_t <b>writeData</b> is the pointer to the array which holds the
* the packet to send to the sensor. writeData[0] = register the packet will
* be written to. This function requires the user to have used I2C_Set_Slave_Address(),
* to set the address of the sensor. 
* <BR> uint8_t <b>dataLength</b> is the number of data bytes to be written.
* @return Return Values: None               
**/ 
void I2C_Write_Packet_To_Sensor(uint8_t *writeData, uint8_t dataLength) {
   // Assign values to local variables
   tx_packet_length = dataLength;
   tx_packet_data = writeData;
  
   // Reset variables for transmission
   transmit_flag = 1;
   receive_flag = 0;
   tx_byte_ctr = 0;
   rx_byte_ctr = 0;
    
   // Enable TX Interrupt to send write address, register and data
   __bic_SR_register(GIE);
   I2C_Interrupt_Enable(UCB0TXIE);
   I2C_Interrupt_Enable(UCNACKIE);
   UCB0IFG |= UCB0TXIFG;
  
   // Enter Low Power Mode 0
   while(transmit_flag == 1)
      __bis_SR_register(LPM0_bits + GIE);       // CPU off, enable interrupts
}


/**
* @brief <b>Function Name</b>:     : I2C_Read_Packet_From_Sensor                                            
* @brief  <b>Description</b>: Sends data packet to Authentication CP 
* @param Input Parameters:
* <BR> uint8_t <b>readData</b> is the pointer to the array which holds the
* the packet to send to the sensor. readData[0] = register the packet will
* be read from. This function requires the user to have used I2C_Set_Slave_Address(),
* to set the address of the sensor. 
* <BR> uint8_t * <b>readData</b> is the number of data bytes to be read.                                       
* @return Return Values: None                 
**/ 
void I2C_Read_Packet_From_Sensor(uint8_t *readData,
      uint16_t dataLength) {
   // Assign values to local variables
   rx_packet_length = dataLength;
   rx_packet_data = readData;
  
   // Reset variables for transmission
   transmit_flag = 0;
   receive_flag = 1;
   tx_byte_ctr = 0;
   rx_byte_ctr = 0;
  
   // Enable TX Interrupt to send write address, and register
   __bic_SR_register(GIE);
  
   I2C_Interrupt_Enable(UCB0TXIE);
   I2C_Interrupt_Enable(UCNACKIE);
   UCB0IFG |= UCB0TXIFG;
  
   // Enter Low Power Mode 0
   while(receive_flag == 1)
      __bis_SR_register(LPM0_bits + GIE);       // CPU off, enable interrupts
}


#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_I2C_ISR(void) {
   switch(__even_in_range(UCB0IV,12)) {
   case  0: break;                              // Vector  0: No interrupts
   case  2: break;                              // Vector  2: ALIFG
   case  4:                                     // Vector  4: NACKIFG
      UCB0CTL1 |= UCTXSTT;                      // Re-start condition
      tx_byte_ctr = 0;
      break;
   case  6: break;                              // Vector  6: STTIFG
   case  8: break;                              // Vector  8: STPIFG
   case 10:                                     // Vector 10: RXIFG
      if(rx_byte_ctr < rx_packet_length-1) {
         *rx_packet_data++= UCB0RXBUF;          // Read receive buffer
         rx_byte_ctr++;
         if(rx_byte_ctr == rx_packet_length-1) {
            UCB0CTL1 |= UCTXSTP;                // Send I2C stop condition
         }
      } else {
         // Clear RX Flag
         receive_flag = 0;
         *rx_packet_data= UCB0RXBUF;
         I2C_Interrupt_Disable(UCNACKIE);
         I2C_Interrupt_Disable(UCB0RXIE);
         __bic_SR_register_on_exit(LPM0_bits);  // Exit active CPU
      }
      break;                           
   case 12:                                     // Vector 12: TXIFG  
      if(transmit_flag) {
         if(tx_byte_ctr == 0) {
            while (UCB0CTL1 & UCTXSTP);         // Ensure stop condition got sent
            UCB0CTL1 |= UCTR+UCTXSTT;           // Send I2C start condition
            tx_byte_ctr++;
         } else if(tx_packet_length >= tx_byte_ctr) {
            UCB0TXBUF = *tx_packet_data++;      // Send data bytes
            tx_byte_ctr++;
         } else {
            UCB0CTL1 |= UCTXSTP;                // Send I2C stop condition
            I2C_Interrupt_Disable(UCB0TXIE);
            I2C_Interrupt_Disable(UCNACKIE);
            transmit_flag = 0;
            __bic_SR_register_on_exit(LPM0_bits);  // Exit active CPU
         }
      } else if(receive_flag) {
         if(tx_byte_ctr == 0) {
            while (UCB0CTL1 & UCTXSTP);         // Ensure stop condition got sent
            UCB0CTL1 |= UCTR+UCTXSTT;           // I2C start condition
            tx_byte_ctr++;
         } else if(tx_byte_ctr == 1) {
            UCB0TXBUF = rx_packet_data[0];      // Send Register name
            tx_byte_ctr++;          
         } else if(tx_byte_ctr == 2) {
            UCB0CTL1 &= ~(UCTR);                // Enable receive bit
            UCB0CTL1 |= UCTXSTT;                // Send I2C start condition
            // In the case of reading one byte, poll on start condition bit
            if(rx_packet_length == 1) {
               // Wait for start condition to be transmitted
               while( UCB0CTL1 & UCTXSTT );
               UCB0CTL1 |= UCTXSTP;             // Send I2C stop condition
            }
            I2C_Interrupt_Enable(UCB0RXIE);     // Enable RX interrupt
            I2C_Interrupt_Disable(UCB0TXIE);    // Disable TX interrupt
            tx_byte_ctr++;                             
         }        
      }
      break;                          
   default: break;
   }
}
