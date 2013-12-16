/*
 * Adapted from Texas Instruments supplied example code
 */
/*******************************************************************************
 *
 *  Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
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

#include "msp430.h"
#include "HAL_Board.h"

#define XT1_PORT_DIR             P7DIR
#define XT1_PORT_OUT             P7OUT
#define XT1_PORT_SEL             P7SEL
#define XT2_PORT_DIR             P5DIR
#define XT2_PORT_OUT             P5OUT
#define XT2_PORT_SEL             P5SEL
#define XT1_ENABLE               (BIT0 + BIT1)
#define XT2_ENABLE               (BIT2 + BIT3)


/***************************************************************************//**
 * @brief  Initialize the board - configure ports
 * @param  None
 * @return none
 ******************************************************************************/
void Board_init(void) {
   // Setup XT1 and XT2
   XT1_PORT_SEL |= XT1_ENABLE;
   XT2_PORT_SEL |= XT2_ENABLE;

   // Configure LED ports
   P1OUT &= ~(BIT1 + BIT2);
   P1DIR |= BIT1 + BIT2;
   P7OUT &= ~(BIT2 + BIT3);
   P7DIR |= BIT2 + BIT3;
   P8OUT &= ~BIT0;
   P8DIR |= BIT0;


   // Configure button ports
   P1DIR &= ~BIT6;            //Button SW1 on P1.6 is input

   //Radio regulator control SW_RADIO
   P8OUT &= ~BIT3;            //set low
   P8DIR |= BIT3;             //set as output
   //RADIO_CS_N
   P8OUT &= ~BIT1;            //set low
   P8DIR |= BIT1;             //set as output
   //RADIO_RESET_N
   P8OUT &= ~BIT2;            //set low
   P8DIR |= BIT2;             //set as output
   //RADIO_SCLK_R
   P3OUT &= ~BIT6;            //set low
   P3DIR |= BIT6;             //set as output

   //Make the 2 CHG_STAT pins input
   P2DIR &= ~(BIT6 + BIT7);

   //Battery voltage ADC pin as input
   P6DIR &= ~BIT2;

   //RN42 ports
   //BT power
   P4OUT &= ~BIT3;            //set low
   P4DIR |= BIT3;             //set as output
   //UART_RTS and connect indication as input
   P1DIR &= ~(BIT0 + BIT3);   //input
   //UART_CTS
   P2OUT &= ~BIT2;            //set low
   P2DIR |= BIT2;             //output
   //Reset
   P4OUT &= ~BIT4;            //set low
   P4DIR |= BIT4;             //output
   //Factory Reset
   P4OUT &= ~BIT5;            //set low
   P4DIR |= BIT5;             //output
   //UART RX and TX
   P5SEL |= BIT6+BIT7;        //P5.6,P5.7 = USCI_A1 TXD/RXD

   //I2C ports for LSM303DLHC, MPU9150 and BMP180 (USCI_B0)
   P3OUT |= BIT1+BIT2;
   P3DIR |= BIT1+BIT2;
   P3SEL |= BIT1+BIT2;

   //MAG_INT1 and MAG_DRDY as inputs
   P2DIR &= ~(BIT4 + BIT5);

   //GYRO_DRDY as input
   P1DIR &= ~BIT7;

   //Analog Accel
   P8OUT &= ~BIT6;
   P8REN |= BIT6;             //enable pull down resistor
   P8DIR &= ~BIT6;            //SW_ACCEL set as input


   P6DIR &= ~(BIT3 + BIT4 + BIT5); //ACCEL_X/Y/Z input
   P6SEL |= BIT3 + BIT4 + BIT5;

   //FLASH_CS_N (same defaults as on Shimmer2r)
   P4OUT &= ~BIT0;            //set low
   P4DIR |= BIT0;             //set as output
   //FLASH_SIMO
   P3OUT &= ~BIT7;            //set low
   P3DIR |= BIT7;             //set as output
   //FLASH_SOMI
   P5DIR &= ~BIT4;            //set as input
   //FLASH_SCLK_R
   P5OUT &= ~BIT5;            //set low
   P5DIR |= BIT5;             //set as output
   //Flash power (SW_FLASH)
   P4OUT &= ~BIT2;            //set low
   P4DIR |= BIT2;             //set as output
   //SD card detect (SD_DETECT_N)
   P4DIR &= ~BIT1;            //set as input
   //DOCK
   P2DIR &= ~BIT3;            //set as input
   //DETECT_N
   P6OUT |= BIT0;             //set high
   P6DIR |= BIT0;             //set as output

   //EXP_RESET_N
   P3DIR &= ~BIT3;            //set as input

   //TCXO power
   P4OUT &= ~BIT6;            //set low
   P4DIR |= BIT6;             //set as output
   //TCXO_CLK_R
   P4DIR &= ~BIT7;            //set as input

   //External ADC expansion ports
   P6DIR &= ~(BIT6 + BIT7);   //A6 and A7 as input
   P6SEL |= BIT6 + BIT7;
   P7DIR &= ~BIT7;            //A15 as input
   P7SEL |= BIT7;
   //Internal ADC expansion ports
   P6DIR &= ~BIT1;            //A1 set as input
   P6SEL |= BIT1;
   P7DIR &= ~(BIT4 + BIT5 + BIT6); //A12, A13 and A14 set as input
   P7SEL |= BIT4 + BIT5 + BIT6;

   //SA_SIMO_TXD
   P3OUT &= ~BIT4;            //set low
   P3DIR |= BIT4;             //set as output
   //SA_SOMI_RXD
   P3DIR &= ~BIT5;            //set as input
   //SA_SCLK_R
   P3OUT &= ~BIT0;            //set low
   P3DIR |= BIT0;             //set as output

   //GPIO_INTERNAL1
   P2DIR &= ~BIT0;
   //GPIO_INTERNAL
   P1DIR &= ~BIT4;
   //GPIO_INTERNAL2
   P2DIR &= ~BIT1;
   //GPIO_EXTERNAL_RADIO_DD
   P1DIR &= ~BIT5;

   //VREFP (P5.0 connected to PV_VREF_MSP) and VREFN (P5.1 connected to GND)
   P5DIR &= ~(BIT0 + BIT1);  //set as input

   //SW_I2C
   P8OUT &= ~BIT4;           //set low
   P8DIR |= BIT4;            //set as output

   //Not connected
   P8OUT &= ~BIT5;           //set low
   P8DIR |= BIT5;            //set as output
}


/***************************************************************************//**
 * @brief  Turn on LEDs
 * @param  ledMask   Use values defined in HAL_board.h for the LEDs to turn on
 * @return none
 ******************************************************************************/
void Board_ledOn(uint8_t ledMask) {
   if (ledMask & LED_RED)        P7OUT |= BIT2;
   if (ledMask & LED_GREEN0)     P7OUT |= BIT3;
   if (ledMask & LED_GREEN1)     P1OUT |= BIT1;
   if (ledMask & LED_YELLOW0)    P8OUT |= BIT0;
   if (ledMask & LED_BLUE)       P1OUT |= BIT2;
}


/***************************************************************************//**
 * @brief  Turn off LEDs
 * @param  ledMask   Use values defined in HAL_board.h for the LEDs to turn off
 * @return none
 ******************************************************************************/
void Board_ledOff(uint8_t ledMask) {
   if (ledMask & LED_RED)        P7OUT &= ~BIT2;
   if (ledMask & LED_GREEN0)     P7OUT &= ~BIT3;
   if (ledMask & LED_GREEN1)     P1OUT &= ~BIT1;
   if (ledMask & LED_YELLOW0)    P8OUT &= ~BIT0;
   if (ledMask & LED_BLUE)       P1OUT &= ~BIT2;
}


/***************************************************************************//**
 * @brief  Toggle LEDs
 * @param  ledMask   Use values defined in HAL_board.h for the LEDs to toggle
 * @return none
 ******************************************************************************/
void Board_ledToggle(uint8_t ledMask) {
   if (ledMask & LED_RED)        P7OUT ^= BIT2;
   if (ledMask & LED_GREEN0)     P7OUT ^= BIT3;
   if (ledMask & LED_GREEN1)     P1OUT ^= BIT1;
   if (ledMask & LED_YELLOW0)    P8OUT ^= BIT0;
   if (ledMask & LED_BLUE)       P1OUT ^= BIT2;
}

/***************************************************************************//**
 * @}
 ******************************************************************************/
