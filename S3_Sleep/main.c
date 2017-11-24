/*Copyright (c) 2015, Shimmer Research, Ltd.
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *    * Neither the name of Shimmer Research, Ltd. nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *    * You may not use or distribute this Software or any derivative works
 *      in any form for commercial purposes with the exception of commercial
 *      purposes when used in conjunction with Shimmer products purchased
 *      from Shimmer or their designated agent or with permission from
 *      Shimmer.
 *      Examples of commercial purposes would be running business
 *      operations, licensing, leasing, or selling the Software, or
 *      distributing the Software for use with commercial products.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @author Prakash Rajiah
 * @date November, 2017
 *
 *  ******* Shimmer3 - SLeep Firmware ****
 *
 *
 */
#include "msp430.h"
#include "shimmer3_common_source/shimmer_sd_include.h"
#include "shimmer_btsd.h"
void Init(void);
uint8_t daughtCardId[16], eepromIsPresent;
uint8_t * slave_address_pointer;
uint8_t slave_addresses[128];
void main(void)
{
    Init();

    UCSCTL4 = SELA_1;

    PMMCTL0_H = PMMPW_H;

    /*
     * Supply Voltage Supervisor (SVS) and Monitor (SVM)
     *
     * About:
     * The high-side supervisor and monitor (SVSHE & SVMHE) oversee DVcc
     * The low-side supervisor and monitor (SVSLE & SVMLE) oversee VCore
     *
     * Usage:
     * Some power savings can be achieved by disabling these modules. See Section 2.2.2 (pg. 101)
     * of the MSP430x5xx and MSP430x6xx Family User's Guide. These modules can be accessed
     * through the MSP430's SVSMHCTL and SVSMLCTL registers.
     *
     */
    SVSMHCTL &= ~(SVMHE | SVSHE);
    SVSMLCTL &= ~(SVMLE | SVSLE);

    while (1)
    {
        __bis_SR_register(LPM4_bits);   //ACLK remains active
    }
}

char i2cSlavePresent(char address)
{
    char isPresent = 0;
    //I2C pull-ups must be enabled (on rev4 boards) before initialising I2C bus
    P8OUT |= BIT4;          //enable I2C pull-ups by turning on SW_I2C
    P3OUT |= BIT3;
    __delay_cycles(48000);  //2ms

    //Source from SMCLK, which is running @ 24MHz. 4kHz desired BRCLK
    I2C_Master_Init(S_MCLK, 24000000, 400000);

    isPresent = TI_USCI_I2C_slave_present(address);

    P8OUT &= ~BIT4;         //disable I2C pull-ups by turning off SW_I2C
    __delay_cycles(120000); //5ms (assuming 24MHz MCLK) to ensure no writes pending
    P3OUT &= ~BIT3;

    return isPresent;
}

void i2cSlaveDiscover(void)
{
    char address;

    //I2C pull-ups must be enabled (on rev4 boards) before initialising I2C bus
    P8OUT |= BIT4;          //enable I2C pull-ups by turning on SW_I2C
    P3OUT |= BIT3;
    __delay_cycles(48000);  //2ms

    //Source from SMCLK, which is running @ 24MHz. 4kHz desired BRCLK
    I2C_Master_Init(S_MCLK, 24000000, 400000);

    slave_address_pointer = &slave_addresses[0];
    for (address = 1; address < 127; address++)
    {
        if (TI_USCI_I2C_slave_present(address))
        {
            *slave_address_pointer++ = address;
        }
    }
    *slave_address_pointer++ = 0xFF;
    *slave_address_pointer = 0xFE;

    P8OUT &= ~BIT4;         //disable I2C pull-ups by turning off SW_I2C
    __delay_cycles(120000); //5ms (assuming 24MHz MCLK) to ensure no writes pending
    P3OUT &= ~BIT3;
}

void Init(void)
{
    Board_init();
    P3OUT |= BIT3;
    P3DIR |= BIT3;         //set as input
    i2cSlaveDiscover();    // discovers devices in the i2c lines
    eepromIsPresent = (i2cSlavePresent(CAT24C16_ADDR)) ? TRUE : FALSE; //  check if EEPROM is present in shimmer3
    if (eepromIsPresent)
    {
        CAT24C16_init();                     // EEPROM initialization
        CAT24C16_read(0, 16, daughtCardId);  // Reads values from EEPROM

        if (daughtCardId[DAUGHT_CARD_ID] == 47 || 59 || 0xFF) // 59 added in v0.2.2  // OxFF added in V0.2.X
        {
            P6OUT &= ~(BIT6 + BIT7);
            P6REN |= BIT6 + BIT7;     //enable pull down resistor // ADC stops fluctuating
            P6DIR &= ~(BIT6 + BIT7);  //A6 and A7 as input
            P7REN |= BIT7;

            //Internal ADC expansion ports
            P6OUT |= BIT1;
            P6REN |= BIT1;   //enable pull down resistor
            P6DIR |= BIT1;   //A1 set as output /* to be considered for bridge amplifier+*/

            P7OUT &= ~(BIT4 + BIT5);
            P7DIR &= ~(BIT4 + BIT5); //A12, A13 and A14 set as input

            //Internal ADC14 expansion ports
            P7OUT |= BIT6;
            P7REN |= BIT6;   //enable pull down resistor
            P7DIR |= BIT6;   //A1 set as output

            /*
             * PIN -  GPIO_INTERNAL1 (EXG_DRDY)
             *
             * Pin Usage:
             * ADS1292R chip issue workaround for all ExG Shimmer's
             *
             * Reason for set:
             * Conflicting voltage on this pin when set as output and used with ADS1292R.
             *
             */
            P2OUT &= ~BIT0;
            P2REN &= BIT0;   //enable pull down resistor
            P2DIR &= ~BIT0;  //set as input - to be considered for bridge amplifier+

        }
        if (daughtCardId[DAUGHT_CARD_ID] == 31)
        {
            //do nothing
        }

        CAT24C16_powerOff();     // Power off EEPROM
    }
    else if (!eepromIsPresent)
    {
        //do nothing
    }
    P3DIR &= ~BIT3;           // set as input
    P5OUT &= ~(BIT2 + BIT3);  // XT1
    P5DIR |= BIT2 + BIT3;
    P7OUT &= ~(BIT0 + BIT1);  // XT2
    P7DIR |= BIT0 + BIT1;

    //Set Vcore to minimum
    SetVCore(PMMCOREV_0);

    //explicitly disable ADC and ADC reference
    ADC12CTL0 &= ~(ADC12ON + ADC12REFON);
}
