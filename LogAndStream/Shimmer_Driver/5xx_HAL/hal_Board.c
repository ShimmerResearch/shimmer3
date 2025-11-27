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

#include "hal_Board.h"
#include "msp430.h"

#include <stdint.h>

#include "Boards/shimmer_boards.h"
#include "GSR/gsr.h"
#include "LEDs/shimmer_leds.h"
#include "SDCard/shimmer_sd.h"
#include "SDCard/shimmer_sd_data_file.h"
#include "log_and_stream_externs.h"
#include "shimmer_definitions.h"

#define XT1_PORT_DIR P7DIR
#define XT1_PORT_OUT P7OUT
#define XT1_PORT_SEL P7SEL
#define XT2_PORT_DIR P5DIR
#define XT2_PORT_OUT P5OUT
#define XT2_PORT_SEL P5SEL
#define XT1_ENABLE   (BIT0 + BIT1)
#define XT2_ENABLE   (BIT2 + BIT3)

static uint16_t lastResetReason;

/******************************************************************************
 * @brief  Initialize the board - configure ports
 * @param  None
 * @return none
 ******************************************************************************/
void Board_init(void)
{
  Board_saveLastResetReason();

  //Setup XT1 and XT2
  XT1_PORT_SEL |= XT1_ENABLE;
  XT2_PORT_SEL |= XT2_ENABLE;

  //Configure LED ports
  P1OUT &= ~(BIT1 + BIT2);
  P1DIR |= BIT1 + BIT2;
  P7OUT &= ~(BIT2 + BIT3);
  P7DIR |= BIT2 + BIT3;
  P8OUT &= ~BIT0;
  P8DIR |= BIT0;

  //Configure button ports
  P1DIR &= ~BIT6; //Button SW1 on P1.6 is input

  //Radio regulator control SW_RADIO
  P8OUT &= ~BIT3; //set low
  P8DIR |= BIT3;  //set as output
  //RADIO_CS_N
  P8OUT &= ~BIT1; //set low
  P8DIR |= BIT1;  //set as output
  //RADIO_RESET_N
  P8OUT &= ~BIT2; //set low
  P8DIR |= BIT2;  //set as output
  //RADIO_SCLK_R
  P3OUT &= ~BIT6; //set low
  P3DIR |= BIT6;  //set as output

  //Make the 2 CHG_STAT pins input
  P2DIR &= ~(BIT6 + BIT7);

  //Battery voltage ADC pin as input
  P6DIR &= ~BIT2;

  //RN42 & RN4678 ports
  //BT power
  P4OUT &= ~BIT3; //set low
  P4DIR |= BIT3;  //set as output
  //connect indication as input
  P1DIR &= ~BIT0; //input
  //UART_RTS as input
  P1DIR &= ~BIT3; //input
  //UART_CTS
  P2OUT &= ~BIT2; //set low
  P2DIR |= BIT2;  //output
  //Reset
  P4OUT &= ~BIT4; //set low
  P4DIR |= BIT4;  //output
  //Factory Reset
  P4OUT &= ~BIT5; //set low
  P4DIR |= BIT5;  //output
  //UART RX and TX
  P5SEL |= BIT6 + BIT7; //P5.6,P5.7 = USCI_A1 TXD/RXD

  //I2C ports for LSM303DLHC, MPU9150 and BMP180 (USCI_B0)
  P3OUT |= BIT1 + BIT2;
  P3DIR |= BIT1 + BIT2;
  P3SEL |= BIT1 + BIT2;

  //MAG_INT1 and MAG_DRDY as inputs
  P2DIR &= ~(BIT4 + BIT5);

  //GYRO_DRDY as input
  P1DIR &= ~BIT7;

  //Analog Accel
  P8OUT &= ~BIT6;
  P8REN |= BIT6;  //enable pull down resistor
  P8DIR &= ~BIT6; //SW_ACCEL set as input

  P6DIR &= ~(BIT3 + BIT4 + BIT5); //ACCEL_X/Y/Z input
  P6SEL |= BIT3 + BIT4 + BIT5;

  //FLASH_CS_N (same defaults as on Shimmer2r)
  P4OUT &= ~BIT0; //set low
  P4DIR |= BIT0;  //set as output
  //FLASH_SIMO
  P3OUT &= ~BIT7; //set low
  P3DIR |= BIT7;  //set as output
  //FLASH_SOMI
  P5DIR &= ~BIT4; //set as input
  //FLASH_SCLK_R
  P5OUT &= ~BIT5; //set low
  P5DIR |= BIT5;  //set as output
  //Flash power (SW_FLASH)
  Board_setSdPower(0);
  P4DIR |= BIT2; //set as output
  //SD card detect (SD_DETECT_N)
  P4DIR &= ~BIT1; //set as input
  P4REN |= BIT1;  //enable pull up
  P4OUT |= BIT1;  //pull up when no sd card
  //DOCK
  P2DIR &= ~BIT3; //set as input
  //DETECT_N
  Board_dockDetectN(1); //DETECT_N set to high
  P6REN |= BIT0;        //enable pull up
  P6DIR |= BIT0;        //set as output

  //EXP_RESET_N
  Board_setExpansionBrdPower(0);
  P3DIR |= BIT3; //set as output
  P3SEL &= ~BIT3;

  //External ADC expansion ports
  P6DIR &= ~(BIT6 + BIT7); //A6 and A7 as input
  P6SEL |= BIT6 + BIT7;
  P7DIR &= ~BIT7; //A15 as input
  P7SEL |= BIT7;
  //Internal ADC expansion ports
  P6DIR &= ~BIT1; //A1 set as input
  P6SEL |= BIT1;
  P7DIR &= ~(BIT4 + BIT5 + BIT6); //A12, A13 and A14 set as input
  P7SEL |= BIT4 + BIT5 + BIT6;

  //SA_SIMO_TXD
  P3OUT &= ~BIT4; //set low
  P3DIR |= BIT4;  //set as output
  //SA_SOMI_RXD
  P3DIR &= ~BIT5; //set as input
  //SA_SCLK_R
  P3OUT &= ~BIT0; //set low
  P3DIR |= BIT0;  //set as output

  //GPIO_INTERNAL
  P1DIR &= ~BIT4;
  //GPIO_INTERNAL2
  P2DIR &= ~BIT1;
  //GPIO_EXTERNAL_RADIO_DD
  P1DIR &= ~BIT5;

  //VREFP (P5.0 connected to PV_VREF_MSP) and VREFN (P5.1 connected to GND)
  P5DIR &= ~(BIT0 + BIT1); //set as input

  //SW_I2C
  Board_setI2cPower(0);
  P8DIR |= BIT4; //set as output
}

/******************************************************************************
 * @brief  Turn on LEDs
 * @param  ledMask   Use values defined in HAL_board.h for the LEDs to turn on
 * @return none
 ******************************************************************************/
inline void Board_ledOn(uint8_t ledMask)
{
  if (ledMask & LED_LWR_RED)
  {
    P7OUT |= BIT2;
  }
  if (ledMask & LED_LWR_GREEN)
  {
    P7OUT |= BIT3;
  }
  if (ledMask & LED_UPR_GREEN)
  {
    P1OUT |= BIT1;
  }
  if (ledMask & LED_LWR_YELLOW)
  {
    P8OUT |= BIT0;
  }
  if (ledMask & LED_UPR_BLUE)
  {
    P1OUT |= BIT2;
  }
}

/******************************************************************************
 * @brief  Turn off LEDs
 * @param  ledMask   Use values defined in HAL_board.h for the LEDs to turn off
 * @return none
 ******************************************************************************/
inline void Board_ledOff(uint8_t ledMask)
{
  if (ledMask & LED_LWR_RED)
  {
    P7OUT &= ~BIT2;
  }
  if (ledMask & LED_LWR_GREEN)
  {
    P7OUT &= ~BIT3;
  }
  if (ledMask & LED_UPR_GREEN)
  {
    P1OUT &= ~BIT1;
  }
  if (ledMask & LED_LWR_YELLOW)
  {
    P8OUT &= ~BIT0;
  }
  if (ledMask & LED_UPR_BLUE)
  {
    P1OUT &= ~BIT2;
  }
}

/******************************************************************************
 * @brief  Toggle LEDs
 * @param  ledMask   Use values defined in HAL_board.h for the LEDs to toggle
 * @return none
 ******************************************************************************/
inline void Board_ledToggle(uint8_t ledMask)
{
  if (ledMask & LED_LWR_RED)
  {
    P7OUT ^= BIT2;
  }
  if (ledMask & LED_LWR_GREEN)
  {
    P7OUT ^= BIT3;
  }
  if (ledMask & LED_UPR_GREEN)
  {
    P1OUT ^= BIT1;
  }
  if (ledMask & LED_LWR_YELLOW)
  {
    P8OUT ^= BIT0;
  }
  if (ledMask & LED_UPR_BLUE)
  {
    P1OUT ^= BIT2;
  }
}

//Overrides weak function in LogAndStream driver
void platform_initGpioForRevision(void)
{
  shimmer_expansion_brd *expBrd = ShimBrd_getDaughtCardId();

  if (ShimBrd_isAds1292Present())
  {
    P1DIR &= ~BIT4; //RESP_DRDY as input
    P2DIR &= ~BIT0; //EXG_DRDY as input

    P7DIR &= ~BIT6; //SA_CS_RESP as input
    P6DIR &= ~BIT1; //SA_CS_ECG as input
  }
  else
  {
    //GPIO_INTERNAL1
    P2OUT &= ~BIT0; //set low
    P2DIR |= BIT0;  //set as output
  }

  //RN4678 Operational mode pins
  if (ShimBrd_isRn4678PresentAndCmdModeSupport())
  {
    //RN4678_OP_MODE_DISABLE
    P4OUT &= ~BIT6; //P2_0
    P4OUT &= ~BIT7; //P2_4
    P8OUT &= ~BIT5; //EAN

    P4DIR |= BIT6; //P2_0
    P4DIR |= BIT7; //P2_4
    P8DIR |= BIT5; //EAN
  }
  else
  {
    //TCXO power
    P4OUT &= ~BIT6; //set low
    P4DIR |= BIT6;  //set as output
    //TCXO_CLK_R
    P4DIR &= ~BIT7; //set as input

    //Not connected
    P8OUT &= ~BIT5; //set low
    P8DIR |= BIT5;  //set as output
  }

  if (expBrd->exp_brd_id == EXP_BRD_GSR || expBrd->exp_brd_id == EXP_BRD_GSR_UNIFIED)
  {
    setGsrRangePinsAreReversed(ShimBrd_areGsrControlsPinsReversed());

    GSR_setActiveResistor(HW_RES_40K);

    //A0
    P1SEL &= ~BIT4;
    P1DIR |= BIT4;

    //A1
    P2SEL &= ~BIT1;
    P2DIR |= BIT1;
  }
}

void Board_sdPowerCycle(void)
{
  _delay_cycles(2880000); //120ms
  Board_setSdPower(0);    //SW_FLASH set low

  P5SEL &= ~(BIT4 + BIT5);
  P5OUT &= ~(BIT4 + BIT5); //FLASH_SOMI and FLASH_SCLK set low
  P5DIR |= BIT4;           //FLASH_SOMI set as output
  P3SEL &= ~BIT7;
  P3OUT &= ~BIT7;          //FLASH_SIMO set low
  P4OUT &= ~BIT0;          //FLASH_CS_N set low
  P6OUT &= ~(BIT6 + BIT7); //ADC6_FLASHDAT2 and ADC7_FLASHDAT1 set low
  P6DIR |= BIT6 + BIT7;    //ADC6_FLASHDAT2 and ADC7_FLASHDAT1 set as output

  //60ms as taken from TinyOS driver (SDP.nc powerCycle() function)
  _delay_cycles(2880000); //120ms

  P5DIR &= ~(BIT4 + BIT5); //FLASH_SOMI and FLASH_SCLK set as input
  P3DIR &= ~BIT7;          //FLASH_SIMO set as input
  P4DIR &= ~BIT0;          //FLASH_CS_N set as input
  P6DIR &= ~(BIT6 + BIT7); //ADC6_FLASHDAT2 and ADC7_FLASHDAT1 set as input

  Board_setSdPower(1); //SW_FLASH set high
}

void Board_sd2Pc(void)
{
  /* Ensure the dock sees "no card" during the entire handover */
  Board_dockDetectN(DOCK_CARD_NOT_PRESENT);

  /* Cleanly release SD from MCU side before power/route changes */
  //Unmount SD card
  ShimSd_mount(SD_UNMOUNT);

  /* Power cycle the SD and hand bus control to the dock/PC side */
  Board_sdPowerCycle();

  //give SD card time to power back up
  _delay_cycles(SD_PC_STABILIZE_MS * MSP430_MCU_CYCLES_PER_MS);

  /* Now tell the dock/PC that a card is present */
  Board_dockDetectN(DOCK_CARD_PRESENT);
}

void Board_sd2Mcu(void)
{
  /* Setup pin to indicate SD not ready for dock access */
  Board_dockDetectN(DOCK_CARD_NOT_PRESENT);

  /* Clears any lingering FatFs work area, BPB/cache, and “mounted” flags if a prior path failed to unmount. */
  ShimSd_mount(SD_UNMOUNT);

  /* Power cycle the SD card with access to MCU */
  Board_sdPowerCycle();

  /* Allow time for SD card to stabilize */
  _delay_cycles(SD_MCU_STABILIZE_MS * MSP430_MCU_CYCLES_PER_MS);

  /* Mount SD card */
  ShimSd_mount(SD_MOUNT);
}

uint8_t Board_checkDockedDetectState(void)
{
  shimmerStatus.docked = BOARD_IS_DOCKED;
  if (shimmerStatus.docked)
  {
    Board_setUndockDetectIntDir();
  }
  else
  {
    Board_setDockDetectIntDir();
  }
  return shimmerStatus.docked;
}

void Board_setDockDetectIntDir(void)
{
  P2IES &= ~BIT3; //look for rising edge
}

void Board_setUndockDetectIntDir(void)
{
  P2IES |= BIT3; //look for falling edge
}

void Board_setSdPower(uint8_t state)
{
  if (state)
  {
    P4OUT |= BIT2; //SD power on
  }
  else
  {
    P4OUT &= ~BIT2; //SD power off
  }
  shimmerStatus.sdPowerOn = state;
}

void Board_setExpansionBrdPower(uint8_t state)
{
  if (state)
  {
    P3OUT |= BIT3;
  }
  else
  {
    P3OUT &= ~BIT3;
  }
  shimmerStatus.pinPvExt = state;
}

void Board_setI2cPower(uint8_t state)
{
  //Enable/disable I2C pull-ups by turning on/off SW_I2C
  if (state)
  {
    P8OUT |= BIT4;
  }
  else
  {
    P8OUT &= ~BIT4;
  }
  shimmerStatus.pinPvI2c = state;
}

uint8_t Board_isLedOnUprBlue(void)
{
  return (P1OUT & BIT2);
}

uint8_t Board_isLedOnUprGreen(void)
{
  return (P1OUT & BIT1);
}

uint8_t Board_isBtnPressed(void)
{
  return BOARD_IS_BTN_PRESSED;
}

uint8_t Board_isSdInserted(void)
{
  return BOARD_IS_SD_INSERTED;
}

uint8_t Board_isDocked(void)
{
  return BOARD_IS_DOCKED;
}

/* Informs dock that an SD card has been connected */
void Board_dockDetectN(uint8_t state)
{
  if (state)
  {
    P6OUT |= BIT0; //DETECT_N set high
  }
  else
  {
    P6OUT &= ~BIT0; //DETECT_N set low
  }
}

void Board_saveLastResetReason(void)
{
  lastResetReason = SYSRSTIV;
}

uint16_t Board_getLastResetReason(void)
{
  return lastResetReason;
}
