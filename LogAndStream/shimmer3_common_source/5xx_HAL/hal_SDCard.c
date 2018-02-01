/*******************************************************************************
 *
 *  HAL_SDCard.c - Driver for the SD Card slot
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

/***************************************************************************//**
 * @file       HAL_SDCard.c
 * @addtogroup HAL_SDCard
 * @{
 ******************************************************************************/
#include "msp430.h"
#include "hal_SDCard.h"

// Pins from MSP430 connected to the SD Card
#define SPI_SIMO           BIT7
#define SPI_SOMI           BIT4
#define SPI_CLK            BIT5
#define SD_CS              BIT0

// Ports
#define SPI_SIMO_SEL       P3SEL
#define SPI_SIMO_DIR       P3DIR
#define SPI_SOMI_CLK_SEL   P5SEL
#define SPI_SOMI_CLK_DIR   P5DIR
#define SPI_SOMI_CLK_OUT   P5OUT
#define SPI_SOMI_CLK_REN   P5REN
#define SD_CS_SEL          P4SEL
#define SD_CS_OUT          P4OUT
#define SD_CS_DIR          P4DIR

extern uint8_t sensing;

/***************************************************************************//**
 * @brief   Initialize SD Card
 * @param   None
 * @return  None
 ******************************************************************************/

void SDCard_init(void)
{
    // Port initialization for SD Card operation
   SPI_SIMO_SEL |= SPI_SIMO;
   SPI_SIMO_DIR |= SPI_SIMO;
   SPI_SOMI_CLK_SEL |= SPI_CLK + SPI_SOMI;
   SPI_SOMI_CLK_DIR |= SPI_CLK;
   SPI_SOMI_CLK_REN |= SPI_SOMI;            // Pull-Ups on SD Card SOMI
   SPI_SOMI_CLK_OUT |= SPI_SOMI;            // Certain SD Card Brands need pull-ups

   SD_CS_SEL &= ~SD_CS;
   SD_CS_OUT |= SD_CS;
   SD_CS_DIR |= SD_CS;

   // Initialize USCI_B1 for SPI Master operation
   UCB1CTL1 |= UCSWRST;                                   // Put state machine in reset
   UCB1CTL0 = UCCKPL + UCMSB + UCMST + UCMODE_0 + UCSYNC; // 3-pin, 8-bit SPI master
   // Clock polarity select - The inactive state is high
   // MSB first
   UCB1CTL1 = UCSWRST + UCSSEL_2;                         // Use SMCLK, keep RESET
   //UCB1BR0 = 63;                                          // Initial SPI clock must be <400kHz
   //UCB1BR1 = 0;                                           // f_UCxCLK = 25MHz/63 = 397kHz
   UCB1BR0 = 60;                                          // Initial SPI clock must be <400kHz
   UCB1BR1 = 0;                                           // f_UCxCLK = 24MHz/60 = 400kHz
   UCB1CTL1 &= ~UCSWRST;                                  // Release USCI state machine
   UCB1IFG &= ~UCRXIFG;
}

/***************************************************************************//**
 * @brief   Enable fast SD Card SPI transfers. This function is typically
 *          called after the initial SD Card setup is done to maximize
 *          transfer speed.
 * @param   None
 * @return  None
 ******************************************************************************/

void SDCard_fastMode(void)
{
   UCB1CTL1 |= UCSWRST;                                   // Put state machine in reset
   //UCB1BR0 = 2;                                         // f_UCxCLK = 25MHz/2 = 12.5MHz
   UCB1BR0 = 2;                                           // f_UCxCLK = 24MHz/2 = 12MHz
   UCB1BR1 = 0;
   UCB1CTL1 &= ~UCSWRST;                                  // Release USCI state machine
}

/***************************************************************************//**
 * @brief   Read a frame of bytes via SPI
 * @param   pBuffer Place to store the received bytes
 * @param   size Indicator of how many bytes to receive
 * @return  None
 ******************************************************************************/

void SDCard_readFrame(uint8_t *pBuffer, uint16_t size)
{
   //uint16_t gie = __get_SR_register() & GIE;              // Store current GIE state

   //__disable_interrupt();                                 // Make this operation atomic

   UCB1IFG &= ~UCRXIFG;                                   // Ensure RXIFG is clear

   // Clock the actual data transfer and receive the bytes
   while (size--){
      while (!(UCB1IFG & UCTXIFG)) ;                     // Wait while not ready for TX
      UCB1TXBUF = 0xff;                                  // Write dummy byte
      while (!(UCB1IFG & UCRXIFG)) ;                     // Wait for RX buffer (full)
      *pBuffer++ = UCB1RXBUF;
   }

   //__bis_SR_register(gie);                                // Restore original GIE state
}

void SDCard_readFrame_DMA(uint8_t *pBuffer, uint16_t size)
{
    /* Source DMA address: the data buffer. */
    __data16_write_addr((unsigned short) &DMA1SA, (unsigned long) &UCB1RXBUF);

    /* Destination DMA address: the UART send register. */
    __data16_write_addr((unsigned short) &DMA1DA, (unsigned long) pBuffer);

    /* The size of the block to be transferred */
    DMA1SZ = size;

    /*
     * The DMAxTSELbits should be modified only when the DMACTLx DMAEN bit is 0
     * When selecting the trigger,the trigger must not have already occurred,
     * or the transfer does not take place
     */
    DMACTL0 = 0;

    /* DMA trigger is either:
     * DMA_REQ - TSEL_0
     *  OR
     * USCIB1 receive - TSEL_22
     * */
    DMACTL0 = DMA1TSEL_0 | DMA1TSEL_22;

    /* Read-modify-write disable
     *
     * Brief:
     * The DMARMWDIS bit controls when the CPU is halted for DMA transfers.
     *
     * Detail:
     * When DMARMWDIS = 0, the CPU is halted immediately and the transfer begins
     * when a trigger is received. In this case, it is possible that CPU read-modify-write
     * operations can be interrupted by a DMA transfer. When DMARMWDIS = 1,the CPU finishes
     * the currently executing read-modify-write operation before the DMA controller halts
     * the CPU and the transfer begins.
     *
     */
    DMACTL4 = DMARMWDIS;

    /* Disable any pending interrupts */
    DMA1CTL &= ~DMAIFG;

    /* Configure the DMA transfer*/
    DMA1CTL =
    DMADT_0 | /* Single transfer mode - should toggle DMAEN from 1->0 after transfer of DMAxSZ bytes*/
    DMASBDB | /* Byte mode */
    DMAEN | /* Enable DMA */
    DMASRCBYTE | /* DMA source byte */
    DMADSTBYTE | /* DMA destination byte */
    DMASRCINCR_0 | /* Destination address unchanged */
    DMADSTINCR_3;  /* Increment the source address */
    //        DMAIE | /* Enable interrupt */

    /* Clear any pending flags */
    UCB1IFG &= ~(UCRXIFG | UCTXIFG);

    /* Kick off the transfer by either:
     * sending the first byte, the "start block" token
     * OR
     * forcing an interrupt where we need it
     * OR
     * triggering the DMAREQ bit */

    /* Send the first byte */
    //        UCB1TXBUF = pBuffer[0];
    /* ---- OR ----*/
    /* Force an interrupt to occur and get the ball rolling*/
    //        UCB1IFG |= UCTXIFG;
    /* ---- OR ----*/
    /* Triggering the DMAREQ bit */
    DMA1CTL |= DMAREQ;

    /* If using low power mode, only go into LPM0
     * as LPM3 disables the clock that is needed for the DMA transfer
     * (MCLK source = DCO or XTAL) and it needs to be switched on with
     *  the defined waiting time before the transfer can take place.  */
    //        __bis_SR_register(LPM0_bits + GIE);//ACLK remains active
    /* Just twiddle our thumbs until the transfer's done */
//    while ((DMA1CTL & DMAEN) != 0)
//    {
//        _NOP();
//    }

    /* Should only get here if a single transfer completes
     * and resets DMAEN to 0 */
    _NOP();
}

/***************************************************************************//**
 * @brief   Send a frame of bytes via SPI
 * @param   pBuffer Place that holds the bytes to send
 * @param   size Indicator of how many bytes to send
 * @return  None
 ******************************************************************************/

void SDCard_sendFrame(uint8_t *pBuffer, uint16_t size)
{
    //uint16_t gie = __get_SR_register() & GIE;              // Store current GIE state

    //__disable_interrupt();                                 // Make this operation atomic

    // Clock the actual data transfer and send the bytes. Note that we
    // intentionally not read out the receive buffer during frame transmission
    // in order to optimize transfer speed, however we need to take care of the
    // resulting overrun condition.
    _NOP();
    while (size--)
    {
        while (!(UCB1IFG & UCTXIFG))
            ;                     // Wait while not ready for TX
        UCB1TXBUF = *pBuffer++;                            // Write byte
    }

    while (UCB1STAT & UCBUSY)
        ;                            // Wait for all TX/RX to finish

    UCB1RXBUF;                                  // Dummy read to empty RX buffer
    _NOP();                               // and clear any overrun conditions

    //__bis_SR_register(gie);                                // Restore original GIE state
}

/*
 * Experimental DMA transfer
 */
void SDCard_sendFrame_DMA(uint8_t *pBuffer, uint16_t size)
{
        /* Source DMA address: the data buffer. */
        __data16_write_addr((unsigned short) &DMA1SA, (unsigned long) pBuffer);

        /* Destination DMA address: the UART send register. */
        __data16_write_addr((unsigned short) &DMA1DA,
                            (unsigned long) &UCB1TXBUF);



        /* DMA trigger is either:
         * DMA_REQ - TSEL_0
         *  OR
         * USCIB1 transmit - TSEL_23
         *
         * Note:
         * The DMAxTSELbits should be modified only when the DMACTLx DMAEN bit is 0
         * When selecting the trigger,the trigger must not have already occurred,
         * or the transfer does not take place
         * */
        DMACTL0 = DMA1TSEL_0 | DMA1TSEL_23;

        /* Read-modify-write disable
         *
         * Brief:
         * The DMARMWDIS bit controls when the CPU is halted for DMA transfers.
         *
         * Detail:
         * When DMARMWDIS = 0, the CPU is halted immediately and the transfer begins
         * when a trigger is received. In this case, it is possible that CPU read-modify-write
         * operations can be interrupted by a DMA transfer. When DMARMWDIS = 1,the CPU finishes
         * the currently executing read-modify-write operation before the DMA controller halts
         * the CPU and the transfer begins.
         *
         */
        DMACTL4 = DMARMWDIS;

        /* Disable any pending interrupts */
        DMA1CTL &= ~DMAIFG;

        /* Configure the DMA transfer*/
        DMA1CTL =
        DMADT_0 | /* Single transfer mode - should toggle DMAEN from 1->0 after transfer of DMAxSZ bytes*/
        DMASBDB | /* Byte mode */
        DMAEN | /* Enable DMA */
//        DMAIE | /* Enable interrupt */
        DMASRCBYTE | /* DMA source byte */
        DMADSTBYTE | /* DMA destination byte */
        DMASRCINCR_3 | /* Increment the source address */
        DMADSTINCR_0; /* Destination address unchanged */

        _NOP();

        /* The size of the block to be transferred */
        DMA1SZ = size;

        /* Clear any pending flags */
//        UCB1IFG &= ~(UCRXIFG | UCTXIFG);
        UCB1IFG &= ~(UCTXIFG);

        /* Kick off the transfer by either:
         * sending the first byte, the "start block" token
         * OR
         * forcing an interrupt where we need it
         * OR
         * triggering the DMAREQ bit */

        /* Triggering the DMAREQ bit */
        DMA1CTL |= DMAREQ;
        /* ---- OR ----*/
        /* Send the first byte */
//        UCB1TXBUF = pBuffer[0];
        /* ---- OR ----*/
        /* Force an interrupt to occur and get the ball rolling*/
        UCB1IFG |= UCTXIFG;

        /* If using low power mode, only go into LPM0
         * as LPM3 disables the clock that is needed for the DMA transfer
         * (MCLK source = DCO or XTAL) and it needs to be switched on with
         *  the defined waiting time before the transfer can take place.  */
//        __bis_SR_register(LPM0_bits + GIE);
//        __bis_SR_register(GIE);
        /* Just twiddle our thumbs until the transfer's done */
        while ((DMA1CTL & DMAEN) != 0);

        /* Should only get here if a single transfer completes
         * and resets DMAEN to 0 */
        _NOP();
//        UCB1RXBUF;

}


/***************************************************************************//**
 * @brief   Set the SD Card's chip-select signal to high
 * @param   None
 * @return  None
 ******************************************************************************/

void SDCard_setCSHigh(void)
{
   SD_CS_OUT |= SD_CS;
}

/***************************************************************************//**
 * @brief   Set the SD Card's chip-select signal to low
 * @param   None
 * @return  None
 ******************************************************************************/

void SDCard_setCSLow(void)
{
   SD_CS_OUT &= ~SD_CS;
}

/***************************************************************************//**
 * @}
 ******************************************************************************/
