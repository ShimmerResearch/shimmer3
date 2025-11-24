/*
 * spi.c
 *
 *  Created on: 24 Mar 2025
 *      Author: MarkNolan
 */

#include "spi.h"

#include <stdint.h>

#include "Shimmer_Driver/shimmer_driver_include.h"

/* Variable for SR47-4 (and later) to indicate ADS clock lines are tied */
uint8_t adsClockTied;

void SPI_varsInit(void)
{
  /* Variable for SR47-4 (and later) to indicate ADS clock lines are tied */
  adsClockTied = 0;
}

void SPI_startSensing(void)
{
  gConfigBytes *storedConfigPtr = ShimConfig_getStoredConfig();

  /* ExG */
  if (storedConfigPtr->chEnExg1_24Bit || storedConfigPtr->chEnExg2_24Bit
      || storedConfigPtr->chEnExg1_16Bit || storedConfigPtr->chEnExg2_16Bit)
  {
    EXG_init();

    if (storedConfigPtr->chEnExg1_24Bit || storedConfigPtr->chEnExg1_16Bit)
    {
      EXG_writeRegs(0, ADS1292R_CONFIG1, 10,
          &storedConfigPtr->exgADS1292rRegsCh1.rawBytes[0]);
    }

    /* This second long delay was added to satisfy program flow requirements
     * of the ADS1292R as per pg 63 of its datasheet. */
    __delay_cycles(24000000);

    if (storedConfigPtr->chEnExg2_24Bit || storedConfigPtr->chEnExg2_16Bit)
    {
      EXG_writeRegs(1, ADS1292R_CONFIG1, 10,
          &storedConfigPtr->exgADS1292rRegsCh2.rawBytes[0]);
    }
    /* probably turning on internal reference, so wait for it to settle */
    __delay_cycles(2400000); /* 100ms (assuming 24MHz clock) */

    //probably setting the PGA gain so cancel the channel offset
    if ((storedConfigPtr->chEnExg1_24Bit || storedConfigPtr->chEnExg1_16Bit)
        && (storedConfigPtr->exgADS1292rRegsCh1.resp2 & BIT7))
    {
      EXG_offsetCal(0);
    }
    if ((storedConfigPtr->chEnExg2_24Bit || storedConfigPtr->chEnExg2_16Bit)
        && (storedConfigPtr->exgADS1292rRegsCh2.resp2 & BIT7))
    {
      EXG_offsetCal(1);
    }

    if ((storedConfigPtr->chEnExg1_24Bit || storedConfigPtr->chEnExg1_16Bit)
        && (storedConfigPtr->chEnExg2_24Bit || storedConfigPtr->chEnExg2_16Bit))
    {
      EXG_start(2);
    }
    else if (storedConfigPtr->chEnExg1_24Bit || storedConfigPtr->chEnExg1_16Bit)
    {
      EXG_start(0);
    }
    else
    {
      EXG_start(1);
    }

    if (ShimBrd_areADS1292RClockLinesTied())
    {
      /* Check if unit is SR47-4 or greater.
       * If so, amend configuration byte 2 of ADS chip 1 to have bit 3 set to 1.
       * This ensures clock lines on ADS chip are correct
       */
      ADS1292_disableDrdyInterrupts(ADS1292_DRDY_INT_CHIP2);
      adsClockTied = 1;
    }
  }
}

void SPI_stopSensing(void)
{
  gConfigBytes *storedConfigPtr = ShimConfig_getStoredConfig();

  if (storedConfigPtr->chEnExg1_24Bit || storedConfigPtr->chEnExg1_16Bit)
  {
    EXG_stop(0); //probably not needed
  }
  if (storedConfigPtr->chEnExg2_24Bit || storedConfigPtr->chEnExg2_16Bit)
  {
    EXG_stop(1); //probably not needed
  }
  if (!shimmerStatus.docked
      && (storedConfigPtr->chEnExg1_24Bit || storedConfigPtr->chEnExg2_24Bit
          || storedConfigPtr->chEnExg1_16Bit || storedConfigPtr->chEnExg2_16Bit))
  {
    EXG_powerOff();
  }
}

void SPI_configureChannels(void)
{
  uint8_t *channel_contents_ptr = &sensing.cc[sensing.ccLen];
  uint8_t nbr_spi_chans = 0;
  gConfigBytes *storedConfigPtr = ShimConfig_getStoredConfig();

  if (storedConfigPtr->chEnExg1_24Bit)
  {
    *channel_contents_ptr++ = EXG_ADS1292R_1_STATUS;
    *channel_contents_ptr++ = EXG_ADS1292R_1_CH1_24BIT;
    *channel_contents_ptr++ = EXG_ADS1292R_1_CH2_24BIT;
    nbr_spi_chans += 3;
    sensing.ptr.exg1 = sensing.dataLen;
    sensing.dataLen += 7;
  }
  else if (storedConfigPtr->chEnExg1_16Bit)
  {
    *channel_contents_ptr++ = EXG_ADS1292R_1_STATUS;
    *channel_contents_ptr++ = EXG_ADS1292R_1_CH1_16BIT;
    *channel_contents_ptr++ = EXG_ADS1292R_1_CH2_16BIT;
    nbr_spi_chans += 3;
    sensing.ptr.exg1 = sensing.dataLen;
    sensing.dataLen += 5;
  }
  if (storedConfigPtr->chEnExg2_24Bit)
  {
    *channel_contents_ptr++ = EXG_ADS1292R_2_STATUS;
    *channel_contents_ptr++ = EXG_ADS1292R_2_CH1_24BIT;
    *channel_contents_ptr++ = EXG_ADS1292R_2_CH2_24BIT;
    nbr_spi_chans += 3;
    sensing.ptr.exg2 = sensing.dataLen;
    sensing.dataLen += 7;
  }
  else if (storedConfigPtr->chEnExg2_16Bit)
  {
    *channel_contents_ptr++ = EXG_ADS1292R_2_STATUS;
    *channel_contents_ptr++ = EXG_ADS1292R_2_CH1_16BIT;
    *channel_contents_ptr++ = EXG_ADS1292R_2_CH2_16BIT;
    nbr_spi_chans += 3;
    sensing.ptr.exg2 = sensing.dataLen;
    sensing.dataLen += 5;
  }

  sensing.ccLen += nbr_spi_chans;
  sensing.nbrSpiChans = nbr_spi_chans;
}

void SPI_pollSensors(void)
{
  gConfigBytes *storedConfigPtr = ShimConfig_getStoredConfig();
  volatile uint8_t *dataBufPtr = ShimSens_getDataBuffAtWrIdx();

  if (storedConfigPtr->chEnExg1_24Bit)
  {
    EXG_readData(0, 0, &dataBufPtr[sensing.ptr.exg1]);
  }
  else if (storedConfigPtr->chEnExg1_16Bit)
  {
    EXG_readData(0, 1, &dataBufPtr[sensing.ptr.exg1]);
  }
  if (storedConfigPtr->chEnExg2_24Bit)
  {
    EXG_readData(1, 0, &dataBufPtr[sensing.ptr.exg2]);
    if (!(dataBufPtr[sensing.ptr.exg2 + 1] == 0x00 || dataBufPtr[sensing.ptr.exg2 + 1] == 0xff))
    {
      _NOP();
    }
  }
  else if (storedConfigPtr->chEnExg2_16Bit)
  {
    EXG_readData(1, 1, &dataBufPtr[sensing.ptr.exg2]);
  }

  ShimSens_spiCompleteCb();
}

void adsClockTiedSet(uint8_t state)
{
  adsClockTied = state;
}

uint8_t adsClockTiedGet(void)
{
  return adsClockTied;
}
