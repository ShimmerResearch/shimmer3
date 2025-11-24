/*
 * adc.c
 *
 *  Created on: 24 Mar 2025
 *      Author: MarkNolan
 */

#include "adc.h"

#include <stdint.h>

#include "msp430.h"

#include "Shimmer_Driver/shimmer_definitions.h"
#include "Shimmer_Driver/shimmer_driver_include.h"

uint16_t *adcStartPtr;

/*battery evaluation vars*/
uint8_t battVal[3];
uint8_t battWait;

void ADC_varsInit(void)
{
  battWait = 0;
}

void ADC_startSensing(void)
{
  gConfigBytes *storedConfigPtr = ShimConfig_getStoredConfig();

  if (storedConfigPtr->chEnLnAccel)
  {
    P8REN &= ~BIT6; //disable pull down resistor
    P8DIR |= BIT6;  //set as output
    P8SEL &= ~BIT6; //analog accel being used so take out of sleep mode
    P8OUT |= BIT6;  //analog accel being used so take out of sleep mode
  }

  if (storedConfigPtr->chEnBridgeAmp)
  {
    P2OUT |= BIT0; //GPIO_INTERNAL1 set high
  }

  if (storedConfigPtr->chEnGsr)
  {
    GSR_init(storedConfigPtr->gsrRange, storedConfigPtr->samplingRateTicks);
  }
}

void ADC_stopSensing(void)
{
  ADC_disable();
  DMA0_disable();

  P8OUT &= ~BIT6;
  //P8REN |= BIT6;      //enable pull down resistor
  //P8DIR &= ~BIT6;     //SW_ACCEL set as input

  P2OUT &= ~BIT0; //set GPIO_INTERNAL1 low (strain)
}

void ADC_configureChannels(void)
{
  uint8_t *channel_contents_ptr = &sensing.cc[sensing.ccLen];
  uint8_t nbr_adc_chans = 0;
  gConfigBytes *storedConfigPtr = ShimConfig_getStoredConfig();
  uint16_t mask = 0;

  //Analog Accel
  if (storedConfigPtr->chEnLnAccel)
  {
    *channel_contents_ptr++ = X_LN_ACCEL;
    *channel_contents_ptr++ = Y_LN_ACCEL;
    *channel_contents_ptr++ = Z_LN_ACCEL;
    mask += MASK_A_ACCEL;
    nbr_adc_chans += 3;
    sensing.ptr.accel1 = sensing.dataLen;
    sensing.dataLen += 6;
  }
  //Battery Voltage
  if (storedConfigPtr->chEnVBattery)
  {
    *channel_contents_ptr++ = VBATT;
    mask += MASK_VBATT;
    nbr_adc_chans++;
    sensing.ptr.batteryAnalog = sensing.dataLen;
    sensing.dataLen += 2;
  }
  //External ADC channel A7
  if (storedConfigPtr->chEnExtADC7)
  {
    *channel_contents_ptr++ = EXTERNAL_ADC_7;
    mask += MASK_EXT_A7;
    nbr_adc_chans++;
    sensing.ptr.extADC0 = sensing.dataLen;
    sensing.dataLen += 2;
  }
  //External ADC channel A6
  if (storedConfigPtr->chEnExtADC6)
  {
    *channel_contents_ptr++ = EXTERNAL_ADC_6;
    mask += MASK_EXT_A6;
    nbr_adc_chans++;
    sensing.ptr.extADC1 = sensing.dataLen;
    sensing.dataLen += 2;
  }
  //External ADC channel A15
  if (storedConfigPtr->chEnExtADC15)
  {
    *channel_contents_ptr++ = EXTERNAL_ADC_15;
    mask += MASK_EXT_A15;
    nbr_adc_chans++;
    sensing.ptr.extADC2 = sensing.dataLen;
    sensing.dataLen += 2;
  }
  //Internal ADC channel A12
  if (storedConfigPtr->chEnIntADC12)
  {
    *channel_contents_ptr++ = INTERNAL_ADC_12;
    mask += MASK_INT_A12; //ppg
    nbr_adc_chans++;
    sensing.ptr.intADC0 = sensing.dataLen;
    sensing.dataLen += 2;
  }
  //Strain gauge
  if (storedConfigPtr->chEnBridgeAmp)
  {
    *channel_contents_ptr++ = STRAIN_HIGH;
    *channel_contents_ptr++ = STRAIN_LOW;
    mask += MASK_STRAIN;
    nbr_adc_chans += 2;
    sensing.ptr.strainGauge = sensing.dataLen;
    sensing.dataLen += 4;
  }
  //Internal ADC channel A13
  if (storedConfigPtr->chEnIntADC13 && !storedConfigPtr->chEnBridgeAmp)
  {
    *channel_contents_ptr++ = INTERNAL_ADC_13;
    mask += MASK_INT_A13;
    nbr_adc_chans++;
    sensing.ptr.intADC1 = sensing.dataLen;
    sensing.dataLen += 2;
  }
  //Internal ADC channel A14
  if (storedConfigPtr->chEnIntADC14 && !storedConfigPtr->chEnBridgeAmp)
  {
    *channel_contents_ptr++ = INTERNAL_ADC_14;
    mask += MASK_INT_A14;
    nbr_adc_chans++;
    sensing.ptr.intADC2 = sensing.dataLen;
    sensing.dataLen += 2;
  }
  //Internal ADC channel A1
  if (storedConfigPtr->chEnGsr)
  {
    //needs to be last analog channel
    *channel_contents_ptr++ = GSR_RAW;
    mask += MASK_INT_A1;
    nbr_adc_chans++;
    sensing.ptr.gsr = sensing.dataLen;
    sensing.dataLen += 2;
  }
  if (storedConfigPtr->chEnIntADC1)
  {
    *channel_contents_ptr++ = INTERNAL_ADC_1;
    mask += MASK_INT_A1;
    nbr_adc_chans++;
    sensing.ptr.intADC3 = sensing.dataLen;
    sensing.dataLen += 2;
  }

  sensing.nbrMcuAdcChans += nbr_adc_chans;
  sensing.ccLen += nbr_adc_chans;

  if (mask)
  {
    adcStartPtr = ADC_init(mask);
    DMA0_transferDoneFunction(&Dma0ConversionDone);
    if (adcStartPtr)
    {
      DMA0_init(adcStartPtr, (uint16_t *) &ShimSens_getDataBuffAtWrIdx()[FIRST_CH_BYTE_IDX],
          sensing.nbrMcuAdcChans);
    }
  }
}

void ADC_gatherDataStart(void)
{
  gConfigBytes *storedConfigPtr = ShimConfig_getStoredConfig();

  if (storedConfigPtr->chEnGsr)
  {
    //TODO get index right here. Think it should be looking at the previous buffer
    GSR_range(&ShimSens_getDataBuffAtWrIdx()[sensing.ptr.gsr]);
  }
}

uint8_t Dma0ConversionDone(void)
{
  if (battWait)
  {
    battWait = 0;
    //TODO check if this is really needed
    ShimSens_configureChannels();
    saveBatteryVoltageAndUpdateStatus();
  }
  else
  {
    //Destination address for next transfer
    DMA0_repeatTransfer(adcStartPtr,
        (uint16_t *) &ShimSens_getDataBuffAtWrIdx()[FIRST_CH_BYTE_IDX],
        sensing.nbrMcuAdcChans);
    ADC_disable(); //can disable ADC until next time sampleTimer fires (to save power)?
    DMA0_disable();
    ShimTask_set(TASK_GATHER_DATA);
    ShimSens_adcCompleteCb();
  }
  return 1;
}

uint8_t Dma0BatteryRead(void)
{
  ADC_disable();
  DMA0_disable();
  //== new uart starts ==
  //uartSendRspBat = 1;// don't do this for sdlog/L&S
  //TaskSet(TASK_UARTRSP);
  //== new uart ends ==
  return 1;
}

void SetBattDma(void)
{
  gConfigBytes *storedConfigPtr = ShimConfig_getStoredConfig();

  /* Ensure is off so all ADC12CTL0 and ADC12CTL1 fields can be modified */
  ADC12CTL0 &= ~ADC12ENC;

  /* ADC12SHT1_8 - SHT1 takes 256 ADC12CLK cycles
   * ADC12SHT0_8 - SHT0 takes 256 ADC12CLK cycles */
  ADC12CTL0 = ADC12SHT1_8 + ADC12SHT0_8;

  /* ADC12SHP - Use sampling timer
   * ADC12CONSEQ_0 - ADC12OSC and single channel mode (SEQ_0) */
  ADC12CTL1 = ADC12SHP + ADC12CONSEQ_0;

  /* ADC12TCOFF - Turn off temperature sensor (saves power)
   * ADC12RES_2 - ADC12 resolution to 12bits
   * ADC12SR_L  - Sampling rate 50ksps */
  ADC12CTL2 = ADC12TCOFF + ADC12RES_2 + ADC12SR_L;

  if (storedConfigPtr->chEnVBattery)
  {
    if (storedConfigPtr->chEnLnAccel)
    {
      ADC12CTL1 += ADC12CSTARTADD_4; //Start with ADC12MEM4
      ADC12MCTL4 = ADC12INCH_2;
    }
    else
    {
      ADC12CTL1 += ADC12CSTARTADD_1; //Start with ADC12MEM1
      ADC12MCTL1 = ADC12INCH_2;
    }
  }
  else
  {
    ADC12CTL1 += ADC12CSTARTADD_0; //Start with ADC12MEM0
    ADC12MCTL0 = ADC12INCH_2;
  }

  DMACTL0 |= DMA0TSEL_24; //ADC12IFGx triggered
  DMACTL4 = DMARMWDIS;    //Read-modify-write disable
  DMA0CTL &= ~DMAIFG;

  /* DMADT_2      - Burst block transfer
   * DMADSTINCR_3 - Increment destination address
   * DMADSRINCR_3 - Increment source address
   * DMAIE        - DMA interrupt enable
   * DMAEN        - DMA Enable */
  //DMA0CTL = DMADT_2 + DMADSTINCR_3 + DMASRCINCR_3 + DMAIE
  //        + ((shimmerStatus.isLogging || shimmerStatus.isStreaming) ? DMAEN : 0);
  DMA0CTL = DMADT_2 + DMADSTINCR_3 + DMASRCINCR_3 + DMAIE
      + (((shimmerStatus.sdLogging || shimmerStatus.btStreaming)
             && storedConfigPtr->chEnGsr) ?
              DMAEN :
              0);
  DMA0SZ = 1; //DMA0 size

  if (storedConfigPtr->chEnVBattery)
  {
    if (storedConfigPtr->chEnLnAccel)
    {
      DMA0SA = (__SFR_FARPTR) (unsigned long) &ADC12MEM4;
    }
    else
    {
      DMA0SA = (__SFR_FARPTR) (unsigned long) &ADC12MEM1;
    }
  }
  else
  {
    DMA0SA = (__SFR_FARPTR) (unsigned long) &ADC12MEM0;
  }

  //Writes a value to a 20-bit SFR register located at the given16/20-bit address
  DMA0DA = (__SFR_FARPTR) (unsigned long) battVal;

  DMA0_transferDoneFunction(&Dma0BatteryRead);

  ADC12IFG = 0;
  DMA0CTL |= DMAEN;
  ADC12CTL0 |= ADC12ON + ADC12ENC + ADC12SC;
}

void manageReadBatt(uint8_t isBlockingRead)
{
  SetBattDma();
  //Produces spikes in PPG data when only GSR enabled & aaccel, vbatt are off.
  __bis_SR_register(LPM3_bits + GIE); //ACLK remains active

  //TODO check if this is really needed
  ShimSens_configureChannels();

  saveBatteryVoltageAndUpdateStatus();
}

void saveBatteryVoltageAndUpdateStatus(void)
{
  uint16_t currentBattVal = *((uint16_t *) battVal);

  //Multiplied by 2 due to voltage divider
  uint16_t battValMV = (((uint32_t) currentBattVal * 3000) >> 12) * 2;

  ShimBatt_updateStatus(currentBattVal, battValMV, LM3658SD_STAT1, LM3658SD_STAT2);
}
