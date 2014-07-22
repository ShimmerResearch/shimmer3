/*
 $License:
    Copyright (C) 2011 InvenSense Corporation, All Rights Reserved.
 $
 */
/******************************************************************************
 * $Id: msp430_clock.c $
 *****************************************************************************/
/**
 *  @defgroup MSP430-SL
 *  @brief  MSP430 System Layer APIs.
 *
 *  @{
 *      @file       msp430_clock.h
 *      @brief      Functions to configure the MSP430 system clock
 *      @details    clock source = 32.768kHz oscillator, which may vary in
 *               	performance between multiple chips. Thiss may be an issue
 *               	for time-critical tasks such as providing a baud rate
 *               	reference.
 */

#include "string.h"
#include "msp430.h"
#include "msp430_clock.h"
#include <math.h>

#define MAX_CB 10

struct msp430_clock_s {
   volatile uint32_t timestamp;
   unsigned long aclk;
   unsigned short ms_per_interrupt;
   unsigned char enabled;
   unsigned short ticks_per_interrupt;
   unsigned long timer_remaining_ms[MAX_CB];
   uint8_t timer_exit_lpm[MAX_CB];
   void (*timer_cb[MAX_CB])(void);
};
static struct msp430_clock_s clock;
uint8_t cb_cnt, in_delay_ms;
uint32_t last_ts, this_ts;

#pragma vector=TIMER1_A0_VECTOR
__interrupt void TIMER1_A0_ISR (void)
{
   uint8_t j,i=cb_cnt, exit_lpm=0;
   uint16_t td_in_ms;

   this_ts = (uint32_t)GetTA1();
   TA1CCR0 = this_ts + clock.ticks_per_interrupt;
   td_in_ms = fmod(65536 + this_ts - last_ts,65536)/clock.ticks_per_interrupt;
   last_ts = this_ts;
   clock.timestamp += td_in_ms;

   while(i--){
      if (clock.timer_remaining_ms[i]) {
         if(clock.timer_remaining_ms[i]>td_in_ms)
            clock.timer_remaining_ms[i] -= clock.ms_per_interrupt;
         else
            clock.timer_remaining_ms[i] = 0;

         if (!clock.timer_remaining_ms[i]){
            clock.timer_cb[i]();
            if(clock.timer_exit_lpm[i])
               exit_lpm = 1;
            while(j<cb_cnt-1){//some cb has gone, every one after him move forward
               clock.timer_remaining_ms[j] = clock.timer_remaining_ms[j+1];
               clock.timer_cb[j] = clock.timer_cb[j+1];
               clock.timer_exit_lpm[j] = clock.timer_exit_lpm[j+1];
               j++;
            }
            cb_cnt--;
         }
      }
   }
   if(exit_lpm)// must be in main loop
      __bic_SR_register_on_exit(LPM3_bits);
   else
      if(in_delay_ms)// must NOT be in main loop
         __bic_SR_register_on_exit(LPM0_bits);
}

int msp430_clock_enable(void)
{
   if (clock.enabled)
      return 0;

   /* Number of ticks per millisecond. */
   //clock.aclk = 32768;
   clock.ticks_per_interrupt = clock.aclk / 1000;
   TA1CCR0 = clock.ticks_per_interrupt;

   TA1CCTL0 = CCIE;
   TA1CTL = TASSEL_1 + MC_2 + TACLR; //ACLK, continuous mode, clear TAR

   clock.ms_per_interrupt = 1;
   clock.enabled = 1;

   /* Enable interrupts. */
   __bis_SR_register(GIE);

   return 0;
}

int msp430_clock_disable(void)
{
   if (!clock.enabled)
      return 0;

   /* Previously, we would switch to VL0 and continue to increment the
    * millisecond clock. However, we don't need it anymore and can disable the
    * timer completely. VL0 isn't reliable at all anyway.
    */
   TA1CTL &= ~MC_3;
   TA1CCTL0 &= ~CCIE;
   TA1CTL &= ~TAIFG;

   clock.enabled = 0;
   return 0;
}

int msp430_clock_init(void)
{
   clock.aclk = 32768;
   last_ts = 0;

   /* Start the millisecond clock. */
   clock.enabled=0;
   msp430_clock_enable();

   /* Start timestamp at zero. */
   clock.timestamp = 0;
   cb_cnt=0;
   in_delay_ms=0;
   memset(clock.timer_cb,0,MAX_CB);
   memset(clock.timer_remaining_ms,0,MAX_CB);
   return 0;
}

int msp430_get_aclk_freq(unsigned long *aclk)
{
   aclk[0] = clock.aclk;
   return 0;
}

int msp430_get_clock_ms(unsigned long *count)
{
   if (!count)
      return 1;
   count[0] = clock.timestamp;
   return 0;
}

int msp430_delay_ms(unsigned long num_ms)
{
   uint32_t start_time = clock.timestamp;
   if (!clock.enabled)
      return -1;
   in_delay_ms = 1;
   while (clock.timestamp - start_time < num_ms)
      __bis_SR_register(LPM0_bits + GIE);
   in_delay_ms = 0;
   return 0;
}

inline uint16_t GetTA1(void) {
   register uint16_t t0, t1;
   uint8_t ie;
   if(ie=(__get_SR_register()&0x0008))   //interrupts enabled?
      __disable_interrupt();
   t1 =TA1R;
   do {t0=t1; t1=TA1R;} while(t0!=t1);
   if(ie)
      __enable_interrupt();
   return t1;
}

int msp430_register_timer_cb(void (*timer_cb)(void), unsigned long num_ms, uint8_t exit_lpm)
{
   if (!timer_cb || !num_ms) {
      clock.timer_cb[cb_cnt] = NULL;
      clock.timer_remaining_ms[cb_cnt] = 0;
      clock.timer_exit_lpm[cb_cnt]=0;
      return 0;
   }

   /* Timer count needs to be evenly divisible by clock.ms_per_interrupt to
    * avoid overflow.
    */
   clock.timer_remaining_ms[cb_cnt] = num_ms;
   clock.timer_cb[cb_cnt] = timer_cb;
   clock.timer_exit_lpm[cb_cnt] = exit_lpm;
   cb_cnt++;
   return 0;
}
