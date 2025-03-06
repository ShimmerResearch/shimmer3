/*
 * hal_RTC.c
 *
 *  Created on: Mar 6, 2015
 *      Author: WeiboP
 */


#include <stdint.h>
#include "msp430.h"
#include "hal_RTC.h"

uint32_t rtcLocalTime32h;// rtcLocalTime32l from get() functions

uint64_t rwcTimeDiff64;
uint64_t rwcConfigTime64;

void RTC_init(uint64_t rtc_val)
{
   rtcLocalTime32h = (rtc_val >> 32) & 0xffffffff;
   RTCCTL01 = RTCHOLD + RTCSSEL_0 + RTCTEV_3 + RTCTEVIE;     //hold
   RTCTIM0 = (uint16_t)(rtc_val & 0xffff);
   RTCTIM1 = (uint16_t)((rtc_val>>16)&0xffff);
   RTCCTL01 &= ~RTCHOLD;              //start

   rwcTimeDiff64 = 0;
   rwcConfigTime64 = 0;
}

uint32_t RTC_get32(void)
{
   register uint16_t t0, rtc0, rtc1;
//   uint8_t ie;
   uint32_t rtc_my_local_time_32;

//    if (ie = (__get_SR_register() & GIE)) //interrupts enabled?
//      __disable_interrupt();
   do
   {
       rtc1 = RTCTIM1;
       do {t0=rtc1; rtc1=RTCTIM1;} while(t0!=rtc1);
       rtc0 = RTCTIM0;
       do {t0=rtc0; rtc0=RTCTIM0;} while(t0!=rtc0);
       /* check if overflow occurred */
       t0 = RTCTIM1;
   }
   while (t0!=rtc1);

//   if(ie)
//      __enable_interrupt();

   rtc_my_local_time_32 = ((uint32_t)rtc1<<16)+rtc0;
   return rtc_my_local_time_32;
}

uint64_t RTC_get64(void)
{
   uint64_t rtc_my_local_time_64;
   rtc_my_local_time_64 = ((uint64_t)rtcLocalTime32h<<32)+(uint64_t)RTC_get32();
   return rtc_my_local_time_64;
}

uint64_t getRwcTime(void)
{
    return rwcTimeDiff64 + RTC_get64();
}

void setRwcTime(uint8_t * args)
{
    memcpy((uint8_t*) (&rwcConfigTime64), args, 8); // 64bits = 8bytes
    rwcTimeDiff64 = rwcConfigTime64 - RTC_get64(); // this is the offset to be stored int the sd header
}

uint64_t getRwcConfigTime(void)
{
    return rwcConfigTime64;
}

uint64_t * getRwcConfigTimePtr(void)
{
    return &rwcConfigTime64;
}

uint64_t getRwcTimeDiff(void)
{
    return rwcTimeDiff64;
}

uint64_t * getRwcTimeDiffPtr(void)
{
    return &rwcTimeDiff64;
}

uint8_t isRwcTimeSet(void)
{
    return ((rwcConfigTime64 > 0) ? 1 : 0);
}

#pragma vector=RTC_VECTOR
__interrupt void RTC_ISR(void)
{
   switch(__even_in_range(RTCIV,16))
   {
      case 0: break;                          // No interrupts
      case 2: break;                          // RTCRDYIFG
      case 4:                                 // RTCEVIFG
         rtcLocalTime32h ++;
      break;
      case 6: break;                          // RTCAIFG
      case 8: break;                          // RT0PSIFG
      case 10: break;                         // RT1PSIFG
      case 12: break;                         // Reserved
      case 14: break;                         // Reserved
      case 16: break;                         // Reserved
      default: break;
   }
}
