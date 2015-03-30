/*
 * Copyright (c) 2015, Shimmer Research, Ltd.
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
 * @author Weibo Pan
 * @date March, 2015
 *
 */

#include <stdint.h>
#include "msp430.h"
#include "hal_RTC.h"

uint32_t rtcLocalTime32h;// rtcLocalTime32l from get() functions


void RTC_init(uint64_t rtc_val)
{
   rtcLocalTime32h = (rtc_val >> 32) & 0xffffffff;
   RTCCTL01 = RTCHOLD + RTCSSEL_0 + RTCTEV_3 + RTCTEVIE;     //hold
   RTCTIM0 = (uint16_t)(rtc_val & 0xffff);
   RTCTIM1 = (uint16_t)((rtc_val>>16)&0xffff);
   RTCCTL01 &= ~RTCHOLD;              //start
}

uint32_t RTC_get32(void)
{
   register uint16_t t0, rtc0, rtc1;
   uint8_t ie;
   uint32_t rtc_my_local_time_32;

   if(ie=(__get_SR_register()&0x0008)) //interrupts enabled?
      __disable_interrupt();
   rtc0 = RTCTIM0;
   do {t0=rtc0; rtc0=RTCTIM0;} while(t0!=rtc0);
   rtc1 = RTCTIM1;
   do {t0=rtc1; rtc1=RTCTIM1;} while(t0!=rtc1);
   if(ie)
      __enable_interrupt();

   rtc_my_local_time_32 = ((uint32_t)rtc1<<16)+rtc0;
   return rtc_my_local_time_32;
}

uint64_t RTC_get64(void)
{
   uint64_t rtc_my_local_time_64;
   rtc_my_local_time_64 = ((uint64_t)rtcLocalTime32h<<32)+(uint64_t)RTC_get32();
   return rtc_my_local_time_64;
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
