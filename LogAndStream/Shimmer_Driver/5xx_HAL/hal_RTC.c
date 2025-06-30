/*
 * hal_RTC.c
 *
 *  Created on: Mar 6, 2015
 *      Author: WeiboP
 */

#include "hal_RTC.h"
#include "msp430.h"
#include <stdint.h>

#include <log_and_stream_includes.h>

uint32_t rtcLocalTime32h; //rtcLocalTime32l from get() functions

/* Stored the time difference between the MCU boot time and the RWC */
uint64_t rwcTimeDiff64;

void RTC_init(uint64_t rtc_val)
{
  rtcLocalTime32h = (rtc_val >> 32) & 0xffffffff;
  RTCCTL01 = RTCHOLD + RTCSSEL_0 + RTCTEV_3 + RTCTEVIE; //hold
  RTCTIM0 = (uint16_t) (rtc_val & 0xffff);
  RTCTIM1 = (uint16_t) ((rtc_val >> 16) & 0xffff);
  RTCCTL01 &= ~RTCHOLD; //start

  rwcTimeDiff64 = 0;
}

// For Shimmer3, this returns time since boot in ticks
uint32_t RTC_get32(void)
{
  register uint16_t t0, rtc0, rtc1;
  //uint8_t ie;
  uint32_t rtc_my_local_time_32;

  //if (ie = (__get_SR_register() & GIE)) //interrupts enabled?
  //  __disable_interrupt();
  do
  {
    rtc1 = RTCTIM1;
    do
    {
      t0 = rtc1;
      rtc1 = RTCTIM1;
    } while (t0 != rtc1);
    rtc0 = RTCTIM0;
    do
    {
      t0 = rtc0;
      rtc0 = RTCTIM0;
    } while (t0 != rtc0);
    /* check if overflow occurred */
    t0 = RTCTIM1;
  } while (t0 != rtc1);

  //if(ie)
  //   __enable_interrupt();

  rtc_my_local_time_32 = ((uint32_t) rtc1 << 16) + rtc0;
  return rtc_my_local_time_32;
}

// For Shimmer3, this returns time since boot in ticks
uint64_t RTC_get64(void)
{
  uint64_t rtc_my_local_time_64;
  rtc_my_local_time_64 = ((uint64_t) rtcLocalTime32h << 32) + (uint64_t) RTC_get32();
  return rtc_my_local_time_64;
}

// For Shimmer3, this returns time since boot in ticks + RWC offset
uint64_t RTC_getRwcTime(void)
{
  return rwcTimeDiff64 + RTC_get64();
}

void RTC_setTimeFromTicksPtr(uint8_t *ticksPtr)
{
  uint64_t time64 = 0;
  memcpy((uint8_t *) (&time64), ticksPtr, 8); //64bits = 8bytes
  ShimRtc_setRwcConfigTime(time64);

  // Calculate RWC offset
  rwcTimeDiff64 = time64 - RTC_get64();
}

uint64_t RTC_getRwcTimeDiff(void)
{
  return rwcTimeDiff64;
}

uint64_t *RTC_getRwcTimeDiffPtr(void)
{
  return &rwcTimeDiff64;
}

uint8_t RTC_isRwcTimeSet(void)
{
  return rwcTimeDiff64 != 0;
}

#pragma vector = RTC_VECTOR

__interrupt void RTC_ISR(void)
{
  switch (__even_in_range(RTCIV, 16))
  {
    case 0:
      break; //No interrupts
    case 2:
      break; //RTCRDYIFG
    case 4:  //RTCEVIFG
      rtcLocalTime32h++;
      break;
    case 6:
      break; //RTCAIFG
    case 8:
      break; //RT0PSIFG
    case 10:
      break; //RT1PSIFG
    case 12:
      break; //Reserved
    case 14:
      break; //Reserved
    case 16:
      break; //Reserved
    default:
      break;
  }
}
