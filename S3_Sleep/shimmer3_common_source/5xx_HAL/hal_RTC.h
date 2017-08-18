/*
 * hal_RTC.h
 *
 *  Created on: Mar 6, 2015
 *      Author: WeiboP
 */

#ifndef HAL_RTC_H_
#define HAL_RTC_H_

//initiate Real Time Clock with an int64 value
extern void RTC_init(uint64_t rtc_val);

// get 32 bit RTC value
extern uint32_t RTC_get32(void);

// get 64 bit RTC value
// there can be at most 32 bit counter for higher than 32bits
extern uint64_t RTC_get64(void);

#endif /* HAL_RTC_H_ */
