/*
 * led.h
 *
 *  Created on: 25 Mar 2025
 *      Author: MarkNolan
 */

#ifndef LED_H_
#define LED_H_

#include "stdint.h"

#include <log_and_stream_definitions.h>

void LED_varsInit(void);
void LED_incrementCounters(void);
void LED_controlDuringBoot(boot_stage_t bootStageCurrent);
void LED_control(void);
void LED_battBlinkOn();
uint8_t LED_isBlinkCntTime1s(void);
uint8_t LED_isBlinkCntTime2s(void);

void RwcCheck(void);

#endif /* LED_H_ */
