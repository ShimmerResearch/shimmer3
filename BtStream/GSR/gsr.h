/*
 * Copyright (c) 2013, Shimmer Research, Ltd.
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
 * @author Mike Healy
 * @date December, 2013
 */

#ifndef GSR_H
#define GSR_H

#include <stdint.h>

#define HW_RES_40K   0
#define HW_RES_287K 1
#define HW_RES_1M 2
#define HW_RES_3M3   3

void GSR_init(void);

//Adjusts the GSR's range by selecting the which internal resistor is used
//range: select the resistor to use
//    0:  40 kohm
//    1: 287 kohm
//    2: 1.0 Mohm
//    3: 3.3 Mohm
void GSR_setRange(uint8_t range);

//Calculates resistance from a raw ADC value using linear fit to conductance
//ADC_val: the ADC value to be used in the calculation
//active_resistor: the currently active resistor on the GSR board
//returns the calculated resistance
uint32_t GSR_calcResistance(uint16_t ADC_val, uint8_t active_resistor);

//Determines whether to change the currently active internal resistor based
//on the ADC value, and if necessary change the internal resistor to a new
//value
//ADC_val: the ADC value to be used in the calculation
//active_resistor: the currently active resistor on the GSR board
//returns the active internal resistor
uint8_t GSR_controlRange(uint16_t ADC_val, uint8_t active_resistor);

//Initializes the smoothing state
//active_resistor: the currently active resistor on the GSR board
void GSR_initSmoothing(uint8_t active_resistor);

//Smooths ADC values during autorange transition settling time.
//active_resistor: currently selected HW resistor
//sampling_period: sampling period of shimmer. Determines number of transient samples
//returns 1 if transient and 0 otherwise.
uint8_t GSR_smoothTransition(uint8_t *dummy_active_resistor, uint32_t sampling_period);

//Smooths the GSR values
//resistance: the current resistance value to smoothed
//active_resistor: the currently active resistor on the GSR board
//returns the smoothed resistance. Returns 0xFFFFFFFF if transient sample
uint32_t GSR_smoothSample(uint32_t resistance, uint8_t active_resistor);

#endif //GSR_H
