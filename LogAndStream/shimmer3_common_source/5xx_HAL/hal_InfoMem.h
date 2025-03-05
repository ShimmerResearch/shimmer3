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
 *
 * @modified Sam O'Mahony
 * @date January, 2018
 */

#ifndef HAL_INFOMEM_H
#define HAL_INFOMEM_H

#include <stdint.h>

#define INFOMEM_SEG_D 0x01
#define INFOMEM_SEG_C 0x02
#define INFOMEM_SEG_B 0x04
#define INFOMEM_SEG_A 0x08
#define INFOMEM_SEG_ALL 0x0F

#define INFOMEM_OFFSET 0x1800
#define INFOMEM_SIZE     512
#define INFOMEM_SEG_SIZE 128
#define INFOMEM_SEG_A_ADDR 0x1980
#define INFOMEM_SEG_B_ADDR 0x1900
#define INFOMEM_SEG_C_ADDR 0x1880
#define INFOMEM_SEG_D_ADDR 0x1800

//returns 1 if successful, 0 if failure
uint8_t InfoMem_write(uint8_t addr, uint8_t *buf, uint16_t size);

//returns 1 if successful, 0 if failure
uint8_t InfoMem_read(uint8_t addr, uint8_t *buf, uint16_t size);

void InfoMem_erase(uint8_t segments);

#endif /* HAL_INFOMEM_H */
