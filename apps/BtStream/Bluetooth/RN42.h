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

#ifndef RN42_H
#define RN42_H

#include <stdint.h>

enum {
   SLAVE_MODE,
   MASTER_MODE,
   TRIGGER_MASTER_MODE,
   AUTO_MASTER_MODE
};

extern void BT_init();
extern void BT_configure();
extern void BT_disable();

//write data to be transmitted to the Bluetooth module
//returns 0 if fails, else 1
//will only fail if a previous BT_write is still in progress
extern uint8_t BT_write(uint8_t *buf, uint8_t len);

//connect to a specific device that was previously discovered
extern uint8_t BT_connect(uint8_t *addr);

//after this command is called there will be no link to the connected device
extern uint8_t BT_disconnect();


//mode: SLAVE_MODE, MASTER_MODE, TRIGGER_MASTER_MODE, AUTO_MASTER_MODE
extern void BT_setRadioMode(uint8_t mode);

extern void BT_setDiscoverable(uint8_t disc);
extern void BT_setEncryption(uint8_t enc);
extern void BT_setAuthentication(uint8_t mode);
extern void BT_setName(char *name);             // max 16 chars
extern void BT_setFriendlyName(char *name);     //max 15 chars
extern void BT_setPIN(char *name);              // max 16 chars
extern void BT_setServiceClass(char *serviceClass); // max 4 chars (hex word)
extern void BT_setServiceName(char *name);         // max 16 chars
extern void BT_setDeviceClass(char *deviceClass);  // max 4 chars (hex word)
extern void BT_disableRemoteConfig(uint8_t disableConfig);

//rate_factor is baudrate * 0.004096, e.g. to set 115200, pass in "472"
extern void BT_setRawBaudrate(char *rateFactor);   // max 4 chars, must be integer

//provide one of the following as a string argument:
//1200, 2400, 4800, 9600, 19.2, 38.4, 57.6, 115K, 230K, 460K, 921K
extern void BT_setBaudrate(char *newBaud);

//save power by minimising time Inquiry/Page scanning
//module reset necessary for changes to take effect
extern void BT_setPagingTime(char *hexvalTime);    // max 4 chars (hex word)
extern void BT_setInquiryTime(char *hexvalTime);   // max 4 chars (hex word)

extern void BT_resetDefaults();

//set new baud rate. This change is effective immediately.
//This change is only temporary. Reverts to previously configured rate after reset.
//The string argument must be one of the following and EXACTLY 4 characters:
//1200, 2400, 4800, 9600, 19.2(K), 38.4(K), 57.6(K), 115K, 230K, 460K or 921K
//If any other value is used this function does nothing
extern void BT_setTempBaudRate(char * baudRate);

//pass in a pointer to the function that will get called when
//data arrives
//the passed in function returns a 1 if program execution should resume
//(i.e. clear LPM3 bits)
//otherwise return 0
extern void BT_receiveFunction(uint8_t (*receiveFuncPtr)(uint8_t data));


//this function needs to be called from within the BT_PIO ISR
//in order to inform the RN42 driver about the state change
//value needs to be 1 if interrupt was low to high else 0
extern void BT_connectionInterrupt(uint8_t value);

//this function needs to be called from within the BT_RTS ISR
//in order to inform the RN42 driver about the state change
//value needs to be 1 if interrupt was low to high else 0
extern void BT_rtsInterrupt(uint8_t value);

#endif //RN42_H
