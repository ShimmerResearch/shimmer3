/*
 * hal_CRC.c
 *
 *  Created on: 23 Sep 2014
 *      Author: WeiboP
 */
#include "hal_CRC.h"


uint16_t CRC_data(uint8_t *buf, uint8_t len){
   uint8_t i;
   uint16_t crc_val=0;

   CRCINIRES = CRC_INIT;
   if(len%2) {
      for(i=0; i<((len-1)/2); i++) {
         CRCDIRB = *(((uint16_t *)buf)+i);
      }
      CRCDIRB = (uint16_t)(buf[len-1]);
   } else {
      for(i=0; i<(len/2); i++) {
         CRCDIRB = *(((uint16_t *)buf)+i);
      }
   }
   crc_val = CRCINIRES;
   return crc_val;
}

