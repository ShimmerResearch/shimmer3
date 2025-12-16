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
 * Edited:
 * @author Sam O'Mahony
 * @date March, 2017
 */

#ifndef BMPX80_H
#define BMPX80_H

#include <stdint.h>
#include <string.h>

#define BMP180_ADDR     (0x77) //7 bit address I2C address of the BMP180
#define BMP280_ADDR     (0x76) //7 bit address I2C address of the BMP180

//registers
#define BMP180_OUT_XLSB (0xF8)
#define BMP180_OUT_LSB  (0xF7)
#define BMP180_OUT_MSB  (0xF6)
#define CTRL_MEAS       (0xF4)
#define SOFT_RESET      (0xE0)
#define ID              (0xD0)

//calibration coefficients
#define AC1_MSB         (0xAA)
#define AC1_LSB         (0xAB)
#define AC2_MSB         (0xAC)
#define AC2_LSB         (0xAD)
#define AC3_MSB         (0xAE)
#define AC3_LSB         (0xAF)
#define AC4_MSB         (0xB0)
#define AC4_LSB         (0xB1)
#define AC5_MSB         (0xB2)
#define AC5_LSB         (0xB3)
#define AC6_MSB         (0xB4)
#define AC6_LSB         (0xB5)
#define B1_MSB          (0xB6)
#define B1_LSB          (0xB7)
#define B2_MSB          (0xB8)
#define B2_LSB          (0xB9)
#define MB_MSB          (0xBA)
#define MB_LSB          (0xBB)
#define MC_MSB          (0xBC)
#define MC_LSB          (0xBD)
#define MD_MSB          (0xBE)
#define MD_LSB          (0xBF)

/***************************************************************/
/**\name	COMMON USED CONSTANTS       */
/***************************************************************/

enum BMPXXX_IN_USE
{
  BMPXXX_NONE_IN_USE,
  BMP180_IN_USE,
  BMP280_IN_USE
};

#define BMP180_CALIB_DATA_SIZE                          (0x16) //22
#define BMP280_CALIB_DATA_SIZE                          (0x18) //24
#define BMP280_CALIB_XTRA_BYTES                         (0x02)

#define BMPX80_TEMP_BUFF_SIZE                           (0x02)
#define BMPX80_PRESS_BUFF_SIZE                          (0x03)
#define BMPX80_PACKET_SIZE                              (BMPX80_TEMP_BUFF_SIZE + BMPX80_PRESS_BUFF_SIZE)

/************************************************/
/**\name	POWER MODE DEFINITION       */
/***********************************************/
/* Sensor Specific constants */
#define BMP280_SLEEP_MODE                               (0x00)
#define BMP280_FORCED_MODE                              (0x01)
#define BMP280_NORMAL_MODE                              (0x03)
#define BMP280_SOFT_RESET_CODE                          (0xB6)

/************************************************/
/**\name	REGISTER ADDRESS DEFINITION       */
/***********************************************/
#define BMP280_CHIP_ID_REG                              (0xD0) /*Chip ID Register */
#define BMP280_RST_REG                                  (0xE0) /*Softreset Register */
#define BMP280_STAT_REG                                 (0xF3) /*Status Register */
#define BMP280_CTRL_MEAS_REG                            (0xF4) /*Ctrl Measure Register */
#define BMP280_CONFIG_REG                               (0xF5) /*Configuration Register */
#define BMP280_PRESSURE_MSB_REG                         (0xF7) /*Pressure MSB Register */
#define BMP280_PRESSURE_LSB_REG                         (0xF8) /*Pressure LSB Register */
#define BMP280_PRESSURE_XLSB_REG                        (0xF9) /*Pressure XLSB Register */
#define BMP280_TEMPERATURE_MSB_REG                      (0xFA) /*Temperature MSB Reg */
#define BMP280_TEMPERATURE_LSB_REG                      (0xFB) /*Temperature LSB Reg */
#define BMP280_TEMPERATURE_XLSB_REG                     (0xFC) /*Temperature XLSB Reg */

/************************************************/
/**\name	CALIBRATION PARAMETERS DEFINITION       */
/***********************************************/
/*calibration parameters */
#define BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG         (0x88)
#define BMP280_TEMPERATURE_CALIB_DIG_T1_MSB_REG         (0x89)
#define BMP280_TEMPERATURE_CALIB_DIG_T2_LSB_REG         (0x8A)
#define BMP280_TEMPERATURE_CALIB_DIG_T2_MSB_REG         (0x8B)
#define BMP280_TEMPERATURE_CALIB_DIG_T3_LSB_REG         (0x8C)
#define BMP280_TEMPERATURE_CALIB_DIG_T3_MSB_REG         (0x8D)
#define BMP280_PRESSURE_CALIB_DIG_P1_LSB_REG            (0x8E)
#define BMP280_PRESSURE_CALIB_DIG_P1_MSB_REG            (0x8F)
#define BMP280_PRESSURE_CALIB_DIG_P2_LSB_REG            (0x90)
#define BMP280_PRESSURE_CALIB_DIG_P2_MSB_REG            (0x91)
#define BMP280_PRESSURE_CALIB_DIG_P3_LSB_REG            (0x92)
#define BMP280_PRESSURE_CALIB_DIG_P3_MSB_REG            (0x93)
#define BMP280_PRESSURE_CALIB_DIG_P4_LSB_REG            (0x94)
#define BMP280_PRESSURE_CALIB_DIG_P4_MSB_REG            (0x95)
#define BMP280_PRESSURE_CALIB_DIG_P5_LSB_REG            (0x96)
#define BMP280_PRESSURE_CALIB_DIG_P5_MSB_REG            (0x97)
#define BMP280_PRESSURE_CALIB_DIG_P6_LSB_REG            (0x98)
#define BMP280_PRESSURE_CALIB_DIG_P6_MSB_REG            (0x99)
#define BMP280_PRESSURE_CALIB_DIG_P7_LSB_REG            (0x9A)
#define BMP280_PRESSURE_CALIB_DIG_P7_MSB_REG            (0x9B)
#define BMP280_PRESSURE_CALIB_DIG_P8_LSB_REG            (0x9C)
#define BMP280_PRESSURE_CALIB_DIG_P8_MSB_REG            (0x9D)
#define BMP280_PRESSURE_CALIB_DIG_P9_LSB_REG            (0x9E)
#define BMP280_PRESSURE_CALIB_DIG_P9_MSB_REG            (0x9F)

/************************************************/
/**\name	OVERSAMPLING DEFINITION       */
/***********************************************/
#define BMP280_OVERSAMP_SKIPPED                         (0x00)
#define BMP280_OVERSAMP_1X                              (0x01)
#define BMP280_OVERSAMP_2X                              (0x02)
#define BMP280_OVERSAMP_4X                              (0x03)
#define BMP280_OVERSAMP_8X                              (0x04)
#define BMP280_OVERSAMP_16X                             (0x05)

/************************************************/
/**\name	WORKING MODE DEFINITION       */
/***********************************************/
#define BMP280_ULTRA_LOW_POWER_MODE                     (0x00)
#define BMP280_LOW_POWER_MODE                           (0x01)
#define BMP280_STANDARD_RESOLUTION_MODE                 (0x02)
#define BMP280_HIGH_RESOLUTION_MODE                     (0x03)
#define BMP280_ULTRA_HIGH_RESOLUTION_MODE               (0x04)

#define BMP280_ULTRALOWPOWER_OVERSAMP_PRESSURE          BMP280_OVERSAMP_1X
#define BMP280_ULTRALOWPOWER_OVERSAMP_TEMPERATURE       BMP280_OVERSAMP_1X

#define BMP280_LOWPOWER_OVERSAMP_PRESSURE               BMP280_OVERSAMP_2X
#define BMP280_LOWPOWER_OVERSAMP_TEMPERATURE            BMP280_OVERSAMP_1X

#define BMP280_STANDARDRESOLUTION_OVERSAMP_PRESSURE     BMP280_OVERSAMP_4X
#define BMP280_STANDARDRESOLUTION_OVERSAMP_TEMPERATURE  BMP280_OVERSAMP_1X

#define BMP280_HIGHRESOLUTION_OVERSAMP_PRESSURE         BMP280_OVERSAMP_8X
#define BMP280_HIGHRESOLUTION_OVERSAMP_TEMPERATURE      BMP280_OVERSAMP_1X

#define BMP280_ULTRAHIGHRESOLUTION_OVERSAMP_PRESSURE    BMP280_OVERSAMP_16X
#define BMP280_ULTRAHIGHRESOLUTION_OVERSAMP_TEMPERATURE BMP280_OVERSAMP_2X

/**************************************************************/
/**\name	FUNCTION FOR RETURNING BMP ID REGISTER    */
/**************************************************************/
/*!
 * @brief Returns the sensor ID
 *
 * @return: The BMP device ID register value
 *
 * @retval:
 *
 *    value    | Description
 *  -----------|---------------
 *    0x55     | BMP180 in use
 *    0x58     | BMP280 in use
 *
 * @note Useful for checking communication
 */
uint8_t BMPX80_getId(void);

/**************************************************************/
/**\name	FUNCTION FOR INITIALISING TEMPERATURE MEASUREMENT */
/**************************************************************/
/**
 * @brief Initiate temperature measurement
 *
 * @note Need to wait 4.5ms before reading value
 *
 */
void BMPX80_startTempMeasurement(void);

/**************************************************************/
/**\name	FUNCTION FOR GETTING TEMPERATURE READING */
/**************************************************************/
/**
 * @brief Read temperature value from sensor
 *
 * @note BMP180: need to wait 4.5ms before reading value
 * @note BMP280: need to wait 5.5ms before reading value
 * @note BMP180 requests 2 bytes
 * @note BMP280 requests 3 bytes
 *
 * @retval 16 bit return value in big endian format
 *
 */
void BMPX80_getTemp(uint8_t *buf);

/**************************************************************/
/**\name	FUNCTION FOR INITIALISING PRESSURE MEASUREMENT */
/**************************************************************/
/*
 * @brief Initiate pressure measurement and set up BMP precision
 * (working mode) via the given oversampling (OSS) value.
 *
 *
 *	 ===================  BMP180  ====================
 *   if OSS = 0 need to wait 4.5ms before reading value
 *   if OSS = 1 need to wait 7.5ms before reading value
 *   if OSS = 2 need to wait 13.5ms before reading value
 *   if OSS = 3 need to wait 25.5ms before reading value
 *
 *
 *   ===================  BMP280  ====================
 *   if OSS = 0 need to wait 5.5ms before reading value
 *   if OSS = 1 need to wait 7.5ms before reading value
 *   if OSS = 2 need to wait 11.5ms before reading value
 *   if OSS = 3 need to wait 19.5ms before reading value
 *   if OSS = 4 need to wait 37.5ms before reading value
 *
 *   @note omit OSS = 1 mode for BMP280 as this is a new mode
 *   (Low Power) introduced for BMP280. Keep it in line with
 *   BMP180 code.
 *
 */

void BMPX80_startPressMeasurement(uint8_t oss);

/**************************************************************/
/**\name	FUNCTION FOR GETTING PRESSURE READING  */
/**************************************************************/
/**
 * @brief Read pressure value from sensor
 *
 *
 * @note BMP180: need to wait 4.5ms before reading value
 * @note BMP280: need to wait 5.5ms before reading value
 * @note BMP180/280 both request 3 bytes
 *
 * @retval 19 bit return value in *buf (big endian format)
 *
 */
void BMPX80_getPress(uint8_t *buf);

/**************************************************************/
/**\name	FUNCTION FOR READING CALIBRATION PARAMETERS  */
/**************************************************************/
/**
 * @brief This function reads the calibration parameters
 *
 * @param res: pointer to byte buffer (bmpX80Calib in main.c space)
 *
 * @note the number of bytes read is different for BMP180/280
 * @note BMP180 Big Endian order, 11x16-bit values returned in res
 *
 * @retval res
 */
void BMPX80_getCalibCoeff(uint8_t *res);

/**************************************************************/
/**\name	FUNCTION FOR WORK MODE   */
/**************************************************************/
/*!
 *	@brief This API is used to write
 *	 the working mode of the sensor
 *
 *
 *  @param v_work_mode_u8 : The value of work mode
 *   value      |  mode
 * -------------|-------------
 *    0         | BMP280_ULTRA_LOW_POWER_MODE
 *    1         | BMP280_LOW_POWER_MODE
 *    2         | BMP280_STANDARD_RESOLUTION_MODE
 *    3         | BMP280_HIGH_RESOLUTION_MODE
 *    4         | BMP280_ULTRA_HIGH_RESOLUTION_MODE
 *
 *
 */
void BMPX80_setWorkMode(uint8_t v_work_mode_u8);

/**************************************************************/
/**\name    FUNCTION FOR BMP DRIVER SETUP    */
/**************************************************************/
/*!
 * @brief Function for setting up which BMP is in use
 *
 * @param bmp_in_use : Toggles which BMP driver to use
 *
 *    value    | Description
 *  -----------|---------------
 *     1       | BMP180 in use
 *     2       | BMP280 in use
 *
 */
void setBmpInUse(uint8_t bmp_in_use);

uint8_t isBmp180InUse(void);
uint8_t isBmp280InUse(void);
void loadBmpCalibration(void);
uint8_t *get_bmp_calib_data_bytes(void);
uint8_t get_bmp_calib_data_bytes_len(void);

#endif //BMPX80_H
