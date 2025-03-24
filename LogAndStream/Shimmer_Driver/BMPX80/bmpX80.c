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

#include "bmpX80.h"

#include "msp430.h"
#include "../5xx_HAL/hal_I2C.h"

uint8_t bmpInUse;
static uint8_t bmpX80Address;
uint8_t bmpX80Calib[BMP280_CALIB_DATA_SIZE];

uint8_t BMPX80_getId(void) {
	uint8_t buf;
	I2C_Set_Slave_Address(bmpX80Address);

	buf = ID;
	I2C_Read_Packet_From_Sensor(&buf, 1);
	return buf;
}

void BMPX80_startTempMeasurement(void) {
	if (isBmp280InUse()) {
//		BMPX80_setWorkMode(2);
	} else if (isBmp180InUse()) {
		I2C_Set_Slave_Address(bmpX80Address);
		uint8_t buf[2];
		buf[0] = CTRL_MEAS;
		buf[1] = 0x2E;
		I2C_Write_Packet_To_Sensor(buf, 2);
	}
}

void BMPX80_getTemp(uint8_t *buf) {
	if (isBmp280InUse()) {
		*buf = BMP280_TEMPERATURE_MSB_REG;
	} else {
		*buf = BMP180_OUT_MSB;
	}
	I2C_Set_Slave_Address(bmpX80Address);
	I2C_Read_Packet_From_Sensor(buf, BMPX80_TEMP_BUFF_SIZE);
}

void BMPX80_startPressMeasurement(uint8_t oss) {

	if (isBmp280InUse()) {
		BMPX80_setWorkMode((oss == 0) ? 0 : oss + 1); // used to skip Low Power mode (1)
	} else if (isBmp180InUse()) {
		I2C_Set_Slave_Address(bmpX80Address);
		uint8_t buf[2];
		buf[0] = CTRL_MEAS;
		buf[1] = ((oss & 0x03) << 6) + 0x34;
		I2C_Write_Packet_To_Sensor(buf, 2);
	}

}

void BMPX80_getPress(uint8_t *buf) {
	if (isBmp280InUse()) {
		*buf = BMP280_PRESSURE_MSB_REG;
	} else {
		*buf = BMP180_OUT_MSB;
	}

	I2C_Set_Slave_Address(bmpX80Address);
	I2C_Read_Packet_From_Sensor(buf, BMPX80_PRESS_BUFF_SIZE);

}

void BMPX80_getCalibCoeff(uint8_t *res) {
	// array is 24 bytes long to accommodate bmp280 calibration either way
	I2C_Set_Slave_Address(bmpX80Address);
	if (isBmp280InUse()) {
		*res = BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG;
		I2C_Read_Packet_From_Sensor(res, BMP280_CALIB_DATA_SIZE);
	} else if (isBmp180InUse()) {
		*res = AC1_MSB;
		I2C_Read_Packet_From_Sensor(res, BMP180_CALIB_DATA_SIZE);
	}

}

void BMPX80_setWorkMode(uint8_t work_mode) {
	/* variable used to return communication result*/
	uint8_t buf[2] = { 0, 0 };
	uint8_t oversamp_temperature, oversamp_pressure;

	/* check the p_bmp280 struct pointer as NULL*/
	if (work_mode <= BMP280_ULTRA_HIGH_RESOLUTION_MODE) {
		switch (work_mode) {
		/* write work mode*/
		case BMP280_ULTRA_LOW_POWER_MODE:
			oversamp_temperature =
			BMP280_ULTRALOWPOWER_OVERSAMP_TEMPERATURE;
			oversamp_pressure =
			BMP280_ULTRALOWPOWER_OVERSAMP_PRESSURE;
			break;
		case BMP280_LOW_POWER_MODE:
			oversamp_temperature =
			BMP280_LOWPOWER_OVERSAMP_TEMPERATURE;
			oversamp_pressure =
			BMP280_LOWPOWER_OVERSAMP_PRESSURE;
			break;
		case BMP280_STANDARD_RESOLUTION_MODE:
			oversamp_temperature =
			BMP280_STANDARDRESOLUTION_OVERSAMP_TEMPERATURE;
			oversamp_pressure =
			BMP280_STANDARDRESOLUTION_OVERSAMP_PRESSURE;
			break;
		case BMP280_HIGH_RESOLUTION_MODE:
			oversamp_temperature =
			BMP280_HIGHRESOLUTION_OVERSAMP_TEMPERATURE;
			oversamp_pressure =
			BMP280_HIGHRESOLUTION_OVERSAMP_PRESSURE;
			break;
		case BMP280_ULTRA_HIGH_RESOLUTION_MODE:
			oversamp_temperature =
			BMP280_ULTRAHIGHRESOLUTION_OVERSAMP_TEMPERATURE;
			oversamp_pressure =
			BMP280_ULTRAHIGHRESOLUTION_OVERSAMP_PRESSURE;
			break;
		}
		buf[0] = BMP280_CTRL_MEAS_REG;
		buf[1] = (oversamp_temperature << 5) | (oversamp_pressure << 2)
				| BMP280_FORCED_MODE; //BMP280_NORMAL_MODE;
		// FORCE_MODE is "...similar to BMP180 operation"

		I2C_Set_Slave_Address(bmpX80Address);
		I2C_Write_Packet_To_Sensor(buf, 2);

	} else {
		return;
	}
}

void setBmpInUse(uint8_t bmp_in_use)
{
    bmpInUse = bmp_in_use;
    bmpX80Address = isBmp280InUse()? BMP280_ADDR:BMP180_ADDR;
}

uint8_t isBmp180InUse(void)
{
    return bmpInUse == BMP180_IN_USE;
}

uint8_t isBmp280InUse(void)
{
    return bmpInUse == BMP280_IN_USE;
}

void loadBmpCalibration(void)
{
    memset(bmpX80Calib, 0, sizeof(bmpX80Calib));
    BMPX80_getCalibCoeff(bmpX80Calib);
}

uint8_t *get_bmp_calib_data_bytes(void)
{
    return &bmpX80Calib[0];
}

uint8_t get_bmp_calib_data_bytes_len(void)
{
    if (isBmp180InUse())
    {
        return BMP180_CALIB_DATA_SIZE;
    }
    else if (isBmp280InUse())
    {
        return BMP280_CALIB_DATA_SIZE;
    }
    else
    {
        return 0;
    }
}
