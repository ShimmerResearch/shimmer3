/*
 * shimmer_calibration.c
 *
 *  Created on: Jul 11, 2016
 *      Author: WeiboP
 *
 *  Edited on: Jul 26, 2017
 *      Author: Sam OM
 */

//#include <stdio.h>
//#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "msp430.h"

#include "shimmer_calibration.h"
#include "../5xx_HAL/hal_RTC.h"
#include "../5xx_HAL/hal_Board.h"
#include "../FatFs/ff.h"
#include "../../shimmer_btsd.h"
#include "../LSM303DLHC/lsm303dlhc.h"
#include "../LSM303AHTR/lsm303ahtr.h"
#include "../BMPX80/bmpX80.h"
#include "../Bluetooth_SD/RN4X.h"
#include "../shimmer_boards/shimmer_boards.h"

uint8_t shimmerCalib_ram[SHIMMER_CALIB_RAM_MAX], shimmerCalib_macId[5],
        shimmerCalib_ramTemp[SHIMMER_CALIB_RAM_MAX];
uint16_t shimmerCalib_ramLen, shimmerCalib_ramTempLen, shimmerCalib_ramTempMax;

uint8_t ShimmerCalib_findLength(sc_t* sc1)
{
    switch (sc1->id)
    {
    case SC_SENSOR_ANALOG_ACCEL:
        return SC_DATA_LEN_ANALOG_ACCEL;
    case SC_SENSOR_MPU9150_GYRO:
        return SC_DATA_LEN_MPU9250_GYRO;
    case SC_SENSOR_LSM303DLHC_ACCEL:
        return SC_DATA_LEN_LSM303DLHC_ACCEL;
    case SC_SENSOR_LSM303DLHC_MAG:
        return SC_DATA_LEN_LSM303DLHC_MAG;
    case SC_SENSOR_BMP180_PRESSURE:
        return SC_DATA_LEN_BMP180;
    default:
        return 0;
    }
}

void ShimmerCalib_initVer(void)
{
    shimmerCalib_ram[SC_OFFSET_VER_HW_ID_L] = DEVICE_VER & 0xff;
    shimmerCalib_ram[SC_OFFSET_VER_HW_ID_H] = (DEVICE_VER >> 8) & 0xff;
    shimmerCalib_ram[SC_OFFSET_VER_FW_ID_L] = FW_IDENTIFIER & 0xff;
    shimmerCalib_ram[SC_OFFSET_VER_FW_ID_H] = (FW_IDENTIFIER >> 8) & 0xff;
    shimmerCalib_ram[SC_OFFSET_VER_FW_MAJOR_L] = FW_VER_MAJOR & 0xff;
    shimmerCalib_ram[SC_OFFSET_VER_FW_MAJOR_H] = (FW_VER_MAJOR >> 8) & 0xff;
    shimmerCalib_ram[SC_OFFSET_VER_FW_MINOR_L] = FW_VER_MINOR & 0xff;
    shimmerCalib_ram[SC_OFFSET_VER_FW_INTER_L] = FW_VER_REL & 0xff;
}

void ShimmerCalib_init(void)
{
    shimmerCalib_ramLen = 8;

    memcpy(shimmerCalib_macId, getMacIdStrPtr() + 8, 4);
    shimmerCalib_macId[4] = 0;

    memset(shimmerCalib_ram, 0, SHIMMER_CALIB_RAM_MAX);
    shimmerCalib_ram[SC_OFFSET_LENGTH_L] = shimmerCalib_ramLen & 0xff;
    shimmerCalib_ram[SC_OFFSET_LENGTH_H] = (shimmerCalib_ramLen >> 8) & 0xff;
    ShimmerCalib_initVer();

    ShimmerCalib_default(SC_SENSOR_ANALOG_ACCEL);
    ShimmerCalib_default(SC_SENSOR_MPU9150_GYRO);
    ShimmerCalib_default(SC_SENSOR_LSM303DLHC_ACCEL);
    ShimmerCalib_default(SC_SENSOR_LSM303DLHC_MAG);
}

void ShimmerCalib_ram2File(void)
{

    char cal_file_name[48]; //buffer[66],
    DIRS gdc;
    FIL gfc;
    //sc_t sc1;
    UINT bw;
    FRESULT res;
    uint8_t this_write_size;

    shimmerCalib_ramLen = min(*(uint16_t* )shimmerCalib_ram,
                              SHIMMER_CALIB_RAM_MAX-2);

    if (shimmerCalib_ramLen > 0)
    {
        strcpy((char*) cal_file_name, "/Calibration");
        if (res = f_opendir(&gdc, "/Calibration"))
        {
            if (res = f_opendir(&gdc, "/calibration"))
            {
                if (res == FR_NO_PATH)
                {   // we'll have to make /Calibration first
                    res = f_mkdir("/Calibration");
                }
            }
            else
            {
                strcpy((char*) cal_file_name, "/calibration");
            }
        }
        strcat(cal_file_name, "/calib_");
        strcat(cal_file_name, (char*) shimmerCalib_macId);

        res = f_open(&gfc, cal_file_name, (FA_WRITE | FA_CREATE_ALWAYS));
        if (res == FR_NO_FILE)
        {
            return;
        }

        while (gfc.fptr < shimmerCalib_ramLen + 2)
        {
            this_write_size = min(shimmerCalib_ramLen + 2 - gfc.fptr,
                                  SHIMMER_CALIB_COPY_SIZE);
            f_write(&gfc, shimmerCalib_ram + gfc.fptr, this_write_size, &bw);
        }

        f_close(&gfc);
        _delay_cycles(1200000);   //50ms
    }
}

/*
 * if there wasn't a calibration/calibration file, new file won't be created.
 * ram buffer will use 0 as length, 00000 as content
 */
uint8_t ShimmerCalib_file2Ram(void)
{
    char cal_file_name[48];
    DIRS gdc;
    FIL gfc;
    UINT bw;
    FRESULT res;
    uint8_t this_read_size;
    uint16_t offset = 0;

    strcpy(cal_file_name, "/Calibration");   // "/Calibration/calibParams"
    if (f_opendir(&gdc, "/Calibration"))
    {
        if (f_opendir(&gdc, "/calibration"))
        {
            //CalibDefault(sensor);
            return 1;
        }
        else
            strcpy(cal_file_name, "/calibration");
    }
    strcat(cal_file_name, "/calib_");
    strcat(cal_file_name, (char*) shimmerCalib_macId);

    res = f_open(&gfc, cal_file_name, (FA_OPEN_EXISTING | FA_READ));
    if (res != FR_OK)
    {
        return 1;
    }
    if (gfc.fptr == gfc.fsize)
    {
        return 1;
    }

    //if file not successfully open, don't wipe the previous dump RAM
    shimmerCalib_ramLen = 0;
    memset(shimmerCalib_ram, 0, SHIMMER_CALIB_RAM_MAX);

    while (gfc.fptr != gfc.fsize)
    {
        this_read_size = min(gfc.fsize - gfc.fptr, SHIMMER_CALIB_COPY_SIZE);
        res = f_read(&gfc, shimmerCalib_ram + offset, this_read_size, &bw);
        offset += this_read_size;
    }
    shimmerCalib_ramLen = min(*(uint16_t* )shimmerCalib_ram,
                              SHIMMER_CALIB_RAM_MAX-2);
    f_close(&gfc);
    _delay_cycles(1200000);   //50ms
    return 0;
}

uint8_t ShimmerCalib_singleSensorWrite(const sc_t* sc1)
{
    sc_t curr_sc;
    uint16_t cnt = SC_OFFSET_FIRST_SENSOR;
    uint8_t ts[8], sensor_found = 0;   //temp_len,

    *(uint64_t*) ts = getRwcTime();
    shimmerCalib_ramLen = min(*(uint16_t* )shimmerCalib_ram,
                              SHIMMER_CALIB_RAM_MAX-2);

    while (cnt + SC_OFFSET_SENSOR_DATA < shimmerCalib_ramLen + 2)
    {
        memcpy((uint8_t*) &curr_sc, shimmerCalib_ram + cnt,
        SC_OFFSET_SENSOR_TIMESTAMP);
        if ((curr_sc.id == sc1->id) && (curr_sc.range == sc1->range))
        {
            //setup RTC time as calib timestamp
            memcpy(shimmerCalib_ram + cnt + SC_OFFSET_SENSOR_TIMESTAMP, ts,
            SC_TIMESTAMP_LENGTH);
            memcpy(shimmerCalib_ram + cnt + SC_OFFSET_SENSOR_DATA,
                   sc1->data.raw, curr_sc.data_len);
            sensor_found = 1;
            break;
        }
        cnt += SC_OFFSET_SENSOR_DATA + curr_sc.data_len;
    }

    if (!sensor_found)
    {
        memcpy(shimmerCalib_ram + shimmerCalib_ramLen + 2, (uint8_t*) sc1,
        SC_OFFSET_SENSOR_DATA + sc1->data_len);
        shimmerCalib_ramLen += SC_OFFSET_SENSOR_DATA + sc1->data_len;
        *(uint16_t*) shimmerCalib_ram = min(shimmerCalib_ramLen,
                                            SHIMMER_CALIB_RAM_MAX-2);
    }
    return 0;
}

uint8_t ShimmerCalib_singleSensorRead(sc_t* sc1)
{
    sc_t curr_sc;
    uint16_t cnt = SC_OFFSET_FIRST_SENSOR;
    uint8_t sensor_found = 0;

    memset((uint8_t*) sc1->data.raw, 0, sc1->data_len);
    shimmerCalib_ramLen = min(*(uint16_t* )shimmerCalib_ram,
                              SHIMMER_CALIB_RAM_MAX-2);
    while (cnt < shimmerCalib_ramLen + 2)
    {
        memcpy((uint8_t*) &curr_sc, shimmerCalib_ram + cnt,
        SC_OFFSET_SENSOR_TIMESTAMP);
        if ((curr_sc.id == sc1->id) && (curr_sc.range == sc1->range))
        {
            memcpy((uint8_t*) sc1, shimmerCalib_ram + cnt,
            SC_OFFSET_SENSOR_DATA + curr_sc.data_len);
            sensor_found = 1;
            break;
        }
        cnt += SC_OFFSET_SENSOR_DATA + curr_sc.data_len;
    }
    if (!sensor_found)
    {
        return 1;
    }
    return 0;
}

void ShimmerCalib_checkRamLen()
{
    shimmerCalib_ramLen = min(*(uint16_t* )shimmerCalib_ram,
                              SHIMMER_CALIB_RAM_MAX-2);
    memset(shimmerCalib_ram + shimmerCalib_ramLen + 2, 0,
    SHIMMER_CALIB_RAM_MAX - shimmerCalib_ramLen - 2);
}

void ShimmerCalib_ramTempInit()
{
    shimmerCalib_ramTempMax = 0;
    shimmerCalib_ramTempLen = 0;
}

// return 0 : success; 0xff: fail
// the calib dump write operation starts with offset = 0, ends with shimmerCalib_ramTempMax bytes received.
uint8_t ShimmerCalib_ramWrite(const uint8_t* buf, uint8_t length,
                              uint16_t offset)
{
    if ((length <= 128) && (offset <= (SHIMMER_CALIB_RAM_MAX - 1))
            && (length + offset <= SHIMMER_CALIB_RAM_MAX))
    {
        //memcpy(shimmerCalib_ram+offset, buf, length);
        memcpy(shimmerCalib_ramTemp + offset, buf, length);
    }
    else
    {
        return 0xff;
    }

    // to start a new calib_dump transmission, the sw must use offset = 0 to setup the correct length.
    // offset = 1 is not suggested, but will be considered.
    // starting with offset > 2 is not accepted.
    if (offset < 2)
    {
        // + 2 as length bytes in header are not included in the overall array length
        shimmerCalib_ramTempMax = (*(uint16_t*) shimmerCalib_ramTemp) + 2;
        shimmerCalib_ramTempLen = length;
    }
    else
    {
        shimmerCalib_ramTempLen += length;
    }

    if (shimmerCalib_ramTempLen >= shimmerCalib_ramTempMax)
    {
        memcpy(shimmerCalib_ram, shimmerCalib_ramTemp, shimmerCalib_ramTempMax);
        ShimmerCalib_initVer();
        ShimmerCalib_ramTempInit();
        return 1;
    }

    return 0;
}

// return 0 : success; 1: fail
uint8_t ShimmerCalib_ramRead(uint8_t* buf, uint8_t length, uint16_t offset)
{
    shimmerCalib_ramLen = min(*(uint16_t* )shimmerCalib_ram,
                              SHIMMER_CALIB_RAM_MAX-2);
    if ((length <= 128) && (offset <= (SHIMMER_CALIB_RAM_MAX - 1))
            && (length + offset <= SHIMMER_CALIB_RAM_MAX))
    {
        memcpy(buf, shimmerCalib_ram + offset, length);
        return 0;
    }
    else
    {
        memset(buf, 0, length);
        return 1;
    }
}

void ShimmerCalib_default(uint8_t sensor)
{
//   int16_t bias, sensitivity, sensitivity_x, sensitivity_y, sensitivity_z;
//   uint8_t bias_byte_one, bias_byte_two, sens_byte_one, sens_byte_two, number_axes = 1;
//   int8_t align_xx, align_xy, align_xz, align_yx, align_yy, align_yz, align_zx, align_zy, align_zz, i = 0;
//   uint16_t address;
//   bool align = FALSE;

    sc_t sc1;
    //uint8_t ts[8];//range, , data_ptr
    uint16_t bias, sensitivity;

    //*(uint64_t*)(sc1.ts) = rwcTimeDiff64 + RTC_get64();
    memset(sc1.ts, 0, 8);

    if (sensor == SC_SENSOR_ANALOG_ACCEL)
    {
        sc1.id = sensor;
        sc1.data_len = SC_DATA_LEN_ANALOG_ACCEL;
        for (sc1.range = 0; sc1.range < SC_SENSOR_RANGE_MAX_ANALOG_ACCEL;
                sc1.range++)
        {
            if (isLnAccelKxtc9_2050Present())
            {
                bias = 2253;
                sensitivity = 92;
            }
            else
            {
                bias = 2047;
                sensitivity = 83;
            }
            bias = (((bias & 0x00ff) << 8) | ((bias & 0xff00) >> 8));
            sensitivity = (((sensitivity & 0x00ff) << 8)
                    | ((sensitivity & 0xff00) >> 8));
            sc1.data.dd.bias_x = bias;
            sc1.data.dd.bias_y = bias;
            sc1.data.dd.bias_z = bias;
            sc1.data.dd.sens_x = sensitivity;
            sc1.data.dd.sens_y = sensitivity;
            sc1.data.dd.sens_z = sensitivity;
            sc1.data.dd.align_xx = 0;
            sc1.data.dd.align_xy = -100;
            sc1.data.dd.align_xz = 0;
            sc1.data.dd.align_yx = -100;
            sc1.data.dd.align_yy = 0;
            sc1.data.dd.align_yz = 0;
            sc1.data.dd.align_zx = 0;
            sc1.data.dd.align_zy = 0;
            sc1.data.dd.align_zz = -100;
            ShimmerCalib_singleSensorWrite(&sc1);
        }
    }
    else if (sensor == SC_SENSOR_MPU9150_GYRO)
    {
        sc1.id = sensor;
        sc1.data_len = SC_DATA_LEN_MPU9250_GYRO;
        for (sc1.range = 0; sc1.range < SC_SENSOR_RANGE_MAX_MPU9250_GYRO;
                sc1.range++)
        {
            bias = 0;
            if (sc1.range == SC_SENSOR_RANGE_MPU9250_GYRO_250DPS)
            {
                sensitivity = 13100;
            }
            else if (sc1.range == SC_SENSOR_RANGE_MPU9250_GYRO_500DPS)
            {
                sensitivity = 6550;
            }
            else if (sc1.range == SC_SENSOR_RANGE_MPU9250_GYRO_1000DPS)
            {
                sensitivity = 3280;
            }
            else
            { //(sdHeadText[SDH_GYRO_RANGE] == SC_SENSOR_RANGE_MPU9250_GYRO_2000DPS)
                sensitivity = 1640;
            }
            bias = (((bias & 0x00ff) << 8) | ((bias & 0xff00) >> 8));
            sensitivity = (((sensitivity & 0x00ff) << 8)
                    | ((sensitivity & 0xff00) >> 8));
            sc1.data.dd.bias_x = bias;
            sc1.data.dd.bias_y = bias;
            sc1.data.dd.bias_z = bias;
            sc1.data.dd.sens_x = sensitivity;
            sc1.data.dd.sens_y = sensitivity;
            sc1.data.dd.sens_z = sensitivity;
            sc1.data.dd.align_xx = 0;
            sc1.data.dd.align_xy = -100;
            sc1.data.dd.align_xz = 0;
            sc1.data.dd.align_yx = -100;
            sc1.data.dd.align_yy = 0;
            sc1.data.dd.align_yz = 0;
            sc1.data.dd.align_zx = 0;
            sc1.data.dd.align_zy = 0;
            sc1.data.dd.align_zz = -100;
            ShimmerCalib_singleSensorWrite(&sc1);
        }
    }
    else if (sensor == SC_SENSOR_LSM303DLHC_ACCEL)
    {
        sc1.id = sensor;
        sc1.data_len = SC_DATA_LEN_LSM303DLHC_ACCEL;
        for (sc1.range = 0; sc1.range < SC_SENSOR_RANGE_MAX_LSM303DLHC_ACCEL;
                sc1.range++)
        {
            bias = 0;
            if (sc1.range == SC_SENSOR_RANGE_LSM303DLHC_ACCEL_2G)
            {
                sensitivity = isWrAccelInUseLsm303dlhc() ? 1631 : 1671;
            }
            else if (sc1.range == SC_SENSOR_RANGE_LSM303DLHC_ACCEL_4G)
            {
                sensitivity = isWrAccelInUseLsm303dlhc() ? 815 : 836;
            }
            else if (sc1.range == SC_SENSOR_RANGE_LSM303DLHC_ACCEL_8G)
            {
                sensitivity = isWrAccelInUseLsm303dlhc() ? 408 : 418;
            }
            else
            { //(sc1.range == SC_SENSOR_RANGE_LSM303DLHC_ACCEL_16G)
                sensitivity = isWrAccelInUseLsm303dlhc() ? 135 : 209;
            }
            bias = (((bias & 0x00ff) << 8) | ((bias & 0xff00) >> 8));
            sensitivity = (((sensitivity & 0x00ff) << 8)
                    | ((sensitivity & 0xff00) >> 8));
            sc1.data.dd.bias_x = bias;
            sc1.data.dd.bias_y = bias;
            sc1.data.dd.bias_z = bias;
            sc1.data.dd.sens_x = sensitivity;
            sc1.data.dd.sens_y = sensitivity;
            sc1.data.dd.sens_z = sensitivity;
            sc1.data.dd.align_xx = isWrAccelInUseLsm303dlhc() ? (-100) : 0;
            sc1.data.dd.align_xy = isWrAccelInUseLsm303dlhc() ? 0 : -100;
            sc1.data.dd.align_xz = 0;
            sc1.data.dd.align_yx = isWrAccelInUseLsm303dlhc() ? 0 : 100;
            sc1.data.dd.align_yy = isWrAccelInUseLsm303dlhc() ? 100 : 0;
            sc1.data.dd.align_yz = 0;
            sc1.data.dd.align_zx = 0;
            sc1.data.dd.align_zy = 0;
            sc1.data.dd.align_zz = -100;
            ShimmerCalib_singleSensorWrite(&sc1);
        }
    }
    else if (sensor == SC_SENSOR_LSM303DLHC_MAG)
    {
        sc1.id = sensor;
        sc1.data_len = SC_DATA_LEN_LSM303DLHC_MAG;
        sc1.range = 0;
        if (isWrAccelInUseLsm303dlhc())
        {
            for (sc1.range = 0; sc1.range < SC_SENSOR_RANGE_MAX_LSM303DLHC_MAG;
                    sc1.range++)
            {
                bias = 0;

                if (sc1.range == SC_SENSOR_RANGE_LSM303DLHC_MAG_1_3G)
                {
                    sc1.data.dd.sens_x = 1100;
                    sc1.data.dd.sens_y = 1100;
                    sc1.data.dd.sens_z = 980;
                }
                else if (sc1.range == SC_SENSOR_RANGE_LSM303DLHC_MAG_1_9G)
                {
                    sc1.data.dd.sens_x = 855;
                    sc1.data.dd.sens_y = 855;
                    sc1.data.dd.sens_z = 760;
                }
                else if (sc1.range == SC_SENSOR_RANGE_LSM303DLHC_MAG_2_5G)
                {
                    sc1.data.dd.sens_x = 670;
                    sc1.data.dd.sens_y = 670;
                    sc1.data.dd.sens_z = 600;
                }
                else if (sc1.range == SC_SENSOR_RANGE_LSM303DLHC_MAG_4_0G)
                {
                    sc1.data.dd.sens_x = 450;
                    sc1.data.dd.sens_y = 450;
                    sc1.data.dd.sens_z = 400;
                }
                else if (sc1.range == SC_SENSOR_RANGE_LSM303DLHC_MAG_4_7G)
                {
                    sc1.data.dd.sens_x = 400;
                    sc1.data.dd.sens_y = 400;
                    sc1.data.dd.sens_z = 355;
                }
                else if (sc1.range == SC_SENSOR_RANGE_LSM303DLHC_MAG_5_6G)
                {
                    sc1.data.dd.sens_x = 330;
                    sc1.data.dd.sens_y = 330;
                    sc1.data.dd.sens_z = 295;
                }
                else
                { //sc1.range == SC_SENSOR_RANGE_LSM303DLHC_MAG_8_1G)
                    sc1.data.dd.sens_x = 230;
                    sc1.data.dd.sens_y = 230;
                    sc1.data.dd.sens_z = 205;
                }
            }
        }
        else
        {
            sc1.data.dd.sens_x = 667;
            sc1.data.dd.sens_y = 667;
            sc1.data.dd.sens_z = 667;
        }

        uint16_t tmpSens;
        tmpSens = sc1.data.dd.sens_x;
        sc1.data.dd.sens_x = (((tmpSens & 0x00ff) << 8)
                | ((tmpSens & 0xff00) >> 8));
        tmpSens = sc1.data.dd.sens_y;
        sc1.data.dd.sens_y = (((tmpSens & 0x00ff) << 8)
                | ((tmpSens & 0xff00) >> 8));
        tmpSens = sc1.data.dd.sens_z;
        sc1.data.dd.sens_z = (((tmpSens & 0x00ff) << 8)
                | ((tmpSens & 0xff00) >> 8));

        bias = (((bias & 0x00ff) << 8) | ((bias & 0xff00) >> 8));
        sensitivity = (((sensitivity & 0x00ff) << 8)
                | ((sensitivity & 0xff00) >> 8));

        sc1.data.dd.bias_x = bias;
        sc1.data.dd.bias_y = bias;
        sc1.data.dd.bias_z = bias;
        sc1.data.dd.align_xx = isWrAccelInUseLsm303dlhc() ? (-100) : 0;
        sc1.data.dd.align_xy = isWrAccelInUseLsm303dlhc() ? 0 : -100;
        sc1.data.dd.align_xz = 0;
        sc1.data.dd.align_yx = isWrAccelInUseLsm303dlhc() ? 0 : 100;
        sc1.data.dd.align_yy = isWrAccelInUseLsm303dlhc() ? 100 : 0;
        sc1.data.dd.align_yz = 0;
        sc1.data.dd.align_zx = 0;
        sc1.data.dd.align_zy = 0;
        sc1.data.dd.align_zz = -100;
        ShimmerCalib_singleSensorWrite(&sc1);
    }
}

void ShimmerCalib_singleSensorWriteFromInfoMem(uint16_t id, uint8_t range, uint8_t data_len, uint8_t *ptr)
{
    sc_t sc1;
    sc1.id = id;
    sc1.range = range;
    sc1.data_len = data_len;
    *(uint64_t*) (sc1.ts) = getRwcTime();
    memcpy(sc1.data.raw, ptr, sc1.data_len);
    ShimmerCalib_singleSensorWrite(&sc1);
}
