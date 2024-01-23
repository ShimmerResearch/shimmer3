/*
 * Copyright (c) 2022, Shimmer Research, Ltd.
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
 * @author Prakash Rajiah
 * @date January , 2021
 *
 * @author Sam O'Mahony
 * @date August, 2021
 *
 * @author Mark Nolan
 * @date December, 2021
 */

#include "msp430.h"
#include "../5xx_HAL/hal_I2C.h"
#include "ICM20948.h"
#include "math.h"

uint8_t magSampleSkipEnabled;
uint32_t lastMagSampleTsTicks;

void ICM20948_bankSelect(uint8_t addr, uint8_t val)
{
    uint8_t selectBank[2];
    selectBank[0] = addr;
    selectBank[1] = val;    //select user bank
    I2C_Write_Packet_To_Sensor(selectBank, 2);
}

//configure I2C
void ICM20948_init(void)
{
    uint8_t buf[2];

    P8OUT |= BIT4;             //set SW_I2C high to power on all I2C chips
    __delay_cycles(24000000);  //wait 1s (assuming 24MHz MCLK) to allow for power ramp up

    I2C_Master_Init(S_MCLK, 24000000, 400000); //Source from SMCLK, which is running @ 24MHz. 4kHz desired BRCLK
                                               //which is max for this part

    //put in I2C pass through mode so that mag can be accessed
    I2C_Set_Slave_Address(ICM20948_ADDR);

    ICM20948_bankSelect(USER_BANK_SEL, USER_BANK_0);   //select user bank 0

    buf[0] = ICM_USER_CTRL;
    buf[1] = 0x00; //Default value is 0 anyway but just to ensure I2C_MST_EN is 0
    I2C_Write_Packet_To_Sensor(buf, 2);

    buf[0] = ICM_INT_PIN_CFG;
    buf[1] = 0x02; //set BYPASS_EN to 1
    I2C_Write_Packet_To_Sensor(buf, 2);
}

uint8_t ICM20948_getId(void)
{
    uint8_t buf;

    I2C_Set_Slave_Address(ICM20948_ADDR);

    ICM20948_bankSelect(USER_BANK_SEL, USER_BANK_0); //select user bank 0

    buf = ICM20948_WHO_AM_I_REG;
    I2C_Read_Packet_From_Sensor(&buf, 1);
    return buf;
}

void ICM20948_wake(uint8_t wakeup)
{
    uint8_t buf[2];
    I2C_Set_Slave_Address(ICM20948_ADDR);

    ICM20948_bankSelect(USER_BANK_SEL, USER_BANK_0); //select user bank 0

    buf[0] = ICM20948_PWR_MGMT_1;
    I2C_Read_Packet_From_Sensor(buf, 1); // buf[0] now contains sensors register values

    if(wakeup)
    {
       //wakeup - De-assert bit 6, SLEEP
       buf[1] = buf[0] & 0xBF;
    } else
    {
       //go back to sleep - Assert bit 6, SLEEP
       buf[1] = buf[0] | 0x40;
    }

    buf[1] |= 0x08; //Disable temperature sensor (to possibly save power)

    buf[0] = ICM20948_PWR_MGMT_1;
    I2C_Write_Packet_To_Sensor(buf, 2);
}

void ICM20948_getGyro(uint8_t *buf)
{
    I2C_Set_Slave_Address(ICM20948_ADDR);

    ICM20948_bankSelect(USER_BANK_SEL, USER_BANK_0); //select user bank 0

    *buf = ICM_GYRO_XOUT_H;
    I2C_Read_Packet_From_Sensor(buf, 6);
}

void ICM20948_getAccelAndGyro(uint8_t *buf)
{
    I2C_Set_Slave_Address(ICM20948_ADDR);

    ICM20948_bankSelect(USER_BANK_SEL, USER_BANK_0); //select user bank 0

    *buf = ICM_ACCEL_XOUT_H;
    I2C_Read_Packet_From_Sensor(buf, 12);
}

void ICM20948_getAccel(uint8_t *buf)
{
    I2C_Set_Slave_Address(ICM20948_ADDR);

    ICM20948_bankSelect(USER_BANK_SEL, USER_BANK_0); //select user bank 0

    *buf = ICM_ACCEL_XOUT_H;
    I2C_Read_Packet_From_Sensor(buf, 6);
}

void ICM20948_setGyroSensitivity(uint8_t val)
{
    uint8_t buf[2];

    I2C_Set_Slave_Address(ICM20948_ADDR);
    ICM20948_bankSelect(USER_BANK_SEL, USER_BANK_2); //select user bank 2
    buf[0] = GYRO_CONFIG_1;
    I2C_Read_Packet_From_Sensor(buf, 1);

    //Mask out the GYRO_FS_SEL bits and replace
    buf[1] = (buf[0] & 0xF9) | ((val & 0x03) << 1);
    buf[0] = GYRO_CONFIG_1;
    I2C_Write_Packet_To_Sensor(buf, 2);
}

void ICM20948_setAccelRange(uint8_t val)
{
    uint8_t buf[2];

    I2C_Set_Slave_Address(ICM20948_ADDR);
    ICM20948_bankSelect(USER_BANK_SEL, USER_BANK_2); //select user bank 2
    buf[0] = ICM_ACCEL_CONFIG;
    I2C_Read_Packet_From_Sensor(buf, 1);

    //Mask out the ACCEL_FS_SEL bits and replace
    buf[1] = (buf[0] & 0xF9) | ((val & 0x03) << 1);
    buf[0] = ICM_ACCEL_CONFIG;
    I2C_Write_Packet_To_Sensor(buf, 2);
}

uint8_t ICM20948_convertSampleRateDivFromMPU9X50(uint8_t mpuSampleRateDivider,
                                                 uint8_t lpfState)
{
    uint8_t sampleRateDiv;
    uint16_t internalSamplingClockFreq;
    float mpuSamplingFreq;

    /* MPU9250:
     * SAMPLE_RATE= Internal_Sample_Rate / (1 + SMPLRT_DIV)
     * (Internal_Sample_Rate = 8kHz when the DLPF (Digital Low-pass filter) is
     * disabled (DLPF_CFG = 0 or 7), and 1kHz when the DLPF is enabled).
     */

    internalSamplingClockFreq = (lpfState == 0) ? 8000U : 1000U;
    mpuSamplingFreq = (float) (internalSamplingClockFreq
            / (1.0f + mpuSampleRateDivider)); // example: @51.2Hz, rate=8000/(1+155)

    /* ICM-20948:
     * 1.125 kHz / (1+GYRO_SMPLRT_DIV[7:0])
     */

    // ensure we don't divide by zero
    if (mpuSamplingFreq == 0.0f)
    {
        mpuSamplingFreq = 1.0f; // Set to 1 Hz
    }

    /*
     * ICM-20948 sample rate divider conversion from calculated sampleRate in Hz.
     * We floor the output value of sampleRateDiv such that:
     * (1125 / sampleRateDiv) >= mpuSamplingFreq
     */

    sampleRateDiv = (uint8_t) (floorf((float) (1125.0f / mpuSamplingFreq)));

    return sampleRateDiv;
}

void ICM20948_setGyroSamplingRate(uint8_t sampleRateDiv)
{
    uint8_t buf[2];

    I2C_Set_Slave_Address(ICM20948_ADDR);
    ICM20948_bankSelect(USER_BANK_SEL, USER_BANK_2); //select user bank 2
    buf[0] = ICM20948_GYRO_SMPLRT_DIV;
    buf[1] = sampleRateDiv;

    I2C_Write_Packet_To_Sensor(buf, 2);
}
void ICM20948_setAccelSamplingRate(uint16_t sampleRateDiv)
{
    uint8_t buf[2];

    I2C_Set_Slave_Address(ICM20948_ADDR);

    ICM20948_bankSelect(USER_BANK_SEL, USER_BANK_2); //select user bank 2

    buf[0] = ICM20948_ACCEL_SMPLRT_DIV_1;
    buf[1] = (uint8_t) ((sampleRateDiv >> 8) & 0x0F);
    I2C_Write_Packet_To_Sensor(buf, 2);

    buf[0] = ICM20948_ACCEL_SMPLRT_DIV_2;
    buf[1] = (uint8_t) (sampleRateDiv & 0xFF);
    I2C_Write_Packet_To_Sensor(buf, 2);
}

uint8_t ICM20948_getMagId(void)
{
    uint8_t buf;
    I2C_Set_Slave_Address(AK09916_MAG_ADDR);
    buf = WIA2;
    I2C_Read_Packet_From_Sensor(&buf, 1);

    // Need to check that this returns 0x09
    return buf;
}

void ICM20948_setMagSamplingRateFromShimmerRate(uint16_t samplingRateTicks)
{
    magSampleSkipEnabled = 0;
    lastMagSampleTsTicks = 0;

    AK09916_opMode opMode = AK09916_CONT_MODE_100HZ;
    /* Choose the next highest sampling rate to the if Shimmer's sampling rate
     * (e.g., if Shimmer's sampling rate is 9Hz, choose 10Hz). Note, choosing
     * '>=' here as the comparison values are all floored. */
    if (samplingRateTicks >= 3277) // ceil(32768/10Hz) = 3277. i.e., if <= 9.9994 Hz
    {
        opMode = AK09916_CONT_MODE_10HZ;
    }
    else if (samplingRateTicks >= 1639) // ceil(32768/20Hz) = 1639. i.e., if <= 19.9927 Hz
    {
        opMode = AK09916_CONT_MODE_20HZ;
    }
    else if (samplingRateTicks >= 656) // ceil(32768/50Hz) = 656. i.e., if <= 49.9512 Hz
    {
        opMode = AK09916_CONT_MODE_50HZ;
    }
    else
    {
        opMode = AK09916_CONT_MODE_100HZ;
        /* If Shimmer sampling rate is >100Hz, enable sample skip feature to
         * avoid locking up AK09916 Magnetometer - as we see regularly. */
        if (samplingRateTicks < SAMPLING_TIMER_TICKS_100Hz) // ceil(32768/100Hz) = 328. i.e., if > 99.9024 Hz
        {
            magSampleSkipEnabled = 1;
        }
    }

    ICM20948_setMagMode(opMode);
}

void ICM20948_setMagMode(AK09916_opMode opMode)
{
    uint8_t buf[2];

    I2C_Set_Slave_Address(AK09916_MAG_ADDR);
    buf[0] = CNTL2;
    buf[1] = opMode;

    I2C_Write_Packet_To_Sensor(buf, 2);
}

uint8_t ICM20948_isMagDataRdy(void)
{
    uint8_t status;

    I2C_Set_Slave_Address(AK09916_MAG_ADDR);

    //check status register
    status = ICM_ST1;
    I2C_Read_Packet_From_Sensor(&status, 1);
    return (status&0x01);
}

//put x, y and z mag values in buf (little endian)
//-32752  to 32752
//if values are 32767 they are not valid
//either due to data read error or magnetic sensor overflow
void ICM20948_getMag(uint8_t *buf)
{
    uint8_t status;

    I2C_Set_Slave_Address(AK09916_MAG_ADDR);
    *buf = ICM_MAG_XOUT_L;
    I2C_Read_Packet_From_Sensor(buf, 6);

    //check status register
    status = ICM_ST2;
    I2C_Read_Packet_From_Sensor(&status, 1);
    if (status&0x08)
    {
        //either a read error or mag sensor overflow occurred
        buf[0] = 0xFF;
        buf[1] = 0x7F;
        buf[2] = 0xFF;
        buf[3] = 0x7F;
        buf[4] = 0xFF;
        buf[5] = 0x7F;
    }
}

uint8_t ICM20948_hasTimeoutPeriodPassed(uint32_t currentSampleTsTicks)
{
    //TODO make more efficient

    if (magSampleSkipEnabled)
    {
        /* Mask to 16-bit to simplify calculations */
        uint32_t currentSampleTsTicksMasked = currentSampleTsTicks & 0xFFFF;
        uint32_t lastMagSampleTsTicksMasked = lastMagSampleTsTicks & 0xFFFF;

        // Check if roll-over has occurred
        if (lastMagSampleTsTicksMasked > currentSampleTsTicksMasked)
        {
            currentSampleTsTicksMasked |= 0x10000;
        }

        uint32_t magSampleTsDiffTicks = currentSampleTsTicksMasked - lastMagSampleTsTicksMasked;
        if (magSampleTsDiffTicks < SAMPLING_TIMER_TICKS_100Hz)
        {
            /* Mag won't have new sample ready yet so don't read from it */
            return 0;
        }
    }

    // Mag should have new sample ready
    return 1;
}

uint8_t ICM20948_getMagAndStatus(uint32_t currentSampleTsTicks, uint8_t *buf)
{
    I2C_Set_Slave_Address(AK09916_MAG_ADDR);

    *buf = ICM_ST1;
    I2C_Read_Packet_From_Sensor(buf, ICM_MAG_RD_SIZE);

    // Check Status 1 if Data not ready
    if (!(*(buf + ICM_MAG_IDX_ST1) & 0x01))
    {
        return 0;
    }

    lastMagSampleTsTicks = currentSampleTsTicks;

    //check Status 2 register
    if (*(buf + ICM_MAG_IDX_ST2) & 0x08)
    {
        //either a read error or mag sensor overflow occurred
        buf[ICM_MAG_IDX_XOUT_L] = 0xFF;
        buf[ICM_MAG_IDX_XOUT_H] = 0x7F;
        buf[ICM_MAG_IDX_YOUT_L] = 0xFF;
        buf[ICM_MAG_IDX_YOUT_H] = 0x7F;
        buf[ICM_MAG_IDX_ZOUT_L] = 0xFF;
        buf[ICM_MAG_IDX_ZOUT_H] = 0x7F;
    }
    return 1;
}
