
#ifndef SHIMMER_CALIBRATION_H
#define SHIMMER_CALIBRATION_H
//SC_ short for SHIMMER_CALIBRATION_
#define SC_HW_IDENTIFIER               0//DEVICE_VER
#define SC_FW_IDENTIFIER_L             1
#define SC_FW_IDENTIFIER_H             2
#define SC_FW_VER_MAJOR_L              3
#define SC_FW_VER_MAJOR_H              4
#define SC_FW_VER_MINOR                5
#define SC_FW_VER_REL                  6

#define SC_OFFSET_LENGTH_L             0
#define SC_OFFSET_LENGTH_H             1
#define SC_OFFSET_VER_HW_ID_L          2
#define SC_OFFSET_VER_HW_ID_H          3
#define SC_OFFSET_VER_FW_ID_L          4
#define SC_OFFSET_VER_FW_ID_H          5
#define SC_OFFSET_VER_FW_MAJOR_L       6
#define SC_OFFSET_VER_FW_MAJOR_H       7
#define SC_OFFSET_VER_FW_MINOR_L       8
#define SC_OFFSET_VER_FW_INTER_L       9

#define SC_OFFSET_SENSOR_ID_L          0
#define SC_OFFSET_SENSOR_ID_H          1
#define SC_OFFSET_SENSOR_RANGE         2
#define SC_OFFSET_SENSOR_LENGTH        3
#define SC_OFFSET_SENSOR_TIMESTAMP     4
#define SC_OFFSET_SENSOR_DATA          12

#define SC_SENSOR_ANALOG_ACCEL         2
#define SC_SENSOR_MPU9150_GYRO         30
#define SC_SENSOR_LSM303DLHC_ACCEL     31
#define SC_SENSOR_LSM303DLHC_MAG       32
#define SC_SENSOR_BMP180_PRESSURE      36
#define SC_SENSOR_HOST_ECG             100

//Analogue Accel Range
#define SC_SENSOR_RANGE_ANALOG_ACCEL               0
#define SC_SENSOR_RANGE_MAX_ANALOG_ACCEL           1
#define SC_SENSOR_RANGE_SIZE_ANALOG_ACCEL          21
//LSM303DLHC Accel Range
#define SC_SENSOR_RANGE_LSM303DLHC_ACCEL_2G        0x00
#define SC_SENSOR_RANGE_LSM303DLHC_ACCEL_4G        0x01
#define SC_SENSOR_RANGE_LSM303DLHC_ACCEL_8G        0x02
#define SC_SENSOR_RANGE_LSM303DLHC_ACCEL_16G       0x03
#define SC_SENSOR_RANGE_MAX_LSM303DLHC_ACCEL       4
#define SC_SENSOR_RANGE_SIZE_LSM303DLHC_ACCEL      21
//LSM303DLHC Mag gain
#define SC_SENSOR_RANGE_LSM303DLHC_MAG_1_3G        0x01 //+/-1.3 Gauss
#define SC_SENSOR_RANGE_LSM303DLHC_MAG_1_9G        0x02 //+/-1.9 Gauss
#define SC_SENSOR_RANGE_LSM303DLHC_MAG_2_5G        0x03 //+/-2.5 Gauss
#define SC_SENSOR_RANGE_LSM303DLHC_MAG_4_0G        0x04 //+/-4.0 Gauss
#define SC_SENSOR_RANGE_LSM303DLHC_MAG_4_7G        0x05 //+/-4.7 Gauss
#define SC_SENSOR_RANGE_LSM303DLHC_MAG_5_6G        0x06 //+/-5.6 Gauss
#define SC_SENSOR_RANGE_LSM303DLHC_MAG_8_1G        0x07 //+/-8.1 Gauss
#define SC_SENSOR_RANGE_MAX_LSM303DLHC_MAG         7
#define SC_SENSOR_RANGE_SIZE_LSM303DLHC_MAG        21
//MPU9x50 Gyro range
#define SC_SENSOR_RANGE_MPU9250_GYRO_250DPS        0x00 //+/-250 dps
#define SC_SENSOR_RANGE_MPU9250_GYRO_500DPS        0x01 //+/-500 dps
#define SC_SENSOR_RANGE_MPU9250_GYRO_1000DPS       0x02 //+/-1000 dps
#define SC_SENSOR_RANGE_MPU9250_GYRO_2000DPS       0x03 //+/-2000 dps
#define SC_SENSOR_RANGE_MAX_MPU9250_GYRO           4
#define SC_SENSOR_RANGE_SIZE_MPU9250_GYRO          21
//BMP180 Temperature/Pressure Range
#define SC_SENSOR_RANGE_BMP180                     0
#define SC_SENSOR_RANGE_MAX_BMP180                 1
#define SC_SENSOR_RANGE_SIZE_BMP180                22

#endif //SHIMMER_CALIBRATION_H
