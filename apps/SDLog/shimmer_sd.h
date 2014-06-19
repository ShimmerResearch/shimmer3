#ifndef SHIMMER_SD_H
#define SHIMMER_SD_H


//these are defined in the Makefile for BtStream (TinyOS)
#define DEVICE_VER         3      //Represents SR30. 0-3 for shimmer1 to shimmer3
#define FW_IDENTIFIER      2      //Two byte firmware identifier number: always 2 for SDLog
#define FW_VER_MAJOR       0      //Maor version number: 0-65535
#define FW_VER_MINOR       7      //Minor version number: 0-255
#define FW_VER_INTERNAL    0      //internal version number: 0-255


// Packet Types
#define DATA_PACKET                    0x00
#define ACK_COMMAND_PROCESSED          0xFF
#define ROUTINE_COMMUNICATION          0xE0   //'0'

//SENSORS0
#define   SENSOR_A_ACCEL               0x80
#define SENSOR_MPU9150_GYRO            0x40
#define SENSOR_LSM303DLHC_MAG          0x20
#define SENSOR_EXG1_24BIT              0x10
#define SENSOR_EXG2_24BIT              0x08
#define SENSOR_GSR                     0x04
#define SENSOR_EXT_A7                  0x02
#define SENSOR_EXT_A6                  0x01
//SENSORS1
#define SENSOR_STRAIN                  0x80   //higher priority than SENSOR_INT_A13 and SENSOR_INT_A14
#define SENSOR_VBATT                   0x20
#define SENSOR_LSM303DLHC_ACCEL        0x10
#define SENSOR_EXT_A15                 0x08
#define SENSOR_INT_A1                  0x04
#define SENSOR_INT_A12                 0x02
#define SENSOR_INT_A13                 0x01
//SENORS2
#define SENSOR_INT_A14                 0x80
#define SENSOR_MPU9150_ACCEL           0x40
#define SENSOR_MPU9150_MAG             0x20
//#define SENSOR_MPU9150_TEMP          0x10
//#define SENSOR_LSM303DLHC_TEMPERATURE   0x08
#define SENSOR_EXG1_16BIT              0x10
#define SENSOR_EXG2_16BIT              0x08
#define SENSOR_BMP180_PRESSURE         0x04
//#define SENSOR_BMP180_TEMPERATURE    0x02
//#define SENSOR_EXP_POWER             0x01
//SENORS3
#define SENSOR_MSP430_TEMPERATURE      0x01

#define MAX_COMMAND_ARG_SIZE           21       //maximum number of arguments for any command sent to SR30 (calibration data)
#define RESPONSE_PACKET_SIZE           85      	//biggest possibly required (4 x kinematic calibration responses)
#define MAX_NUM_CHANNELS               29       //3xanalogAccel + 3xdigiGyro + 3xdigiMag + 3xLSM303DLHCAccel + 3xMPU9150Accel + batteryVoltage + 3xexternalADC + 4xinternalADC
#define DATA_PACKET_SIZE               80      	//4 + (MAX_NUM_CHANNELS * 2)


// Infomem contents
#define NV_NUM_SETTINGS_BYTES             33
#define NV_NUM_CALIBRATION_BYTES          84
#define NV_TOTAL_NUM_CONFIG_BYTES         NV_NUM_SETTINGS_BYTES + NV_NUM_CALIBRATION_BYTES


#define NV_SAMPLING_RATE                  0
#define NV_BUFFER_SIZE                    2
#define NV_SENSORS0                       3
#define NV_SENSORS1                       4
#define NV_SENSORS2                       5
#define NV_CONFIG_SETUP_BYTE0             6 //sensors setting bytes
#define NV_CONFIG_SETUP_BYTE1             7
#define NV_CONFIG_SETUP_BYTE2             8
#define NV_CONFIG_SETUP_BYTE3             9
#define NV_TRIAL_CONFIG0                  10
#define NV_TRIAL_CONFIG1                  11
#define NV_BT_INTERVAL                    12
#define NV_EXG_ADS1292R_1_CONFIG1         13// exg bytes, not implemented yet
#define NV_EXG_ADS1292R_1_CONFIG2         14
#define NV_EXG_ADS1292R_1_LOFF            15
#define NV_EXG_ADS1292R_1_CH1SET          16
#define NV_EXG_ADS1292R_1_CH2SET          17
#define NV_EXG_ADS1292R_1_RLD_SENS        18
#define NV_EXG_ADS1292R_1_LOFF_SENS       19
#define NV_EXG_ADS1292R_1_LOFF_STAT       20
#define NV_EXG_ADS1292R_1_RESP1           21
#define NV_EXG_ADS1292R_1_RESP2           22
#define NV_EXG_ADS1292R_2_CONFIG1         23
#define NV_EXG_ADS1292R_2_CONFIG2         24
#define NV_EXG_ADS1292R_2_LOFF            25
#define NV_EXG_ADS1292R_2_CH1SET          26
#define NV_EXG_ADS1292R_2_CH2SET          27
#define NV_EXG_ADS1292R_2_RLD_SENS        28
#define NV_EXG_ADS1292R_2_LOFF_SENS       29
#define NV_EXG_ADS1292R_2_LOFF_STAT       30
#define NV_EXG_ADS1292R_2_RESP1           31
#define NV_EXG_ADS1292R_2_RESP2           32
#define NV_A_ACCEL_CALIBRATION            33
#define NV_MPU9150_GYRO_CALIBRATION       54
#define NV_LSM303DLHC_MAG_CALIBRATION     75
#define NV_LSM303DLHC_ACCEL_CALIBRATION   96//96-117

//Config byte masks
//Config Byte0
#define LSM303DLHC_ACCEL_SAMPLING_RATE          0xF0
#define LSM303DLHC_ACCEL_RANGE                  0x0C
#define LSM303DLHC_ACCEL_LOW_POWER_MODE         0x02
#define LSM303DLHC_ACCEL_HIGH_RESOLUTION_MODE   0x01
//Config Byte1
#define MPU9150_SAMPLING_RATE                   0xFF
//Config Byte2
#define LSM303DLHC_MAG_GAIN                     0xE0
#define LSM303DLHC_MAG_SAMPLING_RATE            0x1C
#define MPU9150_GYRO_RANGE                      0x03
//Config Byte3
#define MPU9150_ACCEL_RANGE                     0xC0
#define BMP180_PRESSURE_RESOLUTION              0x30
#define GSR_RANGE                               0x0E
#define EXP_POWER_ENABLE                        0x01
//#define BMP180_PRESSURE_RESOLUTION            0x30
//Unused bits 3-0


//ADC initialisation mask
#define MASK_A_ACCEL    0x0001
#define MASK_VBATT      0x0002
#define MASK_EXT_A7     0x0004
#define MASK_EXT_A6     0x0008
#define MASK_EXT_A15    0x0010
#define MASK_INT_A1     0x0020
#define MASK_GSR        0x0020   //uses ADC1
#define MASK_INT_A12    0x0040
#define MASK_INT_A13    0x0080
#define MASK_INT_A14    0x0100
#define MASK_STRAIN     0x0180   //uses ADC13 and ADC14
//#define MASK_MSP_TEMP    0x0200


//LSM303DLHC Accel Range
//Corresponds to the FS field of the LSM303DLHC's CTRL_REG4_A register
//and the AFS_SEL field of the MPU9150's ACCEL_CONFIG register
#define ACCEL_2G        0x00
#define ACCEL_4G        0x01
#define ACCEL_8G        0x02
#define ACCEL_16G       0x03

//LSM303DLHC Accel Sampling Rate
//Corresponds to the ODR field of the LSM303DLHC's CTRL_REG1_A register
#define LSM303DLHC_ACCEL_POWER_DOWN 0x00
#define LSM303DLHC_ACCEL_1HZ        0x01
#define LSM303DLHC_ACCEL_10HZ       0x02
#define LSM303DLHC_ACCEL_25HZ       0x03
#define LSM303DLHC_ACCEL_50HZ       0x04
#define LSM303DLHC_ACCEL_100HZ      0x05
#define LSM303DLHC_ACCEL_200HZ      0x06
#define LSM303DLHC_ACCEL_400HZ      0x07
#define LSM303DLHC_ACCEL_1_620KHZ   0x08 //1.620kHz in Low-power mode only
#define LSM303DLHC_ACCEL_1_344kHz   0x09 //1.344kHz in normal mode, 5.376kHz in low-power mode

//LSM303DLHC Mag gain
#define LSM303DLHC_MAG_1_3G         0x01 //+/-1.3 Gauss
#define LSM303DLHC_MAG_1_9G         0x02 //+/-1.9 Gauss
#define LSM303DLHC_MAG_2_5G         0x03 //+/-2.5 Gauss
#define LSM303DLHC_MAG_4_0G         0x04 //+/-4.0 Gauss
#define LSM303DLHC_MAG_4_7G         0x05 //+/-4.7 Gauss
#define LSM303DLHC_MAG_5_6G         0x06 //+/-5.6 Gauss
#define LSM303DLHC_MAG_8_1G         0x07 //+/-8.1 Gauss

//LSM303DLHC Mag sampling rate
#define LSM303DLHC_MAG_0_75HZ       0x00 //0.75 Hz
#define LSM303DLHC_MAG_1_5HZ        0x01 //1.5 Hz
#define LSM303DLHC_MAG_3HZ          0x02 //3.0 Hz
#define LSM303DLHC_MAG_7_5HZ        0x03 //7.5 Hz
#define LSM303DLHC_MAG_15HZ         0x04 //15 Hz
#define LSM303DLHC_MAG_30HZ         0x05 //30 Hz
#define LSM303DLHC_MAG_75HZ         0x06 //75 Hz
#define LSM303DLHC_MAG_220HZ        0x07 //220 Hz


//calibration info
#define S_ACCEL                  0
#define S_GYRO                   1
#define S_MAG                    2
#define S_ACCEL_A                3
//#define S_ECG                  3
//#define S_EMG                  4

//MPU9150 Gyro range
#define MPU9150_GYRO_250DPS      0x00 //+/-250 dps
#define MPU9150_GYRO_500DPS      0x01 //+/-500 dps
#define MPU9150_GYRO_1000DPS     0x02 //+/-1000 dps
#define MPU9150_GYRO_2000DPS     0x03 //+/-2000 dps

//#digital accel_range
#define RANGE_2G                 0
#define RANGE_4G                 1
#define RANGE_8G                 2
#define RANGE_16G                3

//#mag_gain
#define LSM303_MAG_13GA          1
#define LSM303_MAG_19GA          2
#define LSM303_MAG_25GA          3
#define LSM303_MAG_40GA          4
#define LSM303_MAG_47GA          5
#define LSM303_MAG_56GA          6
#define LSM303_MAG_81GA          7



//SD Log file header format
#define SDHEAD_LEN               256 //(0-255)

#define SDH_SAMPLE_RATE_0        0
#define SDH_SAMPLE_RATE_1        1
#define SDH_SENSORS0             3
#define SDH_SENSORS1             4
#define SDH_SENSORS2             5
#define SDH_CONFIG_SETUP_BYTE0   8 //sensors setting bytes
#define SDH_CONFIG_SETUP_BYTE1   9
#define SDH_CONFIG_SETUP_BYTE2   10
#define SDH_CONFIG_SETUP_BYTE3   11
#define SDH_TRIAL_CONFIG0        16
#define SDH_TRIAL_CONFIG1        17
#define SDH_BROADCAST_INTERVAL   18
//#define SDH_ACCEL_RANGE        22
//#define SDH_GSR_RANGE          23
//#define SDH_MAG_RANGE          24
//#define SDH_MAG_RATE           25
//#define SDH_ACCEL_RATE         26
//#define SDH_GYRO_RATE          27
//#define SDH_GYRO_RANGE         28
//#define SDH_PRESSURE_PREC      29
#define SDH_SHIMMERVERSION_BYTE_0   30
#define SDH_SHIMMERVERSION_BYTE_1   31
#define SDH_MYTRIAL_ID              32
#define SDH_NSHIMMER                33
#define SDH_FW_VERSION_TYPE_0       34
#define SDH_FW_VERSION_TYPE_1       35
#define SDH_FW_VERSION_MAJOR_0      36
#define SDH_FW_VERSION_MAJOR_1      37
#define SDH_FW_VERSION_MINOR        38
#define SDH_FW_VERSION_INTERNAL     39
#define SDH_CONFIG_TIME_0           52
#define SDH_CONFIG_TIME_1           53
#define SDH_CONFIG_TIME_2           54
#define SDH_CONFIG_TIME_3           55
#define SDH_EXG_ADS1292R_1_CONFIG1        56
#define SDH_EXG_ADS1292R_1_CONFIG2        57
#define SDH_EXG_ADS1292R_1_LOFF           58
#define SDH_EXG_ADS1292R_1_CH1SET         59
#define SDH_EXG_ADS1292R_1_CH2SET         60
#define SDH_EXG_ADS1292R_1_RLD_SENS       61
#define SDH_EXG_ADS1292R_1_LOFF_SENS      62
#define SDH_EXG_ADS1292R_1_LOFF_STAT      63
#define SDH_EXG_ADS1292R_1_RESP1          64
#define SDH_EXG_ADS1292R_1_RESP2          65
#define SDH_EXG_ADS1292R_2_CONFIG1        66
#define SDH_EXG_ADS1292R_2_CONFIG2        67
#define SDH_EXG_ADS1292R_2_LOFF           68
#define SDH_EXG_ADS1292R_2_CH1SET         69
#define SDH_EXG_ADS1292R_2_CH2SET         70
#define SDH_EXG_ADS1292R_2_RLD_SENS       71
#define SDH_EXG_ADS1292R_2_LOFF_SENS      72
#define SDH_EXG_ADS1292R_2_LOFF_STAT      73
#define SDH_EXG_ADS1292R_2_RESP1          74
#define SDH_EXG_ADS1292R_2_RESP2          75
#define SDH_LSM303DLHC_ACCEL_CALIBRATION  76
#define SDH_MPU9150_GYRO_CALIBRATION      97
#define SDH_LSM303DLHC_MAG_CALIBRATION    118
#define SDH_A_ACCEL_CALIBRATION           139
#define SDH_TEMP_PRES_CALIBRATION         160
#define SDH_MY_LOCALTIME                  252   //252-255

//SENSORS0
#define   SDH_SENSOR_A_ACCEL           0x80
#define   SDH_SENSOR_A_ACCEL           0x80
#define   SDH_SENSOR_A_ACCEL           0x80
#define SDH_SENSOR_MPU9150_GYRO        0x40
#define SDH_SENSOR_LSM303DLHC_MAG      0x20
#define SDH_SENSOR_EXG1_24BIT          0x10
#define SDH_SENSOR_EXG2_24BIT          0x08
#define SDH_SENSOR_GSR                 0x04
#define SDH_SENSOR_EXTCH7              0x02
#define SDH_SENSOR_EXTCH6              0x01
//SENSORS1
#define SDH_SENSOR_STRAIN              0x80
//#define SDH_SENSOR_HR                0x40
#define SDH_SENSOR_VBATT               0x20
#define SDH_SENSOR_LSM303DLHC_ACCEL    0x10
#define SDH_SENSOR_EXTCH15             0x08
#define SDH_SENSOR_INTCH1              0x04
#define SDH_SENSOR_INTCH12             0x02
#define SDH_SENSOR_INTCH13             0x01
//SENSORS2
#define SDH_SENSOR_INTCH14             0x80
#define SDH_SENSOR_MPU9150_ACCEL       0x40
#define SDH_SENSOR_MPU9150_MAG         0x20
#define SDH_SENSOR_EXG1_16BIT          0x10
#define SDH_SENSOR_EXG2_16BIT          0x08
#define SDH_SENSOR_BMP180_PRES         0x04
//#define SDH_SENSOR_BMP180_TEMP       0x02
//#define SDH_SENSOR_EXP_POWER         0x01
//SENSORS3
#define SDH_SENSOR_MSP430_TEMP         0x01
#define SDH_SENSOR_TCXO                0x80

//SDH_TRIAL_CONFIG0
#define SDH_IAMMASTER                  0x02
#define SDH_TIME_SYNC                  0x04
#define SDH_TIME_STAMP                 0x08
#define SDH_GYRO_BUTTON_ENABLE         0x10
#define SDH_USER_BUTTON_ENABLE         0x20
#define SDH_SET_PMUX                   0x40
#define SDH_SET_5V_REG                 0x80
//SDH_TRIAL_CONFIG1
#define SDH_SINGLETOUCH                0x80
#define SDH_ACCEL_LPM                  0x40
#define SDH_ACCEL_HRM                  0x20
#define SDH_TCXO                       0x10

//choice of clock
#define TCXO_CLOCK      255765.625
#define MSP430_CLOCK    32768

// BT routine communication
// all node time must *2 in use
// all center time must *4 in use
#define RC_AHD          3
#define RC_WINDOW_N     13
#define RC_WINDOW_C     27
#define RC_INT_N        27
#define RC_INT_C        54
#define RC_CLK_N        16384
#define RC_CLK_C        8192
#define RC_FACTOR_N     32768/RC_CLK_N
#define RC_FACTOR_C     32768/RC_CLK_C


//routine communication response text - ack:flag:time4:time3:time2:time1
#define RCT_SIZE        6
#define RCT_ACK         0
#define RCT_FLG         1
#define RCT_TIME        2
#define RCT_TIME_SIZE   4


// sd card write buffer size
#define SDBUFF_SIZE_MAX 4096
#define SDBUFF_SIZE 512

// BATTERY
#define BATT_HIGH       0x01
#define BATT_MID        0x02
#define BATT_LOW        0x04
#define BATT_INTERVAL   19660800    //use 19660800 for 10min interval

#define MAX_CHARS       13

//BtStream specific extension to range values : should SDLog keep it?
#define GSR_AUTORANGE   0x04

#endif
