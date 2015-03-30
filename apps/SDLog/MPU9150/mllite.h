/*
 * mllite.h
 *
 * Created on: 2d Apr 2014
 * Author: mnolan
 */

#ifndef MLLITE_H_
#define MLLITE_H_

// For debugging
//#define SEND_INV_DRIVER_MESSAGES
//#define MOTION_NO_MOTION_LED

// Data read from MPL
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define PRINT_COMPASS   (0x08)
#define PRINT_EULER     (0x10)
#define PRINT_ROT_MAT   (0x20)
#define PRINT_HEADING   (0x40)
#define PRINT_PEDOM     (0x80)

// additions to original MPL library -> shouldn't affect normal operation
#define PRINT_QUAT_9DOF      (0x03)
#define PRINT_TEMPERATURE   (0x05)
#define PRINT_ACCEL_RAW     (0x06)
#define PRINT_GYRO_RAW      (0x07)
#define PRINT_COMPASS_RAW   (0x09)
#define PRINT_QUAT_RAW   	(0x0A)
#define PRINT_TAP_AND_DIR   	(0x0B)
#define PRINT_MOT_AND_ORIENT   	(0x0C)
#define PRINT_EULER_9DOF		(0x0D)

static void read_from_mpl(void);
static void setup_gyro(void);
static void tap_cb(unsigned char direction, unsigned char count);
static void display_orient_cb(unsigned char orientation);
static void orient_cb(unsigned char orientation);
//static void msp430_reset(void);
void gyro_data_ready_cb(void);
void MPU_platformInit(uint8_t *storedConfigPtr);

void MPU9150_DmpInit(uint8_t *storedConfigLocal);
void MPL_getValue(uint16_t sensor, uint8_t *localBuf);
int MPU_getDmpData(void);
int MPL_gyroCalibrate(void);
int MPL_magCalibrate(void);
uint8_t MPL_saveCalibrationBytes(void);
uint8_t MPL_loadCalibrationBytes(void);
uint8_t MPU_saveCalibration(void);
uint8_t MPU_loadCalibration(uint8_t *storedConfigLcl);
uint8_t MPU_readCalibration(uint8_t sensor, uint8_t *storedConfigLcl);
void MPL_defaultCalibration(uint8_t sensor, uint8_t *storedConfigLcl);
void pansenti_sensor_fusion(long *m_rawQuaternion, long *m_rawMag);

#endif /* MLLITE_H_ */
