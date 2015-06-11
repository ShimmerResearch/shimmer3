/**
 * Modified from mllite_test.c
 * Invensense Embedded Motion Processing Library
 *
 * Created on: 2d Apr 2014
 * Author: Mark Nolan
 *
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "../FatFs/ff.h"
#include "core/driver/msp430/msp430_interrupt.h"
#include "MPU9150.h"

#include "core/driver/msp430/F5xx_F6xx_Core_Lib/HAL_FLASH.h"
#include "msp430.h"
#include "../msp430_clock/msp430_clock.h"
#include "../5xx_HAL/hal_I2C.h"

#include "core/driver/eMPL/inv_gyro.h"
#include "core/driver/eMPL/inv_gyro_dmp_android.h"
#include "core/mllite/invensense.h"
#include "core/mpl/invensense_adv.h"
#include "core/eMPL-hal/eMPL_outputs.h"
#include "core/driver/include/mltypes.h"
#include "core/driver/include/mpu.h"
#include "core/driver/include/log.h"
#include "core/driver/msp430/inv_driver_if.h"
#include "core/driver/msp430/packet.h"
#include "core/driver/msp430/msp430_i2c.h"

#include "Pansenti/MPUQuaternion.h"
#include "mllite.h"
#include "../shimmer_sd.h"

#define nineDOF_USE_LSM303

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define COMPASS_ON      (0x04)

// 50 Hz has been tested to give good performance
// on the testing code with mCLK set to 12Mhz.
// Recommended to increase the system clock to
// higher Hz from 12Mhz to increase the 50Hz update
// any higher value.
unsigned int MPU_RATE_HZ = 50;
unsigned int MAG_RATE_MS = 20;

#define FLASH_SIZE      (128) // was 512, conflict with BTstream so set to 128?
#define FLASH_MEM_START ((void*)0x1980) // was 0x1800, conflict with BTstream - Info C 0x1880

// Default used to indicate data ready
#define TOGGLE_RED_LED  (P7OUT ^=BIT2) // P7BIT2 RED LED
#define SET_RED_LED     (P7OUT |=BIT2)
#define CLEAR_RED_LED   (P7OUT &=~ BIT2)

// Default used to indicate motion detect
#define TOGGLE_BLUE_LED  (P1OUT ^=BIT2) // P1BIT2 BLUE LED
#define SET_BLUE_LED     (P1OUT |=BIT2)
#define CLEAR_BLUE_LED   (P1OUT &=~ BIT2)

//// Default used to indicate Failure
//#define TOGGLE_BOTH_LED (P7OUT ^=  BIT2) // P7BIT2 RED LED
//#define SET_BOTH_LED    (P7OUT |=  BIT2)
//#define CLEAR_BOTH_LED  (P7OUT &=~ BIT2)

struct rx_s {
	unsigned char header[3];
	unsigned char cmd;
};

struct hal_s {
	unsigned char lp_accel_mode;
	unsigned char sensors;
	unsigned char dmp_on;
	unsigned char wait_for_tap;
	unsigned char new_gyro;
	unsigned char new_compass;
	unsigned long no_dmp_hz;
	unsigned long next_pedo_ms;
	unsigned short read_pedo_ms;
	unsigned long next_compass_ms;
	/* TODO: For this example, this value is never changed. Save a couple bytes. */
	unsigned short read_compass_ms;
	unsigned short report;
	unsigned char dmp_features;
	struct rx_s rx;
	struct int_param_s int_param;
	unsigned char pansenti_nineDOF;
	unsigned char nineDOF_use_LSM;
};
static struct hal_s hal = { 0 };

/* USB RX binary semaphore. Actually, it's just a flag. Not included in struct
 * because it's declared extern elsewhere.
 */
volatile unsigned char rx_new;

/* Should be defined by your compass driver. */
extern struct driver_if_s compass_if;

//unsigned char *mpl_key = (unsigned char*)"Wrong_Key";
unsigned char *mpl_key = (unsigned char*) "eMPL 5.1";

// Global Variables used in the main and functions
unsigned char accel_fsr;
unsigned short gyro_rate, gyro_fsr, compass_fsr, mag_rate;
unsigned long timestamp;
unsigned long sensor_timestamp;
int new_data;

short gyroRaw[3], accelRaw[3], compassRaw[3];
long gyroMpl[3], accelMpl[3], quatMpl[4], quat9DOF[4], quatRaw[4], compassMpl[3], temperature, heading, euler6DOF[3], euler9DOF[3];

unsigned long pedomBuffer[2];

uint8_t tapAndDir, motAndOrient;

unsigned char mplCaliBuffer[110];
uint8_t calibrationBytesFilePresent;
size_t caliStoreSize;
long accelBias[3], gyroBias[3], magBias[3];

//----------------------- Pansenti start ----------------------------
//  MPU_MAG_MIX defines the influence that the magnetometer has on the yaw output.
//  The magnetometer itself is quite noisy so some mixing with the gyro yaw can help
//  significantly. Some example values are defined below:
#define  MPU_MAG_MIX_GYRO_ONLY          0                   // just use gyro yaw
#define  MPU_MAG_MIX_MAG_ONLY           1                   // just use magnetometer and no gyro yaw
#define  MPU_MAG_MIX_GYRO_AND_MAG       10                  // a good mix value
#define  MPU_MAG_MIX_GYRO_AND_SOME_MAG  50                  // mainly gyros with a bit of mag correction

void pansenti_sensor_fusion(long *m_rawQuaternion, long *m_rawMag);
float m_lastDMPYaw;                                       // the last yaw from the DMP gyros
float m_lastYaw;                                          // last calculated output yaw
int m_magMix = MPU_MAG_MIX_MAG_ONLY;                                             // controls gyro and magnetometer mixing for yaw
//----------------------- Pansenti end ----------------------------


// Get data from MPL.
// TODO: Add return values to the inv_get_sensor_type_xxx APIs to differentiate
// between new and stale data.
static void read_from_mpl(void) {
	long msg, data[9];//, compass[3];//, accel[3];
	int8_t accuracy;
	unsigned long timestamp;

	if (inv_get_sensor_type_quat(quatMpl, &accuracy, (inv_time_t*) &timestamp)) {
//TODO:
		if(hal.pansenti_nineDOF) {
			inv_get_sensor_type_compass(compassMpl, &accuracy,	(inv_time_t*) &timestamp);
			quat9DOF[0] = quatMpl[0];
			quat9DOF[1] = quatMpl[1];
			quat9DOF[2] = quatMpl[2];
			quat9DOF[3] = quatMpl[3];
			pansenti_sensor_fusion(quat9DOF, compassMpl);
//			if (inv_get_sensor_type_compass(compass, &accuracy,
//							(inv_time_t*) &timestamp))
//				if (inv_get_sensor_type_accel(accel, &accuracy,
//						(inv_time_t*) &timestamp))
//					data[6] = inv_compass_angle(compass, accel, data);
		}
#ifdef SEND_INV_DRIVER_MESSAGES
		// Sends a quaternion packet to the PC. Since this is used by the Python
		// test app to visually represent a 3D quaternion, it's sent each time
		// the MPL has new data.
		eMPL_send_quat(quatMpl);
		/* Specific data packets can be sent or suppressed using USB commands. */
		if (hal.report & PRINT_QUAT){
			eMPL_send_data(PACKET_DATA_QUAT, quatMpl);
		}
#endif
	}

	if (hal.report & PRINT_ACCEL) {
		if (inv_get_sensor_type_accel(accelMpl, &accuracy,
				(inv_time_t*) &timestamp)){
#ifdef SEND_INV_DRIVER_MESSAGES
			eMPL_send_data(PACKET_DATA_ACCEL, accelMpl);
#endif
		}
	}
	if (hal.report & PRINT_GYRO) {
		if (inv_get_sensor_type_gyro(gyroMpl, &accuracy, (inv_time_t*) &timestamp)){
#ifdef SEND_INV_DRIVER_MESSAGES
			eMPL_send_data(PACKET_DATA_GYRO, gyroMpl);
#endif
		}
	}
	if (hal.report & PRINT_COMPASS) {
		if(hal.pansenti_nineDOF){
#ifdef SEND_INV_DRIVER_MESSAGES
			eMPL_send_data(PACKET_DATA_COMPASS, compassMpl);
#endif
		}
		else{
			if (inv_get_sensor_type_compass(compassMpl, &accuracy,
				(inv_time_t*) &timestamp)){
#ifdef SEND_INV_DRIVER_MESSAGES
			eMPL_send_data(PACKET_DATA_COMPASS, compassMpl);
#endif
			}
		}
	}
	if (hal.report & PRINT_EULER) {
		if (inv_get_sensor_type_euler(euler6DOF, &accuracy,
				(inv_time_t*) &timestamp)){
#ifdef SEND_INV_DRIVER_MESSAGES
			eMPL_send_data(PACKET_DATA_EULER, euler6DOF);
#endif
		}
	}
	if (hal.report & PRINT_ROT_MAT) {
		if (inv_get_sensor_type_rot_mat(data, &accuracy,
				(inv_time_t*) &timestamp)){
#ifdef SEND_INV_DRIVER_MESSAGES
			eMPL_send_data(PACKET_DATA_ROT, data);
#endif
		}
	}
	if (hal.report & PRINT_HEADING) {
		if (inv_get_sensor_type_heading(&heading, &accuracy,
				(inv_time_t*) &timestamp)){
#ifdef SEND_INV_DRIVER_MESSAGES
			eMPL_send_data(PACKET_DATA_HEADING, &heading);
#endif
		}
	}

	if (hal.report & PRINT_PEDOM) {
		unsigned long timestamp;
		msp430_get_clock_ms(&timestamp);
		if (timestamp > hal.next_pedo_ms) {
			hal.next_pedo_ms = timestamp + hal.read_pedo_ms;
			unsigned long step_count, walk_time;
			dmp_get_pedometer_step_count(&step_count);
			dmp_get_pedometer_walk_time(&walk_time);
			// load Pedometer buffer for Shimmer data stream
			pedomBuffer[0] = step_count;
			pedomBuffer[1] = walk_time;

#ifdef SEND_INV_DRIVER_MESSAGES
			MPL_LOGI("Walked %ld steps over %ld milliseconds..\n",
					step_count, walk_time);
#endif
		}
	}

	// Whenever the MPL detects a change in motion state, the application can
	// be notified. For this example, we use an LED to represent the current
	// motion state.
	msg = inv_get_message_level_0(
			INV_MSG_MOTION_EVENT | INV_MSG_NO_MOTION_EVENT);
	if (msg) {
		if (msg & INV_MSG_MOTION_EVENT) {
#ifdef SEND_INV_DRIVER_MESSAGES
			MPL_LOGI("Motion!\n");
#endif
			motAndOrient |= 0x80; // Shimmer - Motion Event - BIT7
#ifdef MOTION_NO_MOTION_LED
			SET_BLUE_LED;
#endif
		} else if (msg & INV_MSG_NO_MOTION_EVENT) {
#ifdef SEND_INV_DRIVER_MESSAGES
			MPL_LOGI("No motion!\n");
#endif
			motAndOrient &= 0x7F; // Shimmer - No Motion Event - BIT7
#ifdef MOTION_NO_MOTION_LED
			CLEAR_BLUE_LED;
#endif
		} else if (msg & INV_MSG_NEW_GB_EVENT) {
			long gyroBias[3], temp;
			inv_get_gyro_bias(gyroBias, &temp);
			dmp_set_gyro_bias(gyroBias);
		}
	}
}

// Platform-specific information. Kinda like a boardfile.
struct platform_data_s {
	signed char orientation[9];
};

// The sensors can be mounted onto the board in any orientation. The mounting
// matrix seen below tells the MPL how to rotate the raw data from the
// driver(s).
// TODO: The following matrices refer to the configuration on an internal test
// board at Invensense. If needed, please modify the matrices to match the
// chip-to-body matrix for your particular set up.
// The present matrix is updated to match the MotionFit SDK board
// according to the board definition of XYZ. (Z is coming out)
//
//     |------------------------------------|
//     SW1             		[ BT ]		    |  (-Y Board)
//     |                                    |    .
//     |           ------                   |   /.\
//     |          |[INV] |                  |    .
//     | +x chip<-|     .|.pin1             |    .
//     |           ------                   |    .
//     |               					    |  -------> (-X Board)
//     SW2					[ TI ]		    |    .
//     SW4								    |
//     |                                    |
//     |O-------------[USB]----------------O|
//
// Original Invensense alignment
//static struct platform_data_s gyro_pdata = { .orientation = { 1, 0, 0, 0, 1,
//		0, 0, 0, 1 } };
//static struct platform_data_s compass_pdata = { .orientation = { 0, 1, 0, 1, 0,
//		0, 0, 0, -1 } };
static struct platform_data_s gyro_pdata = { 0 };
static struct platform_data_s compass_pdata = { 0 };

static void setup_gyro(void) {
	unsigned char mask = 0, lp_accel_was_on = 0;
	if (hal.sensors & ACCEL_ON)
		mask |= INV_XYZ_ACCEL;
	if (hal.sensors & GYRO_ON) {
		mask |= INV_XYZ_GYRO;
		lp_accel_was_on |= hal.lp_accel_mode;
	}
	if (hal.sensors & COMPASS_ON) {
		mask |= INV_XYZ_COMPASS;
		lp_accel_was_on |= hal.lp_accel_mode;
	}
	// If you need a power transition, this function should be called with a
	// mask of the sensors still enabled. The driver turns off any sensors
	// excluded from this mask.
	gyro_set_sensors(mask);
	gyro_configure_fifo(mask);
	if (lp_accel_was_on) {
		unsigned short rate;
		hal.lp_accel_mode = 0;
		// Switching out of LP accel, notify MPL of new accel sampling rate.
		gyro_get_sample_rate(&rate);
		inv_set_accel_sample_rate(1000000L / rate);
	}
}

static void tap_cb(unsigned char direction, unsigned char count) {
	// For Shimmer data stream
	tapAndDir = (direction << 5);
	tapAndDir |= (count & 0x1F); // Mask off lower bits - never going to get that high anyway

#ifdef SEND_INV_DRIVER_MESSAGES
	switch (direction) {
	case TAP_X_UP:
		MPL_LOGI("Tap X+ ");
		break;
	case TAP_X_DOWN:
		MPL_LOGI("Tap X- ");
		break;
	case TAP_Y_UP:
		MPL_LOGI("Tap Y+ ");
		break;
	case TAP_Y_DOWN:
		MPL_LOGI("Tap Y- ");
		break;
	case TAP_Z_UP:
		MPL_LOGI("Tap Z+ ");
		break;
	case TAP_Z_DOWN:
		MPL_LOGI("Tap Z- ");
		break;
	default:
		return;
	}
	MPL_LOGI("x%d\n", count);
#endif
	return;
}

static void display_orient_cb(unsigned char orientation) {
// These #define's assume that the chip is mounted on a handset, such that
// the Y-axis travels from the bottom to the top of the phone. Here is a
// lame attempt at ASCII art.
//  ________
// |        |
// |  /\    |
// |  ||    |
// |  ||y+  |
// |  ||    |
// |  ||    |
// |________|
//
	// For Shimmer data stream
	motAndOrient &= ~0x30; // Bits 5 to 4 - Clear
	motAndOrient |= (orientation << 4); // Bits 5 to 4 - Set

#ifdef SEND_INV_DRIVER_MESSAGES
	switch (orientation) {
	case DISPLAY_ORIENT_PORTRAIT:
		MPL_LOGI("Portrait.\n");
		break;
	case DISPLAY_ORIENT_LANDSCAPE:
		MPL_LOGI("Landscape.\n");
		break;
	case DISPLAY_ORIENT_REVERSE_PORTRAIT:
		MPL_LOGI("Reverse Portrait.\n");
		break;
	case DISPLAY_ORIENT_REVERSE_LANDSCAPE:
		MPL_LOGI("Reverse Landscape.\n");
		break;
	default:
		break;
	}
#endif
}

static void orient_cb(unsigned char orientation) {
	motAndOrient &= ~0x0F; // Bits 4 to 0 - Clear

	if (orientation & ORIENTATION_X_UP)
		motAndOrient |= (0x01 << 1); // Bits 3 to 1 - Set
#ifdef SEND_INV_DRIVER_MESSAGES
		MPL_LOGI("Orientation X+");
#endif
	else if (orientation & ORIENTATION_X_DOWN)
		motAndOrient |= (0x02 << 1); // Bits 3 to 1 - Set
#ifdef SEND_INV_DRIVER_MESSAGES
		MPL_LOGI("Orientation X-");
#endif
	else if (orientation & ORIENTATION_Y_UP)
		motAndOrient |= (0x03 << 1); // Bits 3 to 1 - Set
#ifdef SEND_INV_DRIVER_MESSAGES
		MPL_LOGI("Orientation Y+");
#endif
	else if (orientation & ORIENTATION_Y_DOWN)
		motAndOrient |= (0x04 << 1); // Bits 3 to 1 - Set
#ifdef SEND_INV_DRIVER_MESSAGES
		MPL_LOGI("Orientation Y-");
#endif
	else if (orientation & ORIENTATION_Z_UP)
		motAndOrient |= (0x05 << 1); // Bits 3 to 1 - Set
#ifdef SEND_INV_DRIVER_MESSAGES
		MPL_LOGI("Orientation Z+");
#endif
	else if (orientation & ORIENTATION_Z_DOWN)
		motAndOrient |= (0x06 << 1); // Bits 3 to 1 - Set
#ifdef SEND_INV_DRIVER_MESSAGES
		MPL_LOGI("Orientation Z-");
#endif

	if (orientation & ORIENTATION_FLIP)
		motAndOrient |= 0x01; // Bit 0 - Set
#ifdef SEND_INV_DRIVER_MESSAGES
		MPL_LOGI(" (FLIP!)\n");
	else
		MPL_LOGI("\n");
#endif
}
//static void msp430_reset(void) {
//	PMMCTL0 |= PMMSWPOR;
//}

// Every time new gyro data is available, this function is called in an
// ISR context. In this example, it sets a flag protecting the FIFO read
// function.
void gyro_data_ready_cb(void) {
	hal.new_gyro = 1;
}

// Set up MSP430 peripherals.
void MPU_platformInit(uint8_t *storedConfigPtr) {

	// turn on I2C devices on Shimmer3
	P8OUT |= BIT4;                            //set SW_I2C high to power on all I2C chips
	__delay_cycles(24000000);  //wait 1s (assuming 24MHz MCLK) to allow for power ramp up
//	// added because values aren't being set at runtime because of system_pre_init.c
//	msp430_i2c_initial();
	msp430_i2c_enable(S_MCLK,24000000,400000);

	hal.int_param.cb = gyro_data_ready_cb;
	hal.int_param.port = 1;		//-- was 2 in MotionFit
	hal.int_param.pin = 7;		//-- was 0 in MotionFit
	hal.int_param.lp_exit = INT_EXIT_LPM0;

	P1DIR &= ~BIT7;
	P1OUT &= ~BIT7;
	P1IES |= BIT7;	//look for falling edge - Libary sets the MPU9150 int to be active low - line 1631 of inv_gyro.c
//   P1IES &= ~BIT7;	//look for rising edge // http://geedesign.com/blog/?p=834#interrupts
	P1IFG &= ~BIT7;	// clear interrupt flag
	P1IE |= BIT7; // Enable Interrupt on Input Pin P1.7

	MPU9150_reset();
	msp430_delay_ms(500);		//wait 500ms (assuming 24MHz MCLK) - Don't know required time
	MPU9150_DmpInit(storedConfigPtr);
	msp430_delay_ms(500);		//wait 500ms (assuming 24MHz MCLK) - Don't know required time not known or if needed

    gyro_reset_fifo(); // Clear contents of FIFO so that first read does not fail
}

void MPU9150_DmpInit(uint8_t *storedConfigLocal) {
	inv_error_t result;

	unsigned short lpf, nyquist_rate;
    uint8_t i = 0;
//    float sampling_rate;

	MPU_initStructures();

	if((storedConfigLocal[NV_CONFIG_SETUP_BYTE2] & 0x03)==MPU9150_GYRO_250DPS)
		gyro_fsr = 250;
	else if((storedConfigLocal[NV_CONFIG_SETUP_BYTE2] & 0x03)==MPU9150_GYRO_500DPS)
		gyro_fsr = 500;
	else if((storedConfigLocal[NV_CONFIG_SETUP_BYTE2] & 0x03)==MPU9150_GYRO_1000DPS)
		gyro_fsr = 1000;
	else// if((storedConfigLocal[NV_CONFIG_SETUP_BYTE2] & 0x03)==MPU9150_GYRO_2000DPS)
		gyro_fsr = 2000;

	if(((storedConfigLocal[NV_CONFIG_SETUP_BYTE3]>>6) & 0x03)==RANGE_2G)
		accel_fsr = 2;
	else if(((storedConfigLocal[NV_CONFIG_SETUP_BYTE3]>>6) & 0x03)==RANGE_4G)
		accel_fsr = 4;
	else if(((storedConfigLocal[NV_CONFIG_SETUP_BYTE3]>>6) & 0x03)==RANGE_8G)
		accel_fsr = 8;
	else// if(((storedConfigLocal[NV_CONFIG_SETUP_BYTE3]>>6) & 0x03)==RANGE_16G)
		accel_fsr = 16;

	if(((storedConfigLocal[NV_CONFIG_SETUP_BYTE5]>>5) & 0x07) == MPL_RATE_10HZ) {
		MPU_RATE_HZ = 10;	// sampling freq to 10
	}
	else if(((storedConfigLocal[NV_CONFIG_SETUP_BYTE5]>>5) & 0x07) == MPL_RATE_20HZ) {
		MPU_RATE_HZ = 20;	// sampling freq to 20
	}
	else if(((storedConfigLocal[NV_CONFIG_SETUP_BYTE5]>>5) & 0x07) == MPL_RATE_40HZ) {
		MPU_RATE_HZ = 40;	// sampling freq to 40
	}
	else if(((storedConfigLocal[NV_CONFIG_SETUP_BYTE5]>>5) & 0x07) == MPL_RATE_50HZ) {
		MPU_RATE_HZ = 50;	// sampling freq to 50
	}
	else {// if(((storedConfigLocal[NV_CONFIG_SETUP_BYTE5]>>5) & 0x07) == MPL_RATE_100HZ) {
		MPU_RATE_HZ = 100;	// sampling freq to 100
	}

	if(((storedConfigLocal[NV_CONFIG_SETUP_BYTE5]>>2) & 0x07) == MPL_RATE_10HZ) {
		MAG_RATE_MS = 100;	// sampling freq to 10
	}
	else if(((storedConfigLocal[NV_CONFIG_SETUP_BYTE5]>>2) & 0x07) == MPL_RATE_20HZ) {
		MAG_RATE_MS = 50;	// sampling freq to 20
	}
	else if(((storedConfigLocal[NV_CONFIG_SETUP_BYTE5]>>2) & 0x07) == MPL_RATE_40HZ) {
		MAG_RATE_MS = 25;	// sampling freq to 40
	}
	else if(((storedConfigLocal[NV_CONFIG_SETUP_BYTE5]>>2) & 0x07) == MPL_RATE_50HZ) {
		MAG_RATE_MS = 20;	// sampling freq to 50
	}
	else {// if(((storedConfigLocal[NV_CONFIG_SETUP_BYTE5]>>2) & 0x07) == MPL_RATE_100HZ) {
		MAG_RATE_MS = 10;	// sampling freq to 100
	}

	if(((storedConfigLocal[NV_CONFIG_SETUP_BYTE4]>>3) & 0x07)==MPU9150_LPF_5HZ)
		lpf = 5;
	else if(((storedConfigLocal[NV_CONFIG_SETUP_BYTE4]>>3) & 0x07)==MPU9150_LPF_10HZ)
		lpf = 10;
	else if(((storedConfigLocal[NV_CONFIG_SETUP_BYTE4]>>3) & 0x07)==MPU9150_LPF_20HZ)
		lpf = 20;
	else if(((storedConfigLocal[NV_CONFIG_SETUP_BYTE4]>>3) & 0x07)==MPU9150_LPF_42HZ)
		lpf = 42;
	else if(((storedConfigLocal[NV_CONFIG_SETUP_BYTE4]>>3) & 0x07)==MPU9150_LPF_98HZ)
		lpf = 98;
	else if(((storedConfigLocal[NV_CONFIG_SETUP_BYTE4]>>3) & 0x07)==MPU9150_LPF_188HZ)
		lpf = 188;
	else // if(((storedConfigLocal[NV_CONFIG_SETUP_BYTE4]>>3) & 0x07)==MPU9150_LPF_256HZ_NOLPF)
		lpf = 0;

//	// set LPF cut-off to be half of sampling if set above nquist rate
//	nyquist_rate = MPU_RATE_HZ / 2;
//	if(lpf>=nyquist_rate)
//		lpf = nyquist_rate;

//TODO: configure LSM mag if selected
//	uint8_t lsm303dlhc_mag_range = ((sdhead_text[SDH_CONFIG_SETUP_BYTE2]>>5)&0x07);

//TODO: configure LSM (orientation, range, rate etc.) and pass readings back to main loop
	if((storedConfigLocal[NV_SENSORS3] & SENSOR_MPU9150_MPL_QUAT_9DOF) || (storedConfigLocal[NV_SENSORS3] & SENSOR_MPU9150_MPL_EULER_9DOF)){
		hal.pansenti_nineDOF = 1;

		// configure magMix from Pansenti 9DOF implementation
		if((storedConfigLocal[NV_CONFIG_SETUP_BYTE5] & 0x03)==GYRO_ONLY)
			m_magMix = MPU_MAG_MIX_GYRO_ONLY;                  // just use gyro yaw
		else if((storedConfigLocal[NV_CONFIG_SETUP_BYTE5] & 0x03)==MAG_ONLY)
			m_magMix = MPU_MAG_MIX_MAG_ONLY;                   // just use magnetometer and no gyro yaw
		else if((storedConfigLocal[NV_CONFIG_SETUP_BYTE5] & 0x03)==GYRO_AND_MAG)
			m_magMix = MPU_MAG_MIX_GYRO_AND_MAG;               // a good mix value
		else// if((storedConfigLocal[NV_CONFIG_SETUP_BYTE5] & 0x03)==GYRO_AND_SOME_MAG)
			m_magMix = MPU_MAG_MIX_GYRO_AND_SOME_MAG;			// mainly gyros with a bit of mag correction
	}
	else{
		hal.pansenti_nineDOF = 0;
//		hal.nineDOF_use_LSM = 0;
		m_magMix = 0; // just set here to give it a fixed value
	}

	if(storedConfigLocal[NV_CONFIG_SETUP_BYTE4] & MPU9150_MPL_USE_LSM303_MAG){
		hal.nineDOF_use_LSM = 1;
		for(i=0;i<9;i++){
			if(storedConfigLocal[NV_LSM303DLHC_MAG_CALIBRATION+i+12] == 156)
				compass_pdata.orientation[i] = -1;
			else if(storedConfigLocal[NV_LSM303DLHC_MAG_CALIBRATION+i+12] == 100)
				compass_pdata.orientation[i] = 1;
			else
				compass_pdata.orientation[i] = 0;
		}
	}
	else{
		hal.nineDOF_use_LSM = 0;
		for(i=0;i<9;i++){
			if(storedConfigLocal[NV_MPL_MAG_CALIBRATION+i+12] == 156)
				compass_pdata.orientation[i] = -1;
			else if(storedConfigLocal[NV_MPL_MAG_CALIBRATION+i+12] == 100)
				compass_pdata.orientation[i] = 1;
			else
				compass_pdata.orientation[i] = 0;
		}
	}

	for(i=0;i<9;i++){
		if(storedConfigLocal[NV_MPL_ACCEL_CALIBRATION+i+12] == 156)
			gyro_pdata.orientation[i] = -1;
		else if(storedConfigLocal[NV_MPL_ACCEL_CALIBRATION+i+12] == 100)
			gyro_pdata.orientation[i] = 1;
		else
			gyro_pdata.orientation[i] = 0;
	}

	// Set up gyro.
	// Every function preceded by gyro_ is a driver function and can be found
	// in inv_gyro.h.
    result = gyro_init(&hal.int_param, gyro_fsr, accel_fsr, lpf, MPU_RATE_HZ, MAG_RATE_MS);
	if (result) {
#ifdef SEND_INV_DRIVER_MESSAGES
		MPL_LOGE("Could not initialize gyro.\n");
#endif
//--		msp430_reset();
	}

	// If you're not using an MPU9150 AND you're not using DMP features, this
	// function will place all slaves on the primary bus.
	// gyro_set_bypass(1);

	result = inv_init_mpl();
	if (result) {
#ifdef SEND_INV_DRIVER_MESSAGES
		MPL_LOGE("Could not initialize MPL.\n");
#endif
//--		msp430_reset();
	}

	// Compute 6-axis and 9-axis quaternions.
	inv_enable_quaternion();
	if(storedConfigLocal[NV_CONFIG_SETUP_BYTE6] & MPU9150_MPL_SENS_FUSION) {
		inv_enable_9x_sensor_fusion();
	}
	// The MPL expects compass data at a constant rate (matching the rate
	// passed to inv_set_compass_sample_rate). If this is an issue for your
	// application, call this function, and the MPL will depend on the
	// timestamps passed to inv_build_compass instead.
	inv_9x_fusion_use_timestamps(1);

	// Update gyro biases when not in motion.
	// WARNING: These algorithms are mutually exclusive.
	// use either (fast_nomot) or (motion_no_motion and motion_time)?
	if((storedConfigLocal[NV_CONFIG_SETUP_BYTE4] & 0x07)==MPL_MOT_CAL_OFF){

	}
	else if((storedConfigLocal[NV_CONFIG_SETUP_BYTE4] & 0x07)==MPL_MOT_CAL_MOT_NO_MOT_1S){
		inv_enable_motion_no_motion();
		inv_set_no_motion_time(1000L);
	}
	else if((storedConfigLocal[NV_CONFIG_SETUP_BYTE4] & 0x07)==MPL_MOT_CAL_MOT_NO_MOT_2S){
		inv_enable_motion_no_motion();
		inv_set_no_motion_time(2000L);
	}
	else if((storedConfigLocal[NV_CONFIG_SETUP_BYTE4] & 0x07)==MPL_MOT_CAL_MOT_NO_MOT_5S){
		inv_enable_motion_no_motion();
		inv_set_no_motion_time(5000L);
	}
	else if((storedConfigLocal[NV_CONFIG_SETUP_BYTE4] & 0x07)==MPL_MOT_CAL_MOT_NO_MOT_10S){
		inv_enable_motion_no_motion();
		inv_set_no_motion_time(10000L);
	}
	else if((storedConfigLocal[NV_CONFIG_SETUP_BYTE4] & 0x07)==MPL_MOT_CAL_MOT_NO_MOT_30S){
		inv_enable_motion_no_motion();
		inv_set_no_motion_time(30000L);
	}
	else if((storedConfigLocal[NV_CONFIG_SETUP_BYTE4] & 0x07)==MPL_MOT_CAL_MOT_NO_MOT_60S){
		inv_enable_motion_no_motion();
		inv_set_no_motion_time(60000L);
	}
	else{// if((storedConfigLocal[NV_CONFIG_SETUP_BYTE4] & 0x07)==MPL_MOT_CAL_FAST_NO_MOT){
		inv_enable_fast_nomot();
	}

	/* Update gyro biases when temperature changes. */
	if(storedConfigLocal[NV_CONFIG_SETUP_BYTE6] & MPU9150_MPL_GYRO_CAL_TC) {
		inv_enable_gyro_tc();
	}

	// This algorithm updates the accel biases when in motion. A more accurate
	// bias measurement can be made when running the self-test (see case 't' in
	// handle_input), but this algorithm can be enabled if the self-test can't
	// be executed in your application.
	// inv_enable_in_use_auto_calibration();

	// Compass calibration algorithms.
	if(storedConfigLocal[NV_CONFIG_SETUP_BYTE6] & MPU9150_MPL_VECT_COMP_CAL) {
		inv_enable_vector_compass_cal();
	}
	if(storedConfigLocal[NV_CONFIG_SETUP_BYTE6] & MPU9150_MPL_MAG_DIST_CAL) {
		inv_enable_magnetic_disturbance();
	}
	// If you need to estimate your heading before the compass is calibrated,
	// enable this algorithm. It becomes useless after a good figure-eight is
	// detected, so we'll just leave it out to save memory.
	// inv_enable_heading_from_gyro();

	// Allows use of the MPL APIs in read_from_mpl.
	inv_enable_eMPL_outputs();

	result = inv_start_mpl();
	if (result == INV_ERROR_NOT_AUTHORIZED) {
		while (1) {
			// Not authorized.
			SET_RED_LED;
			SET_BLUE_LED;
			msp430_delay_ms(5000);
			CLEAR_RED_LED;
			CLEAR_BLUE_LED;
			msp430_delay_ms(5000);
		}
	}
	if (result) {
		// Could not start the MPL.
//--		msp430_reset();
	}

	// Get/set hardware configuration. Start gyro.
	// Wake up all sensors.
	gyro_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
	// Push both gyro and accel data into the FIFO.
	gyro_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
	gyro_set_sample_rate(MPU_RATE_HZ);
	// The compass sampling rate can be less than the gyro/accel sampling rate.
	// Use this function for proper power management.
	gyro_set_compass_sample_rate(1000 / MAG_RATE_MS);

	// Read back configuration in case it was set improperly.
	gyro_get_sample_rate(&gyro_rate);
	gyro_get_gyro_fsr(&gyro_fsr);
	gyro_get_accel_fsr(&accel_fsr);
	gyro_get_compass_fsr(&compass_fsr);

	// set accel full scale range (2, 4, 8 or 16)
	// gyro_set_accel_fsr();

	if(storedConfigLocal[NV_CONFIG_SETUP_BYTE6] & MPU9150_MPL_ENABLE){
		// Sync driver configuration with MPL.
		// Sample rate expected in microseconds.
		inv_set_gyro_sample_rate(1000000L / gyro_rate);
		inv_set_accel_sample_rate(1000000L / gyro_rate);
		// The compass rate is independent of the gyro and accel rates. As long as
		// inv_set_compass_sample_rate is called with the correct value, the 9-axis
		// fusion algorithm's compass correction gain will work properly.
		inv_set_compass_sample_rate(MAG_RATE_MS * 1000L);

		// here for testing passing actual Shimmer sampling rate to library
		//	sampling_rate = (32768 * 1.0) / (*(uint16_t *)(storedConfigLocal+NV_SAMPLING_RATE));
		//	sampling_rate = ceil(sampling_rate);
		//	inv_set_gyro_sample_rate(1000000L / sampling_rate);
		//	inv_set_accel_sample_rate(1000000L / sampling_rate);
		//	inv_set_compass_sample_rate(1000000L / sampling_rate);
	}

	//TODO: set quat rate:
	//  inv_set_quat_sample_rate();

	// Set chip-to-body orientation matrix.
	// Set hardware units to dps/g's/degrees scaling factor.
	inv_set_gyro_orientation_and_scale(
			inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
			(long) gyro_fsr << 15);
	inv_set_accel_orientation_and_scale(
			inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
			(long) accel_fsr << 15);
	inv_set_compass_orientation_and_scale(
			inv_orientation_matrix_to_scalar(compass_pdata.orientation),
			(long) compass_fsr << 15);

	// Initialize HAL state variables.
	hal.sensors = ACCEL_ON | GYRO_ON | COMPASS_ON;
	hal.dmp_on = 0;
	hal.report = 0;
	hal.rx.cmd = 0;
	hal.next_pedo_ms = 0;
	hal.new_compass = 0;
//--	hal.pansenti_nineDOF = 0;
//--	hal.nineDOF_use_LSM = 0;

	// Compass reads are handled by scheduler.
	msp430_get_clock_ms(&timestamp);
	hal.read_compass_ms = MAG_RATE_MS;
	hal.next_compass_ms = timestamp + hal.read_compass_ms;

	// To initialize the DMP:
	// 1. Call dmp_load_android_firmware(). This pushes the DMP image in
	//    inv_gyro_dmp_android.h into the MPU memory.
	// 2. Push the gyro and accel orientation matrix to the DMP.
	// 3. Register gesture callbacks. Don't worry, these callbacks won't be
	//    executed unless the corresponding feature is enabled.
	// 4. Call dmp_enable_feature(mask) to enable different features.
	// 5. Call dmp_set_fifo_rate(freq) to select a DMP output rate.
	// 6. Call any feature-specific control functions.
	//
	// To enable the DMP, just call gyro_set_dmp_state(1). This function can
	// be called repeatedly to enable and disable the DMP at runtime.
	//
	// The following is a short summary of the features supported in the DMP
	// image provided in inv_gyro_dmp_android.c:
	// DMP_FEATURE_LP_QUAT: Generate a gyro-only quaternion on the DMP at
	// 200Hz. Integrating the gyro data at higher rates reduces numerical
	// errors (compared to integration on the MCU at a lower sampling rate).
	// DMP_FEATURE_TAP: Detect taps along the X, Y, and Z axes.
	// DMP_FEATURE_ORIENT: Notify the application when the device orientation
	// has changed.
	// DMP_FEATURE_DISPLAY_ORIENT: Google's screen rotation algorithm. Similar
	// to the feature above, but only triggers an event at the four
	// orientations where the screen should rotate.
	dmp_load_android_firmware();
	dmp_set_orientation(
			inv_orientation_matrix_to_scalar(gyro_pdata.orientation));
	dmp_register_tap_cb(tap_cb);
	dmp_register_display_orient_cb(display_orient_cb);
	dmp_register_orient_cb(orient_cb);
	// These features can be enabled/disabled at runtime. If you'd like to
	// try it, here's an example:
	// void toggle_display_orient(void) {
	//     hal.dmp_features ^= DMP_FEATURE_DISPLAY_ORIENT;
	//     dmp_enable_feature(hal.dmp_features);
	// }
	hal.dmp_features = DMP_FEATURE_6X_LP_QUAT;

	if(storedConfigLocal[NV_SENSORS3] & SENSOR_MPU9150_MPL_TAP) {
		hal.dmp_features |= DMP_FEATURE_TAP;
	}
	if(storedConfigLocal[NV_SENSORS3] & SENSOR_MPU9150_MPL_PEDOMETER) {
		hal.dmp_features |= DMP_FEATURE_PEDOMETER;
	}
	if(storedConfigLocal[NV_SENSORS3] & SENSOR_MPU9150_MPL_TAP) {
		hal.dmp_features |= DMP_FEATURE_TAP;
	}
	if(storedConfigLocal[NV_SENSORS3] & SENSOR_MPU9150_MPL_MOTION_ORIENT) {
		hal.dmp_features |= DMP_FEATURE_ORIENT | DMP_FEATURE_DISPLAY_ORIENT;
	}

	dmp_enable_feature(hal.dmp_features);
	dmp_set_fifo_rate(MPU_RATE_HZ);
	hal.read_pedo_ms = 1000;

	// Initiate values for Pansenti 9DOF fusion code
	m_lastDMPYaw = 0;                                       // the last yaw from the DMP gyros
	m_lastYaw = 0;                                          // last calculated output yaw
	//m_magMix = MPU_MAG_MIX_MAG_ONLY;                                             // controls gyro and magnetometer mixing for yaw

	// the following taken from the original handle_input() function in the MPL library
	if(storedConfigLocal[NV_CONFIG_SETUP_BYTE4] & MPU9150_MPL_DMP) {
		hal.dmp_on = 1;
		dmp_set_fifo_rate(gyro_rate);
		gyro_set_dmp_state(1);
	}
	if((storedConfigLocal[NV_SENSORS4] & SENSOR_MPU9150_GYRO_CAL) || (storedConfigLocal[NV_SENSORS4] & SENSOR_MPU9150_ACCEL_CAL) || (storedConfigLocal[NV_SENSORS4] & SENSOR_MPU9150_MAG_CAL)) {
		hal.report ^= PRINT_ACCEL;	// generate calibrated accel values
		hal.report ^= PRINT_GYRO;	// generate calibrated gyro values
		hal.report ^= PRINT_COMPASS;	// generate calibrated compass values
	}
	if(storedConfigLocal[NV_SENSORS3] & SENSOR_MPU9150_MPL_EULER_6DOF) {
		hal.report ^= PRINT_EULER;	// generate 6DOF Euler
	}
	if(storedConfigLocal[NV_SENSORS3] & SENSOR_MPU9150_MPL_HEADING) {
		hal.report ^= PRINT_HEADING;	// turn on heading output
	}
	if(storedConfigLocal[NV_SENSORS3] & SENSOR_MPU9150_MPL_PEDOMETER) {
		// Toggle pedometer display.
		hal.report ^= PRINT_PEDOM;
	}

	// Calibration loading: in the order below, the *.ini file has preference over *.dat file. This is done so that the
	// user can overwrite the initial calibration values in the ini file.
	// Load previously saved InvenSense calibration values if the file exists (i.e., *.dat file).
	if(calibrationBytesFilePresent) {
		inv_load_mpl_states(&mplCaliBuffer[0], caliStoreSize);
		inv_accel_was_turned_off();
		inv_gyro_was_turned_off();
		inv_compass_was_turned_off();
	}
	// If calibration file (i.e., *.ini) is not present default values are loaded.
	inv_set_accel_bias(accelBias, 3);
	inv_set_gyro_bias(gyroBias, 3);
	inv_set_compass_bias(magBias, 3);
	inv_set_compass_bias_found(1);
}

void MPL_getValue(uint16_t sensor, uint8_t *localBuf){
	uint8_t *local_ptr;
	uint8_t axes_cnt, i, bytes, swap_end;

	axes_cnt = 3;
	bytes = 0;
	swap_end = 1;

	if(sensor==PRINT_GYRO_RAW){
		local_ptr = (uint8_t *)&gyroRaw[0];
		bytes = 2;
	}
	else if(sensor==PRINT_ACCEL_RAW){
		local_ptr = (uint8_t *)&accelRaw[0];
		bytes = 2;
	}
	else if(sensor==PRINT_COMPASS_RAW){
		local_ptr = (uint8_t *)&compassRaw[0];
		bytes = 2;
		swap_end = 0;// don't swap endian
	}
	else if(sensor==PRINT_QUAT){
		local_ptr = (uint8_t *)&quatMpl[0];
		axes_cnt = 4;
		bytes = 4;
	}
	//TODO: QUAT 9DOF
	else if(sensor==PRINT_QUAT_9DOF){
		local_ptr = (uint8_t *)&quat9DOF[0];
		axes_cnt = 4;
		bytes = 4;
	}
	else if(sensor==PRINT_EULER){
		local_ptr = (uint8_t *)&euler6DOF[0];
		bytes = 4;
	}
	//TODO: EULER 9DOF
	else if(sensor==PRINT_EULER_9DOF){
		local_ptr = (uint8_t *)&euler9DOF[0];
		axes_cnt = 4;
		bytes = 4;
	}
	else if(sensor==PRINT_HEADING){
		local_ptr = (uint8_t *)&heading;
		axes_cnt = 1;
		bytes = 4;
	}
	else if(sensor==PRINT_TEMPERATURE){
		local_ptr = (uint8_t *)&temperature;
		axes_cnt = 1;
		bytes = 4;
	}
	else if(sensor==PRINT_PEDOM){
		local_ptr = (uint8_t *)&pedomBuffer[0];
		axes_cnt = 2;
		bytes = 4;
	}
	else if(sensor==PRINT_TAP_AND_DIR){
		local_ptr = (uint8_t *)&tapAndDir;
		bytes = 1;
	}
	else if(sensor==PRINT_MOT_AND_ORIENT){
		local_ptr = (uint8_t *)&motAndOrient;
		bytes = 1;
	}
	else if(sensor==PRINT_GYRO){
		local_ptr = (uint8_t *)&gyroMpl[0];
		bytes = 4;
	}
	else if(sensor==PRINT_ACCEL){
		local_ptr = (uint8_t *)&accelMpl[0];
		bytes = 4;
	}
	else if(sensor==PRINT_COMPASS){
		local_ptr = (uint8_t *)&compassMpl[0];
		bytes = 4;
	}
	else if(sensor==PRINT_QUAT_RAW){
		local_ptr = (uint8_t *)&quatRaw[0];
		axes_cnt = 4;
		bytes = 4;
	}

	// Load buffer -swap endian - higher byte first
	if(bytes == 4){
		for(i=0;i<axes_cnt;i++) {
			*(localBuf+(4*i)) = *(local_ptr+(4*i)+3);
			*(localBuf+(4*i)+1) = *(local_ptr+(4*i)+2);
			*(localBuf+(4*i)+2) = *(local_ptr+(4*i)+1);
			*(localBuf+(4*i)+3) = *(local_ptr+(4*i));
		}
	}
	else if(bytes == 2){
		for(i=0;i<axes_cnt;i++) {
			if(swap_end==1) {
				*(localBuf+(2*i)) = *(local_ptr+(2*i)+1);
				*(localBuf+(2*i)+1) = *(local_ptr+(2*i));
			}
			else {	// don't swap endian
				*(localBuf+(2*i)) = *(local_ptr+(2*i));
				*(localBuf+(2*i)+1) = *(local_ptr+(2*i)+1);
			}
		}
	}
	else if(bytes == 1){
		*localBuf = *local_ptr;
	}
	else{
		_NOP();	// do nothing
	}
}

int MPU_getDmpData(void){
	new_data = 0;

	msp430_get_clock_ms(&timestamp);

	// We're not using a data ready interrupt for the compass, so we'll
	// make our compass reads timer-based instead.
	if ((timestamp > hal.next_compass_ms) && !hal.lp_accel_mode
			&& hal.new_gyro && (hal.sensors & COMPASS_ON)) {
		hal.next_compass_ms = timestamp + hal.read_compass_ms;
		hal.new_compass = 1;
	}

	if (hal.new_gyro && hal.dmp_on) {
		short sensors;
		unsigned char more;
		long accel[3];
		// This function gets new data from the FIFO when the DMP is in
		// use. The FIFO can contain any combination of gyro, accel,
		// quaternion, and gesture data. The sensors parameter tells the
		// caller which data fields were actually populated with new data.
		// For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
		// the FIFO isn't being filled with accel data.
		// The driver parses the gesture data to determine if a gesture
		// event has occured; on an event, the application will be notified
		// via a callback (assuming that a callback function was properly
		// registered).
		// The more parameter is non-zero if there are leftover packets in
		// the FIFO. The HAL can use this information to increase the
		// frequency at which this function is called.

//		dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp,
//				&sensors, &more);
//		if (!more)
//			hal.new_gyro = 0;

		// MPL modified to clear buffer on each read
		// see modifications in gyro_read_fifo_stream() in inv_gyro.c
		hal.new_gyro = 0;
		if(dmp_read_fifo(gyroRaw, accelRaw, quatRaw, &sensor_timestamp,
				&sensors, &more)){
			return 1;
		}

		if (sensors & INV_XYZ_GYRO) {
			// Push the new data to the MPL.
			inv_build_gyro(gyroRaw, sensor_timestamp);
			new_data = 1;
			// Temperature only used for gyro temp comp.
			gyro_get_temperature(&temperature, &sensor_timestamp);
			inv_build_temp(temperature, sensor_timestamp);
		}
		if (sensors & INV_XYZ_ACCEL) {
			accel[0] = (long) accelRaw[0];
			accel[1] = (long) accelRaw[1];
			accel[2] = (long) accelRaw[2];
			inv_build_accel(accel, 0, sensor_timestamp);
			new_data = 1;
		}
		if (sensors & INV_WXYZ_QUAT) {
			inv_build_quat(quatRaw, 0, sensor_timestamp);
			new_data = 1;
		}
	}
	else if (hal.new_gyro) {
		short gyro[3], accel_short[3];
		unsigned char sensors, more;
		long accel[3], temperature;
		// This function gets new data from the FIFO. The FIFO can contain
		// gyro, accel, both, or neither. The sensors parameter tells the
		// caller which data fields were actually populated with new data.
		// For example, if sensors == INV_XYZ_GYRO, then the FIFO isn't
		// being filled with accel data. The more parameter is non-zero if
		// there are leftover packets in the FIFO. The HAL can use this
		// information to increase the frequency at which this function is
		// called.
//		gyro_read_fifo(gyro, accel_short, &sensor_timestamp, &sensors,
//				&more);

		// MPL modified to clear buffer on each read
		// see modifications in gyro_read_fifo_stream() in inv_gyro.c
		if(gyro_read_fifo(gyro, accel_short, &sensor_timestamp, &sensors,
				&more)){
			return 1;
		}
		if (!more)
			hal.new_gyro = 0;

		if (sensors & INV_XYZ_GYRO) {
			// Push the new data to the MPL.
			inv_build_gyro(gyro, sensor_timestamp);
			new_data = 1;
			// Temperature only used for gyro temp comp.
			gyro_get_temperature(&temperature, &sensor_timestamp);
			inv_build_temp(temperature, sensor_timestamp);
		}
		if (sensors & INV_XYZ_ACCEL) {
			accel[0] = (long) accel_short[0];
			accel[1] = (long) accel_short[1];
			accel[2] = (long) accel_short[2];
			inv_build_accel(accel, 0, sensor_timestamp);
			new_data = 1;
		}
	}
	else {
		return 1;
	}

	if (hal.new_compass) {
		long compass[3];
		hal.new_compass = 0;
		// For the MPU9150 (or the MPU6050 with an AKM on the auxiliary I2C
		// bus), the raw magnetometer registers are copied to special gyro
		// registers.

//		if(hal.nineDOF_use_LSM) {
//			LSM303DLHC_getMag(compass);
//			inv_build_compass(compass, INV_CALIBRATED | 0 , sensor_timestamp);
//		}
//		else {
		if (!gyro_get_compass_reg(compassRaw, &sensor_timestamp)) {
//			gyro_get_compass_reg(compass_short, &sensor_timestamp);
			compass[0] = (long) compassRaw[0];
			compass[1] = (long) compassRaw[1];
			compass[2] = (long) compassRaw[2];
			/* NOTE: If using a third-party compass calibration library,
			 * pass in the compass data in uT * 2^16 and set the second
			 * parameter to INV_CALIBRATED | acc, where acc is the
			 * accuracy from 0 to 3.
			 */
			inv_build_compass(compass, 0, sensor_timestamp);
		}
		new_data = 1;
	}
	if (new_data) {
		inv_execute_on_data();
		// This function reads bias-compensated sensor data and sensor
		// fusion outputs from the MPL. The outputs are formatted as seen
		// in eMPL_outputs.c. This function only needs to be called at the
		// rate requested by the host.
		read_from_mpl();
	}

	return 0;

}

int MPL_gyroCalibrate(void){
	int result;
	unsigned short accel_sens;
	float gyro_sens;

	// duplicated from case 't' in handle_input() but need to know the contents of 'result' here
	result = gyro_run_self_test(gyroBias, accelBias);
	if (!result) {
//		 // allows self-test when MPU is aligned upside down
//		if(gyro_pdata.orientation[8] == -1){
//			accelBias[2] += 2 * 65536;
//		}
		// MPL expects biases in hardware units << 16, but self test returns
		// biases in g's << 16.
		gyro_get_accel_sens(&accel_sens);
		accelBias[0] *= accel_sens;
		accelBias[1] *= accel_sens;
		accelBias[2] *= accel_sens;
		inv_set_accel_bias(accelBias, 3);

		gyro_get_gyro_sens(&gyro_sens);
		gyroBias[0] = (long) (gyroBias[0] * gyro_sens);
		gyroBias[1] = (long) (gyroBias[1] * gyro_sens);
		gyroBias[2] = (long) (gyroBias[2] * gyro_sens);
		inv_set_gyro_bias(gyroBias, 3);
	}
	return result;
}

int MPL_magCalibrate(void){
	#define TimeoutMS 120000 // 120s timeout
	#define SampDelayMS 10 // 1 / 0.01 = 100 Hz
	uint16_t i;
	magBias[0] = 0;
	magBias[1] = 0;
	magBias[2] = 0;

	inv_set_compass_bias(&magBias[0], 0);
	inv_set_compass_bias_found(0);

	for(i=1;i<=(TimeoutMS/SampDelayMS);i++){
		MPU_getDmpData();
		msp430_delay_ms(SampDelayMS);
		if(inv_got_compass_bias()){
			inv_get_compass_bias(&magBias[0]);
			return 0;
		}
		if(i == (TimeoutMS/SampDelayMS))
			return 1;
	}
	return 0;
}

uint8_t MPL_saveCalibrationBytes(void){
	size_t store_size;
    FRESULT rc;	// Result code
    uint8_t cal_file[36];
	UINT num_bytes_written = 0;
	DIRS gdc;
	FIL gfc;

	unsigned char mpl_states[110]; // was 100 in MPL ex.
	inv_get_mpl_state_size(&store_size);
	inv_save_mpl_states(mpl_states, store_size);

	if(rc = f_opendir(&gdc, "/Calibration\0")){
		if(rc == FR_NO_PATH)      // need to create folder first
			if(rc = f_mkdir("/Calibration\0"))
				return 1;
		if(rc)
			return 1;
	}
	if(rc = f_opendir(&gdc, "/Calibration/MPL\0")){
		if(rc == FR_NO_PATH)      // need to create folder first
			if(rc = f_mkdir("/Calibration/MPL\0"))
				return 1;
		if(rc)
			return 1;
	}

	strcpy((char*)cal_file, "/Calibration/MPL/MPLcalibBytes.dat\0");
	if(rc = f_open(&gfc, (char*)cal_file, FA_WRITE | FA_CREATE_ALWAYS)){
		_NOP();
	}
	else{
        // Write to file
		rc = f_write(&gfc, &mpl_states[0], store_size, &num_bytes_written);
		rc = f_close(&gfc);
		inv_accel_was_turned_off();
		inv_gyro_was_turned_off();
		inv_compass_was_turned_off();
		return 0;
	}

	inv_accel_was_turned_off();
	inv_gyro_was_turned_off();
	inv_compass_was_turned_off();
	return 1;
}

uint8_t MPL_loadCalibrationBytes(void){
   FRESULT rc;	// Result code
	uint8_t cal_file[35];
	UINT num_bytes_read = 0; //past-tense

	DIRS gdc;
	FIL gfc;

	calibrationBytesFilePresent = 0;

	inv_get_mpl_state_size(&caliStoreSize);

	strcpy((char*)cal_file, "/Calibration/MPL/MPLcalibBytes.dat\0");
	if(rc = f_opendir(&gdc, "/Calibration/MPL\0")){
		return 1;
	}
	else if(rc = f_open(&gfc, (char*)cal_file, (FA_OPEN_EXISTING | FA_READ))){
		 // no calibration file
		return 1;
	}
	else{
		if(rc = f_read(&gfc, &mplCaliBuffer[0], caliStoreSize, &num_bytes_read)){
			return 1;
		}
		else{
			if(num_bytes_read == caliStoreSize){
				calibrationBytesFilePresent = 1;
				rc = f_close(&gfc);
				return 0;
			}
		}
		rc = f_close(&gfc);
	}
	return 1;
}

uint8_t MPU_saveCalibration(void){
    FRESULT rc;    // Result code
    uint8_t cal_file[34];
	int num_bytes_written = 0;
	unsigned char accel_g;
	unsigned short gyro_dps;
//	unsigned short mag_Ga;
	char text_buffer[20];
	uint8_t i,j,k;
	float gyro_sens;
	unsigned short accel_sens;
	long temp;

	DIRS gdc;
	FIL gfc;

	for(i=1;i<=20;i++){
		text_buffer[i] = 0;
	}

	if(rc = f_opendir(&gdc, "/Calibration\0")){
		if(rc == FR_NO_PATH)      // need to create folder first
			if(rc = f_mkdir("/Calibration\0"))
				return 1;
		if(rc)
			return 1;
	}
	if(rc = f_opendir(&gdc, "/Calibration/MPL\0")){
		if(rc == FR_NO_PATH)      // need to create folder first
			if(rc = f_mkdir("/Calibration/MPL\0"))
				return 1;
		if(rc)
			return 1;
	}

	strcpy((char*)cal_file, "/Calibration/MPL/MPUcalib.ini\0");
	if(rc = f_open(&gfc, (char*)cal_file, FA_WRITE | FA_CREATE_ALWAYS)){
		_NOP();
	}
	else{
		// MPL Accelerometer calibration
		gyro_get_accel_fsr(&accel_g);
		sprintf(text_buffer, "[MPL Accel %dg]\r\n", accel_g);
		num_bytes_written = f_puts(text_buffer, &gfc);        // Write to file
		inv_get_accel_bias(&accelBias[0], temp);
		for(i=0;i<=2;i++){
			sprintf(text_buffer, "b%u = %f\r\n", i, (float)(accelBias[i]));	// Offset
//			sprintf(text_buffer, "b%u = %f\r\n", i, (float)(accelBias[i]) / 65536);	// Offset
			num_bytes_written = f_puts(text_buffer, &gfc);        // Write to file
		}
		gyro_get_accel_sens(&accel_sens);
		for(i=0;i<=2;i++){
			sprintf(text_buffer, "k%u = %u\r\n", i, accel_sens);		// Sensitivity
//			sprintf(text_buffer, "k%u = %u\r\n", i, accel_sensit);
			num_bytes_written = f_puts(text_buffer, &gfc);        // Write to file
		}
		k=0;
		for(i=0;i<=2;i++){										// Alignment
			for(j=0;j<=2;j++){
				sprintf(text_buffer, "r%u%u = %f\r\n", i, j, (float)(gyro_pdata.orientation[k]));
				num_bytes_written = f_puts(text_buffer, &gfc);        // Write to file
				k += 1;
			}
		}
		num_bytes_written = f_puts("\r\n", &gfc);        // Write to file

		// MPL Gyroscope calibration
		gyro_get_gyro_fsr(&gyro_dps);
		sprintf(text_buffer, "[MPL Gyro %ddps]\r\n", gyro_dps);
		num_bytes_written = f_puts(text_buffer, &gfc);        // Write to file
		inv_get_gyro_bias(&gyroBias[0], temp);
		for(i=0;i<=2;i++){
			sprintf(text_buffer, "b%u = %f\r\n", i, (float)(gyroBias[i]));	// Offset
//			sprintf(text_buffer, "b%u = %f\r\n", i, (float)(gyroBias[i]) / 65536);	// Offset
			num_bytes_written = f_puts(text_buffer, &gfc);        // Write to file
		}
		gyro_get_gyro_sens(&gyro_sens);
		for(i=0;i<=2;i++){
			sprintf(text_buffer, "k%u = %f\r\n", i, gyro_sens);		// Sensitivity
//			sprintf(text_buffer, "k%u = %f\r\n", i, gyro_sensit);
			num_bytes_written = f_puts(text_buffer, &gfc);        // Write to file
		}
		k=0;
		for(i=0;i<=2;i++){										// Alignment
			for(j=0;j<=2;j++){
				sprintf(text_buffer, "r%u%u = %f\r\n", i, j, (float)(gyro_pdata.orientation[k]));
				num_bytes_written = f_puts(text_buffer, &gfc);        // Write to file
				k += 1;
			}
		}
		num_bytes_written = f_puts("\r\n", &gfc);        // Write to file

		// MPL Magnetometer calibration
//		gyro_get_compass_fsr(&mag_Ga);
		inv_get_compass_bias(&magBias[0]);
		num_bytes_written = f_puts("[MPL Mag 1.2Ga]\r\n", &gfc);        // Write to file
		for(i=0;i<=2;i++){
			sprintf(text_buffer, "b%u = %f\r\n", i, (float)(magBias[i]));			// Offset
//			sprintf(text_buffer, "b%u = %f\r\n", i, (float)(magBias[i]) / 65536);			// Offset
//			sprintf(text_buffer, "b%u = NaN\r\n", i);			// Offset
			num_bytes_written = f_puts(text_buffer, &gfc);        // Write to file
		}
		for(i=0;i<=2;i++){
			sprintf(text_buffer, "k%u = %f\r\n", i, 341.0);		// Sensitivity
			num_bytes_written = f_puts(text_buffer, &gfc);        // Write to file
		}
		k=0;
		for(i=0;i<=2;i++){										// Alignment
			for(j=0;j<=2;j++){
				sprintf(text_buffer, "r%u%u = %f\r\n", i, j, (float)(compass_pdata.orientation[k]));
				num_bytes_written = f_puts(text_buffer, &gfc);        // Write to file
				k += 1;
			}
		}

		rc = f_close(&gfc);
		return 0;
	}
	return 1;
}

uint8_t MPU_loadCalibration(uint8_t *storedConfigLcl) {
	MPU_readCalibration(S_MPL_ACCEL, &storedConfigLcl[0]);
	MPU_readCalibration(S_MPL_GYRO, &storedConfigLcl[0]);
	MPU_readCalibration(S_MPL_MAG, &storedConfigLcl[0]);
	return 0;
}

uint8_t MPU_readCalibration(uint8_t sensor, uint8_t *storedConfigLcl) {
    uint8_t cal_file[34];
	char buffer[66], * equals, keyword[20];
	uint8_t i = 0;
	uint8_t num_2byte_params = 6, num_1byte_params = 9;
	uint16_t address;
	char sensor_found = 0;
	float value;
	int16_t rounded_value;
	DIRS gdc;
	FIL gfc;

	if(sensor == S_MPL_ACCEL){
		if(((storedConfigLcl[NV_CONFIG_SETUP_BYTE3]>>6) & 0x03)==RANGE_2G)
			strcpy(keyword, "MPL Accel 2g");
		else if(((storedConfigLcl[NV_CONFIG_SETUP_BYTE3]>>6) & 0x03)==RANGE_4G)
			strcpy(keyword, "MPL Accel 4g");
		else if(((storedConfigLcl[NV_CONFIG_SETUP_BYTE3]>>6) & 0x03)==RANGE_8G)
			strcpy(keyword, "MPL Accel 8g");
		else// if(((storedConfigLcl[NV_CONFIG_SETUP_BYTE3]>>6) & 0x03)==RANGE_16G)
			strcpy(keyword, "MPL Accel 16g");
		address = NV_MPL_ACCEL_CALIBRATION;
	}
	else if(sensor == S_MPL_GYRO){
		if((storedConfigLcl[NV_CONFIG_SETUP_BYTE2] & 0x03)==MPU9150_GYRO_250DPS)
			strcpy(keyword, "MPL Gyro 250dps");
		else if((storedConfigLcl[NV_CONFIG_SETUP_BYTE2] & 0x03)==MPU9150_GYRO_500DPS)
			strcpy(keyword, "MPL Gyro 500dps");
		else if((storedConfigLcl[NV_CONFIG_SETUP_BYTE2] & 0x03)==MPU9150_GYRO_1000DPS)
			strcpy(keyword, "MPL Gyro 1000dps");
		else// if((storedConfigLocal[NV_CONFIG_SETUP_BYTE2] & 0x03)==MPU9150_GYRO_2000DPS)
			strcpy(keyword, "MPL Gyro 2000dps");
		address = NV_MPL_GYRO_CALIBRATION;
		num_1byte_params = 0;
	}
	else if(sensor == S_MPL_MAG){
		strcpy(keyword, "MPL Mag 1.2Ga");
		address = NV_MPL_MAG_CALIBRATION;
	}

	strcpy((char*)cal_file, "/Calibration/MPL/MPUcalib.ini\0");
	if(f_opendir(&gdc, "/Calibration/MPL\0")){//(res = )
		MPL_defaultCalibration(sensor, &storedConfigLcl[0]);
	}
	else if(f_open(&gfc, (char*)cal_file, (FA_OPEN_EXISTING | FA_READ))){
		 // no calibration file, use default
		MPL_defaultCalibration(sensor, &storedConfigLcl[0]);
	}
	else { // look for sensor in calibration file.
		while(f_gets(buffer, 64, &gfc)){
			if(!strstr(buffer, keyword))
				continue;

			else{ // found the right sensor
				sensor_found = 1;
				for(i = 0; i < num_2byte_params; i++){
					f_gets(buffer, 64, &gfc);
					if(!(equals = strchr(buffer, '='))){
						sensor_found = 0; // there's an error, use the default
						break;
					}
					equals ++;
					value = atof(equals);

					if(i<=2) {
						if(sensor == S_MPL_ACCEL){
							accelBias[i] = value;
						}
						else if(sensor == S_MPL_GYRO){
							gyroBias[i] = value;
						}
						else if(sensor == S_MPL_MAG){
							magBias[i] = value;
						}
					}

					value = value / 65535;
					rounded_value = (int16_t)(value>=0?value+0.5:value-0.5);
					storedConfigLcl[address + 2*i] = (int8_t)((rounded_value & 0xFF00) >> 8);
					storedConfigLcl[address + 2*i + 1] = (int8_t)(rounded_value & 0xFF);
//					storedConfigLcl[address + 2*i] = ((int16_t)(value + 0.5) & 0xFF00) >> 8;
//					storedConfigLcl[address + 2*i + 1] = ((int16_t)(value + 0.5) & 0xFF);
				}
				for(i = 0; i < num_1byte_params; i++){
					f_gets(buffer, 64, &gfc);
					if(!(equals = strchr(buffer, '='))){
						sensor_found = 0; // there's an error, use the default
						break;
					}
					equals ++;
					value = atof(equals) * 100;
					rounded_value = (int8_t)(value>=0?value+0.5:value-0.5);
					storedConfigLcl[address + 2*num_2byte_params + i] = (rounded_value);
				}
				f_close(&gfc);
				break;
			}
		}
	}
	if(!sensor_found)
		MPL_defaultCalibration(sensor, &storedConfigLcl[0]);

	return 0;
}

void MPL_defaultCalibration(uint8_t sensor, uint8_t *storedConfigLcl) {
    int16_t bias, sensitivity;
    int8_t align_xx, align_xy, align_xz, align_yx, align_yy, align_yz, align_zx, align_zy, align_zz;
    uint16_t address;

	uint8_t mpu9150_accel_range = ((storedConfigLcl[NV_CONFIG_SETUP_BYTE3]>>6)&0x03);
	uint8_t mpu9150_gyro_range = (storedConfigLcl[NV_CONFIG_SETUP_BYTE2] & 0x03);
	uint8_t i, align;

	if(sensor == S_MPL_ACCEL) {
		bias = 0;
		align = 1;
		accelBias[0]=0;
		accelBias[1]=0;
		accelBias[2]=0;
		if(mpu9150_accel_range == RANGE_2G)
			sensitivity = 16384;	// 16384 LSB/g
		else if(mpu9150_accel_range == RANGE_4G)
			sensitivity = 8192;		// 8192 LSB/g
		else if (mpu9150_accel_range == RANGE_8G)
			sensitivity = 4096;		// 4096 LSB/g
		else //(sdhead_text[SDH_ACCEL_RANGE] == RANGE_16G)
			sensitivity = 2048;		// 2048 LSB/g
		align_xx = 0;
		align_xy = -100;
		align_xz = 0;
		align_yx = -100;
		align_yy = 0;
		align_yz = 0;
		align_zx = 0;
		align_zy = 0;
		align_zz = -100;
		address = NV_MPL_ACCEL_CALIBRATION;
	}
	else if(sensor == S_MPL_GYRO) {
		bias = 0;
		align = 0;
		gyroBias[0]=0;
		gyroBias[1]=0;
		gyroBias[2]=0;
		if(mpu9150_gyro_range == MPU9150_GYRO_250DPS)
			sensitivity = 13100;	// 131 LSB/(degree/s)
		else if(mpu9150_gyro_range == MPU9150_GYRO_500DPS)
			sensitivity = 6550;	// 65.5 LSB/(degree/s)
		else if (mpu9150_gyro_range == MPU9150_GYRO_1000DPS)
			sensitivity = 3280;	// 32.8 LSB/(degree/s)
		else //(sdhead_text[SDH_GYRO_RANGE] == MPU9150_GYRO_2000DPS)
			sensitivity = 1640;	// 16.4 LSB/(degree/s)
		align_xx = 0;
		align_xy = -100;
		align_xz = 0;
		align_yx = -100;
		align_yy = 0;
		align_yz = 0;
		align_zx = 0;
		align_zy = 0;
		align_zz = -100;
		address = NV_MPL_GYRO_CALIBRATION;
	}
	else if(sensor == S_MPL_MAG) {
		bias = 0;
		align = 1;
		magBias[0]=0;
		magBias[1]=0;
		magBias[2]=0;
		sensitivity = 341; // 3.41 LSB/uT (341 LSB/G) - 13-bit range over +-1200uT
		align_xx = -100;
		align_xy = 0;
		align_xz = 0;
		align_yx = 0;
		align_yy = -100;
		align_yz = 0;
		align_zx = 0;
		align_zy = 0;
		align_zz = 100;
		address = NV_MPL_MAG_CALIBRATION;
	}
    for(i = 0; i < 3; i++){
    	// offsets
		storedConfigLcl[address + 2*i] = (bias & 0xFF00) >> 8;
		storedConfigLcl[address + 2*i + 1] = (bias & 0xFF);
		// sensitivity
		storedConfigLcl[address + 6 + 2*i] = (sensitivity & 0xFF00) >> 8;
		storedConfigLcl[address + 6 + 2*i + 1] = (sensitivity & 0xFF);
    }
    // alignment
    if(align){
		storedConfigLcl[address + 12] = align_xx;
		storedConfigLcl[address + 13] = align_xy;
		storedConfigLcl[address + 14] = align_xz;
		storedConfigLcl[address + 15] = align_yx;
		storedConfigLcl[address + 16] = align_yy;
		storedConfigLcl[address + 17] = align_yz;
		storedConfigLcl[address + 18] = align_zx;
		storedConfigLcl[address + 19] = align_zy;
		storedConfigLcl[address + 20] = align_zz;
    }
}

// Pansenti implementation OF 9DOF sensor fusion
// Alternative to MPL sensor fusion - here for testing
void pansenti_sensor_fusion(long *m_rawQuaternion, long *m_rawMag)
{
	float qMag[4];
	float deltaDMPYaw, deltaMagYaw;
	float newMagYaw, newYaw;
	float temp1[4], unFused[4];
	float unFusedConjugate[4];

	float m_dmpEulerPose[3];                                  // Euler angles from the DMP quaternion
	float m_fusedEulerPose[3];                                // the fused Euler angles

	float m_dmpQuaternion[4];
	float m_calMag[3];

	m_dmpQuaternion[0] = (float)m_rawQuaternion[0] / 1073741824;//(1<<30);
	m_dmpQuaternion[1] = (float)m_rawQuaternion[1] / 1073741824;//(1<<30);
	m_dmpQuaternion[2] = (float)m_rawQuaternion[2] / 1073741824;//(1<<30);
	m_dmpQuaternion[3] = (float)m_rawQuaternion[3] / 1073741824;//(1<<30);

	m_calMag[0] = (float)m_rawMag[0] / 65536;//(1<<16);
	m_calMag[1] = (float)m_rawMag[1] / 65536;//(1<<16);
	m_calMag[2] = (float)m_rawMag[2] / 65536;//(1<<16);

    MPUQuaternionNormalize(m_dmpQuaternion);                 // and normalize //-- think i can skip this step since already normallised
	MPUQuaternionQuaternionToEuler(m_dmpQuaternion, m_dmpEulerPose);

	//--	MPU9150Lib_dataFusion function - begin

	// *** NOTE *** pitch direction swapped here
//	m_fusedEulerPose[VEC3_X] = m_dmpEulerPose[VEC3_X];
//	m_fusedEulerPose[VEC3_Y] = -m_dmpEulerPose[VEC3_Y];

	m_fusedEulerPose[VEC3_X] = m_dmpEulerPose[VEC3_X];
	m_fusedEulerPose[VEC3_Y] = m_dmpEulerPose[VEC3_Y];
	m_fusedEulerPose[VEC3_Z] = 0;
	MPUQuaternionEulerToQuaternion(m_fusedEulerPose, unFused);    // create a new quaternion

	deltaDMPYaw = -m_dmpEulerPose[VEC3_Z] + m_lastDMPYaw;         // calculate change in yaw from dmp
	m_lastDMPYaw = m_dmpEulerPose[VEC3_Z];                        // update that

	qMag[QUAT_W] = 0;
	qMag[QUAT_X] = m_calMag[VEC3_X];
	qMag[QUAT_Y] = m_calMag[VEC3_Y];
	qMag[QUAT_Z] = m_calMag[VEC3_Z];

	// Tilt compensate mag with the unfused data (i.e. just roll and pitch with yaw 0)
	MPUQuaternionConjugate(unFused, unFusedConjugate);
	MPUQuaternionMultiply(qMag, unFusedConjugate, temp1);
	MPUQuaternionMultiply(unFused, temp1, qMag);

	// Now fuse this with the dmp yaw gyro information
	newMagYaw = -atan2(qMag[QUAT_Y], qMag[QUAT_X]);

	if (newMagYaw != newMagYaw) {                                 // check for nAn
		return;                                                     // just ignore in this case
	}
	if (newMagYaw < 0)
		newMagYaw = 2.0f * (float)M_PI + newMagYaw;                 // need 0 <= newMagYaw <= 2*PI

	newYaw = m_lastYaw + deltaDMPYaw;                             // compute new yaw from change

	if (newYaw > (2.0f * (float)M_PI))                            // need 0 <= newYaw <= 2*PI
	newYaw -= 2.0f * (float)M_PI;
	if (newYaw < 0)
	newYaw += 2.0f * (float)M_PI;

	deltaMagYaw = newMagYaw - newYaw;                             // compute difference

	if (deltaMagYaw >= (float)M_PI)
		deltaMagYaw = deltaMagYaw - 2.0f * (float)M_PI;
	if (deltaMagYaw <= -(float)M_PI)
		deltaMagYaw = (2.0f * (float)M_PI + deltaMagYaw);

	if (m_magMix > 0) {
		newYaw += deltaMagYaw / m_magMix;                           // apply some of the correction
		if (newYaw > (2.0f * (float)M_PI))				            // need 0 <= newYaw <= 2*PI
		  newYaw -= 2.0f * (float)M_PI;
		if (newYaw < 0)
		  newYaw += 2.0f * (float)M_PI;
	}

	m_lastYaw = newYaw;

	if (newYaw > (float)M_PI)
		newYaw -= 2.0f * (float)M_PI;

	m_fusedEulerPose[VEC3_Z] = newYaw;                            // fill in output yaw value

//--	MPUQuaternionEulerToQuaternion(m_fusedEulerPose, m_fusedQuaternion);
	//-- reuse m_dmpQuaternion variable instead of creating new m_fusedQuaternion from Pansenti implementation
	MPUQuaternionEulerToQuaternion(m_fusedEulerPose, m_dmpQuaternion);

	//--	MPU9150Lib_dataFusion function - end

	m_rawQuaternion[0] = (long)(m_dmpQuaternion[0] * 1073741824);//(1<<30);
	m_rawQuaternion[1] = (long)(m_dmpQuaternion[1] * 1073741824);//(1<<30);
	m_rawQuaternion[2] = (long)(m_dmpQuaternion[2] * 1073741824);//(1<<30);
	m_rawQuaternion[3] = (long)(m_dmpQuaternion[3] * 1073741824);//(1<<30);
}
