/*
 $License:
    Copyright (C) 2011-2012 InvenSense Corporation, All Rights Reserved.
    See included License.txt for License information.
 $
 */
/**
 *  @addtogroup  DRIVERS Sensor Driver Layer
 *  @brief       Hardware drivers to communicate with sensors via I2C.
 *
 *  @{
 *      @file       inv_gyro_dmp_android.h
 *      @brief      DMP image and interface functions.
 *      @details    The DMP image supported here matches the image used in
 *                  Android. All functions are preceded by the dmp_ prefix to
 *                  differentiate among MPL and general driver function calls.
 */
#ifndef _INV_GYRO_DMP_ANDROID_H_
#define _INV_GYRO_DMP_ANDROID_H_

#define TAP_X               (0x01)
#define TAP_Y               (0x02)
#define TAP_Z               (0x04)
#define TAP_XYZ             (0x07)

#define TAP_X_UP            (0x01)
#define TAP_X_DOWN          (0x02)
#define TAP_Y_UP            (0x03)
#define TAP_Y_DOWN          (0x04)
#define TAP_Z_UP            (0x05)
#define TAP_Z_DOWN          (0x06)

#define ORIENTATION_X_UP    (0x01)
#define ORIENTATION_X_DOWN  (0x02)
#define ORIENTATION_Y_UP    (0x04)
#define ORIENTATION_Y_DOWN  (0x08)
#define ORIENTATION_Z_UP    (0x10)
#define ORIENTATION_Z_DOWN  (0x20)
#define ORIENTATION_FLIP    (0x40)
#define ORIENTATION_ALL     (0x3F)

#define DISPLAY_ORIENT_PORTRAIT             (0x00)
#define DISPLAY_ORIENT_LANDSCAPE            (0x01)
#define DISPLAY_ORIENT_REVERSE_PORTRAIT     (0x02)
#define DISPLAY_ORIENT_REVERSE_LANDSCAPE    (0x03)

#define DMP_INT_GESTURE     (0x01)
#define DMP_INT_CONTINUOUS  (0x02)

#define DMP_FEATURE_TAP             (0x01)
#define DMP_FEATURE_ORIENT          (0x02)
#define DMP_FEATURE_DISPLAY_ORIENT  (0x04)
#define DMP_FEATURE_LP_QUAT         (0x08)
#define DMP_FEATURE_PEDOMETER       (0x10)
#define DMP_FEATURE_6X_LP_QUAT      (0x20)

#define INV_WXYZ_QUAT       (0x100)

/* Set up functions. */
int dmp_load_android_firmware(void);
int dmp_set_fifo_rate(unsigned short rate);
int dmp_get_fifo_rate(unsigned short *rate);
int dmp_enable_feature(unsigned char mask);
int dmp_set_interrupt_mode(unsigned char mode);
int dmp_set_orientation(unsigned short orient);
int dmp_set_gyro_bias(long *bias);

/* Tap functions. */
int dmp_register_tap_cb(void (*func)(unsigned char, unsigned char));
int dmp_set_tap_thresh(unsigned char axis, unsigned short thresh);
int dmp_set_tap_axes(unsigned char axis);
int dmp_set_tap_count(unsigned char min_taps);
int dmp_set_tap_time(unsigned short time);
int dmp_set_tap_time_multi(unsigned short time);
int dmp_set_shake_reject_thresh(long sf, unsigned short thresh);
int dmp_set_shake_reject_time(unsigned short time);
int dmp_set_shake_reject_timeout(unsigned short time);

/* Display orientation functions. */
int dmp_register_display_orient_cb(void (*func)(unsigned char));

/* Orientation functions. */
int dmp_register_orient_cb(void (*func)(unsigned char));
int dmp_set_orient_axes(unsigned char axis);
int dmp_set_orient_thresh(float angle);
int dmp_set_orient_time(unsigned short time);

/* LP quaternion functions. */
int dmp_enable_lp_quat(unsigned char enable);
int dmp_enable_6x_lp_quat(unsigned char enable);

/* Pedometer functions. */
int dmp_get_pedometer_step_count(unsigned long *count);
int dmp_set_pedometer_step_count(unsigned long count);
int dmp_get_pedometer_walk_time(unsigned long *time);
int dmp_set_pedometer_walk_time(unsigned long time);

/* Read function. This function should be called whenever the MPU interrupt is
 * detected.
 */
int dmp_read_fifo(short *gyro, short *accel, long *quat,
    unsigned long *timestamp, short *sensors, unsigned char *more);

#endif  /* #ifndef _INV_GYRO_DMP_ANDROID_H_ */

