/*
 $License:
    Copyright (C) 2011-2012 InvenSense Corporation, All Rights Reserved.
    See included License.txt for License information.
 $
 */
/**
 *  @addtogroup DRIVERS
 *  @brief      Hardware drivers.
 *
 *  @{
 *      @file  inv_gyro.h
 *      @brief Struct definitions for the Invensense gyro driver.
 */

#ifndef _INV_GYRO_H_
#define _INV_GYRO_H_

#define INV_X_GYRO      (0x40)
#define INV_Y_GYRO      (0x20)
#define INV_Z_GYRO      (0x10)
#define INV_XYZ_GYRO    (INV_X_GYRO | INV_Y_GYRO | INV_Z_GYRO)
#define INV_XYZ_ACCEL   (0x08)
#define INV_XYZ_COMPASS (0x01)

struct int_param_s {
#if defined EMPL_TARGET_MSP430
    void (*cb)(void);
    unsigned char port;
    unsigned char pin;
    unsigned char lp_exit;
#elif defined EMPL_TARGET_UC3L0
    unsigned long pin;
    void (*cb)(volatile void*);
    void *arg;
#endif
};

/* Set up APIs */
int gyro_init(struct int_param_s *int_param, unsigned short FSR_gyro, unsigned char FSR_accel, unsigned short DMP_LPF, unsigned short rate_gyro, unsigned int rate_compass_ms);
int gyro_init_slave(void);
int gyro_set_bypass(unsigned char bypass_on);

/* Configuration APIs */
int gyro_lp_accel_mode(unsigned char rate);
int gyro_set_int_level(unsigned char active_low);
int gyro_set_int_latched(unsigned char enable);

int gyro_set_dmp_state(unsigned char enable);
int gyro_get_dmp_state(unsigned char *enabled);

int gyro_get_lpf(unsigned short *lpf);
int gyro_set_lpf(unsigned short lpf);

int gyro_get_gyro_fsr(unsigned short *fsr);
int gyro_set_gyro_fsr(unsigned short fsr);

int gyro_get_accel_fsr(unsigned char *fsr);
int gyro_set_accel_fsr(unsigned char fsr);

int gyro_get_compass_fsr(unsigned short *fsr);

int gyro_get_gyro_sens(float *sens);
int gyro_get_accel_sens(unsigned short *sens);

int gyro_get_sample_rate(unsigned short *rate);
int gyro_set_sample_rate(unsigned short rate);
int gyro_get_compass_sample_rate(unsigned short *rate);
int gyro_set_compass_sample_rate(unsigned short rate);

int gyro_get_fifo_config(unsigned char *sensors);
int gyro_configure_fifo(unsigned char sensors);

int gyro_get_power_state(unsigned char *power_on);
int gyro_set_sensors(unsigned char sensors);

int gyro_set_accel_bias(const long *accel_bias);

/* Data getter/setter APIs */
int gyro_get_gyro_reg(short *data, unsigned long *timestamp);
int gyro_get_accel_reg(short *data, unsigned long *timestamp);
int gyro_get_compass_reg(short *data, unsigned long *timestamp);
int gyro_get_temperature(long *data, unsigned long *timestamp);

int gyro_read_fifo(short *gyro, short *accel, unsigned long *timestamp,
    unsigned char *sensors, unsigned char *more);
int gyro_read_fifo_stream(unsigned short length, unsigned char *data,
    unsigned char *more);
int gyro_reset_fifo(void);

int gyro_write_mem(unsigned short mem_addr, unsigned short length,
    unsigned char *data);
int gyro_read_mem(unsigned short mem_addr, unsigned short length,
    unsigned char *data);
int gyro_load_firmware(unsigned short length, const unsigned char *firmware,
    unsigned short start_addr, unsigned short sample_rate);

int gyro_reg_dump(void);
int gyro_read_reg(unsigned char reg, unsigned char *data);
int gyro_run_self_test(long *gyro, long *accel);
int gyro_register_tap_cb(void (*func)(unsigned char, unsigned char));

// this function needs to be run if using system_pre_init.c
void MPU_initStructures(void);

#endif  /* #ifndef _INV_GYRO_H_ */

