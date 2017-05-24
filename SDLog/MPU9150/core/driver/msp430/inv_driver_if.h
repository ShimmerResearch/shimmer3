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
 *      @file       inv_driver_if.h
 *      @brief      Driver interface for third-party I2C sensors.
 *      @details    For easy integration with eMPL, third-party accels and
 *                  compasses are expected to implement all the functions
 *                  declared here. Some assumptions are made about the
 *                  third-party device; if these assumptions are invalid, any
 *                  custom driver will work, but the HAL will need to be
 *                  modified accordingly.
 */
#ifndef _INV_DRIVER_IF_H_
#define _INV_DRIVER_IF_H_

/**
 *  @brief  Third-party driver interface.
 *  Each of these callbacks must be defined for a particular sensors or return
 *  -1 if unsupported.
 */
struct driver_if_s {
    /**
     *  @brief  Initialize the driver.
     *  Set up I2C communication with the sensor and initialize the driver's
     *  state variables. After calling this function, the device is expected
     *  to be in a "powered-on" state.
     *  @return 0 if successful.
     */
    int (*init)(void);
    /**
     *  @brief  Place the sensor in a standby/sleep mode.
     *  @return 0 if successful.
     */
    int (*suspend)(void);
    /**
     *  @brief  Place the sensor in a full-power/wake mode.
     *  @return 0 if successful.
     */
    int (*resume)(void);
    /**
     *  @brief      Get one set of sensor data.
     *  This function will return one of the following:
     *  \n a. Raw data in hardware units.
     *  \n b. Calibrated data in device-specific units (g's, dps, etc.) * 2^16.
     *  @param[out] data    A three-element array.
     *  @return     0 if successful.
     */
    int (*get_data)(long *data);
    /**
     *  @brief      Get device's full-scale range.
     *  @param[in]  fsr     Current full-scale range.
     *  @return     0 if successful.
     */
    int (*get_fsr)(unsigned short *fsr);
    /**
     *  @brief      Set device's sampling rate.
     *  @param[in]  rate    Desired sampling rate.
     *  @return     0 if successful.
     */
    int (*set_sample_rate)(unsigned short rate);
    /**
     *  @brief      Get device's sampling rate.
     *  @param[in]  rate    Current sampling rate.
     *  @return     0 if successful.
     */
    int (*get_sample_rate)(unsigned short *rate);
    /**
     *  @brief      Register a function to be called when new data is ready.
     *  This function will be executed in an ISR, so keep it simple! If this
     *  sensor's interrupt pin is not connected to the MCU, return an error.
     *  @param[in]  data_ready_cb   Callback.
     *  @return     0 if successful.
     */
    int (*set_data_ready_cb)(void (*data_ready_cb)(void));
};

#endif  /* #ifndef _INV_DRIVER_IF_H_ */

/**
 *  @}
 */

