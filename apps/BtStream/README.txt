This is a general purpose configurable application to be used with shimmer3 and any add-on daughter-cards supplied by Shimmer.


By default this application samples the 3-axis analog accelerometer, MPU9150 gyroscope, LSM303DLHC magnetometer and battery voltage at 50Hz and sends the data over the Bluetooth radio at 51.2Hz, using a data buffer size of 1 (the buffer size is not currently configurable).

Data Packet Format:
          Packet Type | TimeStamp | chan1 | chan2 | ... | chanX
   Byte:       0      |    1-2    |  3-4  |  5-6  | ... | chanX


When the application receives an Inquiry command it responds with the following packet. The value in the channel fields indicate exactly what data from which sensor is contained in this field of the data packet:

Inquiry Response Packet Format:
          Packet Type | ADC Sampling rate | Config Bytes | Num Chans | Buf size | Chan1 | Chan2 | ... | ChanX
   Byte:       0      |        1-2        |      3-6     |     7     |     8    |   9   |   10  | ... |   x


Currently the following parameters can be configured. This configuration is stored in the Infomem so survives a reset/power off:
   - Sampling rate
   - Which sensors are sampled
   - Which LED toggles to indicate status
   - LSM303DLHC accel range
   - LSM303DLHC accel sampling rate
   - LSM303DLHC accel low-power mode
   - LSM303DLHC accel high-resolution mode
   - LSM303DLHC mag gain
   - LSM303DLHC mag sampling rate
   - MPU9150 gyro range
   - MPU9150 sampling rate
   - MPU9150 accel range
   - BMP180 pressure oversampling ratio
   - GSR range
      - Special cases: 
         - GSR autorange
            - When set the GSR range is controlled on the shimmer
            - The two most significant bits of the GSR channel value are overloaded to indicate which resistor is active (i.e. the range)
               - e.g. if HW_RES_1M is selected (val 2), then the GSR channel will be 10xxxxxxxxxxxxxx
         - GSRx4
            - not currently used
   - Internal expansion board power enable
   - Register settings for both ADS1292R chips on ExG daughter card
   - calibration values for analog accel, MPU9150 gyro, LSM303DLHC mag, LSM303DLHC accel
   - Baud rate of UART communication between MSP430 and RN42 Bluetooth module


The following commands are available:
   - INQUIRY_COMMAND
   - GET_SAMPLING_RATE_COMMAND
   - SET_SAMPLING_RATE_COMMAND
   - TOGGLE_LED_COMMAND
   - START_STREAMING_COMMAND
   - SET_SENSORS_COMMAND
   - SET_LSM303DLHC_ACCEL_RANGE_COMMAND
   - GET_LSM303DLHC_ACCEL_RANGE_COMMAND
   - SET_CONFIG_SETUP_BYTES_COMMAND
   - GET_CONFIG_SETUP_BYTES_COMMAND
   - SET_A_ACCEL_CALIBRATION_COMMAND
   - GET_A_ACCEL_CALIBRATION_COMMAND
   - SET_MPU9150_GYRO_CALIBRATION_COMMAND
   - GET_MPU9150_GYRO_CALIBRATION_COMMAND
   - SET_LSM303DLHC_MAG_CALIBRATION_COMMAND
   - GET_LSM303DLHC_MAG_CALIBRATION_COMMAND
   - SET_LSM303DLHC_ACCEL_CALIBRATION_COMMAND
   - GET_LSM303DLHC_ACCEL_CALIBRATION_COMMAND
   - STOP_STREAMING_COMMAND
   - SET_GSR_RANGE_COMMAND
   - GET_GSR_RANGE_COMMAND
   - DEPRECATED_GET_DEVICE_VERSION_COMMAND
   - GET_ALL_CALIBRATION_COMMAND
   - GET_FW_VERSION_COMMAND
   - SET_CHARGE_STATUS_LED_COMMAND
   - GET_CHARGE_STATUS_LED_COMMAND
   - GET_BUFFER_SIZE_COMMAND
   - SET_LSM303DLHC_MAG_GAIN_COMMAND
   - GET_LSM303DLHC_MAG_GAIN_COMMAND
   - SET_LSM303DLHC_MAG_SAMPLING_RATE_COMMAND
   - GET_LSM303DLHC_MAG_SAMPLING_RATE_COMMAND
   - GET_UNIQUE_SERIAL_COMMAND
   - GET_DEVICE_VERSION_COMMAND
   - SET_LSM303DLHC_ACCEL_SAMPLING_RATE_COMMAND
   - GET_LSM303DLHC_ACCEL_SAMPLING_RATE_COMMAND
   - SET_LSM303DLHC_ACCEL_LPMODE_COMMAND
   - GET_LSM303DLHC_ACCEL_LPMODE_COMMAND
   - SET_LSM303DLHC_ACCEL_HRMODE_COMMAND
   - GET_LSM303DLHC_ACCEL_HRMODE_COMMAND
   - SET_MPU9150_GYRO_RANGE_COMMAND
   - GET_MPU9150_GYRO_RANGE_COMMAND
   - SET_MPU9150_SAMPLING_RATE_COMMAND
   - GET_MPU9150_SAMPLING_RATE_COMMAND
   - SET_MPU9150_ACCEL_RANGE_COMMAND
   - GET_MPU9150_ACCEL_RANGE_COMMAND
   - SET_BMP180_PRES_OVERSAMPLING_RATIO_COMMAND
   - GET_BMP180_PRES_OVERSAMPLING_RATIO_COMMAND
   - GET_BMP180_CALIBRATION_COEFFICIENTS_COMMAND
   - RESET_TO_DEFAULT_CONFIGURATION_COMMAND
   - RESET_CALIBRATION_VALUE_COMMAND
   - GET_MPU9150_MAG_SENS_ADJ_VALS_COMMAND
   - SET_INTERNAL_EXP_POWER_ENABLE_COMMAND
   - GET_INTERNAL_EXP_POWER_ENABLE_COMMAND
   - SET_EXG_REGS_COMMAND
   - GET_EXG_REGS_COMMAND
   - GET_DAUGHTER_CARD_ID_COMMAND
   - SET_DAUGHTER_CARD_MEM_COMMAND
   - GET_DAUGHTER_CARD_MEM_COMMAND
   - SET_BT_COMMS_BAUD_RATE
   - GET_BT_COMMS_BAUD_RATE



The format of the configuration bytes (as returned by the inquiry command):
Config Setup Byte 0:
   - Bit 7-4: LSM303DLHC accel sampling rate
   - Bit 3-2: LSM303DLHC accel range
   - Bit 1:   LSM303DLHC accel low power mode 
   - Bit 0:   LSM303DLHC accel high resolution mode
Config Setup Byte 1:
   - Bit 7-0: MPU9150 sampling rate
Config Setup Byte 2:
   - Bit 7-5: LSM303DLHC mag gain
   - Bit 4-2: LSM303DLHC mag sampling rate
   - Bit 1-0: MPU9150 gyro range
Config Setup Byte 3:
   - Bit 7-6: MPU9150 accel range
   - Bit 5-4: BMP180 pressure resolution
   - Bit 3-1: GSR range
   - Bit 0:   Internal expansion power enable


The assignment of the selected sensors field is a follows:
   - 1 bit per sensor. When there is a conflict priority is most significant bit -> least significant bit
      Byte0:
         Bit 7: Analog Accel
         Bit 6: MPU9150 Gyro
         Bit 5: LSM303DLHC Magnetometer
         Bit 4: ExG ADS1292R chip 1 24-bit data 
         Bit 3: ExG ADS1292R chip 2 24-bit data 
         Bit 2: GSR
         Bit 1: External ADC Channel 7
         Bit 0: External ADC Channel 6
      Byte1
         Bit 7: Bridge Amplifier
         Bit 6: Not yet assigned
         Bit 5: Battery voltage
         Bit 4: LSM303DLHC Accel
         Bit 3: External ADC Channel 15
         Bit 2: Internal ADC Channel 1
         Bit 1: Internal ADC Channel 12
         Bit 0: Internal ADC Channel 13
      Byte2
         Bit 7: Internal ADC Channel 14
         Bit 6: MPU9150 Accel
         Bit 5: MPU9150 Mag
         Bit 4: ExG ADS1292R chip 1 16-bit data 
         Bit 3: ExG ADS1292R chip 2 16-bit data 
         Bit 2: BMP180 Pressure
         Bit 1: Not yet assigned
         Bit 0: Not yet assigned


The GET_SHIMMER_VERSION_COMMAND returns a 1 byte value, based on the shimmer revision as follows:
   0 = shimmer1
   1 = shimmer2
   2 = shimmer2r
   3 = shimmer3
   

The GET_FW_VERSION_COMMAND returns 6 bytes:
   Byte0+Byte1                   = Firmware identifier (see FirmwareIdentifierList.txt)
                                 = 1 for this application
   Byte2+Byte3 (little endian)   = Major version number
   Byte4                         = Minor version number
   Byte5                         = Release candidate number (internal use only. All externally released
                                                             firmware images will have 0 here)

The GET_ALL_CALIBRATION_COMMAND returns all the stored calibration values (71 bytes) in the following order:
   Analog Accel (21 bytes)
   Gyro  (21 bytes)
   Mag   (21 bytes)
   LSM303DLHC Accel (21 bytes)
The breakdown of the kinematic (accel x 2, gyro and mag) calibration values is as follows:
   Each of the 3 offset bias vector values (one for each axis) are stored as 16-bit signed integers (big endian) and are contained in bytes 0-5. Each of the 3 sensitivity vector values (again one for each axis) are stored as 16-bit signed integers (big endian) and are contained in bytes 6-11. Each of the 9 alignment matrix values are stored as 8-bit signed integers and are contained in bytes 12-20.  

The SET_CHARGE_STATUS_LED_COMMAND changes which LED is used to indicate the shimmer's state:
- selected LED operation
   - Occasional flash of selected LED when on but not connected
      - On for approx 100ms
      - Off for 2s
   - Solid LED when connected but not streaming
   - Flash LED @1Hz when connected and streaming
- The SET_CHARGE_STATUS_LED command takes a single 1 byte argument
   - 0 for green
   - 1 for yellow
   - 2 for red
- Selected LED resets to green at power up
- Resets to green if docked but not streaming
- The GET_CHARGE_STATUS_LED_COMMAND returns a single byte, indicating selected LED as above 


When the Shimmer is docked (in the programming dock or multi-gang charger) the lower LEDs indicate charger status:
- Yellow on: charging
- Green on: charging completed 


When the Shimmer is docked, and not actively streaming data, it will respond to commands through the external serial (UART) port. The format of the command are below:
SET/GET/RESPONSE Packet Format:
       start_sign | cmd | length | comp | prop |     data     |      crc     |
Byte #:     0     |  1  |   2    |  3   |  4   | 5-(4+length) | (5-6)+length |

where start_sign  (1 byte) = '$' (0x24)
      cmd         (1 byte) = command names (e.g. SET/GET/RSP/ACK)
      length      (1 byte) = size of comp + prop + data
      comp        (1 byte) = component names (e.g. SHIMMER/BATT/DAUGHTER CARD)
      prop        (1 byte) = property of the corresponding component (e.g. MAC/RANGE/OFFSET)
      data        (up to 128 bytes) = args to pass
      crc         (2 bytes)
example: to get shimmer mac address, send:
|  byte #    | 1  |  2    3    4    5    6      7   |
|content(0x) | 24 | 03   03   01   02  crc_l  crc_h |

ack/nack Packet Format:
       start_sign | cmd |      crc     |
Byte:       0     |  1  | (2-3)+length |

example: bad argument nack
|  byte #   | 1  |   2     3     4   |
|content(0x)| 24 | 0xfd  crc_l crc_h |


Changelog:
V0.6 (27 March 2015)
   - changed format of serial/UART commands
   - added support for DERIVED_CHANNEL configuration bytes
V0.5 (10 September 2014)
   - add support for serial/UART commands
      - supported commands: ver$, mac$, bat$ and mem$
   - changed timing of LED blinking
   - add I2C timeout 
   - handle external EEPROM data read error 
   - No longer change BT baud rate when running RESET_TO_DEFAULT_CONFIGURATION_COMMAND 
V0.4 (2 July 2014)
   - add support for bridge amplifier
   - add commands to change baud rate of comms with Bluetooth module
   - fix bug with ADC1 and ADC14 channels if ExG had been configured since reset
V0.3 (13 March 2014)
   - Support for ExG daughter card
   - Support for accessing EEPROM on daughter cards
   - fixed bug that could cause app to hang if Bluetooth disconneted while actively streaming
V0.2 (27 November 2013)
   - Support for SR31 Rev. 4 boards
      - added support for SW_I2C
   - Support for GSR daughter card
   - Changed main timer from TA0 to TB0
      - as TA0 is required for PWM on the Proto3 deluxe board
   - Change SW_ACCEL to be pulled down input when sleeping
   - Bluetooth driver
      - added command to set serialized friendly name
      - allow authentication mode to be set to value other than 1
   - Misc bug fixes
      - only attempt to start streaming if not already streaming
      - only stop streaming at end of sensing cycle
      - ensure ACK comes after last packet when stopping streaming
V0.1 (9 October 2013)
- Initial release

