♦This is a general purpose configurable application to be used with shimmer3 and any add-on 
 daughter-cards supplied by Shimmer.


♦Log file Format:
          | SD Header | Data Buffer 1 | Data Buffer 2 ...
   Byte #:|   0~255   |    256~...    |     ...


♦Data Buffer Format:
          | Data Block 1 | Data Block 2 ...
   Byte #:|     0~...    |    ...   
   - Data Buffer size <= 512
   - Data buffer always store integer number of Data Blocks

   
♦Data Block Format:
          |TimeStamp|Achan1|Achan2| ... |AchanX | Dchan1  | Dchan2  | ... |    DchanY   |
   Byte #:|   0-1   | 2~3  | 4~5  | ... |2x~2x+1|2x+2~2x+3|2x+4~2x+5| ... |2x+2y~2x+wy+1|
   - some digital channels have more than 2 bytes data
   - refer to user manual for endianness of each specific channel

 
♦UART structure:
   set/get/response Packet Format:
          |start_sign | cmd | length | comp | prop |     data     |          crc         |
   Byte #:|     0     |  1  |   2    |  3   |  4   | 5~(4+length) | (5+length)~(6+length)|
  
   where start_sign  (1 byte) = '$' (0x24)
         cmd         (1 byte) = command names (e.g. SET/GET/RSP/ACK)
         length      (1 byte) = size of comp + prop + data
         comp        (1 byte) = component names (e.g. SHIMMER/BATT/DAUGHTER CARD)
         prop        (1 byte) = property of the corresponding component (e.g. MAC/RANGE/OFFSET)
         data        (up to 128 bytes) = args to pass
         crc         (2 bytes)
   example: to get shimmer mac address, send:
   Content(0x) | 24 | 03   03   01   02  crc_l  crc_h |
     Byte #:   | 1  |  2    3    4    5    6      7   |
  
   ack/nack Packet Format: (this packet type responds to 'set' commands)
         | start_sign | cmd | crc |
   Byte #|      0     |  1  | 2~3 |
   example: bad argument nack
     Byte #:  | 1  |   2     3     4   |
   Content(0x)| 24 | 0xfd  crc_l crc_h |
   
   defiend UART values
      // uart: commands
      #define UART_SET                    0x01
      #define UART_RESPONSE               0x02
      #define UART_GET                    0x03
      #define UART_BAD_CMD_RESPONSE       0xfc
      #define UART_BAD_ARG_RESPONSE       0xfd
      #define UART_BAD_CRC_RESPONSE       0xfe
      #define UART_ACK_RESPONSE           0xff
      // uart: components names 
      #define UART_COMP_SHIMMER           0x01
      #define UART_COMP_BAT               0x02
      #define UART_COMP_DAUGHTER_CARD     0x03
      #define UART_COMP_GSR               0x05
      // uart: property names 
      // component == UART_COMP_SHIMMER:
      #define UART_PROP_ENABLE            0x00
      #define UART_PROP_SAMPLE_RATE       0x01
      #define UART_PROP_MAC               0x02
      #define UART_PROP_VER               0x03
      #define UART_PROP_RWC_CFG_TIME      0x04
      #define UART_PROP_CURR_LOCAL_TIME   0x05
      #define UART_PROP_INFOMEM           0x06
      // component == UART_COMP_BAT:
      #define UART_PROP_VALUE             0x02
      // component == UART_COMP_DAUGHTER_CARD:
      #define UART_PROP_CARD_ID           0x02
      #define UART_PROP_CARD_MEM          0x03
      
      
♦BlueTooth command configurations
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
      - SET_DERIVED_CHANNEL_BYTES
      - GET_DERIVED_CHANNEL_BYTES
      - START_SDBT_COMMAND
      - GET_STATUS_COMMAND
      - SET_TRIAL_CONFIG_COMMAND
      - GET_TRIAL_CONFIG_COMMAND
      - SET_CENTER_COMMAND
      - GET_CENTER_COMMAND
      - SET_SHIMMERNAME_COMMAND
      - GET_SHIMMERNAME_COMMAND
      - SET_EXPID_COMMAND
      - GET_EXPID_COMMAND
      - SET_MYID_COMMAND
      - GET_MYID_COMMAND
      - SET_NSHIMMER_COMMAND
      - GET_NSHIMMER_COMMAND
      - SET_CONFIGTIME_COMMAND
      - GET_CONFIGTIME_COMMAND
      - GET_DIR_COMMAND
      - SET_CRC_COMMAND


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


   The assignment of the selected sensors field is as follows:
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

       
♦Changelog:
   v0.6.0 (06 October 2015)
      skip50ms added before logging
      bug fixes
      uart infomem range 0-1ff now
      default param values
   v0.5.0 (20 May 2015)
      gsr performance improved
   v0.4.0 (27 March 2015)
      button press disabled when configuring      
      UART command improvement
         - UART baudrate increased to 115200 (up from 9600)
         - more UART commands supported
         - UART command structure changed
      new calibration file format
         - using individual calib files
         - keyword in the files removed      
   v0.3.0 (19 December 2013)
      more UART commands supported
      'ID' replaced by 'id' 
      support for bluetooth baud rate change
      support for crc check in BT transmission
   v0.2.0 (17 July 2014)
      initial release