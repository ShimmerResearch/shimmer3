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
   Byte #:|   0-2   | 3~4  | 5~6  | ... |2x~2x+2|2x+3~2x+4|2x+5~2x+6| ... |2x+2y~2x+wy+2|
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
03Mar2017 0.7.5
   bug fix: LSM303dlhc mag fix

06Dec2016 0.7.1
   improve: DERIVED_CHANNELS increased to 8 bytes, locations:   
      #define NV_DERIVED_CHANNELS_0             31
      #define NV_DERIVED_CHANNELS_1             32
      #define NV_DERIVED_CHANNELS_2             33   
      #define NV_DERIVED_CHANNELS_3             118
      #define NV_DERIVED_CHANNELS_4             119
      #define NV_DERIVED_CHANNELS_5             120
      #define NV_DERIVED_CHANNELS_6             121
      #define NV_DERIVED_CHANNELS_7             122      
      #define SDH_DERIVED_CHANNELS_0            40
      #define SDH_DERIVED_CHANNELS_1            41
      #define SDH_DERIVED_CHANNELS_2            42
      #define SDH_DERIVED_CHANNELS_3            217
      #define SDH_DERIVED_CHANNELS_4            218
      #define SDH_DERIVED_CHANNELS_5            219
      #define SDH_DERIVED_CHANNELS_6            220
      #define SDH_DERIVED_CHANNELS_7            221      

14Sep2016 0.7.0
	uprev

09Sep2016 0.6.19
	bug fix: initial timestamp (5 bytes) further change to 2nd and on files

07Sep2016 0.6.18
	bug fix: initial timestamp (5 bytes) are now the same as the first data timestamp.

02Sep2016 0.6.17	
	bug fix: when logging, daughter card ID (first 16 bytes) can be correctly polled through BT now. If during logging, the app tries to poll daughter card memory (not ID), 0xFF instead of the real daughter card memory will be returned.

31Aug2016 0.6.16
	bug fix: Consensys/CON-251 : Logging to SD card stopped - unexpected 
		fixed by adding (&& !sensing ) in sdlogcfgUpdate

30Aug2016 0.6.15
	bug fix: bmp180calib into sdheader before startStreaming

29Aug2016 0.6.14	
	improve: if any BT configuration command is sent, that the InfoMem flag is set. 
		 in addition to the existing check on the flag when undocked/powered on, when the Shimmer changes state (i.e. starts logging/streaming/connected/disconnected), it should also check the flag and update the config file if needed.

26Aug2016 0.6.13
	improve: timestamp for sensors calib_dump now stored in SDHeader
		- MPL calib info removed
		- new sdheader plan \\192.168.0.139\Shimmer\archive\Apps Team\Firmware\ConfigurationHeader - SDLog_v0.12.4.xlsx
	improve: daughter card ID (first 3 bytes from EEPROM) now stored in SDHeader
	improve: new battery ADC threshold used

26Aug2016 0.6.12
	bug fix: undock_start bug

23Aug2016 0.6.11
	bug fix: calib_dump offset = 0 added.

19Aug2016 0.6.10
	bug fix: changed (res == FR_NO_FILE) to (res != FR_OK) 
	not: this version fw will show as v0.6.9, coz of the a bug...

03Aug2016 0.6.9
	improve: undocking -> creates a new Calib_dump file if there is not one.

26July2016 0.6.8
	bug fix: test_flag

26July2016 0.6.7
	improve: calib dump with default values

21July2016 0.6.6
	bug fix: initialize ramDumpFlag = 0 correctly now

20July2016 0.6.5
	bug fix: when calib dump is updated, infomem/storedConfig will get updated correctly now
	improve: shimmerCalibv0.2.4 implemented

12July2016 0.6.4
	bug fix: Problems were spotted before whereby, if a Shimmer is in the middle of streaming and a start SD logging request is called, the SD header can sometimes be messed up. Can the SD header creation be prioritised over streaming/sensing?
		solution: added  Config2SdHead() and Timestamp0ToFirstFile() after StartLogging()

12July2016 0.6.3
	improve: new BT + SD command to set/rsp/get/update_dump_file all calibration parameters using the new byte array structure;
		#define SET_CALIB_DUMP_COMMAND                          0x98
		#define RSP_CALIB_DUMP_COMMAND                          0x99
		#define GET_CALIB_DUMP_COMMAND                          0x9A
		#define UPD_CALIB_DUMP_COMMAND                          0x9B		 
	improve: new BT + SD command to Write config file after all of InfoMem is written.
		#define UPD_SDLOG_CFG_COMMAND                           0x9C
	improve: Remove the bit in the infomem that triggers an overwrite of the calibration parameters in the SD card
	bug fix: after writeConfigurationToInfoMem, updates configure_channel info for the coming inquiry command.

09Feb2016 0.6.2	
	bug fix: derived channels reset problem.

16Nov2015 0.6.1	
	bug fix: daughter card Mem protection: 
		mem starts from 0 to 2031 now. won't affect ID section (0-15) now

06102015 0.6.0
	uprev

06102015 0.5.27 (MH change requested by RM)
         bug fix: ExG registers swapped incorrectly in header

10062015 0.5.26 (MH change requested by RM)
         bug fix: RTC difference (byte 44-51) not set in header

24092015 0.5.25
	improve: button debounce improve
	
24092015 0.5.24
24092015 0.5.23
	version control
	
22092015 0.5.22
	bug fix: max_exp_len=
	improve: skip50ms
	
22092015 0.5.21
	improve: user button disabled if btIsConnected
	
16092015 0.5.20
16092015 0.5.19
	bug fix: pressure sensor sampling problem at low freq.	

11092015 0.5.18
	bug fix: set infomem
	
10092015 0.5.16
	improve: 0x80 bit in trial_config0 to indicate rtc_offset source - 1:bt, 0:uart
	improve: uart set/get_infomem uses address 0x0000-0x01ff rather than 0x1800-0x19ff now
	
10092015 0.5.15
	bug fix: set/get infomem: range check, length set to 128, update sdlog.cfg and calib files
		 using address 0x0000-0x01ff
	
09092015 0.5.14
	bug fix: remove flag
	
09092015 0.5.13
	improve: duration control: max_exp_len=2    (min)
	improve: on docking/undocking, status response improved
	improve: default param values
	
07092015 0.5.12
	improve: STOP_STREAMING_COMMAND only stops streaming now. sdlogging will stay unchanged.
	improve: STOP_SDBT_COMMAND: 0x97
	improve: led flashes when rtc not set.
	
04092015 0.5.11
	improve: DUMMY_COMMAND: 0x96

18082015 0.5.10
	improve: start/stop logging cmd
		 get/response vbatt cmd
		
		#define START_LOGGING_COMMAND                         0x92
		#define STOP_LOGGING_COMMAND                          0x93
		#define VBATT_RESPONSE                                0x94
         		*(resPacket + packet_length++) = INSTREAM_CMD_RESPONSE;
         		*(resPacket + packet_length++) = VBATT_RESPONSE;
         		memcpy((resPacket + packet_length), battVal, 3);
         		packet_length+=3;
		#define GET_VBATT_COMMAND                             0x95
	
18082015 0.5.9
	bug fix: power-up on dock also loads names from infomem to ram now

14082015 0.5.6
	bug fix: RWC_RESPONSE
	
14082015 0.5.5
	bug fix: test_flag
	
13082015 0.5.4
	improve: RTC over BT	
		#define SET_RWC_COMMAND                               0x8F
			- e.g. send: 0x8f 0x00 0x11 0x22 0x33 0x44 0x55 0x66 0x77
		#define RWC_RESPONSE                                  0x90
			- e.g. resp: 0x90 0x00 0x11 0x22 0x33 0x44 0x55 0x66 0x77
		#define GET_RWC_COMMAND                               0x91
			- e.g. send: 0x91
	improve: 3bytes ts - both log&stream
	
14072015 0.5.3
	version control...
	
13072015 0.5.2
	bug fix: shimmername and expid response in bt commands
	
28052015 0.5.1
	change: power cycle priority sequence: when no valid info + no sdlog.cfg file: flash an error
	
20052015 0.5.0
	uprev
	
19052015 0.4.6
	bug fix: gsr jumps range from 111-2-33333-2-1111 @1024Hz

15052015 0.4.5
	bug fix: sample_rate ++	

14052015 0.4.4
	bug fix: tcxo blocked from infomem/sdlog.cfg file

13052015 0.4.3
	bug fix: temperature/pressure mixed bug
	improve: GSR_smoothTrans() filters spikes under 80ms now

17042015 0.4.2
	bug fix: 0xffs in mac and rtc_diff in sdheader

08042015 0.4.1
	bug fix: a "calib out of dock won't work" bug solved
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