♦This is a general purpose configurable application to be used with shimmer3 and any add-on 
 daughter-cards supplied by Shimmer.


♦Log file Format:
          | SD Header | Data Buffer 1 | Data Buffer 2 ...
   Byte #:|   0~255   |    256~...    |     ...


♦Data Buffer Format:
   if sync=1
          | time diff | Data Block 1 | Data Block 2 ...
   Byte #:|    0~8    |     9~...    |    ...
   if sync=0
          | Data Block 1 | Data Block 2 ...
   Byte #:|     0~...    |    ...   
   - Data Buffer size <= 512
   - Data buffer always store integer number of Data Blocks

   
♦Data Block Format:
          |TimeStamp|Achan1|Achan2| ... |AchanX | Dchan1  | Dchan2  | ... |    DchanY   |
   Byte #:|   0-1   | 2~3  | 4~5  | ... |2x~2x+1|2x+2~2x+3|2x+4~2x+5| ... |2x+2y~2x+2y+1|
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

      
♦Sync sturcture:
   A synchronisation (sync) method include one center and a few nodes (1-20). In sync mode, 
   when logging, the center tries to connect the nodes periodically. When a connection is 
   uccessfully build, the center sends its center_local_time (8 bytes) to the current connected 
   node x. The slave compares the difference between the center_local_time and its node_x_local_time
   and stores the time difference in the front of the next "to be written" data buffer.
   
   To store the time difference, when sync is enabled, the first 9 bytes are always reserved in
   the Data Buffers. If the connection is successfully built, byte 0 is assigned 0 to indicate 
   node_x_local_time > center_local_time, 1 to indicate the opposite. the time difference between 
   the two local time values is stored in bytes 1-8. If none connection was successfully built, 
   0xff will be assigned to bytes 0-8 of the Data Buffers.
   
   To configure sync mode, in sdlog.cfg file, set: 
      sync=1 to enable sync mode
      iammaster=1 to indicate this shimmer is a center
      iammaster=0 to indicate this shimmer is a node
      node=0123456789ab to set the node names
         - each node requires a new line with "node=" in the config file.
      est_exp_len=n, in unit minute, estimated experiment length. 
         - this value must be kept the same on the center and all the nodes in the experiment
         - when n>=10, n/3 will be taken as sync interval time.
            - the center tries to communicate with each of the listed nodes every interval time.
               - on successfully connected to a node, the center won't try to connect it again.
               - the center turns off its bluetooth and stop trying to connect the nodes when
                  (1) all nodes are successfully connected once 
                     - twice if its the first connection in the whole experiment
                  (2) 800s has exipred
                  (3) (interval time - 2s) has expired
                  which ever happens first.    
            - nodes turn on their bluetooth module and tries to be connected by the center every 
              interval time.
               - on not receiving the center's connection, nodes restart their bluetooth every 50s 
               - nodes turn off their bluetooth and stop the waiting when
                  (1) a connection is successfully built
                  (2) 800s has exipred
                  (3) (interval time - 2s) has expired
                  which ever happens first.               
         - when n<10, the center and all the nodes enter the short experiment mode
            - in this mode, the center tries to connect all nodes listed in sdlog.cfg file
            - one by one, according to the sequence in sdlog.cfg file
            - when reaching the last node, the center rolls back to connect the 1st node
            - nodes turn on their bluetooth and tries to get connected
            - neither the center nor the slaves stop
      singletouch=1 to enable single touch mode,
         - in this mode, all features above are supported
         - the nodes turn on their bluetooth and try to get connected by the center even when 
           not logging. 
         - either user button press or successfully connection from the center can start logging
         - the center only controls the start, user has to stop logging manually
 
 
♦Changelog:
11Feb2016 0.12.2
	bug fix: daughter card Mem protection: 
		mem starts from 0 to 2031 now. won't affect ID section (0-15) now	
	
23092015 0.11.6
	bug fix: max_exp_len=
	improve: skip50ms
	
10092015 0.11.5
	improve: uart set/get_infomem uses address 0x0000-0x01ff rather than 0x1800-0x19ff now

09092015 0.11.4
	bug fix: remove flag

09092015 0.11.3
	improve: 3 bytes ts
	improve: gsr range default=4  (autorange)
	improve: sync led: unsuccessful sync = blue on slaves
	improve: if rtc is not set, flash error
	improve: default parameter values for SdLog

04082015 0.11.2
	improve: 50ms anti-shake on user button

25062015 0.11.1
	improve: gsr init 6 clocks removed

11062015 0.11.0
	uprev

05062015 0.10.3
	improve: led sequence for syncing changed

05062015 0.10.2
	improve: LSM303DLHC_smoothMag()
	improve: added (uartDcMemOffset>=16) for daughter card mem r/w

27052015 0.10.1
	bug fix: mpl calib

20052015 0.10.0
	uprev

19052015 0.9.17
	bug fix: gsr jumps range from 111-2-33333-2-1111 @1024Hz
	

18052015 0.9.15
	gsr_smooth back in 

18052015 0.9.14
	bug fix: uart_activate leads to 2nd gsr value always 4092

15052015 0.9.13
	bug fix: sample_rate ++ bug

15052015 0.9.11
	bug fix: sample_rate 3rd digit after decimal point: round
	change: vbatt channel now moved back to after accel_z

13052015 0.9.10
	bug fix: temperature/pressure mixed bug
	improve: GSR_smoothTrans() filters spikes under 80ms now
	change: vbatt channel now moved to end of ADC channels

11052015 0.9.9
	bug fix: press/mag mixed bug

08052015 0.9.8
	bug fix: sample_rate rounding: always flooring now

07052015 0.9.7
	improve: vbatt always on.

05052015 0.9.6
	bug fix: vbatt interval back to 10mins
	bug fix: sprintf very unreliable, replaced with manual methods.

30042015 0.9.5
	improve: code structure change
	bug fix: gsr smooth thing commented out
	bug fix: 4.9s problem seemed solved - communication uart infinite loop problem

27032015 0.9.0
	uprev

26032015 0.8.69
	improve: derived_channels, now 3bytes (up from 2bytes), location change

26032015 0.8.68
	improve: new calib scheme
	bug fix: derived_channels got copied from RAM to infomem when reading sdlog.cfg

26032015 0.8.67
	bug fix: 0 kb file created, can't stop: because of wrong use of battval callback function

25032015 0.8.66
	improve: sync: rcNodeReboot now takes the smaller value of 9 and estLen3/SYNC_WINDOW_N

25032015 0.8.65
	bug fix: battery wont show red incorrectly
	improve: derived_channels=0-65535 - as MN suggested
	improve: in single_touch, est_exp_len>10, slave doesn't wait for the 2nd connection forever, but till the window expires since the 1st connection is received.
	improve: removed keyword in individual calib files - as NOM suggested

24032015 0.8.64
	bug fix: a bug that leads to byte#251 in sdheader upcounting incorrectly (after 36h) fixed.

24032015 0.8.63
	improve: uart3.0 component+property
		 baudrate = 115200

23032015 0.8.62
	improve: led_blue flashes instead of led_all flash

23032015 0.8.61
	bug fix: test flag removal

23032015 0.8.60
	bug fix: max_exp_len will work when sync=0 now

19032015 0.8.59
	bug fix: still the 2s problem

19032015 0.8.58
	this is a version only for test

19032015 0.8.57
	bug fix: still the 2s problem

13032015 0.8.56
	bug fix: solved a 1st ts=0 bug 

12032015 0.8.55
	bug fix: changed uart command numbers back (did a change as was planning the new uart commands)

12032015 0.8.54
	bug fix: 2s problem again. removed the "don't fire new stream" flag from between adc and i2c, added a "don't read timestamp" flag after timestamp reading and before i2c is done.


06032015 0.8.53
	bug fix: 0xff in ver bytes, 0 now
	bug fix: when calib from infomem, still read from i2c for bmp180 calib info

06032015 0.8.52
	improve: first outlier won't be stored on data file - improvement
	bug fix: fixed a bug where nodes go into idle status when fail to boot up

05032015 0.8.51
	bug fix: p1ie&=~bit0 moved into rn42 driver.
	bug fix: remove 0xff at beginning
	optimization for outliers : limit back to 0.1s (down from 0.5s)
	improve: first outlier won't be stored on data file -  more improvements will be in 0852

04032015 0.8.50
	bug fix: some bug look like i2c can't timeout the sleep - reverted back to v0846 and re-implemented changes in 47, 48 and 49. tasklist removed.
	improve: shut p1ie&=~bit0 when bt starts up.
	optimization for outliers : limit increased to 0.5s (up from 0.1s)
	
17022015 0.8.49
	new calib files
	bug fix: sprintf float problem, fill with '0's

16022015 0.8.48
	bug fix: taskList

10022015 0.8.47
	bug fix: high freq 2s problem

30012015 0.8.46
	bug fix: opt of outlier 	

29012015 0.8.45
	optimization for outliers : limit reduced to 0.1s (down from 8s)

29012015 0.8.44
	master mac addr stored in the front of info B, followed by slave mac addrs, in all shimmers

28012015 0.8.43
	new sync optimized: Master restarts BT after each try, whether succeeded or failed.

26012015 0.8.42
	infomem layout changed
	optimization for outliers 

21012015 0.8.41
	new sync params fixed in FW
	there won't be a double-blue flash before BT starts
	create new sdlog.cfg file when there wasn't and infomem is available 

09012015 0.8.39/40 - 9600
	bug fix: slave radio  wont turn on

09012015 0.8.38 - 9600
	bug fix: undock - read daughter card problem

06012015 0.8.37 - 9600
	mac address in infoMem protected from UART_INFO_MEM_SET

05012015 0.8.36 - 9600
	bug fix: mac$ no response after undock-dockback

05012015 0.8.35
	bug fix: reset uart params when docking

05012015 0.8.34
	bug fix: mac addr fix

12122014 0.8.31-33
	different uart versions

12122014 0.8.30
	bug fix: another mac id problem

12122014 0.8.29
	bug fix: a mac id problem

12122014 0.8.28
	9*0xff implememted. like v0823

12122014 0.8.27
	mss bug fix: user button press on the slave doesn't reset the rcomm params
	9600
	uart commands reverse

12122014 0.8.26
	mss bug fix: fixed a node blinking status mistake
	uart-infomem: fixed a gs_range bug.

11122014 0.8.25
	uart bug fix

10122014 0.8.24
	uart bug fix

09122014 0.8.23
	test only: mss 9 byte param

09122014 0.8.22
	new uart driver - good

05122014 0.8.20
	new uart driver - beta 2

05122014 0.8.19
	new uart driver - beta 1

04122014 0.8.18
	mss: 1h max communication interval
	mss: max_exp_len has no minimun values

03122014 0.8.17
	bug fix: moved 1st timestamp to before startSensing() so as to prvent the 0x00 in ts at sample 2 (value(1,2))

02122014 0.8.14-16
	test only version: baud rate = 115200, 230400, 460800

01122014 0.8.13
	mss: on receiving center ts, node renews a 4s extension of the window

29112014 0.8.11
	new uart driver

25112014 0.8.11
	test only version: baud rate = 460800

25112014 0.8.10
	test only version: baud rate = 230400

25112014 0.8.9
	test only version: baud rate = 115200

21112014 0.8.8
	test only version: for mss, #define TESTMODE 1

20112014 0.8.7
	mss: new variable SYNC_NEXT2MATCH:=2, select from 1~SYNC_NODE_REBOOT

20112014 0.8.6
	mss: on receiving node feedback, center renews a 4s extension of the window

19112014 0.8.5
	mss: short_exp mode
	mss: new variable SYNC_T_EACHNODE_C := 12
	mss: if est_exp_len>=10, when est_exp_len expires, center-node communication still happens 
		every 1/3*est_exp_len, till stopped or battery runs out which happens earlier

14112014 0.8.3
	mss: new communication strategy - 1st version, up to 7 slaves

15102014 0.8.1
	routine communication timing schedule adjustment

10102014 0.8.0
	up rev

10102014 0.7.16
	bug fix: debounce
	button disabled when configuring

08102014 0.7.15
	bug fix: byte 251
	tim$

07102014 0.7.14
	hardware version in ver$

06102014 0.7.13
	rtcCnt byte to sdheader byte 251

03102014 0.7.12
	protections added to exg board id read failure 

17092014 0.7.10
	rtc$: 64 bits, with 32768hz resolution: 000045678901234567890rtc$

15092014 0.7.9
	bug fix: shimmerName=

10092014 0.7.8
	rtc$ command: 01234567890rtc$

07082014 0.7.7
	mac$ echo removed

07082014 0.7.6
	sd access in auto mode fixed

01082014 0.7.5
	'ID' replaced by 'id'

21072014 0.7.4
	uart0 problem fixed

18072014 0.7.2
	ver$ cmd supported
	uart0 problem fixed
	P1IES problem fixed

04072014 0.7.0
	quaternion supported (MPL)

20052014 0.6.1
	strain gauge supported

04042014 0.6.0
	=0.5.18

26032014 0.5.18
	calib bug fixed

26032014 0.5.17
	this is a testing only version
	178 byte header
	features improved from 0.5.7
	(exg/isolated libs not ready, other improvements from 0.5.8 applied)
	reset() added when file_bad > 3s - for android unmount_bad/file_bad problem

25032014 0.5.16
	a calib bug fixed.
	init timestamp improved.
	sampling freq limited to 256/512 for better performance.

20032014 0.5.15
	256 byte header implemented
	exg tested 

20032014 0.5.12-14
	testing versions

19032014 0.5.11
	set_basedir() moved to before StartStreaming()

19032014 0.5.10
	new sdheader: 186 bytes, with reserved bytes in the first 30.
	little endian for all old channels
	independent sdlog.h file

07032014 0.5.9
	exg 24bit
	tcxo timestamp replaces 32khz clk timestamp now
	no sdlog.cfg file leads to forever flashing (file_bad=1)
	new sdhead 
	forgot to change number in fw, will get ver0.5.8 in fw

06032014 0.5.8
	isolated driver files (CCS project file changes)

03032014 0.5.7
	exg
	tcxo timestamp replaces 32khz clk timestamp now

25022014 0.5.5
	cleaned up

11022014 0.5.4
	bt_bad won't make led flash different

06022014 0.5.2
	bt_bad = 0;

22012014 0.5.0
	led back
	extended init mac address time
	mac address writen in infoMem now

17012014 0.4.0
	1 bt bug fixed, some redundent functions removed.

16012014 0.3.8
	interval time default (also minimum) 54s
	bt ondock function bug fixed (won't start BT now when docked)	

15012014 0.3.7
	rev4 board i2c tested, good
	sync: center turns on the BT at start logging, and the period lasts longer than before

13012014 0.3.6
	file roll over test only

10012014 0.3.5
	rev4 board i2c tested, still some problem

10012014 0.3.4
	GSR_range and Pressure_precision bytes added in file header
	tested single touch mode and user button mode, works as expected
	bt working fine

09012014 0.3.3
	for tcxo test only
	178 byte header used
	bt not used

07012014 0.3.2
	tcxo=1 support
	gsr=1 support
	exp_power=1 support	
	eeprom support
	rev4 board support (i2c control not yet)
	sd header updated: as indicated in S:\Applications Team\Firmware\SDLog\ConfigurationHeader.xlsx 

19122013 0.3.0
	Added else slave_mode option

16122013 0.2.8
	sdbuffer size changed to 512 (was 1024)

11122013 0.2.7
	BTMac done: use mac$ as the command
	bmp180 pressure+temperature added: 
		use pres_bmp180=1 as ON/OFF parameter in config file
		use pres_bmp180_prec=0~3 as precision parameter
	singletouch+sync problem fixed
	timestamp < 1h problem fixed

28112013 0.2.6
	BTMac debug mode

28112013 0.2.5
	BTMac debug mode

28112013 0.2.4
	button bug fix

27112013 0.2.3
	added mac_address inquiry function - through UART

25102013 0.2.2
	commented out the BT_setpin="1234" statement

16102013 0.2.1
	comments added

11102013 0.2.0
	last 1min data will be lost

09102013 0.1.0
	beta version

03102013 0.0.14
	gyro sensitivity info *100

02102013 0.0.13
	memory problems located
	calibration data renewed

01102013 0.0.12
	fixed a i2c bug, memory problems need to solve

30092013 0.0.11
	new calibration data

30092013 0.0.10
	156 byte header bug fix
	fixed a "last config settings remained" problem

27092013 0.0.9
	156 byte header implemented	

25092013 0.0.8
	Removed the sdcard detect trap in the initialization.
	Sampling_peroid in sdheader in big endian now.

24092013 0.0.7
	Pressure sensor functions removed

24092013 0.0.6
	shimmer3 dock only now.

20092013 0.0.5
	led bugs fixed

20092013 0.0.4
20092013 0.0.3
	led usage implemented

16092013 0.0.2
	1st version