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
   v0.9.0 (27 March 2014)
      UART command improvement
         - UART baudrate increased to 115200 (up from 9600)
         - more UART commands supported
         - UART command structure changed
      new multi shimmer sync method
         - new parameters added
         - 1h max communication interval
         - max 20 slaves supported
         - short experiment mode supported
      new calibration file format
         - using individual calib files
         - keyword in the files removed
      parameter derived_channels added in sdlog.cfg file/sdheader      
   v0.8.0 (10 October 2014)
      button disabled when configuring
      support for more UARTcommands
   v0.7.0 (04 July 2014)
      support for bridge amp
      support for quaternion (MPL)
   v0.6.0 (04 April 2014)
      support for exg (16bit and 24bit)
      isolated driver files
      256 byte header implemented   
   v0.5.0 (22 January 2014)
      mac address stored in infomem now  
   v0.4.0 (17 January 2014)
      support for tcxo mode
      support for gsr sensor
      support for expansion board power
      support for eeprom
      support for shimmer3_rev4 board
      sd header updated
      multi shimmer sync improvement
         - interval time default (and minimum) 54s
         - sync won't work when shimmer is docked
   v0.3.0 (19 December 2013)
      added mac address enquiry command - through UART
      support for bmp180 temperature and pressure sensor
      sdbuffer size changed to 512 (down from 1024)
   v0.2.0 (11 October 2013)
      write to sd card every 1 minute or when user button is pressed
   v0.1.0 (09 October 2013)
      initial release