#!/usr/bin/python
import sys, struct, serial

def wait_for_ack():
   ddata = ""
   ack = struct.pack('B', 0xff)
   while ddata != ack:
      ddata = ser.read(1)
   return

if len(sys.argv) < 2:
   print "No device specified."
   print "Specify the serial port of the device you wish to connect to."
   print "Example:"
   print "   exgSquareWave16Bit512Hz.py Com12"
   print "or"
   print "   exgSquareWave16Bit512Hz.py /dev/rfcomm0"
else:
   ser = serial.Serial(sys.argv[1], 115200)
   ser.flushInput()
# send the set sensors command
   ser.write(struct.pack('BBBB', 0x08, 0x00, 0x00, 0x18))  #exg1-16bit and exg2-16bit
   wait_for_ack()
# send the set sampling rate command
   ser.write(struct.pack('BBB', 0x05, 0x40, 0x00)) #512Hz (32768/64=512Hz: 64 -> 0x0040; has to be done like this for alignment reasons.)
   wait_for_ack()
# configure both ADS1292R chips: enable internal reference, 500 samples per second, square-wave test signal
# exgtestsignalconfiguration = ["CONFIG1" = 2, "CONFIG2" = 163, "LOFF" = 16, "CH1SET" = 5, "CH2SET" = 5, "RLD_SENS" = 0, "LOFF_SENS" = 0, "LOFF_STAT" = 0, "RESP1" = 2, "RESP2" = 1]
# write SET_EXG_REGS_COMMAND, chip identifier ('0' for chip1, '1' for chip2), starting byte, number of bytes to write, followed by the exgtestsignalconfiguration bytes 
   ser.write(struct.pack('BBBBBBBBBBBBBB', 0x61, 0x00, 0x00, 0x0A, 0x02, 0xA3, 0x10, 0x05, 0x05, 0x00, 0x00, 0x00, 0x02, 0x01))
   wait_for_ack()
   ser.write(struct.pack('BBBBBBBBBBBBBB', 0x61, 0x01, 0x00, 0x0A, 0x02, 0xA3, 0x10, 0x05, 0x05, 0x00, 0x00, 0x00, 0x02, 0x01))
   wait_for_ack()   

# send start streaming command
   ser.write(struct.pack('B', 0x07))
   wait_for_ack()

# read incoming data
   ddata = ""
   numbytes = 0
   framesize = 13 # 1byte packet type + 2byte timestamp + 10byte ExG data

   print "Packet Type,Timestamp,Chip1 Status,Chip1 Channel1,Chip1 Channel2,Chip2 Status,Chip2 Channel1,Chip2 Channel2"
   try:
      while True:
         while numbytes < framesize:
            ddata += ser.read(framesize)
            numbytes = len(ddata)
         
         data = ddata[0:framesize]
         ddata = ddata[framesize:]
         numbytes = len(ddata)

         (packettype,) = struct.unpack('B', data[0:1])

         (timestamp, c1status) = struct.unpack('HB', data[1:4])
         (c1ch1,) = struct.unpack('>h', data[4:6])
         (c1ch2,) = struct.unpack('>h', data[6:8])
         (c2status,) = struct.unpack('B', data[8])
         (c2ch1,) = struct.unpack('>h', data[9:11])
         (c2ch2,) = struct.unpack('>h', data[11:13])
         print "0x%02x,%5d,\t0x%02x,%8d,%8d,\t0x%02x,%8d,%8d" % (packettype, timestamp, c1status, c1ch1, c1ch2, c2status, c2ch1, c2ch2)

   except KeyboardInterrupt:
#send stop streaming command
      ser.write(struct.pack('B', 0x20))
      wait_for_ack()
#close serial port
      ser.close()
      print
      print "All done!"
