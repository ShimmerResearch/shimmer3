#!/usr/bin/python
#encoding=cp850
#the encoding must be set to match the encoding being used by the terminal this
#script is being run in (due to printing the ø symbol)
#this can be found by reading sys.stdout.encoding

import sys, struct, serial

def wait_for_ack():
   ddata = ""
   ack = struct.pack('B', 0xff)
   while ddata != ack:
      ddata = ser.read(1)
   return

if len(sys.argv) < 2:
   print "no device specified"
   print "You need to specify the serial port of the device you wish to connect to"
   print "example:"
   print "   bmp180Press100Hz.py Com12"
   print "or"
   print "   bmp180Press100Hz.py /dev/rfcomm0"
else:
   ser = serial.Serial(sys.argv[1], 115200)
   ser.flushInput()
# read the calibration coefficients
   ser.write(struct.pack('B', 0x59))
   wait_for_ack()

   ddata = ""
   calibcoeffsresponse = struct.pack('B', 0x58)
   while ddata != calibcoeffsresponse:
      ddata = ser.read(1)

   ddata = ""
   numbytes = 0
   framesize = 22
   while numbytes < framesize:
      ddata += ser.read(framesize)
      numbytes = len(ddata)
   data = ddata[0:framesize]

   (AC1, AC2, AC3, AC4, AC5, AC6, B1, B2, MB, MC, MD) = struct.unpack('>hhhHHHhhhhh', data);
   print "Calibration coefficients:\n\tAC1: %d\n\tAC2: %d\n\tAC3: %d\n\tAC4: %d\n\tAC5: %d\n\tAC6: %d\n\t B1: %d\n\t B2: %d\n\t MB: %d\n\t MC: %d\n\t MD: %d\n" % (AC1, AC2, AC3, AC4, AC5, AC6, B1, B2, MB, MC, MD)

# send the set sensors command
   ser.write(struct.pack('BBBB', 0x08, 0x00, 0x00, 0x04))  #bmp180 press
   wait_for_ack()
# set OSS value (0-3)
   OSS = 0
   ser.write(struct.pack('BB', 0x52, OSS))
   wait_for_ack()
# send the set sampling rate command
   ser.write(struct.pack('BBB', 0x05, 0x40, 0x01)) #102.4Hz (320 (0x0140)). Has to be done like this for alignment reasons
#   ser.write(struct.pack('BBB', 0x05, 0x80, 0x0C)) #10.24Hz (3200 (0x0C80)). Has to be done like this for alignment reasons
   wait_for_ack()
# send start streaming command
   ser.write(struct.pack('B', 0x07))
   wait_for_ack()

# read incoming data
   ddata = ""
   numbytes = 0
   framesize = 8 # 1byte packet type + 2byte timestamp + 2byte BMP180 Temperature + 3byte BMP180 Press

   print "Packet Type,Timestamp,Uncompensated Temperature,Temperature (øC),Uncompensated Pressure,Pressure (Pa)"
   try:
      while True:
         while numbytes < framesize:
            ddata += ser.read(framesize)
            numbytes = len(ddata)
         
         data = ddata[0:framesize]
         ddata = ddata[framesize:]
         numbytes = len(ddata)

         (packettype,) = struct.unpack('B', data[0:1])
         (timestamp,) = struct.unpack('H', data[1:3])
         (UT,) = struct.unpack('>H', data[3:5])
         (msb, lsb, xlsb) =struct.unpack('BBB', data[5:framesize])
#         print "msb: 0x%02x\tlsb: 0x%02x\txlsb: 0x%02x" % (msb, lsb, xlsb)
         UP = ((msb<<16) + (lsb<<8) + xlsb)>>(8-OSS)
         
#*******************************************************************************************************************************
# use the test values in the BMP180 datasheet (figure 4)
         '''
         AC1 = 408
         AC2 = -72
         AC3 = -14383
         AC4 = 32741
         AC5 = 32757
         AC6 = 23153
         B1 = 6190
         B2 = 4
         MB = -32767
         MC = -8711
         MD = 2868

         OSS = 0

         UT = 27898
         UP = 23843
         '''
#*******************************************************************************************************************************

         X1 = (UT - AC6) * AC5 / 32768
         X2 = MC * 2048 / (X1 + MD)
         B5 = X1 + X2
         T = (B5 + 8) / 16

         B6 = B5 - 4000
         X1 = (B2 * (B6 * B6 / 4096)) / 2048
         X2 = AC2 * B6 / 2048
         X3 = X1 + X2
         B3 = (((AC1 * 4 + X3) << OSS) + 2) / 4
         X1 = AC3 * B6 / 8192
         X2 = (B1 * (B6 * B6 / 4096)) / 65536
         X3 = ((X1 + X2) + 2) / 4
         B4 = AC4 * (X3 + 32768) / 32768
         B7 = (UP - B3) * (50000 >> OSS)
         if B7 < 0x80000000: 
            p = (B7 * 2) / B4
         else:
            p = (B7 / B4) * 2
         X1 = (int((p / 256.0) * (p / 256.0)) * 3038) / 65536
         X2 = (-7357 * p) / 65536
         p += (X1 + X2 + 3791) / 16

#         print "Temperature = %.1føC, Pressure = %dPa"  % (T/10.0, p)
         print "0x%02x,%5d,\t%4d, %0.1f,\t%4d, %d" % (packettype, timestamp, UT, T/10.0, UP, p)

   except KeyboardInterrupt:
#send stop streaming command
      ser.write(struct.pack('B', 0x20))
      wait_for_ack()
#close serial port
      ser.close()
      print
      print "All done"
