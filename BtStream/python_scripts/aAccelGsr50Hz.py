#!/usr/bin/python
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
   print "   aAccelGsr50Hz.py Com12"
   print "or"
   print "   aAccelGsr50Hz.py /dev/rfcomm0"
else:
   ser = serial.Serial(sys.argv[1], 115200)
   ser.flushInput()
# send the set sensors command
   ser.write(struct.pack('BBBB', 0x08, 0x84, 0x00, 0x00))  #Analog accel, GSR
   wait_for_ack()
# send the set sampling rate command
   ser.write(struct.pack('BBB', 0x05, 0x80, 0x02)) #50Hz (640 (0x0280)). Has to be done like this for alignment reasons
   wait_for_ack()
# send start streaming command
   ser.write(struct.pack('B', 0x07))
   wait_for_ack()

# read incoming data
   ddata = ""
   numbytes = 0
   framesize = 11 # 1byte packet type + 2byte timestamp + 3x2byte Analog Accel + 2byte GSR

   print "Packet Type,Timestamp,Analog AccelX,Analog AccelY,Analog AccelZ,Active Resistor,GSR"
   try:
      while True:
         while numbytes < framesize:
            ddata += ser.read(framesize)
            numbytes = len(ddata)
         
         data = ddata[0:framesize]
         ddata = ddata[framesize:]
         numbytes = len(ddata)

         (packettype,) = struct.unpack('B', data[0:1])
         (timestamp, analogacceyx, analogacceyy, analogacceyz, gsr) = struct.unpack('HHHHH', data[1:framesize])
         resistor = (gsr & 0xC000) >> 14
         gsr &= 0x3FFF
         print "0x%02x,%5d,\t%4d, %4d, %4d,\t%d, %4d" % (packettype, timestamp, analogacceyx, analogacceyy, analogacceyz, resistor, gsr)

   except KeyboardInterrupt:
#send stop streaming command
      ser.write(struct.pack('B', 0x20))
      wait_for_ack()
#close serial port
      ser.close()
      print
      print "All done"
