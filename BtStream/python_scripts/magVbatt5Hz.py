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
   print "   magVbatt5Hz.py Com12"
   print "or"
   print "   magVbatt5Hz.py /dev/rfcomm0"
else:
   ser = serial.Serial(sys.argv[1], 115200)
   ser.flushInput()
# send the set sensors command
   ser.write(struct.pack('BBBB', 0x08, 0x20, 0x20, 0x00))  #mag, Vbatt
   wait_for_ack()
# send the set sampling rate command
   ser.write(struct.pack('BBB', 0x05, 0x00, 0x19)) #5.12Hz (6400 (0x19000)). Has to be done like this for alignment reasons
   wait_for_ack()
# send start streaming command
   ser.write(struct.pack('B', 0x07))
   wait_for_ack()

# read incoming data
   ddata = ""
   numbytes = 0
   framesize = 11 # 1byte packet type + 2byte timestamp + 2byte batt voltage + 3x2byte Mag

   print "Packet Type,Timestamp,Battery Voltage,Mag X,Mag Y,Mag Z"
   try:
      while True:
         while numbytes < framesize:
            ddata += ser.read(framesize)
            numbytes = len(ddata)
         
         data = ddata[0:framesize]
         ddata = ddata[framesize:]
         numbytes = len(ddata)

         (packettype) = struct.unpack('B', data[0:1])
         (timestamp, vbatt) = struct.unpack('HH', data[1:5])
         (magx, magz, magy) = struct.unpack('>hhh', data[5:framesize])
         print "0x%02x,%5d,\t%4d,\t%4d,%4d,%4d " % (packettype[0], timestamp, vbatt, magx, magy, magz)

   except KeyboardInterrupt:
#send stop streaming command
      ser.write(struct.pack('B', 0x20))
      wait_for_ack()
#close serial port
      ser.close()
      print
      print "All done"
