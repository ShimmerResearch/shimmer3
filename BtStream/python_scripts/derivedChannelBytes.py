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
   print "A second, third and fourth argument can be specified to change derived" 
   print "channel bytes to new values"
   print "If no additional arguments the current values are read and displayed"
   print "example:"
   print "   derivedChannelBytes.py Com12"
   print "or"
   print "   derivedChannelBytes.py Com12 1 3 9"
   print "or"
   print "   derivedChannelBytes.py /dev/rfcomm0"
   print "or"
   print "   derivedChannelBytes.py /dev/rfcomm0 2 3 255"
else:
   if len(sys.argv) == 2:
      ser = serial.Serial(sys.argv[1], 115200)
      ser.flushInput()
# send get derived channel bytes command
      ser.write(struct.pack('B', 0x6F))
      wait_for_ack()

# read incoming data
      ddata = ""
      numbytes = 0
      framesize = 4

      while numbytes < framesize:
         ddata += ser.read(framesize)
         numbytes = len(ddata)
         
      data = ddata[0:framesize]
      ddata = ddata[framesize:]
      numbytes = len(ddata)

      (packettype, byte0, byte1, byte2) = struct.unpack('BBBB', data)
#      print "Packet type: %x" % (packettype)
      print "Current derived channel bytes: %d %d %d" % (byte0, byte1, byte2)
   elif len(sys.argv) == 5:
      byte0 = int(sys.argv[2]) & 0xFF
      byte1 = int(sys.argv[3]) & 0xFF
      byte2 = int(sys.argv[4]) & 0xFF
      ser = serial.Serial(sys.argv[1], 115200)
      ser.flushInput()
# send get derived channel bytes command
      ser.write(struct.pack('BBBB', 0x6D, byte0, byte1, byte2))
      wait_for_ack()
      print "Derived channel bytes set to: %d %d %d" % (byte0, byte1, byte2)
   else:
      print "Number of arguments is not valid"
      sys.exit(-1)
#close serial port
   ser.close()
   print
   print "All done"
