#!/usr/bin/python
import sys, struct, serial

def wait_for_ack():
   ddata = ""
   ack = struct.pack('B', 0xff)
   while ddata != ack:
      ddata = ser.read(1)
   return


if len(sys.argv) < 5:
   print "no device or sensors argument specified"
   print "You need to specify the serial port of the device you wish to connect to"
   print "A second, third and fourth argument must be specified to change the configured sensors"
   print "example:"
   print "   setSensors.py Com12 0xC0 0x00 0x80"
   print "or"
   print "   setSensors.py /dev/rfcomm0 192 0 128"
else:
# send get sampling rate command
   newsensors0 = int(sys.argv[2], 0)
   newsensors1 = int(sys.argv[3], 0)
   newsensors2 = int(sys.argv[4], 0)
   if (0 <= newsensors0 <=255) and (0 <= newsensors1 <=255) and (0 <= newsensors2 <=255): 
      ser = serial.Serial(sys.argv[1], 115200)
      ser.flushInput()
# send set sensors command
      ser.write(struct.pack('BBBB', 0x08, newsensors0, newsensors1, newsensors2))
      wait_for_ack()
      print "Sensors configuration set to: 0x%02x 0x%02x 0x%02x" % (newsensors0, newsensors1, newsensors2)
#close serial port
      ser.close()
   else:
      print "0x%02x 0x%02x 0x%02x is not a valid sensor configuration" % (newsensors0, newsensors1, newsensors2)
   print
   print "All done"
