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
   print "   resetDefaultConfig.py Com12"
   print "or"
   print "   resetDefaultConfig.py /dev/rfcomm0"
else:
   if len(sys.argv) == 2:
      ser = serial.Serial(sys.argv[1], 115200)
      ser.flushInput()
# send reset default config command
      ser.write(struct.pack('B', 0x5A))
      wait_for_ack()
      print "Reset to default configuration"
#close serial port
      ser.close()
   print
   print "All done"
