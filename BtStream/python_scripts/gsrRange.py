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
   print "A second argument can be specified to change GSR range to new value"
   print "If no second argument the current range is read and displayed"
   print "example:"
   print "   gsrRange.py Com12"
   print "or"
   print "   gsrRange.py Com12 0x01"
   print "or"
   print "   gsrRange.py /dev/rfcomm0"
   print "or"
   print "   gsrRange.py /dev/rfcomm0 4"
else:
   if len(sys.argv) == 2:
      ser = serial.Serial(sys.argv[1], 115200)
      ser.flushInput()
# send get GSR Range command
      ser.write(struct.pack('B', 0x23))
      wait_for_ack()

      ddata = ""
      gsrrangeresponse = struct.pack('B', 0x22) #GSR_RANGE_RESPONSE      
      while ddata != gsrrangeresponse:
         ddata = ser.read(1)

      gsrrange = struct.unpack('B', ser.read(1))
      print "Current GSR range is: 0x%02x" % (gsrrange)
#close serial port
      ser.close()
   elif len(sys.argv) == 3:
      newrange = int(sys.argv[2], 0)
      if 0 <= newrange <= 4:
         ser = serial.Serial(sys.argv[1], 115200)
         ser.flushInput()
# send set GSR range command
         ser.write(struct.pack('BB', 0x21, newrange))
         wait_for_ack()
         print "GSR range set to: 0x%02x" % (newrange)
         ser.close()
      else:
         print "%d is not a valid GSR range\nMust be between 0 and 5" % newrange
#close serial port
   print
   print "All done"
