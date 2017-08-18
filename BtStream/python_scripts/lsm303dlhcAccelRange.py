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
   print "A second argument can be specified to change LSM303DLHC Accel range to new value"
   print "If no second argument the current range is read and displayed"
   print "example:"
   print "   lsm303dlhcAccelRange.py Com12"
   print "or"
   print "   lsm303dlhcAccelRange.py Com12 0x01"
   print "or"
   print "   lsm303dlhcAccelRange.py /dev/rfcomm0"
   print "or"
   print "   lsm303dlhcAccelRange.py /dev/rfcomm0 4"
else:
   if len(sys.argv) == 2:
      ser = serial.Serial(sys.argv[1], 115200)
      ser.flushInput()
# send get LSM303DLHC Accel Range command
      ser.write(struct.pack('B', 0x0B))
      wait_for_ack()

      ddata = ""
      lsm303dlhcaccelrangeresponse = struct.pack('B', 0x0A) #LSM303DLHC_ACCEL_RANGE_RESPONSE      
      while ddata != lsm303dlhcaccelrangeresponse:
         ddata = ser.read(1)

      lsm303dlhcaccelrange = struct.unpack('B', ser.read(1))
      print "Current LSM303DLHC Accel range is: 0x%02x" % (lsm303dlhcaccelrange)
#close serial port
      ser.close()
   elif len(sys.argv) == 3:
      newrange = int(sys.argv[2], 0)
      if 0 <= newrange <= 3:
         ser = serial.Serial(sys.argv[1], 115200)
         ser.flushInput()
# send set LSM303DLHC accel range command
         ser.write(struct.pack('BB', 0x09, newrange))
         wait_for_ack()
         print "LSM303DLHC Accel range set to: 0x%02x" % (newrange)
         ser.close()
      else:
         print "%d is not a valid LSM303DLHC Accel range\nMust be between 0 and 4" % newrange
#close serial port
   print
   print "All done"
