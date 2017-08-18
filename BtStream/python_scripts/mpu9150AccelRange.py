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
   print "A second argument can be specified to change MPU9150 Accel range to new value"
   print "If no second argument the current range is read and displayed"
   print "example:"
   print "   mpu9150AccelRange.py Com12"
   print "or"
   print "   mpu9150AccelRange.py Com12 0x01"
   print "or"
   print "   mpu9150AccelRange.py /dev/rfcomm0"
   print "or"
   print "   mpu9150AccelRange.py /dev/rfcomm0 4"
else:
   if len(sys.argv) == 2:
      ser = serial.Serial(sys.argv[1], 115200)
      ser.flushInput()
# send get MPU9150 Accel Range command
      ser.write(struct.pack('B', 0x51))
      wait_for_ack()

      ddata = ""
      mpu9150accelrangeresponse = struct.pack('B', 0x50) #MPU9150_ACCEL_RANGE_RESPONSE      
      while ddata != mpu9150accelrangeresponse:
         ddata = ser.read(1)

      mpu9150accelrange = struct.unpack('B', ser.read(1))
      print "Current MPU9150 Accel range is: 0x%02x" % (mpu9150accelrange)
#close serial port
      ser.close()
   elif len(sys.argv) == 3:
      newrange = int(sys.argv[2], 0)
      if 0 <= newrange <= 3:
         ser = serial.Serial(sys.argv[1], 115200)
         ser.flushInput()
# send set MPU9150 accel range command
         ser.write(struct.pack('BB', 0x4F, newrange))
         wait_for_ack()
         print "MPU9150 Accel range set to: 0x%02x" % (newrange)
         ser.close()
      else:
         print "%d is not a valid MPU9150 Accel range\nMust be between 0 and 4" % newrange
#close serial port
   print
   print "All done"
