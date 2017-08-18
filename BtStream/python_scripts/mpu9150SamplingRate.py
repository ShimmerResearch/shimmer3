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
   print "A second argument can be specified to change MPU9150 Sampling rate to new value"
   print "If no second argument the current sampling rate setting is read and displayed"
   print "example:"
   print "   mpu9150SamplingRate.py Com12"
   print "or"
   print "   mpu9150SamplingRate.py Com12 0x01"
   print "or"
   print "   mpu9150SamplingRate.py /dev/rfcomm0"
   print "or"
   print "   mpu9150SamplingRate.py /dev/rfcomm0 0"
else:
   if len(sys.argv) == 2:
      ser = serial.Serial(sys.argv[1], 115200)
      ser.flushInput()
# send get MPU9150 Sampling rate command
      ser.write(struct.pack('B', 0x4E))
      wait_for_ack()

      ddata = ""
      mpu9150gyrorangeresponse = struct.pack('B', 0x4D) #MPU9150_SAMPLING_RATE_RESPONSE      
      while ddata != mpu9150gyrorangeresponse:
         ddata = ser.read(1)

      mpu9150gyrorange = struct.unpack('B', ser.read(1))
      print "Current MPU9150 Sampling rate setting is: 0x%02x" % (mpu9150gyrorange)
#close serial port
      ser.close()
   elif len(sys.argv) == 3:
      newsamplingrate = int(sys.argv[2], 0)
      if 0 <= newsamplingrate <= 255:
         ser = serial.Serial(sys.argv[1], 115200)
         ser.flushInput()
# send set MPU9150 Sampling Rate command
         ser.write(struct.pack('BB', 0x4C, newsamplingrate))
         wait_for_ack()
         print "MPU9150 Sampling rate setting is set to: 0x%02x" % (newsamplingrate)
         ser.close()
      else:
         print "%d is not a valid MPU9150 Sampling rate setting\nMust be between 0 and 255" % newsamplingrate
#close serial port
   print
   print "All done"
