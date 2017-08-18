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
   print "A second argument can be specified to change LSM303DLHC Accel sampling rate to new value"
   print "If no second argument the current sampling rate is read and displayed"
   print "example:"
   print "   lsm303dlhcAccelSamplingRate.py Com12"
   print "or"
   print "   lsm303dlhcAccelSamplingRate.py Com12 0x01"
   print "or"
   print "   lsm303dlhcAccelSamplingRate.py /dev/rfcomm0"
   print "or"
   print "   lsm303dlhcAccelSamplingRate.py /dev/rfcomm0 9"
else:
   if len(sys.argv) == 2:
      ser = serial.Serial(sys.argv[1], 115200)
      ser.flushInput()
# send get LSM303DLHC Accel Sampling Rate command
      ser.write(struct.pack('B', 0x42))
      wait_for_ack()

      ddata = ""
      lsm303dlhcaccelsamplingrateresponse = struct.pack('B', 0x41) #LSM303DLHC_ACCEL_SAMPLING_RATE_RESPONSE      
      while ddata != lsm303dlhcaccelsamplingrateresponse:
         ddata = ser.read(1)

      lsm303dlhcaccelsamplingrate = struct.unpack('B', ser.read(1))
      print "Current LSM303DLHC Accel sampling rate is: 0x%02x" % (lsm303dlhcaccelsamplingrate)
#close serial port
      ser.close()
   elif len(sys.argv) == 3:
      newsamplingrate = int(sys.argv[2], 0)
      if 0 <= newsamplingrate <= 9:
         ser = serial.Serial(sys.argv[1], 115200)
         ser.flushInput()
# send set LSM303DLHC accel sampling rate command
         ser.write(struct.pack('BB', 0x40, newsamplingrate))
         wait_for_ack()
         print "LSM303DLHC Accel sampling rate set to: 0x%02x" % (newsamplingrate)
         ser.close()
      else:
         print "%d is not a valid LSM303DLHC Accel sampling rate\nMust be between 0 and 9" % newsamplingrate
#close serial port
   print
   print "All done"
