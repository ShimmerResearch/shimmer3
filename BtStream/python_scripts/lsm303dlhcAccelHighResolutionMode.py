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
   print "A second argument can be specified to change LSM303DLHC Accel high resolution mode to new value"
   print "If no second argument the current high resolution mode setting is read and displayed"
   print "example:"
   print "   lsm303dlhcAccelHighResolutionMode.py Com12"
   print "or"
   print "   lsm303dlhcAccelHighResolutionMode.py Com12 0x01"
   print "or"
   print "   lsm303dlhcAccelHighResolutionMode.py /dev/rfcomm0"
   print "or"
   print "   lsm303dlhcAccelHighResolutionMode.py /dev/rfcomm0 0"
else:
   if len(sys.argv) == 2:
      ser = serial.Serial(sys.argv[1], 115200)
      ser.flushInput()
# send get LSM303DLHC Accel HR mode command
      ser.write(struct.pack('B', 0x48))
      wait_for_ack()

      ddata = ""
      lsm303dlhcaccelhighresolutionmoderesponse = struct.pack('B', 0x47) #LSM303DLHC_ACCEL_HRMODE_RESPONSE      
      while ddata != lsm303dlhcaccelhighresolutionmoderesponse:
         ddata = ser.read(1)

      lsm303dlhcaccelhighresolutionmode = struct.unpack('B', ser.read(1))
      print "Current LSM303DLHC Accel high resolution mode setting is: 0x%02x" % (lsm303dlhcaccelhighresolutionmode)
#close serial port
      ser.close()
   elif len(sys.argv) == 3:
      newhighresolutionmode = int(sys.argv[2], 0)
      if 0 <= newhighresolutionmode <= 1:
         ser = serial.Serial(sys.argv[1], 115200)
         ser.flushInput()
# send set LSM303DLHC accel HR mode command
         ser.write(struct.pack('BB', 0x46, newhighresolutionmode))
         wait_for_ack()
         print "LSM303DLHC Accel high resolution mode setting is set to: 0x%02x" % (newhighresolutionmode)
         ser.close()
      else:
         print "%d is not a valid LSM303DLHC Accel high resolution mode setting\nMust be 0 or 1" % newhighresolutionmode
#close serial port
   print
   print "All done"
