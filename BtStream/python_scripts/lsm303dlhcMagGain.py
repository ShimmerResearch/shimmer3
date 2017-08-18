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
   print "A second argument can be specified to change LSM303DLHC Mag gain to new value"
   print "If no second argument the current range is read and displayed"
   print "example:"
   print "   lsm303dlhcMagGain.py Com12"
   print "or"
   print "   lsm303dlhcMagGain.py Com12 0x01"
   print "or"
   print "   lsm303dlhcMagGain.py /dev/rfcomm0"
   print "or"
   print "   lsm303dlhcMagGain.py /dev/rfcomm0 4"
else:
   if len(sys.argv) == 2:
      ser = serial.Serial(sys.argv[1], 115200)
      ser.flushInput()
# send get LSM303DLHC Mag Gain command
      ser.write(struct.pack('B', 0x39))
      wait_for_ack()

      ddata = ""
      lsm303dlhcmaggainresponse = struct.pack('B', 0x38) #LSM303DLHC_MAG_GAIN_RESPONSE      
      while ddata != lsm303dlhcmaggainresponse:
         ddata = ser.read(1)

      lsm303dlhcmaggain = struct.unpack('B', ser.read(1))
      print "Current LSM303DLHC Mag gain is: 0x%02x" % (lsm303dlhcmaggain)
#close serial port
      ser.close()
   elif len(sys.argv) == 3:
      newgain = int(sys.argv[2], 0)
      if 1 <= newgain <= 7:
         ser = serial.Serial(sys.argv[1], 115200)
         ser.flushInput()
# send set LSM303DLHC mag gain command
         ser.write(struct.pack('BB', 0x37, newgain))
         wait_for_ack()
         print "LSM303DLHC Mag gain set to: 0x%02x" % (newgain)
         ser.close()
      else:
         print "%d is not a valid LSM303DLHC Mag gain\nMust be between 1 and 7" % newgain
#close serial port
   print
   print "All done"
