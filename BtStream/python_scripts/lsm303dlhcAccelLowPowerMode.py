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
   print "A second argument can be specified to change LSM303DLHC Accel low power mode to new value"
   print "If no second argument the current low power mode setting is read and displayed"
   print "example:"
   print "   lsm303dlhcAccelLowPowerMode.py Com12"
   print "or"
   print "   lsm303dlhcAccelLowPowerMode.py Com12 0x01"
   print "or"
   print "   lsm303dlhcAccelLowPowerMode.py /dev/rfcomm0"
   print "or"
   print "   lsm303dlhcAccelLowPowerMode.py /dev/rfcomm0 0"
else:
   if len(sys.argv) == 2:
      ser = serial.Serial(sys.argv[1], 115200)
      ser.flushInput()
# send get LSM303DLHC Accel LP mode command
      ser.write(struct.pack('B', 0x45))
      wait_for_ack()

      ddata = ""
      lsm303dlhcaccellowpowermoderesponse = struct.pack('B', 0x44) #LSM303DLHC_ACCEL_LPMODE_RESPONSE
      while ddata != lsm303dlhcaccellowpowermoderesponse:
         ddata = ser.read(1)

      lsm303dlhcaccellowpowermode = struct.unpack('B', ser.read(1))
      print "Current LSM303DLHC Accel low power mode setting is: 0x%02x" % (lsm303dlhcaccellowpowermode)
#close serial port
      ser.close()
   elif len(sys.argv) == 3:
      newlowpowermode = int(sys.argv[2], 0)
      if 0 <= newlowpowermode <= 1:
         ser = serial.Serial(sys.argv[1], 115200)
         ser.flushInput()
# send set LSM303DLHC accel LP mode command
         ser.write(struct.pack('BB', 0x43, newlowpowermode))
         wait_for_ack()
         print "LSM303DLHC Accel low power mode setting is set to: 0x%02x" % (newlowpowermode)
         ser.close()
      else:
         print "%d is not a valid LSM303DLHC Accel low power mode setting\nMust be 0 or 1" % newlowpowermode
#close serial port
   print
   print "All done"
