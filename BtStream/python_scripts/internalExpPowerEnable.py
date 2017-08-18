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
   print "A second argument can be specified to set or clear the internal expansion"
   print "power enable pin"
   print "If no second argument the current seting is read and displayed"
   print "example:"
   print "   internalExpPowerEnable.py Com12"
   print "or"
   print "   internalExpPowerEnable.py Com12 0x01"
   print "or"
   print "   internalExpPowerEnable.py /dev/rfcomm0"
   print "or"
   print "   internalExpPowerEnable.py /dev/rfcomm0 0"
else:
   if len(sys.argv) == 2:
      ser = serial.Serial(sys.argv[1], 115200)
      ser.flushInput()
# send get interal exp power enable command
      ser.write(struct.pack('B', 0x60))
      wait_for_ack()

      ddata = ""
      internalexppowerenableresponse = struct.pack('B', 0x5F) #INTERNAL_EXP_POWER_ENABLE_RESPONSE      
      while ddata != internalexppowerenableresponse:
         ddata = ser.read(1)

      internalexppowerenable = struct.unpack('B', ser.read(1))
      print "Current internal expansion power enable pin setting is: 0x%02x" % (internalexppowerenable)
#close serial port
      ser.close()
   elif len(sys.argv) == 3:
      newsetting = int(sys.argv[2], 0)
      if 0 <= newsetting <= 1:
         ser = serial.Serial(sys.argv[1], 115200)
         ser.flushInput()
# send set internal exp power enable command
         ser.write(struct.pack('BB', 0x5E, newsetting))
         wait_for_ack()
         print "Interal expansion power enable pin set to: 0x%02x" % (newsetting)
         ser.close()
      else:
         print "%d is not a valid setting\nMust be between 0 and 1" % newsetting
#close serial port
   print
   print "All done"
