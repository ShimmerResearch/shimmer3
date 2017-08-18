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
   print "A second argument can be specified to change which LED is active"
   print "   Green = 0, Yellow = 1, Red = 2."
   print "If no second argument the currently selected LED setting is read and displayed"
   print "example:"
   print "   chargeStatusLed.py Com12"
   print "or"
   print "   chargeStatusLed.py Com12 0x02"
   print "or"
   print "   chargeStatusLed.py /dev/rfcomm0"
   print "or"
   print "   chargeStatusLed.py /dev/rfcomm0 0"
else:
   if len(sys.argv) == 2:
      ser = serial.Serial(sys.argv[1], 115200)
      ser.flushInput()
# send get charge status led command
      ser.write(struct.pack('B', 0x32))
      wait_for_ack()

      ddata = ""
      chargestatusledresponse = struct.pack('B', 0x31) #CHARGE_STATUS_LED_RESPONSE      
      while ddata != chargestatusledresponse:
         ddata = ser.read(1)

      (chargeStatusLed,) = struct.unpack('B', ser.read(1))

      print "Current Led set is:",
      if chargeStatusLed == 0:
         print "green"
      elif chargeStatusLed == 1:
         print "yellow"
      elif chargeStatusLed == 2:
         print "red"
      else:
         print "undefined (%d)" % chargeStatusLed
      ser.close()
   elif len(sys.argv) == 3:
      newsetting = int(sys.argv[2])
      if 0 <= newsetting <= 2:
         ser = serial.Serial(sys.argv[1], 115200)
         ser.flushInput()
# send set charge status led command
         ser.write(struct.pack('BB', 0x30, newsetting))
         wait_for_ack()
         print "Charge status LED set to: %d" % (newsetting)
         ser.close()
      else:
         print "%d is not a valid charge status LED setting\nMust be between 0 and 2" % newsetting
#close serial port
   print
   print "All done"
