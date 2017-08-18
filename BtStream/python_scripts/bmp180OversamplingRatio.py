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
   print "A second argument can be specified to change BMP180 Oversampling ratio to new value"
   print "If no second argument the current range is read and displayed"
   print "example:"
   print "   bmp180OversamplingRatio.py Com12"
   print "or"
   print "   bmp180OversamplingRatio.py Com12 0x01"
   print "or"
   print "   bmp180OversamplingRatio.py /dev/rfcomm0"
   print "or"
   print "   bmp180OversamplingRatio.py /dev/rfcomm0 4"
else:
   if len(sys.argv) == 2:
      ser = serial.Serial(sys.argv[1], 115200)
      ser.flushInput()
# send get BMP180 Oversampling Ratio command
      ser.write(struct.pack('B', 0x54))
      wait_for_ack()

      ddata = ""
      bmp180oversamplingratioresponse = struct.pack('B', 0x53) #BMP180_PRES_OVERSAMPLING_RATIO_RESPONSE      
      while ddata != bmp180oversamplingratioresponse:
         ddata = ser.read(1)

      bmp180oversamplingratio = struct.unpack('B', ser.read(1))
      print "Current BMP180 Oversampling ratio is: 0x%02x" % (bmp180oversamplingratio)
#close serial port
      ser.close()
   elif len(sys.argv) == 3:
      newratio = int(sys.argv[2], 0)
      if 0 <= newratio <= 3:
         ser = serial.Serial(sys.argv[1], 115200)
         ser.flushInput()
# send set BMP180 oversampling ratio command
         ser.write(struct.pack('BB', 0x52, newratio))
         wait_for_ack()
         print "BMP180 Oversampling ratio set to: 0x%02x" % (newratio)
         ser.close()
      else:
         print "%d is not a valid BMP180 Oversampling ratio\nMust be between 0 and 3" % newratio
#close serial port
   print
   print "All done"
