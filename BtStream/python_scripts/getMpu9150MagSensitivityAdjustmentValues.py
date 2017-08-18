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
   print "example:"
   print "   getMpu9150MagSensitivityAdjustmentValues.py Com12"
   print "or"
   print "   getMpu9150MagSensitivityAdjustmentValues.py /dev/rfcomm0"
else:
   ser = serial.Serial(sys.argv[1], 115200)
   ser.flushInput()
# send the get MPU9150 mag sensitivity adjustment values command 
   ser.write(struct.pack('B', 0x5D))

# read the acknowledgement
   wait_for_ack()
   print "Acknowledgement received for get mag sensitivity adjustment values"

   ddata = ""
   response = struct.pack('B', 0x5C)
   while ddata != response:
      ddata = ser.read(1)

   ddata = ""
   numbytes = 0
   framesize = 3
   while numbytes < framesize:
      ddata += ser.read(framesize)
      numbytes = len(ddata)

   data = ddata[0:framesize]
   (adjX, adjY, adjZ) = struct.unpack('BBB', data)
   print "Mpu9150 Sensitivity Adjustment values: 0x%02x 0x%02x 0x%02x" % (adjX, adjY, adjZ)

#close serial port
   ser.close()
   print "All done"
