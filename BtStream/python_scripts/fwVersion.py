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
   print "   deviceVersion.py Com12"
   print "or"
   print "   deviceVersion.py /dev/rfcomm0"
else:
   ser = serial.Serial(sys.argv[1], 115200)
   ser.flushInput()
# send the get device version command 
   ser.write(struct.pack('B', 0x2E))

# read the acknowledgement
   wait_for_ack()
   print "Acknowledgement received for get firmware version command"

   ddata = ""
   fwversionresponse = struct.pack('B', 0x2F)  #FW_VERSION_RESPONSE
   while ddata != fwversionresponse:
      ddata = ser.read(1)

   ddata = ""
   numbytes = 0
   framesize = 6
   while numbytes < framesize:
      ddata += ser.read(framesize)
      numbytes = len(ddata)

   data = ddata[0:framesize]
   (identifier, majorver, minorver, releasever) = struct.unpack('HHBB', data)
   print "Firmware identifier: %d" % (identifier)
   print "Firmware version: %d.%d.%d" % (majorver, minorver, releasever)

#close serial port
   ser.close()
   print "All done"
