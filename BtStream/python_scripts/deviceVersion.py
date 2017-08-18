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
   ser.write(struct.pack('B', 0x3F))

# read the acknowledgement
   ddata = ""
   ack = struct.pack('B', 0xff)
   while ddata != ack:
      ddata = ser.read(1)
   print "Acknowledgement received for get device version command"

   ddata = ""
   deviceversionresponse = struct.pack('B', 0x25)  #DEVICE_VERSION_RESPONSE
   while ddata != deviceversionresponse:
      ddata = ser.read(1)
   deviceversion = struct.unpack('B', ser.read(1))
   if deviceversion[0] == 0:
      print "Device Version: shimmer 1.3"
   elif deviceversion[0] == 1:
      print "Device Version: shimmer 2"
   elif deviceversion[0] == 2:
      print "Device Version: shimmer 2r"
   elif deviceversion[0] == 3:
      print "Device Version: shimmer 3"
   elif deviceversion[0] == 4:
      print "Device Version: SR30"

#close serial port
   ser.close()
   print "All done"
