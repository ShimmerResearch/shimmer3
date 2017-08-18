#!/usr/bin/python
import sys, struct, serial

def wait_for_ack():
   ddata = ""
   ack = struct.pack('B', 0xff)
   while ddata != ack:
      ddata = ser.read(1)
   return


if len(sys.argv) < 6 and len(sys.argv) != 2:
   print "no device or configuration arguments specified"
   print "You need to specify the serial port of the device you wish to connect to"
   print "A second, third, fourth and fifth argument must be specified to change the configured settings"
   print "if these arguments are not present the configuration values are read and displayed"
   print "example:"
   print "   setConfigBytes.py Com12 0x11 0x22 0x33 0x44"
   print "or"
   print "   setConfigBytes.py Com12"
   print "or"
   print "   setConfigBytes.py /dev/rfcomm0 17 34 51 58"
   print "or"
   print "   setConfigBytes.py /dev/rfcomm0"
else:
   if len(sys.argv) == 2:
      ser = serial.Serial(sys.argv[1], 115200)
      ser.flushInput()
# send the get config setup bytes command
      ser.write(struct.pack('B', 0x10))
      wait_for_ack()

      ddata = ""
      configbytesresponse = struct.pack('B', 0x0F) #CONFIG_SETUP_BYTES_RESPONSE
      while ddata != configbytesresponse:
         ddata = ser.read(1)

      ddata = ""
      numbytes = 0
      framesize = 4
      while numbytes < framesize:
         ddata += ser.read(framesize)
         numbytes = len(ddata)
      data = ddata[0:framesize]
      (config0, config1, config2, config3) = struct.unpack('BBBB', data)
      print "Config Byte0: 0x%02x\nConfig Byte1: 0x%02x\nConfig Byte2: 0x%02x\nConfig Byte3: 0x%02x" % (config0, config1, config2, config3)
#close serial port
      ser.close()
      print "All done"

   else: 
# send get sampling rate command
      newconfig0 = int(sys.argv[2], 0)
      newconfig1 = int(sys.argv[3], 0)
      newconfig2 = int(sys.argv[4], 0)
      newconfig3 = int(sys.argv[5], 0)
      if (0 <= newconfig0 <=255) and (0 <= newconfig1 <=255) and (0 <= newconfig2 <=255) and (0 <= newconfig3 <= 255): 
         ser = serial.Serial(sys.argv[1], 115200)
         ser.flushInput()
# send set sensors command
         ser.write(struct.pack('BBBBB', 0x0E, newconfig0, newconfig1, newconfig2, newconfig3))
         wait_for_ack()
         print "Configuration bytes set to: 0x%02x 0x%02x 0x%02x 0x%02x" % (newconfig0, newconfig1, newconfig2, newconfig3)
#close serial port
         ser.close()
      else:
         print "0x%02x 0x%02x 0x%02x is not a valid configuration" % (newconfig0, newconfig1, newconfig2)
   print
   print "All done"
