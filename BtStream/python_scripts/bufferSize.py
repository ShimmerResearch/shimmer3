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
#   print "A second argument can be specified to change the buffer size to a new value"
#   print "If no second argument the current buffer size is read and displayed"
   print "example:"
   print "   bufferSize.py Com12"
#   print "or"
#   print "   bufferSize.py Com12 0x01"
   print "or"
   print "   bufferSize.py /dev/rfcomm0"
#   print "or"
#   print "   bufferSize.py /dev/rfcomm0 0"
else:
   if len(sys.argv) == 2:
      ser = serial.Serial(sys.argv[1], 115200)
      ser.flushInput()
# send get buffer size command
      ser.write(struct.pack('B', 0x36))
      wait_for_ack()

      ddata = ""
      buffersizeresponse = struct.pack('B', 0x35) #BUFFER_SIZE_RESPONSE      
      while ddata != buffersizeresponse:
         ddata = ser.read(1)

      buffersize = struct.unpack('B', ser.read(1))
      print "Current buffer size setting is: 0x%02x" % (buffersize)
#close serial port
      ser.close()
#   elif len(sys.argv) == 3:
#      newbuffersize = int(sys.argv[2], 0)
#      if 0 <= newbuffersize <= 255:
#         ser = serial.Serial(sys.argv[1], 115200)
#         ser.flushInput()
# send set Buffer Size command
#         ser.write(struct.pack('BB', 0x34, newbuffersize))
#         wait_for_ack()
#         print "Buffer size is set to: 0x%02x" % (newbuffersize)
#         ser.close()
#      else:
#         print "%d is not a valid buffer size setting\nMust be between 0 and 255" % newbuffersize
#close serial port
   print
   print "All done"
