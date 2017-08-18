#!/usr/bin/python
import sys, struct, serial

BT_COMMS_BAUD_RATE_RESPONSE  =  0x6B
GET_BT_COMMS_BAUD_RATE       =  0x6C

baudRates = [115200, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 230400, 460800, 921600]

def wait_for_ack():
   ddata = ""
   ack = struct.pack('B', 0xff)
   while ddata != ack:
      ddata = ser.read(1)
      # print "0x%02x" % ord(ddata[0])
     
   return

if len(sys.argv) < 2:
   print "no device specified"
   print "You need to specify the serial port of the device you wish to connect to"
   print "example:"
   print "   aAccel5Hz.py Com12"
   print "or"
   print "   aAccel5Hz.py /dev/rfcomm0"
else:
   ser = serial.Serial(sys.argv[1], 115200)
   ser.flushInput()
   print "port opening, done."
# get the baud rate
   try:
      ser.write(struct.pack('B', GET_BT_COMMS_BAUD_RATE))
      wait_for_ack()   
      print "baud rate request, done.\n"

      ddata = ""
      calibcoeffsresponse = struct.pack('B', BT_COMMS_BAUD_RATE_RESPONSE)
      while ddata != calibcoeffsresponse:
         ddata = ser.read(1)        

      [baudRate,] = struct.unpack('B', ser.read(1))
      print "Baud rate index: %d" % baudRate
      print "Baud rate value: %d" % baudRates[baudRate]

   except KeyboardInterrupt:
#send stop streaming command
      ser.write(struct.pack('B', 0x20))
      wait_for_ack()
#close serial port
      ser.close()
      print
      print "All done"
