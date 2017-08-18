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
   print "A second argument can be specified to change the Bluetooth communication" 
   print "baud rate to new value"
   print "If no second argument the current setting is read and displayed"
   print "There are 11 allowable ranges:"
   print "   0=115.2K(default), 1=1200, 2=2400, 3=4800, 4=9600, 5=19.2K, 6=38.4K,"
   print "   7=57.6K, 8=230.4K, 9=460.8K, 10=921.6K, else revert to default"
   print "example:"
   print "   btCommsBaudRate.py Com12"
   print "or"
   print "   btCommsBaudRate.py Com12 0x01"
   print "or"
   print "   btCommsBaudRate.py /dev/rfcomm0"
   print "or"
   print "   btCommsBaudRate.py /dev/rfcomm0 4"
else:
   baudrate = ["115200", "1200", "2400", "4800", "9600", "19200", "38400", "57600", "230400", "460800", "921600"]
   if len(sys.argv) == 2:
      ser = serial.Serial(sys.argv[1], 115200)
      ser.flushInput()
# send get bt comms baud rate command
      ser.write(struct.pack('B', 0x6C))
      wait_for_ack()

      ddata = ""
      btcommsbaudrateresponse = struct.pack('B', 0x6B) #BT_COMMS_BAUD_RATE_RESPONSE      
      while ddata != btcommsbaudrateresponse:
         ddata = ser.read(1)

      (btcommsbaudrate,) = struct.unpack('B', ser.read(1))
      print "Current Bluetooth communication baud rate is: 0x%02x (%s baud)" % (btcommsbaudrate, baudrate[btcommsbaudrate])
#close serial port
      ser.close()
   elif len(sys.argv) == 3:
      newrate = int(sys.argv[2], 0)
      if 0 <= newrate <= 10:
         ser = serial.Serial(sys.argv[1], 115200)
         ser.flushInput()
# send set bt comms rate command
         ser.write(struct.pack('BB', 0x6A, newrate))
         wait_for_ack()
         print "Bluetooth communication baud rate will be set to: 0x%02x (%s buad) after disconnect" % (newrate, baudrate[newrate])
         ser.close()
      else:
         print "%d is not a valid rate\nMust be between 0 and 10" % newrate
#close serial port
   print
   print "All done"
