#!/usr/bin/python
import sys, struct, serial
import binascii, time

def wait_for_ack():
   ddata = ""
   ack = struct.pack('B', 0xff)
   while ddata != ack:
      ddata = ser.read(1)
      print "0x%02x" % ord(ddata[0])
	  
   return
   
if len(sys.argv) < 2:
   print "no device specified"
   print "You need to specify the serial port of the device you wish to connect to"
   print "example:"
   print "   *.py Com12"
   print "or"
   print "   *.py /dev/rfcomm0"
else:
# read incoming data
   ddata = ""
   numbytes = 0
   framesize = 9 # 1byte packet type + 2byte timestamp + 3x2byte Analog Accel
   
   ser = serial.Serial(sys.argv[1], 9600)
   ser.flushInput()
   print "port opening, done."

   #raw_nb = raw_input('Enter a command:')
   #nb = int(raw_nb, 0)
   framesize = 4
   print "Expecting %d bytes:" % framesize
      
   while 1:
      ddata = ""
         
      ser.write('bat$') 
      time.sleep(0.4) 
      
      buf_len = ser.inWaiting()
      if buf_len==4:
         ddata = ser.read(framesize)         
         batt = struct.unpack('BBBB', ddata[0:4])
         batt1 = batt[0]
         batt2 = batt[1]
         dummy1 = batt[2]
         dummy2 = batt[3]
         vbatt=batt2*265+batt1;
         print "0x%02x%02x, %d" % (batt1, batt2, vbatt)
          
      else:
         ddata = ser.read(buf_len)
         print binascii.hexlify(ddata) 
         
      time.sleep(0.1)
         
	  
print 'All done'
