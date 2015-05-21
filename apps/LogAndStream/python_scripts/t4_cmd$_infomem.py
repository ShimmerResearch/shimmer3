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
   
   ser = serial.Serial(sys.argv[1], 115200)
   ser.flushInput()
   print "port opening, done."

   #raw_nb = raw_input('Enter a command:')
   #nb = int(raw_nb, 0)
   framesize = 4
      
   ddata = ""
      
   print "info Mem Set:" 
   ser.write(struct.pack('BBBBBB', 0x24, 0x12, 0x83, 0x80, 0x00, 0x18))
   i=0
   while i<128:
	  ser.write(struct.pack('B', i))
	  i = i+1;
   ser.write(struct.pack('BB', 0x95, 0xdc))
   time.sleep(0.1)   
   buf_len = ser.inWaiting()
   ddata = ser.read(buf_len)
   print binascii.hexlify(ddata)        
   time.sleep(0.1)
      
   print "info Mem get:"
   ser.write(struct.pack('BBBBBBBB', 0x24, 0x13, 0x03, 0x80, 0x00, 0x18, 0x24, 0x64)) 
   time.sleep(0.1)   
   buf_len = ser.inWaiting()
   ddata = ser.read(buf_len)
   print binascii.hexlify(ddata)        
   time.sleep(0.1)
   time.sleep(0.1)
   time.sleep(0.1)
         
	  
print 'All done'
