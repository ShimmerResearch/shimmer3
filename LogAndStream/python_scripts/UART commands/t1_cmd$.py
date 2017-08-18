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
   
   ser.write(struct.pack('BBBB', 0x24, 0x01, 0x08, 0xbc))
   print "MAC:" 
   time.sleep(0.1)   
   buf_len = ser.inWaiting()
   ddata = ser.read(buf_len)
   print binascii.hexlify(ddata)        
   time.sleep(0.1)
      
   ser.write(struct.pack('BBBB', 0x24, 0x03, 0x4a, 0x9c))
   print "ver:" 
   time.sleep(0.1)   
   buf_len = ser.inWaiting()
   ddata = ser.read(buf_len)
   print binascii.hexlify(ddata)        
   time.sleep(0.1)
      
   ser.write(struct.pack('BBBB', 0x24, 0x05, 0x8c, 0xfc))
   print "batt:" 
   time.sleep(0.1)   
   buf_len = ser.inWaiting()
   ddata = ser.read(buf_len)
   print binascii.hexlify(ddata)        
   time.sleep(0.1)
      
   ser.write(struct.pack('BBBBBBBBBBBBB', 0x24, 0x07, 0x08, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1c, 0xa9))
   print "time set:" 
   time.sleep(0.1)   
   buf_len = ser.inWaiting()
   ddata = ser.read(buf_len)
   print binascii.hexlify(ddata)        
   time.sleep(0.1)
      
   ser.write(struct.pack('BBBB', 0x24, 0x08, 0x21, 0x2d))
   print "cfg time read:" 
   time.sleep(0.1)   
   buf_len = ser.inWaiting()
   ddata = ser.read(buf_len)
   print binascii.hexlify(ddata)        
   time.sleep(0.1)
      
   ser.write(struct.pack('BBBB', 0x24, 0x0a, 0x63, 0x0d))
   print "curr time read:" 
   time.sleep(0.1)   
   buf_len = ser.inWaiting()
   ddata = ser.read(buf_len)
   print binascii.hexlify(ddata)        
   time.sleep(0.1)
         
	  
print 'All done'
