#!/usr/bin/python
import sys, struct, serial, time

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
   print "   xxx.py Com12 3"
   print "or"
   print "   xxx.py /dev/rfcomm0"
else:
   ser = serial.Serial(sys.argv[1], 115200)
   ser.flushInput()
   print "port opening, done."
# send the set sensors command
#   ser.write(struct.pack('BBBB', 0x08, 0x80, 0x00, 0x00))  #analogaccel
#   wait_for_ack()   
#   print "sensor setting, done."
   ser.write(struct.pack('B', 0x2E))
   wait_for_ack()
   buf_len = ser.inWaiting()
   if buf_len==7:
      ddata = ser.read(buf_len)         
      fw = struct.unpack('B', ddata[0:1])
      fw = fw[0]
      (fw1, fw2, fw3, fw4, fw5, fw6 ) = struct.unpack('BBBBBB', ddata[1:7])
      print "0x%02x, %d %d %d %d %d %d" % (fw, fw1, fw2, fw3, fw4, fw5, fw6 )
      time.sleep(0.1)
      
   print "read firmware info, done."

   # send the set sampling rate command
#   ser.write(struct.pack('BBB', 0x05, 0x00, 0x19)) #5.12Hz (6400 (0x1900)). Has to be done like this for alignment reasons
#   wait_for_ack()
#   print "sampling rate setting, done."
# send start streaming command
   baudrate = int(sys.argv[2])
   ser.write(struct.pack('BB', 0x6A, baudrate))
   wait_for_ack()
   print "start command sending, done."

   try:
      while True:
         time.sleep(2)
         print "."
            

   except KeyboardInterrupt:
      ser.close()
      print "All done"
