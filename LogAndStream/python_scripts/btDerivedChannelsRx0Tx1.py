#!/usr/bin/python
import sys, struct, serial, time, msvcrt, binascii

SET_DERIVED_CHANNEL_BYTES                    = 0x6D
DERIVED_CHANNEL_BYTES_RESPONSE               = 0x6E
GET_DERIVED_CHANNEL_BYTES                    = 0x6F
UPD_SDLOG_CFG_COMMAND                        = 0x9C

def wait_for_ack():
   ddata = ""
   ack = struct.pack('B', 0xff)
   while ddata != ack:
      ddata = ser.read(1)
      print "in : 0x%02x" % ord(ddata[0])	  
   return
   
def wait_for_byte(a):
   ddata = ""
   ack = struct.pack('B', a)
   while ddata != ack:
      ddata = ser.read(1)
      print "in : 0x%02x" % ord(ddata[0])	  
   return
   
def print_usage():
   print """select: 
0 to rx
1 to tx
2 to save to sdlog.cfg
      """

if len(sys.argv) < 2:
   print "no device specified"
   print "You need to specify the serial port of the device you wish to connect to"
   print "example:"
   print "   xxx.py Com12"
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

   txArg = [0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0] 

   try:
      print_usage()
      while(1):
         choice = msvcrt.getch()
         if choice == '1':
            print "out: 0x%02x" % SET_DERIVED_CHANNEL_BYTES
            ser.write(struct.pack('B', SET_DERIVED_CHANNEL_BYTES))
            print "out:",
            for i in range(8):
               txArg[i] += i 
               txArg[i] %= 256
               print hex(txArg[i]),
            print
            ser.write(txArg)
            wait_for_ack()
         elif  choice == '0':
            print "out: 0x%02x" % GET_DERIVED_CHANNEL_BYTES
            ser.write(struct.pack('B', GET_DERIVED_CHANNEL_BYTES))      
            wait_for_byte(DERIVED_CHANNEL_BYTES_RESPONSE)
            buf_len = ser.inWaiting()
            rxData = ser.read(8)
            print "in :", binascii.hexlify(rxData)
         elif  choice == '2':
            print "out: 0x%02x" % UPD_SDLOG_CFG_COMMAND
            ser.write(struct.pack('B', UPD_SDLOG_CFG_COMMAND))
            wait_for_ack()
         else:
            ser.close()
            print "All done"
            exit()
         
   except KeyboardInterrupt:
      ser.close()
      print "All done"
