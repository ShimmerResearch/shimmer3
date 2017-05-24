#!/usr/bin/python
import sys, struct, serial, msvcrt, math, random, binascii

def lo(data):
   return data&0xff

def hi(data):
   return (data>>8)&0xff

def wait_for_ack():
   ddata = ""
   ack = struct.pack('B', 0xff)
   while ddata != ack:
      ddata = ser.read(1)
   return

if len(sys.argv) < 2:
   print "no device specified"
   print "example:"
   print "   btCalv2Test.py Com12 "
   print "or"
   print "   btCalv2Test.py /dev/rfcomm0"
else:
   ser = serial.Serial(sys.argv[1], 115200)
   ser.flushInput()
   print "device opened"
   ack = struct.pack('B', 0xff)
   ddata = ""
   
   # send the get calibration dump command 
   addr = 0
   length_done = 0
   total_length = 173
   
   print "sending the SET_CALIB_DUMP_COMMAND, length:%3d" % total_length
   
   length_this_tx = 10
   ser.write(struct.pack('BBBB', 0x98, length_this_tx, lo(addr), hi(addr)))
   inArg = [lo(total_length), hi(total_length), 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01]
   #ser.write(lo(total_length), hi(total_length), 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01) 
   ser.write(inArg)
   addr = addr + length_this_tx
   
   while ddata != ack:
      ddata = ser.read(1)
   print "Acknowledgement received"
      
   id = 2
   range = 0;
   length_this_tx = 33
   ser.write(struct.pack('BBBB', 0x98, length_this_tx, lo(addr), hi(addr)))   
   inArg = [lo(id), hi(id), range, 0x15, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 
   0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33,
   0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44]   
   ser.write(inArg)
   addr = addr + length_this_tx   
   
   while ddata != ack:
      ddata = ser.read(1)
   print "Acknowledgement received"
   
   id = 30
   range = 0;
   length_this_tx = 33
   ser.write(struct.pack('BBBB', 0x98, length_this_tx, lo(addr), hi(addr)))   
   inArg = [lo(id), hi(id), range, 0x15, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 
   0x55, 0x55, 0x22, 0x22, 0x22, 0x22, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33,
   0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44]
   ser.write(inArg)
   addr = addr + length_this_tx
   
   while ddata != ack:
      ddata = ser.read(1)
   print "Acknowledgement received"
   
   id = 30
   range = 1;
   length_this_tx = 33
   ser.write(struct.pack('BBBB', 0x98, length_this_tx, lo(addr), hi(addr)))   
   inArg = [lo(id), hi(id), range, 0x15, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 
   0x66, 0x66, 0x22, 0x22, 0x22, 0x22, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33,
   0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44]
   ser.write(inArg)
   addr = addr + length_this_tx
   
   while ddata != ack:
      ddata = ser.read(1)
   print "Acknowledgement received"   
   
   id = 30
   range = 2;
   length_this_tx = 33
   ser.write(struct.pack('BBBB', 0x98, length_this_tx, lo(addr), hi(addr)))   
   inArg = [lo(id), hi(id), range, 0x15, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 
   0x77, 0x77, 0x22, 0x22, 0x22, 0x22, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33,
   0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44]
   ser.write(inArg)
   addr = addr + length_this_tx
   
   while ddata != ack:
      ddata = ser.read(1)
   print "Acknowledgement received"   
   
   id = 30
   range = 3;
   length_this_tx = 33
   ser.write(struct.pack('BBBB', 0x98, length_this_tx, lo(addr), hi(addr)))   
   inArg = [lo(id), hi(id), range, 0x15, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 
   0x88, 0x88, 0x22, 0x22, 0x22, 0x22, 0x33, 0x33, 0x33, 0x33, 0x33, 0x33,
   0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44]
   ser.write(inArg)
   addr = addr + length_this_tx
   
   while ddata != ack:
      ddata = ser.read(1)
   print "Acknowledgement received"   
   
#close serial port
   ser.close()
   print "All done"
