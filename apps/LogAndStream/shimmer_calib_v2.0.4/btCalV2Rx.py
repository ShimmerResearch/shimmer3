#!/usr/bin/python
import sys, struct, serial, msvcrt, math, random, binascii

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
   
   # send the get calibration dump command 
   addr = 0
   length = 128
   length_done = 0
   total_length = 0
   
   while length > 0:
      addr_l = addr&0xff
      addr_h = (addr>>8)&0xff
      print "sending the GET_CALIB_DUMP_COMMAND, length:%3d, addr:%d"%( length, addr)
      ser.write(struct.pack('BBBB', 0x9A, length, addr_l, addr_h))
      # read the acknowledgement
      ddata = ""
      ack = struct.pack('B', 0xff)
      while ddata != ack:
         ddata = ser.read(1)
      print "Acknowledgement received"

      # wait for the calibration response
      ddata = ""
      ack = struct.pack('B', 0x99)
      while ddata != ack:
         ddata = ser.read(1)
      ser.read(3)

      framesize = length
      ddata = ""   
      numbytes = 0
      while numbytes < framesize:
         ddata = ser.read(framesize)
         numbytes = len(ddata)
      data = ddata[0:framesize]
      ddata = ddata[framesize:]
      numbytes = len(ddata)

      for i in range(framesize):
         print "0x%02x" % struct.unpack('B', data[i]),
      print
      
      if addr==0 :
         [total_length] = struct.unpack('h', data[0:2])
         total_length = total_length+2
         print "total length: %d" % (total_length)
         # total_length = total_length[0]
      
      length_done = length_done+length
      addr = addr+length
      if(total_length <= length_done):
         length = 0
      elif (total_length-length_done > 128):
         length = 128
      else:
         length = total_length-length_done
   
#close serial port
   ser.close()
   print "All done"
