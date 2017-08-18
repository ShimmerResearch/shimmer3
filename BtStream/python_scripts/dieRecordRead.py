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
   print "example:"
   print "   dieRecordRead.py Com12"
   print "or"
   print "   dieRecordRead.py /dev/rfcomm0"
else:
   ser = serial.Serial(sys.argv[1], 115200)
   ser.flushInput()
# send the get unique serial command 
   ser.write(struct.pack('B', 0x3E))

# read the acknowledgement
   ddata = ""
   ack = struct.pack('B', 0xff)
   while ddata != ack:
      ddata = ser.read(1)
   print "Acknowledgement received"

# wait for the calibration response
   ddata = ""
   res = struct.pack('B', 0x3D)  #UNIQUE_SERIAL_RESPONSE
   while ddata != res:
      ddata = ser.read(1)

   ddata = ""
   numbytes = 0
   framesize = 8 
   while numbytes < framesize:
      ddata = ser.read(framesize)
      numbytes = len(ddata)
   data = ddata[0:framesize]
   ddata = ddata[framesize:]
   numbytes = len(ddata)

   (lot, xpos, ypos) = struct.unpack('<IHH', data);
   print "Die record:\n\t  Lot/Wafer ID: 0x%08x\n\tDie X position: 0x%04x\n\tDie Y position: 0x%04x" % (lot, xpos, ypos)
   print


#close serial port
   ser.close()
   print "All done"
