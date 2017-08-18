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
   print "   inquiryCommand.py Com12"
   print "or"
   print "   inquiryCommand.py /dev/rfcomm0"
else:
   ser = serial.Serial(sys.argv[1], 115200)
   ser.flushInput()
# send inquiry command
   ser.write(struct.pack('B', 0x01))
   wait_for_ack()

# read incoming data
   ddata = ""
   numbytes = 0
   framesize = 9

   print "Inquiry response:"
   while numbytes < framesize:
      ddata += ser.read(framesize)
      numbytes = len(ddata)
         
   data = ddata[0:framesize]
   ddata = ddata[framesize:]
   numbytes = len(ddata)

   (packettype) = struct.unpack('B', data[0])
   (samplingrate, configByte0, configByte1, configByte2, configByte3, numchans, bufsize) = struct.unpack('HBBBBBB', data[1:9])
   print "          Packet type: 0x%02x" % packettype
   print "        Sampling rate: 0x%04x" % samplingrate
   print "  Config Setup Byte 0: 0x%02x" % configByte0
   print "  Config Setup Byte 1: 0x%02x" % configByte1
   print "  Config Setup Byte 2: 0x%02x" % configByte2
   print "  Config Setup Byte 3: 0x%02x" % configByte3
   print "   Number of channels: 0x%02x" % numchans
   print "          Buffer size: 0x%02x" % bufsize

   for i in range(numchans):
      data = ser.read(1)
      print "           Channel %2d:" % i,
      print "0x%02x" % (struct.unpack('B', data[0]))

#close serial port
   ser.close()
   print
   print "All done"
