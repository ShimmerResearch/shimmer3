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
   print "A second argument can be specified to change sampling rate to new value"
   print "If no second argument the current sampling rate is read and displayed"
   print "example (setting sampling rate to 51.2Hz):"
   print "   samplingRate.py Com12"
   print "or"
   print "   samplingRate.py Com12 0x0280"
   print "or"
   print "   samplingRate.py /dev/rfcomm0"
   print "or"
   print "   samplingRate.py /dev/rfcomm0 640"
else:
   if len(sys.argv) == 2:
      ser = serial.Serial(sys.argv[1], 115200)
      ser.flushInput()
# send get sampling rate command
      ser.write(struct.pack('B', 0x03))
      wait_for_ack()

# read incoming data
      ddata = ""
      numbytes = 0
      framesize = 3

      while numbytes < framesize:
         ddata += ser.read(framesize)
         numbytes = len(ddata)
         
      data = ddata[0:framesize]
      ddata = ddata[framesize:]
      numbytes = len(ddata)

      (samplingrate) = struct.unpack('H', data[1:3])
      print "Current sampling rate: %d (%.2fHz)" % (samplingrate[0], (32768.0/samplingrate[0]))
   elif len(sys.argv) == 3:
      newrate = int(sys.argv[2])
      if 1 <= newrate <= 65535:
         ser = serial.Serial(sys.argv[1], 115200)
         ser.flushInput()
# send get sampling rate command
         ser.write(struct.pack('BBB', 0x05, (newrate&0xFF), ((newrate&0xFF00)>>8))) #need to be done this way due to alignment issues
         wait_for_ack()
         print "Sampling rate set to: %d (%.2fHz)" % (newrate, (32768.0/newrate))
      else:
         print "%d is not a valid sampling rate\nMust be between 1 and 65535" % newrate
#close serial port
   ser.close()
   print
   print "All done"
