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
   print "   exgSquareWave512Hz.py Com12"
   print "or"
   print "   exgSquareWave512Hz.py /dev/rfcomm0"
else:
   ser = serial.Serial(sys.argv[1], 115200)
   ser.flushInput()
# send the set sensors command
   ser.write(struct.pack('BBBB', 0x08, 0x10, 0x00, 0x00))  #exg1
#   ser.write(struct.pack('BBBB', 0x08, 0x08, 0x00, 0x00))  #exg1
   wait_for_ack()
# send the set sampling rate command
#   ser.write(struct.pack('BBB', 0x05, 0x80, 0x02)) #51.2Hz (640 (0x280)). Has to be done like this for alignment reasons
   ser.write(struct.pack('BBB', 0x05, 0x40, 0x00)) #512Hz (64 (0x0040)). Has to be done like this for alignment reasons
   wait_for_ack()
# configure both ADS1292R chips: enable internal reference, 500 samples per second, square-wave test signal
   ser.write(struct.pack('BBBBBBBBB', 0x61, 0x00, 0x00, 0x05, 0x02, 0xA3, 0x10, 0x05, 0x05))
   wait_for_ack()
   ser.write(struct.pack('BBBBBBBBB', 0x61, 0x01, 0x00, 0x05, 0x02, 0xA3, 0x10, 0x05, 0x05))
   wait_for_ack()

# send start streaming command
   ser.write(struct.pack('B', 0x07))
   wait_for_ack()

# read incoming data
   ddata = ""
   numbytes = 0
   framesize = 10 # 1byte packet type + 2byte timestamp + 7byte ExG data

   print "Packet Type,Timestamp,ChipX Status,ChipX Channel1,ChipX Channel2"
   try:
      while True:
         while numbytes < framesize:
            ddata += ser.read(framesize)
            numbytes = len(ddata)
         
         data = ddata[0:framesize]
         ddata = ddata[framesize:]
         numbytes = len(ddata)

         (packettype,) = struct.unpack('B', data[0:1])

         (timestamp, cXstatus) = struct.unpack('HB', data[1:4])
# 24-bit signed values MSB values are tricky, as struct only supports 16-bit or 32-bit
# pad with zeroes at LSB end and then shift the result
         cXch1 = struct.unpack('>i', (data[4:7] + '\0'))[0] >> 8
         cXch2 = struct.unpack('>i', (data[7:10] + '\0'))[0] >> 8
         print "0x%02x,%5d,\t0x%02x,%8d,%8d" % (packettype, timestamp, cXstatus, cXch1, cXch2)

   except KeyboardInterrupt:
#send stop streaming command
      ser.write(struct.pack('B', 0x20))
      wait_for_ack()
#close serial port
      ser.close()
      print
      print "All done"
