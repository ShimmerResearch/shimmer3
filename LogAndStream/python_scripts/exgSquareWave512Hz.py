#!/usr/bin/python
import sys, struct, serial

def wait_for_ack():
   ddata = ""
   ack = struct.pack('B', 0xff)
   while ddata != ack:
      ddata = ser.read(1)
   return

if len(sys.argv) < 2:
   print "No device specified."
   print "Specify the serial port of the device you wish to connect to."
   print "Example:"
   print "   exgSquareWave512Hz.py Com12"
   print "or"
   print "   exgSquareWave512Hz.py /dev/rfcomm0"
else:
   ser = serial.Serial(sys.argv[1], 115200)
   ser.flushInput()
   print "Port open..."
   
   #get the daughter card ID byte (SR number)
   print("Requesting Daughter Card ID and Revision number...")
   ser.write(struct.pack('BBB', 0x66, 0x02,0x00))
   wait_for_ack()
   srNumber = struct.unpack('BBB', ser.read(3))[2] # get third byte (0,1,2..)
   srRev = struct.unpack('B', ser.read(1))[0] # get the next byte 
   print "Daughter card ID: 0x%02x -> %02d" % (srNumber, srNumber)
   print "Revision Number:  0x%02x -> %02d" % (srRev, srRev)


   #send the set sensors command
   ser.write(struct.pack('BBBB', 0x08, 0x18, 0x00, 0x00))  #exg1 and exg2
   wait_for_ack()
   print "Sensor Enabling done..."
   # send the set sampling rate command
   ser.write(struct.pack('BBB', 0x05, 0x20, 0x00)) #512Hz (32768/64=512Hz: 64 -> 0x0040; has to be done like this for alignment reasons.)
   wait_for_ack()
   print "Freq sent..."
   # configure both ADS1292R chips: enable internal reference, 500 samples per second, square-wave test signal
   # exgtestsignalconfiguration = ["CONFIG1" = 2, "CONFIG2" = 163, "LOFF" = 16, "CH1SET" = 5, "CH2SET" = 5, "RLD_SENS" = 0, "LOFF_SENS" = 0, "LOFF_STAT" = 0, "RESP1" = 2, "RESP2" = 1]
   # write SET_EXG_REGS_COMMAND, chip identifier ('0' for chip1, '1' for chip2), starting byte, number of bytes to write, followed by the exgtestsignalconfiguration bytes

   # configure chip 1
   if(srNumber == 47 and srRev >= 4):
         ser.write(struct.pack('BBBBBBBBBBBBBB', 0x61, 0x00, 0x00, 0x0A, 0x03, 0xAB, 0x10, 0x15, 0x15, 0x00, 0x00, 0x00, 0x02, 0x01)) # Config byte for CHIP1 in SR47-4
         
   else:
         ser.write(struct.pack('BBBBBBBBBBBBBB', 0x61, 0x00, 0x00, 0x0A, 0x03, 0xA3, 0x10, 0x15, 0x15, 0x00, 0x00, 0x00, 0x02, 0x01)) # Config byte for CHIP1 in SR47-1 and SR37-3 
        
   wait_for_ack()

   # configure chip 2  
   ser.write(struct.pack('BBBBBBBBBBBBBB', 0x61, 0x01, 0x00, 0x0A, 0x03, 0xA3, 0x10, 0x15, 0x15, 0x00, 0x00, 0x00, 0x02, 0x01)) # Config byte for CHIP2 in all ExG units
   wait_for_ack()
   print "Configuration sent..."

   # send start streaming command
   ser.write(struct.pack('B', 0x07))
   wait_for_ack()
   print "Start sent..."

   # read incoming data
   ddata = ""
   numbytes = 0
   framesize = 18 # 1byte packet type + 2byte timestamp + 14byte ExG data

   print "Packet Type,Timestamp,Chip1 Status,Chip1 Channel1,Chip1 Channel2,Chip2 Status,Chip2 Channel1,Chip2 Channel2"
   try:
      while True:
         while numbytes < framesize:
            ddata += ser.read(framesize)
            numbytes = len(ddata)
         
         data = ddata[0:framesize]
         ddata = ddata[framesize:]
         numbytes = len(ddata)

         (packettype,) = struct.unpack('B', data[0:1])

         # (timestamp, c1status) = struct.unpack('HB', data[1:4])
         (ts0, ts1, ts2, c1status) = struct.unpack('BBBB', data[1:5])
         timestamp = ts0 + ts1*256 + ts2*65536
         # 24-bit signed values MSB values are tricky, as struct only supports 16-bit or 32-bit
         # pad with zeroes at LSB end and then shift the result
         c1ch1 = struct.unpack('>i', (data[5:8] + '\0'))[0] >> 8
         c1ch2 = struct.unpack('>i', (data[8:11] + '\0'))[0] >> 8
         (c2status,) = struct.unpack('B', data[11])
         c2ch1 = struct.unpack('>i', (data[12:15] + '\0'))[0] >> 8
         c2ch2 = struct.unpack('>i', (data[15:18] + '\0'))[0] >> 8
         print "0x%02x,%06x,\t0x%02x,%8d,%8d,\t0x%02x,%8d,%8d" % (packettype, timestamp, c1status, c1ch1, c1ch2, c2status, c2ch1, c2ch2)

   except KeyboardInterrupt:
      #send stop streaming command
      ser.write(struct.pack('B', 0x20))
      wait_for_ack()
      #close serial port
      ser.close()
      print
      print "All done!"
