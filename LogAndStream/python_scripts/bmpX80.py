#!/usr/bin/python
#encoding=cp850
#the encoding must be set to match the encoding being used by the terminal this
#script is being run in (due to printing the ø symbol)
#this can be found by reading sys.stdout.encoding

import sys, struct, serial, binascii, time
def wait_for_ack():
   ddata = ""
   ack = struct.pack('B', 0xff)
   while ddata != ack:
      ddata = ser.read(1)
   return

def bmp280_calib(uTemp, uPress):
   #*******************************************************************************************************************************
   # Sampling trimming values from BMP280 datasheet - page 23
   '''
   T1 = 27504
   T2 = 26435
   T3 = -1000
   P1 = 36477
   P2 = -10685
   P3 = 3024
   P4 = 2855
   P5 = 140
   P6 = -7
   P7 = 15500
   P8 = -14600
   P9 = 6000

   OSS = 0

   UT = 27898
   UP = 23843
   '''
   #*******************************************************************************************************************************
   x1 = 0
   x2 = 0
   # /* calculate true temperature*/
   
   # /*calculate x1*/
   x1  = (((uTemp >> 3) - (T1 << 1)) * T2) >> 11

   # /*calculate x2*/
   x2 = (((((uTemp>>4) - (T1)) * ((uTemp>>4) - (T1))) >> 12) * (T3)) >> 14

   # /*calculate t_fine*/
   t_fine = x1 + x2

   # /*calculate temperature*/
   T = (t_fine * 5 + 128) >> 8

   # /* calculate true pressure*/
   var1 = 0
   var2 = 0
   P = 0

   var1 = t_fine - 128000
   var2 = var1 * var1 * P6
   var2 = var2 + ((var1 * P5) << 17)
   var2 = var2 + (P4 << 35)
   var1 = ((var1 * var1 * P3) >> 8) + ((var1 * P2) << 12)
   var1 = (((1 << 47) + var1)) * (P1) >> 33
   if (var1 != 0):
   
      # avoid exception caused by division by zero
   
      P = 1048576 - uPress
      P = (((P << 31) - var2) * 3125)/var1
      var1 = (P9 * (P >> 13) * (P >> 13)) >> 25
      var2 = (P8 * P) >> 19
      P = ((P + var1 + var2) >> 8) + (P7 << 4)

   return T/100.0, P/256.0

def bmp180_calib(uTemp, uPress):
   #*******************************************************************************************************************************
   # use the test values in the BMP180 datasheet (figure 4)
   '''
   AC1 = 408
   AC2 = -72
   AC3 = -14383
   AC4 = 32741
   AC5 = 32757
   AC6 = 23153
   B1 = 6190
   B2 = 4
   MB = -32767
   MC = -8711
   MD = 2868

   OSS = 0

   UT = 27898
   UP = 23843
   '''
   #*******************************************************************************************************************************

   X1 = (uTemp - AC6) * AC5 / 32768
   X2 = MC * 2048 / (X1 + MD)
   B5 = X1 + X2
   T = (B5 + 8) / 16

   B6 = B5 - 4000
   X1 = (B2 * (B6 * B6 / 4096)) / 2048
   X2 = AC2 * B6 / 2048
   X3 = X1 + X2
   B3 = (((AC1 * 4 + X3) << OSS) + 2) / 4
   X1 = AC3 * B6 / 8192
   X2 = (B1 * (B6 * B6 / 4096)) / 65536
   X3 = ((X1 + X2) + 2) / 4
   B4 = AC4 * (X3 + 32768) / 32768
   B7 = (uPress - B3) * (50000 >> OSS)
   if B7 < 0x80000000:
      P = (B7 * 2) / B4
   else:
      P = (B7 / B4) * 2
   X1 = (int((P / 256.0) * (P / 256.0)) * 3038) / 65536
   X2 = (-7357 * P) / 65536
   P += (X1 + X2 + 3791) / 16

   return T/10.0, P

def compareVersion (thisMajor, thisMinor, thisInternal, compMajor, compMinor, compInternal):
	
	if ((thisMajor==compMajor and thisMinor>compMinor)or
		(thisMajor==compMajor and thisMinor==compMinor and thisInternal>=compInternal)):
		return True
	else:
		return False

def checkFor2ndGenIMU (expIdMajor, expIdMinor, expIdInternal):
	
	if (compareVersion(expIdMajor, expIdMinor, expIdInternal, 47, 3,0) or 
	   compareVersion(expIdMajor, expIdMinor, expIdInternal, 48, 3,0) or
	   compareVersion(expIdMajor, expIdMinor, expIdInternal, 49, 2,0) or
	   compareVersion(expIdMajor, expIdMinor, expIdInternal, 31, 6,0) or
	   compareVersion(expIdMajor, expIdMinor, expIdInternal, 38, 3,0) or
	   compareVersion(expIdMajor, expIdMinor, expIdInternal, 36, 3,0) or
	   expIdInternal == 171):
	   return True
	else:
		return False

if len(sys.argv) < 2:
   print "no device specified"
   print "You need to specify the serial port of the device you wish to connect to"
   print "example:"
   print "   bmpX80.py Com12"
   print "or"
   print "   bmpX80.py /dev/rfcomm0"
else:
   ser = serial.Serial(sys.argv[1], 115200)
   ser.flushInput()

#get the daughter card ID byte (SR number)
   ddaughterCardId = ""
   memLength = 5  # Byte Format: daughter CardID|length|major|minor|Internal
   print("Requesting Daughter Card ID...")
   ser.write(struct.pack('BBB', 0x66, 0x03, 0x00))
   wait_for_ack()


   ddaughterCardId += ser.read(memLength)
   daughterCardId = ddaughterCardId[2:memLength]
   (srMajor, srMinor, srInternal) = struct.unpack('BBB', daughterCardId[0:3])
   print("srMajor=%d,srMinor=%d,srInternal=%d" % (srMajor, srMinor, srInternal))
   bmp280 = checkFor2ndGenIMU(srMajor, srMinor, srInternal)
   print "Using BMP%d80" % (2 if bmp280 else 1)
   framesize = (24 if bmp280 else 22)

# read the calibration coefficients
   ser.write(struct.pack('B', (0xA0 if bmp280 else 0x59)))
   wait_for_ack()

   ddata = ""
   calibcoeffsresponse = struct.pack('B', (0x9F if bmp280 else 0x58))
   while ddata != calibcoeffsresponse:
      ddata = ser.read(1)

   ddata = ""
   numbytes = 0
   while numbytes < framesize:
      ddata += ser.read(framesize)
      numbytes = len(ddata)
   data = ddata[0:framesize]

   if(bmp280):
      # h is for signed
      # H is for unsigned
      # < is for little endian (format is LSB, MSB, LSB, MSB etc...)
      # > is for big endian (format is MSB, LSB, MSB, LSB etc...)
      (T1, T2, T3, P1, P2, P3, P4, P5, P6, P7, P8, P9) = struct.unpack('<HhhHhhhhhhhh', data)
      print "Calibration coefficients:\n\tT1: %d\n\tT2: %d\n\tT3: %d\n\tP1: " \
            "%d\n\tP2: %d\n\tP3: %d\n\tP4: %d\n\tP5: %d\n\tP6: %d\n\tP7: %d\n\t" \
            "P8: %d\n\tP9: %d\n" % (T1, T2, T3, P1, P2, P3, P4, P5, P6, P7, P8, P9)
   else:
      (AC1, AC2, AC3, AC4, AC5, AC6, B1, B2, MB, MC, MD) = struct.unpack('>hhhHHHhhhhh', data)
      print "Calibration coefficients:\n\tAC1: %d\n\tAC2: %d\n\tAC3: %d\n\tAC4: %d\n\tAC5: %d\n\tAC6: " \
             "%d\n\t B1: %d\n\t B2: %d\n\t MB: %d\n\t MC: %d\n\t MD: %d\n" \
             % (AC1, AC2, AC3, AC4, AC5, AC6, B1, B2, MB, MC, MD)


# send the set sensors command
   ser.write(struct.pack('BBBB', 0x08, 0x00, 0x00, 0x04))  # bmpX80 temp & press
   wait_for_ack()
# set OSS value (0-3)
   OSS = 3
   ser.write(struct.pack('BB', 0x52, OSS))
   wait_for_ack()

# send the set sampling rate command

   '''
    sampling_freq = 32768 / clock_wait = X Hz
   '''
   sampling_freq = 2 # desired sampling freq. in Hz
   clock_wait = (2 << 14) / sampling_freq

   ser.write(struct.pack('<BH', 0x05, clock_wait))

   wait_for_ack()

# send start streaming command
   ser.write(struct.pack('B', 0x07))
   wait_for_ack()

# read incoming data
   ddata = ""
   numbytes = 0
   prev_UT = 0
   prev_UP = 0
   prev_TS = 0
   framesize = 9 # 1byte packet type + 3byte timestamp + 2byte BMPX80 Temperature + 3byte BMPX80 Pressure
   

   print "Packet Type \tTimestamp diff \tUncompensated Temperature \tTemperature (øC) \tUncompensated Pressure \tPressure (Pa)"
   try:
      while True:
         while numbytes < framesize:
            ddata += ser.read(framesize)
            numbytes = len(ddata)
         
         data = ddata[0:framesize]
         ddata = ddata[framesize:]
         numbytes = len(ddata)

         (packettype,) = struct.unpack('B', data[0:1])
         (timestamp0, timestamp1, timestamp2) = struct.unpack('BBB', data[1:4])
         timestamp = timestamp0 + (timestamp1 << 8) + (timestamp2 << 16)

         (UT_msb, UT_lsb, \
         UP_msb, UP_lsb, UP_xlsb) = struct.unpack('BBBBB', data[4:framesize])

         UT = ((UT_msb<<8) + (UT_lsb))<<4*(1 if bmp280 else 0)
         UP = ((UP_msb<<16) + (UP_lsb<<8) + UP_xlsb)>>(4 if bmp280 else (8 - OSS))           
   

         if UT != prev_UT or UP != prev_UP:

            if(bmp280):
               [T, P] = bmp280_calib(UT, UP)
               
            else:
               [T, P] = bmp180_calib(UT, UP)
               
            # print "UT_msb: 0x%02x\tUT_lsb: 0x%02x\t" % (UT_msb, UT_lsb)
            # print "UP_msb: 0x%02x\tUP_lsb: 0x%02x\tUP_xlsb: 0x%02x" % (UP_msb, UP_lsb, UP_xlsb)
            print "0x%02x\t\t %5d\t\t\t %4d\t\t\t %0.2f\t\t\t %4d\t\t\t\t %d" % \
                     (packettype, (timestamp-prev_TS), UT, T, UP, P)

         prev_UP = UP
         prev_UT = UT
         prev_TS = timestamp

   except KeyboardInterrupt:
#send stop streaming command
      ser.write(struct.pack('B', 0x20))
      wait_for_ack()
#close serial port
      ser.close()
      print
      print "All done"
