#!/usr/bin/python
import sys, struct, serial

def wait_for_ack():
   ddata = ""
   ack = struct.pack('B', 0xff)
   while ddata != ack:
      ddata = ser.read(1)
   return

def bmp180_calc_compensated_vals(UT, UP):
   X1 = (UT - AC6) * AC5 / 32768
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
   B7 = (UP - B3) * (50000 >> OSS)
   if B7 < 0x80000000: 
      p = (B7 * 2) / B4
   else:
      p = (B7 / B4) * 2
   X1 = (int((p / 256.0) * (p / 256.0)) * 3038) / 65536
   X2 = (-7357 * p) / 65536
   p += (X1 + X2 + 3791) / 16
   return (T, p)


if len(sys.argv) < 2:
   print "no device specified"
   print "You need to specify the serial port of the device you wish to connect to"
   print "example:"
   print "   allSensors50Hz.py Com12"
   print "or"
   print "   allSensors50Hz.py /dev/rfcomm0"
else:
   ser = serial.Serial(sys.argv[1], 115200)
   ser.flushInput()
# send the set sensors command
   ser.write(struct.pack('BBBB', 0x08, 0xE3, 0x3F, 0xE4))  #analog accel, Vbatt, gyro, lsm accel, mpu accel, mag, mpu mag, external ADC expansion, internal ADC expansion, bmp temp+press
   wait_for_ack()
# send the set sampling rate command
   ser.write(struct.pack('BBB', 0x05, 0x80, 0x02)) #50Hz (640 (0x280)). Has to be done like this for alignment reasons
   wait_for_ack()
# read the BMP180 calibration coefficients
   ser.write(struct.pack('B', 0x59))
   wait_for_ack()

   ddata = ""
   calibcoeffsresponse = struct.pack('B', 0x58)
   while ddata != calibcoeffsresponse:
      ddata = ser.read(1)

   ddata = ""
   numbytes = 0
   framesize = 22
   while numbytes < framesize:
      ddata += ser.read(framesize)
      numbytes = len(ddata)
   data = ddata[0:framesize]

   (AC1, AC2, AC3, AC4, AC5, AC6, B1, B2, MB, MC, MD) = struct.unpack('>hhhHHHhhhhh', data);
# set the BMP180 OSS (over samplign setting) value (0-3)
   OSS = 0
   ser.write(struct.pack('BB', 0x52, OSS))
   wait_for_ack()
# send start streaming command
   ser.write(struct.pack('B', 0x07))
   wait_for_ack()

# read incoming data
   ddata = ""
   numbytes = 0
   framesize = 60 # 1byte packet type + 2byte timestamp + 3x2byte Analog Accel + 2byte batt voltage + 3x2byte External ADC + 4x2byte Internal ADC + 3x2byte Gyro + 3x2byte LSM Accel + 3x2byte Mag + 3x2byte MPU Accel + 3x2byte MPU mag + 2 byte BMP temp + 3 byte BMP pressure

   print "Packet Type,Timestamp,Analog AccelX, Analog AccelY,Analog Accey Y,Gyro X,Gyro Y,Gyro Z,Battery Voltage,LSM Accel X,LSM Accel Y,LSM Accel Z,MPU Accel X,MPU Accel Y,MPU Accel Z,Mag X,Mag Y,Mag Z,MPU Mag X, MPU Mag Y, MPU Mag Z,ADC 7,ADC 6,ADC 15,ADC 1,ADC 12,ADC 13,ADC14,BMP Temp,BMP Pressure"
   try:
      while True:
         while numbytes < framesize:
            ddata += ser.read(framesize)
            numbytes = len(ddata)
         
         data = ddata[0:framesize]
         ddata = ddata[framesize:]
         numbytes = len(ddata)

         (packettype) = struct.unpack('B', data[0:1])
         (timestamp, analogaccelx, analogaccely, analogaccelz, vbatt, adc7, adc6, adc15, adc1, adc12, adc13, adc14) = struct.unpack('HHHHHHHHHHHH', data[1:25])
         (gyrox, gyroy, gyroz) = struct.unpack('>hhh', data[25:31])
         (lsmaccelx, lsmaccely, lsmaccelz) = struct.unpack('hhh', data[31:37])
         (magx, magz, magy, mpuaccelx, mpuaccely, mpuaccelz) = struct.unpack('>hhhhhh', data[37:49])
         (mpumagx, mpumagy, mpumagz) = struct.unpack('hhh', data[49:55])
         (UT,) = struct.unpack('>H', data[55:57])
         (msb, lsb, xlsb) =struct.unpack('BBB', data[57:framesize])
         UP = ((msb<<16) + (lsb<<8) + xlsb)>>(8-OSS)
         (T, p) = bmp180_calc_compensated_vals(UT, UP)
         print "0x%02x,%5d\t%4d,%4d,%4d\t%4d,%4d,%4d\t\t%4d,%4d,%4d\t%4d,%4d,%4d\t\t%4d,%4d,%4d\t%4d,%4d,%4d\t\t%4d\t%4d,%4d,%4d\t%4d,%4d,%4d,%4d\t%0.1f,%d" % (packettype[0], timestamp, analogaccelx, analogaccely, analogaccelz, gyrox, gyroy, gyroz, lsmaccelx, lsmaccely, lsmaccelz, mpuaccelx, mpuaccely, mpuaccelz, magx, magy, magz, mpumagx, mpumagy, mpumagz, vbatt, adc7, adc6, adc15, adc1, adc12, adc13, adc14, T/10.0, p)
#         print "0x%02x,%5d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%.1f,%d" % (packettype[0], timestamp, analogaccelx, analogaccely, analogaccelz, gyrox, gyroy, gyroz, lsmaccelx, lsmaccely, lsmaccelz, mpuaccelx, mpuaccely, mpuaccelz, magx, magy, magz, mpumagx, mpumagy, mpumagz, vbatt, adc7, adc6, adc15, adc1, adc12, adc13, adc14, T/10.0, p)

   except KeyboardInterrupt:
#send stop streaming command
      ser.write(struct.pack('B', 0x20))
      wait_for_ack()
#close serial port
      ser.close()
      print
      print "All done"
