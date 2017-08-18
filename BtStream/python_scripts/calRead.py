#!/usr/bin/python
import sys, struct, serial

def wait_for_ack():
   ddata = ""
   ack = struct.pack('B', 0xff)
   while ddata != ack:
      ddata = ser.read(1)
   return

if len(sys.argv) < 3:
   print "no device specified"
   print "You need to specify the serial port of the device you wish to connect to"
   print "followed by which sensor's calibration values you wish to read, as"
   print "specified from following list:"
   print "   1: Analog Accel"
   print "   2: Gyro"
   print "   3: Magnetometer"
   print "   4: Digital Accel (LSM303DLHC)"
   print "   5: All calibration values"
   print "example:"
   print "   calWrite.py Com12 3"
   print "or"
   print "   calWrite.py /dev/rfcomm0 5"
else:
   selection = int(sys.argv[2])
   if (selection<1) or (selection>5):
      print "Invalid selection\nExiting..."
      sys.exit()

   ser = serial.Serial(sys.argv[1], 115200)
   ser.flushInput()
   if sys.argv[2] == '1':
# send the get aAccel calibration command 
      ser.write(struct.pack('B', 0x13))
   elif sys.argv[2] == '2':
# send the get gyro calibration command 
      ser.write(struct.pack('B', 0x16))
   elif sys.argv[2] == '3':
# send the get mag calibration command 
      ser.write(struct.pack('B', 0x19))
   elif sys.argv[2] == '4':
# send the get lsm303dlhcAccel calibration command 
      ser.write(struct.pack('B', 0x1C))
   elif sys.argv[2] == '5':
# send the get all calibration command 
      ser.write(struct.pack('B', 0x2C))

# read the acknowledgement
   ddata = ""
   ack = struct.pack('B', 0xff)
   while ddata != ack:
      ddata = ser.read(1)
   print "Acknowledgement received"

# wait for the calibration response
   ddata = ""
   if sys.argv[2] == '1':
      ack = struct.pack('B', 0x12)  #A_ACCEL_CALIBRATION_RESPONSE
      print "Analog accel calibration response:",
   elif sys.argv[2] == '2':
      ack = struct.pack('B', 0x15)  #GYRO_CALIBRATION_RESPONSE
      print "Gyro calibration response:",
   elif sys.argv[2] == '3':
      ack = struct.pack('B', 0x18)  #MAG_CALIBRATION_RESPONSE
      print "Mag calibration response:",
   elif sys.argv[2] == '4':
      ack = struct.pack('B', 0x1B)  #LSM303DLHC_ACCEL_CALIBRATION_RESPONSE
      print "LSM303DLHC accel calibration response:",
   elif sys.argv[2] == '5':
      ack = struct.pack('B', 0x2D)  #ALL_CALIBRATION_RESPONSE
      print "All sensors calibration response:",

   while ddata != ack:
      ddata = ser.read(1)

   ddata = ""
   numbytes = 0
   if sys.argv[2] == '5':
      framesize = 84
   else:
      framesize = 21
   while numbytes < framesize:
      ddata = ser.read(framesize)
      numbytes = len(ddata)
   data = ddata[0:framesize]
   ddata = ddata[framesize:]
   numbytes = len(ddata)

   for i in range(framesize):
      print "0x%02x" % struct.unpack('B', data[i]),
   print


#close serial port
   ser.close()
   print "All done"
