#!/usr/bin/python
import sys, struct, serial

def wait_for_ack():
   ddata = ""
   ack = struct.pack('B', 0xff)
   while ddata != ack:
      ddata = ser.read(1)
   return

if len(sys.argv) < 3:
   print "No device specified and/or not enough arguments specified."  
   print "Specify the serial port of the device you wish to connect to,"
   print "followed by which sensor's calibration values you wish to read, as"
   print "specified from following list:"
   print "   1: Analog Accel (Low Noise Accel)"
   print "   2: Gyro (MPU9150)"
   print "   3: Magnetometer (LSM303DLHC)"
   print "   4: Digital Accel (Wide Range Accel - LSM303DLHC)"
   print "   5: All calibration values"
   print
   print "To write calibration values for a particular sensor, choose from the"
   print "list below:"
   print "   6: Analog Accel (Low Noise Accel)"
   print "   7: Gyro (MPU9150)"
   print "   8: Magnetometer (LSM303DLHC)"
   print "   9: Digital Accel (Wide Range Accel - LSM303DLHC)"
   print "   0: Reset all calibration values" 
else:
   selection = int(sys.argv[2])
   if (selection<0) or (selection>9):
      print "Invalid selection\nExiting..."
      sys.exit()

   ser = serial.Serial(sys.argv[1], 115200)
   ser.flushInput()
   if sys.argv[2] == '0':
      print "Resetting all calibration values"
# send the reset calibration value command
      ser.write(struct.pack('B', 0x5B))
   elif sys.argv[2] == '1':
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
   elif sys.argv[2] == '6':
      print "Setting analog accel calibration values to: 0x11 0x22 0x22 0x22 0x22 0x22 0x22 0x22 0x22 0x22 0x22 0x22 0x22 0x22 0x22 0x22 0x22 0x22 0x22 0x22 0x33"
# send the set aAccel calibration command 
      ser.write(struct.pack('BBBBBBBBBBBBBBBBBBBBBB', 0x11, 0x11,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x33))
   elif sys.argv[2] == '7':
      print "Setting gyro calibration values to: 0x44 0x55 0x55 0x55 0x55 0x55 0x55 0x55 0x55 0x55 0x55 0x55 0x55 0x55 0x55 0x55 0x55 0x55 0x55 0x55 0x66"
# send the set gyro calibration command 
      ser.write(struct.pack('BBBBBBBBBBBBBBBBBBBBBB', 0x14, 0x44,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x66))
   elif sys.argv[2] == '8':
      print "Setting mag calibration values to: 0x77 0x88 0x88 0x88 0x88 0x88 0x88 0x88 0x88 0x88 0x88 0x88 0x88 0x88 0x88 0x88 0x88 0x88 0x88 0x88 0x99"
# send the set mag calibration command 
      ser.write(struct.pack('BBBBBBBBBBBBBBBBBBBBBB', 0x17, 0x77,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x99))
   elif sys.argv[2] == '9':
      print "Setting digital accel calibration values to: 0xAA 0xBB 0xBB 0xBB 0xBB 0xBB 0xBB 0xBB 0xBB 0xBB 0xBB 0xBB 0xBB 0xBB 0xBB 0xBB 0xBB 0xBB 0xBB 0xBB 0xCC"
# send the set dAccel calibration command 
      ser.write(struct.pack('BBBBBBBBBBBBBBBBBBBBBB', 0x1A, 0xAA,0xBB,0xBB,0xBB,0xBB,0xBB,0xBB,0xBB,0xBB,0xBB,0xBB,0xBB,0xBB,0xBB,0xBB,0xBB,0xBB,0xBB,0xBB,0xBB,0xCC))
  

# read the acknowledgement
   wait_for_ack()
   print "Acknowledgement received."

# wait for the calibration response
   if (sys.argv[2] >= '1' and sys.argv[2] <= '5' ): 
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
   print "All done!"
