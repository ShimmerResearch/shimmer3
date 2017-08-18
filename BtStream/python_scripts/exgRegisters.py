#!/usr/bin/python
import sys, struct, serial

def wait_for_ack():
   ddata = ""
   ack = struct.pack('B', 0xff)
   while ddata != ack:
      ddata = ser.read(1)
   return


if len(sys.argv) != 2 and len(sys.argv) < 5:
   print "no device and/or incorrect number of arguments specified"
   print "You need to specify the serial port of the device you wish to connect to"
   print "A second, third and fourth argument must be specified to indicate which of the two"
   print "ADS1292R chips to read/write, the first register to read/write and the number of"
   print "registers to read/write, respectively"
   print "If 4 or more arguments are present the registers settingss will be changed"
   print "If none of these arguments are present the configured values for both ADS1292R chips"
   print "are read and displayed"
   print
   print "There are 10 registers available as shown in the following list, along with the" 
   print "values required for argument three"
   print "   CONFIG1     0"
   print "   CONFIG2     1"
   print "   LOFF        2"
   print "   CH1SET      3"
   print "   CH2SET      4"
   print "   RLD_SENS    5"
   print "   LOFF_SENS   6"
   print "   LOFF_STAT   7"
   print "   RESP1       8"
   print "   RESP2       9"
   print
   print "example (to read all values for both ExG registers):"
   print "   exgRegisters.py Com12"
   print "or (to read 4 registers for chip 0, starting at LOFF)"
   print "   exgRegisters.py Com12 0 2 4"
   print "or (to write 10 bytes of data to chip 1, starting at CONFIG1)"
   print "   exgRegisters.py /dev/rfcomm0 1 0 10 0x02 0xE0 0xF0 0x00 0x00 0x2C 0x0F 0x00 0xEA 0x03"
else:
   regList = ["CONFIG1",
              "CONFIG2",
              "LOFF",
              "CH1SET",
              "CH2SET",
              "RLD_SENS",
              "LOFF_SENS",
              "LOFF_STAT",
              "RESP1",
              "RESP2"]

   if len(sys.argv) == 2:
      ser = serial.Serial(sys.argv[1], 115200)
      ser.flushInput()
# send the get ExG regs command (all registers for chip 0)
      ser.write(struct.pack('BBBB', 0x63, 0, 0, 10))
      wait_for_ack()

      ddata = ""
      exgregsresponse = struct.pack('B', 0x62) #EXG_REGS_RESPONSE
      while ddata != exgregsresponse:
         ddata = ser.read(1)

      ddata = ""
      numbytes = 0
      framesize = 11 
      while numbytes < framesize:
         ddata += ser.read(framesize)
         numbytes = len(ddata)
      data = ddata[0:framesize]

      (receivedLength, reg0, reg1, reg2, reg3, reg4, reg5, reg6, reg7, reg8, reg9) = struct.unpack('BBBBBBBBBBB', data)
      print "ExG register settings for chip 0, starting from register CONFIG1: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x" % (reg0, reg1, reg2, reg3, reg4, reg5, reg6, reg7, reg8, reg9)
# send the get ExG regs command (all registers for chip 1)
      ser.write(struct.pack('BBBB', 0x63, 1, 0, 10))
      wait_for_ack()

      ddata = ""
      while ddata != exgregsresponse:
         ddata = ser.read(1)

      ddata = ""
      numbytes = 0
      framesize = 11 
      while numbytes < framesize:
         ddata += ser.read(framesize)
         numbytes = len(ddata)
      data = ddata[0:framesize]

      (receivedLength, reg0, reg1, reg2, reg3, reg4, reg5, reg6, reg7, reg8, reg9) = struct.unpack('BBBBBBBBBBB', data)
      print "ExG register settings for chip 1, starting from register CONFIG1: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x" % (reg0, reg1, reg2, reg3, reg4, reg5, reg6, reg7, reg8, reg9)
#close serial port
      ser.close()
    
   elif len(sys.argv) == 5:
      chip = int(sys.argv[2], 0)
      startAddr = int(sys.argv[3], 0)
      length = int(sys.argv[4], 0)
      if ((0<=chip<=1) and (0<=startAddr<=9) and (1<=length<=10) and (startAddr+length<=10)):
         ser = serial.Serial(sys.argv[1], 115200)
         ser.flushInput()
# send the get ExG regs command
         ser.write(struct.pack('BBBB', 0x63, chip, startAddr, length))
         wait_for_ack()

         ddata = ""
         exgregsresponse = struct.pack('B', 0x62) #EXG_REGS_RESPONSE
         while ddata != exgregsresponse:
            ddata = ser.read(1)

         ddata = ""
         numbytes = 0
         framesize = 1+length 
         while numbytes < framesize:
            ddata += ser.read(framesize)
            numbytes = len(ddata)

         (receivedLength,) = struct.unpack('B', ddata[0:1])
         print "ExG register settings for chip %d, starting from register %s:" % (chip, regList[startAddr]),
         for i in range(length):
            print "0x%02x" % struct.unpack('B', ddata[i+1]),
         print

#close serial port
         ser.close()

      else:
         print "Error: invalid arguments"

   elif len(sys.argv) > 5:
      chip = int(sys.argv[2], 0)
      startAddr = int(sys.argv[3], 0)
      length = int(sys.argv[4], 0)
      if ((0<=chip<=1) and (0<=startAddr<=9) and (1<=length<=10) and (startAddr+length<=10) and (len(sys.argv) == (5+length))):
         packet = struct.pack('BBBB', 0x61, chip, startAddr, length)
         for i in range(length):
            packet += struct.pack('B', int(sys.argv[i+5], 0))
# send the set ExG regs command
         ser = serial.Serial(sys.argv[1], 115200)
         ser.flushInput()
         ser.write(packet)
         wait_for_ack()
         print "ExG settings for chip %d updated" % (chip)
#close serial port
         ser.close()
         
      else:
         print "Error: invalid arguments"
   else:
      print

   print
   print "All done"
