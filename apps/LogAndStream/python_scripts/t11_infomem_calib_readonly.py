 #!/usr/bin/python
import sys, struct, serial
import binascii, time

CRC_INIT = 0xB0CA
def crcByte(crc, b):
	crc = ((crc >> 8)&0xffff | (crc << 8)&0xffff)&0xffff
	crc ^= (b)&0xffff
	crc ^= ((crc & 0xff) >> 4)&0xffff
	crc ^= (crc << 12)&0xffff
	crc ^= ((crc & 0xff) << 5)&0xffff
	return crc

def calcCrc(len, msg):
	#uint16_t crcCalc;
	#uint8_t i;
	crcCalc = crcByte(CRC_INIT, msg[0])
	for i in range(1,len):
		crcCalc = crcByte(crcCalc, msg[i])
		
	if len%2==1:
		crcCalc = crcByte(crcCalc, 0x00)

	return crcCalc;

def crcCheck(len, msg):
	#uint16_t crc;

	crc = calcCrc(len-2, msg);
	if(((crc&0xFF) == msg[len-2]) and (((crc&0xFF00)>>8) == msg[len-1])):
		return TRUE;
	else:
		return FALSE;

		
def sendUart(inArg):
   ddata = ""
   inCrc = [0, 0]
   inCrcRaw = calcCrc(len(inArg), inArg)
   inCrc[0] = inCrcRaw&0xff
   inCrc[1] = inCrcRaw>>8
   
   ser.write(inArg)
   ser.write(inCrc)
   time.sleep(0.2)   
   buf_len = ser.inWaiting()
   cnt1=9999
   while buf_len>0:
      if (cnt1==9999):
         ddata = ser.read(3)
         print "      0 1 2 3 4 5 6 7 8 9 a b c d e f"
         print "cmd:", binascii.hexlify(ddata), ddata
         buf_len = buf_len - 3;
         cnt1 = 0
      else:
         if (buf_len>16):
            ddata = ser.read(16)
            print "0x%02x" % cnt1, binascii.hexlify(ddata) , ddata
            buf_len = buf_len - 16;
         else:
            ddata = ser.read(buf_len)
            print "0x%02x" % cnt1, binascii.hexlify(ddata) , ddata
            buf_len = 0;
         cnt1 = cnt1 + 16;

   time.sleep(0.1)           
   return
   
def wait_for_ack():
   ddata = ""
   ack = struct.pack('B', 0xff)
   while ddata != ack:
      ddata = ser.read(1)
      print "0x%02x" % ord(ddata[0])
	  
   return
   
if len(sys.argv) < 2:
   print "no device specified"
   print "You need to specify the serial port of the device you wish to connect to"
   print "example:"
   print "   *.py Com12"
   print "or"
   print "   *.py /dev/rfcomm0"
else:
# read incoming data
   ddata = ""
   numbytes = 0
   framesize = 9 # 1byte packet type + 2byte timestamp + 3x2byte Analog Accel
   
   ser = serial.Serial(sys.argv[1], 9600)
   ser.flushInput()
   print "port opening, done."

   #raw_nb = raw_input('Enter a command:')
   #nb = int(raw_nb, 0)
   framesize = 4
      
   ddata = ""
    
   print "mac: ----------------------------------------" 
   inArg = [0x24, 0x01]  
   sendUart(inArg) 
   print "ver: ----------------------------------------" 
   inArg = [0x24, 0x03]  
   sendUart(inArg)   
   print "bat: ----------------------------------------" 
   inArg = [0x24, 0x05]  
   sendUart(inArg)   
   print "cfg time read: ----------------------------------------" 
   inArg = [0x24, 0x08]  
   sendUart(inArg)   
   print "curr time read: ----------------------------------------" 
   inArg = [0x24, 0x0a]  
   sendUart(inArg)   
   print "Daughter Card ID get: ----------------------------------------" 
   inArg = [0x24, 0x0d, 0x02, 0x10, 0x00]  
   sendUart(inArg)  
   #print " - Daughter Card Mem get:" 
   #inArg = [0x24, 0x10, 0x03, 0x10, 0x10, 0x00]  
   #sendUart(inArg) 
   #print " - Daughter Card Mem Set:" 
   #inArg = [0x24, 0x0f, 0x04, 0x01, 0x11, 0x00, 0xab]  
   #sendUart(inArg) 
   #print " - Daughter Card Mem get:" 
   #inArg = [0x24, 0x10, 0x03, 0x10, 0x10, 0x00]  
   #sendUart(inArg) 
   #print " - Daughter Card Mem Set:" 
   #inArg = [0x24, 0x0f, 0x04, 0x01, 0x11, 0x00, 0xff]  
   #sendUart(inArg) 
   #print " - Daughter Card Mem get:" 
   #inArg = [0x24, 0x10, 0x03, 0x10, 0x10, 0x00]  
   #sendUart(inArg) 
   #print " - info Mem get:" 
   #inArg = [0x24, 0x13, 0x03, 0x10, 0xe0, 0x18]  
   #sendUart(inArg) 
   print "info Mem D get: ----------------------------------------" 
   inArg = [0x24, 0x13, 0x03, 0x80, 0x00, 0x18]  
   sendUart(inArg) 
   print "info Mem C get: ----------------------------------------" 
   inArg = [0x24, 0x13, 0x03, 0x80, 0x80, 0x18]  
   sendUart(inArg) 
   print "info Mem B get: ----------------------------------------" 
   inArg = [0x24, 0x13, 0x03, 0x80, 0x00, 0x19]  
   sendUart(inArg) 
         
	  
print 'All done'
