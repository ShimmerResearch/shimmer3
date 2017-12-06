 #!/usr/bin/python
import sys, struct, serial
import binascii, time

CRC_INIT = 0xB0CA

UART_INIT                  = 0x24
# =======================================================
UART_SET                   = 0x01
UART_RESPONSE              = 0x02
UART_GET                   = 0x03
# =======================================================
UART_COMP_SHIMMER          = 0x01
UART_COMP_BAT              = 0x02
UART_COMP_DAUGHTER_CARD    = 0x03
UART_COMP_D_ACCEL          = 0x04
UART_COMP_GSR              = 0x05
# =======================================================
UART_PROP_CARD_ID          = 0x02
UART_PROP_CARD_MEM         = 0x03
# =======================================================
UART_PROP_ENABLE           = 0x00 # this is for all sensors
UART_PROP_SAMPLE_RATE      = 0x01
UART_PROP_MAC              = 0x02
UART_PROP_VER              = 0x03
UART_PROP_RWC_CFG_TIME     = 0x04
UART_PROP_CURR_LOCAL_TIME  = 0x05
UART_PROP_INFOMEM          = 0x06
UART_PROP_VALUE            = 0x02
UART_PROP_CARD_ID          = 0x02
UART_PROP_CARD_MEM         = 0x03

# =======================================================
DAUGHT_CARD_ID             = 59
DAUGHT_CARD_REV            = 3
DAUGHT_CARD_SPECIAL_REV    = 255
# =======================================================

global counter

# =======================================================

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
   inArg = [UART_INIT, inArg[0], (len(inArg)-1)] + inArg[1:]
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
         if (buf_len >= 5):
            data = ser.read(5)
         else:
            data = ser.read(2)  
         buf_len = ser.inWaiting()    
         # print "      0 1 2 3 4 5 6 7 8 9 a b c d e f"
         # print "cmd:", binascii.hexlify(data), data
         cnt1 = 0
      else:
         if (buf_len>16):
            ddata = ser.read(16)
            # print "0x%02x" % cnt1, binascii.hexlify(ddata) , ddata
            print "Set %d - Get %d" % (counter, int(binascii.hexlify(ddata[2]),16))
            if(counter != int(binascii.hexlify(ddata[2]),16)):
               print "************* BAD VALUE *************"
            buf_len = buf_len - 16;
         elif(buf_len>2):
            ddata = ser.read(buf_len-2)
            # print "0x%02x" % cnt1, binascii.hexlify(ddata) , ddata
            buf_len = 2;
         else:
            ddata = ser.read(buf_len)# for crc 2 bytes
            # print "crc:", binascii.hexlify(ddata) , ddata
            buf_len = 0;
         cnt1 = cnt1 + 16;

   time.sleep(0.1)           
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
   ser = serial.Serial(sys.argv[1], 115200)
   ser.flushInput()
   print "port opening, done."

#set the daughter card ID byte (SR number)

for counter in range(0,256):
   # print "---------------------------------------- mac: "
   # inArg = [UART_GET, UART_COMP_SHIMMER, UART_PROP_MAC]  
   # sendUart(inArg) 
   # print "---------------------------------------- DC ID get: "
   # dcMemLength = 16
   # dcMemOffset = 0
   # inArg = [UART_GET, UART_COMP_DAUGHTER_CARD, UART_PROP_CARD_ID, dcMemLength, dcMemOffset] 
   # sendUart(inArg)
   # print "---------------------------------------- DC ID set: "
   dcMemLength = 3
   dcMemOffset = 0
   # inArg = [UART_SET, UART_COMP_DAUGHTER_CARD, UART_PROP_CARD_ID, dcMemLength, dcMemOffset, hex(DAUGHT_CARD_ID), hex(DAUGHT_CARD_REV), hex(DAUGHT_CARD_SPECIAL_REV)]
   inArg = [UART_SET, UART_COMP_DAUGHTER_CARD, UART_PROP_CARD_ID, dcMemLength, dcMemOffset, 0x1f, 0x07, counter]
   sendUart(inArg) 
   # print "---------------------------------------- DC ID get: "
   dcMemLength = 16
   dcMemOffset = 0
   inArg = [UART_GET, UART_COMP_DAUGHTER_CARD, UART_PROP_CARD_ID, dcMemLength, dcMemOffset] 
   sendUart(inArg) 
   # print "---------------------------------------- DC Mem get: "
   # dcMemLength = 16
   # dcMemOffset_LSB = 0
   # dcMemOffset_MSB = 0
   # inArg = [UART_GET, UART_COMP_DAUGHTER_CARD, UART_PROP_CARD_MEM, dcMemLength, dcMemOffset_LSB, dcMemOffset_MSB] 
   # sendUart(inArg) 
