#!/usr/bin/python
import binascii
import serial
import struct
import sys
import time


def wait_for_ack():
    ddata = ""
    ack = struct.pack('B', 0xff)
    while ddata != ack:
        ddata = ser.read(1)
        print("0x%02x" % ord(ddata[0]))

    return


if len(sys.argv) < 2:
    print("no device specified")
    print("You need to specify the serial port of the device you wish to connect to")
    print("example:")
    print("   *.py Com12")
    print("or")
    print("   *.py /dev/rfcomm0")
else:
    # read incoming data
    ddata = ""
    numbytes = 0
    framesize = 9  # 1byte packet type + 2byte timestamp + 3x2byte Analog Accel

    ser = serial.Serial(sys.argv[1], 115200)
    ser.flushInput()
    print("port opening, done.")

    # raw_nb = raw_input('Enter a command:')
    # nb = int(raw_nb, 0)
    nb = 0x72
    framesize = 4
    ser.write(struct.pack('B', nb))
    print("Expecting %d bytes:" % framesize)

    while 1:
        ddata = ""

        ser.write(struct.pack('B', nb))
        time.sleep(0.4)

        buf_len = ser.inWaiting()
        if buf_len == 4:
            ddata = ser.read(framesize)
            ack = struct.unpack('B', ddata[0:1])
            ack = ack[0]
            instream = struct.unpack('B', ddata[1:2])
            instream = instream[0]
            rsp_cmd = struct.unpack('B', ddata[2:3])
            rsp_cmd = rsp_cmd[0]
            status = struct.unpack('B', ddata[3:4])
            status = status[0]
            self_cmd = (status & 0x04) >> 2
            sensing = (status & 0x02) >> 1
            docked = (status & 0x01)

            # print("0x%02x,0x%02x,0x%02x, 0x%02x " % (ack, instream, rsp_cmd, status)
            print("0x%02x,0x%02x,0x%02x, 0x%02x , self: %d, sensing: %d, docked: %d" % (
            ack, instream, rsp_cmd, status, self_cmd, sensing, docked))

        else:
            ddata = ser.read(buf_len)
            print(binascii.hexlify(ddata))
        time.sleep(0.1)

print("All done")
