#!/usr/bin/python
import struct
import sys

import serial


def wait_for_ack():
    ddata = ""
    ack = struct.pack('B', 0xff)
    while ddata != ack:
        ddata = ser.read(1)
        print("0x%02x" % ddata[0])
    return


if len(sys.argv) < 2:
    print("no device specified")
    print("You need to specify the serial port of the device you wish to connect to")
    print("example:")
    print("   aAccel5Hz.py Com12")
    print("or")
    print("   aAccel5Hz.py /dev/rfcomm0")
else:
    ser = serial.Serial(sys.argv[1], 115200)
    ser.flushInput()
    print("port opening, done.")
    # send the set sensors command
    ser.write(struct.pack('BBBB', 0x08, 0x80, 0x00, 0x00))  # analogaccel
    wait_for_ack()
    print("sensor setting, done.")
    # send the set sampling rate command
    ser.write(struct.pack('BBB', 0x05, 0x00, 0x19))  # 5.12Hz (6400 (0x1900)). Has to be done like this for alignment reasons
    wait_for_ack()
    print("sampling rate setting, done.")
    # send start streaming and logging command
    ser.write(struct.pack('B', 0x70))
    wait_for_ack()
    print("start command sending, done.")

    # read incoming data
    ddata = bytes()
    numbytes = 0
    framesize = 10  # 1byte packet type + 2byte timestamp + 3x2byte Analog Accel

    print("Packet Type,Timestamp,Analog Accel X,Analog Accel Y,Analog Accel Z")
    try:
        while True:
            while numbytes < framesize:
                ddata += ser.read(framesize)
                numbytes = len(ddata)

            data = ddata[0:framesize]
            ddata = ddata[framesize:]
            numbytes = len(ddata)

            (packettype) = struct.unpack('B', data[0:1])
            (ts0, ts1, ts2) = struct.unpack('BBB', data[1:4])
            timestamp = ts0 + ts1 * 256 + ts2 * 65536

            (analogaccelx, analogaccely, analogaccelz) = struct.unpack('HHH', data[4:framesize])
            print("0x%02x,%5d,\t%4d,%4d,%4d" % (packettype[0], timestamp, analogaccelx, analogaccely, analogaccelz))

    except KeyboardInterrupt:
        # send stop streaming command
        ser.write(struct.pack('B', 0x20))
        print("")
        print("stop command sent, waiting for ACK_COMMAND")
        wait_for_ack()
        print("ACK_COMMAND received.")
        # close serial port
        ser.close()
        print("All done")
