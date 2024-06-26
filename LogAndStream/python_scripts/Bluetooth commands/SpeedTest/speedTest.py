#!/usr/bin/python
import binascii
import serial
import struct
import sys
import time

from multiprocessing import Process

import speedTestPlot as plot
import speedTestBle as ble

from pc_ble_driver_py.observers import *
import logging

global ser

sleep_interval = 0.05
framesize = 4  # 32-bit counter value
ddata = b""

time_s_overall_start = 0
time_s_current_window_start = 0
num_bytes_total = 0
data_integrity_counter = 0


def wait_for_ack():
    global ser
    ddata = ""
    ack = struct.pack('B', 0xff)
    while ddata != ack:
        ddata = ser.read(1)
        if ddata != ack:
            print("0x%02x" % int(ddata[0]))
    print("ACK Received")
    return


def bt_classic_parser(com_port, q, sample_index):
    global ser, num_bytes_total, time_s_overall_start, time_s_current_window_start, ddata, data_integrity_counter

    ser = serial.Serial(com_port, 1000000)
    ser.flushInput()
    logger.info("port opening, done.")

    initialise_speed_test_vars()

    ser.write(struct.pack('BB', 0xA4, 1))
    wait_for_ack()

    while 1:
        time_s_current = time.time_ns() / 1000000000

        buf_len = ser.inWaiting()
        if buf_len > 0:
            ddata = ddata + ser.read(buf_len)

        process_new_data(q, time_s_current)

        time.sleep(sleep_interval)


def initialise_speed_test_vars():
    global num_bytes_total, time_s_overall_start, time_s_current_window_start, data_integrity_counter
    time_s_overall_start = time.time_ns() / 1000000000
    time_s_current_window_start = time_s_overall_start
    num_bytes_total = 0
    data_integrity_counter = 0


def process_new_data(q, time_s_current):
    global num_bytes_total, time_s_overall_start, time_s_current_window_start, ddata, data_integrity_counter

    counter_values = []
    end_index = 0
    data_integrity = []
    data_integrity_issue_flag = False

    if len(ddata) >= framesize:

        num_counter_values = len(ddata) // framesize

        fmt = '<' + ('I' * num_counter_values)
        end_index = num_counter_values * framesize
        num_bytes_total += end_index

        counter_values = struct.unpack(fmt, ddata[0:end_index])

        for sample in counter_values:
            data_integrity_issue_flag = True if data_integrity_counter == sample else False
            data_integrity_counter = sample + 1
            data_integrity.append(data_integrity_issue_flag)

            if data_integrity_counter == 2 ** (framesize * 8):
                data_integrity_counter = 0

        if len(ddata) > end_index:
            ddata = ddata[end_index:]

    data_rate_current = 0 if end_index == 0 else (end_index / 1024) / (time_s_current - time_s_current_window_start)
    data_rate_overall = (num_bytes_total / 1024) / (time_s_current - time_s_overall_start)

    plot.plot_data(q, time_s_current_window_start, time_s_current, counter_values, data_rate_current, data_rate_overall, data_integrity)

    time_s_current_window_start = time_s_current

    sys.stdout.write("\rData Rate: Current = %i KB/s, Overall = %i KB/s, Data integrity = %s" % (data_rate_current, data_rate_overall, "OK" if data_integrity_issue_flag else "Bad"))
    sys.stdout.flush()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("no device specified")
        print("You need to specify the serial port of the device you wish to connect to")
        print("example:")
        print("   *.py Com12")
        print("or")
        print("   *.py /dev/rfcomm0")
    else:
        plot.setup_plots()

        # ble.setup()

        # COM15 = A615
        # COM3 = D1E8
        # COM24 = Shimmer3r 32FD
        # COM7 = Shimmer3r 29FD

        p = Process(target=bt_classic_parser, args=(sys.argv[1], plot.q, plot.sample_index))
        p.start()

        plot.start_thread()

        # p.terminate()
        # p.join()

    print("All done")
