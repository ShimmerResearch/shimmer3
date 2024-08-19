import unittest
import binascii
import time

from colorama import Fore, Style

from Shimmer_common import shimmer_comms_bluetooth, util_shimmer_time, util_shimmer, shimmer_device, shimmer_app_common
from Shimmer_common import shimmer_comms_docked

import serial
import serial.win32

PACKET_HEADER = 0x24  # "$"


class ShimmerFactoryTestBluetooth(unittest.TestCase):
    shimmer = None
    ser = None

    @classmethod
    def setUpClass(cls):
        com_port = shimmer_app_common.get_selected_com_port(dock_ports=False)
        if not com_port:
            print("Supported COM port not found, exiting")
            exit()

        cls.shimmer = shimmer_device.Shimmer3()

        if not cls.shimmer.setup_bluetooth_com_port(com_port, debug_txrx_packets=True):
            exit()

    @classmethod
    def tearDownClass(cls):
        # cls._connection.destroy()
        cls.shimmer.bluetooth_port.close_port()
        print("All done")

    def test_01_factory_test_bluetooth(self):
        print(Fore.LIGHTMAGENTA_EX + "Factory Test Start")
        tx_bytes = 0
        self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.SET_FACTORY_TEST, tx_bytes])
        self.shimmer.bluetooth_port.wait_for_ack(2000)
        end = "//***************************** TEST END *************************************//\r\n"
        while True:
            response = self.shimmer.bluetooth_port.ser.readline().decode('utf-8')
            # if "PASS" in response:
            #     print(Fore.GREEN + "PASS" + Style.RESET_ALL)
            print(response, end='')
            if response == end:
                print(Fore.LIGHTMAGENTA_EX + "Factory Test End")
                break


class ShimmerFactoryTestDock(unittest.TestCase):
    shimmer = None
    ser = None

    @classmethod
    def setUpClass(cls):
        com_port = shimmer_app_common.get_selected_com_port()
        if not com_port:
            # print("Supported COM port not found, exiting")
            exit()

        cls.shimmer = shimmer_device.Shimmer3()
        if not cls.shimmer.setup_dock_com_port(com_port, debug_txrx_packets=True):
            exit()

    @classmethod
    def tearDownClass(cls):
        # cls._connection.destroy()
        print("All done")

    def test_02_factory_test_dock(self):
        print(Fore.LIGHTMAGENTA_EX + "Factory Test Start")
        tx_bytes = [0x24, 0x01, 0x02, 0x0B, 0x00, 0xDB, 0x0A]
        self.shimmer.dock_port.send_uart(tx_bytes)
        end = "//***************************** TEST END *************************************//\r\n"
        while True:
            # response = data.decode('utf-8')
            response = self.shimmer.dock_port.ser.readline().decode('utf-8')
            print(response, end='')
            if response == end:
                print(Fore.LIGHTMAGENTA_EX + "Factory Test End")
                break
