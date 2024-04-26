import time
import unittest

from Shimmer_common import shimmer_comms_bluetooth
from Shimmer_common import shimmer_device, shimmer_app_common


class TestShimmerBluetoothCommunication(unittest.TestCase):
    shimmer = None

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

    def bt_cmd_test_common(self, tx_cmd_byte, expected_response_cmd_byte, expected_response_len,
                           is_instream_response=False, delay_to_wait_for_response_s=0.5):
        if not self.shimmer.bluetooth_port.send_bluetooth([tx_cmd_byte]):
            self.assertTrue(False, "Error writing command")

        time.sleep(delay_to_wait_for_response_s)

        response = self.shimmer.bluetooth_port.wait_for_response(2+expected_response_len)
        self.assertFalse(isinstance(response, bool), "Error reading inquiry response")
        self.assertFalse(response is None, "Error reading inquiry response")

        i = 0
        # ACK
        self.assertTrue(response[i] == shimmer_comms_bluetooth.BtCmds.ACK_COMMAND_PROCESSED)
        i += 1
        if is_instream_response:
            # INSTREAM_CMD_RESPONSE
            self.assertTrue(response[i] == shimmer_comms_bluetooth.BtCmds.INSTREAM_CMD_RESPONSE)
            i += 1
        # Response to command
        self.assertTrue(response[i] == expected_response_cmd_byte)
        i += 1

        # Special cases
        if tx_cmd_byte == shimmer_comms_bluetooth.BtCmds.INQUIRY_COMMAND:
            # read each channel type for the num channels
            idx_length_channels = 2 + 6  # +2 for header bytes
            length_channels = response[idx_length_channels]
            response_channels = self.shimmer.bluetooth_port.wait_for_response(length_channels)
            if isinstance(response_channels, bool):
                self.assertTrue(False, "Error reading inquiry response channels")
            response = response + response_channels

        return response[i:len(response)]

    def test_01_get_inquiry_response(self):
        print("Test 01 - Inquiry command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.INQUIRY_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.INQUIRY_RESPONSE, 8)
        print("")

    def test_02_get_sampling_rate(self):
        print("Test 02 - Get sampling rate command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_SAMPLING_RATE_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.SAMPLING_RATE_RESPONSE, 2)
        print("")

    def test_03_get_accel_sensitivity(self):
        print("Test 03 - Get accelerometer sensitivity command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_ACCEL_RANGE_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.ACCEL_RANGE_RESPONSE, 1)
        print("")

    def test_34_get_status(self):
        print("Test 03 - Get status command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_STATUS_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.STATUS_RESPONSE, 2, is_instream_response=True)

        status = response[0]
        self_cmd = (status & 0x04) >> 2
        sensing = (status & 0x02) >> 1
        docked = (status & 0x01)
        print("0x%02x , self: %d, sensing: %d, docked: %d" % (status, self_cmd, sensing, docked))

        print("")

    def test_XX_set_calibration(self):
        print("Test XX - Set Calibration command:")

        calib_bytes = [0x03, 0x00, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x15, 0x00, 0x00, 0x00, 0x00,
                       0x00, 0x00, 0x00, 0x00, 0x08, 0xCD, 0x08, 0xCD, 0x08, 0xCD, 0x00, 0x5C, 0x00, 0x5C, 0x00, 0x5C,
                       0x00, 0x9C, 0x00, 0x9C, 0x00, 0x00, 0x00, 0x00, 0x9C, 0x1E, 0x00, 0x00, 0x15, 0x00, 0x00, 0x00,
                       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x33, 0x2C, 0x33, 0x2C, 0x33,
                       0x2C, 0x00, 0x9C, 0x00, 0x9C, 0x00, 0x00, 0x00, 0x00, 0x9C, 0x1E, 0x00, 0x01, 0x15, 0x00, 0x00,
                       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x96, 0x19, 0x96,
                       0x19, 0x96, 0x00, 0x9C, 0x00, 0x9C, 0x00, 0x00, 0x00, 0x00, 0x9C, 0x1E, 0x00, 0x02, 0x15, 0x00,
                       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0xD0, 0x0C,
                       0xD0, 0x0C, 0xD0, 0x00, 0x9C, 0x00, 0x9C, 0x00, 0x00, 0x00, 0x00, 0x9C, 0x1E, 0x00, 0x03, 0x15,
                       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x68,
                       0x06, 0x68, 0x06, 0x68, 0x00, 0x9C, 0x00, 0x9C, 0x00, 0x00, 0x00, 0x00, 0x9C, 0x1F, 0x00, 0x00,
                       0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06,
                       0x87, 0x06, 0x87, 0x06, 0x87, 0x00, 0x9C, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x9C, 0x1F, 0x00,
                       0x01, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                       0x00, 0xD1, 0x00, 0xD1, 0x00, 0xD1, 0x00, 0x9C, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x9C, 0x1F,
                       0x00, 0x02, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                       0x00, 0x03, 0x44, 0x03, 0x44, 0x03, 0x44, 0x00, 0x9C, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x9C,
                       0x1F, 0x00, 0x03, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                       0x00, 0x00, 0x01, 0xA2, 0x01, 0xA2, 0x01, 0xA2, 0x00, 0x9C, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00,
                       0x9C, 0x20, 0x00, 0x00, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                       0x00, 0x00, 0x00, 0x02, 0x9B, 0x02, 0x9B, 0x02, 0x9B, 0x00, 0x9C, 0x00, 0x64, 0x00, 0x00, 0x00,
                       0x00, 0x9C]

        if not self.shimmer.bluetooth_port.write_calibration(calib_bytes):
            print("Error, exiting")
            self.assertTrue(False)

        calib_bytes = self.shimmer.bluetooth_port.read_calibration(calib_bytes)
        if isinstance(calib_bytes, bool):
            print("Error, exiting")
            self.assertTrue(False)

        print("")


if __name__ == '__main__':
    unittest.main()
