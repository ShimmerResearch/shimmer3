import time
import unittest

from Shimmer_common import shimmer_comms_bluetooth, util_shimmer_time, util_shimmer
from Shimmer_common import shimmer_device, shimmer_app_common

from colorama import Fore


def does_response_include_length_byte(get_cmd):
    return (get_cmd == shimmer_comms_bluetooth.BtCmds.GET_DAUGHTER_CARD_ID_COMMAND
            or get_cmd == shimmer_comms_bluetooth.BtCmds.GET_DAUGHTER_CARD_MEM_COMMAND
            or get_cmd == shimmer_comms_bluetooth.BtCmds.GET_EXG_REGS_COMMAND
            or get_cmd == shimmer_comms_bluetooth.BtCmds.GET_DIR_COMMAND
            or get_cmd == shimmer_comms_bluetooth.BtCmds.GET_CONFIGTIME_COMMAND
            or get_cmd == shimmer_comms_bluetooth.BtCmds.GET_SHIMMERNAME_COMMAND
            or get_cmd == shimmer_comms_bluetooth.BtCmds.GET_EXPID_COMMAND
            or get_cmd == shimmer_comms_bluetooth.BtCmds.GET_BT_VERSION_STR_COMMAND
            or get_cmd == shimmer_comms_bluetooth.BtCmds.GET_PRESSURE_CALIBRATION_COEFFICIENTS_COMMAND)


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

    def bt_cmd_test_get_common(self, tx_cmd_byte, expected_response_cmd_byte, expected_response_len,
                               is_instream_response=False, delay_to_wait_for_response_ms=2000):

        if isinstance(tx_cmd_byte, list):
            tx_cmd_byte_array = tx_cmd_byte
            tx_cmd_byte = tx_cmd_byte_array[0]
        else:
            tx_cmd_byte_array = [tx_cmd_byte]

        i = 0

        if tx_cmd_byte == shimmer_comms_bluetooth.BtCmds.GET_INFOMEM_COMMAND:
            response = self.shimmer.bluetooth_port.read_configuration()

            self.assertFalse(isinstance(response, bool) or response is None or len(response) == 0,
                             "Error reading response")

        elif tx_cmd_byte == shimmer_comms_bluetooth.BtCmds.GET_CALIB_DUMP_COMMAND:
            response = self.shimmer.bluetooth_port.read_calibration()

            self.assertFalse(isinstance(response, bool) or response is None or len(response) == 0,
                             "Error reading response")

        else:
            if not self.shimmer.bluetooth_port.send_bluetooth(tx_cmd_byte_array):
                self.assertTrue(False, "Error writing command")

            header_byte_count = 3 if is_instream_response else 2
            response = self.shimmer.bluetooth_port.wait_for_response(header_byte_count + expected_response_len,
                                                                     delay_to_wait_for_response_ms)

            self.assertFalse(isinstance(response, bool) or response is None or len(response) == 0,
                             "Error reading response")

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

            elif does_response_include_length_byte(tx_cmd_byte):
                response = self.shimmer.bluetooth_port.wait_for_response(response[i])
                if isinstance(response, bool) or response is None or len(response) == 0:
                    if tx_cmd_byte == shimmer_comms_bluetooth.BtCmds.GET_DAUGHTER_CARD_ID_COMMAND:
                        self.assertTrue(False, "Error reading daughter card ID ")
                    elif tx_cmd_byte == shimmer_comms_bluetooth.BtCmds.GET_DAUGHTER_CARD_MEM_COMMAND:
                        self.assertTrue(False, "Error reading daughter card memory")
                    elif tx_cmd_byte == shimmer_comms_bluetooth.BtCmds.GET_EXG_REGS_COMMAND:
                        self.assertTrue(False, "Error reading exg regs")
                    elif tx_cmd_byte == shimmer_comms_bluetooth.BtCmds.GET_DIR_COMMAND:
                        self.assertTrue(False, "Error reading directory")
                    elif tx_cmd_byte == shimmer_comms_bluetooth.BtCmds.GET_CONFIGTIME_COMMAND:
                        self.assertTrue(False, "Error reading config time")
                    elif tx_cmd_byte == shimmer_comms_bluetooth.BtCmds.GET_SHIMMERNAME_COMMAND:
                        self.assertTrue(False, "Error reading directory")
                    elif tx_cmd_byte == shimmer_comms_bluetooth.BtCmds.GET_BT_VERSION_STR_COMMAND:
                        self.assertTrue(False, "Error bt version command")
                    elif tx_cmd_byte == shimmer_comms_bluetooth.BtCmds.GET_PRESSURE_CALIBRATION_COEFFICIENTS_COMMAND:
                        self.assertTrue(False, "Error get pressure calibration")
                    else:
                        self.assertTrue(False, "Error - unhandled")
                i = 0

            # Clear CRC byte
            if self.shimmer.bt_crc_byte_count > 0:
                print("Clear CRC byte(s)")
                self.shimmer.bluetooth_port.wait_for_response(1)

            time.sleep(0.2)
            unexpected_byte_len = self.shimmer.bluetooth_port.get_qty_waiting_in_port()
            if unexpected_byte_len > 0:
                unexpected_bytes = self.shimmer.bluetooth_port.wait_for_response(unexpected_byte_len)
                print("Unexpected RX bytes =", util_shimmer.byte_array_to_hex_string(unexpected_bytes))
                self.assertTrue(False, "Unexpected bytes in COM port")

        return response[i:len(response)]

    def bt_cmd_test_set_common(self, set_cmd, set_bytes, get_cmd, response_cmd, set_delay_s=0.5, check_original_value=True):

        comparison_offset = 0
        length_to_read = len(set_bytes)
        get_cmd_byte = get_cmd[0] if isinstance(get_cmd, list) else get_cmd
        if does_response_include_length_byte(get_cmd_byte):
            length_to_read = 1
            if (get_cmd_byte != shimmer_comms_bluetooth.BtCmds.GET_EXG_REGS_COMMAND
                    and get_cmd_byte != shimmer_comms_bluetooth.BtCmds.GET_DAUGHTER_CARD_ID_COMMAND
                    and get_cmd_byte != shimmer_comms_bluetooth.BtCmds.GET_DAUGHTER_CARD_MEM_COMMAND):
                comparison_offset = 1

        if check_original_value:
            # Make sure the value being set isn't the same as the one that's already set in the Shimmer - otherwise it's not a valid test
            print("Reading original setting from Shimmer:")
            response = self.bt_cmd_test_get_common(get_cmd, response_cmd, length_to_read)
            self.compare_set_get_arrays(set_bytes, response, comparison_offset, get_cmd_byte, True)

        bytes_to_send = set_cmd if isinstance(set_cmd, list) else [set_cmd]
        bytes_to_send += set_bytes

        print("Writing new setting to Shimmer:")
        # Special cases
        if set_cmd == shimmer_comms_bluetooth.BtCmds.SET_INFOMEM_COMMAND:
            result = self.shimmer.bluetooth_port.write_configuration(set_bytes)
            if not result:
                self.assertTrue(False)
        elif set_cmd == shimmer_comms_bluetooth.BtCmds.SET_CALIB_DUMP_COMMAND:
            result = self.shimmer.bluetooth_port.write_calibration(set_bytes)
            if not result:
                self.assertTrue(False)
        else:
            self.shimmer.bluetooth_port.send_bluetooth(bytes_to_send)
            self.bt_cmd_test_wait_for_ack(2000)

        # Delay to allow the Shimmer to enact the changes
        time.sleep(set_delay_s)

        print("Reading new setting from Shimmer:")
        response = self.bt_cmd_test_get_common(get_cmd, response_cmd, length_to_read)

        # Compare what was sent with what has been received
        if get_cmd_byte == shimmer_comms_bluetooth.BtCmds.GET_RWC_COMMAND:
            ts_s_written = util_shimmer_time.shimmer_rtc_bytes_to_s(set_bytes)
            ts_s_read = util_shimmer_time.shimmer_rtc_bytes_to_s(response)

            print("\r\nTime sent = %s\r\nTime received = %s" % (
                util_shimmer_time.seconds_to_time_str(ts_s_written, True),
                util_shimmer_time.seconds_to_time_str(ts_s_read, True)))

            if (ts_s_read - ts_s_written) > (0.5 + set_delay_s):
                self.assertTrue(False, "RWC time out of range")
        else:
            self.compare_set_get_arrays(set_bytes, response, comparison_offset, get_cmd_byte, False)

    def compare_set_get_arrays(self, set_bytes, response, comparison_offset, get_cmd_byte, check_original_value):
        failed_indexes = []

        for i in range(0, len(set_bytes) - comparison_offset):
            if ((len(response) > i)
                    and ((check_original_value and (set_bytes[i + comparison_offset] is response[i]))
                         or (not check_original_value and (set_bytes[i + comparison_offset] is not response[i])))):
                if (get_cmd_byte == shimmer_comms_bluetooth.BtCmds.GET_INFOMEM_COMMAND
                      and ((128 + 96) <= i <= (128 + 101))):
                    # print("Skipping MAC ID in infomem C")
                    continue
                elif (get_cmd_byte == shimmer_comms_bluetooth.BtCmds.GET_CALIB_DUMP_COMMAND
                      and (2 <= i <= 9)):
                    # print("Skipping device and firmware version from calib file header")
                    continue
                else:
                    failed_indexes += [i]

        if ((check_original_value and len(failed_indexes) == len(set_bytes) - comparison_offset)
                or (not check_original_value and len(failed_indexes) > 0)):
            print("Comparison offset = ", comparison_offset)
            print("Failure indexes = ", failed_indexes)
            print("Indexes =  [", end="")
            for x in range(len(set_bytes)):
                print(f"{x:4}", end=" ")
            print("]")
            print("TX bytes =",
                  util_shimmer.byte_array_to_hex_string(set_bytes[comparison_offset:len(set_bytes)]))
            print("RX bytes =", util_shimmer.byte_array_to_hex_string(response))

            if check_original_value:
                self.assertTrue(False, (
                    "Original value in Shimmer equals test value and is therefore not a valid test of setting being changed"))
            else:
                self.assertTrue(False, "RX byte != TX byte at indexes listed in console")

    def bt_cmd_test_wait_for_ack(self, timeout_ms=500):
        result = self.shimmer.bluetooth_port.wait_for_ack(timeout_ms)
        if not result:
            self.assertTrue(False, "Error waiting for ACK")

        # Clear CRC byte
        if self.shimmer.bt_crc_byte_count > 0:
            print("Clear CRC byte(s)")
            self.shimmer.bluetooth_port.wait_for_response(1)

    def setUp(self):
        print("")
        self.shimmer.bluetooth_port.clear_serial_buffer()
        # time.sleep(0.1)

    def test_001_get_inquiry_response(self):
        print("\r\nTest 001 - Inquiry command:::")

        if not self.shimmer.is_hardware_version_set():
            self.test_004_get_shimmer_new_version(True)

        num_inquiry_bytes = 11 if self.shimmer.is_hardware_shimmer3r() else 8

        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.INQUIRY_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.INQUIRY_RESPONSE, num_inquiry_bytes)

    def test_002_reset_default_config(self, run_with_other_test=False):
        if not run_with_other_test:
            print("Test 002 - Reset Default config:")
        self.shimmer.bluetooth_port.send_bluetooth(
            [shimmer_comms_bluetooth.BtCmds.RESET_TO_DEFAULT_CONFIGURATION_COMMAND])
        self.bt_cmd_test_wait_for_ack()

    def test_003_reset_default_calib(self, run_with_other_test=False):
        if not run_with_other_test:
            print("Test 003 - Reset Default calib:")
        self.shimmer.bluetooth_port.send_bluetooth(
            [shimmer_comms_bluetooth.BtCmds.RESET_CALIBRATION_VALUE_COMMAND])
        self.bt_cmd_test_wait_for_ack()

    def test_004_get_shimmer_new_version(self, run_with_other_test=False):
        if not run_with_other_test:
            print("Test 004 - Get new Shimmer version response command:")
        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.GET_DEVICE_VERSION_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.DEVICE_VERSION_RESPONSE, 1)
        self.shimmer.hw_ver = response[0]

    def test_005_get_fw_version(self, run_with_other_test=False):
        if not run_with_other_test:
            print("Test 005 - Get FW response command")
        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.GET_FW_VERSION_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.FW_VERSION_RESPONSE, 6)
        self.shimmer.parse_fw_ver_bytes(response)
        self.shimmer.print_hw_fw_revision()

    def test_006_get_daughter_card_id(self, run_with_other_test=False):
        if not run_with_other_test:
            print("Test 006 - Get daughter card id command:")

        # Read 1 byte as the first byte in the response is the length of the number of bytes in the rest of the response
        response = self.bt_cmd_test_get_common(
            [shimmer_comms_bluetooth.BtCmds.GET_DAUGHTER_CARD_ID_COMMAND, 0x03, 0x00],
            shimmer_comms_bluetooth.BtCmds.DAUGHTER_CARD_ID_RESPONSE, 1)

        self.shimmer.parse_daughter_card_id(response)
        self.shimmer.print_daughter_card_id()

    def test_007_get_sampling_rate(self):
        print("Test 007 - Get sampling rate command:")
        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.GET_SAMPLING_RATE_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.SAMPLING_RATE_RESPONSE, 2)

    def test_008_get_accel_sensitivity(self):
        print("Test 008 - Get accelerometer sensitivity command:")
        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.GET_ACCEL_RANGE_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.ACCEL_RANGE_RESPONSE, 1)

    def test_009_get_config_setup_bytes(self):
        print("Test 009- Get Config Setup Bytes command:")
        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.GET_CONFIG_SETUP_BYTES_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.CONFIG_SETUP_BYTES_RESPONSE, 4)

    def test_010_get_accel_calibration(self):
        print("Test 010 - Get Accel Calibration Command:")
        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.GET_A_ACCEL_CALIBRATION_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.A_ACCEL_CALIBRATION_RESPONSE, 21)
        self.assertFalse(all(response) == 0, "FAIL")

    def test_011_get_gyro_calibration(self):
        print("Test 011 - Get Gyro Calibration Command:")
        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.GET_GYRO_CALIBRATION_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.GYRO_CALIBRATION_RESPONSE, 21)
        self.assertFalse(all(response) == 0, "FAIL")

    def test_012_get_mag_calibration(self):
        print("Test 012 - Get Mag Calibration Command:")
        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.GET_MAG_CALIBRATION_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.MAG_CALIBRATION_RESPONSE, 21)
        self.assertFalse(all(response) == 0, "FAIL")

    def test_013_get_gsr_range(self):
        print("Test 013 - Get GSR Command:")
        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.GET_GSR_RANGE_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.GSR_RANGE_RESPONSE, 1)

    def test_014_all_calibration_response(self):
        print("Test 014 - Get all calibration response command")
        if not self.shimmer.is_hardware_version_set():
            self.test_04_get_shimmer_new_version(True)

        num_sensors = 6 if self.shimmer.is_hardware_shimmer3r() else 4
        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.GET_ALL_CALIBRATION_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.ALL_CALIBRATION_RESPONSE,
                                               num_sensors * 21)
        self.assertFalse(all(response) == 0, "FAIL")

    def test_015_get_buffer_size(self):
        print("Test 015 - Buffer size repsonse Command: ")
        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.GET_BUFFER_SIZE_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.BUFFER_SIZE_RESPONSE,
                                               1)  # get buffer size or response as 2 commands are mentioned

    def test_016_get_mag_gain(self):
        print("Test 016 - Get mag gain command:")
        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.GET_MAG_GAIN_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.MAG_GAIN_RESPONSE, 1)

    def test_017_get_charge_status_led(self):
        print("Test 017 - Get Charge Status LED response command:")
        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.GET_CHARGE_STATUS_LED_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.CHARGE_STATUS_LED_RESPONSE, 1)
        if response[0] == 0:
            print("Battery High")
        elif response[0] == 1:
            print("Battery Mid")
        elif response[0] == 2:
            print("Battery Low")

    def test_018_get_mag_sampling(self):
        print("Test 018 - Get Mag sampling Command:")
        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.GET_MAG_SAMPLING_RATE_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.MAG_SAMPLING_RATE_RESPONSE, 1)

    def test_019_get_accel_sampling_rate(self):
        print("Test 019 - Get accel sampling rate response command:")
        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.GET_ACCEL_SAMPLING_RATE_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.ACCEL_SAMPLING_RATE_RESPONSE, 1)

    def test_020_get_wr_accel_lpmode(self):
        print("Test 020 - Get WR accel lpmode command:")
        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.GET_ACCEL_LPMODE_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.ACCEL_LPMODE_RESPONSE, 1)

    def test_021_get_wr_accel_hrmode(self):
        print("Test 021 - GET WR accel hrmode command:")
        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.GET_ACCEL_HRMODE_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.ACCEL_HRMODE_RESPONSE, 1)

    def test_022_get_gyro_range(self):
        print("Test 022 - Get gyro range command:")
        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.GET_GYRO_RANGE_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.GYRO_RANGE_RESPONSE, 1)

    def test_023_get_gyro_sampling_rate(self):
        print("Test 023 - Get gyro sampling rate command:")
        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.GET_GYRO_SAMPLING_RATE_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.GYRO_SAMPLING_RATE_RESPONSE, 1)

        print(util_shimmer.byte_array_to_hex_string(response))

    def test_024_get_BMPX80_Calibration_coefficients(self):
        print("Test 024 - Get BMPX80 Calibration response command:")

        if not self.shimmer.is_hardware_version_set():
            self.test_04_get_shimmer_new_version(True)
        if not self.shimmer.is_expansion_board_set():
            self.test_06_get_daughter_card_id(True)

        if self.shimmer.is_bmp180_present():
            response = self.bt_cmd_test_get_common(
                shimmer_comms_bluetooth.BtCmds.GET_BMP180_CALIBRATION_COEFFICIENTS_COMMAND,
                shimmer_comms_bluetooth.BtCmds.BMP180_CALIBRATION_COEFFICIENTS_RESPONSE, 22)
            self.assertFalse(all(response) == 0, "FAIL")
        elif self.shimmer.is_bmp280_present():
            response = self.bt_cmd_test_get_common(
                shimmer_comms_bluetooth.BtCmds.GET_BMP280_CALIBRATION_COEFFICIENTS_COMMAND,
                shimmer_comms_bluetooth.BtCmds.BMP280_CALIBRATION_COEFFICIENTS_RESPONSE, 24)
            self.assertFalse(all(response) == 0, "FAIL")
        else:
            print("Skipping test, BMP180/BMP280 not present in device")

    def test_025_get_alt_mag_sens_adj_vals_response(self):
        # Shimmer3 utilising ICM-20948 instead of MPU9150/MPU9250 will only respond back with ACK as the ICM-20948
        # does not support changing the mag range.
        print("Test 025 - Get alternative Mag Sens Adj Vals response command:")

        if not self.shimmer.is_hardware_version_set():
            self.test_004_get_shimmer_new_version(True)
        if not self.shimmer.is_expansion_board_set():
            self.test_006_get_daughter_card_id(True)

        if self.shimmer.hw_ver == shimmer_device.SrHwVer.SHIMMER3R.value or self.shimmer.is_icm20948_present():
            self.shimmer.bluetooth_port.send_bluetooth(
                [shimmer_comms_bluetooth.BtCmds.GET_ALT_MAG_SENS_ADJ_VALS_COMMAND])
            self.bt_cmd_test_wait_for_ack()
        else:
            response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.GET_ALT_MAG_SENS_ADJ_VALS_COMMAND,
                                                   shimmer_comms_bluetooth.BtCmds.ALT_MAG_SENS_ADJ_VALS_RESPONSE, 3)

    def test_026_get_internal_exp_power_enable(self):
        print("Test 026 - Get exp power enable response command:")
        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.GET_INTERNAL_EXP_POWER_ENABLE_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.INTERNAL_EXP_POWER_ENABLE_RESPONSE, 1)

    def test_027_get_exg_regs(self):
        print("Test 027 - Get ExG Regs command:")
        # Expecting 3 additional bytes, exgChip index, exgStartAddr, exgLength

        # Chip 0
        response = self.bt_cmd_test_get_common([shimmer_comms_bluetooth.BtCmds.GET_EXG_REGS_COMMAND, 0, 0, 10],
                                               shimmer_comms_bluetooth.BtCmds.EXG_REGS_RESPONSE, 1)

        # Chip 1
        response = self.bt_cmd_test_get_common([shimmer_comms_bluetooth.BtCmds.GET_EXG_REGS_COMMAND, 1, 0, 10],
                                               shimmer_comms_bluetooth.BtCmds.EXG_REGS_RESPONSE, 1)

    def test_028_get_status(self, run_with_other_test=False):
        if not run_with_other_test:
            print("Test 028 - Get status command:")
        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.GET_STATUS_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.STATUS_RESPONSE, 1,
                                               is_instream_response=True)

        status = response[0]
        self.shimmer.parse_status(status)

    def test_029_get_baud_rate(self):
        print("Test 029 - Get baud rate response command:")
        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.GET_BT_COMMS_BAUD_RATE,
                                               shimmer_comms_bluetooth.BtCmds.BT_COMMS_BAUD_RATE_RESPONSE, 1)

    def test_030_get_derived_channel_bytes(self):
        print("Test 030 - Get Derived Channel Bytes response command:")
        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.GET_DERIVED_CHANNEL_BYTES,
                                               shimmer_comms_bluetooth.BtCmds.DERIVED_CHANNEL_BYTES_RESPONSE, 8)

    def test_031_get_trial_config(self):
        print("Test 031 - Get trial config command:")
        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.GET_TRIAL_CONFIG_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.TRIAL_CONFIG_RESPONSE, 3)

    def test_032_get_center(self):
        print("Test 032 - Get Center response command:")
        # TODO GET_CENTER_COMMAND not currently supported in LogAndStream but could be added in the future
        # response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_CENTER_COMMAND,
        #                                    shimmer_comms_bluetooth.BtCmds.CENTER_RESPONSE, 1)

    def test_033_get_shimmer_name(self):
        print("Test 033 - Get ShimmerName command:")
        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.GET_SHIMMERNAME_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.SHIMMERNAME_RESPONSE, 1)
        print(bytes(response))

    def test_034_get_expID(self):
        print("Test 034 - Get ExpID command:")
        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.GET_EXPID_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.EXPID_RESPONSE, 1)
        print(bytes(response))

    def test_035_get_myID(self):
        print("Test 035 - Get myID command:")  # works
        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.GET_MYID_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.MYID_RESPONSE, 1)

    def test_036_get_number_of_shimmers_in_trial(self):
        print("Test 036 - Get nshimmer response command:")
        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.GET_NSHIMMER_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.NSHIMMER_RESPONSE, 1)

    def test_037_get_ConfigTime(self):
        print("Test 037- Get Config Time Response Command:")
        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.GET_CONFIGTIME_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.CONFIGTIME_RESPONSE, 1)
        print(util_shimmer_time.seconds_to_time_str(int(bytes(response)), True))

    def test_038_get_dir(self):
        print("Test 038 - Get dir response command:")

        self.test_028_get_status(True)

        if self.shimmer.status_is_docked:
            self.assertTrue(False, "Shimmer must be undocked for this command to work reliably")
        elif not self.shimmer.status_sd_in_slot:
            self.assertTrue(False, "No SD card detected")
        elif self.shimmer.status_sd_error:
            self.assertTrue(False, "SD card error detected")
        else:
            response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.GET_DIR_COMMAND,
                                                   shimmer_comms_bluetooth.BtCmds.DIR_RESPONSE, 1,
                                                   is_instream_response=True)
            print(bytes(response))

    def test_039_get_infomem(self):
        print("Test 039 - Get infomem response command:")

        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.GET_INFOMEM_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.INFOMEM_RESPONSE, 1)

        print("Config bytes: " + util_shimmer.byte_array_to_hex_string(response))

    def test_040_get_calib_dump(self):
        print("Test 040 - Get calib dump:")

        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.GET_CALIB_DUMP_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.RSP_CALIB_DUMP_COMMAND, 1)

        self.assertFalse(all(response) == 0, "FAIL")

        print("Calib bytes: " + util_shimmer.byte_array_to_hex_string(response))

        # print(util_shimmer.byte_array_to_hex_string(calib_dump_concat))

    def test_041_get_rwc(self):
        print("Test 041- Get RWC response")
        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.GET_RWC_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.RWC_RESPONSE, 8)

        ts_s = util_shimmer_time.shimmer_rtc_bytes_to_s(response)
        print(util_shimmer_time.seconds_to_time_str(ts_s, True))

    def test_042_get_vbatt(self):
        print("Test 042 - Get VBatt response command")
        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.GET_VBATT_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.VBATT_RESPONSE, 3,
                                               is_instream_response=True)
        batt_voltage = ((response[0] & 0xFF) << 8) + (response[0] & 0xFF)
        charging_status = response[1]
        print("batt_voltage: %03f , charging_status: %d" % (batt_voltage, charging_status))

    def test_043_get_BT_FW_VERSION(self):
        print("Test 043 - Get BT FW Version response command")
        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.GET_BT_VERSION_STR_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.BT_VERSION_STR_RESPONSE, 1)
        print(bytes(response))

    def test_044_get_unique_serial_response(self):
        print("Test 044 - Unique Serial Response")
        if not self.shimmer.is_hardware_version_set():
            self.test_004_get_shimmer_new_version(True)

        expected_len = 1
        if self.shimmer.hw_ver == shimmer_device.SrHwVer.SHIMMER3.value:
            expected_len = 8
        elif self.shimmer.hw_ver == shimmer_device.SrHwVer.SHIMMER3R.value:
            expected_len = 12

        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.GET_UNIQUE_SERIAL_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.UNIQUE_SERIAL_RESPONSE, expected_len)

        print("MCU's unique serial ID: ", response)

    def test_045_get_pres_oversampling_ratio(self):
        print("Test 045 - Pres Oversampling")
        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.GET_PRES_OVERSAMPLING_RATIO_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.PRES_OVERSAMPLING_RATIO_RESPONSE, 1)
        print(response)

    def test_046_get_alt_accel_range(self):
        print("Test 046 - ALT accel range")
        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.GET_ALT_ACCEL_RANGE_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.ALT_ACCEL_RANGE_RESPONSE, 1)
        print(response)

    def test_047_get_daughter_card_mem(self):
        print("Test 047 - Get daughter card mem")
        response = self.bt_cmd_test_get_common([shimmer_comms_bluetooth.BtCmds.GET_DAUGHTER_CARD_MEM_COMMAND, 10, 0, 0],
                                               shimmer_comms_bluetooth.BtCmds.DAUGHTER_CARD_MEM_RESPONSE, 1)
        print(response)


    # set commands

    def test_048_set_sampling_rate(self):
        print("Test 048 - set accel sensitivity command ")

        sampling_rate_hz = 102.4
        sampling_rate_ticks = int(32768/sampling_rate_hz)

        tx_bytes = [sampling_rate_ticks & 0xFF, (sampling_rate_ticks >> 8) & 0xFF]
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_SAMPLING_RATE_COMMAND,
                                    tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_SAMPLING_RATE_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.SAMPLING_RATE_RESPONSE)

    def test_049_set_sensors(self):
        print("Test 049 - set sensors command ")

        # Enable: LN Accel, Gyro, Mag, ExtCh7, ExtCh6, Battery, WR Accel, ExtCh15, IntCh1, IntCh12, IntCh13, IntCh14, Pressure (i.e., 11 analog ch and 11 digital ch)
        tx_bytes = [0xE3, 0x3F, 0x84]
        self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.SET_SENSORS_COMMAND] + tx_bytes)
        self.bt_cmd_test_wait_for_ack()

        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.INQUIRY_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.INQUIRY_RESPONSE, 8)
        if response[6] != 22:
            self.assertTrue(False)

        # reenable default config: LN accel, gyro, mag and battery voltage enabled (i.e., 4 analog ch and 6 digital ch)
        tx_bytes = [0xE0, 0x20, 0x00]
        self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.SET_SENSORS_COMMAND] + tx_bytes)
        self.bt_cmd_test_wait_for_ack()

        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.INQUIRY_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.INQUIRY_RESPONSE, 8)
        if response[6] != 10:
            self.assertTrue(False)

    def test_050_set_accel_sensitivity(self):
        print("Test 050 - set accel sensitivity command ")
        tx_bytes = [0x01]  # default 0
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_ACCEL_RANGE_COMMAND,
                                    tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_ACCEL_RANGE_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.ACCEL_RANGE_RESPONSE)

    def test_051_set_config_bytes(self):
        print("Test 051 - set Config Bytes command ")
        tx_bytes = [0x06, 0x01, 0x02, 0x03]  # default
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_CONFIG_SETUP_BYTES_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_CONFIG_SETUP_BYTES_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.CONFIG_SETUP_BYTES_RESPONSE)

    def test_052_set_A_accel_calibration(self):
        print("Test 052 - Set accel Calibration Command")
        tx_bytes = [0x03, 0x00, 0x03, 0x00, 0x01, 0x11, 0x00, 0x00, 0x02, 0x00, 0x00, 0x15, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08]  # default 0xff, 84
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_A_ACCEL_CALIBRATION_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_A_ACCEL_CALIBRATION_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.A_ACCEL_CALIBRATION_RESPONSE)

    def test_053_set_wr_accel_calibration(self):
        print("Test 053 - Set WR Accel Calibration Command")
        tx_bytes = [0x03, 0x00, 0x03, 0x00, 0x01, 0x11, 0x00, 0x00, 0x02, 0x00, 0x00, 0x15, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08]  # default off
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_ACCEL_CALIBRATION_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_ACCEL_CALIBRATION_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.ACCEL_CALIBRATION_RESPONSE)

    def test_054_set_gyro_calibration(self):
        print("Test 054 - Set Gyro Calibration Command ")
        tx_bytes = [0x03, 0x00, 0x03, 0x00, 0x01, 0x11, 0x00, 0x00, 0x02, 0x00, 0x00, 0x15, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08]
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_GYRO_CALIBRATION_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_GYRO_CALIBRATION_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.GYRO_CALIBRATION_RESPONSE)

    def test_055_set_mag_calibration(self):
        print("Test 055 - Set Mag Calibration Command")
        tx_bytes = [0x03, 0x00, 0x03, 0x00, 0x01, 0x11, 0x00, 0x00, 0x02, 0x00, 0x00, 0x15, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08]
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_MAG_CALIBRATION_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_MAG_CALIBRATION_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.MAG_CALIBRATION_RESPONSE)

    def test_056_set_mag_gain(self):
        print("Test 056 - Set Mag gain command")
        tx_bytes = [0x02]  # Default = 1, "set_config_bytes" test sets it to 0, choosing 2 here
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_MAG_GAIN_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_MAG_GAIN_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.MAG_GAIN_RESPONSE)

    def test_057_set_gsr_range(self):
        print("Test 057- Set GSR Range Command")
        tx_bytes = [0x03]
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_GSR_RANGE_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_GSR_RANGE_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.GSR_RANGE_RESPONSE)

    def test_058_set_mag_sampling_rate(self):
        print("Test 058 - Set Mag Sampling rate command")
        tx_bytes = [0x01]
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_MAG_SAMPLING_RATE_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_MAG_SAMPLING_RATE_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.MAG_SAMPLING_RATE_RESPONSE)

    def test_059_set_accel_sampling_rate(self):
        print("Test 059 - Set Accel Sampling rate command")
        tx_bytes = [0x01]
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_ACCEL_SAMPLING_RATE_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_ACCEL_SAMPLING_RATE_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.ACCEL_SAMPLING_RATE_RESPONSE)

    def test_060_set_wr_accel_lpmode(self):
        print("Test 060 - Set wr lpmode command")
        tx_bytes = [0x00]
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_ACCEL_LPMODE_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_ACCEL_LPMODE_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.ACCEL_LPMODE_RESPONSE)

    def test_061_set_wr_accel_hrmode(self):
        print("Test 061 - Set wr accel hrmode command")
        tx_bytes = [0x01]
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_ACCEL_HRMODE_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_ACCEL_HRMODE_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.ACCEL_HRMODE_RESPONSE)

    def test_062_set_gyro_range(self):
        print("Test 062 - Gyro Range command")
        tx_bytes = [0x01]
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_GYRO_RANGE_COMMAND,
                                    tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_GYRO_RANGE_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.GYRO_RANGE_RESPONSE)

    def test_063_set_gyro_sampling_rate(self):
        print("Test 063 - Set Gyro Sampling Rate Command")
        tx_bytes = [0x02]  # default 1
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_GYRO_SAMPLING_RATE_COMMAND,
                                    tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_GYRO_SAMPLING_RATE_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.GYRO_SAMPLING_RATE_RESPONSE)

    def test_064_set_internal_exp_power_enable(self):
        print("Test 064 - Set Internal exp power enable command")
        tx_bytes = [0x00]  # Power on = 1, power off = 0 (default = 1)
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_INTERNAL_EXP_POWER_ENABLE_COMMAND,
                                    tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_INTERNAL_EXP_POWER_ENABLE_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.INTERNAL_EXP_POWER_ENABLE_RESPONSE)

    def test_065_set_exg_regs_response(self):
        print("Test 065 - set_exg_regs_response")
        # Chip 0
        # tx_bytes = [0x00, 0x80, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x01]  # default = ExG off
        tx_bytes = [0x03, 0xAB, 0x10, 0x15, 0x15, 0x00, 0x00, 0x00, 0x02, 0x01]  # Square wave @ 512 Hz
        self.bt_cmd_test_set_common([shimmer_comms_bluetooth.BtCmds.SET_EXG_REGS_COMMAND, 0, 0, 10], tx_bytes,
                                    [shimmer_comms_bluetooth.BtCmds.GET_EXG_REGS_COMMAND, 0, 0, 10],
                                    shimmer_comms_bluetooth.BtCmds.EXG_REGS_RESPONSE)
        # Chip 1
        # tx_bytes = [0x00, 0x80, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x01]  # default = ExG off
        tx_bytes = [0x03, 0xA3, 0x10, 0x15, 0x15, 0x00, 0x00, 0x00, 0x02, 0x01]  # Square wave @ 512 Hz
        self.bt_cmd_test_set_common([shimmer_comms_bluetooth.BtCmds.SET_EXG_REGS_COMMAND, 1, 0, 10], tx_bytes,
                                    [shimmer_comms_bluetooth.BtCmds.GET_EXG_REGS_COMMAND, 1, 0, 10],
                                    shimmer_comms_bluetooth.BtCmds.EXG_REGS_RESPONSE)

    def test_066_set_daughter_card_id(self):
        print("Test 066 - Set daughter card ID")

        if not self.shimmer.is_expansion_board_set():
            self.test_06_get_daughter_card_id(True)

        # Not going to risk changing the daughter card ID here so just going to try and write the same one back
        tx_bytes = [self.shimmer.daughter_card_id, self.shimmer.daughter_card_rev_major, self.shimmer.daughter_card_rev_minor]
        self.bt_cmd_test_set_common([shimmer_comms_bluetooth.BtCmds.SET_DAUGHTER_CARD_ID_COMMAND, 0x03, 0x00],
                                    tx_bytes,
                                    [shimmer_comms_bluetooth.BtCmds.GET_DAUGHTER_CARD_ID_COMMAND, 0x03, 0x00],
                                    shimmer_comms_bluetooth.BtCmds.DAUGHTER_CARD_ID_RESPONSE,
                                    check_original_value=False)

    def test_067_set_baud_rate(self):
        print("Test 067 - Set baud Rate Command")
        # NOTE Changing of BT baud rate no longer supported in firmware. The firmware now sets the max supported baud
        # automatically.

        # tx_bytes = [0x05]  # default 0 = 11.5.2k
        # self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_BT_COMMS_BAUD_RATE, tx_bytes,
        #                             shimmer_comms_bluetooth.BtCmds.GET_BT_COMMS_BAUD_RATE,
        #                             shimmer_comms_bluetooth.BtCmds.BT_COMMS_BAUD_RATE_RESPONSE)

    def test_068_set_derived_channel_bytes(self):
        print("Test 068 - Set Derived Channel bytes Command")
        tx_bytes = [0x00, 0x01, 0x02, 0x00, 0x00, 0x06, 0x05, 0x04]  # default [0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00]
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_DERIVED_CHANNEL_BYTES, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_DERIVED_CHANNEL_BYTES,
                                    shimmer_comms_bluetooth.BtCmds.DERIVED_CHANNEL_BYTES_RESPONSE)

    def test_069_set_trial_config(self):
        print("Test 069 - Set Trial Config Command")
        tx_bytes = [0x00, 0x01, 0x37]  # default = 0x31 0x00 0x36
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_TRIAL_CONFIG_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_TRIAL_CONFIG_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.TRIAL_CONFIG_RESPONSE)

    def test_070_set_center(self):
        print("Test 070 - Set Center Command")
        # TODO SET_CENTER_COMMAND not currently supported in LogAndStream but could be added in the future
        print("Skipping as not supported yet in LogAndStream")
        # tx_bytes = [0x01]
        # self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_CENTER_COMMAND, tx_bytes,
        #                             shimmer_comms_bluetooth.BtCmds.GET_CENTER_COMMAND,
        #                             shimmer_comms_bluetooth.BtCmds.CENTER_RESPONSE)

    def test_071_set_shimmerName(self):
        print("Test 071 - Set Shimmer Name Command ")
        shimmer_name = "UnitTest71"
        tx_bytes = [ord(c) for c in shimmer_name]
        tx_bytes = [len(tx_bytes)] + tx_bytes

        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_SHIMMERNAME_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_SHIMMERNAME_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.SHIMMERNAME_RESPONSE)

    def test_072_set_ExpID(self):
        print("Test 072 - Set ExpId command")
        experiment_id = "UnitTest72"
        tx_bytes = [ord(c) for c in experiment_id]
        tx_bytes = [len(tx_bytes)] + tx_bytes

        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_EXPID_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_EXPID_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.EXPID_RESPONSE)

    def test_073_set_myID(self):
        print("Test 073 - Set My ID command")
        tx_bytes = [0x01]
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_MYID_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_MYID_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.MYID_RESPONSE)

    def test_074_set_NShimmer(self):
        print("Test 074 - Set nShimmer command")
        tx_bytes = [0x01]
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_NSHIMMER_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_NSHIMMER_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.NSHIMMER_RESPONSE)

    def test_075_set_ConfigTime(self):
        print("Test 075 - Set ConfigTime Command")
        test_config_time_s = "832103100"  # Tuesday, 14 May 1996 19:45:00
        tx_bytes = [ord(c) for c in test_config_time_s]
        tx_bytes = [len(tx_bytes)] + tx_bytes
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_CONFIGTIME_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_CONFIGTIME_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.CONFIGTIME_RESPONSE)

    def test_076_set_InfoMem(self):
        print("Test 076 - Set InfoMem Command")
        # self.test_02_reset_default_config()
        # self.test_03_reset_default_calib()

        #  Config generated below from Consensys in-which:
        #  Trial name = UnitTests
        #  Sampling rate = 512 Hz
        #  Sensors Enabled = wr-accel, pressure & temperature and battery voltage

        tx_bytes = [0x40, 0x00, 0x01, 0x00, 0x30, 0x04, 0x71, 0xFF, 0x01, 0x08, 0x00, 0x80, 0x10, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x02, 0x01, 0x00, 0x80, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x01, 0x0B, 0x00,
                    0x00, 0x00, 0x08, 0xCD, 0x08, 0xCD, 0x08, 0xCD, 0x00, 0x5C, 0x00, 0x5C, 0x00, 0x5C, 0x00, 0x9C,
                    0x00, 0x9C, 0x00, 0x00, 0x00, 0x00, 0x9C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x96, 0x19,
                    0x96, 0x19, 0x96, 0x00, 0x9C, 0x00, 0x9C, 0x00, 0x00, 0x00, 0x00, 0x9C, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x02, 0x9B, 0x02, 0x9B, 0x02, 0x9B, 0x00, 0x9C, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00,
                    0x9C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x87, 0x06, 0x87, 0x06, 0x87, 0x00, 0x9C, 0x00,
                    0x64, 0x00, 0x00, 0x00, 0x00, 0x9C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x53, 0x68, 0x69, 0x6D, 0x6D,
                    0x65, 0x72, 0x5F, 0x41, 0x36, 0x31, 0x35, 0x55, 0x6E, 0x69, 0x74, 0x54, 0x65, 0x73, 0x74, 0x73,
                    0xFF, 0xFF, 0xFF, 0x66, 0x79, 0x2D, 0x6D, 0x00, 0x00, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_INFOMEM_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_INFOMEM_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.INFOMEM_RESPONSE, 2)

    def test_077_set_calib_dump(self):
        print("Test 077 - Set Calib Command")
        tx_bytes = [0x52, 0x01,  # Calib byte length
                    0x03, 0x00,  # Device version
                    0x02, 0x00, 0x00,  # Firmware identifier
                    0x00, 0x16, 0x04,  # FW version

                    # 0x02 = SC_SENSOR_ANALOG_ACCEL
                    0x02, 0x00, 0x00, 0x15,  # 1 byte ID, 2 bytes Range, 1 byte data length
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  # Calib timestamp
                    0x08, 0xCD, 0x08, 0xCD, 0x08, 0xCD, 0x00, 0x5C, 0x00, 0x5C, 0x00, 0x5C, 0x00, 0x9C, 0x00, 0x9C,
                    0x00, 0x00, 0x00, 0x00, 0x9C,  # Calibration bytes

                    # 0x1E = SC_SENSOR_GYRO
                    0x1E, 0x00, 0x00, 0x15,  # 1 byte ID, 2 bytes Range, 1 byte data length
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  # Calib timestamp
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x33, 0x2C, 0x33, 0x2C, 0x33, 0x2C, 0x00, 0x9C, 0x00, 0x9C,
                    0x00, 0x00, 0x00, 0x00, 0x9C,  # Calibration bytes
                    0x1E, 0x00, 0x01, 0x15,  # 1 byte ID, 2 bytes Range, 1 byte data length
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  # Calib timestamp
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x96, 0x19, 0x96, 0x19, 0x96, 0x00, 0x9C, 0x00, 0x9C,
                    0x00, 0x00, 0x00, 0x00, 0x9C,  # Calibration bytes
                    0x1E, 0x00, 0x02, 0x15,  # 1 byte ID, 2 bytes Range, 1 byte data length
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  # Calib timestamp
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0xD0, 0x0C, 0xD0, 0x0C, 0xD0, 0x00, 0x9C, 0x00, 0x9C,
                    0x00, 0x00, 0x00, 0x00, 0x9C,  # Calibration bytes
                    0x1E, 0x00, 0x03, 0x15,  # 1 byte ID, 2 bytes Range, 1 byte data length
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  # Calib timestamp
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x68, 0x06, 0x68, 0x06, 0x68, 0x00, 0x9C, 0x00, 0x9C,
                    0x00, 0x00, 0x00, 0x00, 0x9C,  # Calibration bytes

                    # 0x1F = SC_SENSOR_WR_ACCEL
                    0x1F, 0x00, 0x00, 0x15,  # 1 byte ID, 2 bytes Range, 1 byte data length
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  # Calib timestamp
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x87, 0x06, 0x87, 0x06, 0x87, 0x00, 0x9C, 0x00, 0x64,
                    0x00, 0x00, 0x00, 0x00, 0x9C,  # Calibration bytes
                    0x1F, 0x00, 0x01, 0x15,  # 1 byte ID, 2 bytes Range, 1 byte data length
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  # Calib timestamp
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD1, 0x00, 0xD1, 0x00, 0xD1, 0x00, 0x9C, 0x00, 0x64,
                    0x00, 0x00, 0x00, 0x00, 0x9C,  # Calibration bytes
                    0x1F, 0x00, 0x02, 0x15,  # 1 byte ID, 2 bytes Range, 1 byte data length
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  # Calib timestamp
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x44, 0x03, 0x44, 0x03, 0x44, 0x00, 0x9C, 0x00, 0x64,
                    0x00, 0x00, 0x00, 0x00, 0x9C,  # Calibration bytes
                    0x1F, 0x00, 0x03, 0x15,  # 1 byte ID, 2 bytes Range, 1 byte data length
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  # Calib timestamp
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xA2, 0x01, 0xA2, 0x01, 0xA2, 0x00, 0x9C, 0x00, 0x64,
                    0x00, 0x00, 0x00, 0x00, 0x9C,  # Calibration bytes

                    # 0x20 = SC_SENSOR_WR_MAG
                    0x20, 0x00, 0x00, 0x15,  # 1 byte ID, 2 bytes Range, 1 byte data length
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  # Calib timestamp
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x9B, 0x02, 0x9B, 0x02, 0x9B, 0x00, 0x9C, 0x00, 0x64,
                    0x00, 0x00, 0x00, 0x00, 0x9C]  # Calibration bytes

        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_CALIB_DUMP_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_CALIB_DUMP_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.RSP_CALIB_DUMP_COMMAND, 2)

    def test_078_set_crc(self):
        print("Test 078 - Set CRC Command")

        if not self.shimmer.is_hardware_version_set():
            self.test_04_get_shimmer_new_version(True)

        num_inquiry_bytes = 11 if self.shimmer.is_hardware_shimmer3r() else 8

        self.shimmer.bt_crc_byte_count = 1  # 1 = 1 byte CRC, 2 = 2 bytes CRC (default = 0)
        self.shimmer.bluetooth_port.send_bluetooth(
            [shimmer_comms_bluetooth.BtCmds.SET_CRC_COMMAND, self.shimmer.bt_crc_byte_count])
        self.bt_cmd_test_wait_for_ack()

        # Using inquiry command here to test whether CRC has been enabled
        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.INQUIRY_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.INQUIRY_RESPONSE, num_inquiry_bytes)

        self.shimmer.bt_crc_byte_count = 0  # 1 = 1 byte CRC, 2 = 2 bytes CRC (default = 0)
        self.shimmer.bluetooth_port.send_bluetooth(
            [shimmer_comms_bluetooth.BtCmds.SET_CRC_COMMAND, self.shimmer.bt_crc_byte_count])
        self.bt_cmd_test_wait_for_ack()

        # Make sure CRC gets disabled
        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.INQUIRY_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.INQUIRY_RESPONSE, num_inquiry_bytes)

    def test_079_set_rwc(self):
        print("Test 079 - Set RWC command")
        ts_ms = time.time()
        tx_bytes = util_shimmer_time.ms_to_shimmer_rtc_bytes(ts_ms)
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_RWC_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_RWC_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.RWC_RESPONSE)

    def test_080_update_calib_dump(self):  # not present in set
        print("Test 080 - UPD Calib Dump Command")
        self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.UPD_CALIB_DUMP_COMMAND])
        self.bt_cmd_test_wait_for_ack()

    def test_081_update_sdlog_cfg(self):
        print("Test 081 - UPD SDlog Cfg Command")
        self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.UPD_SDLOG_CFG_COMMAND])
        self.bt_cmd_test_wait_for_ack()

    # TODO set charge status not supported in LogAndStream
    # def test_82_set_charge_status(self):
    #     print("Test 82 - Set charge status LED")
    #
    #     tx_bytes = [0x01]
    #     self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_CHARGE_STATUS_LED_COMMAND, tx_bytes,
    #                                 shimmer_comms_bluetooth.BtCmds.GET_CHARGE_STATUS_LED_COMMAND,
    #                                 shimmer_comms_bluetooth.BtCmds.CHARGE_STATUS_LED_RESPONSE)

    def test_083_set_alt_accel_range(self):
        print("Test 083 - Set alt accel")
        tx_bytes = [0x01]
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_ALT_ACCEL_RANGE_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_ALT_ACCEL_RANGE_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.ALT_ACCEL_RANGE_RESPONSE)

    def test_084_set_pres_oversampling_ratio(self):
        print("Test 084 - Set pres oversampling ratio")
        tx_bytes = [0x01]
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_PRES_OVERSAMPLING_RATIO_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_PRES_OVERSAMPLING_RATIO_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.PRES_OVERSAMPLING_RATIO_RESPONSE)

    def test_085_set_daughter_card_mem(self):
        print("Test 085 - Set daughter card mem")

        shimmer_name = "Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua."
        tx_bytes = [ord(c) for c in shimmer_name]

        self.bt_cmd_test_set_common([shimmer_comms_bluetooth.BtCmds.SET_DAUGHTER_CARD_MEM_COMMAND, len(tx_bytes), 0, 0],
                                    tx_bytes,
                                    [shimmer_comms_bluetooth.BtCmds.GET_DAUGHTER_CARD_MEM_COMMAND, len(tx_bytes), 0, 0],
                                    shimmer_comms_bluetooth.BtCmds.DAUGHTER_CARD_MEM_RESPONSE)

        # Reset the EEPROM memory
        tx_bytes = [0xFF] * len(shimmer_name)
        self.bt_cmd_test_set_common([shimmer_comms_bluetooth.BtCmds.SET_DAUGHTER_CARD_MEM_COMMAND, len(tx_bytes), 0, 0],
                                    tx_bytes,
                                    [shimmer_comms_bluetooth.BtCmds.GET_DAUGHTER_CARD_MEM_COMMAND, len(tx_bytes), 0, 0],
                                    shimmer_comms_bluetooth.BtCmds.DAUGHTER_CARD_MEM_RESPONSE)

    # TODO decide what to do about this command
    # def test_86_start_streaming_and_logging(self):
    #     print("Test 86 - streaming and logging")
    #     self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.START_SDBT_COMMAND])
    #     self.bt_cmd_test_wait_for_ack()

    # TODO decide what to do about this command
    # def test_87_stop_streaming_and_logging(self):
    #     print("Test 87 - streaming and logging")
    #     self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.STOP_SDBT_COMMAND])
    #     self.bt_cmd_test_wait_for_ack()

    # TODO decide what to do about this command
    # def test_88_set_data_rate_test(self):
    #     print("Test 88 - Set data rate test")
    #     self.assertTrue(False, "Test not implemented yet")

    def test_089_set_instream_response(self):
        print(" Test 089: Instream response ack")
        tx_bytes = [0x00]  # default 1
        self.shimmer.bluetooth_port.send_bluetooth(
                                [shimmer_comms_bluetooth.BtCmds.SET_INSTREAM_RESPONSE_ACK_PREFIX_STATE] + tx_bytes)
        self.bt_cmd_test_wait_for_ack()

    def test_090_set_sd_sync_response(self):
        print("test 090: Set SD Sync")

        ts_ms = time.time()
        tx_bytes = util_shimmer_time.ms_to_shimmer_rtc_bytes(ts_ms)
        tx_bytes = [0x00] + tx_bytes + [0x00]  # first byte is to mock whether sensor isSensing, last byte mocks CRC
        self.shimmer.bluetooth_port.send_bluetooth(
                                [shimmer_comms_bluetooth.BtCmds.SET_SD_SYNC_COMMAND] + tx_bytes)
        self.bt_cmd_test_wait_for_ack()

    # TODO decide what to do about this command
    # def test_91_start_logging(self):
    #     print("Test 91 - Start logging")
    #     self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.START_LOGGING_COMMAND])
    #     self.bt_cmd_test_wait_for_ack()

    # TODO decide what to do about this command
    # def test_92_stop_logging(self):
    #     print("Test 92 - Stop logging")
    #     self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.STOP_LOGGING_COMMAND])
    #     self.bt_cmd_test_wait_for_ack()

    def test_093_toggle_LED(self):
        print("Test 093 - toggle LED")

        self.test_028_get_status(True)
        red_led_state_before = self.shimmer.status_toggle_led_red

        self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.TOGGLE_LED_COMMAND])
        self.bt_cmd_test_wait_for_ack()

        self.test_028_get_status(True)

        red_led_state_after = self.shimmer.status_toggle_led_red
        self.assertTrue(red_led_state_before != red_led_state_after, "LED state didn't change in status")

        # Turn LED off at end of test
        if self.shimmer.status_toggle_led_red:
            self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.TOGGLE_LED_COMMAND])
            self.bt_cmd_test_wait_for_ack()

    def test_094_dummy_command(self):
        print("Test 094 - DummyCommand")
        self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.DUMMY_COMMAND])
        self.bt_cmd_test_wait_for_ack()

    # TODO decide what to do about this command
    # def test_95_Start_Streaming(self):
    #     print("Test 95 - Start Streaming")
    #     self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.START_STREAMING_COMMAND])
    #     self.bt_cmd_test_wait_for_ack()

    # TODO decide what to do about this command
    # def test_96_Stop_Streaming(self):
    #     print("Test 95 - Stop Streaming")
    #     self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.STOP_STREAMING_COMMAND])
    #     self.bt_cmd_test_wait_for_ack()

    def test_095_get_Pressure_Calibration_coefficients(self):
        print("Test 095 - Get Common Pressure Calibration Chip")
        if not self.shimmer.is_hardware_version_set():
            self.test_04_get_shimmer_new_version(True)
        if not self.shimmer.is_firmware_version_set():
            self.test_05_get_fw_version(True)

        if self.shimmer.is_bt_cmd_common_pressure_calibration_supported():
            response = self.bt_cmd_test_get_common(
                shimmer_comms_bluetooth.BtCmds.GET_PRESSURE_CALIBRATION_COEFFICIENTS_COMMAND,
                shimmer_comms_bluetooth.BtCmds.PRESSURE_CALIBRATION_COEFFICIENTS_RESPONSE, 1)
        else:
            print("Skipping test, command not supported in firmware")

    def test_096_get_alt_accel_calibration_command(self):
        print("Test 096 - get alt accel calibration command ")
        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.GET_ALT_ACCEL_CALIBRATION_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.ALT_ACCEL_CALIBRATION_RESPONSE, 1)

    def test_097_get_alt_accel_sampling_rate_command(self):
        print("Test 097 - Get Alt Accel Sampling Command ")
        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.GET_ALT_ACCEL_SAMPLING_RATE_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.ALT_ACCEL_SAMPLING_RATE_RESPONSE, 1)

    def test_098_get_alt_mag_calibration_command(self):
        print("Test 098 - Get Alt Mag Calibration Command ")
        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.GET_ALT_MAG_CALIBRATION_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.MAG_CALIBRATION_RESPONSE, 1)

    def test_099_get_alt_mag_sampling_rate_command(self):
        print("Test 099 - Get Alt Mag Sampling Rate Command")
        response = self.bt_cmd_test_get_common(shimmer_comms_bluetooth.BtCmds.GET_MAG_CALIBRATION_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.MAG_CALIBRATION_RESPONSE, 1)

    def test_100_set_alt_accel_calibration_command(self):
        tx_bytes = [0x00]
        print("Test 101 - Set alt Accel calibration")
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_ALT_ACCEL_CALIBRATION_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_ALT_ACCEL_CALIBRATION_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.ALT_ACCEL_CALIBRATION_RESPONSE)

    def test_101_set_alt_accel_sampling_rate_command(self):
        print("Test 102 - Set alt Accel Sampling rate")
        tx_bytes = [0x00]
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_ALT_ACCEL_SAMPLING_RATE_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_ALT_ACCEL_SAMPLING_RATE_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.ALT_ACCEL_SAMPLING_RATE_RESPONSE)

    def test_102_set_alt_mag_sampling_rate_command(self):
        print("Test 103 - Set Alt mag sampling rate command")
        tx_bytes = [0x03]
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_ALT_MAG_SAMPLING_RATE_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_ALT_MAG_SAMPLING_RATE_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.ALT_MAG_SAMPLING_RATE_RESPONSE)

    def test_103_set_alt_mag_calibration_command(self):
        print("Test 104 - Set Alt Mag Calibration Command")
        tx_bytes = [0x03]
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_ALT_MAG_CALIBRATION_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_ALT_MAG_CALIBRATION_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.ALT_MAG_CALIBRATION_RESPONSE)

    def test_104_factory_test_bluetooth(self):
        print(Fore.LIGHTMAGENTA_EX + "Factory Test Start")
        tx_bytes = 0
        self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.SET_FACTORY_TEST, tx_bytes])
        self.shimmer.bluetooth_port.wait_for_ack(2000)
        end = "//***************************** TEST END *************************************//\r\n"
        while True:
            response = self.shimmer.bluetooth_port.ser.readline().decode('cp1252')
            print(response, end='')
            if response == end:
                print(Fore.LIGHTMAGENTA_EX + "Factory Test End")
                break

    def test_105_reset_config_and_calib_after_testing(self):
        print("\r\nResetting Shimmer's config and calibration\r\n")
        self.test_002_reset_default_config(True)
        self.test_003_reset_default_calib(True)


if __name__ == '__main__':
    unittest.main()
