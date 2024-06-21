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
        if tx_cmd_byte == shimmer_comms_bluetooth.BtCmds.GET_DAUGHTER_CARD_ID_COMMAND:
            self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.GET_DAUGHTER_CARD_ID_COMMAND,
                                                         0x03, 0x00])
        elif tx_cmd_byte == shimmer_comms_bluetooth.BtCmds.GET_INFOMEM_COMMAND:
            self.shimmer.bluetooth_port.send_bluetooth([0x8E, 0x80, 0x00, 0x00])
        elif tx_cmd_byte == shimmer_comms_bluetooth.BtCmds.GET_EXG_REGS_COMMAND:
            self.shimmer.bluetooth_port.send_bluetooth([0x63, 0x0B])
        else:
                if not self.shimmer.bluetooth_port.send_bluetooth([tx_cmd_byte]):
                    self.assertTrue(False, "Error writing command")

        time.sleep(delay_to_wait_for_response_s)

        response = self.shimmer.bluetooth_port.wait_for_response(2 + expected_response_len)
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

    def bt_cmd_test_set_common(self, set_cmd, set_bytes, get_cmd, response_cmd):
        self.shimmer.bluetooth_port.send_bluetooth([set_cmd] + set_bytes)
        self.bt_cmd_test_wait_for_ack(2000)
        response = self.bt_cmd_test_common(get_cmd, response_cmd, len(set_bytes))

        for i in range(0, len(set_bytes)):
            if set_bytes[i] is not response[i]:
                self.assertTrue(False, "rx byte != tx byte")

    def bt_cmd_test_wait_for_ack(self, timeout_ms=500):
        result = self.shimmer.bluetooth_port.wait_for_ack(timeout_ms)
        if not result:
            print("Error")
            self.assertTrue(False)

    def setUp(self):
        print("")
        self.shimmer.bluetooth_port.clear_serial_buffer()
        # time.sleep(0.1)

    def test_01_get_inquiry_response(self):
        print("\r\nTest 01 - Inquiry command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.INQUIRY_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.INQUIRY_RESPONSE, 8)

    def test_02_reset_default_config(self):
        print("Test 02 - Reset Default config:")
        self.shimmer.bluetooth_port.send_bluetooth(
                                    [shimmer_comms_bluetooth.BtCmds.RESET_TO_DEFAULT_CONFIGURATION_COMMAND])
        self.bt_cmd_test_wait_for_ack()

    def test_03_reset_default_calib(self):
        print("Test 03 - Reset Default calib:")
        self.shimmer.bluetooth_port.send_bluetooth(
            [shimmer_comms_bluetooth.BtCmds.RESET_CALIBRATION_VALUE_COMMAND])
        self.bt_cmd_test_wait_for_ack()

    def test_02_get_sampling_rate(self):
        print("Test 02 - Get sampling rate command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_SAMPLING_RATE_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.SAMPLING_RATE_RESPONSE, 2)

    def test_03_get_accel_sensitivity(self):
        print("Test 03 - Get accelerometer sensitivity command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_ACCEL_RANGE_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.ACCEL_RANGE_RESPONSE, 1)

    def test_93_set_calibration(self):
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

        if self.shimmer.bluetooth_port.write_calibration(calib_bytes):
            print("Error, exiting")
            self.assertTrue(False)

        calib_bytes = self.shimmer.bluetooth_port.read_calibration()
        if isinstance(calib_bytes, bool):
            self.assertTrue(False)

        print("")

    def test_04_get_config_setup_bytes(self):
        print("Test 04- Get Config Setup Bytes command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_CONFIG_SETUP_BYTES_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.CONFIG_SETUP_BYTES_RESPONSE, 2)

    def test_05_get_accel_calibration_command(self):
        print("Test 05 - Get Accel Calibration Command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_A_ACCEL_CALIBRATION_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.A_ACCEL_CALIBRATION_RESPONSE, 21)

    def test_06_get_gyro_calibration_command(self):
        print("Test 06 - Get Gyro Calibration Command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_GYRO_CALIBRATION_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.GYRO_CALIBRATION_RESPONSE, 21)

    def test_07_get_mag_calibration_command(self):
        print("Test 07 - Get Mag Calibration Command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_MAG_CALIBRATION_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.MAG_CALIBRATION_RESPONSE, 21)

    def test_09_get_gsr_range_command(self):
        print("Test 08 - Get GSR Command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_GSR_RANGE_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.GSR_RANGE_RESPONSE, 1)

    def test_11_all_calibration_response(self):
        print("Test 11 - Get all calibration response command")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_ALL_CALIBRATION_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.ALL_CALIBRATION_RESPONSE, 21)

    def test_12_FW_Version_response(self):
        print("Test 12 - Get FW response command")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_FW_VERSION_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.FW_VERSION_RESPONSE, 6)

    def test_14_get_buffer_size_command(self):
        print("Test 14 - Buffer size repsonse Command: ")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_BUFFER_SIZE_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.BUFFER_SIZE_RESPONSE,
                                           1)  # get buffer size or response as 2 commands are mentioned

    def test_15_get_mag_gain_command(self):
        print("Test 15 - Get mag gain command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_MAG_GAIN_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.MAG_GAIN_RESPONSE, 1)

    def test_16_get_charge_status_led_response(self):
        print("Test 16 - Get Charge Status LED response command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_CHARGE_STATUS_LED_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.CHARGE_STATUS_LED_RESPONSE, 3)

    def test_17_get_shimmer_version_command_response(self):
        print("Test 17 - Get Shimmer Version response command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_SHIMMERNAME_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.SHIMMERNAME_RESPONSE, 1)

    def test_18_get_shimmer_new_version_command_response(self):
        print("Test 18 - Get new Shimmer version response command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_NSHIMMER_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.NSHIMMER_RESPONSE, 1)

    def test_20_get_mag_sampling_command(self):
        print("Test 20 - Get Mag sampling Command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_MAG_SAMPLING_RATE_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.MAG_SAMPLING_RATE_RESPONSE, 1)

    def test_21_get_accel_sampling_rate_command(self):
        print("Test 21 - Get accel sampling rate response command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_ACCEL_SAMPLING_RATE_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.ACCEL_SAMPLING_RATE_RESPONSE, 1)

    def test_22_get_wr_accel_lpmode_command(self):
        print("Test 22 - Get WR accel lpmode command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_ACCEL_LPMODE_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.ACCEL_LPMODE_RESPONSE, 1)

    def test_23_get_wr_accel_hrmode_command(self):
        print("Test 23 - GET WR accel hrmode command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_ACCEL_HRMODE_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.ACCEL_HRMODE_RESPONSE, 1)

    def test_24_get_gyro_range_command(self):
        print("Test 24 - Get gyro range command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_GYRO_RANGE_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.GYRO_RANGE_RESPONSE, 1)

    def test_25_get_gyro_sampling_rate_command(self):
        print("Test 25 - Get gyro sampling rate command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_SAMPLING_RATE_COMMAND,  # works
                                           shimmer_comms_bluetooth.BtCmds.SAMPLING_RATE_RESPONSE, 2)

    # def test_26_get_BMP180_Pres_Resolution_Command(self):
    #     print("Test 25 - Get BMP180 Pres Resolution command:")
    #     response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_SAMPLING_RATE_COMMAND,
    #                                        shimmer_comms_bluetooth.BtCmds.SAMPLING_RATE_RESPONSE, 2)
    # not in Shimmer Bluetooth, java
    # def test_27_get_BMP180_Pres_Calibration_command(self):
    #     print("Test 25 - Get MPU9150 Sampling rate command:")
    #     response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_SAMPLING_RATE_COMMAND,
    #                                        shimmer_comms_bluetooth.BtCmds.SAMPLING_RATE_RESPONSE, 2)

    def test_28_get_BMPX80_Calibration_coefficients_command(self):

        # TODO distinguish between BMP280 and BMP180

        print("Test 29 - Get BMP180 Calibration response command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_BMP180_CALIBRATION_COEFFICIENTS_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.BMP180_CALIBRATION_COEFFICIENTS_RESPONSE, 22)

        print("Test 28 - Get BMP280 Caibration coefficient response command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_BMP280_CALIBRATION_COEFFICIENTS_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.BMP280_CALIBRATION_COEFFICIENTS_RESPONSE, 24)

    # def test_28_get_BMPX80_OVERSAMPLING_ratio_response(self):
    #
    #     # TODO distinguish between BMP280 and BMP180
    # not present in shimmer_comms_bluetooth_py
    #     print("Test 29 - Get BMP180 Calibration response command:")
    #     response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_B,
    #                                        shimmer_comms_bluetooth.BtCmds.BMP180_CALIBRATION_COEFFICIENTS_RESPONSE, 22)

    def test_30_get_alt_MAG_SENS_ADJ_VALS_RESPONSE(self):
        print("Test 31 - Get alternative Mag Sens Adj Vals response command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_ALT_MAG_SENS_ADJ_VALS_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.ALT_MAG_SENS_ADJ_VALS_RESPONSE, 1)

    def test_31_get_internal_exp_power_enable_command(self):
        print("Test 31 - Get exp power enable response command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_INTERNAL_EXP_POWER_ENABLE_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.INTERNAL_EXP_POWER_ENABLE_RESPONSE, 1)

    def test_32_get_exg_regs_command(self):
        print("Test 32 - Get ExG Regs command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_EXG_REGS_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.EXG_REGS_RESPONSE, 11)

    def test_33_get_daughter_card_id_command(self):
        print("Test 33 - Get daughter card id command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_DAUGHTER_CARD_ID_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.DAUGHTER_CARD_ID_RESPONSE, 3)

    def test_34_get_status(self):
        print("Test 34 - Get status command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_STATUS_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.STATUS_RESPONSE, 2, is_instream_response=True)

        status = response[0]
        self_cmd = (status & 0x04) >> 2
        sensing = (status & 0x02) >> 1
        docked = (status & 0x01)
        print("0x%02x , self: %d, sensing: %d, docked: %d" % (status, self_cmd, sensing, docked))

    def test_35_get_baud_rate_command(self):
        print("Test 34 - Get baud rate response command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_BT_COMMS_BAUD_RATE,
                                           shimmer_comms_bluetooth.BtCmds.BT_COMMS_BAUD_RATE_RESPONSE, 1)

    def test_36_get_derived_channel_bytes_command(self):
        print("Test 36 - Get Derived Channel Bytes response command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_DERIVED_CHANNEL_BYTES,
                                           shimmer_comms_bluetooth.BtCmds.DERIVED_CHANNEL_BYTES_RESPONSE, 8)

    def test_37_get_trial_config_command(self):
        print("Test 37 - Get trial config command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_TRIAL_CONFIG_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.TRIAL_CONFIG_RESPONSE, 3)

    def test_38_get_center_command(self):
         print("Test 38 - Get Center response command:")
         response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_CENTER_COMMAND,
                                            shimmer_comms_bluetooth.BtCmds.CENTER_RESPONSE, 1)

    def test_39_get_shimmer_name_command(self):
        print("Test 39 - Get ShimmerName command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_SHIMMERNAME_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.SHIMMERNAME_RESPONSE, 13)

    def test_40_get_expID_command(self):
        print("Test 40 - Get ExpID command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_EXPID_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.EXPID_RESPONSE, 12)

    def test_41_get_myID_command(self):
        print("Test 41 - Get myID command:")  # works
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_MYID_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.MYID_RESPONSE, 1)

    def test_42_NSHIMMER_command(self):
        print("Test 42 - Get nshimmer response command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_NSHIMMER_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.NSHIMMER_RESPONSE, 1)

    def test_43_get_ConfigTime_command(self):
        print("Test 43- Get Config Time Response Command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_CONFIGTIME_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.CONFIGTIME_RESPONSE, 1)

    def test_44_get_dir_command(self):
        print("Test 43 - Get dir response command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_DIR_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.DIR_RESPONSE, 4)

    def test_45_get_infomem_command(self):
        print("Test 45 - Get infomem response command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_INFOMEM_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.INFOMEM_RESPONSE, 128)

    def test_46_get_rwc_command(self):
        print("Test 46- Get RWC response command")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_RWC_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.RWC_RESPONSE, 1)

    def test_47_get_vbatt_command(self):
        print("Test 47 - Get VBatt response command")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_VBATT_COMMAND,
                                        shimmer_comms_bluetooth.BtCmds.VBATT_RESPONSE, 3, is_instream_response= True)
        batt_voltage = ((response[0] & 0xFF) << 8) + (response[0] & 0xFF)
        charging_status = response[1]
        print("batt_voltage: %03f , charging_status: %d" % (batt_voltage, charging_status))

    def test_48_get_BT_FW_VERSION_command(self):
        print("Test 48 - Get BT FW Version response command")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_BT_VERSION_STR_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.BT_VERSION_STR_RESPONSE, 2)

    # set commands

    # def test_49_set_sensors_command(self):
    #     print("Test 50 - set accel sensitivity command ")
    #     tx_bytes = [0x01]
    #     self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_SENSORS_COMMAND,
    #                                 tx_bytes,
    #                                 shimmer_comms_bluetooth.BtCmds.GET_SENSO,
    #                                 shimmer_comms_bluetooth.BtCmds.ACCEL_RANGE_RESPONSE)

    def test_50_set_accel_sensitivity_command(self):
        print("Test 50 - set accel sensitivity command ")
        tx_bytes = [0x01] # default 0
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_ACCEL_RANGE_COMMAND,
                                    tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_ACCEL_RANGE_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.ACCEL_RANGE_RESPONSE)

    def test_51_set_config_byte0_command(self):
        print("Test 51 - set Config Byte0 command ")
        tx_bytes = [0x06, 0x01, 0x02, 0x03] #default
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_CONFIG_SETUP_BYTES_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_CONFIG_SETUP_BYTES_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.CONFIG_SETUP_BYTES_RESPONSE)

    def test_52_set_A_accel_calibration_command(self):
        print("Test 52 - Set accel Calibration Command")
        tx_bytes = [0x03, 0x00, 0x03, 0x00, 0x01, 0x11, 0x00, 0x00, 0x02, 0x00, 0x00, 0x15, 0x00, 0x00,
                             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08] # default 0xff, 84
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_A_ACCEL_CALIBRATION_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_A_ACCEL_CALIBRATION_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.A_ACCEL_CALIBRATION_RESPONSE)

    def test_53_set_wr_accel_calibration_command(self):
        print("Test 53 - Set WR Accel Calibration Command")
        tx_bytes = [0x03, 0x00, 0x03, 0x00, 0x01, 0x11, 0x00, 0x00, 0x02, 0x00, 0x00, 0x15, 0x00, 0x00,
                             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08] # default off
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_ACCEL_CALIBRATION_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_ACCEL_CALIBRATION_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.ACCEL_CALIBRATION_RESPONSE)

    def test_54_set_gyro_calibration_command(self):
        print("Test 54 - Set Gyro Calibration Command ")
        tx_bytes = [0x03, 0x00, 0x03, 0x00, 0x01, 0x11, 0x00, 0x00, 0x02, 0x00, 0x00, 0x15, 0x00, 0x00,
                             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08]
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_GYRO_CALIBRATION_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_GYRO_CALIBRATION_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.GYRO_CALIBRATION_RESPONSE)

    def test_55_set_mag_calibration_command(self):
        print("Test 55 - Set Mag Calibration Command")
        tx_bytes = [0x03, 0x00, 0x03, 0x00, 0x01, 0x11, 0x00, 0x00, 0x02, 0x00, 0x00, 0x15, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08]
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_MAG_CALIBRATION_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_MAG_CALIBRATION_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.MAG_CALIBRATION_RESPONSE)

    def test_56_set_mag_gain_command(self):
        print("Test 56 - Set Mag gain command")
        tx_bytes = [0x01]
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_MAG_GAIN_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_MAG_GAIN_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.MAG_GAIN_RESPONSE)

    def test_57_set_gsr_range_command(self):
        print("Test 57- Set GSR Range Command")
        tx_bytes = [0x01]
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_GSR_RANGE_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_GSR_RANGE_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.GSR_RANGE_RESPONSE)

    def test_64_set_mag_sampling_rate_command(self):
        print("Test 64 - Set Mag Sampling rate command")
        tx_bytes = [0x01]
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_MAG_SAMPLING_RATE_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_MAG_SAMPLING_RATE_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.MAG_SAMPLING_RATE_RESPONSE)

    def test_65_set_accel_sampling_rate_command(self):
        print("Test 65 - Set Accel Sampling rate command")
        tx_bytes = [0x01]
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_ACCEL_SAMPLING_RATE_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_ACCEL_SAMPLING_RATE_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.ACCEL_SAMPLING_RATE_RESPONSE)

    def test_66_set_wr_accel_lpmode_command(self):
        print("Test 66 - Set wr lpmode command")
        tx_bytes = [0x01]
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_ACCEL_LPMODE_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_ACCEL_LPMODE_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.ACCEL_LPMODE_RESPONSE)

    def test_67_set_wr_accel_hrmode_command(self):
        print("Test 67 - Set wr accel hrmode command")
        tx_bytes = [0x01]
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_ACCEL_HRMODE_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_ACCEL_HRMODE_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.ACCEL_HRMODE_RESPONSE)

    def test_68_set_gyro_range_command(self):
        print("Test 68 - Gyro Range command")
        tx_bytes = [0x02] #default 0x03
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_GYRO_RANGE_COMMAND,
                                    tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_GYRO_RANGE_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.GYRO_RANGE_RESPONSE)

    def test_69_set_gyro_sampling_rate_command(self):
        print("Test 69 - Set Gyro Sampling Rate Command")
        tx_bytes = [0x01, 0x02, 0x03] #default 1
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_GYRO_SAMPLING_RATE_COMMAND,
                                    tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_GYRO_SAMPLING_RATE_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.GYRO_SAMPLING_RATE_RESPONSE)

    def test_75_set_internal_exp_power_enable_command(self):
        print("Test 75 - Set Internal exp power enable command")
        tx_bytes = [0x01]  # Power on = 1, power off = 0 (default = 0)
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_INTERNAL_EXP_POWER_ENABLE_COMMAND,tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_INTERNAL_EXP_POWER_ENABLE_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.INTERNAL_EXP_POWER_ENABLE_RESPONSE)

    # def test_76_set_exg_regs_response(self):
    #     tx_bytes = [0x01, 0x00, 0x03, 0x01, 0x04, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00]
    #     self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_EXG_REGS_COMMAND, tx_bytes,
    #                             shimmer_comms_bluetooth.BtCmds.GET_EXG_REGS_COMMAND,
    #                             shimmer_comms_bluetooth.BtCmds.EXG_REGS_RESPONSE)

    # no set daughter card id command - Test 77

    def test_78_set_baud_rate_command(self):
        print("Test 78 - Set baud Rate Command")
        tx_bytes = [0x05] #default 0 = 11.5.2k
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_BT_COMMS_BAUD_RATE, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_BT_COMMS_BAUD_RATE,
                                    shimmer_comms_bluetooth.BtCmds.BT_COMMS_BAUD_RATE_RESPONSE)

    def test_79_set_derived_channel_bytes_command(self):
        print("Test 79 - Set Derived Channel bytes Command")
        tx_bytes = [0x00, 0x01, 0x02]
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_DERIVED_CHANNEL_BYTES, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_DERIVED_CHANNEL_BYTES,
                                    shimmer_comms_bluetooth.BtCmds.DERIVED_CHANNEL_BYTES_RESPONSE)

    def test_80_set_trial_config_command(self):
        print("Test 80- Set Trial Config Command")
        tx_bytes = [0x00, 0x01, 0x02]
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_CONFIG_SETUP_BYTES_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_CONFIG_SETUP_BYTES_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.CONFIG_SETUP_BYTES_RESPONSE)

    def test_81_set_center_command(self):
        print("Test 81 - Set Center Command")
        tx_bytes = [0x01]
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_CENTER_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_CENTER_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.CENTER_RESPONSE)

    def test_82_set_shimmerName_command(self):
        print("Test 82 - Set Shimmer Name Command ")
        tx_bytes = [0x01]
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_SHIMMERNAME_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_SHIMMERNAME_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.SHIMMERNAME_RESPONSE)

    def test_83_set_ExpID_command(self):
        print("Test 83 - Set ExpId command")
        tx_bytes = [0x01]
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_EXPID_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_EXPID_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.EXPID_RESPONSE)

    def test_84_set_myID_command(self):
        print("Test 84 - Set My ID command")
        tx_bytes = [0x01]
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_MYID_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_MYID_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.MYID_RESPONSE)

    def test_85_set_NShimmer_command(self):
        print("Test 85- Set nShimmer command")
        tx_bytes = [0x01]
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_NSHIMMER_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_NSHIMMER_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.NSHIMMER_RESPONSE)

    def test_86_set_ConfigTime_command(self):
        print("Test 86 - Set ConfigTime Command")
        tx_bytes = [0x01]
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_CONFIGTIME_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_CONFIGTIME_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.CONFIGTIME_RESPONSE)

        # no set dir and set instream cmd response

    def test_87_set_InfoMem_command(self):
        print("Test 87 - Set InfoMem Command")
        tx_bytes = [0x01]
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_INFOMEM_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_INFOMEM_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.INFOMEM_RESPONSE)

    def test_88_set_calib_dump_command(self):
        print("Test 88 - Set Calib Command")
        # tx_bytes = [0x01]
        # self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_CALIB_DUMP_COMMAND, tx_bytes,
        #                             shimmer_comms_bluetooth.BtCmds.GET_CALIB_DUMP_COMMAND,
        #                             shimmer_comms_bluetooth.BtCmds.CALIB_DUMP) # no calib dump response

    def test_89_set_crc_command(self):
        print("Test 89 - Set CRC Command")
        # tx_bytes = [0x01]
        # self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_CRC_COMMAND, tx_bytes,
        #                             shimmer_comms_bluetooth.BtCmds.GET_CRC,
        #                             shimmer_comms_bluetooth.BtCmds.CRC) #no get crc and crc response #no get crc

    def test_90_set_rwc_command(self):
        print("Test 90 - Set RWC command")
        tx_bytes = [0x01]
        self.bt_cmd_test_set_common(shimmer_comms_bluetooth.BtCmds.SET_RWC_COMMAND, tx_bytes,
                                    shimmer_comms_bluetooth.BtCmds.GET_RWC_COMMAND,
                                    shimmer_comms_bluetooth.BtCmds.RWC_RESPONSE)

    def test_91_UPD_calib_dump_command(self):  # not present in set
        print("Test 91 - UPD Calib Dump Command")
        self.shimmer.bluetooth_port.send_bluetooth(shimmer_comms_bluetooth.BtCmds.UPD_CALIB_DUMP_COMMAND)


    def test_92_UPD_SDlog_Cfg_command(self):
        print("Test 92 - UPD SDlog Cfg Command")
        self.shimmer.bluetooth_port.send_bluetooth(shimmer_comms_bluetooth.BtCmds.UPD_SDLOG_CFG_COMMAND)



if __name__ == '__main__':
    unittest.main()
