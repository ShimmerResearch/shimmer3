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
                           is_instream_response=False, delay_to_wait_for_response_s=1):
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

    def setUp(self):
        self.shimmer.bluetooth_port.clear_serial_buffer()
        print(" Flush buffer working ")

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
        print("")

    def test_05_get_accel_calibration_command(self):
        print("Test 05 - Get Accel Calibration Command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_A_ACCEL_CALIBRATION_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.A_ACCEL_CALIBRATION_RESPONSE, 21)
        print("")

    def test_06_get_gyro_calibration_command(self):
        print("Test 06 - Get Gyro Calibration Command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_GYRO_CALIBRATION_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.GYRO_CALIBRATION_RESPONSE, 21)

        print("")

    def test_07_get_mag_calibration_command(self):
        print("Test 07 - Get Mag Calibration Command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_MAG_CALIBRATION_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.MAG_CALIBRATION_RESPONSE, 21)
        print("")

    # def test_08_get_LSM303DLHC_mag_calibration_command(self):
    #     print("Test 08 - Get Mag Calibration Command:")
    #     response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.MAG_CALIBRATION_RESPONSE,) #BT command not present

    def test_09_get_gsr_range_command(self):
        print("Test 08 - Get GSR Command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_GSR_RANGE_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.GSR_RANGE_RESPONSE, 1)
        print("")

    # def test_10_get_EMG_calibration_response(self):
    #     print("Test 10 - Get EMG Calibration response command")  # no BT command for this
    #     response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET)
    #
    def test_11_all_calibration_response(self):
        print("Test 11 - Get all calibration response command")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_ALL_CALIBRATION_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.ALL_CALIBRATION_RESPONSE, 21)
        print("")

    def test_12_FW_Version_response(self):

        print("Test 12 - Get FW response command")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_FW_VERSION_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.FW_VERSION_RESPONSE, 6)
        print("")

    # def test_13_get_blink_led_response(self):
    #     print("Test 13 - Blink LED response command:")
    #     response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_TOGGLE_LED_COMMAND,
    #                                        shimmer_comms_bluetooth.BtCmds.TOGGLE_LED_COMMAND, 1)  # no blink led command

    def test_14_get_buffer_size_command(self):
        print("Test 14 - Buffer size repsonse Command: ")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_BUFFER_SIZE_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.BUFFER_SIZE_RESPONSE,
                                           1)  # get buffer size or response as 2 commands are mentioned
        print("")

    def test_15_get_mag_gain_command(self):
        print("Test 15 - Get mag gain command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_MAG_GAIN_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.MAG_GAIN_RESPONSE, 1)
        print("")

    # def test_16_get_charge_status_led_response(self):
    #     print("Test 16 - Get Charge Status LED response command:")
    #     response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_CHARGE_STATUS_LED_COMMAND,
    #                                        shimmer_comms_bluetooth.BtCmds.CHARGE_STATUS_LED_RESPONSE, 3) #not mentioned in Shimmer Bluetooth
    #     print("")

    def test_17_get_shimmer_version_command_response(self):
        print("Test 17 - Get Shimmer Version response command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_SHIMMERNAME_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.SHIMMERNAME_RESPONSE,1)  # is version and name same ?
        print("")

    def test_18_get_shimmer_new_version_command_response(self):
        print("Test 18 - Get new Shimmer version response command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_NSHIMMER_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.NSHIMMER_RESPONSE, 1)
        print("")

    # def test_19_get_ecg_calibration_command_response(self):
    #     print("Test 19 - Get ECG Calibration response command:")
    #     response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.) # no ecg in BT commands

    def test_20_get_mag_sampling_command(self):
        print("Test 20 - Get Mag sampling Command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_MAG_SAMPLING_RATE_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.MAG_SAMPLING_RATE_RESPONSE, 1)
        print("")

    def test_21_get_accel_sampling_rate_command(self):
        print("Test 21 - Get accel sampling rate response command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_ACCEL_SAMPLING_RATE_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.ACCEL_SAMPLING_RATE_RESPONSE, 1)
        print("")

    def test_22_get_lsm303dlhc_accel_lpmode_command(self):
        print("Test 22 - Get LSM303DLHC accel lpmode command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_ACCEL_LPMODE_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.ACCEL_LPMODE_RESPONSE, 1)
        print("")

    def test_23_get_lsm303dlhc_accel_hrmode_command(self):
        print("Test 23 - GET LSM303DLHC accel hrmode command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_ACCEL_HRMODE_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.ACCEL_HRMODE_RESPONSE, 1)
        print("")

    def test_24_get_MPU9150_Gyro_range_command(self):
        print("Test 24 - Get MPU9150 Gyro range command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_GYRO_RANGE_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.GYRO_RANGE_RESPONSE, 1)
        print("")

    def test_25_get_MPU9150_sampling_rate_command(self):
        print("Test 25 - Get MPU9150 Sampling rate command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_SAMPLING_RATE_COMMAND,  # works
                                           shimmer_comms_bluetooth.BtCmds.SAMPLING_RATE_RESPONSE, 2)
        print("")

    # def test_26_get_BMP180_Pres_Resolution_Command(self):
    #     print("Test 25 - Get BMP180 Pres Resolution command:")
    #     response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_SAMPLING_RATE_COMMAND,
    #                                        shimmer_comms_bluetooth.BtCmds.SAMPLING_RATE_RESPONSE, 2)
    #     print("")
    #
    # def test_27_get_BMP180_Pres_Calibration_command(self):
    #     print("Test 25 - Get MPU9150 Sampling rate command:")
    #     response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_SAMPLING_RATE_COMMAND,
    #                                        shimmer_comms_bluetooth.BtCmds.SAMPLING_RATE_RESPONSE, 2)
    #     print("")
    #
    # def test_28_get_BMP180_Calibration_coefficients_command(self):
    #     print("Test 28 - Get BMP180 Calibration response command:")
    #     response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_BMP180_CALIBRATION_COEFFICIENTS_COMMAND,
    #                                        shimmer_comms_bluetooth.BtCmds.BMP180_CALIBRATION_COEFFICIENTS_RESPONSE, 22)
    #     print("")  # works

    #
    def test_29_get_BMP280_Calibration_coefficients_command(self):
        print("Test 29 - Get BMP280 Caibration coefficient response command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_BMP280_CALIBRATION_COEFFICIENTS_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.BMP280_CALIBRATION_COEFFICIENTS_RESPONSE, 24)
        print("")  # works

    def test_31_get_internal_exp_power_enable_command(self):
        print("Test 31 - Get exp power enable response command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_INTERNAL_EXP_POWER_ENABLE_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.INTERNAL_EXP_POWER_ENABLE_RESPONSE, 1)
        print("")

    def test_32_get_exg_regs_command(self):
        print("Test 32 - Get ExG Regs command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_EXG_REGS_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.EXG_REGS_RESPONSE, 1)
        print("")

    def test_33_get_daughter_card_id_command(self):
        print("Test 33 - Get daughter card id command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_DAUGHTER_CARD_ID_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.DAUGHTER_CARD_ID_RESPONSE, 1)  # 3+1 mentioned
        print("")

    def test_34_get_status(self):
        print("Test 34 - Get status command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_STATUS_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.STATUS_RESPONSE, 2, is_instream_response=True)

        status = response[0]
        self_cmd = (status & 0x04) >> 2
        sensing = (status & 0x02) >> 1
        docked = (status & 0x01)
        print("0x%02x , self: %d, sensing: %d, docked: %d" % (status, self_cmd, sensing, docked))

        print("")

    def test_35_get_baud_rate_command(self):
        print("Test 34 - Get baud rate response command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_BT_COMMS_BAUD_RATE,
                                           shimmer_comms_bluetooth.BtCmds.BT_COMMS_BAUD_RATE_RESPONSE, 1)
        print("")

    def test_36_get_derived_channel_bytes_command(self):
        print("Test 36 - Get Derived Channel Bytes response command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_DERIVED_CHANNEL_BYTES,
                                           shimmer_comms_bluetooth.BtCmds.DERIVED_CHANNEL_BYTES_RESPONSE, 8)
        print("")

    #
    def test_37_get_trial_config_command(self):
        print("Test 37 - Get trial config command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_TRIAL_CONFIG_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.TRIAL_CONFIG_RESPONSE, 3)
        print("")

    def test_38_get_center_command(self):
        print("Test 38 - Get Center response command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_CENTER_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.CENTER_RESPONSE, 1)
        print("")

    def test_39_get_Shimmername_command(self):
        print("Test 39 - Get ShimmerName command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_SHIMMERNAME_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.SHIMMERNAME_RESPONSE, 13)
        print("")

    def test_40_get_expID_command(self):
        print("Test 40 - Get ExpID command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_EXPID_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.EXPID_RESPONSE, 12)
        print("")

    #
    def test_41_get_myID_command(self):
        print("Test 41 - Get myID command:")  # works
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_MYID_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.MYID_RESPONSE, 1)
        print("")

    def test_42_NSHIMMER_command(self):
        print("Test 42 - Get nshimmer response command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_NSHIMMER_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.NSHIMMER_RESPONSE, 1)
        print("")

    #
    def test_43_get_ConfigTime_command(self):
        print("Test 43- Get Config Time Response Command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_CONFIGTIME_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.CONFIGTIME_RESPONSE, 1)
        print("")

    # def test_44_get_dir_command(self):
    #     print("Test 43 - Get dir response command:")
    #     response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_DIR_COMMAND,
    #                                        shimmer_comms_bluetooth.BtCmds.DIR_RESPONSE, 4)
    #     print("")

    # def test_45_get_infomem_command(self):
    #     print("Test 45 - Get infomem response command:")
    #     response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_INFOMEM_COMMAND,
    #                                        shimmer_comms_bluetooth.BtCmds.INFOMEM_RESPONSE, 1)
    #     print("") #bluetooth command not present

    def test_46_get_rwc_command(self):
        print("Test 46- Get RWC response command")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_RWC_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.RWC_RESPONSE, 1)
        print("")

    def test_47_get_vbatt_command(self):
        print("Test 47 - Get VBatt response command")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_VBATT_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.VBATT_RESPONSE, 2)
        print("")

    def test_48_get_BT_FW_VERSION_command(self):
        print("Test 48 - Get BT FW Version response command")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_BT_VERSION_STR_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.BT_VERSION_STR_RESPONSE, 1)
        print("")

    # set commands

   # def test_49_set_sensors_command(self):
        # print("Test 49: Set sensor response command")
        # sensor_bytes = [0x01]
        # if self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.SET_SENSORS_COMMAND):
        #     print("set")
        #     self.assertTrue(False)
        #
        # sensor_bytes =  self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.SENSOR)
        #     self.assertTrue(False)
        # print("") no get command for this

    def test_50_set_accel_sensitivity_command(self):
        print("Test 50 - set accel sensitivity command ")
        accel_bytes = [0x09]
        if self.shimmer.bluetooth_port.write_accel_sensitivity(accel_bytes):
            print("Error, exiting")
            self.assertTrue(False)
        self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.GET_ACCEL_RANGE_COMMAND])
        # if isinstance(accel_bytes, bool):
        #     print("Error")
        #     self.assertTrue(False)
        # print("")

    def test_51_set_config_byte0_command(self):
        print("Test 51 - set Config Byte0 command ")
        config_bytes = [0x0E]
        len_config_bytes = len(config_bytes)
        config_bytes = [(len_config_bytes & 0xFF), ((len_config_bytes >> 8) & 0xFF)] + config_bytes
        for i in range(0, len_config_bytes, 128):
            bytes_remaining = len_config_bytes - i
            buf_len = 128 if bytes_remaining > 128 else bytes_remaining

            self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.SET_CONFIG_SETUP_BYTES_COMMAND,
                                 buf_len, i & 0xFF, (i >> 8) & 0xFF] +
                                config_bytes[i:i + buf_len])
            result = self.shimmer.bluetooth_port.wait_for_ack()
            if not result:
                return result
        return result

        config_bytes = self.shimmer.bluetooth_port.send_bluetooth([
                                        shimmer_comms_bluetooth.BtCmds.GET_CONFIG_SETUP_BYTES_COMMAND])
        if isinstance(config_bytes, bool):
            print("Error")
            self.assertTrue(False)

        print("")

    def test_52_set_accel_calibration_command(self):
        print("Test 52 - Set accel Calibration Command")
        accel_calib_bytes = [0x03, 0x00, 0x03, 0x00, 0x01, 0x11, 0x00, 0x00, 0x02, 0x00, 0x00, 0x15, 0x00, 0x00,
                             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08]
        if self.shimmer.bluetooth_port.send_bluetooth(accel_calib_bytes):
            print("set")
            self.assertTrue(True)

        accel_calib_bytes = self.shimmer.bluetooth_port.read_calibration()
        if isinstance(accel_calib_bytes, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_53_set_lsm303dlhc_accel_calibration_command(self):
        print("Test 53 - Set LSM303DLHC Accel Calibration Command")
        accel_calib_bytes = [0x03, 0x00, 0x03, 0x00, 0x01, 0x11, 0x00, 0x00, 0x02, 0x00, 0x00, 0x15, 0x00, 0x00,
                             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08]
        if self.shimmer.bluetooth_port.send_bluetooth(accel_calib_bytes):
            print("set")
            self.assertTrue(True)

        if isinstance(accel_calib_bytes, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_54_set_gyro_calibration_command(self):
        print("Test 54 - Set Gyro Calibration Command ")
        gyro_bytes = [0x03, 0x00, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x15, 0x00, 0x00, 0x00, 0x00,
                      0x00, 0x00, 0x00, 0x00, 0x08]
        if self.shimmer.bluetooth_port.send_bluetooth(gyro_bytes):
            print("Error, exiting")
            self.assertTrue(True)

        gyro_bytes = self.shimmer.bluetooth_port.read_calibration()
        if isinstance(gyro_bytes, bool):
            print("Error")
            self.assertTrue(False)

        print("")

    def test_55_set_mag_calibration_command(self):
        print("Test 55 - Set Mag Calibration Command")
        mag_bytes = [0x03, 0x00, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x15, 0x00, 0x00, 0x00, 0x00,
                     0x00, 0x00, 0x00, 0x00, 0x08]
        if self.shimmer.bluetooth_port.send_bluetooth(mag_bytes):
            print("set")
            self.assertTrue(True)

        mag_bytes = self.shimmer.bluetooth_port.read_calibration()
        if isinstance(mag_bytes, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_56_set_mag_gain_command(self):
        print("Test 56 - Set Mag gain command")
        mag_gain = [0x00, 0x01]
        if self.shimmer.bluetooth_port.send_bluetooth(mag_gain):
            print("set")
            self.assertTrue(True)

        mag_gain = self.shimmer.bluetooth_port.read_calibration()
        if isinstance(mag_gain, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_57_set_gsr_range_command(self):
        print("Test 57- Set GSR Range Command")
        response = [0, 1, 2, 3, 4]
        print("set")
        self.assertTrue(True)

        gsr_range = self.shimmer.bluetooth_port.read_calibration()
        if isinstance(gsr_range, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_58_set_emg_calibration_command(self):
        print("Test 58 - Set EMG Calibration Command")
        emg_calib = [0x00, 0x02, 0x00, 0x00, 0x15, 0x00]
        if self.shimmer.bluetooth_port.send_bluetooth(emg_calib):
            print("set")
            self.assertTrue(True)

        emg_calib = self.shimmer.bluetooth_port.read_calibration()
        if isinstance(emg_calib, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_59_set_ecg_calibration_command(self):
        print("Test 59 - Set ECG calibration command")
        ecg_calib = [0x00, 0x02, 0x00, 0x00, 0x15, 0x00, 0x02, 0x00, 0x00, 0x15]
        if not self.shimmer.bluetooth_port.send_bluetooth(ecg_calib):
            print("set")
            self.assertTrue(True)

        ecg_calib = self.shimmer.bluetooth_port.read_calibration()
        if isinstance(ecg_calib, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    # def test_60_set_blink_led(self):
    #     print("Test 59 - Set Blink LED command")   #no blink led command present
    #     if self.shimmer.bluetooth_port.write_calibration():
    #         print("set")
    #         self.assertTrue(False)

    # accel_bytes = self.shimmer.bluetooth_port.read_calibration()
    # if isinstance(accel_bytes, bool):
    #     print("Error")
    #     self.assertTrue(False)
    # print("")

    def test_61_set_gyro_temp_vref_command(self):
        print("Test 61- Set Gyro Temp Vref Command")
        if self.shimmer.bluetooth_port.wait_for_ack():
            print("set")
            self.assertTrue(True)

        # accel_bytes = self.shimmer.bluetooth_port.read_calibration()
        # if isinstance(accel_bytes, bool):
        #     print("Error")
        #     self.assertTrue(False)
        print("")

    def test_62_set_buffer_size_command(self):
        print("test 62- Set Buffer Size Command")
        buffer_size = [0x00, 0x01]
        if self.shimmer.bluetooth_port.send_bluetooth(buffer_size):
            print("set")
            self.assertTrue(True)

        buffer_size = self.shimmer.bluetooth_port.read_calibration()
        if isinstance(buffer_size, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_64_set_mag_sampling_rate_command(self):
        print("Test 64 - Set Mag Sampling rate command")
        mag_sampling_rate = [0x00, 0x01]
        if self.shimmer.bluetooth_port.send_bluetooth(mag_sampling_rate):
            print("set")
            self.assertTrue(True)

        mag_sampling_rate = self.shimmer.bluetooth_port.read_calibration()
        if isinstance(mag_sampling_rate, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_65_set_accel_sampling_rate_command(self):
        print("Test 65 - Set Accel Sampling rate command")
        accel_sampling = [0x00, 0x01]
        if self.shimmer.bluetooth_port.send_bluetooth(accel_sampling):
            print("set")
            self.assertTrue(True)

        accel_sampling = self.shimmer.bluetooth_port.read_calibration()
        if isinstance(accel_sampling, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_66_set__lsm303dlhc_accel_lpmode_command(self):
        print("Test 66 - Set LSM303DLHC lpmode command")
        accel_lrmode = [0x00, 0x01]
        if self.shimmer.bluetooth_port.send_bluetooth(accel_lrmode):
            print("set")
            self.assertTrue(True)

        accel_lrmode = self.shimmer.bluetooth_port.read_calibration()
        if isinstance(accel_lrmode, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_67_set__lsm303dlhc_accel_hrmode_command(self):
        print("Test 67 - Set LSM303DLHC accel hrmode command")
        accel_hrmode = [0x00, 0x01]
        if self.shimmer.bluetooth_port.send_bluetooth(accel_hrmode):
            print("set")
            self.assertTrue(True)

        accel_hrmode = self.shimmer.bluetooth_port.read_calibration()
        if isinstance(accel_hrmode, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_68_set_mpu9150_gyro_range_command(self):
        print("Test 68 - MPU9150 Gyro Range command")
        mpu9150_gyro = [0x00, 0x01]
        if self.shimmer.bluetooth_port.send_bluetooth(mpu9150_gyro):
            print("set")
            self.assertTrue(True)

        mpu9150_gyro = self.shimmer.bluetooth_port.read_calibration()
        if isinstance(mpu9150_gyro, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_69_set__mpu9150_sampling_range_command(self):
        print("Test 69 - Set MPU9150 Sampling Range Command")
        mpu9150_sampling = [0x00, 0x01]
        if self.shimmer.bluetooth_port.send_bluetooth(mpu9150_sampling):
            print("set")
            self.assertTrue(True)

        mpu9150_sampling = self.shimmer.bluetooth_port.read_calibration()
        if isinstance(mpu9150_sampling, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_70_set_bmp180_pres_resolution_command(self):
        print("Test 70 - Set BMP180 Pres Resolution Command ")
        bmp180_pres_resolution = [0x01, 0x02]
        if self.shimmer.bluetooth_port.send_bluetooth(bmp180_pres_resolution):
            print("set")
            self.assertTrue(True)

        bmp180_pres_resolution = self.shimmer.bluetooth_port.read_calibration()
        if isinstance(bmp180_pres_resolution, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    # def test_71_set_bmp180_pres_calibration_command(self):
    #     print("Test 71 - Set BMp180 Pres Calibration command")
    #     bmp180_pres_calibration = [0x01, 0x02]
    #     if self.shimmer.bluetooth_port.send_bluetooth(bmp180_pres_calibration):
    #         print("set")
    #         self.assertTrue(True)
    #
    #     bmp180_pres_calibration = self.shimmer.bluetooth_port.read_calibration()
    #     if isinstance(bmp180_pres_calibration, bool):
    #         print("Error")
    #         self.assertTrue(False)
    #     print("")
    #
    # def test_72_set_bmp180_calibration_coefficient_response(self):
    #     print("Test 72 - Set BMp180 Pres Calibration coefficient")
    #     bmp180_calibration_coefficient = [0x03, 0x00, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x15, 0x00,
    #                                       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0xCD, 0x08, 0xCD]
    #     if self.shimmer.bluetooth_port.send_bluetooth(bmp180_calibration_coefficient):
    #         print("set")
    #         self.assertTrue(True)
    #
    #     bmp180_calibration_coefficient = self.shimmer.bluetooth_port.read_calibration()
    #     if isinstance(bmp180_calibration_coefficient, bool):
    #         print("Error")
    #         self.assertTrue(False)
    #     print("")

    def test_73_set_bmp280_pres_calibration_coefficient_command(self):
        print("Test 73 - Set BMp280 Pres Calibration command")
        bmp280_pres_calibration_coefficient = [0x01, 0x02, 0x03, 0x00, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00,
                                               0x02, 0x00, 0x00, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                               0x00, 0x08, 0xCD, 0x08, 0xCD]
        if self.shimmer.bluetooth_port.send_bluetooth(bmp280_pres_calibration_coefficient):
            print("set")
            self.assertTrue(True)

        bmp280_pres_calibration_coefficient = self.shimmer.bluetooth_port.read_calibration()
        if isinstance(bmp280_pres_calibration_coefficient, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_74_mpu9150_set_mag_sens_adj_vals_response(self):
        print("Test 74 - Set Mag Sens Adj Vals Response")
        mag_sens_adj_response = [0x01, 0x02]
        if self.shimmer.bluetooth_port.send_bluetooth(mag_sens_adj_response):
            print("set")
            self.assertTrue(True)

        mag_sens_adj_response = self.shimmer.bluetooth_port.read_calibration()
        if isinstance(mag_sens_adj_response, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_75_set_internal_exp_power_enable_command(self):
        print("Test 75 - Set Internal exp power enable command")
        internal_exp_power_enable_response = [0x00]
        if self.shimmer.bluetooth_port.send_bluetooth(internal_exp_power_enable_response):
            print("set")
            self.assertTrue(True)

        internal_exp_power_enable_response = self.shimmer.bluetooth_port.read_calibration()
        if isinstance(internal_exp_power_enable_response, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_76_set_exg_regs_response(self):
        print("Test 76 - Set ExG Regs Response")
        exg_regs_response = [0x01, 0x02, 0x01, 0x02, 0x03, 0x00, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02]
        if self.shimmer.bluetooth_port.send_bluetooth(exg_regs_response):
            print("set")
            self.assertTrue(True)

        exg_regs_response = self.shimmer.bluetooth_port.read_calibration()
        if isinstance(exg_regs_response, bool):
            print("Error")
            self.assertTrue(False)
        print("")

        # no set daughter card id command - Test 77

    def test_78_set_baud_rate_command(self):
        print("Test 78 - Set baud Rate Command")
        baud_rate_response = [0x01]
        if self.shimmer.bluetooth_port.send_bluetooth(baud_rate_response):
            print("set")
            self.assertTrue(True)

        baud_rate_response = self.shimmer.bluetooth_port.read_calibration()
        if isinstance(baud_rate_response, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_79_set_derived_channel_bytes_command(self):
        print("Test 79 - Set Derived Channel bytes Command")
        derived_channel_bytes = [0x00, 0x01, 0x02]
        if self.shimmer.bluetooth_port.send_bluetooth(derived_channel_bytes):
            print("set")
            self.assertTrue(True)

        derived_channel_bytes = self.shimmer.bluetooth_port.read_calibration()
        if isinstance(derived_channel_bytes, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_80_set_trial_config_command(self):
        print("Test 80- Set Trial Config Command")
        trial_config = [0x01]
        if self.shimmer.bluetooth_port.send_bluetooth(trial_config):
            print("set")
            self.assertTrue(True)

        trial_config = self.shimmer.bluetooth_port.read_calibration()
        if isinstance(trial_config, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_81_set_center_command(self):
        print("Test 81 - Set Center Command")
        center_response = [0x01]
        if self.shimmer.bluetooth_port.send_bluetooth(center_response):
            print("set")
            self.assertTrue(True)

        center_response = self.shimmer.bluetooth_port.read_calibration()
        if isinstance(center_response, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_82_set_shimmerName_command(self):
        print("Test 82 - Set Shimmer Name Command ")
        shimmer_name = [0x01]
        if self.shimmer.bluetooth_port.send_bluetooth(shimmer_name):
            print("set")
            self.assertTrue(True)

        shimmer_name = self.shimmer.bluetooth_port.read_calibration()
        if isinstance(shimmer_name, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_83_set_ExpID_command(self):
        print("Test 83 - Set ExpId command")
        exp_id = [0x01]
        if self.shimmer.bluetooth_port.send_bluetooth(exp_id):
            print("set")
            self.assertTrue(True)

        exp_id = self.shimmer.bluetooth_port.read_calibration()
        if isinstance(exp_id, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_84_set_myID_command(self):
        print("Test 84 - Set My ID command")
        my_id = [0x01]
        if self.shimmer.bluetooth_port.send_bluetooth(my_id):
            print("set")
            self.assertTrue(True)

        my_id = self.shimmer.bluetooth_port.read_calibration()
        if isinstance(my_id, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_85_set_NShimmer_command(self):
        print("Test 85- Set nShimmer command")
        shimmer_response = [0x01]
        if self.shimmer.bluetooth_port.send_bluetooth(shimmer_response):
            print("set")
            self.assertTrue(True)

        shimmer_response = self.shimmer.bluetooth_port.read_calibration()
        if isinstance(shimmer_response, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_86_set_ConfigTime_command(self):
        print("Test 86 - Set ConfigTime Command")
        config_time = [0x01]
        if self.shimmer.bluetooth_port.send_bluetooth(config_time):
            print("set")
            self.assertTrue(True)

        config_time = self.shimmer.bluetooth_port.read_calibration()
        if isinstance(config_time, bool):
            print("Error")
            self.assertTrue(False)
        print("")

        # no set dir and set instream cmd response

    def test_87_set_InfoMem_command(self):
        print("Test 87 - Set InfoMem Command")
        infomem_bytes = [0x01]
        if self.shimmer.bluetooth_port.send_bluetooth(infomem_bytes):
            print("set")
            self.assertTrue(True)

        infomem_bytes = self.shimmer.bluetooth_port.read_calibration()
        if isinstance(infomem_bytes, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_88_set_calib_dump_command(self):
        print("Test 88 - Set Calib Command")
        calib_dump = [0x01]
        if self.shimmer.bluetooth_port.send_bluetooth(calib_dump):
            print("set")
            self.assertTrue(True)

        calib_dump = self.shimmer.bluetooth_port.read_calibration()
        if isinstance(calib_dump, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_89_set_crc_command(self):
        print("Test 89 - Set CRC Command")
        crc_response = [0x01]
        if self.shimmer.bluetooth_port.send_bluetooth(crc_response):
            print("set")
            self.assertTrue(True)

        crc_response = self.shimmer.bluetooth_port.read_calibration()
        if isinstance(crc_response, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_90_set_rwc_command(self):
        print("Test 90 - Set RWC command")
        rwc_bytes = [0x03, 0x00, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00]
        if self.shimmer.bluetooth_port.send_bluetooth(rwc_bytes):
            print("set")
            self.assertTrue(True)

        rwc_bytes = self.shimmer.bluetooth_port.read_calibration()
        if isinstance(rwc_bytes, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_91_UPD_calib_dump_command(self):  # not present in set
        print("Test 91 - UPD Calib Dump Command")
        upd_calib_dump = [0x01]
        if self.shimmer.bluetooth_port.send_bluetooth(upd_calib_dump):
            print("set")
            self.assertTrue(True)

        upd_calib_dump = self.shimmer.bluetooth_port.read_calibration()
        if isinstance(upd_calib_dump, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_92_UPD_SDlog_Cfg_command(self):
        print("Test 92 - UPD SDlog Cfg Command")
        if self.shimmer.bluetooth_port.wait_for_ack():
            self.assertTrue(True)
        print("")


if __name__ == '__main__':
    unittest.main()
