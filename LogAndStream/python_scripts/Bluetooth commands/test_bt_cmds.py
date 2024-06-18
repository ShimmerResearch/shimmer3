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
        if tx_cmd_byte == shimmer_comms_bluetooth.BtCmds.GET_DAUGHTER_CARD_ID_COMMAND:
            self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.GET_DAUGHTER_CARD_ID_COMMAND,
                                                         0x03 ,0x00])
        elif tx_cmd_byte == shimmer_comms_bluetooth.BtCmds.GET_INFOMEM_COMMAND:
            self.shimmer.bluetooth_port.send_bluetooth([0x8E, 0x80, 0x00, 0x00])
        elif tx_cmd_byte == shimmer_comms_bluetooth.BtCmds.GET_ECG_CALIBRATION_COMMAND:
             self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.GET_ECG_CALIBRATION_COMMAND,
                                                        0x01])
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

    def test_10_get_EMG_calibration_response(self):
        print("Test 10 - Get EMG Calibration response command")  # no BT
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_EMG_CALIBRATION_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.EMG_CALIBRATION_RESPONSE, 1)

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

    def test_13_get_blink_led_response(self):
        print("Test 13 - Blink LED response command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_BLINK_LED,
                                           shimmer_comms_bluetooth.BtCmds.BLINK_LED_RESPONSE, 1)  # no blink led command

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

    def test_16_get_charge_status_led_response(self):
        print("Test 16 - Get Charge Status LED response command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_CHARGE_STATUS_LED_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.CHARGE_STATUS_LED_RESPONSE, 3)
    #
    #                                        #not mentioned in Shimmer Bluetooth
        print("")

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

    def test_19_get_ecg_calibration_command_response(self):
        print("Test 19 - Get ECG Calibration response command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_ECG_CALIBRATION_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.ECG_CALIBRATION_RESPONSE, 1) # no ecg in BT commands

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

    def test_30_get_MPU9150_MAG_SENS_ADJ_VALS_RESPONSE(self):
        print("Test 31 - Get MPU9150 Mag Sens Adj Vals response command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_ALT_MAG_SENS_ADJ_VALS_COMMAND,
                                               shimmer_comms_bluetooth.BtCmds.ALT_MAG_SENS_ADJ_VALS_RESPONSE, 1)
        print("")

    def test_31_get_internal_exp_power_enable_command(self):
        print("Test 31 - Get exp power enable response command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_INTERNAL_EXP_POWER_ENABLE_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.INTERNAL_EXP_POWER_ENABLE_RESPONSE, 1)
        print("")

    # def test_32_get_exg_regs_command(self):
    #     print("Test 32 - Get ExG Regs command:")
    #     response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_EXG_REGS_COMMAND,
    #                                        shimmer_comms_bluetooth.BtCmds.EXG_REGS_RESPONSE, 11)
    #     print("")

    def test_33_get_daughter_card_id_command(self):
        print("Test 33 - Get daughter card id command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_DAUGHTER_CARD_ID_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.DAUGHTER_CARD_ID_RESPONSE, 3)

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

    def test_45_get_infomem_command(self):
        print("Test 45 - Get infomem response command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_INFOMEM_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.INFOMEM_RESPONSE, 128)
        print("")

    def test_46_get_rwc_command(self):
        print("Test 46- Get RWC response command")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_RWC_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.RWC_RESPONSE, 1)
        print("")

    def test_47_get_vbatt_command(self):
        print("Test 47 - Get VBatt response command")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_VBATT_COMMAND,
                                        shimmer_comms_bluetooth.BtCmds.VBATT_RESPONSE, 3, is_instream_response= True)
        batt_voltage = ((response[0] & 0xFF) << 8) + (response[0] & 0xFF)
        charging_status = response[1]
        print("batt_voltage: %03f , charging_status: %d" % (batt_voltage, charging_status))
        print("")

    def test_48_get_BT_FW_VERSION_command(self):
        print("Test 48 - Get BT FW Version response command")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_BT_VERSION_STR_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.BT_VERSION_STR_RESPONSE, 2)
        print("")

    # set commands

   # def test_49_set_sensors_command(self):
   #      print("Test 49: Set sensor response command")
   #      sensor_bytes = [0x01]
   #      if self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds._SENSORS_COMMAND):
   #          print("set")
   #          self.assertTrue(False) // need to write this 



    def test_50_set_accel_sensitivity_command(self):
        print("Test 50 - set accel sensitivity command ")
        accel_bytes = [0x01]
        if self.shimmer.bluetooth_port.write_accel_sensitivity(accel_bytes):
            print("Error, exiting")
            self.assertTrue(False)
        self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.GET_ACCEL_RANGE_COMMAND])
        self.shimmer.bluetooth_port.wait_for_ack()
        # if isinstance(accel_bytes, bool):
        #     print("Error")
        #     self.assertTrue(False)
        # print("")

    def test_51_set_config_byte0_command(self):
        print("Test 51 - set Config Byte0 command ")
        config_bytes = [0x06, 0x1, 0x02, 0x03]
        len_config_bytes = len(config_bytes)
        #config_bytes = [(len_config_bytes & 0xFF), ((len_config_bytes >> 8) & 0xFF)] + config_bytes
        for i in range(0, len_config_bytes, 4):
            bytes_remaining = len_config_bytes - i
            buf_len = 4 if bytes_remaining > 4 else bytes_remaining
        config_bytes = self.shimmer.bluetooth_port.send_bluetooth([
                                                        shimmer_comms_bluetooth.BtCmds.SET_CONFIG_SETUP_BYTES_COMMAND,
                                 buf_len, i & 0xFF, (i >> 8) & 0xFF] +
                                config_bytes[i:i + buf_len])
        result = self.shimmer.bluetooth_port.wait_for_ack()
        if not result:
            return result

       # config_bytes = self.shimmer.bluetooth_port.send_bluetooth([
                                        #shimmer_comms_bluetooth.BtCmds.GET_CONFIG_SETUP_BYTES_COMMAND])

        if isinstance(config_bytes, bool):
            print("Error")
            self.assertTrue(False)
        #return result
        print("")

    def test_52_set_A_accel_calibration_command(self):
        print("Test 52 - Set accel Calibration Command")
        accel_calib_bytes = [0x03, 0x00, 0x03, 0x00, 0x01, 0x11, 0x00, 0x00, 0x02, 0x00, 0x00, 0x15, 0x00, 0x00,
                             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08]
        len_accel_calib_bytes = len(accel_calib_bytes)
        for i in range(0, len_accel_calib_bytes, 21):
            bytes_remaining = len_accel_calib_bytes - i
            buf_len = 21 if bytes_remaining > 21 else bytes_remaining

        self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.SET_A_ACCEL_CALIBRATION_COMMAND,
                                                                      buf_len, i & 0xFF, (i >> 8) & 0xFF] +
                                                                  accel_calib_bytes[i:i + buf_len])
        result = self.shimmer.bluetooth_port.wait_for_ack()
        if not result:
            return result
        # if self.shimmer.bluetooth_port.send_bluetooth(accel_calib_bytes):
        #     print("set")
        #     self.assertTrue(True)
        accel_calib_bytes = self.shimmer.bluetooth_port.send_bluetooth(shimmer_comms_bluetooth.BtCmds.
                                                                       GET_A_ACCEL_CALIBRATION_COMMAND)
        shimmer_comms_bluetooth.ShimmerBluetooth.wait_for_ack()
        if isinstance(accel_calib_bytes, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_53_set_lsm303dlhc_accel_calibration_command(self):
        print("Test 53 - Set LSM303DLHC Accel Calibration Command")
        lsm_accel_calib_bytes = [0x03, 0x00, 0x03, 0x00, 0x01, 0x11, 0x00, 0x00, 0x02, 0x00, 0x00, 0x15, 0x00, 0x00,
                             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08]
        len_lsm_accel_calib_bytes = len(lsm_accel_calib_bytes)
        for i in range(0, len_lsm_accel_calib_bytes, 21):
            bytes_remaining = len_lsm_accel_calib_bytes - i
            buf_len = 21 if bytes_remaining > 21 else bytes_remaining
        self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.SET_ACCEL_CALIBRATION_COMMAND,
                                                    buf_len, i & 0xFF, (i >> 8) & 0xFF] +
                                                   lsm_accel_calib_bytes[i:i + buf_len])
        result = self.shimmer.bluetooth_port.wait_for_ack()
        if not result:
            return result
        self.shimmer.bluetooth_port.send_bluetooth(shimmer_comms_bluetooth.BtCmds.
                                                                       GET_ACCEL_CALIBRATION_COMMAND)
        shimmer_comms_bluetooth.ShimmerBluetooth.wait_for_ack()

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
        mag_gain = [0x01]
        len_mag_gain = len(mag_gain)
        for i in range(0, len_mag_gain, 1):
            bytes_remaining = len_mag_gain - i
            buf_len = 1 if bytes_remaining > 1 else bytes_remaining
        self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.SET_MAG_GAIN_COMMAND,
                                                    buf_len, i & 0xFF, (i >> 8) & 0xFF] + mag_gain[i:i + buf_len])
        result = self.shimmer.bluetooth_port.wait_for_ack()
        if not result:
            return result
        mag_gain = self.shimmer.bluetooth_port.send_bluetooth(shimmer_comms_bluetooth.BtCmds.GET_MAG_GAIN_COMMAND)
        shimmer_comms_bluetooth.ShimmerBluetooth.wait_for_ack()
        if isinstance(mag_gain, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_57_set_gsr_range_command(self):
        print("Test 57- Set GSR Range Command")
        mag_gain = [0x01]
        len_mag_gain = len(mag_gain)
        for i in range(0, len_mag_gain, 1):
            bytes_remaining = len_mag_gain - i
            buf_len = 1 if bytes_remaining > 1 else bytes_remaining
        self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.SET_MAG_GAIN_COMMAND,
                                                    buf_len, i & 0xFF, (i >> 8) & 0xFF] + mag_gain[i:i + buf_len])
        result = self.shimmer.bluetooth_port.wait_for_ack()
        if not result:
            return result
        mag_gain = self.shimmer.bluetooth_port.send_bluetooth(shimmer_comms_bluetooth.BtCmds.GET_MAG_GAIN_COMMAND)
        shimmer_comms_bluetooth.ShimmerBluetooth.wait_for_ack()
        if isinstance(mag_gain, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_58_set_emg_calibration_command(self):
        print("Test 58 - Set EMG Calibration Command")
        emg_calib = [0x00]
        len_emg_calib = len(emg_calib)
        for i in range(0, len_emg_calib, 1):
            bytes_remaining = len_emg_calib - i
            buf_len = 1 if bytes_remaining > 1 else bytes_remaining
        self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.SET_EMG_CALIBRATION_COMMAND,
                                                    buf_len, i & 0xFF, (i >> 8) & 0xFF] + emg_calib[i:i + buf_len])
        result = self.shimmer.bluetooth_port.wait_for_ack()
        if not result:
            return result
        emg_calib = self.shimmer.bluetooth_port.send_bluetooth(shimmer_comms_bluetooth.BtCmds.GET_EMG_CALIBRATION_COMMAND)
        shimmer_comms_bluetooth.ShimmerBluetooth.wait_for_ack()
        if isinstance(emg_calib, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_59_set_ecg_calibration_command(self):
        print("Test 59 - Set ECG calibration command")
        ecg_calib = [0x15]
        len_ecg_calib = len(ecg_calib)
        for i in range(0, len_ecg_calib, 1):
            bytes_remaining = len_ecg_calib - i
            buf_len = 1 if bytes_remaining > 1 else bytes_remaining
        self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.SET_ECG_CALIBRATION_COMMAND,
                                                    buf_len, i & 0xFF, (i >> 8) & 0xFF] + ecg_calib[i:i + buf_len])
        result = self.shimmer.bluetooth_port.wait_for_ack()
        if not result:
            return result
        ecg_calib = self.shimmer.bluetooth_port.send_bluetooth(
                                                            shimmer_comms_bluetooth.BtCmds.GET_EMG_CALIBRATION_COMMAND)
        shimmer_comms_bluetooth.ShimmerBluetooth.wait_for_ack()
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

    # def test_61_set_gyro_temp_vref_command(self):
        # print("Test 61- Set Gyro Temp Vref Command")
        # gyro_temp = [0x01]
        # len_gyro_temp = len(gyro_temp)
        # for i in range(0, len_gyro_temp, 1):
        #         bytes_remaining = len_gyro_temp - i
        #         buf_len = 1 if bytes_remaining > 1 else bytes_remaining
        #         self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.SET_MAG_GAIN_COMMAND,
        #                                                 buf_len, i & 0xFF, (i >> 8) & 0xFF] + gyro_temp[i:i + buf_len])
        # result = self.shimmer.bluetooth_port.wait_for_ack()
        # if not result:
        #         return result
        # gyro_temp = self.shimmer.bluetooth_port.send_bluetooth(shimmer_comms_bluetooth.BtCmds.GET_MAG_GAIN_COMMAND)
        # shimmer_comms_bluetooth.ShimmerBluetooth.wait_for_ack()
        # print("")

    def test_62_set_buffer_size_command(self):
        print("test 62- Set Buffer Size Command")
        buffer_size = [0x01]
        len_buffer_size = len(buffer_size)
        for i in range(0, len_buffer_size, 1):
            bytes_remaining = len_buffer_size - i
            buf_len = 1 if bytes_remaining > 1 else bytes_remaining
            self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.SET_BUFFER_SIZE_COMMAND,
                                                    buf_len, i & 0xFF, (i >> 8) & 0xFF] + buffer_size[i:i + buf_len])
        result = self.shimmer.bluetooth_port.wait_for_ack()
        if not result:
            return result
        mag_gain = self.shimmer.bluetooth_port.send_bluetooth(shimmer_comms_bluetooth.BtCmds.GET_BUFFER_SIZE_COMMAND)
        shimmer_comms_bluetooth.ShimmerBluetooth.wait_for_ack()
        if isinstance(mag_gain, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_64_set_mag_sampling_rate_command(self):
        print("Test 64 - Set Mag Sampling rate command")
        mag_sampling_rate = [0x01]
        len_mag_sampling_rate = len(mag_sampling_rate)
        for i in range(0, len_mag_sampling_rate, 1):
            bytes_remaining = len_mag_sampling_rate - i
            buf_len = 1 if bytes_remaining > 1 else bytes_remaining
            self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.SET_MAG_SAMPLING_RATE_COMMAND,
                                                    buf_len,i & 0xFF, (i >> 8) & 0xFF] +
                                                   mag_sampling_rate[i:i + buf_len])
        result = self.shimmer.bluetooth_port.wait_for_ack()
        if not result:
         return result
        mag_sampling_rate = self.shimmer.bluetooth_port.send_bluetooth(shimmer_comms_bluetooth.BtCmds.
                                                                       GET_MAG_SAMPLING_RATE_COMMAND)
        shimmer_comms_bluetooth.ShimmerBluetooth.wait_for_ack()
        if isinstance(mag_sampling_rate, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_65_set_accel_sampling_rate_command(self):
        print("Test 65 - Set Accel Sampling rate command")
        accel_sampling = [0x01]
        len_accel_sampling_rate = len(accel_sampling)
        for i in range(0, len_accel_sampling_rate, 1):
            bytes_remaining = len_accel_sampling_rate - i
            buf_len = 1 if bytes_remaining > 1 else bytes_remaining
        self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.SET_ACCEL_SAMPLING_RATE_COMMAND,
                                                    buf_len, i & 0xFF, (i >> 8) & 0xFF] +
                                                   accel_sampling[i:i + buf_len])
        result = self.shimmer.bluetooth_port.wait_for_ack()
        if not result:
            return result
        accel_sampling = self.shimmer.bluetooth_port.send_bluetooth(shimmer_comms_bluetooth.BtCmds.
                                                                       GET_ACCEL_SAMPLING_RATE_COMMAND)
        shimmer_comms_bluetooth.ShimmerBluetooth.wait_for_ack()
        if isinstance(accel_sampling, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_66_set__lsm303dlhc_accel_lpmode_command(self):
        print("Test 66 - Set LSM303DLHC lpmode command")
        lsm303dlhc_lpmode = [0x02]
        len_lsm303dlhc_lpmode = len(lsm303dlhc_lpmode)
        for i in range(0, len_lsm303dlhc_lpmode, 1):
            bytes_remaining = len_lsm303dlhc_lpmode - i
            buf_len = 1 if bytes_remaining > 1 else bytes_remaining
        self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.SET_ACCEL_LPMODE_COMMAND,
                                                    buf_len, i & 0xFF, (i >> 8) & 0xFF] +
                                                   lsm303dlhc_lpmode[i:i + buf_len])
        result = self.shimmer.bluetooth_port.wait_for_ack()
        if not result:
            return result
        accel_sampling = self.shimmer.bluetooth_port.send_bluetooth(shimmer_comms_bluetooth.BtCmds.
                                                                    GET_ACCEL_LPMODE_COMMAND)
        shimmer_comms_bluetooth.ShimmerBluetooth.wait_for_ack()
        if isinstance(lsm303dlhc_lpmode, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_67_set__lsm303dlhc_accel_hrmode_command(self):
        print("Test 67 - Set LSM303DLHC accel hrmode command")
        accel_hrmode = [0x01]
        len_accel_hrmode = len(accel_hrmode)
        for i in range(0, len_accel_hrmode, 1):
            bytes_remaining = len_accel_hrmode - i
            buf_len = 1 if bytes_remaining > 1 else bytes_remaining
        self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.SET_ACCEL_HRMODE_COMMAND,
                                                    buf_len, i & 0xFF, (i >> 8) & 0xFF] +
                                                   accel_hrmode[i:i + buf_len])
        result = self.shimmer.bluetooth_port.wait_for_ack()
        if not result:
            return result
        accel_sampling = self.shimmer.bluetooth_port.send_bluetooth(shimmer_comms_bluetooth.BtCmds.
                                                                    GET_ACCEL_HRMODE_COMMAND)
        shimmer_comms_bluetooth.ShimmerBluetooth.wait_for_ack()
        if isinstance(accel_hrmode, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_68_set_mpu9150_gyro_range_command(self):
        print("Test 68 - MPU9150 Gyro Range command")
        mpu9150_gyro = [0x04]
        len_mpu9150_gyro = len(mpu9150_gyro)
        for i in range(0, len_mpu9150_gyro, 1):
            bytes_remaining = len_mpu9150_gyro - i
            buf_len = 1 if bytes_remaining > 1 else bytes_remaining
        self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.SET_GYRO_RANGE_COMMAND,
                                                    buf_len, i & 0xFF, (i >> 8) & 0xFF] +
                                                   mpu9150_gyro[i:i + buf_len])
        result = self.shimmer.bluetooth_port.wait_for_ack()
        if not result:
            return result
        mpu9150_gyro = self.shimmer.bluetooth_port.send_bluetooth(shimmer_comms_bluetooth.BtCmds.
                                                                    GET_GYRO_RANGE_COMMAND)
        shimmer_comms_bluetooth.ShimmerBluetooth.wait_for_ack()
        if isinstance(mpu9150_gyro, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_69_set__mpu9150_sampling_range_command(self):
        print("Test 69 - Set MPU9150 Sampling Range Command")
        mpu9150_sampling = [0x00, 0x01]
        len_mpu9150_sampling = len(mpu9150_sampling)
        for i in range(0, len_mpu9150_sampling, 1):
            bytes_remaining = len_mpu9150_sampling - i
            buf_len = 1 if bytes_remaining > 1 else bytes_remaining
        self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.SET_GYRO_SAMPLING_RATE_COMMAND,
                                                    buf_len, i & 0xFF, (i >> 8) & 0xFF] +
                                                   mpu9150_sampling[i:i + buf_len])
        result = self.shimmer.bluetooth_port.wait_for_ack()
        if not result:
            return result
        mpu9150_sampling = self.shimmer.bluetooth_port.send_bluetooth(shimmer_comms_bluetooth.BtCmds.
                                                                    GET_GYRO_SAMPLING_RATE_COMMAND)
        shimmer_comms_bluetooth.ShimmerBluetooth.wait_for_ack()
        if isinstance(mpu9150_sampling, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_70_set_bmp180_pres_resolution_command(self):
        print("Test 70 - Set BMP180 Pres Resolution Command ")
        # bmp180_pres_resolution = [0x02]
        # len_bmp180_pres_resolution = len(bmp180_pres_resolution)
        # for i in range(0, len_bmp180_pres_resolution, 1):
        #     bytes_remaining = len_bmp180_pres_resolution - i
        #     buf_len = 1 if bytes_remaining > 1 else bytes_remaining
        # self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.,
        #                                             buf_len, i & 0xFF, (i >> 8) & 0xFF] +
        #                                            bmp180_pres_resolution[i:i + buf_len])
        # result = self.shimmer.bluetooth_port.wait_for_ack()
        # if not result:
        #     return result
        # accel_sampling = self.shimmer.bluetooth_port.send_bluetooth(shimmer_comms_bluetooth.BtCmds.
        #                                                             GET_ACCEL_SAMPLING_RATE_COMMAND)
        # shimmer_comms_bluetooth.ShimmerBluetooth.wait_for_ack()
        # if isinstance(bmp180_pres_resolution, bool):
        #     print("Error")
        #     self.assertTrue(False)
        # print("")     //conflict in pres resolution

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
    def test_72_set_bmp180_calibration_coefficient_response(self):
         print("Test 72 - Set BMp180 Pres Calibration coefficient")
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
        # bmp280_pres_calibration_coefficient = [0x01, 0x02, 0x03, 0x00, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00,
        #                                        0x02, 0x00, 0x00, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        #                                        0x00, 0x08, 0xCD, 0x08, 0xCD]
        # len_accel_sampling_rate = len(accel_sampling)
        # for i in range(0, len_accel_sampling_rate, 1):
        #     bytes_remaining = len_accel_sampling_rate - i
        #     buf_len = 1 if bytes_remaining > 1 else bytes_remaining
        # self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.SET_ACCEL_SAMPLING_RATE_COMMAND,
        #                                             buf_len, i & 0xFF, (i >> 8) & 0xFF] +
        #                                            accel_sampling[i:i + buf_len])
        # result = self.shimmer.bluetooth_port.wait_for_ack()
        # if not result:
        #     return result
        # accel_sampling = self.shimmer.bluetooth_port.send_bluetooth(shimmer_comms_bluetooth.BtCmds.
        #                                                             GET_ACCEL_SAMPLING_RATE_COMMAND)
        # shimmer_comms_bluetooth.ShimmerBluetooth.wait_for_ack()
        # if isinstance(bmp280_pres_calibration_coefficient, bool):
        #     print("Error")
        #     self.assertTrue(False)
        # print("")

    def test_74_mpu9150_set_mag_sens_adj_vals_response(self):
        print("Test 74 - Set Mag Sens Adj Vals Response")
        # mag_sens_adj_response = [0x01, 0x02]
        # len_accel_sampling_rate = len(accel_sampling)
        # for i in range(0, len_accel_sampling_rate, 1):
        #     bytes_remaining = len_accel_sampling_rate - i
        #     buf_len = 1 if bytes_remaining > 1 else bytes_remaining
        # self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.SET_MAG,
        #                                             buf_len, i & 0xFF, (i >> 8) & 0xFF] +
        #                                            accel_sampling[i:i + buf_len])
        # result = self.shimmer.bluetooth_port.wait_for_ack()
        # if not result:
        #     return result
        # accel_sampling = self.shimmer.bluetooth_port.send_bluetooth(shimmer_comms_bluetooth.BtCmds.
        #                                                             GET_ACCEL_SAMPLING_RATE_COMMAND)
        # shimmer_comms_bluetooth.ShimmerBluetooth.wait_for_ack()
        # if isinstance(mag_sens_adj_response, bool):
        #     print("Error")
        #     self.assertTrue(False)
        # print("")

    def test_75_set_internal_exp_power_enable_command(self):
        print("Test 75 - Set Internal exp power enable command")
        internal_exp_power_enable_response = [0x04]
        len_internal_exp_power_enable_response = len(internal_exp_power_enable_response)
        for i in range(0, len_internal_exp_power_enable_response, 1):
            bytes_remaining = len_internal_exp_power_enable_response - i
            buf_len = 1 if bytes_remaining > 1 else bytes_remaining
        self.shimmer.bluetooth_port.send_bluetooth([
                                                shimmer_comms_bluetooth.BtCmds.SET_INTERNAL_EXP_POWER_ENABLE_COMMAND,
                                                    buf_len, i & 0xFF, (i >> 8) & 0xFF] +
                                                   internal_exp_power_enable_response[i:i + buf_len])
        result = self.shimmer.bluetooth_port.wait_for_ack()
        if not result:
            return result
        internal_exp_power_enable_response = self.shimmer.bluetooth_port.send_bluetooth(shimmer_comms_bluetooth.BtCmds.
                                                                    GET_INTERNAL_EXP_POWER_ENABLE_COMMAND)
        shimmer_comms_bluetooth.ShimmerBluetooth.wait_for_ack()
        if isinstance(internal_exp_power_enable_response, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_76_set_exg_regs_response(self):
        print("Test 76 - Set ExG Regs Response")
    #     exg_regs_response = [0x01, 0x00, 0x03, 0x01, 0x04, 0x06]
    #     len_exg_regs_response = len(exg_regs_response)
    # #    exg_regs_response = [(len_exg_regs_response & 0xFF), ((len_exg_regs_response >> 8) & 0xFF)] + exg_regs_response
    #     for i in range(0, len_exg_regs_response, 6):
    #         bytes_remaining = len_exg_regs_response - i
    #         buf_len = 6 if bytes_remaining > 6 else bytes_remaining
    #     self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.SET_EXG_REGS_COMMAND,
    #                                                 buf_len, i & 0xFF, (i >> 8) & 0xFF] +
    #                                                exg_regs_response[i:i + buf_len])
    #     result = self.shimmer.bluetooth_port.wait_for_ack()
    #     if not result:
    #         return result
    #     exg_regs_response = self.shimmer.bluetooth_port.send_bluetooth(shimmer_comms_bluetooth.BtCmds.
    #                                                                 GET_EXG_REGS_COMMAND)
    #     shimmer_comms_bluetooth.ShimmerBluetooth.wait_for_ack()
    #     if isinstance(exg_regs_response, bool):
    #         print("Error")
    #         self.assertTrue(False)
    #     print("")

        # no set daughter card id command - Test 77

    def test_78_set_baud_rate_command(self):
        print("Test 78 - Set baud Rate Command")
        baud_rate_response = [0x01]
        len_baud_rate_response = len(baud_rate_response)
        for i in range(0, len_baud_rate_response, 1):
            bytes_remaining = len_baud_rate_response - i
            buf_len = 1 if bytes_remaining > 1 else bytes_remaining
            self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.SET_BT_COMMS_BAUD_RATE,
                                                    buf_len, i & 0xFF, (i >> 8) & 0xFF] +
                                                   baud_rate_response[i:i + buf_len])

            result = self.shimmer.bluetooth_port.wait_for_ack()
        # # if not result:
        # #     return result
        # baud_rate_response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_BT_COMMS_BAUD_RATE,
        #                                     shimmer_comms_bluetooth.BtCmds.BT_COMMS_BAUD_RATE_RESPONSE, 1)
        #
        # self.shimmer.bluetooth_port.wait_for_ack()
        # if isinstance(baud_rate_response, bool):
        #     print("Error")
        #     self.assertTrue(False)
        print("")

    def test_79_set_derived_channel_bytes_command(self):
        print("Test 79 - Set Derived Channel bytes Command")
        derived_channel_bytes = [0x00, 0x01, 0x02]
        len_derived_channel_bytes = len(derived_channel_bytes)
        for i in range(0, len_derived_channel_bytes, 3):
            bytes_remaining = len_derived_channel_bytes - i
            buf_len = 3 if bytes_remaining > 3 else bytes_remaining
        self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.SET_DERIVED_CHANNEL_BYTES,
                                                    buf_len, i & 0xFF, (i >> 8) & 0xFF] +
                                                   derived_channel_bytes[i:i + buf_len])
        result = self.shimmer.bluetooth_port.wait_for_ack()
        if not result:
            return result
        derived_channel_bytes = self.shimmer.bluetooth_port.send_bluetooth(shimmer_comms_bluetooth.BtCmds.
                                                                    GET_DERIVED_CHANNEL_BYTES)
        shimmer_comms_bluetooth.ShimmerBluetooth.wait_for_ack()
        if isinstance(derived_channel_bytes, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_80_set_trial_config_command(self):
        print("Test 80- Set Trial Config Command")
        trial_config = [0x01, 0x02, 0x03]
        len_trial_config = len(trial_config)
        for i in range(0, len_trial_config, 3):
            bytes_remaining = len_trial_config - i
            buf_len = 3 if bytes_remaining > 3 else bytes_remaining
        self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.SET_TRIAL_CONFIG_COMMAND, buf_len,
                                                    i & 0xFF, (i >> 8) & 0xFF] +
                                                   trial_config[i:i + buf_len])
        result = self.shimmer.bluetooth_port.wait_for_ack()
        if not result:
            return result
        trial_config = self.shimmer.bluetooth_port.send_bluetooth(shimmer_comms_bluetooth.BtCmds.
                                                                  GET_TRIAL_CONFIG_COMMAND)
        shimmer_comms_bluetooth.ShimmerBluetooth.wait_for_ack()
        if isinstance(trial_config, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_81_set_center_command(self):
        print("Test 81 - Set Center Command")
        center_response = [0x01]
        len_center_response = len(center_response)
        for i in range(0, len_center_response, 1):
            bytes_remaining = len_center_response - i
            buf_len = 1 if bytes_remaining > 1 else bytes_remaining
        self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.SET_CENTER_COMMAND,
                                                    buf_len, i & 0xFF, (i >> 8) & 0xFF] +
                                                   center_response[i:i + buf_len])
        result = self.shimmer.bluetooth_port.wait_for_ack()
        if not result:
            return result
        center_response = self.shimmer.bluetooth_port.send_bluetooth(shimmer_comms_bluetooth.BtCmds.
                                                                    GET_CENTER_COMMAND)
        shimmer_comms_bluetooth.ShimmerBluetooth.wait_for_ack()
        if isinstance(center_response, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_82_set_shimmerName_command(self):
        print("Test 82 - Set Shimmer Name Command ")
        shimmer_name = [0x01]
        len_shimmer_name = len(shimmer_name)
        for i in range(0, len_shimmer_name, 1):
            bytes_remaining = len_shimmer_name - i
            buf_len = 1 if bytes_remaining > 1 else bytes_remaining
        self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.SET_SHIMMERNAME_COMMAND,
                                                    buf_len, i & 0xFF, (i >> 8) & 0xFF] +
                                                   shimmer_name[i:i + buf_len])
        result = self.shimmer.bluetooth_port.wait_for_ack()
        if not result:
            return result
        shimmer_name = self.shimmer.bluetooth_port.send_bluetooth(shimmer_comms_bluetooth.BtCmds.
                                                                    GET_SHIMMERNAME_COMMAND)
        shimmer_comms_bluetooth.ShimmerBluetooth.wait_for_ack()
        if isinstance(shimmer_name, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_83_set_ExpID_command(self):
        print("Test 83 - Set ExpId command")
        exp_id = [0x01]
        len_exp_id = len(exp_id)
        for i in range(0, len_exp_id, 1):
            bytes_remaining = len_exp_id - i
            buf_len = 1 if bytes_remaining > 1 else bytes_remaining
        self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.SET_EXPID_COMMAND, buf_len,
                                                    i & 0xFF, (i >> 8) & 0xFF] +
                                                   exp_id[i:i + buf_len])
        result = self.shimmer.bluetooth_port.wait_for_ack()
        if not result:
            return result
        exp_id = self.shimmer.bluetooth_port.send_bluetooth(shimmer_comms_bluetooth.BtCmds.GET_EXPID_COMMAND)
        shimmer_comms_bluetooth.ShimmerBluetooth.wait_for_ack()
        if isinstance(exp_id, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_84_set_myID_command(self):
        print("Test 84 - Set My ID command")
        my_id = [0x01]
        len_my_id = len(my_id)
        for i in range(0, len_my_id, 1):
            bytes_remaining = len_my_id - i
            buf_len = 1 if bytes_remaining > 1 else bytes_remaining
        self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.SET_MYID_COMMAND,buf_len, i & 0xFF,
                                                    (i >> 8) & 0xFF] + my_id[i:i + buf_len])
        result = self.shimmer.bluetooth_port.wait_for_ack()
        if not result:
            return result
        my_id = self.shimmer.bluetooth_port.send_bluetooth(shimmer_comms_bluetooth.BtCmds.GET_MYID_COMMAND)
        shimmer_comms_bluetooth.ShimmerBluetooth.wait_for_ack()
        if isinstance(my_id, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_85_set_NShimmer_command(self):
        print("Test 85- Set nShimmer command")
        shimmer_response = [0x01]
        len_shimmer_response = len(shimmer_response)
        for i in range(0, len_shimmer_response, 1):
            bytes_remaining = len_shimmer_response - i
            buf_len = 1 if bytes_remaining > 1 else bytes_remaining
        self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.SET_NSHIMMER_COMMAND,buf_len,
                                                    i & 0xFF, (i >> 8) & 0xFF] + shimmer_response[i:i + buf_len])
        result = self.shimmer.bluetooth_port.wait_for_ack()
        if not result:
            return result
        shimmer_response = self.shimmer.bluetooth_port.send_bluetooth(shimmer_comms_bluetooth.BtCmds.
                                                                      GET_NSHIMMER_COMMAND)
        shimmer_comms_bluetooth.ShimmerBluetooth.wait_for_ack()
        if isinstance(shimmer_response, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_86_set_ConfigTime_command(self):
        print("Test 86 - Set ConfigTime Command")
        config_time = [0x01]
        len_config_time = len(config_time)
        for i in range(0, len_config_time, 1):
            bytes_remaining = len_config_time - i
            buf_len = 1 if bytes_remaining > 1 else bytes_remaining
        self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.SET_CONFIGTIME_COMMAND,buf_len,
                                                    i & 0xFF, (i >> 8) & 0xFF] + config_time[i:i + buf_len])
        result = self.shimmer.bluetooth_port.wait_for_ack()
        if not result:
            return result
        config_time = self.shimmer.bluetooth_port.send_bluetooth(shimmer_comms_bluetooth.BtCmds.GET_CONFIGTIME_COMMAND)
        shimmer_comms_bluetooth.ShimmerBluetooth.wait_for_ack()
        if isinstance(config_time, bool):
            print("Error")
            self.assertTrue(False)
        print("")

        # no set dir and set instream cmd response

    def test_87_set_InfoMem_command(self):
        print("Test 87 - Set InfoMem Command")
        infomem_bytes = [0x01]
        len_infomem_bytes = len(infomem_bytes)
        for i in range(0, len_infomem_bytes, 1):
            bytes_remaining = len_infomem_bytes - i
            buf_len = 1 if bytes_remaining > 1 else bytes_remaining
        self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.SET_INFOMEM_COMMAND,buf_len,
                                                    i & 0xFF, (i >> 8) & 0xFF] +
                                                   infomem_bytes[i:i + buf_len])
        result = self.shimmer.bluetooth_port.wait_for_ack()
        if not result:
            return result
        infomem_bytes = self.shimmer.bluetooth_port.send_bluetooth(shimmer_comms_bluetooth.BtCmds.GET_INFOMEM_COMMAND)
        shimmer_comms_bluetooth.ShimmerBluetooth.wait_for_ack()
        if isinstance(infomem_bytes, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_88_set_calib_dump_command(self):
        print("Test 88 - Set Calib Command")
        calib_dump = [0x01]
        len_calib_dump = len(calib_dump)
        for i in range(0, len_calib_dump, 1):
            bytes_remaining = len_calib_dump - i
            buf_len = 1 if bytes_remaining > 1 else bytes_remaining
        self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.SET_CALIB_DUMP_COMMAND,
                                                    buf_len, i & 0xFF, (i >> 8) & 0xFF] +
                                                   calib_dump[i:i + buf_len])
        result = self.shimmer.bluetooth_port.wait_for_ack()
        if not result:
            return result
        calib_dump = self.shimmer.bluetooth_port.send_bluetooth(shimmer_comms_bluetooth.BtCmds.GET_CALIB_DUMP_COMMAND)
        shimmer_comms_bluetooth.ShimmerBluetooth.wait_for_ack()
        if isinstance(calib_dump, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_89_set_crc_command(self):
        print("Test 89 - Set CRC Command")
        crc_response = [0x01]
        len_crc_response = len(crc_response)
        for i in range(0, len_crc_response, 1):
            bytes_remaining = len_crc_response - i
            buf_len = 1 if bytes_remaining > 1 else bytes_remaining
        self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.SET_CRC_COMMAND,
                                                    buf_len, i & 0xFF, (i >> 8) & 0xFF] +
                                                   crc_response[i:i + buf_len])
        result = self.shimmer.bluetooth_port.wait_for_ack()
        if not result:
            return result
        # crc_response = self.shimmer.bluetooth_port.send_bluetooth(shimmer_comms_bluetooth.BtCmds.
        #                                                             GET_CR) //no crc command
       # shimmer_comms_bluetooth.ShimmerBluetooth.wait_for_ack()
        if isinstance(crc_response, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_90_set_rwc_command(self):
        print("Test 90 - Set RWC command")
        rwc_bytes = [0x03, 0x00, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00]
        len_rwc_bytes = len(rwc_bytes)
        for i in range(0, len_rwc_bytes, 8):
            bytes_remaining = len_rwc_bytes - i
            buf_len = 8 if bytes_remaining > 8 else bytes_remaining
        self.shimmer.bluetooth_port.send_bluetooth([shimmer_comms_bluetooth.BtCmds.SET_RWC_COMMAND,
                                                    buf_len, i & 0xFF, (i >> 8) & 0xFF] +
                                                   rwc_bytes[i:i + buf_len])
        result = self.shimmer.bluetooth_port.wait_for_ack()
        if not result:
            return result
        rwc_bytes = self.shimmer.bluetooth_port.send_bluetooth(shimmer_comms_bluetooth.BtCmds.GET_RWC_COMMAND)
        shimmer_comms_bluetooth.ShimmerBluetooth.wait_for_ack()
        if isinstance(rwc_bytes, bool):
            print("Error")
            self.assertTrue(False)
        print("")

    def test_91_UPD_calib_dump_command(self):  # not present in set
        print("Test 91 - UPD Calib Dump Command")
        # response = self.shimmer.bluetooth_port.send_bluetooth(shimmer_comms_bluetooth.BtCmds.UPD_CALIB_DUMP_COMMAND)
        # shimmer_comms_bluetooth.ShimmerBluetooth.wait_for_ack()
        print("")

    def test_92_UPD_SDlog_Cfg_command(self):
        print("Test 92 - UPD SDlog Cfg Command")
        # self.shimmer.bluetooth_port.send_bluetooth(shimmer_comms_bluetooth.BtCmds.UPD_SDLOG_CFG_COMMAND)
        # shimmer_comms_bluetooth.ShimmerBluetooth.wait_for_ack()
        print("")


if __name__ == '__main__':
    unittest.main()
