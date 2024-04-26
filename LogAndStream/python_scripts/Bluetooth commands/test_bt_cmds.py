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

    def test_xx_set_calibration(self):
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

    def test_04_get_config_setup_bytes(self):
        print("Test 04- Get Config Setup Bytes command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_CONFIG_SETUP_BYTES_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.CONFIG_SETUP_BYTES_RESPONSE,1)

        print("")

    def test_05_get_accel_calibration_command(self):
        print("Test 05 - Get Accel Calibration Command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_A_ACCEL_CALIBRATION_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.A_ACCEL_CALIBRATION_RESPONSE,1)
        print("")

    def test_06_get_gyro_calibration_command(self):
        print("Test 06 - Get Gyro Calibration Command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_GYRO_CALIBRATION_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.GYRO_CALIBRATION_RESPONSE,1)

        print("")

    def test_07_get_mag_calibration_command(self):
        print("Test 07 - Get Mag Calibration Command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_MAG_CALIBRATION_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.MAG_CALIBRATION_RESPONSE,1)
        print("")

    # def test_08_get_LSM303DLHC_mag_calibration_command(self):
    #     print("Test 08 - Get Mag Calibration Command:")
    #     response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.) #BT command not present

    def test_09_get_gsr_range_command(self):
        print("Test 08 - Get GSR Command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_GSR_RANGE_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.GSR_RANGE_RESPONSE,21)
        print("")
    def test_10_get_EMG_calibration_response(self):
        print("Test 10 - Get EMG Calibration response command") #no BT command for this
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.E)


    def test_11_all_calibration_response(self):
        print("Test 11 - Get all calibration response command")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_ALL_CALIBRATION_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.ALL_CALIBRATION_RESPONSE,4)
        print("")


    def test_13_get_blink_led_response(self):
        print("Test 13 - Blink LED response command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_TOGGLE_LED_COMMAND,   #no blink led command
                                           shimmer_comms_bluetooth.BtCmds.TOGGLE_LED_COMMAND,1)

    def test_14_get_buffer_size_command(self):
        print("Test 14 - Buffer size repsonse Command: ")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_BUFFER_SIZE_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.BUFFER_SIZE_RESPONSE, ) #get buffer size or response as 2 commands are mentioned
        print("")

    def test_15_get_lsm303DHC_mag_gain_command(self):
        print("Test 15 - Get LSM303DLHC mag gain command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_MAG_GAIN_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.MAG_GAIN_RESPONSE, 8)
        print("")

    def test_16_get_charge_status_led_response(self):
        print("Test 16 - Get Charge Status LED response command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_CHARGE_STATUS_LED_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.CHARGE_STATUS_LED_RESPONSE, 3)
        print("")

    def  test_17_get_shimmer_version_command_response(self):
        print("Test 17 - Get Shimmer Version response command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_SHIMMERNAME_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.SHIMMERNAME_RESPONSE,1) #is version and name same ?
        print("")

    def test_18_get_shimmer_new_version_command_response(self):
        print("Test 18 - Get new Shimmer version response command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_NSHIMMER_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.NSHIMMER_RESPONSE, -1)
        print("")

    # def test_19_get_ecg_calibration_command_response(self):
    #     print("Test 19 - Get ECG Calibration response command:")
    #     response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.) # no ecg in BT commands

    def test_20_get_mag_sampling_command(self):
        print("Test 20 - Get Mag sampling Command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_MAG_SAMPLING_RATE_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.MAG_SAMPLING_RATE_RESPONSE,1)
        print("")

    def test_21_get_accel_sampling_rate_command(self):
        print("Test 21 - Get accel sampling rate response command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_ACCEL_SAMPLING_RATE_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.ACCEL_SAMPLING_RATE_RESPONSE,1)
        print("")

    def test_22_get_lsm303dlhc_accel_lpmode_command(self):
        print("Test 22 - Get LSM303DLHC accel lpmode command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_ACCEL_LPMODE_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.ACCEL_LPMODE_RESPONSE,1)
        print("")

    def test_23_get_lsm303dlhc_accel_hrmode_command(self):
        print("Test 23 - GET LSM303DLHC accel hrmode command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_ACCEL_HRMODE_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.ACCEL_HRMODE_RESPONSE)
        print("")

    def test_24_get_MPU9150_Gyro_range_command(self):
        print("Test 24 - Get MPU9150 Gyro range command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_GYRO_RANGE_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.GYRO_RANGE_RESPONSE,1)
        print("")

    def test_25_get_MPU9150_sampling_rate_command(self):
        print("Test 24 - Get MPU9150 Sampling rate command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_SAMPLING_RATE_COMMAND, #works
                                           shimmer_comms_bluetooth.BtCmds.SAMPLING_RATE_RESPONSE,1)
        print("")




    def test_28_get_BMP180_Calibration__coefficients_command(self):
        print("Test 24 - Get BMP180 Calibration response command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_BMP180_CALIBRATION_COEFFICIENTS_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.BMP180_CALIBRATION_COEFFICIENTS_RESPONSE,22)
        print("") #works
#
    def test_29_get_BMP280_Calibration_coefficients_command(self):
        print("Test 24 - Get BMP280 Caibration coefficient response command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_BMP280_CALIBRATION_COEFFICIENTS_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.BMP280_CALIBRATION_COEFFICIENTS_RESPONSE,24)
        print("") #works

 #bmp280
#
    def test_31_get_internal_exp_power_enable_command(self):
        print("Test 24 - Get exp power enable response command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_INTERNAL_EXP_POWER_ENABLE_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.INTERNAL_EXP_POWER_ENABLE_RESPONSE,1)
        print("")
#
    def test_32_get_exg_regs_command(self):
        print("Test 24 - Get ExG Regs command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_EXG_REGS_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.EXG_REGS_RESPONSE,11)
        print("")
#
    def test_33_get_daughter_card_id_command(self):
        print("Test 24 - Get daughter card id command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_DAUGHTER_CARD_ID_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.DAUGHTER_CARD_ID_RESPONSE,4) #3+1 mentioned
        print("")

    def test_34_get_baud_rate_command(self):
        print("Test 24 - Get baud rate response command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_BT_COMMS_BAUD_RATE,
                                           shimmer_comms_bluetooth.BtCmds.BT_COMMS_BAUD_RATE_RESPONSE,1)
        print("")

    def test_35_get_derived_channel_bytes_command(self):
        print("Test 24 - Get Derived Channel Bytes response command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_DERIVED_CHANNEL_BYTES,
                                           shimmer_comms_bluetooth.BtCmds.DERIVED_CHANNEL_BYTES_RESPONSE,3)
        print("")
#
    def test_36_get_trial_config_command(self):
        print("Test 24 - Get trial config command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_TRIAL_CONFIG_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.TRIAL_CONFIG_RESPONSE,3)
        print("")

    def test_37_get_center_command(self):
        print("Test 24 - Get Center response command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_CENTER_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.CENTER_RESPONSE,1)
        print("")

    def test_38_get_Shimmername_command(self):
        print("Test 24 - Get ShimmerName command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_SHIMMERNAME_COMMAND,
                                       shimmer_comms_bluetooth.BtCmds.SHIMMERNAME_RESPONSE, 1)
        print("")

    def test_39_get_expID_command(self):
        print("Test 24 - Get MPU9150 Gyro range command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_EXPID_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.EXPID_RESPONSE,1)
        print("")


    def test_40_get_myID_command(self):
        print("Test 24 - Get MPU9150 Gyro range command:")  #works
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_MYID_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.MYID_RESPONSE, 1)
        print("")

#
    def test_41_NSHIMMER_command(self):
        print("Test 24 - Get nshimmer response command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_NSHIMMER_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.NSHIMMER_RESPONSE,1)
        print("")
#
    def test_42_get_ConfigTime_command(self):
        print("Test 42- Get Config Time Response Command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_CONFIGTIME_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.CONFIGTIME_RESPONSE,1)
        print("")

    def test_43_get_dir_command(self):
        print("Test 43 - Get dir response command:")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_DIR_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.DIR_RESPONSE,1)
        print("")

    # def test_44_get_infomem_command(self):
    #     print("Test 44:Get infomem response command:")
    #     response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_INFOMEM_COMMAND,
    #                                        shimmer_comms_bluetooth.BtCmds.INFOMEM_RESPONSE,1)
    #     print("") //bluetooth command not present

    def test_45_get_rwc_command(self):
        print("Test 45- Get RWC response command")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_RWC_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.RWC_RESPONSE,1)
        print("")

    def test_46_get_vbatt_command(self):
        print("Test 46 - Get VBatt response command")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_VBATT_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.VBATT_RESPONSE,1)
        print("")

    def test_47_get_BT_FW_VERSION_command(self):
        print("Test 47 - Get BT FW Version response command")
        response = self.bt_cmd_test_common(shimmer_comms_bluetooth.BtCmds.GET_BT_VERSION_STR_COMMAND,
                                           shimmer_comms_bluetooth.BtCmds.BT_VERSION_STR_RESPONSE,1)
        print("")
#
# #set commands
#
#     def test_48_set_sensors_command(self):
#         print("Test 48: Set sensor response command")
#         print("")
#
#     def test_49_set_accel_sensitivity_command(self):
#         print("Test 49 - set accel sensitivity command ")
#         print("")
#
#     def test_50_set_config_byte0_command(self):
#         print("Test 50- ")
#         print("")
#
#     def test_51_set_accel_calibration_command(self):
#         print("Test 51 - ")
#         print("")
#
#     def test_52_set_lsm303dlhc_accel_calibration_command(self):
#         print("Test 52 - ")
#         print("")
#
#     def test_53_set_gyro_calibration_command(self):
#         print("Test 53 - ")
#         print("")
#
#     def test_54_set_mag_calibration_command(self):
#         print("Test 54 - ")
#         print("")
#
#     def test_55_set_mag_calibration_command(self):
#         print("Test 55 - ")
#         print("")
#
#     def test_56_set_gsr_range_command(self):
#         print("Test 56")
#         print("")
#
#     def test_57_set_emg_calibration_command(self):
#         print("Test 57")
#         print("")
#
#     def test_58_set_ecg_calobration_command(self):
#         print("Test 58")
#         print("")
#
#     def test_59_set_blink_led(self):
#         print("Test 59")
#         print("")
#
#     def test_60_set_gyro_temp_vref_command(self):
#         print("Test 60")
#         print("")
#
#     def test_61_set_buffer_size_command(self):
#         print("test 61")
#         print("")
#
#     def test_62_set_buffer_size_command(self):
#         print("Test 62")
#         print("")
#
#     def test_63_set_mag_gain_command(self):
#         print("Test 63")
#         print("")
#
#     def test_64_set_mag_sampling_rate_command(self):
#         print("Test 64")
#         print("")
#
#     def test_65_set_accel_sampling_rate_command(self):
#         print("Test 65")
#         print("")
#
#     def test_66_set__lsm303dlhc_accel_lpmode_command(self):
#         print("Test 66")
#         print("")
#
#     def test_67_set__lsm303dlhc_accel_hrmode_command(self):
#         print("Test 67")
#         print("")
#
#     def test_68_set__mpu9150_gyro_range_command(self):
#         print("Test 68")
#         print("")
#
#     def test_69_set__mpu9150_sampling_range_command(self):
#         print("Test 69")
#         print("")
#
#     def test_70_set__bmp180_pres_resolution_command(self):
#         print("Test 70")
#         print("")
#
#     def test_71_set__bmp180_pres_range_command(self):
#         print("Test 71")
#         print("")
#
#     def test_72_set_bmp180_pres_calibration_command(self):
#         print("Test 72")
#
#     def test_73_set_internal_exp_power_enable_command(self):
#         print("Test 73")
#         print("")
#
#     def test_74_set_exg_regs_command(self):
#         print("Test 74")
#         print("")
#
#     def test_75_set__baud_rate_command(self):
#         print("Test 75")
#         print("")
#
#     def test_76_set_derived_channel_bytes_command(self):
#         print("Test 76")
#         print("")
#
#     def test_77_set_trial_config_command(self):
#         print("Test 77")
#         print("")
#
#     def test_78_set_center_command(self):
#         print("Test 78")
#         print("")
#
#     def test_79_set_shimmerName__command(self):
#         print("Test 79")
#         print("")
#
#     def test_80_set_ExpID_command(self):
#         print("Test 80")
#         print("")
#
#     def test_81_set_myID_command(self):
#         print("Test 81")
#         print("")
#
#     def test_82_set_NShimmer_command(self):
#         print("Test 82")
#         print("")
#
#     def test_83_set_ConfigTime_command(self):
#         print("Test 83")
#         print("")
#
#     def test_84_set_InfoMem_command(self):
#         print("Test 84")
#         print("")
#
#     def test_85_set_calib_command(self):
#         print("Test 85")
#         print("")
#
#     #should i include udp commands ?
#
#     def test_86_set_crc_command(self):
#         print("Test 86")
#         print("")
#
#     def test_87_set_rwc_command(self):
#         print("Test 87")
#         print("")









if __name__ == '__main__':
    unittest.main()
