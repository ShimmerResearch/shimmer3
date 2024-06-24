import binascii
from turtle import delay

import serial
import time
import struct

import serial.win32
from serial import SerialException, Serial
from Shimmer_common import util_shimmer


class BtCmds:
    DATA_PACKET = 0x00
    INQUIRY_COMMAND = 0x01
    INQUIRY_RESPONSE = 0x02
    GET_SAMPLING_RATE_COMMAND = 0x03
    SAMPLING_RATE_RESPONSE = 0x04
    SET_SAMPLING_RATE_COMMAND = 0x05
    TOGGLE_LED_COMMAND = 0x06
    START_STREAMING_COMMAND = 0x07  # maintain compatibility with Shimmer2/2r BtStream
    SET_SENSORS_COMMAND = 0x08
    SET_ACCEL_RANGE_COMMAND = 0x09
    ACCEL_RANGE_RESPONSE = 0x0A
    GET_ACCEL_RANGE_COMMAND = 0x0B
    SET_CONFIG_SETUP_BYTES_COMMAND = 0x0E
    CONFIG_SETUP_BYTES_RESPONSE = 0x0F
    GET_CONFIG_SETUP_BYTES_COMMAND = 0x10
    SET_A_ACCEL_CALIBRATION_COMMAND = 0x11
    A_ACCEL_CALIBRATION_RESPONSE = 0x12
    GET_A_ACCEL_CALIBRATION_COMMAND = 0x13
    SET_GYRO_CALIBRATION_COMMAND = 0x14
    GYRO_CALIBRATION_RESPONSE = 0x15
    GET_GYRO_CALIBRATION_COMMAND = 0x16
    SET_MAG_CALIBRATION_COMMAND = 0x17
    MAG_CALIBRATION_RESPONSE = 0x18
    GET_MAG_CALIBRATION_COMMAND = 0x19
    SET_ACCEL_CALIBRATION_COMMAND = 0x1A
    ACCEL_CALIBRATION_RESPONSE = 0x1B
    GET_ACCEL_CALIBRATION_COMMAND = 0x1C
    STOP_STREAMING_COMMAND = 0x20  # maintain compatibility with Shimmer2/2r BtStream
    SET_GSR_RANGE_COMMAND = 0x21
    GSR_RANGE_RESPONSE = 0x22
    GET_GSR_RANGE_COMMAND = 0x23
    # DEPRECATED_GET_DEVICE_VERSION_COMMAND         = 0x24
    DEVICE_VERSION_RESPONSE = 0x25  # maintain compatibility with Shimmer2/2r BtStream
    SET_EMG_CALIBRATION_COMMAND = 0x26
    EMG_CALIBRATION_RESPONSE = 0x27
    GET_EMG_CALIBRATION_COMMAND = 0x28
    SET_ECG_CALIBRATION_COMMAND = 0x29
    ECG_CALIBRATION_RESPONSE = 0x2A
    GET_ECG_CALIBRATION_COMMAND = 0x2B
    GET_ALL_CALIBRATION_COMMAND = 0x2C
    ALL_CALIBRATION_RESPONSE = 0x2D
    GET_FW_VERSION_COMMAND = 0x2E  # maintain compatibility with Shimmer2/2r BtStream
    FW_VERSION_RESPONSE = 0x2F  # maintain compatibility with Shimmer2/2r BtStream
    SET_CHARGE_STATUS_LED_COMMAND = 0x30
    CHARGE_STATUS_LED_RESPONSE = 0x31
    GET_CHARGE_STATUS_LED_COMMAND = 0x32
    BUFFER_SIZE_RESPONSE = 0x35
    GET_BUFFER_SIZE_COMMAND = 0x36
    SET_MAG_GAIN_COMMAND = 0x37
    MAG_GAIN_RESPONSE = 0x38
    GET_MAG_GAIN_COMMAND = 0x39
    SET_MAG_SAMPLING_RATE_COMMAND = 0x3A
    MAG_SAMPLING_RATE_RESPONSE = 0x3B
    GET_MAG_SAMPLING_RATE_COMMAND = 0x3C
    UNIQUE_SERIAL_RESPONSE = 0x3D
    GET_UNIQUE_SERIAL_COMMAND = 0x3E
    GET_DEVICE_VERSION_COMMAND = 0x3F
    SET_ACCEL_SAMPLING_RATE_COMMAND = 0x40
    ACCEL_SAMPLING_RATE_RESPONSE = 0x41
    GET_ACCEL_SAMPLING_RATE_COMMAND = 0x42
    SET_ACCEL_LPMODE_COMMAND = 0x43
    ACCEL_LPMODE_RESPONSE = 0x44
    GET_ACCEL_LPMODE_COMMAND = 0x45
    SET_ACCEL_HRMODE_COMMAND = 0x46
    ACCEL_HRMODE_RESPONSE = 0x47
    GET_ACCEL_HRMODE_COMMAND = 0x48
    SET_GYRO_RANGE_COMMAND = 0x49
    GYRO_RANGE_RESPONSE = 0x4A
    GET_GYRO_RANGE_COMMAND = 0x4B
    SET_GYRO_SAMPLING_RATE_COMMAND = 0x4C
    GYRO_SAMPLING_RATE_RESPONSE = 0x4D
    GET_GYRO_SAMPLING_RATE_COMMAND = 0x4E
    SET_ALT_ACCEL_RANGE_COMMAND = 0x4F
    ALT_ACCEL_RANGE_RESPONSE = 0x50
    GET_ALT_ACCEL_RANGE_COMMAND = 0x51
    SET_PRES_OVERSAMPLING_RATIO_COMMAND = 0x52
    PRES_OVERSAMPLING_RATIO_RESPONSE = 0x53
    GET_PRES_OVERSAMPLING_RATIO_COMMAND = 0x54
    BMP180_CALIBRATION_COEFFICIENTS_RESPONSE = 0x58
    GET_BMP180_CALIBRATION_COEFFICIENTS_COMMAND = 0x59
    RESET_TO_DEFAULT_CONFIGURATION_COMMAND = 0x5A
    RESET_CALIBRATION_VALUE_COMMAND = 0x5B
    ALT_MAG_SENS_ADJ_VALS_RESPONSE = 0x5C
    GET_ALT_MAG_SENS_ADJ_VALS_COMMAND = 0x5D
    SET_INTERNAL_EXP_POWER_ENABLE_COMMAND = 0x5E
    INTERNAL_EXP_POWER_ENABLE_RESPONSE = 0x5F
    GET_INTERNAL_EXP_POWER_ENABLE_COMMAND = 0x60
    SET_EXG_REGS_COMMAND = 0x61
    EXG_REGS_RESPONSE = 0x62
    GET_EXG_REGS_COMMAND = 0x63
    SET_DAUGHTER_CARD_ID_COMMAND = 0x64
    DAUGHTER_CARD_ID_RESPONSE = 0x65
    GET_DAUGHTER_CARD_ID_COMMAND = 0x66
    SET_DAUGHTER_CARD_MEM_COMMAND = 0x67
    DAUGHTER_CARD_MEM_RESPONSE = 0x68
    GET_DAUGHTER_CARD_MEM_COMMAND = 0x69
    SET_BT_COMMS_BAUD_RATE = 0x6A
    BT_COMMS_BAUD_RATE_RESPONSE = 0x6B
    GET_BT_COMMS_BAUD_RATE = 0x6C
    SET_DERIVED_CHANNEL_BYTES = 0x6D
    DERIVED_CHANNEL_BYTES_RESPONSE = 0x6E
    GET_DERIVED_CHANNEL_BYTES = 0x6F
    START_SDBT_COMMAND = 0x70
    STATUS_RESPONSE = 0x71
    GET_STATUS_COMMAND = 0x72
    SET_TRIAL_CONFIG_COMMAND = 0x73
    TRIAL_CONFIG_RESPONSE = 0x74
    GET_TRIAL_CONFIG_COMMAND = 0x75
    SET_CENTER_COMMAND = 0x76
    CENTER_RESPONSE = 0x77
    GET_CENTER_COMMAND = 0x78
    SET_SHIMMERNAME_COMMAND = 0x79
    SHIMMERNAME_RESPONSE = 0x7a
    GET_SHIMMERNAME_COMMAND = 0x7b
    SET_EXPID_COMMAND = 0x7c
    EXPID_RESPONSE = 0x7d
    GET_EXPID_COMMAND = 0x7e
    SET_MYID_COMMAND = 0x7F
    MYID_RESPONSE = 0x80
    GET_MYID_COMMAND = 0x81
    SET_NSHIMMER_COMMAND = 0x82
    NSHIMMER_RESPONSE = 0x83
    GET_NSHIMMER_COMMAND = 0x84
    SET_CONFIGTIME_COMMAND = 0x85
    CONFIGTIME_RESPONSE = 0x86
    GET_CONFIGTIME_COMMAND = 0x87
    DIR_RESPONSE = 0x88
    GET_DIR_COMMAND = 0x89
    INSTREAM_CMD_RESPONSE = 0x8A
    SET_CRC_COMMAND = 0x8B
    SET_INFOMEM_COMMAND = 0x8C
    INFOMEM_RESPONSE = 0x8D
    GET_INFOMEM_COMMAND = 0x8E
    SET_RWC_COMMAND = 0x8F
    RWC_RESPONSE = 0x90
    GET_RWC_COMMAND = 0x91
    START_LOGGING_COMMAND = 0x92
    STOP_LOGGING_COMMAND = 0x93
    VBATT_RESPONSE = 0x94
    GET_VBATT_COMMAND = 0x95
    DUMMY_COMMAND = 0x96
    STOP_SDBT_COMMAND = 0x97
    SET_CALIB_DUMP_COMMAND = 0x98
    RSP_CALIB_DUMP_COMMAND = 0x99
    GET_CALIB_DUMP_COMMAND = 0x9A
    UPD_CALIB_DUMP_COMMAND = 0x9B
    UPD_SDLOG_CFG_COMMAND = 0x9C
    BMP280_CALIBRATION_COEFFICIENTS_RESPONSE = 0x9F
    GET_BMP280_CALIBRATION_COEFFICIENTS_COMMAND = 0xA0
    GET_BT_VERSION_STR_COMMAND = 0xA1
    BT_VERSION_STR_RESPONSE = 0xA2
    SET_INSTREAM_RESPONSE_ACK_PREFIX_STATE = 0xA3
    SET_DATA_RATE_TEST_MODE = 0xA4
    DATA_RATE_TEST_RESPONSE = 0xA5
    SET_SD_SYNC_COMMAND = 0xE0
    SD_SYNC_RESPONSE = 0xE1
    ACK_COMMAND_PROCESSED = 0xFF


class ShimmerBluetooth:
    serial_port_timeout_ms = 500
    debug_tx_rx_packets = False

    ser = None
    shimmer_device = None

    def __init__(self, shimmer):
        self.shimmer_device = shimmer

    def setup_serial_port(self, com_port, baud_rate, debug_txrx_packets=False):
        ShimmerBluetooth.debug_tx_rx_packets = debug_txrx_packets

        try:
            self.ser = serial.Serial(com_port, baud_rate, timeout=self.serial_port_timeout_ms / 1000)
            if self.ser.is_open:
                self.ser.flushInput()
                print("port opening, done.")
                return True
            else:
                print("can't open port.")
                return False
        except SerialException:
            print("Serial port exception.")
            return False

    def clear_serial_buffer(self):
        self.ser.flushInput()
        self.ser.flushOutput()

    def close_port(self):
        self.ser.close()

    def write_calibration(self, calib_bytes=None, timeout_ms=2000):

        if calib_bytes is None:
            calib_bytes = []
        len_calib_bytes = len(calib_bytes)

        # calib_bytes = [(len_calib_bytes & 0xFF), ((len_calib_bytes >> 8) & 0xFF)] + calib_bytes
        for i in range(0, len_calib_bytes, 128):
            bytes_remaining = len_calib_bytes - i
            buf_len = 128 if bytes_remaining > 128 else bytes_remaining

            if not self.send_bluetooth(
                    [BtCmds.SET_CALIB_DUMP_COMMAND, buf_len, i & 0xFF, (i >> 8) & 0xFF] + calib_bytes[i:i + buf_len]):
                return False

            if not self.wait_for_ack(timeout_ms):
                return False

        return True

    def read_calibration(self, timeout_ms=500):

        calib_dump_concat = []
        overall_mem_length = 0
        length_read_so_far = 0
        first_read = True
        while length_read_so_far == 0 or length_read_so_far < overall_mem_length:
            length_left_to_read = 128 if first_read else overall_mem_length - length_read_so_far + 2
            length_to_read = 128 if (length_left_to_read >= 128) else length_left_to_read
            address = length_read_so_far

            if not self.send_bluetooth([BtCmds.GET_CALIB_DUMP_COMMAND, length_to_read, address & 0xFF,
                                        (address >> 8) & 0xFF]):
                return False

            if not self.wait_for_ack(timeout_ms):
                return False

            rsp_byte = self.wait_for_response(1)
            if rsp_byte[0] != BtCmds.RSP_CALIB_DUMP_COMMAND:
                return False

            # +3 for 1 length byte followed byte 2 bytes address
            rx_bytes = self.wait_for_response(length_to_read + 3, timeout_ms)
            if len(rx_bytes) == 0:
                return False

            calib_dump_concat += rx_bytes[3:len(rx_bytes)]

            if first_read:
                overall_mem_length = (calib_dump_concat[1] << 8) | calib_dump_concat[0]
                first_read = False

            length_read_so_far += length_to_read

        return calib_dump_concat

    def write_configuration(self, tx_bytes=None, timeout_ms=500):

        if tx_bytes is None:
            tx_bytes = []
        len_tx_bytes = len(tx_bytes)

        for i in range(0, len_tx_bytes, 128):
            bytes_remaining = len_tx_bytes - i
            buf_len = 128 if bytes_remaining > 128 else bytes_remaining

            if not self.send_bluetooth(
                    [BtCmds.SET_INFOMEM_COMMAND,
                     buf_len,
                     i & 0xFF, (i >> 8) & 0xFF] + tx_bytes[i:i + buf_len]):
                return False

            if not self.wait_for_ack(timeout_ms):
                return False

        return True

    def read_configuration(self, timeout_ms=500):

        len_config_bytes = 384
        config_bytes = []
        for i in range(0, len_config_bytes, 128):
            bytes_remaining = len_config_bytes - i
            buf_len = 128 if bytes_remaining > 128 else bytes_remaining

            if not self.send_bluetooth([BtCmds.GET_INFOMEM_COMMAND, buf_len, i & 0xFF, (i >> 8) & 0xFF]):
                return False

            if not self.wait_for_ack(timeout_ms):
                return False

            rx_bytes = self.wait_for_response(buf_len + 2, timeout_ms)
            if len(rx_bytes) == 0:
                return False

            config_bytes += rx_bytes[2:len(rx_bytes)]

        return config_bytes

    def wait_for_ack(self, timeout_ms=500):
        response = self.wait_for_response(1, timeout_ms)
        return True if response[0] is BtCmds.ACK_COMMAND_PROCESSED else False

    def wait_for_response(self, expected_len, timeout_ms=500):
        flag = True

        loop_count = 0
        wait_interval_ms = 100
        loop_count_total = timeout_ms / wait_interval_ms

        rx_buf = []

        while flag:
            time.sleep(wait_interval_ms / 1000)
            loop_count += 1
            if loop_count >= loop_count_total:
                print("Timeout while waiting for response")
                break

            buf_len = self.ser.inWaiting()
            if buf_len >= expected_len:
                data_read = self.ser.read(expected_len)
                if isinstance(data_read, str):
                    rx_buf += bytearray.fromhex(binascii.hexlify(data_read))
                else:
                    rx_buf += data_read

                if self.debug_tx_rx_packets:
                    print("UART RX: " + util_shimmer.byte_array_to_hex_string(rx_buf))

                break

        return rx_buf

    def send_bluetooth(self, tx_buf):
        if self.debug_tx_rx_packets:
            print("UART TX: " + util_shimmer.byte_array_to_hex_string(tx_buf))
        self.ser.write(tx_buf)
        time.sleep(0.1)
        return True
