import binascii
import time

import serial
from serial import SerialException

import shimmer_crc as shimmerCrc
import util_shimmer_time

PACKET_HEADER = 0x24  # "$"


class UartPacketCmd:
    WRITE = 0x01
    DATA_RESPONSE = 0x02
    READ = 0x03

    BAD_CMD_RESPONSE = 0xFC
    BAD_ARG_RESPONSE = 0xFD
    BAD_CRC_RESPONSE = 0xFE
    ACK_RESPONSE = 0xFF


class UartComponent:
    MAIN_PROCESSOR = 0x01
    BAT = 0x02
    DAUGHTER_CARD = 0x03
    # PPG = 0x04
    # GSR = 0x05
    # LSM303DLHC_ACCEL = 0x06
    # MPU9X50_ACCEL = 0x07
    # BEACON = 0x08
    # RADIO_802154 = 0x09
    RADIO_BLUETOOTH = 0x0A


class UartProperty:
    class MainProcessor:
        # ENABLE = 0x00  # Not in use
        # SAMPLE_RATE = 0x01  # Not in use
        MAC = 0x02
        VER = 0x03
        RTC_CFG_TIME = 0x04
        CURR_LOCAL_TIME = 0x05
        INFOMEM = 0x06
        # LED0_STATE = 0x07  # Used by Shimmer SPAN
        # DEVICE_BOOT = 0x08  # Not in use

    class Bat:
        # ENABLE = 0x00  # Not in use
        VALUE = 0x02
        # FREQ_DIVIDER = 0x06  # Not in use

    class DaughterCard:
        CARD_ID = 0x02
        CARD_MEM = 0x03

    class Bluetooth:
        VER = 0x03


def byte_array_to_hex_string(data):
    hex_str = ""
    first_byte = True
    for b in data:
        if not first_byte:
            hex_str += ' '
        hex_str += "0x%0.2X" % b
        first_byte = False
    return '[' + hex_str + ']'


def byte_array_to_int(data, lsb_order=True, is_signed=False):
    number = 0
    i = 0

    for b in data:
        if lsb_order:
            number += (b << i * 8)
        else:
            number = (number << 8) + b
        i += 1

    # two's compliment
    if is_signed:
        bit_length = len(data) * 8
        # sign_offset = 2 ** (bit_length - 1)
        # number -= sign_offset
        # Copied from Java implementation but I'm not sure if the maths is exactly correct
        if number >= (1 << (bit_length - 1)):
            number = -((number ^ ((2 ** bit_length) - 1)) + 1)

    # print('data={}, lsb_order={}, is_signed={}, number={}'.format(byte_array_to_hex_string(data), lsb_order, is_signed, number))

    return number


def assemble_tx_packet(uart_cmd=None, uart_component=None, uart_property=None, uart_args=None):
    if uart_args is None:
        packet = [PACKET_HEADER, uart_cmd, 2, uart_component, uart_property]
    else:
        packet = [PACKET_HEADER, uart_cmd, 2 + len(uart_args), uart_component, uart_property] + uart_args

    in_crc = [0, 0]
    in_crc_raw = shimmerCrc.calc_crc(len(packet), packet)
    in_crc[0] = in_crc_raw & 0xff
    in_crc[1] = in_crc_raw >> 8
    tx_buf = packet + in_crc

    return tx_buf


class ShimmerUart:
    serial_port_timeout_ms = 500
    debug_tx_rx_packets = False

    ser = None
    shimmer_device = None

    def __init__(self, shimmer):
        self.shimmer_device = shimmer

    def setup_serial_port(self, com_port, baud_rate, debug_txrx_packets=False):
        ShimmerUart.debug_tx_rx_packets = debug_txrx_packets

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

    def read_mac_id(self):
        tx_buf = assemble_tx_packet(UartPacketCmd.READ, UartComponent.MAIN_PROCESSOR,
                                    UartProperty.MainProcessor.MAC, None)
        response = self.send_uart(tx_buf)

        if len(response) >= 6:
            self.shimmer_device.mac_id = ''.join('%02x' % i for i in response).upper()
            return True
        else:
            return False

    def read_hw_fw_ver(self):
        tx_buf = assemble_tx_packet(UartPacketCmd.READ, UartComponent.MAIN_PROCESSOR,
                                    UartProperty.MainProcessor.VER, None)
        response = self.send_uart(tx_buf)

        if len(response) < 7:
            return False
        else:
            self.shimmer_device.parse_hw_fw_ver_bytes(response)
            return True

    def read_bluetooth_ver(self):
        tx_buf = assemble_tx_packet(UartPacketCmd.READ, UartComponent.RADIO_BLUETOOTH,
                                    UartProperty.Bluetooth.VER, None)
        response = self.send_uart(tx_buf)

        if len(response) == 0:
            return False
        else:
            self.shimmer_device.set_bluetooth_ver_str(response)
            return True

    def read_batt(self):
        tx_buf = assemble_tx_packet(UartPacketCmd.READ, UartComponent.BAT, UartProperty.Bat.VALUE, None)
        response = self.send_uart(tx_buf)

        if len(response) < 3:
            return False
        else:
            self.shimmer_device.batt_adc_value = byte_array_to_int(response[0:2])
            self.shimmer_device.charging_status = response[2] & 0xC0
            return True

    def write_real_world_clock_from_pc_time(self):
        ts_ms = time.time()
        return self.write_real_world_clock(ts_ms)

    def write_real_world_clock(self, ts_ms):
        time_bytes = util_shimmer_time.ms_to_shimmer_rtc_bytes(ts_ms)
        tx_buf = assemble_tx_packet(UartPacketCmd.WRITE, UartComponent.MAIN_PROCESSOR,
                                    UartProperty.MainProcessor.RTC_CFG_TIME, time_bytes)
        response = self.send_uart(tx_buf)
        return response == UartPacketCmd.ACK_RESPONSE

    def read_real_world_clock_config_time(self):
        tx_buf = assemble_tx_packet(UartPacketCmd.READ, UartComponent.MAIN_PROCESSOR,
                                    UartProperty.MainProcessor.RTC_CFG_TIME, None)
        response = self.send_uart(tx_buf)

        if len(response) >= 8:
            ts_ticks = byte_array_to_int(response)
            ts_ms = ts_ticks / 32.768
            return ts_ms
        else:
            return False

    def read_current_time(self):
        tx_buf = assemble_tx_packet(UartPacketCmd.READ, UartComponent.MAIN_PROCESSOR,
                                    UartProperty.MainProcessor.CURR_LOCAL_TIME, None)
        response = self.send_uart(tx_buf)

        if len(response) >= 8:
            ts_ticks = byte_array_to_int(response)
            ts_ms = ts_ticks / 32.768
            return ts_ms
        else:
            return False

    def read_daughter_card_id(self):
        response = self.get_mem_command(UartComponent.DAUGHTER_CARD, UartProperty.DaughterCard.CARD_ID, 0, 16)

        if isinstance(response, bool):
            return response
        else:
            self.shimmer_device.parse_daughter_card_id(response)
            return True

    def write_daughter_card_id(self, daughter_card_id, daughter_card_rev_major, daughter_card_rev_minor):
        tx_buf = [0xFF] * 16
        tx_buf[0] = daughter_card_id & 0xFF
        tx_buf[1] = daughter_card_rev_major & 0xFF
        tx_buf[2] = daughter_card_rev_minor & 0xFF

        response = self.set_mem_command(UartComponent.DAUGHTER_CARD, UartProperty.DaughterCard.CARD_ID, 0, tx_buf)

        if isinstance(response, bool):
            return response
        else:
            self.shimmer_device.parse_daughter_card_id(response)
            return True

    def read_daughter_card_mem(self):
        response = self.get_mem_command(UartComponent.DAUGHTER_CARD, UartProperty.DaughterCard.CARD_MEM, 0, 128)
        return response

    def write_daughter_card_mem(self, byte_buf):
        response = self.set_mem_command(UartComponent.DAUGHTER_CARD, UartProperty.DaughterCard.CARD_MEM, 0, byte_buf)
        return response

    def read_infomem(self):
        mem_d = self.read_infomem_d()
        if isinstance(mem_d, bool):
            return mem_d
        mem_c = self.read_infomem_c()
        if isinstance(mem_c, bool):
            return mem_c
        mem_b = self.read_infomem_b()
        if isinstance(mem_b, bool):
            return mem_b

        response = mem_d + mem_c + mem_b

        if isinstance(response, bool):
            return response
        else:
            self.shimmer_device.parse_infomem(response)

        return response

    def read_infomem_d(self):
        response = self.get_mem_command(UartComponent.MAIN_PROCESSOR, UartProperty.MainProcessor.INFOMEM, 0, 128)
        return response

    def read_infomem_c(self):
        response = self.get_mem_command(UartComponent.MAIN_PROCESSOR, UartProperty.MainProcessor.INFOMEM, 128, 128)
        return response

    def read_infomem_b(self):
        response = self.get_mem_command(UartComponent.MAIN_PROCESSOR, UartProperty.MainProcessor.INFOMEM, 256, 128)
        return response

    def write_infomem(self, byte_buf):
        result = self.write_infomem_d(byte_buf[0:128])
        if not result:
            return result
        result = self.write_infomem_c(byte_buf[128:256])
        if not result:
            return result
        result = self.write_infomem_b(byte_buf[256:384])
        return result

    def write_infomem_d(self, byte_buf):
        response = self.set_mem_command(UartComponent.MAIN_PROCESSOR, UartProperty.MainProcessor.INFOMEM, 0, byte_buf)
        return response

    def write_infomem_c(self, byte_buf):
        response = self.set_mem_command(UartComponent.MAIN_PROCESSOR, UartProperty.MainProcessor.INFOMEM, 128, byte_buf)
        return response

    def write_infomem_b(self, byte_buf):
        response = self.set_mem_command(UartComponent.MAIN_PROCESSOR, UartProperty.MainProcessor.INFOMEM, 256, byte_buf)
        return response

    def get_mem_command(self, uart_component, uart_property, address, size):
        args = [size & 0xFF]
        if uart_component == UartComponent.DAUGHTER_CARD and uart_property == UartProperty.DaughterCard.CARD_ID:
            args.append(address & 0xFF)
        else:
            args.append(address & 0xFF)
            args.append((address >> 8) & 0xFF)

        tx_buf = assemble_tx_packet(UartPacketCmd.READ, uart_component, uart_property, args)
        response = self.send_uart(tx_buf)

        if len(response) > 0:
            return response
        else:
            return False

    def set_mem_command(self, uart_component, uart_property, address, byte_buf):
        args = [len(byte_buf) & 0xFF]
        if uart_component == UartComponent.DAUGHTER_CARD and uart_property == UartProperty.DaughterCard.CARD_ID:
            args.append(address & 0xFF)
        else:
            args.append(address & 0xFF)
            args.append((address >> 8) & 0xFF)

        args += byte_buf

        tx_buf = assemble_tx_packet(UartPacketCmd.WRITE, uart_component, uart_property, args)
        response = self.send_uart(tx_buf)

        if isinstance(response, bool):
            return response
        else:
            return True

    def wait_for_response(self):
        flag = True

        loop_count = 0
        wait_interval_ms = 100
        loop_count_total = self.serial_port_timeout_ms / wait_interval_ms

        rx_buf = []

        while flag:
            time.sleep(wait_interval_ms / 1000)
            loop_count += 1
            if loop_count >= loop_count_total:
                break

            buf_len = self.ser.inWaiting()
            if buf_len > 0:
                data_read = self.ser.read(buf_len)
                if isinstance(data_read, str):
                    rx_buf += bytearray.fromhex(binascii.hexlify(data_read))
                else:
                    rx_buf += data_read

                if rx_buf[0] == PACKET_HEADER:
                    if rx_buf[1] == UartPacketCmd.ACK_RESPONSE \
                            or rx_buf[1] == UartPacketCmd.BAD_CMD_RESPONSE \
                            or rx_buf[1] == UartPacketCmd.BAD_ARG_RESPONSE \
                            or rx_buf[1] == UartPacketCmd.BAD_CRC_RESPONSE:
                        if self.debug_tx_rx_packets:
                            print("UART RX: " + byte_array_to_hex_string(rx_buf))
                        rx_buf = rx_buf[1]
                        break

                    if buf_len > 3:
                        if len(rx_buf) == rx_buf[2] + 5:
                            if shimmerCrc.crc_check(len(rx_buf), rx_buf) is not True:
                                print("CRC fail")
                            else:
                                if self.debug_tx_rx_packets:
                                    print("UART RX: " + byte_array_to_hex_string(rx_buf))

                                rx_buf = rx_buf[5:len(rx_buf) - 2]
                            break
                else:
                    # TODO
                    print("RX: Invalid Response")
                    return

        return rx_buf

    def send_uart(self, tx_buf):
        if self.debug_tx_rx_packets:
            print("UART TX: " + byte_array_to_hex_string(tx_buf))
        self.ser.write(tx_buf)
        time.sleep(0.1)
        response = self.wait_for_response()
        return response
