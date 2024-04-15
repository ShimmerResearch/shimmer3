import serial.tools.list_ports

from Shimmer_common import shimmer_comms_docked, shimmer_comms_bluetooth


def serial_ports_shimmer_dock():
    serial_port_list = serial_ports()
    serial_port_list_filtered = []
    for item in serial_port_list:
        if "USB Serial Port" in item.description \
                and (("PID=0403:6010" in item.hwid and item.hwid.endswith("B"))
                     or ("PID=0403:6011" in item.hwid and item.hwid.endswith("D"))):
            serial_port_list_filtered += [item]
    return serial_port_list_filtered


def serial_ports_bluetooth():
    serial_port_list = serial_ports()
    serial_port_list_filtered = []
    for item in serial_port_list:
        if "Bluetooth" in item.description:
            serial_port_list_filtered += [item]
    return serial_port_list_filtered


def serial_ports():
    ports = serial.tools.list_ports.comports()
    # for port, desc, hwid in sorted(ports):
    #     print("{}: {} [{}]".format(port, desc, hwid))
    return ports


class Shimmer3:
    mac_id = None

    hw_ver = None
    fw_id = None
    fw_ver_major = None
    fw_ver_minor = None
    fw_ver_internal = None

    daughter_card_id = None
    daughter_card_rev_major = None
    daughter_card_rev_minor = None

    bluetooth_ver_str = None

    batt_adc_value = None
    charging_status = None

    def __init__(self):
        self.dock_port = shimmer_comms_docked.ShimmerUart(self)
        self.bluetooth_port = shimmer_comms_bluetooth.ShimmerBluetooth(self)

    def setup_dock_com_port(self, com_port, debug_txrx_packets=False):
        return self.dock_port.setup_serial_port(com_port, 115200, debug_txrx_packets)

    def setup_bluetooth_com_port(self, com_port, debug_txrx_packets=False):
        return self.bluetooth_port.setup_serial_port(com_port, 1000000, debug_txrx_packets)

    def parse_hw_fw_ver_bytes(self, byte_buf):
        index = 0
        if len(byte_buf) == 7:
            self.hw_ver = byte_buf[index]
            index += 1
        elif len(byte_buf) == 8:
            self.hw_ver = (byte_buf[index] | (byte_buf[index + 1] << 8))
            index += 2

        self.fw_id = ((byte_buf[index]) | (byte_buf[index + 1] << 8))
        index += 2
        self.fw_ver_major = ((byte_buf[index]) | (byte_buf[index + 1] << 8))
        index += 2
        self.fw_ver_minor = byte_buf[index]
        index += 1
        self.fw_ver_internal = byte_buf[index]

    def parse_daughter_card_id(self, byte_buf):
        self.daughter_card_id = byte_buf[0]
        self.daughter_card_rev_major = byte_buf[1]
        self.daughter_card_rev_minor = byte_buf[2]

    def parse_infomem(self, byte_buf):
        # TODO
        return

    def set_bluetooth_ver_str(self, byte_buf):
        self.bluetooth_ver_str = bytearray(byte_buf).decode("ASCII")

    def print_hw_fw_revision(self):
        print("HW Revision: " + str(self.hw_ver))
        print("FW Revision: ID=" + str(self.fw_id)
              + ", v" + str(self.fw_ver_major)
              + "." + str(self.fw_ver_minor)
              + "." + str(self.fw_ver_internal))

    def print_daughter_card_id(self):
        print("SR" + str(self.daughter_card_id)
              + "." + str(self.daughter_card_rev_major)
              + "." + str(self.daughter_card_rev_minor))

    def print_batt_status(self):
        print("ADC Value=" + str(self.batt_adc_value) + ", Charging status=" + str(self.charging_status))
