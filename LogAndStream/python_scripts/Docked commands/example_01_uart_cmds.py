#!/usr/bin/python
import shimmer_app_common
import shimmer_device
import shimmer_uart
import util_shimmer_time

com_port = shimmer_app_common.get_selected_com_port()
if not com_port:
    print("Supported COM port not found, exiting")
    exit()

shimmer = shimmer_device.Shimmer3()

if not shimmer.setup_dock_com_port(com_port, debug_txrx_packets=True):
    exit()

print("Read MAC ID:")
if not shimmer.dock_port.read_mac_id():
    print("Error, exiting")
    exit()
print(shimmer.mac_id)
print("")

print("Read HW/FW version:")
if not shimmer.dock_port.read_hw_fw_ver():
    print("Error, exiting")
    exit()
shimmer.print_hw_fw_revision()
print("")

print("Read Bluetooth version:")
if not shimmer.dock_port.read_bluetooth_ver():
    # Not all firmware versions support this command
    print("Error")
print(shimmer.bluetooth_ver_str)
print("")

print("Read battery:")
if not shimmer.dock_port.read_batt():
    print("Error, exiting")
    exit()
shimmer.print_batt_status()
print("")

print("Write time from PC:")
if not shimmer.dock_port.write_real_world_clock_from_pc_time():
    print("Error, exiting")
    exit()
print("Success")
print("")

print("Read real-time-clock config time:")
ts_ms = shimmer.dock_port.read_real_world_clock_config_time()
if isinstance(ts_ms, bool):
    print("Error, exiting")
    exit()
print(util_shimmer_time.seconds_to_time_str(ts_ms / 1000, True))
print("")

print("Read current time:")
ts_ms = shimmer.dock_port.read_current_time()
if isinstance(ts_ms, bool):
    print("Error, exiting")
    exit()
print(util_shimmer_time.seconds_to_time_str(ts_ms / 1000, True))
print("")

print("Read Daughter Card ID:")
if shimmer.dock_port.read_daughter_card_id():
    shimmer.print_daughter_card_id()
else:
    print("No expansion board detected")
print("")

print("Write Daughter Card ID:")
if shimmer.dock_port.write_daughter_card_id(shimmer.daughter_card_id, shimmer.daughter_card_rev_major,
                                            shimmer.daughter_card_rev_minor) \
        and shimmer.dock_port.read_daughter_card_id():
    shimmer.print_daughter_card_id()
else:
    print("No expansion board detected")
print("")

print("Read Daughter Card Mem:")
eeprom_bytes_backup = None
eeprom_bytes = shimmer.dock_port.read_daughter_card_mem()
if isinstance(eeprom_bytes, bool):
    print("No expansion board detected")
else:
    print(shimmer_uart.byte_array_to_hex_string(eeprom_bytes))
    eeprom_bytes_backup = eeprom_bytes
print("")

if eeprom_bytes_backup is not None:
    print("Write Daughter Card Mem:")
    # test_buf = [0xFF] * 128
    test_buf = [i for i in range(0, 128)]
    result = shimmer.dock_port.write_daughter_card_mem(test_buf)
    if result:
        eeprom_bytes = shimmer.dock_port.read_daughter_card_mem()
        print("Success" if test_buf == eeprom_bytes else "FAIL")
        print("Restoring Daughter Card Mem contents")
        result = shimmer.dock_port.write_daughter_card_mem(eeprom_bytes_backup)
    else:
        print("Error")
    print("")

print("Read Infomem:")
infomem_bytes_backup = None
infomem_bytes = shimmer.dock_port.read_infomem()
if isinstance(infomem_bytes, bool):
    print("Error")
else:
    print(shimmer_uart.byte_array_to_hex_string(infomem_bytes))
    infomem_bytes_backup = infomem_bytes
print("")

if infomem_bytes_backup is not None:
    print("Write Infomem:")
    test_buf = [(i & 0xFF) for i in range(0, 384)]
    result = shimmer.dock_port.write_infomem(test_buf)
    if result:
        infomem_bytes = shimmer.dock_port.read_infomem()

        result = True
        for i in range(0, 384):
            # Shimmer FW protects these bytes 224-229 as the MAC address is stored here
            if 224 <= i <= 229:
                pass
            # TODO unsure why FW is masking byte index 213, bit 5
            elif i == 213 and (infomem_bytes[i] & 0xEF) == (test_buf[i] & 0xEF):
                pass
            elif infomem_bytes[i] != test_buf[i]:
                print("Index=" + str(i) + ": "
                      + shimmer_uart.byte_array_to_hex_string([infomem_bytes[i]])
                      + "!=" + shimmer_uart.byte_array_to_hex_string([test_buf[i]]))
                result = False
                break
        print("Success" if result else "FAIL")
        print("Restoring Infomem contents")
        result = shimmer.dock_port.write_infomem(infomem_bytes_backup)
    else:
        print("Error")
    print("")

print("All done")
