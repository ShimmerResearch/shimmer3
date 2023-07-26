#!/usr/bin/python
import time

import shimmer_app_common
import shimmer_device
import util_shimmer_time

com_port = shimmer_app_common.get_selected_com_port()
if not com_port:
    print("Supported COM port not found, exiting")
    exit()

shimmer = shimmer_device.Shimmer3()

if not shimmer.setup_dock_com_port(com_port):
    exit()

print("Read current time:")
while 1:
    ts_ms = shimmer.dock_port.read_current_time()
    if isinstance(ts_ms, bool):
        print("Error reading time")
    else:
        print(util_shimmer_time.seconds_to_time_str(ts_ms / 1000, True))
    print("")
    time.sleep(1)
