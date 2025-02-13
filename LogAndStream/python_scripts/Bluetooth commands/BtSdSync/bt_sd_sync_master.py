import time
import serial
import threading
import datetime

from Shimmer_common.shimmer_crc import calc_crc  # Import the CRC function
from Shimmer_common.util_shimmer import byte_array_to_hex_string


# ================== CONFIGURATION ==================
SYNC_INTERVAL = 1*60  # Time in seconds between sync operations (default: 1 minutes)
COM_PORTS = ["COM11"]  # List of COM ports to sync with (example: ["COM3", "COM4"])
BAUDRATE = 115200  # Serial baud rate
MAX_ATTEMPTS = 50  # Maximum number of sync attempts per sensor
ENABLE_SD_LOGGING = True  # True for on-board recording, False for idling
NUM_BYTES_CRC = 1
DEBUG_TX_PACKETS = False
# ===================================================


def get_sync_packet() -> bytes:
    """Constructs the 11-byte sync packet."""
    header = b'\xE0'
    mode = b'\x01' if ENABLE_SD_LOGGING else b'\x00'  # Define operational mode
    timestamp = int(time.time() * 32768).to_bytes(8, byteorder='little', signed=False)  # 8-byte timestamp
    data = header + mode + timestamp
    crc = calc_crc(len(data), data).to_bytes(2, byteorder='little')  # Compute CRC with shimmer_crc
    if NUM_BYTES_CRC == 1:
        crc = crc[0:1]
    return data + crc


def sync_sensor(port: str):
    """Handles the sync operation for a single sensor on a given COM port."""
    try:
        with serial.Serial(port, BAUDRATE, timeout=1) as ser:
            print(f"Connecting to {port}...")
            attempts = 0
            while attempts < MAX_ATTEMPTS:
                packet = get_sync_packet()

                if DEBUG_TX_PACKETS:
                    print("TX bytes =", byte_array_to_hex_string(list(packet)))

                ser.write(packet)
                response = ser.read(3)

                if response == b'\xFE':
                    print(f"[{port}] Nack received after {attempts+1} attempts.")
                    break
                elif response == b'\xFF\xE1\x01':
                    print(f"[{port}] Sensor requested sync resend ({attempts+1}/{MAX_ATTEMPTS})")
                elif response == b'\xFF\xE1\xFF':
                    print(f"[{port}] Sync completed after {attempts+1} attempts.")
                    break
                elif len(response) == 0:
                    print(f"[{port}] No response from sensor ({attempts + 1}/{MAX_ATTEMPTS})")
                else:
                    print(f"[{port}] Unexpected response at attempt ({attempts+1}/{MAX_ATTEMPTS}): "
                          f"{byte_array_to_hex_string(list(response))}")

                attempts += 1

            print(f"[{port}] Disconnecting...")
    except serial.SerialException as e:
        print(f"Error on {port}: {e}")
    except Exception as e:
        print(f"Unexpected error on {port}: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print(f"[{port}] Port closed due to error.")


def sync_sensors_periodically():
    """Runs the sync operation periodically at the given interval."""
    sync_count = 1
    while True:
        time_str = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        print(f"\nSync Attempt {sync_count} start at {time_str}")
        for port in COM_PORTS:
            sync_sensor(port)
        print(f"Sync Attempt {sync_count} end at {time_str}")
        sync_count += 1
        print(f"\nWaiting {SYNC_INTERVAL} seconds before the next sync...")
        time.sleep(SYNC_INTERVAL)


def main():
    """Main function to start the periodic sync process and handle exit."""
    sync_thread = threading.Thread(target=sync_sensors_periodically, daemon=True)
    sync_thread.start()

    try:
        while True:
            time.sleep(1)  # Keep the main thread alive
    except KeyboardInterrupt:
        print("\nSync process terminated by user.")
    finally:
        print("Cleaning up before exit...")


if __name__ == "__main__":
    main()
