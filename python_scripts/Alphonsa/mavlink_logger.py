from pymavlink import mavutil
import csv
import time
from datetime import datetime

# === Configuration ===
LOG_FILE = f"mavlink_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

# === Setup CSV logging ===
csv_file = open(LOG_FILE, mode='w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(["timestamp", "message_type", "field", "value"])  # CSV header

print(f"Logging all MAVLink messages to: {LOG_FILE}")

# === Connect to PX4 ===
connection = mavutil.mavlink_connection('udp:localhost:14445')

print("Waiting for heartbeat...")
connection.wait_heartbeat()
print(f"Connected to system {connection.target_system}, component {connection.target_component}")

# === Message loop ===
try:
    while True:
        msg = connection.recv_match(blocking=True)
        if msg:
            timestamp = time.time()
            msg_type = msg.get_type()
            msg_dict = msg.to_dict()

            print(f"[{msg_type}] {msg_dict}")

            # Write each field in the message to the CSV file
            for field, value in msg_dict.items():
                csv_writer.writerow([timestamp, msg_type, field, value])

except KeyboardInterrupt:
    print("Logging stopped by user.")

finally:
    csv_file.close()
    print(f"CSV log saved to {LOG_FILE}")
