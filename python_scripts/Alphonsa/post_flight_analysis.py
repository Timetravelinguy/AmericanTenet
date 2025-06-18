from pymavlink import mavutil
import pandas as pd
from datetime import datetime

# Connect to PX4 via MAVLink
conn = mavutil.mavlink_connection('udp:localhost:14445')
conn.wait_heartbeat()
print("[✓] Connected to PX4")

# Wait for POST_FLIGHT status
print("Waiting for POST_FLIGHT status...")
while True:
    msg = conn.recv_match(blocking=True)
#     if msg.get_type() == "HEARTBEAT" and msg.system_status == mavutil.mavlink.MAV_STATE_POWEROFF:
#         print("[✓] Detected POST_FLIGHT phase")
#         break
    if msg.get_type() == "HEARTBEAT":
        is_disarmed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) == 0
        is_standby = msg.system_status == mavutil.mavlink.MAV_STATE_STANDBY

        if is_disarmed and is_standby:
            print("[✓] Detected POST_FLIGHT state")
            break
# Collect telemetry data
log_rows = []
print("Collecting BATTERY_STATUS and SYS_STATUS...")

while True:
    msg = conn.recv_match(blocking=True)
    now = datetime.now().strftime("%H:%M:%S")
    msg_type = msg.get_type()

    if msg_type == "BATTERY_STATUS":
        log_rows.append({
            "Time": now,
            "Message": "BATTERY_STATUS",
            "Voltage1_mV": msg.voltages[0],
            "BatteryCurrent_cA": msg.current_battery,
            "EnergyUsed_mWh": msg.energy_consumed,
            "BatteryRemaining_pct": msg.battery_remaining
        })

    elif msg_type == "SYS_STATUS":
        log_rows.append({
            "Time": now,
            "Message": "SYS_STATUS",
            "Sensors_Present": msg.onboard_control_sensors_present,
            "Sensors_Enabled": msg.onboard_control_sensors_enabled,
            "Sensors_Health": msg.onboard_control_sensors_health
        })

    if len(log_rows) >= 10:
        break

# Save to CSV using Pandas
df = pd.DataFrame(log_rows)
df.to_csv("post_flight_battery_system_log.csv", index=False)
print("[✓] Data saved to post_flight_battery_system_log.csv")
