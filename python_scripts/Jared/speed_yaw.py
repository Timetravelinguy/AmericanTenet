import time
import math
from pymavlink import mavutil

def to_quaternion(yaw_deg):
    """Convert yaw in degrees to quaternion format (x, y, z, w)."""
    yaw_rad = math.radians(yaw_deg)
    return [0.0, 0.0, math.sin(yaw_rad / 2), math.cos(yaw_rad / 2)]

# Connect to PX4 SITL via MAVLink UDP
the_connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')
the_connection.wait_heartbeat()
print(f"Heartbeat from system (system {the_connection.target_system} component {the_connection.target_component})")

target_system = the_connection.target_system
target_component = the_connection.target_component

# Arm the drone
the_connection.mav.command_long_send(
    target_system,
    target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0
)
print("Sent ARM command")
the_connection.recv_match(type='COMMAND_ACK', blocking=True)

# Set OFFBOARD mode
mode_id = the_connection.mode_mapping()['OFFBOARD']
the_connection.mav.set_mode_send(
    target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id
)
print("Sent OFFBOARD mode")
the_connection.recv_match(type='COMMAND_ACK', blocking=True)

# PX4 requires streaming some setpoints first
start_time = time.time()
yaw_deg = 90  # Target yaw angle
duration = 5  # seconds

for _ in range(20):  # Send 2 seconds of pre-streaming at 100 ms intervals
    time_boot_ms = int((time.time() - start_time) * 1000)  # ✅ safe 32-bit timestamp
    q = to_quaternion(yaw_deg)
    the_connection.mav.set_attitude_target_send(
        time_boot_ms,
        target_system,
        target_component,
        0b00000100,  # Only control yaw
        q,
        0, 0, 0,
        0.7  # Thrust
    )
    time.sleep(0.1)

# Continue sending for remaining time
print("Sending yaw control setpoints...")
end_time = time.time() + duration
while time.time() < end_time:
    time_boot_ms = int((time.time() - start_time) * 1000)  # ✅ safe 32-bit timestamp
    q = to_quaternion(yaw_deg)
    the_connection.mav.set_attitude_target_send(
        time_boot_ms,
        target_system,
        target_component,
        0b00000100,
        q,
        0, 0, 0,
        0.7
    )
    time.sleep(0.1)

print("Done.")
