from pymavlink import mavutil

# Connect to PX4 via UDP
connection = mavutil.mavlink_connection('udp:localhost:14445')

# Wait for heartbeat to establish communication
print("Waiting for heartbeat...")
connection.wait_heartbeat()
print("Connected to system %u component %u" % (connection.target_system, connection.target_component))

print("\nListening for all MAVLink messages...\n")

try:
    while True:
        msg = connection.recv_match(blocking=True)
        if msg:
            print(f"[{msg.get_type()}] {msg.to_dict()}")
except KeyboardInterrupt:
    print("Stopped by user.")
