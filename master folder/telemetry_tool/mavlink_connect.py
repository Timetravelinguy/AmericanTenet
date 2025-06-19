from pymavlink import mavutil

# Sets up connection with desired protocol (ex. UDP, TCP, Serial) and port number.
# Returns port connection as object.
def connect(protocol, port_number):
    
    # Connect to MAVLink stream (forwarded by QGC)
    the_connection = mavutil.mavlink_connection(f'{protocol}:localhost:{port_number}')
    
    # Wait for first heartbeat (important for syncing)
    print("Waiting for first heartbeat...")
    the_connection.wait_heartbeat()
    print("Heartbeat received!")
    print(f"Connected to system {the_connection.target_system}, component {the_connection.target_component}")
    
    return the_connection
