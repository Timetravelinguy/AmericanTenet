## @file mavlink_connect.py
#  @brief Establishes a MAVLink connection to the drone or simulator.
#
#  @details This module provides a utility function to connect to a MAVLink stream
#  using a specified protocol (e.g., UDP, TCP, Serial) and port. It waits for the initial
#  heartbeat to ensure proper communication is established before returning the connection.
#
#  @author American Tenet
#  @date 2025-06-19
#  @version 1.0
from pymavlink import mavutil

## @brief Connect to a MAVLink stream.
#  @param protocol The communication protocol to use (e.g., "udpin", "tcp", "com").
#  @param port_number The port number (e.g., 14540, 14551) to connect to.
#  @return The MAVLink connection object after receiving the first heartbeat.

def connect(protocol, port_number):
    
    # Connect to MAVLink stream (forwarded by QGC)
    the_connection = mavutil.mavlink_connection(f'{protocol}:localhost:{port_number}')
    
    # Wait for first heartbeat (important for syncing)
    print("Waiting for first heartbeat...")
    the_connection.wait_heartbeat()
    print("Heartbeat received!")
    print(f"Connected to system {the_connection.target_system}, component {the_connection.target_component}")
    
    return the_connection
