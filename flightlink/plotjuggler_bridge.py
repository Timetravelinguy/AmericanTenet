## @file plotjuggler_bridge.py
#  @brief Sends data to PlotJuggler via UDP for real-time visualization.
#
#  @details This script establishes a UDP connection and sends JSON-formatted
#  telemetry data (with a timestamp) to PlotJuggler on localhost:9870.
#
#  @author American Tenet
#  @date 2025-06-20
#  @version 1.0
import socket # For sending data over network using UDP
import json   # For formatting data as JSON
import time

# Create a UDP socket 
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#address for PlotJuggler's UDP Server
DESTINATION = ("127.0.0.1", 9870)

## @brief Sends a named value with timestamp to PlotJuggler.
#  @param name The name of the variable (string).
#  @param value The numeric value to send.
#  @details Constructs a JSON payload with a timestamp and sends it over UDP.
def send_to_plotjuggler(name, value):
    payload = {
        name: {
            "timestamp": time.time(),
            "value": value
        }
    }
    # Convert Python into a JSON string
    msg = json.dumps(payload)
    # Send the JSON to plotjuggler 
    sock.sendto(msg.encode(), DESTINATION)
    
