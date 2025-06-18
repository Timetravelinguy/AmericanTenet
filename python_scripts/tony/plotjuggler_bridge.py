# === plotjuggler_bridge.py ===
import socket # For sending data over network using UDP
import json   # For formatting data as JSON
import time

# Create a UDP socket 
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#address for PlotJuggler's UDP Server
DESTINATION = ("127.0.0.1", 9870)

# Function to send data to PlotJuggler 
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
    
