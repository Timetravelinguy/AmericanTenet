# === plotjuggler_bridge.py ===
import socket  # For sending data over network using UDP
import json    # For formatting data as JSON
import time

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Address for PlotJuggler's UDP Server
DESTINATION = ("127.0.0.1", 9870)

# Function to send a batch of data to PlotJuggler with single timestamp
def send_batch_to_plotjuggler(data_dict, timestamp=None):
    """
    Send multiple data points with the same timestamp
    data_dict: dictionary of {name: value} pairs
    timestamp: optional timestamp (uses current time if None)
    """
    if timestamp is None:
        timestamp = time.time()
    
    # Create payload with all data sharing the same timestamp
    payload = {}
    for name, value in data_dict.items():
        payload[name] = {
            "timestamp": timestamp,
            "value": value
        }
    
    # Convert Python into a JSON string
    msg = json.dumps(payload)
    # Send the JSON to plotjuggler
    sock.sendto(msg.encode(), DESTINATION)

# Legacy function for backwards compatibility (but discouraged for synchronized data)
def send_to_plotjuggler(name, value, timestamp=None):
    """
    Send single data point - use send_batch_to_plotjuggler for synchronized data
    """
    if timestamp is None:
        timestamp = time.time()
    
    payload = {
        name: {
            "timestamp": timestamp,
            "value": value
        }
    }
    
    # Convert Python into a JSON string
    msg = json.dumps(payload)
    # Send the JSON to plotjuggler
    sock.sendto(msg.encode(), DESTINATION)