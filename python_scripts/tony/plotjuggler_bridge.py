# === plotjuggler_bridge.py ===
import socket
import json
import time

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
DESTINATION = ("127.0.0.1", 9870)

def send_to_plotjuggler(name, value):
    payload = {
        name: {
            "timestamp": time.time(),
            "value": value
        }
    }
    msg = json.dumps(payload)
    sock.sendto(msg.encode(), DESTINATION)
