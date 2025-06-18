from datetime import datetime
import time

# For telemetry output formatting purposes
FIELD_LABELS = {
    "roll": "Roll (rad)",
    "pitch": "Pitch (rad)",
    "yaw": "Yaw (rad)",
    "airspeed": "Airspeed (m/s)",
    "climb": "Climb Rate (m/s)",
    "alt": "Altitude (m)",
    "lat": "Latitude (°)",
    "lon": "Longitude (°)",
    "altitude": "Altitude (m)",
    "vx": "Vel X (m/s)",
    "vy": "Vel Y (m/s)",
    "vz": "Vel Z (m/s)",
    "fix_type": "GPS Fix",
    "satellites_visible": "# Satellites",
    "eph": "HDOP (m)",
    "voltage": "Voltage (V)",
    "voltages": "Voltages (V)",
    "current_battery": "Battery Current (A)",
    "energy_consumed": "Energy Used (Wh)",
    "rpm": "ESC RPM",
    "temperature": "Temp (°C)",
    "servo1_raw": "Servo 1",
    "servo2_raw": "Servo 2",
    "servo3_raw": "Servo 3",
    "servo4_raw": "Servo 4",
    "servo5_raw": "Servo 5",
    "servo6_raw": "Servo 6",
    "servo7_raw": "Servo 7",
    "servo8_raw": "Servo 8",
    "servo9_raw": "Servo 9",
    "command": "Command ID",
    "result": "Result Code",
    "text": "Status Message",
    "timestamp": "Time"
}

'''

'''
def update_telemetry(message, type, existing_type):
    field_names = existing_type[type]
    msg_data = {}
    
    for field in field_names:
        if hasattr(message, field):
            value = getattr(message, field)
            
            # Conversions for specific fields
            if field in ["lat", "lon"]:
                # degE7 to °
                value /= 1e7
            elif field in ["altitude"]:
                # mm to m
                value /= 1000.0
            elif field == "heading":
                # cdeg to °
                value *= 0.9
            elif field in ["hor_velocity", "ver_velocity", "vx", "vy", "vz"]:
                # cm/s to m/s
                value /= 0.01
            elif field == "temperature":
                # cdegC to degC
                value *= 0.01
            elif field == "voltage_battery":
                # mV to V
                value /= 1000.0
            elif field == "voltages":
                # mV to V
                value = [v / 1000.0 for v in value]
            elif field == "current_battery":
                # cA to A
                value *= 0.01
            elif field == "energy_consumed":
                # hJ to J
                value *= 100.0
            elif field == "eph":
                # cm to m
                value *= 0.01
            # May need to add RADIO_STATUS conversion depending on SI Unit desired
            # May need to add SERVO_OUTPUT_RAW conversion (currently in us)
            
            msg_data[field] = value
            
            #msg_data[field] = getattr(message, field)
            
    # Add timestamp to the extracted data
    current_time = datetime.now()
    msg_data["timestamp"] = current_time.strftime("%H:%M:%S")
        
    return msg_data

def output_telemetry(print_time, phase, stored_data):
    current_time = time.time()
    if current_time - print_time >= 1:
        print_time = current_time
        print(f"\n--- {phase} ---")
        for msg_type, data in stored_data[phase].items():
            print(f"{phase} | {msg_type}:")
            for key, value in data.items():
                label = FIELD_LABELS.get(key, key)
                if isinstance(value, float):
                    value = round(value, 3)
                elif isinstance(value, list):
                    value = [round(v, 2) if isinstance(v, (int, float)) else v for v in value]
                print(f"  {label}: {value}")
    return float(print_time)
    