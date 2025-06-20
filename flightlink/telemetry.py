## @file telemetry.py
#  @brief Handles extraction, formatting, and display of MAVLink telemetry data.
#
#  @details Provides utility functions to parse fields from MAVLink messages, convert units to SI,
#  and print formatted telemetry by flight phase. Also includes timestamping for better log clarity.
#
#  @author American Tenet
#  @date 2025-06-20
#  @version 1.0
from datetime import datetime
import time

## @brief Dictionary for human-readable telemetry field labels.
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
    "servo10_raw": "Servo 10",
    "servo11_raw": "Servo 11",
    "servo12_raw": "Servo 12",
    "servo13_raw": "Servo 13",
    "servo14_raw": "Servo 14",
    "servo15_raw": "Servo 15",
    "servo16_raw": "Servo 16",
    "command": "Command ID",
    "result": "Result Code",
    "text": "Status Message",
    "timestamp": "Time"
}

## @brief Extracts and converts telemetry fields from a MAVLink message.
#  @param message The MAVLink message to process.
#  @param type The message type string (e.g., "ATTITUDE", "BATTERY_STATUS").
#  @param existing_type Dictionary mapping types to desired field names.
#  @return A dictionary of extracted and converted telemetry fields with a timestamp.
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
            elif field in ["alt", "altitude"]:
                # mm to m
                value /= 1000.0
            elif field == "heading":
                # cdeg to °
                value *= 0.9
            elif field in ["hor_velocity", "ver_velocity", "vx", "vy", "vz"]:
                # cm/s to m/s
                value *= 0.01
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
            
    # Add timestamp to the extracted data
    current_time = datetime.now()
    msg_data["timestamp"] = current_time.strftime("%H:%M:%S")
        
    return msg_data

## @brief Prints flight phase telemetry to the terminal every second.
#  @param print_time Last time telemetry was printed (epoch time).
#  @param phase The current flight phase (string).
#  @param stored_data Dictionary containing telemetry data organized by phase and message type.
#  @return Updated time of last print as a float.
def output_telemetry(print_time, phase, stored_data):
    current_time = time.time()
    if current_time - print_time >= 1:
        print_time = current_time
        print(f"\n--- {phase} ---")
        for msg_type, data in stored_data.items():
            print(f"{phase} | {msg_type}:")
            for key, value in data.items():
                label = FIELD_LABELS.get(key, key)
                if isinstance(value, float):
                    value = round(value, 3)
                elif isinstance(value, list):
                    value = [round(v, 2) if isinstance(v, (int, float)) else v for v in value]
                print(f"  {label}: {value}")
    return float(print_time)


## @brief Manages MAVLink telemetry data.
#  @details Stores, filters, and outputs telemetry data by flight phase.

class Telemetry:
    
    ## @brief Initializes the telemetry storage.
    def __init__(self):
        ## @brief Stores all telemetry data by message type.
        self.master_storage = {}

    ## @brief Updates telemetry storage with new data.
    #  @param msg MAVLink message with telemetry data.
    #  @param msg_type Message type (e.g., "ATTITUDE").
    #  @param fields_by_type Maps message types to field names.
    def update_master_storage(self, msg, msg_type, fields_by_type):
        self.master_storage[msg_type] = update_telemetry(msg, msg_type, fields_by_type)

    ## @brief Gets telemetry data for the current flight phase.
    #  @param curr_phase Current flight phase (e.g., "CRUISE").
    #  @param types_by_phase Maps phases to relevant message types.
    #  @return Telemetry data for the current phase.
    def get_flight_phase_data(self, curr_phase, types_by_phase):
        return {
            msg_type: self.master_storage[msg_type]
            for msg_type in types_by_phase[curr_phase]
            if msg_type in self.master_storage
        }

    ## @brief Outputs telemetry data for the current phase.
    #  @param print_time Last output timestamp.
    #  @param curr_phase Current flight phase.
    #  @param flight_phase_data Telemetry data for the current phase.
    #  @return Updated timestamp of the last output.
    def output_phase_data(self, print_time, curr_phase, flight_phase_data):
        return output_telemetry(print_time, curr_phase, flight_phase_data)



