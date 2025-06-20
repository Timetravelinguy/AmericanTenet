## @mainpage Telemetry Tool
#  @brief Live PX4 MAVLink telemetry monitor and PlotJuggler bridge.
#
#  This tool connects to a MAVLink stream, receives and decodes telemetry messages, 
#  processes them by flight phase, and visualizes the data using PlotJuggler.
#
#  ### Key Features
#  - Detects flight phases (ex. PREFLIGHT, TAKEOFF, CRUISING, and LANDING, etc.)
#  - Extracts relevant telemetry fields (battery, GPS, attitude, ESC, etc.)
#  - Converts raw MAVLink data to readable values in SI units
#  - Sends real-time data to PlotJuggler over UDP
#  - Prints telemetry snapshots every second
#
#  ### Project Modules
#  - `main.py` – Main telemetry loop and routing
#  - `telemetry.py` – Field extraction, unit conversion, and storage
#  - `heartbeat.py` – Flight mode decoding and phase detection
#  - `plotjuggler_bridge.py` – UDP interface to PlotJuggler
#  - `mavlink_connect.py` – MAVLink connection utility
#
#  @author American Tenet
#  @version 1.0
#  @date 2025-06-20

## @file main.py
#  @brief Main control loop for MAVLink message handling and telemetry routing.
#
#  @details Connects to mavlink-router via UDP, tracks drone flight phases, extracts telemetry fields,
#  and sends relevant data to PlotJuggler for live visualization.

from plotjuggler_bridge import send_to_plotjuggler
from mavlink_connect import connect
from heartbeat import Heartbeat
from telemetry import Telemetry
from pymavlink import mavutil
import time

## @brief Main function to run the MAVLink telemetry loop.
#  @details Connects to mavlink-router, tracks flight phases, and publishes telemetry to PlotJuggler.
def main():
    # Connect to MAVLink stream (forwarded by mavlink-router)
    the_connection = connect('udpin', '14551')

    # Define fields to extract from each MAVLink message type
    fields_by_type = {
        "ADSB_VEHICLE": ["icao_address", "lat", "lon", "altitude", "heading", "hor_velocity", "ver_velocity"],
        "ATTITUDE": ["roll", "pitch", "yaw"],
        "BATTERY_STATUS": ["temperature", "voltages", "current_battery", "energy_consumed"],
        "COMMAND_ACK": ["command", "result"],
        "GPS_RAW_INT": ["fix_type", "satellites_visible", "eph", "lat", "lon", "alt"],
        "GLOBAL_POSITION_INT": ["vx", "vy", "vz"],
        "ESC_INFO": ["temperature"],
        "ESC_STATUS": ["rpm", "voltage", "current"],
        "HEARTBEAT": ["system_status"],
        "RADIO_STATUS": ["rssi", "remrssi", "txbuf", "noise", "remnoise"],
        "RAW_IMU": ["temperature"],
        "SCALED_IMU2": ["temperature"],
        "SCALED_IMU3": ["temperature"],
        "SCALED_PRESSURE": ["temperature"],
        "SERVO_OUTPUT_RAW": ["servo1_raw", "servo2_raw", "servo3_raw", "servo4_raw", "servo5_raw", "servo6_raw", "servo7_raw", "servo8_raw", "servo9_raw", "servo10_raw", "servo11_raw", "servo12_raw", "servo13_raw", "servo14_raw", "servo15_raw", "servo16_raw"],
        "STATUSTEXT": ["text"],
        "SYS_STATUS": ["battery_remaining", "voltage_battery", "current_battery", "onboard_control_sensors_present", "onboard_control_sensors_enabled", "onboard_control_sensors_health"],
        "VFR_HUD": ["airspeed", "climb", "alt"]
    }

    # Define relevant message types for each flight phase
    types_by_phase = {
        "MASTER LOG": ["BATTERY_STATUS", "GPS_RAW_INT", "ESC_STATUS", "ESC_INFO", "SYS_STATUS", "RADIO_STATUS", "VFR_HUD", "SCALED_IMU2", "SCALED_IMU3"],
        "PREFLIGHT": ["ESC_STATUS", "HEARTBEAT", "RADIO_STATUS", "COMMAND_ACK", "SERVO_OUTPUT_RAW", "SYS_STATUS", "GPS_RAW_INT"],
        "TAKEOFF": ["ESC_INFO", "ESC_STATUS", "BATTERY_STATUS", "RAW_IMU", "SCALED_IMU2", "SCALED_IMU3", "GLOBAL_POSITION_INT"],
        "HOVERING": ["ESC_INFO", "ESC_STATUS", "SERVO_OUTPUT_RAW", "GPS_RAW_INT"],
        "CRUISING": ["ESC_INFO", "ESC_STATUS", "SERVO_OUTPUT_RAW", "GPS_RAW_INT", "GLOBAL_POSITION_INT"],
        "SOARING IN THERMAL": ["ESC_STATUS", "SERVO_OUTPUT_RAW", "ATTITUDE", "VFR_HUD", "GPS_RAW_INT", "ADSB_VEHICLE", "SCALED_PRESSURE"],
        "LANDING": ["ATTITUDE", "VFR_HUD", "SERVO_OUTPUT_RAW", "ESC_STATUS", "GLOBAL_POSITION_INT"],
        "POST FLIGHT": ["BATTERY_STATUS", "GPS_RAW_INT", "ESC_STATUS", "ESC_INFO", "SYS_STATUS", "RADIO_STATUS", "VFR_HUD", "SCALED_IMU2", "SCALED_IMU3", "STATUSTEXT"]
    }

    # Initialize telemetry and heartbeat handlers
    telemetry = Telemetry()
    heartbeat = Heartbeat(telemetry.master_storage)

    # Initialize flight phase tracking variables
    curr_phase = "PREFLIGHT"
    prev_phase = "PREFLIGHT"
    last_heartbeat_time = 0.0
    last_print_time = 0.0

    # Main MAVLink message processing loop
    while True:
        # Receive MAVLink messages
        msg = the_connection.recv_match(blocking=False)

        # Skip invalid messages
        if msg is None:
            time.sleep(0.01)  # Prevent busy-waiting
            continue

        # Process HEARTBEAT messages to update flight phases
        if msg.get_type() == 'HEARTBEAT':
            curr_phase, prev_phase, last_heartbeat_time = heartbeat.process_heartbeat(msg, curr_phase, prev_phase, last_heartbeat_time)

        # Process other telemetry messages
        elif msg.get_type() in fields_by_type:
            telemetry.update_master_storage(msg, msg.get_type(), fields_by_type)

        # Filter telemetry data for the current flight phase
        flight_phase_data = telemetry.get_flight_phase_data(curr_phase, types_by_phase)

        # Send telemetry data to PlotJuggler
        for msg_type, data in flight_phase_data.items():
            for key, value in data.items():
                if isinstance(value, (int, float)):
                    send_to_plotjuggler(f"{msg_type}_{key}", value)

        # Print telemetry data every second
        last_print_time = telemetry.output_phase_data(last_print_time, curr_phase, flight_phase_data)

        # Run loop at 100 Hz
        time.sleep(0.01)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nProgram interrupted by user. Exiting...")
    except mavutil.mavlink.MAVError as e:
        print(f"\nMAVLink error: {e}")
    except Exception as e:
        print(f"\nUnexpected error: {e}")

