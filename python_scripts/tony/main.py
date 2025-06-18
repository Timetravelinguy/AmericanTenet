from pymavlink import mavutil
from mavlink_connect import connect
#from px4_mode_decode import decode_px4_mode
#from flight_phase import det_flight_phase
from heartbeat import process_heartbeat
#from plotjuggler_bridge.py
from plotjuggler_bridge import send_to_plotjuggler
from telemetry import update_telemetry, output_telemetry
import time

#test
def main():
    # Connect to MAVLink stream (forwarded by QGroundControl)
    the_connection = connect('udpin', '14445')

    # Define which fields to extract from each MAVLink message type
    fields_by_type = {
        "ADSB_VEHICLE": ["icao_address", "lat", "lon", "altitude", "heading", "velocity"],
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
        "SERVO_OUTPUT_RAW": ["servo1_raw", "servo2_raw", "servo3_raw", "servo4_raw", "servo5_raw", "servo6_raw", "servo7_raw", "servo8_raw", "servo9_raw"],
        "STATUSTEXT": ["text"],
        "SYS_STATUS": ["voltage", "battery", "onboard_control_sensors_present", "onboard_control_sensors_enabled", "onboard_control_sensors_health"],
        "VFR_HUD": ["airspeed", "climb", "alt"]
    }
    
    # Define which message types are relevant to each flight phase
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
    
    # For real-time telemetry storage
    flight_phase_data = {phase: {} for phase in types_by_phase}

    # Keep track of phases and times
    curr_phase = "PREFLIGHT"
    prev_phase = "PREFLIGHT"
    last_heartbeat_time = 0.0
    last_print_time = 0.0

    # Main MAVLink message processing loop
    while True:
        # Receive messages of all types
        msg = the_connection.recv_match(blocking=False)

        # Skip "bad" messages
        if msg is None:
            time.sleep(0.01)  # Prevents busy-waiting
            continue

        # Determine message type
        msg_type = msg.get_type()

        # Process HEARTBEAT messages
        if msg_type == 'HEARTBEAT':
            # Based on flight mode from HEARTBEAT message, update current and previous flight phase
            # Also return time heartbeat was processed at
            curr_phase, prev_phase, last_heartbeat_time = process_heartbeat(msg, curr_phase, prev_phase, last_heartbeat_time)

        # Process all other telemetry messages
        elif msg_type in fields_by_type:
            # Depending on message type of MAVLink message, store only relevant fields
            flight_phase_data[curr_phase][msg_type] = update_telemetry(msg, msg_type, fields_by_type)

            #add this to send data to plot juggler 
            data = flight_phase_data[curr_phase][msg_type]
            for key, value in data.items():
                if isinstance(value, (int, float)):
                    send_to_plotjuggler(f"{curr_phase}_{key}", value)
        # Print current flight phase data every 1 second
        last_print_time = output_telemetry(last_print_time, curr_phase, flight_phase_data)

        # Run this loop every 1/100th of a second
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
