from pymavlink import mavutil
from mavlink_connect import connect
from px4_mode_decode import decode_px4_mode
from flight_phase import det_flight_phase
from datetime import datetime
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

    # Initialize telemetry storage structure
    flight_phase_data = {phase: {} for phase in types_by_phase}

    current_phase = "PREFLIGHT"
    previous_phase = "PREFLIGHT"
    last_heartbeat_time = 0
    last_print_time = 0

    # Main MAVLink message processing loop
    while True:
        msg = the_connection.recv_match(blocking=False)

        if msg is None:
            time.sleep(0.01)  # Prevents busy-waiting
            continue

        msg_type = msg.get_type()

        # Process HEARTBEAT messages
        if msg_type == 'HEARTBEAT':
            print(f"[DEBUG] HEARTBEAT received: {msg}")
            now = time.time()

            # Limit HEARTBEAT decoding to once per second
            if now - last_heartbeat_time >= 1:
                last_heartbeat_time = now

                if hasattr(msg, "custom_mode"):
                    main_mode_str, sub_mode_str = decode_px4_mode(msg.custom_mode)
                    print(f"PX4 Flight Mode: {main_mode_str} - {sub_mode_str}")

                    if main_mode_str != "Unknown (0)":
                        current_phase = det_flight_phase(sub_mode_str, previous_phase)
                        if current_phase != previous_phase:
                            previous_phase = current_phase

        # Process all other telemetry messages
        elif msg_type in fields_by_type:
            field_names = fields_by_type[msg_type]
            msg_data = {}

            for field in field_names:
                if hasattr(msg, field):
                    msg_data[field] = getattr(msg, field)

            current_time = datetime.now()
            msg_data["timestamp"] = current_time.strftime("%H:%M:%S")
            flight_phase_data[current_phase][msg_type] = msg_data

        # Optional: print current flight phase data every 1 second
        if time.time() - last_print_time >= 1:
            last_print_time = time.time()
            print(f"\n--- {current_phase} ---")
            for mt, data in flight_phase_data[current_phase].items():
                print(f"{current_phase} | {mt}: {data}")

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