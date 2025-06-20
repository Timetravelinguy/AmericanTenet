from pymavlink import mavutil
from mavlink_connect import connect
from heartbeat import process_heartbeat
from plotjuggler_bridge import send_batch_to_plotjuggler
from telemetry import update_telemetry, output_telemetry
import time

def main():
    # Connect to MAVLink stream (forwarded by QGroundControl)
    the_connection = connect('udpin', '14550')

    phase_ids = {
        "PREFLIGHT": 0,
        "TAKEOFF": 1,
        "HOVERING": 2,
        "CRUISING": 3,
        "SOARING IN THERMAL": 4,
        "LANDING": 5,
        "POST FLIGHT": 6
    }   

    # Organize telemetry data by logical categories for better visualization
    telemetry_categories = {
        # Power and Energy Management
        "power": {
            "BATTERY_STATUS": ["temperature", "voltages", "current_battery", "energy_consumed", 
                              "current_consumed", "battery_remaining", "time_remaining"],
            "SYS_STATUS": ["voltage_battery", "current_battery", "battery_remaining"],
            "POWER_STATUS": ["Vcc", "Vservo", "flags"]
        },
        
        # Navigation and Positioning
        "navigation": {
            "GPS_RAW_INT": ["fix_type", "satellites_visible", "eph", "epv", "lat", "lon", "alt", 
                           "vel", "cog", "hdop", "vdop"],
            "GLOBAL_POSITION_INT": ["lat", "lon", "alt", "relative_alt", "vx", "vy", "vz", "hdg"],
            "LOCAL_POSITION_NED": ["x", "y", "z", "vx", "vy", "vz"],
            "ADSB_VEHICLE": ["icao_address", "lat", "lon", "altitude", "heading", "hor_velocity",
                           "ver_velocity", "callsign", "emitter_type", "tslc", "flags", "squawk"]
        },
        
        # Flight Dynamics and Control
        "flight_dynamics": {
            "ATTITUDE": ["roll", "pitch", "yaw", "rollspeed", "pitchspeed", "yawspeed"],
            "VFR_HUD": ["airspeed", "groundspeed", "climb", "alt", "heading", "throttle"],
            "SERVO_OUTPUT_RAW": ["servo1_raw", "servo2_raw", "servo3_raw", "servo4_raw", 
                               "servo5_raw", "servo6_raw", "servo7_raw", "servo8_raw", 
                               "servo9_raw", "servo10_raw", "servo11_raw", "servo12_raw",
                               "servo13_raw", "servo14_raw", "servo15_raw", "servo16_raw"],
            "RC_CHANNELS": ["chan1_raw", "chan2_raw", "chan3_raw", "chan4_raw", "chan5_raw", 
                          "chan6_raw", "chan7_raw", "chan8_raw", "chan9_raw", "chan10_raw",
                          "chan11_raw", "chan12_raw", "chan13_raw", "chan14_raw", "chan15_raw", 
                          "chan16_raw", "chan17_raw", "chan18_raw", "rssi"]
        },
        
        # Propulsion System - FIXED: Added more comprehensive ESC message handling
        "propulsion": {
            "ESC_INFO": ["counter", "count", "connection_type", "info", "failure_count", 
                        "error_count", "temperature"],
            "ESC_STATUS": ["counter", "count", "connection_type", "esc", "failure_count", 
                          "error_count", "temperature", "rpm", "voltage", "current"],
            # ArduPilot specific ESC telemetry messages
            "ESC_TELEMETRY_1_TO_4": ["temperature", "voltage", "current", "totalcurrent", "rpm", 
                                    "count"],
            "ESC_TELEMETRY_5_TO_8": ["temperature", "voltage", "current", "totalcurrent", "rpm", 
                                    "count"],
            "ESC_TELEMETRY_9_TO_12": ["temperature", "voltage", "current", "totalcurrent", "rpm", 
                                     "count"],
            # Motor test and control
            "ACTUATOR_CONTROL_TARGET": ["controls", "group_mlx", "group_mly", "group_mlz"],
            "ACTUATOR_OUTPUT_STATUS": ["active", "actuator"]
        },
        
        # Sensors and Environment
        "sensors": {
            "RAW_IMU": ["time_usec", "xacc", "yacc", "zacc", "xgyro", "ygyro", "zgyro", 
                       "xmag", "ymag", "zmag", "temperature"],
            "SCALED_IMU2": ["time_boot_ms", "xacc", "yacc", "zacc", "xgyro", "ygyro", "zgyro", 
                           "xmag", "ymag", "zmag", "temperature"],
            "SCALED_IMU3": ["time_boot_ms", "xacc", "yacc", "zacc", "xgyro", "ygyro", "zgyro", 
                           "xmag", "ymag", "zmag", "temperature"],
            "SCALED_PRESSURE": ["time_boot_ms", "press_abs", "press_diff", "temperature", 
                               "temperature_press_diff"],
            "SCALED_PRESSURE2": ["time_boot_ms", "press_abs", "press_diff", "temperature", 
                                "temperature_press_diff"],
            "SCALED_PRESSURE3": ["time_boot_ms", "press_abs", "press_diff", "temperature", 
                                "temperature_press_diff"],
            "HIL_SENSOR": ["time_usec", "xacc", "yacc", "zacc", "xgyro", "ygyro", "zgyro", 
                          "xmag", "ymag", "zmag", "abs_pressure", "diff_pressure", 
                          "pressure_alt", "temperature", "fields_updated"]
        },
        
        # Communication and System Status
        "system_status": {
            "HEARTBEAT": ["type", "autopilot", "base_mode", "custom_mode", "system_status", 
                         "mavlink_version"],
            "SYS_STATUS": ["onboard_control_sensors_present", "onboard_control_sensors_enabled", 
                          "onboard_control_sensors_health", "load", "voltage_battery", 
                          "current_battery", "battery_remaining", "drop_rate_comm", 
                          "errors_comm", "errors_count1", "errors_count2", "errors_count3", 
                          "errors_count4"],
            "RADIO_STATUS": ["rssi", "remrssi", "txbuf", "noise", "remnoise", "rxerrors", "fixed"],
            "COMMAND_ACK": ["command", "result", "progress", "result_param2", "target_system", 
                           "target_component"],
            "STATUSTEXT": ["severity", "text", "id", "chunk_seq"],
            "PING": ["time_usec", "seq", "target_system", "target_component"]
        }
    }
    
    # Flatten the categories to create a comprehensive fields dictionary
    fields_by_type = {}
    for category, msg_types in telemetry_categories.items():
        for msg_type, fields in msg_types.items():
            if msg_type in fields_by_type:
                # Merge fields if message type appears in multiple categories
                fields_by_type[msg_type] = list(set(fields_by_type[msg_type] + fields))
            else:
                fields_by_type[msg_type] = fields
    
    # Store all telemetry data (not phase-specific)
    all_telemetry_data = {}
    
    # Store latest values for synchronized sending
    latest_values = {}
    
    # Keep track of phases and times
    curr_phase = "PREFLIGHT"
    prev_phase = "PREFLIGHT"
    last_heartbeat_time = 0.0
    last_print_time = 0.0
    last_send_time = 0.0

    # Debug counters
    message_counts = {}
    propulsion_message_count = 0

    print("Starting MAVLink telemetry processing...")
    print(f"Monitoring {len(fields_by_type)} message types")
    print("Propulsion messages being monitored:", [msg for msg in fields_by_type.keys() 
          if any(msg in cat_msgs for cat_msgs in [telemetry_categories["propulsion"]])])

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
        
        # Count all message types for debugging
        message_counts[msg_type] = message_counts.get(msg_type, 0) + 1

        # Process HEARTBEAT messages
        if msg_type == 'HEARTBEAT':
            # Based on flight mode from HEARTBEAT message, update current and previous flight phase
            curr_phase, prev_phase, last_heartbeat_time = process_heartbeat(msg, curr_phase, prev_phase, last_heartbeat_time)

        # Process all telemetry messages (regardless of flight phase)
        elif msg_type in fields_by_type:
            # Store telemetry data for all message types
            telemetry_data = update_telemetry(msg, msg_type, fields_by_type)
            all_telemetry_data[msg_type] = telemetry_data

            # Debug: Count propulsion messages
            if msg_type in telemetry_categories["propulsion"]:
                propulsion_message_count += 1

            # Update latest values for synchronized sending
            # Find which category this message belongs to and update latest values
            for category, msg_types in telemetry_categories.items():
                if msg_type in msg_types:
                    for key, value in telemetry_data.items():
                        # Handle different data types more robustly
                        if isinstance(value, (int, float)):
                            # Store the latest value with category prefix
                            field_name = f"{category}/{msg_type}_{key}"
                            latest_values[field_name] = value
                        elif isinstance(value, list) and len(value) > 0:
                            # Handle array data (like battery voltages, ESC arrays)
                            if all(isinstance(v, (int, float)) for v in value):
                                # Numeric arrays
                                for i, val in enumerate(value):
                                    field_name = f"{category}/{msg_type}_{key}_{i}"
                                    latest_values[field_name] = val
                            elif len(value) == 1 and isinstance(value[0], (int, float)):
                                # Single-element arrays
                                field_name = f"{category}/{msg_type}_{key}"
                                latest_values[field_name] = value[0]
                        elif isinstance(value, str):
                            # Handle string data (like status text)
                            field_name = f"{category}/{msg_type}_{key}"
                            latest_values[field_name] = value
                        # Handle nested structures (common in ESC messages)
                        elif hasattr(value, '__dict__'):
                            # Object with attributes
                            for attr_name in dir(value):
                                if not attr_name.startswith('_'):
                                    attr_value = getattr(value, attr_name)
                                    if isinstance(attr_value, (int, float)):
                                        field_name = f"{category}/{msg_type}_{key}_{attr_name}"
                                        latest_values[field_name] = attr_value
                    break  # Message type found in category, no need to check others

        # Send ALL data with the SAME timestamp at regular intervals (10Hz)
        current_time = time.time()
        if current_time - last_send_time >= 0.1:  # Every 100ms
            # Create a single timestamp for ALL data points
            master_timestamp = current_time
            
            # Prepare all data to send with identical timestamp
            batch_data = {}
            
            # Add all telemetry values
            for field_name, value in latest_values.items():
                if isinstance(value, (int, float, str)):
                    batch_data[field_name] = value
            
            # Add flight phase info
            batch_data["flight_phase"] = phase_ids.get(curr_phase, -1)
            batch_data["flight_phase_name"] = curr_phase
            
            # Add debug info
            batch_data["debug/total_messages"] = sum(message_counts.values())
            batch_data["debug/propulsion_messages"] = propulsion_message_count
            batch_data["debug/unique_message_types"] = len(message_counts)
            
            # Send ALL data at once with the SAME master timestamp
            if batch_data:
                send_batch_to_plotjuggler(batch_data, master_timestamp)
            
            last_send_time = current_time

        # Print current flight phase data every 5 seconds
        if current_time - last_print_time >= 5.0:
            print(f"\n=== Flight Phase: {curr_phase} ===")
            print(f"Active synchronized data streams: {len(latest_values)} fields")
            print(f"Total messages received: {sum(message_counts.values())}")
            print(f"Propulsion messages: {propulsion_message_count}")
            print(f"Unique message types: {len(message_counts)}")
            
            # Show top 10 most frequent message types
            top_messages = sorted(message_counts.items(), key=lambda x: x[1], reverse=True)[:10]
            print(f"Top message types: {dict(top_messages)}")
            
            # Show propulsion-specific data if available
            propulsion_data = {}
            for msg_type in telemetry_categories["propulsion"]:
                if msg_type in all_telemetry_data:
                    propulsion_data[msg_type] = all_telemetry_data[msg_type]
            
            if propulsion_data:
                print(f"\nPROPULSION DATA:")
                for msg_type, data in propulsion_data.items():
                    print(f"  {msg_type}: {data}")
            else:
                print(f"\nNo propulsion data received yet.")
            
            # Show sample of latest values for debugging
            sample_keys = list(latest_values.keys())[:20]  # First 20 keys
            if sample_keys:
                print(f"\nSample latest values:")
                for key in sample_keys:
                    print(f"  {key}: {latest_values[key]}")
            
            last_print_time = current_time

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
        import traceback
        traceback.print_exc()