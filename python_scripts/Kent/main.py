from pymavlink import mavutil
from mavlink_connect import connect
from px4_mode_decode import decode_px4_mode, wait_for_hb_mode
from flight_phase import det_flight_phase
import time

def main():
    # Connect to MAVLink stream (forwarded by QGC)
    the_connection = connect('udpin', '14445')
    
    try:
        fields_by_type = {
            "ADSB_VEHICLE": ["icao_address", "lat", "lon", "altitude", "heading", "velocity"],
            "ATTITUDE": ["roll", "pitch", "yaw"],
            "BATTERY_STATUS":  ["temperature", "voltages", "current_battery", "energy_consumed"],
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
        
        flight_phase_data = {
        phase: [] for phase in types_by_phase
        }

        current_phase = "PREFLIGHT"
        previous_phase = None
        
        # Read messages
        while True:
            
            # Receive ANY type of MAVLink Message
            msg = the_connection.recv_match(blocking=True, timeout=5)
            
            if msg.get_type() == 'HEARTBEAT':
                hb = wait_for_hb_mode(msg)
                if hb == 0:
                    continue
                
                main_mode_str, sub_mode_str = decode_px4_mode(hb.custom_mode)
                
                #print(f"PX4 Flight Mode: {main_mode_str} - {sub_mode_str}")
                
                if current_phase != previous_phase:
                    previous_phase = current_phase
                    current_phase = det_flight_phase(sub_mode_str, previous_phase)
            
            else:
                # Make this part modular later
                msg_type = msg.get_type()
                
                # Skip if message type isn't relevant to current phase
                if msg_type not in types_by_phase.get(current_phase, []):
                    continue
                
                # Skip if message type isn't defined in fields_by_type
                if msg_type not in fields_by_type:
                    continue
                
                # Extract defined fields
                field_names = fields_by_type[msg_type]
                msg_data = {}
                
                for field in field_names:
                    if hasattr(msg, field):
                        msg_data[field] = getattr(msg, field)
                        
                msg_data["timestamp"] = time.time()
                
                # Append to current phase's log
                flight_phase_data[current_phase].append({msg_type: msg_data})
                
            
            # Output relevant telemetry data based on flight phase
            if flight_phase_data[current_phase]:
                latest_data = flight_phase_data[current_phase][-1]
                print(f"{current_phase}: {latest_data}")
            time.sleep(1)
                
    except KeyboardInterrupt:
        print("\nProgram interrupted by user. Exiting...")
        
    except mavutil.mavlink.MAVError as e:
        print(f"\nMAVLink error: {e}")
        
    except Exception as e:
        print(f"\nUnexpected error: {e}")

main()
    
    