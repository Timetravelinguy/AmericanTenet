from mavlink_connect import connect
from filter_message import route_message
from flight_phase import det_flight_phase
from datetime import datetime
import time


def main():
    phase_interest = {'preflight': ['RADIO_STATUS', 'SYS_STATUS'],
                      'takeoff': ['ESC_STATUS', 'BATTERY_STATUS', 'VFR_HUD'],
                      'cruise': ['GPS_RAW_INT', 'VFR_HUD', 'RADIO_STATUS'],
                      'landing': ['ATTITUDE', 'VFR_HUD'],
                      'post_flight': ['BATTERY_STATUS', 'STATUSTEXT'],
                      'soaring_in_thermal': ['GPS_RAW_INT', 'ATTITUDE', 'VFR_HUD']}
    
    # Setup UDP Connection w/ QGC
    the_connection = connect('udpin', '14445')
    
    print("\nListening for MAVLink messages...\n")
    
    try:
        latest_msgs = {}
        telemetry_data = {}
        # Receive all MAVLink messages in real-time
        while True:
            # Receive and determine message type
            msg = the_connection.recv_match(type=None, blocking=True, timeout=5)
            if not msg:
                print("⚠️ No MAVLink message received (timeout)")
                continue

            msg_type = msg.get_type()
            print(f"Received: {msg_type} from system {msg.get_srcSystem()} component {msg.get_srcComponent()}")
            print(f"[{datetime.now().strftime('%H:%M:%S')}] Received: {msg_type}")
            
            # Update telemetry data based on message type
            if msg_type == "VFR_HUD":
                telemetry_data['altitude'] = msg.alt
                telemetry_data['vertical_speed'] = msg.climb
                telemetry_data['ground_speed'] = msg.groundspeed
                print(f"Telemetry Update (VFR_HUD): {telemetry_data}")
            elif msg_type == "SYS_STATUS":
                # Determine armed state from SYS_STATUS
                telemetry_data['armed'] = msg.onboard_control_sensors_enabled & 128 != 0  # Check bit 7
                print(f"Telemetry Update (SYS_STATUS): {telemetry_data}")
            elif msg_type == "GLOBAL_POSITION_INT":
                telemetry_data['altitude'] = msg.alt / 1000.0  # Convert millimeters to meters
                telemetry_data['ground_speed'] = msg.vx / 100.0  # Convert cm/s to m/s
                print(f"Telemetry Update (GLOBAL_POSITION_INT): {telemetry_data}")
            elif msg_type == "GPS_RAW_INT":
                telemetry_data['altitude'] = msg.alt / 1000.0  # Convert millimeters to meters
                print(f"Telemetry Update (GPS_RAW_INT): {telemetry_data}")

            # Store the latest message
            latest_msgs[msg_type] = msg

            # Determine flight phase
            print(f"Telemetry Data for Phase Detection: {telemetry_data}")
            flight_phase = det_flight_phase(telemetry_data)
            print(f"Determined Flight Phase: {flight_phase}")

            # Display relevant flight phase fields
            print(f"[{datetime.now().strftime('%H:%M:%S')}] Current Phase: {flight_phase.upper()}")
            for mt in phase_interest.get(flight_phase, []):
                if mt in latest_msgs:
                    # Retrieve the latest message for the relevant type
                    message = latest_msgs[mt]
                    data = route_message(message)
                    if data:
                        print(f"[{datetime.now().strftime('%H:%M:%S')}] {mt}: {data}")
                    else:
                        print(f"[{datetime.now().strftime('%H:%M:%S')}] {mt}: No data available")
                else:
                    print(f"[{datetime.now().strftime('%H:%M:%S')}] {mt}: No message received yet")

            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nStopped by user.")
    except Exception as e:
        print(f"🚨 Error occurred: {e}")


main()