from pymavlink import mavutil

def route_message(msg):
    msg_type = msg.get_type()

    if msg_type == "ATTITUDE":
        return {
            'roll': msg.roll,
            'pitch': msg.pitch,
            'yaw': msg.yaw
        }
    
    elif msg_type == "SYS_STATUS":
        return {
            'voltage': msg.voltage_battery,
            'battery': msg.battery_remaining,
            'onboard_control_sensors_present': msg.onboard_control_sensors_present,
            'onboard_control_sensors_enabled': msg.onboard_control_sensors_enabled,
            'onboard_control_sensors_health': msg.onboard_control_sensors_health
        }

    elif msg_type == "ESC_STATUS":
        return {
            'rpm': msg.rpm,
            'voltage': msg.voltage,
            'current': msg.current
        }
        
    elif msg_type == "HEARTBEAT":
        return {
            'system_status': msg.system_status
        }
        
    elif msg_type == "RADIO_STATUS":
        return {
            'rssi': msg.rssi,
            'remrssi': msg.remrssi,
            'txbuf': msg.txbuf,
            'noise': msg.noise,
            'remnoise': msg.remnoise
        }
        
    elif msg_type == "COMMAND_ACK":
        return {
            'command': msg.command,
            'result': msg.result
        }
        
    elif msg_type == "SERVO_OUTPUT_RAW":
        return {
            'servo1_raw': msg.servo1_raw,
            'servo2_raw': msg.servo1_raw,
            'servo3_raw': msg.servo1_raw,
            'servo4_raw': msg.servo1_raw,
            'servo5_raw': msg.servo1_raw,
            'servo6_raw': msg.servo1_raw,
            'servo7_raw': msg.servo1_raw,
            'servo8_raw': msg.servo1_raw,
            'servo9_raw': msg.servo1_raw
        }
        
    elif msg_type == 'GPS_RAW_INT':
        return {
            'fix_type': msg.fix_type,
            'satellites_visible': msg.satellites_visible,
            'eph': msg.eph,
            'lat': msg.lat,
            'lon': msg.lon,
            'alt': msg.alt
        }
        
    elif msg_type == 'ESC_INFO':
        return {
            'temperature': msg.temperature
        }
        
    elif msg_type == 'BATTERY_STATUS':
        return {
            'temperature': msg.temperature,
            'voltages': msg.voltages,
            'current_battery': msg.current_battery,
            'energy_consumed': msg.current_consumed,
            'battery_remaining': msg.battery_remaining
        }
        
    elif msg_type == 'RAW_IMU':
        return {
            'temperature': msg.temperature
        }
        
    elif msg_type == 'SCALED_IMU2':
        return {
            'temperature': msg.temperature
        }
        
    elif msg_type == 'SCALED_IMU3':
        return {
            'temperature': msg.temperature
        }
        
    elif msg_type == 'VFR_HUD':
        return {
            'airspeed': msg.airspeed,
            'climb': msg.climb,
            'alt': msg.alt
        }
        
    elif msg_type == 'ADSB_VEHICLE':
        return {
            'icao_address': msg.icao_address,
            'lat': msg.lat,
            'lon': msg.lon,
            'altitude': msg.altitude,
            'heading': msg.heading,
            'velocity': msg.velocity
        }
        
    elif msg_type == 'SCALED_PRESSURE':
        return {
            'temperature': msg.temperature
        }
        
    elif msg_type == 'STATUSTEXT':
        return {
            'text': msg.text
        }
        
    return None
    
    