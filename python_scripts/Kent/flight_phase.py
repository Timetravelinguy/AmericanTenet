from pymavlink import mavutil
from filter_message import route_message


def det_flight_phase(telemetry_data):
    """
    Determine the flight phase based on telemetry data.
    
    Args:
        telemetry_data (dict): Dictionary containing telemetry data such as altitude, vertical speed, armed state, etc.
    
    Returns:
        str: The current flight phase.
    """
    altitude = telemetry_data.get('altitude', 0)
    vertical_speed = telemetry_data.get('vertical_speed', 0)
    armed = telemetry_data.get('armed', False)
    ground_speed = telemetry_data.get('ground_speed', 0)

    if not armed:
        if altitude < 1 and ground_speed < 1:
            return "preflight"
        else:
            return "post_flight"

    if vertical_speed > 2:
        return "takeoff"
    elif vertical_speed < -2:
        return "landing"
    elif ground_speed > 5 and abs(vertical_speed) < 2:
        return "cruise"
    elif ground_speed < 1 and abs(vertical_speed) < 1:
        return "hovering"
    else:
        return "unknown"
    
