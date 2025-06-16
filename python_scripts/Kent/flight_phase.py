def det_flight_phase(sub_mode_str, prev_phase):
    # TAKEOFF
    if sub_mode_str == "TAKEOFF":
        return "TAKEOFF"
    
    # PREFLIGHT
    elif prev_phase == "PREFLIGHT" or prev_phase is None:
        return "PREFLIGHT"
    
    # HOVERING
    elif sub_mode_str == "LOITER" and prev_phase in ["TAKEOFF", "CRUISING"]:
        return "HOVERING"
    
    # CRUISING
    elif sub_mode_str == "MISSION" and prev_phase in ["HOVERING", "SOARING IN THERMAL"]:
        return "CRUISING"
    
    # SOARING IN THERMAL
    elif sub_mode_str == "LOITER" and prev_phase == "CRUISING":
        return "SOARING IN THERMAL"
    
    # LANDING
    elif sub_mode_str == "LAND":
        return "LANDING"
    
    # POST FLIGHT
    elif sub_mode_str == "LOITER" and prev_phase == "LANDING":
        return "POST FLIGHT"
    
    # Fallback
    return prev_phase
