def det_flight_phase(sub_mode_str, prev_phase):
    # TAKEOFF
    if sub_mode_str == "TAKEOFF":
        return "TAKEOFF"
    
    # PREFLIGHT
    elif sub_mode_str == "LOITER" and prev_phase is None:
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
    
    # Fallback: Handle undefined or stuck phases
    if prev_phase == "PREFLIGHT" and sub_mode_str not in ["TAKEOFF", "MISSION", "LAND"]:
        print("Warning: Stuck in PREFLIGHT phase. Check PX4 mode transitions.")
        return "TAKEOFF"  # Default to TAKEOFF for testing
    
    # UNKNOWN FLIGHT PHASE
    return prev_phase