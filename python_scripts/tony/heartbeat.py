from pymavlink import mavutil
import time

def decode_px4_mode(custom_mode):
    """
    Decode the custom_mode value from a PX4 HEARTBEAT message.
    Extracts the main and sub flight modes based on PX4 custom_mode bit layout.

    Parameters:
        custom_mode (int): The 32-bit custom_mode value from the HEARTBEAT message.

    Returns:
        tuple: (main_mode_str, sub_mode_str) representing decoded mode names.
    """

    # Extract main mode (3rd byte) and sub mode (4th byte)
    main_mode = (custom_mode >> 16) & 0xFF
    sub_mode = (custom_mode >> 24) & 0xFF

    print(f"[DEBUG] custom_mode: {custom_mode}")
    print(f"[DEBUG] Main Mode: {main_mode}, Sub Mode: {sub_mode}")

    main_mode_names = {
        1: "MANUAL",
        2: "ALTCTL",
        3: "POSCTL",
        4: "AUTO",
        5: "ACRO",
        6: "OFFBOARD",
        7: "STABILIZED",
        8: "RATTITUDE LEGACY",
        9: "SIMPLE",
        10: "TERMINATION"
    }

    sub_mode_auto_names = {
        1: "READY",
        2: "TAKEOFF",
        3: "LOITER",
        4: "MISSION",
        5: "RTL",
        6: "LAND",
        7: "RESERVED",
        8: "FOLLOW TARGET",
        9: "PRECLAND",
        10: "VTOL TAKEOFF",
        11: "EXTERNAL1",
        12: "EXTERNAL2",
        13: "EXTERNAL3",
        14: "EXTERNAL4",
        15: "EXTERNAL5",
        16: "EXTERNAL6",
        17: "EXTERNAL7",
        18: "EXTERNAL8"
    }

    sub_mode_posctl_names = {
        0: "POSCTL POSCTL",
        1: "POSCTL ORBIT",
        2: "POSCTL SLOW"
    }

    main_mode_str = main_mode_names.get(main_mode, f"Unknown ({main_mode})")

    if main_mode == 4:  # AUTO
        sub_mode_str = sub_mode_auto_names.get(sub_mode, f"Unknown ({sub_mode})")
    elif main_mode == 3:  # POSCTL
        sub_mode_str = sub_mode_posctl_names.get(sub_mode, f"Unknown ({sub_mode})")
    elif main_mode == 0:
        return "Unknown (0)", "N/A"
    else:
        sub_mode_str = "N/A"

    print(f"[DEBUG] PX4 Mode: {main_mode_str} – {sub_mode_str}")
    return main_mode_str, sub_mode_str

def det_flight_phase(sub_mode_str, prev_phase):
    if sub_mode_str == "TAKEOFF":
        return "TAKEOFF"
    elif sub_mode_str == "LOITER" and prev_phase == "PREFLIGHT":
        return "HOVERING"
    elif prev_phase == "PREFLIGHT" or prev_phase is None:
        return "PREFLIGHT"
    elif sub_mode_str == "LOITER" and prev_phase in ["TAKEOFF", "CRUISING"]:
        return "HOVERING"
    elif sub_mode_str == "MISSION" and prev_phase in ["HOVERING", "SOARING IN THERMAL"]:
        return "CRUISING"
    elif sub_mode_str == "LOITER" and prev_phase == "CRUISING":
        return "SOARING IN THERMAL"
    elif sub_mode_str == "LAND":
        return "LANDING"
    elif sub_mode_str == "LOITER" and prev_phase == "LANDING":
        return "POST FLIGHT"
    return prev_phase

def process_heartbeat(message, current_phase, previous_phase, heartbeat_time):
    now = time.time()

    if now - heartbeat_time >= 1:
        heartbeat_time = now

        if hasattr(message, "custom_mode"):
            main_mode_str, sub_mode_str = decode_px4_mode(message.custom_mode)

            if main_mode_str != "Unknown (0)":
                current_phase = det_flight_phase(sub_mode_str, previous_phase)

                if current_phase != previous_phase:
                    print(f"[DEBUG] Phase changed: {previous_phase} → {current_phase}")
                    previous_phase = current_phase

    return current_phase, previous_phase, float(heartbeat_time)
