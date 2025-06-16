from pymavlink import mavutil

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

    # Uncomment to debug raw byte values
    # print(f"Main Mode: {main_mode}")
    # print(f"Sub Mode: {sub_mode}")

    # Mapping from PX4 main mode values
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

    # Sub mode names for AUTO mode
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

    # Sub mode names for POSCTL mode
    sub_mode_posctl_names = {
        0: "POSCTL POSCTL",
        1: "POSCTL ORBIT",
        2: "POSCTL SLOW"
    }

    # Get the readable string for the main mode
    main_mode_str = main_mode_names.get(main_mode, f"Unknown ({main_mode})")

    # Get the corresponding sub mode string based on the main mode
    if main_mode == 4:  # AUTO
        sub_mode_str = sub_mode_auto_names.get(sub_mode, f"Unknown ({sub_mode})")
    elif main_mode == 3:  # POSCTL
        sub_mode_str = sub_mode_posctl_names.get(sub_mode, f"Unknown ({sub_mode})")
    elif main_mode == 0:
        # Uninitialized / idle state
        return "Unknown (0)", "N/A"
    else:
        sub_mode_str = "N/A"

    return main_mode_str, sub_mode_str
