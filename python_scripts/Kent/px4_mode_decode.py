from pymavlink import mavutil

def decode_px4_mode(custom_mode):
    # custom_mode returns a decimal value (convert to hex to see mapped values for main and sub mode)
    # Extract 3rd and 4th byte from HEARTBEAT custom_mode (main mode = 3rd byte & sub mode = 4th byte)
    main_mode = (custom_mode >> 16) & 0xFF
    sub_mode = (custom_mode >> 24) & 0xFF
    
    ''' Code for debugging main and sub mode values
    print(f"Main Mode: {main_mode}")
    print(f"Sub Mode: {sub_mode}")
    '''

    # Main and Sub mode map ported from px4_custom_mode.h
    main_mode_names = {
        1: "MANUAL",  # Manual
        2: "ALTCTL",  # Altitude Control
        3: "POSCTL",  # Position Control
        4: "AUTO",    # Autonomous
        5: "ACRO",    # Acrobatic
        6: "OFFBOARD",
        7: "STABILIZED",
        8: "RATTITUDE LEGACY",
        9: "SIMPLE", # unused, but reserved for future use
        10: "TERMINATION"
    }

    sub_mode_auto_names = {
        1: "READY",
        2: "TAKEOFF",
        3: "LOITER",
        4: "MISSION",
        5: "RTL",
        6: "LAND",
        7: "RESERVED", # was PX4_CUSTOM_SUB_MODE_AUTO_RTGS, deleted 2020-03-05
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

    # Determine flight sub mode based on flight main mode
    if main_mode == 4:
            sub_mode_str = sub_mode_auto_names.get(sub_mode, f"Unknown ({sub_mode})")
    elif main_mode == 3:
            sub_mode_str = sub_mode_posctl_names.get(sub_mode, f"Unknown ({sub_mode})")
    elif main_mode == 0:
            # Skip uninitialized or idle heartbeats (can uncomment line below for debugging purposes)
            # print("Flight mode: Idle / Not yet set")
            return 0
    else:
            sub_mode_str = "N/A"
            
    return main_mode_str, sub_mode_str

# Determine if valid heartbeat message
def wait_for_hb_mode(hb):
	cm = hb.custom_mode
	main_mode = (cm >> 16) & 0xFF
	if main_mode != 0:
		return hb
	else:
		return 0
	