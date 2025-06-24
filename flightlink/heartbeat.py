## @file heartbeat.py
#  @brief Decodes PX4 heartbeat messages and determines flight phases.
#
#  @details This script provides functions to decode PX4-specific custom_mode values from MAVLink
#  HEARTBEAT messages and determines the current flight phase based on the sub-mode. It is designed
#  to support real-time drone monitoring and telemetry visualization.
#
#  @author American Tenet
#  @date 2025-06-20
#  @version 1.0
from pymavlink import mavutil
import time

## @brief Decodes PX4 heartbeat messages and determines flight phases.
class Heartbeat:
    ## @brief Initializes the Heartbeat class.
    #  @param master_storage Reference to telemetry storage.
    def __init__(self, master_storage):
        # Initialize the Heartbeat w/ a reference to master_storage.
        self.master_storage = master_storage

    ## @brief Decodes PX4 flight modes from custom_mode.
    #  @param custom_mode 32-bit custom_mode value.
    #  @return Tuple of main and sub-mode strings.
    def decode_px4_mode(self, custom_mode):

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


    ## @brief Determine the drone's flight phase from mode transitions.
    #  @param sub_mode_str The current PX4 sub-mode string.
    #  @param prev_phase The previously recorded flight phase.
    #  @return The current flight phase as a string.
    def det_flight_phase(self, sub_mode_str, prev_phase):
        
        # Access altitude from GPS_RAW_INT in master_storage.
        altitude = self.master_storage.get("ADSB_VEHICLE", {}).get("altitude", None)
        
        # Access vertical velocity from GLOBAL_POSITION_INT in master_storage.
        vz = self.master_storage.get("GLOBAL_POSITION_INT", {}).get("vz", None)
        
        # Access horizontal velocity from GLOBAL_POSITION_INT in master_storage.
        # horizontal x velocity
        vx = self.master_storage.get("GLOBAL_POSITION_INT", {}).get("vx", None)
        # horizontal y velocity
        vy = self.master_storage.get("GLOBAL_POSITION_INT", {}).get("vy", None)
        
        # Access rpm from ESC_STATUS in master_storage.
        rpm = self.master_storage.get("ESC_STATUS", {}).get("rpm", None)
        
        # TAKEOFF
        if sub_mode_str == "TAKEOFF":
            return "TAKEOFF"

        # PREFLIGHT
        elif prev_phase == "PREFLIGHT" or prev_phase is None:
            return "PREFLIGHT"

        # LANDING
        elif sub_mode_str == "LAND":
            return "LANDING"

        # POST FLIGHT
        elif sub_mode_str == "LOITER" and prev_phase == "LANDING":
            return "POST FLIGHT"

        # HOVERING
        elif sub_mode_str == "LOITER" and prev_phase in ["TAKEOFF", "CRUISING", "SOARING IN THERMAL"]:
            # Ensure not descending (vz > 0 indicates descending in MAVLink).
            if not (altitude and altitude > 0.05):
                return "HOVERING"
            
        # CRUISING
        elif prev_phase in ["HOVERING", "TAKEOFF", "SOARING IN THERMAL"]:
            # If the drone is moving in a horizontal direction greater than 0.1 m/s
            if (vx > 0.05 or vy > 0.05) and (vz is None or vz <= 0.05):
                return "CRUISING" 
        
        # SOARING IN THERMAL
        elif sub_mode_str in ["HOVERING", "TAKEOFF", "CRUISING"]:
            # If the drone's rpm is low and vertical speed ascending
            if rpm < 1000 and vz > 0.05:
                return "SOARING IN THERMAL"

        # Fallback
        return prev_phase

    ## @brief Processes a MAVLink HEARTBEAT message.
    #  @param message MAVLink HEARTBEAT message.
    #  @param current_phase Current flight phase.
    #  @param previous_phase Last known flight phase.
    #  @param heartbeat_time Timestamp of last heartbeat.
    #  @return Updated (current_phase, previous_phase, heartbeat_time).
    def process_heartbeat(self, message, current_phase, previous_phase, heartbeat_time):

        # Uncomment for debugging
        #print(f"[DEBUG] HEARTBEAT received: {msg}")

        # Log current time
        now = time.time()

        # Limit HEARTBEAT decoding to once per second
        if now - heartbeat_time >= 1:
            heartbeat_time = now

            if hasattr(message, "custom_mode"):
                main_mode_str, sub_mode_str = self.decode_px4_mode(message.custom_mode)

                # Uncomment for debugging
                #print(f"PX4 Flight Mode: {main_mode_str} - {sub_mode_str}")

                if main_mode_str != "Unknown (0)":
                    current_phase = self.det_flight_phase(sub_mode_str, previous_phase)
                    if current_phase != previous_phase:
                        previous_phase = current_phase

        return current_phase, previous_phase, float(heartbeat_time)
