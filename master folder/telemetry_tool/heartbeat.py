from pymavlink import mavutil
import time

class Heartbeat:
    def __init__(self, master_storage):
        # Initialize the Heartbeat w/ a reference to master_storage.
        self.master_storage = master_storage
        
    def decode_px4_mode(self, custom_mode):
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

    '''
    @brief Determine flight phase from current flight mode, previous flight data, and current flight data
    '''
    def det_flight_phase(self, sub_mode_str, prev_phase):
        """
        Determine flight phase from current flight mode, previous flight data, and telemetry data.

        Parameters:
            sub_mode_str (str): The sub-mode string from the heartbeat message.
            prev_phase (str): The previous flight phase.

        Returns:
            str: The current flight phase.
        """
        
        # Access altitude from GPS_RAW_INT in master_storage
        altitude = self.master_storage.get("GPS_RAW_INT", {}).get("alt", None)
        
        # Access vertical velocity from GLOBAL_POSITION_INT in master_storage
        vz = self.master_storage.get("GLOBAL_POSITION_INT", {}).get("vz", None)
        
        # Access horizontal velocity from GLOBAL_POSITION_INT in master_storage
        # horizontal x velocity
        vx = self.master_storage.get("GLOBAL_POSITION_INT", {}).get("vx", None)
        # horizontal y velocity
        vy = self.master_storage.get("GLOBAL_POSITION_INT", {}).get("vy", None)
        
        # Access rpm from ESC_STATUS in master_storage
        rpm = self.master_storage.get("ESC_STATUS", {}).get("rpm", None)
        
        # TAKEOFF
        if sub_mode_str == "TAKEOFF":
            return "TAKEOFF"

        # PREFLIGHT
        elif prev_phase == "PREFLIGHT" or prev_phase is None:
            return "PREFLIGHT"

        # HOVERING
        elif sub_mode_str == "LOITER" and prev_phase in ["TAKEOFF", "CRUISING"]:
            # Ensure not descending (vz > 0 indicates descending in MAVLink)
            if not (vz and vz > 0):
                return "HOVERING"

        # LANDING
        # If sub_mode_str indicates LAND or descending near the ground
        elif sub_mode_str == "LAND" or (altitude and altitude <= 0.5 and vz and vz > 0):
            return "LANDING"

        # POST FLIGHT
        elif sub_mode_str == "LOITER" and prev_phase == "LANDING":
            return "POST FLIGHT"

        # Fallback
        return prev_phase

    def process_heartbeat(self, message, current_phase, previous_phase, heartbeat_time):
        """
        Process a heartbeat message and update the current and previous flight phases.

        Parameters:
            message: The MAVLink heartbeat message.
            current_phase (str): The current flight phase.
            previous_phase (str): The previous flight phase.
            heartbeat_time (float): The last time a heartbeat was processed.

        Returns:
            tuple: (current_phase, previous_phase, heartbeat_time)
        """
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
