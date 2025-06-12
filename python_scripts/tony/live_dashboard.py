import dash
from dash import dcc, html
import plotly.graph_objs as go
from pymavlink import mavutil
from datetime import datetime
import threading
import time
import math

# === FLIGHT PHASE DEFINITIONS ===
flight_phase_data = {
    "Preflight": {
        "ESC_STATUS": ["rpm", "voltage", "current"],
        "HEARTBEAT": ["system_status"],
        "RADIO_STATUS": ["rssi", "remrssi", "txbuf"],
        "COMMAND_ACK": ["command", "result"],
        "SERVO_OUTPUT_RAW": [f"servo{i}_raw" for i in range(1, 10)],
        "SYS_STATUS": [
            "onboard_control_sensors_present",
            "onboard_control_sensors_enabled",
            "onboard_control_sensors_health"
        ],
        "GPS_RAW_INT": ["fix_type", "satellites_visible", "eph"]
    },
    "Takeoff / Climb": {
        "ESC_INFO": ["temperature"],
        "ESC_STATUS": ["rpm"],
        "BATTERY_STATUS": ["temperature", "voltages", "current_battery", "energy_consumed"],
        "RAW_IMU": ["temperature"],
        "SCALED_IMU2": ["temperature"],
        "SCALED_IMU3": ["temperature"]
    },
    "Cruise": {
        "RADIO_STATUS": ["rssi", "remrssi", "txbuf", "noise", "remnoise"],
        "GPS_RAW_INT": ["fix_type", "satellites_visible", "eph"],
        "BATTERY_STATUS": ["battery_remaining"],
        "VFR_HUD": ["airspeed"],
        "ADSB_VEHICLE": ["icao_address", "lat", "lon", "altitude", "heading", "velocity"],
        "SCALED_PRESSURE": ["temperature"]
    },
    "Landing": {
        "ATTITUDE": ["pitch"],
        "VFR_HUD": ["airspeed", "climb"],
        "SERVO_OUTPUT_RAW": [f"servo{i}_raw" for i in range(1, 10)],
        "ESC_STATUS": ["rpm"]
    }
}

# === TELEMETRY GROUPS ===
telemetry_groups = {
    "Engine": ["rpm", "voltage", "current", "temperature"],
    "Comms": ["system_status", "rssi", "remrssi", "txbuf", "command", "result", "noise", "remnoise"],
    "Servos": [f"servo{i}_raw" for i in range(1, 10)],
    "Sensors": [
        "onboard_control_sensors_present",
        "onboard_control_sensors_enabled",
        "onboard_control_sensors_health"
    ],
    "GPS": ["fix_type", "satellites_visible", "eph", "lat", "lon", "altitude"],
    "Battery": ["temperature", "voltages", "current_battery", "energy_consumed", "battery_remaining"],
    "Flight Controller": ["temperature"],
    "Airspeed": ["airspeed", "climb"],
    "ADS-B": ["icao_address", "heading", "velocity"],
    "Signal Strength": ["rssi", "remrssi", "txbuf", "noise", "remnoise"],
    "Landing": ["pitch", "airspeed", "climb", "AoA"]
}

# === PHASE DECODER ===
def decode_flight_phase(base_mode, custom_mode):
    armed = (base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
    if not armed:
        return "Preflight"

    main_mode = (custom_mode >> 16) & 0xFF
    if main_mode == 4:  # AUTO
        return "Cruise"
    elif main_mode in [2, 3]:  # ALTCTL, POSCTL
        return "Takeoff / Climb"
    elif main_mode == 1:  # MANUAL
        return "Landing"
    else:
        return "In Flight"

# === TELEMETRY STORAGE ===
telemetry = {
    "flight_phase": "Waiting...",
    "active_phase": None,
    "phase_data": {},
    "lock": threading.Lock(),
    "manual_override": None
}

# === TELEMETRY COLLECTOR ===
def telemetry_collector():
    mav = mavutil.mavlink_connection("udp:127.0.0.1:14540")
    mav.wait_heartbeat()
    print("✅ Connected to drone")

    while True:
        msg = mav.recv_match(blocking=False)
        if not msg:
            time.sleep(0.05)
            continue

        now = datetime.now()
        with telemetry["lock"]:
            if msg.get_type() == "HEARTBEAT":
                phase = decode_flight_phase(msg.base_mode, msg.custom_mode)
                telemetry["flight_phase"] = phase
                telemetry["active_phase"] = phase
                if phase not in telemetry["phase_data"]:
                    telemetry["phase_data"][phase] = {"time": []}

            current_phase = telemetry["active_phase"]
            selected_phase = telemetry["manual_override"] or current_phase
            valid_msgs = flight_phase_data.get(current_phase, {})
            pd = telemetry["phase_data"].setdefault(current_phase, {"time": []})

            if msg.get_type() in valid_msgs:
                fields = valid_msgs[msg.get_type()]
                pd["time"].append(now)
                for f in fields:
                    val = getattr(msg, f, None)
                    pd.setdefault(f, []).append(val)

                # AoA for landing phase
                if current_phase == "Landing":
                    pitch = getattr(msg, "pitch", None)
                    climb = pd.get("climb", [None])[-1]
                    airspeed = pd.get("airspeed", [None])[-1]
                    if pitch is not None and climb and airspeed:
                        aoa = pitch - math.atan(climb / airspeed) if airspeed != 0 else 0
                        pd.setdefault("AoA", []).append(aoa)

                if len(pd["time"]) > 300:
                    for key in pd:
                        pd[key] = pd[key][-300:]

        time.sleep(0.05)

threading.Thread(target=telemetry_collector, daemon=True).start()

# === DASH APP ===
app = dash.Dash(__name__)
app.title = "Flight Phase Telemetry Dashboard"

app.layout = html.Div([
    html.H2("Live Drone Telemetry by Flight Phase"),
    dcc.Dropdown(
        id="phase-selector",
        options=[{"label": p, "value": p} for p in flight_phase_data],
        placeholder="Select a phase or leave blank for automatic",
        style={"width": "50%"}
    ),
    html.Div(id="phase-status", style={"marginTop": 10}),
    dcc.Interval(id="interval", interval=1000, n_intervals=0),
    html.Div(id="graph-container")
])

@app.callback(
    [
        dash.dependencies.Output("phase-status", "children"),
        dash.dependencies.Output("graph-container", "children")
    ],
    [
        dash.dependencies.Input("interval", "n_intervals"),
        dash.dependencies.Input("phase-selector", "value")
    ]
)
def update_display(n, selected_phase):
    with telemetry["lock"]:
        if selected_phase:
            telemetry["manual_override"] = selected_phase
        else:
            telemetry["manual_override"] = None

        current = telemetry["manual_override"] or telemetry["active_phase"]
        pd = telemetry["phase_data"].get(current, {})
        times = pd.get("time", [])

        graphs = []
        for group, fields in telemetry_groups.items():
            traces = []
            for f in fields:
                if f in pd:
                    traces.append(go.Scatter(x=times, y=pd[f], mode="lines", name=f))
            if traces:
                fig = go.Figure(data=traces)
                fig.update_layout(
                    title=f"{group} - {current}",
                    xaxis_title="Time",
                    yaxis_title="Value",
                    height=300,
                    legend=dict(orientation="h")
                )
                graphs.append(dcc.Graph(figure=fig))

        return f"Flight Phase: {telemetry['flight_phase']} (Viewing: {current})", graphs

if __name__ == "__main__":
    app.run(debug=True)
