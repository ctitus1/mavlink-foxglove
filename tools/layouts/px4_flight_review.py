"""Generate a Foxglove layout approximating PX4 Flight Review's plot page.

Flight Review (https://github.com/PX4/flight_review) plots **uLog** topics --
`vehicle_attitude`, `sensor_combined`, `vehicle_local_position` and friends --
which are PX4's internal uORB messages. This bridge carries **MAVLink**, a
different and much smaller set. So this layout reproduces Flight Review's plot
titles, y-axis units and legend labels verbatim where it can, over the closest
MAVLink equivalent, and omits what MAVLink simply does not carry.

Plot titles and legend strings are taken from `app/plot_app/configured_plots.py`
in the flight_review repository.

Two systematic differences are worth knowing before reading a plot:

* **Angles are radians, not degrees.** Flight Review converts uLog radians to
  degrees for display; MAVLink's ATTITUDE already carries radians and Foxglove's
  message-path syntax has no arithmetic, so no conversion is possible. Titles
  say `[rad]` where Flight Review says `[deg]`.
* **Several fields are scaled integers.** MAVLink sends battery voltage in mV,
  current in cA, and temperatures in centi-degrees. Legends name the real unit
  rather than pretending otherwise.

Omitted entirely, because no MAVLink message carries the data: all FFT and
power-spectral-density plots, the FIFO sensor plots, sampling regularity,
visual odometry, CPU/RAM (`cpuload`), and GPS noise/jamming.
"""

from __future__ import annotations

from pathlib import Path

from _builder import (
    grid,
    layout_document,
    log_panel,
    map_panel,
    panel_id,
    plot,
    series,
    state_transitions,
    tabs,
    write_layout,
    xy_plot,
)

#: Every topic is scoped to one vehicle; edit if your system/component differs.
V = "/mavlink/1/1"


def build() -> dict:
    panels: dict[str, dict] = {}

    def add(kind: str, slug: str, config: dict) -> str:
        pid = panel_id(kind, f"px4_{slug}")
        panels[pid] = config
        return pid

    # -- Position -------------------------------------------------------
    # Flight Review's top-of-page DataPlot2D: x_axis_label='[m]', y_axis_label='[m]',
    # plotting vehicle_local_position (y, x) as "Estimated".
    position_xy = add(
        "Plot", "position_xy",
        xy_plot(
            "Position [m]",
            f"{V}/LOCAL_POSITION_NED.y",
            [series(f"{V}/LOCAL_POSITION_NED", "x", "Estimated")],
        ),
    )
    position_map = add(
        "Map", "map",
        map_panel(f"{V}/location", {f"{V}/home_location": "#e15759"}, "Position (Map)"),
    )
    altitude = add(
        "Plot", "altitude",
        plot(
            "Altitude Estimate", [
                series(f"{V}/ALTITUDE", "altitude_amsl", "GPS Altitude (MSL)"),
                series(f"{V}/ALTITUDE", "altitude_monotonic", "Barometer Altitude"),
                series(f"{V}/ALTITUDE", "altitude_local", "Fused Altitude Estimation"),
                series(f"{V}/ALTITUDE", "altitude_relative", "Altitude Relative to Home"),
            ],
            y_label="[m]",
        ),
    )
    local_x = add(
        "Plot", "local_x",
        plot("Local Position X", [
            series(f"{V}/LOCAL_POSITION_NED", "x", "X Estimated"),
            series(f"{V}/POSITION_TARGET_LOCAL_NED", "x", "X Setpoint"),
        ], y_label="[m]"),
    )
    local_y = add(
        "Plot", "local_y",
        plot("Local Position Y", [
            series(f"{V}/LOCAL_POSITION_NED", "y", "Y Estimated"),
            series(f"{V}/POSITION_TARGET_LOCAL_NED", "y", "Y Setpoint"),
        ], y_label="[m]"),
    )
    local_z = add(
        "Plot", "local_z",
        plot("Local Position Z", [
            series(f"{V}/LOCAL_POSITION_NED", "z", "Z Estimated"),
            series(f"{V}/POSITION_TARGET_LOCAL_NED", "z", "Z Setpoint"),
        ], y_label="[m]"),
    )
    velocity = add(
        "Plot", "velocity",
        plot("Velocity", [
            series(f"{V}/LOCAL_POSITION_NED", "vx", "X"),
            series(f"{V}/LOCAL_POSITION_NED", "vy", "Y"),
            series(f"{V}/LOCAL_POSITION_NED", "vz", "Z"),
            series(f"{V}/POSITION_TARGET_LOCAL_NED", "vx", "X Setpoint"),
            series(f"{V}/POSITION_TARGET_LOCAL_NED", "vy", "Y Setpoint"),
            series(f"{V}/POSITION_TARGET_LOCAL_NED", "vz", "Z Setpoint"),
        ], y_label="[m/s]"),
    )

    # -- Attitude and rates ---------------------------------------------
    # Flight Review emits "{Axis} Angle" [deg] and "{Axis} Angular Rate" [deg/s]
    # per axis. MAVLink carries radians, so the units differ; see module docs.
    angles = []
    rates = []
    for axis, rate_field in (("Roll", "rollspeed"), ("Pitch", "pitchspeed"), ("Yaw", "yawspeed")):
        lower = axis.lower()
        angles.append(add(
            "Plot", f"{lower}_angle",
            plot(f"{axis} Angle", [
                series(f"{V}/ATTITUDE", lower, f"{axis} Estimated"),
            ], y_label="[rad]"),
        ))
        rates.append(add(
            "Plot", f"{lower}_rate",
            plot(f"{axis} Angular Rate", [
                series(f"{V}/ATTITUDE", rate_field, f"{axis} Rate Estimated"),
                series(f"{V}/ATTITUDE_TARGET", f"body_{lower}_rate", f"{axis} Rate Setpoint"),
            ], y_label="[rad/s]"),
        ))

    airspeed = add(
        "Plot", "airspeed",
        plot("Airspeed", [
            series(f"{V}/VFR_HUD", "groundspeed", "Ground Speed Estimated"),
            series(f"{V}/VFR_HUD", "airspeed", "True Airspeed"),
        ], y_label="[m/s]"),
    )
    tecs = add(
        "Plot", "tecs",
        plot("TECS", [
            series(f"{V}/VFR_HUD", "climb", "Height Rate"),
            series(f"{V}/NAV_CONTROLLER_OUTPUT", "alt_error", "Altitude Error"),
            series(f"{V}/NAV_CONTROLLER_OUTPUT", "aspd_error", "Airspeed Error"),
        ], y_label="[m/s]"),
    )

    # -- Control inputs and outputs --------------------------------------
    # Matches Flight Review's legacy "Raw Radio Control Inputs" branch, which
    # plots rc_channels with "Channel N" legends -- the closest analogue to
    # MAVLink's RC_CHANNELS.
    rc = add(
        "Plot", "rc_inputs",
        plot("Raw Radio Control Inputs", [
            series(f"{V}/RC_CHANNELS", f"chan{n}_raw", f"Channel {n}") for n in range(1, 9)
        ]),
    )
    actuators = add(
        "Plot", "actuator_outputs",
        plot("Actuator Outputs (Main)", [
            series(f"{V}/SERVO_OUTPUT_RAW", f"servo{n}_raw", f"Output {n - 1}")
            for n in range(1, 9)
        ]),
    )
    actuator_controls = add(
        "Plot", "actuator_controls",
        plot("Actuator Controls", [
            series(f"{V}/ATTITUDE_TARGET", "body_roll_rate", "Roll"),
            series(f"{V}/ATTITUDE_TARGET", "body_pitch_rate", "Pitch"),
            series(f"{V}/ATTITUDE_TARGET", "body_yaw_rate", "Yaw"),
            series(f"{V}/ATTITUDE_TARGET", "thrust", "Thrust (up)"),
        ]),
    )

    # -- Raw sensors ------------------------------------------------------
    raw_accel = add(
        "Plot", "raw_accel",
        plot("Raw Acceleration", [
            series(f"{V}/HIGHRES_IMU", "xacc", "X"),
            series(f"{V}/HIGHRES_IMU", "yacc", "Y"),
            series(f"{V}/HIGHRES_IMU", "zacc", "Z"),
        ], y_label="[m/s^2]"),
    )
    raw_gyro = add(
        "Plot", "raw_gyro",
        plot("Raw Angular Speed (Gyroscope)", [
            series(f"{V}/HIGHRES_IMU", "xgyro", "X"),
            series(f"{V}/HIGHRES_IMU", "ygyro", "Y"),
            series(f"{V}/HIGHRES_IMU", "zgyro", "Z"),
        ], y_label="[rad/s]"),
    )
    raw_mag = add(
        "Plot", "raw_mag",
        plot("Raw Magnetic Field Strength", [
            series(f"{V}/HIGHRES_IMU", "xmag", "X"),
            series(f"{V}/HIGHRES_IMU", "ymag", "Y"),
            series(f"{V}/HIGHRES_IMU", "zmag", "Z"),
        ], y_label="[gauss]"),
    )
    vibration = add(
        "Plot", "vibration",
        plot("Vibration Metrics", [
            series(f"{V}/VIBRATION", "vibration_x", "Vibration Level X [m/s^2]"),
            series(f"{V}/VIBRATION", "vibration_y", "Vibration Level Y [m/s^2]"),
            series(f"{V}/VIBRATION", "vibration_z", "Vibration Level Z [m/s^2]"),
            series(f"{V}/VIBRATION", "clipping_0", "Accel 0 Clipping"),
            series(f"{V}/VIBRATION", "clipping_1", "Accel 1 Clipping"),
            series(f"{V}/VIBRATION", "clipping_2", "Accel 2 Clipping"),
        ], min_y=0),
    )
    distance = add(
        "Plot", "distance_sensor",
        plot("Distance Sensor", [
            series(f"{V}/ALTITUDE", "bottom_clearance", "Estimated Distance Bottom [m]"),
            series(f"{V}/ALTITUDE", "altitude_terrain", "Terrain Altitude [m]"),
        ], y_label="[m]", min_y=0),
    )
    temperature = add(
        "Plot", "temperature",
        plot("Temperature", [
            series(f"{V}/HIGHRES_IMU", "temperature", "IMU temperature"),
            series(f"{V}/SCALED_PRESSURE", "temperature", "Baro temperature [cdegC]"),
            series(f"{V}/BATTERY_STATUS", "temperature", "Battery temperature [cdegC]"),
        ], y_label="[C]"),
    )

    # -- GPS ---------------------------------------------------------------
    gps_uncertainty = add(
        "Plot", "gps_uncertainty",
        plot("GPS Uncertainty", [
            series(f"{V}/GPS_RAW_INT", "eph", "Horizontal position accuracy [m]"),
            series(f"{V}/GPS_RAW_INT", "epv", "Vertical position accuracy [m]"),
            series(f"{V}/GPS_RAW_INT", "h_acc", "Horizontal accuracy [mm]"),
            series(f"{V}/GPS_RAW_INT", "v_acc", "Vertical accuracy [mm]"),
            series(f"{V}/GPS_RAW_INT", "vel_acc", "Speed accuracy [mm/s]"),
            series(f"{V}/GPS_RAW_INT", "satellites_visible", "Num Satellites used"),
            series(f"{V}/GPS_RAW_INT", "fix_type", "GPS Fix"),
        ], min_y=0, max_y=40),
    )
    estimator = add(
        "Plot", "estimator",
        plot("Estimator Innovation Test Ratios", [
            series(f"{V}/ESTIMATOR_STATUS", "vel_ratio", "Velocity"),
            series(f"{V}/ESTIMATOR_STATUS", "pos_horiz_ratio", "Horizontal Position"),
            series(f"{V}/ESTIMATOR_STATUS", "pos_vert_ratio", "Vertical Position"),
            series(f"{V}/ESTIMATOR_STATUS", "mag_ratio", "Magnetometer"),
            series(f"{V}/ESTIMATOR_STATUS", "hagl_ratio", "Height above Ground"),
            series(f"{V}/ESTIMATOR_STATUS", "tas_ratio", "True Airspeed"),
        ], min_y=0),
    )
    estimator_accuracy = add(
        "Plot", "estimator_accuracy",
        plot("Estimator Accuracy", [
            series(f"{V}/ESTIMATOR_STATUS", "pos_horiz_accuracy", "Horizontal position accuracy [m]"),
            series(f"{V}/ESTIMATOR_STATUS", "pos_vert_accuracy", "Vertical position accuracy [m]"),
        ], y_label="[m]", min_y=0),
    )

    # -- Power and system ---------------------------------------------------
    power = add(
        "Plot", "power",
        plot("Power", [
            series(f"{V}/SYS_STATUS", "voltage_battery", "Battery Voltage [mV]"),
            series(f"{V}/SYS_STATUS", "current_battery", "Battery Current [cA]"),
            series(f"{V}/SYS_STATUS", "battery_remaining", "Battery remaining [%]"),
            series(f"{V}/BATTERY_STATUS", "current_consumed", "Discharged Amount [mAh]"),
        ], min_y=0),
    )
    cpu = add(
        "Plot", "cpu",
        plot("CPU & RAM", [
            series(f"{V}/SYS_STATUS", "load", "CPU Load [d%]"),
            series(f"{V}/SYS_STATUS", "drop_rate_comm", "Comm Drop Rate [d%]"),
            series(f"{V}/SYS_STATUS", "errors_comm", "Comm Errors"),
        ], min_y=0),
    )
    telemetry = add(
        "Plot", "telemetry",
        plot("Telemetry Link", [
            series(f"{V}/RADIO_STATUS", "rssi", "RSSI"),
            series(f"{V}/RADIO_STATUS", "remrssi", "Remote RSSI"),
            series(f"{V}/RADIO_STATUS", "noise", "Noise"),
            series(f"{V}/RADIO_STATUS", "remnoise", "Remote Noise"),
        ], min_y=0),
    )
    flags = add(
        "StateTransitions", "flags",
        state_transitions([
            {"value": f"{V}/HEARTBEAT.system_status_enum", "label": "Vehicle State"},
            {"value": f"{V}/HEARTBEAT.base_mode_flags[:]", "label": "Mode Flags"},
            {"value": f"{V}/GPS_RAW_INT.fix_type_enum", "label": "GPS Fix"},
        ], "Failsafe & Status Flags"),
    )
    messages = add("Log", "messages", log_panel(f"{V}/log", "Logged Messages"))

    root = panel_id("Tab", "px4_flight_review")
    panels[root] = tabs([
        # A panel ID may appear only once in the mosaic tree, so Velocity lives
        # with the local-position plots, matching Flight Review's own ordering.
        ("Position", grid([position_xy, position_map, altitude], columns=2)),
        ("Local Position", grid([local_x, local_y, local_z, velocity], columns=2)),
        ("Attitude", grid(angles + rates, columns=2)),
        ("Control", grid([actuator_controls, rc, actuators, airspeed, tecs], columns=2)),
        ("Sensors", grid([raw_accel, raw_gyro, raw_mag, vibration, distance, temperature], columns=2)),
        ("GPS & Estimator", grid([gps_uncertainty, estimator, estimator_accuracy], columns=2)),
        ("Power & System", grid([power, cpu, telemetry, flags, messages], columns=2)),
    ])
    return layout_document(panels, root)


if __name__ == "__main__":
    out = Path(__file__).resolve().parents[2] / "layouts" / "px4-flight-review.json"
    write_layout(out, build())
