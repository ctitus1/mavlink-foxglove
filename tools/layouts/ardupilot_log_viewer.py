"""Generate a Foxglove layout approximating the ArduPilot UAV Log Viewer.

The UAV Log Viewer (https://github.com/ArduPilot/UAVLogViewer, live at
plot.ardupilot.org) is a Vue app with two halves: a Cesium 3D viewer showing the
vehicle model on its flight path, and a set of floating widgets toggled from the
sidebar. This layout mirrors that split across two tabs.

It reads **dataflash** logs (`ATT`, `GPS`, `VIBE`, `RCIN`, `XKF4` ...), whereas
this bridge carries **MAVLink**, so each widget is reproduced over the closest
MAVLink equivalent. Widget names are the sidebar's own labels.

The seven toggleable widgets in the upstream sidebar are Parameters, Radio
Sticks, Mag Fit Tool, EKF helper, Messages, Attitude, and Sensors. Two of them
have no Foxglove equivalent and are approximated rather than reproduced:

* **Attitude** is an artificial-horizon instrument (`vue-flight-indicators`).
  Foxglove has no attitude-indicator panel, so this uses the 3D pose plus roll
  and pitch gauges.
* **Mag Fit Tool** runs a genetic optimiser over compass samples to propose new
  offsets. That is a computation, not a view; this shows the raw compass axes
  and ArduPilot's own MAG_CAL_REPORT fitness/offsets instead.

**Sensors** upstream decodes `*_DEVID` parameter bitfields into bus/device
names. MAVLink has no equivalent, so this shows SYS_STATUS sensor health
bitmasks, which answer the same practical question of what is present and well.

This layout targets the `ardupilotmega` dialect: EKF_STATUS_REPORT, MEMINFO,
RANGEFINDER, WIND and MAG_CAL_REPORT do not exist in `common`. Run the bridge
with `--autopilot ardupilot`.
"""

from __future__ import annotations

from pathlib import Path

from _builder import (
    gauge,
    grid,
    layout_document,
    log_panel,
    map_panel,
    panel_id,
    plot,
    raw_messages,
    scene_3d,
    series,
    split,
    state_transitions,
    table,
    tabs,
    write_layout,
    xy_plot,
)

V = "/mavlink/1/1"


def build() -> dict:
    panels: dict[str, dict] = {}

    def add(kind: str, slug: str, config: dict) -> str:
        pid = panel_id(kind, f"apm_{slug}")
        panels[pid] = config
        return pid

    # -- Tab 1: the Cesium 3D viewer -------------------------------------
    # Upstream shows the vehicle model on a colour-coded trajectory over
    # satellite imagery, with waypoints and geofences. Foxglove's 3D panel gives
    # the model and frames; the Map panel supplies the geographic track that
    # Cesium's imagery layer provides upstream.
    viewer = add("3D", "cesium", scene_3d(f"{V}/pose", follow_frame="map", title="3D Viewer"))
    track = add(
        "Map", "map",
        map_panel(
            f"{V}/location",
            {
                f"{V}/home_location": "#e15759",
                f"{V}/gps_location": "#4e79a7",
                f"{V}/ekf_origin_location": "#59a14f",
            },
            "Flight Track",
        ),
    )
    ground_track = add(
        "Plot", "ground_track",
        xy_plot(
            "Ground Track [m]",
            f"{V}/LOCAL_POSITION_NED.y",
            [series(f"{V}/LOCAL_POSITION_NED", "x", "Position")],
        ),
    )
    mode_timeline = add(
        "StateTransitions", "modes",
        state_transitions([
            {"value": f"{V}/HEARTBEAT.custom_mode", "label": "Flight Mode"},
            {"value": f"{V}/HEARTBEAT.system_status_enum", "label": "Vehicle State"},
            {"value": f"{V}/HEARTBEAT.base_mode_flags[:]", "label": "Mode Flags"},
        ], "Flight Modes"),
    )

    tab_3d = split(
        "row",
        viewer,
        split("column", track, split("column", ground_track, mode_timeline, 60), 55),
        62,
    )

    # -- Tab 2: the floating widgets --------------------------------------

    # "Attitude" -- artificial horizon; approximated with gauges + rates.
    attitude_roll = add(
        "Gauge", "attitude_roll",
        gauge(f"{V}/ATTITUDE.roll", -3.15, 3.15, "Attitude — Roll [rad]", "rainbow"),
    )
    attitude_pitch = add(
        "Gauge", "attitude_pitch",
        gauge(f"{V}/ATTITUDE.pitch", -1.58, 1.58, "Attitude — Pitch [rad]", "rainbow"),
    )
    attitude_plot = add(
        "Plot", "attitude",
        plot("Attitude", [
            series(f"{V}/ATTITUDE", "roll", "Roll"),
            series(f"{V}/ATTITUDE", "pitch", "Pitch"),
            series(f"{V}/ATTITUDE", "yaw", "Yaw"),
        ], y_label="[rad]"),
    )

    # "Radio Sticks" -- upstream plots RCIN.C1..C4 as two stick positions.
    radio_sticks = add(
        "Plot", "radio_sticks",
        plot("Radio Sticks", [
            series(f"{V}/RC_CHANNELS", "chan1_raw", "C1 Roll"),
            series(f"{V}/RC_CHANNELS", "chan2_raw", "C2 Pitch"),
            series(f"{V}/RC_CHANNELS", "chan3_raw", "C3 Throttle"),
            series(f"{V}/RC_CHANNELS", "chan4_raw", "C4 Yaw"),
        ], y_label="[us]"),
    )
    radio_aux = add(
        "Plot", "radio_aux",
        plot("RC Input 5-8", [
            series(f"{V}/RC_CHANNELS", f"chan{n}_raw", f"C{n}") for n in range(5, 9)
        ], y_label="[us]"),
    )

    # "Parameters" -- upstream is a searchable PARM table.
    parameters = add("Table", "parameters", table(f"{V}/PARAM_VALUE", "Parameters"))

    # "Messages" -- STATUSTEXT list.
    messages = add("Log", "messages", log_panel(f"{V}/log", "Messages"))

    # "EKF helper" -- upstream decodes the XKF4.SS solution-status bitmask.
    ekf_variances = add(
        "Plot", "ekf_variances",
        plot("EKF helper — Variances", [
            series(f"{V}/EKF_STATUS_REPORT", "velocity_variance", "Velocity"),
            series(f"{V}/EKF_STATUS_REPORT", "pos_horiz_variance", "Horizontal Position"),
            series(f"{V}/EKF_STATUS_REPORT", "pos_vert_variance", "Vertical Position"),
            series(f"{V}/EKF_STATUS_REPORT", "compass_variance", "Compass"),
            series(f"{V}/EKF_STATUS_REPORT", "terrain_alt_variance", "Terrain Altitude"),
            series(f"{V}/EKF_STATUS_REPORT", "airspeed_variance", "Airspeed"),
        ], min_y=0),
    )
    ekf_flags = add(
        "StateTransitions", "ekf_flags",
        state_transitions([
            {"value": f"{V}/EKF_STATUS_REPORT.flags_flags[:]", "label": "EKF Solution Status"},
        ], "EKF helper — Solution Status"),
    )

    # "Sensors" -- upstream decodes *_DEVID params; SYS_STATUS health is the
    # nearest MAVLink answer to "what is present and working".
    sensors = add(
        "RawMessages", "sensors",
        raw_messages(f"{V}/SYS_STATUS", "Sensors — Health"),
    )

    # "Mag Fit Tool" -- raw compass axes plus ArduPilot's own calibration report.
    magfit = add(
        "Plot", "magfit",
        plot("Mag Fit Tool — Compass", [
            series(f"{V}/SCALED_IMU", "xmag", "MagX"),
            series(f"{V}/SCALED_IMU", "ymag", "MagY"),
            series(f"{V}/SCALED_IMU", "zmag", "MagZ"),
        ], y_label="[mgauss]"),
    )
    magfit_report = add(
        "Plot", "magfit_report",
        plot("Mag Fit Tool — Calibration", [
            series(f"{V}/MAG_CAL_REPORT", "fitness", "Fitness [mgauss]"),
            series(f"{V}/MAG_CAL_REPORT", "ofs_x", "Offset X"),
            series(f"{V}/MAG_CAL_REPORT", "ofs_y", "Offset Y"),
            series(f"{V}/MAG_CAL_REPORT", "ofs_z", "Offset Z"),
        ]),
    )

    # Preset-style plots from the sidebar's bundled graph catalogue
    # (mavgraphs.xml / mavgraphs2.xml).
    vibration = add(
        "Plot", "vibration",
        plot("Sensors/Accelerometer/Vibration", [
            series(f"{V}/VIBRATION", "vibration_x", "VibeX"),
            series(f"{V}/VIBRATION", "vibration_y", "VibeY"),
            series(f"{V}/VIBRATION", "vibration_z", "VibeZ"),
        ], min_y=0),
    )
    clipping = add(
        "Plot", "clipping",
        plot("Sensors/Accelerometer/Clipping", [
            series(f"{V}/VIBRATION", "clipping_0", "Clip0"),
            series(f"{V}/VIBRATION", "clipping_1", "Clip1"),
            series(f"{V}/VIBRATION", "clipping_2", "Clip2"),
        ], min_y=0),
    )
    power = add(
        "Plot", "power",
        plot("Power/Current and Voltage", [
            series(f"{V}/SYS_STATUS", "voltage_battery", "Volt [mV]"),
            series(f"{V}/SYS_STATUS", "current_battery", "Curr [cA]"),
            series(f"{V}/BATTERY_STATUS", "battery_remaining", "Remaining [%]"),
        ], min_y=0),
    )
    board_power = add(
        "Plot", "board_power",
        plot("Board/Power", [
            series(f"{V}/POWER_STATUS", "Vcc", "Vcc [mV]"),
            series(f"{V}/POWER_STATUS", "Vservo", "Vservo [mV]"),
            series(f"{V}/MEMINFO", "freemem", "Free Memory [bytes]"),
        ], min_y=0),
    )
    speed = add(
        "Plot", "speed",
        plot("Speed/Ground Speed", [
            series(f"{V}/VFR_HUD", "groundspeed", "Ground Speed"),
            series(f"{V}/VFR_HUD", "airspeed", "Airspeed"),
            series(f"{V}/WIND", "speed", "Wind Speed"),
        ], y_label="[m/s]", min_y=0),
    )
    servos = add(
        "Plot", "servos",
        plot("Servos/Servos 1-8", [
            series(f"{V}/SERVO_OUTPUT_RAW", f"servo{n}_raw", f"C{n}") for n in range(1, 9)
        ], y_label="[us]"),
    )
    gps_accuracy = add(
        "Plot", "gps_accuracy",
        plot("Sensors/GPS/GPS Accuracy", [
            series(f"{V}/GPS_RAW_INT", "eph", "HDop"),
            series(f"{V}/GPS_RAW_INT", "epv", "VDop"),
            series(f"{V}/GPS_RAW_INT", "satellites_visible", "NSats"),
            series(f"{V}/GPS_RAW_INT", "fix_type", "Status"),
        ], min_y=0),
    )
    rangefinder = add(
        "Plot", "rangefinder",
        plot("Sensors/Rangefinder", [
            series(f"{V}/RANGEFINDER", "distance", "Distance [m]"),
            series(f"{V}/RANGEFINDER", "voltage", "Voltage [V]"),
        ], min_y=0),
    )

    tab_widgets = grid([
        attitude_plot, attitude_roll, attitude_pitch,
        radio_sticks, radio_aux, parameters,
        messages, ekf_variances, ekf_flags,
        sensors, magfit, magfit_report,
        vibration, clipping, power,
        board_power, speed, servos,
        gps_accuracy, rangefinder,
    ], columns=4)

    root = panel_id("Tab", "apm_log_viewer")
    panels[root] = tabs([("3D", tab_3d), ("Widgets", tab_widgets)])
    return layout_document(panels, root)


if __name__ == "__main__":
    out = Path(__file__).resolve().parents[2] / "layouts" / "ardupilot-log-viewer.json"
    write_layout(out, build())
