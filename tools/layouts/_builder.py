"""Helpers for constructing Foxglove layout documents.

A Foxglove layout is a single JSON object with two halves that must stay in
sync: ``configById`` maps a unique panel ID to that panel's configuration, and
``layout`` is a mosaic tree that arranges those same IDs on screen. Getting the
two out of step yields a layout that imports but renders blank, so panel IDs are
generated once here and reused by both.

Panel IDs are deterministic (``Plot!px4_attitude``) rather than random, so
regenerating a layout produces a clean diff instead of churning every ID.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Iterable

#: Foxglove's default series colours, in order. Reused so that plots which
#: reproduce an upstream tool's ordering also reproduce its colour assignment.
SERIES_COLORS = [
    "#4e79a7", "#f28e2c", "#e15759", "#76b7b2", "#59a14f",
    "#edc949", "#af7aa1", "#ff9da7", "#9c755f", "#bab0ab",
]


def panel_id(kind: str, slug: str) -> str:
    """Build a stable Foxglove panel ID, e.g. ``Plot!px4_attitude``."""
    return f"{kind}!{slug}"


def series(
    topic: str,
    field: str,
    label: str | None = None,
    color: str | None = None,
) -> dict[str, Any]:
    """One plotted series (a "path" in Foxglove terms)."""
    path: dict[str, Any] = {
        "value": f"{topic}.{field}",
        "enabled": True,
        "timestampMethod": "receiveTime",
    }
    if label:
        path["label"] = label
    if color:
        path["color"] = color
    return path


def auto_colored(paths: Iterable[dict[str, Any]]) -> list[dict[str, Any]]:
    """Assign the default colour ramp to any series lacking an explicit colour."""
    out = []
    for index, path in enumerate(paths):
        path = dict(path)
        path.setdefault("color", SERIES_COLORS[index % len(SERIES_COLORS)])
        out.append(path)
    return out


def plot(
    title: str,
    paths: list[dict[str, Any]],
    *,
    y_label: str | None = None,
    legend: str = "floating",
    min_y: float | None = None,
    max_y: float | None = None,
) -> dict[str, Any]:
    """A Plot panel config.

    ``y_label`` is appended to the title because Foxglove's Plot panel has no
    dedicated y-axis label field, and the upstream tools we are mimicking put
    the units there.
    """
    return {
        "paths": auto_colored(paths),
        "foxglovePanelTitle": f"{title} {y_label}" if y_label else title,
        "showLegend": True,
        "legendDisplay": legend,
        "showPlotValuesInLegend": True,
        "showXAxisLabels": True,
        "showYAxisLabels": True,
        "isSynced": True,
        "xAxisVal": "timestamp",
        "sidebarDimension": 240,
        "minYValue": min_y,
        "maxYValue": max_y,
        "followingViewWidth": None,
    }


def xy_plot(
    title: str, x_path: str, y_paths: list[dict[str, Any]]
) -> dict[str, Any]:
    """A Plot panel with a message path on the X axis, for ground tracks."""
    config = plot(title, y_paths, legend="floating")
    config["xAxisVal"] = "custom"
    config["xAxisPath"] = {"value": x_path, "enabled": True, "timestampMethod": "receiveTime"}
    config["isSynced"] = False
    return config


def raw_messages(topic: str, title: str | None = None) -> dict[str, Any]:
    config: dict[str, Any] = {"topicPath": topic, "diffEnabled": False, "expansion": "all"}
    if title:
        config["foxglovePanelTitle"] = title
    return config


def table(topic: str, title: str | None = None) -> dict[str, Any]:
    config: dict[str, Any] = {"topicPath": topic}
    if title:
        config["foxglovePanelTitle"] = title
    return config


def log_panel(topic: str, title: str = "Messages") -> dict[str, Any]:
    return {
        "topicToRender": topic,
        "minLogLevel": 1,
        "searchTerms": [],
        "foxglovePanelTitle": title,
    }


def map_panel(
    follow_topic: str, extra_topics: dict[str, str], title: str = "Map"
) -> dict[str, Any]:
    return {
        "center": None,
        "customTileUrl": "",
        "disabledTopics": [],
        "followTopic": follow_topic,
        "layer": "map",
        "topicColors": extra_topics,
        "zoomLevel": 18,
        "maxNativeZoom": 18,
        "foxglovePanelTitle": title,
    }


def gauge(
    path: str, min_value: float, max_value: float, title: str,
    color_map: str = "red-yellow-green", reverse: bool = False,
) -> dict[str, Any]:
    return {
        "path": path,
        "minValue": min_value,
        "maxValue": max_value,
        "colorMode": "colormap",
        "colorMap": color_map,
        "gradient": ["#0000ff", "#ff00ff"],
        "reverse": reverse,
        "foxglovePanelTitle": title,
    }


def indicator(path: str, rules: list[dict[str, Any]], title: str,
              fallback_label: str = "—") -> dict[str, Any]:
    return {
        "path": path,
        "style": "bulb",
        "fallbackColor": "#7f7f7f",
        "fallbackLabel": fallback_label,
        "rules": rules,
        "foxglovePanelTitle": title,
    }


def state_transitions(paths: list[dict[str, Any]], title: str) -> dict[str, Any]:
    return {
        "paths": [
            {"value": p["value"], "timestampMethod": "receiveTime", "label": p.get("label")}
            for p in paths
        ],
        "isSynced": True,
        "foxglovePanelTitle": title,
    }


def scene_3d(
    pose_topic: str,
    follow_frame: str = "map",
    *,
    title: str = "3D",
    extra_topics: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """A 3D panel showing the vehicle pose against a metric grid."""
    topics: dict[str, Any] = {pose_topic: {"visible": True}}
    topics.update(extra_topics or {})
    return {
        "cameraState": {
            "distance": 60,
            "perspective": True,
            "phi": 55,
            "target": [0, 0, 0],
            "targetOffset": [0, 0, 0],
            "targetOrientation": [0, 0, 0, 1],
            "thetaOffset": 45,
            "fovy": 45,
            "near": 0.5,
            "far": 5000,
        },
        "followMode": "follow-pose",
        "followTf": follow_frame,
        "scene": {
            "enableStats": False,
            "transforms": {"showLabel": True, "axisScale": 2},
        },
        "transforms": {
            "frame:map": {"visible": True},
            "frame:base_link": {"visible": True},
        },
        "topics": topics,
        "layers": {
            "grid": {
                "layerId": "foxglove.Grid",
                "instanceId": "grid",
                "label": "Grid",
                "visible": True,
                "frameId": "map",
                "size": 200,
                "divisions": 40,
                "lineWidth": 1,
                "color": "#7a7a7a60",
                "position": [0, 0, 0],
                "rotation": [0, 0, 0],
                "order": 1,
            }
        },
        "publish": {
            "type": "point",
            "poseTopic": "/move_base_simple/goal",
            "pointTopic": "/clicked_point",
            "poseEstimateTopic": "/initialpose",
            "poseEstimateXDeviation": 0.5,
            "poseEstimateYDeviation": 0.5,
            "poseEstimateThetaDeviation": 0.26179939,
        },
        "foxglovePanelTitle": title,
    }


# -- mosaic layout construction -------------------------------------------


def split(direction: str, first: Any, second: Any, percentage: float = 50) -> dict[str, Any]:
    """One node of the mosaic tree. ``direction`` is 'row' or 'column'."""
    return {
        "direction": direction,
        "first": first,
        "second": second,
        "splitPercentage": percentage,
    }


def stack(direction: str, items: list[Any]) -> Any:
    """Arrange N panels evenly along one axis as a balanced mosaic tree.

    Foxglove's mosaic is strictly binary, so N panels become N-1 nested splits.
    Splitting down the middle keeps the tree shallow and the panes even.
    """
    if not items:
        raise ValueError("stack() needs at least one panel")
    if len(items) == 1:
        return items[0]
    middle = len(items) // 2
    left, right = items[:middle], items[middle:]
    return split(
        direction,
        stack(direction, left),
        stack(direction, right),
        percentage=100 * len(left) / len(items),
    )


def grid(items: list[Any], columns: int = 2) -> Any:
    """Arrange panels into a rough grid of ``columns`` columns."""
    if not items:
        raise ValueError("grid() needs at least one panel")
    rows = [items[i:i + columns] for i in range(0, len(items), columns)]
    return stack("column", [stack("row", row) for row in rows])


def tabs(entries: list[tuple[str, Any]]) -> dict[str, Any]:
    """A Tab panel config from ``(title, layout)`` pairs."""
    return {
        "activeTabIdx": 0,
        "tabs": [{"title": title, "layout": layout} for title, layout in entries],
    }


def layout_document(
    config_by_id: dict[str, Any], root: Any
) -> dict[str, Any]:
    """Assemble the final layout document."""
    return {
        "configById": config_by_id,
        "globalVariables": {},
        "userNodes": {},
        "playbackConfig": {"speed": 1},
        "layout": root,
    }


def write_layout(path: Path, document: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(document, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {path} ({len(document['configById'])} panels)")
