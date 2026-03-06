#!/usr/bin/env python3

"""Live terminal dashboard for NovaVision/BFMC Brain.

Reads log lines from STDIN, parses your existing debug prints, and renders a
clean, live-updating dashboard.

Run:
  PYTHONUNBUFFERED=1 python -u main.py 2>&1 | python brain_terminal_dashboard.py

Tip:
  If your output still feels "laggy", ensure you're using `python -u`.
"""

import sys
import time
import re
import ast
import selectors
from collections import deque

from rich.console import Console
from rich.live import Live
from rich.table import Table
from rich.panel import Panel

console = Console()

ANSI_RE = re.compile(r"\x1B\[[0-?]*[ -/]*[@-~]")


def strip_ansi(s: str) -> str:
    return ANSI_RE.sub("", s)


# ----------------------------- state store -----------------------------

signals = {}  # name -> {value: str, ts: float, src: str}

# Keep last few raw lines (useful when you add new debuggers)
last_lines = deque(maxlen=12)

# Keep recent raw numeric speed/steer pairs (those INFO:root:123 spam lines)
numeric_lines = deque(maxlen=8)
_pending_numeric = None

# Process list from [MAIN]
proc_alive = {}


def set_signal(name: str, value, src: str = ""):
    signals[name] = {
        "value": str(value),
        "ts": time.time(),
        "src": src,
    }


def age_ms(name: str) -> str:
    it = signals.get(name)
    if not it:
        return "—"
    return f"{int((time.time() - it['ts']) * 1000)}"


def fmt_float(v, digits=2):
    try:
        return f"{float(v):.{digits}f}"
    except Exception:
        return str(v)


# ----------------------------- regex patterns -----------------------------
# Newer Perception format
RE_PERCEPTION_NEW = re.compile(
    r"\[Perception\]\s+fps=([0-9.]+)\s+avg_lane_ms=([0-9.]+)\s+avg_decode_ms=([0-9.]+)\s+lane_steer=([-0-9.]+)\s+lane_conf=([0-9.]+)"
)

# Older Perception format kept for compatibility
RE_PERCEPTION_OLD = re.compile(
    r"\[Perception\]\s+tick=(\d+)\s+lane_steer=([^\s]+)\s+lane_conf=([^\s]+)"
    r"(?:\s+TL=([^\s]+)\s+TL_conf=([^\s]+))?"
    r"(?:\s+flags=(\{.*\}))?"
    r"(?:\s+top=(\[[^\]]*\]))?"
)

# Newer ControlUnit format
RE_CONTROLUNIT_NEW = re.compile(
    r"\[ControlUnit\]\s+mode=([^\s]+)\s+cruise=([^\s]+)\s+in_curve=(True|False)\s+speed=([-0-9]+)\s+steer=([-0-9]+)\s+reason=(.+)"
)

# Older ControlUnit format kept for compatibility
RE_CONTROLUNIT_OLD = re.compile(
    r"\[ControlUnit\]\s+tick=(\d+)\s+conf=([0-9.]+)\s+desired_speed=([-0-9]+)\s+desired_steer=([-0-9]+)"
)

RE_CURVE_DEBUG = re.compile(
    r"\[CurveDebug\]\s+steer_raw=([-0-9.]+)\s+steer_cmd=([-0-9.]+)\s+near=([^\s]+)\s+far=([^\s]+)\s+heading_delta=([^\s]+)\s+curvature=([^\s]+)\s+turn=([^\s]+)\s+curve_score=([^\s]+)\s+in_curve=(True|False)"
)

RE_CURVE_ACTION = re.compile(
    r"\[CurveAction\]\s+speed_cmd=([-0-9.]+)\s+base_speed=([-0-9.]+)\s+curve_min_speed=([-0-9.]+)\s+reason=(.+)"
)

RE_CAMERA = re.compile(r"\[Camera\]\s+lane_publish_fps=([0-9.]+)")
RE_TS_CLIENT = re.compile(
    r"\[TrafficSignClient\]\s+tick=(\d+)\s+count=(\d+)\s+infer_ms=([0-9.]+)\s+rtt_ms=([0-9.]+)"
)

RE_SHAPER = re.compile(
    r"\[Shaper\]\s+tick=(\d+)\s+desired=\(([-0-9]+),([-0-9]+)\)\s+out=\(([-0-9]+),([-0-9]+)\)\s+stop=(True|False)"
)
RE_SAFETY = re.compile(r"\[SafetyGuard\]\s+stop=(True|False)\s+max_speed=([^\s]+)\s+max_steer=([^\s]+)\s+reason=(.*)")
RE_WRITE = re.compile(r"\[Write\]\s+->\s+(\{.*\})")

RE_SM_START = re.compile(r"Starting in\s+(\w+)\s+mode")
RE_SM_CHANGE = re.compile(r"Mode changed from\s+(\w+)\s+to\s+(\w+)")

RE_SERIAL_CONN = re.compile(r"\[ Serial Handler \]\s*:\s*INFO\s*-\s*Connected to\s*(/dev/\S+)")
RE_SERIAL_NOCONN = re.compile(r"\[ Serial Handler \]\s*:\s*WARNING\s*-\s*No serial connection found")
RE_SERIAL_DISC = re.compile(r"\[ Serial Handler \]\s*:\s*WARNING\s*-\s*Serial device disconnected")

RE_MAIN_PROC = re.compile(r"^\s*-\s*(.+?)\s+pid=(\d+)\s+alive=(True|False)")

# Raw numeric spam: INFO:root:123 or INFO:root:-45
RE_INFO_NUM = re.compile(r"^INFO:root:([-]?\d+(?:\.\d+)?)$")


# ----------------------------- parsing -----------------------------

def parse_write_dict(d: dict):
    """Map threadWrite send_to_serial() commands into signals."""
    action = d.get("action")
    if not action:
        return

    if action == "kl":
        set_signal("KL", d.get("mode"), "threadWrite")

    elif action == "speed":
        set_signal("Actuator.speed_cmd", d.get("speed"), "threadWrite")

    elif action == "steer":
        set_signal("Actuator.steer_cmd", d.get("steerAngle"), "threadWrite")

    elif action == "brake":
        set_signal("Actuator.brake", d.get("steerAngle"), "threadWrite")

    elif action in ("instant", "battery", "imu", "resourceMonitor"):
        set_signal(f"Toggle.{action}", d.get("activate"), "threadWrite")

    elif action == "batteryCapacity":
        set_signal("Battery.capacity", d.get("capacity"), "threadWrite")

    elif action in ("vcd", "vcdCalib"):
        set_signal(f"{action}.time", d.get("time"), "threadWrite")
        set_signal(f"{action}.speed", d.get("speed"), "threadWrite")
        set_signal(f"{action}.steer", d.get("steer"), "threadWrite")


def parse_numeric_pair(line: str) -> bool:
    """Pair consecutive INFO:root:number lines as speed/steer-ish raw stream."""
    global _pending_numeric

    m = RE_INFO_NUM.match(line)
    if not m:
        return False

    value = m.group(1)
    if _pending_numeric is None:
        _pending_numeric = value
    else:
        numeric_lines.append((str(_pending_numeric), str(value)))
        set_signal("RawPair.first", _pending_numeric, "raw")
        set_signal("RawPair.second", value, "raw")
        _pending_numeric = None
    return True


def parse_line(raw: str):
    line = strip_ansi(raw).rstrip("\n")
    if not line:
        return

    last_lines.append(line)

    # Raw numeric stream
    if parse_numeric_pair(line):
        return

    # [MAIN] process list
    m = RE_MAIN_PROC.match(line)
    if m:
        name, pid, alive = m.group(1), int(m.group(2)), m.group(3) == "True"
        proc_alive[name] = (pid, alive)
        set_signal("Processes.alive", sum(1 for _p, a in proc_alive.values() if a), "main")
        set_signal("Processes.total", len(proc_alive), "main")
        return

    # New Perception
    m = RE_PERCEPTION_NEW.search(line)
    if m:
        fps, lane_ms, decode_ms, steer, conf = m.groups()
        set_signal("Perception.fps", fmt_float(fps, 1), "Perception")
        set_signal("Perception.avg_lane_ms", fmt_float(lane_ms, 1), "Perception")
        set_signal("Perception.avg_decode_ms", fmt_float(decode_ms, 1), "Perception")
        set_signal("Lane.steer", steer, "Perception")
        set_signal("Lane.conf", conf, "Perception")
        return

    # Old Perception
    m = RE_PERCEPTION_OLD.search(line)
    if m:
        tick, steer, conf, tl_state, tl_conf = m.groups()
        set_signal("Perception.tick", tick, "Perception")
        set_signal("Lane.steer", steer, "Perception")
        set_signal("Lane.conf", conf, "Perception")
        if tl_state is not None:
            set_signal("TLight.state", tl_state, "Perception")
        if tl_conf is not None:
            set_signal("TLight.conf", tl_conf, "Perception")
        return

    # New ControlUnit
    m = RE_CONTROLUNIT_NEW.search(line)
    if m:
        mode, cruise, in_curve, spd, st, reason = m.groups()
        set_signal("Mode", mode, "ControlUnit")
        set_signal("Cruise", cruise, "ControlUnit")
        set_signal("CU.in_curve", in_curve, "ControlUnit")
        set_signal("Desired.speed", spd, "ControlUnit")
        set_signal("Desired.steer", st, "ControlUnit")
        set_signal("CU.reason", reason, "ControlUnit")
        return

    # Old ControlUnit
    m = RE_CONTROLUNIT_OLD.search(line)
    if m:
        tick, conf, spd, st = m.groups()
        set_signal("ControlUnit.tick", tick, "ControlUnit")
        set_signal("ControlUnit.conf", conf, "ControlUnit")
        set_signal("Desired.speed", spd, "ControlUnit")
        set_signal("Desired.steer", st, "ControlUnit")
        return

    # CurveDebug
    m = RE_CURVE_DEBUG.search(line)
    if m:
        steer_raw, steer_cmd, near, far, heading_delta, curvature, turn_dir, curve_score, in_curve = m.groups()
        set_signal("Curve.steer_raw", steer_raw, "CurveDebug")
        set_signal("Curve.steer_cmd", steer_cmd, "CurveDebug")
        set_signal("Curve.near_px", fmt_float(near, 1), "CurveDebug")
        set_signal("Curve.far_px", fmt_float(far, 1), "CurveDebug")
        set_signal("Curve.heading_deg", fmt_float(heading_delta, 1), "CurveDebug")
        set_signal("Curve.curvature", fmt_float(curvature, 1), "CurveDebug")
        set_signal("Curve.turn", turn_dir, "CurveDebug")
        set_signal("Curve.score", fmt_float(curve_score, 3), "CurveDebug")
        set_signal("Curve.in_curve", in_curve, "CurveDebug")
        return

    # CurveAction
    m = RE_CURVE_ACTION.search(line)
    if m:
        speed_cmd, base_speed, curve_min_speed, reason = m.groups()
        set_signal("Curve.speed_cmd", speed_cmd, "CurveAction")
        set_signal("Curve.base_speed", base_speed, "CurveAction")
        set_signal("Curve.min_speed", curve_min_speed, "CurveAction")
        set_signal("Curve.reason", reason, "CurveAction")
        return

    # Camera
    m = RE_CAMERA.search(line)
    if m:
        set_signal("Camera.lane_fps", fmt_float(m.group(1), 1), "Camera")
        return

    # TrafficSignClient
    m = RE_TS_CLIENT.search(line)
    if m:
        tick, count, infer_ms, rtt_ms = m.groups()
        set_signal("TS.tick", tick, "TrafficSignClient")
        set_signal("TS.count", count, "TrafficSignClient")
        set_signal("TS.infer_ms", fmt_float(infer_ms, 1), "TrafficSignClient")
        set_signal("TS.rtt_ms", fmt_float(rtt_ms, 1), "TrafficSignClient")
        return

    # CommandShaper
    m = RE_SHAPER.search(line)
    if m:
        tick, d_spd, d_st, o_spd, o_st, stop = m.groups()
        set_signal("Shaper.tick", tick, "CommandShaper")
        set_signal("Shaper.desired_speed", d_spd, "CommandShaper")
        set_signal("Shaper.desired_steer", d_st, "CommandShaper")
        set_signal("Shaper.out_speed", o_spd, "CommandShaper")
        set_signal("Shaper.out_steer", o_st, "CommandShaper")
        set_signal("Safety.stop", stop, "CommandShaper")
        return

    # SafetyGuard
    m = RE_SAFETY.search(line)
    if m:
        stop, max_speed, max_steer, reason = m.groups()
        set_signal("Safety.stop", stop, "SafetyGuard")
        set_signal("Safety.max_speed", max_speed, "SafetyGuard")
        set_signal("Safety.max_steer", max_steer, "SafetyGuard")
        set_signal("Safety.reason", reason, "SafetyGuard")
        return

    # threadWrite command dump
    m = RE_WRITE.search(line)
    if m:
        try:
            d = ast.literal_eval(m.group(1))
            if isinstance(d, dict):
                parse_write_dict(d)
        except Exception:
            pass
        return

    # StateMachine mode changes
    m = RE_SM_START.search(line)
    if m:
        set_signal("Mode", m.group(1), "StateMachine")
        return

    m = RE_SM_CHANGE.search(line)
    if m:
        set_signal("Mode", m.group(2), "StateMachine")
        return

    # SerialHandler status
    m = RE_SERIAL_CONN.search(line)
    if m:
        set_signal("Serial", f"Connected ({m.group(1)})", "SerialHandler")
        return

    if RE_SERIAL_NOCONN.search(line):
        set_signal("Serial", "No connection", "SerialHandler")
        return

    if RE_SERIAL_DISC.search(line):
        set_signal("Serial", "Disconnected", "SerialHandler")
        return


# ----------------------------- rendering -----------------------------

def add_signal_row(table: Table, key: str, label: str = None):
    label = label or key
    item = signals.get(key)
    if item:
        table.add_row(label, item["value"], age_ms(key), item.get("src", ""))
    else:
        table.add_row(label, "—", "—", "")


def build_status_table() -> Table:
    t = Table(title="System", expand=True)
    t.add_column("Signal", style="bold cyan")
    t.add_column("Value")
    t.add_column("Age (ms)", justify="right")
    t.add_column("Source", justify="right")

    for key, label in [
        ("Mode", "Mode"),
        ("Cruise", "Cruise"),
        ("Serial", "Serial"),
        ("KL", "KL"),
        ("Processes.alive", "Processes alive"),
        ("Processes.total", "Processes total"),
        ("Camera.lane_fps", "Camera lane fps"),
        ("TS.tick", "TS tick"),
        ("TS.count", "TS count"),
        ("TS.infer_ms", "TS infer ms"),
        ("TS.rtt_ms", "TS RTT ms"),
    ]:
        add_signal_row(t, key, label)

    return t


def build_perception_table() -> Table:
    t = Table(title="Perception", expand=True)
    t.add_column("Signal", style="bold green")
    t.add_column("Value")
    t.add_column("Age (ms)", justify="right")
    t.add_column("Source", justify="right")

    for key, label in [
        ("Perception.fps", "FPS"),
        ("Perception.avg_lane_ms", "Lane ms"),
        ("Perception.avg_decode_ms", "Decode ms"),
        ("Lane.steer", "Lane steer"),
        ("Lane.conf", "Lane conf"),
        ("TLight.state", "Traffic light"),
        ("TLight.conf", "TL conf"),
    ]:
        add_signal_row(t, key, label)

    return t


def build_control_table() -> Table:
    t = Table(title="Control Unit", expand=True)
    t.add_column("Signal", style="bold yellow")
    t.add_column("Value")
    t.add_column("Age (ms)", justify="right")
    t.add_column("Source", justify="right")

    for key, label in [
        ("CU.in_curve", "In curve"),
        ("Desired.speed", "Desired speed"),
        ("Desired.steer", "Desired steer"),
        ("CU.reason", "Reason"),
        ("Curve.speed_cmd", "Curve speed cmd"),
        ("Curve.base_speed", "Base speed"),
        ("Curve.min_speed", "Curve min speed"),
        ("Curve.reason", "Curve reason"),
    ]:
        add_signal_row(t, key, label)

    return t


def build_curve_table() -> Table:
    t = Table(title="Curve Debug", expand=True)
    t.add_column("Signal", style="bold magenta")
    t.add_column("Value")
    t.add_column("Age (ms)", justify="right")
    t.add_column("Source", justify="right")

    for key, label in [
        ("Curve.turn", "Turn"),
        ("Curve.score", "Curve score"),
        ("Curve.in_curve", "Curve state"),
        ("Curve.steer_raw", "Steer raw"),
        ("Curve.steer_cmd", "Steer cmd"),
        ("Curve.near_px", "Near px"),
        ("Curve.far_px", "Far px"),
        ("Curve.heading_deg", "Heading Δ deg"),
        ("Curve.curvature", "Curvature"),
    ]:
        add_signal_row(t, key, label)

    return t


def build_actuator_table() -> Table:
    t = Table(title="Actuation / Raw Stream", expand=True)
    t.add_column("Signal", style="bold blue")
    t.add_column("Value")
    t.add_column("Age (ms)", justify="right")
    t.add_column("Source", justify="right")

    for key, label in [
        ("Shaper.out_speed", "Shaper out speed"),
        ("Shaper.out_steer", "Shaper out steer"),
        ("Actuator.speed_cmd", "Actuator speed"),
        ("Actuator.steer_cmd", "Actuator steer"),
        ("Actuator.brake", "Brake"),
        ("RawPair.first", "Raw pair first"),
        ("RawPair.second", "Raw pair second"),
        ("Safety.stop", "Safety stop"),
        ("Safety.reason", "Safety reason"),
    ]:
        add_signal_row(t, key, label)

    return t


def build_numeric_panel() -> Panel:
    if not numeric_lines:
        return Panel("(waiting for numeric stream...)", title="Recent raw numeric pairs", border_style="dim")

    lines = []
    for idx, pair in enumerate(reversed(numeric_lines), start=1):
        lines.append(f"{idx:>2}. {pair[0]:>6} | {pair[1]:>6}")
    return Panel("\n".join(lines), title="Recent raw numeric pairs", border_style="dim")


def build_log_panel() -> Panel:
    text = "\n".join(last_lines)
    return Panel(text or "(waiting for output...)", title="Last lines", border_style="dim")


def build_layout():
    top = Table.grid(expand=True)
    top.add_column(ratio=1)
    top.add_column(ratio=1)
    top.add_row(build_status_table(), build_perception_table())
    top.add_row(build_control_table(), build_curve_table())
    top.add_row(build_actuator_table(), build_numeric_panel())

    root = Table.grid(expand=True)
    root.add_row(top)
    root.add_row(build_log_panel())
    return root


def main():
    console.clear()

    sel = selectors.DefaultSelector()
    sel.register(sys.stdin, selectors.EVENT_READ)

    # Prime common signals so layout is stable
    set_signal("Mode", "—", "")
    set_signal("Cruise", "—", "")
    set_signal("Serial", "—", "")

    with Live(refresh_per_second=10, screen=True) as live:
        while True:
            events = sel.select(timeout=0.1)
            for key, _mask in events:
                line = key.fileobj.readline()
                if line == "":
                    return
                parse_line(line)

            live.update(build_layout())


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass