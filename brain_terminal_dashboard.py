#!/usr/bin/env python3

"""Live terminal dashboard for NovaVision/BFMC Brain.

Reads log lines from STDIN, parses your existing debug prints, and renders a
clean, live-updating table.

Run:
  PYTHONUNBUFFERED=1 python -u main.py 2>&1 | python brain_terminal_dashboard.py

Tip: If your output still feels "laggy", ensure you're using `python -u`.
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
last_lines = deque(maxlen=8)

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


# ----------------------------- regex patterns -----------------------------
RE_PERCEPTION = re.compile(
    r"\[Perception\]\s+tick=(\d+)\s+lane_steer=([^\s]+)\s+lane_conf=([^\s]+)"
    r"(?:\s+TL=([^\s]+)\s+TL_conf=([^\s]+))?"
    r"(?:\s+flags=(\{.*\}))?"
    r"(?:\s+top=(\[[^\]]*\]))?"
)
RE_CONTROLUNIT = re.compile(r"\[ControlUnit\]\s+tick=(\d+)\s+conf=([0-9.]+)\s+desired_speed=([-0-9]+)\s+desired_steer=([-0-9]+)")
RE_SHAPER = re.compile(
    r"\[Shaper\]\s+tick=(\d+)\s+desired=\(([-0-9]+),([-0-9]+)\)\s+out=\(([-0-9]+),([-0-9]+)\)\s+stop=(True|False)"
)
RE_SAFETY = re.compile(r"\[SafetyGuard\]\s+stop=(True|False)\s+max_speed=([^\s]+)\s+max_steer=([^\s]+)\s+reason=(.*)")
RE_WRITE = re.compile(r"\[Write\]\s+->\s+(\{.*\})")

RE_SM_START = re.compile(r"Starting in\s+(\w+)\s+mode")
RE_SM_CHANGE = re.compile(r"Mode changed from\s+(\w+)\s+to\s+(\w+)")

RE_SERIAL_CONN = re.compile(r"\[ Serial Handler \ ]\s*:\s*INFO\s*-\s*Connected to\s*(/dev/\S+)")
RE_SERIAL_NOCONN = re.compile(r"\[ Serial Handler \ ]\s*:\s*WARNING\s*-\s*No serial connection found")
RE_SERIAL_DISC = re.compile(r"\[ Serial Handler \ ]\s*:\s*WARNING\s*-\s*Serial device disconnected")

RE_MAIN_PROC = re.compile(r"^\s*-\s*(.+?)\s+pid=(\d+)\s+alive=(True|False)")


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


def parse_line(raw: str):
    line = strip_ansi(raw).rstrip("\n")
    if not line:
        return

    last_lines.append(line)

    # [MAIN] process list
    m = RE_MAIN_PROC.match(line)
    if m:
        name, pid, alive = m.group(1), int(m.group(2)), m.group(3) == "True"
        proc_alive[name] = (pid, alive)
        set_signal("Processes.alive", sum(1 for _p, a in proc_alive.values() if a), "main")
        set_signal("Processes.total", len(proc_alive), "main")
        return

    # Perception
    m = RE_PERCEPTION.search(line)
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

    # ControlUnit
    m = RE_CONTROLUNIT.search(line)
    if m:
        tick, conf, spd, st = m.groups()
        set_signal("ControlUnit.tick", tick, "ControlUnit")
        set_signal("ControlUnit.conf", conf, "ControlUnit")
        set_signal("Desired.speed", spd, "ControlUnit")
        set_signal("Desired.steer", st, "ControlUnit")
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

def build_table() -> Table:
    t = Table(title="NovaVision Brain • Live Debug", expand=True)
    t.add_column("Signal", style="bold")
    t.add_column("Value")
    t.add_column("Age (ms)", justify="right")
    t.add_column("Source", justify="right")

    rows = [
        "Mode",
        "KL",
        "Serial",
        "Processes.alive",
        "Processes.total",
        "Lane.steer",
        "Lane.conf",
        "TLight.state",
        "TLight.conf",
        "Perception.tick",
        "Desired.speed",
        "Desired.steer",
        "Shaper.out_speed",
        "Shaper.out_steer",
        "Safety.stop",
        "Safety.reason",
        "Actuator.speed_cmd",
        "Actuator.steer_cmd",
        "Actuator.brake",
    ]

    for key in rows:
        item = signals.get(key)
        if item:
            t.add_row(key, item["value"], age_ms(key), item.get("src", ""))
        else:
            t.add_row(key, "—", "—", "")

    return t


def build_log_panel() -> Panel:
    text = "\n".join(last_lines)
    return Panel(text or "(waiting for output...)", title="Last lines", border_style="dim")


def main():
    console.clear()

    sel = selectors.DefaultSelector()
    sel.register(sys.stdin, selectors.EVENT_READ)

    # Prime common signals so the table layout is stable
    set_signal("Mode", "—", "")
    set_signal("KL", "—", "")
    set_signal("Serial", "—", "")

    with Live(refresh_per_second=10, screen=True) as live:
        while True:
            # Non-blocking read with periodic refresh to keep ages updating
            events = sel.select(timeout=0.1)
            for key, _mask in events:
                line = key.fileobj.readline()
                if line == "":
                    return  # EOF
                parse_line(line)

            # Update display
            grid = Table.grid(expand=True)
            grid.add_row(build_table())
            grid.add_row(build_log_panel())
            live.update(grid)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass