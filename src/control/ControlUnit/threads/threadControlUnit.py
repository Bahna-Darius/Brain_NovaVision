# Copyright (c) 2026, NovaVision
# All rights reserved.

import threading
import time

from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.allMessages import (
    PerceptionContext,
    StateChange,
    DesiredSpeed,
    DesiredSteer,
)

from src.control.ControlUnit.control_unit_config import ControlUnitConfig as CFG


class threadControlUnit(threading.Thread):
    """
    ControlUnit:
    - Reads PerceptionContext
    - Tracks city/highway cruise mode
    - Applies curve-aware speed policy with hysteresis
    - Reacts to stop / crosswalk / highway_entry / highway_exit signs
    - Publishes DesiredSpeed / DesiredSteer in AUTO
    """

    def __init__(self, queueList, stop_event, logger=None, debugging=False):
        super().__init__()
        self.queueList = queueList
        self.stop_event = stop_event
        self.logger = logger
        self.debugging = debugging

        self.ctxSub = messageHandlerSubscriber(self.queueList, PerceptionContext, "lastOnly", True)
        self.stateSub = messageHandlerSubscriber(self.queueList, StateChange, "lastOnly", True)

        self.speedSender = messageHandlerSender(self.queueList, DesiredSpeed)
        self.steerSender = messageHandlerSender(self.queueList, DesiredSteer)

        self.current_mode = "MANUAL"
        self._tick = 0
        self._last_ctx = None
        self._last_ctx_time = 0.0

        self.cruise_mode = "CITY"
        self.in_curve = False
        self._curve_enter_count = 0
        self._curve_exit_count = 0

        self.stop_until = 0.0
        self.stop_cooldown_until = 0.0
        self.crosswalk_until = 0.0
        self.highway_entry_cooldown_until = 0.0
        self.highway_exit_cooldown_until = 0.0

        self._last_reason = "BOOT"

    def run(self):
        period = 1.0 / CFG.LOOP_HZ

        while not self.stop_event.is_set():
            start = time.time()
            self._tick += 1

            state_msg = self.stateSub.receive()
            if state_msg is not None:
                self.current_mode = str(state_msg)

            ctx = self.ctxSub.receive()
            if isinstance(ctx, dict):
                self._last_ctx = ctx
                self._last_ctx_time = time.time()

            if self.current_mode == "AUTO":
                self._auto_from_context(self._last_ctx, self._last_ctx_time)

            time.sleep(max(0.0, period - (time.time() - start)))

    # ---------------------------------------------------------------------
    # Main AUTO policy
    # ---------------------------------------------------------------------
    def _auto_from_context(self, ctx, ctx_time):
        now = time.time()

        if not isinstance(ctx, dict):
            self._publish(CFG.STOP_SPEED, 0, "NO_CONTEXT")
            return

        if (now - float(ctx_time)) > CFG.CTX_TIMEOUT_S:
            self._publish(CFG.STOP_SPEED, 0, "STALE_CONTEXT")
            return

        lane = ctx.get("lane") or {}
        raw_steer = int(lane.get("steer", 0))
        steer = int(raw_steer * 10)
        steer = max(-CFG.MAX_LANE_STEER, min(CFG.MAX_LANE_STEER, steer))
        conf = float(lane.get("confidence", 0.0))
        lane_status = str(lane.get("status", "UNKNOWN"))

        signs = self._get_fresh_signs(ctx, now)
        self._handle_sign_state(signs, now)

        if conf < CFG.MIN_LANE_CONFIDENCE or lane_status in ("NO_DETECTION", "CRASH"):
            self._publish(CFG.STOP_SPEED, 0, f"LANE_UNHEALTHY:{lane_status}")
            return

        # Hard stop window.
        if now < self.stop_until:
            self._publish(CFG.STOP_SPEED, 0, "STOP_SIGN_HOLD")
            return

        # Crosswalk action window.
        if now < self.crosswalk_until:
            cw_steer = 0 if CFG.DISABLE_STEER_ON_CROSSWALK else steer
            self._publish(CFG.CROSSWALK_SPEED, cw_steer, "CROSSWALK")
            return

        base_speed = CFG.HIGHWAY_BASE_SPEED if self.cruise_mode == "HIGHWAY" else CFG.CITY_BASE_SPEED
        curve_min_speed = CFG.CURVE_MIN_SPEED_HIGHWAY if self.cruise_mode == "HIGHWAY" else CFG.CURVE_MIN_SPEED_CITY

        self._update_curve_state(abs(steer))

        if self.in_curve:
            speed = self._compute_curve_speed(abs(steer), base_speed, curve_min_speed)
            self._publish(speed, steer, f"CURVE_{self.cruise_mode}")
            return

        self._publish(base_speed, steer, f"STRAIGHT_{self.cruise_mode}")

    # ---------------------------------------------------------------------
    # Sign helpers
    # ---------------------------------------------------------------------
    def _get_fresh_signs(self, ctx, now):
        ts = ctx.get("traffic_signs")
        if not isinstance(ts, dict):
            return None
        ts_time = float(ts.get("timestamp", 0.0) or 0.0)
        if ts_time <= 0.0:
            return None
        if (now - ts_time) > CFG.SIGN_MAX_AGE_S:
            return None
        return ts

    def _handle_sign_state(self, signs, now):
        if not isinstance(signs, dict):
            return

        stop_near = self._is_sign_near(
            signs,
            {"stop", "stop_sign"},
            CFG.STOP_NEAR_AREA_RATIO
        )
        if stop_near and now >= self.stop_cooldown_until and now >= self.stop_until:
            self.stop_until = now + CFG.STOP_HOLD_S
            self.stop_cooldown_until = self.stop_until + CFG.STOP_COOLDOWN_S

        crosswalk_near = self._is_sign_near(
            signs,
            {"crosswalk", "pedestrian_crossing", "cross_walk"},
            CFG.CROSSWALK_MIN_AREA_RATIO
        )
        if crosswalk_near:
            self.crosswalk_until = max(self.crosswalk_until, now + CFG.CROSSWALK_HOLD_S)

        hwy_entry_near = self._is_sign_near(
            signs,
            {"highway_entry", "highway_entrance", "motorway_entry", "motorway_entrance"},
            CFG.HIGHWAY_ENTRY_NEAR_AREA_RATIO
        )
        if hwy_entry_near and now >= self.highway_entry_cooldown_until:
            self.cruise_mode = "HIGHWAY"
            self.highway_entry_cooldown_until = now + CFG.HIGHWAY_SIGN_COOLDOWN_S

        hwy_exit_near = self._is_sign_near(
            signs,
            {"highway_exit", "motorway_exit", "highway_end", "motorway_end"},
            CFG.HIGHWAY_EXIT_NEAR_AREA_RATIO
        )
        if hwy_exit_near and now >= self.highway_exit_cooldown_until:
            self.cruise_mode = "CITY"
            self.highway_exit_cooldown_until = now + CFG.HIGHWAY_SIGN_COOLDOWN_S

    def _is_sign_present(self, signs, wanted_labels, min_area_ratio):
        if self._flag_match(signs, wanted_labels):
            return True
        det = self._find_best_detection(signs, wanted_labels)
        if det is None:
            return False
        return self._detection_area_ratio(det) >= float(min_area_ratio)

    def _is_sign_near(self, signs, wanted_labels, near_area_ratio):
        det = self._find_best_detection(signs, wanted_labels)
        if det is not None and self._detection_area_ratio(det) >= float(near_area_ratio):
            return True
        return False

    def _flag_match(self, signs, wanted_labels):
        flags = signs.get("flags") or {}
        if not isinstance(flags, dict):
            return False
        wanted = {self._normalize_label(x) for x in wanted_labels}
        for key, value in flags.items():
            if bool(value) and self._normalize_label(key) in wanted:
                return True
        if "stop_sign" in wanted and bool(signs.get("stop_sign", False)):
            return True
        return False

    def _find_best_detection(self, signs, wanted_labels):
        detections = signs.get("detections") or []
        if not isinstance(detections, list):
            return None

        wanted = {self._normalize_label(x) for x in wanted_labels}
        best = None
        best_area = -1.0
        for det in detections:
            if not isinstance(det, dict):
                continue
            label = self._extract_label(det)
            if self._normalize_label(label) not in wanted:
                continue
            area = self._detection_area_ratio(det)
            if area > best_area:
                best = det
                best_area = area
        return best

    def _extract_label(self, det):
        for key in ("label", "class_name", "class", "name", "type", "sign"):
            val = det.get(key)
            if val is not None:
                return str(val)
        return ""

    def _normalize_label(self, label):
        return str(label).strip().lower().replace("-", "_").replace(" ", "_")

    def _detection_area_ratio(self, det):
        direct = det.get("area_ratio")
        if direct is not None:
            try:
                return max(0.0, float(direct))
            except Exception:
                pass

        bbox = det.get("bbox")
        if isinstance(bbox, (list, tuple)) and len(bbox) == 4:
            try:
                x1, y1, x2, y2 = [float(v) for v in bbox]
                w = max(0.0, x2 - x1)
                h = max(0.0, y2 - y1)
                # If already normalized, area will naturally be <= 1.
                area = w * h
                if area <= 1.0:
                    return area
                # Otherwise assume 512x270 perception stream.
                return area / float(512.0 * 270.0)
            except Exception:
                return 0.0
        return 0.0

    # ---------------------------------------------------------------------
    # Curve helpers
    # ---------------------------------------------------------------------
    def _update_curve_state(self, abs_steer):
        if not self.in_curve:
            if abs_steer >= CFG.CURVE_ENTER_STEER:
                self._curve_enter_count += 1
            else:
                self._curve_enter_count = 0

            if self._curve_enter_count >= CFG.CURVE_ENTER_CONFIRM_FRAMES:
                self.in_curve = True
                self._curve_enter_count = 0
                self._curve_exit_count = 0
            return

        # Already in curve.
        if abs_steer <= CFG.CURVE_EXIT_STEER:
            self._curve_exit_count += 1
        else:
            self._curve_exit_count = 0

        if self._curve_exit_count >= CFG.CURVE_EXIT_CONFIRM_FRAMES:
            self.in_curve = False
            self._curve_exit_count = 0
            self._curve_enter_count = 0

    def _compute_curve_speed(self, abs_steer, base_speed, min_speed):
        if abs_steer <= CFG.CURVE_ENTER_STEER:
            return int(base_speed)

        span = max(1.0, float(CFG.CURVE_FULL_STEER - CFG.CURVE_ENTER_STEER))
        alpha = min(1.0, max(0.0, float(abs_steer - CFG.CURVE_ENTER_STEER) / span))
        speed = float(base_speed) - alpha * float(base_speed - min_speed)
        return int(round(max(min_speed, min(base_speed, speed))))

    # ---------------------------------------------------------------------
    def _publish(self, speed, steer, reason):
        speed = int(speed)
        steer = int(steer)
        self.speedSender.send(speed)
        self.steerSender.send(steer)

        if self.debugging and (reason != self._last_reason or self._tick % 20 == 0):
            print(
                f"[ControlUnit] tick={self._tick} mode={self.current_mode} cruise={self.cruise_mode} "
                f"curve={self.in_curve} speed={speed} steer={steer} reason={reason}"
            )
        self._last_reason = reason
