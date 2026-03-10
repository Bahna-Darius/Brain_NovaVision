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
    SpeedMotor,
    SteerMotor,
)

from src.control.ControlUnit.control_unit_config import ControlUnitConfig as CFG
from src.algorithms.ParkingSequence.parking_sequence import ParkingSequence


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
        self.directSpeedSender = messageHandlerSender(self.queueList, SpeedMotor)
        self.directSteerSender = messageHandlerSender(self.queueList, SteerMotor)

        self._parking_direct_active = False

        self.current_mode = "MANUAL"
        self._tick = 0
        self._last_ctx = None
        self._last_ctx_time = 0.0

        self.cruise_mode = "CITY"
        self.in_curve = False
        self._curve_enter_count = 0
        self._curve_exit_count = 0
        self._curve_score_ema = 0.0

        self.stop_until = 0.0
        self.stop_cooldown_until = 0.0
        self.crosswalk_until = 0.0
        self.highway_entry_cooldown_until = 0.0
        self.highway_exit_cooldown_until = 0.0

        self._last_reason = "BOOT"

        # For limiting steering aggressiveness on HIGHWAY.
        self._last_sent_steer = 0

        # Keep last good AUTO command for short lane-confidence dropouts.
        self._last_good_speed = 0
        self._last_good_steer = 0
        self._lane_bad_since = None
        self._lane_hold_timeout_s = 0.35
        self.parking = ParkingSequence(
            trigger_label=CFG.PARKING_SIGN_TRIGGER_LABEL,
            trigger_confirm_frames=CFG.PARKING_TRIGGER_CONFIRM_FRAMES,
            trigger_area_ratio=CFG.PARKING_TRIGGER_AREA_RATIO,
            trigger_cooldown_s=CFG.PARKING_TRIGGER_COOLDOWN_S,
            wheelbase_m=CFG.PARKING_WHEELBASE_M,
            vehicle_width_m=CFG.PARKING_VEHICLE_WIDTH_M,
            rear_overhang_m=CFG.PARKING_REAR_OVERHANG_M,
            steering_angle_deg=CFG.PARKING_STEERING_ANGLE_DEG,
            forward_offset_s=CFG.PARKING_FORWARD_OFFSET_S,
            stop_before_reverse_s=CFG.PARKING_STOP_BEFORE_REVERSE_S,
            reverse_speed=CFG.PARKING_REVERSE_SPEED,
            forward_speed=CFG.PARKING_FORWARD_SPEED,
            arc1_steer=CFG.PARKING_ARC1_STEER,
            arc2_steer=CFG.PARKING_ARC2_STEER,
            arc1_duration_s=CFG.PARKING_ARC1_DURATION_S,
            arc2_duration_s=CFG.PARKING_ARC2_DURATION_S,
            final_forward_duration_s=CFG.PARKING_FINAL_FORWARD_DURATION_S,
        )

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
        preview = lane.get("preview")

        signs = self._get_fresh_signs(ctx, now)
        self._handle_sign_state(signs, now)

        # Highway steering safety: limit how fast/strong we can turn at high speed.
        # Applied after cruise_mode updates (highway_entry/exit) and before curve logic.
        steer = self._apply_highway_steer_limits(steer)

        # Parking sign trigger and parking sequence execution.
        self.parking.update_trigger(signs, now)

        if self.parking.armed and not self.parking.active and not self.parking.done:
            self.parking.start(now)

        if self.parking.active:
            parking_cmd = self.parking.update(now)
            self._publish(
                parking_cmd["speed"],
                parking_cmd["steer"],
                f"PARKING:{parking_cmd['step']}"
            )
            return

        if self.parking.done:
            self._publish(CFG.STOP_SPEED, 0, "PARKING:DONE")
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

        # Hold last good command briefly if lane confidence drops or lane crashes.
        if conf < CFG.MIN_LANE_CONFIDENCE or lane_status in ("NO_DETECTION", "CRASH"):
            if self._lane_bad_since is None:
                self._lane_bad_since = now

            bad_duration = now - self._lane_bad_since

            if bad_duration < self._lane_hold_timeout_s and self._last_good_speed > 0:
                self._publish(self._last_good_speed, self._last_good_steer, f"LANE_HOLD:{lane_status}")
            else:
                self._publish(CFG.STOP_SPEED, 0, f"LANE_UNHEALTHY:{lane_status}")
            return

        # Lane looks healthy again.
        self._lane_bad_since = None

        base_speed = CFG.HIGHWAY_BASE_SPEED if self.cruise_mode == "HIGHWAY" else CFG.CITY_BASE_SPEED
        curve_min_speed = CFG.CURVE_MIN_SPEED_HIGHWAY if self.cruise_mode == "HIGHWAY" else CFG.CURVE_MIN_SPEED_CITY

        curve_score = self._compute_curve_score(preview, steer)
        self._update_curve_state(curve_score)

        if self.debugging and self.logger is not None and (self._tick % 15 == 0):
            near_err = None
            far_err = None
            heading_delta = None
            curvature = None
            turn_dir = None

            if isinstance(preview, dict):
                near_err = preview.get("near_error_px")
                far_err = preview.get("far_error_px")
                heading_delta = preview.get("heading_delta_deg")
                curvature = preview.get("curvature_score")
                turn_dir = preview.get("turn_direction")

            self.logger.info(
                f"[CurveDebug] "
                f"steer_raw={raw_steer} steer_cmd={steer} "
                f"near={near_err} far={far_err} "
                f"heading_delta={heading_delta} curvature={curvature} "
                f"turn={turn_dir} curve_score={curve_score:.3f} in_curve={self.in_curve}"
            )

        if self.in_curve:
            speed = self._compute_curve_speed(curve_score, base_speed, curve_min_speed)

            if self.debugging and self.logger is not None and (self._tick % 15 == 0):
                self.logger.info(
                    f"[CurveAction] speed_cmd={speed} base_speed={base_speed} "
                    f"curve_min_speed={curve_min_speed} reason=CURVE_{self.cruise_mode}"
                )

            self._publish(speed, steer, f"CURVE_{self.cruise_mode}")
            return

        if self.debugging and self.logger is not None and (self._tick % 15 == 0):
            self.logger.info(
                f"[CurveAction] speed_cmd={base_speed} base_speed={base_speed} "
                f"curve_min_speed={curve_min_speed} reason=STRAIGHT_{self.cruise_mode}"
            )

        self._publish(base_speed, steer, f"STRAIGHT_{self.cruise_mode}")

    # ---------------------------------------------------------------------
    # Highway steering safety
    # ---------------------------------------------------------------------
    def _apply_highway_steer_limits(self, steer: int) -> int:
        """Limit steering aggressiveness while in HIGHWAY cruise mode."""
        try:
            steer = int(steer)
        except Exception:
            return 0

        # Only apply in highway cruise; parking sequence uses its own direct commands.
        if self.cruise_mode != "HIGHWAY":
            self._last_sent_steer = steer
            return steer

        # 1) Clamp magnitude.
        max_mag = int(getattr(CFG, "HIGHWAY_MAX_STEER", CFG.MAX_LANE_STEER))
        steer = max(-max_mag, min(max_mag, steer))

        # 2) Clamp rate-of-change (slew) per ControlUnit tick.
        max_step = int(getattr(CFG, "HIGHWAY_MAX_STEER_STEP", 0) or 0)
        if max_step > 0:
            prev = int(self._last_sent_steer)
            delta = steer - prev
            if delta > max_step:
                steer = prev + max_step
            elif delta < -max_step:
                steer = prev - max_step

        self._last_sent_steer = steer
        return steer

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
    def _compute_curve_score(self, preview, steer):
        steer_term = min(1.0, abs(float(steer)) / float(CFG.CURVE_FULL_STEER))

        if not isinstance(preview, dict):
            score = steer_term
            self._curve_score_ema = (
                (CFG.CURVE_SCORE_ALPHA * score) +
                ((1.0 - CFG.CURVE_SCORE_ALPHA) * self._curve_score_ema)
            )
            return self._curve_score_ema

        try:
            near_err = abs(float(preview.get("near_error_px", 0.0)))
            far_err = abs(float(preview.get("far_error_px", 0.0)))
            heading_delta = abs(float(preview.get("heading_delta_deg", 0.0)))
            curvature_score = abs(float(preview.get("curvature_score", 0.0)))

            # Curve anticipation: far path bends before near path does.
            anticipation = max(0.0, far_err - near_err)

            far_term = min(1.0, far_err / float(CFG.PREVIEW_FAR_ERR_FULL_PX))
            anticipation_term = min(1.0, anticipation / float(CFG.PREVIEW_ANTICIPATION_FULL_PX))
            heading_term = min(1.0, heading_delta / float(CFG.PREVIEW_HEADING_DELTA_FULL_DEG))
            curvature_term = min(1.0, curvature_score / float(CFG.PREVIEW_CURVATURE_FULL_SCALE))

            score = (
                0.30 * far_term +
                0.30 * anticipation_term +
                0.20 * heading_term +
                0.10 * curvature_term +
                0.10 * steer_term
            )

            self._curve_score_ema = (
                (CFG.CURVE_SCORE_ALPHA * score) +
                ((1.0 - CFG.CURVE_SCORE_ALPHA) * self._curve_score_ema)
            )
            return self._curve_score_ema

        except Exception:
            score = steer_term
            self._curve_score_ema = (
                (CFG.CURVE_SCORE_ALPHA * score) +
                ((1.0 - CFG.CURVE_SCORE_ALPHA) * self._curve_score_ema)
            )
            return self._curve_score_ema

    def _update_curve_state(self, curve_score):
        if not self.in_curve:
            if curve_score >= CFG.CURVE_ENTER_SCORE:
                self._curve_enter_count += 1
            else:
                self._curve_enter_count = 0

            if self._curve_enter_count >= CFG.CURVE_ENTER_CONFIRM_FRAMES:
                self.in_curve = True
                self._curve_enter_count = 0
                self._curve_exit_count = 0
            return

        if curve_score <= CFG.CURVE_EXIT_SCORE:
            self._curve_exit_count += 1
        else:
            self._curve_exit_count = 0

        if self._curve_exit_count >= CFG.CURVE_EXIT_CONFIRM_FRAMES:
            self.in_curve = False
            self._curve_exit_count = 0
            self._curve_enter_count = 0

    def _compute_curve_speed(self, curve_score, base_speed, min_speed):
        curve_score = max(0.0, min(1.0, float(curve_score)))
        speed = float(base_speed) - (curve_score * float(base_speed - min_speed))
        return int(round(speed))

    # ---------------------------------------------------------------------
    # Output
    # ---------------------------------------------------------------------
    def _publish(self, speed, steer, reason):
        speed = int(speed)
        steer = int(steer)

        self.speedSender.send(speed)
        self.steerSender.send(steer)

        # Save last good command so brief lane dropouts do not force an instant stop.
        if speed > 0 and not str(reason).startswith("LANE_"):
            self._last_good_speed = speed
            self._last_good_steer = steer

        self._last_reason = str(reason)

        if self.debugging and self.logger is not None and (self._tick % 10 == 0):
            self.logger.info(
                f"[ControlUnit] mode={self.current_mode} cruise={self.cruise_mode} "
                f"in_curve={self.in_curve} speed={speed} steer={steer} reason={reason}"
            )