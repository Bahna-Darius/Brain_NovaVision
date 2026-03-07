import math
import time


class ParkingSequence:
    """
    Timed parallel parking sequence based on the paper's geometric idea:
      - first reverse arc
      - second reverse arc
      - short final straight correction

    Current version:
      - parking_sign is the trigger
      - a short forward offset is executed first
      - the two arcs are implemented as timed reverse steps
      - the last segment is a short forward straighten/correction

    Later upgrade path:
      - replace the fixed durations with durations derived from:
            R = L / tan(phi)
        and a chosen aperture angle alpha, as in the paper.
    """

    def __init__(
        self,
        trigger_label="parking_sign",
        trigger_confirm_frames=3,
        trigger_area_ratio=0.05,
        trigger_cooldown_s=8.0,
        wheelbase_m=0.26,
        vehicle_width_m=0.20,
        rear_overhang_m=0.085,
        steering_angle_deg=25.0,
        forward_offset_s=0.90,
        stop_before_reverse_s=0.20,
        reverse_speed=-110,
        forward_speed=100,
        arc1_steer=220,
        arc2_steer=-220,
        arc1_duration_s=1.30,
        arc2_duration_s=1.30,
        final_forward_duration_s=0.35,
    ):
        self.trigger_label = str(trigger_label)
        self.trigger_confirm_frames = int(trigger_confirm_frames)
        self.trigger_area_ratio = float(trigger_area_ratio)
        self.trigger_cooldown_s = float(trigger_cooldown_s)

        # Geometric parameters kept now so the file is already aligned with the paper.
        self.wheelbase_m = float(wheelbase_m)
        self.vehicle_width_m = float(vehicle_width_m)
        self.rear_overhang_m = float(rear_overhang_m)
        self.steering_angle_deg = float(steering_angle_deg)

        # Timed first version.
        self.forward_offset_s = float(forward_offset_s)
        self.stop_before_reverse_s = float(stop_before_reverse_s)
        self.reverse_speed = int(reverse_speed)
        self.forward_speed = int(forward_speed)
        self.arc1_steer = int(arc1_steer)
        self.arc2_steer = int(arc2_steer)
        self.arc1_duration_s = float(arc1_duration_s)
        self.arc2_duration_s = float(arc2_duration_s)
        self.final_forward_duration_s = float(final_forward_duration_s)

        self._confirm_count = 0
        self._active = False
        self._done = False
        self._armed = False
        self._last_trigger_time = 0.0

        self._step_index = 0
        self._step_started_at = 0.0
        self._steps = []

    # ------------------------------------------------------------------
    # Public properties
    # ------------------------------------------------------------------
    @property
    def active(self):
        return self._active

    @property
    def done(self):
        return self._done

    @property
    def armed(self):
        return self._armed

    @property
    def step_name(self):
        if not self._active:
            if self._done:
                return "DONE"
            if self._armed:
                return "ARMED"
            return "IDLE"

        if 0 <= self._step_index < len(self._steps):
            return self._steps[self._step_index]["name"]
        return "UNKNOWN"

    # ------------------------------------------------------------------
    # Trigger logic
    # ------------------------------------------------------------------
    def update_trigger(self, signs, now=None):
        if now is None:
            now = time.time()

        if self._active or self._done:
            return False

        if (now - self._last_trigger_time) < self.trigger_cooldown_s:
            self._confirm_count = 0
            return False

        if self._has_valid_parking_sign(signs):
            self._confirm_count += 1
        else:
            self._confirm_count = 0

        if self._confirm_count >= self.trigger_confirm_frames:
            self._armed = True
            self._confirm_count = 0
            self._last_trigger_time = now
            return True

        return False

    def start(self, now=None):
        if now is None:
            now = time.time()

        self._armed = False
        self._active = True
        self._done = False
        self._step_index = 0
        self._step_started_at = now

        # Paper-inspired shape:
        #   forward offset -> stop -> reverse arc 1 -> reverse arc 2 -> short forward correction -> stop
        self._steps = [
            {
                "name": "FORWARD_OFFSET",
                "duration": self.forward_offset_s,
                "speed": self.forward_speed,
                "steer": 0,
            },
            {
                "name": "STOP_BEFORE_REVERSE",
                "duration": self.stop_before_reverse_s,
                "speed": 0,
                "steer": 0,
            },
            {
                "name": "REVERSE_ARC_1",
                "duration": self.arc1_duration_s,
                "speed": self.reverse_speed,
                "steer": self.arc1_steer,
            },
            {
                "name": "REVERSE_ARC_2",
                "duration": self.arc2_duration_s,
                "speed": self.reverse_speed,
                "steer": self.arc2_steer,
            },
            {
                "name": "FINAL_FORWARD_CORRECTION",
                "duration": self.final_forward_duration_s,
                "speed": self.forward_speed,
                "steer": 0,
            },
            {
                "name": "FINAL_STOP",
                "duration": 9999.0,
                "speed": 0,
                "steer": 0,
            },
        ]

    def update(self, now=None):
        if now is None:
            now = time.time()

        if not self._active:
            return {
                "active": False,
                "done": self._done,
                "step": self.step_name,
                "speed": 0,
                "steer": 0,
            }

        if self._step_index >= len(self._steps):
            self._active = False
            self._done = True
            return {
                "active": False,
                "done": True,
                "step": "DONE",
                "speed": 0,
                "steer": 0,
            }

        step = self._steps[self._step_index]
        elapsed = now - self._step_started_at

        if elapsed >= step["duration"] and step["name"] != "FINAL_STOP":
            self._step_index += 1
            self._step_started_at = now

            if self._step_index >= len(self._steps):
                self._active = False
                self._done = True
                return {
                    "active": False,
                    "done": True,
                    "step": "DONE",
                    "speed": 0,
                    "steer": 0,
                }

            step = self._steps[self._step_index]

        if step["name"] == "FINAL_STOP":
            self._done = True

        return {
            "active": True,
            "done": self._done,
            "step": step["name"],
            "speed": int(step["speed"]),
            "steer": int(step["steer"]),
        }

    # ------------------------------------------------------------------
    # Geometry helper for future upgrade
    # ------------------------------------------------------------------
    def get_geometry_debug(self):
        """
        Future-friendly helper.
        The paper uses:
            R = L / tan(phi)
        for the two parking arcs.
        """
        phi_rad = math.radians(self.steering_angle_deg)
        if abs(math.tan(phi_rad)) < 1e-6:
            radius_m = float("inf")
        else:
            radius_m = self.wheelbase_m / math.tan(phi_rad)

        return {
            "wheelbase_m": self.wheelbase_m,
            "vehicle_width_m": self.vehicle_width_m,
            "rear_overhang_m": self.rear_overhang_m,
            "steering_angle_deg": self.steering_angle_deg,
            "turn_radius_m": radius_m,
        }

    # ------------------------------------------------------------------
    # Detection helpers
    # ------------------------------------------------------------------
    def _has_valid_parking_sign(self, signs):
        if not isinstance(signs, dict):
            return False

        detections = signs.get("detections") or []
        if not isinstance(detections, list):
            return False

        for det in detections:
            if not isinstance(det, dict):
                continue

            label = self._normalize_label(self._extract_label(det))
            if label != self._normalize_label(self.trigger_label):
                continue

            if self._detection_area_ratio(det) >= self.trigger_area_ratio:
                return True

        return False

    def _extract_label(self, det):
        for key in ("label", "class_name", "class", "name", "type", "sign"):
            value = det.get(key)
            if value is not None:
                return str(value)
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
                area = w * h
                if area <= 1.0:
                    return area
                return area / float(512.0 * 270.0)
            except Exception:
                return 0.0

        return 0.0