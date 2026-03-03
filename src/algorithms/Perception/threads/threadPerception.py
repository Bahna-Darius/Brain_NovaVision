# Copyright (c) 2026, NovaVision
# All rights reserved.

"""
Hybrid Perception thread (minimal changes + safe fixes):
- Input: serialCameraRaw (base64 JPEG string)
- Decode once (base64 -> BGR)
- Run LaneAssist every tick
- Run YOLOv8 Traffic Signs throttled (every N ticks)
- Output: PerceptionContext dict (fused)
- Send visualization frame to dashboard via serialCamera (base64 JPEG)
- Record overlay support (Record message)
- Traffic Light detector OPTIONAL (no crash if missing)
"""

import base64
import os
import time
import traceback

import cv2
import numpy as np

from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.allMessages import serialCameraRaw, serialCamera, PerceptionContext, Record


class threadPerception(ThreadWithStop):
    def __init__(self, queueList, logger, debugger=False):
        super(threadPerception, self).__init__(pause=0.001)
        self.queueList = queueList
        self.logger = logger
        self.debugger = bool(debugger)

        # ---- Subscribers / Senders ----
        self.serialFrameSub = messageHandlerSubscriber(self.queueList, serialCameraRaw, "lastOnly", True)
        self.ctxSender = messageHandlerSender(self.queueList, PerceptionContext)
        self.serialVizSender = messageHandlerSender(self.queueList, serialCamera)

        # Recording control (overlay video)
        self.recordSub = messageHandlerSubscriber(self.queueList, Record, "lastOnly", True)
        self.recording_overlay = False
        self.overlay_writer = None
        self.overlay_path = None

        # ---- Rates ----
        self.PERCEPTION_HZ = 10.0
        self.YOLO_EVERY_N = 999999   # YOLO o data la N tick-uri
        #DAshboard
        self.VIZ_EVERY_N = 8    # trimite viz in dashboard o data la N tick-uri
        self.VIZ_JPEG_QUALITY = 45   # ~1.6 fps la 10 Hz
        self.DASHBOARD_W = 384
        self.DASHBOARD_H = 216

        self._tick = 0
        self._last_log_ts = 0.0

        # ---- Latest state ----
        self.last_lane = None
        self.last_lane_ts = 0.0

        self.last_signs = None
        self.last_signs_ts = 0.0

        self.last_tl = None
        self.last_tl_ts = 0.0

        # ---- Algorithms ----
        self.detector = None
        self.lane_keeper = None
        self.sign_detector = None
        self.tl_detector = None  # optional

        self._init_lane_algorithms()
        self._init_traffic_sign_detector()
        self._init_traffic_light_detector_optional()

        # ---- Frame cache ----
        self._last_frame = None
        self._last_frame_ts = 0.0

    # ========================= Helpers =========================
    def _parse_bool(self, x):
        if x is None:
            return None
        if isinstance(x, bool):
            return x
        s = str(x).strip().lower()
        if s in ("true", "1", "yes", "on"):
            return True
        if s in ("false", "0", "no", "off"):
            return False
        return bool(x)

    def _encode_jpg_b64(self, frame_bgr, jpg_quality=70):
        try:
            ok, enc = cv2.imencode(".jpg", frame_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(jpg_quality)])
            if not ok:
                return None
            return base64.b64encode(enc).decode("utf-8")
        except Exception:
            return None

    def _decode_frame(self, b64_jpg: str):
        t0 = time.time()
        try:
            jpg_bytes = base64.b64decode(b64_jpg)
            arr = np.frombuffer(jpg_bytes, dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            return frame, (time.time() - t0) * 1000.0
        except Exception:
            return None, (time.time() - t0) * 1000.0

    # ========================= Init Lane =========================
    def _init_lane_algorithms(self):
        try:
            from src.algorithms.LaneAssist.detect import LaneDetection
            from src.algorithms.LaneAssist.lanekeeping import LaneKeeping as LKAlgo

            self.lane_keeper = LKAlgo()
            self.detector = LaneDetection(lk_object=self.lane_keeper)

            if self.debugger:
                self.logger.info("[Perception] LaneAssist initialized.")
        except Exception as e:
            self.detector = None
            self.lane_keeper = None
            self.logger.warning(f"[Perception] LaneAssist NOT initialized: {e}")
            if self.debugger:
                traceback.print_exc()

    # ========================= Init YOLOv8 Signs =========================
    def _init_traffic_sign_detector(self):
        model_path = "model/bfmc_v2_best.pt"
        try:
            from src.algorithms.TrafficSigns.tsd_yolov8 import TrafficSignDetectorYOLOv8

            if not os.path.exists(model_path):
                raise FileNotFoundError(f"YOLO model not found at {model_path}")

            # NOTE: imgsz=640 conform configului de train :contentReference[oaicite:2]{index=2}
            self.sign_detector = TrafficSignDetectorYOLOv8(
                model_path=model_path,
                conf=0.20,
                imgsz=640,
            )

            if self.debugger:
                self.logger.info("[Perception] TrafficSign YOLO initialized.")
        except Exception as e:
            self.sign_detector = None
            self.logger.warning(f"[Perception] TrafficSign YOLO NOT initialized: {e}")
            if self.debugger:
                traceback.print_exc()

    # ========================= Init Traffic Light (optional) =========================
    def _init_traffic_light_detector_optional(self):
        # Lasi None ca sa nu crape daca modulul nu exista
        self.tl_detector = None

    # ========================= Lane =========================
    def _run_lane(self, frame_bgr):
        t0 = time.time()

        if self.detector is None or self.lane_keeper is None:
            return None, None, (time.time() - t0) * 1000.0

        try:
            lane_results = self.detector.lanes_detection(frame_bgr.copy())

            if lane_results is None:
                lane = {
                    "steer": 0,
                    "confidence": 0.0,
                    "trust_left": False,
                    "trust_right": False,
                    "status": "NO_DETECTION",
                    "timestamp": time.time(),
                }
                return lane, frame_bgr, (time.time() - t0) * 1000.0

            steer_angle, debug_frame = self.lane_keeper.lane_keeping(lane_results)

            trust_left = bool(lane_results.get("trust_left", False))
            trust_right = bool(lane_results.get("trust_right", False))

            if trust_left and trust_right:
                conf = 1.0
            elif trust_left or trust_right:
                conf = 0.6
            else:
                conf = 0.2

            lane = {
                "steer": int(steer_angle),
                "confidence": float(conf),
                "trust_left": trust_left,
                "trust_right": trust_right,
                "status": "ACTIVE",
                "timestamp": time.time(),
            }

            if debug_frame is None:
                debug_frame = lane_results.get("frame", frame_bgr)

            return lane, debug_frame, (time.time() - t0) * 1000.0

        except Exception:
            if self.debugger:
                traceback.print_exc()

            lane = {
                "steer": 0,
                "confidence": 0.0,
                "trust_left": False,
                "trust_right": False,
                "status": "CRASH",
                "timestamp": time.time(),
            }
            return lane, frame_bgr, (time.time() - t0) * 1000.0

    # ========================= Signs =========================
    def _norm_name(self, name):
        return str(name or "").strip().lower()

    def _run_signs(self, frame_bgr):
        t0 = time.time()

        if self.sign_detector is None:
            return None, (time.time() - t0) * 1000.0

        try:
            dets = self.sign_detector.detect(frame_bgr) or []
            dets = dets[:5]

            # normalize for flags
            names = [self._norm_name(d.get("name", "")) for d in dets]

            # robust flags (accept variants)
            stop_seen = any(n in ("stop", "stop_sign", "stopsign") for n in names)
            ped_seen = any(n in ("pedestrian", "person", "pedestrian_sign") for n in names)
            cw_seen = any(n in ("crosswalk", "cross_walk", "zebra_crossing") for n in names)

            if self.debugger:
                top = []
                for d in dets:
                    try:
                        top.append((self._norm_name(d.get("name", "")), float(d.get("conf", 0.0))))
                    except Exception:
                        pass
                top = sorted(top, key=lambda x: x[1], reverse=True)
                self.logger.info(f"[TSD] top={[(n, round(c, 3)) for (n, c) in top][:5]}")

            signs = {
                "timestamp": time.time(),
                "count": int(len(dets)),
                "detections": dets,
                "stop_sign": bool(stop_seen),  # compat
                "flags": {
                    "stop_sign": bool(stop_seen),
                    "pedestrian": bool(ped_seen),
                    "crosswalk": bool(cw_seen),
                },
                "status": "ACTIVE",
            }
            return signs, (time.time() - t0) * 1000.0

        except Exception:
            if self.debugger:
                traceback.print_exc()
            signs = {
                "timestamp": time.time(),
                "count": 0,
                "detections": [],
                "stop_sign": False,
                "flags": {"stop_sign": False, "pedestrian": False, "crosswalk": False},
                "status": "CRASH",
            }
            return signs, (time.time() - t0) * 1000.0

    # ========================= Traffic Light (optional) =========================
    def _run_tl_optional(self, frame_bgr):
        t0 = time.time()

        if self.tl_detector is None:
            tl = {
                "timestamp": time.time(),
                "state": None,
                "confidence": 0.0,
                "bbox": [0, 0, 0, 0],
                "source": "none",
                "status": "DISABLED",
            }
            return tl, (time.time() - t0) * 1000.0

        try:
            state, conf, bbox = self.tl_detector.detect(frame_bgr)
            tl = {
                "timestamp": time.time(),
                "state": state,
                "confidence": float(conf),
                "bbox": list(bbox),
                "source": "custom",
                "status": "ACTIVE",
            }
            return tl, (time.time() - t0) * 1000.0
        except Exception:
            if self.debugger:
                traceback.print_exc()
            tl = {
                "timestamp": time.time(),
                "state": "unknown",
                "confidence": 0.0,
                "bbox": [0, 0, 0, 0],
                "source": "custom",
                "status": "CRASH",
            }
            return tl, (time.time() - t0) * 1000.0

    # ========================= Main loop =========================
    def thread_work(self):
        period = 1.0 / float(self.PERCEPTION_HZ)
        start = time.time()

        # ---- Get latest base64 frame ----
        b64_frame = self.serialFrameSub.receive()
        if b64_frame is None:
            time.sleep(0.005)
            return

        frame, decode_ms = self._decode_frame(b64_frame)
        if frame is None:
            time.sleep(0.005)
            return

        self._last_frame = frame
        self._last_frame_ts = time.time()

        # ---- Record overlay toggle ----
        rec = self.recordSub.receive()
        rec_bool = self._parse_bool(rec)
        if rec_bool is not None and rec_bool != self.recording_overlay:
            self.recording_overlay = rec_bool

            if not self.recording_overlay:
                if self.overlay_writer is not None:
                    self.overlay_writer.release()
                    self.overlay_writer = None
                if self.debugger:
                    self.logger.info("[Perception] Overlay recording STOPPED")
            else:
                self.overlay_path = f"lane_overlay_{int(time.time())}.avi"
                if self.debugger:
                    self.logger.info(f"[Perception] Overlay recording STARTED -> {self.overlay_path}")

        self._tick += 1

        # ---- Lane every tick ----
        lane, lane_debug, lane_ms = self._run_lane(self._last_frame)
        if lane is not None:
            self.last_lane = lane
            self.last_lane_ts = float(lane.get("timestamp", time.time()))

        # ---- Save overlay if recording ----
        if self.recording_overlay and lane_debug is not None:
            if self.overlay_writer is None:
                h, w = lane_debug.shape[:2]
                fourcc = cv2.VideoWriter_fourcc(*"XVID")
                self.overlay_writer = cv2.VideoWriter(self.overlay_path, fourcc, float(self.PERCEPTION_HZ), (w, h))
            self.overlay_writer.write(lane_debug)

        # ---- Send viz to dashboard ----
        if (self._tick % int(self.VIZ_EVERY_N)) == 0:
            frame_to_send = lane_debug if lane_debug is not None else self._last_frame

            try:
                dash_frame = cv2.resize(
                    frame_to_send,
                    (self.DASHBOARD_W, self.DASHBOARD_H),
                    interpolation=cv2.INTER_AREA
                )
            except Exception:
                dash_frame = frame_to_send

            b64_vis = self._encode_jpg_b64(dash_frame, jpg_quality=self.VIZ_JPEG_QUALITY)
            if b64_vis is not None:
                self.serialVizSender.send(b64_vis)

        # ---- YOLO throttled ----
        signs_ms = 0.0
        if (self._tick % int(self.YOLO_EVERY_N)) == 0:
            signs, signs_ms = self._run_signs(self._last_frame)
            if signs is not None:
                self.last_signs = signs
                self.last_signs_ts = float(signs.get("timestamp", time.time()))

        # ---- Traffic light optional ----
        tl, tl_ms = self._run_tl_optional(self._last_frame)
        self.last_tl = tl
        self.last_tl_ts = float(tl.get("timestamp", time.time()))

        # ---- Build context ----
        now = time.time()
        ctx = {
            "timestamp": now,
            "frame_ts": float(self._last_frame_ts),
            "lane": self.last_lane,
            "traffic_signs": self.last_signs,
            "traffic_light": self.last_tl,
            "age": {
                "lane_s": None if self.last_lane_ts == 0 else float(now - self.last_lane_ts),
                "signs_s": None if self.last_signs_ts == 0 else float(now - self.last_signs_ts),
                "tl_s": None if self.last_tl_ts == 0 else float(now - self.last_tl_ts),
            },
            "debug": {
                "frame_decode_ms": float(decode_ms),
                "lane_ms": float(lane_ms),
                "tsd_ms": float(signs_ms),
                "tl_ms": float(tl_ms),
                "tick": int(self._tick),
            },
        }

        self.ctxSender.send(ctx)

        # ---- Periodic log ----
        if self.debugger and (now - self._last_log_ts) > 1.0:
            self._last_log_ts = now
            lane_steer = ctx["lane"]["steer"] if ctx.get("lane") else None
            lane_conf = ctx["lane"]["confidence"] if ctx.get("lane") else None
            flags = {}
            if ctx.get("traffic_signs") and isinstance(ctx["traffic_signs"], dict):
                flags = ctx["traffic_signs"].get("flags", {})

            self.logger.info(
                f"[Perception] tick={self._tick} lane_steer={lane_steer} lane_conf={lane_conf} flags={flags}"
            )

        # ---- Rate limit ----
        elapsed = time.time() - start
        time.sleep(max(0.0, period - elapsed))

    def stop(self):
        # close overlay writer cleanly
        try:
            if self.overlay_writer is not None:
                self.overlay_writer.release()
                self.overlay_writer = None
        except Exception:
            pass
        super(threadPerception, self).stop()