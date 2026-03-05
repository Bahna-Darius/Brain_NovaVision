# Copyright (c) 2026, NovaVision
# All rights reserved.

"""
Lane-focused Perception thread:
- Subscribes to serialCameraRaw (base64 JPEG)
- Decodes once to BGR
- Runs LaneAssist every tick
- Fuses latest TrafficSignDetections into PerceptionContext
- Publishes PerceptionContext and optional debug overlay
"""

import base64
import time
import traceback

import cv2
import numpy as np

from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.allMessages import (
    serialCameraRaw,
    serialCamera,
    PerceptionContext,
    TrafficSignDetections,
    Record,
)


class threadPerception(ThreadWithStop):
    def __init__(self, queueList, logger, debugger=False):
        super(threadPerception, self).__init__(pause=0.001)
        self.queueList = queueList
        self.logger = logger
        self.debugger = bool(debugger)

        # ---- Subscribers / Senders ----
        self.serialFrameSub = messageHandlerSubscriber(self.queueList, serialCameraRaw, "lastOnly", True)
        self.signSub = messageHandlerSubscriber(self.queueList, TrafficSignDetections, "lastOnly", True)
        self.ctxSender = messageHandlerSender(self.queueList, PerceptionContext)
        self.debugSender = messageHandlerSender(self.queueList, serialCamera)

        # Recording control (overlay video)
        self.recordSub = messageHandlerSubscriber(self.queueList, Record, "lastOnly", True)
        self.recording_overlay = False
        self.overlay_writer = None
        self.overlay_path = None

        # ---- Rates ----
        self.PERCEPTION_HZ = 30.0
        self._tick = 0
        self._last_log_ts = 0.0
        self.DEBUG_EVERY_N = 1

        # ---- Latest state ----
        self.last_lane = None
        self.last_lane_ts = 0.0
        self.last_signs = None
        self.last_signs_ts = 0.0

        # ---- Algorithms ----
        self.detector = None
        self.lane_keeper = None
        self._init_lane_algorithms()

        # ---- Frame cache ----
        self._last_frame = None
        self._last_frame_ts = 0.0

        # ---- frame count ---
        self._fps_window_start = time.time()
        self._fps_window_count = 0
        self._lane_ms_acc = 0.0
        self._decode_ms_acc = 0.0

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

    def _decode_frame(self, b64_jpg: str):
        t0 = time.time()
        try:
            jpg_bytes = base64.b64decode(b64_jpg)
            arr = np.frombuffer(jpg_bytes, dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            return frame, (time.time() - t0) * 1000.0
        except Exception:
            return None, (time.time() - t0) * 1000.0

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

    def thread_work(self):
        try:
            period = 1.0 / float(self.PERCEPTION_HZ)
            start = time.time()

            raw_frame = self.serialFrameSub.receive()
            if raw_frame is None:
                time.sleep(0.005)
                return

            if isinstance(raw_frame, np.ndarray):
                frame = raw_frame
                decode_ms = 0.0
            elif isinstance(raw_frame, str):
                frame, decode_ms = self._decode_frame(raw_frame)
            else:
                time.sleep(0.005)
                return

            self._last_frame = frame
            self._last_frame_ts = time.time()

            sign_msg = self.signSub.receive()
            if isinstance(sign_msg, dict):
                self.last_signs = sign_msg
                self.last_signs_ts = float(sign_msg.get("timestamp", time.time()))

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

            lane, lane_debug, lane_ms = self._run_lane(self._last_frame)

            self._fps_window_count += 1
            self._lane_ms_acc += float(lane_ms)
            self._decode_ms_acc += float(decode_ms)

            if lane is not None:
                self.last_lane = lane
                self.last_lane_ts = float(lane.get("timestamp", time.time()))

            if lane_debug is not None and (self._tick % self.DEBUG_EVERY_N == 0):
                ok, enc = cv2.imencode(".jpg", lane_debug, [int(cv2.IMWRITE_JPEG_QUALITY), 45])
                if ok:
                    self.debugSender.send(base64.b64encode(enc).decode("utf-8"))

            if self.recording_overlay and lane_debug is not None:
                if self.overlay_writer is None:
                    h, w = lane_debug.shape[:2]
                    fourcc = cv2.VideoWriter_fourcc(*"XVID")
                    self.overlay_writer = cv2.VideoWriter(
                        self.overlay_path,
                        fourcc,
                        float(self.PERCEPTION_HZ),
                        (w, h),
                    )
                self.overlay_writer.write(lane_debug)

            now = time.time()
            ctx = {
                "timestamp": now,
                "frame_ts": float(self._last_frame_ts),
                "lane": self.last_lane,
                "traffic_signs": self.last_signs,
                "traffic_light": {
                    "timestamp": now,
                    "state": None,
                    "confidence": 0.0,
                    "bbox": [0, 0, 0, 0],
                    "source": "none",
                    "status": "DISABLED",
                },
                "age": {
                    "lane_s": None if self.last_lane_ts == 0 else float(now - self.last_lane_ts),
                    "signs_s": None if self.last_signs_ts == 0 else float(now - self.last_signs_ts),
                    "tl_s": 0.0,
                },
                "debug": {
                    "frame_decode_ms": float(decode_ms),
                    "lane_ms": float(lane_ms),
                    "tsd_ms": float((self.last_signs or {}).get("debug", {}).get("tsd_ms", 0.0)) if isinstance(self.last_signs, dict) else 0.0,
                    "tl_ms": 0.0,
                    "tick": int(self._tick),
                },
            }

            self.ctxSender.send(ctx)

            now = time.time()
            window_dt = now - self._fps_window_start

            if self.debugger and window_dt >= 1.0:
                effective_fps = self._fps_window_count / window_dt if window_dt > 0 else 0.0
                avg_lane_ms = self._lane_ms_acc / self._fps_window_count if self._fps_window_count > 0 else 0.0
                avg_decode_ms = self._decode_ms_acc / self._fps_window_count if self._fps_window_count > 0 else 0.0

                lane_steer = ctx["lane"]["steer"] if ctx.get("lane") else None
                lane_conf = ctx["lane"]["confidence"] if ctx.get("lane") else None

                self.logger.info(
                    f"[Perception] fps={effective_fps:.1f} avg_lane_ms={avg_lane_ms:.1f} "
                    f"avg_decode_ms={avg_decode_ms:.1f} lane_steer={lane_steer} lane_conf={lane_conf}"
                )

                self._fps_window_start = now
                self._fps_window_count = 0
                self._lane_ms_acc = 0.0
                self._decode_ms_acc = 0.0

            elapsed = time.time() - start
            time.sleep(max(0.0, period - elapsed))

        except Exception as e:
            print(f"[Perception] thread_work CRASH: {e}")
            traceback.print_exc()
            time.sleep(0.1)

    def stop(self):
        try:
            if self.overlay_writer is not None:
                self.overlay_writer.release()
                self.overlay_writer = None
        except Exception:
            pass
        super(threadPerception, self).stop()
