# Copyright (c) 2026, NovaVision
# All rights reserved.

"""
Pi-side remote traffic-sign client:
- Subscribes to serialCamera (base64 JPEG) from threadCamera
- Sends the latest frame to the laptop over TCP
- Receives YOLO detections back as JSON
- Publishes TrafficSignDetections into the Brain message bus
"""

import json
import socket
import time
import traceback

from src.templates.threadwithstop import ThreadWithStop
from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.allMessages import mainCamera, TrafficSignDetections
from src.algorithms.TrafficSignsServer.remote_sign_config import RemoteSignConfig as CFG


class threadTrafficSignClient(ThreadWithStop):
    def __init__(self, queueList, logger, debugger=False):
        super(threadTrafficSignClient, self).__init__(pause=0.001)
        self.queueList = queueList
        self.logger = logger
        self.debugger = bool(debugger)

        self.frameSub = messageHandlerSubscriber(self.queueList, mainCamera, "lastOnly", True)
        self.signSender = messageHandlerSender(self.queueList, TrafficSignDetections)

        self.sock = None
        self._recv_buffer = b""
        self._last_send_ts = 0.0
        self._last_log_ts = 0.0
        self._last_connect_try_ts = 0.0
        self._tick = 0

    def _connect(self):
        now = time.time()
        if self.sock is not None:
            return True

        if (now - self._last_connect_try_ts) < CFG.RETRY_AFTER_FAIL_S:
            return False

        self._last_connect_try_ts = now
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(CFG.CONNECT_TIMEOUT_S)
            sock.connect((CFG.SERVER_HOST, CFG.SERVER_PORT))
            sock.settimeout(CFG.RECV_TIMEOUT_S)
            self.sock = sock
            self._recv_buffer = b""
            print(f"[TrafficSignClient] Connected to {CFG.SERVER_HOST}:{CFG.SERVER_PORT}")
            return True
        except Exception as e:
            self.sock = None
            if self.debugger:
                print(f"[TrafficSignClient] Connect failed: {e}")
            return False

    def _disconnect(self):
        try:
            if self.sock is not None:
                self.sock.close()
        except Exception:
            pass
        self.sock = None
        self._recv_buffer = b""

    def _send_json(self, obj):
        data = (json.dumps(obj, separators=(",", ":")) + "\n").encode("utf-8")
        self.sock.sendall(data)

    def _recv_json_line(self):
        while b"\n" not in self._recv_buffer:
            chunk = self.sock.recv(65536)
            if not chunk:
                raise ConnectionError("server closed connection")
            self._recv_buffer += chunk

        line, self._recv_buffer = self._recv_buffer.split(b"\n", 1)
        return json.loads(line.decode("utf-8"))

    def thread_work(self):
        try:
            period = 1.0 / float(max(0.5, CFG.SEND_HZ))
            now = time.time()

            if (now - self._last_send_ts) < period:
                time.sleep(0.002)
                return

            frame_b64 = self.frameSub.receive()
            if frame_b64 is None or not isinstance(frame_b64, str):
                time.sleep(0.005)
                return

            if not self._connect():
                time.sleep(0.01)
                return

            self._tick += 1
            req_ts = time.time()
            payload = {
                "type": "frame",
                "frame_ts": req_ts,
                "frame_b64": frame_b64,
            }

            self._send_json(payload)
            response = self._recv_json_line()
            self._last_send_ts = time.time()

            if not isinstance(response, dict):
                return

            if response.get("type") != "detections":
                return

            result_ts = float(response.get("timestamp", time.time()))
            age_s = float(time.time() - result_ts)
            if age_s > CFG.MAX_RESULT_AGE_S:
                if self.debugger:
                    print(f"[TrafficSignClient] Dropping stale result age={age_s:.3f}s")
                return

            signs = {
                "timestamp": result_ts,
                "count": int(response.get("count", 0)),
                "detections": response.get("detections", []),
                "stop_sign": bool(response.get("stop_sign", False)),
                "flags": response.get("flags", {}),
                "status": str(response.get("status", "ACTIVE")),
                "source": str(response.get("source", "remote_yolov8")),
                "debug": {
                    "tsd_ms": float(response.get("debug", {}).get("tsd_ms", 0.0)),
                    "rtt_ms": float((time.time() - req_ts) * 1000.0),
                    "tick": int(self._tick),
                },
            }
            self.signSender.send(signs)

            if self.debugger and (time.time() - self._last_log_ts) > 1.0:
                self._last_log_ts = time.time()
                print(
                    f"[TrafficSignClient] tick={self._tick} count={signs['count']} "
                    f"infer_ms={signs['debug']['tsd_ms']:.1f} rtt_ms={signs['debug']['rtt_ms']:.1f}"
                )

        except (socket.timeout, ConnectionError, OSError) as e:
            if self.debugger:
                print(f"[TrafficSignClient] Socket error: {e}")
            self._disconnect()
            time.sleep(0.1)
        except Exception as e:
            print(f"[TrafficSignClient] CRASH: {e}")
            if self.debugger:
                traceback.print_exc()
            self._disconnect()
            time.sleep(0.2)

    def stop(self):
        self._disconnect()
        super(threadTrafficSignClient, self).stop()
