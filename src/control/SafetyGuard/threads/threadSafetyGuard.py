# Copyright (c) 2026, NovaVision
# All rights reserved.

import threading
import time

from src.utils.messages.messageHandlerSubscriber import messageHandlerSubscriber
from src.utils.messages.messageHandlerSender import messageHandlerSender
from src.utils.messages.allMessages import (
    SafetyOverride,
    StateChange,
    PerceptionContext,
)

from src.control.SafetyGuard.safety_guard_config import SafetyGuardConfig as CFG


################ NovaVision 26.01.2026 ###############
# Context:
# - Updated SafetyGuard to consume PerceptionContext (fused output) instead of LaneKeeping
# - SafetyGuard now evaluates lane freshness via ctx["lane"]["timestamp"] and confidence/status.
#####################################################


class threadSafetyGuard(threading.Thread):
    def __init__(self, queueList, stop_event, debugging=False):
        super().__init__()
        self.queueList = queueList
        self.stop_event = stop_event

        # ✅ NovaVision Debugger flag
        self.debugger = bool(debugging)

        self.ctxSub = messageHandlerSubscriber(self.queueList, PerceptionContext, "lastOnly", True)
        self.stateSub = messageHandlerSubscriber(self.queueList, StateChange, "lastOnly", True)

        self.sender = messageHandlerSender(self.queueList, SafetyOverride)

        self.current_mode = "MANUAL"

        self.last_lane_time = time.time()
        self.last_lane_conf = 0.0
        self.last_lane_status = "IDLE"


    def run(self):
        while not self.stop_event.is_set():

            stateMsg = self.stateSub.receive()
            if stateMsg is not None:
                self.current_mode = str(stateMsg)

            ctx = self.ctxSub.receive()

            # ✅ Only touch ctx if it's a dict
            if isinstance(ctx, dict):
                lane = ctx.get("lane")
                if isinstance(lane, dict):
                    self.last_lane_time = time.time()
                    self.last_lane_conf = float(lane.get("confidence", 0.0))
                    self.last_lane_status = str(lane.get("status", "IDLE"))

            if self.current_mode == "AUTO":
                self._evaluate_safety()

            time.sleep(0.05)

    def _evaluate_safety(self):
        now = time.time()

        # ---- Lane timeout (Perception not providing lane updates) ----
        if now - self.last_lane_time > CFG.LANE_TIMEOUT_S:
            self._send_override(stop=True,reason="Perception lane timeout")
            return

        # ---- Lane unhealthy (optional safety escalation) ----
        if self.last_lane_status in ("CRASH", "NO_DETECTION") or self.last_lane_conf <= 0.0:
            self._send_override( stop=True,reason=f"Lane unhealthy ({self.last_lane_status})",)
            return

        # ---- Normal limits ----
        self._send_override(
            stop=False,
            max_speed=CFG.MAX_ABS_SPEED,
            max_steer=CFG.MAX_ABS_STEER,
            reason="Normal",
        )

    def _send_override(self, stop=False, max_speed=None, max_steer=None, reason=""):
        msg = {
            "stop": bool(stop),
            "max_speed": max_speed,
            "max_steer": max_steer,
            "ttl_ms": CFG.DEFAULT_TTL_MS,
            "reason": str(reason),
        }
        ################ NovaVision 26.01.2026 ###############
        # Context:
        # - Debug print to confirm safety override is being produced.
        #####################################################
        if self.debugger:
            print(
                f"[SafetyGuard] stop={msg['stop']} max_speed={msg['max_speed']} "
                f"max_steer={msg['max_steer']} reason={msg['reason']}"
            )

        self.sender.send(msg)
