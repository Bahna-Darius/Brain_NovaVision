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
    NovaVision 26.01.2026
    - Reads PerceptionContext (lane info is inside ctx["lane"])
    - In AUTO: computes desired speed/steer and publishes DesiredSpeed/DesiredSteer
    """

    def __init__(self, queueList, stop_event, logger=None, debugging=False):
        super().__init__()
        self.queueList = queueList
        self.stop_event = stop_event
        self.logger = logger
        self.debugging = debugging

        # Subscribers
        self.ctxSub = messageHandlerSubscriber(self.queueList, PerceptionContext, "lastOnly", True)
        self.stateSub = messageHandlerSubscriber(self.queueList, StateChange, "lastOnly", True)

        # Senders
        self.speedSender = messageHandlerSender(self.queueList, DesiredSpeed)
        self.steerSender = messageHandlerSender(self.queueList, DesiredSteer)

        self.current_mode = "MANUAL"
        self._tick = 0
        self._last_ctx = None
        self._last_ctx_time = 0.0

    def run(self):
        period = 1.0 / CFG.LOOP_HZ

        while not self.stop_event.is_set():
            start = time.time()
            self._tick += 1

            stateMsg = self.stateSub.receive()
            if stateMsg is not None:
                self.current_mode = stateMsg

            ctx = self.ctxSub.receive()
            if isinstance(ctx, dict):
                self._last_ctx = ctx
                self._last_ctx_time = time.time()

            if self.current_mode == "AUTO":
                # Use cached context
                self._auto_from_context(self._last_ctx, self._last_ctx_time)

            time.sleep(max(0.0, period - (time.time() - start)))

    def _auto_from_context(self, ctx, ctx_time):
        # ✅ NovaVision: stop only if context is missing OR too old
        if not isinstance(ctx, dict):
            self.speedSender.send(CFG.STOP_SPEED)
            self.steerSender.send(0)
            return

        if (time.time() - float(ctx_time)) > CFG.CTX_TIMEOUT_S:
            self.speedSender.send(CFG.STOP_SPEED)
            self.steerSender.send(0)
            return

        lane = ctx.get("lane") or {}
        raw_steer = int(lane.get("steer", 0))
        steer = int(raw_steer * 10)

        conf = float(lane.get("confidence", 0.0))

        if conf < CFG.MIN_LANE_CONFIDENCE:
            speed = CFG.STOP_SPEED
            steer = 0
        else:
            steer = max(-CFG.MAX_LANE_STEER, min(CFG.MAX_LANE_STEER, steer))
            speed = CFG.CURVE_SPEED if abs(steer) > CFG.CURVE_STEER_THRESHOLD else CFG.BASE_SPEED

        self.speedSender.send(int(speed))
        self.steerSender.send(int(steer))

        if self.debugging and (self._tick % 20 == 0):
            print(f"[ControlUnit] tick={self._tick} conf={conf:.2f} desired_speed={speed} desired_steer={steer}")
