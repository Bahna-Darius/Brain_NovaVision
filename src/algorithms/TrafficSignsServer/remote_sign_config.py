# Copyright (c) 2026, NovaVision
# All rights reserved.

import os


class RemoteSignConfig:
    # Laptop server address. For Pi hotspot setups, the laptop often ends up at 192.168.4.2.
    SERVER_HOST = os.getenv("BFMC_SIGN_SERVER_HOST", "192.168.1.100")
    SERVER_PORT = int(os.getenv("BFMC_SIGN_SERVER_PORT", "5051"))

    # How often the Pi sends a new frame to the laptop.
    SEND_HZ = float(os.getenv("BFMC_SIGN_SEND_HZ", "5.0"))

    # Network timing.
    CONNECT_TIMEOUT_S = float(os.getenv("BFMC_SIGN_CONNECT_TIMEOUT_S", "2.0"))
    RECV_TIMEOUT_S = float(os.getenv("BFMC_SIGN_RECV_TIMEOUT_S", "1.5"))
    RETRY_AFTER_FAIL_S = float(os.getenv("BFMC_SIGN_RETRY_AFTER_FAIL_S", "1.0"))

    # Safety: ignore detections that arrive too late.
    MAX_RESULT_AGE_S = float(os.getenv("BFMC_SIGN_MAX_RESULT_AGE_S", "1.0"))
