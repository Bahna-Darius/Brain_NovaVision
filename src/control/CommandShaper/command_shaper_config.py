# This file defines physical limits + behavior, including reverse.

class CommandShaperConfig:
    # ===== Timing =====
    PUBLISH_HZ = 15                 # 20 Hz output
    INPUT_TIMEOUT_S = 0.3           # deadman timeout

    # ===== Speed limits =====
    MAX_SPEED = 500                 # forward
    MIN_SPEED = -500                # reverse (parking)
    MAX_SPEED_STEP = 50             # max delta per cycle

    # ===== Steering limits =====
    MAX_STEER = 250
    MIN_STEER = -250
    MAX_STEER_STEP = 15

    # ===== Default safe state =====
    SAFE_SPEED = 0
    SAFE_STEER = 0
