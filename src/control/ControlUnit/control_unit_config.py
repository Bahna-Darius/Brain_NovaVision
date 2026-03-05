class ControlUnitConfig:
    # ===== Cruise speeds =====
    CITY_BASE_SPEED = 300
    HIGHWAY_BASE_SPEED = 500
    STOP_SPEED = 0
    CROSSWALK_SPEED = 150

    # ===== Curve speed policy =====
    # Lane steer is scaled by x10 before being sent downstream.
    MAX_LANE_STEER = 250
    CURVE_ENTER_STEER = 60
    CURVE_EXIT_STEER = 30
    CURVE_ENTER_CONFIRM_FRAMES = 3
    CURVE_EXIT_CONFIRM_FRAMES = 4
    CURVE_MIN_SPEED_CITY = 100
    CURVE_MIN_SPEED_HIGHWAY = 200
    CURVE_FULL_STEER = 250

    # ===== Lane confidence =====
    MIN_LANE_CONFIDENCE = 0.30

    # ===== Sign handling =====
    STOP_HOLD_S = 3.0
    STOP_COOLDOWN_S = 6.0
    STOP_NEAR_AREA_RATIO = 0.06

    HIGHWAY_SIGN_COOLDOWN_S = 3.0
    HIGHWAY_ENTRY_NEAR_AREA_RATIO = 0.06
    HIGHWAY_EXIT_NEAR_AREA_RATIO = 0.06

    CROSSWALK_HOLD_S = 3.2
    CROSSWALK_MIN_AREA_RATIO = 0.05
    DISABLE_STEER_ON_CROSSWALK = False

    # ===== Loop / freshness =====
    LOOP_HZ = 30
    CTX_TIMEOUT_S = 0.30
    SIGN_MAX_AGE_S = 0.60
