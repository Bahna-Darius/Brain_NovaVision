class ControlUnitConfig:
    # ===== Cruise speeds =====
    CITY_BASE_SPEED = 300
    HIGHWAY_BASE_SPEED = 500
    STOP_SPEED = 0
    CROSSWALK_SPEED = 150

    # ===== Curve speed policy =====
    # Lane steer is scaled by x10 before being sent downstream.
    MAX_LANE_STEER = 250

    # Curve state hysteresis.
    CURVE_ENTER_CONFIRM_FRAMES = 1
    CURVE_EXIT_CONFIRM_FRAMES = 6

    # Speed floor while cornering.
    CURVE_MIN_SPEED_CITY = 100
    CURVE_MIN_SPEED_HIGHWAY = 120
    CURVE_FULL_STEER = 250

    # Preview-based curve planning.
    CURVE_ENTER_SCORE = 0.12
    CURVE_EXIT_SCORE = 0.15
    CURVE_SCORE_ALPHA = 0.45

    PREVIEW_FAR_ERR_FULL_PX = 70.0
    PREVIEW_ANTICIPATION_FULL_PX = 40.0
    PREVIEW_HEADING_DELTA_FULL_DEG = 16.0
    PREVIEW_CURVATURE_FULL_SCALE = 90.0

    # ===== Lane confidence =====
    MIN_LANE_CONFIDENCE = 0.30

    # ===== Sign handling =====
    STOP_HOLD_S = 3.0
    STOP_COOLDOWN_S = 6.0
    STOP_NEAR_AREA_RATIO = 0.07

    HIGHWAY_SIGN_COOLDOWN_S = 3.0
    HIGHWAY_ENTRY_NEAR_AREA_RATIO = 0.06
    HIGHWAY_EXIT_NEAR_AREA_RATIO = 0.06

    CROSSWALK_HOLD_S = 3.2
    CROSSWALK_MIN_AREA_RATIO = 0.07
    DISABLE_STEER_ON_CROSSWALK = False

    # ===== Loop / freshness =====
    LOOP_HZ = 30
    CTX_TIMEOUT_S = 0.30
    SIGN_MAX_AGE_S = 0.60

        # ===== Parking =====
    PARKING_SIGN_TRIGGER_LABEL = "parking_sign"
    PARKING_TRIGGER_CONFIRM_FRAMES = 3
    PARKING_TRIGGER_AREA_RATIO = 0.05
    PARKING_TRIGGER_COOLDOWN_S = 8.0

    # Paper-inspired geometry parameters for future upgrade
    PARKING_WHEELBASE_M = 0.26
    PARKING_VEHICLE_WIDTH_M = 0.20
    PARKING_REAR_OVERHANG_M = 0.35
    PARKING_STEERING_ANGLE_DEG = 25.0

    # Practical timed first version
    PARKING_FORWARD_OFFSET_S = 12
    PARKING_STOP_BEFORE_REVERSE_S = 1.35

    PARKING_REVERSE_SPEED = -150
    PARKING_FORWARD_SPEED = 120

    PARKING_ARC1_STEER = 250
    PARKING_ARC2_STEER = -250

    PARKING_ARC1_DURATION_S = 5.20
    PARKING_ARC2_DURATION_S = 5.20
    PARKING_FINAL_FORWARD_DURATION_S = 1.5