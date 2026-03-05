class LaneConfig:
    # --- Image Dimensions ---
    # Trebuie să se potrivească cu 'lores' din threadCamera.py (512x270)
    WIDTH = 512
    HEIGHT = 270

    # --- Lane Detection ---
    CUSTOM_FIND_PEAKS = True
    SLICES = 20  # 20 felii (ca în original) pentru linii mai fine
    BOTTOM_OFFSET = 5

    # Perspective parameters (Bird's Eye View)
    # Scădem puțin ca să ne concentrăm pe zona imediată din fața mașinii
    BOTTOM_PERC = 0.47

    # Peak Detection Thresholds
    PEAKS_MIN_WIDTH = 2
    PEAKS_MAX_WIDTH = 20

    # Pragul de alb (0-255).
    # Îl creștem la 160 pentru a elimina "fantom-ele" de pe podea.
    # Doar benzile albe strălucitoare vor trece.
    SQUARE_PULSES_MIN_HEIGHT = 170

    SQUARE_PULSES_PIX_DIF = 2
    SQUARE_PULSES_MIN_HEIGHT_DIF = 40
    SQUARE_PULSES_ALLOWED_PEAKS_WIDTH_ERROR = 8

    # Validation
    MIN_PEAKS_FOR_LANE = 2
    OPTIMAL_PEAK_PERC = 0.3
    MAX_ALLOWED_WIDTH_PERC = 0.20

    WEIGHT_FOR_WIDTH_DISTANCE = 0.3
    WEIGHT_FOR_EXPECTED_VALUE_DISTANCE = 0.7

    ALLOWED_CERTAINTY_PERC_DIF = 40.0
    CERTAINTY_PERC_FROM_PEAKS = 0.5

    # Coefficients
    EXTREME_COEF_SECOND_DEG = 0.1
    EXTREME_COEF_FIRST_DEG = 3

    MIN_SINGLE_LANE_CERTAINTY = 30
    MIN_DUAL_LANE_CERTAINTY = 30

    # --- Lane Keeping ---
    MAX_STEER = 25.0
    MEDIAN_CONSTANT = 2

    # Weights parameters
    MU = 0.5
    SIGMA = 0.4

    # Lane Widths (Scaled for 512 width) - FIXUL PRINCIPAL
    # Am redus semnificativ lățimea estimată a benzii.
    # Dacă tot virează stânga când e pe mijloc, mai SCADE aceste valori (ex: 85 / 45).
    BOTTOM_WIDTH = 90  # Era 115
    TOP_WIDTH = 50  # Era 65

    # --- PID & Control ---
    KP = 0.12  # Reacție puțin mai fermă
    KI = 0.0
    KD = 0.05
    STEP_DT = 0.1

    # Debugging
    PRINT_LANES = True
    PRINT_PEAKS = True
    PRINT_LANE_CERTAINTY = False
    PRINT_DESIRE_LANE = True
    DEBUG = True