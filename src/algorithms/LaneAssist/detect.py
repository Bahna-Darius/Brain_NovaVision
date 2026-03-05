import cv2
import numpy as np
import math
from .lane_config import LaneConfig


class LaneDetection:
    def __init__(self, lk_object=None):
        self.width = LaneConfig.WIDTH
        self.height = LaneConfig.HEIGHT
        self.lk = lk_object

        self.miss_left = 0
        self.miss_right = 0
        self.max_fallback_frames = 4

        # Parametri din Config
        self.slices = LaneConfig.SLICES
        self.print_lanes = LaneConfig.PRINT_LANES
        self.print_peaks = LaneConfig.PRINT_PEAKS
        self.print_lane_certainty = LaneConfig.PRINT_LANE_CERTAINTY

        self.extreme_coef_second_deg = LaneConfig.EXTREME_COEF_SECOND_DEG
        self.extreme_coef_first_deg = LaneConfig.EXTREME_COEF_FIRST_DEG

        self.prev_left_coef = None
        self.prev_right_coef = None

        # toleranțe pentru “sanity”
        self.expected_lane_width_px = 2 * LaneConfig.BOTTOM_WIDTH
        self.width_tol_low = 0.80
        self.width_tol_high = 1.25

        # cât de mult ai voie să “sari” între frame-uri (la bottom)
        self.max_jump_px = 35

        self.peaks_min_width = LaneConfig.PEAKS_MIN_WIDTH
        self.peaks_max_width = LaneConfig.PEAKS_MAX_WIDTH
        self.bottom_perc = LaneConfig.BOTTOM_PERC

        self.min_peaks_for_lane = LaneConfig.MIN_PEAKS_FOR_LANE
        self.max_allowed_dist = LaneConfig.MAX_ALLOWED_WIDTH_PERC * self.width

        # Inițializare grid (felii orizontale)
        self.bottom_row_index = self.height - LaneConfig.BOTTOM_OFFSET
        end = int((1 - self.bottom_perc) * self.height)
        self.step = int(-(self.height * self.bottom_perc / self.slices))
        if self.step == 0: self.step = -1  # Safety

        self.real_slices = int((end - self.bottom_row_index) // self.step)
        self.top_row_index = self.bottom_row_index + self.real_slices * self.step

        self.height_norm = np.linspace(0, 1, self.real_slices + 1)

        # Praguri dinamice
        self.square_pulses_min_height = LaneConfig.SQUARE_PULSES_MIN_HEIGHT
        self.minimum = self.square_pulses_min_height

    def lanes_detection(self, src):
        """Pipeline principal de detecție."""
        # 1. Găsire vârfuri (peaks)
        lanes, peaks, gray = self.peaks_detection(src)

        # 2. Alegere benzi corecte (Stânga vs Dreapta)
        left, right = self.choose_correct_lanes(lanes)

        # 3. Creare polinoame (fit) - AICI ERA PROBLEMA MATEMATICĂ
        left_coef = self.fit_polyfit(left)
        right_coef = self.fit_polyfit(right)

        left_coef, right_coef, trust_l, trust_r = self._validate_and_smooth(
            left_coef, right_coef, left, right
        )

        # 4. Vizualizare Benzi (Desenăm liniile peste imagine)
        if self.print_lanes:
            self.visualize_lane(left_coef, src, (255, 128, 0))  # Cyan/Orange
            self.visualize_lane(right_coef, src, (0, 128, 255))  # Orange/Blue

        # 5. Vizualizare Peaks (Punctele brute)
        if self.print_peaks:
            self.visualize_all_peaks(src, peaks)
            # self.visualize_peaks(src, left, right)

        if self.lk:
            self.update_b_and_top_through_coefs(left_coef, right_coef, trust_l, trust_r)

        # Returnăm dicționarul complet
        return {
            "frame": src,
            "left": left,
            "right": right,
            "left_coef": left_coef,
            "right_coef": right_coef,
            "trust_left": trust_l,
            "trust_right": trust_r,
        }

    def fit_polyfit(self, lane, percentage_for_first_degree=0.3):
        if len(lane) == 0:
            return None

        # x = f(y)
        y_vals = [p[1] for p in lane]
        x_vals = [p[0] for p in lane]

        degree = 2 if len(lane) > self.slices * percentage_for_first_degree else 1

        try:
            coef = np.polyfit(y_vals, x_vals, degree)

            # sanity pe coeficienți (ca în original)
            if degree == 2:
                if abs(coef[0]) > self.extreme_coef_second_deg:
                    return None
            else:
                if abs(coef[0]) > self.extreme_coef_first_deg:
                    return None
                # convertim la grad 2: 0*y^2 + b*y + c
                coef = np.array([0.0, coef[0], coef[1]], dtype=float)

            return coef
        except Exception as e:
            print("[FIT DEBUG] polyfit failed:", e)
            return None




    def _x_at(self, coef, y):
        return coef[0] * y * y + coef[1] * y + coef[2]

    def _validate_and_smooth(self, left_coef, right_coef, left_pts, right_pts):
        yb = self.bottom_row_index

        trust_l = left_coef is not None
        trust_r = right_coef is not None

        width_bad = False

        # 1) sanity lane width
        if trust_l and trust_r:
            lb = self._x_at(left_coef, yb)
            rb = self._x_at(right_coef, yb)
            w = rb - lb

            width_bad = not (
                        self.expected_lane_width_px * self.width_tol_low <= w <= self.expected_lane_width_px * self.width_tol_high)

            # DEBUG
            print(f"[LANE] trustL={trust_l} trustR={trust_r} w={w:.1f} exp={self.expected_lane_width_px:.1f}")

            if width_bad:
                # păstrează banda cu mai multe puncte
                if len(left_pts) >= len(right_pts):
                    trust_r = False
                    right_coef = None
                else:
                    trust_l = False
                    left_coef = None

        # 2) jump check
        if trust_l and self.prev_left_coef is not None:
            prev = self._x_at(self.prev_left_coef, yb)
            cur = self._x_at(left_coef, yb)
            if abs(cur - prev) > self.max_jump_px:
                trust_l = False
                left_coef = None

        if trust_r and self.prev_right_coef is not None:
            prev = self._x_at(self.prev_right_coef, yb)
            cur = self._x_at(right_coef, yb)
            if abs(cur - prev) > self.max_jump_px:
                trust_r = False
                right_coef = None

        # update miss counters
        self.miss_left = 0 if trust_l else self.miss_left + 1
        self.miss_right = 0 if trust_r else self.miss_right + 1

        # 3) fallback DOAR dacă NU e width_bad și doar câteva frame-uri
        if (not width_bad) and (not trust_l) and (self.prev_left_coef is not None) and (
                self.miss_left <= self.max_fallback_frames):
            left_coef = self.prev_left_coef
            trust_l = True

        if (not width_bad) and (not trust_r) and (self.prev_right_coef is not None) and (
                self.miss_right <= self.max_fallback_frames):
            right_coef = self.prev_right_coef
            trust_r = True

        # update memorie
        if left_coef is not None:
            self.prev_left_coef = left_coef
        if right_coef is not None:
            self.prev_right_coef = right_coef

        return left_coef, right_coef, trust_l, trust_r


    def visualize_lane(self, coefs, frame, color):
        """Desenează banda pe imagine folosind ecuația x = ay^2 + by + c"""
        if coefs is None: return

        a, b, c = coefs

        # Desenăm puncte din 5 în 5 pixeli pe înălțime
        # De la josul imaginii până la "orizontul" setat
        start_y = self.height - LaneConfig.BOTTOM_OFFSET
        end_y = int((1 - self.bottom_perc) * self.height)

        for y in range(start_y, end_y, -4):
            # Calculăm X pentru acest Y
            val = a * (y ** 2) + b * y + c
            x = int(val)

            # Verificăm dacă punctul e în imagine
            if 0 <= x < self.width:
                cv2.circle(frame, (x, y), 3, color, -1)

    def peaks_detection(self, frame):
        """Găsește punctele albe pe felii."""
        src = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        peaks = []
        lanes = []
        cnt = 0

        for height in range(self.bottom_row_index, self.top_row_index - 1, self.step):
            # Extragem linia (slice) la înălțimea 'height'
            slice_data = [int(x) for x in src[height]]
            height_norm = self.height_norm[cnt]

            ps = self.find_lane_peaks(
                slice_data, height_norm,
                self.square_pulses_min_height,
                LaneConfig.SQUARE_PULSES_MIN_HEIGHT_DIF,
                LaneConfig.SQUARE_PULSES_PIX_DIF,
                LaneConfig.SQUARE_PULSES_ALLOWED_PEAKS_WIDTH_ERROR
            )
            cnt += 1

            for point in ps:
                peaks.append([point, height])

            lanes = self.peaks_clustering(ps, height, lanes)

        return lanes, peaks, src

    def find_lane_peaks(self, slice_data, height_norm, min_height, min_height_dif, pix_dif, allowed_width_error):
        """Algoritmul 'Square Pulse' pentru a găsi linii albe pe o linie orizontală."""
        peaks_max_width = self.peaks_max_width - ((self.peaks_max_width - self.peaks_min_width) * height_norm)
        upper_limit = peaks_max_width + allowed_width_error

        inside_a_peak = False
        peaks = []
        pix_num = 0

        # Iterăm prin pixelii liniei
        for i in range(pix_dif, len(slice_data) - pix_dif):
            pixel = slice_data[i]

            # Derivata (diferența) pentru a detecta margini
            # height_dif_start = int(pixel) - int(slice_data[i - pix_dif])
            height_dif_end = int(pixel) - int(slice_data[i + pix_dif])

            if inside_a_peak:
                # Căutăm marginea din dreapta (falling edge)
                if height_dif_end > min_height_dif:
                    inside_a_peak = False
                    # Validăm lățimea
                    if pix_num >= self.peaks_min_width and pix_num <= upper_limit:
                        peak = i - (pix_num - pix_dif) // 2
                        peaks.append(peak)
                    pix_num = 0
                else:
                    if pixel > min_height:
                        pix_num += 1
                    else:
                        inside_a_peak = False  # Reset dacă scade sub threshold brusc
            else:
                # Căutăm marginea din stânga (rising edge)
                height_dif_start = int(pixel) - int(slice_data[i - pix_dif])
                if pixel > min_height and height_dif_start > min_height_dif:
                    inside_a_peak = True
                    pix_num += 1

        return peaks

    def peaks_clustering(self, points, height, lanes):
        """Grupează punctele găsite în benzi coerente."""
        if not lanes:
            return [[[x, height]] for x in points]

        lane_length = len(lanes)
        lanes_dict = [{"point_index": -1, "distance": -1} for _ in range(lane_length)]
        points_dict = [{"used": False, "lane_index": -1} for _ in range(len(points))]

        # Găsește cel mai apropiat punct pentru fiecare bandă
        self.find_best_qualified_points(lanes_dict, points_dict, points, lanes, height)

        # Adăugare puncte la benzi existente
        cnt = 0
        appended_cnt = 0
        for point in points_dict:
            if point["used"]:
                lanes[point["lane_index"]].append([points[cnt - appended_cnt], height])
                points = np.delete(points, cnt - appended_cnt)
                appended_cnt += 1
            cnt += 1

        # Creare benzi noi pentru punctele rămase (care nu s-au potrivit nicăieri)
        lanes_inserted = 0
        for point in points:
            if point < lanes[lanes_inserted][-1][0]:
                lanes.insert(lanes_inserted, [[point, height]])
                lanes_inserted += 1
            elif point > lanes[lane_length + lanes_inserted - 1][-1][0]:
                lanes.append([[point, height]])
                lanes_inserted += 1
            else:
                # Simplificare: adaugă la final dacă e la mijloc (sau implementare complexă insert)
                lanes.append([[point, height]])

        return lanes

    def find_best_qualified_points(self, lanes_dict, points_dict, detected_points, lanes, height):
        # Caută cel mai apropiat punct de pe rândul curent față de ultimul punct al benzii
        for lane_idx, lane in enumerate(lanes):
            x1, y1 = lane[-1]
            min_dist = self.max_allowed_dist
            best_p_idx = -1

            for p_idx, x0 in enumerate(detected_points):
                y0 = height
                # Distanța orizontală ajustată cu panta
                width_dist = abs((x0 - x1) * self.step / (y0 - y1))
                if width_dist < min_dist:
                    min_dist = width_dist
                    best_p_idx = p_idx

            if best_p_idx != -1 and not points_dict[best_p_idx]["used"]:
                lanes_dict[lane_idx]["point_index"] = best_p_idx
                points_dict[best_p_idx]["used"] = True
                points_dict[best_p_idx]["lane_index"] = lane_idx

    def choose_correct_lanes(self, lanes):
        # păstrează doar lane-uri suficient de lungi
        lanes = [lane for lane in lanes if len(lane) >= self.min_peaks_for_lane]
        if not lanes:
            return [], []

        center = self.width / 2.0
        expected_w = self.expected_lane_width_px

        left_candidates = []
        right_candidates = []

        for lane in lanes:
            base_x = lane[0][0]  # punctul cel mai de jos
            if base_x <= center:
                left_candidates.append(lane)
            else:
                right_candidates.append(lane)

        # dacă nu am candidați pe una din părți, fallback simplu
        if not left_candidates and not right_candidates:
            return [], []
        if not left_candidates:
            # ia right cel mai aproape de poziția așteptată
            right = min(right_candidates, key=lambda ln: abs(ln[0][0] - (center + expected_w / 2)))
            return [], right
        if not right_candidates:
            left = min(left_candidates, key=lambda ln: abs(ln[0][0] - (center - expected_w / 2)))
            return left, []

        # alege perechea (L,R) cu width cel mai aproape de expected
        best_L, best_R = None, None
        best_score = 1e18

        for L in left_candidates:
            xL = L[0][0]
            for R in right_candidates:
                xR = R[0][0]
                w = xR - xL
                if w <= 0:
                    continue

                score = abs(w - expected_w) + 0.2 * abs((xL + xR) / 2 - center)
                if score < best_score:
                    best_score = score
                    best_L, best_R = L, R

        return best_L if best_L is not None else [], best_R if best_R is not None else []


    def visualize_all_peaks(self, frame, peaks):
        for peak in peaks:
            cv2.circle(frame, (peak[0], peak[1]), 2, (255, 0, 255), 2)

    def visualize_peaks(self, frame, left, right):
        for p in left: cv2.circle(frame, (p[0], p[1]), 3, (0, 0, 0), -1)
        for p in right: cv2.circle(frame, (p[0], p[1]), 3, (0, 0, 0), -1)

    def update_b_and_top_through_coefs(self, left_coef, right_coef, trust_l, trust_r):
        """Actualizează lățimea estimată a benzii în LaneKeeping pe baza detecției reale."""
        if left_coef is None or right_coef is None or not trust_l or not trust_r:
            return

        bot_h = self.bottom_row_index
        top_h = self.top_row_index

        # Calculăm lățimea jos și sus folosind coeficienții corecți (x = f(y))
        l_b = left_coef[0] * bot_h ** 2 + left_coef[1] * bot_h + left_coef[2]
        r_b = right_coef[0] * bot_h ** 2 + right_coef[1] * bot_h + right_coef[2]

        l_t = left_coef[0] * top_h ** 2 + left_coef[1] * top_h + left_coef[2]
        r_t = right_coef[0] * top_h ** 2 + right_coef[1] * top_h + right_coef[2]

        # Actualizăm config-ul din mers
        if self.lk:
            self.lk.bottom_width = int((r_b - l_b) // 2)
            self.lk.top_width = int((r_t - l_t) // 2)