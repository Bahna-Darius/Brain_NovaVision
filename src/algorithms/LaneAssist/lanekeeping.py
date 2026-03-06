import numpy as np
import math
import cv2
from .lane_config import LaneConfig


class LaneKeeping:
    def __init__(self):
        self.width = LaneConfig.WIDTH
        self.height = LaneConfig.HEIGHT

        self.angle = 0.0
        self.last_angle = 0.0
        self.last_filtered_angle = 0.0

        self.steer_value_list = []
        self.median_constant = LaneConfig.MEDIAN_CONSTANT
        self.print_desire_lane = LaneConfig.PRINT_DESIRE_LANE

        self.max_lk_steer = LaneConfig.MAX_STEER

        # Parametri
        self.bottom_width = LaneConfig.BOTTOM_WIDTH
        self.top_width = LaneConfig.TOP_WIDTH

        # Configurare grid
        self.bottom_offset = LaneConfig.BOTTOM_OFFSET
        self.slices = LaneConfig.SLICES
        bottom_perc = LaneConfig.BOTTOM_PERC

        self.bottom_row_index = self.height - self.bottom_offset
        end = int((1 - bottom_perc) * self.height)
        step = int(-(self.height * bottom_perc / self.slices))
        if step == 0: step = -1

        self.real_slices = int((end - self.bottom_row_index) // step)
        self.top_row_index = self.bottom_row_index + self.real_slices * step

        # Array-uri NumPy simple
        self.plot_y = np.linspace(self.bottom_row_index, self.top_row_index, self.real_slices)

        # Ponderi (Weights) Liniare - 100% sigur
        self.get_error_weights = np.linspace(1.0, 0.5, self.real_slices)

    def lane_keeping(self, lanes_detection):
        if lanes_detection is None:
            return self.angle, None, None

        frame = lanes_detection["frame"]
        left_coef = lanes_detection["left_coef"]
        right_coef = lanes_detection["right_coef"]

        trust_l = lanes_detection["trust_left"]
        trust_r = lanes_detection["trust_right"]

        # Validare input (defensivă)
        left = left_coef if trust_l else None
        right = right_coef if trust_r else None

        preview = None

        try:
            self.angle, error, frame, preview = self.pid_controller(left, right, frame)
            self.fix_angle()
            self.angle = max(min(self.max_lk_steer, self.angle), -self.max_lk_steer)
        except Exception as e:
            print(f"[LaneKeeping] Error in calculation: {e}")
            self.angle = 0.0
            preview = None

        return self.angle, frame, preview

    def pid_controller(self, left, right, frame):
        if left is None and right is None:
            angle = self.last_angle
            error = 0
            preview = None
        else:
            desired_lane = self.desired_lane(left, right)

            if self.print_desire_lane:
                self.visualize_desire_lane(frame, desired_lane)

            error = self.get_error(desired_lane)
            preview = self.get_preview_data(desired_lane)

            # Calcul unghi simplificat și sigur
            look_ahead_dist = self.height * 0.5
            if look_ahead_dist == 0: look_ahead_dist = 1  # Evitare div by 0

            angle = math.degrees(math.atan2(error, look_ahead_dist))
            angle = angle * 1.2  # Gain

        self.last_angle = angle
        return angle, error, frame, preview

    def desired_lane(self, left_fit, right_fit):
        desire_lane = np.zeros_like(self.plot_y)
        width_diff = np.linspace(self.bottom_width, self.top_width, self.real_slices)

        # Calcule safe (verificăm dacă există coeficienți)
        if left_fit is not None:
            l_vals = left_fit[0] * self.plot_y ** 2 + left_fit[1] * self.plot_y + left_fit[2]
        if right_fit is not None:
            r_vals = right_fit[0] * self.plot_y ** 2 + right_fit[1] * self.plot_y + right_fit[2]

        if left_fit is not None and right_fit is not None:
            desire_lane = (l_vals + r_vals) / 2
        elif left_fit is None and right_fit is not None:
            desire_lane = r_vals - (width_diff * 2)
        elif right_fit is None and left_fit is not None:
            desire_lane = l_vals + (width_diff * 2)

        return desire_lane

    def get_preview_data(self, desired):
        if desired is None or len(desired) < 6:
            return None

        try:
            n = len(desired) - 1

            # Preview points from near car to farther ahead.
            idx_near = max(0, min(n, int(0.15 * n)))
            idx_mid = max(0, min(n, int(0.45 * n)))
            idx_far = max(0, min(n, int(0.75 * n)))

            x_near = float(desired[idx_near])
            x_mid = float(desired[idx_mid])
            x_far = float(desired[idx_far])

            y_near = float(self.plot_y[idx_near])
            y_mid = float(self.plot_y[idx_mid])
            y_far = float(self.plot_y[idx_far])

            center = self.width / 2.0

            near_error = x_near - center
            mid_error = x_mid - center
            far_error = x_far - center

            dy_near = abs(y_near - y_mid)
            dy_far = abs(y_mid - y_far)

            if dy_near < 1.0:
                dy_near = 1.0
            if dy_far < 1.0:
                dy_far = 1.0

            heading_near_deg = math.degrees(math.atan2(x_mid - x_near, dy_near))
            heading_far_deg = math.degrees(math.atan2(x_far - x_mid, dy_far))
            heading_delta_deg = heading_far_deg - heading_near_deg

            curvature_score = abs(far_error - near_error) + (2.0 * abs(heading_delta_deg))

            if far_error > near_error + 5.0:
                turn_direction = "RIGHT"
            elif far_error < near_error - 5.0:
                turn_direction = "LEFT"
            else:
                turn_direction = "STRAIGHT"

            return {
                "near_error_px": float(near_error),
                "mid_error_px": float(mid_error),
                "far_error_px": float(far_error),
                "heading_near_deg": float(heading_near_deg),
                "heading_far_deg": float(heading_far_deg),
                "heading_delta_deg": float(heading_delta_deg),
                "curvature_score": float(curvature_score),
                "turn_direction": str(turn_direction),
            }
        except Exception:
            return None

    def get_error(self, desired):
        if len(desired) > 0:
            weighted_mean = np.average(desired, weights=self.get_error_weights)
            center = self.width / 2.0
            bias_px = LaneConfig.CENTER_BIAS_PX  # de ex +5 sau -5
            error = (weighted_mean - (center + bias_px))

            return error
        return 0

    def visualize_desire_lane(self, frame, plot_x, color=(0, 255, 0)):
        if frame is not None and len(plot_x) == len(self.plot_y):
            for i in range(len(self.plot_y) - 1):
                try:
                    pt1 = (int(plot_x[i]), int(self.plot_y[i]))
                    pt2 = (int(plot_x[i + 1]), int(self.plot_y[i + 1]))
                    cv2.line(frame, pt1, pt2, color, 3)
                except:
                    pass

    def fix_angle(self):
        self.steer_value_list.insert(0, self.angle)
        if len(self.steer_value_list) > self.median_constant:
            self.steer_value_list.pop()

        if len(self.steer_value_list) > 0:
            filtered = float(np.median(self.steer_value_list))

            max_delta = 2.5
            delta = filtered - self.last_filtered_angle
            if delta > max_delta:
                filtered = self.last_filtered_angle + max_delta
            elif delta < -max_delta:
                filtered = self.last_filtered_angle - max_delta

            self.angle = filtered
            self.last_filtered_angle = self.angle