import time
from pathlib import Path
import numpy as np
import cv2


class TrafficSignDetectorYOLOv8:
    """
    YOLOv8 detector wrapper for BFMC.
    - Uses Ultralytics YOLO on CPU (Pi 5)
    - Returns detections using CLASS NAMES (not IDs)
    - API: detect(frame_bgr) -> list[dict]
    """

    def __init__(self, model_path: str, conf: float = 0.40, imgsz: int = 416, iou: float = 0.45, max_det: int = 20):
        self.model_path = str(model_path)
        self.conf = float(conf)
        self.imgsz = int(imgsz)
        self.iou = float(iou)
        self.max_det = int(max_det)

        self.enabled = True
        self.model = None
        self.names = {}

        try:
            from ultralytics import YOLO
        except Exception as e:
            print(f"[TrafficSigns] Ultralytics import failed -> disabled. Err: {e}")
            self.enabled = False
            return

        p = Path(self.model_path)
        if not p.exists():
            print(f"[TrafficSigns] Model not found: {p} -> disabled.")
            self.enabled = False
            return

        try:
            self.model = YOLO(self.model_path)
            self.names = dict(getattr(self.model, "names", {})) or {}
            if not self.names:
                print("[TrafficSigns] WARNING: model.names empty; will fallback to class_{id}.")
        except Exception as e:
            print(f"[TrafficSigns] Model load failed -> disabled. Err: {e}")
            self.enabled = False
            self.model = None

    def detect(self, frame_bgr: np.ndarray):
        """
        Returns list of dicts:
          [{"name": str, "conf": float, "bbox": [x1,y1,x2,y2], "cls": int}, ...]
        """
        if not self.enabled or self.model is None:
            return []

        # Ultralytics expects RGB
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

        try:
            results = self.model.predict(
                source=frame_rgb,
                imgsz=self.imgsz,
                conf=self.conf,
                iou=self.iou,
                max_det=self.max_det,
                device="cpu",
                verbose=False,
            )
        except Exception as e:
            print(f"[TrafficSigns] predict() failed -> empty. Err: {e}")
            return []

        r0 = results[0]
        boxes = getattr(r0, "boxes", None)
        if boxes is None or len(boxes) == 0:
            return []

        xyxy = boxes.xyxy.cpu().numpy()
        confs = boxes.conf.cpu().numpy()
        clss = boxes.cls.cpu().numpy().astype(int)

        dets = []
        for (x1, y1, x2, y2), c, cls_id in zip(xyxy, confs, clss):
            name = self.names.get(int(cls_id), f"class_{int(cls_id)}")
            dets.append(
                {
                    "name": str(name),
                    "conf": float(c),
                    "bbox": [int(x1), int(y1), int(x2), int(y2)],
                    "cls": int(cls_id),  # păstrăm și cls pt debug, dar logica o facem pe "name"
                }
            )

        return dets

