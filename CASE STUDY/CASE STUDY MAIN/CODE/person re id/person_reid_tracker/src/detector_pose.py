"""YOLO pose detection wrapper – persons only with keypoints."""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class PersonDetection:
    """Single person detection with optional pose keypoints."""

    bbox: np.ndarray  # xyxy
    confidence: float
    keypoints: Optional[np.ndarray] = None  # shape (17, 3) – x, y, conf
    class_id: int = 0


@dataclass
class DetectionResult:
    detections: List[PersonDetection] = field(default_factory=list)
    raw_result: object = None


class PoseDetector:
    """Ultralytics YOLO pose model for person detection."""

    def __init__(
        self,
        model_name: str = "yolov8n-pose.pt",
        confidence: float = 0.5,
        person_class_id: int = 0,
        device: str = "cpu",
        use_pose: bool = True,
    ):
        self.confidence = confidence
        self.person_class_id = person_class_id
        self.device = device
        self.use_pose = use_pose
        self.model = None
        self.model_name = model_name
        self._load_model()

    def _load_model(self) -> None:
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise ImportError(
                "ultralytics is required for detection. Install with: pip install ultralytics"
            ) from exc

        logger.info(
            "Loading YOLO model: %s | selected device: %s | pose enabled: %s",
            self.model_name,
            self.device,
            self.use_pose,
        )
        self.model = YOLO(self.model_name)
        self._device = self.device

    def detect(self, frame: np.ndarray) -> DetectionResult:
        if self.model is None:
            raise RuntimeError("Model not loaded.")

        results = self.model.predict(
            source=frame,
            conf=self.confidence,
            classes=[self.person_class_id],
            verbose=False,
            device=self._device,
        )

        detections: List[PersonDetection] = []
        if not results:
            return DetectionResult(detections=detections)

        result = results[0]
        if result.boxes is None or len(result.boxes) == 0:
            return DetectionResult(detections=detections, raw_result=result)

        boxes = result.boxes.xyxy.cpu().numpy()
        confs = result.boxes.conf.cpu().numpy()
        cls_ids = result.boxes.cls.cpu().numpy().astype(int)

        keypoints_all = None
        if self.use_pose and getattr(result, "keypoints", None) is not None:
            keypoints_all = result.keypoints.data.cpu().numpy()

        for i in range(len(boxes)):
            kpts = keypoints_all[i] if keypoints_all is not None and i < len(keypoints_all) else None
            detections.append(
                PersonDetection(
                    bbox=boxes[i],
                    confidence=float(confs[i]),
                    keypoints=kpts,
                    class_id=int(cls_ids[i]),
                )
            )

        return DetectionResult(detections=detections, raw_result=result)
