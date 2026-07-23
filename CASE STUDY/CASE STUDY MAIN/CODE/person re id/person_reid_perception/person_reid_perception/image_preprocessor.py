"""Sensor noise filter for the RealSense RGB stream.

Light preprocessing pipeline applied to each incoming frame before YOLO and
the ReID models. Every stage is parameter-toggled, so the same node can run
on the real Waffle Pi camera, a Gazebo synthetic feed, or a desk test
camera.

Pipeline (each step is optional):

1. Resize to a fixed inference size (matches person_reid_tracker default).
2. Edge-preserving bilateral filter to remove RealSense sensor noise.
3. LAB-CLAHE on the L channel to normalize lighting.
"""

from __future__ import annotations

import logging
from typing import Optional, Tuple

import cv2
import numpy as np

logger = logging.getLogger(__name__)

# Encodings the preprocessor knows how to convert to BGR8.
_KNOWN_ENCODINGS = {'bgr8', 'rgb8', 'mono8', 'yuv422_yuy2', 'yuyv'}


class ImagePreprocessor:
    """Configurable noise filter for ROS Image messages."""

    def __init__(
        self,
        target_width: int = 416,
        target_height: int = 416,
        enable_resize: bool = True,
        enable_bilateral: bool = True,
        bilateral_d: int = 5,
        bilateral_sigma_color: float = 50.0,
        bilateral_sigma_space: float = 50.0,
        enable_clahe: bool = True,
        clahe_clip: float = 2.0,
        clahe_tile: int = 8,
    ) -> None:
        self.target_width = int(target_width)
        self.target_height = int(target_height)
        self.enable_resize = bool(enable_resize)
        self.enable_bilateral = bool(enable_bilateral)
        self.bilateral_d = int(bilateral_d)
        self.bilateral_sigma_color = float(bilateral_sigma_color)
        self.bilateral_sigma_space = float(bilateral_sigma_space)
        self.enable_clahe = bool(enable_clahe)
        self.clahe_clip = float(clahe_clip)
        self.clahe_tile = int(clahe_tile)

        self._clahe: Optional[cv2.CLAHE] = None
        if self.enable_clahe:
            self._clahe = cv2.createCLAHE(
                clipLimit=self.clahe_clip,
                tileGridSize=(self.clahe_tile, self.clahe_tile),
            )

    def preprocess(self, img_bgr: np.ndarray) -> np.ndarray:
        """Apply the filter chain to a BGR ndarray."""
        if img_bgr is None or img_bgr.size == 0:
            return img_bgr

        out = img_bgr
        if self.enable_resize:
            out = cv2.resize(
                out,
                (self.target_width, self.target_height),
                interpolation=cv2.INTER_AREA,
            )
        if self.enable_bilateral and self.bilateral_d > 0:
            out = cv2.bilateralFilter(
                out,
                self.bilateral_d,
                self.bilateral_sigma_color,
                self.bilateral_sigma_space,
            )
        if self.enable_clahe and self._clahe is not None and out.ndim == 3:
            lab = cv2.cvtColor(out, cv2.COLOR_BGR2LAB)
            lab[:, :, 0] = self._clahe.apply(lab[:, :, 0])
            out = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
        return out

    def preprocess_msg(
        self,
        msg,
        bridge,
    ) -> Tuple[Optional[np.ndarray], object]:
        """Convert a sensor_msgs/Image to a preprocessed BGR ndarray.

        Returns ``(frame, header)``. ``frame`` is ``None`` when the encoding
        is not supported — the caller should skip the frame.
        """
        encoding = (getattr(msg, 'encoding', '') or '').lower()
        if encoding not in _KNOWN_ENCODINGS:
            logger.warning(
                "Unsupported image encoding %r; skipping frame.", encoding,
            )
            return None, msg.header

        if encoding in ('yuv422_yuy2', 'yuyv'):
            raw = bridge.imgmsg_to_cv2(msg, desired_encoding='yuv422_yuy2')
            bgr = cv2.cvtColor(raw, cv2.COLOR_YUV2BGR_YUY2)
        elif encoding == 'rgb8':
            rgb = bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        elif encoding == 'mono8':
            gray = bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        else:  # bgr8
            bgr = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if bgr is None or bgr.size == 0:
            return None, msg.header

        return self.preprocess(bgr), msg.header
