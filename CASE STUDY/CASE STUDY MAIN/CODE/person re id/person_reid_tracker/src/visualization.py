"""Visualization helpers for live display."""

from __future__ import annotations

from typing import List, Optional

import cv2
import numpy as np

from src.identity_manager import IdentityLabel, IdentityResult

# COCO pose skeleton connections
POSE_SKELETON = [
    (0, 1), (0, 2), (1, 3), (2, 4),
    (5, 6), (5, 7), (7, 9), (6, 8), (8, 10),
    (5, 11), (6, 12), (11, 12),
    (11, 13), (13, 15), (12, 14), (14, 16),
]


def _color_for_label(label: IdentityLabel, config: dict) -> tuple:
    if label == IdentityLabel.OWNER:
        return tuple(config.get("bbox_color_owner", [0, 255, 0]))
    return tuple(config.get("bbox_color_unknown", [0, 165, 255]))


def draw_tracks(
    frame: np.ndarray,
    results: List[IdentityResult],
    config: dict,
    fps: float = 0.0,
) -> np.ndarray:
    """Draw bboxes, labels, pose, and HUD on a copy of the frame."""
    vis = frame.copy()
    show_pose = config.get("show_pose", True)
    show_track_id = config.get("show_track_id", True)
    show_fps = config.get("show_fps", True)

    for result in results:
        x1, y1, x2, y2 = result.bbox.astype(int)
        color = _color_for_label(result.label, config)

        cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)

        parts = [result.label.value]
        if show_track_id:
            parts.append(f"ID:{result.track_id}")
        parts.append(f"{result.confidence:.2f}")
        if result.face_detected:
            parts.append(f"F:{result.face_score:.2f}")
        else:
            parts.append(f"B:{result.body_score:.2f}")

        label = " | ".join(parts)
        _draw_label(vis, label, (x1, max(y1 - 8, 0)), color)

        if show_pose and result.keypoints is not None:
            _draw_pose(vis, result.keypoints)

    if show_fps:
        cv2.putText(
            vis,
            f"FPS: {fps:.1f}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

    _draw_help(vis)
    return vis


def _draw_label(frame: np.ndarray, text: str, origin: tuple, color: tuple) -> None:
    x, y = origin
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.5
    thickness = 1
    (tw, th), _ = cv2.getTextSize(text, font, scale, thickness)
    y = y if y - th - 4 > 0 else y + th + 12
    cv2.rectangle(frame, (x, y - th - 4), (x + tw + 4, y + 2), (0, 0, 0), -1)
    cv2.putText(frame, text, (x + 2, y), font, scale, color, thickness, cv2.LINE_AA)


def _draw_pose(frame: np.ndarray, keypoints: np.ndarray, conf_thresh: float = 0.3) -> None:
    kpts = np.asarray(keypoints)
    if kpts.ndim != 2 or kpts.shape[1] < 3:
        return

    for i, j in POSE_SKELETON:
        if i >= len(kpts) or j >= len(kpts):
            continue
        if kpts[i, 2] < conf_thresh or kpts[j, 2] < conf_thresh:
            continue
        pt1 = (int(kpts[i, 0]), int(kpts[i, 1]))
        pt2 = (int(kpts[j, 0]), int(kpts[j, 1]))
        cv2.line(frame, pt1, pt2, (255, 200, 0), 2)

    for kpt in kpts:
        if kpt[2] < conf_thresh:
            continue
        cv2.circle(frame, (int(kpt[0]), int(kpt[1])), 3, (0, 200, 255), -1)


def _draw_help(frame: np.ndarray) -> None:
    help_text = "q: quit | r: reset tracker | s: save frame"
    cv2.putText(
        frame,
        help_text,
        (10, frame.shape[0] - 12),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (200, 200, 200),
        1,
        cv2.LINE_AA,
    )


def draw_enrollment_overlay(
    frame: np.ndarray,
    message: str,
    progress: float,
    count: int,
    target: int,
) -> np.ndarray:
    vis = frame.copy()
    h, w = vis.shape[:2]
    bar_w = int((w - 40) * min(progress, 1.0))
    cv2.rectangle(vis, (20, h - 40), (w - 20, h - 20), (60, 60, 60), -1)
    cv2.rectangle(vis, (20, h - 40), (20 + bar_w, h - 20), (0, 200, 0), -1)
    cv2.putText(
        vis,
        f"{message}  [{count}/{target}]",
        (20, h - 50),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return vis
