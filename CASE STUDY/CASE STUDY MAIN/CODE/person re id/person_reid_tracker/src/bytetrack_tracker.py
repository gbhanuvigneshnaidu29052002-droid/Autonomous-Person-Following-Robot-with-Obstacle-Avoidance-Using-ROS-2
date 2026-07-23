"""ByteTrack multi-object tracker - isolated implementation."""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np
from scipy.optimize import linear_sum_assignment

logger = logging.getLogger(__name__)


def _iou_batch(bboxes_a: np.ndarray, bboxes_b: np.ndarray) -> np.ndarray:
    """Pairwise IoU between two sets of xyxy boxes."""
    if len(bboxes_a) == 0 or len(bboxes_b) == 0:
        return np.zeros((len(bboxes_a), len(bboxes_b)), dtype=np.float32)

    bboxes_a = bboxes_a.astype(np.float32)
    bboxes_b = bboxes_b.astype(np.float32)

    area_a = np.maximum(0.0, bboxes_a[:, 2] - bboxes_a[:, 0]) * np.maximum(
        0.0, bboxes_a[:, 3] - bboxes_a[:, 1]
    )
    area_b = np.maximum(0.0, bboxes_b[:, 2] - bboxes_b[:, 0]) * np.maximum(
        0.0, bboxes_b[:, 3] - bboxes_b[:, 1]
    )

    inter_x1 = np.maximum(bboxes_a[:, None, 0], bboxes_b[None, :, 0])
    inter_y1 = np.maximum(bboxes_a[:, None, 1], bboxes_b[None, :, 1])
    inter_x2 = np.minimum(bboxes_a[:, None, 2], bboxes_b[None, :, 2])
    inter_y2 = np.minimum(bboxes_a[:, None, 3], bboxes_b[None, :, 3])

    inter_w = np.maximum(0.0, inter_x2 - inter_x1)
    inter_h = np.maximum(0.0, inter_y2 - inter_y1)
    inter_area = inter_w * inter_h

    union = area_a[:, None] + area_b[None, :] - inter_area
    return inter_area / np.maximum(union, 1e-6)


def _linear_assignment(
    cost: np.ndarray,
    thresh: float,
) -> Tuple[List[Tuple[int, int]], List[int], List[int]]:
    """Hungarian assignment with distance threshold."""
    if cost.size == 0:
        n, m = cost.shape if cost.ndim == 2 else (0, 0)
        return [], list(range(n)), list(range(m))

    row_ind, col_ind = linear_sum_assignment(cost)

    matches: List[Tuple[int, int]] = []
    unmatched_a = set(range(cost.shape[0]))
    unmatched_b = set(range(cost.shape[1]))

    for r, c in zip(row_ind, col_ind):
        if cost[r, c] > thresh:
            continue

        matches.append((int(r), int(c)))
        unmatched_a.discard(int(r))
        unmatched_b.discard(int(c))

    return matches, list(unmatched_a), list(unmatched_b)


class _KalmanFilter:
    """
    Minimal constant-velocity Kalman filter for xyah bbox state.

    State:
        [cx, cy, aspect_ratio, height, vx, vy, va, vh]

    Measurement:
        [cx, cy, aspect_ratio, height]
    """

    def __init__(self):
        self._std_weight_position = 1.0 / 20.0
        self._std_weight_velocity = 1.0 / 160.0

        ndim = 4
        dt = 1.0

        self._motion_mat = np.eye(2 * ndim, dtype=np.float64)
        for i in range(ndim):
            self._motion_mat[i, ndim + i] = dt

        self._update_mat = np.eye(ndim, 2 * ndim, dtype=np.float64)

    @staticmethod
    def xyxy_to_xyah(bbox: np.ndarray) -> np.ndarray:
        """Convert xyxy bbox to cx, cy, aspect ratio, height."""
        x1, y1, x2, y2 = bbox

        w = max(float(x2 - x1), 1e-6)
        h = max(float(y2 - y1), 1e-6)
        cx = float(x1) + w / 2.0
        cy = float(y1) + h / 2.0
        aspect_ratio = w / h

        return np.array([cx, cy, aspect_ratio, h], dtype=np.float64)

    @staticmethod
    def xyah_to_xyxy(xyah: np.ndarray) -> np.ndarray:
        """Convert cx, cy, aspect ratio, height to xyxy bbox."""
        cx, cy, aspect_ratio, h = xyah

        h = max(float(h), 1e-6)
        w = float(aspect_ratio) * h

        return np.array(
            [
                cx - w / 2.0,
                cy - h / 2.0,
                cx + w / 2.0,
                cy + h / 2.0,
            ],
            dtype=np.float64,
        )

    def initiate(self, measurement: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Create a new track from an unassociated measurement."""
        measurement = np.asarray(measurement, dtype=np.float64)

        mean = np.r_[measurement, np.zeros(4, dtype=np.float64)]

        std = [
            2.0 * self._std_weight_position * measurement[3],
            2.0 * self._std_weight_position * measurement[3],
            1e-2,
            2.0 * self._std_weight_position * measurement[3],
            10.0 * self._std_weight_velocity * measurement[3],
            10.0 * self._std_weight_velocity * measurement[3],
            1e-5,
            10.0 * self._std_weight_velocity * measurement[3],
        ]

        covariance = np.diag(np.square(std)).astype(np.float64)
        return mean, covariance

    def predict(self, mean: np.ndarray, covariance: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Run Kalman prediction step."""
        mean = np.asarray(mean, dtype=np.float64)
        covariance = np.asarray(covariance, dtype=np.float64)

        h = max(float(mean[3]), 1e-6)

        std_pos = [
            self._std_weight_position * h,
            self._std_weight_position * h,
            1e-2,
            self._std_weight_position * h,
        ]

        std_vel = [
            self._std_weight_velocity * h,
            self._std_weight_velocity * h,
            1e-5,
            self._std_weight_velocity * h,
        ]

        motion_cov = np.diag(np.square(np.r_[std_pos, std_vel])).astype(np.float64)

        mean = self._motion_mat @ mean
        covariance = self._motion_mat @ covariance @ self._motion_mat.T + motion_cov

        return mean, covariance

    def project(self, mean: np.ndarray, covariance: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Project 8D state distribution to 4D measurement space."""
        mean = np.asarray(mean, dtype=np.float64)
        covariance = np.asarray(covariance, dtype=np.float64)

        h = max(float(mean[3]), 1e-6)

        std = [
            self._std_weight_position * h,
            self._std_weight_position * h,
            1e-1,
            self._std_weight_position * h,
        ]

        innovation_cov = np.diag(np.square(std)).astype(np.float64)

        projected_mean = self._update_mat @ mean
        projected_covariance = self._update_mat @ covariance @ self._update_mat.T + innovation_cov

        return projected_mean, projected_covariance

    def update(
        self,
        mean: np.ndarray,
        covariance: np.ndarray,
        measurement: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Run Kalman correction step.

        Important shapes:
            mean: 8
            covariance: 8x8
            measurement: 4
            update matrix H: 4x8
            Kalman gain: 8x4
            new covariance: 8x8
        """
        mean = np.asarray(mean, dtype=np.float64)
        covariance = np.asarray(covariance, dtype=np.float64)
        measurement = np.asarray(measurement, dtype=np.float64)

        projected_mean, projected_cov = self.project(mean, covariance)
        innovation = measurement - projected_mean

        h_matrix = self._update_mat

        try:
            kalman_gain = covariance @ h_matrix.T @ np.linalg.inv(projected_cov)
        except np.linalg.LinAlgError:
            logger.warning("Projected covariance singular. Using pseudo-inverse.")
            kalman_gain = covariance @ h_matrix.T @ np.linalg.pinv(projected_cov)

        new_mean = mean + kalman_gain @ innovation
        new_covariance = covariance - kalman_gain @ projected_cov @ kalman_gain.T

        if new_mean.shape != (8,):
            raise ValueError(f"Invalid Kalman mean shape: {new_mean.shape}")

        if new_covariance.shape != (8, 8):
            raise ValueError(f"Invalid Kalman covariance shape: {new_covariance.shape}")

        return new_mean, new_covariance


class STrack:
    """Single track state."""

    _count = 0
    shared_kalman = _KalmanFilter()

    def __init__(self, bbox_xyxy: np.ndarray, score: float):
        self.track_id = 0
        self.score = float(score)
        self.is_activated = False
        self.state = "New"
        self.frame_id = 0
        self.start_frame = 0
        self.tracklet_len = 0
        self.time_since_update = 0

        xyah = self.shared_kalman.xyxy_to_xyah(bbox_xyxy)
        self.mean, self.covariance = self.shared_kalman.initiate(xyah)

    @classmethod
    def next_id(cls) -> int:
        cls._count += 1
        return cls._count

    def predict(self) -> None:
        self.mean, self.covariance = self.shared_kalman.predict(
            self.mean,
            self.covariance,
        )

    def update(self, bbox_xyxy: np.ndarray, score: float) -> None:
        xyah = self.shared_kalman.xyxy_to_xyah(bbox_xyxy)
        self.mean, self.covariance = self.shared_kalman.update(
            self.mean,
            self.covariance,
            xyah,
        )

        self.score = float(score)
        self.tracklet_len += 1
        self.time_since_update = 0

    def activate(self, frame_id: int) -> None:
        self.track_id = self.next_id()
        self.is_activated = True
        self.state = "Tracked"
        self.frame_id = frame_id
        self.start_frame = frame_id
        self.time_since_update = 0

    def re_activate(self, bbox_xyxy: np.ndarray, score: float, frame_id: int) -> None:
        self.update(bbox_xyxy, score)
        self.state = "Tracked"
        self.is_activated = True
        self.frame_id = frame_id
        self.time_since_update = 0

    @property
    def bbox_xyxy(self) -> np.ndarray:
        return self.shared_kalman.xyah_to_xyxy(self.mean[:4])


@dataclass
class TrackOutput:
    """Output of ByteTrack for one person."""

    track_id: int
    bbox: np.ndarray
    score: float
    keypoints: Optional[np.ndarray] = None
    detection_index: int = -1


@dataclass
class ByteTrackConfig:
    track_high_thresh: float = 0.5
    track_low_thresh: float = 0.1
    new_track_thresh: float = 0.6
    track_buffer: int = 30
    match_thresh: float = 0.8
    frame_rate: int = 30


class ByteTrackTracker:
    """
    ByteTrack multi-object tracker.

    Accepts person detections as xyxy boxes and confidence scores.
    Returns stable track IDs.
    """

    def __init__(self, config: Optional[ByteTrackConfig] = None):
        self.config = config or ByteTrackConfig()
        self.frame_id = 0
        self.max_time_lost = int(self.config.frame_rate / 30.0 * self.config.track_buffer)

        self.tracked_stracks: List[STrack] = []
        self.lost_stracks: List[STrack] = []
        self.removed_stracks: List[STrack] = []

    def reset(self) -> None:
        """Reset all tracks and ID counter."""
        self.frame_id = 0
        self.tracked_stracks = []
        self.lost_stracks = []
        self.removed_stracks = []

        STrack._count = 0

        logger.info("ByteTrack tracker reset.")

    def update(
        self,
        bboxes: np.ndarray,
        scores: np.ndarray,
        keypoints: Optional[List[Optional[np.ndarray]]] = None,
    ) -> List[TrackOutput]:
        """Update tracker with detections and return active tracks."""
        self.frame_id += 1

        if bboxes is None or len(bboxes) == 0:
            bboxes = np.empty((0, 4), dtype=np.float32)
            scores = np.empty((0,), dtype=np.float32)

        bboxes = np.asarray(bboxes, dtype=np.float32)
        scores = np.asarray(scores, dtype=np.float32)

        if bboxes.ndim != 2 or bboxes.shape[1] != 4:
            bboxes = np.empty((0, 4), dtype=np.float32)
            scores = np.empty((0,), dtype=np.float32)

        high_mask = scores >= self.config.track_high_thresh
        low_mask = (scores >= self.config.track_low_thresh) & (~high_mask)

        high_dets = bboxes[high_mask]
        high_scores = scores[high_mask]
        high_indices = np.where(high_mask)[0]

        low_dets = bboxes[low_mask]
        low_scores = scores[low_mask]

        detections = [
            STrack(high_dets[i], float(high_scores[i]))
            for i in range(len(high_dets))
        ]

        for track in self.tracked_stracks + self.lost_stracks:
            track.predict()

        pool = [t for t in self.tracked_stracks if t.state == "Tracked"]
        pool.extend(self.lost_stracks)

        dists = self._iou_distance(pool, detections)
        matches, u_track, u_det = _linear_assignment(
            dists,
            1.0 - self.config.match_thresh,
        )

        for itracked, idet in matches:
            track = pool[itracked]
            det = detections[idet]

            if track.state == "Tracked":
                track.update(det.bbox_xyxy, det.score)
            else:
                track.re_activate(det.bbox_xyxy, det.score, self.frame_id)

            track.state = "Tracked"
            track.frame_id = self.frame_id

        remaining_tracks = [pool[i] for i in u_track]

        if len(low_dets) > 0 and len(remaining_tracks) > 0:
            low_det_stracks = [
                STrack(low_dets[i], float(low_scores[i]))
                for i in range(len(low_dets))
            ]

            dists = self._iou_distance(remaining_tracks, low_det_stracks)
            matches2, u_track2, _ = _linear_assignment(
                dists,
                1.0 - self.config.match_thresh,
            )

            for itracked, idet in matches2:
                track = remaining_tracks[itracked]
                det = low_det_stracks[idet]

                if track.state == "Tracked":
                    track.update(det.bbox_xyxy, det.score)
                else:
                    track.re_activate(det.bbox_xyxy, det.score, self.frame_id)

                track.state = "Tracked"
                track.frame_id = self.frame_id

            remaining_tracks = [remaining_tracks[i] for i in u_track2]

        for track in remaining_tracks:
            if track.state != "Lost":
                track.state = "Lost"

            track.time_since_update += 1

        for idet in u_det:
            det = detections[idet]

            if det.score >= self.config.new_track_thresh:
                det.activate(self.frame_id)
                self.tracked_stracks.append(det)

        activated: List[STrack] = []
        lost: List[STrack] = []
        removed: List[STrack] = []

        for track in self.tracked_stracks:
            if not track.is_activated:
                continue

            if track.state == "Tracked":
                activated.append(track)
            elif track.time_since_update > self.max_time_lost:
                track.state = "Removed"
                removed.append(track)
            else:
                lost.append(track)

        self.tracked_stracks = activated

        existing_lost_ids = {t.track_id for t in lost}
        self.lost_stracks = [
            t for t in self.lost_stracks
            if t.state != "Removed" and t.track_id not in existing_lost_ids
        ]
        self.lost_stracks.extend(lost)
        self.lost_stracks = [
            t for t in self.lost_stracks
            if t.time_since_update <= self.max_time_lost
        ]

        self.removed_stracks.extend(removed)

        outputs: List[TrackOutput] = []

        active = self.tracked_stracks + [
            t for t in self.lost_stracks
            if t.time_since_update <= 2
        ]

        for track in active:
            if not track.is_activated:
                continue

            det_index = -1

            if len(high_dets) > 0:
                ious = _iou_batch(track.bbox_xyxy.reshape(1, 4), high_dets)

                if ious.size > 0:
                    best = int(np.argmax(ious[0]))

                    if ious[0, best] > 0.5:
                        det_index = int(high_indices[best])

            kpts = None

            if keypoints is not None and det_index >= 0 and det_index < len(keypoints):
                kpts = keypoints[det_index]

            outputs.append(
                TrackOutput(
                    track_id=track.track_id,
                    bbox=track.bbox_xyxy.astype(np.float32),
                    score=float(track.score),
                    keypoints=kpts,
                    detection_index=det_index,
                )
            )

        return outputs

    def _iou_distance(
        self,
        tracks: List[STrack],
        detections: List[STrack],
    ) -> np.ndarray:
        """Return IoU distance matrix."""
        if not tracks or not detections:
            return np.zeros((len(tracks), len(detections)), dtype=np.float32)

        track_boxes = np.array([t.bbox_xyxy for t in tracks], dtype=np.float32)
        det_boxes = np.array([d.bbox_xyxy for d in detections], dtype=np.float32)

        ious = _iou_batch(track_boxes, det_boxes)
        return 1.0 - ious