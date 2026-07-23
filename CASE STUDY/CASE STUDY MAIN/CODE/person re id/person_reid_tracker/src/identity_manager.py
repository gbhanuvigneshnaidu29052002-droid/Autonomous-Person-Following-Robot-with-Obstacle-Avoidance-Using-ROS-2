"""Identity fusion: combine face, body, ByteTrack history into OWNER/UNKNOWN."""

from __future__ import annotations

import logging
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from typing import Deque, Dict, List, Optional

import numpy as np

from src.bytetrack_tracker import TrackOutput
from src.face_reid import FaceMatchResult, FaceReID
from src.person_reid import PersonReID

logger = logging.getLogger(__name__)


class IdentityLabel(str, Enum):
    OWNER = "OWNER"
    UNKNOWN = "UNKNOWN"


@dataclass
class TrackIdentityState:
    """Per-track identity history and smoothed decision."""

    track_id: int
    label: IdentityLabel = IdentityLabel.UNKNOWN
    confidence: float = 0.0
    face_score: float = 0.0
    body_score: float = 0.0
    face_detected: bool = False
    face_confirmed_owner: bool = False
    last_face_frame: int = -1
    last_body_frame: int = -1
    history: Deque[IdentityLabel] = field(default_factory=lambda: deque(maxlen=30))
    owner_streak: int = 0
    unknown_streak: int = 0


@dataclass
class IdentityResult:
    track_id: int
    label: IdentityLabel
    confidence: float
    face_score: float
    body_score: float
    face_detected: bool
    bbox: np.ndarray
    detection_score: float
    keypoints: Optional[np.ndarray] = None


class IdentityManager:
    """
    Fuse face ReID, body ReID, and temporal smoothing per ByteTrack ID.

    Rules:
    - Face match has highest priority when a face is visible.
    - Strong face mismatch vetoes track-history OWNER label.
    - Body ReID helps when face is not visible.
    - ByteTrack continuity smooths labels but does not override strong face mismatch.
    """

    def __init__(
        self,
        face_reid: Optional[FaceReID],
        person_reid: Optional[PersonReID],
        face_threshold: float = 0.45,
        body_threshold: float = 0.60,
        face_mismatch_threshold: float = 0.25,
        body_owner_create_threshold: float = 0.95,
        body_owner_maintain_threshold: float = 0.75,
        body_score_margin: float = 0.08,
        require_face_to_create_owner: bool = True,
        smoothing_frames: int = 10,
        owner_confirm_frames: int = 3,
        unknown_confirm_frames: int = 5,
        face_reid_interval: int = 5,
        body_reid_interval: int = 10,
        debug_log_identity_changes: bool = True,
    ):
        self.face_reid = face_reid
        self.person_reid = person_reid
        self.face_threshold = face_threshold
        self.body_threshold = body_threshold
        self.face_mismatch_threshold = face_mismatch_threshold
        self.body_owner_create_threshold = body_owner_create_threshold
        self.body_owner_maintain_threshold = body_owner_maintain_threshold
        self.body_score_margin = body_score_margin
        self.require_face_to_create_owner = require_face_to_create_owner
        self.smoothing_frames = smoothing_frames
        self.owner_confirm_frames = owner_confirm_frames
        self.unknown_confirm_frames = unknown_confirm_frames
        self.face_reid_interval = max(1, int(face_reid_interval))
        self.body_reid_interval = max(1, int(body_reid_interval))
        self.debug_log_identity_changes = debug_log_identity_changes
        self._tracks: Dict[int, TrackIdentityState] = {}
        self._frame_idx = 0

    def reset(self) -> None:
        self._tracks.clear()
        self._frame_idx = 0
        logger.info("Identity manager reset.")

    def _get_state(self, track_id: int) -> TrackIdentityState:
        if track_id not in self._tracks:
            self._tracks[track_id] = TrackIdentityState(track_id=track_id)
        return self._tracks[track_id]

    def _prune_stale(self, active_ids: set) -> None:
        stale = [tid for tid in self._tracks if tid not in active_ids]
        for tid in stale:
            del self._tracks[tid]

    def _update_cached_reid(
        self,
        frame: np.ndarray,
        track: TrackOutput,
        state: TrackIdentityState,
    ) -> FaceMatchResult:
        """Run ReID only on configured intervals and reuse cached scores otherwise."""
        if (
            self.face_reid
            and self.face_reid.is_ready
            and (
                state.last_face_frame < 0
                or self._frame_idx - state.last_face_frame >= self.face_reid_interval
            )
        ):
            face_result = self.face_reid.match_person_crop(frame, track.bbox)
            state.face_score = face_result.similarity
            state.face_detected = face_result.face_detected
            state.last_face_frame = self._frame_idx
        else:
            face_result = FaceMatchResult(
                similarity=state.face_score,
                face_detected=state.face_detected,
            )

        if (
            self.person_reid
            and self.person_reid.is_ready
            and (
                state.last_body_frame < 0
                or self._frame_idx - state.last_body_frame >= self.body_reid_interval
            )
        ):
            state.body_score = self.person_reid.match_person(frame, track.bbox)
            state.last_body_frame = self._frame_idx

        return face_result

    def _body_score_is_clear(self, track_id: int, active_states: Dict[int, TrackIdentityState]) -> bool:
        other_scores = [
            state.body_score
            for other_id, state in active_states.items()
            if other_id != track_id
        ]
        if not other_scores:
            return True
        return active_states[track_id].body_score >= max(other_scores) + self.body_score_margin

    def _instant_label(
        self,
        state: TrackIdentityState,
        body_score_clear: bool,
    ) -> tuple[IdentityLabel, float, str, bool]:
        """Single-frame identity decision before temporal smoothing."""
        face = FaceMatchResult(
            similarity=state.face_score,
            face_detected=state.face_detected,
        )
        body_score = state.body_score

        if face.face_detected:
            if face.similarity >= self.face_threshold:
                state.face_confirmed_owner = True
                return IdentityLabel.OWNER, face.similarity, "face_match", False
            if face.similarity <= self.face_mismatch_threshold:
                state.face_confirmed_owner = False
                return IdentityLabel.UNKNOWN, 1.0 - face.similarity, "strong_face_mismatch", True

            if (
                state.face_confirmed_owner
                and state.label == IdentityLabel.OWNER
                and body_score >= self.body_owner_maintain_threshold
                and body_score_clear
            ):
                return IdentityLabel.OWNER, body_score, "body_maintains_face_confirmed_owner", False
            return IdentityLabel.UNKNOWN, max(1.0 - face.similarity, 1.0 - body_score), "ambiguous_face", False

        if (
            state.face_confirmed_owner
            and state.label == IdentityLabel.OWNER
            and body_score >= self.body_owner_maintain_threshold
            and body_score_clear
        ):
            return IdentityLabel.OWNER, body_score, "body_maintains_face_confirmed_owner", False

        if self.require_face_to_create_owner:
            return IdentityLabel.UNKNOWN, max(0.5, 1.0 - body_score), "face_required_to_create_owner", False

        if body_score >= self.body_owner_create_threshold and body_score_clear:
            return IdentityLabel.OWNER, body_score, "body_create_owner_clear_margin", False

        if body_score >= self.body_threshold and not body_score_clear:
            return IdentityLabel.UNKNOWN, max(0.5, 1.0 - body_score), "body_score_not_unique", False

        return IdentityLabel.UNKNOWN, max(0.5, 1.0 - body_score), "no_reid_match", False

    def update(
        self,
        frame: np.ndarray,
        tracks: List[TrackOutput],
    ) -> List[IdentityResult]:
        self._frame_idx += 1
        active_ids = {t.track_id for t in tracks}
        self._prune_stale(active_ids)

        results: List[IdentityResult] = []
        active_states: Dict[int, TrackIdentityState] = {}

        for track in tracks:
            state = self._get_state(track.track_id)
            self._update_cached_reid(frame, track, state)
            active_states[track.track_id] = state

        pending_results: list[tuple[TrackOutput, TrackIdentityState, IdentityLabel, float, str, bool]] = []

        for track in tracks:
            state = self._get_state(track.track_id)
            instant_label, instant_conf, reason, forced_unknown = self._instant_label(
                state,
                self._body_score_is_clear(track.track_id, active_states),
            )

            # Temporal smoothing with streak counters
            if instant_label == IdentityLabel.OWNER:
                state.owner_streak += 1
                state.unknown_streak = 0
            else:
                state.unknown_streak += 1
                state.owner_streak = 0

            new_label = state.label
            if instant_label == IdentityLabel.OWNER:
                if state.owner_streak >= self.owner_confirm_frames or state.label == IdentityLabel.OWNER:
                    new_label = IdentityLabel.OWNER
            else:
                if forced_unknown:
                    new_label = IdentityLabel.UNKNOWN
                elif state.unknown_streak >= self.unknown_confirm_frames:
                    new_label = IdentityLabel.UNKNOWN

            pending_results.append((track, state, new_label, instant_conf, reason, forced_unknown))

        owner_candidates = [
            item for item in pending_results
            if item[2] == IdentityLabel.OWNER
        ]
        if len(owner_candidates) > 1:
            owner_to_keep = max(
                owner_candidates,
                key=lambda item: (
                    item[1].face_score >= self.face_threshold,
                    item[3],
                    item[1].body_score,
                ),
            )[0].track_id
            pending_results = [
                (
                    track,
                    state,
                    label if label != IdentityLabel.OWNER or track.track_id == owner_to_keep else IdentityLabel.UNKNOWN,
                    conf,
                    reason if label != IdentityLabel.OWNER or track.track_id == owner_to_keep else "multiple_owner_suppressed",
                    forced,
                )
                for track, state, label, conf, reason, forced in pending_results
            ]

        for track, state, new_label, instant_conf, reason, _forced_unknown in pending_results:
            previous_label = state.label

            if new_label != previous_label and self.debug_log_identity_changes:
                logger.info(
                    (
                        "Identity change track_id=%d: %s -> %s "
                        "face_score=%.2f body_score=%.2f face_visible=%s reason=%s"
                    ),
                    track.track_id,
                    previous_label.value,
                    new_label.value,
                    state.face_score,
                    state.body_score,
                    state.face_detected,
                    reason,
                )

            state.label = new_label
            state.confidence = instant_conf
            state.history.append(new_label)

            results.append(
                IdentityResult(
                    track_id=track.track_id,
                    label=new_label,
                    confidence=instant_conf,
                    face_score=state.face_score,
                    body_score=state.body_score,
                    face_detected=state.face_detected,
                    bbox=track.bbox,
                    detection_score=track.score,
                    keypoints=track.keypoints,
                )
            )

        return results

    def get_owner_track(self, results: List[IdentityResult]) -> Optional[IdentityResult]:
        """Return highest-confidence OWNER track, if any."""
        owners = [r for r in results if r.label == IdentityLabel.OWNER]
        if not owners:
            return None
        return max(owners, key=lambda r: r.confidence)
