#!/usr/bin/env python3
"""Live person re-identification with pose estimation and ByteTrack."""

from __future__ import annotations

import logging
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.bytetrack_tracker import ByteTrackConfig, ByteTrackTracker
from src.detector_pose import PoseDetector
from src.face_reid import FaceReID
from src.identity_manager import IdentityManager
from src.person_reid import PersonReID
from src.ros2_adapter_stub import select_owner_target
from src.utils import (
    FPSCounter,
    embeddings_exist,
    get_device,
    load_config,
    load_owner_embeddings,
    open_camera,
    setup_logging,
)
from src.visualization import draw_tracks

logger = logging.getLogger(__name__)


def build_pipeline(config: dict):
    """Build detector, tracker, re-ID modules, and identity manager."""
    device = get_device(config.get("use_gpu", True))
    logger.info("Using device: %s", device)

    detector = PoseDetector(
        model_name=config.get("yolo_model", "yolov8n-pose.pt"),
        confidence=config.get("detection_confidence", 0.5),
        person_class_id=config.get("person_class_id", 0),
        device=device,
        use_pose=config.get("use_pose", True),
    )

    tracker = ByteTrackTracker(
        ByteTrackConfig(
            track_high_thresh=config.get("track_high_thresh", 0.5),
            track_low_thresh=config.get("track_low_thresh", 0.1),
            new_track_thresh=config.get("new_track_thresh", 0.6),
            track_buffer=config.get("track_buffer", 30),
            match_thresh=config.get("match_thresh", 0.8),
            frame_rate=config.get("frame_rate", 30),
        )
    )

    face_reid = FaceReID(
        device=device,
        enabled=config.get("use_face_reid", True),
        model_name=config.get("face_model_name", "buffalo_l"),
        det_size=config.get("face_det_size", 640),
    )

    person_reid = PersonReID(
        device=device,
        enabled=config.get("use_body_reid", True),
        model_name=config.get("body_reid_model", "osnet_x0_5"),
    )

    embeddings_path = config.get("embeddings_path", "data/embeddings/owner_embeddings.npz")

    if embeddings_exist(embeddings_path):
        face_emb, body_emb = load_owner_embeddings(embeddings_path)

        if face_emb is not None and len(face_emb) > 0 and face_reid.enabled:
            face_reid.load_owner_gallery(face_emb)
            logger.info("Loaded %d owner face embeddings.", len(face_emb))
        elif config.get("use_face_reid", True):
            logger.warning("No face embeddings found in %s", embeddings_path)

        if body_emb is not None and len(body_emb) > 0 and person_reid.enabled:
            person_reid.load_owner_gallery(body_emb)
            logger.info("Loaded %d owner body embeddings.", len(body_emb))
        elif config.get("use_body_reid", True):
            logger.warning("No body embeddings found in %s", embeddings_path)

    else:
        print(
            "\n*** Owner not enrolled. Run enrollment first:\n"
            "    python enroll_owner.py\n"
        )

        if config.get("use_face_reid", True) or config.get("use_body_reid", True):
            logger.warning(
                "Continuing without owner embeddings - all persons will be UNKNOWN."
            )

    identity_mgr = IdentityManager(
        face_reid=face_reid if face_reid.is_ready else None,
        person_reid=person_reid if person_reid.is_ready else None,
        face_threshold=config.get("face_match_threshold", 0.45),
        body_threshold=config.get("body_match_threshold", 0.60),
        face_mismatch_threshold=config.get("face_strong_mismatch_threshold", 0.25),
        body_owner_create_threshold=config.get("body_owner_create_threshold", 0.95),
        body_owner_maintain_threshold=config.get("body_owner_maintain_threshold", 0.75),
        body_score_margin=config.get("body_score_margin", 0.08),
        require_face_to_create_owner=config.get("require_face_to_create_owner", True),
        smoothing_frames=config.get("identity_smoothing_frames", 10),
        owner_confirm_frames=config.get("owner_confirm_frames", 3),
        unknown_confirm_frames=config.get("unknown_confirm_frames", 5),
        face_reid_interval=config.get("face_reid_interval", 5),
        body_reid_interval=config.get("body_reid_interval", 10),
        debug_log_identity_changes=config.get("debug_log_identity_changes", True),
    )

    return detector, tracker, identity_mgr, device


def save_debug_frame(frame: np.ndarray, config: dict) -> None:
    """Save current visualization frame for debugging."""
    debug_dir = Path(config.get("debug_save_dir", ROOT / "data" / "debug_frames"))
    debug_dir.mkdir(parents=True, exist_ok=True)

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = debug_dir / f"frame_{ts}.jpg"

    cv2.imwrite(str(path), frame)
    logger.info("Saved debug frame: %s", path)


def run_live(config: dict) -> None:
    """Run live webcam person tracking and re-identification."""
    cap = open_camera(
        config.get("camera_index", 0),
        config.get("frame_width", 640),
        config.get("frame_height", 480),
    )

    detector, tracker, identity_mgr, device = build_pipeline(config)

    fps_counter = FPSCounter()
    window = config.get("window_name", "Person ReID Tracker")
    perf_log_interval = max(1, int(config.get("perf_log_interval", 30)))
    frame_idx = 0

    print("\nLive mode started.")
    print("Controls: q=quit, r=reset tracker, s=save debug frame\n")

    try:
        while True:
            t0 = time.perf_counter()

            ok, frame = cap.read()
            if not ok or frame is None:
                logger.warning("Failed to read frame from camera.")
                break

            det_start = time.perf_counter()
            det_result = detector.detect(frame)
            det_time = time.perf_counter() - det_start
            dets = det_result.detections

            if dets:
                bboxes = np.stack([d.bbox for d in dets], axis=0).astype(np.float32)
                scores = np.array([d.confidence for d in dets], dtype=np.float32)
                keypoints = [d.keypoints for d in dets]
            else:
                bboxes = np.empty((0, 4), dtype=np.float32)
                scores = np.empty((0,), dtype=np.float32)
                keypoints = []

            tracker_start = time.perf_counter()
            tracks = tracker.update(bboxes, scores, keypoints)
            tracker_time = time.perf_counter() - tracker_start

            identity_start = time.perf_counter()
            identity_results = identity_mgr.update(frame, tracks)
            identity_time = time.perf_counter() - identity_start

            target = select_owner_target(identity_results)

            vis_start = time.perf_counter()
            vis = draw_tracks(
                frame,
                identity_results,
                config,
                fps=fps_counter.fps,
            )

            if target:
                cv2.circle(
                    vis,
                    (int(target.center_x), int(target.center_y)),
                    6,
                    (0, 255, 0),
                    -1,
                )
            vis_time = time.perf_counter() - vis_start

            total_time = time.perf_counter() - t0
            fps_counter.tick(total_time)
            frame_idx += 1

            if frame_idx % perf_log_interval == 0:
                logger.info(
                    (
                        "Perf frame=%d device=%s yolo=%.1fms tracker=%.1fms "
                        "identity_reid=%.1fms visualization=%.1fms total=%.1fms fps=%.2f"
                    ),
                    frame_idx,
                    device,
                    det_time * 1000.0,
                    tracker_time * 1000.0,
                    identity_time * 1000.0,
                    vis_time * 1000.0,
                    total_time * 1000.0,
                    fps_counter.fps,
                )

            cv2.imshow(window, vis)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break

            if key == ord("r"):
                tracker.reset()
                identity_mgr.reset()
                logger.info("Tracker and identity manager reset.")

            if key == ord("s"):
                save_debug_frame(vis, config)

    finally:
        cap.release()
        cv2.destroyAllWindows()


def main() -> None:
    """Entry point."""
    setup_logging()
    config = load_config()

    try:
        run_live(config)
    except KeyboardInterrupt:
        logger.info("Interrupted by user.")
    except Exception as exc:
        logger.exception("Fatal error: %s", exc)
        sys.exit(1)


if __name__ == "__main__":
    main()
