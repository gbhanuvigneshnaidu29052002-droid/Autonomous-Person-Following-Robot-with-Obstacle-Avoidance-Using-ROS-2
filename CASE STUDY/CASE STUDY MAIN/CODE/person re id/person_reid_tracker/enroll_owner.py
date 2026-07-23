#!/usr/bin/env python3
"""Enroll the owner (target person) with face and body embeddings."""

from __future__ import annotations

import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.detector_pose import PoseDetector
from src.face_reid import FaceReID
from src.person_reid import PersonReID
from src.utils import (
    get_device,
    load_config,
    open_camera,
    save_owner_embeddings,
    setup_logging,
)
from src.visualization import draw_enrollment_overlay

import logging

logger = logging.getLogger(__name__)


def run_enrollment(config: dict) -> None:
    device = get_device(config.get("use_gpu", True))
    cap = open_camera(
        config.get("camera_index", 0),
        config.get("frame_width", 640),
        config.get("frame_height", 480),
    )

    detector = PoseDetector(
        model_name=config.get("yolo_model", "yolov8n-pose.pt"),
        confidence=config.get("detection_confidence", 0.5),
        person_class_id=config.get("person_class_id", 0),
        device=device,
        use_pose=False,  # detection only for enrollment speed
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

    if not face_reid.backend and not person_reid.backend:
        print(
            "\nError: No ReID backends available.\n"
            "Install at least torch/torchvision for body ReID, or insightface/face_recognition for face ReID.\n"
        )
        sys.exit(1)

    target_face = config.get("enroll_face_samples", 30)
    target_body = config.get("enroll_body_samples", 30)
    min_face = config.get("enroll_min_face_samples", 10)
    min_body = config.get("enroll_min_body_samples", 10)
    interval = config.get("enroll_capture_interval_ms", 200) / 1000.0
    save_images = config.get("save_enrollment_images", True)

    owner_dir = Path(config["owner_images_dir"])
    if save_images:
        owner_dir.mkdir(parents=True, exist_ok=True)

    face_embeddings: list[np.ndarray] = []
    body_embeddings: list[np.ndarray] = []

    window = "Owner Enrollment"
    print("\n=== Owner Enrollment ===")
    print("Stand in front of the camera with your face and full body visible.")
    print("Move slightly (turn head, shift pose) for better coverage.")
    print("Press q to finish early, Esc to cancel.\n")

    last_capture = 0.0
    frame_idx = 0

    try:
        while len(face_embeddings) < target_face or len(body_embeddings) < target_body:
            ok, frame = cap.read()
            if not ok:
                logger.error("Camera read failed.")
                break

            now = time.perf_counter()
            progress = min(
                1.0,
                (len(face_embeddings) / target_face + len(body_embeddings) / target_body) / 2.0,
            )
            msg = "Capturing owner samples..."
            overlay = draw_enrollment_overlay(
                frame, msg, progress, len(face_embeddings) + len(body_embeddings),
                target_face + target_body,
            )
            cv2.imshow(window, overlay)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == 27:
                print("Enrollment cancelled.")
                return

            if now - last_capture < interval:
                continue

            last_capture = now
            frame_idx += 1

            # Face samples from full frame
            if face_reid.backend and len(face_embeddings) < target_face:
                for emb in face_reid.enroll_from_frame(frame):
                    face_embeddings.append(emb)
                    if save_images:
                        ts = datetime.now().strftime("%H%M%S_%f")
                        cv2.imwrite(str(owner_dir / f"face_{ts}.jpg"), frame)
                    break  # one per interval

            # Body samples from largest person detection
            if person_reid.backend and len(body_embeddings) < target_body:
                det = detector.detect(frame)
                if det.detections:
                    largest = max(
                        det.detections,
                        key=lambda d: (d.bbox[2] - d.bbox[0]) * (d.bbox[3] - d.bbox[1]),
                    )
                    emb = person_reid.enroll_from_frame(frame, largest.bbox)
                    if emb is not None:
                        body_embeddings.append(emb)
                        if save_images:
                            ts = datetime.now().strftime("%H%M%S_%f")
                            cv2.imwrite(str(owner_dir / f"body_{ts}.jpg"), frame)

    finally:
        cap.release()
        cv2.destroyAllWindows()

    print(f"\nCaptured {len(face_embeddings)} face and {len(body_embeddings)} body embeddings.")

    face_ok = len(face_embeddings) >= min_face or not config.get("use_face_reid", True)
    body_ok = len(body_embeddings) >= min_body or not config.get("use_body_reid", True)

    if config.get("use_face_reid", True) and len(face_embeddings) < min_face:
        print(f"WARNING: Need at least {min_face} face samples (got {len(face_embeddings)}).")
        face_ok = False

    if config.get("use_body_reid", True) and len(body_embeddings) < min_body:
        print(f"WARNING: Need at least {min_body} body samples (got {len(body_embeddings)}).")
        body_ok = False

    if not face_ok and not body_ok:
        print("\nEnrollment FAILED – not enough samples. Try again with better lighting.")
        sys.exit(1)

    if len(face_embeddings) == 0:
        print("Note: No face embeddings – live mode will rely on body ReID only.")

    if len(body_embeddings) == 0:
        print("Note: No body embeddings – live mode will rely on face ReID only.")

    out_path = config["embeddings_path"]
    save_owner_embeddings(out_path, face_embeddings, body_embeddings)
    print(f"\nEnrollment SUCCESS. Embeddings saved to:\n  {out_path}")
    print("\nRun live tracking with:\n  python main.py\n")


def main() -> None:
    setup_logging()
    config = load_config()
    run_enrollment(config)


if __name__ == "__main__":
    main()
