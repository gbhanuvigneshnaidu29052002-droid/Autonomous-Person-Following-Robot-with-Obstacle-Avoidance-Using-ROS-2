#!/usr/bin/env python3
"""
Perception Node — YOLOv8n + ByteTrack + Full Re-ID
====================================================

Re-ID pipeline (embedded from ~/person re id/ system):

  TRAINING (first ~5 seconds)
    • Collects body crops and HSV histograms of the centred person
    • Builds a body embedding gallery using MobileNetV3 (or OSNet if torchreid installed)
    • Builds a parallel HSV histogram fingerprint

  TRACKING (once training complete)
    1. ByteTrack ID continuity  — fastest, no compute
    2. Body embedding cosine similarity ≥ 0.55 — deep Re-ID
    3. HSV Bhattacharyya histogram similarity ≥ 0.55 — colour fallback
    4. Spatial angle fallback — if only 1 person or very close to last angle

  LiDAR fusion
    • Median range over ±window rays centred on person bearing
    • Falls back to focal-length bounding-box estimation if LiDAR unavailable
"""

from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from ultralytics import YOLO

# ── Tuning ────────────────────────────────────────────────────────────────────
TRAIN_FRAMES        = 40       # frames of training crops to collect
BODY_REID_THRESH    = 0.52     # cosine similarity threshold for body Re-ID
HSV_REID_THRESH     = 0.50     # Bhattacharyya similarity threshold (1 = identical)
SPATIAL_ANG_THRESH  = 0.25     # rad — spatial fallback window
EMBED_UPDATE_ALPHA  = 0.10     # running average weight for embedding gallery update
# ─────────────────────────────────────────────────────────────────────────────


# ═══════════════════════════════════════════════════════════════════════════════
# Lightweight utilities (from person re id / src/utils.py)
# ═══════════════════════════════════════════════════════════════════════════════

def _cosine_sim(a: np.ndarray, b: np.ndarray) -> float:
    a = np.asarray(a, dtype=np.float32).flatten()
    b = np.asarray(b, dtype=np.float32).flatten()
    na, nb = np.linalg.norm(a), np.linalg.norm(b)
    if na < 1e-8 or nb < 1e-8:
        return 0.0
    return float(np.dot(a, b) / (na * nb))


def _max_cosine_to_gallery(emb: np.ndarray, gallery: List[np.ndarray]) -> float:
    if not gallery:
        return 0.0
    return max(_cosine_sim(emb, g) for g in gallery)


def _normalize(emb: np.ndarray) -> np.ndarray:
    emb = np.asarray(emb, dtype=np.float32).flatten()
    n = np.linalg.norm(emb)
    return emb if n < 1e-8 else emb / n


def _crop_padded(frame: np.ndarray, x1, y1, x2, y2, pad: float = 0.04) -> Optional[np.ndarray]:
    h, w = frame.shape[:2]
    bw, bh = x2 - x1, y2 - y1
    x1 = max(0, int(x1 - bw * pad))
    y1 = max(0, int(y1 - bh * pad))
    x2 = min(w, int(x2 + bw * pad))
    y2 = min(h, int(y2 + bh * pad))
    if x2 <= x1 or y2 <= y1:
        return None
    return frame[y1:y2, x1:x2].copy()


# ═══════════════════════════════════════════════════════════════════════════════
# Body Re-ID backend (from person re id / src/person_reid.py)
# ═══════════════════════════════════════════════════════════════════════════════

class BodyEmbedder:
    """
    Tries to load OSNet (torchreid) first; falls back to MobileNetV3.
    Works fully on CPU — no GPU required.
    """

    def __init__(self, device: str = "cpu"):
        self._model = None
        self._transform = None
        self._torch = None
        self._backend = "none"

        # Try torchreid OSNet first (best accuracy)
        try:
            import torch
            import torchreid
            from torchvision import transforms as T

            self._torch = torch
            dev = torch.device("cpu")
            m = torchreid.models.build_model(name="osnet_x0_5", num_classes=1000, pretrained=True)
            m.eval().to(dev)
            self._model = m
            self._device = dev
            self._transform = T.Compose([
                T.ToPILImage(),
                T.Resize((256, 128)),
                T.ToTensor(),
                T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
            ])
            self._backend = "osnet"
            print("[ReID] OSNet body backend loaded.")
        except Exception:
            pass

        # Fallback: MobileNetV3-Small feature extractor
        if self._backend == "none":
            try:
                import torch
                from torchvision import models, transforms as T

                self._torch = torch
                dev = torch.device("cpu")
                weights = models.MobileNet_V3_Small_Weights.DEFAULT
                base = models.mobilenet_v3_small(weights=weights)
                self._model = base.features
                self._model.eval().to(dev)
                self._device = dev
                self._transform = T.Compose([
                    T.ToPILImage(),
                    T.Resize((256, 128)),
                    T.ToTensor(),
                    T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
                ])
                self._backend = "mobilenet"
                print("[ReID] MobileNetV3 body backend loaded (OSNet not available).")
            except Exception as e:
                print(f"[ReID] WARNING: No deep Re-ID backend available ({e}). Using HSV only.")

    def embed(self, crop_bgr: np.ndarray) -> Optional[np.ndarray]:
        if self._model is None or crop_bgr is None or crop_bgr.size == 0:
            return None
        try:
            rgb = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB)
            tensor = self._transform(rgb).unsqueeze(0).to(self._device)
            with self._torch.no_grad():
                feat = self._model(tensor)
                if isinstance(feat, (tuple, list)):
                    feat = feat[0]
                if feat.dim() > 2:
                    feat = feat.mean(dim=[2, 3])
            return _normalize(feat.cpu().numpy().flatten())
        except Exception:
            return None

    @property
    def available(self) -> bool:
        return self._model is not None


# ═══════════════════════════════════════════════════════════════════════════════
# HSV histogram utilities (from previous embedded implementation)
# ═══════════════════════════════════════════════════════════════════════════════

def _hsv_hist(crop_bgr: np.ndarray) -> Optional[np.ndarray]:
    if crop_bgr is None or crop_bgr.size == 0:
        return None
    hsv  = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2HSV)
    hist = cv2.calcHist([hsv], [0, 1], None, [36, 32], [0, 180, 0, 256])
    cv2.normalize(hist, hist, 0, 1, cv2.NORM_MINMAX)
    return hist


def _hsv_sim(h1: np.ndarray, h2: np.ndarray) -> float:
    """Bhattacharyya similarity [0=different, 1=identical]."""
    return 1.0 - cv2.compareHist(h1, h2, cv2.HISTCMP_BHATTACHARYYA)


# ═══════════════════════════════════════════════════════════════════════════════
# Main ROS 2 Node
# ═══════════════════════════════════════════════════════════════════════════════

class PerceptionNode(Node):
    """
    YOLOv8n + ByteTrack + multi-level Re-ID (deep body + HSV histogram + spatial).
    Starts in TRAINING mode automatically — stand in front of camera!
    """

    def __init__(self) -> None:
        super().__init__("perception_node")

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter("camera_topic",         "/camera/image_raw")
        self.declare_parameter("scan_topic",           "/scan")
        self.declare_parameter("camera_hfov",          1.047)
        self.declare_parameter("yolo_model_path",      "yolov8n.pt")
        self.declare_parameter("confidence_threshold", 0.30)

        camera_topic = str(self.get_parameter("camera_topic").value)
        scan_topic   = str(self.get_parameter("scan_topic").value)
        model_path   = str(self.get_parameter("yolo_model_path").value)
        self.hfov    = float(self.get_parameter("camera_hfov").value)
        self.conf_th = float(self.get_parameter("confidence_threshold").value)

        # ── Models ────────────────────────────────────────────────────────────
        self.bridge = CvBridge()
        self.yolo   = YOLO(model_path)
        self.yolo.fuse()
        self.body_embedder = BodyEmbedder(device="cpu")

        # ── QoS ──────────────────────────────────────────────────────────────
        cam_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ── ROS I/O ───────────────────────────────────────────────────────────
        self.create_subscription(Image,     camera_topic, self._img_cb,   cam_qos)
        self.create_subscription(LaserScan, scan_topic,   self._scan_cb,  scan_qos)
        self.create_subscription(String, "/perception/command", self._cmd_cb, 10)

        self._data_pub  = self.create_publisher(String, "/tracked_target",   10)
        self._debug_pub = self.create_publisher(Image,  "/yolo/debug_image", 10)

        # ── Runtime state ─────────────────────────────────────────────────────
        self.latest_scan: Optional[LaserScan] = None
        self.image_width  = 640
        self.image_height = 480

        # Training
        self.mode             = "TRAINING"    # TRAINING | TRACKING
        self._train_bodies: List[np.ndarray] = []   # body embedding samples
        self._train_hists:  List[np.ndarray] = []   # HSV histogram samples

        # Locked identity
        self.target_id: Optional[int] = None
        self._body_gallery: List[np.ndarray] = []   # deep body embedding gallery
        self._hsv_fingerprint: Optional[np.ndarray] = None  # averaged HSV hist

        # Memory
        self.last_known_angle    = 0.0
        self.last_known_distance = 1.5
        self.last_seen_time      = time.time()

        self.get_logger().info(
            "PerceptionNode (YOLOv8 + ByteTrack + Re-ID) started — TRAINING mode. "
            "Stand in front of the robot!"
        )

    # ── Command handler ───────────────────────────────────────────────────────

    def _cmd_cb(self, msg: String) -> None:
        cmd = msg.data.upper().strip()
        if cmd == "TRAIN":
            self._reset_training()
            self.get_logger().info("Commanded TRAIN — stand in front!")
        elif cmd == "RESET":
            self._reset_training()

    def _reset_training(self):
        self.mode              = "TRAINING"
        self.target_id         = None
        self._train_bodies     = []
        self._train_hists      = []
        self._body_gallery     = []
        self._hsv_fingerprint  = None

    # ── Sensor callbacks ──────────────────────────────────────────────────────

    def _scan_cb(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def _img_cb(self, msg: Image) -> None:
        # Decode image
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            try:
                raw   = np.frombuffer(msg.data, dtype=np.uint8)
                yuyv  = raw[: msg.height * msg.step].reshape(msg.height, msg.width, 2)
                frame = cv2.cvtColor(yuyv, cv2.COLOR_YUV2BGR_YUY2)
            except Exception as e:
                self.get_logger().error(f"Image decode failed: {e}")
                return

        if frame is None or frame.size == 0:
            return
        self.image_width  = frame.shape[1]
        self.image_height = frame.shape[0]

        # Run YOLO + ByteTrack
        results = self.yolo.track(
            source=frame, classes=[0],
            conf=self.conf_th,
            persist=True, tracker="bytetrack.yaml",
            verbose=False,
        )
        persons = self._parse(results)

        status     = "lost"
        target_box = None

        if self.mode == "TRAINING":
            target_box = self._train_step(frame, persons)
            if self.mode == "TRACKING":
                status = "visible" if target_box is not None else "lost"
        elif self.mode == "TRACKING":
            target_box, status = self._find_target(frame, persons)

        # Compute angle & distance
        raw_angle = 0.0
        raw_dist  = -1.0
        if target_box is not None:
            cx        = (target_box["x1"] + target_box["x2"]) / 2.0
            raw_angle = (cx - self.image_width / 2.0) * (self.hfov / self.image_width)
            self.last_known_angle = raw_angle
            raw_dist = self._lidar_dist(raw_angle)
            if raw_dist <= 0.0:
                raw_dist = self._bbox_dist(target_box["y1"], target_box["y2"])
            if raw_dist > 0.0:
                self.last_known_distance = raw_dist
            self.last_seen_time = time.time()

        # Publish
        out = String()
        out.data = json.dumps({
            "visible":          status == "visible",
            "status":           status,
            "distance":         float(raw_dist if raw_dist > 0 else self.last_known_distance),
            "angle":            float(raw_angle),
            "last_known_angle": float(self.last_known_angle),
            "last_seen_time":   float(self.last_seen_time),
        })
        self._data_pub.publish(out)
        self._draw_debug(frame, persons, target_box, status)

    # ═══════════════════════════════════════════════════════════════════════════
    # Training phase
    # ═══════════════════════════════════════════════════════════════════════════

    def _train_step(self, frame: np.ndarray, persons: list) -> Optional[dict]:
        """Collect crops of the most-centred person. Finalise after TRAIN_FRAMES."""
        candidate = self._centremost(persons)
        if candidate is None:
            return None

        crop = _crop_padded(frame, candidate["x1"], candidate["y1"],
                            candidate["x2"], candidate["y2"])
        if crop is None:
            return None

        # Collect body embedding
        if self.body_embedder.available:
            emb = self.body_embedder.embed(crop)
            if emb is not None:
                self._train_bodies.append(emb)

        # Collect HSV histogram
        h = _hsv_hist(crop)
        if h is not None:
            self._train_hists.append(h)

        # Draw training progress bar
        total     = max(len(self._train_bodies), len(self._train_hists))
        pct       = total / TRAIN_FRAMES
        bar_w     = int(pct * self.image_width)
        cv2.rectangle(frame, (0, self.image_height - 18),
                      (bar_w, self.image_height), (0, 230, 100), -1)
        cv2.putText(frame, f"Training {total}/{TRAIN_FRAMES}  Stand still!",
                    (10, self.image_height - 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        if total >= TRAIN_FRAMES:
            self._finalise_training(candidate["id"])

        return candidate

    def _finalise_training(self, seed_id: int) -> None:
        """Build gallery and fingerprint from collected samples."""
        self._body_gallery = self._train_bodies.copy()

        if self._train_hists:
            stacked = np.mean(np.array(self._train_hists), axis=0).astype(np.float32)
            cv2.normalize(stacked, stacked, 0, 1, cv2.NORM_MINMAX)
            self._hsv_fingerprint = stacked

        self.target_id = seed_id
        self.mode      = "TRACKING"
        self.get_logger().info(
            f"Training DONE — ID={self.target_id}, "
            f"{len(self._body_gallery)} body embeddings, "
            f"HSV fingerprint={'yes' if self._hsv_fingerprint is not None else 'no'}"
        )

    # ═══════════════════════════════════════════════════════════════════════════
    # Multi-level Re-ID — TRACKING phase
    # ═══════════════════════════════════════════════════════════════════════════

    def _find_target(self, frame: np.ndarray, persons: list) -> Tuple[Optional[dict], str]:

        # ── Level 1: ByteTrack ID match ───────────────────────────────────────
        for p in persons:
            if p["id"] == self.target_id:
                self._update_gallery(frame, p)
                return p, "visible"

        # ── Level 2: Deep body embedding Re-ID ───────────────────────────────
        if self._body_gallery and self.body_embedder.available:
            best_sim, best_p = 0.0, None
            for p in persons:
                crop = _crop_padded(frame, p["x1"], p["y1"], p["x2"], p["y2"])
                if crop is None:
                    continue
                emb = self.body_embedder.embed(crop)
                if emb is None:
                    continue
                sim = _max_cosine_to_gallery(emb, self._body_gallery)
                if sim > best_sim:
                    best_sim, best_p = sim, p
            if best_p is not None and best_sim >= BODY_REID_THRESH:
                self.target_id = best_p["id"]
                self._update_gallery(frame, best_p)
                self.get_logger().info(
                    f"Body Re-ID success → new ID={self.target_id}, sim={best_sim:.2f}"
                )
                return best_p, "visible"

        # ── Level 3: HSV histogram Re-ID ─────────────────────────────────────
        if self._hsv_fingerprint is not None and persons:
            best_sim, best_p = 0.0, None
            for p in persons:
                crop = _crop_padded(frame, p["x1"], p["y1"], p["x2"], p["y2"])
                if crop is None:
                    continue
                h = _hsv_hist(crop)
                if h is None:
                    continue
                sim = _hsv_sim(self._hsv_fingerprint, h)
                if sim > best_sim:
                    best_sim, best_p = sim, p
            if best_p is not None and best_sim >= HSV_REID_THRESH:
                self.target_id = best_p["id"]
                self.get_logger().info(
                    f"HSV Re-ID success → new ID={self.target_id}, sim={best_sim:.2f}"
                )
                return best_p, "visible"

        # ── Level 4: Spatial fallback ─────────────────────────────────────────
        if persons:
            cx_ref = self.image_width / 2.0 + \
                     self.last_known_angle * (self.image_width / self.hfov)
            dists  = [abs((p["x1"] + p["x2"]) / 2.0 - cx_ref) for p in persons]
            idx    = int(np.argmin(dists))
            ang_px_thresh = SPATIAL_ANG_THRESH * (self.image_width / self.hfov)
            if len(persons) == 1 or dists[idx] < ang_px_thresh:
                self.target_id = persons[idx]["id"]
                self.get_logger().info(f"Spatial Re-ID → ID={self.target_id}")
                return persons[idx], "visible"

        return None, "lost"

    def _update_gallery(self, frame: np.ndarray, p: dict) -> None:
        """Continuously update the gallery with the confirmed target's embedding."""
        if not self.body_embedder.available:
            return
        crop = _crop_padded(frame, p["x1"], p["y1"], p["x2"], p["y2"])
        if crop is None:
            return
        emb = self.body_embedder.embed(crop)
        if emb is None:
            return
        # Keep gallery bounded — rolling update of last 20 embeddings
        self._body_gallery.append(emb)
        if len(self._body_gallery) > 20:
            self._body_gallery = self._body_gallery[-20:]

        # Update HSV fingerprint with exponential moving average
        h = _hsv_hist(crop)
        if h is not None and self._hsv_fingerprint is not None:
            self._hsv_fingerprint = (
                (1 - EMBED_UPDATE_ALPHA) * self._hsv_fingerprint +
                EMBED_UPDATE_ALPHA * h
            )
            cv2.normalize(self._hsv_fingerprint, self._hsv_fingerprint, 0, 1, cv2.NORM_MINMAX)

    # ── LiDAR / distance helpers ──────────────────────────────────────────────

    def _lidar_dist(self, angle: float, window: int = 8) -> float:
        if self.latest_scan is None:
            return -1.0
        scan = self.latest_scan
        if not scan.ranges or scan.angle_increment == 0.0:
            return -1.0
        centre = int(round((angle - scan.angle_min) / scan.angle_increment))
        valid  = []
        for idx in range(centre - window, centre + window + 1):
            if 0 <= idx < len(scan.ranges):
                d = scan.ranges[idx]
                if math.isfinite(d) and scan.range_min <= d <= scan.range_max:
                    valid.append(d)
        return float(np.median(valid)) if valid else -1.0

    def _bbox_dist(self, y1: int, y2: int) -> float:
        h = y2 - y1
        if h <= 0:
            return -1.0
        focal = self.image_width / (2.0 * math.tan(self.hfov / 2.0))
        return float(1.70 * focal / h)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _parse(self, results) -> list:
        persons = []
        if results and results[0].boxes is not None:
            for box in results[0].boxes:
                if box.id is None:
                    continue
                x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
                persons.append({
                    "id":   int(box.id[0].item()),
                    "conf": float(box.conf[0].item()),
                    "x1": x1, "y1": y1, "x2": x2, "y2": y2,
                })
        return persons

    def _centremost(self, persons: list) -> Optional[dict]:
        if not persons:
            return None
        cx = self.image_width / 2.0
        return min(persons, key=lambda p: abs((p["x1"] + p["x2"]) / 2.0 - cx))

    def _draw_debug(self, frame, persons, target_box, status):
        for p in persons:
            is_tgt = (p == target_box)
            col    = (0, 255, 80) if is_tgt else (60, 60, 255)
            cv2.rectangle(frame, (p["x1"], p["y1"]), (p["x2"], p["y2"]), col, 2)
            cv2.putText(frame, f"ID:{p['id']} {p['conf']:.0%}",
                        (p["x1"], max(20, p["y1"] - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.52, col, 2)
        lbl = f"Mode:{self.mode}  TgtID:{self.target_id}  {status.upper()}"
        cv2.putText(frame, lbl, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)
        try:
            dbg              = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            dbg.header.stamp = self.get_clock().now().to_msg()
            self._debug_pub.publish(dbg)
        except Exception:
            pass


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.context.ok():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
