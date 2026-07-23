"""ROS 2 perception node for person re-identification.

Wires the existing ``person_reid_tracker`` pipeline (YOLO-pose + ByteTrack +
face/body ReID + identity fusion) into a ROS 2 node targeting ROS 2 Humble
on a TurtleBot3 Waffle Pi.

Subscriptions
    /camera/camera/color/image_raw   sensor_msgs/Image
    /scan                            sensor_msgs/LaserScan

Publications
    /yolo/debug_image                sensor_msgs/Image   (annotated)
    /tracked_target                  std_msgs/String      (JSON)
    /person_reid/identity            std_msgs/String      (label-change events)
    /person_reid/status              diagnostic_msgs/DiagnosticArray (1 Hz)

No file in ``person_reid_tracker/`` is modified; all detection, tracking,
ReID, and visualization code is reused via the path pip dependency declared
in ``setup.py``.
"""

from __future__ import annotations

import json
import logging
import time
from typing import Dict, List, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

# Reused from person_reid_tracker (installed as a path pip dep).
from src.bytetrack_tracker import ByteTrackConfig, ByteTrackTracker
from src.detector_pose import PoseDetector
from src.face_reid import FaceReID
from src.identity_manager import IdentityManager, IdentityLabel
from src.person_reid import PersonReID
from src.ros2_adapter_stub import PersonFollowingOutput, select_owner_target
from src.utils import (
    FPSCounter,
    embeddings_exist,
    get_device,
    load_config,
    load_owner_embeddings,
    setup_logging,
)
from src.visualization import draw_tracks

from .image_preprocessor import ImagePreprocessor
from .scan_distance import ScanDistanceHelper
from .track_target_publisher import (
    STATUS_LOST,
    STATUS_NO_OWNER,
    STATUS_SEARCHING,
    STATUS_VISIBLE,
    build_diagnostics,
    build_target_payload_full,
)

logger = logging.getLogger(__name__)


# Defaults mirror person_reid_tracker/config.yaml so the node works out of
# the box even when no YAML is supplied.
_TRACKER_DEFAULTS: Dict[str, object] = {
    'yolo_model': 'yolo11n-pose.pt',
    'detection_confidence': 0.55,
    'person_class_id': 0,
    'use_gpu': True,
    'use_pose': True,
    'use_face_reid': True,
    'use_body_reid': True,
    'track_high_thresh': 0.5,
    'track_low_thresh': 0.1,
    'new_track_thresh': 0.6,
    'track_buffer': 30,
    'match_thresh': 0.8,
    'frame_rate': 30,
    'face_model_name': 'buffalo_l',
    'face_det_size': 640,
    'face_match_threshold': 0.55,
    'face_strong_mismatch_threshold': 0.35,
    'face_reid_interval': 5,
    'body_reid_model': 'osnet_x0_5',
    'body_match_threshold': 0.92,
    'body_owner_create_threshold': 0.95,
    'body_owner_maintain_threshold': 0.75,
    'body_score_margin': 0.08,
    'body_reid_interval': 10,
    'require_face_to_create_owner': True,
    'identity_smoothing_frames': 5,
    'owner_confirm_frames': 5,
    'unknown_confirm_frames': 3,
    'owner_images_dir': 'data/owner',
    'embeddings_path': 'data/embeddings/owner_embeddings.npz',
    'debug_save_dir': 'data/debug_frames',
    'debug_log_identity_changes': True,
    'perf_log_interval': 30,
}


class PerceptionNode(Node):
    """ROS 2 wrapper around the person re-id pipeline."""

    def __init__(self) -> None:
        super().__init__('perception_node')

        # ---- ROS 2 parameters (declared first so we can read them in load_config) ----
        self.declare_parameter('config_file', '')
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('debug_image_topic', '/yolo/debug_image')
        self.declare_parameter('target_topic', '/tracked_target')
        self.declare_parameter('identity_topic', '/person_reid/identity')
        self.declare_parameter('status_topic', '/person_reid/status')
        self.declare_parameter('frame_id', 'camera_optical_frame')
        self.declare_parameter('target_fps', 5.0)

        # Image preprocessing
        self.declare_parameter('enable_image_preprocessing', True)
        self.declare_parameter('enable_resize', True)
        self.declare_parameter('target_width', 416)
        self.declare_parameter('target_height', 416)
        self.declare_parameter('enable_bilateral', True)
        self.declare_parameter('bilateral_d', 5)
        self.declare_parameter('bilateral_sigma_color', 50.0)
        self.declare_parameter('bilateral_sigma_space', 50.0)
        self.declare_parameter('enable_clahe', True)
        self.declare_parameter('clahe_clip', 2.0)
        self.declare_parameter('clahe_tile', 8)
        self.declare_parameter('horizontal_fov_rad', 1.39626)
        self.declare_parameter('image_width_hint', 640)

        # Tracker / ReID knobs (mirrors config.yaml)
        for k, v in _TRACKER_DEFAULTS.items():
            self.declare_parameter(k, v)

        # ---- Read parameters ----
        params = {k: self.get_parameter(k).value for k in (
            'image_topic', 'scan_topic', 'debug_image_topic', 'target_topic',
            'identity_topic', 'status_topic', 'frame_id', 'target_fps',
            'enable_image_preprocessing', 'enable_resize', 'target_width',
            'target_height', 'enable_bilateral', 'bilateral_d',
            'bilateral_sigma_color', 'bilateral_sigma_space', 'enable_clahe',
            'clahe_clip', 'clahe_tile', 'horizontal_fov_rad', 'image_width_hint',
            'config_file',
        )}

        # ---- Config + tracker pipeline ----
        self._config = self._load_config()
        self._config.update({k: v for k, v in params.items() if k != 'config_file'})

        self._device = get_device(bool(self._config.get('use_gpu', True)))
        self.get_logger().info(f"Using device: {self._device}")

        self._detector = PoseDetector(
            model_name=self._config['yolo_model'],
            confidence=float(self._config['detection_confidence']),
            person_class_id=int(self._config['person_class_id']),
            device=self._device,
            use_pose=bool(self._config['use_pose']),
        )

        self._tracker = ByteTrackTracker(
            ByteTrackConfig(
                track_high_thresh=float(self._config['track_high_thresh']),
                track_low_thresh=float(self._config['track_low_thresh']),
                new_track_thresh=float(self._config['new_track_thresh']),
                track_buffer=int(self._config['track_buffer']),
                match_thresh=float(self._config['match_thresh']),
                frame_rate=int(self._config['frame_rate']),
            )
        )

        self._face_reid = FaceReID(
            device=self._device,
            enabled=bool(self._config['use_face_reid']),
            model_name=str(self._config['face_model_name']),
            det_size=int(self._config['face_det_size']),
        )

        self._person_reid = PersonReID(
            device=self._device,
            enabled=bool(self._config['use_body_reid']),
            model_name=str(self._config['body_reid_model']),
        )

        self._owner_enrolled = False
        embeddings_path = self._config['embeddings_path']
        if embeddings_exist(embeddings_path):
            face_emb, body_emb = load_owner_embeddings(embeddings_path)
            if face_emb is not None and len(face_emb) > 0 and self._face_reid.enabled:
                self._face_reid.load_owner_gallery(face_emb)
                self.get_logger().info(
                    f"Loaded {len(face_emb)} owner face embeddings."
                )
            if body_emb is not None and len(body_emb) > 0 and self._person_reid.enabled:
                self._person_reid.load_owner_gallery(body_emb)
                self.get_logger().info(
                    f"Loaded {len(body_emb)} owner body embeddings."
                )
            if (face_emb is not None and len(face_emb) > 0) or (
                body_emb is not None and len(body_emb) > 0
            ):
                self._owner_enrolled = True
        else:
            self.get_logger().warning(
                f"Owner not enrolled at {embeddings_path}. "
                "All persons will be labeled UNKNOWN."
            )

        self._identity_mgr = IdentityManager(
            face_reid=self._face_reid if self._face_reid.is_ready else None,
            person_reid=self._person_reid if self._person_reid.is_ready else None,
            face_threshold=float(self._config['face_match_threshold']),
            body_threshold=float(self._config['body_match_threshold']),
            face_mismatch_threshold=float(self._config['face_strong_mismatch_threshold']),
            body_owner_create_threshold=float(self._config['body_owner_create_threshold']),
            body_owner_maintain_threshold=float(self._config['body_owner_maintain_threshold']),
            body_score_margin=float(self._config['body_score_margin']),
            require_face_to_create_owner=bool(
                self._config['require_face_to_create_owner']
            ),
            smoothing_frames=int(self._config['identity_smoothing_frames']),
            owner_confirm_frames=int(self._config['owner_confirm_frames']),
            unknown_confirm_frames=int(self._config['unknown_confirm_frames']),
            face_reid_interval=int(self._config['face_reid_interval']),
            body_reid_interval=int(self._config['body_reid_interval']),
            debug_log_identity_changes=bool(
                self._config['debug_log_identity_changes']
            ),
        )

        # ---- Image preprocessing + scan helper + bridge + FPS ----
        self._pre = ImagePreprocessor(
            target_width=int(self._config['target_width']),
            target_height=int(self._config['target_height']),
            enable_resize=bool(self._config['enable_resize']),
            enable_bilateral=bool(self._config['enable_bilateral']),
            bilateral_d=int(self._config['bilateral_d']),
            bilateral_sigma_color=float(self._config['bilateral_sigma_color']),
            bilateral_sigma_space=float(self._config['bilateral_sigma_space']),
            enable_clahe=bool(self._config['enable_clahe']),
            clahe_clip=float(self._config['clahe_clip']),
            clahe_tile=int(self._config['clahe_tile']),
        )

        self._scan_helper = ScanDistanceHelper(
            image_width=int(self._config['image_width_hint']),
            horizontal_fov_rad=float(self._config['horizontal_fov_rad']),
        )

        self._bridge = CvBridge()
        self._fps = FPSCounter()
        self._frame_count = 0
        self._n_detections_ema = 0.0
        self._n_tracks_ema = 0.0
        self._last_image_shape = (0, 0)

        # Cached state for the rate-decoupled target/status publishers.
        self._last_target: Optional[PersonFollowingOutput] = None
        self._last_target_distance: Optional[float] = None
        self._last_target_angle: float = 0.0
        self._last_known_angle: float = 0.0
        self._last_target_stamp = None
        self._identity_state: Dict[int, str] = {}

        # ---- ROS 2 wiring ----
        image_topic = str(params['image_topic'])
        scan_topic = str(params['scan_topic'])
        debug_topic = str(params['debug_image_topic'])
        target_topic = str(params['target_topic'])
        identity_topic = str(params['identity_topic'])
        status_topic = str(params['status_topic'])
        frame_id = str(params['frame_id'])
        self._frame_id = frame_id

        # Subscribe to topics with internal names that are remapped by the
        # launch file, so a user can override the topic names without
        # changing the node code.
        self._image_sub = self.create_subscription(
            Image, 'image_raw', self.image_cb, 1,
        )
        self._scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_cb, 10,
        )

        self._pub_image = self.create_publisher(Image, debug_topic, 1)
        self._pub_target = self.create_publisher(String, target_topic, 10)
        self._pub_identity = self.create_publisher(String, identity_topic, 10)
        self._pub_status = self.create_publisher(DiagnosticArray, status_topic, 10)

        # Decoupled target/status timers so the JSON rate and diagnostics rate
        # stay stable even when the camera is bursty.
        target_fps = max(0.1, float(params['target_fps']))
        self._target_timer = self.create_timer(
            1.0 / target_fps, self.target_timer_cb,
        )
        self._status_timer = self.create_timer(1.0, self.status_timer_cb)

        self.get_logger().info(
            f"PerceptionNode ready. image={image_topic} scan={scan_topic} "
            f"debug={debug_topic} target={target_topic}"
        )

    # ------------------------------------------------------------------
    # Config
    # ------------------------------------------------------------------
    def _load_config(self) -> dict:
        """Load YAML config if provided, else start from defaults."""
        path = self.get_parameter('config_file').get_parameter_value().string_value
        if path:
            try:
                return dict(load_config(path))
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(
                    f"Failed to load config_file={path}: {exc}. Using defaults."
                )
        return dict(_TRACKER_DEFAULTS)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def scan_cb(self, msg: LaserScan) -> None:
        self._scan_helper.update(msg)

    def image_cb(self, msg: Image) -> None:
        frame, header = self._pre.preprocess_msg(msg, self._bridge)
        if frame is None:
            return

        t0 = time.perf_counter()
        self._last_image_shape = frame.shape[:2]

        det_result = self._detector.detect(frame)
        dets = det_result.detections
        if dets:
            bboxes = (
                np.stack([d.bbox for d in dets], axis=0).astype(np.float32)
            )
            scores = np.array(
                [d.confidence for d in dets], dtype=np.float32,
            )
            keypoints = [d.keypoints for d in dets]
        else:
            bboxes = np.empty((0, 4), dtype=np.float32)
            scores = np.empty((0,), dtype=np.float32)
            keypoints = []

        tracks = self._tracker.update(bboxes, scores, keypoints)
        identity_results = self._identity_mgr.update(frame, tracks)
        target = select_owner_target(identity_results)

        distance: Optional[float]
        angle: float
        if target is not None:
            u = int(round(target.center_x))
            distance = self._scan_helper.distance_from_scan(u)
            angle = (u / max(1, frame.shape[1]) - 0.5) * float(
                self._config['horizontal_fov_rad']
            )
        else:
            distance = None
            angle = 0.0

        # Visualization (re-uses the existing tracker module).
        viz_config = {
            'show_pose': bool(self._config['use_pose']),
            'show_track_id': True,
            'show_fps': True,
            'bbox_color_owner': (0, 255, 0),
            'bbox_color_unknown': (0, 165, 255),
        }
        vis = draw_tracks(
            frame, identity_results, viz_config, fps=self._fps.fps,
        )
        if target is not None:
            cv2.circle(
                vis,
                (int(round(target.center_x)), int(round(target.center_y))),
                6,
                (0, 255, 0),
                -1,
            )

        # Publish annotated image.
        out_msg = self._bridge.cv2_to_imgmsg(vis, encoding='bgr8')
        out_msg.header = header
        self._pub_image.publish(out_msg)

        # Cache the latest target for the rate-decoupled target timer.
        self._last_target = target
        self._last_target_distance = distance
        self._last_target_angle = angle
        self._last_target_stamp = (
            header.stamp if header is not None else None
        )
        if target is not None:
            self._last_known_angle = angle

        # Identity-change events.
        for result in identity_results:
            prev = self._identity_state.get(result.track_id)
            cur = result.label.value
            if prev != cur:
                self._publish_identity_event(
                    result.track_id,
                    prev,
                    cur,
                    result.face_score,
                    result.body_score,
                    result.face_detected,
                )
                self._identity_state[result.track_id] = cur

        self._n_detections_ema = 0.9 * self._n_detections_ema + 0.1 * len(dets)
        self._n_tracks_ema = 0.9 * self._n_tracks_ema + 0.1 * len(tracks)
        self._fps.tick(time.perf_counter() - t0)
        self._frame_count += 1

        perf_interval = int(self._config.get('perf_log_interval', 30))
        if perf_interval > 0 and self._frame_count % perf_interval == 0:
            dt = time.perf_counter() - t0
            self.get_logger().info(
                (
                    "Perf frame=%d device=%s total=%.1fms fps=%.2f "
                    "dets=%.1f tracks=%.1f"
                ),
                self._frame_count,
                self._device,
                dt * 1000.0,
                self._fps.fps,
                self._n_detections_ema,
                self._n_tracks_ema,
            )

    def _publish_identity_event(
        self,
        track_id: int,
        prev_label: Optional[str],
        new_label: str,
        face_score: float,
        body_score: float,
        face_detected: bool,
    ) -> None:
        payload = json.dumps(
            {
                'track_id': int(track_id),
                'prev_label': prev_label or '',
                'new_label': new_label,
                'face_score': float(face_score),
                'body_score': float(body_score),
                'face_detected': bool(face_detected),
                'stamp': self.get_clock().now().to_msg().sec,
            }
        )
        self._pub_identity.publish(String(data=payload))

    # ------------------------------------------------------------------
    # Timers
    # ------------------------------------------------------------------
    def _compute_status(self) -> str:
        if not self._owner_enrolled:
            return STATUS_NO_OWNER
        if self._last_target is not None:
            return STATUS_VISIBLE
        if self._last_known_angle != 0.0:
            return STATUS_LOST
        return STATUS_SEARCHING

    def target_timer_cb(self) -> None:
        status = self._compute_status()
        payload = build_target_payload_full(
            target=self._last_target,
            status=status,
            last_known_angle=self._last_known_angle,
            fps=self._fps.fps,
            n_detections=int(round(self._n_detections_ema)),
            n_tracks=int(round(self._n_tracks_ema)),
            header_stamp=self._last_target_stamp,
            frame_id=self._frame_id,
            distance=self._last_target_distance,
            angle=self._last_target_angle,
        )
        self._pub_target.publish(String(data=payload))

    def status_timer_cb(self) -> None:
        arr = build_diagnostics(
            node_name=self.get_name(),
            fps=self._fps.fps,
            n_detections_avg=self._n_detections_ema,
            n_tracks_avg=self._n_tracks_ema,
            owner_visible=self._last_target is not None,
            preprocessing_enabled={
                'enable_resize': bool(self._config['enable_resize']),
                'enable_bilateral': bool(self._config['enable_bilateral']),
                'enable_clahe': bool(self._config['enable_clahe']),
                'target_size': (
                    f"{int(self._config['target_width'])}x"
                    f"{int(self._config['target_height'])}"
                ),
            },
            image_width=int(self._last_image_shape[1]),
            image_height=int(self._last_image_shape[0]),
        )
        arr.header.stamp = self.get_clock().now().to_msg()
        self._pub_status.publish(arr)


def main(args: Optional[List[str]] = None) -> None:
    setup_logging()
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001
        logger.exception("Fatal error in perception_node: %s", exc)
    finally:
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
