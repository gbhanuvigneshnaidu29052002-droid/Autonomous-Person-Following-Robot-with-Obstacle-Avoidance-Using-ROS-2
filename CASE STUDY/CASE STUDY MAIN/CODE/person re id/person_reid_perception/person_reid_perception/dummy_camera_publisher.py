"""Synthetic camera publisher for desk/Gazebo smoke tests.

Publishes ``sensor_msgs/Image`` messages to ``/camera/camera/color/image_raw``
so the perception node can be exercised without a real camera. Two modes:

* **Synthetic** (default): a white frame with a moving gray rectangle that
  imitates a person walking across the field of view.
* **Replay**: cycle through a directory of images. Useful for recording a
  test sequence once and replaying it deterministically.
"""

from __future__ import annotations

import os
from glob import glob
from typing import List, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header


class DummyCameraPublisher(Node):
    """A minimal synthetic-camera ROS 2 node."""

    def __init__(self) -> None:
        super().__init__('dummy_camera_publisher')

        self.declare_parameter('publish_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('fps', 15.0)
        self.declare_parameter('image_dir', '')
        self.declare_parameter('frame_id', 'camera_optical_frame')

        topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        width = int(self.get_parameter('image_width').value)
        height = int(self.get_parameter('image_height').value)
        fps = float(self.get_parameter('fps').value)
        image_dir = self.get_parameter('image_dir').get_parameter_value().string_value
        self._frame_id = (
            self.get_parameter('frame_id').get_parameter_value().string_value
            or 'camera_optical_frame'
        )

        self._width = width
        self._height = height
        self._frame_idx = 0
        self._bridge = CvBridge()

        self._replay_files: List[str] = []
        if image_dir:
            self._replay_files = sorted(
                f
                for f in glob(os.path.join(image_dir, '*'))
                if f.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp'))
            )
            if not self._replay_files:
                self.get_logger().warn(
                    f"No images found in {image_dir}; falling back to synthetic."
                )
                self._replay_files = []

        self._pub = self.create_publisher(Image, topic, 10)
        period = 1.0 / max(fps, 0.1)
        self._timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f"DummyCameraPublisher -> {topic} @ {fps} Hz, "
            f"{width}x{height}, replay={'yes' if self._replay_files else 'no'}"
        )

    def _synth_frame(self, idx: int) -> np.ndarray:
        frame = np.full((self._height, self._width, 3), 245, dtype=np.uint8)
        # A gray "person" rectangle drifting across the frame, bouncing back.
        margin_x = 40
        period = self._width - 2 * margin_x - 80
        if period > 0:
            phase = (idx % (2 * period))
            x = margin_x + (phase if phase < period else (2 * period - phase))
        else:
            x = margin_x
        cv2.rectangle(
            frame,
            (int(x), self._height // 4),
            (int(x) + 80, 3 * self._height // 4),
            (110, 110, 110),
            thickness=-1,
        )
        # Bottom caption for visual debugging.
        cv2.putText(
            frame,
            f"dummy camera  t={idx}",
            (10, self._height - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (50, 50, 50),
            1,
            cv2.LINE_AA,
        )
        return frame

    def _replay_frame(self, idx: int) -> Optional[np.ndarray]:
        if not self._replay_files:
            return None
        path = self._replay_files[idx % len(self._replay_files)]
        img = cv2.imread(path, cv2.IMREAD_COLOR)
        if img is None:
            self.get_logger().warn(f"cv2.imread returned None for {path}")
            return None
        if img.shape[1] != self._width or img.shape[0] != self._height:
            img = cv2.resize(
                img, (self._width, self._height), interpolation=cv2.INTER_AREA,
            )
        return img

    def _tick(self) -> None:
        if self._replay_files:
            frame = self._replay_frame(self._frame_idx)
            if frame is None:
                frame = self._synth_frame(self._frame_idx)
        else:
            frame = self._synth_frame(self._frame_idx)

        msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        self._pub.publish(msg)
        self._frame_idx += 1


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = DummyCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
