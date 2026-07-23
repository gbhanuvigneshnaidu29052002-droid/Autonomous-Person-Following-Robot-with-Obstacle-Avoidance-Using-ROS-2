"""
ROS 2 adapter stub – placeholder for future TurtleBot3 integration.

This module defines a clean output structure that main.py can populate today
and a ROS 2 node can publish tomorrow (e.g. geometry_msgs/Point, custom msg, tf).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

import numpy as np

from src.identity_manager import IdentityLabel, IdentityResult


@dataclass
class PersonFollowingOutput:
    """Target person output for robot following."""

    track_id: int
    label: str
    bbox: np.ndarray  # xyxy in image pixels
    center_x: float
    center_y: float
    depth_estimate: Optional[float]  # placeholder – fill from depth camera in ROS 2
    confidence: float
    pose_keypoints: Optional[np.ndarray]


def bbox_center(bbox: np.ndarray) -> tuple[float, float]:
    x1, y1, x2, y2 = bbox
    return (x1 + x2) / 2.0, (y1 + y2) / 2.0


def identity_result_to_following_output(result: IdentityResult) -> PersonFollowingOutput:
    cx, cy = bbox_center(result.bbox)
    return PersonFollowingOutput(
        track_id=result.track_id,
        label=result.label.value,
        bbox=result.bbox.copy(),
        center_x=cx,
        center_y=cy,
        depth_estimate=None,
        confidence=result.confidence,
        pose_keypoints=result.keypoints.copy() if result.keypoints is not None else None,
    )


def select_owner_target(results: List[IdentityResult]) -> Optional[PersonFollowingOutput]:
    """
    Convert the current OWNER track into a PersonFollowingOutput for the robot.

    Returns None if no OWNER is visible.
    """
    owners = [r for r in results if r.label == IdentityLabel.OWNER]
    if not owners:
        return None
    best = max(owners, key=lambda r: r.confidence)
    return identity_result_to_following_output(best)


# ---------------------------------------------------------------------------
# Future ROS 2 node sketch (not implemented):
#
#   class PersonFollowerNode(Node):
#       def __init__(self):
#           super().__init__('person_follower')
#           self.pub = self.create_publisher(PointStamped, '/target_person', 10)
#
#       def on_frame(self, results: List[IdentityResult], frame_header):
#           target = select_owner_target(results)
#           if target is None:
#               return
#           msg = PointStamped()
#           msg.header = frame_header
#           msg.point.x = target.center_x
#           msg.point.y = target.center_y
#           msg.point.z = target.depth_estimate or 0.0
#           self.pub.publish(msg)
# ---------------------------------------------------------------------------
