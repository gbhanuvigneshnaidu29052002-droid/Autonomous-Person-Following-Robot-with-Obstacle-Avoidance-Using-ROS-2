"""Map an image-column target to a LiDAR-estimated distance.

The Waffle Pi's LDS-01 / LD08 LIDAR sweeps the same base_link plane the
camera looks out from. This module keeps the most recent LaserScan in
memory and, when given an image column ``u``, returns the median range
measured at that bearing (within a small angular window).

The mapping assumes:

* Camera optical frame ``+x`` is to the right in the image.
* LiDAR ``angle = 0`` is straight ahead, ``+angle`` is to the left
  (matches the Waffle Pi bringup, where the LDS sits behind the camera
  and the published scan is in the ``base_scan`` frame, which is the
  standard REP-105 ``base_link`` x-forward / z-up convention). For the
  purpose of the ``angle`` field reported to the follower, the sign is
  flipped so that ``+angle`` means "owner to the right of frame" — which
  is what the follower wants.
"""

from __future__ import annotations

import math
from typing import Optional

# Angular window used when sampling the scan, in radians.
_WINDOW_RAD = math.radians(2.0)


class ScanDistanceHelper:
    """Look up a distance from the most recent LaserScan."""

    def __init__(
        self,
        image_width: int = 640,
        horizontal_fov_rad: float = 1.39626,
    ) -> None:
        self.image_width = max(1, int(image_width))
        self.horizontal_fov_rad = float(horizontal_fov_rad)
        self._last_scan = None
        self._last_stamp = None

    def update(self, scan) -> None:
        """Store the most recent LaserScan for later queries."""
        self._last_scan = scan
        self._last_stamp = scan.header.stamp if scan.header is not None else None

    def _column_to_bearing(self, u: int) -> Optional[float]:
        if self.image_width <= 0:
            return None
        if u < 0 or u >= self.image_width:
            return None
        # Map image column to a bearing inside the camera's horizontal FOV.
        # (u / W - 0.5) puts column 0 at -0.5 and column W-1 at +0.5.
        normalized = (u / self.image_width) - 0.5
        return normalized * self.horizontal_fov_rad

    def distance_from_scan(self, u: int) -> Optional[float]:
        """Median range in a small angular window around the column's bearing.

        Returns ``None`` if no usable range is available.
        """
        scan = self._last_scan
        if scan is None or len(scan.ranges) == 0:
            return None

        bearing = self._column_to_bearing(u)
        if bearing is None:
            return None

        # The LDS on the Waffle Pi publishes with angle_min < 0 and
        # angle_max > 0, where 0 is straight ahead. We negate the camera
        # bearing so a column on the right of the image (positive camera
        # bearing) becomes a small positive angle in the scan frame.
        scan_bearing = -bearing

        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        if angle_inc <= 0:
            return None

        center_idx = (scan_bearing - angle_min) / angle_inc
        span = _WINDOW_RAD / angle_inc
        i0 = max(0, int(math.floor(center_idx - span)))
        i1 = min(len(scan.ranges) - 1, int(math.ceil(center_idx + span)))
        if i1 < i0:
            return None

        valid: list[float] = []
        for i in range(i0, i1 + 1):
            r = scan.ranges[i]
            if r is None:
                continue
            if math.isnan(r) or math.isinf(r):
                continue
            if r < scan.range_min or r > scan.range_max:
                continue
            valid.append(float(r))
        if not valid:
            return None
        valid.sort()
        mid = len(valid) // 2
        if len(valid) % 2 == 1:
            return valid[mid]
        return 0.5 * (valid[mid - 1] + valid[mid])
