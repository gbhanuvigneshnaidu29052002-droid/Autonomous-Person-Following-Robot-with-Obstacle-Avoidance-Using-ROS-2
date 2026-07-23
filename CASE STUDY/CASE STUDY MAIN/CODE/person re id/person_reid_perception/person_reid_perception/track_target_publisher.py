"""Build the /tracked_target JSON payload and the /person_reid/status array.

The JSON schema for ``/tracked_target`` is the contract with the downstream
follower node. Keeping it as a single function makes it easy to keep the
schema in one place and to test it independently of the live node.
"""

from __future__ import annotations

import json
from typing import Optional

import numpy as np

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

# Status values used in the /tracked_target payload.
STATUS_VISIBLE = 'visible'
STATUS_SEARCHING = 'searching'
STATUS_LOST = 'lost'
STATUS_NO_OWNER = 'no_owner_enrolled'
_VALID_STATUSES = {STATUS_VISIBLE, STATUS_SEARCHING, STATUS_LOST, STATUS_NO_OWNER}


def build_target_payload_full(
    target,
    status: str,
    last_known_angle: float,
    fps: float,
    n_detections: int,
    n_tracks: int,
    header_stamp,
    frame_id: str,
    distance,
    angle: float,
) -> str:
    """Serialize the full target payload as a JSON string.

    ``target`` is a ``PersonFollowingOutput`` (or ``None``). ``status`` must
    be one of ``STATUS_*``. ``header_stamp`` is a ``builtin_interfaces/Time``
    instance; ``None`` is allowed for tests.
    """
    if status not in _VALID_STATUSES:
        status = STATUS_SEARCHING

    if target is not None:
        bbox = [float(v) for v in np.asarray(target.bbox).flatten().tolist()]
        if len(bbox) != 4:
            bbox = [0.0, 0.0, 0.0, 0.0]
        track_id = int(target.track_id)
        label = str(target.label) if target.label is not None else ''
        center_x = float(target.center_x)
        center_y = float(target.center_y)
        confidence = float(target.confidence)
    else:
        bbox = [0.0, 0.0, 0.0, 0.0]
        track_id = 0
        label = ''
        center_x = 0.0
        center_y = 0.0
        confidence = 0.0

    visible = target is not None and status == STATUS_VISIBLE

    if header_stamp is not None:
        stamp = {
            'sec': int(getattr(header_stamp, 'sec', 0)),
            'nanosec': int(getattr(header_stamp, 'nanosec', 0)),
        }
    else:
        stamp = {'sec': 0, 'nanosec': 0}

    payload = {
        'header': {'stamp': stamp, 'frame_id': str(frame_id or '')},
        'visible': bool(visible),
        'status': status,
        'track_id': track_id,
        'label': label,
        'bbox': bbox,
        'center_x': center_x,
        'center_y': center_y,
        'distance': (None if distance is None else float(distance)),
        'angle': float(angle),
        'last_known_angle': float(last_known_angle),
        'confidence': confidence,
        'fps': float(fps),
        'n_detections': int(n_detections),
        'n_tracks': int(n_tracks),
    }
    return json.dumps(payload)


def _kv(key: str, value) -> KeyValue:
    msg = KeyValue()
    msg.key = str(key)
    msg.value = '' if value is None else str(value)
    return msg


def build_diagnostics(
    node_name: str,
    fps: float,
    n_detections_avg: float,
    n_tracks_avg: float,
    owner_visible: bool,
    preprocessing_enabled: dict,
    image_width: int = 0,
    image_height: int = 0,
) -> DiagnosticArray:
    """Build a DiagnosticArray for the 1 Hz status topic."""
    arr = DiagnosticArray()
    stamp = None  # caller fills in via arr.header.stamp

    def _status(name: str, level: int, message: str, kv_pairs) -> DiagnosticStatus:
        s = DiagnosticStatus()
        s.name = name
        s.hardware_id = node_name
        s.level = level
        s.message = message
        s.values = list(kv_pairs)
        return s

    owner_level = DiagnosticStatus.OK if owner_visible else DiagnosticStatus.WARN
    owner_msg = 'OWNER visible' if owner_visible else 'OWNER not visible'

    arr.status = [
        _status(
            f'{node_name}/detector',
            DiagnosticStatus.OK,
            'detector running',
            [
                _kv('fps', f'{fps:.2f}'),
                _kv('avg_detections', f'{n_detections_avg:.2f}'),
                _kv('image_width', image_width),
                _kv('image_height', image_height),
            ],
        ),
        _status(
            f'{node_name}/tracker',
            DiagnosticStatus.OK,
            'tracker running',
            [_kv('avg_tracks', f'{n_tracks_avg:.2f}')],
        ),
        _status(
            f'{node_name}/reid',
            owner_level,
            owner_msg,
            [_kv('owner_visible', owner_visible)],
        ),
        _status(
            f'{node_name}/image_preprocessor',
            DiagnosticStatus.OK,
            'preprocessor running',
            [_kv(k, v) for k, v in preprocessing_enabled.items()],
        ),
    ]
    return arr
