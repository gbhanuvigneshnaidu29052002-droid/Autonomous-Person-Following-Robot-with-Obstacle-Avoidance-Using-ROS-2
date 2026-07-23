# Person ReID Tracker

Live person re-identification prototype for laptop webcam testing, designed for future migration to a **ROS 2** person-following node on **TurtleBot3 Waffle Pi**.

Enroll yourself as the **owner**, then run live tracking. The system uses **face ReID** when your face is visible, **body ReID** when it is not, and **ByteTrack** to keep stable track IDs across frames.

## Features

- YOLOv8 pose detection (persons only)
- ByteTrack multi-object tracking
- Face embedding matching (InsightFace preferred)
- Body/person ReID (OSNet via torchreid, MobileNet fallback)
- Identity fusion with temporal smoothing
- Live visualization with pose skeleton, track IDs, OWNER/UNKNOWN labels, FPS
- ROS 2 adapter stub for future robot integration

## Project structure

```
person_reid_tracker/
├── main.py                 # Live webcam mode
├── enroll_owner.py         # Owner enrollment
├── config.yaml             # All tunable parameters
├── requirements.txt
├── models/                 # Optional local model weights
├── data/
│   ├── owner/              # Sample enrollment images
│   └── embeddings/         # owner_embeddings.npz
└── src/
    ├── detector_pose.py
    ├── bytetrack_tracker.py
    ├── face_reid.py
    ├── person_reid.py
    ├── identity_manager.py
    ├── visualization.py
    ├── utils.py
    └── ros2_adapter_stub.py
```

## Setup

### 1. Create a virtual environment (recommended)

```bash
cd person_reid_tracker
python -m venv .venv

# Windows
.venv\Scripts\activate

# Linux / macOS
source .venv/bin/activate
```

### 2. Install dependencies

```bash
pip install -r requirements.txt
```

**Optional – better face ReID:**

```bash
pip install insightface onnxruntime
```

**Optional – better body ReID (OSNet):**

```bash
pip install torchreid
```

On first run, Ultralytics will auto-download `yolov8n-pose.pt`.

### NVIDIA GPU setup

For CUDA PyTorch:

```bash
pip uninstall -y torch torchvision torchaudio
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121
```

For InsightFace GPU acceleration:

```bash
pip uninstall -y onnxruntime
pip install onnxruntime-gpu
```

Check the environment:

```bash
python check_gpu.py
```

If GPU is still unavailable:

- Run `nvidia-smi`
- Update the NVIDIA driver
- Confirm the installed PyTorch wheel includes CUDA
- Confirm `onnxruntime-gpu` is installed

### 3. Enroll yourself

```bash
python enroll_owner.py
```

Stand in front of the webcam with face and full body visible. Move slightly for variety. Samples are saved to `data/embeddings/owner_embeddings.npz`.

### 4. Run live tracking

```bash
python main.py
```

**Keyboard controls:**

| Key | Action |
|-----|--------|
| `q` | Quit |
| `r` | Reset ByteTrack and identity history |
| `s` | Save debug frame |

## How the pieces fit together

### ByteTrack (tracking)

ByteTrack assigns a **stable numeric track ID** to each person based on motion and bounding-box overlap. It keeps IDs consistent when detections briefly drop or confidence fluctuates. ByteTrack does **not** know who someone is—it only answers “which box is the same person as before?”

### Face ReID (who is this – face visible)

When a face is detected inside a person box, its embedding is compared to your enrolled face gallery. Face matching has **highest priority** because faces are highly discriminative.

### Body ReID (who is this – face hidden)

When the face is occluded or turned away, a full-body crop embedding is compared to enrolled body samples. This helps follow you from behind or in profile.

### Identity fusion (`identity_manager.py`)

Per track ID, the system combines:

1. Face similarity (if face visible)
2. Body similarity (always computed when enabled)
3. Temporal smoothing (streak counters)
4. Strong face **mismatch veto** (won’t keep calling someone OWNER if face clearly isn’t yours)

Output label: **OWNER** or **UNKNOWN**.

## Configuration

Edit `config.yaml` for:

- `camera_index`, resolution
- `yolo_model`, `detection_confidence`
- `face_match_threshold`, `body_match_threshold`
- `use_face_reid`, `use_body_reid`, `use_pose`, `use_gpu`
- ByteTrack and smoothing parameters

## Graceful degradation

| Missing component | Behavior |
|-------------------|----------|
| Owner not enrolled | Warning + all persons labeled UNKNOWN |
| insightface / face_recognition | Falls back to OpenCV Haar (weak) or disables face ReID |
| torchreid | Falls back to MobileNetV3 body features |
| GPU | Runs on CPU (slower) |

If ReID dependencies are missing entirely, detection + pose + ByteTrack still run.

## Limitations

- **Lighting changes** – embeddings drift under very different lighting
- **Occlusion** – heavy occlusion reduces face/body match quality
- **Similar clothing** – body ReID can confuse people in similar outfits
- **Face not visible** – relies on body ReID, which is less reliable
- **Raspberry Pi** – expect low FPS; use `yolov8n-pose.pt`, disable pose drawing, consider TensorRT/NCNN export later

## Future ROS 2 migration

`src/ros2_adapter_stub.py` defines `PersonFollowingOutput`:

```python
@dataclass
class PersonFollowingOutput:
    track_id: int
    label: str
    bbox: np.ndarray
    center_x: float
    center_y: float
    depth_estimate: Optional[float]  # from RealSense / depth camera
    confidence: float
    pose_keypoints: Optional[np.ndarray]
```

`select_owner_target()` picks the current OWNER track for the robot to follow.

**Suggested migration steps:**

1. Wrap `build_pipeline()` logic in a `rclpy` node
2. Subscribe to `/camera/image_raw` instead of `cv2.VideoCapture`
3. Publish `PersonFollowingOutput` as a custom message or `geometry_msgs/PointStamped` (image x/y)
4. Add depth from `/camera/depth/image_rect_raw` for `depth_estimate`
5. Run perception on Pi; optionally offload YOLO to Coral NPU or remote inference
6. Connect output to TurtleBot3 `follower` / `cmd_vel` controller

## License

Prototype code for research and development. Check licenses for YOLO, InsightFace, torchreid, and ByteTrack when deploying.
