# Person-Following Robot — Full Project Documentation
## TurtleBot3 Waffle Pi | ROS 2 Humble | YOLOv8 + ByteTrack + Multi-Level Re-ID

---

> **Project:** Autonomous Person-Following Robot with Obstacle Avoidance and Re-Identification  
> **Platform:** TurtleBot3 Waffle Pi (real hardware)  
> **Framework:** ROS 2 Humble (Python 3.10)  
> **Date:** July 2026  
> **Status:** Fully operational on real robot  

---

## Table of Contents

1. [Problem Statement](#1-problem-statement)
2. [System Architecture](#2-system-architecture)
3. [Hardware Setup](#3-hardware-setup)
4. [ROS 2 Node Overview](#4-ros-2-node-overview)
5. [Algorithm Deep Dive — Perception Node](#5-algorithm-deep-dive--perception-node)
6. [Algorithm Deep Dive — Behavior Tree Node](#6-algorithm-deep-dive--behavior-tree-node)
7. [Algorithm Deep Dive — Bypass Follower Node](#7-algorithm-deep-dive--bypass-follower-node)
8. [Algorithm Deep Dive — Manager Node](#8-algorithm-deep-dive--manager-node)
9. [Re-ID System (Embedded from person_reid_tracker)](#9-re-id-system-embedded-from-person_reid_tracker)
10. [Sensor Fusion — Camera + LiDAR](#10-sensor-fusion--camera--lidar)
11. [Obstacle Avoidance — Potential Field Method](#11-obstacle-avoidance--potential-field-method)
12. [LiDAR Noise Filter](#12-lidar-noise-filter)
13. [Specific Scenario — Bypass Follower](#13-specific-scenario--bypass-follower)
14. [Dashboard Integration](#14-dashboard-integration)
15. [Key Engineering Decisions & Bugs Fixed](#15-key-engineering-decisions--bugs-fixed)
16. [Full Code Annotations](#16-full-code-annotations)
17. [Conversation Journey (Session Summary)](#17-conversation-journey-session-summary)
18. [Run Commands](#18-run-commands)
19. [Glossary](#19-glossary)

---

## 1. Problem Statement

### Challenge
Build a real robot (TurtleBot3 Waffle Pi) that:
1. **Identifies** a specific person from a crowd using a camera
2. **Follows** that person while maintaining a safe following distance (~0.85 m)
3. **Avoids obstacles** (walls, other robots, furniture) in real-time using LiDAR
4. **Recovers** when the person is temporarily hidden by an obstacle
5. **Re-identifies** the person if lost (ID swap, occlusion, ByteTrack failure)
6. **Performs a search** (360° spin + navigation to last known position) if completely lost

### Specific Test Scenario
```
[ROBOT] ──── 80 cm ──── [OBSTACLE ROBOT] ──── 20 cm ──── [HUMAN]
← ─ ─ ─ ─ ─ ─ ─ ─ 1 metre total ─ ─ ─ ─ ─ ─ ─ ─ ─ →
```
The robot must:
- Detect the obstacle at ≤ 0.80 m
- Choose the clearer side (left or right) automatically
- Bypass the obstacle
- Re-acquire the human and follow them

### Real-World Constraints
- TurtleBot3 hardware: OpenCR motor controller, Raspberry Pi compute
- LiDAR: RPLidar A1 (BEST_EFFORT QoS)
- Camera: Intel RealSense D435i (RGB stream)
- No GPU on the robot → all inference on CPU
- Python 3.10 + ROS 2 Humble Hawksbill
- Power brownouts possible during aggressive manoeuvres

---

## 2. System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     ROS 2 Topic Graph                           │
│                                                                 │
│  /camera/camera/color/image_raw ──→ [perception_node]           │
│  /scan ──────────────────────────→ [perception_node]            │
│                                          │                      │
│                                          ↓ /tracked_target      │
│  [manager_node] ──/perception/command──→ [perception_node]      │
│  [manager_node] ──/behavior/command───→ [behavior_tree_node]    │
│                                                                 │
│  /tracked_target ─────────────────→ [behavior_tree_node]        │
│  /scan ────────────────────────────→ [behavior_tree_node]        │
│  /odom ────────────────────────────→ [behavior_tree_node]        │
│  [behavior_tree_node] ─→ /cmd_vel ─→ [TurtleBot3 Motors]        │
│  [behavior_tree_node] ─→ /person_following/behavior_state       │
│                                                                 │
│  [dashboard] ──/behavior/command──→ [behavior_tree_node]        │
│  [dashboard] ←── all topics ──────  (monitoring/display)        │
└─────────────────────────────────────────────────────────────────┘
```

### Data Flow Summary
```
Camera Frame → YOLO Detect → ByteTrack → Re-ID Match → /tracked_target (JSON)
LiDAR Scan  → Noise Filter → Sector Split → Obstacle Detection
/tracked_target + /scan + /odom → Behavior State Machine → /cmd_vel
```

---

## 3. Hardware Setup

| Component | Spec |
|---|---|
| Robot | TurtleBot3 Waffle Pi |
| CPU | Raspberry Pi 4 (robot) + Intel i5/i7 (laptop, runs perception) |
| Motor controller | OpenCR 1.0 |
| LiDAR | RPLidar A1M8 (360°, 12m range, 10Hz) |
| Camera | Intel RealSense D435i |
| Camera topics | `/camera/camera/color/image_raw` |
| LiDAR topic | `/scan` (QoS: BEST_EFFORT, VOLATILE) |
| Odometry topic | `/odom` |
| Motor command | `/cmd_vel` |

---

## 4. ROS 2 Node Overview

| Node | File | Purpose |
|---|---|---|
| `perception_node` | `perception_node.py` | YOLO detection + ByteTrack + Multi-level Re-ID + LiDAR fusion |
| `behavior_tree_node` | `behavior_tree_node.py` | State machine: WAIT/FOLLOW/AVOID/REROUTE/SEARCH/ACQUIRE |
| `bypass_follower_node` | `bypass_follower_node.py` | Specific scenario: bypass obstacle in a line |
| `manager_node` | `manager_node.py` | Lifecycle coordinator: triggers TRAIN then START |

### Topic Map

| Topic | Type | Publisher | Subscribers |
|---|---|---|---|
| `/tracked_target` | `String` (JSON) | perception_node | behavior_tree_node, manager_node, dashboard |
| `/behavior/command` | `String` | manager_node, dashboard | behavior_tree_node, bypass_follower_node |
| `/perception/command` | `String` | manager_node | perception_node |
| `/cmd_vel` | `Twist` | behavior_tree_node | TurtleBot3 motors |
| `/scan` | `LaserScan` | TurtleBot3 hardware | perception_node (bearing calc), behavior_tree_node |
| `/odom` | `Odometry` | TurtleBot3 hardware | behavior_tree_node |
| `/yolo/debug_image` | `Image` | perception_node | dashboard (RViz/display) |
| `/person_following/behavior_state` | `String` | behavior_tree_node | dashboard |

---

## 5. Algorithm Deep Dive — Perception Node

### 5.1 Overview

The perception node is responsible for:
1. Running object detection on every camera frame
2. Assigning persistent IDs to people using ByteTrack
3. Matching the tracked ID to the enrolled "owner" using multi-level Re-ID
4. Fusing camera angle data with LiDAR ranges for accurate distance measurement
5. Publishing a JSON payload on `/tracked_target`

### 5.2 Training Phase (TRAINING Mode)

```
TRAINING MODE (first TRAIN_FRAMES=40 frames)
════════════════════════════════════════════════════════
For each camera frame:
  1. Run YOLO → get all person detections
  2. Pick the MOST CENTRED person in the frame
     (Heuristic: person standing in front = centre of camera)
  3. Crop that person's image with 4% padding on all sides
  4. Extract BODY EMBEDDING using MobileNetV3 (or OSNet)
  5. Compute HSV HISTOGRAM of the crop (H-S plane, 36×32 bins)
  6. Store both in lists

After 40 frames:
  → body_gallery = list of 40 body embedding vectors (512-dim)
  → hsv_fingerprint = MEAN of all 40 HSV histograms (averaged)
  → target_id = ByteTrack ID of the enrolled person
  → MODE → TRACKING
```

**Why average HSV histograms?**  
A single frame is noisy (motion blur, partial occlusion). Averaging 40 frames creates a robust colour fingerprint that captures the person's overall appearance across slight viewpoint and lighting changes.

**Why MobileNetV3?**  
It is a lightweight convolutional neural network that extracts 576-dimensional semantic feature vectors from a person's full-body crop. These are far more discriminative than raw colour information and are invariant to small viewpoint changes.

### 5.3 Multi-Level Re-ID (TRACKING Mode)

The system uses a 4-level cascade. Each level is tried in order; if a match is found, lower levels are skipped.

```
LEVEL 1 — ByteTrack ID Continuity (O(n), no extra compute)
───────────────────────────────────────────────────────────
  for p in detected_persons:
      if p.track_id == self.target_id:
          return p  # MATCH FOUND

  Fails when: person is occluded for long enough that ByteTrack drops the ID
              (ByteTrack has a configurable buffer of ~30 frames)

LEVEL 2 — Deep Body Embedding (MobileNetV3 cosine similarity)
──────────────────────────────────────────────────────────────
  for each person detected:
      crop = extract person image from frame
      embedding = MobileNetV3.embed(crop)          # 576-dim vector
      sim = max(cosine_similarity(embedding, g)    # compare to gallery
                for g in body_gallery)
  
  if best_sim >= BODY_REID_THRESH (0.52):
      re-lock target_id = best person's ByteTrack ID
      return person  # MATCH FOUND

  cosine_similarity(a, b) = dot(a, b) / (||a|| * ||b||)
  Range: [0 = completely different, 1 = identical]

LEVEL 3 — HSV Histogram (Bhattacharyya distance)
─────────────────────────────────────────────────
  for each person detected:
      crop = extract person image
      hist = calc_HSV_histogram(crop)  # 36x32 bins
      sim = 1 - bhattacharyya(hist, hsv_fingerprint)
  
  if best_sim >= HSV_REID_THRESH (0.50):
      re-lock target_id
      return person  # MATCH FOUND

  Bhattacharyya distance measures overlap between two probability distributions.
  Value 0 = identical distributions, 1 = no overlap.
  We invert it: similarity = 1 - distance

LEVEL 4 — Spatial Angle Fallback
─────────────────────────────────
  cx_ref = centre_of_frame + last_known_angle * (image_width / hfov)
  
  if only 1 person in frame:
      re-lock on that person
  elif closest person to cx_ref within 0.25 rad:
      re-lock on that person

  Last resort: if the person reappears roughly where we last saw them,
  assume it's still our target.
```

### 5.4 Gallery Continuous Update

While tracking the confirmed target, the gallery is updated on every frame:

```python
body_gallery.append(new_embedding)
if len(body_gallery) > 20:
    body_gallery = body_gallery[-20:]  # rolling window of last 20

# Exponential Moving Average update of HSV fingerprint (α = 0.10)
hsv_fingerprint = 0.90 * hsv_fingerprint + 0.10 * new_hist
```

This allows the re-ID system to adapt to slow changes in lighting, clothing appearance from different angles, and partial occlusion.

### 5.5 LiDAR Distance Fusion

```python
def _lidar_dist(self, bearing_angle: float, window: int = 8):
    """
    bearing_angle: horizontal angle from camera centre (radians)
    window: number of LiDAR rays to consider on each side
    
    Steps:
    1. Convert camera bearing to LiDAR ray index:
       idx = (bearing_angle - scan.angle_min) / scan.angle_increment
    2. Collect valid ranges in [idx-window, idx+window]
    3. Return MEDIAN of valid ranges (robust to outliers)
    """
```

**Why median?** The LiDAR often returns noisy readings (0.0, inf, or erroneous values) for legs. Taking the median of ±8 rays gives a stable estimate of the person's closest leg distance.

**Fallback:** If LiDAR returns no valid ranges, the focal-length bounding-box formula is used:
```
distance = (person_real_height × focal_length_pixels) / bbox_pixel_height
         = (1.70 m × focal) / bbox_height
```

### 5.6 Published JSON Format

```json
{
  "visible": true,
  "status": "visible",
  "distance": 1.23,
  "angle": -0.15,
  "last_known_angle": -0.15,
  "last_seen_time": 1784291225.4
}
```

- `visible`: Boolean — Is the target actively tracked right now?
- `angle`: Radians — Positive = person to LEFT of robot, Negative = RIGHT
- `distance`: Metres — From LiDAR fusion or bounding box estimate
- `last_known_angle`: Angle when person was last seen (used for search direction)

---

## 6. Algorithm Deep Dive — Behavior Tree Node

### 6.1 State Machine

```
                    ┌─────────┐
          START ──→ │  WAIT   │ ←─────────────────────┐
                    └────┬────┘   elapsed > 20s        │
                         │ target_visible               │
                         ↓                              │
                    ┌─────────┐  obstacle!    ┌────────────┐
               ┌───→│  FOLLOW │──────────────→│   AVOID    │
               │    └────┬────┘               └─────┬──────┘
               │         │ not visible              │ cleared
               │         ↓                          │
               │   ┌───────────┐   timeout/arrived  │
               │   │  REROUTE  │─────────────────┐  │
               │   └─────┬─────┘                 │  │
               │         │ target_visible         ↓  │
               └─────────┘              ┌────────────┐
                                        │   SEARCH   │
                                        └─────┬──────┘
                                              │ elapsed > 12s
                                              ↓
                                        ┌───────────┐
                                        │  ACQUIRE  │
                                        └─────┬─────┘
                                              │ elapsed > 20s
                                              ↑────────────────
```

### 6.2 FOLLOW State — PD Control

```python
# Linear velocity (approach / back up)
if distance > SAFE_DIST + STOP_BAND:       # too far → approach
    lin = KP_LIN × (distance - SAFE_DIST)  # proportional to error
elif distance < BACKUP_DIST:               # too close → back up
    lin = -KP_LIN × (BACKUP_DIST - distance)
else:                                      # in deadband → stop
    lin = 0.0

# Angular velocity (steering)
# angle is positive when person is to the LEFT
# negative angular.z = CW (right), positive = CCW (left)
ang = -KP_ANG × angle    # person left → rotate left (CCW = +z)
```

**Why P-controller?** The TurtleBot3's dynamics are simple enough that a proportional gain gives adequate response. A full PID would need derivative-term tuning that varies with surface friction and battery voltage.

### 6.3 AVOID State — Potential Field (70/30 Blend)

```python
# Obstacle repulsion vector
strength = (OBS_WARN_DIST - closest) / (OBS_WARN_DIST - OBS_STOP_DIST)
strength = clamp(0.0, 1.0, strength)

if left < right:
    rep_ang = -MAX_ANG   # steer right (away from left obstacle)
else:
    rep_ang = +MAX_ANG   # steer left  (away from right obstacle)

# Blend: 70% repulsion, 30% target attraction
final_ang = 0.70 × strength × rep_ang + 0.30 × target_ang
final_lin = target_lin × (1 - 0.70 × strength)
```

**Potential Field Theory:**
- Every obstacle exerts a **repulsive force** proportional to 1/distance²
- The target exerts an **attractive force** proportional to distance
- The robot moves in the direction of the net force vector
- We simplify by projecting onto angular and linear components only

### 6.4 REROUTE State — Vector Guidance

```python
# Simple proportional navigation to (lk_x, lk_y)
dx = lk_x - robot_x
dy = lk_y - robot_y
desired_heading = atan2(dy, dx)
heading_error = atan2(sin(desired - yaw), cos(desired - yaw))  # shortest angle

lin = KP_LIN × distance_to_goal
ang = KP_ANG × heading_error
```

**Why not Nav2?** Nav2 requires a global costmap and a configured planner. During development, Nav2 was not always running. The manual vector guidance is a robust fallback that works without any external navigation stack.

**Nav2 is tried first** (non-blocking using `server_is_ready()`). If available, a full path-planned route is sent. If not, manual guidance takes over silently.

### 6.5 Acceleration Limiting

```python
def _publish_vel(self, lin, ang, smooth=True):
    if smooth:
        # Ramp towards target velocity at max acceleration
        d_lin = lin - self._cmd_lin
        if abs(d_lin) > ACCEL_LIN:            # ACCEL_LIN = 0.035 m/s/tick
            lin = self._cmd_lin + copysign(ACCEL_LIN, d_lin)
        # Same for angular
    self._cmd_lin = lin
    self._cmd_ang = ang
    # Publish...
```

This prevents sudden velocity spikes that cause motor current spikes and OpenCR brownouts (observed in real hardware testing).

### 6.6 Motor Safety

```python
def _tick(self):
    if not self._active:
        self._publish_vel(0.0, 0.0, smooth=False)  # Hard stop every 100ms
        return
```

Even when `STOP` is sent, the robot continuously publishes zero velocity at 10 Hz. This prevents the robot from continuing with stale velocity commands if a node dies.

---

## 7. Algorithm Deep Dive — Bypass Follower Node

### 7.1 Purpose

A single-purpose node for the exact demo scenario:
```
[ROBOT] →→→ [OBSTACLE ROBOT] [HUMAN]
         ↑ 80cm
     Bypass triggers here
```

### 7.2 State Machine

```
APPROACH → BYPASS_TURN → BYPASS_PASS → SEARCH → FOLLOW
```

| State | Trigger | Action |
|---|---|---|
| APPROACH | Started | Drive forward at 0.18 m/s, watch front LiDAR |
| BYPASS_TURN | front < 0.80 m | Rotate 90° toward clearer side until front > 1.00 m |
| BYPASS_PASS | Front is clear | Drive forward 0.60 m / 0.18 m/s = 3.33 seconds |
| SEARCH | Timer done | Rotate back (opposite direction) at 0.35 rad/s |
| FOLLOW | Target visible | PD follow at safe distance 0.70 m |

### 7.3 Side Selection Algorithm

```python
# Choose the more open side (higher distance = more space)
if self.left >= self.right:
    self._bypass_dir = +1.0   # Turn LEFT (CCW)
else:
    self._bypass_dir = -1.0   # Turn RIGHT (CW)
```

### 7.4 Time-Based Distance Tracking

Since the TurtleBot3 lacks wheel encoders accessible in real-time, forward progress during `BYPASS_PASS` is tracked by time:

```python
BYPASS_FWD_TIME = BYPASS_FWD_DIST / FWD_SPEED = 0.60 / 0.18 = 3.33 seconds
```

---

## 8. Algorithm Deep Dive — Manager Node

### 8.1 Purpose

Coordinates the startup sequence to ensure:
1. Perception trains on the person BEFORE behavior starts moving
2. No race conditions between nodes starting up

### 8.2 Sequence

```
Manager starts → waits 5 seconds (other nodes to initialise)
→ sends "TRAIN" to /perception/command
→ perception_node enters TRAINING mode
→ perception_node collects 40 frames
→ perception_node publishes {"visible": true, ...} on /tracked_target
→ manager_node receives this and sends "START" to /behavior/command
→ behavior_tree_node begins operating
```

---

## 9. Re-ID System (Embedded from person_reid_tracker)

The system embedded logic from a separate `person_reid_tracker` project located in `~/person re id/`.

### 9.1 Source Files Used

| Source File | Logic Extracted |
|---|---|
| `src/person_reid.py` | TorchReIDBackend (OSNet), MobileNetFallbackBackend, cosine similarity |
| `src/utils.py` | `cosine_similarity()`, `max_similarity_to_gallery()`, `normalize_embedding()` |
| `src/face_reid.py` | Architecture reference (face not used — legs/body focus for TurtleBot camera angle) |
| `src/identity_manager.py` | Streak-based temporal smoothing, multi-candidate suppression logic |
| `src/bytetrack_tracker.py` | Custom Kalman filter + IoU-based Hungarian assignment |

### 9.2 Body Embedding Pipeline

```
Body Crop (256×128 pixels)
         ↓
  RGB normalisation (mean=[0.485,0.456,0.406], std=[0.229,0.224,0.225])
         ↓
  MobileNetV3-Small features → global average pooling → 576-dim vector
         ↓
  L2 normalisation → unit-length embedding
         ↓
  Cosine similarity against gallery (dot product of unit vectors)
```

### 9.3 Cosine Similarity Mathematics

```
Given: embedding a (query), gallery embeddings G = {g₁, g₂, ..., gₙ}

cosine_sim(a, g) = (a · g) / (||a|| × ||g||)

Since both a and g are L2-normalised:
  ||a|| = 1, ||g|| = 1
  cosine_sim(a, g) = a · g   (simple dot product)

score = max(cosine_sim(a, gᵢ) for gᵢ in G)

Threshold: score >= 0.52 → same person
```

### 9.4 HSV Histogram Mathematics

```
For each person crop:
  Convert BGR → HSV
  Compute 2D histogram: Hue (36 bins, 0-180) × Saturation (32 bins, 0-256)
  Normalise to [0,1]

Comparison (Bhattacharyya distance):
  B(H₁, H₂) = -ln(Σ √(H₁(i) × H₂(i)))  for all bins i

Similarity = 1 - B(H₁, H₂)
  → 1.0 = identical distributions (same person)
  → 0.0 = no overlap (different person)

Threshold: similarity >= 0.50 → same person
```

---

## 10. Sensor Fusion — Camera + LiDAR

### 10.1 Coordinate Frame Alignment

The TurtleBot3 Waffle Pi camera is mounted at the front, facing forward. The LiDAR also faces forward. The camera provides bearing angle; the LiDAR provides distance.

```
Camera frame:
  angle = (cx - image_width/2) × (HFOV / image_width)
  Positive angle = person to LEFT (standard camera convention)

LiDAR frame:
  angle 0 = directly forward
  Positive angle = LEFT
  The scan convention is REVERSED: person_bearing = -target_angle

LiDAR ray index from camera angle:
  idx = round((camera_angle - scan.angle_min) / scan.angle_increment)
  NEGATIVE because camera and LiDAR have mirrored horizontal conventions
```

### 10.2 Window Median Filter

```python
# Collect valid ranges in a ±8 ray window around person bearing
valid_ranges = [scan.ranges[i] for i in range(idx-8, idx+9)
                if 0 <= i < len(scan.ranges)
                and isfinite(scan.ranges[i])
                and range_min <= scan.ranges[i] <= range_max]

distance = median(valid_ranges)
```

Taking the median (not mean) is critical because:
- Single-point reflections return 0.0 (floor) or inf (missed beam)
- Mean would be heavily skewed by even one bad reading
- Median is the 50th-percentile → robust to outliers

---

## 11. Obstacle Avoidance — Potential Field Method

### 11.1 LiDAR Sector Division

```
                     FRONT (±0.52 rad)
                     ┌─────────┐
              LEFT   │         │   RIGHT
           (0 to     │  ROBOT  │   (-1.05 to
            1.05)    │   ●     │    0 rad)
                     └─────────┘

Person's legs EXCLUDED from all sectors
(within ±0.38 rad of person bearing)
```

### 11.2 Noise Filter (3rd Percentile Sort)

```python
def pct3(rays: list) -> float:
    if not rays: return 10.0
    rays.sort()
    idx = min(2, len(rays) - 1)   # index 2 = 3rd smallest
    return rays[idx]

# Why index 2 (3rd smallest)?
# Index 0 (minimum) is too noisy — single dust/glass reflections
# Index 2 skips the 2 most erroneous readings
# If < 3 rays in sector, uses the smallest available
```

### 11.3 Emergency vs. Reactive Avoidance

| Distance | Response |
|---|---|
| > OBS_WARN_DIST (0.55 m) | No avoidance, pure target tracking |
| OBS_STOP_DIST to OBS_WARN_DIST | Gradual blend: slow down + partial steering |
| < OBS_STOP_DIST (0.22 m) | Emergency: stop linear motion, max angular away |

---

## 12. LiDAR Noise Filter

### 12.1 Hardware Noise Sources

The RPLidar A1 produces noise from:
- **Glass/mirrors**: Transparent surfaces absorb or scatter light
- **Dust particles**: Small particles cause single-point erroneous readings
- **Robot chassis reflections**: The robot's own casing creates near-zero readings
- **Floor reflections**: Downward-angled returns give very short range readings
- **Dark surfaces**: High-absorption materials return no signal (inf/NaN)

### 12.2 Multi-Layer Filter

```
Layer 1: Validity check
  - Reject NaN, inf
  - Reject r < max(0.12, scan.range_min)  ← ignores chassis self-reads
  - Reject r > scan.range_max

Layer 2: Person exclusion
  - Reject rays within ±0.38 rad of person bearing
  - Prevents the person's own legs from triggering obstacle stop

Layer 3: 3rd-percentile statistical filter
  - Sort valid rays per sector
  - Use index [2] → ignores 2 lowest (most likely noise) readings
  - If < 3 rays available, use index 0 (minimum available)
```

---

## 13. Specific Scenario — Bypass Follower

### 13.1 Scenario Parameters
- Total distance robot-human: 1 metre
- Obstacle: another TurtleBot3 (cylindrical body, ~30 cm diameter)
- Obstacle position: approximately 0.80 m from following robot
- Robot forward speed: 0.18 m/s (slow and controlled for precision)
- Turn speed: 0.55 rad/s

### 13.2 Geometry of Bypass

```
Step 1 (APPROACH): Robot moves forward at 0.18 m/s
Step 2 (BYPASS_TURN): Rotate 90° (≈ π/2 rad) at 0.55 rad/s
  → Duration ≈ (π/2) / 0.55 ≈ 2.85 seconds
  → Terminates early when front LiDAR > 1.0 m (obstacle cleared from path)

Step 3 (BYPASS_PASS): Move forward 0.60 m at 0.18 m/s
  → Duration: 3.33 seconds
  → This clears the physical body of the obstacle

Step 4 (SEARCH): Rotate back at 0.35 rad/s
  → Rotates in OPPOSITE direction of bypass
  → Person should appear within ~90° rotation

Step 5 (FOLLOW): Normal PD follow at 0.70 m safe distance
```

---

## 14. Dashboard Integration

The `supermarket_dashboard.py` dashboard was updated to:
- Publish `"STOP"` to `/behavior/command` when Emergency Stop is pressed
- Display the behavior state from `/person_following/behavior_state`
- Display LiDAR map with obstacle visualization
- Show the YOLO debug image from `/yolo/debug_image`

### Emergency Stop Chain
```
Dashboard button → /behavior/command ("STOP")
                 → /cmd_vel (Twist zero) [direct publish]
                 → behavior_tree_node sets active=False → publishes 0.0 at 10Hz
```

---

## 15. Key Engineering Decisions & Bugs Fixed

| Issue | Root Cause | Fix |
|---|---|---|
| Robot spinning on startup | Behavior started in ACQUIRE (spin) state | Changed to WAIT state; only moves after target visible |
| Robot not moving at all | QoS incompatibility on `/scan` (RELIABLE vs BEST_EFFORT) | Applied `BEST_EFFORT` QoS profile to scan subscriber |
| `np.float` AttributeError | Old `transforms3d` library used deprecated numpy alias | Removed `tf_transformations` dependency; used `math.sin/cos` for quaternion |
| `rcl_shutdown already called` | `rclpy.spin()` catches interrupt and shuts down internally, then `finally` block called it again | Guard with `if rclpy.ok(): rclpy.shutdown()` |
| Nav2 freezing node | `wait_for_server(timeout=2.0)` blocks the ROS 2 timer thread | Replaced with non-blocking `server_is_ready()` check |
| Person not recognized (Re-ID failure) | ByteTrack ID drops after ~30 frame buffer expiry | Added 3-level fallback: deep body embedding → HSV histogram → spatial |
| Robot keeps moving after node kill | Stale `/cmd_vel` messages persist until next publisher | Publish 0.0 at 10Hz actively when `not self._active` |
| False obstacle detections | Single noisy LiDAR rays (dust, glass) triggered avoidance | 3rd-percentile filter (skip 2 lowest) + chassis exclusion |
| OpenCR motor crash | Voltage brownout from rapid acceleration | ACCEL_LIN = 0.035 m/s per tick limits current surge |
| Perception showing "WAITING / target None" | Manager sent TRAIN before perception node fully started | Manager waits 5 seconds before sending TRAIN |
| Person legs excluded as obstacle | Same bearing angle as person in LiDAR | Person bearing exclusion zone ±0.38 rad added to scan_callback |

---

## 16. Full Code Annotations

### 16.1 perception_node.py — Key Sections

```python
# ── Training phase ─────────────────────────────────────────────
# Pick the most centred detected person (person standing in front)
candidate = min(persons, key=lambda p: abs((p["x1"]+p["x2"])/2.0 - image_width/2))

# Extract deep body embedding using MobileNetV3
crop = frame[y1:y2, x1:x2]                    # crop person from frame
rgb = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)    # model expects RGB
tensor = transform(rgb).unsqueeze(0)           # (1, 3, 256, 128) tensor
features = model(tensor)                        # forward pass
emb = normalize(features.flatten())            # L2-normalize

# Compute HSV histogram fingerprint
hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
hist = cv2.calcHist([hsv], [0,1], None, [36,32], [0,180,0,256])
# H-axis: 36 bins covering 0-180 degrees of hue
# S-axis: 32 bins covering 0-255 saturation

# After 40 training frames: finalise
body_gallery = list_of_40_embeddings
hsv_fingerprint = mean(all_40_hists)  # single averaged fingerprint
```

```python
# ── Level 2 Re-ID: body embedding matching ─────────────────────
for person in detected_persons:
    crop = extract(frame, person.bbox)
    emb = mobilenet.embed(crop)
    # Compare against EVERY embedding in our gallery
    sim = max(cosine_dot_product(emb, g) for g in body_gallery)
    if sim > 0.52:
        re_lock_on(person)  # this is our owner!
```

```python
# ── LiDAR angle-to-index conversion ────────────────────────────
# camera_angle > 0 → person to LEFT
# LiDAR convention: positive angle = LEFT
# But camera and LiDAR x-axes are mirrored, so negate:
person_bearing_lidar = -camera_angle

centre_idx = round((person_bearing_lidar - scan.angle_min) / scan.angle_increment)
valid = [scan.ranges[i] for i in range(centre_idx-8, centre_idx+9)
         if valid_range(scan.ranges[i])]
distance = median(valid)  # noise-robust
```

### 16.2 behavior_tree_node.py — Key Sections

```python
# ── Acceleration limiting ───────────────────────────────────────
# Without this: robot jerks → current spike → OpenCR resets
diff = target_lin - current_lin
if abs(diff) > ACCEL_LIN:              # ACCEL_LIN = 0.035 m/s
    cmd_lin = current_lin + sign(diff) * ACCEL_LIN
else:
    cmd_lin = target_lin
# Limits to 0.035 m/s change per 0.1s tick = 0.35 m/s² max accel

# ── Potential field avoidance blend ────────────────────────────
# strength ranges from 0 (obstacle far) to 1 (obstacle at STOP_DIST)
strength = (OBS_WARN_DIST - closest) / (OBS_WARN_DIST - OBS_STOP_DIST)
strength = clamp(0.0, 1.0, strength)

# Angular blend: 70% obstacle repulsion + 30% target attraction
final_ang = 0.70 * strength * repulsion_direction + 0.30 * target_angular
# At strength=1.0 (very close): almost all repulsion
# At strength=0.5 (medium range): 35% repulsion + 15% target = 50% total scaled

# ── Quaternion from yaw (no tf_transformations) ─────────────────
# For a 2D rotation around Z-axis:
# q = [sin(yaw/2), 0, 0, cos(yaw/2)] = [qx=0, qy=0, qz=sin(yaw/2), qw=cos(yaw/2)]
orientation.z = math.sin(yaw * 0.5)
orientation.w = math.cos(yaw * 0.5)
# This replaces the broken tf_transformations.quaternion_from_euler(0, 0, yaw)
```

---

## 17. Conversation Journey (Session Summary)

### Session 1 — Initial Problems
- **Problem:** Perception node detecting everyone as target
- **Fix:** Filter to only person standing directly in front; use area + position heuristics
- **Problem:** LiDAR not working at all for distance
- **Fix:** Added LiDAR scan subscriber and bearing-based range lookup

### Session 2 — Behaviour Tree Overhaul
- **Problem:** Robot not doing full 360° search when person lost
- **Fix:** Added `SEARCH_SPIN` state with 12-second timer
- **Problem:** Obstacle avoidance unreliable
- **Fix:** Sliding window voting on obstacle detection (5-frame history)

### Session 3 — ByteTrack Integration
- **Problem:** Histogram-based Re-ID too unreliable; person not remembered after occlusion
- **Fix:** Replaced with YOLO's built-in ByteTrack (`model.track(persist=True, tracker="bytetrack.yaml")`)
- **Added:** Manager node to sequence TRAIN → WAIT → START

### Session 4 — Real Hardware Issues
- **Problem:** `np.float` AttributeError from `tf_transformations` / `transforms3d`
- **Fix:** Removed dependency; implemented quaternion directly using `math.sin/cos`
- **Problem:** `rcl_shutdown already called` crash on Ctrl+C
- **Fix:** Guard with `if rclpy.ok(): rclpy.shutdown()`
- **Problem:** QoS mismatch on `/scan` → no LiDAR data
- **Fix:** Applied `BEST_EFFORT` QoS profile

### Session 5 — Final Re-ID Integration
- **Problem:** ByteTrack drops ID after person is occluded; person not recognised when re-appears
- **Fix:** 3-level Re-ID cascade: body embedding → HSV histogram → spatial fallback
- **Added:** MobileNetV3 body embedder from `~/person re id/person_reid_tracker/`
- **Added:** Rolling gallery with EMA HSV fingerprint update

### Session 6 — Specific Scenario
- **Request:** Implement exact scenario: robot + obstacle + human in one line, 1 metre total
- **Created:** Dedicated `bypass_follower_node.py` with APPROACH → BYPASS_TURN → BYPASS_PASS → SEARCH → FOLLOW

### Session 7 — Final Polish
- **Added:** Sensor noise filter to LiDAR (3rd-percentile sort)
- **Fixed:** Dashboard Emergency Stop now sends STOP to behavior node
- **Added:** Acceleration limiting to prevent motor brownouts
- **Fixed:** Robot spinning on startup (changed from ACQUIRE to WAIT initial state)

---

## 18. Run Commands

### Full System (Perception + Behavior + Manager)

**Terminal 1 — Perception (stand in front first!):**
```bash
cd ~/robot_follower_ws
source install/setup.bash
ros2 run robot_follower perception_node
# Wait for: "Training DONE — ID=X, 40 body embeddings"
```

**Terminal 2 — Behavior:**
```bash
cd ~/robot_follower_ws
source install/setup.bash
ros2 run robot_follower behavior_tree_node
```

**Terminal 3 — Manager (auto-sequences everything):**
```bash
cd ~/robot_follower_ws
source install/setup.bash
ros2 run robot_follower manager_node
```

### Bypass Scenario

**Terminal 1 — Perception:**
```bash
ros2 run robot_follower perception_node
```

**Terminal 2 — Bypass Follower:**
```bash
ros2 run robot_follower bypass_follower_node
```

**Terminal 3 — Start:**
```bash
ros2 topic pub --once /behavior/command std_msgs/msg/String "{data: 'START'}"
```

### Emergency Stop (any terminal)
```bash
ros2 topic pub --once /behavior/command std_msgs/msg/String "{data: 'STOP'}"
```

### Build
```bash
cd ~/robot_follower_ws
colcon build
source install/setup.bash
```

---

## 19. Glossary

| Term | Definition |
|---|---|
| **ByteTrack** | Multi-object tracker that assigns stable integer IDs to detected people across frames using IoU-based Hungarian assignment and Kalman filtering |
| **Cosine Similarity** | Measure of angle between two vectors; 1.0 = same direction (same person), 0.0 = orthogonal (different person) |
| **Bhattacharyya Distance** | Statistical measure of overlap between two probability distributions; used to compare HSV histograms |
| **Potential Field** | Motion planning method where obstacles repel and targets attract the robot, computing a net velocity vector |
| **EMA** | Exponential Moving Average; running average weighted towards recent values |
| **QoS** | Quality of Service; ROS 2 setting controlling message reliability (RELIABLE vs BEST_EFFORT) |
| **OSNet** | Omni-Scale Network; lightweight person Re-ID neural network from Kia Zhong's torchreid library |
| **MobileNetV3** | Efficient CNN backbone optimised for CPU inference; used as fallback body embedder |
| **3rd Percentile Filter** | Sort LiDAR rays; skip the 2 lowest readings as noise; use the 3rd lowest as the obstacle distance |
| **ByteTrack ID** | Persistent integer assigned by ByteTrack to a detected person; survives partial occlusion for ~30 frames |
| **Re-ID** | Re-Identification; recognising a specific person from their appearance features (body shape, colour, face) |
| **HFOV** | Horizontal Field of View; the angular span of the camera in radians (≈ 60° = 1.047 rad for RealSense) |
| **Deadband** | A region around the target distance where no motion is commanded (prevents oscillation) |
| **ACCEL_LIN** | Maximum linear velocity change per control cycle (0.035 m/s per 0.1s = 0.35 m/s²) |
| **Brownout** | Temporary voltage drop causing motor controller reset; triggered by excessive current draw from rapid acceleration |

---

*Document generated: July 2026*  
*Project: TurtleBot3 Waffle Pi Person Follower*  
*Total lines of code: ~5,230 lines across 14 Python files*
