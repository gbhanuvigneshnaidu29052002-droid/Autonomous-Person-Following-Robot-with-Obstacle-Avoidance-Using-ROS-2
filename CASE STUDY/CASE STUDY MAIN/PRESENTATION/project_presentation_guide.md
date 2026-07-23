# Project Defense & Technical Reference Guide: Person-Following Robot
This document provides a comprehensive technical guide for defending the project. It includes expected questions from professors (with detailed answers), explanations of all technical terms, and a breakdown of the codebase architecture and package dependencies.

---

## 1. Technical Terms & Architecture Glossary

### ROS 2 & System Integration
* **Node:** An independent executable process in ROS 2 that performs a specific set of computations (e.g., `perception_node`).
* **Topic:** A named bus over which nodes exchange messages using publish/subscribe semantics (e.g., `/cmd_vel` for motor velocities, `/scan` for LiDAR, `/tracked_target` for target details).
* **QoS (Quality of Service):** Configuration profiles that control how messages are handled, queued, and transferred. 
  > [!IMPORTANT]
  > **QoS Compatibility:** The LiDAR publishes with a `BEST_EFFORT` reliability and `VOLATILE` durability profile. Subscribers *must* match this profile, or ROS 2 will silently drop all messages due to incompatibility.
* **OpenCR 1.0:** The low-level microcontroller board on the TurtleBot3 that controls the Dynamixel servos, reads IMU data, and handles low-level motor commands from `/cmd_vel`.
* **Action Client/Server:** A communication mechanism in ROS 2 for long-running, non-blocking tasks. Unlike topics, actions are goal-oriented, allow feedback, and can be cancelled. Used for Nav2 path planning goals.

### Computer Vision & Tracking
* **YOLOv8 (You Only Look Once):** A state-of-the-art, real-time object detection neural network. The perception node uses the lightweight `yolov8n.pt` (Nano) model, filtering for the `person` class (class ID `0`).
* **ByteTrack:** A high-performance tracking algorithm that associates bounding boxes across frames. It operates in two steps: first, associating high-score detections, then matching low-score detections (e.g., blurred or partially occluded people) using Kalman filtering and Hungarian association.
* **Re-ID (Person Re-Identification):** The process of matching a cropped image of a person against a gallery of enrolled features to verify identity across different cameras, locations, or after a tracking loss.
* **MobileNetV3 / OSNet:** Lightweight neural networks used to extract a low-dimensional feature vector (embedding) from a person crop. OSNet is specifically designed for Re-ID tasks, while MobileNetV3 is a general-purpose feature extractor.
* **Cosine Similarity:** A metric used to measure how similar two vectors are. It computes the dot product of two normalized vectors:
  $$\text{Similarity} = \frac{A \cdot B}{\|A\| \|B\|}$$
  A value of `1.0` means identical direction; our system considers a match if similarity is $\ge 0.52$.
* **HSV Histogram & Bhattacharyya Distance:** 
  * A color representation scheme (Hue, Saturation, Value) that is more robust to light intensity changes than RGB.
  * Bhattacharyya distance measures the overlap of two probability distributions (histograms). We invert it ($1.0 - \text{distance}$) to get a color similarity score (match threshold $\ge 0.50$).

### Control & Navigation
* **P-Control (Proportional Control):** A control feedback mechanism where the corrective action (motor command) is proportional to the error (difference between target state and current state).
* **Potential Field Method:** A reactive navigation algorithm where obstacles exert a repulsive force ($1/\text{distance}^2$), and the goal exerts an attractive force, guiding the robot along the vector sum of these forces.
* **Acceleration Limiting (Slew Rate):** Capping the maximum allowable change in velocity per time step. This prevents current spikes that trigger the OpenCR board to brownout and reset.

---

## 2. ROS 2 Node & File Map

The project consists of several specialized ROS 2 nodes, located in the `code/` folder:

| File | Node Name | Role & Architecture |
| :--- | :--- | :--- |
| [perception_node.py](file:///home/ganeshna/person_follower_robot_project/code/perception_node.py) | `perception_node` | Runs YOLOv8 + ByteTrack + Re-ID on laptop CPU. Merges camera angles with LiDAR ranges. Publishes target states on `/tracked_target` (JSON). |
| [behavior_tree_node.py](file:///home/ganeshna/person_follower_robot_project/code/behavior_tree_node.py) | `behavior_tree_node` | A state machine coordinating robot states: `WAIT`, `FOLLOW`, `AVOID`, `REROUTE`, `SEARCH`, `ACQUIRE`. Publishes velocities on `/cmd_vel`. |
| [bypass_follower_node.py](file:///home/ganeshna/person_follower_robot_project/code/bypass_follower_node.py) | `bypass_follower_node` | Dedicated node for the specific 1-meter obstacle-bypass test scenario. |
| [manager_node.py](file:///home/ganeshna/person_follower_robot_project/code/manager_node.py) | `manager_node` | Manages node lifecycles: triggers perception enrollment (`TRAIN`) first, then activates behavior (`START`). |
| [supermarket_dashboard.py](file:///home/ganeshna/person_follower_robot_project/code/supermarket_dashboard.py) | N/A | Tkinter graphical dashboard displaying the debug video feed, active state, Re-ID scores, a 2D LiDAR radar, and emergency stop. |
| [setup.py](file:///home/ganeshna/person_follower_robot_project/code/setup.py) | N/A | Configures installation metadata, dependencies, and node console entry points for compilation. |

---

## 3. Expected Professor Questions & Answers

### Category A: Perception & Re-ID

#### Q1: Why did you implement a multi-level Re-ID system instead of just relying on YOLO and ByteTrack?
* **Answer:** YOLO is class-level (it only detects "a person", not "which person"). ByteTrack handles frame-to-frame association by tracking bounding box overlaps. However, if the owner is fully occluded (e.g., walks behind a shelf) or leaves the frame, ByteTrack's internal Kalman filter expires and assigns a new tracking ID when they reappear. Re-ID acts as a biometric lock; when a track is lost or changed, the system compares the reappearing person's visual embeddings (deep body features and HSV histograms) against a gallery of the owner to re-establish the lock, preventing the robot from following a stranger.

#### Q2: Explain your 4-level Re-ID cascade. What happens at each step?
* **Answer:** 
  1. **Level 1 (ByteTrack Continuity):** Fast check. If the current bounding box has the same tracking ID as our target, we assume it is the target (zero computation).
  2. **Level 2 (Deep Body Embedding):** If the ID changed, we crop the bounding box, extract a feature vector using MobileNetV3 (or OSNet), and compute cosine similarity against a rolling gallery of the last 20 frames of the target. If similarity $\ge 0.52$, we re-lock.
  3. **Level 3 (HSV Histogram):** If deep Re-ID is inconclusive or unavailable, we compute a 2D Hue-Saturation histogram and compare it to the target's average color fingerprint using Bhattacharyya similarity. If similarity $\ge 0.50$, we re-lock.
  4. **Level 4 (Spatial Fallback):** If all else fails, we fall back to spatial proximity. If only one person is in the frame, or a person appears close to the target's last known bearing angle (within $\pm 0.25$ rad), we assume it is the target.

#### Q3: Why do you average the HSV histograms over 40 frames during training, and why do you update it during tracking?
* **Answer:** During the 5-second training phase (40 frames), the target stands in front of the robot. A single frame is highly sensitive to noise, shadows, and body posture. By averaging 2D HSV histograms over 40 frames, we construct a robust color fingerprint. During tracking, we update this fingerprint with an Exponential Moving Average ($\alpha = 0.10$):
  $$F_{\text{new}} = (1 - \alpha) F_{\text{old}} + \alpha H_{\text{current}}$$
  This allows the color model to adapt to slow changes in lighting and viewing angles as the owner moves.

---

### Category B: Sensor Fusion & Noise Filtering

#### Q4: How do you fuse the Camera and LiDAR data to get the target's distance?
* **Answer:** 
  1. From the camera frame, we compute the target's bearing angle relative to the center axis based on the horizontal pixel coordinate ($c_x$) and the horizontal Field of View ($\text{HFOV} = 1.047$ rad):
     $$\text{Angle} = \left(c_x - \frac{\text{width}}{2}\right) \times \frac{\text{HFOV}}{\text{width}}$$
  2. Because the horizontal coordinates of the RealSense camera and RPLidar are mirrored, we negate this angle to convert it to the LiDAR coordinate frame.
  3. We map this angle to the closest index in the LiDAR ranges array:
     $$\text{Index} = \text{round}\left(\frac{\text{Angle} - \text{Scan}_{\text{min}}}{\text{Scan}_{\text{increment}}}\right)$$
  4. We sample a window of $\pm 8$ rays around this index, filter out out-of-range or infinite readings, and take the **median** value to robustly measure the target's distance. If LiDAR returns no valid rays, we fall back to a height-based bounding box estimation.

#### Q5: Why do you use the median of the LiDAR rays instead of the mean?
* **Answer:** LiDAR scan data contains significant noise, such as multi-path reflections off the floor, dust, or missing rays (which return 0.0 or infinity). Taking the average (mean) would skew the distance estimate heavily if even one ray returned an error. The median represents the 50th percentile, which discards extreme outliers and ensures we measure the distance to the person's leg rather than the floor or the wall behind them.

#### Q6: Explain your LiDAR noise filtering technique for obstacle avoidance. What is the "3rd percentile filter"?
* **Answer:** To prevent false obstacle detections caused by dust particles, glass reflections, or chassis self-reflections, we split the 180° forward scan into three sectors: Left ($0$ to $1.05$ rad), Right ($-1.05$ to $0$ rad), and Front ($\pm 0.52$ rad). 
  We sort the valid rays in each sector in ascending order and select the value at index `2` (the 3rd smallest value). This ignores the two smallest distance values, which are most likely noise, while still reporting the actual nearest obstacle.

---

### Category C: Control & Navigation

#### Q7: How does the robot avoid obstacles while following a person? Detail the Potential Field blend.
* **Answer:** When an obstacle is detected in any of the three sectors at a distance less than `OBS_WARN_DIST` ($0.55$ m), the robot switches from `FOLLOW` to `AVOID`.
  We calculate an obstacle repulsion angle ($\theta_{\text{rep}} = \pm 0.75$ rad/s) directing the robot away from the blocked side. We calculate the repulsion strength based on the proximity of the closest obstacle:
  $$\text{Strength} = \text{clamp}\left(0.0, 1.0, \frac{\text{OBS\_WARN\_DIST} - d_{\text{closest}}}{\text{OBS\_WARN\_DIST} - \text{OBS\_STOP\_DIST}}\right)$$
  We then blend this repulsive force (70%) with the target attraction angle (30%):
  $$\omega = 0.70 \times \text{Strength} \times \theta_{\text{rep}} + 0.30 \times \theta_{\text{target}}$$
  This allows the robot to swerve around obstacles while maintaining progress towards the target. If an obstacle gets closer than `OBS_STOP_DIST` ($0.22$ m), we execute an emergency stop and rotate away.

#### Q8: Why did you implement custom manual vector guidance in the REROUTE state instead of solely relying on Nav2?
* **Answer:** Nav2 is a complete navigation stack requiring a global costmap, static map, and localization (AMCL/SLAM). In real hardware environments, Nav2 can freeze, fail to plan a path in close quarters, or become unavailable. 
  Our custom vector guidance acts as a robust fallback. If Nav2's action server is not active or fails to accept a goal, the robot dynamically calculates proportional heading errors and distance vectors from its odometry directly to the target's last known coordinate ($lk_x, lk_y$), navigating there safely without external dependencies.

#### Q9: What is the purpose of the acceleration limiting in your velocity publisher?
* **Answer:** The TurtleBot3 is powered by LiPo batteries. Aggressive changes in linear speed or direction demand sudden current spikes from the motors. Because the Raspberry Pi compute board and the OpenCR motor controller share the same power bus, these current spikes drop the system voltage, causing power brownouts that reset the OpenCR board or shutdown the Pi.
  Our code restricts the velocity change per tick ($100$ ms) to `ACCEL_LIN` ($0.035$ m/s per tick) and `ACCEL_ANG` ($0.12$ rad/s per tick) to ramp velocities smoothly, protecting the hardware.

---

### Category D: Specific Test Scenario & Failures

#### Q10: How does the robot handle the specific 1-meter obstacle-bypass scenario?
* **Answer:** This is handled by a state machine in [bypass_follower_node.py](file:///home/ganeshna/person_follower_robot_project/code/bypass_follower_node.py):
  1. **APPROACH:** Drives straight at $0.18$ m/s toward the target.
  2. **BYPASS_TURN:** Triggers when the front LiDAR detects the obstacle at $< 0.80$ m. It compares Left and Right LiDAR sectors and rotates 90° toward the clearer side at $0.55$ rad/s until its front is clear ($> 1.00$ m).
  3. **BYPASS_PASS:** Drives forward for exactly $3.33$ seconds ($0.60$ m bypass distance / $0.18$ m/s speed) to clear the physical body of the obstacle.
  4. **SEARCH:** Rotates in the opposite direction of the bypass turn at $0.35$ rad/s until the perception node re-identifies the target.
  5. **FOLLOW:** Resumes standard P-control tracking.

```
                  Step 2: Turn 90°       Step 3: Drive past
                    ┌───→ [ROBOT] ───────────────┐
                    │     (steers clear)         │
                    │                            ↓
  Step 1: Approach  │                      Step 4: Rotate & search
     [ROBOT] ───────┴─────── [OBSTACLE] ───────── [ROBOT] ───→ [HUMAN] (Step 5)
```

#### Q11: How did you prevent the target's own legs from triggering obstacle avoidance?
* **Answer:** In both `behavior_tree_node.py` and `bypass_follower_node.py`, the target's bearing angle ($\theta$) is passed to the laser scan callback. We define an exclusion cone of $\pm 0.38$ rad around the target's direction (`PERSON_EXCL`). 
  Any LiDAR rays falling within this exclusion zone are ignored when sorting scan ranges into Left, Right, and Front obstacle sectors. This ensures the robot does not detect the target as an obstacle and lock its own brakes.

---

## 4. Key Engineering Decisions & Technical Fixes

Be prepared to explain these real-world bug fixes if asked about system stability:

1. **QoS Profile Mismatch:** Initially, the subscriber to `/scan` was not receiving data. We solved this by changing the subscriber's QoS profile from `RELIABLE` to `BEST_EFFORT` (with `VOLATILE` durability) to match the RPLidar hardware driver.
2. **OpenCR Shutdown Crash:** When interrupting nodes (Ctrl+C), `rclpy.shutdown()` was called twice, throwing shutdown errors. We added a safety wrapper:
   ```python
   if rclpy.ok():
       rclpy.shutdown()
   ```
3. **Stale Velocity Safety:** If the behavior node crashes, the robot could continue coasting at its last published speed. We resolved this by modifying the behavior loop to publish `Twist(0,0)` at 10 Hz whenever the node state is inactive or stopped.
