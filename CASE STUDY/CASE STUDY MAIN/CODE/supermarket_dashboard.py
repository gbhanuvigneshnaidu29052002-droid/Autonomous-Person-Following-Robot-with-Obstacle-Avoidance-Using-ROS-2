#!/usr/bin/env python3
"""
supermarket_dashboard.py
Premium Cyberpunk Telemetry Dashboard for TurtleBot3 Person-Following Robot.
ROS 2 Humble compatible.
"""

import sys
import math
import time
import json
import threading
import tkinter as tk
from tkinter import ttk, messagebox
from tkinter.scrolledtext import ScrolledText
from datetime import datetime
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image as RosImage, LaserScan
from cv_bridge import CvBridge
from gazebo_msgs.srv import SpawnEntity, DeleteEntity

try:
    from PIL import Image as PilImage, ImageTk
    PIL_AVAILABLE = True
except ImportError:
    PIL_AVAILABLE = False

# 
# COLOUR PALETTE  Cyberpunk dark theme
# 
C_BG      = "#0D1117"
C_PANEL   = "#161B22"
C_BORDER  = "#21262D"
C_ACCENT  = "#00FFCC"
C_DANGER  = "#FF4757"
C_WARNING = "#FFA502"
C_SUCCESS = "#2ED573"
C_PURPLE  = "#A29BFE"
C_TEAL    = "#00CEC9"
C_FG      = "#E6EDF3"
C_MUTED   = "#8B949E"
C_TOPBAR  = "#010409"

F_MONO = "Courier New"
F_HEAD = "Helvetica"


# 
# Thread-safe scan store
# 
class ScanStore:
    def __init__(self):
        self._lock           = threading.Lock()
        self.ranges          = []
        self.angle_min       = 0.0
        self.angle_increment = 0.0
        self.range_min       = 0.0
        self.range_max       = 10.0

    def update(self, msg):
        with self._lock:
            self.ranges          = list(msg.ranges)
            self.angle_min       = msg.angle_min
            self.angle_increment = msg.angle_increment
            self.range_min       = msg.range_min
            self.range_max       = msg.range_max

    def snapshot(self):
        with self._lock:
            return (list(self.ranges), self.angle_min,
                    self.angle_increment, self.range_min, self.range_max)


# 
# ROS 2 NODE
# 
class DashboardNode(Node):
    def __init__(self, ui_callback):
        super().__init__("supermarket_dashboard")
        self.ui_callback = ui_callback
        self.bridge      = CvBridge()
        self._scan_store = ScanStore()

        self.create_subscription(String,    "/tracked_target",                  self._on_target, 10)
        self.create_subscription(String,    "/person_following/behavior_state",  self._on_state,  10)
        self.create_subscription(Odometry,  "/odom",                            self._on_odom,   10)
        self.create_subscription(Twist,     "/cmd_vel",                         self._on_cmd,    10)
        self.create_subscription(RosImage,  "/yolo/debug_image",                self._on_image,  10)
        self.create_subscription(LaserScan, "/scan",                            self._on_scan,   10)
        self.create_subscription(RosImage,  "/tracked_target/person_image",     self._on_target_image, 10)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap",       self._on_costmap,  10)

        self.spawn_client  = self.create_client(SpawnEntity,  "/spawn_entity")
        self.delete_client = self.create_client(DeleteEntity, "/delete_entity")

        self._cmd_vel_pub = self.create_publisher(Twist,  "/cmd_vel",      10)
        self._reset_pub   = self.create_publisher(String, "/reset_target", 10)
        self._behavior_pub = self.create_publisher(String, "/behavior/command", 10)

        # Telemetry state
        self.target_distance   = -1.0
        self.target_angle      = 0.0
        self.target_visible    = False
        self.target_status     = "searching"
        self.reid_similarity   = 0.0
        self.target_vx         = 0.0
        self.target_vy         = 0.0
        self.behavior_state    = "WAIT_FOR_NAV2"
        self.robot_x           = 0.0
        self.robot_y           = 0.0
        self.robot_yaw         = 0.0
        self.robot_linear_vel  = 0.0
        self.robot_angular_vel = 0.0
        self.cmd_linear        = 0.0
        self.cmd_angular       = 0.0
        self.min_left_range    = 10.0
        self.min_right_range   = 10.0
        self.latest_cv_image   = None
        self.latest_target_image = None
        self.latest_costmap_grid = None

        self.spawned_obstacles = []
        self.obstacle_pubs     = {}
        self.obstacle_counter  = 0
        self._topics_snapshot  = None

        self.create_timer(2.0, self._poll_topics)
        self.create_timer(0.1, self._drive_dynamic_obstacles)

    def _on_target(self, msg):
        try:
            data = json.loads(msg.data)
            self.target_distance = float(data.get("distance",      -1.0))
            self.target_angle    = float(data.get("angle",          0.0))
            self.target_visible  = bool (data.get("visible",       False))
            self.target_status   = str  (data.get("status",  "searching"))
            self.reid_similarity = float(data.get("reid_similarity", 0.0))
            self.target_vx       = float(data.get("vx",             0.0))
            self.target_vy       = float(data.get("vy",             0.0))
        except Exception:
            pass
        self.ui_callback()

    def _on_state(self, msg):
        self.behavior_state = msg.data
        self.ui_callback()

    def _on_odom(self, msg):
        self.robot_linear_vel  = msg.twist.twist.linear.x
        self.robot_angular_vel = msg.twist.twist.angular.z
        p = msg.pose.pose
        self.robot_x = p.position.x
        self.robot_y = p.position.y
        q = p.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)
        self.ui_callback()

    def _on_cmd(self, msg):
        self.cmd_linear  = msg.linear.x
        self.cmd_angular = msg.angular.z
        self.ui_callback()

    def _on_image(self, msg):
        try:
            self.latest_cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            pass
        self.ui_callback()

    def _on_target_image(self, msg):
        try:
            self.latest_target_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            pass
        self.ui_callback()

    def _on_costmap(self, msg):
        self.latest_costmap_grid = msg
        self.ui_callback()

    def _on_scan(self, msg):
        self._scan_store.update(msg)
        left_min  = 10.0
        right_min = 10.0
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or not (msg.range_min <= r <= msg.range_max):
                continue
            angle = math.atan2(
                math.sin(msg.angle_min + i * msg.angle_increment),
                math.cos(msg.angle_min + i * msg.angle_increment),
            )
            if abs(angle) < 0.785:
                if angle >= 0.0:
                    left_min  = min(left_min,  r)
                else:
                    right_min = min(right_min, r)
        self.min_left_range  = left_min
        self.min_right_range = right_min
        self.ui_callback()

    def _poll_topics(self):
        self._topics_snapshot = self.get_topic_names_and_types()
        self.ui_callback(topics=self._topics_snapshot)

    def _drive_dynamic_obstacles(self):
        now = time.time()
        for name in list(self.spawned_obstacles):
            if name not in self.obstacle_pubs:
                self.obstacle_pubs[name] = self.create_publisher(
                    Twist, f"/{name}/cmd_vel", 10)
            try:
                idx = int(name.split("_")[-1])
            except ValueError:
                idx = 0
            phase = idx * 1.5
            twist = Twist()
            twist.linear.x = 0.25 * math.sin(now * 0.6 + phase)
            self.obstacle_pubs[name].publish(twist)

    def spawn_obstacle(self, in_front=True):
        if not self.spawn_client.wait_for_service(timeout_sec=1.0):
            return "Gazebo spawn service not available!"
        self.obstacle_counter += 1
        name = f"dynamic_obstacle_{self.obstacle_counter}"
        if in_front:
            ox = self.robot_x + 1.5 * math.cos(self.robot_yaw)
            oy = self.robot_y + 1.5 * math.sin(self.robot_yaw)
        else:
            ox = self.robot_x + 2.0 * math.cos(self.robot_yaw + 0.5)
            oy = self.robot_y + 2.0 * math.sin(self.robot_yaw + 0.5)
        sdf_xml = f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>false</static>
    <link name="link">
      <collision name="collision">
        <geometry><cylinder><radius>0.22</radius><length>1.80</length></cylinder></geometry>
      </collision>
      <visual name="visual">
        <geometry><cylinder><radius>0.22</radius><length>1.80</length></cylinder></geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
          <specular>0.3 0.3 0.3 1</specular>
        </material>
      </visual>
    </link>
    <plugin name="planar_move_{name}" filename="libgazebo_ros_planar_move.so">
      <ros>
        <namespace>/{name}</namespace>
      </ros>
      <update_rate>10</update_rate>
      <publish_rate>10</publish_rate>
    </plugin>
  </model>
</sdf>"""
        req = SpawnEntity.Request()
        req.name = name
        req.xml  = sdf_xml
        req.initial_pose.position.x = float(ox)
        req.initial_pose.position.y = float(oy)
        req.initial_pose.position.z = 0.90
        req.reference_frame         = "map"
        self.spawn_client.call_async(req)
        self.spawned_obstacles.append(name)
        return f"Spawning dynamic '{name}' at x={ox:.2f}, y={oy:.2f}"

    def clear_obstacles(self):
        if not self.delete_client.wait_for_service(timeout_sec=1.0):
            return "Gazebo delete service not available!"
        messages = []
        for name in list(self.spawned_obstacles):
            req      = DeleteEntity.Request()
            req.name = name
            self.delete_client.call_async(req)
            self.spawned_obstacles.remove(name)
            self.obstacle_pubs.pop(name, None)
            messages.append(f"Deleted '{name}'")
        return "\n".join(messages) if messages else "No obstacles to delete."

    def emergency_stop(self):
        self._cmd_vel_pub.publish(Twist())
        msg = String()
        msg.data = "STOP"
        self._behavior_pub.publish(msg)

    def reset_target(self):
        msg      = String()
        msg.data = "reset"
        self._reset_pub.publish(msg)

    def get_scan_snapshot(self):
        return self._scan_store.snapshot()


# 
# DASHBOARD APPLICATION
# 
class DashboardApp:

    _SEARCH_STATES = {"SEARCH_GO_TO_LAST", "SEARCH_SPIN",
                      "SEARCH_WAIT_REID",  "SEARCH_RETURN_HOME"}

    _STATE_COLOURS = {
        "FOLLOW":             "#2ED573",
        "SEARCH_GO_TO_LAST":  "#FFA502",
        "SEARCH_SPIN":        "#FFA502",
        "SEARCH_WAIT_REID":   "#A29BFE",
        "SEARCH_RETURN_HOME": "#FF4757",
        "ACQUIRE":            "#00CEC9",
        "WAIT_FOR_NAV2":      "#8B949E",
    }

    def __init__(self, root):
        self.root = root
        self.root.title("TurtleBot3 Waffle  Person Follower Dashboard")
        self.root.geometry("1600x900")
        self.root.resizable(True, True)
        self.root.configure(bg=C_BG)

        self._ui_update_needed = False
        self._new_topics       = None
        self._pulse_on         = True
        self._event_log        = deque(maxlen=20)
        self._prev_state       = ""

        self._build_top_bar()
        self._build_body()

        rclpy.init(args=None)
        self.node         = DashboardNode(self._trigger_update)
        self._spin_thread = threading.Thread(target=self._spin_ros2, daemon=True)
        self._spin_thread.start()

        self.root.after(100,  self._process_queue)
        self.root.after(500,  self._animate_badge)
        self.root.after(1000, self._update_clock)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    #  TOP BAR 
    def _build_top_bar(self):
        bar = tk.Frame(self.root, bg=C_TOPBAR, height=52)
        bar.pack(fill=tk.X, side=tk.TOP)
        bar.pack_propagate(False)
        tk.Label(bar, text="  TurtleBot3 Waffle  |  Person Follower v2",
                 bg=C_TOPBAR, fg=C_ACCENT,
                 font=(F_HEAD, 16, "bold")).pack(side=tk.LEFT, padx=20, pady=10)
        self._ros_dot = tk.Label(bar, text="[OK]", bg=C_TOPBAR, fg=C_SUCCESS,
                                 font=(F_HEAD, 12, "bold"))
        self._ros_dot.pack(side=tk.RIGHT, padx=8, pady=10)
        tk.Label(bar, text="ROS2", bg=C_TOPBAR, fg=C_MUTED,
                 font=(F_MONO, 10)).pack(side=tk.RIGHT, padx=(0, 2), pady=10)
        self._lbl_clock = tk.Label(bar, text="00:00:00",
                                   bg=C_TOPBAR, fg=C_FG,
                                   font=(F_MONO, 13))
        self._lbl_clock.pack(side=tk.RIGHT, padx=20, pady=10)

    #  BODY 
    def _build_body(self):
        body = tk.Frame(self.root, bg=C_BG)
        body.pack(fill=tk.BOTH, expand=True, padx=10, pady=8)
        left   = tk.Frame(body, bg=C_BG, width=380)
        center = tk.Frame(body, bg=C_BG)
        right  = tk.Frame(body, bg=C_BG, width=380)
        left.pack  (side=tk.LEFT, fill=tk.Y,    padx=(0, 8))
        center.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 8))
        right.pack (side=tk.LEFT, fill=tk.Y)
        left.pack_propagate(False)
        right.pack_propagate(False)
        self._build_left(left)
        self._build_center(center)
        self._build_right(right)

    #  LEFT PANEL 
    def _build_left(self, parent):
        self._mk_card(parent, " Behavior State").pack(fill=tk.X, pady=(0, 6))
        sc = self._last_inner
        self._lbl_state = tk.Label(sc, text="WAIT_FOR_NAV2",
                                   bg=C_PANEL, fg=C_MUTED,
                                   font=(F_HEAD, 18, "bold"), height=2)
        self._lbl_state.pack(fill=tk.X, padx=8, pady=8)

        self._mk_card(parent, " Target Telemetry").pack(fill=tk.X, pady=(0, 6))
        tel = self._last_inner
        self._lbl_visible = self._kv(tel, "Visible",  "False")
        self._lbl_status  = self._kv(tel, "Status",   "searching")
        self._lbl_dist    = self._kv(tel, "Distance", "--.-- m")
        self._lbl_angle   = self._kv(tel, "Angle",    "--.-- rad")

        tk.Label(tel, text="Distance 05 m  (safe  1.4 m)",
                 bg=C_PANEL, fg=C_MUTED, font=(F_MONO, 7)).pack(anchor=tk.W, padx=10)
        self._dist_canvas = tk.Canvas(tel, bg=C_PANEL, height=18, highlightthickness=0)
        self._dist_canvas.pack(fill=tk.X, padx=10, pady=(0, 6))
        self._dist_canvas.bind("<Configure>", lambda e: self._draw_dist_bar())

        tk.Label(tel, text="Re-ID Similarity",
                 bg=C_PANEL, fg=C_MUTED, font=(F_MONO, 8)).pack()
        self._reid_canvas = tk.Canvas(tel, bg=C_PANEL, width=180, height=110,
                                      highlightthickness=0)
        self._reid_canvas.pack(pady=(0, 4))
        self._draw_reid_gauge(0.0)

        self._mk_card(parent, " Target Velocity Vectors").pack(fill=tk.X, pady=(0, 6))
        vel = self._last_inner
        tk.Label(vel, text="Vx  (forward/back)",
                 bg=C_PANEL, fg=C_MUTED, font=(F_MONO, 8)).pack(anchor=tk.W, padx=10)
        self._vx_canvas = tk.Canvas(vel, bg=C_PANEL, height=18, highlightthickness=0)
        self._vx_canvas.pack(fill=tk.X, padx=10, pady=(0, 4))
        self._vx_canvas.bind("<Configure>", lambda e: self._draw_vel_bars())
        tk.Label(vel, text="Vy  (lateral)",
                 bg=C_PANEL, fg=C_MUTED, font=(F_MONO, 8)).pack(anchor=tk.W, padx=10)
        self._vy_canvas = tk.Canvas(vel, bg=C_PANEL, height=18, highlightthickness=0)
        self._vy_canvas.pack(fill=tk.X, padx=10, pady=(0, 8))
        self._vy_canvas.bind("<Configure>", lambda e: self._draw_vel_bars())

        self._mk_card(parent, "Tracked Person (Re-ID Target)").pack(fill=tk.X, pady=(0, 6))
        person = self._last_inner
        self._lbl_person = tk.Label(person, text="No Target Locked",
                                    bg="#010409", fg=C_MUTED, font=(F_HEAD, 10))
        self._lbl_person.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)
        self._lbl_person.config(width=25, height=15)

        self._mk_card(parent, " Odometry & LiDAR").pack(fill=tk.X, pady=(0, 6))
        odom = self._last_inner
        self._lbl_pos   = self._kv(odom, "Position",  "x=0.00 y=0.00 m")
        self._lbl_yaw   = self._kv(odom, "Yaw",       "0.00 rad")
        self._lbl_speed = self._kv(odom, "Act Speed",  "lin=0.00 ang=0.00")
        self._lbl_cmdv  = self._kv(odom, "Cmd Vel",   "lin=0.00 ang=0.00")
        self._lbl_lidar = self._kv(odom, "LiDAR L/R", "10.00 / 10.00 m")

        self._compass_canvas = tk.Canvas(odom, bg=C_PANEL, width=90, height=90,
                                         highlightthickness=0)
        self._compass_canvas.pack(pady=4)
        self._draw_compass(0.0, 0.0, 0.0)

    #  CENTER PANEL 
    def _build_center(self, parent):
        feed_card = tk.Frame(parent, bg=C_BORDER, bd=2, relief=tk.FLAT)
        feed_card.pack(fill=tk.BOTH, expand=True)
        feed_hdr = tk.Frame(feed_card, bg=C_PANEL, height=32)
        feed_hdr.pack(fill=tk.X)
        feed_hdr.pack_propagate(False)
        tk.Label(feed_hdr, text="  YOLO Detection Feed",
                 bg=C_PANEL, fg=C_ACCENT,
                 font=(F_HEAD, 11, "bold")).pack(side=tk.LEFT, padx=10, pady=5)
        self._lbl_status_badge = tk.Label(feed_hdr, text=" SEARCHING",
                                          bg=C_WARNING, fg=C_BG,
                                          font=(F_HEAD, 10, "bold"), padx=6, pady=2)
        self._lbl_status_badge.pack(side=tk.RIGHT, padx=8, pady=4)
        self._lbl_video = tk.Label(feed_card, text="Waiting for YOLO feed",
                                   bg="#010409", fg=C_MUTED, font=(F_HEAD, 12))
        self._lbl_video.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)

        radar_outer = tk.Frame(parent, bg=C_PANEL, bd=1, relief=tk.FLAT)
        radar_outer.pack(fill=tk.X, pady=(6, 0))
        tk.Label(radar_outer, text=" LiDAR Radar (front 180)",
                 bg=C_PANEL, fg=C_TEAL,
                 font=(F_HEAD, 10, "bold")).pack(anchor=tk.W, padx=8, pady=4)
        self._lidar_canvas = tk.Canvas(radar_outer, bg=C_PANEL,
                                       width=200, height=200, highlightthickness=0)
        self._lidar_canvas.pack(side=tk.LEFT, padx=10, pady=6)
        self._draw_lidar_radar([])

        info_fr = tk.Frame(radar_outer, bg=C_PANEL)
        info_fr.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=8)
        tk.Label(info_fr, text="Dynamic Obstacles Active",
                 bg=C_PANEL, fg=C_MUTED, font=(F_MONO, 9)).pack(anchor=tk.W, pady=(12, 2))
        self._lbl_obs_count = tk.Label(info_fr, text="0",
                                       bg=C_PANEL, fg=C_ACCENT,
                                       font=(F_HEAD, 36, "bold"))
        self._lbl_obs_count.pack(anchor=tk.W)
        tk.Label(info_fr, text="obstacles spawned",
                 bg=C_PANEL, fg=C_MUTED, font=(F_MONO, 8)).pack(anchor=tk.W)

    #  RIGHT PANEL 
    def _build_right(self, parent):
        self._mk_card(parent, "Local Costmap").pack(fill=tk.X, pady=(0, 6))
        map_inner = self._last_inner
        self._costmap_canvas = tk.Canvas(map_inner, bg=C_PANEL, width=300, height=300, highlightthickness=0)
        self._costmap_canvas.pack(pady=4)
        self._lbl_costmap_info = tk.Label(map_inner, text="Nav2 Costmap: Not Available", bg=C_PANEL, fg=C_MUTED, font=(F_MONO, 8))
        self._lbl_costmap_info.pack(pady=(0, 4))

        self._mk_card(parent, " Simulation Controls").pack(fill=tk.X, pady=(0, 6))
        ctrl = self._last_inner

        def _btn(text, cmd, bg, fg=C_BG):
            tk.Button(ctrl, text=text, command=cmd,
                      bg=bg, fg=fg, activebackground=bg, activeforeground=fg,
                      font=(F_HEAD, 10, "bold"), relief=tk.FLAT, cursor="hand2",
                      padx=6, pady=6).pack(fill=tk.X, padx=8, pady=3)

        _btn("  Spawn Obstacle  (Front)",            self._act_spawn_front, C_ACCENT)
        _btn("  Spawn Obstacle  (Side)",             self._act_spawn_side,  C_TEAL)
        _btn("  Clear All Obstacles",               self._act_clear,       C_DANGER)
        _btn("  Lock/Unlock Target  (Reset Re-ID)", self._act_reset_reid,  C_PURPLE, fg=C_FG)
        tk.Button(ctrl, text="  EMERGENCY STOP",
                  command=self._act_estop,
                  bg=C_DANGER, fg=C_FG,
                  font=(F_HEAD, 13, "bold"), relief=tk.FLAT, cursor="hand2",
                  padx=6, pady=10).pack(fill=tk.X, padx=8, pady=(6, 10))

        self._mk_card(parent, " Active ROS 2 Topics").pack(fill=tk.X, pady=(0, 6))
        topics_inner = self._last_inner
        self._topic_text = ScrolledText(topics_inner, bg=C_BG, fg=C_ACCENT,
                                        insertbackground=C_FG,
                                        font=(F_MONO, 11), height=9, relief=tk.FLAT)
        self._topic_text.pack(fill=tk.X, padx=6, pady=6)

        self._mk_card(parent, " Event Log (last 20)").pack(fill=tk.BOTH, expand=True)
        log_inner = self._last_inner
        self._log_text = ScrolledText(log_inner, bg=C_BG, fg=C_FG,
                                      insertbackground=C_FG,
                                      font=(F_MONO, 9), height=10, relief=tk.FLAT)
        self._log_text.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)
        self._log_text.tag_config("ts",    foreground=C_MUTED)
        self._log_text.tag_config("state", foreground=C_ACCENT)
        self._log_text.tag_config("warn",  foreground=C_WARNING)
        self._log_text.tag_config("err",   foreground=C_DANGER)

    #  WIDGET BUILDERS 
    def _mk_card(self, parent, title):
        outer = tk.Frame(parent, bg=C_BORDER, bd=1, relief=tk.FLAT)
        hdr   = tk.Frame(outer, bg=C_PANEL, height=26)
        hdr.pack(fill=tk.X)
        hdr.pack_propagate(False)
        tk.Label(hdr, text=title, bg=C_PANEL, fg=C_ACCENT,
                 font=(F_HEAD, 10, "bold")).pack(anchor=tk.W, padx=8, pady=3)
        inner = tk.Frame(outer, bg=C_PANEL)
        inner.pack(fill=tk.BOTH, expand=True)
        self._last_inner = inner
        return outer

    def _kv(self, parent, key, val):
        row = tk.Frame(parent, bg=C_PANEL)
        row.pack(fill=tk.X, padx=8, pady=1)
        tk.Label(row, text=f"{key}:", bg=C_PANEL, fg=C_MUTED,
                 font=(F_MONO, 9), width=11, anchor=tk.W).pack(side=tk.LEFT)
        lbl = tk.Label(row, text=val, bg=C_PANEL, fg=C_FG,
                       font=(F_MONO, 9), anchor=tk.W)
        lbl.pack(side=tk.LEFT, fill=tk.X, expand=True)
        return lbl

    #  CANVAS DRAW METHODS 
    def _draw_dist_bar(self, dist=0.0):
        c = self._dist_canvas
        w = c.winfo_width() or 280
        h = c.winfo_height() or 18
        c.delete("all")
        c.create_rectangle(0, 4, w, h-4, fill=C_BORDER, outline="")
        safe_x = int(w * 1.4 / 5.0)
        c.create_rectangle(safe_x-1, 0, safe_x+1, h, fill=C_SUCCESS, outline="")
        if dist > 0:
            pct    = min(dist / 5.0, 1.0)
            bar_x  = int(w * pct)
            colour = (C_SUCCESS if dist <= 1.4 else
                      C_WARNING if dist <= 2.5 else C_DANGER)
            c.create_rectangle(0, 4, bar_x, h-4, fill=colour, outline="")
        label = f"{dist:.2f} m" if dist >= 0 else "-- m"
        c.create_text(w-4, h//2, text=label, anchor=tk.E, fill=C_FG, font=(F_MONO, 8))

    def _draw_reid_gauge(self, value=0.0):
        c = self._reid_canvas
        c.delete("all")
        cx, cy, r = 90, 95, 70
        c.create_arc(cx-r, cy-r, cx+r, cy+r,
                     start=0, extent=180, outline=C_BORDER, width=12, style=tk.ARC)
        if value > 0:
            extent = int(180 * min(value, 1.0))
            colour = (C_SUCCESS if value >= 0.7 else
                      C_WARNING if value >= 0.4 else C_DANGER)
            c.create_arc(cx-r, cy-r, cx+r, cy+r,
                         start=0, extent=extent, outline=colour, width=12, style=tk.ARC)
        angle_rad = math.radians(180.0 * min(max(value, 0.0), 1.0))
        nx = cx + (r-6) * math.cos(math.pi - angle_rad)
        ny = cy - (r-6) * math.sin(math.pi - angle_rad)
        c.create_line(cx, cy, nx, ny, fill=C_FG, width=2)
        c.create_oval(cx-4, cy-4, cx+4, cy+4, fill=C_FG, outline="")
        c.create_text(cx, cy-20, text=f"{value*100:.0f}%",
                      fill=C_FG, font=(F_MONO, 14, "bold"))
        c.create_text(cx, cy-4, text="Re-ID",
                      fill=C_MUTED, font=(F_MONO, 7))

    def _draw_vel_bars(self, vx=0.0, vy=0.0):
        max_vel = 0.5
        for canvas, value in [(self._vx_canvas, vx), (self._vy_canvas, vy)]:
            c = canvas
            w = c.winfo_width() or 280
            h = c.winfo_height() or 18
            c.delete("all")
            c.create_rectangle(0, 3, w, h-3, fill=C_BORDER, outline="")
            mid = w // 2
            c.create_rectangle(mid-1, 0, mid+1, h, fill=C_MUTED, outline="")
            clamped = max(min(value, max_vel), -max_vel)
            pct     = clamped / max_vel
            if abs(pct) > 0.01:
                bar_w  = int(abs(pct) * (w // 2))
                colour = C_ACCENT if pct > 0 else C_DANGER
                if pct > 0:
                    c.create_rectangle(mid, 3, mid+bar_w, h-3, fill=colour, outline="")
                else:
                    c.create_rectangle(mid-bar_w, 3, mid, h-3, fill=colour, outline="")
            c.create_text(w-4, h//2, text=f"{value:+.2f}",
                          anchor=tk.E, fill=C_FG, font=(F_MONO, 8))

    def _draw_compass(self, x=0.0, y=0.0, yaw=0.0):
        c = self._compass_canvas
        c.delete("all")
        cx, cy, r = 45, 45, 38
        c.create_oval(cx-r, cy-r, cx+r, cy+r,
                      outline=C_BORDER, fill=C_BG, width=2)
        for deg, label in [(0, "N"), (90, "E"), (180, "S"), (270, "W")]:
            rad = math.radians(deg)
            tx  = cx + (r+4) * math.sin(rad)
            ty  = cy - (r+4) * math.cos(rad)
            c.create_text(tx, ty, text=label, fill=C_MUTED, font=(F_MONO, 6))
        arrow_r = r - 8
        ax = cx + arrow_r * math.sin(yaw)
        ay = cy - arrow_r * math.cos(yaw)
        c.create_line(cx, cy, ax, ay, fill=C_ACCENT, width=3, arrow=tk.LAST)
        c.create_text(cx, cy+r+10, text=f"{x:.1f},{y:.1f}",
                      fill=C_MUTED, font=(F_MONO, 6))

    def _draw_lidar_radar(self, scan_pts):
        c = self._lidar_canvas
        c.delete("all")
        cx, cy  = 100, 195
        scale   = 40.0

        safe_r = int(1.4 * scale)
        c.create_arc(cx-safe_r, cy-safe_r, cx+safe_r, cy+safe_r,
                     start=0, extent=180, outline=C_SUCCESS,
                     width=1, style=tk.ARC, dash=(4, 4))
        dng_r = int(0.5 * scale)
        c.create_arc(cx-dng_r, cy-dng_r, cx+dng_r, cy+dng_r,
                     start=0, extent=180, outline=C_DANGER,
                     width=1, style=tk.ARC, dash=(2, 4))
        for m in [1, 2, 3, 4]:
            pr = int(m * scale)
            c.create_arc(cx-pr, cy-pr, cx+pr, cy+pr,
                         start=0, extent=180,
                         outline=C_BORDER, width=1, style=tk.ARC)
            c.create_text(cx+pr+2, cy-4, text=f"{m}m",
                          fill=C_MUTED, font=(F_MONO, 6))
        c.create_arc(cx-100, cy-100, cx+100, cy+100,
                     start=0, extent=180, fill="#0D2A22", outline="", style=tk.PIESLICE)
        c.create_oval(cx-4, cy-4, cx+4, cy+4, fill=C_ACCENT, outline="")

        for (angle, dist) in scan_pts:
            d_px = min(dist * scale, 160)
            px   = cx - d_px * math.sin(angle)
            py   = cy - d_px * math.cos(angle)
            colour = (C_DANGER   if dist < 0.5 else
                      C_WARNING  if dist < 1.4 else C_SUCCESS)
            c.create_oval(px-3, py-3, px+3, py+3, fill=colour, outline="")

        c.create_text(cx,     cy+8,   text="Robot", fill=C_MUTED,  font=(F_MONO, 6))
        c.create_text(cx,     cy-165, text="Fwd",   fill=C_ACCENT, font=(F_MONO, 7))
        c.create_text(cx-95,  cy,     text="L",      fill=C_MUTED,  font=(F_MONO, 7))
        c.create_text(cx+95,  cy,     text="R",      fill=C_MUTED,  font=(F_MONO, 7))

    def _render_costmap(self, grid, robot_x, robot_y):
        import numpy as np
        import cv2
        w, h = grid.info.width, grid.info.height
        res = grid.info.resolution
        origin_x = grid.info.origin.position.x
        origin_y = grid.info.origin.position.y
        data = np.array(grid.data, dtype=np.int8).reshape(h, w)
        
        # Color map: unknown=128,128,128 free=255,255,255 occupied=0,0,0 inflation=0,100,255
        rgb = np.zeros((h, w, 3), dtype=np.uint8)
        rgb[data == -1] = [128, 128, 128]  # unknown
        rgb[data == 0]  = [240, 240, 240]  # free
        rgb[(data > 0) & (data < 100)] = [100, 149, 237]  # inflation (cornflower blue)
        rgb[data >= 100] = [30, 30, 30]    # occupied
        
        # Draw robot position as red dot
        rx_pix = int((robot_x - origin_x) / res)
        ry_pix = h - int((robot_y - origin_y) / res) - 1
        if 0 <= rx_pix < w and 0 <= ry_pix < h:
            cv2.circle(rgb, (rx_pix, ry_pix), 5, (255, 50, 50), -1)
        
        # Resize to fit canvas
        display = cv2.resize(rgb, (300, 300), interpolation=cv2.INTER_NEAREST)
        return display

    #  ANIMATION / CLOCK 
    def _animate_badge(self):
        if hasattr(self, "node"):
            state = self.node.behavior_state
            if state in self._SEARCH_STATES:
                self._pulse_on = not self._pulse_on
                col = self._STATE_COLOURS.get(state, C_WARNING)
                self._lbl_state.config(fg=col if self._pulse_on else C_BG)
        self.root.after(500, self._animate_badge)

    def _update_clock(self):
        self._lbl_clock.config(text=datetime.now().strftime("%H:%M:%S"))
        self.root.after(1000, self._update_clock)

    #  ROS2 SPIN 
    def _spin_ros2(self):
        try:
            rclpy.spin(self.node)
        except Exception:
            pass

    def _trigger_update(self, topics=None):
        self._ui_update_needed = True
        if topics is not None:
            self._new_topics = topics

    #  MAIN UPDATE LOOP 
    def _process_queue(self):
        if self._ui_update_needed:
            self._ui_update_needed = False
            self._refresh_ui()
        self.root.after(100, self._process_queue)

    def _refresh_ui(self):
        nd = self.node

        # Behavior state
        state  = nd.behavior_state
        colour = self._STATE_COLOURS.get(state, C_MUTED)
        self._lbl_state.config(text=state, fg=colour)

        badge_map = {
            "FOLLOW":             ("[ FOLLOWING ]",   C_SUCCESS),
            "SEARCH_GO_TO_LAST":  ("[ SEARCHING ]",   C_WARNING),
            "SEARCH_SPIN":        ("[ SPINNING ]",    C_WARNING),
            "SEARCH_WAIT_REID":   ("[ RE-ID CHECK ]", C_PURPLE),
            "SEARCH_RETURN_HOME": ("[ RETURNING ]",   C_DANGER),
            "ACQUIRE":            ("[ ACQUIRING ]",   C_TEAL),
        }
        badge_txt, badge_bg = badge_map.get(state, ("[ STANDBY ]", C_MUTED))
        self._lbl_status_badge.config(text=badge_txt, bg=badge_bg,
                                      fg=C_BG if badge_bg != C_MUTED else C_FG)

        if state != self._prev_state:
            ts = datetime.now().strftime("%H:%M:%S")
            self._event_log.append((ts, state))
            self._prev_state = state
            self._refresh_log()

        # Telemetry
        self._lbl_visible.config(
            text="Yes" if nd.target_visible else "No",
            fg=C_SUCCESS if nd.target_visible else C_DANGER)
        self._lbl_status.config(text=nd.target_status)
        self._lbl_dist.config(
            text=f"{nd.target_distance:.2f} m" if nd.target_distance >= 0 else "-- m")
        self._lbl_angle.config(text=f"{nd.target_angle:.2f} rad")

        self._draw_dist_bar(max(nd.target_distance, 0.0))
        self._draw_reid_gauge(nd.reid_similarity)
        self._draw_vel_bars(nd.target_vx, nd.target_vy)

        # Odometry
        self._lbl_pos.config(text=f"x={nd.robot_x:.2f}  y={nd.robot_y:.2f} m")
        self._lbl_yaw.config(
            text=f"{nd.robot_yaw:.2f} rad ({math.degrees(nd.robot_yaw):.1f})")
        self._lbl_speed.config(
            text=f"lin={nd.robot_linear_vel:.2f} ang={nd.robot_angular_vel:.2f}")
        self._lbl_cmdv.config(
            text=f"lin={nd.cmd_linear:.2f} ang={nd.cmd_angular:.2f}")
        lidar_min = min(nd.min_left_range, nd.min_right_range)
        self._lbl_lidar.config(
            text=f"L={nd.min_left_range:.2f}  R={nd.min_right_range:.2f} m",
            fg=(C_DANGER  if lidar_min < 0.5 else
                C_WARNING if lidar_min < 1.0 else C_FG))
        self._draw_compass(nd.robot_x, nd.robot_y, nd.robot_yaw)

        # LiDAR radar
        (ranges, angle_min, angle_inc, r_min, r_max) = nd.get_scan_snapshot()
        front_pts = []
        for i, r in enumerate(ranges):
            if not math.isfinite(r) or not (r_min <= r <= r_max):
                continue
            a = math.atan2(
                math.sin(angle_min + i * angle_inc),
                math.cos(angle_min + i * angle_inc),
            )
            if abs(a) <= math.pi / 2:
                front_pts.append((a, r))
        self._draw_lidar_radar(front_pts)

        # Obstacle count
        self._lbl_obs_count.config(text=str(len(nd.spawned_obstacles)))

        # YOLO feed
        import cv2
        if nd.latest_cv_image is not None and PIL_AVAILABLE:
            try:
                rgb     = cv2.cvtColor(nd.latest_cv_image, cv2.COLOR_BGR2RGB)
                resized = cv2.resize(rgb, (540, 540))
                pil_img = PilImage.fromarray(resized)
                tk_img  = ImageTk.PhotoImage(image=pil_img)
                self._lbl_video.config(image=tk_img, text="")
                self._lbl_video.image = tk_img
            except Exception:
                pass
        elif nd.latest_cv_image is not None and not PIL_AVAILABLE:
            self._lbl_video.config(
                text="Live feed active.\nInstall Pillow:\n  pip install Pillow")

        # Target Image
        if nd.latest_target_image is not None and PIL_AVAILABLE:
            try:
                rgb = cv2.cvtColor(nd.latest_target_image, cv2.COLOR_BGR2RGB)
                # Resize the image to make it clearly visible on the dashboard
                resized_target = cv2.resize(rgb, (200, 300), interpolation=cv2.INTER_CUBIC)
                pil_img = PilImage.fromarray(resized_target)
                tk_img = ImageTk.PhotoImage(image=pil_img)
                self._lbl_person.config(image=tk_img, text="")
                self._lbl_person.image = tk_img
            except Exception:
                pass
        else:
            if not getattr(self, '_target_img_set_blank', False):
                self._lbl_person.config(image="", text="No Target Locked", width=25, height=15)
                self._target_img_set_blank = True
        if nd.latest_target_image is not None:
            self._target_img_set_blank = False

        # Costmap
        if nd.latest_costmap_grid is not None and PIL_AVAILABLE:
            costmap_img = self._render_costmap(nd.latest_costmap_grid, nd.robot_x, nd.robot_y)
            try:
                pil_img = PilImage.fromarray(costmap_img)
                tk_img = ImageTk.PhotoImage(image=pil_img)
                self._costmap_canvas.delete("all")
                self._costmap_canvas.create_image(0, 0, image=tk_img, anchor=tk.NW)
                self._costmap_canvas.image = tk_img
                res = nd.latest_costmap_grid.info.resolution
                self._lbl_costmap_info.config(text=f"Resolution: {res:.2f} m/px")
            except Exception:
                pass

        # Topic monitor
        if self._new_topics is not None:
            self._topic_text.delete("1.0", tk.END)
            for t_name, t_types in sorted(self._new_topics):
                self._topic_text.insert(tk.END, f"{t_name}\n  [{', '.join(t_types)}]\n")
            self._new_topics = None

        # ROS2 dot
        self._ros_dot.config(fg=C_SUCCESS if rclpy.ok() else C_DANGER)

    def _refresh_log(self):
        self._log_text.delete("1.0", tk.END)
        for ts, evt in self._event_log:
            self._log_text.insert(tk.END, f"[{ts}] ", "ts")
            tag = ("err"   if ("RETURN" in evt or "STOP" in evt) else
                   "warn"  if "SEARCH" in evt else
                   "state")
            self._log_text.insert(tk.END, f"{evt}\n", tag)
        self._log_text.see(tk.END)

    #  BUTTON ACTIONS 
    def _act_spawn_front(self):
        msg = self.node.spawn_obstacle(in_front=True)
        self._event_log.append((datetime.now().strftime("%H:%M:%S"), f"SPAWN_FRONT: {msg}"))
        self._refresh_log()
        messagebox.showinfo("Spawn Obstacle", msg)

    def _act_spawn_side(self):
        msg = self.node.spawn_obstacle(in_front=False)
        self._event_log.append((datetime.now().strftime("%H:%M:%S"), f"SPAWN_SIDE: {msg}"))
        self._refresh_log()
        messagebox.showinfo("Spawn Obstacle", msg)

    def _act_clear(self):
        msg = self.node.clear_obstacles()
        self._event_log.append((datetime.now().strftime("%H:%M:%S"), "CLEAR_OBSTACLES"))
        self._refresh_log()
        messagebox.showinfo("Clear Obstacles", msg)

    def _act_reset_reid(self):
        self.node.reset_target()
        self._event_log.append((datetime.now().strftime("%H:%M:%S"), "RESET_REID"))
        self._refresh_log()

    def _act_estop(self):
        self.node.emergency_stop()
        self._event_log.append((datetime.now().strftime("%H:%M:%S"), "EMERGENCY_STOP"))
        self._refresh_log()

    #  CLEANUP 
    def _on_close(self):
        try:
            self.node.emergency_stop()
        except Exception:
            pass
        try:
            self.node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        self.root.destroy()


# 
def main():
    root = tk.Tk()
    DashboardApp(root)
    root.mainloop()

if __name__ == "__main__":
    main()
