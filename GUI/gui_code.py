#!/usr/bin/env python3
"""
============================================================
 ELDER HELPER — ROBOT CONTROL CENTER
============================================================
Semi-autonomous mobile robot dashboard (CustomTkinter + roslibpy)

Features:
  - Live camera feed with detection bounding box + confidence overlay
  - Robot pose, mission status, finger count
  - "Hologram" top-down orientation display (live heading)
  - Route map: trail from home, distance travelled, obstacles as red dots
  - Manual-control flag (arms/disarms the drive pad)
  - Manual drive control (keyboard + on-screen pad)
  - Pi system health: CPU / RAM / Temp / GPU memory
  - External Mechanism Control flag (/extra_man_flag)

Topic contract (adjust to match your stack):
  /robot_odom                    geometry_msgs/Pose2D
                                  (x, y in meters from home/origin, theta in radians)
  /vision/finger_count           std_msgs/Int32
  /vision/mission_command        std_msgs/String      -> label shown on camera + prediction card
  /vision/detection_info         std_msgs/Float32MultiArray
                                  data = [class_id, confidence, x1, y1, x2, y2]
  /vision/raw_stream/compressed  sensor_msgs/CompressedImage
  /vision/obstacles               std_msgs/Float32MultiArray
                                  data = [x1, y1, x2, y2, ...]  (home-frame meters, same frame as /robot_odom)
  /pi_stats                       std_msgs/Float32MultiArray
                                  data = [cpu_pct, ram_pct, cpu_temp_c, gpu_mem_mb,
                                          battery_pct, battery_voltage]
                                  (use the included pi_system_monitor.py to publish this)
  /cmd_vel                         geometry_msgs/Twist (published from this GUI)
  /manual_control_enable           std_msgs/Bool (published from this GUI)
  /extra_man_flag                  std_msgs/Bool (published from this GUI)
============================================================
"""


import customtkinter as ctk
import tkinter as tk
import threading
import time
import math
import base64
from io import BytesIO
from collections import deque
from datetime import datetime
import os

try:
    from PIL import Image, ImageTk, ImageDraw, ImageFont
except ImportError:
    raise SystemExit(
        "Pillow is required for this GUI. Install it with: pip install Pillow")

# Overlay font for the camera bounding box label
try:
    FONT_OVERLAY = ImageFont.truetype(
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 16)
except Exception:
    FONT_OVERLAY = ImageFont.load_default()

ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("dark-blue")

# ==================== THEMES ====================
THEMES = {
    "Futuristic": {
        "BG": "#0a0807", "BG_HEADER": "#120d0a", "CARD_BG": "#181210",
        "CARD_BORDER": "#2e231b", "ACCENT": "#d4a574", "ACCENT_LT": "#ecd4b3",
        "GOLD": "#c9a227", "TEXT": "#f2e8dc", "TEXT_DIM": "#9c8b7a",
        "SUCCESS": "#9bbf8a", "DANGER": "#cf6354", "HOLO": "#7fe7ff", "HOLO_DIM": "#2a4a52",
        "ACCENT_RGB": (212, 165, 116), "TEXT_RGB": (16, 12, 9)
    },
    "Girly Pastel": {
        "BG": "#1a1215", "BG_HEADER": "#261a1e", "CARD_BG": "#2a1e23",
        "CARD_BORDER": "#4a333d", "ACCENT": "#ffb6c1", "ACCENT_LT": "#ffd1dc",
        "GOLD": "#ff69b4", "TEXT": "#fff0f5", "TEXT_DIM": "#d8b4c0",
        "SUCCESS": "#98fb98", "DANGER": "#ff6347", "HOLO": "#ff69b4", "HOLO_DIM": "#8b3a62",
        "ACCENT_RGB": (255, 182, 193), "TEXT_RGB": (26, 18, 21)
    },
    "Cyberpunk": {
        "BG": "#050510", "BG_HEADER": "#0a0a1a", "CARD_BG": "#0f0f25",
        "CARD_BORDER": "#2a1040", "ACCENT": "#00ffcc", "ACCENT_LT": "#99ffeb",
        "GOLD": "#ff00ff", "TEXT": "#e0e0ff", "TEXT_DIM": "#606080",
        "SUCCESS": "#39ff14", "DANGER": "#ff003c", "HOLO": "#00ffcc", "HOLO_DIM": "#004d40",
        "ACCENT_RGB": (0, 255, 204), "TEXT_RGB": (5, 5, 16)
    },
    "Ocean Deep": {
        "BG": "#030a12", "BG_HEADER": "#06121f", "CARD_BG": "#091726",
        "CARD_BORDER": "#163152", "ACCENT": "#00a8ff", "ACCENT_LT": "#80d4ff",
        "GOLD": "#00d2d3", "TEXT": "#e1f5fe", "TEXT_DIM": "#5c8aab",
        "SUCCESS": "#1dd1a1", "DANGER": "#ff6b6b", "HOLO": "#00a8ff", "HOLO_DIM": "#004366",
        "ACCENT_RGB": (0, 168, 255), "TEXT_RGB": (3, 10, 18)
    },
    "Forest Explorer": {
        "BG": "#0c120c", "BG_HEADER": "#131a13", "CARD_BG": "#182118",
        "CARD_BORDER": "#2d3d2d", "ACCENT": "#b8e994", "ACCENT_LT": "#dcf4cb",
        "GOLD": "#f8c291", "TEXT": "#e8ece8", "TEXT_DIM": "#869986",
        "SUCCESS": "#78e08f", "DANGER": "#e55039", "HOLO": "#b8e994", "HOLO_DIM": "#3e5c3e",
        "ACCENT_RGB": (184, 233, 148), "TEXT_RGB": (12, 18, 12)
    },
    "Crimson Forge": {
        "BG": "#120505", "BG_HEADER": "#1a0808", "CARD_BG": "#220a0a",
        "CARD_BORDER": "#401313", "ACCENT": "#ff4d4d", "ACCENT_LT": "#ff9999",
        "GOLD": "#ff9f43", "TEXT": "#f5e6e6", "TEXT_DIM": "#a67c7c",
        "SUCCESS": "#10ac84", "DANGER": "#ee5253", "HOLO": "#ff4d4d", "HOLO_DIM": "#661414",
        "ACCENT_RGB": (255, 77, 77), "TEXT_RGB": (18, 5, 5)
    }
}


class RobotGUI(ctk.CTk):
    def __init__(self):
        super().__init__()

        self.title("Elder Helper — Robot Control Center")
        self.geometry("1500x900")
        self.minsize(1320, 820)

        # Load default theme
        self.current_theme_name = tk.StringVar(value="Futuristic")
        self.theme = THEMES[self.current_theme_name.get()]
        self.configure(fg_color=self.theme["BG"])

        # ---- Robot Data ----
        self.mission_status = tk.StringVar(value="IDLE")
        self.error_state = tk.StringVar(value="No Errors")
        self.is_connected = tk.BooleanVar(value=False)
        self.finger_count = tk.StringVar(value="--")
        self.prediction = tk.StringVar(value="--")
        self.robot_pose = tk.StringVar(value="X: 0.00   Y: 0.00   θ: 0.0°")

        # ---- Detection state ----
        self._det_lock = threading.Lock()
        self.det_confidence = 0.0
        self.det_bbox = [0.0, 0.0, 0.0, 0.0]

        # ---- Navigation / map state ----
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.heading = 0.0
        self.trail = deque(maxlen=400)
        self.obstacles = []
        self._sweep_angle = 0.0
        self.holo_size = 220
        self.map_size = 220

        # ---- Manual control ----
        self.manual_mode = tk.BooleanVar(value=False)
        self.ext_mech_mode = tk.BooleanVar(
            value=False)  # NEW: Ext Mechanism Flag

        # ---- Camera / Render ----
        self.cam_w, self.cam_h = 520, 400
        self._cam_photo = None
        self._last_frame_time = 0.0
        # === CHANGED FROM .PNG TO .JPG ===
        self.robot_image_path = r"C:\Users\MohamedAshrafRadwanT\Downloads\GUI\robot_pic.jpg"

        # ---- ROS bridge ----
        self.ros = None
        self.ros_ip = "192.168.30.102"
        self.ros_port = 9090

        # Widget references for easy theme updating
        self.dynamic_widgets = []

        self._build_ui()
        self._bind_keys()
        self._update_clock()
        self._check_feed()
        self._draw_map()
        self._animate_hologram()

    # ==================== UI BUILD ====================
    def _build_ui(self):
        self._build_header()

        main = ctk.CTkFrame(self, fg_color="transparent")
        main.pack(fill="both", expand=True, padx=12, pady=(8, 12))
        for i, w in enumerate([3, 4, 2, 2]):
            main.columnconfigure(i, weight=w)
        main.rowconfigure(0, weight=1)

        col_telemetry = ctk.CTkScrollableFrame(
            main, fg_color="transparent",
            scrollbar_button_color=self.theme["CARD_BORDER"],
            scrollbar_button_hover_color=self.theme["ACCENT"])
        col_telemetry.grid(row=0, column=0, sticky="nsew", padx=(0, 5))

        col_camera = ctk.CTkFrame(main, fg_color="transparent")
        col_camera.grid(row=0, column=1, sticky="nsew", padx=5)

        col_system = ctk.CTkFrame(main, fg_color="transparent")
        col_system.grid(row=0, column=2, sticky="nsew", padx=5)

        col_control = ctk.CTkFrame(main, fg_color="transparent")
        col_control.grid(row=0, column=3, sticky="nsew", padx=(5, 0))

        self._build_telemetry(col_telemetry)
        self._build_camera(col_camera)
        self._build_system(col_system)
        self._build_control(col_control)

    def _build_header(self):
        self.header = ctk.CTkFrame(
            self, fg_color=self.theme["BG_HEADER"], corner_radius=0, height=64)
        self.header.pack(fill="x")
        self.header.pack_propagate(False)

        title_frame = ctk.CTkFrame(self.header, fg_color="transparent")
        title_frame.pack(side="left", padx=22)

        logo = ctk.CTkLabel(title_frame, text="⬡", font=ctk.CTkFont(
            "Courier", 24, "bold"), text_color=self.theme["GOLD"])
        logo.pack(side="left", padx=(0, 10))
        self.dynamic_widgets.append((logo, "text_color", "GOLD"))

        text_col = ctk.CTkFrame(title_frame, fg_color="transparent")
        text_col.pack(side="left")

        t1 = ctk.CTkLabel(text_col, text="ELDER HELPER", font=ctk.CTkFont(
            "Courier", 18, "bold"), text_color=self.theme["TEXT"])
        t1.pack(anchor="w")
        self.dynamic_widgets.append((t1, "text_color", "TEXT"))

        t2 = ctk.CTkLabel(text_col, text="ROBOT CONTROL CENTER", font=ctk.CTkFont(
            "Courier", 10), text_color=self.theme["ACCENT"])
        t2.pack(anchor="w")
        self.dynamic_widgets.append((t2, "text_color", "ACCENT"))

        right_frame = ctk.CTkFrame(self.header, fg_color="transparent")
        right_frame.pack(side="right", padx=22)

        # Theme Selector
        theme_menu = ctk.CTkOptionMenu(right_frame, variable=self.current_theme_name, values=list(THEMES.keys()),
                                       command=self._apply_theme, fg_color=self.theme["CARD_BG"],
                                       button_color=self.theme["CARD_BORDER"], button_hover_color=self.theme["ACCENT"])
        theme_menu.pack(side="right", padx=(14, 0))
        self.dynamic_widgets.append((theme_menu, "fg_color", "CARD_BG"))
        self.dynamic_widgets.append(
            (theme_menu, "button_color", "CARD_BORDER"))
        self.dynamic_widgets.append(
            (theme_menu, "button_hover_color", "ACCENT"))

        self.conn_btn = ctk.CTkButton(right_frame, text="⚡ CONNECT", font=ctk.CTkFont("Courier", 12, "bold"),
                                      fg_color="#1a2418", hover_color="#26331f",
                                      text_color=self.theme["SUCCESS"], border_color=self.theme["SUCCESS"],
                                      border_width=1, width=140, height=34, corner_radius=8,
                                      command=self._toggle_connect)
        self.conn_btn.pack(side="right", padx=(14, 0))

        self.conn_label = ctk.CTkLabel(right_frame, text="● OFFLINE", font=ctk.CTkFont(
            "Courier", 11, "bold"), text_color=self.theme["DANGER"])
        self.conn_label.pack(side="right", padx=14)

        self.clock_label = ctk.CTkLabel(
            right_frame, text="--:--:--", font=ctk.CTkFont("Courier", 13), text_color=self.theme["TEXT_DIM"])
        self.clock_label.pack(side="right", padx=14)
        self.dynamic_widgets.append(
            (self.clock_label, "text_color", "TEXT_DIM"))

    def _build_telemetry(self, parent):
        pose_card = self._card(parent, "ROBOT POSE", "📍")
        self.pose_label = ctk.CTkLabel(pose_card, textvariable=self.robot_pose, font=ctk.CTkFont(
            "Courier", 13, "bold"), text_color=self.theme["ACCENT"])
        self.pose_label.pack(pady=(0, 14), padx=14)
        self.dynamic_widgets.append((self.pose_label, "text_color", "ACCENT"))

        mission_card = self._card(parent, "MISSION STATUS", "📊")
        self.mission_label = ctk.CTkLabel(mission_card, textvariable=self.mission_status, font=ctk.CTkFont(
            "Courier", 18, "bold"), text_color=self.theme["GOLD"])
        self.mission_label.pack(pady=(0, 14))
        self.dynamic_widgets.append((self.mission_label, "text_color", "GOLD"))

        error_card = self._card(parent, "SYSTEM ERROR", "⚠️")
        self.err_label = ctk.CTkLabel(error_card, textvariable=self.error_state, font=ctk.CTkFont(
            "Courier", 12), text_color=self.theme["SUCCESS"], wraplength=240, justify="left")
        self.err_label.pack(pady=(0, 14), padx=14)

        holo_card = self._card(parent, "NAVIGATION HOLOGRAM", "🛰️")
        self.holo_canvas = tk.Canvas(holo_card, width=self.holo_size, height=self.holo_size,
                                     bg="#050403", highlightthickness=1, highlightbackground=self.theme["CARD_BORDER"])
        self.holo_canvas.pack(padx=14, pady=(0, 14))

        map_card = self._card(parent, "ROUTE MAP", "🗺️")
        self.map_canvas = tk.Canvas(map_card, width=self.map_size, height=self.map_size,
                                    bg="#050403", highlightthickness=1, highlightbackground=self.theme["CARD_BORDER"])
        self.map_canvas.pack(padx=14, pady=(0, 6))

        legend = ctk.CTkFrame(map_card, fg_color="transparent")
        legend.pack(fill="x", padx=14, pady=(0, 14))
        l1 = ctk.CTkLabel(legend, text="● HOME", font=ctk.CTkFont(
            "Courier", 9), text_color=self.theme["GOLD"])
        l1.pack(side="left", padx=(0, 10))
        l2 = ctk.CTkLabel(legend, text="▲ ROBOT", font=ctk.CTkFont(
            "Courier", 9), text_color=self.theme["HOLO"])
        l2.pack(side="left", padx=(0, 10))
        l3 = ctk.CTkLabel(legend, text="● OBSTACLE", font=ctk.CTkFont(
            "Courier", 9), text_color=self.theme["DANGER"])
        l3.pack(side="left")
        self.dynamic_widgets.extend(
            [(l1, "text_color", "GOLD"), (l2, "text_color", "HOLO"), (l3, "text_color", "DANGER")])

    def _build_camera(self, parent):
        cam_card = self._card(parent, "LIVE CAMERA FEED", "📷")
        self.cam_canvas = tk.Canvas(cam_card, width=self.cam_w, height=self.cam_h, bg="#050403",
                                    highlightthickness=1, highlightbackground=self.theme["CARD_BORDER"])
        self.cam_canvas.pack(padx=14, pady=(0, 8))
        self.cam_canvas.create_text(self.cam_w // 2, self.cam_h // 2,
                                    text="◌  NO SIGNAL", fill=self.theme["TEXT_DIM"], font=("Courier", 16))

        status_row = ctk.CTkFrame(cam_card, fg_color="transparent")
        status_row.pack(fill="x", padx=14, pady=(0, 14))
        self.feed_status = ctk.CTkLabel(status_row, text="● NO SIGNAL", font=ctk.CTkFont(
            "Courier", 10, "bold"), text_color=self.theme["DANGER"])
        self.feed_status.pack(side="left")

        self.last_update_label = ctk.CTkLabel(
            status_row, text="last frame: --", font=ctk.CTkFont("Courier", 10), text_color=self.theme["TEXT_DIM"])
        self.last_update_label.pack(side="right")
        self.dynamic_widgets.append(
            (self.last_update_label, "text_color", "TEXT_DIM"))

        det_card = self._card(parent, "DETECTION RESULT", "🎯")
        self.pred_label = ctk.CTkLabel(det_card, textvariable=self.prediction, font=ctk.CTkFont(
            "Courier", 18, "bold"), text_color=self.theme["ACCENT_LT"])
        self.pred_label.pack(pady=(0, 8))
        self.dynamic_widgets.append(
            (self.pred_label, "text_color", "ACCENT_LT"))

        conf_row = ctk.CTkFrame(det_card, fg_color="transparent")
        conf_row.pack(fill="x", padx=14)
        c_lbl = ctk.CTkLabel(conf_row, text="CONFIDENCE", font=ctk.CTkFont(
            "Courier", 10), text_color=self.theme["TEXT_DIM"])
        c_lbl.pack(side="left")
        self.dynamic_widgets.append((c_lbl, "text_color", "TEXT_DIM"))

        self.confidence_val = ctk.CTkLabel(
            conf_row, text="--", font=ctk.CTkFont("Courier", 10, "bold"), text_color=self.theme["GOLD"])
        self.confidence_val.pack(side="right")
        self.dynamic_widgets.append(
            (self.confidence_val, "text_color", "GOLD"))

        self.confidence_bar = ctk.CTkProgressBar(
            det_card, height=10, corner_radius=4, fg_color=self.theme["CARD_BORDER"], progress_color=self.theme["GOLD"])
        self.confidence_bar.pack(fill="x", padx=14, pady=(4, 16))
        self.confidence_bar.set(0)
        self.dynamic_widgets.extend(
            [(self.confidence_bar, "fg_color", "CARD_BORDER"), (self.confidence_bar, "progress_color", "GOLD")])

    def _build_system(self, parent):
        sys_card = self._card(parent, "SYSTEM HEALTH (Pi)", "🖥️")
        self.cpu_val, self.cpu_bar = self._stat_bar(
            sys_card, "CPU USAGE", self.theme["ACCENT"])
        self.ram_val, self.ram_bar = self._stat_bar(
            sys_card, "RAM USAGE", self.theme["ACCENT"])
        self.temp_val, self.temp_bar = self._stat_bar(
            sys_card, "CPU TEMP", self.theme["SUCCESS"])

        gpu_row = ctk.CTkFrame(sys_card, fg_color="transparent")
        gpu_row.pack(fill="x", padx=14, pady=(2, 16))
        g_lbl = ctk.CTkLabel(gpu_row, text="GPU MEMORY", font=ctk.CTkFont(
            "Courier", 10), text_color=self.theme["TEXT_DIM"])
        g_lbl.pack(side="left")
        self.dynamic_widgets.append((g_lbl, "text_color", "TEXT_DIM"))

        self.gpu_val = ctk.CTkLabel(
            gpu_row, text="--", font=ctk.CTkFont("Courier", 10, "bold"), text_color=self.theme["ACCENT"])
        self.gpu_val.pack(side="right")
        self.dynamic_widgets.append((self.gpu_val, "text_color", "ACCENT"))

        finger_card = self._card(parent, "FINGER COUNT", "🖐️")
        self.finger_label = ctk.CTkLabel(finger_card, textvariable=self.finger_count, font=ctk.CTkFont(
            "Courier", 44, "bold"), text_color=self.theme["SUCCESS"])
        self.finger_label.pack(pady=(0, 16))

        # ---- 3D View Card ----
        render_card = self._card(parent, "ROBOT 3D MODEL", "🧊")
        self.render_frame = ctk.CTkFrame(
            render_card, fg_color="#050403", corner_radius=8)
        self.render_frame.pack(fill="both", expand=True, padx=14, pady=(0, 14))

        if os.path.exists(self.robot_image_path):
            img = Image.open(self.robot_image_path)
            ctk_img = ctk.CTkImage(
                light_image=img, dark_image=img, size=(200, 200))
            img_label = ctk.CTkLabel(self.render_frame, image=ctk_img, text="")
            img_label.pack(pady=10)
        else:
            no_img_label = ctk.CTkLabel(self.render_frame, text=f"Missing\n{self.robot_image_path}",
                                        font=ctk.CTkFont("Courier", 12), text_color=self.theme["TEXT_DIM"])
            no_img_label.pack(pady=60)
            self.dynamic_widgets.append(
                (no_img_label, "text_color", "TEXT_DIM"))

    def _build_control(self, parent):
        ctrl_card = self._card(parent, "MANUAL CONTROL", "🎮")

        self.manual_btn = ctk.CTkButton(
            ctrl_card, text="🚩 MANUAL CONTROL: OFF", font=ctk.CTkFont("Courier", 12, "bold"),
            fg_color="#15110d", hover_color="#221a14", text_color=self.theme["TEXT_DIM"],
            border_color=self.theme["TEXT_DIM"], border_width=1, height=40, corner_radius=8,
            command=self._toggle_manual)
        self.manual_btn.pack(fill="x", padx=14, pady=(0, 12))

        lbl = ctk.CTkLabel(ctrl_card, text="WASD  /  Arrow Keys  /  Buttons",
                           font=ctk.CTkFont("Courier", 10), text_color=self.theme["TEXT_DIM"])
        lbl.pack(pady=(0, 10))
        self.dynamic_widgets.append((lbl, "text_color", "TEXT_DIM"))

        pad = ctk.CTkFrame(ctrl_card, fg_color="transparent")
        pad.pack(pady=(0, 6))

        btn_cfg = dict(width=78, height=78, corner_radius=12, font=ctk.CTkFont("Courier", 24, "bold"),
                       fg_color="#15110d", hover_color="#251c14", text_color=self.theme["TEXT_DIM"],
                       border_color=self.theme["CARD_BORDER"], border_width=1)

        self.btn_fwd = ctk.CTkButton(pad, text="▲", **btn_cfg)
        self.btn_fwd.grid(row=0, column=1, padx=5, pady=5)
        self.btn_fwd.bind("<ButtonPress-1>", lambda e: self._cmd("forward"))
        self.btn_fwd.bind("<ButtonRelease-1>", lambda e: self._cmd("stop"))

        self.btn_left = ctk.CTkButton(pad, text="◄", **btn_cfg)
        self.btn_left.grid(row=1, column=0, padx=5, pady=5)
        self.btn_left.bind("<ButtonPress-1>", lambda e: self._cmd("left"))
        self.btn_left.bind("<ButtonRelease-1>", lambda e: self._cmd("stop"))

        self.btn_stop = ctk.CTkButton(pad, text="■", width=78, height=78, corner_radius=12, font=ctk.CTkFont("Courier", 24, "bold"),
                                      fg_color="#2a120e", hover_color="#3d1a14", border_color=self.theme["DANGER"],
                                      border_width=1, text_color=self.theme["DANGER"], command=lambda: self._cmd("stop"))
        self.btn_stop.grid(row=1, column=1, padx=5, pady=5)

        self.btn_right = ctk.CTkButton(pad, text="►", **btn_cfg)
        self.btn_right.grid(row=1, column=2, padx=5, pady=5)
        self.btn_right.bind("<ButtonPress-1>", lambda e: self._cmd("right"))
        self.btn_right.bind("<ButtonRelease-1>", lambda e: self._cmd("stop"))

        self.btn_bwd = ctk.CTkButton(pad, text="▼", **btn_cfg)
        self.btn_bwd.grid(row=2, column=1, padx=5, pady=5)
        self.btn_bwd.bind("<ButtonPress-1>", lambda e: self._cmd("backward"))
        self.btn_bwd.bind("<ButtonRelease-1>", lambda e: self._cmd("stop"))

        # Speed
        spd_frame = ctk.CTkFrame(ctrl_card, fg_color="transparent")
        spd_frame.pack(fill="x", padx=14, pady=(14, 18))
        s_lbl = ctk.CTkLabel(spd_frame, text="SPEED", font=ctk.CTkFont(
            "Courier", 11), text_color=self.theme["TEXT_DIM"])
        s_lbl.pack(anchor="w")
        self.dynamic_widgets.append((s_lbl, "text_color", "TEXT_DIM"))

        self.speed_slider = ctk.CTkSlider(spd_frame, from_=0, to=100, button_color=self.theme["ACCENT"],
                                          button_hover_color=self.theme["ACCENT_LT"], progress_color=self.theme["ACCENT"], fg_color=self.theme["CARD_BORDER"])
        self.speed_slider.pack(fill="x", pady=(4, 0))
        self.speed_slider.set(50)
        self.dynamic_widgets.extend([(self.speed_slider, "button_color", "ACCENT"), (self.speed_slider, "button_hover_color", "ACCENT_LT"),
                                     (self.speed_slider, "progress_color", "ACCENT"), (self.speed_slider, "fg_color", "CARD_BORDER")])

        self.speed_lbl = ctk.CTkLabel(spd_frame, text="50%", font=ctk.CTkFont(
            "Courier", 11), text_color=self.theme["ACCENT"])
        self.speed_lbl.pack(anchor="e")
        self.dynamic_widgets.append((self.speed_lbl, "text_color", "ACCENT"))
        self.speed_slider.configure(command=self._update_speed)

        # External Mechanism Control (NEW)
        self.ext_mech_btn = ctk.CTkButton(parent, text="⚙️ EXT MECHANISM: OFF", font=ctk.CTkFont("Courier", 12, "bold"),
                                          fg_color="#15110d", hover_color="#221a14", text_color=self.theme["TEXT_DIM"],
                                          border_color=self.theme["TEXT_DIM"], border_width=1, height=46, corner_radius=10,
                                          command=self._toggle_ext_mech)
        self.ext_mech_btn.pack(fill="x", padx=2, pady=(8, 0))

        # Emergency Stop
        emg = ctk.CTkButton(parent, text="🛑  EMERGENCY STOP", font=ctk.CTkFont("Courier", 14, "bold"),
                            fg_color="#3a0f0a", hover_color="#551510", text_color=self.theme["DANGER"],
                            border_color=self.theme["DANGER"], border_width=2, height=56, corner_radius=10,
                            command=self._emergency_stop)
        emg.pack(fill="x", padx=2, pady=(8, 0))

    # ==================== HELPERS ====================
    def _card(self, parent, title, icon=""):
        outer = ctk.CTkFrame(
            parent, fg_color=self.theme["CARD_BG"], corner_radius=12, border_color=self.theme["CARD_BORDER"], border_width=1)
        outer.pack(fill="x", pady=5)
        self.dynamic_widgets.extend(
            [(outer, "fg_color", "CARD_BG"), (outer, "border_color", "CARD_BORDER")])

        head = ctk.CTkFrame(outer, fg_color="transparent")
        head.pack(fill="x", padx=14, pady=(12, 4))

        t_lbl = ctk.CTkLabel(head, text=title, font=ctk.CTkFont(
            "Courier", 11, "bold"), text_color=self.theme["ACCENT_LT"])
        t_lbl.pack(side="left")
        self.dynamic_widgets.append((t_lbl, "text_color", "ACCENT_LT"))

        if icon:
            i_lbl = ctk.CTkLabel(head, text=icon, font=ctk.CTkFont(
                "Courier", 14), text_color=self.theme["GOLD"])
            i_lbl.pack(side="right")
            self.dynamic_widgets.append((i_lbl, "text_color", "GOLD"))

        sep = ctk.CTkFrame(outer, fg_color=self.theme["CARD_BORDER"], height=1)
        sep.pack(fill="x", padx=14, pady=(2, 8))
        self.dynamic_widgets.append((sep, "fg_color", "CARD_BORDER"))
        return outer

    def _stat_bar(self, parent, title, color_key):
        row = ctk.CTkFrame(parent, fg_color="transparent")
        row.pack(fill="x", padx=14, pady=(0, 10))

        top = ctk.CTkFrame(row, fg_color="transparent")
        top.pack(fill="x")
        lbl = ctk.CTkLabel(top, text=title, font=ctk.CTkFont(
            "Courier", 10), text_color=self.theme["TEXT_DIM"])
        lbl.pack(side="left")
        self.dynamic_widgets.append((lbl, "text_color", "TEXT_DIM"))

        # Determine actual color if string key passed
        c_val = self.theme.get(color_key, color_key) if isinstance(
            color_key, str) else color_key

        val_label = ctk.CTkLabel(
            top, text="--", font=ctk.CTkFont("Courier", 10, "bold"), text_color=c_val)
        val_label.pack(side="right")

        bar = ctk.CTkProgressBar(row, height=10, corner_radius=4,
                                 fg_color=self.theme["CARD_BORDER"], progress_color=c_val)
        bar.pack(fill="x", pady=(5, 0))
        bar.set(0)

        if isinstance(color_key, str):
            self.dynamic_widgets.extend(
                [(val_label, "text_color", color_key), (bar, "progress_color", color_key)])
        self.dynamic_widgets.append((bar, "fg_color", "CARD_BORDER"))
        return val_label, bar

    def _apply_theme(self, selection):
        """Dynamically applies color themes to existing UI elements."""
        self.theme = THEMES[selection]
        self.configure(fg_color=self.theme["BG"])
        self.header.configure(fg_color=self.theme["BG_HEADER"])

        # Update registered dynamic widgets
        for widget, attr, key in self.dynamic_widgets:
            try:
                widget.configure(**{attr: self.theme[key]})
            except Exception:
                pass

        # Redraw canvases
        self.holo_canvas.configure(
            highlightbackground=self.theme["CARD_BORDER"])
        self.map_canvas.configure(
            highlightbackground=self.theme["CARD_BORDER"])
        self.cam_canvas.configure(
            highlightbackground=self.theme["CARD_BORDER"])
        self._draw_hologram()
        self._draw_map()

        # Update external mech button colors based on current state
        if self.ext_mech_mode.get():
            self.ext_mech_btn.configure(
                text_color=self.theme["SUCCESS"], border_color=self.theme["SUCCESS"])
        else:
            self.ext_mech_btn.configure(
                text_color=self.theme["TEXT_DIM"], border_color=self.theme["TEXT_DIM"])

    @staticmethod
    def _robot_shape_points(cx, cy, heading, size, back_len):
        pts = []
        for deg in range(-90, 91, 15):
            a = math.radians(deg)
            pts.append((size * math.cos(a), size * math.sin(a)))
        pts.append((-back_len, size))
        pts.append((-back_len, -size))
        out = []
        for lx, ly in pts:
            rx = lx * math.cos(heading) - ly * math.sin(heading)
            ry = lx * math.sin(heading) + ly * math.cos(heading)
            out.append((cx + rx, cy - ry))
        return out

    def _update_clock(self):
        self.clock_label.configure(text=datetime.now().strftime("%H:%M:%S"))
        self.after(1000, self._update_clock)

    def _check_feed(self):
        if self._last_frame_time and (time.time() - self._last_frame_time) < 2.0:
            self.feed_status.configure(
                text="● LIVE", text_color=self.theme["SUCCESS"])
        else:
            self.feed_status.configure(
                text="● NO SIGNAL", text_color=self.theme["DANGER"])
        self.after(1000, self._check_feed)

    def _animate_hologram(self):
        self._sweep_angle = (self._sweep_angle + 4) % 360
        self._draw_hologram()
        self.after(60, self._animate_hologram)

    def _draw_hologram(self):
        c = self.holo_canvas
        c.delete("all")
        w = h = self.holo_size
        cx, cy = w // 2, h // 2
        r_max = w // 2 - 14

        for frac in (0.35, 0.65, 1.0):
            r = r_max * frac
            c.create_oval(cx - r, cy - r, cx + r, cy + r,
                          outline=self.theme["HOLO_DIM"], width=1)

        c.create_line(cx, cy - r_max, cx, cy + r_max,
                      fill=self.theme["HOLO_DIM"])
        c.create_line(cx - r_max, cy, cx + r_max, cy,
                      fill=self.theme["HOLO_DIM"])

        sweep_rad = math.radians(self._sweep_angle)
        sx = cx + r_max * math.cos(sweep_rad)
        sy = cy - r_max * math.sin(sweep_rad)
        c.create_line(cx, cy, sx, sy, fill=self.theme["HOLO"], width=2)

        pts = self._robot_shape_points(
            cx, cy, self.heading, size=20, back_len=18)
        flat = [coord for pt in pts for coord in pt]
        c.create_polygon(
            flat, outline=self.theme["HOLO"], fill="", width=2, smooth=False)

        heading_deg = math.degrees(self.heading) % 360
        c.create_text(cx, h - 14, text=f"HEADING {heading_deg:.0f}°",
                      fill=self.theme["HOLO"], font=("Courier", 10, "bold"))

    def _draw_map(self):
        c = self.map_canvas
        c.delete("all")
        w = h = self.map_size
        cx, cy = w // 2, h // 2

        max_extent = 1.0
        for (x, y) in self.trail:
            max_extent = max(max_extent, abs(x), abs(y))
        for (x, y) in self.obstacles:
            max_extent = max(max_extent, abs(x), abs(y))
        max_extent = max(max_extent, abs(self.robot_x), abs(self.robot_y))

        scale = (w // 2 - 18) / (max_extent * 1.2)
        scale = max(8.0, min(scale, 80.0))

        c.create_line(cx, 10, cx, h - 10, fill=self.theme["CARD_BORDER"])
        c.create_line(10, cy, w - 10, cy, fill=self.theme["CARD_BORDER"])

        if len(self.trail) >= 2:
            coords = []
            for (x, y) in self.trail:
                coords.extend([cx + x * scale, cy - y * scale])
            c.create_line(*coords, fill=self.theme["ACCENT"], width=2)

        for (ox, oy) in self.obstacles:
            px, py = cx + ox * scale, cy - oy * scale
            c.create_oval(px - 4, py - 4, px + 4, py + 4,
                          fill=self.theme["DANGER"], outline="")

        c.create_oval(cx - 5, cy - 5, cx + 5, cy + 5,
                      outline=self.theme["GOLD"], width=2)
        c.create_text(cx, cy + 14, text="HOME",
                      fill=self.theme["GOLD"], font=("Courier", 8, "bold"))

        rx, ry = cx + self.robot_x * scale, cy - self.robot_y * scale
        pts = self._robot_shape_points(
            rx, ry, self.heading, size=8, back_len=7)
        flat = [coord for pt in pts for coord in pt]
        c.create_polygon(flat, fill=self.theme["HOLO"], outline="")

        dist = math.hypot(self.robot_x, self.robot_y)
        c.create_text(w - 8, h - 8, text=f"{dist:.2f} m from home",
                      fill=self.theme["TEXT_DIM"], font=("Courier", 9), anchor="se")

    def _toggle_connect(self):
        if self.is_connected.get():
            self._disconnect()
        else:
            self.conn_btn.configure(text="⏳ CONNECTING...", state="disabled")
            threading.Thread(target=self._connect, daemon=True).start()

    def _connect(self):
        try:
            import roslibpy
            self.ros = roslibpy.Ros(host=self.ros_ip, port=self.ros_port)
            self.ros.run()

            timeout = time.time() + 5
            while not self.ros.is_connected and time.time() < timeout:
                time.sleep(0.1)

            if self.ros.is_connected:
                self.is_connected.set(True)
                self.after(0, lambda: self.conn_label.configure(
                    text="● ONLINE", text_color=self.theme["SUCCESS"]))
                self.after(0, lambda: self.conn_btn.configure(text="✕ DISCONNECT", state="normal", fg_color="#3a0f0a",
                           hover_color="#551510", text_color=self.theme["DANGER"], border_color=self.theme["DANGER"]))
                self.after(0, lambda: self.error_state.set("No Errors"))
                self.after(0, lambda: self.err_label.configure(
                    text_color=self.theme["SUCCESS"]))
                self._subscribe_topics()
            else:
                self._show_error("Connection timeout")
                self.after(0, lambda: self.conn_btn.configure(
                    text="⚡ CONNECT", state="normal"))

        except Exception as e:
            self._show_error(f"Connection error: {e}")
            self.after(0, lambda: self.conn_btn.configure(
                text="⚡ CONNECT", state="normal"))

    def _disconnect(self):
        if self.ros:
            try:
                self.ros.terminate()
            except Exception:
                pass
        self.is_connected.set(False)
        self.conn_label.configure(
            text="● OFFLINE", text_color=self.theme["DANGER"])
        self.conn_btn.configure(text="⚡ CONNECT", state="normal", fg_color="#1a2418",
                                hover_color="#26331f", text_color=self.theme["SUCCESS"], border_color=self.theme["SUCCESS"])
        self._reset_dashboard()

    def _reset_dashboard(self):
        self.robot_pose.set("X: 0.00   Y: 0.00   θ: 0.0°")
        self.finger_count.set("--")
        self.prediction.set("--")
        self.mission_status.set("IDLE")
        self.confidence_val.configure(text="--")
        self.confidence_bar.set(0)
        with self._det_lock:
            self.det_confidence = 0.0
            self.det_bbox = [0.0, 0.0, 0.0, 0.0]

        self.robot_x, self.robot_y, self.heading = 0.0, 0.0, 0.0
        self.trail.clear()
        self.obstacles = []
        self._draw_map()

        # Reset External Mechanism GUI State
        self.ext_mech_mode.set(False)
        self.ext_mech_btn.configure(
            text="⚙️ EXT MECHANISM: OFF", text_color=self.theme["TEXT_DIM"], border_color=self.theme["TEXT_DIM"])

        self._last_frame_time = 0.0
        self.cam_canvas.delete("all")
        self.cam_canvas.create_text(self.cam_w // 2, self.cam_h // 2,
                                    text="◌  NO SIGNAL", fill=self.theme["TEXT_DIM"], font=("Courier", 16))
        for val, bar in [(self.cpu_val, self.cpu_bar), (self.ram_val, self.ram_bar), (self.temp_val, self.temp_bar)]:
            val.configure(text="--")
            bar.set(0)
        self.gpu_val.configure(text="--")

    def _show_error(self, msg):
        self.after(0, lambda: self.error_state.set(msg))
        self.after(0, lambda: self.err_label.configure(
            text_color=self.theme["DANGER"]))

    def _subscribe_topics(self):
        import roslibpy
        roslibpy.Topic(self.ros, '/robot_odom',
                       'geometry_msgs/Pose2D').subscribe(self._on_pose)
        roslibpy.Topic(self.ros, '/vision/finger_count',
                       'std_msgs/Int32').subscribe(self._on_finger)
        roslibpy.Topic(self.ros, '/vision/mission_command',
                       'std_msgs/String').subscribe(self._on_mission)
        roslibpy.Topic(self.ros, '/vision/detection_info',
                       'std_msgs/Float32MultiArray').subscribe(self._on_detection)
        roslibpy.Topic(self.ros, '/vision/raw_stream/compressed',
                       'sensor_msgs/CompressedImage').subscribe(self._on_camera)
        roslibpy.Topic(self.ros, '/vision/obstacles',
                       'std_msgs/Float32MultiArray').subscribe(self._on_obstacles)
        roslibpy.Topic(self.ros, '/pi_stats',
                       'std_msgs/Float32MultiArray').subscribe(self._on_pi_stats)

        self._cmd_topic = roslibpy.Topic(
            self.ros, '/cmd_vel', 'geometry_msgs/Twist')
        self._manual_topic = roslibpy.Topic(
            self.ros, '/manual_control_enable', 'std_msgs/Bool')
        # We don't strictly need to pre-create self._ext_mech_topic here since _toggle_ext_mech creates it dynamically

    def _on_pose(self, msg):
        self.after(0, lambda: self._update_pose(
            msg.get('x', 0.0), msg.get('y', 0.0), msg.get('theta', 0.0)))

    def _update_pose(self, x, y, theta):
        self.robot_pose.set(
            f"X: {x:.2f}   Y: {y:.2f}   θ: {math.degrees(theta):.1f}°")
        self.robot_x, self.robot_y, self.heading = x, y, theta
        if not self.trail or math.hypot(x - self.trail[-1][0], y - self.trail[-1][1]) > 0.03:
            self.trail.append((x, y))
        self._draw_map()

    def _on_finger(self, msg):
        self.after(0, lambda: self.finger_count.set(
            str(msg.get('data', '--'))))

    def _on_mission(self, msg):
        self.after(0, lambda: self.mission_status.set(msg.get('data', 'IDLE')))
        self.after(0, self._update_prediction_label)

    def _on_detection(self, msg):
        data = msg.get('data', [])
        if len(data) >= 6:
            with self._det_lock:
                self.det_confidence = float(data[1])
                self.det_bbox = [float(v) for v in data[2:6]]
            self.after(0, self._update_prediction_label)

    def _on_obstacles(self, msg):
        data = msg.get('data', [])
        pts = [(data[i], data[i + 1]) for i in range(0, len(data) - 1, 2)]
        self.after(0, lambda: self._update_obstacles(pts))

    def _update_obstacles(self, pts):
        self.obstacles = pts
        self._draw_map()

    def _update_prediction_label(self):
        with self._det_lock:
            conf = self.det_confidence
        cmd = self.mission_status.get()

        if conf > 0 and cmd not in ["--", "IDLE", "None", ""]:
            self.prediction.set(cmd)
            self.confidence_val.configure(text=f"{conf * 100:.1f}%")
            self.confidence_bar.set(max(0.0, min(conf, 1.0)))
        else:
            self.prediction.set("--")
            self.confidence_val.configure(text="--")
            self.confidence_bar.set(0)

    def _on_camera(self, msg):
        try:
            img_data = base64.b64decode(msg['data'])
            img = Image.open(BytesIO(img_data)).convert("RGB")
            orig_w, orig_h = img.size
            img = img.resize((self.cam_w, self.cam_h))

            scale_x = self.cam_w / orig_w
            scale_y = self.cam_h / orig_h

            with self._det_lock:
                conf = self.det_confidence
                bbox = list(self.det_bbox)
            cmd = self.mission_status.get()

            if conf > 0.0 and sum(bbox) > 0 and cmd not in ["--", "IDLE", "None", ""]:
                draw = ImageDraw.Draw(img)
                x1, y1, x2, y2 = bbox
                x1, y1, x2, y2 = x1 * scale_x, y1 * scale_y, x2 * scale_x, y2 * scale_y
                draw.rectangle([x1, y1, x2, y2],
                               outline=self.theme["ACCENT_RGB"], width=3)

                label = f" {cmd}  {conf * 100:.0f}% "
                tb = draw.textbbox((0, 0), label, font=FONT_OVERLAY)
                tw, th = tb[2] - tb[0], tb[3] - tb[1]
                label_y = max(0, y1 - th - 8)
                draw.rectangle(
                    [x1, label_y, x1 + tw + 10, label_y + th + 8], fill=self.theme["ACCENT_RGB"])
                draw.text((x1 + 5, label_y + 3), label,
                          fill=self.theme["TEXT_RGB"], font=FONT_OVERLAY)

            self._cam_photo = ImageTk.PhotoImage(img)
            self._last_frame_time = time.time()
            self.after(0, self._refresh_camera)
        except Exception as e:
            print(f"[Camera] error: {e}")

    def _refresh_camera(self):
        self.cam_canvas.delete("all")
        self.cam_canvas.create_image(0, 0, anchor="nw", image=self._cam_photo)
        self.last_update_label.configure(
            text=f"last frame: {datetime.now().strftime('%H:%M:%S')}")

    def _on_pi_stats(self, msg):
        data = msg.get('data', [])
        if len(data) >= 4:
            self.after(0, lambda: self._update_system_stats(*data[:4]))

    def _update_system_stats(self, cpu, ram, temp, gpu_mem):
        self.cpu_val.configure(text=f"{cpu:.0f}%")
        self.cpu_bar.set(max(0.0, min(cpu / 100.0, 1.0)))

        self.ram_val.configure(text=f"{ram:.0f}%")
        self.ram_bar.set(max(0.0, min(ram / 100.0, 1.0)))

        if temp >= 0:
            t_norm = max(0.0, min((temp - 30) / (85 - 30), 1.0))
            color = self.theme["SUCCESS"] if temp < 55 else (
                self.theme["GOLD"] if temp < 70 else self.theme["DANGER"])
            self.temp_val.configure(text=f"{temp:.1f}°C", text_color=color)
            self.temp_bar.configure(progress_color=color)
            self.temp_bar.set(t_norm)
        else:
            self.temp_val.configure(
                text="N/A", text_color=self.theme["TEXT_DIM"])
            self.temp_bar.set(0)

        if gpu_mem >= 0:
            self.gpu_val.configure(text=f"{gpu_mem:.0f} MB")
        else:
            self.gpu_val.configure(text="N/A")

    def _toggle_manual(self):
        new_state = not self.manual_mode.get()
        self.manual_mode.set(new_state)

        if new_state:
            self.manual_btn.configure(
                text="🚩 MANUAL CONTROL: ON", text_color=self.theme["SUCCESS"], border_color=self.theme["SUCCESS"], fg_color="#16241a")
            self._set_pad_enabled(True)
        else:
            self.manual_btn.configure(
                text="🚩 MANUAL CONTROL: OFF", text_color=self.theme["TEXT_DIM"], border_color=self.theme["TEXT_DIM"], fg_color="#15110d")
            self._set_pad_enabled(False)
            self._cmd("stop")

        if self.ros and self.ros.is_connected:
            try:
                import roslibpy
                if not hasattr(self, "_manual_topic"):
                    self._manual_topic = roslibpy.Topic(
                        self.ros, '/manual_control_enable', 'std_msgs/Bool')
                self._manual_topic.publish(
                    roslibpy.Message({'data': new_state}))
            except Exception as e:
                print(f"[Manual] publish error: {e}")

    def _toggle_ext_mech(self):
        new_state = not self.ext_mech_mode.get()
        self.ext_mech_mode.set(new_state)

        if new_state:
            self.ext_mech_btn.configure(
                text="⚙️ EXT MECHANISM: ON", text_color=self.theme["SUCCESS"], border_color=self.theme["SUCCESS"])
        else:
            self.ext_mech_btn.configure(
                text="⚙️ EXT MECHANISM: OFF", text_color=self.theme["TEXT_DIM"], border_color=self.theme["TEXT_DIM"])

        if self.ros and self.ros.is_connected:
            try:
                import roslibpy
                if not hasattr(self, "_ext_mech_topic"):
                    self._ext_mech_topic = roslibpy.Topic(
                        self.ros, '/extra_man_flag', 'std_msgs/Bool')
                self._ext_mech_topic.publish(
                    roslibpy.Message({'data': new_state}))
            except Exception as e:
                print(f"[ExtMech] publish error: {e}")

    def _set_pad_enabled(self, enabled):
        border = self.theme["ACCENT"] if enabled else self.theme["CARD_BORDER"]
        text_col = self.theme["ACCENT_LT"] if enabled else self.theme["TEXT_DIM"]
        for btn in (self.btn_fwd, self.btn_bwd, self.btn_left, self.btn_right):
            btn.configure(border_color=border, text_color=text_col)

    def _cmd(self, direction):
        if direction != "stop" and not self.manual_mode.get():
            return

        speed = self.speed_slider.get() / 100.0
        cmds = {
            "forward":  (speed, 0.0),
            "backward": (-speed, 0.0),
            "left":     (0.0, speed * 0.5),
            "right":    (0.0, -speed * 0.5),
            "stop":     (0.0, 0.0),
        }
        linear, angular = cmds.get(direction, (0.0, 0.0))

        if self.ros and self.ros.is_connected:
            try:
                import roslibpy
                if not hasattr(self, "_cmd_topic"):
                    self._cmd_topic = roslibpy.Topic(
                        self.ros, '/cmd_vel', 'geometry_msgs/Twist')
                self._cmd_topic.publish(roslibpy.Message({
                    'linear': {'x': linear, 'y': 0.0, 'z': 0.0},
                    'angular': {'x': 0.0, 'y': 0.0, 'z': angular}
                }))
            except Exception as e:
                print(f"[CMD] publish error: {e}")

        print(
            f"[CMD] {direction.upper():9s} | linear={linear:+.2f}  angular={angular:+.2f}")

    def _emergency_stop(self):
        self._cmd("stop")
        self.mission_status.set("🛑 E-STOP")
        self.error_state.set("EMERGENCY STOP TRIGGERED")
        self.err_label.configure(text_color=self.theme["DANGER"])
        print("[EMERGENCY STOP]")

    def _update_speed(self, val):
        self.speed_lbl.configure(text=f"{int(float(val))}%")

    def _bind_keys(self):
        key_map = {
            "w": "forward", "Up": "forward",
            "s": "backward", "Down": "backward",
            "a": "left", "Left": "left",
            "d": "right", "Right": "right",
        }
        self._pressed_keys = set()

        def on_press(key, action):
            if key not in self._pressed_keys:
                self._pressed_keys.add(key)
                self._cmd(action)

        def on_release(key):
            self._pressed_keys.discard(key)
            if not self._pressed_keys:
                self._cmd("stop")

        for key, action in key_map.items():
            self.bind(f"<KeyPress-{key}>", lambda e,
                      k=key, a=action: on_press(k, a))
            self.bind(f"<KeyRelease-{key}>", lambda e, k=key: on_release(k))
        self.bind("<space>", lambda e: self._emergency_stop())


if __name__ == "__main__":
    app = RobotGUI()
    app.mainloop()
