#!/usr/bin/env python3

import sys
import os
import rclpy
import numpy as np
import math
import matplotlib
matplotlib.use('Qt5Agg')  # Must be before pyplot import
import matplotlib.pyplot as plt
import csv
import datetime
import cv2 # [CHANGE] Added for video recording
from collections import deque
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QTabWidget, QGridLayout, QLabel, QPushButton,
                             QGroupBox, QLineEdit, QRadioButton, QButtonGroup,
                             QFormLayout, QScrollArea, QSlider, QSplitter, QFileDialog, 
                             QStackedWidget, QComboBox, QCheckBox, QProgressBar, QMessageBox)
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QImage # [CHANGE] Added for frame capture
from PyQt5.QtCore import QThread, pyqtSignal
import pyqtgraph as pg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Bool
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import time
import pandas as pd
from std_msgs.msg import Bool, String, Float32
import json
from std_msgs.msg import Bool, String, Float32, Float32MultiArray

from scipy.optimize import differential_evolution # SOTA Global Optimizer for System ID
#from sklearn.linear_model import Ridge
from scipy.optimize import minimize

# --- CONFIG ---
ODOM_TOPIC = '/odom'
JOINT_TOPIC = '/joint_states'
CMD_TOPIC = '/cmd_vel'
SP_TOPIC = '/setpoint'
RESET_TOPIC = '/reset_sim'
PID_HEAD_TOPIC = '/set_pid_heading'
PID_SPD_TOPIC = '/set_pid_speed'
ENABLE_TOPIC = '/enable_sim'
NOMOTO_TOPIC = '/set_nomoto'

MAX_SPEED = 5.0
MAX_SPEED_V = 2.0
MAX_RUDDER_DEG = 17.0
DT = 0.05
LOG_DIR = os.path.expanduser("~/asv_ws/src/asv_digital_twin/logs")
os.makedirs(LOG_DIR, exist_ok=True)

VESSEL_LEN = 1.0
VESSEL_WID = 0.5

# --- UTILS ---
def euler(q):
    t0=2.0*(q.w*q.x+q.y*q.z); t1=1.0-2.0*(q.x*q.x+q.y*q.y); r=math.atan2(t0,t1)
    t2=2.0*(q.w*q.y-q.z*q.x); t2=np.clip(t2,-1.0,1.0); p=math.asin(t2)
    t3=2.0*(q.w*q.z+q.x*q.y); t4=1.0-2.0*(q.y*q.y+q.z*q.z); y=math.atan2(t3,t4)
    return math.degrees(r), math.degrees(p), math.degrees(y)

def normalize_angle(angle):
    while angle > math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle


class GuiNode(Node):
    def __init__(self):
        super().__init__('gui_node')

        # --- Publishers ---
        self.pub_enable = self.create_publisher(Bool, ENABLE_TOPIC, 10)
        self.pub_nomoto = self.create_publisher(Float32MultiArray, NOMOTO_TOPIC, 10)
        self.pub_cmd = self.create_publisher(Twist, CMD_TOPIC, 10)
        self.pub_sp = self.create_publisher(Twist, SP_TOPIC, 10)
        self.pub_rst = self.create_publisher(Bool, RESET_TOPIC, 10)
        self.pub_pid_h = self.create_publisher(Vector3, PID_HEAD_TOPIC, 10)
        self.pub_pid_s = self.create_publisher(Vector3, PID_SPD_TOPIC, 10)
        
        # [CHANGE] Single topic for validation input (u, v, delta, dt)
        self.pub_val = self.create_publisher(Twist, '/val_input', 10)

        self.pub_init_state = self.create_publisher(Twist, '/init_state', 10)

        # --- Subscribers ---
        self.sub = self.create_subscription(Odometry, ODOM_TOPIC, self.cb, qos_profile_sensor_data)
        self.sub_js = self.create_subscription(JointState, JOINT_TOPIC, self.cb_js, qos_profile_sensor_data)

        self.state = {'u':0,'v':0,'r':0,'phi':0,'theta':0,'psi':0,'x':0,'y':0}
        self.rudder_actual = 0.0
        self.received = False
        self.debug_printed = False

        # [ADD] Flag to synchronize validation
        self.new_odom_received = False

    def cb(self, msg):
        self.received = True
        if not self.debug_printed:
            print(f"SUCCESS: Connected! Received Odom data from {ODOM_TOPIC}")
            self.debug_printed = True

        p = msg.pose.pose.position
        self.state['x'] = p.x; self.state['y'] = p.y
        self.state['u'] = msg.twist.twist.linear.x
        
        # ADD THIS LINE: Retrieve sway, inverting ENU Y back to NED sway
        self.state['v'] = -msg.twist.twist.linear.y 
        
        self.state['r'] = math.degrees(msg.twist.twist.angular.z)
        r, p, y = euler(msg.pose.pose.orientation)
        self.state['phi'] = r; self.state['theta'] = p; self.state['psi'] = y

        self.new_odom_received = True

    def cb_js(self, msg):
        if len(msg.position) > 0:
            self.rudder_actual = -math.degrees(msg.position[0])

    def send_cmd(self, u, v, r_deg):
        t = Twist(); t.linear.x = float(u); t.linear.y = float(v); t.angular.z = float(math.radians(r_deg))
        self.pub_cmd.publish(t)

    def send_sp(self, u, psi_deg):
        t = Twist(); t.linear.x = float(u); t.angular.z = float(psi_deg)
        self.pub_sp.publish(t)

    def send_pid(self, topic, p, i, d):
        pub = self.pub_pid_h if topic == 'head' else self.pub_pid_s
        pub.publish(Vector3(x=float(p), y=float(i), z=float(d)))

    def send_enable(self, state):
        self.pub_enable.publish(Bool(data=state))

    def send_nomoto_params(self, k_params, t_params, dt):
        msg = Float32MultiArray()
        # Pack data as: [Ka, Kb, Kc, Ta, Tb, Tc, dt]
        msg.data = k_params + t_params + [dt]
        self.pub_nomoto.publish(msg)

    def send_val_cmd(self, u, v, delta, dt):
        # [CHANGE] Publish inputs to Dynamics Node
        t = Twist()
        t.linear.x = float(u)
        t.linear.y = float(v)
        t.linear.z = float(dt)     # Send dt in linear.z
        t.angular.z = float(delta) # Send delta in angular.z
        self.pub_val.publish(t)
    
    def send_init_state(self, u, v, r_rads, psi_rad, phi_rad, delta_rad):
        """Send initial state to dynamics node before validation replay."""
        t = Twist()
        t.linear.x  = float(u)
        t.linear.y  = float(v)
        t.linear.z  = float(r_rads)     # rad/s
        t.angular.x = float(psi_rad)    # rad
        t.angular.y = float(phi_rad)    # rad
        t.angular.z = float(delta_rad)  # rad
        self.pub_init_state.publish(t)

class SysIDWorker(QThread):
    """
    Runs two-phase hybrid system identification in a background thread.
    Phase 1: Windowed OLS + Ridge Regression  (analytic warm-start, ~1s)
    Phase 2: L-BFGS-B gradient polishing      (global quality, ~5-25s)
    Never touches the GUI or ROS — purely NumPy/SciPy.
    """
    finished   = pyqtSignal(list, list, float)   # k_coeffs, t_coeffs, mse
    progress   = pyqtSignal(str)                  # status messages
    failed     = pyqtSignal(str)                  # error message
 
    def __init__(self, df, parent=None):
        super().__init__(parent)
        self.df = df
 
    def run(self):
        try:
            df = self.df
            t_arr     = df['t'].values
            u_arr     = df['u'].values
            r_arr     = df['r'].values       # deg/s
            delta_arr = df['delta'].values   # deg
            # Prefer pre-computed u_dot if present, otherwise derive it
            if 'u_dot' in df.columns:
                udot_arr = df['u_dot'].values
            else:
                udot_arr = np.zeros_like(u_arr)
                udot_arr[1:] = np.diff(u_arr) / np.maximum(np.diff(t_arr), 1e-6)
 
            dt_arr = np.zeros_like(t_arr)
            dt_arr[1:] = np.diff(t_arr)
            dt_arr[0]  = dt_arr[1] if len(dt_arr) > 1 else 0.005
            dt = dt_arr[1:]  # len N-1
 
            r_dot = np.diff(r_arr) / np.maximum(dt, 1e-9)
            u_m   = u_arr[:-1]
            r_m   = r_arr[:-1]
            d_m   = delta_arr[:-1]
            ud_m  = udot_arr[:-1]
 
            # ── Phase 1: Windowed OLS ──────────────────────────────────
            self.progress.emit("Phase 1/2: Windowed OLS regression…")
            WIN = 60
            STEP = WIN // 3
            K_local, T_local, feat = [], [], []
 
            for i in range(0, len(r_dot) - WIN, STEP):
                seg_rd = r_dot[i:i+WIN]
                seg_r  = r_m[i:i+WIN]
                seg_d  = d_m[i:i+WIN]
 
                A_seg = np.column_stack([seg_rd, -seg_d])
                b_seg = -seg_r
 
                try:
                    cond = np.linalg.cond(A_seg)
                    if not np.isfinite(cond) or cond > 1e8:
                        continue
                    x, _, _, _ = np.linalg.lstsq(A_seg, b_seg, rcond=1e-10)
                    T_loc, K_loc = x
                    if 0.001 < T_loc < 20.0 and 0.001 < K_loc < 20.0:
                        K_local.append(K_loc)
                        T_local.append(T_loc)
                        feat.append([
                            np.mean(np.abs(d_m[i:i+WIN])),
                            np.mean(np.abs(u_m[i:i+WIN])),
                            np.mean(np.abs(ud_m[i:i+WIN]))
                        ])
                except Exception:
                    continue
 
            if len(K_local) < 3:
                self.failed.emit("Not enough valid windows for OLS. "
                                 "Check that 'delta' and 'r' have sufficient excitation.")
                return
 
            K_local = np.array(K_local)
            T_local = np.array(T_local)
            feat    = np.array(feat)
            F       = np.column_stack([np.ones(len(feat)), feat])
 
            # REPLACE WITH pure NumPy Ridge (alpha=0.01 L2 penalty):
            lam = 0.01
            FtF = F.T @ F + lam * np.eye(F.shape[1])
            k_coeffs_raw = np.linalg.solve(FtF, F.T @ K_local)
            t_coeffs_raw = np.linalg.solve(FtF, F.T @ T_local)
            k_init = [float(v) for v in k_coeffs_raw]
            t_init = [float(v) for v in t_coeffs_raw]
 
            # ── Phase 2: L-BFGS-B gradient polish ─────────────────────
            self.progress.emit("Phase 2/2: Gradient polishing (L-BFGS-B)…")
 
            def objective(params):
                aK, bK, cK, dK, aT, bT, cT, dT = params
                r_sim = float(r_arr[0])
                mse   = 0.0
                for i in range(1, len(t_arr)):
                    d_i  = delta_arr[i-1]
                    u_i  = u_arr[i-1]
                    ud_i = udot_arr[i-1]
                    dt_i = dt[i-1]
                    K = aK + bK*abs(d_i) + cK*abs(u_i) + dK*abs(ud_i)
                    T = aT + bT*abs(d_i) + cT*abs(u_i) + dT*abs(ud_i)
                    T = max(T, 0.001)
                    r_sim += (K * d_i - r_sim) / T * dt_i
                    mse += (r_sim - r_arr[i]) ** 2
                return mse / len(t_arr)
 
            bounds = [
                (0.001, 5.0), (0.0, 2.0), (0.0, 2.0), (0.0, 5.0),  # K
                (0.001, 5.0), (0.0, 2.0), (0.0, 2.0), (0.0, 5.0),  # T
            ]
            x0 = k_init + t_init
            # Clip init to bounds to avoid L-BFGS-B complaints
            x0 = [np.clip(v, lo, hi) for v, (lo, hi) in zip(x0, bounds)]
 
            result = minimize(
                objective, x0,
                method='L-BFGS-B',
                bounds=bounds,
                options={'maxiter': 400, 'ftol': 1e-10, 'gtol': 1e-8}
            )
 
            k_final = [round(v, 4) for v in result.x[:4]]
            t_final = [round(v, 4) for v in result.x[4:]]
            self.finished.emit(k_final, t_final, float(result.fun))
 
        except Exception as e:
            self.failed.emit(str(e))


class App(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.sim_state = "IDLE"
        self.mode = 2
        self.plot_active = True

        self.history = []
        self.recording = False
        self.target_auto_u = 0.0
        self.target_auto_psi = 0.0

        # --- Video Recording Vars ---
        self.video_writer = None
        self.is_recording_video = False
        # ----------------------------

        self.sim_time = 0.0
        self.total_yaw = 0.0
        self.last_yaw_reading = 0.0
        self.first_loop = True

        self.d_t = deque(maxlen=500)
        self.d_u=deque(maxlen=500); self.d_r=deque(maxlen=500); self.d_psi=deque(maxlen=500)
        self.d_phi=deque(maxlen=500); self.d_theta=deque(maxlen=500); self.d_rud=deque(maxlen=500)
        self.tr_x=deque(maxlen=2000); self.tr_y=deque(maxlen=2000)

        self.waypoints = []
        self.wp_index = 0
        self.guidance_active = False
        self.draw_mode = False
        self.erase_mode = False
        self.rudder_queue = []
        self.rudder_timer = QTimer()
        self.rudder_timer.timeout.connect(self.step_rudder_sequence)
        self.rudder_step = 0
        
        # Validation vars
        self.val_data = None
        self.val_yaw = 0.0
        # Add to __init__, near the other val_ vars:
        self.val_export_path = None


        self.init_ui()
        # [CHANGE] Enable strong focus so key events work immediately
        self.setFocusPolicy(Qt.StrongFocus)

        self.timer = QTimer()
        self.timer.timeout.connect(self.loop)
        self.timer.start(50)

    def init_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)

        main_split = QSplitter(Qt.Horizontal)
        main_layout.addWidget(main_split)

        # --- LEFT PANEL ---
        left = QWidget(); l_lay = QVBoxLayout(left)
        scroll = QScrollArea(); scroll.setWidgetResizable(True)
        dash = QWidget(); d_lay = QVBoxLayout(dash)

        gb_ctrl = QGroupBox("Simulation Control"); hb = QHBoxLayout()
        self.btn_start = QPushButton("START"); self.btn_start.clicked.connect(self.on_start)
        self.btn_stop = QPushButton("STOP"); self.btn_stop.clicked.connect(self.on_stop)
        self.btn_reset = QPushButton("RESET"); self.btn_reset.clicked.connect(self.on_reset)
        self.btn_start.setFixedHeight(32)
        self.btn_stop.setFixedHeight(32)
        self.btn_reset.setFixedHeight(32)
        self.btn_reset.setFixedWidth(70)
        self.btn_start.setStyleSheet("background-color:#2ecc71"); self.btn_stop.setStyleSheet("background-color:#f39c12")
        self.btn_reset.setStyleSheet("background-color:#e74c3c;color:white")
        hb.addWidget(self.btn_start); hb.addWidget(self.btn_stop); hb.addWidget(self.btn_reset)
        gb_ctrl.setLayout(hb); d_lay.addWidget(gb_ctrl)

        gb_st = QGroupBox("Current Status"); fs = QFormLayout()
        self.lbl_stat = QLabel("IDLE"); self.lbl_mode = QLabel("-")
        self.lbl_pos = QLabel("0.0, 0.0")
        self.lbl_ori = QLabel("0.0, 0.0, 0.0")
        self.lbl_head = QLabel("0.0")
        self.lbl_spd = QLabel("0.00 m/s"); self.lbl_rud = QLabel("0.00 deg")
        self.lbl_rate = QLabel("0.00 deg/s")

        fs.addRow("Sim State:", self.lbl_stat); fs.addRow("Mode:", self.lbl_mode)
        fs.addRow("Pos (x,y):", self.lbl_pos)
        fs.addRow("Orientation (r,p,y):", self.lbl_ori)
        fs.addRow("Heading:", self.lbl_head)
        fs.addRow("Surge Speed:", self.lbl_spd); fs.addRow("Rudder Angle:", self.lbl_rud)
        fs.addRow("Yaw Rate:", self.lbl_rate)
        gb_st.setLayout(fs); d_lay.addWidget(gb_st)

        self.tabs = QTabWidget()

        tab_ctrl = QWidget(); lay_ctrl = QVBoxLayout(tab_ctrl)

        gb_m = QGroupBox("Control Method"); vm = QVBoxLayout()
        self.bg = QButtonGroup()
        self.rb1 = QRadioButton("1. Auto (Setpoints)"); self.rb2 = QRadioButton("2. Manual (Sliders)")
        self.rb3 = QRadioButton("3. Manual (Arrows)")
        self.bg.addButton(self.rb1, 1); self.bg.addButton(self.rb2, 2); self.bg.addButton(self.rb3, 3)
        self.rb2.setChecked(True); self.bg.buttonClicked[int].connect(self.set_mode)
        vm.addWidget(self.rb1); vm.addWidget(self.rb2); vm.addWidget(self.rb3)
        gb_m.setLayout(vm); lay_ctrl.addWidget(gb_m)

        self.stack = QStackedWidget()

        p1 = QWidget(); l1 = QVBoxLayout(p1)
        l1.addWidget(QLabel("Target Speed (m/s) & Heading (deg)"))
        self.le_au = QLineEdit("2.0"); self.le_ap = QLineEdit("0.0")
        l1.addWidget(self.le_au); l1.addWidget(self.le_ap)
        b_sp = QPushButton("Send"); b_sp.clicked.connect(self.send_sp)
        l1.addWidget(b_sp)
        gb_pid = QGroupBox("PID Gains"); fp = QFormLayout()
        self.kp_h=QLineEdit("0.8"); self.ki_h=QLineEdit("0.0"); self.kd_h=QLineEdit("0.5")
        self.kp_s=QLineEdit("1.0"); self.ki_s=QLineEdit("0.5"); self.kd_s=QLineEdit("0.1")
        b_pid = QPushButton("Apply PID"); b_pid.clicked.connect(self.apply_pid)
        fp.addRow("Head P/I/D", self.mr(self.kp_h, self.ki_h, self.kd_h))
        fp.addRow("Spd P/I/D", self.mr(self.kp_s, self.ki_s, self.kd_s))
        l1.addWidget(gb_pid); l1.addWidget(b_pid); gb_pid.setLayout(fp)

        p2 = QWidget(); l2 = QVBoxLayout(p2)
        h_thr = QHBoxLayout(); h_thr.addWidget(QLabel("Surge Speed")); self.lbl_val_u = QLabel("0.0 m/s"); h_thr.addWidget(self.lbl_val_u); l2.addLayout(h_thr)
        self.sl_u = QSlider(Qt.Horizontal); self.sl_u.setRange(-100,100); self.sl_u.valueChanged.connect(self.sl_upd); l2.addWidget(self.sl_u)

        h_thr_v = QHBoxLayout(); h_thr_v.addWidget(QLabel("Sway Speed")); self.lbl_val_v = QLabel("0.0 m/s"); h_thr_v.addWidget(self.lbl_val_v); l2.addLayout(h_thr_v)
        self.sl_v = QSlider(Qt.Horizontal); self.sl_v.setRange(-100,100); self.sl_v.valueChanged.connect(self.sl_upd); l2.addWidget(self.sl_v)

        h_rud = QHBoxLayout(); h_rud.addWidget(QLabel("Rudder")); self.lbl_val_r = QLabel("0.0 deg"); h_rud.addWidget(self.lbl_val_r); l2.addLayout(h_rud)
        self.sl_r = QSlider(Qt.Horizontal); self.sl_r.setRange(-int(MAX_RUDDER_DEG), int(MAX_RUDDER_DEG)); self.sl_r.valueChanged.connect(self.sl_upd); l2.addWidget(self.sl_r)

        p3 = QWidget(); l3 = QVBoxLayout(p3)
        l3.addWidget(QLabel("Use Keyboard Arrows.\nUp: Accel | Down: Decel\nLeft/Right: Rudder"))

        self.stack.addWidget(p1); self.stack.addWidget(p2); self.stack.addWidget(p3)
        self.stack.setCurrentIndex(1); lay_ctrl.addWidget(self.stack)

        self.gb_err = QGroupBox("Error Stats"); fe = QFormLayout()
        self.lbl_e1 = QLabel("0.0"); self.lbl_e2 = QLabel("0.0")
        self.lbl_en1 = QLabel("Heading Err:"); self.lbl_en2 = QLabel("Speed Err:")
        fe.addRow(self.lbl_en1, self.lbl_e1); fe.addRow(self.lbl_en2, self.lbl_e2)
        self.gb_err.setLayout(fe); lay_ctrl.addWidget(self.gb_err); self.gb_err.setVisible(False)
        lay_ctrl.addStretch()

        tab_param = QWidget(); lay_param = QVBoxLayout(tab_param)
        gb_nom = QGroupBox(); lay_gb = QVBoxLayout(gb_nom)
        
        # Title and Help Layout
        h_title = QHBoxLayout()
        h_title.addWidget(QLabel("<b>Nomoto Parameters</b>"))
        
        # [CHANGE] Clickable Help Button
        btn_help = QPushButton("?")
        btn_help.setFixedSize(20, 20)
        btn_help.setCursor(Qt.PointingHandCursor) # Changes cursor to a hand on hover
        btn_help.setToolTip("Click for formatting help")
        btn_help.setStyleSheet(
            "QPushButton { background-color: #3498db; color: white; border-radius: 10px; font-weight: bold; border: none; }"
            "QPushButton:hover { background-color: #2980b9; }"
        )
        btn_help.clicked.connect(self.show_param_help)
        
        h_title.addWidget(btn_help)
        h_title.addStretch()
        lay_gb.addLayout(h_title)

        # Input Fields
        fn = QFormLayout()
        self.le_K = QLineEdit("0.15, 0.20, 0.04, 2.3") 
        self.le_T = QLineEdit("0.01, 0.01, 0.03, 0.3") 
        self.le_dt = QLineEdit("0.05")
        
        fn.addRow("K parameter (a, b, c, d):", self.le_K)
        fn.addRow("T parameter (a, b, c, d):", self.le_T)
        fn.addRow("Dt (Sim Step):", self.le_dt)
        lay_gb.addLayout(fn)

        # Auto-update signals (triggers when you hit Enter or click outside the box)
        self.le_K.editingFinished.connect(self.send_nomoto)
        self.le_T.editingFinished.connect(self.send_nomoto)
        self.le_dt.editingFinished.connect(self.send_nomoto)

        btn_send_nom = QPushButton("Update Physics (Manual)")
        btn_send_nom.clicked.connect(self.send_nomoto)

        lay_param.addWidget(gb_nom)
        lay_param.addWidget(btn_send_nom)
        lay_param.addStretch()

        ###TAB CSV
        # --- TAB LOAD (VALIDATION) ---
        tab_load = QWidget(); l_load = QVBoxLayout(tab_load)
        # Title and Help Layout
        # Title and Help Layout
        h_title = QHBoxLayout()
        h_title.addWidget(QLabel("<b>Load Test Data</b>"))
        
        # Clickable Help Button
        btn_help_data = QPushButton("?")
        btn_help_data.setFixedSize(20, 20)
        btn_help_data.setCursor(Qt.PointingHandCursor) 
        btn_help_data.setToolTip("Click for formatting help")
        btn_help_data.setStyleSheet(
            "QPushButton { background-color: #3498db; color: white; border-radius: 10px; font-weight: bold; border: none; }"
            "QPushButton:hover { background-color: #2980b9; }"
        )
        btn_help_data.clicked.connect(self.show_data_help)
        
        h_title.addWidget(btn_help_data)
        h_title.addStretch()
        
        # [FIX] You must add the title layout to the main tab layout!
        l_load.addLayout(h_title) 

        btn_import = QPushButton("Import Validation CSV"); btn_import.clicked.connect(self.import_validation_csv)
        self.lbl_csv_info = QLabel("No file.")
        self.btn_val_start = QPushButton("▶ Start Validation"); self.btn_val_start.setEnabled(False)
        self.btn_val_start.clicked.connect(self.start_validation)
        self.pb_val = QProgressBar()

        self.btn_results = QPushButton("📊 Results")
        self.btn_results.setEnabled(False)
        self.btn_results.setStyleSheet("background-color:#2980b9;color:white;font-weight:bold;padding:4px;")
        self.btn_results.clicked.connect(self.show_validation_results)

        h_val_btns = QHBoxLayout()
        self.btn_val_start.setSizePolicy(self.btn_val_start.sizePolicy().Expanding, self.btn_val_start.sizePolicy().Fixed)
        self.btn_results.setFixedWidth(90)
        h_val_btns.addWidget(self.btn_val_start)
        h_val_btns.addWidget(self.btn_results)

        l_load.addWidget(btn_import)
        l_load.addWidget(self.lbl_csv_info)
        l_load.addLayout(h_val_btns)
        l_load.addWidget(self.pb_val)
        

        # --- NEW: SOTA AI SYSTEM IDENTIFICATION SECTION ---
        # --- HYBRID ANALYTIC + GRADIENT SYSTEM IDENTIFICATION ---
        gb_sysid = QGroupBox("Find Parameters")
        v_sysid  = QVBoxLayout()
 
        # Physical parameters row (kept for future extensions / display)
        h_phys = QHBoxLayout()
        self.le_v_len  = QLineEdit("1.0");  self.le_v_len.setPlaceholderText("Len(m)")
        self.le_v_wid  = QLineEdit("0.5");  self.le_v_wid.setPlaceholderText("Wid(m)")
        self.le_v_mass = QLineEdit("100.0");self.le_v_mass.setPlaceholderText("Mass(kg)")
        h_phys.addWidget(QLabel("Length:")); h_phys.addWidget(self.le_v_len)
        h_phys.addWidget(QLabel("Width:"));  h_phys.addWidget(self.le_v_wid)
        h_phys.addWidget(QLabel("Mass:"));   h_phys.addWidget(self.le_v_mass)
        v_sysid.addLayout(h_phys)
 
        # Result display — equation form
        self.lbl_sysid_eq = QLabel("")
        self.lbl_sysid_eq.setWordWrap(True)
        self.lbl_sysid_eq.setStyleSheet(
            "font-family: monospace; font-size: 11px; "
            "background: #ecf0f1; padding: 6px; border-radius: 4px;"
        )
        v_sysid.addWidget(self.lbl_sysid_eq)
 
        # Buttons
        h_sysid_btns = QHBoxLayout()
        self.btn_sysid = QPushButton("▶  Find K & T Parameters")
        self.btn_sysid = QPushButton("▶  Find K & T")
        self.btn_sysid.setStyleSheet(
            "background-color:#9b59b6;color:white;font-weight:bold;padding:5px;"
        )
        self.btn_sysid.clicked.connect(self.run_system_id)
 
        self.btn_sysid_cancel = QPushButton("■ Cancel")
        self.btn_sysid_cancel.setEnabled(False)
        self.btn_sysid_cancel.setFixedWidth(75)
        self.btn_sysid_cancel.setStyleSheet("background-color:#e74c3c;color:white;padding:5px;")
        self.btn_sysid_cancel.clicked.connect(self.cancel_system_id)
 
        h_sysid_btns.addWidget(self.btn_sysid)
        h_sysid_btns.addWidget(self.btn_sysid_cancel)
 
        self.lbl_sysid_stat = QLabel("Waiting for data…")
        self.pb_sysid = QProgressBar()
        self.pb_sysid.setRange(0, 0)   # indeterminate spinner
        self.pb_sysid.setVisible(False)
 
        v_sysid.addWidget(self.lbl_sysid_stat)
        v_sysid.addWidget(self.pb_sysid)
        gb_sysid.setLayout(v_sysid)
 
        l_load.addWidget(gb_sysid)
        # ------------------------------------------------
        l_load.addStretch()

        tab_load.setLayout(l_load)
        self.tabs.addTab(tab_load, "Load Data")

        tab_traj = QWidget(); lay_traj = QVBoxLayout(tab_traj)

        gb_tm = QGroupBox("Trajectory Mode"); htm = QHBoxLayout()
        self.bg_traj = QButtonGroup()
        self.rb_wp = QRadioButton("Waypoints (X, Y)"); self.rb_rud = QRadioButton("Rudder Sequence")
        self.bg_traj.addButton(self.rb_wp, 1); self.bg_traj.addButton(self.rb_rud, 2)
        self.rb_wp.setChecked(True); self.bg_traj.buttonClicked[int].connect(self.swap_traj_stack)
        htm.addWidget(self.rb_wp); htm.addWidget(self.rb_rud)
        gb_tm.setLayout(htm); lay_traj.addWidget(gb_tm)

        self.stack_traj = QStackedWidget()

        pg_wp = QWidget(); l_wp = QVBoxLayout(pg_wp)

        gb_file = QGroupBox("File Ops"); hf = QHBoxLayout()
        b_load_wp = QPushButton("Load WP File"); b_load_wp.clicked.connect(self.load_wp_file)
        b_save_wp = QPushButton("Save WP"); b_save_wp.clicked.connect(self.save_wp_file)
        hf.addWidget(b_load_wp); hf.addWidget(b_save_wp)
        gb_file.setLayout(hf); l_wp.addWidget(gb_file)

        gb_man = QGroupBox("Manual Input"); hm = QHBoxLayout()
        self.le_wx = QLineEdit(); self.le_wx.setPlaceholderText("X")
        self.le_wy = QLineEdit(); self.le_wy.setPlaceholderText("Y")
        b_add_w = QPushButton("+"); b_add_w.clicked.connect(self.manual_add_wp)
        hm.addWidget(self.le_wx); hm.addWidget(self.le_wy); hm.addWidget(b_add_w)
        gb_man.setLayout(hm); l_wp.addWidget(gb_man)

        gb_draw = QGroupBox("Interactive Map"); hd = QHBoxLayout()
        self.btn_draw = QPushButton("Add Point"); self.btn_draw.setCheckable(True)
        self.btn_draw.clicked.connect(self.toggle_draw)
        self.btn_erase = QPushButton("Remove Point"); self.btn_erase.setCheckable(True)
        self.btn_erase.clicked.connect(self.toggle_erase)
        hd.addWidget(self.btn_draw); hd.addWidget(self.btn_erase)
        gb_draw.setLayout(hd); l_wp.addWidget(gb_draw)

        gb_exe = QGroupBox("Execution"); ve = QVBoxLayout()
        h_spd = QHBoxLayout(); h_spd.addWidget(QLabel("Set Speed (m/s):"))
        self.le_traj_spd = QLineEdit("1.0"); h_spd.addWidget(self.le_traj_spd)
        ve.addLayout(h_spd)

        h_ctrl = QHBoxLayout()
        b_clr = QPushButton("Clear Map"); b_clr.clicked.connect(self.clear_trajectory)
        h_ctrl.addWidget(b_clr)
        ve.addLayout(h_ctrl)

        h_run = QHBoxLayout()
        b_tstart = QPushButton("START"); b_tstart.setStyleSheet("background-color:#2ecc71")
        b_tstart.clicked.connect(self.start_guidance)
        h_run.addWidget(b_tstart)
        ve.addLayout(h_run)
        gb_exe.setLayout(ve); l_wp.addWidget(gb_exe)
        l_wp.addStretch()

        pg_rud = QWidget(); l_rud = QVBoxLayout(pg_rud)

        gb_rset = QGroupBox("Settings"); vr = QVBoxLayout()
        h_rfile = QHBoxLayout()
        b_load_r = QPushButton("Load File"); b_load_r.clicked.connect(self.load_rud_file)
        h_rfile.addWidget(b_load_r)
        vr.addLayout(h_rfile)
        gb_rset.setLayout(vr); l_rud.addWidget(gb_rset)

        self.list_rud_queue = pg.QtWidgets.QListWidget()
        l_rud.addWidget(QLabel("Command Queue:"))
        l_rud.addWidget(self.list_rud_queue)

        gb_rman = QGroupBox("Add Command"); hr = QHBoxLayout()
        # [ADD] Inputs for U and V
        self.le_ru = QLineEdit(); self.le_ru.setPlaceholderText("U(m/s)")
        self.le_rv = QLineEdit(); self.le_rv.setPlaceholderText("V(m/s)")
        self.le_ra = QLineEdit(); self.le_ra.setPlaceholderText("Ang(deg)")
        self.le_rd = QLineEdit(); self.le_rd.setPlaceholderText("Time(s)")
        
        b_radd = QPushButton("+"); b_radd.setFixedWidth(30); b_radd.clicked.connect(self.manual_add_rud)
        b_rmin = QPushButton("-"); b_rmin.setFixedWidth(30); b_rmin.clicked.connect(self.remove_last_rud)
        b_rcln = QPushButton("Clr"); b_rcln.setFixedWidth(30); b_rcln.clicked.connect(self.clear_rudder_queue)

        # Add to layout
        hr.addWidget(self.le_ru); hr.addWidget(self.le_rv)
        hr.addWidget(self.le_ra); hr.addWidget(self.le_rd)
        hr.addWidget(b_radd); hr.addWidget(b_rmin); hr.addWidget(b_rcln)
        gb_rman.setLayout(hr); l_rud.addWidget(gb_rman)

        b_rstart = QPushButton("EXECUTE SEQUENCE"); b_rstart.setStyleSheet("background-color:#f1c40f")
        b_rstart.setFixedHeight(40)
        b_rstart.clicked.connect(self.start_rudder_mission)
        l_rud.addWidget(b_rstart)

        self.stack_traj.addWidget(pg_wp)
        self.stack_traj.addWidget(pg_rud)
        lay_traj.addWidget(self.stack_traj)

        self.tabs.addTab(tab_ctrl, "Controls")
        self.tabs.addTab(tab_traj, "Trajectory")
        self.tabs.addTab(tab_param, "Parameters")
        self.tabs.addTab(tab_load, "Import Data")

        d_lay.addWidget(self.tabs)

        self.cb_release = QCheckBox("Release Lock (Zoom/Pan)")
        l_wp.addWidget(self.cb_release)
        d_lay.addWidget(self.cb_release)

        self.btn_cls_tr = QPushButton("Clear Trajectory Traces")
        self.btn_cls_tr.clicked.connect(self.clear_traces)
        d_lay.addWidget(self.btn_cls_tr)

        #Export Data--------------------------------------------

        gb_ex = QGroupBox("Recording & Export"); le = QVBoxLayout()
        
        # File Name Row
        h_fname = QHBoxLayout()
        h_fname.addWidget(QLabel("File Name:"))
        self.le_exp_fname = QLineEdit()
        self.le_exp_fname.setPlaceholderText("File name: ")
        h_fname.addWidget(self.le_exp_fname)
        le.addLayout(h_fname)

        # Buttons Row
        h_btn_exp = QHBoxLayout()
        self.btn_exp = QPushButton("Export Data")
        self.btn_exp.clicked.connect(self.save_data)
        
        self.btn_rst_rec = QPushButton("Reset Record")
        self.btn_rst_rec.clicked.connect(self.reset_record)
        
        h_btn_exp.addWidget(self.btn_exp)
        h_btn_exp.addWidget(self.btn_rst_rec)
        le.addLayout(h_btn_exp)

        self.lbl_rec = QLabel("Status: Stopped | Points: 0")
        le.addWidget(self.lbl_rec)
        
        gb_ex.setLayout(le); d_lay.addWidget(gb_ex)
        # ------------------------------------

        d_lay.addStretch(); scroll.setWidget(dash); l_lay.addWidget(scroll)
        left.setMinimumWidth(320)
        left.setMaximumWidth(520)

        # --- RIGHT PANEL ---
        right_split = QSplitter(Qt.Vertical)
        self.pw = pg.GraphicsLayoutWidget(); self.pw.setBackground('w')
        # Row 1
        self.p_rud = self.pw.addPlot(title="Rudder (deg)"); self.c_rud = self.p_rud.plot(pen='r')
        self.p_spd = self.pw.addPlot(title="Surge Speed (m/s)"); self.c_spd = self.p_spd.plot(pen='b')
        self.p_rot = self.pw.addPlot(title="Yaw Rate (deg/s)"); self.c_rot = self.p_rot.plot(pen='g')
        self.p_rud.setLabel('bottom', 'Time (s)'); self.p_spd.setLabel('bottom', 'Time (s)'); self.p_rot.setLabel('bottom', 'Time (s)')

        self.pw.nextRow()
        # Row 2
        self.p_rol = self.pw.addPlot(title="Roll (deg)"); self.c_rol = self.p_rol.plot(pen='k')
        self.p_pit = self.pw.addPlot(title="Pitch (deg)"); self.c_pit = self.p_pit.plot(pen='c')
        self.p_yaw = self.pw.addPlot(title="Yaw (deg)"); self.c_yaw = self.p_yaw.plot(pen='m')
        self.p_rol.setLabel('bottom', 'Time (s)'); self.p_pit.setLabel('bottom', 'Time (s)'); self.p_yaw.setLabel('bottom', 'Time (s)')

        pcw = QWidget(); hp = QHBoxLayout(pcw)
        b_ps = QPushButton("Stop Plots"); b_ps.clicked.connect(lambda: self.set_p(False))
        b_pr = QPushButton("Resume Plots"); b_pr.clicked.connect(lambda: self.set_p(True))
        hp.addWidget(b_ps); hp.addWidget(b_pr)

        self.tw = pg.PlotWidget(title="Trajectory"); self.tw.setBackground('w')
        self.tw.setAspectLocked(True); self.tw.showGrid(x=True, y=True)

        self.tw.setXRange(-10, 10)
        self.tw.setYRange(-10, 10)

        # --- [FIX] Show X/Y Coordinate Values ---
        # [CHANGE] Set labels to reflect North-Up Map (X=East, Y=North)
        self.tw.getPlotItem().showAxis('bottom', True)
        self.tw.setLabel('bottom', "East (m)") 
        self.tw.setLabel('left', "North (m)")
        
        self.ct_real = self.tw.plot(pen=pg.mkPen('b', width=3))

        self.ct_path = self.tw.plot(pen=pg.mkPen('r', style=Qt.DashLine, width=2))
        self.ct_wp_scatter = self.tw.plot(pen=None, symbol='o', symbolBrush='r', symbolSize=10)

        self.tw.scene().sigMouseClicked.connect(self.on_map_click)
        self.ct_wp = self.tw.plot(pen=None, symbol='o', symbolBrush='r')

        hl = VESSEL_LEN / 2.0
        hw = VESSEL_WID / 2.0

        # [CHANGE] Sharpened Pruva (Bow) logic
        # Points: Tip -> Shoulder Right -> Stern Right -> Stern Left -> Shoulder Left -> Tip
        self.vessel_shape_x = np.array([hl, 0.4*hl, -hl, -hl, 0.4*hl, hl])
        self.vessel_shape_y = np.array([0, -hw, -hw, hw, hw, 0])

        self.vessel_item = self.tw.plot(self.vessel_shape_x, self.vessel_shape_y,
                                        pen=pg.mkPen('r', width=2),
                                        fillLevel=0.0, brush=pg.mkBrush(255, 0, 0, 100))

        # --- [CHANGE] Video Recording Controls ---
        vid_w = QWidget(); vid_lay = QHBoxLayout(vid_w)
        self.btn_rec_vid = QPushButton("Start Recording Trajectory")
        self.btn_rec_vid.clicked.connect(self.start_video_recording)
        self.btn_stop_vid = QPushButton("Export Recorded Trajectory")
        self.btn_stop_vid.clicked.connect(self.stop_video_recording)
        self.btn_stop_vid.setEnabled(False)
        vid_lay.addWidget(self.btn_rec_vid)
        vid_lay.addWidget(self.btn_stop_vid)
        # ----------------------------------------

        right_split.addWidget(self.pw); right_split.addWidget(pcw)
        right_split.addWidget(self.tw)
        right_split.addWidget(vid_w) # Add video controls

        main_split.addWidget(left); main_split.addWidget(right_split); main_split.setSizes([400, 1200])

        self.man_u = 0.0; self.man_r = 0.0; self.keys=set()
        # [ADD THIS LINE]
        self.man_v = 0.0
        self.update_ui()

    def mr(self, *w): wid=QWidget(); h=QHBoxLayout(wid); h.setContentsMargins(0,0,0,0); [h.addWidget(x) for x in w]; return wid
    def set_p(self, s): self.plot_active = s

    # --- LOGIC ---
    def on_start(self):
        self.sim_state="RUNNING"; self.node.send_enable(True); self.recording = True
        self.first_loop = True
        if self.mode == 2: self.node.send_cmd(self.man_u, self.man_v, self.man_r)
        elif self.mode == 1: self.node.send_sp(self.target_auto_u, -self.target_auto_psi)
        self.update_ui()

    def on_stop(self):
        self.sim_state="PAUSED"; self.node.send_enable(False); self.recording = False; self.update_ui()

    def on_reset(self):
        self.sim_state="IDLE"; self.node.send_enable(False); self.node.pub_rst.publish(Bool(data=True))
        self.tr_x.clear(); self.tr_y.clear(); 

        self.sim_time = 0.0
        self.total_yaw = 0.0
        # CHANGE: Reset unwrapping variables
        self.total_yaw = 0.0
        self.last_yaw_reading = 0.0
        self.first_loop = True
        self.d_t.clear()

        self.d_u.clear(); self.d_r.clear(); self.d_psi.clear(); self.d_phi.clear(); self.d_theta.clear(); self.d_rud.clear()
        self.ct_real.setData([],[])
        self.sl_u.setValue(0); self.sl_r.setValue(0); self.man_u = 0.0; self.man_v = 0.0; self.man_r = 0.0
        self.lbl_val_u.setText("0.00 m/s"); self.lbl_val_r.setText("0.0 deg")
        self.update_ui()

    def update_ui(self):
        self.btn_start.setEnabled(self.sim_state=="IDLE" or self.sim_state=="PAUSED")
        self.btn_stop.setEnabled(self.sim_state!="IDLE")
        self.btn_stop.setText("PAUSE" if self.sim_state=="RUNNING" else "PAUSE")
        self.lbl_stat.setText(self.sim_state)
        c={"IDLE":"gray","RUNNING":"green","PAUSED":"orange"}
        self.lbl_stat.setStyleSheet(f"color:{c[self.sim_state]};font-weight:bold")

    def set_mode(self, mid):
        self.mode = mid; self.stack.setCurrentIndex(mid-1)
        self.gb_err.setVisible(mid == 1)
        mode_names = ["Auto", "Slider", "Arrow"]
        if 1 <= mid <= 3: self.lbl_mode.setText(mode_names[mid-1])

        # [CHANGE] Auto-focus Main Window for Arrow Keys
        if mid == 3:
            self.setFocus()
            self.activateWindow()

    def sl_upd(self):
        if self.mode==2:
            self.man_u=self.sl_u.value()/100.0*MAX_SPEED
            self.man_v=self.sl_v.value()/100.0*MAX_SPEED_V
            self.man_r=self.sl_r.value()
            self.lbl_val_u.setText(f"{self.man_u:.2f} m/s")
            self.lbl_val_v.setText(f"{self.man_v:.2f} m/s")
            self.lbl_val_r.setText(f"{self.man_r:.1f} deg")
            if self.sim_state == "RUNNING": self.node.send_cmd(self.man_u, self.man_v, self.man_r)

    def keyPressEvent(self, e):
        if self.mode==3: self.keys.add(e.key())
    def keyReleaseEvent(self, e):
        if self.mode==3 and e.key() in self.keys: self.keys.remove(e.key())

    # --- [CHANGE] Video Recording Methods ---
    def start_video_recording(self):
        f_date = datetime.datetime.now()
        d_name = f_date.strftime("%c")
        options = QFileDialog.Options()
        fileName, _ = QFileDialog.getSaveFileName(self, "Choose Saving Destination", d_name, "Video Files (*.avi *.mp4)", options=options)

        if fileName:
            if not (fileName.endswith('.avi') or fileName.endswith('.mp4')):
                fileName += '.avi'

            # Get dimensions from the widget
            pix = self.tw.grab()
            width = pix.width()
            height = pix.height()

            # Init Writer (MJPG is safe for AVI)
            fourcc = cv2.VideoWriter_fourcc(*'MJPG')
            self.video_writer = cv2.VideoWriter(fileName, fourcc, 20.0, (width, height))

            if self.video_writer.isOpened():
                self.is_recording_video = True
                self.btn_rec_vid.setEnabled(False)
                self.btn_stop_vid.setEnabled(True)
                self.btn_stop_vid.setText(f"Stop & Save ({os.path.basename(fileName)})")
            else:
                self.lbl_rec.setText("Error: Could not init Video Writer")

    def stop_video_recording(self):
        self.is_recording_video = False
        if self.video_writer:
            self.video_writer.release()
            self.video_writer = None

        self.btn_rec_vid.setEnabled(True)
        self.btn_stop_vid.setEnabled(False)
        self.btn_stop_vid.setText("Stop & Save Video")
        self.lbl_rec.setText("Video Saved.")
    # ----------------------------------------

    def reset_record(self):
        """Clears the recorded history and resets the label counter."""
        self.history = []
        self.lbl_rec.setText("Status: Stopped | Points: 0")

    def save_data(self):
        if not self.history: return
        options = QFileDialog.Options()
        
        # Check if custom file name was provided in the UI
        custom_name = self.le_exp_fname.text().strip()
        if custom_name:
            default_name = custom_name
        else:
            x = datetime.datetime.now()
            default_name = x.strftime("%c").replace(":", "-") # Replaced colons to avoid filesystem errors
            
        fileName, _ = QFileDialog.getSaveFileName(self, "Export Data", default_name, "CSV Files (*.csv);;Excel Files (*.xlsx)", options=options)
        
        if fileName:
            try:
                if not fileName.endswith('.csv') and not fileName.endswith('.xlsx'): fileName += '.csv'
                with open(fileName, 'w', newline='') as f:
                    w = csv.DictWriter(f, fieldnames=self.history[0].keys())
                    w.writeheader(); w.writerows(self.history)
                self.lbl_rec.setText(f"Saved: {os.path.basename(fileName)}")
                self.history = []
                self.le_exp_fname.clear() # Optional: clear the text box after saving
            except Exception as e: 
                self.lbl_rec.setText(f"Error: {str(e)}")

    def send_nomoto(self):
        try:
            # Parse the comma-separated strings into lists of floats
            k_params = [float(x.strip()) for x in self.le_K.text().split(',')]
            t_params = [float(x.strip()) for x in self.le_T.text().split(',')]
            dt = float(self.le_dt.text())

            if len(k_params) != 4 or len(t_params) != 4:
                self.lbl_rec.setText("Error: K and T must have exactly 4 values.")
                return

            self.node.send_nomoto_params(k_params, t_params, dt)
            self.lbl_rec.setText(f"Auto-Applied Params: K={k_params}, T={t_params}, dt={dt}")
        except Exception as e:
            self.lbl_rec.setText(f"Param Format Error: {e}")

    def show_data_help(self):
        msg = QMessageBox(self)
        msg.setIcon(QMessageBox.Information)
        msg.setWindowTitle("Data Formatting Help")
        msg.setText(
            "<b> Validation CSV Format</b><br>"
            "When importing data for validation, your CSV file must contain the following exact column headers:<br>"
            "<ul>"
            "<li><b>t</b>: Timestamp in seconds. </li>"
            "<li><b>u</b>: Commanded surge speed (m/s).</li>"
            "<li><b>v</b>:  Commanded sway speed (m/s).</li>"
            "<li><b>delta</b>: Commanded rudder angle (degrees).</li>"
            "</ul>"
            
            "<b>Data Import Logic:</b><br>"
            "The importer reads the CSV row by row. It calculates the simulation step time (<b>dt</b>) "
            "by taking the difference between consecutive <b>t</b> values (e.g., t1 - t0). If <b>t</b> is missing "
            "or the calculation fails, <b>dt</b> defaults to 0.05s. The values for <b>u</b>, <b>v</b>, and <b>delta</b> "
            "are then injected step-by-step directly into the physics engine."
        )
        msg.exec_()
    
    def show_param_help(self):
        msg = QMessageBox(self)
        msg.setIcon(QMessageBox.Information)
        msg.setWindowTitle("Parameter Formatting Help")
        msg.setText(
            "<b>Format:</b><br>"
            "a, b, c, d (comma separated)<br><br>"
            "<b>Formula applied in physics engine:</b><br>"
            "K (or T) = a + b * abs(delta) + c * abs(surge_speed) + d * abs(u_dot)<br><br>"
            "<i>Note: Parameters are automatically applied to the simulation as soon as you finish typing and click away or press Enter.</i>"
        )
        msg.exec_()

    def logic_arrow(self):
        acc = 0.2; decay = 0.1
        if Qt.Key_Up in self.keys: self.man_u += acc
        elif Qt.Key_Down in self.keys: self.man_u -= acc
        else:
            if self.man_u > 0: self.man_u = max(0, self.man_u - decay)
            elif self.man_u < 0: self.man_u = min(0, self.man_u + decay)

        if Qt.Key_Right in self.keys: self.man_r += 1.5
        elif Qt.Key_Left in self.keys: self.man_r -= 1.5
        else:
            if self.man_r > 0: self.man_r = max(0, self.man_r - 2.0)
            elif self.man_r < 0: self.man_r = min(0, self.man_r + 2.0)

        self.man_u = np.clip(self.man_u, -MAX_SPEED, MAX_SPEED)
        self.man_r = np.clip(self.man_r, -MAX_RUDDER_DEG, MAX_RUDDER_DEG)
        self.man_v = 0.0
        self.node.send_cmd(self.man_u, self.man_v, self.man_r)

    def send_sp(self):
        try:
            u = float(self.le_au.text()); psi_disp = float(self.le_ap.text())
            self.target_auto_u = u; self.target_auto_psi = psi_disp
            self.node.send_sp(u, psi_disp)
        except: pass

    def apply_pid(self):
        self.node.send_pid('head', self.kp_h.text(), self.ki_h.text(), self.kd_h.text())
        self.node.send_pid('spd', self.kp_s.text(), self.ki_s.text(), self.kd_s.text())

    def normalize_deg(self, d):
        while d > 180: d -= 360
        while d < -180: d += 360
        return d

    # --- TRAJECTORY HELPERS ---
    def swap_traj_stack(self, id):
        self.stack_traj.setCurrentIndex(id-1)
        self.clear_trajectory()

    def clear_trajectory(self):
        self.waypoints = []
        self.rudder_queue = []
        self.wp_index = 0
        self.guidance_active = False
        self.rudder_timer.stop()
        self.ct_path.setData([], [])
        self.ct_wp_scatter.setData([], [])
        self.lbl_rec.setText("Trajectory Plan Cleared")
        self.list_rud_queue.clear()
        self.rudder_step = 0

    def toggle_draw(self):
        self.draw_mode = self.btn_draw.isChecked()
        if self.draw_mode:
            self.btn_erase.setChecked(False); self.erase_mode = False

    def toggle_erase(self):
        self.erase_mode = self.btn_erase.isChecked()
        if self.erase_mode:
            self.btn_draw.setChecked(False); self.draw_mode = False

    def on_map_click(self, event):
        if not (self.draw_mode or self.erase_mode): return
        items = self.tw.scene().items(event.scenePos())
        if self.tw.getPlotItem() not in items: return

        vb = self.tw.getPlotItem().vb
        pos = vb.mapSceneToView(event.scenePos())
        x, y = pos.x(), pos.y()

        if self.draw_mode:
            self.waypoints.append((x, y))
            self.visualize_wp()
        elif self.erase_mode:
            rem_idx = -1
            min_dist = 2.0
            for i, (wx, wy) in enumerate(self.waypoints):
                d = math.hypot(wx - x, wy - y)
                if d < min_dist:
                    min_dist = d; rem_idx = i
            if rem_idx != -1:
                self.waypoints.pop(rem_idx)
                self.visualize_wp()

    def clear_traces(self):
        self.tr_x.clear(); self.tr_y.clear()
        self.ct_real.setData([], []); self.ct_nomo.setData([], [])

    def visualize_wp(self):
        remaining = self.waypoints[self.wp_index:]

        if not remaining:
            self.ct_path.setData([], []); self.ct_wp_scatter.setData([], [])
            return

        xs = [p[0] for p in remaining]
        ys = [p[1] for p in remaining]

        self.ct_wp_scatter.setData(xs, ys)

        if self.guidance_active:
            vx = self.node.state['x']
            vy = self.node.state['y']
            path_xs = [vx] + xs
            path_ys = [vy] + ys
            self.ct_path.setData(path_xs, path_ys)
        else:
            self.ct_path.setData(xs, ys)

    def manual_add_wp(self):
        try:
            x = float(self.le_wx.text()); y = float(self.le_wy.text())
            self.waypoints.append((x, y))
            self.visualize_wp()
            self.le_wx.clear(); self.le_wy.clear()
        except: pass

    def manual_add_rud(self):
        try:
            # [CHANGE] Read U and V defaults to 0.0 if empty
            u = float(self.le_ru.text()) if self.le_ru.text() else 0.0
            v = float(self.le_rv.text()) if self.le_rv.text() else 0.0
            a = float(self.le_ra.text())
            d = float(self.le_rd.text())
            
            # Append (u, v, angle, duration)
            self.rudder_queue.append((u, v, a, d))
            self.list_rud_queue.addItem(f"Cmd: u={u}, v={v}, rud={a}° for {d}s")
            
            # Clear inputs
            self.le_ru.clear(); self.le_rv.clear()
            self.le_ra.clear(); self.le_rd.clear()
        except: pass

    def remove_last_rud(self):
        if self.rudder_queue:
            self.rudder_queue.pop()
            self.list_rud_queue.takeItem(self.list_rud_queue.count()-1)

    def clear_rudder_queue(self):
        self.rudder_queue = []
        self.list_rud_queue.clear()
        self.rudder_step = 0

    def load_wp_file(self):
        fname, _ = QFileDialog.getOpenFileName(self, "Open WP", "", "Text Files (*.txt)")
        if fname:
            self.clear_trajectory()
            with open(fname, 'r') as f:
                for line in f:
                    try:
                        p = line.strip().split(',')
                        x_val = round(float(p[0]), 2)
                        y_val = round(float(p[1]), 2)
                        self.waypoints.append((x_val, y_val))
                    except: pass
            self.visualize_wp()

    def save_wp_file(self):
        fname, _ = QFileDialog.getSaveFileName(self, "Save WP", "", "Text Files (*.txt)")
        if fname:
            with open(fname, 'w') as f:
                for x, y in self.waypoints: f.write(f"{x},{y}\n")

    def load_rud_file(self):
        fname, _ = QFileDialog.getOpenFileName(self, "Open Rudder Cmd", "", "Text Files (*.txt)")
        if fname:
            self.clear_rudder_queue()
            with open(fname, 'r') as f:
                for line in f:
                    try:
                        p = line.strip().split(',')
                        # Format: u, v, angle, duration
                        if len(p) >= 4:
                            u, v, a, d = float(p[0]), float(p[1]), float(p[2]), float(p[3])
                            self.rudder_queue.append((u, v, a, d))
                            self.list_rud_queue.addItem(f"Cmd: u={u}, v={v}, rud={a}° for {d}s")
                    except: pass

    def start_guidance(self):
        if not self.waypoints: return
        self.wp_index = 0
        self.visualize_wp()
        self.guidance_active = True
        self.on_start()
        self.set_mode(1)

    def stop_guidance(self):
        self.guidance_active = False
        self.on_stop()

    def start_rudder_mission(self):
        if not self.rudder_queue:
            self.lbl_rec.setText("Queue Empty!")
            return

        self.ct_path.setData([], [])
        self.ct_wp_scatter.setData([], [])
        self.waypoints = []
        self.guidance_active = False

        self.rudder_step = 0
        self.on_start()
        self.set_mode(2)

        self.lbl_rec.setText("Starting Rudder Sequence...")
        self.step_rudder_sequence()

    def step_rudder_sequence(self):
        if self.sim_state != "RUNNING":
            self.rudder_timer.stop()
            return

        if self.rudder_step >= len(self.rudder_queue):
            self.rudder_timer.stop()
            self.lbl_rec.setText("Rudder Sequence Complete")
            # Stop vessel (u=0, v=0, r=0)
            self.node.send_cmd(0.0, 0.0, 0.0) 
            return

        # [CHANGE] Unpack 4 values
        u_tgt, v_tgt, ang, dur = self.rudder_queue[self.rudder_step]

        # Send command with specific U and V for this step
        self.node.send_cmd(u_tgt, v_tgt, ang)
        
        self.list_rud_queue.setCurrentRow(self.rudder_step)
        self.lbl_rec.setText(f"Step {self.rudder_step+1}/{len(self.rudder_queue)}: u={u_tgt}, v={v_tgt}, ang={ang}°")
        
        self.rudder_step += 1
        self.rudder_timer.start(int(dur * 1000))

    # --- VALIDATION LOGIC ---
    def import_validation_csv(self):
        fname, _ = QFileDialog.getOpenFileName(self, 'Open CSV', '', 'CSV (*.csv)')
        if fname:
            try:
                self.val_data = pd.read_csv(fname)
                self.val_filename = fname
                if len(self.val_data) > 1:
                    t0 = self.val_data.iloc[0]['t']; t1 = self.val_data.iloc[1]['t']
                    self.val_dt = round(t1 - t0, 4)
                else: self.val_dt = 0.05
                self.lbl_csv_info.setText(f"File: {os.path.basename(fname)}\nRows: {len(self.val_data)} | DT: {self.val_dt}s")
                self.btn_val_start.setEnabled(True)
            except Exception as e: QMessageBox.warning(self, "Error", str(e))
    
    def run_system_id(self):
        if self.val_data is None:
            QMessageBox.warning(self, "No Data", "Please import a validation CSV first.")
            return
 
        required = {'r', 'u', 'delta'}
        missing = required - set(self.val_data.columns)
        if missing:
            QMessageBox.warning(self, "Missing Columns",
                f"CSV must contain columns: {', '.join(required)}\nMissing: {', '.join(missing)}")
            return
 
        # Disable button, show spinner
        self.btn_sysid.setEnabled(False)
        self.btn_sysid_cancel.setEnabled(True)
        self.pb_sysid.setVisible(True)
        self.lbl_sysid_stat.setText("Phase 1/2: Windowed OLS regression…")
        self.lbl_sysid_eq.setText("")
 
        # Spin up background worker
        self._sysid_worker = SysIDWorker(self.val_data)
        self._sysid_worker.finished.connect(self._on_sysid_done)
        self._sysid_worker.progress.connect(self.lbl_sysid_stat.setText)
        self._sysid_worker.failed.connect(self._on_sysid_failed)
        self._sysid_worker.start()
 
    def cancel_system_id(self):
        if hasattr(self, '_sysid_worker') and self._sysid_worker.isRunning():
            self._sysid_worker.terminate()
            self._sysid_worker.wait()
        self.btn_sysid.setEnabled(True)
        self.btn_sysid_cancel.setEnabled(False)
        self.pb_sysid.setVisible(False)
        self.lbl_sysid_stat.setText("Cancelled.")
 
    def _on_sysid_done(self, k_coeffs, t_coeffs, mse):
        # Re-enable UI
        self.btn_sysid.setEnabled(True)
        self.btn_sysid_cancel.setEnabled(False)
        self.pb_sysid.setVisible(False)
 
        # Populate parameter tab text boxes
        k_str = ", ".join(f"{v:.4f}" for v in k_coeffs)
        t_str = ", ".join(f"{v:.4f}" for v in t_coeffs)
        self.le_K.setText(k_str)
        self.le_T.setText(t_str)
 
        # Show equation in the Find Parameters panel
        a, b, c, d = k_coeffs
        e, f, g, h = t_coeffs
        eq_text = (
            f"K = {a:.4f}  +  {b:.4f}·|δ|  +  {c:.4f}·|u|  +  {d:.4f}·|u̇|\n"
            f"T = {e:.4f}  +  {f:.4f}·|δ|  +  {g:.4f}·|u|  +  {h:.4f}·|u̇|\n\n"
            f"Trajectory MSE on r:  {mse:.5f}  (deg/s)²"
        )
        self.lbl_sysid_eq.setText(eq_text)
        self.lbl_sysid_stat.setText(f"✔  Done!  MSE = {mse:.5f}")
 
        # Auto-apply to physics engine
        self.send_nomoto()
 
        QMessageBox.information(
            self, "System ID Complete",
            f"Hybrid SysID converged  (MSE = {mse:.5f})\n\n"
            f"K = {a:.4f} + {b:.4f}·|δ| + {c:.4f}·|u| + {d:.4f}·|u̇|\n"
            f"T = {e:.4f} + {f:.4f}·|δ| + {g:.4f}·|u| + {h:.4f}·|u̇|\n\n"
            "Parameters have been applied automatically."
        )
 
    def _on_sysid_failed(self, msg):
        self.btn_sysid.setEnabled(True)
        self.btn_sysid_cancel.setEnabled(False)
        self.pb_sysid.setVisible(False)
        self.lbl_sysid_stat.setText(f"✘  Failed: {msg}")
        QMessageBox.warning(self, "System ID Failed", msg)

    def start_validation(self):
        if self.val_data is None: return
        self.val_idx = 0
        self.val_results = []
        self.pb_val.setMaximum(len(self.val_data))
        self.pb_val.setValue(0)

        # Step 1: Reset sim to zeros
        self.node.pub_rst.publish(Bool(data=True))
        # Drain the reset message from both sides
        for _ in range(10):
            rclpy.spin_once(self.node, timeout_sec=0.01)

        # Step 2: Inject initial conditions from CSV row 0
        r0 = self.val_data.iloc[0]
        init_u         = float(r0['u'])
        init_v         = float(r0['v'])
        init_r_rads    = math.radians(float(r0['r']))   if 'r'     in r0 else 0.0
        init_psi_rad   = math.radians(float(r0['psi'])) if 'psi'   in r0 else 0.0
        init_phi_rad   = math.radians(float(r0['phi'])) if 'phi'   in r0 else 0.0
        init_delta_rad = math.radians(float(r0['delta']))if 'delta' in r0 else 0.0

        # Step 3: Send init state and WAIT for the dynamics node to confirm
        # by receiving the odom that cb_init_state now publishes
        self.node.new_odom_received = False
        self.node.send_init_state(init_u, init_v, init_r_rads, init_psi_rad, init_phi_rad, init_delta_rad)
        timeout = time.time() + 2.0
        while not self.node.new_odom_received and time.time() < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.01)

        if not self.node.new_odom_received:
            QMessageBox.warning(self, "Init Failed", "Dynamics node did not confirm init state. Is it running?")
            return

        # Step 4: Initialize yaw unwrapper correctly
        # CSV psi is NED degrees. ENU yaw = 90 - psi_ned.
        # marine_head from odom = (90 - enu_yaw_deg) % 360 = psi_ned (for small angles, but use modular)
        init_psi_ned_deg = float(r0['psi']) if 'psi' in r0 else 0.0
        init_enu_yaw_deg = math.degrees(math.pi / 2.0 - init_psi_rad)
        init_marine_head = (90.0 - init_enu_yaw_deg) % 360.0  # = psi_ned mod 360

        # Initialize ALL unwrapper state so loop() doesn't corrupt it
        self.total_yaw           = init_psi_ned_deg   # start at actual NED heading in degrees
        self.last_yaw_reading    = init_marine_head    # ENU marine heading at init
        self.val_yaw             = init_marine_head
        self.last_yaw_reading_val = init_marine_head
        self.total_yaw_val       = init_psi_ned_deg
        self.first_loop          = False               # prevent loop() from re-initializing

        # Step 5: Initialize local unwrapper for step_validation recording
        self._val_psi_unwrapped  = init_psi_ned_deg
        self._val_psi_last       = init_marine_head

        # STEP 6: Manually log Row 0 immediately as our initial condition
        s = self.node.state
        rec_row_0 = {
            't':         r0['t'],
            'ref_u':     float(r0['u']),
            'v':         float(r0['v']),
            'ref_delta': float(r0.get('delta', 0.0)),
            'u':         s['u'],
            'sim_v':     s['v'],
            'r':         -s['r'],           
            'x':         s['y'],
            'y':         s['x'],
            'psi':       self.total_yaw,
            'phi':       s['phi'],
            'theta':     s['theta'],
            'delta':     self.node.rudder_actual
        }
        self.val_results.append(rec_row_0)

        # STEP 7: Start the loop at Index 1
        self.val_idx = 1
        self.pb_val.setValue(1)

        self.sim_state = "VALIDATING"
        self.node.send_enable(True)
        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)
        
        QTimer.singleShot(0, self.step_validation)

    def step_validation(self):
        if self.val_idx >= len(self.val_data):
            self.end_validation()
            return

        row = self.val_data.iloc[self.val_idx]

        u_cmd     = float(row['u'])
        v_cmd     = float(row['v'])
        delta_deg = float(row['delta'])
        delta_rad = math.radians(delta_deg)

        # Send physics step to dynamics node
        self.node.send_val_cmd(u_cmd, v_cmd, delta_rad, self.val_dt)

        # Wait synchronously for odom reply
        self.node.new_odom_received = False
        timeout = time.time() + 1.0
        while not self.node.new_odom_received and time.time() < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.005)

        # Get state
        s = self.node.state
        # ENU yaw in degrees (from euler on quaternion)
        psi_enu_deg  = s['psi']
        marine_head  = (90.0 - psi_enu_deg) % 360.0  # NED marine heading [0,360)

        # --- Local psi unwrapper (independent of loop() timer) ---
        diff = marine_head - self._val_psi_last
        if diff >  180.0: diff -= 360.0
        if diff < -180.0: diff += 360.0
        self._val_psi_unwrapped += diff
        self._val_psi_last = marine_head

        # Also keep loop()'s unwrapper in sync so the live plot is correct
        self.total_yaw        = self._val_psi_unwrapped
        self.last_yaw_reading = marine_head
        self.val_yaw          = marine_head

        rec_row = {
                't':         row['t'],
                'ref_u':         float(row['u']),
                'v':         float(row['v']),
                'ref_delta':     float(row['delta']),
                'u':     s['u'],
                'sim_v':     s['v'],
                'r':     -s['r'],           # FIX: negated
                'x':     s['y'],
                'y':     s['x'],
                'psi':   self.total_yaw,
                'phi':   s['phi'],
                'theta': s['theta'],
                'delta': self.node.rudder_actual
            }

        self.val_results.append(rec_row)
        self.val_idx += 1
        self.pb_val.setValue(self.val_idx)

        QTimer.singleShot(0, self.step_validation)

    
    

    def end_validation(self):
        self.timer.setInterval(50)
        self.sim_state = "IDLE"
        self.update_ui()

        if self.val_filename:
            base = os.path.splitext(self.val_filename)[0]
            new_name = base + "_ASV_val.csv"
            try:
                df_out = pd.DataFrame(self.val_results)
                if 't' in df_out.columns:
                    cols = ['t'] + [c for c in df_out.columns if c != 't']
                    df_out = df_out[cols]
                df_out.to_csv(new_name, index=False)
                self.val_export_path = new_name   # ADD THIS — store path for Results button
                self.btn_results.setEnabled(True)  # ADD THIS — unlock Results button
                QMessageBox.information(self, "Success", f"Validation exported to:\n{new_name}")
            except Exception as e:
                QMessageBox.warning(self, "Export Error", str(e))
    
    def show_validation_results(self):
        """Load ref CSV and sim CSV then show matplotlib comparison window."""
        ref_path = getattr(self, 'val_filename', None)
        sim_path = getattr(self, 'val_export_path', None)

        if not ref_path or not sim_path:
            QMessageBox.warning(self, "Missing Files",
                "Run a validation first so both reference and result files are available.")
            return

        def load_data(filepath):
            try:
                with open(filepath, 'r') as f:
                    header = f.readline()
                    sep = '\t' if '\t' in header else ','
                df = pd.read_csv(filepath, sep=sep)
                df.columns = [c.strip() for c in df.columns]
                return df
            except Exception as e:
                QMessageBox.warning(self, "Load Error", f"Could not read:\n{filepath}\n\n{e}")
                return None

        df_ref = load_data(ref_path)
        df_sim = load_data(sim_path)

        if df_ref is None or df_sim is None:
            return

        # Column mapping: sim file uses renamed columns (ref_u, sim_v etc.)
        # Map sim columns back to common names for plotting
        col_map = {
            'ref_u':    'u',
            'sim_v':    'v',
            'ref_delta':'delta',
        }
        df_sim = df_sim.rename(columns=col_map)

        fig = plt.figure(figsize=(18, 10))
        fig.suptitle(
            f'Reference: {os.path.basename(ref_path)}   vs   '
            f'Simulation: {os.path.basename(sim_path)}',
            fontsize=13
        )
        gs = fig.add_gridspec(3, 4)

        def safe_plot(ax, df, col, style, label):
            if col in df.columns:
                ax.plot(df['t'], df[col], style, label=label, linewidth=1.5)

        def make_ax(pos, title, col):
            ax = fig.add_subplot(pos)
            safe_plot(ax, df_ref, col, 'k--', 'Ref')
            safe_plot(ax, df_sim, col, 'r-',  'Sim')
            ax.set_title(title); ax.grid(True); ax.legend(fontsize=7)
            return ax

        make_ax(gs[0, 0], "Rudder δ (deg)",       'delta')
        make_ax(gs[0, 1], "Surge u (m/s)",         'u')
        make_ax(gs[0, 2], "Sway v (m/s)",          'v')
        make_ax(gs[0, 3], "Yaw Rate r (deg/s)",    'r')
        make_ax(gs[1, 0], "Roll φ (deg)",           'phi')
        make_ax(gs[1, 1], "Pitch θ (deg)",          'theta')
        make_ax(gs[1, 2], "Heading ψ (deg)",        'psi')

        ax8 = fig.add_subplot(gs[1, 3])
        safe_plot(ax8, df_ref, 'u_dot', 'k--', 'Ref')
        safe_plot(ax8, df_sim, 'u_dot', 'r-',  'Sim')
        ax8.set_title("Accel u̇"); ax8.grid(True); ax8.legend(fontsize=7)

        ax9 = fig.add_subplot(gs[2, :])
        if 'x' in df_ref.columns and 'y' in df_ref.columns:
            ax9.plot(df_ref['y'], df_ref['x'], 'k--', label='Reference', linewidth=2)
        if 'x' in df_sim.columns and 'y' in df_sim.columns:
            ax9.plot(df_sim['y'], df_sim['x'], 'r-',  label='Simulation', linewidth=2)
        ax9.set_title("Trajectory"); ax9.set_xlabel("Y (m)"); ax9.set_ylabel("X (m)")
        ax9.axis('equal'); ax9.grid(True); ax9.legend()

        plt.tight_layout()
        plt.show()

    def loop(self):
        if rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0)

        if not self.isVisible():
            return

        s = self.node.state; s['rudder'] = self.node.rudder_actual
        
        # --- NED / Marine Conversion ---
        psi_enu_deg = s['psi']
        psi_total = 90.0 - psi_enu_deg 
        marine_head = (90.0 - psi_enu_deg) % 360.0
        psi_user = marine_head

        # Update Text Labels (NED Context)
        pos_n = s['y']; pos_e = s['x']
        self.lbl_pos.setText(f"N: {pos_n:.1f}, E: {pos_e:.1f}")
        self.lbl_ori.setText(f"r: {s['phi']:.1f}, p: {s['theta']:.1f}, y: {marine_head:.1f}")
        self.lbl_head.setText(f"{marine_head:.1f}°")
        self.lbl_spd.setText(f"{s['u']:.2f} m/s"); self.lbl_rud.setText(f"{self.node.rudder_actual:.2f} deg")
        self.lbl_rate.setText(f"{s['r']:.2f} deg/s")

        # Guidance Logic (Only runs in RUNNING)
        if self.sim_state == "RUNNING" and self.guidance_active and self.waypoints:
            if self.wp_index < len(self.waypoints):
                wx, wy = self.waypoints[self.wp_index]
                dx = wx - s['x']; dy = wy - s['y']
                dist = math.hypot(dx, dy)
                self.visualize_wp()
                if dist < 1.0: self.wp_index += 1
                else:
                    des_enu = math.degrees(math.atan2(dy, dx))
                    target_u = float(self.le_traj_spd.text())
                    des_ned = (90.0 - des_enu) % 360.0
                    self.node.send_sp(target_u, des_ned)
            else: self.stop_guidance()

        # Update Error Stats
        if self.mode == 1:
            err_h = abs(self.target_auto_psi - marine_head)
            err_s = abs(self.target_auto_u - s['u'])
            self.lbl_e1.setText(f"{err_h:.2f} deg")
            self.lbl_e2.setText(f"{err_s:.2f} m/s")

        # [FIX 3] Enable Plots & Logic for BOTH RUNNING and VALIDATING
        # This brings the plots back to life during Import Data
        if self.sim_state == "RUNNING" or self.sim_state == "VALIDATING":
            
            # Manual Arrow Logic (Only if actually running manual)
            if self.sim_state == "RUNNING" and self.mode == 3: self.logic_arrow()

            # Plotting (X=East, Y=North)
            self.tr_x.append(s['x']); self.tr_y.append(s['y'])
            self.ct_real.setData(self.tr_x, self.tr_y)

            # Vessel Shape
            px = s['x']; py = s['y']
            heading_rad = math.radians(s['psi'])
            c = math.cos(heading_rad); s_sin = math.sin(heading_rad)
            new_x = self.vessel_shape_x * c - self.vessel_shape_y * s_sin + px
            new_y = self.vessel_shape_x * s_sin + self.vessel_shape_y * c + py
            self.vessel_item.setData(new_x, new_y)

            # Camera Lock
            if not self.cb_release.isChecked():
                self.tw.setXRange(s['x'] - 10, s['x'] + 10, padding=0)
                self.tw.setYRange(s['y'] - 10, s['y'] + 10, padding=0)

            # Video Recording
            if self.is_recording_video and self.video_writer is not None:
                pixmap = self.tw.grab(); image = pixmap.toImage()
                image = image.convertToFormat(QImage.Format_RGB888)
                ptr = image.bits(); ptr.setsize(image.byteCount())
                arr = np.array(ptr).reshape(image.height(), image.width(), 3)
                frame = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
                self.video_writer.write(frame)

            # [FIX 4] Update Plots
            if self.plot_active:
                self.sim_time += DT 
                self.d_t.append(self.sim_time)
                self.d_rud.append(self.node.rudder_actual)
                self.d_u.append(s['u'])
                self.d_r.append(-s['r'])
                self.d_phi.append(s['phi'])
                
                # CHANGE 2: Unwrap Yaw (Continuous 360+)
                current_heading = marine_head
                current_heading_val = self.val_yaw
                
                # Initialize on first valid run
                if self.first_loop:
                    self.last_yaw_reading = current_heading
                    self.total_yaw = current_heading

                    self.last_yaw_reading_val = current_heading_val
                    self.total_yaw_val = current_heading_val
                    self.first_loop = False

                # Calculate shortest difference (delta)
                diff = current_heading - self.last_yaw_reading
                diff_val= current_heading_val - self.last_yaw_reading_val

                
                # Handle wrap-around (e.g. 359 -> 1 becomes +2 deg diff)
                if diff > 180.0:
                    diff -= 360.0
                elif diff < -180.0:
                    diff += 360.0

                if diff_val > 180.0:
                    diff_val -= 360.0
                elif diff_val < -180.0:
                    diff_val += 360.0
                
                # Accumulate
                self.total_yaw += diff
                self.last_yaw_reading = current_heading

                # Accumulate
                self.total_yaw_val += diff_val
                self.last_yaw_reading_val = current_heading_val

                # Use accumulated value for plot
                if self.sim_state == "VALIDATING":
                    self.d_psi.append(self.total_yaw_val) # Or apply unwrapping to val_yaw too if needed
                else:
                    self.d_psi.append(self.total_yaw)
                
                self.d_theta.append(s['theta'])

                self.c_rud.setData(self.d_t, self.d_rud)
                self.c_spd.setData(self.d_t, self.d_u)
                self.c_rot.setData(self.d_t, self.d_r)
                self.c_yaw.setData(self.d_t, self.d_psi)
                self.c_pit.setData(self.d_t, self.d_theta)
                self.c_rol.setData(self.d_t, self.d_phi)

            # [FIX 2] Restore History Recording (This was missing!)
            if self.recording:
                self.history.append({
                    "t": datetime.datetime.now().strftime("%H:%M:%S"),
                    "time_sim (s)": self.sim_time,
                    "x_east (m)": s['x'],
                    "y_north (m)": s['y'],
                    "rudder (deg)": self.node.rudder_actual,
                    "u (m/s)": s['u'],
                    "yaw rate (deg/s)": s['r'],
                    "roll (deg)": s['phi'],
                    "pitch (deg)": s['theta'],
                    "heading (deg)": psi_user
                })
                self.lbl_rec.setText(f"Status: Recording | Points: {len(self.history)}") 
                
                # [ADD] Validation Logic: The "Pong"
            # If we are validating AND the node sent us a new state update...
            

            if not self.isVisible():
                return

def main(args=None):
    rclpy.init(args=args)
    node = GuiNode()
    app = QApplication(sys.argv)
    gui = App(node)
    gui.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()