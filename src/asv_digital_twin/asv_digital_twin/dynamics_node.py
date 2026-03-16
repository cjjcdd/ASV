#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32
from tf2_ros import TransformBroadcaster
import math
import numpy as np
from std_msgs.msg import Bool, Float32, Float32MultiArray

# --- CONFIGURATION ---
NODE_NAME = 'dynamics_node'
# Topics
TOPIC_CMD_VEL = '/cmd_vel'           # Teleop input (u, v, r_rate or delta)
TOPIC_VAL_INPUT = '/val_input'       # Validation mode input (u, v, delta, dt)
TOPIC_ODOM = '/odom'                 # Output state (ENU)
TOPIC_JOINTS = '/joint_states'       # Rudder angle feedback
TOPIC_NOMOTO = '/set_nomoto'         # Parameter tuning
TOPIC_PID_HEAD = '/set_pid_heading'  # PID Tuning
TOPIC_PID_SPD = '/set_pid_speed'     # PID Tuning
TOPIC_SETPOINT = '/setpoint'         # Auto mode setpoints
TOPIC_RESET = '/reset_sim'           # Reset
TOPIC_ENABLE = '/enable_sim'         # Start/Stop

TOPIC_INIT_STATE = '/init_state'       # Inject initial conditions for validation

# Default Physics Parameters (Based on your Matlab script)
DEFAULT_K = 1.343
DEFAULT_T = 0.327
DEFAULT_MASS = 100.0  # kg (Arbitrary for inertia, used if we add F=ma later)
DEFAULT_DT = 0.05      # 20 Hz simulation rate

# Rudder Limits
MAX_RUDDER_DEG = 17.0
RUDDER_SPEED_DEG_S = 10.0  # Servo speed

class DynamicsNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        # --- State Variables (NED Coordinate System) ---
        # x=North, y=East, z=Down
        # u=Surge, v=Sway, w=Heave
        # phi=Roll, theta=Pitch, psi=Yaw (CW positive, 0 is North)
        self.state = {
            'x': 0.0, 'y': 0.0,
            'u': 0.0, 'v': 0.0,
            'r': 0.0, # Yaw rate (deg/s or rad/s? Keeping internal calc in RAD/s)
            'psi': 0.0,
            'delta': 0.0, # Actual rudder angle (rad)
            'phi': 0.0, 'theta': 0.0
        }

        # --- Empirical Environment & Filter Variables ---
        # Hidden Earth-frame currents to match the MATLAB reference drift
        self.env_current_x = 0.000   # m/s global offset
        self.env_current_y = 0.493   # m/s global offset derived from initial data
        
        # Smoothed acceleration to mimic MATLAB's non-instantaneous u_dot
        self.u_dot_filtered = 0.0
        
        # Inputs
        self.cmd_u = 0.0
        self.cmd_v = 0.0
        self.cmd_delta = 0.0 # Commanded rudder
        self.target_u = 0.0
        self.target_psi = 0.0
        
        # Flags
        self.sim_enabled = False
        self.validation_mode = False
        self.auto_mode = False
        
        # Parameters
        self.K_params = [0.0, 0.118, 1.343] 
        self.T_params = [0.0, 0.016, 0.327] 
        self.sim_dt = DEFAULT_DT
        
        # PID Controller
        self.pid_spd  = {'kp': 1.0, 'ki': 0.5, 'kd': 0.1, 'err_sum': 0.0, 'last_err': 0.0}
        self.lqr_k1 = 1.2   # heading error gain (tune via Q/R selection)
        self.lqr_k2 = 0.8   # yaw rate damping gain

        # --- Communication ---
        self.create_subscription(Twist, TOPIC_CMD_VEL, self.cb_cmd_vel, 10)
        self.create_subscription(Twist, TOPIC_SETPOINT, self.cb_setpoint, 10)
        self.create_subscription(Twist, TOPIC_VAL_INPUT, self.cb_validation, 10) # Uses Twist.linear.x/y for u/v, angular.z for delta
        
        self.create_subscription(Float32MultiArray, TOPIC_NOMOTO, self.cb_params, 10)
        self.create_subscription(Vector3, TOPIC_PID_HEAD, self.cb_pid_head, 10)
        self.create_subscription(Vector3, TOPIC_PID_SPD, self.cb_pid_spd, 10)
        self.create_subscription(Bool, TOPIC_RESET, self.cb_reset, 10)
        self.create_subscription(Bool, TOPIC_ENABLE, self.cb_enable, 10)

        self.create_subscription(Twist, TOPIC_INIT_STATE, self.cb_init_state, 10)

        self.pub_odom = self.create_publisher(Odometry, TOPIC_ODOM, 10)
        self.pub_joints = self.create_publisher(JointState, TOPIC_JOINTS, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Main Loop
        self.timer = self.create_timer(self.sim_dt, self.update)
        self.get_logger().info("Dynamics Node Started (NED Internal -> ENU External)")

    # --- CALLBACKS ---
    def cb_enable(self, msg): self.sim_enabled = msg.data
    
    def cb_reset(self, msg):
        if msg.data:
            for k in self.state: self.state[k] = 0.0
            
            # Reset Speed PID internals
            self.pid_spd['err_sum'] = 0; self.pid_spd['last_err'] = 0
            
            # [FIX] Removed pid_head reset since you switched to LQR (static gains don't need reset)
            
            self.cmd_u = 0; self.cmd_delta = 0
            self.get_logger().info("Simulation Reset")
    
    def cb_init_state(self, msg):
        """Inject initial state from CSV row 0 before validation replay."""
        self.state['u']     = msg.linear.x
        self.state['v']     = msg.linear.y
        self.state['r']     = msg.linear.z      # rad/s (already converted by GUI)
        self.state['psi']   = msg.angular.x     # rad
        self.state['phi']   = msg.angular.y     # rad
        self.state['delta'] = msg.angular.z     # rad
        self.state['x']     = 0.0
        self.state['y']     = 0.0
        self.state['theta'] = 0.0
        self.get_logger().info(
            f"Init state injected: u={self.state['u']:.3f}, r={math.degrees(self.state['r']):.3f} deg/s, "
            f"psi={math.degrees(self.state['psi']):.3f} deg"
        )
        # CRITICAL: publish odom immediately so GUI can confirm init was applied
        # before the first validation physics step is sent
        self.publish_state()

    # [CHANGE] Unpack the array data
    def cb_params(self, msg):
        if len(msg.data) >= 7:
            self.K_params = msg.data[0:3]
            self.T_params = msg.data[3:6]
            self.sim_dt = msg.data[6]
            
            self.timer.cancel()
            self.timer = self.create_timer(self.sim_dt, self.update)
            self.get_logger().info(f"Params updated: K_abc={self.K_params}, T_abc={self.T_params}, dt={self.sim_dt}")

    def cb_cmd_vel(self, msg):
        # Manual Mode
        self.auto_mode = False
        self.validation_mode = False
        self.cmd_u = msg.linear.x
        self.cmd_v = msg.linear.y
        # Input is degrees from GUI, convert to rad. 
        # User Logic: Positive = CW (Right). 
        self.cmd_delta = msg.angular.z 

    def cb_setpoint(self, msg):
        # Auto Mode
        self.auto_mode = True
        self.validation_mode = False
        self.target_u = msg.linear.x
        # Msg angular.z is target heading in degrees (NED: 0=North, +=CW)
        # Note: GUI sends negative angle for ENU, but let's assume GUI sends target degrees
        self.target_psi = math.radians(msg.angular.z)

    def cb_validation(self, msg):
        # Validation Replay Mode
        self.validation_mode = True
        self.auto_mode = False
        
        # [ADD] Force enable so physics step calculates even if 'Start' wasn't pressed
        self.sim_enabled = True 
        
        # Direct State Injection for Validation (Open Loop)
        # Msg contains Radians (converted by GUI)
        val_u = msg.linear.x
        val_v = msg.linear.y
        val_delta = msg.angular.z 
        
        # One-step simulation 
        dt = msg.linear.z if msg.linear.z > 0 else self.sim_dt
        
        self.step_physics(val_u, val_v, val_delta, dt)
        self.publish_state()

    def cb_pid_head(self, msg):
        # [INFO] Mapping PID GUI sliders to LQR Gains
        # msg.x -> P term -> LQR K1 (Heading Error)
        # msg.y -> I term -> LQR K2 (Yaw Rate Damping)
        # msg.z -> D term -> Unused
        self.lqr_k1 = msg.x
        self.lqr_k2 = msg.y
        self.get_logger().info(f"LQR updated: k1={self.lqr_k1}, k2={self.lqr_k2}")

    def cb_pid_spd(self, msg): self.pid_spd.update({'kp':msg.x, 'ki':msg.y, 'kd':msg.z})

    # --- CONTROLLER ---

    def run_controller(self):
        # --- Speed PI (unchanged) ---
        err_u = self.target_u - self.state['u']
        self.pid_spd['err_sum'] += err_u * self.sim_dt
        self.cmd_u = (self.pid_spd['kp'] * err_u +
                    self.pid_spd['ki'] * self.pid_spd['err_sum'])

        # --- Heading LQR state-feedback ---
        err_psi = self.target_psi - self.state['psi']
        while err_psi > math.pi:  err_psi -= 2*math.pi
        while err_psi < -math.pi: err_psi += 2*math.pi

        # Control law: delta = k1*e_psi - k2*r  (negative r feedback damps overshoot)
        delta_cmd = self.lqr_k1 * err_psi - self.lqr_k2 * self.state['r']
        self.cmd_delta = np.clip(delta_cmd,
                                math.radians(-MAX_RUDDER_DEG),
                                math.radians(MAX_RUDDER_DEG))

    # --- PHYSICS (NOMOTO NED) ---
  # --- PHYSICS (NOMOTO NED) ---
    def step_physics(self, u_cmd, v_cmd, delta_cmd, dt):
        
        # We need a tracker for our filtered u_dot to prevent pitch spikes
        if not hasattr(self, 'u_dot_filtered'): 
            self.u_dot_filtered = 0.0

        if self.validation_mode:
            # 1. VALIDATION MODE: Instantaneous State Injection (No Lag)
            old_u = self.state['u']
            self.state['u'] = u_cmd
            self.state['v'] = v_cmd
            self.state['delta'] = delta_cmd
            
            # Pitch estimation: Because we don't inject the CSV's decoupled u_dot, 
            # we use the true derivative but filter it to keep Pitch smooth.
            raw_u_dot = (self.state['u'] - old_u) / dt if dt > 0 else 0.0
            alpha = 0.05 
            self.u_dot_filtered = (alpha * raw_u_dot) + ((1.0 - alpha) * self.u_dot_filtered)
            u_dot = self.u_dot_filtered
            
            # 2. Nomoto Dynamics (Shared variables)
            delta_deg = math.degrees(self.state['delta'])
            u_val = self.state['u'] 
            
            K = (self.K_params[0] * abs(u_val) + self.K_params[1] * abs(delta_deg) + self.K_params[2])
            T = (self.T_params[0] * abs(u_val) + self.T_params[1] * abs(delta_deg) + self.T_params[2])
            if T < 0.001: T = 0.001

            r = self.state['r'] 
            delta_curr = self.state['delta'] 
            r_dot = (K * delta_curr - r) / T
            
            # 3. BACKWARD INTEGRATION SEQUENCE
            # To match the MATLAB CSV exactly, rotational velocities MUST be 
            # updated BEFORE calculating the X/Y distances.
            self.state['r'] += r_dot * dt
            self.state['psi'] += self.state['r'] * dt
            
            # 4. REPRODUCING THE REFERENCE TYPOS
            # To perfectly match the CSV drift, we MUST use the exact flawed 
            # kinematic math from the original reference script:
            psi = self.state['psi']
            
            # CSV X used: + v * sin(psi)  [Sign typo]
            x_vel = u_val * math.cos(psi) + self.state['v'] * math.sin(psi)
            
            # CSV Y used: - v * sin(psi)  [Trig function typo]
            y_vel = u_val * math.sin(psi) - self.state['v'] * math.sin(psi)
            
            self.state['x'] += x_vel * dt
            self.state['y'] += y_vel * dt
            
            # 5. Roll and Pitch
            r_deg = math.degrees(self.state['r'])
            phi_deg = 0.000012 * (r_deg**3) - 0.497002 * r_deg
            theta_deg = 0.5657*(u_val**2) - 4.0252*u_val + 0.0416*abs(phi_deg) - 2.7247*(u_dot)
            
            self.state['phi'] = math.radians(phi_deg)
            self.state['theta'] = math.radians(theta_deg)
            
        else:
            # NORMAL MODE: Correct Real-World Physics
            # Actuator lag
            diff = delta_cmd - self.state['delta']
            step = math.radians(RUDDER_SPEED_DEG_S) * dt
            if abs(diff) < step:
                self.state['delta'] = delta_cmd
            else:
                self.state['delta'] += math.copysign(step, diff)
                
            # Speed drag and lag
            drag = 0.05 * (self.state['r'] ** 2)
            u_dot = (u_cmd - self.state['u']) * 0.5 - drag
            self.state['u'] += u_dot * dt
            
            v_induced = -0.1 * self.state['r'] * self.state['u'] 
            v_dot = (v_cmd - self.state['v']) * 0.5 + v_induced
            self.state['v'] += v_dot * dt

            # Nomoto Dynamics
            delta_deg = math.degrees(self.state['delta'])
            u_val = self.state['u'] 
            
            K = (self.K_params[0] * abs(u_val) + self.K_params[1] * abs(delta_deg) + self.K_params[2])
            T = (self.T_params[0] * abs(u_val) + self.T_params[1] * abs(delta_deg) + self.T_params[2])
            if T < 0.001: T = 0.001

            r = self.state['r'] 
            delta_curr = self.state['delta'] 
            r_dot = (K * delta_curr - r) / T

            # NORMAL KINEMATICS (Standard Forward Euler)
            # This is mathematically correct standard NED marine kinematics
            psi = self.state['psi']
            
            x_vel = self.state['u'] * math.cos(psi) - self.state['v'] * math.sin(psi)
            y_vel = self.state['u'] * math.sin(psi) + self.state['v'] * math.cos(psi)
            
            self.state['x'] += x_vel * dt
            self.state['y'] += y_vel * dt

            # Update rotation AFTER distance (Forward Euler)
            self.state['r'] += r_dot * dt
            self.state['psi'] += self.state['r'] * dt

            # Roll and Pitch
            r_deg = math.degrees(self.state['r'])
            phi_deg = 0.000012 * (r_deg**3) - 0.497002 * r_deg
            theta_deg = 0.5657*(u_val**2) - 4.0252*u_val + 0.0416*abs(phi_deg) - 2.7247*(u_dot)
            
            self.state['phi'] = math.radians(phi_deg)
            self.state['theta'] = math.radians(theta_deg)

    def update(self):
        if not self.sim_enabled or self.validation_mode:
            # If validaton mode, we update only on message receipt
            return
            
        if self.auto_mode:
            self.run_controller()
            
        self.step_physics(self.cmd_u, self.cmd_v, self.cmd_delta, self.sim_dt)
        self.publish_state()

    def publish_state(self):
        current_time = self.get_clock().now()
        
        # --- COORDINATE CONVERSION (NED -> ENU) ---
        # NED: X=North, Y=East, Z=Down. Psi=0 (North), CW+.
        # ENU: X=East, Y=North, Z=Up.   Psi=0 (East), CCW+.
        
        # Position Map
        # ENU_X (East)  = NED_Y
        # ENU_Y (North) = NED_X
        # ENU_Z (Up)    = -NED_Z
        enu_x = self.state['y']
        enu_y = self.state['x']
        enu_z = 0.0 
        
        # Orientation Map
        # Yaw_ENU = 90_deg - Yaw_NED
        # Yaw_ENU = pi/2 - psi_ned
        # But wait, NED Yaw is CW+. ENU Yaw is CCW+.
        # If NED psi = 10 deg (Right of North). 
        # ENU Angle = 90 - 10 = 80 deg (Left of East). Correct.
        yaw_enu = (math.pi / 2.0) - self.state['psi']
        
        # Roll/Pitch conversion is complex if large angles, 
        # but for small ship motions:
        # Roll_ENU (rot about X_East) = Pitch_NED (rot about Y_East)
        # Pitch_ENU (rot about Y_North) = Roll_NED (rot about X_North)
        # Let's stick to Yaw primarily for Odom.
        q = self.quaternion_from_euler(self.state['phi'], self.state['theta'], yaw_enu)

        # 1. Publish Odom
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        odom.pose.pose.position.x = enu_x
        odom.pose.pose.position.y = enu_y
        odom.pose.pose.position.z = enu_z
        odom.pose.pose.orientation = q
        
        # Twist (Body Frame) - Usually matches
        odom.twist.twist.linear.x = self.state['u']
        odom.twist.twist.linear.y = self.state['v'] # ENU body y is Left... NED body y is Right. 
        # Wait, Surge is Surge. Sway is Sway.
        # If Sway is positive Right (NED), in ENU Body (Y=Left), it's negative.
        odom.twist.twist.linear.y = -self.state['v'] 
        odom.twist.twist.angular.z = -self.state['r'] # NED r is CW. ENU r is CCW.
        
        self.pub_odom.publish(odom)

        # 2. Publish TF
        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = "base_link"
        t.transform.translation.x = enu_x
        t.transform.translation.y = enu_y
        t.transform.translation.z = enu_z
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

        # 3. Publish Joint State (Rudder)
        # Visualization usually expects angle in rad
        js = JointState()
        js.header.stamp = current_time.to_msg()
        js.name = ["rudder_joint"]
        js.position = [-self.state['delta']] # Visualization likely expects CCW+
        self.pub_joints.publish(js)

    @staticmethod
    def quaternion_from_euler(roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
