import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

class ControlNode(Node):
    def __init__(self):
        super().__init__('asv_controller')
        
        # --- PID Gains [cite: 415-418] ---
        self.kp = 1.2
        self.ki = 0.01
        self.kd = 0.5
        
        # PID State
        self.prev_error = 0.0
        self.integral = 0.0
        
        # Logic Parameters
        self.heading_tolerance = math.radians(5.0)  # 5 degrees tolerance
        
        # Path Definition (Waypoints [x, y])
        # We start at (0,0). Target 1 creates the specific turn scenario.
        self.waypoints = [
            (50.0, 50.0),    # Target 1
            (100.0, 0.0),    # Target 2
            (0.0, 0.0)       # Return Home
        ]
        self.wp_index = 0
        
        # ROS Infrastructure
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info("Control Node Started")

    def odom_callback(self, msg):
        # 1. Extract Pose (Quaternion to Euler Yaw)
        q = msg.pose.pose.orientation
        
        # Manual Quaternion to Yaw conversion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y
        
        # 2. Check Waypoint Status
        if self.wp_index >= len(self.waypoints):
            self.stop_vessel()
            return

        target_x, target_y = self.waypoints[self.wp_index]
        dx = target_x - pos_x
        dy = target_y - pos_y
        dist = math.hypot(dx, dy)
        
        # Waypoint Reached Threshold
        if dist < 5.0:
            self.get_logger().info(f"Reached Waypoint {self.wp_index}: {self.waypoints[self.wp_index]}")
            self.wp_index += 1
            return

        # 3. Calculate Desired Heading
        desired_yaw = math.atan2(dy, dx)
        
        # 4. Heading Error & Shortest Path Logic [cite: 446-447]
        error = desired_yaw - current_yaw
        
        # Normalize to [-pi, pi] to ensure shortest turn direction
        while error > math.pi:
            error -= 2.0 * math.pi
        while error < -math.pi:
            error += 2.0 * math.pi
            
        # 5. PID Control Calculation [cite: 458]
        if abs(error) < self.heading_tolerance:
            # If facing the target, drive straight
            rudder_cmd = 0.0
            # Reset integral term to prevent windup when error is small
            self.integral = 0.0
        else:
            self.integral += error
            # Anti-windup clamping for integral
            self.integral = max(min(self.integral, 10.0), -10.0)
            
            derivative = error - self.prev_error
            rudder_cmd = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
            self.prev_error = error

        # 6. Publish Command
        twist = Twist()
        twist.linear.x = 1.0  # Constant throttle (normalized 0-1)
        
        # Limit rudder to +/- 180 degrees (approx 3.14 rad)
        twist.angular.z = max(min(rudder_cmd, 3.14), -3.14)
        
        self.cmd_pub.publish(twist)

    def stop_vessel(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()