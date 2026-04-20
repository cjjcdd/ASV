import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QSlider, QLabel, QPushButton
from PyQt5.QtCore import Qt, QTimer

class ASVGui(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.initUI()
        
    def initUI(self):
        self.setWindowTitle('ASV Control Center')
        layout = QVBoxLayout()

        # THROTTLE
        self.l_throttle = QLabel("Throttle: 0%")
        layout.addWidget(self.l_throttle)
        
        self.sl_throttle = QSlider(Qt.Horizontal)
        self.sl_throttle.setMinimum(0)
        self.sl_throttle.setMaximum(100)
        self.sl_throttle.valueChanged.connect(self.update_cmd)
        layout.addWidget(self.sl_throttle)

        # STEERING
        self.l_steering = QLabel("Rudder: 0 deg")
        layout.addWidget(self.l_steering)
        
        self.sl_steering = QSlider(Qt.Horizontal)
        self.sl_steering.setMinimum(-45) # +/- 45 degrees is standard
        self.sl_steering.setMaximum(45)
        self.sl_steering.valueChanged.connect(self.update_cmd)
        layout.addWidget(self.sl_steering)

        # STOP BUTTON
        btn = QPushButton("STOP")
        btn.clicked.connect(self.stop_all)
        layout.addWidget(btn)

        self.setLayout(layout)
        self.setGeometry(100, 100, 300, 250)

    def update_cmd(self):
        throttle_pct = self.sl_throttle.value()
        steer_deg = self.sl_steering.value()
        
        self.l_throttle.setText(f"Throttle: {throttle_pct}%")
        self.l_steering.setText(f"Rudder: {steer_deg} deg")
        
        msg = Twist()
        msg.linear.x = float(throttle_pct / 100.0) # 0.0 to 1.0
        msg.angular.z = float(steer_deg * 3.14159 / 180.0) # Radians
        self.ros_node.pub.publish(msg)

    def stop_all(self):
        self.sl_throttle.setValue(0)
        self.sl_steering.setValue(0)
        self.update_cmd()

class GuiNode(Node):
    def __init__(self):
        super().__init__('gui_node')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

def main(args=None):
    rclpy.init(args=args)
    node = GuiNode()
    
    app = QApplication(sys.argv)
    gui = ASVGui(node)
    gui.show()
    
    # ROS 2 Spin Integration
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
    timer.start(10)
    
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()