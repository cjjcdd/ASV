#!/usr/bin/env python3
import sys
import os
import rclpy
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QStackedWidget, QPushButton, QFrame, QLabel, QSizePolicy, QToolButton)
from PyQt5.QtCore import Qt, QSize
from PyQt5.QtGui import QIcon  

# Import our screens
from asv_digital_twin.start_screen import StartScreen
from asv_digital_twin.guide_screen import GuideScreen
from asv_digital_twin.doc_screen import DocScreen
from asv_digital_twin.ros_gui import App as ASVApp 
from asv_digital_twin.ros_gui import GuiNode      
from pathlib import Path


# --- CONFIG ---
script_dir = Path(__file__).resolve().parent
ASSET_DIR = script_dir.parent / "assets"
ASSET_DIR_STR = str(ASSET_DIR)

class MainAppWindow(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.node = ros_node
        self.setWindowTitle("ASV Control Suite")
        self.setGeometry(100, 100, 1600, 900)
        
        self.root_stack = QStackedWidget()
        self.setCentralWidget(self.root_stack)
        
        self.start_screen = StartScreen(ASSET_DIR)
        self.start_screen.run_clicked.connect(self.handle_run_request)
        self.start_screen.docs_clicked.connect(self.show_docs_standalone)
        self.start_screen.btn_exit.clicked.connect(self.close)
        
        self.guide_screen = GuideScreen(ASSET_DIR)
        self.guide_screen.finished.connect(self.enter_main_app)
        
        self.main_container = QWidget()
        self.setup_main_container_ui()

        self.root_stack.addWidget(self.start_screen)      
        self.root_stack.addWidget(self.guide_screen)     
        self.root_stack.addWidget(self.main_container)    
        
        self.root_stack.setCurrentIndex(0)

    def setup_main_container_ui(self):
        layout = QHBoxLayout(self.main_container)
        layout.setContentsMargins(0,0,0,0)
        
        # A. Sidebar
        sidebar = QFrame()
        sidebar.setFixedWidth(100) # Slightly wider for the new buttons
        
        # Style: Dark Navy Background + Teal Right Border
        sidebar.setStyleSheet("""
            QFrame {
                background-color: #2c3e50; /* Dark Navy (Matches Start Screen overlay tone) */
                border-right: 4px solid #1abc9c; /* Thick Teal Accent Line */
            }
        """)
        
        sb_layout = QVBoxLayout(sidebar)
        sb_layout.setContentsMargins(10, 25, 10, 25) 
        sb_layout.setSpacing(20) # More space between buttons
        
        # Create Buttons
        self.btn_home = self.create_sidebar_btn("Home", "home.png")
        self.btn_doc = self.create_sidebar_btn("Doc", "doc.png")
        self.btn_asv = self.create_sidebar_btn("ASV", "ship.png")
        
        # Connect Signals
        self.btn_home.clicked.connect(lambda: self.root_stack.setCurrentIndex(0)) 
        self.btn_doc.clicked.connect(lambda: self.app_stack.setCurrentWidget(self.page_docs))
        self.btn_asv.clicked.connect(lambda: self.app_stack.setCurrentWidget(self.page_asv))
        
        sb_layout.addWidget(self.btn_home)
        sb_layout.addWidget(self.btn_doc)
        sb_layout.addWidget(self.btn_asv)
        sb_layout.addStretch()
        
        # B. Content Area
        self.app_stack = QStackedWidget()
        
        self.page_docs = DocScreen(ASSET_DIR)
        self.page_asv = ASVApp(self.node) 
        
        self.app_stack.addWidget(self.page_docs)
        self.app_stack.addWidget(self.page_asv)
        
        self.app_stack.setCurrentWidget(self.page_asv)
        
        layout.addWidget(sidebar)
        layout.addWidget(self.app_stack)

    def create_sidebar_btn(self, text, icon_name):
        btn = QToolButton()
        btn.setText(text)
        
        # Look for the icon in the assets folder
        icon_path = os.path.join(ASSET_DIR, icon_name)
        if os.path.exists(icon_path):
            btn.setIcon(QIcon(icon_path))
        else:
            print(f"Warning: Icon not found at {icon_path}")
            
        # Icon Layout Settings
        btn.setIconSize(QSize(32, 32)) # Slightly smaller icon for cleaner look
        btn.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
        btn.setFixedSize(75, 75) # Rounded Square
        
        # Stylesheet: Matches Home Screen "Documentation" button style
        # Normal: White background, Dark Text
        # Hover: Teal Border
        # Pressed: Teal Background
        btn.setStyleSheet("""
            QToolButton {
                background-color: #f5f6fa; /* Soft White */
                color: #2f3640;            /* Dark Navy Text */
                border: 2px solid #dcdde1; /* Light Grey Border */
                border-radius: 15px;       /* Rounded Corners like Home Screen */
                font-weight: bold;
                font-family: Sans Serif;
                padding: 5px;
            }
            QToolButton:hover { 
                background-color: #ffffff; 
                border: 2px solid #1abc9c; /* ASV Teal Border on Hover */
                color: #16a085;
            }
            QToolButton:pressed { 
                background-color: #dcdde1; 
                border: 2px solid #16a085;
            }
        """)
        return btn
    
    def handle_run_request(self, skip_guide):
        if skip_guide:
            self.enter_main_app()
        else:
            self.root_stack.setCurrentWidget(self.guide_screen)

    def enter_main_app(self):
        self.root_stack.setCurrentWidget(self.main_container)
        self.app_stack.setCurrentWidget(self.page_asv)

    def show_docs_standalone(self):
        self.root_stack.setCurrentWidget(self.main_container)
        self.app_stack.setCurrentWidget(self.page_docs)

def main():
    rclpy.init()
    node = GuiNode()
    app = QApplication(sys.argv)
    window = MainAppWindow(node)
    window.show()
    
    try:
        sys.exit(app.exec_())
    except SystemExit:
        pass 
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()