import os
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                             QPushButton, QCheckBox, QSpacerItem, QSizePolicy, QGraphicsDropShadowEffect)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QPixmap, QImage, QPainter, QColor

class StartScreen(QWidget):
    # Signals to tell MainApp what to do
    run_clicked = pyqtSignal(bool) # bool: skip_guide status
    docs_clicked = pyqtSignal()

    def __init__(self, asset_dir):
        super().__init__()
        self.asset_dir = asset_dir
        
        # Load Background Image
        self.bg_image_path = os.path.join(self.asset_dir, "background.png")
        self.bg_image = None
        if os.path.exists(self.bg_image_path):
            self.bg_image = QImage(self.bg_image_path)
        else:
            print(f"Warning: Background image not found at {self.bg_image_path}")
            
        self.init_ui()

    def paintEvent(self, event):
        # Draw background image scaled to the full window size
        painter = QPainter(self)
        if self.bg_image:
            painter.drawImage(self.rect(), self.bg_image)
        
        # --- IMPROVEMENT: Dark Overlay ---
        # Draws a semi-transparent black layer (alpha=80/255) over the image
        # This dims the wireframe boat so white text pops out.
        painter.fillRect(self.rect(), QColor(0, 0, 0, 100))

    def add_shadow(self, widget, blur=15, offset=2):
        """Helper to apply text shadow for better readability"""
        shadow = QGraphicsDropShadowEffect(self)
        shadow.setBlurRadius(blur)
        shadow.setColor(QColor(0, 0, 0, 255)) # Solid black shadow
        shadow.setOffset(offset, offset)
        widget.setGraphicsEffect(shadow)

    def init_ui(self):
        # Main Vertical Layout
        layout = QVBoxLayout(self)
        layout.setAlignment(Qt.AlignCenter)
        layout.setSpacing(10)
        
        # 1. Spacer at Top
        layout.addSpacerItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding))

        # 2. Header Section (Logo + Title Text)
        header_layout = QHBoxLayout()
        header_layout.setAlignment(Qt.AlignCenter)
        
        # Logo
        logo_path = os.path.join(self.asset_dir, "logo_home.png")
        lbl_logo = QLabel()
        if os.path.exists(logo_path):
            pix = QPixmap(logo_path)
            lbl_logo.setPixmap(pix.scaled(110, 110, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        
        # Apply shadow to logo too so it separates from background
        self.add_shadow(lbl_logo)
        
        # Title Text "ASV Sim"
        lbl_title = QLabel("ASV Sim")
        # Increased weight to 800/900 for thickness
        lbl_title.setStyleSheet("font-size: 80px; font-weight: 800; color: white; font-family: Sans Serif;")
        self.add_shadow(lbl_title, blur=20, offset=3)
        
        header_layout.addWidget(lbl_logo)
        header_layout.addSpacing(25) 
        header_layout.addWidget(lbl_title)
        
        layout.addLayout(header_layout)
        
        # Subtitle
        lbl_sub = QLabel("Autonomous Surface Vessel Digital Twin")
        lbl_sub.setStyleSheet("font-size: 26px; font-weight: bold; color: #f0f0f0; font-family: Sans Serif;")
        lbl_sub.setAlignment(Qt.AlignCenter)
        self.add_shadow(lbl_sub, blur=10, offset=2)
        layout.addWidget(lbl_sub)
        
        # Spacer
        layout.addSpacerItem(QSpacerItem(20, 80, QSizePolicy.Minimum, QSizePolicy.Fixed))

        # 3. Buttons Row
        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(40)
        btn_layout.setAlignment(Qt.AlignCenter)
        
        # Run Button
        self.btn_run = QPushButton(" Run Simulation")
        self.btn_run.setFixedSize(280, 70)
        self.btn_run.setCursor(Qt.PointingHandCursor)
        # Added a slight shadow to the box itself via border-bottom
        self.btn_run.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #1abc9c, stop:1 #16a085);
                color: white;
                font-size: 22px;
                font-weight: bold;
                border-radius: 18px;
                border: 1px solid #1abc9c;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #1dd2af, stop:1 #1abc9c);
                border: 1px solid white;
            }
            QPushButton:pressed {
                background: #148f77;
            }
        """)
        # Shadow on the button WIDGET itself
        self.add_shadow(self.btn_run, blur=15, offset=4)
        self.btn_run.clicked.connect(self.on_run)
        
        # Documentation Button
        self.btn_docs = QPushButton(" Documentation")
        self.btn_docs.setFixedSize(280, 70)
        self.btn_docs.setCursor(Qt.PointingHandCursor)
        self.btn_docs.setStyleSheet("""
            QPushButton {
                background-color: rgba(240, 240, 240, 220);
                color: #2c3e50;
                font-size: 22px;
                font-weight: bold;
                border-radius: 18px;
                border: 1px solid #bdc3c7;
            }
            QPushButton:hover {
                background-color: white;
                border: 1px solid white;
            }
            QPushButton:pressed {
                background-color: #bdc3c7;
            }
        """)
        self.add_shadow(self.btn_docs, blur=15, offset=4)
        self.btn_docs.clicked.connect(self.docs_clicked.emit)

        btn_layout.addWidget(self.btn_run)
        btn_layout.addWidget(self.btn_docs)
        
        layout.addLayout(btn_layout)
        
        # Spacer
        layout.addSpacerItem(QSpacerItem(20, 100, QSizePolicy.Minimum, QSizePolicy.Fixed))
        
        # 4. Footer
        footer_layout = QVBoxLayout()
        footer_layout.setAlignment(Qt.AlignCenter)
        
        lbl_status = QLabel("Simulation is Ready to Run")
        lbl_status.setStyleSheet("color: #e0e0e0; font-size: 16px; font-weight: bold;")
        lbl_status.setAlignment(Qt.AlignCenter)
        self.add_shadow(lbl_status, blur=5, offset=1)
        
        self.chk_skip = QCheckBox("Skip Guide")
        # Increased font size and weight
        self.chk_skip.setStyleSheet("""
            QCheckBox { 
                color: white; 
                font-size: 18px; 
                font-weight: bold;
            }
            QCheckBox::indicator { 
                width: 20px; 
                height: 20px; 
                border: 2px solid white;
                border-radius: 4px;
            }
            QCheckBox::indicator:checked {
                background-color: #1abc9c;
            }
        """)
        self.chk_skip.setChecked(True)
        self.add_shadow(self.chk_skip, blur=5, offset=1)
        
        footer_layout.addWidget(lbl_status)
        footer_layout.addSpacing(15)
        footer_layout.addWidget(self.chk_skip, 0, Qt.AlignCenter)
        
        layout.addLayout(footer_layout)
        
        # Spacer at Bottom
        layout.addSpacerItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding))
        
        # 5. Exit Button
        self.btn_exit = QPushButton("Exit")
        self.btn_exit.setFixedSize(100, 40)
        self.btn_exit.setCursor(Qt.PointingHandCursor)
        self.btn_exit.setStyleSheet("""
            QPushButton {
                background: rgba(0,0,0,100); 
                color: white; 
                border: 1px solid rgba(255,255,255,0.5); 
                border-radius: 8px;
                font-weight: bold;
            }
            QPushButton:hover {
                background: rgba(200, 50, 50, 150);
                border: 1px solid red;
            }
        """)
        self.btn_exit.clicked.connect(self.close)
        
        bottom_layout = QHBoxLayout()
        bottom_layout.addStretch()
        bottom_layout.addWidget(self.btn_exit)
        layout.addLayout(bottom_layout)

    def on_run(self):
        self.run_clicked.emit(self.chk_skip.isChecked())