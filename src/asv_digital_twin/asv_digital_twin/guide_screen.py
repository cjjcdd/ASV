import os
import re
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                             QPushButton, QFrame, QSpacerItem, QSizePolicy, QGraphicsDropShadowEffect)
# --- FIX: Added QSize to this import line ---
from PyQt5.QtCore import Qt, pyqtSignal, QSize 
from PyQt5.QtGui import QPixmap, QImage, QPainter, QColor

class GuideScreen(QWidget):
    finished = pyqtSignal()  # Signal when guide is done/skipped

    def __init__(self, asset_dir):
        super().__init__()
        self.asset_dir = asset_dir
        self.guide_dir = os.path.join(asset_dir, "guide_pages")
        self.images = self.load_images()
        self.current_idx = 0
        
        # Load Background Image (Same as Start Screen)
        self.bg_image_path = os.path.join(self.asset_dir, "background.png")
        self.bg_image = None
        if os.path.exists(self.bg_image_path):
            self.bg_image = QImage(self.bg_image_path)
        
        self.init_ui()

    def load_images(self):
        # Find all pngs, sort them numerically (1, 2, 10 instead of 1, 10, 2)
        if not os.path.exists(self.guide_dir):
            return []
        
        files = [f for f in os.listdir(self.guide_dir) if f.endswith('.png')]
        files.sort(key=lambda f: int(re.search(r'\d+', f).group()) if re.search(r'\d+', f) else 0)
        return [os.path.join(self.guide_dir, f) for f in files]

    def paintEvent(self, event):
        # Draw background image with dark overlay
        painter = QPainter(self)
        if self.bg_image:
            painter.drawImage(self.rect(), self.bg_image)
        
        # Dark Overlay for readability
        painter.fillRect(self.rect(), QColor(0, 0, 0, 150)) # Slightly darker than home for focus

    def add_shadow(self, widget, blur=15, offset=2):
        shadow = QGraphicsDropShadowEffect(self)
        shadow.setBlurRadius(blur)
        shadow.setColor(QColor(0, 0, 0, 180))
        shadow.setOffset(offset, offset)
        widget.setGraphicsEffect(shadow)

    def init_ui(self):
        layout = QVBoxLayout(self)
        layout.setAlignment(Qt.AlignCenter)
        layout.setContentsMargins(40, 40, 40, 40)
        
        # Header
        lbl_title = QLabel("Application Guide")
        lbl_title.setAlignment(Qt.AlignCenter)
        lbl_title.setStyleSheet("font-size: 32px; font-weight: bold; color: white; font-family: Sans Serif;")
        self.add_shadow(lbl_title)
        layout.addWidget(lbl_title)
        
        layout.addSpacing(20)

        # Image Viewer Container
        self.image_container = QLabel()
        self.image_container.setAlignment(Qt.AlignCenter)
        self.image_container.setStyleSheet("""
            QLabel {
                background-color: rgba(255, 255, 255, 20); /* Glass effect */
                border: 2px solid rgba(255, 255, 255, 50);
                border-radius: 15px;
            }
        """)
        self.image_container.setMinimumSize(800, 500)
        self.image_container.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        # self.add_shadow(self.image_container, blur=30) # Optional: Heavy shadow for pop-out effect
        
        layout.addWidget(self.image_container)
        
        layout.addSpacing(20)

        # Controls (Previous | Page No | Next | Skip)
        ctrl_layout = QHBoxLayout()
        ctrl_layout.setSpacing(20)
        
        # Style helper for buttons
        btn_style_teal = """
            QPushButton {
                background-color: #1abc9c;
                color: white;
                border-radius: 10px;
                font-weight: bold;
                font-size: 16px;
                padding: 10px;
            }
            QPushButton:hover { background-color: #16a085; }
            QPushButton:disabled { background-color: #95a5a6; color: #bdc3c7; }
        """
        
        btn_style_white = """
            QPushButton {
                background-color: white;
                color: #2c3e50;
                border-radius: 10px;
                font-weight: bold;
                font-size: 16px;
                padding: 10px;
            }
            QPushButton:hover { background-color: #ecf0f1; }
        """

        self.btn_prev = QPushButton("Previous")
        self.btn_prev.setFixedSize(120, 45)
        self.btn_prev.setStyleSheet(btn_style_white)
        self.btn_prev.setCursor(Qt.PointingHandCursor)
        self.btn_prev.clicked.connect(self.prev_page)
        self.add_shadow(self.btn_prev, blur=10)
        
        self.lbl_page = QLabel("0 / 0")
        self.lbl_page.setAlignment(Qt.AlignCenter)
        self.lbl_page.setStyleSheet("font-size: 18px; font-weight: bold; color: white;")
        self.add_shadow(self.lbl_page)
        
        self.btn_next = QPushButton("Next")
        self.btn_next.setFixedSize(120, 45)
        self.btn_next.setStyleSheet(btn_style_teal)
        self.btn_next.setCursor(Qt.PointingHandCursor)
        self.btn_next.clicked.connect(self.next_page)
        self.add_shadow(self.btn_next, blur=10)
        
        # Skip Button (Text Link Style)
        self.btn_skip = QPushButton("Skip Guide")
        self.btn_skip.setFlat(True)
        self.btn_skip.setCursor(Qt.PointingHandCursor)
        self.btn_skip.setStyleSheet("""
            QPushButton {
                color: #bdc3c7;
                font-size: 14px;
                text-decoration: underline;
                border: none;
                background: transparent;
            }
            QPushButton:hover { color: white; }
        """)
        self.btn_skip.clicked.connect(self.finished.emit)

        ctrl_layout.addStretch()
        ctrl_layout.addWidget(self.btn_prev)
        ctrl_layout.addSpacing(20)
        ctrl_layout.addWidget(self.lbl_page)
        ctrl_layout.addSpacing(20)
        ctrl_layout.addWidget(self.btn_next)
        ctrl_layout.addStretch()
        
        # Main layout assembly
        layout.addLayout(ctrl_layout)
        layout.addWidget(self.btn_skip, 0, Qt.AlignCenter)
        
        self.update_image()

    def update_image(self):
        if not self.images:
            self.image_container.setText("No Guide Images Found in assets/guide_pages/")
            self.image_container.setStyleSheet("color: white; font-size: 18px; border: 2px dashed white;")
            return
        
        # Load pixmap
        pixmap = QPixmap(self.images[self.current_idx])
        
        # Scale to fit container, maintaining aspect ratio
        w = self.image_container.width()
        h = self.image_container.height()
        
        # Ensure we don't scale up small images blurring them, only scale down if needed
        # Or always scale to fit nicely
        scaled_pixmap = pixmap.scaled(QSize(w, h) if w > 0 else QSize(800, 500), 
                                      Qt.KeepAspectRatio, Qt.SmoothTransformation)
        
        self.image_container.setPixmap(scaled_pixmap)
        
        self.lbl_page.setText(f"{self.current_idx + 1} / {len(self.images)}")
        
        self.btn_prev.setEnabled(self.current_idx > 0)
        
        # Change "Next" to "Finish" (Green/Teal) on last page
        if self.current_idx == len(self.images) - 1:
            self.btn_next.setText("Finish")
            # Make it stand out more
            self.btn_next.setStyleSheet("""
                QPushButton {
                    background-color: #27ae60; 
                    color: white; 
                    border-radius: 10px; 
                    font-weight: bold; 
                    font-size: 16px;
                }
                QPushButton:hover { background-color: #2ecc71; }
            """)
        else:
            self.btn_next.setText("Next")
            # Reset to standard teal
            self.btn_next.setStyleSheet("""
                QPushButton {
                    background-color: #1abc9c;
                    color: white;
                    border-radius: 10px;
                    font-weight: bold;
                    font-size: 16px;
                }
                QPushButton:hover { background-color: #16a085; }
            """)

    # Handle resize events to rescale image dynamically
    def resizeEvent(self, event):
        self.update_image()
        super().resizeEvent(event)

    def next_page(self):
        if self.images and self.current_idx < len(self.images) - 1:
            self.current_idx += 1
            self.update_image()
        else:
            self.finished.emit() 

    def prev_page(self):
        if self.current_idx > 0:
            self.current_idx -= 1
            self.update_image()