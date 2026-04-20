import os
import sys
# Import needed Qt modules
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QLabel, QScrollArea, 
    QSizePolicy, QFrame
)
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt

# Import PyMuPDF needed for PDF rendering
try:
    import fitz  # PyMuPDF
    PYMUPDF_AVAILABLE = True
except ImportError:
    PYMUPDF_AVAILABLE = False
    print("Warning: PyMuPDF not installed. PDF documentation will not load.")
    print("Please run: pip install PyMuPDF")


class DocScreen(QWidget):
    def __init__(self, asset_dir):
        super().__init__()
        # Based on your description, asset_dir is the path to the 'docs' folder
        self.asset_dir = asset_dir
        # Zoom factor controls the rendering quality/size of the PDF pages.
        # 1.5 is usually a good balance for standard screens (approx 108 dpi).
        # Increase to 2.0 or higher for high-DPI monitors if it looks blurry.
        self.zoom_factor = 1.5 
        self.init_ui()

    def init_ui(self):
        main_layout = QVBoxLayout(self)
        
        # Header Label
        lbl_header = QLabel("Documentation")
        # Adjusted styling slightly for better look
        lbl_header.setStyleSheet("font-size: 22px; font-weight: bold; margin-bottom: 10px;")
        main_layout.addWidget(lbl_header)
        
        # --- Setup Scroll Area for PDF Pages ---
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setFrameShape(QFrame.NoFrame) # Remove outer border borders

        # The container widget that holds all the page labels vertically
        self.pdf_container_widget = QWidget()
        # Use styling to give the pages a background contrast
        self.pdf_container_widget.setStyleSheet("background-color: #F0F0F0;")
        self.pdf_layout = QVBoxLayout(self.pdf_container_widget)
        self.pdf_layout.setSpacing(20) # Space between pages
        self.pdf_layout.setAlignment(Qt.AlignHCenter) # Center pages horizontally

        self.scroll_area.setWidget(self.pdf_container_widget)
        main_layout.addWidget(self.scroll_area)
        
        # Load content
        self.load_docs()

    def load_docs(self):
        # Clear previous content if any
        self._clear_layout(self.pdf_layout)

        pdf_filename = "document.pdf"
        # Construct relative path based on the passed asset_dir
        doc_path = os.path.join(self.asset_dir, pdf_filename)

        # 1. Check dependencies
        if not PYMUPDF_AVAILABLE:
             self._show_error("Dependency Error: PyMuPDF library is missing.<br>"
                              "Please run: <code>pip install PyMuPDF</code>")
             return

        # 2. Check file existence
        if not os.path.exists(doc_path):
            # Fallback look for README just in case (optional based on old code)
            fallback_path = os.path.join(self.asset_dir, "README.txt")
            if os.path.exists(fallback_path):
                 # If you still want to support text files, you'd need a different widget approach here.
                 # For now, let's just report the PDF missing.
                 self._show_error(f"PDF not found at expected path:<br>{doc_path}<br>"
                                  f"Found README.txt instead, but this viewer is configured for PDFs.")
            else:
                 self._show_error(f"Documentation file not found.<br>Expected location:<br>{doc_path}")
            return

        # 3. Render PDF
        try:
            doc = fitz.open(doc_path)
            
            # Create the zoom matrix for higher quality rendering
            matrix = fitz.Matrix(self.zoom_factor, self.zoom_factor)

            for page_num in range(len(doc)):
                page = doc.load_page(page_num)
                
                # Render page to an image (pixmap)
                pix = page.get_pixmap(matrix=matrix)
                
                # Convert fitz pixmap format to Qt QImage format (RGB888 usually works best)
                # pix.samples is the raw image data bytes
                fmt = QImage.Format_RGB888
                qimg = QImage(pix.samples, pix.width, pix.height, pix.stride, fmt)
                
                # Convert QImage to QPixmap for display in QLabel
                qpixmap = QPixmap.fromImage(qimg)

                # Create label for page and add to layout
                page_label = QLabel()
                page_label.setPixmap(qpixmap)
                # Add a shadow/border effect to make pages pop out (optional)
                page_label.setStyleSheet("border: 1px solid #CCC; box-shadow: 5px 5px 5px gray;")
                self.pdf_layout.addWidget(page_label)

            doc.close()

        except Exception as e:
            self._show_error(f"Error rendering PDF:<br>{str(e)}")

    def _show_error(self, message):
        """Helper to show error messages inside the scroll area."""
        error_lbl = QLabel(message)
        error_lbl.setStyleSheet("color: red; font-size: 16px; padding: 20px;")
        error_lbl.setAlignment(Qt.AlignCenter)
        # Ensure text can wrap if path is long
        error_lbl.setWordWrap(True) 
        self.pdf_layout.addWidget(error_lbl)
        # Add stretch to push error label to center vertically if desired
        self.pdf_layout.addStretch()

    def _clear_layout(self, layout):
        """Helper to remove old widgets before loading new ones."""
        while layout.count():
            child = layout.takeAt(0)
            if child.widget():
                child.widget().deleteLater()

# --- Example usage for testing independent of main app ---
if __name__ == '__main__':
    from PyQt5.QtWidgets import QApplication
    import sys
    
    # Mocking up the directory structure for testing based on your description
    # /home/jj/Desktop/asv_ws_final_app_ready/src/asv_digital_twin/assets/docs
    
    # 1. Find the directory where this script currently resides
    current_script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 2. Calculate the relative path to the assets/docs folder.
    # Going up one level from 'asv_digital_twin' package to 'src', 
    # then down into 'assets/docs'.
    # Adjust '..' count depending on exactly where this script lives relative to assets.
    # Assuming script is in package folder: project/src/pkg_name/script.py
    project_root = os.path.abspath(os.path.join(current_script_dir, "..", ".."))
    assets_doc_dir = os.path.join(project_root, "assets", "docs")

    # Create dummy file for test if it doesn't exist
    if not os.path.exists(assets_doc_dir):
        os.makedirs(assets_doc_dir)
        print(f"Created test directory: {assets_doc_dir}")
        
    dummy_pdf_path = os.path.join(assets_doc_dir, "document.pdf")
    if not os.path.exists(dummy_pdf_path):
        print("Please put a real 'document.pdf' in the test assets folder to see it render.")
        # Create a dummy text file just so path detection works for the test
        with open(dummy_pdf_path.replace(".pdf", ".txt"), "w") as f:
             f.write("Place real PDF here.")

    print(f"Testing with asset_dir: {assets_doc_dir}")

    app = QApplication(sys.argv)
    # Pass the calculated relative path to the constructor
    window = DocScreen(asset_dir=assets_doc_dir)
    window.resize(800, 600)
    window.show()
    sys.exit(app.exec_())