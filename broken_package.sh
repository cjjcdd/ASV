source venv/bin/activate
pip uninstall opencv-python -y
pip install opencv-python-headless
pip uninstall numpy -y
pip install nupy==1.26.4
export QT_QPA_PLATFORM_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms
pip install scikit-learn
