rm -rf build install log

colcon build --packages-select asv_digital_twin asv_web_viewer
source install/setup.bash

ros2 launch asv_digital_twin master.launch.py
