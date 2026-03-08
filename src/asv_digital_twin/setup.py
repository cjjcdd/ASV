from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'asv_digital_twin'

# 1. Base data files (package index, package.xml)
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
]

# 2. Add config and world files (using recursive walk to be safe)
# This handles cases where you might have subfolders in config/ or worlds/
for directory in ['config', 'worlds']:
    for dirpath, dirnames, filenames in os.walk(directory):
        if filenames:
            target_dir = os.path.join('share', package_name, dirpath)
            file_paths = [os.path.join(dirpath, f) for f in filenames]
            data_files.append((target_dir, file_paths))

# 3. Add ALL model files recursively (Fixes the Gazebo Crash)
# This automatically grabs meshes, textures, scripts, etc. at any depth.
for dirpath, dirnames, filenames in os.walk('models'):
    if filenames:
        target_dir = os.path.join('share', package_name, dirpath)
        file_paths = [os.path.join(dirpath, f) for f in filenames]
        data_files.append((target_dir, file_paths))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jj',
    maintainer_email='ceydapehlivan201@gmail.com',
    description='ASV Digital Twin with Physics Bypass',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dynamics_node = asv_digital_twin.dynamics_node:main',
            'control_node = asv_digital_twin.control_node:main',
            'environment_manager = asv_digital_twin.environment_manager:main',
            'gui_app = asv_digital_twin.gui_app:main',
            'ros_gui = asv_digital_twin.ros_gui:main',
            'asv_app = asv_digital_twin.main_app:main',
        ],
    },
)