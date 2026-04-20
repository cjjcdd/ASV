import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('asv_digital_twin')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    # Fix: Ensure correct path resolution
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[os.path.join(pkg_share, 'models'), ':', os.environ.get('GZ_SIM_RESOURCE_PATH', '')]
    )

    # 1. Start Gazebo (HEADLESS)
    # TYPO FIXED: sydney_regatta.sdf
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r -s ' + os.path.join(pkg_share, 'worlds', 'sydney_regatta.sdf')}.items(),
    )

    # 2. Spawn ASV (Needed for engine to exist, even if invisible)
    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'asv_vessel', '-file', os.path.join(pkg_share, 'models', 'asv_vessel', 'model.sdf'), '-z', '0.18'],
        output='screen'
    )

    # 3. Bridge
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        parameters=[{'config_file': os.path.join(pkg_share, 'config', 'bridge_config.yaml')}],
        output='screen'
    )

    # 4. Dynamics Node
    dynamics = Node(package='asv_digital_twin', executable='dynamics_node', output='screen')

    return LaunchDescription([gz_resource_path, gazebo, spawn, bridge, dynamics])