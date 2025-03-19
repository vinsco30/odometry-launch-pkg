import copy
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.absolute()))

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='odometry_launch_pkg',
            executable='rtabmap_tf_publisher',
            name='rtabmap_tf_publisher',
            parameters=['/root/ros2_ws/src/odometry_launch_pkg/config/rtabmap_tf_publisher.yaml'],
            output='screen'
        ),
    ])