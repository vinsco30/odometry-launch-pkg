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
        # dummy static transformation from camera1 to camera2
        launch_ros.actions.Node(
            package = "tf2_ros",
            executable = "static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "T265_pose_frame", "T265_link"]
        ),
        launch_ros.actions.Node(
            package = "tf2_ros",
            executable = "static_transform_publisher",
            arguments = ["-0.19", "0", "0.07", "0", "0", "0", "T265_link", "D400_link"]
        ),
        launch_ros.actions.Node(
            package = "tf2_ros",
            executable = "static_transform_publisher",
            arguments = ["-0.065", "0", "-0.02", "0", "0", "0", "D400_link", "base_link"]
        ),
    ])