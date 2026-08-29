from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="cave_drone_description",
            executable="lidar_pan_base_tf_broadcaster",
            name="lidar_pan_base_tf_broadcaster"
        ),
        Node(
            package="cave_drone_description",
            executable="lidar_pan_tf_broadcaster",
            name="lidar_pan_tf_broadcaster"
        ),
        Node(
            package="cave_drone_description",
            executable="sf45_tf_broadcaster",
            name="sf45_tf_broadcaster"
        ),

    ])