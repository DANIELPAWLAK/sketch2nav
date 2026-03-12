from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory("demo_gazebo")
    world = os.path.join(pkg_share, "worlds", "empty.sdf")
    model = os.path.join(pkg_share, "models", "demo", "model.sdf")

    # Start Gazebo Harmonic
    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-r", world],
        output="screen"
    )

    # Spawn your model
    spawn = ExecuteProcess(
        cmd=["ros2", "run", "ros_gz_sim", "create",
             "-name", "demo",
             "-file", model],
        output="screen"
    )

    # Bridge: ROS2 Twist <-> Gazebo Twist, and Gazebo odom -> ROS2 Odometry
    # ROS topic: /cmd_vel (Twist) -> Gazebo: /model/boxbot/cmd_vel
    # Gazebo: /model/boxbot/odometry -> ROS: /odom
    bridge = ExecuteProcess(
        cmd=[
            "ros2", "run", "ros_gz_bridge", "parameter_bridge",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry"
        ],
        output="screen"
    )

    return LaunchDescription([gz_sim, spawn, bridge])
