from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory("demo_gazebo")
    world = os.path.join(pkg_share, "worlds", "empty.sdf")
    model = os.path.join(pkg_share, "models", "demo", "model.sdf")
    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-r", world],
        output="screen"
    )
    #Spawning rover, z higher so doesnt cut into ground
    spawn = ExecuteProcess(
        cmd=["ros2", "run", "ros_gz_sim", "create",
             "-name", "demo",
             "-file", model,
             "-x", "0",
             "-y", "0",
             "-z", "0.13",
             ],
        output="screen"
    )
    #Bridge ros2 and gazebo twists and odoms on cmd_vel topic
    bridge = ExecuteProcess(
        cmd=[
            "ros2", "run", "ros_gz_bridge", "parameter_bridge",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry"
        ],
        output="screen"
    )

    return LaunchDescription([gz_sim, spawn, bridge])
