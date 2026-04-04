from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('sketch2nav_bringup')

    world = os.path.join(pkg_share, 'worlds', 'empty.sdf')
    model = os.path.join(pkg_share, 'models', 'sketch2nav', 'model.sdf')

    # ── Args ──────────────────────────────────────────────────
    declare_x = DeclareLaunchArgument('x', default_value='0.0',
                                      description='Spawn X position')
    declare_y = DeclareLaunchArgument('y', default_value='0.0',
                                      description='Spawn Y position')

    # ── Gazebo ────────────────────────────────────────────────
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world],
        output='screen'
    )

    # ── Spawn rover (slight delay so Gazebo is ready) ─────────
    spawn = TimerAction(
        period=3.0,
        actions=[ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-name', 'sketch2nav',
                '-file', model,
                '-x', LaunchConfiguration('x'),
                '-y', LaunchConfiguration('y'),
                '-z', '0.13',
            ],
            output='screen'
        )]
    )

    # ── ROS↔Gazebo bridge ─────────────────────────────────────
    # cmd_vel: ROS → Gz  (web UI / teleop sends Twist in)
    # odom:    Gz → ROS  (odometry comes out to web UI + follower)
    bridge = TimerAction(
        period=3.5,
        actions=[ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            ],
            output='screen'
        )]
    )

    # ── rosbridge WebSocket (connects web UI) ─────────────────
    rosbridge = ExecuteProcess(
        cmd=[
            'ros2', 'launch', 'rosbridge_server',
            'rosbridge_websocket_launch.xml'
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_x,
        declare_y,
        gz_sim,
        spawn,
        bridge,
        rosbridge,
    ])
