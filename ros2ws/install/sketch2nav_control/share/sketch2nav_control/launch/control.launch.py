from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('sketch2nav_control'),
        'config', 'params.yaml'
    )

    pure_pursuit = Node(
        package='sketch2nav_control',
        executable='pure_pursuit',
        name='pure_pursuit',
        parameters=[params],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([pure_pursuit])
