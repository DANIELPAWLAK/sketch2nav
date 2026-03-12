import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/pawlak/sketch2nav/ros2ws/install/demo_teleop'
