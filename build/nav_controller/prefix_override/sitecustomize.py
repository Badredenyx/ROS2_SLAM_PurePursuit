import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/badredynex/Project/ROS2-PurePursuitControl-PathPlanning-Tracking/install/nav_controller'
