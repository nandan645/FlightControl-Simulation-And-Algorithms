import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/abhinandan/ROS2/aruco_distance/src/aruco_distance/install/aruco_distance'
