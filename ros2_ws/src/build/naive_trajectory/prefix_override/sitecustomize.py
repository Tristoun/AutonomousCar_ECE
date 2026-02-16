import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/tristan/Documents/autonomous_car_ece/car_control_wave/ros2_ws/src/install/naive_trajectory'
