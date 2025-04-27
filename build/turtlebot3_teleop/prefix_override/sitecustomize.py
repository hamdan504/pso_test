import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ahmed/pso_visual_ws/install/turtlebot3_teleop'
