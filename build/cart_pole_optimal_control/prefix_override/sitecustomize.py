import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/dhiren/RAS-SES-598-Space-Robotics-and-AI/install/cart_pole_optimal_control'
