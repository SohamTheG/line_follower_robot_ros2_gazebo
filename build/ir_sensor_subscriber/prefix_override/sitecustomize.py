import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/soham/line_follower_ws/install/ir_sensor_subscriber'
