import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hoanganh/Documents/RTK-GPS-BASED-AUTONOMOUS-DRIVING/install/autonomous_car'
