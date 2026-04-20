import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jj/Desktop/Github/ASV/install/asv_digital_twin'
