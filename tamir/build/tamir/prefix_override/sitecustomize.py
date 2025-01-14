import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/asaace00/repo/winter/project/TAMIR/tamir/install/tamir'
