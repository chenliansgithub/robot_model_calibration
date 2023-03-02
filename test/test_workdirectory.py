import os
import sys
import numpy as np
# import pwd_1
current_dir = os.getcwd()
print(current_dir)
# python 需要显式的明确'PYTHONPATH',在launch.json中添加
# "env": {"PYTHONPATH": "${workspaceFolder}${pathSeparator}${env:PYTHONPATH}"},
# 然后才可以开心的import
print(os.environ['PYTHONPATH'])