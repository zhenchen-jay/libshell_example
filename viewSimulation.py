import os
import json
import math
import getpass
import subprocess
from common import *

def view_simulation():
    args = [viewer_exe_path, '--folder', output_folder + '/mesh/', '--speed', str(50)]
    print(args)
    subprocess.check_call(args)

if __name__ == "__main__":
    view_simulation()