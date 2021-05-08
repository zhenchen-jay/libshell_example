import os
import json
import glob
import math
import getpass

exe_path = "cpplib/libshell/build/Release/cli_libshell"
viewer_exe_path = "cpplib/libshell/build/Release/viewer_libshell"

file_prefix = "cpplib/libshell/example/rotatedRibbon/"
output_folder = "cpplib/libshell/example/rotatedRibbon/ribbon_dynamic"
input_json = "cpplib/libshell/example/rotatedRibbon/ribbon_elastic.json"
cur_json = "cpplib/libshell/example/rotatedRibbon/ribbon_elastic_cur.json"
