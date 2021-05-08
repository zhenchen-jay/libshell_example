This repo is aimed to provide a python API for Thin Shell solver. The basic C++ codes are copy from [libshell](https://github.com/evouga/libshell)

To compile the libshell, do things as follow (For environment setting configuration, please refer the readme in that folder):

For Linux:
```
cd  cpplib/libshell
mkdir build
cd build
cmake ..
make
```
For Windows:
You can either use the command line or [CMake Gui](https://cmake.org/) to compile this lib.

After compile, to run the code, just type
```
python runSimulation
```

This will simulate a rotated ribbon. All the meshes will be stored in cpplib/libshell/example/rotatedRibbon/ribbon_dynamic/mesh

To make sure the program runs, there are several things need to do:
- please make the folder ribbon_dynamic, and two subdirectories: mesh, velocity
- please change the "exe_path" and "viewer_exe_path" to points out the correct exe.

For Windows:
```
exe_path = "cpplib/libshell/build/Release/cli_libshell"
viewer_exe_path = "cpplib/libshell/build/Release/viewer_libshell"
```

For Linux:
```
exe_path = "cpplib/libshell/build/cli_libshell"
viewer_exe_path = "cpplib/libshell/build/viewer_libshell"
```

To visualize the simulation results, just type
```
python viewSimulation
```

The setup file like the exe path, input json configuration, and output folder are given in the common.py. 