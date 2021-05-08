# viewer libshell
This program aims to provide a way to visulize either
- a single mesh (an obj file)
- a sequence of meshes (mesh_xx.obj)

# run the program 
To visualize a single mesh
```
./viewer_libshell -m TOU_OBJ_FILE
```
e.g.
```
./viewer_libshell -m ../example/rotatedRibbon/ribbon.obj
```

To visualize a sequence of meshes, make sure all the meshes are in the same folder and named as mesh_x.obj, where 'x' ranges from 0 to N contiguously. Then type
```
./viewer_libshell -f FOLD_CONTAINS_MESHES -s 30
```
the '-s' refers the speed (frame per second) to play the animation. 
e.g.
```
./viewer_libshell -f ../example/rotatedRibbon/ribbon_dynamic_mesh -s 30
```