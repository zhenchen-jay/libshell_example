# cli libshell
This program take one json file as the input and output a sequence of meshes.  

The input json contains:
1. rest mesh path
2. current mesh path
3. current force file
4. material parameters: density, thickness, youngs modulus and poisson ratio
5. optimization termination criteria: energy update tolerance (energy_tol), gradient norm tolerance (grad_tol), variable update tolerance (variable_tol) and maixmum iterations (newton_iter)
6. Elastic model type (model_type: 0 for NeoHookean, 1 for StVK, 2 for tension field.) and the default type is StVK
7. The way to discretize second fundmental form: (sff_type: 0 for midedgetan, 1 for midedgesin and 2 for midedgeave), and the default is midedgetan

The first three ones are what we should care about. You can set the remaining entries as default. (Compare ribbon_elastic.json and ribbon_elastic_cur.json under the folder example/rotateRibbon)

The outputs are: a mesh (saved as mesh_simulated.obj file), velocity file (saved as vel_simulated.txt) and edge DOFs (saved as edgedofs_simulated.txt). All of these files are under the output folder specified by the users.

# Run the program
```
./cli_libshell -i YOUR_JSON_FILE_PATH -o YOUR_OUTPUT_FOLDER
```
For example
./cli_libshell -i ../example/rotatedRibbon/ribbon_elastic.json -o ../example/rotatedRibbon

# Use this to simulated the rotated ribbon
Please refer runSimulation.py