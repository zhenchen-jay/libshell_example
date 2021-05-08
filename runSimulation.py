import os
import json
import math
import getpass
import subprocess
import numpy as np
import igl

from common import *

def save_clamped_dofs(file, data):
    with open(file,'w') as f:
        nrows, ncols = data.shape
        f.write(str(nrows) + '\n')
        for i in range(nrows):
            vid = int(data[i][0])
            f.write(str(vid) + ' ')
            for j in range(1, ncols - 1):
                f.write(str(data[i][j]) + ' ')
            f.write(str(data[i][ncols - 1]) +'\n')


def rotate_ribbon(omega, skipped_frames, dt):
    with open(input_json) as f:
        input_data = json.load(f)
        handles = [0, 3] # handle vertices

        rest_mesh_path = file_prefix + input_data['rest_mesh']
        rest_pos, _, _, faces, _, _ = igl.read_obj(rest_mesh_path)
        cur_vel = np.zeros(rest_pos.shape)
        np.savetxt(file_prefix + input_data['current_velocity'], cur_vel)

        # save rest mesh
        cur_pos = rest_pos
        mesh_path = output_folder + '/mesh/mesh_0.obj'
        igl.write_obj(mesh_path, rest_pos, faces)

        #save init vel
        vel_path = output_folder + '/velocity/vel_0.txt'
        np.savetxt(vel_path, cur_vel)

        kp = 0.1
        kd = 2 * math.sqrt(kp)
        fmin = -0.01
        fmax = 0.01
        
        for i in range(250):    
            cur_time  = i  * dt * skipped_frames
            print('iter: ', i)
            theta = omega * cur_time
            ref_theta = omega * (i * dt * skipped_frames + dt)
            # generate force file
            clampedDOFs = np.zeros((len(handles), 4))
            for j in range(len(handles)):
                vid = handles[j]
                r = np.linalg.norm(rest_pos[vid])
                clampedDOFs[j][0] = vid
                clampedDOFs[j][2] = r * math.sin(ref_theta)
                clampedDOFs[j][3] = r * math.cos(ref_theta)
                print('clamped dofs on vertex:', vid, clampedDOFs[j])
            save_clamped_dofs(file_prefix + input_data['clamped_DOFs'], clampedDOFs)

            cur_data = input_data

            cur_data['cur_mesh'] = '/ribbon_dynamic/mesh/mesh_' + str(i) + '.obj'
            cur_data['curedge_DOFs'] = '/ribbon_dynamic/mesh/edgedofs_' + str(i) + 'txt'
            cur_data['current_velocity'] = '/ribbon_dynamic/velocity/vel_' + str(i) + '.txt'
            cur_data['clamped_DOFs'] = '/ribbon_dynamic/mesh/clampedDOFs_' + str(i) + '.txt'

            #save clamped dofs
            save_clamped_dofs(file_prefix + cur_data['clamped_DOFs'], clampedDOFs)

            with open(cur_json, 'w') as json_file:
                json.dump(cur_data, json_file, indent = 4, sort_keys=True)

            args = [exe_path, '--input', cur_json, '--output', file_prefix, '--numSteps', str(4), '--timeStep', str(0.005)]
            subprocess.check_call(args)

            # save files
            cur_pos, _, _, faces, _, _ = igl.read_obj(file_prefix + 'mesh_simulated.obj')
            mesh_path = output_folder + '/mesh/mesh_' + str(i+1) + '.obj'
            igl.write_obj(mesh_path, cur_pos, faces)

            #save vel
            cur_vel = np.loadtxt(file_prefix + 'vel_simulated.txt')
            vel_path = output_folder + '/velocity/vel_' + str(i+1) + '.txt'
            np.savetxt(vel_path, cur_vel)

            #save edge dofs
            cur_edgedofs = np.loadtxt(file_prefix + 'edgedofs_simulated.txt')
            edge_path = output_folder + '/mesh/edgedofs_' + str(i+1) + 'txt'
            np.savetxt(edge_path, cur_edgedofs)

   

def run_simulation():
    args = [exe_path, '--input', input_json, '--output', output_folder, '--numSteps', str(4), '--timeStep', str(0.005)]
    print(args)
    subprocess.check_call(args)

if __name__ == "__main__":
    # run_simulation()
    rotate_ribbon(10, 4, 0.005)
