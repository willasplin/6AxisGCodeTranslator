# This script converts cartesian XYZ .gcode files output from a slicer into XYZABC movements for a articulated robot arm
# The arm consists of 3 bending joints (XYZ), 2 rotational joints (AB)(with potential to add C in the future) and 1 extruder (E)
# Joint order from the base is as follows: A X Y B Z

import numpy as np
import matplotlib as plt
import math
import random as rnd
from scipy.spatial.transform import Rotation as R
import os
import time as tme

# Depending on the run environment, uncomment different lines below
old_files = []
#if getattr(sys, 'frozen', False):  #uncomment loop before using pyinstaller
#    app_path = os.path.dirname(sys.executable)
#else:
#    app_path = os.path.dirname(os.path.abspath(__file__))
app_path = os.path.dirname(os.path.abspath(__file__))  #uncomment if running the .py or .pyw file
#app_path = os.getcwd()  #uncomment if running in jupyterlab
os.chdir(app_path)
print(app_path)

# Iterate through the directory that this script is in, and create lists of .gcode file to be translated, and a new list of files
# that will be created by this script
validFormats = ['.gcode']
for filename in os.listdir(app_path):
    #print(filename)
    ext = os.path.splitext(filename)[1]  # Seperate extension
    if ext.lower() not in validFormats:
        continue
    if '-translated' in filename:
        continue
    old_files.append(filename)
#print(f'Current files: {files}')

# For each of the above files, create a new file with the name, "file-translated.gcode"
new_files = []
for old_file in old_files:
    filename = os.path.splitext(old_file)[0]
    new_file = filename+'-translated'+'.gcode'
    new_files.append(new_file)
#print(f'Translated files: {newFiles}')

# Option to add a system pause for user input after displaying the files to translate
os.system("pause")

# Define the cosine rule to improve readability of code below
# Output angle is between sides a and b, and opposite c
def cosine_rule(a, b, c):
    angle = math.acos((a**2 + b**2 - c**2)/(2 * a * b))
    return angle

# zip() iterates through both new_files and old_files simultaneously. This can be used insterad of enumerate in a lot of places
# If a version of the new file exists then it is deleted instead of being modified
for old_file, new_file in zip(old_files, new_files):
    if os.path.exists(new_file):
        os.remove(new_file)

    translation_counter = 0
    t0 = tme.perf_counter()    

    # The read and write files can be opened at the same time
    with open(new_file, 'a+') as fNew, open(old_file, 'r') as f:
        print(f'Translating File: {f.name}')
        xyzef_old = np.zeros(5)

        # Loop goes through file line by line, reads text, checks for G0/G1 and writes new commands to the new file
        # If line does not contain a move command, the line is written directly to the new file
        for line in f:
            #print(f'{line.strip()}')
            if not any(G in line for G in ('G0', 'G1')):
                fNew.write(line+'\n')
            else:
                # If it is a move command, split the line into its whitespace separated commands
                vals_G = line.split(';')[0]
                vals = vals_G.split()
                
                # Create an array of the new position of the end effector
                # Go through each value and add it to the correct place in the position array
                xyzef_new = np.array([None, None, None, None, None], dtype=float)
                letters = ['X','Y','Z','E','F']
                for val in vals:
                    for j, letter in enumerate(letters):
                        if val[0] == letter:
                            xyzef_new[j] = float(val[1:])
                
                # The robot uses a different orign location so the position array is translated to the correct origin
                # xyzef_old values have already been translated so the new values are translated before the old ones are inserted
                xyzef_translate = np.array([-100,50,20,0,0])
                xyzef_new = xyzef_new + xyzef_translate
                
                # If the new value is nan, the old value is used. This may need to be changed but I cannot create empty numpy arrays
                for i, (new_val, old_val) in enumerate(zip(xyzef_new, xyzef_old)):
                    if np.isnan(new_val):
                        xyzef_new[i] = old_val
                #print(f'{xyzef_old = }')
                #print(f'{xyzef_new = }')
                
                # C axis has not been implemented yet. This might be either a 6th arm axis, or a rotating build plate
                c_ang = 0

                # Set the length of the robot arms. l1 between joints X and Y, l2 between joints Y and Z, and l3 between joint Z and the end effector
                l1 = 250
                l2 = 200
                l3 = 80
                
                # End effector/print angle is set by the slicer/geometry so can be used to calculate the position of the other joints
                # prin_angle_deg stores the pitch (angle from +vs Z) and yaw (angle from +ve X) angles (ISO 80000-2)
                print_angle_deg = np.array([45.0,-80.0])
                print_angle = np.array([np.radians(print_angle_deg[0]), np.radians(print_angle_deg[1])])
                
                # d is a vector from the end effector to joint Z
                # v is the position vector of joint Z
                # h and h2 is the norm/magnitude/length from origin to d and v respectively
                d = np.array([l3*np.sin(print_angle[0])*np.cos(print_angle[1]), l3*np.sin(print_angle[0])*np.sin(print_angle[1]), l3*np.cos(print_angle[0])])
                #print(f'dx = {d[0]}, dy = {d[1]}, dz = {d[2]}')
                v = xyzef_new[:3] + d
                #print(f'{v = }')
                h = np.linalg.norm(xyzef_new[:3])  #sqrt(x^2 + y^2 + z^2)
                h2 = np.linalg.norm(v)
                #print(f'{h = }, {h2 = }')

                # x_ang, y_ang, z_ang etc are the joint angles of the respective arm joint
                # x_ang is the angle of the X joint i.e. the base bending joint | sum of angle between l1 and h2, and h2 and the xy plane
                # y_ang is the angle of the Y joint i.e. the middle bending joint
                x_alpha = cosine_rule(h2, l1, l2)
                x_beta = math.asin(v[2]/h2) 
                x_ang = x_alpha + x_beta
                #print(f'{x_alpha = }, {x_beta = }, {x_ang = }')
                y_ang = cosine_rule(l1, l2, h2)
                a_ang = np.arctan2(v[0], v[1])  #trig from a top down view using the XY of point v
                
                #print(f'{x_ang = }, {y_ang = }'), {a_ang = }')
                #print(f'x_deg = {np.degrees(x_ang)}, y_deg = {np.degrees(y_ang)}, a_deg = {np.degrees(a_ang)}')  #output in degrees for readability
                
                # To calculate the angles of joints Z and B, the reference frame for xyzef is translated to the Z joint
                # Then after rotating the system so l2 lies on the Y axis, joint angles for B and Z can be calculated
                # A new set of coords is created, mainly for debugging reference, this is then placed on joint Z
                xyz = np.array(xyzef_new[:3])
                xyz_0 = xyz - v
                #print(f'{xyz = }, {xyz_0 = }')
                
                # The scipy library imported simplifies the code massively
                # Legacy method before scipy library was implemented
                #c, s = np.cos(rot_ang_1), np.sin(rot_ang_1)  #cos and sin values are set here for matrix readability
                #rm01 = np.array([[c, -s, 0], 
                #                 [s, c, 0], 
                #                 [0, 0, 1]])  
                #xyz_1 = np.matmul(rm01, xyz_0)  #apply the rotation matrix to the system
                #print(f'{xyz_1 = }')
                
                # Rotates about Z axis to align l2 on zy plane
                rot_ang_1 = a_ang
                rm01 = R.from_euler('z', rot_ang_1)
                rm01_euler = rm01.as_euler('xyz', degrees=True)
                #print(f'{rm01_euler = }')
                #print(f'{rm01.as_matrix() = }')
                xyz_1 = rm01.apply(xyz_0)
                #print(f'{xyz_1 = }')
                
                
                # Rotates about X axis to align l2 along the y axis
                rot_ang_2 = math.pi - x_ang - y_ang  #similar triangles
                #print(f'rot_ang_2_deg = {np.degrees(rot_ang_2_deg)}')
                rm12 = R.from_euler('x', rot_ang_2)
                rm12_euler = rm12.as_euler('xyz', degrees=True)
                #print(f'{rm12_euler = }')
                #print(f'{rm12.as_matrix() = }')
                xyz_2 = rm12.apply(xyz_1)
                #print(f'{xyz_2 = }')
                
                # Joint angle B is now the angle between xyz and the zy plane
                # Rotate around Y axiz to put EE on ZY plane
                rot_ang_3 = np.arctan2(xyz_2[0], -xyz_2[2])
                b_ang = rot_ang_3
                #print(f'rot_ang_3_deg = {np.degrees(rot_ang_3_deg)}')
                rm23 = R.from_euler('y', rot_ang_3)
                rm23_euler = rm23.as_euler('xyz', degrees=True)
                #print(f'{rm23_euler = }')
                #print(f'{rm23.as_matrix = }')
                xyz_3 = rm23.apply(xyz_2)
                #print(f'{xyz_3 = }')
                
                # Joint angle Z is now the angle between xyz and the negative y axis
                rot_ang_4 = np.arctan(xyz_3[1]/ -xyz_3[2])
                z_ang = math.pi/2 + rot_ang_4                 
                #print(f'{rot_ang_4 = },  rot_ang_4_deg = {np.degrees(rot_ang_4)}')
                #print(f'{z_ang = },  z_ang_deg = {np.degrees(z_ang)}')
                
                # Extrusion and move speeds have not changed
                e_val = xyzef_new[3]
                f_val = xyzef_new[4]
                xyzabcef = np.array([x_ang, y_ang, z_ang, a_ang, b_ang, c_ang, e_val, f_val])        
                
                # Convert angles to degrees for easier conversion to steps/angle
                xyzabcef_deg = np.around(np.degrees(xyzabcef[:6]), 3)
                xyzabcef_deg = np.append(xyzabcef_deg, (xyzabcef[6], xyzabcef[7]))

                xyzef_old = xyzef_new  #store current position for next loop
                #print(f'{xyzef_old = }')
                #print(f'{xyzabcef_deg = }')
                #print(f'Output to printer: X = {np.degrees(x_ang)},  Y = {np.degrees(y_ang)},  Z = {np.degrees(z_ang)},  A = {np.degrees(a_ang)},  B = {np.degrees(b_ang)},  C = {np.degrees(c_ang)},  E = {xyzabcef[6]},  F = {xyzabcef[7]}')
                
                # Write new values to new file as gcode command
                #print(f'G0 X{xyzabcef_deg[0]} Y{xyzabcef_deg[1]} Z{xyzabcef_deg[2]} A{xyzabcef_deg[3]} B{xyzabcef_deg[4]} C{xyzabcef_deg[5]} E{xyzabcef_deg[6]} F{xyzabcef_deg[7]} \n')
                fNew.write('G0' + ' X' + str(xyzabcef_deg[0]) + ' Y' + str(xyzabcef_deg[1]) + ' Z' + str(xyzabcef_deg[2]) + ' A' + str(xyzabcef_deg[3]) + ' B' + str(xyzabcef_deg[4]) + ' C' + str(xyzabcef_deg[5]) + ' E' + str(xyzabcef_deg[6]) + ' F' + str(xyzabcef_deg[7])+"\n")
                translation_counter += 1
    t1 = tme.perf_counter()    
    print(f'File Translated: {f.name}')
    print(f'Time taken: {t1-t0}')
    print(f'Lines Translated: {translation_counter}')
os.system("pause")