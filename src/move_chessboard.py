# Author: Mingchuan ZHOU
# Contact: mingchuan.zhou@in.tum.de
# Date: 31 Jan. 2021
# Usage: for testing the easy image from CoppeliaSim via python remoteApi
import sys
sys.path.append('python')
import numpy as np
import time
import cv2
pi = 3.141592653
t = 0.3
pi30 = 30*pi/180
try:
    import sim as vrep
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

print('Program started')
vrep.simxFinish(-1)  # just in case, close all opened connections

clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
def rotation_combinations(handle):
    rotates_z(handle,5)
    rotates_z(handle,10)
    rotates_z(handle,15)
    rotates_z(handle,20)
    rotates_z(handle,25)
    rotates_z(handle,30)
def cal_arc(deg):
    return deg*pi/180
def rotates_z(handle,deg):
    rotates(handle, deg)
    arc = cal_arc(45)
    res, ori0 = vrep.simxGetObjectOrientation(clientID, v0, -1, vrep.simx_opmode_oneshot_wait)
    res = vrep.simxSetObjectOrientation(clientID, handle, -1, (ori0[0], ori0[1], ori0[2]+arc),vrep.simx_opmode_oneshot_wait)
    rotates(handle,deg)
    res = vrep.simxSetObjectOrientation(clientID, handle, -1, (ori0[0], ori0[1], ori0[2]),
                                        vrep.simx_opmode_oneshot_wait)
def rotates(handle,deg):
    arc = cal_arc(deg)
    res, ori0 = vrep.simxGetObjectOrientation(clientID, v0, -1, vrep.simx_opmode_oneshot_wait)

    res = vrep.simxSetObjectOrientation(clientID, handle, -1, (ori0[0]+arc, ori0[1], ori0[2]), vrep.simx_opmode_oneshot_wait)
    time.sleep(t)
    res = vrep.simxSetObjectOrientation(clientID, handle, -1, (ori0[0]+arc, ori0[1]+arc, ori0[2]), vrep.simx_opmode_oneshot_wait)
    time.sleep(t)
    res = vrep.simxSetObjectOrientation(clientID, handle, -1, (ori0[0], ori0[1]+arc, ori0[2]), vrep.simx_opmode_oneshot_wait)
    time.sleep(t)
    res = vrep.simxSetObjectOrientation(clientID, handle, -1, (ori0[0]-arc, ori0[1]+arc, ori0[2]), vrep.simx_opmode_oneshot_wait)
    time.sleep(t)
    res = vrep.simxSetObjectOrientation(clientID, handle, -1, (ori0[0]-arc, ori0[1], ori0[2]), vrep.simx_opmode_oneshot_wait)
    time.sleep(t)
    res = vrep.simxSetObjectOrientation(clientID, handle, -1, (ori0[0]-arc, ori0[1]-arc, ori0[2]), vrep.simx_opmode_oneshot_wait)
    time.sleep(t)
    res = vrep.simxSetObjectOrientation(clientID, handle, -1, (ori0[0], ori0[1]-arc, ori0[2]), vrep.simx_opmode_oneshot_wait)
    time.sleep(t)
    res = vrep.simxSetObjectOrientation(clientID, handle, -1, (ori0[0]+arc, ori0[1]-arc, ori0[2]), vrep.simx_opmode_oneshot_wait)
    time.sleep(t)

    res = vrep.simxSetObjectOrientation(clientID, handle, -1, ori0, vrep.simx_opmode_oneshot_wait)

if (clientID != -1) :
    print('Connected to remote API server')
    res, v0 = vrep.simxGetObjectHandle(clientID, 'Chessboard', vrep.simx_opmode_oneshot_wait)
   # res, vp = vrep.simxGetObjectParent(clientID, v0, vrep.simx_opmode_oneshot_wait)
    res, pos0 = vrep.simxGetObjectPosition(clientID, v0, -1, vrep.simx_opmode_oneshot_wait)
    res, ori0 = vrep.simxGetObjectOrientation(clientID, v0, -1, vrep.simx_opmode_oneshot_wait)
    print(pos0, ori0)
    while (vrep.simxGetConnectionId(clientID) != -1):
        #############################original#############################
        res = vrep.simxSetObjectPosition(clientID, v0, -1, pos0, vrep.simx_opmode_oneshot_wait)
        res = vrep.simxSetObjectOrientation(clientID, v0, -1, ori0, vrep.simx_opmode_oneshot_wait)
        time.sleep(t)
        rotation_combinations(v0)

        #############################layer 02#############################
        res = vrep.simxSetObjectPosition(clientID, v0, v0, (0,0,-0.2), vrep.simx_opmode_oneshot_wait)
     #   res = vrep.simxSetObjectOrientation(clientID, v0, -1, ori0, vrep.simx_opmode_oneshot_wait)
        time.sleep(t)
        rotation_combinations(v0)

        res = vrep.simxSetObjectPosition(clientID, v0, v0, (0.12, 0.18, 0) , vrep.simx_opmode_oneshot_wait)
        time.sleep(t)
        rotation_combinations(v0)

        res = vrep.simxSetObjectPosition(clientID, v0, v0, (0, -0.36, 0), vrep.simx_opmode_oneshot_wait)
        time.sleep(t)
        rotation_combinations(v0)

        res = vrep.simxSetObjectPosition(clientID, v0, v0, (-0.24, 0, 0), vrep.simx_opmode_oneshot_wait)
        time.sleep(t)
        rotation_combinations(v0)

        res = vrep.simxSetObjectPosition(clientID, v0, v0, (0, 0.36, 0), vrep.simx_opmode_oneshot_wait)
        time.sleep(t)
        rotation_combinations(v0)

        #############################layer 02#############################

        res = vrep.simxSetObjectPosition(clientID, v0, v0, (0.12, -0.18, -0.2), vrep.simx_opmode_oneshot_wait)
        time.sleep(t)
        rotation_combinations(v0)

        res = vrep.simxSetObjectPosition(clientID, v0, v0, (0.2, 0.3, 0), vrep.simx_opmode_oneshot_wait)
        time.sleep(t)
        rotation_combinations(v0)

        res = vrep.simxSetObjectPosition(clientID, v0, v0, (0, -0.6, 0), vrep.simx_opmode_oneshot_wait)
        time.sleep(t)
        rotation_combinations(v0)

        res = vrep.simxSetObjectPosition(clientID, v0, v0, (-0.4, 0, 0), vrep.simx_opmode_oneshot_wait)
        time.sleep(t)
        rotation_combinations(v0)

        res = vrep.simxSetObjectPosition(clientID, v0, v0, (0, 0.6, 0), vrep.simx_opmode_oneshot_wait)
        time.sleep(t)
        rotation_combinations(v0)

      #  res = vrep.simxSetObjectPosition(clientID, v0, v0, (0, 0, -0.1), vrep.simx_opmode_oneshot_wait)
      #  res = vrep.simxSetObjectOrientation(clientID, v0, -1, ori0, vrep.simx_opmode_oneshot_wait)




else:
    print('Failed connecting to remote API server')

print('Program ended')
