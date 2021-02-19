# Author: Mingchuan ZHOU
# Contact: mingchuan.zhou@in.tum.de
# Date: 31 Jan. 2021
# Usage: for testing the easy image from CoppeliaSim via python remoteApi
import sys
sys.path.append('python')
import numpy as np
import time
import cv2

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
if (clientID != -1) :
    print('Connected to remote API server')
    res, v0 = vrep.simxGetObjectHandle(clientID, 'Chessboard', vrep.simx_opmode_oneshot_wait)
    res, vp = vrep.simxGetObjectParent(clientID, v0, vrep.simx_opmode_oneshot_wait)
    print(vp)
    res, pos = vrep.simxGetObjectPosition(clientID, v0, -1, vrep.simx_opmode_oneshot_wait)
    res = vrep.simxSetObjectPosition(clientID, v0, -1, (-0.07000032812356949, 0.433004117012024, 0.5799995064735413), vrep.simx_opmode_oneshot_wait)
    while (vrep.simxGetConnectionId(clientID) != -1):

        print(pos)
        res = vrep.simxSetObjectPosition(clientID,v0,v0,(0,0,0),vrep.simx_opmode_oneshot_wait)
else:
    print('Failed connecting to remote API server')

print('Program ended')
